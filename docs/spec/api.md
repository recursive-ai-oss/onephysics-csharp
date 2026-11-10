# Public API surface

Normative for what the two worlds promise to have in common, which parts of the package still carry `[Experimental]`, and how both are checked. The listings are written from the code; the tests below fail when the code and a listing disagree.

## The two worlds have the same members

`PhysicsWorld2D` and `PhysicsWorld3D`, and the types around them (`Body`, `Shape`, `Joint`, the definitions, options, events, queries and presets), expose the same members with 2D or 3D math types. A planar angle is a `float`, a spatial rotation a `QuaternionF`; positions cross the boundary as `Vector2D` or `Vector3D`.

`DimensionParityTests` compares the two public surfaces by reflection over the built assembly:

- Types are paired by name once the `2D` or `3D` suffix is dropped. Members are compared by kind and name (overloads count once; parameter lists are not compared, because the math types differ).
- A short vocabulary folds the dimensions' own words together before comparing: sphere and circle, hinge and revolute, fixed and weld, slider and prismatic, rotation and angle, convex hull and polygon, soft ball and soft circle, local frame and local anchor.
- Every remaining difference must be listed in `testdata/api/parity-allow.txt` with its reason. A listed difference that no longer exists fails the test too, so the list can only shrink or be reasoned about.

Two kinds of entries exist:

- **Intentional.** The physics does not apply or is modelled differently: voxels and meshes are 3D, tile grids, chains and carvable terrain are 2D, a planar world has one rotation axis (no cone, swing-twist, six-degree-of-freedom or point joints), side-view and spatial vehicles report different state.
- **`gap:`** An asymmetry that is not physics. These are work still to do, not decisions: 2D `Joint` kinds returned as the general `Joint` where 3D has typed views, the 3D distance-joint motor, the 3D fixed-joint springs, hit sub-shape ids in 2D, and a few smaller ones. The list is the to-do list.

Members added while writing the list, so the two worlds agree: 2D `Body.ApplyAngularImpulse(float)`, 2D `OverlapCapsule`/`OverlapCapsuleAny`/`OverlapCapsuleAll`, 3D `Body.IsBullet`, and a public 2D `WorldOptions2D.Validate()`.

## The happy path needs no struct callback

Tier 1 — the world, `Body`, `Shape`, `Joint` and `Character` and their one-liners — never requires writing a struct that implements a callback interface. Filtering there is the layer matrix plus the optional `PairFilter` delegate on the world options; contact modification is the `ContactModifier` delegate property; events are C# events and polled arrays.

The middle and raw tiers take struct callbacks as generic arguments so the call is specialised and allocation-free. `TierOneTests` finds every public method with a struct-constrained type parameter and requires either a plain (non-generic) overload of the same name on the same type or an entry in `testdata/api/tier1-struct-callbacks.txt` naming the delegate form that covers the happy path. It also checks that no non-generic member of the tier-1 types takes one of the library's callback interfaces.

## `[Experimental]` inventory

Each marker names a diagnostic id. A consumer opts in with `#pragma warning disable <id>` or `<NoWarn>` in the project. The package itself, the README examples and the guide examples do not suppress them in code a user would copy. Nothing is listed here that is not marked; nothing marked is missing.

| Id | Covers | Count |
|---|---|---|
| `OPHYS0001` | The raw tier: the whole `RecursiveAI.OnePhysics.LowLevel` namespace (body and constraint stores, colouring, broadphase trees, narrowphase routines, shape kernels); in addition the rope, pulley and belt types that sit in the world namespaces and in `Cables` and `Rods` (see below) | 147 markers |
| `OPHYS0002` | The 3D engine: every type of `RecursiveAI.OnePhysics.Physics3D` (the world, `Body`, `Shape`, `Joint` and the joint kinds, characters, vehicles, ragdolls, soft bodies and cloth, fluids, water, terrain tiles, queries, options), the 3D members of shared types, and in `.Tools` the two members that return a 3D compound or shape specification (`ToCompoundParts`, `ToShapeSpec`); 2D: the character-water event | 178 markers |
| `OPHYS0003` | Features added after the first previews, in both dimensions: active ragdolls and walking (`ActiveRagdollDef`, `BalanceDef`, `WalkerDef`, `BipedBones`, `ActiveMode`, `AnimationRootMode`, `GaitPhase`), articulations (`Articulation*` in both worlds, `ArticulationDrive`, `ArticulationJointKind`, `ArticulationStateException`), actuators and aerodynamics (`AeroSurface`, `AeroCoefficients`, `AeroMedium`, `AeroSurfaceKind`, `Thruster*`, `Boat*`, `Hover*`), 2D vehicles, tracked vehicles, rail vehicles and bicycles, 2D ragdolls, 2D soft bodies and strips (`SoftBody*`, `SoftMaterial`, `StripParams`), and the debug records for active ragdolls and articulations | 101 markers |

`Physics3D` types that also belong to the later features carry `OPHYS0002` (the 3D engine is the wider marker); `OPHYS0003` on a 3D type appears only for `Aero`, `Thruster*`, `Boat*`, `Hover*` and `Articulation*`.

### Where a marker needs a decision before a release without markers

- **Rods and ropes, pulleys and belts** (`Rod`, `RodDef`, `RodAttachment`, `RodMaterial`, `RodParticles`, `RodStepStatistics`, `RopeEnd`, `Pulley`, `PulleyNodeDef`, `PulleySystem`, `PulleySystemDef`, `Sprocket`, the cable enums, and the `AddPulleySystem`/`AddRopeOverPulleys`/`AddWinch`/`AddBelt` members of both worlds) are user features but carry the raw tier's id `OPHYS0001`. Opting in to the raw tier opts in to them.
- **The 3D engine** is one marker over the whole namespace; its 2D twin is unmarked. A release that drops the 3D marker drops it for every type at once, or splits it per feature.
- **`OPHYS0003` members** are individually marked on both worlds and on the debug records; they share one id so a single decision covers them.
- **`.Tools`** has two marked members only (above); the rest of the package is unmarked.

No marker is removed by this document; it only lists them.

## Documentation comments

The projects build with `GenerateDocumentationFile` and warnings as errors, so a missing comment on any public type or member of the runtime or `.Tools` package fails the build (CS1591).
