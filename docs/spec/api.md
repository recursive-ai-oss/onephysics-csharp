# Public API surface

Normative for what the two worlds promise to have in common, which part of the package still carries `[Experimental]`, and how both are checked. The listings are written from the code; the tests below fail when the code and a listing disagree.

## The two worlds have the same members

`PhysicsWorld2D` and `PhysicsWorld3D`, and the types around them (`Body`, `Shape`, `Joint`, the definitions, options, events, queries and presets), expose the same members with 2D or 3D math types. A planar angle is a `float`, a spatial rotation a `QuaternionF`; positions cross the boundary as `Vector2D` or `Vector3D`.

`DimensionParityTests` compares the two public surfaces by reflection over the built assembly:

- Types are paired by name once the `2D` or `3D` suffix is dropped. Members are compared by kind and name (overloads count once; parameter lists are not compared, because the math types differ).
- A short vocabulary folds the dimensions' own words together before comparing: sphere and circle, hinge and revolute, fixed and weld, slider and prismatic, rotation and angle, convex hull and polygon, soft ball and soft circle, local frame and local anchor.
- Every remaining difference must be listed in `testdata/api/parity-allow.txt` with its reason. A listed difference that no longer exists fails the test too, so the list can only shrink or be reasoned about.

Every entry is intentional: the physics does not apply or is modelled differently: voxels and meshes are 3D, tile grids, chains and carvable terrain are 2D, a planar world has one rotation axis (no cone, swing-twist, six-degree-of-freedom or point joints), side-view and spatial vehicles report different state. Asymmetries that are not physics are closed, not listed.

Members added while writing the list, so the two worlds agree: 2D `Body.ApplyAngularImpulse(float)`, 2D `OverlapCapsule`/`OverlapCapsuleAny`/`OverlapCapsuleAll`, 3D `Body.IsBullet`, and a public 2D `WorldOptions2D.Validate()`. Later, the remaining non-physics gaps were closed: 2D `ShapeDef.LocalPosition`/`LocalAngle` (baked into the shape at creation), 2D `CreateSdf`, 3D `OverlapSoftBodies`, 2D `QueryHit.SubShapeId` (0 for rays, casts and closest points, -1 for overlaps), 3D `Collision.TangentImpulse`, 2D `WorldOptions2D.SpeculativeDistance`, value equality and text on the 2D `Fluid`, typed 2D `GearJoint`, `RackAndPinionJoint`, `PulleyJoint` and `PathJoint` views and the 2D `IJointDef2D`, the 3D distance-joint motor, and 3D fixed-joint springs (`FixedJointDef.LinearHertz`, `LinearDampingRatio`, `AngularHertz`, `AngularDampingRatio` and `SetFixedSprings`, the 2D weld joint's `SetWeldSprings`).

## The happy path needs no struct callback

Tier 1 — the world, `Body`, `Shape`, `Joint` and `Character` and their one-liners — never requires writing a struct that implements a callback interface. Filtering there is the layer matrix plus the optional `PairFilter` delegate on the world options; contact modification is the `ContactModifier` delegate property; events are C# events and polled arrays.

The middle and raw tiers take struct callbacks as generic arguments so the call is specialised and allocation-free. `TierOneTests` finds every public method with a struct-constrained type parameter and requires either a plain (non-generic) overload of the same name on the same type or an entry in `testdata/api/tier1-struct-callbacks.txt` naming the delegate form that covers the happy path. It also checks that no non-generic member of the tier-1 types takes one of the library's callback interfaces.

## `[Experimental]` inventory

One marker remains: **`OPHYS0001`, the raw tier**, the `RecursiveAI.OnePhysics.LowLevel` namespace (body and constraint stores, colouring, broadphase trees, narrowphase routines, shape kernels) and the `Core` accessors of the two worlds. Its shapes follow the engine's internals and may change in a minor release. A consumer opts in with `#pragma warning disable OPHYS0001` or `<NoWarn>` in the project. The package itself, the README examples and the guide examples do not suppress it in code a user would copy.

Everything else is supported: the 3D engine, rods, ropes, cables, pulleys and belts, and the features added after the first previews carry no marker and follow the compatibility promise in the README.

## Documentation comments

The projects build with `GenerateDocumentationFile` and warnings as errors, so a missing comment on any public type or member of the runtime or `.Tools` package fails the build (CS1591).
