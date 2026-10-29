# Names and conventions in the public API

One place for the choices a host has to know. The two worlds (`PhysicsWorld2D`, `PhysicsWorld3D`) share every name here; only the math types differ.

## Units and sizes

- **Full sizes.** Every `Add*` one-liner and every `CreateBox` takes full extents (`AddBox(size)`, `CreateBox(body, size, ...)`, `ShapeSpec3D.Box(size)`). A parameter that is a half extent says so in its name: `halfLength` (capsule, both dimensions), `halfHeight` (cylinder), `RagdollPartDef.ShapeHalfSize`, and the cone's `halfAngle`.
- **Radians everywhere.** All angles, limits, motor speeds and rates are radians (per second). A parameter that is an angle in a method that also takes other numbers is named for it: `angleRadians` on the 2D creators, queries and `SetTransform`. `CharacterDef.SlopeLimit` and `Character.SlopeLimit` are radians; Unity's degrees need converting.
- **Meters, kilograms, seconds.** Positions cross the boundary as `double`; everything else is `float`.

## Handles

- `Body`, `Shape`, `Joint` (and its typed views), `Character`, and in 3D `Ragdoll` and `Vehicle` are small structs (world, index, generation). They allocate nothing and throw `StaleHandleException` naming the object once it is destroyed.
- The raw, serializable form of each is its `RawHandle` (`BodyHandle`, `ShapeHandle`, `JointHandle`, `CharacterHandle`, `RagdollHandle`, `VehicleHandle`). The world turns one back into a live struct with `GetBody`, `GetCharacter`, `GetRagdoll`, `GetVehicle` and answers `IsBodyAlive`, `IsCharacterAlive`, ... without throwing.
- Event and query results (`Collision`, `Trigger`, `QueryHit2D/3D`, `ControllerColliderHit`, `JointBreak`) expose their bodies, shapes and joints as readonly **fields**, so `hit.Other.LinearVelocity = v` compiles. `Body.SetLinearVelocity`, `SetAngularVelocity` and `SetPosition` do the same where a property cannot be assigned (a list element, a ref return).
- Both dimensions use the same type names. To use both in one file, alias the ones you need: `using Body3D = RecursiveAI.OnePhysics.Physics3D.Body;`.

## Joints: one-liner, definition, view

The `Add*` one-liners use Unity's vocabulary; the definitions (`CreateJoint(ref XDef)`) use the engine names. Both return the same typed view, which converts implicitly to `Joint`. Members of the general `Joint` that do not apply to its kind throw `InvalidJointException` naming the kind.

| One-liner (2D) | Definition | Typed view |
|---|---|---|
| `AddHinge` | `RevoluteJointDef` | `HingeJoint` |
| `AddSlider` | `PrismaticJointDef` | `SliderJoint` |
| `AddFixed` | `WeldJointDef` | `FixedJoint` |
| `AddWheel` | `WheelJointDef` | `WheelJoint` |
| `AddDistance`, `AddSpring` | `DistanceJointDef` | `DistanceJoint` |
| `AddTarget` | `MouseJointDef` | `TargetJoint` |
| `AddRelative` | `MotorJointDef` | `RelativeJoint` |

| One-liner (3D) | Definition | Typed view |
|---|---|---|
| `AddFixed` | `FixedJointDef` | `FixedJoint` |
| `AddPoint` | `PointJointDef` | `PointJoint` |
| `AddDistance`, `AddSpring` | `DistanceJointDef` | `DistanceJoint` |
| `AddHinge` | `HingeJointDef` | `HingeJoint` |
| `AddSlider` | `SliderJointDef` | `SliderJoint` |
| `AddCone` | `ConeJointDef` | `ConeJoint` |
| `AddCharacterJoint` | `SwingTwistJointDef` | `SwingTwistJoint` |
| `AddConfigurable` | `SixDofJointDef` | `SixDofJoint` |
| `AddPath` | `PathJointDef` | `PathJoint` |
| `AddGear` | `GearJointDef` | `GearJoint` |
| `AddRackAndPinion` | `RackAndPinionJointDef` | `RackAndPinionJoint` |
| `AddPulley` | `PulleyJointDef` | `PulleyJoint` |

Member names are the definition's names on the handle:

| Definition field | Joint member |
|---|---|
| `EnableLimit`, `EnableMotor`, `EnableSpring` | the same, read and write |
| `LowerAngle`, `UpperAngle` (revolute) | the same |
| `LowerTranslation`, `UpperTranslation` (prismatic, wheel) | the same |
| `MinLength`, `MaxLength` (distance) | the same |
| `MaxMotorTorque` (rotational motors), `MaxMotorForce` (linear motors) | the same; asking a rotational joint for a force, or the reverse, throws and names the right member |

## Characters

Presets are on the definition: `CharacterDef.Humanoid`, `Child`, `Giant` (and in 3D `RigidHumanoid`). Take one, change fields, pass it to `AddCharacter`.

## Callbacks and rollback

A body's `OnCollision*`, `OnTrigger*`, `OnSleep` and `OnWake` handlers belong to its handle, so they survive a `Restore` that brings the same handle back; the world keeps the handlers of the 1024 most recently destroyed bodies, joints and characters of each kind, and only once it has taken a snapshot (a world that never snapshots drops them at destroy), so anything older is released.

Rollback hosts should read polled events (`Events`, `GetEvents`) and resimulate inside `using (world.SuppressCallbacks())`: no handler fires there, polled events are still produced, and the scope allocates nothing and nests. `Restore` returns an `OptionChanges` value naming the construction options the snapshot replaced (`None` when it changed none).
