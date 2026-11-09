# Body controls

This document applies to both dimensions. New body controls are optional; an
unconfigured body retains the original world-wide behavior.

## Sleep

`Body.SleepSettings` and `BodyDef.SleepSettings` are a `BodySleepSettings` value.
The default value inherits `WorldOptions.SleepThreshold` (the existing engine
motion measure: linear speed plus angular speed times maximum extent) and
`TimeToSleep`. An explicit value uses separate linear speed (m/s), angular speed
(rad/s), and required continuous rest time (seconds). Thresholds are finite and
nonnegative; time is finite and positive. In 2D angular speed is absolute angular
velocity; in 3D it is the Euclidean length of world angular velocity. Explicit
threshold comparisons sum squared components in double, in axis order X/Y/Z.
The existing engine motion measure still prevents sleep while touching an active
kinematic body. World sleep must be enabled. Every body of an island must satisfy
its own duration before that island sleeps. Split candidates compare rest time
minus the body's required duration, with the existing island-id tie break.

`Body.NeverSleep` mirrors the existing `BodyDef.AllowSleep = false` flag and
prevents automatic sleep for the entire island. Explicit `Sleep` remains allowed.
Changing either property wakes the island and clears that body's rest timer,
including when replacing a policy by the inherited default. Static and kinematic
bodies reject policy setters with `BodyTypeException`. All accessors validate the
body generation. Raw callers use `WorldCore.GetBodySleepSettings` and
`SetBodySleepSettings`; `BodyBits.NoSleep` is the raw never-sleep flag.

### Snapshot section

Core snapshot format remains version 1. The optional section `0x42534C50` is
registered before engine sections and present only when a live body has an
explicit sleep policy. Its version-1 payload is little-endian:

- int32 version (1), int32 policy count (positive);
- per explicit policy in ascending body slot: int32 slot, float32 linear
  threshold, float32 angular threshold, float32 rest duration.

Slots must be live dynamic bodies in the incoming core state, unique and sorted.
All values are range-checked before replacing the store. Restore without this
section resets policies to inheritance. Destroying a body removes its entry;
reused slots do not inherit policy. NoSleep and the accumulated rest timers stay
in the unchanged core body state. Fork copies the policy store independently.
The section is written through the canonical snapshot writer and thus included
in the world hash. Worlds with no overrides retain their snapshot bytes and hash.
Named body fields participate in desync and region tooling. Full-state scene text
carries this section unchanged; replay records the named `Body.SleepSettings` and
`Body.NeverSleep` mutation effects.

## Fixed-step presentation

Each dimension supplies `FixedStepAccumulator(world, fixedDt = 1/60f,
maximumStepsPerFrame = 8)`, `BodyPose`, and previous/current/interpolated pose
accessors. Only the helper receives variable frame durations. Neither simulation
nor the world reads a clock. Fixed dt is float32, finite and positive. The
accumulator uses double seconds: add each finite nonnegative caller duration,
then subtract the exact double representation of fixed dt after each successful
step, up to the positive step limit. Overflowing the accumulated time is rejected
before it changes. No epsilon, rounding of step counts, dropped time, or dt
stretching is used. A zero-time call may consume backlog. Exceptions from stepping
propagate and do not consume a dt for the failed step.

Before each step capture live body-origin poses; after it capture the new poses.
Storage is indexed by slot and generation, growing only when body storage grows.
New bodies have previous = current until first stepped; stale and foreign handles
throw. Bodies created during step callbacks also start with previous = current.
`alpha = min(1, remainder / fixedDt)`; capping therefore renders the newest pose
while retaining the full backlog for future frames. Normally interpolation renders
one fixed step behind the simulation.

Positions use double linear interpolation. 2D rotation uses the shortest signed
arc, `atan2(cross, dot)` followed by sine/cosine from deterministic math. A signed
half-turn tie follows atan2. 3D uses onemath quaternion Slerp. Alpha zero/one returns
the stored endpoint exactly. Poses are presentation data only, never simulation
input and never in snapshots, replay effects or world hashes. The helper is not a
world owner: it does not dispose, restore or shift the world. Call `Resynchronize`
after a teleport, external step, snapshot restore or origin shift; it resets both
pose endpoints without clearing carried time.

## Rigid constraints and limits

`Body.Controls` / `BodyDef.Controls` are `BodyControlSettings`: world-axis
`RigidbodyConstraints`, maximum linear speed (m/s), angular speed (rad/s), and
contact depenetration speed (m/s). Convenience body properties preserve the other
settings. Limits are finite and nonnegative; default `float.MaxValue` inherits
the world cap. The world retains `MaxLinearSpeed = 400`, `PushOutVelocity = 3`
and `MaxRotationPerSubstep`; the new `MaxAngularSpeed = float.MaxValue` adds no
cap by default. `MaxDepenetrationSpeed` is an alias for `PushOutVelocity`.

Locks use the centre-of-mass position and world angular velocity, not the body
origin or Euler angles. Only X/Y position and Z rotation apply in 2D. The common
FreezePosition, FreezeRotation and FreezeAll composite aliases are accepted;
individual inapplicable axes are refused. Angular velocities on locked axes are
zero before constraints see them and after every substep, including the final
restitution pass. Rotational motion on allowed world axes may change Euler
angles about other axes; Euler coordinates are not the constrained quantities.

Following Catto, *Understanding Constraints*, GDC 2014, the constraint mass is
`K = J M^-1 J^T`, and velocity response is `M^-1 J^T lambda`. A locked linear
axis has zero inverse mass in **both** expressions. In 3D, restrict the world
inertia quadratic form to the allowed angular coordinates and invert that
principal block. This includes ideal lock reaction torques; simply erasing rows
of an inverse tensor gives the wrong response for rotated anisotropic inertia.
The solver uses that restricted inverse inertia for contacts, joints and torques.
Projected point-joint and rolling blocks use a pseudoinverse when singular; in a 3-row block,
onemath's symmetric eigen decomposition treats eigenvalues at most 1e-6 times
the largest as numerical null modes. Scalar 2-row rank-one blocks use the exact
trace formula. These rules affect configured worlds only. Contact reductions
and row/feature order otherwise remain unchanged. The contact kernels specialize
standard and configured policies: unconfigured worlds retain the original scratch
row stride and do not load control-only fields. For joints touching locked bodies, point
blocks use the scalar joint-row solver rather than the unrestricted wide block.

Linear and angular caps limit magnitudes, not components; the stricter per-body
and world value wins. Capped scaling is rounded down one float step so magnitude
does not overshoot because of division rounding. Contact bias is bounded by the
minimum of the two bodies' depenetration limits and world PushOutVelocity.
This is a normal relative push-out speed cap, not a cap on externally supplied
motion, restitution or motor energy. Speed caps apply after forces, before pose
integration and after the final constraint pass. Explicitly selected world
linear caps also apply after solving, without requiring a body override.

Paper: https://box2d.org/files/ErinCatto_UnderstandingConstraints_GDC2014.pdf
(SHA-256 4649a7ca9a6d8b00af28725d1fd0816c85ae5941eb570aeea41510bfb6bd5c5d).

## Mass properties

Mass (kg), local centre of mass (m), and inertia about that centre (kg m²) can be
set independently and reset independently. Mass and positive principal moments
must lie in 1e-18..1e30; centre components are finite within ±2097152. 2D uses a
scalar inertia. 3D takes a principal diagonal and its orientation in the body
frame (normalized on assignment), using onemath inertia transforms. The inverse
is computed from reciprocal principal moments, avoiding determinant underflow.
Changing mass alone does not rescale inertia. Without an inertia override, shape
inertia is recomputed about the chosen centre using shape densities. Changing the
centre keeps the body origin fixed. Overrides survive later shape changes.
Only dynamic bodies support mass-property setters; invalid values fail before
changing mass state. The BodyDef nullable override fields use the same rules.
Raw-handle world mutators and `ResetBodyMassProperties` provide the same behavior.

## Kinematic targets

`MoveKinematic`, `MovePosition` and `MoveRotation` queue the next step's target.
Component setters preserve the latest queued component, or the current component
when no target is pending. Positions are rounded to the 2^-32 grid. 2D records
the deterministic full-angle cosine/sine; 3D normalizes the quaternion once.
Before stepping, all pending target motions are validated against axis locks and
speed limits at the chosen dt. Conflicts throw an argument error without clearing
an earlier drive's velocity. Targets outside the public coordinate bound, invalid
rotations and non-kinematic bodies are rejected. Dynamic soft drives are not
provided by this API; use joints for that purpose.

The target implies centre-of-mass linear velocity and shortest-arc world angular
velocity over dt, visible to contacts, characters and broadphase throughout the
step. At the end, the stored pose is set to the exact canonical target endpoint,
removing float integration error. Pending targets are consumed once. A driven
kinematic body stops on the next step unless given another target (last-step
velocity remains available until then). Explicit linear/angular velocity setters
or teleporting cancel the queued/driven mode and return to velocity control.
Pending targets shift with the origin.

### Additional snapshot section

Section `0x4243544C`, version 1, is optional and registered after the sleep-policy
section, before broadphase and engine sections. It is present when any body uses
rigid controls/overrides/kinematic driving or world MaxAngularSpeed is not its
default. Its little-endian payload is:

- int32 version = 1, float32 world angular speed cap, int32 entry count;
- ascending live body slot records: int32 slot; int32 constraint bits; float32
  linear/angular/depenetration caps; byte override flags; float32 mass;
  three float32 centre coordinates; three float32 principal moments;
  four float32 inertia-frame quaternion components XYZW;
  three float64 target-position coordinates; four float32 target rotation values.

Flags: 1 mass override, 2 centre override, 4 inertia override, 8 pending target,
16 previously driven kinematic body (stop without another target). In 2D target
rotation is `(0,0,sin(angle),cos(angle))`; in 3D it is a quaternion. Unused scalar
and vector fields start zero and resets clear them. Planar Z centre/target and
principal Y/Z are zero. The kinematic marker remains after target consumption;
target values themselves are cleared. Restore validates against the incoming core
body table, not the old world's table. Fork copies every field, no subscriptions.
Named mutations, full-state scenes, region/desync tooling and world hashing use
the section through the standard writer. Worlds that use no new controls retain
their original snapshot bytes and per-step hashes. World angular-option mutation
uses the same named mutation transport rather than changing the fixed-size legacy
live-option payload.

## Authoring scene records

Version-1 authoring scenes additionally accept these records. Body records apply
to the most recent rigid body (not a character-script record). SI units and
validation match the public controls above. Records are optional and omitted
from canonical output when unused, preserving existing generated scene bytes.

```
world_limits <maxLinear> <maxAngular> <maxDepenetration>
controls <constraintBits> <maxLinear> <maxAngular> <maxDepenetration>
sleep <neverSleep:0|1> <linearThreshold> <angularThreshold> <restSeconds>
mass <kg>
center <x> <y> [<z>]
inertia <scalar>                            # 2D
inertia <Ix> <Iy> <Iz> <qx> <qy> <qz> <qw>   # 3D
target <x> <y> <angle>                      # 2D
target <x> <y> <z> <qx> <qy> <qz> <qw>      # 3D
```

`sleep` with all three numeric values zero inherits the world sleep policy while
still allowing neverSleep to be set. `world_limits` is world-wide, not attached
to the current body. Repeated optional records replace their previous value.
Nullable public authoring properties expose the same optional mass and target
records. All new records participate in canonical authoring write/read; full
version-2 state text continues to transport the entire snapshot unchanged.
