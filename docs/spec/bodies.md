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
