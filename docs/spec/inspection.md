# Inspection

Inspection is read-only, between steps, and does not enter a snapshot or world
hash. Both worlds expose `Counters` and `Profile`; both body handles expose
`Inspect()`. The raw world exposes the same objects and `InspectBody(handle)`.
These APIs do not require callbacks with struct constraints. They allocate only
when the world is created, not when a step or inspection runs.

## Optional detail collection

`Options.DetailedInspection` is false by default, in both dimensions and in the
raw world options. Counters, host-timestamp profiling, body sleep/island state,
and stored contact payload views remain available without it. Enable it before
stepping to collect whole-step body impulses, per-substep damping impulses,
joint reactions and joint row errors. Enable it before applying public direct
impulses as well if those inputs must be recorded. Changing the option between
steps takes effect on the next step. It is a host diagnostic setting: snapshots
and world hashes exclude it, and restore preserves the receiving world's setting.
Simulation bytes, events and hashes are identical with collection on and off.

Detail getters (including raw impulse/row spans) throw
`DetailedInspectionDisabledException`, with a message naming
`Options.DetailedInspection`, unless enabled for the latest step. A joint view
may be obtained before the first step when enabled; its detail values become
available after stepping. Disabling and stepping invalidates the previous detail.
The inspection debug overlay always includes constraint metadata and stored
contact impulses, but omits joint force/torque arrows and row labels when detail
collection is off. No detail is reconstructed by running the solver on demand.

## Counters

`StepCounters` is a world-owned object reused by each step. Copy values if they
must survive another step. Before the first step all values are zero. Restore
neither restores nor clears diagnostics; the next step replaces them. A failed
step may leave incomplete diagnostics.

- `AwakeBodies` includes dynamic and kinematic bodies in the solver input list.
- `Manifolds`, `ContactPoints`, `Joints`, `ConstraintsPerColor` (24 entries), and
  `SerialOverflow` count active solver inputs immediately after active-list
  construction. Sleeping contacts are excluded. These counts precede joint
  break processing.
- `AwakeIslands` and `AsleepIslands` count live islands after sleep processing.
- `BroadphasePairs` counts the sorted candidates returned by this step's move
  search, not the persistent pair table. `UpdatedPairs` counts persistent pairs
  selected for collision update, including reused manifolds and early rejections.
  `NarrowphasePairs` counts actual fresh geometric queries, excluding early
  rejection and successful convex or triangle-soup reuse. A pair can produce
  several manifolds in 3D.
- `WorldSubsteps`, `WorldSolvePasses`, `WorldRelaxPasses` and their `Contact*`
  counterparts count executions of each schedule program, not island count
  multiplied by that schedule. Warm-start and restitution passes are not included
  in solve or relaxation counts. An unused contact-only program reports zero.
- `SleepTransitions` and `WakeTransitions` count body transitions during the core
  step, independently of event masks. Explicit host transitions between steps
  are not counted.

Counters are combined on the stepping thread. Collision workers write only their
own chunk's count and chunks combine in ascending index order. Neither thread
count nor SIMD width determines a counter. Counters do not change stepping
options, scheduling, solver order, or floating-point calculations.

## Host-sourced profiling

Set `world.Profile.TimestampSource` to a parameterless function returning a
monotonic signed 64-bit timestamp. Null, the default, disables timing. The host
owns the clock and timestamp-to-seconds conversion. The source must not throw,
mutate a world, or step a world. Set it only between steps. Timestamp differences
and accumulated durations must fit signed 64-bit integers.

`Durations[(int)StepStage.X]` is the sum of timestamp differences for that stage
in the latest step, in host timestamp units. The source is captured at the start
of a core step. Durations are cleared even when timing is disabled. A decreasing
timestamp contributes zero and increments `InvalidTimestampCount`; it does not
alter simulation. All marks run on the stepping thread. Repeated soft-body stage
visits accumulate. `End` and `SoftEnd` are sentinels with zero durations. Gaps
between the rigid end and soft setup, and work after the soft end, are not timed.

Boundaries follow the serial pipeline: bounds; moves and initialization where
present; pairs (with search, sort, and buffer-finalization boundaries inside the
broadphase); narrowphase; ordered apply; colouring/scheduling; preparation;
solve/integrate; finalization; storage; sleep; events; rigid end. Soft worlds then
report setup, predict, detect, pairs, constraints, finish, and soft end. Stages
without a visited boundary stay zero. The source's own overhead is included in
the measured differences. Public collision callbacks are outside these stages.
Profiling is available in Release. The benchmark's profile mode uses this public
path; Profile builds can additionally collect the older solve-run diagnostics.

## Bodies and contacts

`body.Inspect()` creates a live value view. `Island`, `IsAwake`, and `SleepTime`
read current stores; sleep time is in seconds. Every body getter validates the
body generation, so an inspection held after destruction throws
`StaleHandleException`, even if the slot is reused. A default view throws
`InvalidHandleException`.

`Constraints` enumerates live contacts and joints involving that body in
ascending constraint-slot order. Sensors are excluded. Static bodies are included,
although they do not own solver adjacency lists. The enumeration scans constraint
slots: O(world constraint high-water), with no allocation. `ConstraintCount`
performs the same scan. Do not mutate the world while reading the enumeration or
any returned span or constraint view.

A constraint view identifies both bodies and its slot, colour (0..23, 255 for
serial overflow, 254 for uncoloured), and read-only persistent payload. A contact
additionally exposes its world-space A-to-B normal (two components in 2D, three in
3D) and its points in manifold order. Each point reports stored signed separation
(negative means penetration), normal impulse, and signed tangent impulses. The
second tangent impulse is explicitly zero in 2D; those payload slots are used for
other cached data. Point indexing outside `[0, PointCount)` throws
`ArgumentOutOfRangeException`.

**These impulses are final-substep warm-start values, not whole-step integrals.**
Their units are kg·m/s. At static equilibrium, dividing by the owning schedule's
substep dt estimates support force; dividing by full step dt does not. Separation
is the stored manifold-update value, not a fresh penetration calculation at the
final pose. Joint payloads have their type-specific layout and are not contact
points. The existing debug geometry already supplies contact normal impulses,
body sleep states, and island ids as plain data.


## Contact lifetime counters

The contact-apply stage marks newly allocated manifolds. For refreshed
manifolds, the existing warm-start feature matching also produces a matched-id
mask. The manifold-write stage reduces that mask to the number of new points
before it overwrites the old ids. Recycled manifolds keep their feature ids unchanged.
Chunk-local accumulators fold the completed pair outputs while those outputs
are still in cache, outside geometric and point-solving loops, and publish once
per chunk. Chunks combine in ascending index order. The core combines these markers with active-list point
counts, without copying/scanning a second full feature history each step.
Thus:

- `Manifolds = NewManifolds + PersistedManifolds + RecycledManifolds`;
- `ContactPoints = NewContactPoints + PersistedContactPoints + RecycledContactPoints`.

`CcdBodies` counts bodies whose motion passes the sweep threshold (including the
coarse static-bounds query in 3D); `CcdHits` counts those stopped before the end of
their sweep, once per body. `BulletBodies` is the selected bullet subset.
`SoftParticles` and `SoftBlocks` count particles and interior blocks of awake
soft bodies selected for the soft step; pinned particles are included, the
serial boundary region is not another block. `Characters` and `Vehicles` are
live controller counts before callbacks. Vehicles and soft bodies currently
report zero in 2D, where those controllers/stores are not present.

## Whole-step joint readback

`joint.Inspect()` and `world.InspectJoint(joint)` expose dimension-specific
`JointInspection` values. The raw view is `Core.JointReadback.Inspect(handle)`.
`ForceA`/`ForceB` are mean forces in world axes; `TorqueA`/`TorqueB` are mean
torques **about each body's centre of mass**, including the force moment at the
joint anchor. Torque is a signed scalar in 2D and a world vector in 3D. Forces
and torques are accumulated independently for A and B: gears, racks and pulleys
do not generally have equal and opposite component vectors.

At every scalar row application with Jacobian `(LA, AA, LB, AB)` and applied
impulse increment lambda, readback adds each Jacobian component times lambda to
its corresponding double accumulator. This includes warm starting and every
solve/relax pass, for every executed substep. SIMD point blocks report the actual
applied impulse vector before rounding it into the persistent warm-start
accumulator, and the float32 cross products of its current arms with that
impulse. Accumulators combine only within one constraint, in application order;
they are never reduced across workers. Readback divides the whole-step sums by
the supplied full-step dt, not substep dt. It never changes a velocity, persistent
impulse, row coefficient, break criterion, snapshot or world hash.

The raw impulse layout is twelve doubles per constraint slot: A linear xyz,
A angular xyz, B linear xyz, B angular xyz. Two-dimensional linear z and angular
x/y are zero. Before a joint's first step its forces and torques are zero. An
unsolved sleeping joint has zero last-step sums. `Joint.ReactionForce` and
`ReactionTorque` retain their earlier final-substep magnitude semantics and break
threshold behavior; these do not substitute for whole-step readback.

`Rows` contains 32 entries indexed by persistent impulse slot (the type-specific
slot maps are in the world specifications). Unused entries have `Flags=None`.
The last non-restitution pass reports the scalar solver error supplied to each
row, with role and activity bits: locked, spring, motor and limit rows. Errors
are metres or radians; nonlinear angular rows use their solver's small-angle
rotation coordinate, not a newly computed exact angle. A positive limit error
is remaining clearance; a limit is active if violated/touching or carrying a
nonzero impulse. A motor is active when enabled with nonzero capacity; its error
is target-relative velocity. Pose drives report their spring rotation error.
Gear and rack constraints couple velocities without a stored positional phase:
their rows explicitly report velocity error instead of inventing a position
error. Rows are cleared before each non-restitution pass, so an inactive swing
limit does not retain an old error. The colour and dynamic island identify the
last solve input (A's island when dynamic, otherwise B's).

A break record retains sums, row data, colour and island after destruction, and
`IsBroken` becomes true. A matching broken joint can still be inspected via its
handle or the happy path during that step's callbacks. Break records expire on
the next step or slot reuse. Other destroyed/default/foreign joints raise the
usual typed handle errors. Observations are not restored from snapshots.

Analytic checks use a 1% force tolerance for a hanging mass and applied couple,
and 2% per-rod force tolerance for a symmetric two-rod truss.

## Body input impulses

`BodyInspection` adds `GravityImpulse`, `ExternalImpulse`,
`ExternalAngularImpulse`, `DampingImpulse` and `DampingAngularImpulse`, all in
world axes. Happy-path inspection uses 2D/3D double linear vectors and scalar/3D angular impulses. Raw shared inspection uses xyz; 2D uses linear xy and angular z.
The raw `Core.BodyReadback.Impulses` has fifteen doubles per body slot in that
order. Gravity and force/torque inputs are integrated over the executed substeps.
Public direct impulses since the preceding step are included with external
inputs; forces applied through the public body API are included through their
integrated force/torque. Solver/contact impulses, service-internal impulses,
gyroscopic correction and speed clamps are not labelled external or damping.
Damping reports actual pre-clamp momentum change, including angular momentum
about the centre of mass. Gravity is reported before damping.

Both engines' per-lane arithmetic is unchanged: damping observations use
pre- and post-multiplication velocity values, then convert their difference to
momentum using mass/inertia. Damping momentum changes accumulate in double in substep order. Force, torque,
gravity scale and inverse mass are fixed during a step: the finalize stage copies
those inputs before clearing them, with duration `(double)(dt / substeps) *
substeps` from the body's schedule. Their double impulse projection is computed
once on first inspection, not in every substep; reading the raw impulse span
projects any remaining bodies. This lazy diagnostic cache is not simulation
state. Public direct impulses use a world-owned pending list, consumed at step
start. `ConstraintColorMask` uses bits 0..23 for colours and
bit 24 for serial overflow; it is computed by read-only constraint enumeration.

## Plain-data overlays

`DebugContent.Inspection` emits body sleep/island entries plus:

- `ConstraintInspection`: stable constraint slot, colour, endpoint centres of
  mass and both island ids;
- `InspectionVectors`: contact normal and combined tangent impulse arrows at
  each contact point, and mean joint force and COM torque arrows at B's centre
  of mass. Values retain physical units; viewers choose display scale;
- `JointErrors`: enabled row errors and their units/activity bits, labelled at
  B's centre of mass in constraint-slot then row-slot order.

Tangents use the solver's basis: clockwise normal rotation in 2D; in 3D choose
`(ny,-nx,0)` when `abs(nx)>=0.57735`, otherwise `(0,nz,-ny)`, normalize to t1,
and take `t2=cross(n,t1)`. A zero-length first tangent falls back to `(1,0,0)`.
No rendering or engine-specific interpretation is needed. Geometry owns and
reuses its arrays; its first capacity growth can allocate.
