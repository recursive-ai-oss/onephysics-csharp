# Inspection

Inspection is read-only, between steps, and does not enter a snapshot or world
hash. Both worlds expose `Counters` and `Profile`; both body handles expose
`Inspect()`. The raw world exposes the same objects and `InspectBody(handle)`.
These APIs do not require callbacks with struct constraints. They allocate only
when the world is created, not when a step or inspection runs.

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
  `RecycledManifolds` counts successful manifold reuse, not cached separation
  rejections. A pair can produce several manifolds in 3D.
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
