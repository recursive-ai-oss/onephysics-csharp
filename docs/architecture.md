# Architecture of the shared core

This document is normative for the dimension-independent core in `src/OnePhysics/Core/`: everything a port must reproduce to get the same bits. The 2D and 3D engines sit on it and add shapes, collision, solvers and joints through the stage interface below. The core is **not frozen**: it may still change while the 3D engine is being built; each change updates this file.

All numbers are little-endian, floats are IEEE 754 binary32/binary64, and only `+ − × ÷ sqrt` are used on them.

## Layers and files

| File | Contents |
|---|---|
| `Handles.cs` | `BodyHandle`, `ShapeHandle`, `JointHandle`, `HandleTable` |
| `Exceptions.cs` | `PhysicsException` and its typed subclasses |
| `Options.cs` | `DimensionLayout`, `ManifoldLayout`, `EventMask`, `WorldOptions` |
| `BodyStore.cs` | struct-of-arrays bodies |
| `ConstraintStore.cs` | struct-of-arrays constraints, constraint adjacency, `ShapeStore` |
| `ConstraintColoring.cs` | graph coloring, batches, active lists |
| `Islands.cs` | persistent islands |
| `TaskExecutor.cs` | `TaskExecutor`, `IParallelBody`: the only place that uses threads |
| `Events.cs` | event kinds, `PhysicsEvent`, `EventCollector` |
| `State.cs` | `StateWriter` / `StateReader`, `ISnapshotSection` |
| `WorldCore.cs` | the world: pipeline, sleeping, origin, snapshot, restore, hash; `IWorldStages` |
| `Reference/ParticleScene.cs` | a tiny engine (particles, floor, grid contacts, distance joints) used by tests, fuzzing and smokes |

Struct generics are used where code is the same across dimensions and hot: the step pipeline is generic over `TStages : IWorldStages`, and each parallel phase is a generic struct implementing `IParallelBody`, so the JIT specializes the calls and nothing allocates on the serial path. The storage itself is not generic: positions, rotations, velocities and inertias are flat arrays whose per-body stride (`DimensionLayout`) is the only difference between 2D and 3D, so both engines share one scheduler, one island manager, one snapshot writer.

## Handles

A handle is `(world id, index, generation)`; the default value (generation 0) is the null handle and never valid. `WorldCore.Resolve` checks a handle and returns the slot index, or throws:

* `InvalidHandleException`: null handle, or a joint handle whose slot holds another kind of constraint;
* `ForeignHandleException`: made by another world (world ids come from `WorldOptions.WorldId`, or a process-wide counter when 0; the id is not part of snapshots or hashes, so a host that wants handles to survive restoring into a fresh world sets the id);
* `StaleHandleException`: the object was destroyed (its slot's generation moved on).

Slot allocation (`HandleTable`) is **lowest free index first**, implemented as a binary min-heap; with no free slot a new index above the high-water mark is used. A freed slot's generation increases by one; a slot whose generation would overflow is retired (never reused). Generations start at 1. The exact heap array is saved in snapshots, so allocation after a restore continues identically. Joints share the constraint index space with contacts and sensors; a `JointHandle` resolves only to a joint.

## Storage

Per body (index = body slot; slots below `HighWater` are valid, dead slots are zeroed with island −1):

| Column | Type | Lanes per body |
|---|---|---|
| position | double | 2 / 3 (relative to the world origin) |
| rotation | float | 2 (cos, sin) / 4 (quaternion x, y, z, w) |
| linear velocity | float | 2 / 3 |
| angular velocity | float | 1 / 3 |
| inverse mass | float | 1 (zero for static and kinematic) |
| inverse inertia | float | 1 / 6 (xx, xy, xz, yy, yz, zz of the symmetric tensor in the body frame) |
| sleep time | float | 1 |
| type | byte | Static 0, Kinematic 1, Dynamic 2 |
| flags | byte | bit 0 Awake, bit 1 Bullet, bit 2 NoSleep |
| island | int | −1 for non-dynamic bodies |

`CreateBody` validates: position finite and each component within ±2^21 (`ArgumentOutOfRangeException` naming the value and the bound), rotation finite with squared length within 1e-3 of 1, velocities finite, a dynamic body's inverse mass finite and positive. A dynamic body starts awake in its own island.

Per constraint (one index space for contacts, sensor overlaps and joints):

| Column | Meaning |
|---|---|
| kind | Contact 1, Sensor 2, Joint 3 |
| body A, body B | body slots (different; either may be static or kinematic) |
| shape A, shape B (+ generations) | shape slots, −1 / 0 for none |
| sub-shape A, B | compound child or mesh triangle id; part of the contact key |
| flags | bit 0 ReportPersist, bit 1 BreakRequested |
| color | 0..23, 255 overflow, 254 uncolored (sensors, constraints with no dynamic body) |
| point count | manifold points in use (contacts) |
| feature ids | `MaxManifoldPoints` uint per constraint, matches points across frames |
| break value | what triggered a break request |
| payload | `PayloadFloats` floats of persistent data |

**Manifold payload layout** (`ManifoldLayout`, floats): normal x, y, z (0–2), friction (3), restitution (4), rolling resistance (5), rolling impulse x, y, z (6–8), then up to four points of 12 floats: anchor on A (3), anchor on B (3), separation, normal impulse, tangent-1 impulse, tangent-2 impulse, max normal impulse. 2D uses the first two points. `PayloadFloats` is 40 for 2D (header + 2 points + 7 spare) and 64 for 3D (header + 4 points + 7 spare). **Joint payload** is engine-defined: up to six rows (one per degree of freedom) fit in the same floats.

Shapes are slots with an owner body (`ShapeStore`); geometry lives in the engines' own arrays indexed by shape slot and is saved through `ISnapshotSection`.

**Constraint adjacency.** Every constraint with a dynamic body is linked into that body's doubly linked edge list (edge = 2 × constraint + side). Static and kinematic bodies have no list (the ground would have thousands of entries). The list order is not part of any result.

## Scheduler

### Coloring

`ConstraintColoring` assigns each non-sensor constraint with at least one dynamic body a color in 0..23 when it is added: the lowest color that is free on both of its **dynamic** bodies (Catto, Solver2D and the Box2D v3 release posts on graph coloring). Static and kinematic bodies take no part. A constraint between two dynamic bodies searches colors 0..23 upward; a constraint touching one dynamic body searches 20..23 first, then 0..19, so ground contacts are solved late in an iteration. A constraint that finds no free color is **overflow** (255). Each dynamic body keeps a 64-bit mask of the colors in use; removing a constraint clears its bit. The color is a pure function of the order of additions and removals, stored per constraint, saved in snapshots, and independent of thread count and vector width. Restoring checks that no two constraints on a dynamic body share a color.

Within a step the active constraints (not sensors, not uncolored, at least one awake dynamic body) are bucketed by color in ascending constraint index: the active list is `overflow, color 0, color 1, …, color 23`.

### Batches

A **batch** is `ConstraintColoring.Lanes` (8) consecutive constraints of one color in the list; the last may be short. 8 is a constant of the format, not of the hardware: a kernel of any vector width walks a batch with the same grouping, and constraints of one color touch disjoint dynamic bodies so lane order inside a color cannot change a result. A parallel **task** is `BatchesPerTask` (4) consecutive batches; chunks of bodies and constraints for the other phases are 64 consecutive items. Splits depend only on counts, never on thread count.

### Executor

`TaskExecutor.For(taskCount, body)` runs tasks `0..taskCount−1` on at most `ThreadCount` threads (the shared worker pool when more than 1, a plain loop for 1, which is the only mode on single-threaded WASM). Each task writes only data it owns, so any interleaving gives the same bits. `ThreadCount` comes from `WorldOptions` (default 1), is fixed at world creation and is never read from the machine. The core's only thread-dependent scheduling is in `TaskExecutor.cs`. `TaskExecutor.ForStages(stageCount, body, mostTasks)` runs a sequence of stages (`IStagedBody`: task count per stage, run a task) in one call: every thread walks all stages in order, runs its own contiguous block of the stage first and then helps with the other blocks, and waits for the whole stage before the next one. The solver runs all substeps as one staged call (integrate velocities, then for each pass the overflow constraints serially and each color in ascending order, integrate positions, ...). A call with fewer than four tasks in every stage stays on the calling thread.

### Pipeline

`WorldCore.Step<TStages>(stages, dt)` (dt finite and positive; options revalidated):

1. clear last step's delivered events; build the awake-body list (as of the start of the step);
2. `UpdateContacts` (serial): broadphase pairs and narrowphase; add and remove constraints in ascending key order;
3. rebuild the awake list (new constraints may have woken islands); build the active lists;
4. `Prepare` over chunks of the active list;
5. for each of `Substeps`: `IntegrateVelocities` over chunks of awake bodies; solve phase `WarmStart`; `VelocityIterations` × phase `Solve`; `IntegratePositions`; `RelaxIterations` × phase `Relax`. A solve phase runs the overflow set serially in batches, then colors 0…23 in order, each color's tasks in parallel;
6. phase `Restitution` once, then `FinalizeBodies` over chunks of awake bodies (default no-op: forces, continuous collision);
7. `StoreResults` over chunks (may flag joint breaks); breaks are processed serially in ascending constraint index (joint removed, break event queued);
8. sleeping (below);
9. persist events; deliver events (sorted); step counter +1.

Dimension-specific work is behind `IWorldStages` (`UpdateContacts`, `Prepare`, `IntegrateVelocities`, `SolveBatch`, `IntegratePositions`, `FinalizeBodies`, `StoreResults`, `BodyMotion`). `WorldCore.Sleep(body)` puts an island to sleep immediately. Contract: everything except `UpdateContacts` and `BodyMotion` may run concurrently on disjoint data; solvers write only the dynamic bodies of their batch and treat static and kinematic bodies as read-only; mutable stage state lives in reference-type fields written serially.

## Islands and sleeping

Islands hold dynamic bodies connected by non-sensor constraints between two dynamic bodies (Catto, "Simulation Islands", Box2D v3 blog). Each dynamic body is created in its own island.

* **Merge** when such a constraint is added: the lower island id survives, its body list is the ascending merge of both. Adding any non-sensor constraint first wakes the islands of its dynamic bodies.
* **Removal** only increments the island's removal counter (and wakes the dynamic bodies' islands).
* **Split** happens in the sleeping pass: among awake islands with removal count > 0 whose slowest body has rested at least `TimeToSleep`, the one with the largest rest time (lowest id on ties) is split by depth-first search over the constraints between dynamic bodies. Components are discovered from the lowest unvisited body: the first keeps the island id, the others take the lowest free ids in discovery order. Each island's body list is ascending; counters reset. At most one split per step.
* **Sleep time:** each awake dynamic body adds `dt` to its sleep time when `BodyMotion ≤ SleepThreshold`, else resets to 0 (always 0 with `SleepEnabled` false or the NoSleep flag). An awake island with removal count 0 whose minimum sleep time is ≥ `TimeToSleep` sleeps: bodies lose the Awake flag, velocities are zeroed, a Sleep event per body is queued. Islands are visited in ascending id.
* **Wake:** `Wake(body)`, adding or removing a constraint on a body, or merging wakes the whole island: Awake set, sleep time 0, a Wake event per body. Constraints of sleeping islands are inactive (not in the active lists, not solved).
* Moving kinematic bodies do not keep touching bodies awake by themselves; engines wake what a kinematic body touches (`Wake`).

Saved island state is the id table, each island's sleep flag and removal count, and each body's island id; lists are rebuilt in ascending body order.

## Events

Kinds in sort order: ContactBegin 0, ContactPersist 1, ContactEnd 2, SensorEnter 3, SensorExit 4, JointBreak 5, Sleep 6, Wake 7, ContactHit 8 (opt-in; `Value` is the approach speed). `WorldOptions.Events` masks kinds (default: all but ContactPersist and ContactHit); a snapshot accepts mask bits up to 0x1FF.

* Adding a contact or sensor queues Begin/Enter; removing queues End/Exit; neither is queued for joints (a break queues JointBreak with the break value); sleeping and waking queue one event per body; ContactPersist is queued at the end of a step for awake contacts flagged ReportPersist.
* Events are collected into per-slot buffers (serial code uses slot 0, parallel stage code its own chunk slot) and delivered by sorting with `PhysicsEvent.Compare`: kind, then body A, body B, shape A, shape B, sub-shape A, sub-shape B, joint (slot indices), then the same handles' generations, then the value's bits. The comparison is total over every field, so delivery order is independent of slots and threads.
* Events raised between steps (destroying a body, creating a joint) wait in the collector and are delivered at the end of the next step. `EventCollector.Delivered` stays valid until the next step starts.

## World hash and snapshot

**State order** (shared by snapshot and hash):

1. options: layout (7 × i32: position, rotation, linear, angular, inertia lanes, max manifold points, payload floats), gravity (3 × f32), substeps, velocity iterations, relax iterations (3 × i32), sleep enabled (u8), time to sleep, sleep threshold (2 × f32), event mask (u32). Thread count and world id are not saved.
2. step count (i64); origin (position lanes × f64).
3. bodies: handle table, then the columns of the table above in the order position, rotation, linear velocity, angular velocity, inverse mass, inverse inertia, sleep time, type, flags, island, each for all slots below the high-water mark, then the slot count again as a framing check.
4. shapes: handle table, owner (i32 per slot).
5. constraints: handle table, then kind, body A, body B, shape A, shape A generation, shape B, shape B generation, sub-shape A, sub-shape B, flags, color, point count, feature ids, break value, payload, then the slot count.
6. islands: handle table, then per slot: asleep (u8), removal count (i32).
7. (snapshot only) events: delivered count and events, pending count and events; an event is its kind (u8), body A, body B, shape A, shape B (world, index, generation: 3 × i32 each), sub-shape A and B, joint (3 × i32), value (f32).
8. sections: count (i32), then per registered section its id (i32); in a snapshot also its byte length (i32) and bytes.

A **handle table** is: high-water mark (i32); generation per slot (i32); live per slot (u8); free-list length (i32); the free min-heap array (i32 each).

**Snapshot** = u32 magic `0x4E53504F` (bytes `O P S N`), u32 version (**1**), then the state in the order above. **Restore** validates everything before touching the world: magic and version; layout equal to the world's; options valid; origin, positions, rotations, velocities, masses, inertias, sleep times, payloads finite; handle tables (generations ≥ 1, free list a valid min-heap of distinct dead slots, every dead slot free or retired); dead slots zeroed; body type, flags, mass and island rules; constraint kind, flags, point count, bodies and shapes live with matching generations, bodies distinct; colors consistent and no two constraints on a dynamic body sharing one; every dynamic body in a live island whose sleep flag matches its Awake flag; constraints joining dynamic bodies of one island; counts never larger than the remaining bytes could hold; section count and ids; no trailing bytes. Any problem throws `SnapshotFormatException` (with the byte offset) and leaves the world as it was; failures inside a section are undone by restoring a snapshot taken before.

**World hash** = the 64-bit hash (`StateWriter`) of the state order with the options, step count, origin, bodies, shapes, constraints, islands and sections, **without** the magic, version, events and section lengths. h starts at `0xCBF29CE484222325`; each primitive is widened to an unsigned 64-bit word w (u8, i32 and u32 zero-extended as 32-bit patterns, floats and doubles by IEEE bit pattern) and folded as `h = (h xor w) × 0x9E3779B97F4A7C15 (mod 2^64)`, then `h = h xor (h >> 32)`. Equal states hash equal on every platform; the hash matrix compares it per step.

Registered sections (`ISnapshotSection`) let an engine save its own state (shape geometry, joint parameters) in the same file and hash; a world restoring a snapshot must register the same sections in the same order.

## Origin and coordinates

Body positions are double and relative to the world origin; the public boundary takes them as double, exact within ±2^21 (a 31.32 fixed-point value converts exactly there), and rejects a component beyond it. Rotations, velocities and solver math are float32. `ShiftOrigin(offset)` subtracts a double offset (finite, a multiple of 2^-31, magnitude ≤ 2^22, one value per position lane) from every body position and adds it to the saved origin, so `origin + position` is unchanged; positions on the 2^-31 grid shift exactly. The host calls it between steps, on every peer at the same step. Engines shift whatever they keep in world coordinates themselves. The origin is part of the snapshot and the hash.

## 3D requirements

| 3D needs | How the core meets it |
|---|---|
| Larger manifolds | Four manifold points × 12 floats plus a 9-float header with rolling impulse; 64 payload floats per constraint in 3D; feature ids per point (4 per constraint). 2D uses the same code with 2 points and a 40-float payload; its engine keeps a body-fixed copy of the manifold in the floats the layout leaves unused (`world2d.md`, manifold recycling). |
| 6-DOF joint rows | The payload holds up to six rows of engine-defined state; coloring and islands treat joints like contacts (a joint is a constraint with kind Joint, a handle, and a break value); joint breaks are a core event. |
| Compound and mesh sub-shape ids in contact keys | Every constraint stores shape A/B with generations and sub-shape A/B; events and the event sort order carry them; feature ids match points across frames. |
| CCD and speculative contacts | Speculative contacts are ordinary contacts created within a margin (the stage decides the margin; separation may be positive in the payload). Continuous collision runs inside the engine's `UpdateContacts`/`IntegratePositions`; the Bullet body flag is stored; `Restitution` and `StoreResults` phases exist for post-solve work; contacts may be added in any step in ascending order. |
| Substepping | `Substeps`, `VelocityIterations`, `RelaxIterations` are options; the pipeline runs the soft-step loop (integrate velocities, warm start, solve, integrate positions, relax) per substep; `StepContext` carries the substep length and its reciprocal computed once. |
| Soft-body particles | A particle is a dynamic body that uses the position, linear velocity and inverse mass lanes only; it gets islands, sleeping, colors and snapshots for free. Edge, bend and volume constraints are constraints of new kinds referencing particle slots (the coloring is kind-agnostic; the byte for the kind has room). A soft body is one island. Rows beyond six per constraint, or very many particles, may want a second payload stride; that is a later addition. |
| Many constraint rows per body pair | Several constraints between one pair of bodies are separate constraints with different colors. |
| Large worlds | Double positions plus origin shifting. |

## Determinism notes

* No transcendental functions, FMA, estimates, clocks, randomness or hash-order iteration in the core. Ties break on indices; every sort uses a total order.
* Parallel phases split work by counts, write disjoint data and combine nothing across tasks; events are sorted at delivery.
* The core has no vector kernels yet; the batch size is a constant (`ConstraintColoring.Lanes`) so kernels can be written at any width and processed per batch with scalar twins.
* The reference particle engine is for tests: float math follows the same rules, with `Math.Sqrt` as the only function beyond arithmetic.
