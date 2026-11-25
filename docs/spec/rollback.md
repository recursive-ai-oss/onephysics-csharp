# Marks and rollback

`Mark()` and `RollbackTo(mark)` on `PhysicsWorld2D` and `PhysicsWorld3D` (identical members). A rollback host marks every frame and goes back to an old frame when a late input arrives. The same cost model serves a planner that tries several futures from one state. This page is normative for what a mark holds, how marks live and die, and what the cost is; the method and the numbers that chose the design are at the end.

## Members

| Member | Meaning |
|---|---|
| `WorldMark Mark()` | Saves the whole simulation state. |
| `OptionChanges RollbackTo(WorldMark)` | Returns the world to the marked state; returns the saved options the rollback changed, as `Restore` does. |
| `bool ReleaseMark(WorldMark)` | Frees one mark's storage for reuse. False (never an exception) when the mark is not valid. |
| `void ReleaseMarksBefore(WorldMark)` | Frees every live mark older than the given one. |
| `int ReleaseOldMarks(int keep)` | Frees the oldest live marks until `keep` remain; returns how many it freed. The ring helper: needs no mark in hand, so it works after a rollback has shortened the history. |
| `void ReleaseAllMarks()` | Frees every mark and the memory behind them. |
| `int LiveMarkCount`, `bool IsMarkValid(WorldMark)` | Inspection. |

`WorldMark` is a small value (world id, slot, generation, and the step count at the mark: `Step`). It owns nothing. The default value is the null mark.

## What a rollback guarantees

After `RollbackTo(m)` the world is the world at `Mark()` that returned `m`: the snapshot has the same bytes, `ComputeHash()` is the same, and every later step, with the same inputs, gives the same bits as a world that never rolled back. Everything a snapshot holds is covered, including contact caches and warm-start impulses, coloring, islands, sleep state, the delivered and pending events, the origin, the saved options, the length unit and every optional section (soft bodies, rods, fluids, water, destruction, articulations, characters, vehicles, ...). Anything created after the mark is gone, anything destroyed after it is back, and handles taken before the mark are valid again (the same rules as `Restore`, including the handlers the world keeps for destroyed handles).

Not part of the state, as for a snapshot: event handlers, delegates and callbacks, the struct contact filter, worker threads, capacities and the like. The per-step lists some optional features publish and a snapshot does not hold (shallow-water events, character water events) are empty after a rollback until the next step, exactly as after `Restore`.

## Lifetime

- Marks are numbered in the order they were taken. Any number may be live.
- `RollbackTo(m)` keeps `m` valid (roll back to it as often as you like) and keeps every older mark. It **releases every mark taken after `m`**: their state describes a future that no longer exists. Using one afterwards throws `StaleHandleException`.
- A released mark, a mark passed by a rollback and a mark from `ReleaseAllMarks` are invalid for good: a slot's generation changes every time it is reused, so an old mark never revives.
- A null mark or a mark of another world throws `InvalidHandleException`.
- `Restore`, `Fork`, `Snapshot`, steps and every mutation leave existing marks valid. A fork does not inherit marks.
- Marks are not part of a snapshot, a replay or the hash. A recording sees `RollbackTo` as one mutation (its effect is the complete state, as for `Restore`); `Mark` is not recorded.
- Call both between steps on the world's owning thread, not from inside a callback.

## Cost

- **Memory.** A mark keeps its own copy of the world's used state: about the size of the world's stores up to their high-water marks (bodies, shapes, constraints with their payloads, islands, events) plus the written form of each optional section that holds state. For a pile that is a little more than a snapshot (the constraint payload is copied whole, not compacted). A ring of N marks costs N times that. Storage is pooled per slot: releasing a mark keeps its arrays for the next one.
- **Time.** `Mark` and `RollbackTo` copy memory (`Array.Copy` of the used prefix of each store array) and serialize or parse only the sections that hold state; neither validates (the bytes are the world's own). Cost therefore follows the amount of live state, not the capacity, and a world with no optional features pays nothing for them. There is no per-step cost while no mark is live and none while one is: stepping is unchanged.
- **Allocation.** Once a slot has held a mark of the world's present size, marking into it, rolling back and releasing allocate nothing (a ring of marks, tested with a counter). The first mark of each slot allocates, and so does a mark after the world's storage has grown (more contacts than the arrays held, new bodies past the capacity) because the slot's arrays are resized to match. `Warmup()` and the capacity options size the world's storage so that growth happens before the ring is warm. A world with soft bodies, vehicles and the like allocates in proportion to what those features write.

## What does not move

Snapshot bytes, hashes and step results are unchanged; nothing is added to the snapshot. A world that never calls `Mark` carries one empty list and no other state. `Fork` is changed in cost only (see below).

## Fork

`Fork()` copies state directly, as before, and is now cheaper: it copies the stores up to their high-water marks, gives the copied constraint store only the capacity it needs (it grows by doubling when stepping needs more), leaves out the scratch that is sized on first use (the solver's batch arrays, the staging arrays a restore reads into, the restore's input buffer, empty joint records). A fork does not step or snapshot differently: with the same inputs it follows the source bit for bit, which the fork tests, the matrix's fork scenarios and the rollback probe check.

## Design record

- **Chosen: copy up to the high-water mark; sections as bytes.** The stores that hold nearly all of a world (bodies, shapes, constraints with the 104-float payloads, islands, events, coloring) are copied array for array into pooled storage; every other registered section is saved with its own `Write` and read back with its own `Read`. This covers every feature section by construction, because the snapshot already says what each of them is.
- **Not chosen: write barriers or per-store dirty bits.** The solver and the engines write the stores through spans in hot loops and through many mutators; a miss would corrupt a rollback silently and a barrier would slow every step. A content comparison to find changed pages reads two arrays to avoid writing one, which is no cheaper than copying them. Awake worlds change every contact payload every step, so dirty tracking would copy nearly everything anyway; the part that is unchanged (sleeping islands, static data) is small next to the payload. Cost follows live state instead of capacity, which is what the measurement showed to matter (the constraint store's spare capacity was most of its array bytes).
- **Not chosen: a raw copy of the engine sections.** The engines carry hundreds of fields, many of them caches; going through their own writer and reader keeps one definition of their state. The engine and broadphase sections are now the largest part of a rollback (see the measurements), and the place to look if a host needs more.
- A fork and a mark share the array helpers (`ForkArrays.Into`, `Prefix`), so the two paths cannot disagree about what a store's state is.

## Measurements (indicative)

Median milliseconds of 15 trials after 3 warm-up trials, one process per thread count pinned to eight cores of a 16-core Ryzen 9 7950X3D, release build, tiered compilation on, on a host that other jobs were loading (load average 20 to 34 during the run), so read ratios, not absolute values. A world per kind of measurement (a restore swaps a world's storage for its spare, which would otherwise disturb the marks); "after k steps" is the number of steps between the mark and the rollback, or between the snapshot and the restore. Source: `bench/Bar`, `rollback-timing`.

| scene (1 thread) | snapshot + restore, k = 1 / 7 / 60 | mark + rollback, k = 1 / 7 / 60 | fork, k = 1 / 7 / 60 |
|---|---|---|---|
| large-pile (2D) | 4.22 / 4.57 / 4.82 | 2.40 / 2.65 / 2.67 | 4.61 / 5.38 / 1.47 |
| large-pile-3d | 6.43 / 6.50 / 6.85 | 2.97 / 3.02 / 3.15 | 3.15 / 3.77 / 5.64 |
| ragdoll-pile-3d | 3.68 / 3.78 / 3.75 | 1.88 / 1.97 / 1.97 | 1.81 / 1.86 / 1.56 |
| sleep-wake-3d | 1.05 / 1.07 / 1.19 | 0.58 / 0.60 / 0.67 | 0.66 / 0.77 / 1.71 |

At eight threads (the copies are single-threaded; the world's thread pool is only started):

| scene (8 threads) | snapshot + restore, k = 1 / 7 / 60 | mark + rollback, k = 1 / 7 / 60 | fork, k = 1 / 7 / 60 |
|---|---|---|---|
| large-pile (2D) | 5.20 / 4.93 / 4.57 | 3.30 / 2.97 / 2.72 | 5.22 / 4.83 / 4.08 |
| large-pile-3d | 6.32 / 6.33 / 6.50 | 2.94 / 2.90 / 2.90 | 3.41 / 4.94 / 1.67 |
| ragdoll-pile-3d | 3.65 / 3.58 / 3.63 | 1.85 / 1.85 / 1.89 | 0.70 / 1.98 / 0.71 |
| sleep-wake-3d | 1.06 / 1.05 / 1.19 | 0.60 / 0.59 / 0.68 | 0.61 / 0.66 / 1.75 |

A fork is a wall-clock lottery in a process that allocates a world's worth of arrays each time (it moves with the collector), which is why its medians jump between 1.5 and 5.6 ms on the same scene; its allocation fell from 57 MB to 20 MB (3D large pile) and from 21 MB to 17 MB (2D).

What the first version taught: copying every array at full capacity, with the sections' bytes, cost 8.7 ms for mark + rollback on large-pile-3d against 7.3 for snapshot + restore, because the constraint store holds a 104-float payload per slot and most of its capacity was unused. Copying only the slots below the high-water mark (and the events up to their count) took it to 3.0 ms. Where the time goes now (large-pile-3d, one thread, mark / rollback): the engine section 0.8 / 1.5 ms and the broadphase 0.2 / 0.3 ms through their writers and readers, the constraint payload copy about 0.3 ms either way, everything else under 0.1 ms. The engine's pair table is what a raw copy would remove next.

Step time with no live mark (`recording-indicative --variant off`, alternating this tree and the tree before, same job, load 36 to 40): large-pile 1.84 / 1.86 / 1.84 ms against 2.99 / 2.00 / 1.82 before, large-pile-3d 9.67 / 9.37 / 9.17 against 14.16 / 9.59 / 9.40; the final world hashes are the same. A step pays one read of a static field.

## Tests

`WorldMarks2DTests` and `WorldMarks3DTests` (bytes, hash and future after 1, 7 and 60 steps with changed options, created bodies and a restore in between; lifetime rules; stale and foreign marks; recording; allocation of a ring). The hash-matrix tool's `--rollback-probe` marks, perturbs, steps and rolls back every world every scenario steps, over 1, 7 and 60 steps and a ring of three, and requires the marked state's bytes and hash and the same future as a fork that never rolled back; its manifest must equal the plain run's. `rollback/ring-2d` and `rollback/ring-3d` are matrix rows. Fuzz targets `rollback2d` and `rollback3d` interleave steps, marks, rollbacks, releases, body creation and destruction, velocity and gravity changes against a snapshot-and-restore oracle. `tests/Common/RollbackSmoke.cs` runs in the NativeAOT and WebAssembly smoke apps.
