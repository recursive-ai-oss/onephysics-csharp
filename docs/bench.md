# Bench harness

Normative method for the reference runs. Everything lives under `bench/Bar` (runner, scene generators, metrics, the in-process Box2D.NET, Bepu and Jitter2 drivers), `bench/drivers/{box2d,rapier,jolt,box3d,rapier3d}` (native drivers) and `tools/refs`, `tools/bench` (scripts).

## One command

```
tools/bench/run.sh stability [--scenes a,b|all] [--engines box2d,rapier2d,box2dnet,jolt,box3d,rapier3d,bepu,jitter2] [--out DIR]
tools/bench/run.sh timing    [--scenes ...] [--engines ...] [--threads 1,8,all] [--repeat N] [--out DIR]
```

Both build the runner, set up the reference drivers when missing (`tools/refs/setup.sh STATE [box2d|rapier|rapier3d|jolt|box3d|all|all3d]`: fetch by pinned URL and sha256, build unattended, install under `STATE`, default `~/onephysics`), regenerate and verify the scenes, and run scene × engine × thread count. A scene runs on the engines of its dimension (2D scenes: Box2D, Rapier2D, Box2D.NET; 3D scenes: Jolt, Box3D, Rapier3D, Bepu, Jitter2). `timing` takes the shared benchmark lock itself (atomically, released on exit) and records the lock as held; `stability` refuses to run while the lock exists and runs under `nice`. `BAR_JOB_ID` is recorded in `meta.txt`.

Output directory: `meta.txt` (machine, CPU model, logical cores, OS, runtime, job id, system load before, after each run and after), `timing.csv`, `metrics.csv` (stability mode), `summary.md`, and `raw/` (per-run result files, dumps, `time` output).

## Method

- **Step time**: wall time of the engine's step call only, one sample per step, steady clock, in-process. The first `warmup` steps of the scene are dropped; median, mean, p90, p99, min and max are reported over the rest. With `--repeat N` the whole process is run N times and the run with the lowest median is kept.
- **Threads**: 1, 8 and the machine's logical core count (`all`). Box2D: `workerCount` set to the thread count and the library's own task system used. Rapier2D: the `parallel` feature (the documented way to multithread the whole step) and the step runs inside a rayon pool of exactly that many threads. Box2D.NET has no documented multithreading, so it runs 1 thread only.
- **Memory**: peak resident set size of the driver process from `/usr/bin/time -v` (Linux) or `-l` (macOS), whole run including scene loading.
- **Query throughput**: after the scene's `settle` steps, each query kind runs over all its queries `query_reps` times; the first repetition is untimed; queries per second = queries ÷ time for that kind. Hit counts are recorded so engines can be compared for agreement.
- **Stability**: the driver dumps state every `dump_every` steps (not timed); `bar metrics` computes the metrics from the dump (docs/scenes.md).
- **Defaults**: each reference runs with its default configuration: Box2D v3 world definition defaults with 4 substeps (the documented recommendation) and sleeping on; Rapier default integration parameters and the dt from the scene; Box2D.NET defaults with 4 substeps. Gravity and dt come from the scene. Bullets: Box2D `isBullet`, Rapier `ccd_enabled`.
- **Machines**: x64 timing runs on the shared worker under the benchmark lock; arm64 on the Mac mini with nothing else running. A run that overlapped another project's heavy job is not publishable.

## References

| engine | version | licence | how it runs |
|---|---|---|---|
| Box2D v3 | 3.2.0 (tag archive, sha256 in `tools/refs/pins.env`) | MIT | CMake static build, C driver written from the public documentation |
| Rapier2D | 0.36.1 (`=` pin in `bench/drivers/rapier/Cargo.toml`, `Cargo.lock` committed) | Apache-2.0 | Rust driver, `parallel` feature |
| Box2D.NET | 3.1.654 (NuGet, `Bar.csproj`) | MIT | in-process driver `bar box2dnet-driver` |
| Jolt | 5.6.0 (tag archive, sha256 in `tools/refs/pins.env`) | MIT | CMake static build (`Distribution`, every library option at its default: cross-platform determinism off, single precision, no double-precision positions), C++ driver |
| Box3D | 0.1.0 (tag archive, sha256 in `pins.env`; alpha) | MIT | CMake static build, C driver |
| Rapier3D | 0.36.1 (`=` pin in `bench/drivers/rapier3d/Cargo.toml`, `Cargo.lock` committed) | Apache-2.0 | Rust driver, `parallel` feature |
| BepuPhysics | 2.5.0-beta.33 (NuGet, `Bar.csproj`) | Apache-2.0 | in-process driver `bar bepu-driver` |
| Jitter2 | 2.13.0 (NuGet, `Bar.csproj`) | MIT | in-process driver `bar jitter2-driver` |

### 3D reference configurations

Each engine runs its documented defaults; a driver only supplies what a scene needs (shapes, joints, sleep state, thread count). Step time is the engine's step call only.

| engine | step | solver | threads | joints | other |
|---|---|---|---|---|---|
| Jolt | one collision step per `Update` | library defaults (10 velocity, 2 position iterations) | `JobSystemThreadPool` with N−1 workers plus the caller | point constraint in body-centre space; a group filter keeps joined pairs from colliding | static bodies in the non-moving layer, the rest in the moving layer; sleeping on; mesh and height-field shapes native; bullets use linear-cast motion quality; temp allocator 512 MB |
| Box3D | 4 sub-steps (the documented recommendation) | library defaults | `workerCount` = N with the built-in task system | spherical joint, frames measured from body origins, no collision between joined bodies | sleeping on; mesh edges identified (the documented way to avoid snagging); mesh winding and the height field's row order, winding and origin are confirmed by probing the created shape with rays and its bounds |
| Rapier3D | integration parameters at their defaults, dt from the scene | library defaults | `parallel` feature, a rayon pool of exactly N threads | spherical joint, no collision between joined bodies | CCD enabled on bullets; sleeping on |
| Bepu | `SolveDescription(8, 1)` (8 velocity iterations, 1 substep, the library default) | contact spring 30 Hz, damping 1, friction 0.6, recovery velocity 2 m/s (set by the driver, as the contact callback must) | `ThreadDispatcher(N)` | ball-socket with a 30 Hz spring; joined pairs are filtered out in the narrow-phase callbacks | static shapes are `Statics`; terrain is a triangle mesh (no height-field shape); mesh triangles are one-sided so the winding is reversed; sleeping on |
| Jitter2 | defaults (6 solver and 4 relaxation iterations, 1 substep, regular solve mode) | defaults | thread pool resized to N (caller included), `Step(dt, multiThread: N > 1)` | ball-socket from a world anchor; a broad-phase filter drops joined pairs | static bodies; terrain as one triangle shape per mesh triangle; hulls as point clouds; sleeping on |

Densities: Jolt and Box3D take the density per shape, Rapier per collider; Bepu takes a mass computed from the shape volume; Jitter2 shapes assume density 1, so the body mass is set from the volume afterwards.

Queries (`queries-3d`): Jolt, Box3D and Rapier3D run ray, sphere overlap and sphere cast through their documented world queries. Bepu and Jitter2 are not driven for queries: neither documents a whole-world sphere-overlap query with comparable counts, so their rows report step time and stability only.

Known gaps: Rapier3D reports no bodies asleep at creation, as in 2D. Engines that report velocity at the centre of mass differ from the origin velocity only for bodies whose centre of mass is offset from their origin (the offset capsules and hulls of `large-pile-3d`); that does not matter at the thresholds' precision.

None of their sources or headers are read; drivers are written from the published documentation and compile against the released libraries.

The `onephysics` engine slot is reserved in the runner (`--engines onephysics`) and runs nothing yet.
