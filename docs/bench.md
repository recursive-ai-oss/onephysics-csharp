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

- **Step time**: wall time of the engine's step call only, one sample per step, steady clock, in-process, with tiered compilation off (`Bar.csproj`) so that every managed engine, ours and the in-process references, runs optimised code from its first call; with it on, small scenes finish before the hot methods are promoted and measure the JIT instead. The first `warmup` steps of the scene are dropped; median, mean, p90, p99, min and max are reported over the rest. With `--repeat N` the whole process is run N times and the run with the lowest median is kept.
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

The `onephysics` engine runs this library's 2D world in a child process (`bar onephysics-driver`, built by `SceneBuilder2D`) with the same protocol as the others: `--engines onephysics`, threads 1/8/all, per-step dumps in stability mode. Joints are skipped (scenes `joint-grid` and `chain` build without their joints). `queries` runs its settling steps and then the ray (closest hit, `Raycast`), overlap (`OverlapCircleAll` into a 256-entry buffer, shapes counted) and sweep (`CircleCast`, closest hit) queries single-threaded; a ray starting inside a shape hits it at distance 0, as Rapier's solid rays do. `chain` shapes become one two-sided segment per link, as in the other drivers. Defaults: 4 substeps, one solve and one relax iteration per substep.

## 3D stability thresholds (proposed)

Measured at one thread, 2026-10-27, on an x64 machine (Ryzen 9 7950X3D, 32 logical cores, shared, load 17–29) and an Apple M2; the two agree except where noted. Each threshold is the worst value among the engines that are stable on the scene, plus margin. Always: `nan` = 0; `escapes` = `tunnelled` = 0 unless stated.

| scene | maxSpeed | maxPenetration | finalPenetration | other |
|---|---|---|---|---|
| pyramid-3d-10 | ≤ 0.1 | ≤ 0.01 | ≤ 0.01 | topDrop ≤ 0.06, topLateral ≤ 0.05, restKE ≤ 0.01 |
| pyramid-3d-30 | ≤ 1 | ≤ 0.05 | ≤ 0.04 | topDrop ≤ 0.1, topLateral ≤ 0.25, restKE ≤ 0.01, jitter ≤ 0.05 |
| large-pile-3d | ≤ 15 | ≤ 0.5 | ≤ 0.25 | restKE ≤ 0.005, jitter ≤ 0.25 |
| joint-chains-3d | ≤ 45 | | | jointError ≤ 1.0 |
| joint-grid-3d | ≤ 5 | | | jointError ≤ 0.1 |
| ragdoll-pile-3d | ≤ 30 | ≤ 0.4 | ≤ 0.2 | jointError ≤ 0.3, restKE ≤ 0.5 |
| mesh-terrain-3d, heightfield-terrain-3d | ≤ 15 | ≤ 0.15 | ≤ 0.1 | tunnelled = 0 |
| sleep-wake-3d | ≤ 45 | ≤ 1.5 | ≤ 1.0 | escapes ≤ 10 (the ball scatters bodies off the slab), asleepBeforeWake = 1440 |
| queries-3d | ≤ 25 | | | throughput row only |

Evidence (maxPenetration / finalPenetration / other, x64 run; the stable engines set the threshold):

- **pyramid-3d-10:** all five stable, worst 0.0018 / 0.0018, topDrop ≤ 0.0036.
- **pyramid-3d-30:** Box3D, Jolt and Rapier3D stable (≤ 0.0063, topLateral ≤ 0.065, maxSpeed ≤ 0.13). Bepu (0.45, topLateral 0.89) and Jitter2 on x64 (0.50; 0.008 on the Mac, so its result varies between machines) are not, at their defaults.
- **large-pile-3d:** all stable; worst 0.226 / 0.128, maxSpeed 10, jitter ≤ 0.036.
- **joint-chains-3d** (500:1 end mass): Box3D 0.24 m and Rapier3D 0.61 m are stable; Jolt 19.5 m, Bepu 33 m and Jitter2 16 m stretch at their default iteration counts. Checked that the Jolt driver is correct: with the end mass lowered to 10:1 and 1:1 the same driver gives 0.09 m and 0.13 m (Box3D 0.05 / 0.03, Rapier3D 0.16 / 0.07, Bepu 0.36 / 0.19, Jitter2 0.10 / 0.13). It is the solver at 10 velocity iterations, not the constraint set-up.
- **joint-grid-3d:** Bepu, Box3D, Jolt and Rapier3D stable (jointError ≤ 0.03, maxSpeed ≤ 1.5); Jitter2 at its default single substep blows up (1323 bodies escape), and is stable with 4 or 8 substeps (jointError 0.09 / 0.06).
- **ragdoll-pile-3d:** all stable; worst jointError 0.16, 0.27 / 0.08, maxSpeed 17.4, restKE 0.24.
- **terrain (mesh and height field):** Bepu, Jolt, Rapier3D and Box3D (mesh) stable, worst 0.057 / 0.056, maxSpeed 10. Jitter2 lets 3 bodies through (0.32). Box3D's height field reaches 0.41 / 0.39 where its mesh gives 0.034 / 0.021, although probes of the created height field match the scene's heights and cell split to 0.02 m: reported as a Box3D height-field finding (alpha release), not a threshold input.
- **sleep-wake-3d:** all engines but Rapier3D create the bodies asleep (Rapier3D reports none asleep, as in 2D). The ball crushes the stacks (0.33–1.28 deep) and scatters 0–7 bodies off the slab, so the thresholds are loose; enlarging the metric's container does not remove the scatter.
- **queries-3d** (Apple M2, 1 thread, queries per second ray / overlap / cast; hits ray / overlap / cast): Jolt 2.2 M / 0.19 M / 0.78 M, 2941 / 34028 / 2256; Box3D 1.7 M / 0.25 M / 1.3 M, 2939 / 35159 / 2265; Rapier3D 3.4 M / 0.47 M / 0.42 M, 2941 / 34982 / 2263. Hit counts agree within 3.3%.

## Stability under the robust metrics (x64, one thread; robust / raw)

Same run as the proposed thresholds in `docs/scenes.md` (metric definitions there). Each cell is the robust value, then the raw (old-definition) value. Engines are listed where they ran the scene; `onephysics` is this library. These values supersede the old-metric evidence above where they differ: the old single-frame definitions were `jitterRaw`, `restKERaw`, `finalPenetrationRaw`, `escapesRaw`.

**jitter**

| scene | box2d | box2dnet | rapier2d | jolt | box3d | rapier3d | bepu | jitter2 | onephysics |
|---|---|---|---|---|---|---|---|---|---|
| pyramid-20 | 0 / 0 | 0 / 0 | 0 / 0 |  |  |  |  |  | 0 / 0 |
| pyramid-100 | 0.115 / 0.01 | 0 / 0 | 0.00965 / 0 |  |  |  |  |  | 0 / 0 |
| large-pile | 0.0449 / 0.0933 | 0.0137 / 1.04 | 0.078 / 0.0265 |  |  |  |  |  | 0.041 / 0.122 |
| pyramid-3d-10 |  |  |  | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| pyramid-3d-30 |  |  |  | 0 / 0 | 0 / 0 | 0 / 0 | 0.0457 / 0.131 | 0 / 0 | 0 / 0 |
| large-pile-3d |  |  |  | 0.00313 / 0.0158 | 8.77e-06 / 0 | 0.000332 / 0.0291 | 0.00664 / 0.0361 | 0.000343 / 0.0214 | 0.000121 / 0.0362 |

**restKE**

| scene | box2d | box2dnet | rapier2d | jolt | box3d | rapier3d | bepu | jitter2 | onephysics |
|---|---|---|---|---|---|---|---|---|---|
| pyramid-20 | 0 / 0 | 0 / 0 | 0 / 0 |  |  |  |  |  | 0 / 0 |
| pyramid-100 | 0.00224 / 0.000728 | 0 / 0 | 0 / 0 |  |  |  |  |  | 0 / 0 |
| large-pile | 0.00432 / 0.00014 | 0.000543 / 0.000131 | 0.00834 / 0.000486 |  |  |  |  |  | 0.0011 / 3.38e-06 |
| pyramid-3d-10 |  |  |  | 0 / 0 | 0 / 0 | 0 / 0 | 2.41e-07 / 8.86e-08 | 2.12e-12 / 7.65e-13 | 0 / 0 |
| pyramid-3d-30 |  |  |  | 0 / 0 | 0 / 0 | 0 / 0 | 0.0294 / 0.0254 | 9.14e-06 / 3.02e-06 | 0 / 0 |
| large-pile-3d |  |  |  | 9.32e-08 / 1.07e-06 | 0 / 0 | 5.95e-07 / 1.77e-05 | 5.87e-06 / 8.12e-06 | 2.97e-08 / 4.51e-06 | 1.71e-08 / 1.9e-05 |
| ragdoll-pile-3d |  |  |  | 0.0315 / 0.0571 | 0.00897 / 0.162 | 0.0131 / 0.131 | 0.0169 / 0.245 | 0.00486 / 0.00566 | 0.126 / 0.821 |

**finalPenetration**

| scene | box2d | box2dnet | rapier2d | jolt | box3d | rapier3d | bepu | jitter2 | onephysics |
|---|---|---|---|---|---|---|---|---|---|
| pyramid-20 | 0.00304 / 0.0036 | 0.00301 / 0.00347 | 0.00304 / 0.0033 |  |  |  |  |  | 0.00206 / 0.00235 |
| pyramid-100 | 0.0152 / 0.018 | 0.0156 / 0.0209 | 0.0151 / 0.0174 |  |  |  |  |  | 0.0102 / 0.0123 |
| large-pile | 0.0489 / 0.148 | 0.0502 / 0.179 | 0.0499 / 0.124 |  |  |  |  |  | 0.0353 / 0.11 |
| tumbler | 0.0316 / 0.0891 | 0.0342 / 0.0715 | 0.0353 / 0.0932 |  |  |  |  |  | 0.0253 / 0.0661 |
| sleep-wake | 0.0355 / 0.104 | 0.0481 / 0.0692 | 0.0351 / 0.0777 |  |  |  |  |  | 0.0362 / 0.129 |
| mixed-terrain | 0.00503 / 0.0139 | 0.00481 / 0.0149 | 0.00504 / 0.0194 |  |  |  |  |  | 0.00356 / 0.0107 |
| pyramid-3d-10 |  |  |  | 0.000546 / 0.000978 | 0.000422 / 0.000543 | 0.000405 / 0.000464 | 0.00081 / 0.00167 | 0.000935 / 0.00174 | 0.000393 / 0.000467 |
| pyramid-3d-30 |  |  |  | 0.00267 / 0.00626 | 0.00132 / 0.00189 | 0.00127 / 0.00172 | 0.00215 / 0.454 | 0.0034 / 0.495 | 0.00125 / 0.00166 |
| large-pile-3d |  |  |  | 0.0305 / 0.0687 | 0.00919 / 0.128 | 0.00518 / 0.0833 | 0.00735 / 0.0418 | 0.00998 / 0.0186 | 0.00729 / 0.0314 |
| ragdoll-pile-3d |  |  |  | 0.0202 / 0.0275 | 0.00394 / 0.0821 | 0.00172 / 0.0232 | 0.00581 / 0.0351 | 0.01 / 0.0112 | 0.00378 / 0.0169 |
| mesh-terrain-3d |  |  |  | 0.02 / 0.0272 | 0.00346 / 0.0214 | 0.00115 / 0.0061 | 0.000876 / 0.0276 | 0.01 / 0.323 | 0.000367 / 0.00492 |
| heightfield-terrain-3d |  |  |  | 0.0209 / 0.0556 | 0.195 / 0.385 | 0.00115 / 0.0061 | 0.000876 / 0.0276 | 0.01 / 0.324 | 0.00893 / 0.039 |
| sleep-wake-3d |  |  |  | 0.119 / 0.671 | 0.033 / 0.368 | 0.0413 / 0.297 | 0.067 / 0.909 | 0.193 / 0.472 | 0.103 / 0.357 |

**escapes**

| scene | box2d | box2dnet | rapier2d | jolt | box3d | rapier3d | bepu | jitter2 | onephysics |
|---|---|---|---|---|---|---|---|---|---|
| pyramid-20 | 0 / 0 | 0 / 0 | 0 / 0 |  |  |  |  |  | 0 / 0 |
| pyramid-100 | 0 / 0 | 0 / 0 | 0 / 0 |  |  |  |  |  | 0 / 0 |
| large-pile | 0 / 0 | 0 / 0 | 0 / 0 |  |  |  |  |  | 0 / 0 |
| tumbler | 0 / 0 | 0 / 0 | 0 / 0 |  |  |  |  |  | 0 / 0 |
| sleep-wake | 0 / 0 | 0 / 0 | 0 / 0 |  |  |  |  |  | 0 / 0 |
| mixed-terrain | 0 / 0 | 0 / 0 | 0 / 0 |  |  |  |  |  | 0 / 0 |
| pyramid-3d-10 |  |  |  | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| pyramid-3d-30 |  |  |  | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| large-pile-3d |  |  |  | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| ragdoll-pile-3d |  |  |  | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| mesh-terrain-3d |  |  |  | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 3 / 3 | 0 / 0 |
| heightfield-terrain-3d |  |  |  | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 3 / 3 | 0 / 0 |
| sleep-wake-3d |  |  |  | 0 / 0 | 0 / 4 | 0 / 5 | 0 / 7 | 0 / 0 | 0 / 0 |

`ragdoll-pile-3d` `restKE` per engine is in the `restKE` table above: bepu 0.0169 / 0.245, box3d 0.00897 / 0.162, jitter2 0.00486 / 0.00566, jolt 0.0315 / 0.0571, rapier3d 0.0131 / 0.131, onephysics 0.126 / 0.821.
