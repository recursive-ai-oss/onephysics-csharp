# Bench harness

Normative method for the reference runs. Everything lives under `bench/Bar` (runner, scene generators, metrics, the in-process Box2D.NET, Bepu and Jitter2 drivers), `bench/drivers/{box2d,rapier,jolt,box3d,rapier3d}` (native drivers) and `tools/refs`, `tools/bench` (scripts).

## One command

```
tools/bench/run.sh stability [--scenes a,b|all] [--engines box2d,rapier2d,box2dnet,jolt,box3d,rapier3d,bepu,jitter2] [--out DIR]
tools/bench/run.sh timing    [--scenes ...] [--engines ...] [--threads 1,8,all] [--repeat N] [--out DIR]
```

(`profile` is described under "Per-stage profile".)

A scene whose `engines` param lists fewer engines than have a driver runs only those; `--force-engines` runs the engines named on the command line anyway (used for drivers written after the scene, see "Jolt drivers for the vehicle, character, ragdoll and buoyancy scenes").

Both build the runner, set up the reference drivers when missing (`tools/refs/setup.sh STATE [box2d|rapier|rapier3d|jolt|box3d|all|all3d]`: fetch by pinned URL and sha256, build unattended, install under `STATE`, default `~/onephysics`), regenerate and verify the scenes, and run scene × engine × thread count. A scene runs on the engines of its dimension (2D scenes: Box2D, Rapier2D, Box2D.NET; 3D scenes: Jolt, Box3D, Rapier3D, Bepu, Jitter2). `timing` takes the shared benchmark lock itself (atomically, released on exit) and records the lock as held; `stability` refuses to run while the lock exists and runs under `nice`. `BAR_JOB_ID` is recorded in `meta.txt`.

Output directory: `meta.txt` (machine, CPU model, logical cores, OS, runtime, job id, system load before, after each run and after), `timing.csv`, `metrics.csv` (stability mode), `summary.md`, and `raw/` (per-run result files, dumps, `time` output).

## Per-stage profile

```
tools/bench/run.sh profile        [--scenes large-pile,pyramid-100,tumbler,large-pile-3d,mesh-terrain-3d] [--engines box2d,rapier2d,rapier3d,onephysics] [--threads 1]
tools/bench/run.sh profile-locked [same options]
```

`bar run --mode profile` is a timing run that also reads each engine's own per-stage timers and writes `profile.md` (the side-by-side table, ours divided by each reference, and every native stage and counter) next to `timing.csv`. Medians over the measured steps (after the scene's warm-up). `profile` runs niced without the bench lock and records `timing_grade=NO` in `meta.txt`; `profile-locked` takes the lock like `timing` and is timing-grade. The default engines are Box2D v3 and Rapier2D/3D (whichever matches the scene's dimension) and ours.

**Where the stage times come from**

- **Box2D v3**: `b2World_GetProfile` after every step (the fields of `b2Profile` in the rendered API reference) and `b2World_GetCounters`. The driver prints the median of each field when `OP_PROFILE` is set; the normal build and its timing are untouched.
- **Rapier**: the documented performance counters (`counters` of the physics pipeline: step, collision detection, broad phase, narrow phase, solver, assembly, velocity resolution, velocity update, CCD and island construction timers). They only measure in a build with the crate's `profiler` feature, so profile mode uses a separate driver binary (`rapier-driver-profile`, `rapier3d-driver-profile`, built by `tools/refs/setup.sh STATE profile`); the timing binaries never carry it. The profiler's own overhead is part of those times.
- **Ours**: the library reports stage boundaries when it is built in the `Profile` configuration (Release plus the `STAGE_TIMERS` symbol; `dotnet build -c Profile`). The calls and the `StageProbe` type exist only under that symbol, so the package build contains none of it and steps are identical either way; the clock is read by the benchmark (`bench/Bar/StageProfile.cs`), not the library. The boundaries are on the stepping thread, so a stage's time is wall time including its parallel work. The solver's substeps run as one staged call, so integration, warm start, solve, relax and restitution are one stage.
- **Jolt**: not available. Its documentation says the profiler is part of the Release build type and the `JPH_PROFILE_ENABLED` option, not Distribution (the reference configuration), and describes no query API for per-stage times; the dump facility is only in a header the documentation does not describe. No profile variant is built.
- **Box3D**: the rendered 3D API reference lists `b3World_GetProfile` but no description of the profile fields, so Box3D has no profile driver yet.

**Bucket mapping** (`bench/Bar/ProfileReport.cs`; stages that contain other stages are shown natively and never summed; "other" is the engine's own step total minus the mapped stages)

| bucket | ours | Box2D v3 | Rapier |
|---|---|---|---|
| broadphase/pairs | boxes (proxy boxes, tree moves, start poses), pairs (broadphase pair search) | pairs, refit | broad phase |
| narrowphase | narrowphase (pair list and manifolds), apply (serial constraint add/remove) | collide | narrow phase |
| solver prepare | prepare (joint chunks and contact batches) | solverSetup, prepareConstraints | assembly |
| solve+integrate | solve (all substeps, one staged call) | integrateVelocities, warmStart, solveImpulses, integratePositions, relaxImpulses, restitution | velocity resolution, velocity update |
| finalize/store | finalize (per-body pass, bullets), store | storeImpulses, transforms, hitEvents, jointEvents, bullets, sensors, sensorHits | CCD, update |
| islands/sleep | sleep (joint breaks, sleep timers, island split and sleep) | splitIslands, sleepIslands | island construction |
| other | coloring, events | step minus the above | step minus the above |
| aggregates, not summed | | step, solve, constraints | step, collision detection, solver |

Observed hierarchy in Box2D 3.2.0 (large-pile, one thread): `constraints` is about the sum of the solver leaves and `solve` is about `constraints` plus `transforms`, which is why `transforms` is counted under finalize and `solve` and `constraints` are not summed. The buckets are for aiming optimisation work, not an exact accounting: engines draw their stage borders differently (for example, where body-pose updates and contact preparation fall), and the ratios are most reliable at bucket level.

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
| Jolt | one collision step per `Update` (four in scenes with vehicles, see "Jolt drivers for the vehicle, character, ragdoll and buoyancy scenes") | library defaults (10 velocity, 2 position iterations) | `JobSystemThreadPool` with N−1 workers plus the caller | point constraint in body-centre space; a group filter keeps joined pairs from colliding | static bodies in the non-moving layer, the rest in the moving layer; sleeping on; mesh and height-field shapes native; bullets use linear-cast motion quality; temp allocator 512 MB |
| Box3D | 4 sub-steps (the documented recommendation) | library defaults | `workerCount` = N with the built-in task system | spherical joint, frames measured from body origins, no collision between joined bodies | sleeping on; mesh edges identified (the documented way to avoid snagging); mesh winding and the height field's row order, winding and origin are confirmed by probing the created shape with rays and its bounds |
| Rapier3D | integration parameters at their defaults, dt from the scene | library defaults | `parallel` feature, a rayon pool of exactly N threads | spherical joint, no collision between joined bodies | CCD enabled on bullets; sleeping on |
| Bepu | `SolveDescription(8, 1)` (8 velocity iterations, 1 substep, the library default) | contact spring 30 Hz, damping 1, friction 0.6, recovery velocity 2 m/s (set by the driver, as the contact callback must) | `ThreadDispatcher(N)` | ball-socket with a 30 Hz spring; joined pairs are filtered out in the narrow-phase callbacks | static shapes are `Statics`; terrain is a triangle mesh (no height-field shape); mesh triangles are one-sided so the winding is reversed; sleeping on |
| Jitter2 | defaults (6 solver and 4 relaxation iterations, 1 substep, regular solve mode) | defaults | thread pool resized to N (caller included), `Step(dt, multiThread: N > 1)` | ball-socket from a world anchor; a broad-phase filter drops joined pairs | static bodies; terrain as one triangle shape per mesh triangle; hulls as point clouds; sleeping on |

Densities: Jolt and Box3D take the density per shape, Rapier per collider; Bepu takes a mass computed from the shape volume; Jitter2 shapes assume density 1, so the body mass is set from the volume afterwards.

Queries (`queries-3d`): Jolt, Box3D and Rapier3D run ray, sphere overlap and sphere cast through their documented world queries. Bepu and Jitter2 are not driven for queries: neither documents a whole-world sphere-overlap query with comparable counts, so their rows report step time and stability only.

Known gaps: Rapier3D reports no bodies asleep at creation, as in 2D. Engines that report velocity at the centre of mass differ from the origin velocity only for bodies whose centre of mass is offset from their origin (the offset capsules and hulls of `large-pile-3d`); that does not matter at the thresholds' precision.

None of their sources or headers are read; drivers are written from the published documentation and compile against the released libraries.

The `onephysics` engine runs this library's 2D world in a child process (`bar onephysics-driver`, built by `SceneBuilder2D`) with the same protocol as the others: `--engines onephysics`, threads 1/8/all, per-step dumps in stability mode. Joints are skipped (scenes `joint-grid` and `chain` build without their joints). `queries` runs its settling steps and then the ray (closest hit, `Raycast`), overlap (`OverlapCircleAll` into a 256-entry buffer, shapes counted) and sweep (`CircleCast`, closest hit) queries single-threaded; a ray starting inside a shape hits it at distance 0, as Rapier's solid rays do. `chain` shapes become one two-sided segment per link, as in the other drivers. Defaults: 4 substeps, one solve and one relax iteration per substep.

### Jolt drivers for the vehicle, character, ragdoll and buoyancy scenes

Written from the rendered class reference pages of Jolt 5.6.0 (no header, source or sample was read) and run in stability mode: `bar run --mode stability --engines jolt,onephysics --force-engines` (the option runs the listed engines on scenes whose `engines` param names fewer). Where the reference pages did not settle a point it was settled by a black-box experiment, noted below. Rows are 1 thread, Release, Ryzen 9 7950X3D (32 logical cores, shared, load about 6), step times indicative only (not a timing run); metrics and thresholds as in `docs/scenes.md`.

| scene | Jolt configuration | class reference pages used | Jolt | onephysics | Jolt vs the thresholds |
|---|---|---|---|---|---|
| `vehicles-3d` | `VehicleConstraint` with a `WheeledVehicleController`, the scene's `car` preset; four collision steps per update; wheels tested with `VehicleCollisionTesterCastSphere`; `mEnableSuspensionForcePoint` on at the neutral wheel centre; the chassis keeps its centre of mass | `VehicleConstraintSettings`, `WheelSettings`, `WheelSettingsWV`, `WheeledVehicleControllerSettings`, `VehicleCollisionTesterCastSphere` / `CastCylinder` / `Ray`, `OffsetCenterOfMassShape`, `Wheel` | rollovers 0, wheelThroughGround 0, maxPenetration 0, maxSpeed 18.8, suspensionJitter 0.0076 | 0, 0, 0, 17.9, 0.0020 | stable |
| `tracked-3d` | `VehicleConstraint` with a `TrackedVehicleController`: the scene's `tank` wheels (`WheelSettingsTV`), two tracks (the x < 0 wheels left, the first wheel of each driven), library engine, transmission and track settings; forward input = the larger track command, track ratios = the split; four collision steps; sphere-cast wheel test | `TrackedVehicleControllerSettings`, `TrackedVehicleController`, `VehicleTrackSettings`, `WheelSettingsTV` | rollovers 0, maxPenetration 0, maxSpeed 5.9, suspensionJitter 0 | 0, 0, 5.6, 0.0002 | stable |
| `motorcycle-3d` | `MotorcycleController` (lean controller on, 40 degree limit), the scene's `bike` preset, rear-wheel differential (`mLeftWheel = -1`); four collision steps; sphere-cast wheel test; centre of mass 0.2 m down | `MotorcycleControllerSettings`, `MotorcycleController`, `WheeledVehicleControllerSettings`, `OffsetCenterOfMassShape` | fallen 0, leanError 0.023, maxSpeed 8.1, suspensionJitter 0 | 0, 0.0059, 8.5, 0 | stable |
| `characters-3d` | `CharacterVirtual` driven by `ExtendedUpdate` as its page describes (ground velocity plus walk plus jump plus delta time x gravity on the ground; step up 0.3 m, stick to floor 0.3 m); capsule lifted so its bottom is the character's origin; slope limit 45 degrees; supporting volume at the capsule's lower sphere | `CharacterVirtual`, `CharacterVirtualSettings`, `CharacterBaseSettings`, `CharacterVirtual::ExtendedUpdateSettings` | stairSuccess 0.33, charJitter 3.17, slopeViolations 0, maxPenetration 0.020, maxSpeed 17.9 | 1, 0, 0, 0.015, 20.2 | **unstable** (stairSuccess below 1, charJitter above 0.01) |
| `ragdoll-powered-3d` | one `SwingTwistConstraint` per joint (limits from the scene), the pose drive as a position motor on the swing and twist motors with the scene's stiffness and damping used directly (spring mode `StiffnessAndDamping`, so the frequency and damping ratio need no conversion: k = I w^2 and c = 2 zeta sqrt(k I) would give the same spring for an effective inertia I) and the scene's torque cap; target set every step with `SetTargetOrientationCS`; driven bodies kept awake; collision steps = the scene's 8 | `SwingTwistConstraintSettings` (with its picture), `SwingTwistConstraint`, `MotorSettings`, `SpringSettings`, the "Constraint Motors" section of the architecture page | poseErrorP95 0.058, fallen 0, jointError 0, maxSpeed 6.8 | 0.059, 0, 0.010, 6.9 | stable |
| `buoyancy-3d` | `BodyInterface::ApplyBuoyancyImpulse` once per step per awake body: buoyancy factor = fluid density over body density, linear drag 0.5, angular drag 0.01, the scene's current as fluid velocity | `BodyInterface` | equilibriumDepthError 0.031, sinkersAfloat 0, restKE 0.049, maxSpeed 6.3 | 0.0033, 0, 0.0255, 5.6 | **unstable** (equilibriumDepthError above 0.02, restKE above 0.04; both narrowly) |

Findings that decided the configurations:

- **Collision steps.** At the documented default of one collision step the car driver rolled 68 of 100 cars (two steps: 1, four steps: 0; suspensionJitter 0.034 and 0.008), the bike driver dropped all 20 motorcycles, and the tank driver was unaffected (stable at one and at four). A single car on flat ground, on a flat height field and on a flat mesh drives straight at one step; on the scene's gently bumped terrain its wheels lose and regain contact. Four steps is therefore the vehicle drivers' setting; this is an assumption of the row, not the library's default configuration, so the timing rows for these scenes must say that Jolt runs four collision steps.
- **Wheel test shape.** With four steps the default cylinder cast gave suspensionJitter 0.029 on the cars (unstable), the ray 0.007 and the sphere cast 0.008. `mMaxPitchRollAngle` was not used (it would hide roll-overs rather than prevent them); a lowered centre of mass was used only for the motorcycle (leanError 0.27 rad without, 0.023 with); for the cars at four steps it raised suspensionJitter to 0.014.
- **Steering sign and tracks.** The scene's positive steering turns towards +x, which is the vehicle's left with +y up and +z forward, while Jolt's steering input is positive to the right; the drivers negate it. A tracked vehicle given zero track ratios while braking produced non-finite wheel transforms, so the ratios stay at one without throttle.
- **Characters.** The page asks for a shape whose bottom is at the origin; the first driver centred the capsule and could not climb stairs. The vertical speed on the ground is delta time x gravity as the `ExtendedUpdate` page gives it (a constant 1 m/s downward stopped stair climbing altogether). Jolt then climbs the 0.20 m stairs completely but the 0.25 and 0.29 m risers slowly: the capsule rides up each riser edge as a slope below the 45 degree limit instead of stepping, about half a second a riser, so 24 of 36 climbers had not reached the top when the script stops. A larger minimum forward step (0.2 m) raised that to 16 of 36; the defaults are kept. The charJitter of 3.2 m/s is two riders of the sliding platforms that end the run standing on the ground in front of the platform, at the platform's speed.
- **Powered ragdolls.** The swing-twist constraint space was determined by experiment (drive a known rotation, read `GetRotationInConstraintSpace`): x is the twist axis, y the normal axis (plane axis x twist axis) and z the plane axis, so the scene's joint-frame drive axis (x, y, z) is passed as (x, -z, y); with the picture's reading the arm and elbow targets were off by up to 1.3 rad.

Which of these bind the library as stable leaders: `vehicles-3d`, `tracked-3d`, `motorcycle-3d` and `ragdoll-powered-3d` (Jolt is the only reference with a driver, and it is stable); `characters-3d` and `buoyancy-3d` have no stable reference under the proposed thresholds.

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
| ragdoll-powered-3d | ≤ 10 | | | poseErrorP95 ≤ 0.10, fallen = 0, jointError ≤ 0.05 |

Evidence (maxPenetration / finalPenetration / other, x64 run; the stable engines set the threshold):

- **pyramid-3d-10:** all five stable, worst 0.0018 / 0.0018, topDrop ≤ 0.0036.
- **pyramid-3d-30:** Box3D, Jolt and Rapier3D stable (≤ 0.0063, topLateral ≤ 0.065, maxSpeed ≤ 0.13). Bepu (0.45, topLateral 0.89) and Jitter2 on x64 (0.50; 0.008 on the Mac, so its result varies between machines) are not, at their defaults.
- **large-pile-3d:** all stable; worst 0.226 / 0.128, maxSpeed 10, jitter ≤ 0.036.
- **joint-chains-3d** (500:1 end mass): Box3D 0.24 m and Rapier3D 0.61 m are stable; Jolt 19.5 m, Bepu 33 m and Jitter2 16 m stretch at their default iteration counts. Checked that the Jolt driver is correct: with the end mass lowered to 10:1 and 1:1 the same driver gives 0.09 m and 0.13 m (Box3D 0.05 / 0.03, Rapier3D 0.16 / 0.07, Bepu 0.36 / 0.19, Jitter2 0.10 / 0.13). It is the solver at 10 velocity iterations, not the constraint set-up.
- **joint-grid-3d:** Bepu, Box3D, Jolt and Rapier3D stable (jointError ≤ 0.03, maxSpeed ≤ 1.5); Jitter2 at its default single substep blows up (1323 bodies escape), and is stable with 4 or 8 substeps (jointError 0.09 / 0.06).
- **ragdoll-pile-3d:** all stable; worst jointError 0.16, 0.27 / 0.08, maxSpeed 17.4, restKE 0.24.
- **ragdoll-powered-3d:** onephysics only so far (indicative, 1 and 8 threads, shared x64 machine at load ~6): poseErrorP95 0.058, fallen 0, jointError 0.0097, maxSpeed 6.86; 11.3 ms per step at 1 thread, 8.75 ms at 8. No reference row yet; thresholds are our result plus margin.
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
