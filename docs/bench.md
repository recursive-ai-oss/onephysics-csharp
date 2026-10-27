# Bench harness

Normative method for the reference runs. Everything lives under `bench/Bar` (runner, scene generator, metrics, the Box2D.NET driver), `bench/drivers/{box2d,rapier}` (native drivers) and `tools/refs`, `tools/bench` (scripts).

## One command

```
tools/bench/run.sh stability [--scenes a,b|all] [--engines box2d,rapier2d,box2dnet] [--out DIR]
tools/bench/run.sh timing    [--scenes ...] [--engines ...] [--threads 1,8,all] [--repeat N] [--out DIR]
```

Both build the runner, set up the reference drivers when missing (`tools/refs/setup.sh STATE [box2d|rapier|all]`: fetch by pinned URL and sha256, build unattended, install under `STATE`, default `~/onephysics`), regenerate and verify the scenes, and run scene × engine × thread count. `timing` takes the shared benchmark lock itself (atomically, released on exit) and records the lock as held; `stability` refuses to run while the lock exists and runs under `nice`. `BAR_JOB_ID` is recorded in `meta.txt`.

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

None of their sources or headers are read; drivers are written from the published documentation and compile against the released libraries.

The `onephysics` engine slot is reserved in the runner (`--engines onephysics`) and runs nothing yet.
