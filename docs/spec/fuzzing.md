# Fuzzing

Every input that crosses the public boundary has a fuzz target in `fuzz/Fuzz`: hull and mesh construction, height fields and compounds, the broadphase, 2D and 3D manifolds, both worlds' definitions, services, joints, characters, vehicles, ragdolls and fluids, and every snapshot reader (`tools/fuzz.sh list` prints them). A target takes a byte array and decodes it into an input. Its allowed outcomes are success or the typed error the API documents; any other exception, a hang, or a result that breaks an invariant (a non-finite number, a normal that is not a unit vector, a snapshot that does not re-serialise to the bytes it was restored from, a rejected snapshot that changed the world) is a finding.

## Modes

- **Replay:** `Fuzz <target> <file>...` runs a target over files.
- **Seeded random:** `Fuzz <target> --random <seed> <count> [bytes]` runs it over a pseudo-random byte stream. This is what `tools/check.sh` uses (smoke only).
- **Coverage-guided:** `Fuzz <target> --guided <seconds> --corpus <dir> --crashes <dir>`. The library assembly is instrumented by SharpFuzz (a tool and build-time dependency of the fuzz project only; the package itself still depends only on onemath). Instrumented branches fill an edge map; the driver in `fuzz/Fuzz/Guided.cs` keeps inputs that reach a new (edge, hit-count class) and mutates them (bit and byte edits, arithmetic, interesting values, insertion, deletion, duplication, splicing between corpus entries). It runs in-process, so no native fuzzer is needed. Each distinct finding (exception type plus its top stack frames) is saved once as `crash-<id>` with a `.txt` stack trace; the input in flight is also kept in `.current`, so a crash that kills the process leaves its input. A watchdog saves the input of any run longer than 30 s and exits. `--guided` prints `edges` (map entries touched) and `features` (entry and hit-count class pairs).
- **Minimise:** `Fuzz <target> --minimize <input> <output>` removes chunks and zeroes bytes while the same finding persists.

## Running

```
tools/fuzz.sh build                    # Checked build, library instrumented, into $FUZZ_HOME/instr
tools/fuzz.sh seeds                    # copy testdata/fuzz-seeds into the corpora
tools/fuzz.sh run mesh 600             # one target for 10 minutes
tools/fuzz.sh run all 1500 4           # every target, 25 minutes each, four at a time (niced)
tools/fuzz.sh minimize world3d-snapshot $FUZZ_HOME/crashes/world3d-snapshot/crash-0123456789
tools/fuzz.sh report                   # executions, coverage, wall time and findings per target over all chunks
```

State (instrumented build, corpora, crashes, logs, `stats.tsv`) lives under `$FUZZ_HOME`, default `~/onephysics/fuzz`, never in the checkout. Corpora persist between chunks, so coverage keeps accumulating; the heap is capped at 2 GB per process so runaway allocation shows up as a finding.

## Seeds and regression tests

`testdata/fuzz-seeds/<target>/` holds small seed inputs written by our own generator (`dotnet run --project fuzz/Fuzz -- --seeds testdata/fuzz-seeds`): seeded random bytes of growing length, a valid snapshot for the raw snapshot reader, and a cube for the hull builder.

A finding becomes a minimised file at `testdata/fuzz/<target>/<name>.bin` (that is what `minimize` writes), plus a fix. `FuzzRegressionTests` replays every such file through its target in the normal test run.

`testdata/fuzz-open/<target>/` holds minimised inputs of findings that are not fixed yet; nothing replays them (it is empty at the moment). When a fix lands the file moves to `testdata/fuzz/`.
