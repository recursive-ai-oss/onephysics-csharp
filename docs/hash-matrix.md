# The hash matrix

Bit-identical results on every platform is a claim that has to be checked continuously. The hash matrix runs deterministic
*scenarios* in many environments, reduces each to a SHA-256 per simulation step, and compares the results.

## Scenarios

A scenario is a named function that advances something (a kernel today, a world later) and writes the bytes that must
match into a `ScenarioContext`, calling `EndStep()` after each step:

```csharp
internal static class MyScenarios
{
    [ModuleInitializer]
    internal static void Init() => Scenarios.Register("area/thing", ctx =>
    {
        for (int step = 0; step < 100; step++)
        {
            // advance one fixed step ...
            ctx.Write(someInt); ctx.Write(someFloat); ctx.Write(someDouble); ctx.Write(someBytes);
            ctx.EndStep();
        }
    });
}
```

- Put the file under `tools/HashMatrix/Scenarios/<area>/`. The project compiles everything beneath it; no shared file is
  edited. Module initializers need no reflection, so registration works under NativeAOT, trimming and WebAssembly.
- Names are lower-case, `/`-separated and unique. The manifest orders them by ordinal comparison.
- `ctx.Threads` is the runner's `--threads` value. It is an option, never read from the machine. A scenario that runs in
  parallel uses it for the worker count; its output must not depend on it.
- Floating-point values are written by bit pattern, so a signed zero counts. Every NaN is written as one canonical quiet NaN,
  because the sign and payload of an arithmetic NaN come from the CPU.
- Integers are little-endian. A state too large to write every step can be folded into a few words by a multiply-xor hash
  (as the batching scenarios do); any hash that changes when any input bit changes will do.
- A scenario that never calls `EndStep()` is a single step.
- Source files outside `Scenarios/` that a scenario needs (for example the experiment's simulation code) are linked into
  `HashMatrix.csproj` with a `Compile Include`.

## The runner

`HashMatrix [--threads N] [--filter PREFIX] [--out FILE] [--steps] [--list]`

Output, one item per line:

```
<scenario name> <sha256 hex>              one per scenario, ordinal order
manifest <sha256 hex>                     SHA-256 of the lines above (UTF-8, '\n' line ends)
step <scenario name> <index> <hex16>      with --steps: first 8 bytes of every step digest
```

A scenario's hash is the SHA-256 of its step digests in order; a step digest is the SHA-256 of the bytes written during that
step. Two platforms agree on a scenario exactly when the scenario lines are identical; the `step` lines say where they part.

## The driver

`tools/hash-matrix.sh [-o OUTDIR] [-p PREFIX] [column ...]` runs the columns and compares every manifest in `OUTDIR` with
`jit.txt`, printing for each scenario that differs the first step at which the digests disagree.

| Column | What differs from `jit` |
|---|---|
| `jit` | .NET JIT, 1 thread (the reference) |
| `threads8`, `threads-all` | 8 threads; one thread per logical processor (the count is read by the script and passed explicitly) |
| `scalar` | `DOTNET_EnableHWIntrinsic=0`: vector operations fall back to software |
| `noavx512`, `noavx2` | `DOTNET_EnableAVX512F=0`, `DOTNET_EnableAVX2=0` |
| `tc0` | `DOTNET_TieredCompilation=0` and ReadyToRun off: optimised code from the first call |
| `aot` | NativeAOT |
| `wasm`, `wasm-aot` | browser-wasm under Node, interpreted and ahead-of-time compiled |

Other machines run the same script with `-p <prefix>` (for example `arm64-`, columns `jit` and `aot`) and the resulting
`<prefix><column>.txt` and `.steps` files are copied into the same directory; `tools/hash-matrix.sh -o OUTDIR compare`
then compares them with `jit.txt`. `HASHMATRIX_FILTER` restricts every column to a scenario prefix; a column may hold fewer
scenarios than the reference, and only the shared ones are compared (the report says how many).

Scenarios under `batching/` must also agree *within* one manifest across their last name component (the SIMD widths of one
family): the script reports a disagreement even when all columns agree with each other.

Output goes to `$ONEPHYSICS_ARTIFACTS` (build output) and the `-o` directory (manifests).

## Registered scenarios

`batching/*` (kernel widths), `core/golden` and `core/smoke` (the reference particle engine; final world hash of the golden scene 918517127736045633), `collision/hash2d`, `collision/hash3d`, `broadphase/hash` (one value each), and `world2d/pyramid-20`, `world2d/large-pile` (first 300 steps), `world2d/restore-and-shift` (a rotating-box scene with a snapshot, restore and origin shift): world hash after every step. `world2d/services`: layer-matrix and filter changes, a struct contact filter, sensors, hit, sleep and wake events and body callbacks, queries of every kind, debug geometry and a snapshot restore; per step the world hash, delivered events, callback log, query results and (every 20 steps) a digest of the debug geometry.
