# Building

Requires the .NET SDK pinned in `global.json` (10.0.400 or a later feature band). Browser-WASM also needs the `wasm-tools` workload and Node.

## Restore: getting onemath

`src/OnePhysics` depends on `RecursiveAI.OneMath`. Until it is on nuget.org, copy the package into the ignored `.localfeed/` directory (restore picks the directory up when it exists):

```
ONEMATH_FEED=<directory holding RecursiveAI.OneMath.0.1.0-preview.2.nupkg> tools/localfeed.sh
```

That is the only setup step. `.localfeed/` is not tracked, so when building on a machine that only receives the git working tree, ship that directory along with it.

## Build, test, smoke

```
tools/check.sh              # scans, builds the solution, runs tests and the fuzz harness (Release)
tools/check.sh Checked      # the same with bounds-checked internal accessors
tools/smoke-aot.sh          # NativeAOT publish (zero trim/AOT warnings) and run
tools/smoke-wasm.sh         # browser-wasm publish and run under Node
tools/hash-matrix.sh        # runs every hash-matrix scenario in every column and compares the manifests (see docs/spec/hash-matrix.md)
```

Build output goes to `$ONEPHYSICS_ARTIFACTS` (default `~/.cache/onephysics-artifacts`), never into the checkout.

`tools/check-allow.txt` is the allow-list for the source scans in `tools/check.sh`; it starts empty and every entry needs a reason.

## Rules the scans enforce

Only IEEE-exact `+ - * /` and square root decide a result; transcendental functions come from onemath. No fused multiply-add, estimate instructions, clocks, `Vector<T>`, hardware-width queries or unsafe code in `src/` without a recorded exception. The package has one dependency: onemath.
