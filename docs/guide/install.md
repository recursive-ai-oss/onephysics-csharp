# Install

```
dotnet add package RecursiveAI.OnePhysics --prerelease
```

The package id is `RecursiveAI.OnePhysics`. Its only dependency is `RecursiveAI.OneMath` (the deterministic vectors and functions the API is written in: `Vector2F`, `Vector2D`, `Vector3F`, `Vector3D`), which NuGet restores for you.

- **Target:** `net10.0`.
- **NativeAOT and trimming:** the package is marked AOT-compatible and trimmable and has no reflection over your types. The package is one assembly for both engines; trimming keeps what you reference, but a 2D-only app shedding the whole 3D engine is a goal, not yet a guarantee.
- **Browser WebAssembly:** supported, single-threaded (leave `ThreadCount` at its default of 1 there). A world gives the same bits in the browser as on a desktop.
- **Platforms:** results are bit-identical on Windows, Linux, macOS, x64 and arm64.

## The experimental raw tier

One part of the package carries `[Experimental("OPHYS0001")]` and fails to compile until you opt in, so you cannot use it by accident: the raw tier, namespace `RecursiveAI.OnePhysics.LowLevel` (storage layouts, constraint batches and colours, trees, narrowphase routines). Opt in with `<NoWarn>$(NoWarn);OPHYS0001</NoWarn>` in the project, or `#pragma warning disable OPHYS0001` in a file. Its shapes follow the engine's internals and may change in a minor release; everything else, the 3D engine included, is supported under the [compatibility promise](determinism.md). [The API notes](../spec/api.md) list what is marked.

## Next

[Quick start](quick-start.md).
