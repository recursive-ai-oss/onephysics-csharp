# Install

```
dotnet add package RecursiveAI.OnePhysics --prerelease
```

The package id is `RecursiveAI.OnePhysics`. Its only dependency is `RecursiveAI.OneMath` (the deterministic vectors and functions the API is written in: `Vector2F`, `Vector2D`, `Vector3F`, `Vector3D`), which NuGet restores for you.

- **Target:** `net10.0`.
- **NativeAOT and trimming:** the package is marked AOT-compatible and trimmable and has no reflection over your types. The package is one assembly for both engines; trimming keeps what you reference, but a 2D-only app shedding the whole 3D engine is a goal, not yet a guarantee.
- **Browser WebAssembly:** supported, single-threaded (leave `ThreadCount` at its default of 1 there). A world gives the same bits in the browser as on a desktop.
- **Platforms:** results are bit-identical on Windows, Linux, macOS, x64 and arm64.

## Preview markers

Two parts of the package carry `[Experimental]` and fail to compile until you opt in, so you cannot use them by accident:

| Part | Diagnostic | Opt in |
|---|---|---|
| The 3D engine, namespace `RecursiveAI.OnePhysics.Physics3D` | `OPHYS0002` | `<NoWarn>$(NoWarn);OPHYS0002</NoWarn>` in the project, or `#pragma warning disable OPHYS0002` in a file |
| The raw tier, namespace `RecursiveAI.OnePhysics.LowLevel` | `OPHYS0001` | the same, with `OPHYS0001` |

Both may change between previews. **0.x previews are not bit-compatible with each other**: do not carry snapshots, saved games or replays across a package update (the [changelog](../../CHANGELOG.md) says which previews moved results).

## Next

[Quick start](quick-start.md).
