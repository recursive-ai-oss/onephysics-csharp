# Determinism guarantees

**The promise:** a world's state after N steps depends only on its inputs (the options, the calls you made, in order, and the dt you passed) and on the package version. It does not depend on the operating system, the CPU, the SIMD width, the runtime version or JIT tier, native versus NativeAOT versus browser WebAssembly, or the thread count.

## How, in short

- Only IEEE-exact `+ - * /` and `sqrt` on `float` and `double`, in a fixed order; no fused multiply-add and no estimate instructions. Every transcendental function comes from onemath, which returns the correctly rounded result.
- Reductions and parallel work combine in a fixed order. Constraints are colored and batched from the constraint graph alone, so one thread, all cores and single-threaded WebAssembly give the same bits. Wide SIMD kernels compute, lane by lane, exactly what the scalar code computes.
- Ties break on stable indices (body, shape, constraint, feature id, insertion order), never on hash codes or the order of a dictionary.
- The world has no clock. Step and iteration counts come from options; `dt` is yours.

The repository's hash-matrix tool steps its scenes and compares the per-step world hash across x64 and arm64, scalar and every vector width, one thread and all cores, JIT and NativeAOT, and WebAssembly. [../spec/hash-matrix.md](../spec/hash-matrix.md) describes it and [../spec/architecture.md](../spec/architecture.md) defines the hash and the snapshot layout.

## Compatibility across 1.x

- Results are bit-identical across operating systems, CPUs, vector widths, WebAssembly modes and thread counts in every release.
- Within 1.x, patch releases never change a result: the same inputs give the same world, bit for bit.
- A minor release may change results (for example a faster or more accurate solver path); the [changelog](../../CHANGELOG.md) says so whenever one does. Peers in a lockstep or rollback session must run the same minor version.
- Snapshots, delta snapshots, replays and scene files written by any 1.x release load in every later 1.x release.
- The public API follows semantic versioning.

## What is not promised

- **Before 1.0.** The 0.x previews were not bit-compatible with each other; nothing saved by one carries to 1.0.
- **Different inputs.** Two worlds fed the same calls in a different order are different worlds: creation order decides handles and slots, and slots decide event order. Different `dt`, substeps, iterations or gravity are different simulations. `ThreadCount` and `WorldId` are the exceptions: they never change a result.
- **Your code.** The world is deterministic; your game around it must be too. Do not feed it `Math.Sin` results, wall-clock times or a hash-ordered loop, and use the same `dt` on every peer. onemath provides the deterministic functions (`RecursiveAI.OneMath.Math64` and friends).
- **Single writer.** Call a world from one thread at a time. Queries only read, and single queries may run concurrently when nothing writes.

## The coordinate rule

Body positions cross the public API as `double` (`Vector2D`, `Vector3D`). Inside the engine they are stored as `double` relative to a world origin; rotations, velocities and solver math are `float`.

- **Exact within ±2^21.** A host that keeps positions in Q31.32 fixed point can pass any such value: it converts to `double` exactly for |v| ≤ 2^21 (about 2.1 million units), including both endpoints, and the engine stores it exactly. A body created beyond ±2^21 is rejected with an `ArgumentOutOfRangeException` naming the value and the bound.
- **Fixed bridge.** OneMath 1.0 `FixedBridge.ExclusiveMagnitudeBound` deliberately checks the open interval |v| < 2^21. Worlds accept the closed interval; do not pass its endpoints to that helper. The wider ±2^22 simulation bound allows integration and shape headroom; it is not a wider creation or teleport range.
- **Larger worlds: `ShiftOrigin`.** Call `world.ShiftOrigin(offset)` between steps, by the same offset at the same step on every peer. Every body position decreases by `offset`, `world.Origin` grows by it, and a host coordinate is `world.Origin + body.Position`. Offset components must be multiples of 2^-31 (every other Q31.32 raw value) within ±2^22. A grid violation raises `ArgumentException`; a non-finite or oversized offset raises `ArgumentOutOfRangeException`, naming the value; nothing changes when it refuses. A shifted world then steps exactly like the unshifted one, translated: positions differ by the offset and everything else is bit-identical (positions advance on a 2^-32 grid for this; see the world spec). The origin is part of the snapshot and of the hash.
- **Past ±2^22** a moving body's bounds are out of range and `Step` throws `BodyOutOfBoundsException` before changing anything.

<!-- example: Guide/Determinism.cs -->
```csharp
var world = new PhysicsWorld2D();
var crate = world.AddBox(new Vector2F(1, 1), at: new Vector2D(2_000_000, 0));
world.Step(1f / 60);
world.ShiftOrigin(new Vector2D(2_000_000, 0));          // between steps, by the same amount on every peer
double hostX = world.Origin.X + crate.Position.X;       // the position in host coordinates is unchanged
```

Shifting exists for the ±2^21 bound. Contact and solver math runs on float32 values relative to a body or contact point, so a large coordinate does not make contacts coarser.
