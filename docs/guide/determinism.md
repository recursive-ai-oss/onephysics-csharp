# Determinism guarantees

**The promise:** a world's state after N steps depends only on its inputs (the options, the calls you made, in order, and the dt you passed) and on the package version. It does not depend on the operating system, the CPU, the SIMD width, the runtime version or JIT tier, native versus NativeAOT versus browser WebAssembly, or the thread count.

## How, in short

- Only IEEE-exact `+ - * /` and `sqrt` on `float` and `double`, in a fixed order; no fused multiply-add and no estimate instructions. Every transcendental function comes from onemath, which returns the correctly rounded result.
- Reductions and parallel work combine in a fixed order. Constraints are colored and batched from the constraint graph alone, so one thread, all cores and single-threaded WebAssembly give the same bits. Wide SIMD kernels compute, lane by lane, exactly what the scalar code computes.
- Ties break on stable indices (body, shape, constraint, feature id, insertion order), never on hash codes or the order of a dictionary.
- The world has no clock. Step and iteration counts come from options; `dt` is yours.

The repository's hash-matrix tool steps its scenes and compares the per-step world hash across x64 and arm64, scalar and every vector width, one thread and all cores, JIT and NativeAOT, and WebAssembly. [../spec/hash-matrix.md](../spec/hash-matrix.md) describes it and [../spec/architecture.md](../spec/architecture.md) defines the hash and the snapshot layout.

## What is not promised

- **Between previews.** 0.x previews are not bit-compatible with each other. A snapshot, saved game or replay from one preview does not carry to another, and a world stepped on a newer preview may differ from one stepped on an older. The [changelog](../../CHANGELOG.md) lists the previews that moved results. The 1.0 contract is the one that will be kept across versions.
- **Different inputs.** Two worlds fed the same calls in a different order are different worlds: creation order decides handles and slots, and slots decide event order. Different `dt`, substeps, iterations or gravity are different simulations. `ThreadCount` and `WorldId` are the exceptions: they never change a result.
- **Your code.** The world is deterministic; your game around it must be too. Do not feed it `Math.Sin` results, wall-clock times or a hash-ordered loop, and use the same `dt` on every peer. onemath provides the deterministic functions (`RecursiveAI.OneMath.Math64` and friends).
- **Single writer.** Call a world from one thread at a time. Queries only read, and single queries may run concurrently when nothing writes.

## The coordinate rule

Body positions cross the public API as `double` (`Vector2D`, `Vector3D`). Inside the engine they are stored as `double` relative to a world origin; rotations, velocities and solver math are `float`.

- **Exact within ±2^21.** A host that keeps positions in Q31.32 fixed point can pass any such value: it converts to `double` exactly for |v| < 2^21 (about 2.1 million units), and the engine stores it exactly. A body created beyond ±2^21 is rejected with an `ArgumentOutOfRangeException` naming the value and the bound.
- **Larger worlds: `ShiftOrigin`.** Call `world.ShiftOrigin(offset)` between steps, by the same offset at the same step on every peer. Every body position decreases by `offset`, `world.Origin` grows by it, and a host coordinate is `world.Origin + body.Position`. Offset components must be multiples of 2^-31 (every Q31.32 value is) within ±2^22, else an exception names the value; nothing changes when it refuses. The origin is part of the snapshot and of the hash.
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
