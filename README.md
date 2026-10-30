# OnePhysics for .NET

Deterministic 2D rigid-body physics for .NET, written from first principles: the same world state, bit for bit, on every platform, CPU, vector width and thread count. Package `RecursiveAI.OnePhysics`, built on [onemath](https://github.com/recursive-ai-oss/onemath-csharp) for its deterministic math.

**Status: preview.** The 2D engine is the supported surface: circles, capsules, rounded polygons, segments and chains; revolute, prismatic, weld, wheel, distance, motor and mouse joints; bullets and continuous collision; a character mover; layers, filters, sensors, events and queries; byte-exact snapshot and restore; per-step world hashes; debug geometry. The 3D engine (`RecursiveAI.OnePhysics.Physics3D`) ships as a **preview marked `[Experimental("OPHYS0002")]`**: it works and is deterministic, but its API and its results may change between previews. The raw tier (`RecursiveAI.OnePhysics.LowLevel`) is `[Experimental("OPHYS0001")]`. Previews are not bit-compatible with each other: a snapshot or replay from one preview does not carry to the next.

## Install

```
dotnet add package RecursiveAI.OnePhysics --prerelease
```

Targets `net10.0`; works under NativeAOT, trimming and browser WebAssembly. Its only dependency is `RecursiveAI.OneMath`.

## 2D in a minute

<!-- example: Quickstart2D.cs -->
```csharp
using System;
using RecursiveAI.OneMath;               // Vector2F, Vector2D
using RecursiveAI.OnePhysics;            // BodyType, events, exceptions
using RecursiveAI.OnePhysics.Physics2D;  // PhysicsWorld2D, Body, Character

var world = new PhysicsWorld2D();
world.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -1), type: BodyType.Static);
var ball = world.AddCircle(0.5f, at: new Vector2D(0, 5));
ball.OnCollisionEnter += c => Console.WriteLine($"hit {c.Other} along {c.Normal}");

var player = world.AddCharacter(at: new Vector2D(3, 1));
for (int i = 0; i < 600; i++)
{
    player.Move(new Vector2F(0.05f, -0.1f), 1f / 60);
    world.Step(1f / 60);                        // fixed dt; events fire after the step, in a deterministic order
}
if (world.Raycast(new Vector2D(0, 10), new Vector2F(0, -1), out var hit)) Console.WriteLine(hit.Point);

byte[] saved = world.Snapshot();                // byte-exact; Restore(saved) continues bit for bit
ulong hash = world.ComputeHash();               // the same on every platform
```

## 3D, a preview

The 3D world has the same members with 3D math types. Using it raises the experimental diagnostic `OPHYS0002` until you acknowledge it:

<!-- example: Quickstart3D.cs -->
```csharp
#pragma warning disable OPHYS0002        // the 3D engine is a preview
using System;
using RecursiveAI.OneMath;               // Vector3F, Vector3D
using RecursiveAI.OnePhysics;            // BodyType, events, exceptions
using RecursiveAI.OnePhysics.Physics3D;  // PhysicsWorld3D, Body, Character

var world = new PhysicsWorld3D();
world.AddBox(new Vector3F(20, 1, 20), at: new Vector3D(0, -1, 0), type: BodyType.Static);
var ball = world.AddSphere(0.5f, at: new Vector3D(0, 5, 0));
ball.OnCollisionEnter += c => Console.WriteLine($"hit {c.Other} along {c.Normal}");

for (int i = 0; i < 300; i++) world.Step(1f / 60);
if (world.Raycast(new Vector3D(0, 10, 0), new Vector3F(0, -1, 0), out var hit)) Console.WriteLine(hit.Point);

byte[] saved = world.Snapshot();
ulong hash = world.ComputeHash();
```

## What you get

- **Deterministic.** Only IEEE-exact `+ - * /` and `sqrt` and our own transcendental functions; fixed-order reductions and constraint coloring. The world hash after N steps is the same on x64, arm64 and WebAssembly, scalar or SIMD, one thread or all cores.
- **Three tiers, one engine.** One-liners (`world.AddBox(...)`, `body.OnCollisionEnter`), definitions and polled events, and a raw tier, each built on the one below.
- **Rollback-ready.** Byte-exact `Snapshot`/`TrySnapshot` and `Restore`, `ComputeHash` per step, `SuppressCallbacks` for resimulation, allocation-free steady state after `Warmup`.
- **Handles you can hold.** `Body`, `Shape` and `Joint` are small structs; a destroyed one throws a `StaleHandleException` naming it.
- **Hostile input is refused,** with typed errors that name the value and the limit.

## Guide

- [Install](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v0.4.0-preview.1/docs/guide/install.md) and [quick start](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v0.4.0-preview.1/docs/guide/quick-start.md)
- [The three tiers](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v0.4.0-preview.1/docs/guide/tiers.md)
- [Events](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v0.4.0-preview.1/docs/guide/events.md)
- [Queries, layers and filters](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v0.4.0-preview.1/docs/guide/queries-and-layers.md)
- [Rollback and lockstep](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v0.4.0-preview.1/docs/guide/rollback.md)
- [Determinism guarantees](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v0.4.0-preview.1/docs/guide/determinism.md)
- [Units and sizes](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v0.4.0-preview.1/docs/guide/units.md), [names and conventions](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v0.4.0-preview.1/docs/guide/api-names.md), [input limits](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v0.4.0-preview.1/docs/guide/limits.md)
- [Coming from Unity](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v0.4.0-preview.1/docs/guide/unity-mapping.md)
- [Port specifications](https://github.com/recursive-ai-oss/onephysics-csharp/tree/v0.4.0-preview.1/docs/spec): the normative, language-neutral description of every choice that affects the bits.
- [Changelog](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v0.4.0-preview.1/CHANGELOG.md) and [contributing](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v0.4.0-preview.1/CONTRIBUTING.md)

## Licence

`MIT OR Apache-2.0`, at your option. Copyright (c) Recursive AI, LLC. See `LICENSE-MIT` and `LICENSE-APACHE`.
