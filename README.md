# OnePhysics for .NET

Deterministic 2D and 3D physics for .NET (`RecursiveAI.OnePhysics`), written from first principles: the same world state, bit for bit, on every OS, CPU, vector width, WebAssembly mode and thread count. Rigid bodies, shapes, contacts and joints; rods, ropes, cables, cloth and soft bodies; fracture and carving; particle fluids, shallow water and granular matter; articulations; fields; characters, vehicles and ragdolls; rollback tooling. No dependencies beyond the .NET base class library and [onemath](https://github.com/recursive-ai-oss/onemath-csharp), which supplies the deterministic math. NativeAOT and trimming compatible.

**Status.** Each part is either supported or marked `[Experimental]` (the 3D engine, the raw tier and the later features carry diagnostic ids `OPHYS0001`–`OPHYS0003`; the list of what each covers is in [docs/spec/api.md](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/spec/api.md)). The [changelog](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/CHANGELOG.md) says which releases moved results; a snapshot or replay from a release that did is not guaranteed to load or reproduce in the next.

```
dotnet add package RecursiveAI.OnePhysics
```

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

## 3D

The 3D world has the same members with 3D math types. While the 3D engine carries `[Experimental("OPHYS0002")]` (see [the API surface](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/spec/api.md)), using it raises that diagnostic until you acknowledge it:

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

- **Deterministic.** Only IEEE-exact `+ - * /` and `sqrt` plus onemath's transcendental functions; fixed-order reductions and constraint colouring. The world hash after N steps is the same on x64, arm64 and WebAssembly, scalar or SIMD, one thread or all cores. Positions cross the API as `double`, exact within ±2^21 (a deterministic game host's Q31.32 fixed-point format converts without loss); larger worlds shift their origin.
- **Three tiers, one engine.** One-liners (`world.AddBox(...)`, `body.OnCollisionEnter`, `world.AddCharacter(...)`) with Unity's vocabulary, definitions and polled events with struct callbacks for the hot paths, and a raw tier (`LowLevel`) with the storage, colouring, trees and narrowphase. Each tier is built on the one below. Nothing on the happy path asks you to write a struct callback.
- **Two engines, the same members.** `PhysicsWorld2D` and `PhysicsWorld3D` share `Body`, `Shape`, `Joint`, events, queries, options and presets in shape; where physics differs (voxels and meshes in 3D, tile grids and chains in 2D) [the API notes](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/spec/api.md) list it.
- **Rollback-ready.** Byte-exact `Snapshot` and `Restore`, delta snapshots, `ComputeHash` per step, a desync finder, world fork, a replay format, `SuppressCallbacks` for resimulation, allocation-free steady state after `Warmup`.
- **Handles you can hold.** `Body`, `Shape` and `Joint` are small structs; a destroyed one throws a `StaleHandleException` naming it.
- **Hostile input is refused,** with typed errors that name the value and the limit.

## Platforms

`net10.0` on Linux, macOS and Windows (x64 and arm64) and browser WebAssembly, under the JIT and NativeAOT.

## Related packages

- `RecursiveAI.OnePhysics.Tools`: authoring-time geometry, never needed at runtime. Convex decomposition of triangle meshes, hull simplification, mesh cleaning and mass properties, tetrahedralization and quality triangulation, ragdoll fitting from a skeleton, 2D polygon decomposition and sprite outlines. See [its page](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/src/OnePhysics.Tools/README.md) and [the specification](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/spec/tools.md).
<!-- Interchange: when RecursiveAI.OnePhysics.Interchange ships, add its line here: importers for glTF physics, URDF, MJCF, Tiled and LDtk. -->

## Guide

- [Install](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/guide/install.md) and [quick start](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/guide/quick-start.md)
- [The three tiers](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/guide/tiers.md)
- [Events](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/guide/events.md) and [queries, layers and filters](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/guide/queries-and-layers.md)
- [Rollback and lockstep](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/guide/rollback.md) and [determinism guarantees](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/guide/determinism.md)
- [Units and sizes](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/guide/units.md), [names and conventions](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/guide/api-names.md), [input limits](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/guide/limits.md)
- [Soft bodies and cloth](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/guide/soft-bodies.md), [fluids](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/guide/fluids.md), [water](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/guide/water.md), [terrain](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/guide/terrain.md), [destruction](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/guide/destruction.md), [articulations](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/guide/articulations.md), [world scale](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/guide/world-scale.md)
- [Coming from Unity](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/guide/unity-mapping.md)
- [Port specifications](https://github.com/recursive-ai-oss/onephysics-csharp/tree/main/docs/spec): the normative, language-neutral description of every choice that affects the bits.
- [Changelog](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/CHANGELOG.md), [contributing](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/CONTRIBUTING.md) and [third-party notices](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/THIRD-PARTY-NOTICES.md)

## Licence

`MIT OR Apache-2.0`, at your option. Copyright (c) Recursive AI, LLC. See `LICENSE-MIT` and `LICENSE-APACHE`.
