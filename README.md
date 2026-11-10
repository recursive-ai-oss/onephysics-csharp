# OnePhysics for .NET

Deterministic 2D and 3D physics for .NET (`RecursiveAI.OnePhysics`), written from first principles: the same world state, bit for bit, on every OS, CPU, vector width, WebAssembly mode and thread count. Rigid bodies, shapes, contacts and joints; rods, ropes, cables, cloth and soft bodies; fracture and carving; particle fluids, shallow water and granular matter; articulations; fields; characters, vehicles and ragdolls; rollback tooling. No dependencies beyond the .NET base class library and [onemath](https://github.com/recursive-ai-oss/onemath-csharp), which supplies the deterministic math. NativeAOT and trimming compatible.

**Status.** 1.0. Everything is supported except the raw tier (`LowLevel`: storage layouts, constraint batches and colours, trees), which stays `[Experimental("OPHYS0001")]` because its shapes follow the engine's internals; [docs/spec/api.md](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/spec/api.md) lists it.

**Compatibility across 1.x.**
- Results are bit-identical across operating systems, CPUs, vector widths, WebAssembly modes and thread counts in every release.
- Within 1.x, patch releases never change a result: the same inputs give the same world, bit for bit.
- A minor release may change results (for example a faster or more accurate solver path); the [changelog](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/CHANGELOG.md) says so whenever one does. Peers in a lockstep or rollback session must run the same minor version.
- Snapshots, delta snapshots, replays and scene files written by any 1.x release load in every later 1.x release.
- The public API follows semantic versioning.

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

The 3D world has the same members with 3D math types:

<!-- example: Quickstart3D.cs -->
```csharp
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
- **Two engines, the same members.** `PhysicsWorld2D` and `PhysicsWorld3D` share `Body`, `Shape`, `Joint`, events, queries, options and presets in shape; where physics differs (voxels and meshes in 3D, tile grids and chains in 2D) [the API notes](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/spec/api.md) list it.
- **Rollback-ready.** Byte-exact `Snapshot` and `Restore`, delta snapshots, `ComputeHash` per step, a desync finder, world fork, a replay format, `SuppressCallbacks` for resimulation, allocation-free steady state after `Warmup`.
- **Handles you can hold.** `Body`, `Shape` and `Joint` are small structs; a destroyed one throws a `StaleHandleException` naming it.
- **Hostile input is refused,** with typed errors that name the value and the limit.

## Speed

Measured against the managed .NET engines in their default configurations (BepuPhysics 2.5.0-beta.33, Jitter2 2.13.0, Box2D.NET 3.1.654), median step time, on the same scene files, with tiered compilation off. Machines: AMD Ryzen 9 7950X3D (x64, 32 logical CPUs; 8-thread rows on one core complex) and Apple M2 (arm64, 8 cores). Method and scenes: [docs/spec/bench.md](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/spec/bench.md).

- **2D:** at or ahead of Box2D.NET on every scene at every thread count (large pile 0.11× its step time at 1 thread on x64, 0.17× on arm64).
- **3D:** within 15% of the fastest stable managed engine, or faster, on large piles, mesh and height-field terrain, joint grids and queries at 1 thread; joint chains and tall pyramids, where no managed engine is stable at its defaults, are gated on stability only.
- **Soft bodies:** ahead of Jitter2 on cloth. Our cloth sleeps once it settles, so the median step is a sleeping one; while awake, an 8-thread step is about 0.46 ms against Jitter2's 0.13–0.19 ms.
- **Where we are slower** (ratio of our step time to the fastest stable managed engine; every row that misses is listed; all other measured rows are within 15% or faster):

| scene | x64 1 thread | x64 8 threads | x64 all cores | arm64 1 thread | arm64 8 threads |
|---|---|---|---|---|---|
| large pile (3000 bodies) | 0.92× (pass) | 1.05× (pass) | 1.19× | 0.81× (pass) | 0.76× (pass) |
| ragdoll pile (1500 parts) | 2.19× | 1.28× | 1.64× | 1.43× | 1.03× (pass) |
| sleep–wake (3000 bodies) | 2.30× | 1.45× | 1.18× | 1.64× | 1.03× (pass) |
| queries (3000-body pile) | 1.14× (pass) | 1.22× | 1.29× | 0.93× | 0.87× |

  The gap is per-pair contact cost and the solver's pass structure (four substeps for jointed islands), and at all cores the cost of synchronising many threads on small stages; the determinism rules don't cost it.

- **Stable everywhere:** every scene meets its stability thresholds on both machines (the 113 scenes and their limits are in [docs/spec/scenes.md](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/spec/scenes.md)).

## Platforms

`net10.0` on Linux, macOS and Windows (x64 and arm64) and browser WebAssembly, under the JIT and NativeAOT.

## Related packages

- `RecursiveAI.OnePhysics.Tools`: authoring-time geometry, never needed at runtime. Convex decomposition of triangle meshes, hull simplification, mesh cleaning and mass properties, tetrahedralization and quality triangulation, ragdoll fitting from a skeleton, 2D polygon decomposition and sprite outlines. See [its page](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/src/OnePhysics.Tools/README.md) and [the specification](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/spec/tools.md).
- `RecursiveAI.OnePhysics.Interchange`: importers for URDF and MJCF robots, glTF physics scenes, and Tiled and LDtk levels (see Importers below).

## Importers

`RecursiveAI.OnePhysics.Interchange` reads URDF and MJCF robots into articulations, glTF files with physics extensions into 3D scenes, and Tiled and LDtk levels into 2D tile grids, shapes and markers. It touches no file system, reports what it cannot express as warnings, and refuses hostile files with a typed error. See [the importer specification](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/spec/interchange.md).

## Guide

- [Install](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/guide/install.md) and [quick start](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/guide/quick-start.md)
- [The three tiers](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/guide/tiers.md)
- [Events](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/guide/events.md) and [queries, layers and filters](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/guide/queries-and-layers.md)
- [Rollback and lockstep](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/guide/rollback.md) and [determinism guarantees](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/guide/determinism.md)
- [Units and sizes](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/guide/units.md), [names and conventions](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/guide/api-names.md), [input limits](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/guide/limits.md)
- [Soft bodies and cloth](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/guide/soft-bodies.md), [fluids](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/guide/fluids.md), [water](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/guide/water.md), [terrain](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/guide/terrain.md), [destruction](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/guide/destruction.md), [articulations](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/guide/articulations.md), [world scale](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/guide/world-scale.md)
- [Coming from Unity](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/docs/guide/unity-mapping.md)
- [Port specifications](https://github.com/recursive-ai-oss/onephysics-csharp/tree/v1.0.0/docs/spec): the normative, language-neutral description of every choice that affects the bits.
- [Changelog](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/CHANGELOG.md), [contributing](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/CONTRIBUTING.md) and [third-party notices](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.0.0/THIRD-PARTY-NOTICES.md)

## Licence

`MIT OR Apache-2.0`, at your option. Copyright (c) Recursive AI, LLC. See `LICENSE-MIT` and `LICENSE-APACHE`.
