# onephysics-csharp

Deterministic 2D and 3D rigid-body physics for .NET, written from first principles: the same world state, bit for bit, on every platform, CPU and thread count. Package `RecursiveAI.OnePhysics` (name provisional), built on [onemath](https://github.com/recursive-ai-oss/onemath-csharp) for its deterministic math.

**Status: preview (0.1).** The 2D engine (circles, capsules, rounded polygons, segments and chains; revolute, prismatic, weld, wheel, distance, motor and mouse joints; bullets and continuous collision; a character mover; layers, filters, sensors, events and queries; byte-exact snapshot and restore; per-step world hashes; debug geometry) is complete and deterministic across x64, arm64 and browser WebAssembly, at any vector width and thread count. The 3D engine is in progress. Speed is not yet at parity with the fastest native engines, and the shared core (storage, scheduler, snapshot format) may still change before the 3D release.

```csharp
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

See [CONTRIBUTING.md](CONTRIBUTING.md) to build.

## Licence

`MIT OR Apache-2.0`, at your option. Copyright (c) Recursive AI, LLC. See `LICENSE-MIT` and `LICENSE-APACHE`.
