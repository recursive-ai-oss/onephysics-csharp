# onephysics-csharp

Deterministic 2D and 3D rigid-body physics for .NET, written from first principles: the same world state, bit for bit, on every platform, CPU and thread count. Package `RecursiveAI.OnePhysics` (name provisional), built on [onemath](https://github.com/recursive-ai-oss/onemath-csharp) for its deterministic math.

**Status: preview (0.1).** The 2D engine (circles, capsules, rounded polygons, segments and chains; revolute, prismatic, weld, wheel, distance, motor and mouse joints; bullets and continuous collision; a character mover; layers, filters, sensors, events and queries; byte-exact snapshot and restore; per-step world hashes; debug geometry) is complete and deterministic across x64, arm64 and browser WebAssembly, at any vector width and thread count. The 3D engine is in progress. Speed is not yet at parity with the fastest native engines, and the raw tier (`RecursiveAI.OnePhysics.LowLevel`) may still change.

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

The 3D world has the same members with 3D math types:

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

Joints come back as typed views (`HingeJoint`, `SliderJoint`, ... each converts implicitly to `Joint`), so a member that another kind of joint has does not compile. Rollback hosts restore a snapshot and resimulate inside `SuppressCallbacks()`, which mutes per-body handlers but not polled events:

```csharp
using System;
using RecursiveAI.OneMath;               // Vector2F, Vector2D
using RecursiveAI.OnePhysics;            // BodyType, EventKind
using RecursiveAI.OnePhysics.Physics2D;  // PhysicsWorld2D, Body, HingeJoint

var world = new PhysicsWorld2D();
var ground = world.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -1), type: BodyType.Static);
var ball = world.AddCircle(0.5f, at: new Vector2D(0, 3));
HingeJoint hinge = world.AddHinge(ground, ball, new Vector2D(0, 3));   // a typed view: only the members a hinge has
hinge.EnableMotor = true;
hinge.MaxMotorTorque = 5f;
int sounds = 0;
ball.OnCollisionEnter += c => sounds++;

byte[] before = world.Snapshot();                // one per frame in a real host
for (int i = 0; i < 120; i++) world.Step(1f / 60);
world.Restore(before);                           // a late input arrived: roll back and resimulate
using (world.SuppressCallbacks())                // no sounds or particles twice; polled events still work
{
    for (int i = 0; i < 120; i++) world.Step(1f / 60);
}
```

## Namespaces

- `RecursiveAI.OnePhysics`: what game code names everywhere: `BodyType`, `EventKind`, `EventMask`, `PhysicsEvent`, the handles (`BodyHandle`, `ShapeHandle`, `JointHandle`) and the exceptions.
- `RecursiveAI.OnePhysics.Physics2D` and `RecursiveAI.OnePhysics.Physics3D`: the world, `Body`, `Shape`, `Joint`, `Character`, the definitions (`BodyDef`, `ShapeDef`, the joint definitions, `CharacterDef`; each is valid at its zero value and then means its documented defaults), queries and options. The two namespaces have the same member names.

  To use both dimensions in one file, import neither and alias what you use: `using Body2D = RecursiveAI.OnePhysics.Physics2D.Body;` and `using Body3D = RecursiveAI.OnePhysics.Physics3D.Body;`. The names stay the same in both so code reads alike; the alias says which world a variable belongs to.

Sizes are full sizes (`AddBox(size)`), angles are radians (`CharacterDef.SlopeLimit` included), and a parameter that is a half extent says `half` in its name. [docs/guide/api-names.md](docs/guide/api-names.md) lists every convention, the table from `AddHinge`-style names to joint definitions, and the rule for handlers across `Restore`.
- `RecursiveAI.OnePhysics.Debug`: debug geometry as plain data.
- `RecursiveAI.OnePhysics.LowLevel`: the raw tier (storage, scheduler, broadphase trees, narrowphase and manifold routines, snapshot readers and writers). Marked `[Experimental("OPHYS0001")]`: using it needs `<NoWarn>$(NoWarn);OPHYS0001</NoWarn>`, and it may change between previews.

See [CONTRIBUTING.md](CONTRIBUTING.md) to build.

## Licence

`MIT OR Apache-2.0`, at your option. Copyright (c) Recursive AI, LLC. See `LICENSE-MIT` and `LICENSE-APACHE`.
