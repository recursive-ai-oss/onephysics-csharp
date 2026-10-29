# Quick start

The namespaces you import: `RecursiveAI.OneMath` for the vector types, `RecursiveAI.OnePhysics` for what both dimensions share (`BodyType`, `EventKind`, `EventMask`, exceptions, raw handles), and `RecursiveAI.OnePhysics.Physics2D` (or `Physics3D`) for the world, `Body`, `Shape`, `Joint`, `Character` and the definitions.

## A world, some bodies, a step loop

<!-- example: Guide/QuickStart.cs -->
```csharp
var world = new PhysicsWorld2D();
var ground = world.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
var crate = world.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 3), material: new Material2D(friction: 0.8f));
var ball = world.AddCircle(0.5f, at: new Vector2D(2, 4));
ball.LinearVelocity = new Vector2F(-1, 0);
crate.OnCollisionEnter += c => Console.WriteLine($"crate hit {c.Other}");
for (int i = 0; i < 120; i++) world.Step(1f / 60);
Console.WriteLine($"{crate.Position} {crate.LinearVelocity} awake={crate.IsAwake}");
```

- Positions are `double` (`Vector2D`); sizes, velocities and everything else are `float` (`Vector2F`).
- `AddBox(size)` takes the **full** size. See [units](units.md).
- `Body` is a small struct. Copy it freely; it allocates nothing. After `body.Destroy()` any use throws `StaleHandleException`.
- `Step` takes a fixed dt. The world has no clock; always pass the same value (see [determinism](determinism.md)). After the step, handlers run and the events of the step are available to poll ([events](events.md)).

## A character and a ray

<!-- example: Guide/QuickStart.cs -->
```csharp
var player = world.AddCharacter(at: new Vector2D(-3, 1));
player.Move(new Vector2F(0.05f, -0.1f), 1f / 60);   // this frame's displacement, not a velocity
world.Step(1f / 60);
bool grounded = player.IsGrounded;
if (world.Raycast(new Vector2D(0, 10), new Vector2F(0, -1), out var hit)) Console.WriteLine($"{hit.Body} at {hit.Point}, {hit.Distance} away");
```

`Move` takes the displacement for this frame (use `MoveVelocity` for a velocity times dt) and slides along slopes, climbs steps and pushes bodies; `CharacterDef.Humanoid`, `Child` and `Giant` are presets, and `AddCharacter(at, def)` takes your own tuned copy.

## 3D, a preview

The 3D engine is `[Experimental("OPHYS0002")]`; acknowledge it with `#pragma warning disable OPHYS0002` or a `NoWarn`. The world has the same members with 3D math types.

<!-- example: Guide/QuickStart3D.cs -->
```csharp
var world = new PhysicsWorld3D();
world.AddBox(new Vector3F(20, 1, 20), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
var crate = world.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 3, 0));
var ball = world.AddSphere(0.5f, at: new Vector3D(2, 4, 0));
for (int i = 0; i < 120; i++) world.Step(1f / 60);
Console.WriteLine($"{crate.Position} {ball.Position}");
```

To use both dimensions in one file, import neither dimension namespace and alias what you use: `using Body2D = RecursiveAI.OnePhysics.Physics2D.Body;` and `using Body3D = RecursiveAI.OnePhysics.Physics3D.Body;`.

## Next

[The three tiers](tiers.md) for definitions, filters and polled events; [rollback](rollback.md) if your host resimulates.
