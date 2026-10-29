# The three tiers

Each tier is built on the one below, so you can start at the top and drop down for one object without rewriting the rest.

## 1. The happy path

`new PhysicsWorld2D()`, `world.AddBox(size, at: ...)`, `body.OnCollisionEnter += ...`, `world.Raycast(...)`, `world.AddCharacter(...)`, `world.Step(dt)`. Unity's vocabulary, full sizes, radians, sensible defaults (gravity -9.81, four substeps, friction 0.6, sleeping on). Nothing here asks you to implement an interface or write a generic struct. See the [quick start](quick-start.md).

Filtering on this tier is a layer matrix (`body.Layer`, `world.IgnoreLayerCollision(a, b)`) plus an optional delegate (`WorldOptions2D.PairFilter`, a `PairPredicate`) and query filters with an optional predicate. See [queries, layers and filters](queries-and-layers.md).

## 2. The middle tier

Definitions with defaults, polled events, a reusable query object, struct callbacks and solver control.

- **`BodyDef`, `ShapeDef` and the joint definitions** are valid at their zero value and then mean their documented defaults (`new ShapeDef { Friction = 0.9f }` changes only friction). They are structs; fill one, then pass it to `CreateBody`, `CreateBox` and the other `Create*` methods, or `CreateJoint(ref ...)`. The `Add*` one-liners are thin wrappers over these.
- **`WorldOptions2D`** (`WorldOptions3D` in 3D): gravity, `Substeps`, `VelocityIterations`, `RelaxIterations`, sleep settings, the event mask, `ThreadCount`, `WorldId` and capacity options. Substep and iteration counts are inputs of the simulation, never chosen from the clock or the machine.
- **`GetEvents(kind)` and `Events`** return the events of the last step as a span, with no allocation.
- **`PhysicsQuery2D`/`PhysicsQuery3D`** hold a world and a filter and add batch forms that run many queries in parallel.
- **`SetContactFilter<T>(T filter) where T : struct, IContactFilter`** decides per shape pair, specialized for your struct.

<!-- example: Guide/Tiers.cs -->
```csharp
struct NotTheSameBody : IContactFilter
{
    public bool ShouldCollide(Shape a, Shape b) => a.Body != b.Body;
}
```

<!-- example: Guide/Tiers.cs -->
```csharp
var world = new PhysicsWorld2D(new WorldOptions2D
{
    Gravity = new Vector2F(0, -9.81f),
    Substeps = 4,                 // substeps per Step call
    VelocityIterations = 1,       // solve passes with bias per substep
    RelaxIterations = 1,          // relax passes without bias per substep
});
var floor = world.CreateBody(new BodyDef { Type = BodyType.Static, Position = new Vector2D(0, -0.5) });
world.CreateBox(floor, new Vector2F(20, 1), def: new ShapeDef { Friction = 0.9f, Layer = 1 });
var box = world.CreateBody(new BodyDef { Position = new Vector2D(0, 2), Angle = 0.3f });
world.CreateBox(box, new Vector2F(1, 1), def: new ShapeDef { Density = 2f, Restitution = 0.1f, Layer = 2 });
world.IgnoreLayerCollision(2, 3);

world.SetContactFilter(new NotTheSameBody());

for (int i = 0; i < 60; i++)
{
    world.Step(1f / 60);
    foreach (var e in world.GetEvents(EventKind.ContactBegin))
    {
        Console.WriteLine($"{world.GetBody(e.BodyA)} touched {world.GetBody(e.BodyB)}");
    }
}

var query = new PhysicsQuery2D(world) { Filter = QueryFilter2D.OnLayer(1) };
var hits = new QueryHit2D[16];
int n = query.RaycastAll(new Vector2D(-5, -0.25), new Vector2F(1, 0), hits);
```

The contact filter is not saved in snapshots: install it again after `Restore`.

Joints follow the same pattern: the one-liner (`AddHinge`) or a definition (`RevoluteJointDef`) returns a typed view (`HingeJoint`) that converts implicitly to `Joint`. A member another kind of joint has does not compile on the view. See [names and conventions](api-names.md) for the table.

## 3. The raw tier

`RecursiveAI.OnePhysics.LowLevel`: struct-of-arrays body and constraint storage, the constraint coloring and scheduler, broadphase trees, narrowphase and manifold routines, snapshot readers and writers. It is `[Experimental("OPHYS0001")]`: using it needs `<NoWarn>$(NoWarn);OPHYS0001</NoWarn>`, and it may change between previews. Use it when you build your own pipeline; nothing on tiers 1 and 2 requires it. The behaviour it implements is written down in [../spec](../spec/).

## Namespaces

- `RecursiveAI.OnePhysics`: what game code names in both dimensions: `BodyType`, `EventKind`, `EventMask`, `PhysicsEvent`, `OptionChanges`, the raw handles (`BodyHandle`, `ShapeHandle`, `JointHandle`) and the exceptions.
- `RecursiveAI.OnePhysics.Physics2D` and `Physics3D`: the world, `Body`, `Shape`, `Joint`, `Character`, definitions, queries and options. Both use the same member names.
- `RecursiveAI.OnePhysics.Debug`: debug geometry (shapes, contacts, joints, islands, broadphase bounds) as plain data for you to draw.
- `RecursiveAI.OnePhysics.LowLevel`: the raw tier.
