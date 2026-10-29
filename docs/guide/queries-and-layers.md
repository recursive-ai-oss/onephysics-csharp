# Queries, layers and filters

## Layers

Every shape has a layer, 0 to 31 (`ShapeDef.Layer`; `body.Layer` reads the first shape's layer and sets all of them, as Unity's per-object layer does). The world has a 32 x 32 matrix saying which layers collide; by default all do.

<!-- example: Guide/Queries.cs -->
```csharp
const int Ground = 1, Player = 2, Enemy = 3;
var world = new PhysicsWorld2D();
var ground = world.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
var player = world.AddCircle(0.5f, at: new Vector2D(-2, 1));
var enemy = world.AddCircle(0.5f, at: new Vector2D(2, 1));
ground.Layer = Ground;
player.Layer = Player;
enemy.Layer = Enemy;
world.IgnoreLayerCollision(Player, Enemy);               // layers 2 and 3 pass through each other
bool collide = world.GetLayerCollision(Player, Ground);
```

## Deciding whether two shapes collide

The first rule that answers decides (the full text is in [../spec/world2d.md](../spec/world2d.md)):

1. Two sensors never collide.
2. The layer matrix.
3. The group index: shapes with the same non-zero `GroupIndex` collide only if it is positive.
4. Category and mask bits: `(maskA & categoryB) != 0` and `(categoryA & maskB) != 0`.
5. The optional `WorldOptions2D.PairFilter` delegate (happy path) and then the optional struct `IContactFilter` from `SetContactFilter` (middle tier), called in a fixed order.

Changing a layer, mask, group or the matrix, or installing a filter, re-evaluates the existing pairs at the next step: contacts that are no longer allowed end (with an event). Call `RefilterPairs()` when your delegate's answer changes without any of those. A filter must depend only on the two shapes' state, never on the clock or on other mutable data, or two peers will disagree.

## Queries

Queries read the world; they never change it, so querying or not does not change the world hash. Each kind has a nearest-hit form, an `...Any` form (stops at the first hit, cheapest), and an `...All` form that fills your span or returns an array.

| Query | Methods |
|---|---|
| Ray | `Raycast`, `RaycastAny`, `RaycastAll` |
| Swept shape | `CircleCast`, `BoxCast`, `CapsuleCast`, `ShapeCast` (each with `Any`/`All`) |
| Overlap | `OverlapPoint`, `OverlapCircle`, `OverlapBox`, `OverlapShape` (each with `Any`/`All`) |
| Nearest point | `ClosestPoint` (with `Any`/`All`) |

Every query takes an optional `QueryFilter2D`: `LayerMask`, `CategoryBits`/`MaskBits`, `IncludeSensors` (default false) and a `Predicate` (`Func<Shape, bool>`). `QueryFilter2D.OnLayer(n)` and `QueryFilter2D.Layers(mask)` cover the common cases.

<!-- example: Guide/Queries.cs -->
```csharp
var enemiesOnly = QueryFilter2D.OnLayer(Enemy);
var notThePlayer = new QueryFilter2D { Predicate = s => s.Body != player };
var around = world.OverlapCircleAll(new Vector2D(2, 1), 1.5f, enemiesOnly);
bool cast = world.CircleCast(new Vector2D(-5, 1), 0.25f, new Vector2F(1, 0), out var hit, 100f, notThePlayer);
if (cast) Console.WriteLine($"first thing in the way: {hit.Body} after {hit.Distance} m");
```

A `QueryHit2D` has `Body`, `Shape`, `Point`, `Normal`, `Fraction` along the cast and `Distance`. Positions are `double`, directions and distances `float`. Pass a span or array to the `...All` forms with a `Span` overload and a steady-state query allocates nothing; the array-returning forms allocate the result.

For many queries at once, `PhysicsQuery2D` (see [the tiers](tiers.md)) has batch forms (`RaycastBatch`, `CircleCastBatch`, `OverlapCircleCountBatch`) that run in parallel on the world's workers with results in input order. Do not query while the world is stepping.
