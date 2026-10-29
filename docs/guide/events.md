# Events

A step produces events: contacts beginning and ending, sensor overlaps, sleeping and waking, joint breaks. They are **collected during the step and delivered after it**, in a fixed order that does not depend on threads or platform. You can consume them two ways.

## Per-body handlers

Subscribe on the body. Handlers run at the end of `Step`, after the simulation has finished.

<!-- example: Guide/Events.cs -->
```csharp
var ball = world.AddCircle(0.5f, at: new Vector2D(0, 3));
ball.OnCollisionEnter += c => enters++;
ball.OnCollisionStay += c => stays++;
ball.OnCollisionHit += c => { hitCount++; Console.WriteLine($"landed at {c.ApproachSpeed} m/s"); };
```

`Body` has `OnCollisionEnter`, `OnCollisionStay`, `OnCollisionExit`, `OnCollisionHit`, `OnTriggerEnter`, `OnTriggerExit`, `OnSleep` and `OnWake`. The `Collision` argument has `Body`, `Other`, `Shape`, `OtherShape`, `Normal`, `Point`/`GetPoint(i)`, `NormalImpulse` and `ApproachSpeed`. Sensors report through `OnTriggerEnter`/`OnTriggerExit` with a `Trigger`:

<!-- example: Guide/Events.cs -->
```csharp
var zone = world.CreateBody(new BodyDef { Type = BodyType.Static, Position = new Vector2D(0, 1.5) });
world.CreateBox(zone, new Vector2F(4, 1), def: new ShapeDef { IsSensor = true });
zone.OnTriggerEnter += t => { triggers++; Console.WriteLine($"{t.Other} entered the zone"); };
```

## Polled events

`world.Events` is every event of the last step; `world.GetEvents(kind)` is the slice of one kind. Both are spans, valid until the next `Step`, and allocate nothing.

<!-- example: Guide/Events.cs -->
```csharp
world.Step(1f / 60);
polled += world.GetEvents(EventKind.ContactBegin).Length;
foreach (var e in world.Events)
{
    if (e.Kind == EventKind.Sleep) Console.WriteLine($"{world.GetBody(e.BodyA)} fell asleep");
}
```

A `PhysicsEvent` carries raw handles (`BodyA`, `BodyB`, `ShapeA`, `ShapeB`, `Joint`) and a `Value` (the break value of a joint, the approach speed of a hit). Turn a raw handle into a live `Body` with `world.GetBody(handle)`; `world.IsBodyAlive(handle)` tells you whether it still exists without throwing. Prefer polling when you roll back (below) or want events in your own order.

## Which events exist: `EventMask`

`WorldOptions2D.Events` chooses what the world records; a disabled kind costs nothing.

| Kind | In `EventMask.Default` | Notes |
|---|---|---|
| `ContactBegin`, `ContactEnd` | yes | |
| `SensorEnter`, `SensorExit` | yes | |
| `JointBreak` | yes | |
| `Sleep`, `Wake` | yes | |
| `ContactPersist` | **no** | one event per touching pair per step; `OnCollisionStay` needs it |
| `ContactHit` | **no** | a contact that approached faster than the world's hit threshold; `OnCollisionHit` needs it |

<!-- example: Guide/Events.cs -->
```csharp
var world = new PhysicsWorld2D(new WorldOptions2D
{
    Events = EventMask.Default | EventMask.ContactPersist | EventMask.ContactHit,
});
```

## Order

Events are sorted after the step on every field: first by kind (`ContactBegin`, `ContactPersist`, `ContactEnd`, `SensorEnter`, `SensorExit`, `JointBreak`, `Sleep`, `Wake`, `ContactHit`), then by the bodies' and shapes' slots, so the same inputs give the same list on every platform and thread count. Handlers run in that same order, first for body A of a pair and then body B. The precise sort keys are in [../spec/world2d.md](../spec/world2d.md).

## Rollback

After `Restore`, handlers stay attached to their handles, and resimulating would call them a second time. Wrap resimulation in `world.SuppressCallbacks()`; polled events are still produced. See [rollback](rollback.md).
