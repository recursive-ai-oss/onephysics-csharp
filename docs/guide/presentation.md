# A follower world for cosmetics

An authoritative world is small on purpose: it is hashed, snapshotted, rolled back and sent over a network. Debris, character cloth, hair, ropes, particles and secondary ragdoll motion belong in a second world that follows it. `PresentationWorld3D` (and `PresentationWorld2D`) is that world: it has the same kernels as any other world, it is fed by the authority one way, and it is never part of the authority's snapshot, hash or network traffic.

<!-- example: Guide/Presentation.cs -->
```csharp
var follower = new PresentationWorld3D(new WorldOptions3D { WorldId = 2 });
follower.Follow(floor);                                      // a copy of the floor: static
Body capeMirror = follower.Follow(character);                // a kinematic copy that is moved to the character's pose every step
Body ropeAnchor = follower.Follow(platform);
```

`Follow` copies the shapes (sensors excluded) into the follower and returns the mirror, a normal `Body` of `follower.World`. Everything cosmetic goes into that world with the normal calls and attaches to the mirrors: a rod or cloth attached to a mirror follows the authoritative body and, because a mirror is a kinematic body, cosmetic bodies collide with it.

<!-- example: Guide/Presentation.cs -->
```csharp
follower.World.AddRope(new Vector3D(0, 1, -3), new Vector3D(0, 0, -3), 10, pinStart: false).Attach(0, ropeAnchor);
follower.Quality = PresentationQuality.Medium;                // budgets: bodies, particles, cloth and rod effort
```

Once per authoritative step, after it:

<!-- example: Guide/Presentation.cs -->
```csharp
authority.Step(dt);
// reads the authority's poses and fracture events, then steps the follower
follower.Step(authority, dt);
```

`Step(authority, dt)` is `Observe(authority)` followed by `Step(dt)`. The authority is only read: its bytes and its hash are the same with or without a follower, and a follower can be added or dropped at any time.

## Events become cosmetics

A fracture in the authority removes the source's mirror, follows the authoritative fragments, and throws a burst of small shards from the impact point (`follower.Debris`: count, size, speed, density, seed; the shards are the same on every platform). Set `follower.FractureMapper` to do something else. Other events go through `EventMapper` and `MappedEvents`, for instance a hard contact that spawns sparks: the handler receives the authority and the event, and spawns with `follower.World` or `follower.SpawnShards`.

## Budgets

`PresentationQuality` (`Low`, `Medium`, `High`, `Unbounded`, or your own) sets the most cosmetic dynamic bodies and particles, the soft-body schedule, and caps on rod substeps and iterations. Set it between steps. When a budget is lowered, rigid bodies go lightest first, then oldest; particles go oldest first. Raising a budget never changes what is there. `Pin(body)` keeps a body out of the budget (a cosmetic ragdoll you manage yourself).

## When the authority rolls back

Keep calling `Step(authority, dt)` with whatever the authority now says. A mirror that would have to move faster than `TeleportSpeed` (30 m/s) or turn faster than `TeleportSpin` (30 rad/s) in one step is placed with zero velocity instead, so the cosmetics resting on it are not thrown; whatever it now overlaps is pushed out at the world's depenetration speed. After every step no dynamic body is faster than `MaxSpeed` (30 m/s by default). The rollback tests go back 120 steps and never exceed that bound.

The follower steps with callbacks suppressed so that its recording replays exactly; read `follower.World.Events` after the step.

## Recording it

<!-- example: Guide/Presentation.cs -->
```csharp
using var file = new MemoryStream();
using (follower.StartRecording(file))
{
    for (int i = 0; i < 60; i++)
    {
        authority.Step(dt);
        follower.Step(authority, dt);
    }
}

file.Position = 0;
var player = new PresentationPlayer(file);
PresentationWorld3D replayed = player.CreateFollower3D();
while (player.ReadNextStep()) { }                            // checks the follower's hash after every step; the authority is not needed
```

The recording holds the poses and step lengths the follower consumed, plus a full snapshot whenever something else changed it between steps (a spawn, a cull, a budget change, a body you added). It is a small file next to a replay of the authority, can be opened and replayed on any platform, and the format is in [the spec](../spec/presentation.md). The follower is not saved into the authority's snapshot and has no network form.
