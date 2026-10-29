# Rollback and lockstep

A rollback host runs the same fixed-dt simulation on every peer, keeps a ring of recent snapshots, and when a late input arrives restores an old frame and resimulates. The world is built for it: snapshots are byte-exact, restores are atomic, and a world hash per step lets peers verify they agree.

## The loop

<!-- example: Guide/Rollback.cs -->
```csharp
const float dt = 1f / 60;                               // one fixed dt for the whole session
var world = new PhysicsWorld2D(new WorldOptions2D
{
    WorldId = 1,                                        // handles stay valid across worlds built with the same id
    PairCapacity = 256, ContactCapacity = 256, IslandCapacity = 64,
});
world.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
var ball = world.AddCircle(0.5f, at: new Vector2D(0, 4));
int sounds = 0;
ball.OnCollisionEnter += c => sounds++;
world.Warmup();                                         // after the bodies exist: steps, snapshots and restores no longer allocate

const int Ring = 8;
var ring = new byte[Ring][];
var lengths = new int[Ring];
var hashes = new ulong[Ring];
for (int i = 0; i < Ring; i++) ring[i] = new byte[world.GetSnapshotSize() * 2];   // headroom for a world that grows

int frame = 0;
void Advance()
{
    int slot = frame % Ring;
    if (!world.TrySnapshot(ring[slot], out lengths[slot])) throw new InvalidOperationException("snapshot buffer too small");
    world.Step(dt);
    hashes[slot] = world.ComputeHash();                 // exchange these with peers to catch a desync
    frame++;
}
```

<!-- example: Guide/Rollback.cs -->
```csharp
int back = 5;                                           // a late input for frame 95 arrived
int target = frame - back;
OptionChanges changed = world.Restore(ring[target % Ring].AsSpan(0, lengths[target % Ring]));
frame = target;
using (world.SuppressCallbacks())                       // no sound twice; polled events still work
{
    for (int i = 0; i < back; i++) Advance();
}
```

Apply the late input to your game state (and set body velocities or call `Move` before the step) between `Restore` and the resimulation steps, as your game requires; the world itself only needs the same dt and the same inputs to produce the same bits.

## The pieces

- **`TrySnapshot(span, out written)`** writes into your buffer and returns false if it is too small; `GetSnapshotSize()` is the current size. `Snapshot()` allocates a new array. The same world writes the same bytes on every platform. Snapshots hold contact caches, warm-start impulses, sleep state and the events of the last step, which is why a restored world continues bit for bit.
- **`Restore(bytes)`** validates the whole snapshot first and changes nothing if it is invalid (`SnapshotFormatException`). It returns an **`OptionChanges`** flags value naming which saved options (gravity, substeps, iterations, sleep settings, event mask) the snapshot replaced in this world, or `None`. Options that are not part of the saved state (`ThreadCount`, capacities, solver tuning, the pair filter delegate and struct filter) are left alone: install a struct contact filter again after a restore.
- **`WorldId` and handle validity.** A handle carries the id of the world that made it. Handles taken before a snapshot stay valid in a world with the same `WorldId` that holds the same bodies, so you can keep `Body` values across a restore, or restore into a freshly built world (a new peer joining). With the default `WorldId` of 0 the world takes the next number from a process-wide counter, which depends on creation order; set it explicitly when you need stable ids.
- **Handlers across a restore.** Per-body handlers belong to the handle. A restore that brings a body back keeps its handlers. The world remembers the handlers of the 1024 most recently destroyed bodies, joints and characters of each kind, and only once it has taken a snapshot; anything older is released, so rolling back past more than 1024 destroys of one kind loses those handlers.
- **`SuppressCallbacks()`** returns a scope that stops per-body handlers firing while it is open; polled events (`Events`, `GetEvents`) are still produced. It allocates nothing and nests. Use it for the frames you replay, so sounds and particles are not triggered twice. Or avoid handlers in rollback code and read polled events.
- **`ComputeHash()`** is a 64-bit hash over the canonical state in a documented order (see [../spec/architecture.md](../spec/architecture.md)): equal worlds give equal hashes on every platform. It does not cover handlers, the events of the last step, or the snapshot header.
- **Zero allocation.** `Warmup()` sizes storage to `PairCapacity`, `ContactCapacity` and `IslandCapacity` (derived from the body count where they are 0), starts the worker threads and exercises the hash, snapshot and restore paths once, so steady-state steps, queries, `ComputeHash`, `TrySnapshot` and `Restore` allocate nothing while the world stays inside those counts. Call it after creating the bodies, from the thread that will query. The first step after creating many new bodies may still grow storage; raise the capacities if you create bodies mid-session.

## Lockstep without rollback

The same pieces work without a ring: step every peer with the same inputs and dt, exchange `ComputeHash()` every N steps, and on a mismatch send a snapshot from the authority to the peer and `Restore` it. A mismatch is a bug in your inputs or in the engine, never a platform difference (see [determinism](determinism.md)).
