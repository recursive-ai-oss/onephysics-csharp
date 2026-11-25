# Partial worlds: streaming what a client can see

A client in a networked game predicts the part of the world it can see, not all of it. The authority writes, for each client, a stream of messages about what lies inside that client's *interest region*; the client applies them to its own world and steps it between messages. `docs/spec/interest.md` is the byte layout.

<!-- example: Guide/Interest.cs -->
```csharp
var authority = new PhysicsWorld3D();
authority.AddBox(new Vector3F(200, 1, 200), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
for (int i = 0; i < 20; i++) authority.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(-10 + i, 0.5 + 1.001 * (i % 2), 0));

var clientWorld = new PhysicsWorld3D();                       // the client's own world; it may hold other bodies as well
InterestWriter writer = authority.CreateInterestWriter();     // one writer per client
InterestClient client = clientWorld.CreateInterestClient();

byte[] message = Array.Empty<byte>();
var region = new InterestRegion();
for (int step = 0; step < 120; step++)
{
    authority.Step(1f / 60f);
    region = new InterestRegion().AddBox(new Vector3D(-6 + step * 0.1, -5, -20), new Vector3D(6 + step * 0.1, 40, 20));
    message = writer.Write(region);                           // send these bytes to the client
    client.Apply(message);                                    // the client's world now holds what the region holds
    if (step % 10 == 0) clientWorld.Step(1f / 60f);           // predict: the next Apply puts the bodies back where the authority says
}

bool same = client.ComputeSharedHash() == authority.ComputeInterestHash(region);
```

`PhysicsWorld2D` has the same members with 2D boxes (`AddBox(Vector2D, Vector2D)`).

## What the client holds

- **Inside bodies** are real bodies in the client's world, with their shapes and joints, and the contacts they had among themselves with their solver state when they arrived. A dynamic or kinematic body is inside when its position is in a box of the region or when you named it with `AddBody`; a ragdoll is inside whole when one of its parts is. Static bodies are never inside because of a box.
- **Proxies** are the bodies outside that something inside touches or hangs from: a dynamic one arrives as a *kinematic* body (the client's solver cannot push it; it moves at the velocity it had and is put back on the authority's pose at every message), a static one stays static (the ground under a pile appears the first time the pile touches it). A joint between an inside body and a proxy is kept, so a chain that crosses the region's edge hangs from where it hangs. Turn proxies off with `new InterestOptions { Proxies = false }` to send inside bodies only.
- **Entering and leaving** is decided at every message from the world's state alone: a body that enters arrives with its shapes, joints and contacts; one that leaves, or is destroyed, is destroyed in the client. A dynamic body that changes between inside and proxy leaves and enters again.
- **The client's world may hold other bodies** (its own effects, the local player's props). The client touches only the bodies it created; do not destroy those yourself, call `client.Reset()`.

## The shared hash

`authority.ComputeInterestHash(region)` and `client.ComputeSharedHash()` fold the same record of every inside body in ascending body slot: slot, generation, type, position, rotation, linear and angular velocity. Proxies, joints, contacts and sleeping are not in it. After the client applies the message written for `region` (and before it steps), the two are equal. After it steps ahead, the client's hash is how far it ran from the authority; the next `Apply` puts every body it holds back to the authority's last state before applying the new message, so a prediction never leaks into the shared state.

## Messages

- The first message of a stream is a snapshot; the rest are deltas that name the previous message. Messages must reach the client in order, once. A client that missed one (`InterestStreamException` naming `baseSequence`) needs a new snapshot: `writer.Reset()`, then write again.
- A delta lists only what changed: bodies that entered or left, joints that appeared or went, and the state of each body whose pose, velocity or sleeping changed. A pile that has gone to sleep costs about 65 bytes a step whatever its size; a pile that jitters costs about 45 bytes per body (2D: 37).
- The region can be different at every message (a moving camera, a player's area).
- `world.SnapshotInterest(region)` and `world.RestoreInterest(bytes)` do the one-shot version: build the region's bodies inside another world that holds other things, and go on with deltas through the returned client. Where the region is isolated and at rest, the restored bodies continue bit for bit like the source; bodies that are still colliding continue deterministically (every client given the same stream agrees) but not necessarily bit for bit like the source, because the engine's pair and constraint order depends on the allocation history the receiving world does not have.
- Anything that a packet cannot carry (characters, vehicles, articulations, rods, active ragdolls; see [Moving bodies between worlds](transfer.md)) makes `Write` throw a `BodySetTransferException` naming it, when it would be sent. Keep such bodies out of the regions, or send them by other means.
- Bytes from the network are checked completely before the client's world is touched; a bad message throws `InterestStreamException` naming the field and byte offset and changes nothing.
