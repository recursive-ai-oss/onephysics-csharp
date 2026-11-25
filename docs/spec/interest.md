# Interest streams (partial worlds)

Normative description of interest messages, of what `InterestWriter.Write` puts in them and of what `InterestClient.Apply` does, in both worlds. A port can reproduce the bytes and the client's state from this page alone. Task-oriented introduction: `docs/guide/interest.md`. Bodies enter a client as body-set packets (`transfer.md`); this page adds the selection of what enters, the delta, and the hash.

## Roles

For a region (boxes and named bodies) and a world, every live body has a role, computed from the world's state alone:

1. **Inside** seeds: a dynamic or kinematic body whose stored position (the centre of mass, relative to the origin, plus the origin: absolute, double) lies in at least one box (all lanes inclusive); every named live body whose handle carries this world's id (any type).
2. Closure: a ragdoll with a part inside is inside whole (`ExpandSet` of the transfer).
3. **Proxy** (only with `Proxies`, default): a live body that is not inside and either shares a joint with, or has a contact constraint with a shape of, a body that is inside **and dynamic**; then closed over ragdolls. A body inside that is kinematic or static makes no proxies (nothing there could be pushed).
4. **Skipped.** With the inside and proxy roles known, every present body that a packet cannot carry (`transfer.md`: a body that belongs to an articulation, a rod or cable attachment, a bicycle, a rail vehicle, an actuator, a breakable record, a streamed tile, a local time scale or per-triangle materials; a character that holds a rope; a character seated on, hanging from or climbing a body that is not present) is taken out of the roles, repeatedly, because taking one out can strand another (a ragdoll goes whole or not at all; a character whose seat was taken out is taken out). A skipped body is absent: it is never sent, it is not part of the shared hash, and nothing here throws. `InterestWriter.LastSkipped` lists them after each `Write`, in ascending body slot, each with whether it would have been inside or a proxy and the message of the refusal an extraction would have thrown. The same roles, skips included, decide `ComputeSharedHash` and the writer's hash (with the writer's `Proxies` option).
5. Every other body is absent.

**Entities.** Characters and vehicles of inside bodies are carried (`transfer.md`, Entities): their records travel in the entrants' packet, so a client creates the character or vehicle when the body enters, and destroys it with the body. A proxy is a plain kinematic body: no entity record. A kinematic body that carries a character changes role in place for any other kind of body, but not for these: its record only travels with an inside body, so a character that changes between inside and proxy **leaves and enters again** like a dynamic body. The record is sent once, at entry; deltas carry the body states only, so a client that steps its world ahead does not drive a vehicle or a character itself (it holds the inputs and ground state of the entry message) until the next `Apply` puts the bodies back on the authority's state. A host that wants a client-side vehicle to keep driving sets its inputs in the client's world between messages.

A joint is **held** by the client when both bodies are present, at least one of them is inside and dynamic, and its kind can be written in a packet (pulley and mouse joints cannot). Contacts between two present bodies are carried for the entering bodies when at least one of them is inside and dynamic.

## Message

All integers little-endian, floats IEEE-754 binary32/64 bits.

```
u32 magic 0x4E49504F ("OPIN")   u32 version 1
i32 dimension (2 or 3)          f64 metresPerUnit
u8  kind (0 snapshot, 1 delta)  i64 sequence   i64 baseSequence (-1 for a snapshot, else sequence - 1)
i32 leaveCount;       leaveCount x { i32 slot, i32 generation }                  ascending slot
i32 entrantCount;     entrantCount x { i32 slot, i32 generation, u8 role }       ascending slot; role 1 inside, 2 proxy
i32 roleChangeCount;  roleChangeCount x { i32 slot, i32 generation, u8 role }    ascending slot
i32 packetLength;     packetLength bytes: a body-set packet (transfer.md)
i32 jointRemoveCount; jointRemoveCount x { i32 slot, i32 generation }            ascending (slot, generation)
i32 jointAddCount;    jointAddCount x { i32 slot, i32 generation, i32 bodyASlot, i32 bodyAGeneration, i32 bodyBSlot, i32 bodyBGeneration,
                                         u8 flags (bit 0 report-persist, bit 2 has-lifetime), u8 color, i32 blobLength, blob, f32 payload[payloadFloats] }   ascending (slot, generation)
i32 stateCount;       stateCount x { i32 slot, i32 generation, state }           ascending slot
```

Slots and generations are those of the authority's world. `payloadFloats` is 104 (3D) or 40 (2D); a joint blob is the joint record of the transfer packet (`transfer.md`).

**state** (`8 P + 4 (R + L + A) + 1` bytes; 3D 65, 2D 37): `f64 position[P]` (absolute: origin plus stored position), `f32 rotation[R]`, `f32 linear[L]`, `f32 angular[A]`, `u8 asleep` (1 for a dynamic body that is not awake). `P` = 3 or 2, `R` = 4 or 2, `L` = 3 or 2, `A` = 3 or 1.

## Writing

`Write(region)` computes the roles, then, against what the previous message left (the writer keeps each body's role, generation and last state, and the set of held joints with their bodies' slots):

1. **Leaves:** bodies present before whose slot is now dead, reused (generation changed), absent, or **re-entering**: a body whose role changed that is dynamic (it is dynamic inside and kinematic as a proxy, which cannot change in place) or carries a character or vehicle. Ascending slot.
2. **Entrants:** present bodies that were not present before, or are leaving. **Role changes:** present before, role changed, not dynamic (a static or kinematic body changes role in place).
3. **Packet:** the entrants extracted as in `ExtractBodies` with `Move` off and `Boundary = Break`, with these changes: a proxy body is written static (if it was static) or kinematic, awake, without controls or sleep settings; a joint is written only when held; a contact only when carried (above); ragdolls, characters and vehicles only for inside bodies. No entrants: length 0. The packet holds exactly the entrants, in ascending slot, and no anchors.
4. **Joints:** let `held` be the writer's joints whose two bodies are not leaving. `jointRemoves` = held joints not in the new held set (ascending). `jointAdds` = joints of the new held set not in `held` and not travelling in the packet (both bodies entering). The writer's joint set becomes the new held set.
5. **States:** every present body that is not an entrant and whose state differs byte for byte from the last state written (or was never written), ascending slot. Entrants' last state is set from the live bodies (the packet carries them).
6. Snapshot (kind 0, sequence 0) for the first message or after `Reset`; then sequence n has baseSequence n-1.

## Applying

`Apply` reads and checks the whole message, and checks it against the client (below), before changing anything. Then:

1. A snapshot destroys every body the client created and forgets its joints. A delta first **reverts**: every body held whose live state (position, rotation, velocities; not the sleeping byte) differs from the last state received is written back to it, then sleeping is fixed (bodies the authority has asleep are slept, then bodies it has awake are woken).
2. The leaving bodies are taken out of the client's table.
3. The packet is inserted as `InsertBodies` does with the identity transform (lowest free slots, packet order); the client's table gets, for each entrant, its role, the new local handle and the state of the live body.
4. The leaving bodies' local bodies are destroyed (joints and contacts with them go with them).
5. Joint removes destroy the joints (a joint already gone is skipped). Joint adds create the joint between the two local bodies through the engine's joint creation, with the colour as a hint and the payload copied.
6. Role changes are recorded. States are written (pose, velocities; broadphase proxies moved) and recorded as the last received; then sleeping is fixed for the written bodies.
7. The client's sequence becomes the message's.

**Checks.** Bad magic, version, dimension or length unit (compared by bits); a count that needs more bytes than remain; a negative slot or non-positive generation; slots not ascending; a role other than 1 or 2; a packet that is not valid, does not have exactly the entrants in order, or has anchors; a joint blob that is not a valid joint record; a payload float, state float or position that is not finite; a position beyond ±2^21; a rotation of length² differing from 1 by more than 0.01; a speed above 1e6; an asleep byte above 1; trailing bytes — all `InterestStreamException` naming the field (`states[3].rotation`) and the byte offset. Against the client: a snapshot has sequence 0; a delta's base must equal the client's last sequence; leaves, role changes and states must name held bodies with the same generation; entrants must not be held; a joint remove must name a held joint; a joint add's bodies must be held (or entering) and different, and the joint not already held; a state record must not name an entrant.

## Shared-entity hash

64-bit, the engine's hash fold (`StateWriter` in hashing mode, `architecture.md`) of: `i32 dimension`, `i32 count` of inside bodies, then per inside body in ascending slot `i32 slot`, `i32 generation`, `u8 type`, and its state record without the final sleeping byte. The authority computes it for a region from the seeds and closure above (proxies are not part of it); the client computes it over the bodies it holds as inside, from their live state, so after applying a message and before stepping it equals the authority's for that message's region at that moment. Absolute positions are used, so worlds with different origins agree.

## Regional restore without matching topology

`SnapshotInterest(region)` is a snapshot message; `RestoreInterest(bytes)` applies it with a fresh client in a world of any other content. Bodies take the lowest free slots, so ids differ from the source's; the client's table maps them. Where the region is isolated (no contact with anything outside it that is not itself in the message) and at rest, stepping the receiving world continues the region bit for bit like the source (tested: a settled pile at 400, 700 and 1200 steps, beside unrelated bodies). Bodies still colliding when restored continue deterministically but not necessarily bit for bit like the source: the engine's contact and pair order depends on the allocation history of the world (pair-table and constraint slots), which a transfer does not carry. The existing region archive (`snapshots.md`) remains the value resync for worlds of identical topology.

## Limits

A message is read with a total size limit (default 256 MiB); counts are checked against the bytes that remain before anything is allocated; the packet inside is read with its own checks (`transfer.md`). Slots and joint ids are those of the authority world and need not be dense.
