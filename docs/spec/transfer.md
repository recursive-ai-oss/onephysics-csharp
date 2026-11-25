# Body-set transfer

Normative description of body-set packets and of what `ExtractBodies` and `InsertBodies` do, in both worlds. A port can reproduce the bytes and the receiving world's state from this page alone. Task-oriented introduction: `docs/guide/transfer.md`.

A **packet** holds a set of bodies taken out of a world: their shapes, the joints among them, the contact constraints among them (with the solver's warm-start state), the contacts they have with static shapes outside the set, and the ragdolls they belong to. Packets are not snapshots: `Restore` rejects them by their magic.

## Extraction

`ExtractBodies(bodies, options)` (options: `Move` default true, `Boundary` default `Refuse`):

1. Every handle is resolved (stale, foreign and null handles throw the usual handle errors); a body listed twice is an `ArgumentException`; an empty list is an `ArgumentException`.
2. **Closure.** Every part of a ragdoll one of whose parts is in the set is added. Nothing else is added: joints, contacts and attachments do not pull bodies in.
3. **Refusals** (`BodySetTransferException`, naming the feature and its slot; the world is not changed). A body of the set belongs to a character, vehicle, bicycle, rail vehicle or rail track, articulation, rod or cable attachment, thruster, boat, hovercraft or aerodynamic surface, breakable or debris record, cloth attachment (3D), or a streamed tile; has a local time scale; a shape has per-triangle (3D) or per-segment (2D) contact materials, or is a voxel grid; a joint among the set is a pulley or mouse joint, or (2D) a path or configurable joint; the ragdoll is active, or has lost a part's joint.
4. **Boundary.** A joint with exactly one end in the set: `Refuse` throws (the message names the joint and the outside body); `Break` ignores the joint; `HoldAsAnchor` carries it and adds the outside body as an *anchor*, one anchor per outside body.
5. The packet is built (below). With `Move` the set's bodies are destroyed afterwards, highest slot first, which also destroys their shapes, joints, contacts and ragdolls; without it the source world is not touched (`FlushPostStep` aside).

**Packet order.** Bodies by ascending slot, then anchors by ascending slot of the body they stand for; shapes by ascending slot; joints by ascending constraint slot; ragdolls by ascending slot. Contacts: pairs of two shapes of the set by ascending (shape A, shape B), each pair's constraints in the pair's chain order (ascending sub-shape ids); support contacts by member shape, then outside shape, then chain order. A pair starts where `newPair` is 1.

**Body record.** Member bodies hold their state as the world holds it: `position` is the centre of mass in world coordinates (the world origin plus the stored position, exact in double), rotation, velocities, the engine's per-body floats and flags (damping, gravity scale, last displacement, force and torque accumulators, fixed rotation), sleep state and timer, body controls (axis locks, speed limits, mass, centre and inertia overrides, kinematic target) and sleep settings. An **anchor** is shapeless, static when the outside body is static and kinematic otherwise; its `position` is the outside body's *origin* (not its centre of mass), its velocity that of the origin, and it carries no other state, so the joint anchors, which are given in the body's origin frame, land where they were.

**Colour.** Each joint and contact records the solve colour it had (0..23, 254, or 255 for the serial overflow set). Insertion asks for the same colour and takes it when neither body of the constraint already uses it; otherwise the first free one, as for any new constraint. With the same colours the receiving solver visits the set's constraints in the order the source did.

**Support contacts.** A contact between a member shape and a shape of a *static* body outside the set is recorded with that outside shape's geometry and pose (a *support*). See Insertion.

## Insertion

`InsertBodies(packet, transform)`; the transform is `rotation * p + at` (3D: a quaternion and a vector; 2D: an angle in radians and a vector); rotation is about the world origin.

1. The packet's dimension and length unit must equal the world's (`BodySetTransferException` otherwise; units are compared as doubles by bits). Nothing is rescaled.
2. Everything that can be decided without changing the world is decided first: each body's new position (`rotation * (origin + p) + at - origin`, in double, rounded to the 2^-32 grid unless the transform is the identity, in which case the position is copied as it is) must lie within ±2^21 on every axis; a kinematic target cannot be rotated; each support is looked up (below).
3. **Ids.** Bodies are created in packet order, each in the lowest free body slot, awake, with its velocity; then shapes in packet order, each in the lowest free shape slot and appended to its body's shape list; then the bodies are finished (overrides, accumulators, sleep timer, the centre of mass put back at the packet's position, broadphase proxies moved to it); then joints in packet order in the lowest free constraint slots; then ragdolls in the lowest free ragdoll slots; then the contacts. The old-to-new map lists bodies, shapes, joints and ragdolls in packet order.
4. **Transform.** Positions, kinematic-target positions and support positions go through `R p + at`. Linear velocity, angular velocity (3D), force and displacement accumulators go through `R v`; orientations through `R q` (3D: quaternion product, renormalised; 2D: the (cosine, sine) pair turned by the angle, renormalised); torque (3D) as a vector, 2D torque and angular velocity are unchanged. A transform with an identity rotation copies these values untouched. Contact payloads: the normal and the anchors (arms) are rotated; in 3D the rolling impulse and the tangent impulses are set to zero (their basis follows from the normal and is not rotation-covariant), in 2D they are kept (the tangent is the normal turned a quarter). Body-local manifold-reuse data is kept.
5. **Joints** are created through the engine's own joint creation, so the same validation applies; the joint's payload (its accumulated impulses) is copied over, and the remaining lifetime is restored through the lifetime slots. A ragdoll's part joints keep their drive state.
6. **Ragdolls** are rebuilt from the record (definition, part to body and joint maps, powered flag and strength). Shapes whose collision group is the old ragdoll's (-(slot + 1), for the self-collision mode `None`) get the new ragdoll's.
7. **Contacts among the set.** For each pair in packet order the pair table gets an entry for the two new shapes unless the receiving world's filters (layer matrix, category and mask, groups, pair predicate, contact filter, joints that forbid collision) do not let them collide, in which case the pair's contacts are dropped. Each constraint is added through the core's constraint add (so islands, edges and colours are those of any new contact), with the recorded sub-shape ids, flags, point count, feature ids and payload.
8. **Support contacts.** A support is matched to the lowest-slot live shape that lies on a static body of the receiving world and has the same geometry (3D: equal recipe bytes and local pose; 2D: equal shape record), and whose body has exactly the transformed position and rotation (bit equality). An unmatched support is dropped with its contacts. For a matched one the pair is created like any other, in shape-slot order; when the receiving order differs from the source's, the payload is swapped: normal negated, arms exchanged, feature id halves exchanged, sub-shape ids exchanged, tangent and rolling impulses and the manifold-reuse data cleared.
9. **Sleep.** The dynamic bodies are grouped by the packet's joints and contacts (static and kinematic bodies join nothing); a group of which every body was asleep is put to sleep, one `Sleep` call per group, in packet order.
10. **Events.** Constraints are added through the core, so each contact queues a begin event as any new contact does; sleep calls queue sleep events. Nothing is dispatched until the next step.
11. **Failure.** If anything throws after the world has been changed, the bodies created so far are destroyed (highest first) and the exception propagates; slots and generations used are not given back.

Recording: both calls are recorded mutations (`ExtractBodies` only when it moves); a replay plays an insertion back by its resulting state.

## Byte layout

All integers little-endian; floats IEEE-754 binary32/64; booleans one byte, 0 or 1. The bytes depend on the set and nothing else (no world id, no step count). Counts are checked against the bytes that remain before anything is allocated; every float and double is finite; a reader rejects unknown bits, dangling indices, out-of-order records and trailing bytes with a `BodySetPacketException` naming the field (`bodies[3].rotation`) and the byte offset.

```
u32  magic          0x5342504F  ("OPBS")
u32  version        1
i32  dimension      2 or 3
f64  metresPerUnit  1e-6 .. 1e6 (the length unit)
i32  positionLanes, rotationLanes, linearLanes, angularLanes, maxManifoldPoints, payloadFloats
                    3D: 3 4 3 3 4 104     2D: 2 2 2 1 2 40
i32  members, anchors, shapes, joints, contacts, supports, supportContacts, ragdolls   (each 0 .. 2^24)
body[members + anchors]
shape[shapes]  joint[joints]  contact[contacts]  support[supports]  supportContact[supportContacts]  ragdoll[ragdolls]
```

**body**: `i32 slot, i32 generation` (the source handle; both positive), `u8 type` (0 static, 1 kinematic, 2 dynamic; anchors 0 or 1), `u8 bits` (bit 0 bullet, bit 1 never sleeps), `u8 asleep` (dynamic only), `u8 engineFlags` (bit 0 fixed rotation), `f32 sleepTime` (not negative), `f64 position[positionLanes]` (|p| ≤ 2^23), `f32 rotation[rotationLanes]` (unit within 1e-3: 3D x y z w, 2D cosine sine), `f32 linear[linearLanes]`, `f32 angular[angularLanes]`, `f32 extra[3 + 2*linearLanes + angularLanes]` = linear damping, angular damping (both ≥ 0), gravity scale, displacement, force, torque; `u8 hasControls` then, if 1: `i32 constraints` (bits 0..5 axis locks), `f32 maxLinearSpeed, maxAngularSpeed, maxDepenetrationSpeed`, `u8 overrides` (bit 0 mass, 1 centre, 2 inertia, 3 kinematic target position and rotation, 4 kinematic drive), `f32 mass`, `f32 center[3]`, `f32 principal[3]`, `f32 inertiaRotation[4]`, `f64 targetPosition[3]` (world coordinates), `f32 targetRotation[4]` (2D packets keep the unused lanes zero); `u8 hasSleep` then, if 1: `f32 linearThreshold, angularThreshold, timeToSleep`.

**shape**: `i32 slot, i32 generation` (ascending slot), `i32 body` (index of a member), `i32 blobLength`, blob. *3D blob*: `i32 recipeLength`, recipe bytes (the geometry recipe of `collision3d.md`; voxel grids are refused), `f32 localRotation[4], localPosition[3]`, `f32 friction, restitution, rolling, density`, `u32 category, mask`, `i32 group`, `u8 flags` (bit 0 sensor), `u8 layer` (< 32), `u8 hasSurface` then 12 floats: velocity[3], oneWayDirection[3], frictionAxis[3], longitudinal, transverse, spinning. *2D blob*: the 2D shape record of `snapshots.md` (kind, count, radius, flags, centroid, two ghost points, eight vertex and normal pairs), then the same material, filter, flag, layer and surface fields (no local pose; it is baked into the geometry). Triangle meshes, height fields and planes only on static bodies.

**joint**: `i32 slot, i32 generation` (ascending), `i32 bodyA, bodyB` (indices into the bodies, different, at least one dynamic), `u8 flags` (bit 0 report-persist, bit 2 has-lifetime), `u8 color`, `f32 breakValue` (0), `i32 blobLength`, blob = the joint record of the snapshot (`world3d.md` / `world2d.md`) followed by `i32 remainingLifetimeSteps`; `f32 payload[payloadFloats]`.

**contact**: `i32 slot, i32 generation`, `i32 shapeA, shapeB` (indices into the shapes, A below B, owned by different bodies), `i32 subA, subB` (not negative), `u8 flags` (bit 0 report-persist), `u8 color`, `u8 points` (≤ maxManifoldPoints), `u8 newPair`, `u32 features[maxManifoldPoints]`, `f32 breakValue`, `f32 payload[payloadFloats]`. `newPair` is 1 exactly when the constraint's (shapeA, shapeB) differs from the previous record's; pairs ascend; within a pair (subA, subB) ascend.

**support**: `f64 position[positionLanes]`, `f32 rotation[rotationLanes]` of the outside static body, `i32 blobLength`, shape blob (as above).
**supportContact**: `i32 member` (shape index), `i32 support`, `u8 memberIsA`, `i32 subA, subB`, `u8 flags, color, points, newPair`, `u32 features[maxManifoldPoints]`, `f32 breakValue`, `f32 payload[payloadFloats]`; consecutive records of one pair share `memberIsA`.
**ragdoll**: `i32 slot, i32 generation` (ascending), `i32 blobLength`, blob = the ragdoll record of the snapshot (`world3d.md` / `world2d.md`) in which the body and joint slots are indices into the packet's bodies and joints (generation 1) and the generations of the part joints are 1; every part is a dynamic member body, every non-root part's joint is the engine's ragdoll joint (3D swing-twist, 2D revolute) between the part's parent and the part.

## Limits and validation

Up to 2^24 of each kind of record; a reader's total size limit defaults to 256 MiB. Positions within ±2^23 in a packet and ±2^21 on insertion; a body's mass, inertia, material and joint values are validated by the same code as a snapshot restore; a hostile recipe is built (and refused) when the packet is read. Reading allocates nothing before a count has been checked against the bytes that remain.
