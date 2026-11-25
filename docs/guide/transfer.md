# Moving bodies between worlds

A set of bodies can leave one world as a packet of bytes and arrive in another: a crate carried through a portal, a vehicle handed from one region's world to the next, a player's props moved to the server that now owns them. The packet carries the bodies' shapes, the joints among them, the contacts among them with their solver state (so a resting stack does not pop), the ragdolls they belong to, and what a joint that crosses the boundary needs. `docs/spec/transfer.md` is the byte layout.

<!-- example: Guide/Transfer.cs -->
```csharp
var level = new PhysicsWorld3D();
level.AddBox(new Vector3F(60, 1, 60), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
var crate = level.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0.6, 0));
level.AddRagdoll(RagdollDef.Humanoid, new Vector3D(6, 1.2, 0));
for (int i = 0; i < 60; i++) level.Step(1f / 60f);

BodySetPacket packet = level.ExtractBodies(new[] { crate });             // moves the crate out of `level`
byte[] wire = packet.ToArray();                                          // send these bytes to the other world

var other = new PhysicsWorld3D();
other.AddBox(new Vector3F(60, 1, 60), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);   // the same ground
BodySetHandleMap map = other.InsertBodies(BodySetPacket.Read(wire));
Body arrived = other.GetBody(map.NewBodies[0]);

// A portal: the body comes out turned a quarter about y, a hundred metres away, with its velocity turned too.
var portalWorld = new PhysicsWorld3D();
var ball = portalWorld.AddSphere(0.5f, at: new Vector3D(0, 2, 0));
ball.LinearVelocity = new Vector3F(3, 0, 0);
BodySetPacket thrown = portalWorld.ExtractBodies(new[] { ball }, new BodySetOptions { Move = false });
var far = new PhysicsWorld3D();
Body through = far.GetBody(far.InsertBodies(thrown, at: new Vector3D(100, 0, 0), rotation: QuaternionF.CreateFromAxisAngle(new Vector3F(0, 1, 0), 1.5707964f)).NewBodies[0]);
```

`PhysicsWorld2D` has the same two methods; its transform is `InsertBodies(packet, at, angle)`.

## What goes in the packet

- **The set** is the bodies you name, plus every part of a ragdoll one of them belongs to. A packet holds rigid bodies, their shapes (every kind except voxel grids, which are edited in place), their joints (every kind except pulley and mouse joints, which are anchored in world coordinates; in 2D also path and configurable joints), ragdolls, and the contact constraints among them.
- **Contacts with something outside the set** are carried when the other shape is a static shape that the receiving world also holds (same geometry, same pose): the crate keeps its resting contact with the floor. Anything else outside is found again by the receiving world's first step.
- **Sensor overlaps are not carried**: the receiving world finds them again and reports an enter event in its first step.
- **Not carried, and refused with a message that names it:** characters, vehicles, bicycles, rail vehicles, articulations, rods and cables attached to a body, thrusters, boats and other actuators, breakable and debris records, streamed tiles, cloth attachments, active ragdolls, local time scales and per-triangle contact materials. Destroy the feature, or leave its bodies out.
- **A joint that crosses the boundary** (one body in the set, the other not) is refused until you choose a policy: `BoundaryPolicy.Break` leaves the joint behind, `BoundaryPolicy.HoldAsAnchor` carries the outside body along as a shapeless anchor that keeps its pose and velocity (static stays static, anything else becomes kinematic), so the set hangs where it hung.
- **Move or copy.** `ExtractBodies` moves by default; `new BodySetOptions { Move = false }` copies and leaves the source untouched.

## What you get back

`InsertBodies` returns a `BodySetHandleMap`: the old and new handle of every body, shape, joint and ragdoll, in the order of the packet, and the anchors. New ids are the lowest free slots in packet order (bodies by their old slot, then anchors; shapes and joints by their old slot), so two worlds in the same state that insert the same packet are in the same state afterwards, hash for hash. The packet is validated completely when you read it (`BodySetPacket.Read` throws `BodySetPacketException` naming the field) and again against the receiving world when you insert it (`BodySetTransferException`); a failed insertion leaves nothing behind.

A packet of the same set from two peers is byte-identical, so it can be signed, compared or sent as it is. Both worlds must use the same `LengthUnit`; packets are not rescaled.

## Portals

`InsertBodies(packet, at, rotation)` places the set at `rotation * p + at`. Velocities, spins, orientations and the contact normals turn with it, so a body keeps its speed and spin relative to the portal. Impulses that depend on the contact's tangent basis start again from zero when the rotation is not the identity. With no rotation and no translation the set arrives where it was, bit for bit.

## How exact it is

Into a world that holds the same static ground, a resting stack and a lying ragdoll go on exactly as in the source world: every pose over the next 120 steps is bit-identical, because the packet carries the contact warm start, the joint state and the order the solver visits constraints in. A world that holds something else where the source had its ground can only find the contacts again: expect a few millimetres of settling, not a pop.

Sleeping bodies arrive asleep when every body they were connected to travels with them. Contact events for the carried contacts are reported like those of any new contact, in the first step events are delivered. Recording a world records the insertion; a replay plays it back.
