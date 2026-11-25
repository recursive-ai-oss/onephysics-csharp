# Moving bodies between worlds

A set of bodies can leave one world as a packet of bytes and arrive in another: a crate carried through a portal, a vehicle handed from one region's world to the next, a player's props moved to the server that now owns them. The packet carries the bodies' shapes, the joints among them, the contacts among them with their solver state (so a resting stack does not pop), the ragdolls they belong to, the characters and vehicles that stand on them, and what a joint that crosses the boundary needs. `docs/spec/transfer.md` is the byte layout.

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

## Characters and vehicles

A character or vehicle travels whole with its body: definition, ground state, abilities (seated, climbing, swimming), inputs, drivetrain and wheels. Name the bodies together, a driver with her car; the handle map lists the new `Character` and `Vehicle` handles.

<!-- example: Guide/TransferEntities.cs -->
```csharp
var north = new PhysicsWorld3D();
north.AddBox(new Vector3F(400, 1, 400), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
Body chassis = north.AddBox(new Vector3F(1.8f, 0.6f, 4.2f), at: new Vector3D(0, 1.0, 0), density: 357f);
Vehicle car = north.AddVehicle(chassis, VehicleDef.Car);
Character driver = north.AddCharacter(new Vector3D(0, 2, 0));
driver.Mount(chassis, new Vector3F(-0.4f, 0.5f, 0.2f));
for (int i = 0; i < 90; i++) { car.Drive(throttle: 0.5f); north.Step(1f / 60f); }

// The car and its driver cross to the world that owns the next stretch of road.
BodySetPacket packet = north.ExtractBodies(new[] { chassis, driver.Body });
var south = new PhysicsWorld3D();
south.AddBox(new Vector3F(400, 1, 400), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);   // the same road
BodySetHandleMap map = south.InsertBodies(BodySetPacket.Read(packet.ToArray()));
Vehicle arrived = south.GetVehicle(map.NewVehicles[0]);
Character passenger = south.GetCharacter(map.NewCharacters[0]);
```

Into a world with the same road the car drives on exactly as it would have: its next 120 steps are bit-identical to the source's. Three things to know:

- A character seated on, hanging from or climbing a body must travel with that body (extraction refuses otherwise, naming it), and a character holding a rope cannot travel (a packet carries no rods).
- Ground the character or a wheel stood on that is not in the set is not carried: the character arrives in the air and a wheel out of contact until the next move or step finds the ground in the receiving world.
- A portal's rotation turns the car's wheel contacts, the character's ground normal and its velocity with it; a character's up direction turns too, unless the rotation is a turn about it, which leaves it bit for bit.

Bicycles are not carried yet.

## What goes in the packet

- **The set** is the bodies you name, plus every part of a ragdoll one of them belongs to. A packet holds rigid bodies, their characters and vehicles (below), their shapes (every kind except voxel grids, which are edited in place), their joints (every kind except pulley and mouse joints, which are anchored in world coordinates; in 2D also path and configurable joints), ragdolls, and the contact constraints among them.
- **Contacts with something outside the set** are carried when the other shape is a static shape that the receiving world also holds (same geometry, same pose): the crate keeps its resting contact with the floor. Anything else outside is found again by the receiving world's first step.
- **Sensor overlaps are not carried**: the receiving world finds them again and reports an enter event in its first step.
- **Not carried, and refused with a message that names it:** bicycles, rail vehicles, articulations, rods and cables attached to a body, thrusters, boats and other actuators, breakable and debris records, streamed tiles, cloth attachments, active ragdolls, local time scales and per-triangle contact materials. Destroy the feature, or leave its bodies out.
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
