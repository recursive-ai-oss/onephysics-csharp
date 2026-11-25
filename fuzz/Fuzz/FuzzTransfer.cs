using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// Body-set packets: whatever the reader accepts must write back to the same bytes, insert into a world of its dimension (or be refused with a typed error) and step.
/// </summary>
internal static class FuzzTransfer
{
    public static void Run(byte[] bytes)
    {
        BodySetPacket packet;
        try { packet = BodySetPacket.Read(bytes, 1 << 22); }
        catch (BodySetPacketException) { return; }
        if (!packet.AsSpan().SequenceEqual(bytes)) { throw new InvalidOperationException("an accepted packet does not write back to its bytes"); }
        try
        {
            if (packet.Dimension == 2)
            {
                var world = new PhysicsWorld2D(new WorldOptions2D { WorldId = 31 });
                world.AddBox(new(40, 1), at: new(0, -0.5), type: BodyType.Static);
                world.InsertBodies(packet, new Vector2D(1, 2), 0.3f);
                for (int i = 0; i < 3; i++) { world.Step(1f / 60f); }
            }
            else
            {
                var world = new PhysicsWorld3D(new WorldOptions3D { WorldId = 31 });
                world.AddBox(new(40, 1, 40), at: new(0, -0.5, 0), type: BodyType.Static);
                world.InsertBodies(packet, new Vector3D(1, 2, 0), QuaternionF.CreateFromAxisAngle(new Vector3F(0, 1, 0), 0.3f));
                for (int i = 0; i < 3; i++) { world.Step(1f / 60f); }
            }
        }
        catch (PhysicsException) { }
        catch (ArgumentException) { }
    }

    public static byte[] Seed(int dimension)
    {
        if (dimension == 2)
        {
            var world = new PhysicsWorld2D(new WorldOptions2D { WorldId = 30 });
            world.AddBox(new(40, 1), at: new(0, -0.5), type: BodyType.Static);
            var a = world.AddBox(new(1, 1), at: new(0, 0.5));
            var b = world.AddBox(new(1, 1), at: new(0.1, 1.5));
            world.AddRagdoll(RecursiveAI.OnePhysics.Physics2D.RagdollDef.Humanoid, new Vector2D(5, 1.2), 1.5f);
            world.CreateJoint(new RevoluteJointDef { BodyA = a, BodyB = b, LocalAnchorA = new Vector2F(0, 0.5f), LocalAnchorB = new Vector2F(0, -0.5f) });
            for (int i = 0; i < 20; i++) { world.Step(1f / 60f); }
            var all = new System.Collections.Generic.List<RecursiveAI.OnePhysics.Physics2D.Body>();
            foreach (var body in world.Bodies) { if (body.Type == BodyType.Dynamic) { all.Add(body); } }
            return world.ExtractBodies(all.ToArray(), new BodySetOptions { Move = false }).ToArray();
        }
        else
        {
            var world = new PhysicsWorld3D(new WorldOptions3D { WorldId = 30 });
            world.AddBox(new(40, 1, 40), at: new(0, -0.5, 0), type: BodyType.Static);
            var a = world.AddBox(new(1, 1, 1), at: new(0, 0.5, 0));
            var b = world.AddBox(new(1, 1, 1), at: new(0.1, 1.5, 0));
            world.AddRagdoll(RecursiveAI.OnePhysics.Physics3D.RagdollDef.Humanoid, new Vector3D(5, 1.2, 0));
            world.CreateJoint(new FixedJointDef { BodyA = a, BodyB = b, LocalFrameA = new JointFrame3D(new Vector3F(0, 0.5f, 0), QuaternionF.Identity), LocalFrameB = new JointFrame3D(new Vector3F(0, -0.5f, 0), QuaternionF.Identity) });
            for (int i = 0; i < 20; i++) { world.Step(1f / 60f); }
            var all = new System.Collections.Generic.List<RecursiveAI.OnePhysics.Physics3D.Body>();
            foreach (var body in world.Bodies) { if (body.Type == BodyType.Dynamic) { all.Add(body); } }
            return world.ExtractBodies(all.ToArray(), new BodySetOptions { Move = false }).ToArray();
        }
    }

    /// <summary>A packet that carries a driven car, a seated driver and a walking character (version 2 records).</summary>
    public static byte[] SeedEntities(int dimension)
    {
        if (dimension == 2)
        {
            var world = new PhysicsWorld2D(new WorldOptions2D { WorldId = 32 });
            world.AddBox(new(40, 1), at: new(0, -0.5), type: BodyType.Static);
            var car = world.AddCar(new Vector2D(0, 0));
            var driver = world.AddCharacter(new Vector2D(0, 2));
            driver.Mount(car.Chassis, new Vector2F(0.2f, 1.4f));
            var walker = world.AddCharacter(new Vector2D(6, 1));
            for (int i = 0; i < 20; i++) { car.Drive(0.5f); walker.Move(new Vector2F(0.02f, -0.02f), 1f / 60f); world.Step(1f / 60f); }
            return world.ExtractBodies(new[] { car.Chassis, driver.Body, walker.Body }, new BodySetOptions { Move = false }).ToArray();
        }
        else
        {
            var world = new PhysicsWorld3D(new WorldOptions3D { WorldId = 32 });
            world.AddBox(new(40, 1, 40), at: new(0, -0.5, 0), type: BodyType.Static);
            var chassis = world.AddBox(new(1.8f, 0.6f, 4.2f), at: new(0, 1.0, 0), density: 357f);
            var car = world.AddVehicle(chassis, RecursiveAI.OnePhysics.Physics3D.VehicleDef.Car);
            var driver = world.AddCharacter(new Vector3D(0, 2, 0));
            driver.Mount(chassis, new Vector3F(-0.4f, 0.5f, 0.2f));
            var walker = world.AddCharacter(new Vector3D(6, 1, 0));
            for (int i = 0; i < 20; i++) { car.Drive(0.5f); walker.Move(new Vector3F(0.02f, -0.02f, 0), 1f / 60f); world.Step(1f / 60f); }
            return world.ExtractBodies(new[] { chassis, driver.Body, walker.Body }, new BodySetOptions { Move = false }).ToArray();
        }
    }
}
