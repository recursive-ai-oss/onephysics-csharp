using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

// Body-set transfer: the packet bytes and the world hash after every step of the receiving world, with and without a portal transform and a boundary anchor.
internal static class TransferScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("transfer/world3d", World3);
        Scenarios.Register("transfer/world2d", World2);
        Scenarios.Register("transfer/entities3d", Entities3);
        Scenarios.Register("transfer/entities2d", Entities2);
    }

    private static void World3(ScenarioContext ctx)
    {
        var a = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 1101, ThreadCount = ctx.Threads, SleepEnabled = false });
        a.AddBox(new(60, 1, 60), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var stack = new P3.Body[5];
        for (int i = 0; i < stack.Length; i++) stack[i] = a.AddBox(new(1, 1, 1), new Vector3D(0.02 * i, 0.5 + 1.001 * i, 0));
        a.AddRagdoll(P3.RagdollDef.Humanoid, new Vector3D(8, 0.4, 0), QuaternionF.CreateFromAxisAngle(new Vector3F(1, 0, 0), 1.5707964f));
        var pivot = a.AddBox(new(0.4f, 0.4f, 0.4f), new Vector3D(-8, 8, 0), type: BodyType.Static);
        var arm = a.AddBox(new(0.2f, 2f, 0.2f), new Vector3D(-8, 7, 0));
        arm.AngularVelocity = new Vector3F(2, 0, 0);
        a.CreateJoint(new P3.HingeJointDef { BodyA = pivot, BodyB = arm, LocalFrameB = new P3.JointFrame3D(new Vector3F(0, 1, 0), QuaternionF.Identity) });
        for (int i = 0; i < 90; i++) a.Step(1f / 60);

        var all = new System.Collections.Generic.List<P3.Body>();
        foreach (var body in a.Bodies) if (body.Type == BodyType.Dynamic) all.Add(body);
        var packet = a.ExtractBodies(all.ToArray(), new BodySetOptions { Move = false, Boundary = BoundaryPolicy.HoldAsAnchor });
        ctx.Write(packet.AsSpan());
        ctx.Write(packet.BodyCount); ctx.Write(packet.AnchorCount); ctx.Write(packet.ShapeCount); ctx.Write(packet.JointCount); ctx.Write(packet.ContactCount);
        ctx.EndStep();

        // Two receivers: one in place, one through a portal that turns and moves the set.
        var inPlace = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 1102, ThreadCount = ctx.Threads, SleepEnabled = false });
        inPlace.AddBox(new(60, 1, 60), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        inPlace.InsertBodies(packet);
        var portal = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 1103, ThreadCount = ctx.Threads, SleepEnabled = false });
        portal.Options.Gravity = default;
        var map = portal.InsertBodies(BodySetPacket.Read(packet.ToArray()), new Vector3D(100, 5, -30), QuaternionF.CreateFromAxisAngle(new Vector3F(0, 1, 0), 0.6f));
        ctx.Write((long)inPlace.ComputeHash()); ctx.Write((long)portal.ComputeHash()); ctx.Write(map.NewBodies.Length); ctx.Write(map.NewShapes.Length);
        ctx.EndStep();
        for (int i = 0; i < 60; i++)
        {
            inPlace.Step(1f / 60); portal.Step(1f / 60);
            ctx.Write((long)inPlace.ComputeHash()); ctx.Write((long)portal.ComputeHash());
            ctx.EndStep();
        }

        // Moving instead of copying leaves the source world with what it did not give away.
        var moved = a.ExtractBodies(stack);
        ctx.Write(moved.ByteCount); ctx.Write(a.BodyCount); ctx.Write((long)a.ComputeHash());
        ctx.EndStep();
    }

    private static void World2(ScenarioContext ctx)
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 1111, ThreadCount = ctx.Threads, SleepEnabled = false });
        a.AddBox(new(60, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        var stack = new P2.Body[6];
        for (int i = 0; i < stack.Length; i++) stack[i] = a.AddBox(new(1, 1), new Vector2D(0.02 * i, 0.5 + 1.001 * i));
        a.AddRagdoll(P2.RagdollDef.Humanoid, new Vector2D(8, 1.2), 1.5707964f);
        var pivot = a.AddBox(new(0.4f, 0.4f), new Vector2D(-8, 8), type: BodyType.Static);
        var arm = a.AddBox(new(0.2f, 2f), new Vector2D(-8, 7));
        arm.AngularVelocity = 2f;
        a.CreateJoint(new P2.RevoluteJointDef { BodyA = pivot, BodyB = arm, LocalAnchorB = new Vector2F(0, 1) });
        for (int i = 0; i < 90; i++) a.Step(1f / 60);

        var all = new System.Collections.Generic.List<P2.Body>();
        foreach (var body in a.Bodies) if (body.Type == BodyType.Dynamic) all.Add(body);
        var packet = a.ExtractBodies(all.ToArray(), new BodySetOptions { Move = false, Boundary = BoundaryPolicy.HoldAsAnchor });
        ctx.Write(packet.AsSpan());
        ctx.Write(packet.BodyCount); ctx.Write(packet.AnchorCount); ctx.Write(packet.ShapeCount); ctx.Write(packet.JointCount); ctx.Write(packet.ContactCount);
        ctx.EndStep();

        var inPlace = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 1112, ThreadCount = ctx.Threads, SleepEnabled = false });
        inPlace.AddBox(new(60, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        inPlace.InsertBodies(packet);
        var portal = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 1113, ThreadCount = ctx.Threads, SleepEnabled = false });
        portal.Options.Gravity = default;
        var map = portal.InsertBodies(BodySetPacket.Read(packet.ToArray()), new Vector2D(100, 5), 0.6f);
        ctx.Write((long)inPlace.ComputeHash()); ctx.Write((long)portal.ComputeHash()); ctx.Write(map.NewBodies.Length); ctx.Write(map.NewShapes.Length);
        ctx.EndStep();
        for (int i = 0; i < 60; i++)
        {
            inPlace.Step(1f / 60); portal.Step(1f / 60);
            ctx.Write((long)inPlace.ComputeHash()); ctx.Write((long)portal.ComputeHash());
            ctx.EndStep();
        }

        var moved = a.ExtractBodies(stack);
        ctx.Write(moved.ByteCount); ctx.Write(a.BodyCount); ctx.Write((long)a.ComputeHash());
        ctx.EndStep();
    }

    // Characters and vehicles in packets: a driven car with a seated driver, a tank and a walker; the packet bytes, then two receivers (in place and through a portal) stepped with the same driving inputs.
    private static void Entities3(ScenarioContext ctx)
    {
        var a = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 1121, ThreadCount = ctx.Threads, SleepEnabled = false });
        a.AddBox(new(400, 1, 400), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var chassis = a.AddBox(new(1.8f, 0.6f, 4.2f), new Vector3D(0, 1.0, 0), density: 357f);
        var car = a.AddVehicle(chassis, P3.VehicleDef.Car);
        var driver = a.AddCharacter(new Vector3D(0, 2, 0), P3.CharacterDef.Humanoid);
        driver.Mount(chassis, new Vector3F(-0.4f, 0.5f, 0.2f));
        var tank = a.AddTank(new Vector3D(12, 1.2, 0));
        var moto = a.AddMotorcycle(new Vector3D(-12, 1.0, 0));
        var walker = a.AddCharacter(new Vector3D(6, 1, 0));
        for (int i = 0; i < 70; i++)
        {
            car.Drive(0.5f, (float)(0.3 * Math64.Sin(i * 0.03))); tank.SetInput(throttle: 0.7f, steer: 0.2f); moto.SetInput(throttle: 0.5f, steer: 0.1f * (float)Math64.Sin(i * 0.05)); walker.Move(new Vector3F(0.03f, -0.02f, 0.01f), 1f / 60); a.Step(1f / 60);
        }

        var packet = a.ExtractBodies(new[] { chassis, driver.Body, tank.Chassis, moto.Chassis, walker.Body }, new BodySetOptions { Move = false });
        ctx.Write(packet.AsSpan());
        ctx.Write(packet.BodyCount); ctx.Write(packet.CharacterCount); ctx.Write(packet.VehicleCount); ctx.Write(packet.ContactCount);
        ctx.EndStep();

        var inPlace = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 1122, ThreadCount = ctx.Threads, SleepEnabled = false });
        inPlace.AddBox(new(400, 1, 400), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var m1 = inPlace.InsertBodies(packet);
        var portal = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 1123, ThreadCount = ctx.Threads, SleepEnabled = false });
        portal.AddBox(new(400, 1, 400), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var m2 = portal.InsertBodies(BodySetPacket.Read(packet.ToArray()), new Vector3D(30, 0, -20), QuaternionF.CreateFromAxisAngle(new Vector3F(0, 1, 0), 0.6f));
        ctx.Write((long)inPlace.ComputeHash()); ctx.Write((long)portal.ComputeHash()); ctx.Write(m1.NewCharacters.Length); ctx.Write(m2.NewVehicles.Length);
        ctx.EndStep();
        for (int i = 0; i < 90; i++)
        {
            foreach (var (w, m) in new[] { (inPlace, m1), (portal, m2) })
            {
                for (int k = 0; k < m.NewVehicles.Length; k++)
                {
                    var v = w.GetVehicle(m.NewVehicles[k]);
                    if (k == 0) { v.Drive(0.5f, (float)(0.3 * Math64.Sin((70 + i) * 0.03))); } else if (k == 1) { v.SetInput(throttle: 0.7f, steer: 0.2f); } else { v.SetInput(throttle: 0.5f, steer: 0.1f * (float)Math64.Sin((70 + i) * 0.05)); }
                }

                var ch = w.GetCharacter(m.NewCharacters[1]);
                ch.Move(new Vector3F(0.03f, -0.02f, 0.01f), 1f / 60);
                w.Step(1f / 60);
            }

            ctx.Write((long)inPlace.ComputeHash()); ctx.Write((long)portal.ComputeHash());
            ctx.EndStep();
        }
    }

    private static void Entities2(ScenarioContext ctx)
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 1131, ThreadCount = ctx.Threads, SleepEnabled = false });
        a.AddBox(new(400, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        var car = a.AddCar(new Vector2D(0, 0));
        var driver = a.AddCharacter(new Vector2D(0, 2));
        driver.Mount(car.Chassis, new Vector2F(0.2f, 1.4f));
        var tank = a.AddTank(new Vector2D(20, 0));
        var moto = a.AddMotorcycle(new Vector2D(-20, 0));
        var walker = a.AddCharacter(new Vector2D(6, 1));
        for (int i = 0; i < 70; i++)
        {
            car.Drive(0.5f); tank.Drive(0.7f); moto.Drive(0.5f, 0.1f * (float)Math64.Sin(i * 0.05)); walker.Move(new Vector2F(0.03f, -0.02f), 1f / 60); a.Step(1f / 60);
        }

        var packet = a.ExtractBodies(new[] { car.Chassis, driver.Body, tank.Chassis, moto.Chassis, walker.Body }, new BodySetOptions { Move = false });
        ctx.Write(packet.AsSpan());
        ctx.Write(packet.BodyCount); ctx.Write(packet.CharacterCount); ctx.Write(packet.VehicleCount); ctx.Write(packet.ContactCount);
        ctx.EndStep();

        var inPlace = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 1132, ThreadCount = ctx.Threads, SleepEnabled = false });
        inPlace.AddBox(new(400, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        var m1 = inPlace.InsertBodies(packet);
        var portal = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 1133, ThreadCount = ctx.Threads, SleepEnabled = false });
        portal.AddBox(new(400, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        var m2 = portal.InsertBodies(BodySetPacket.Read(packet.ToArray()), new Vector2D(30, 0), 0f);
        ctx.Write((long)inPlace.ComputeHash()); ctx.Write((long)portal.ComputeHash()); ctx.Write(m1.NewCharacters.Length); ctx.Write(m2.NewVehicles.Length);
        ctx.EndStep();
        for (int i = 0; i < 90; i++)
        {
            foreach (var (w, m) in new[] { (inPlace, m1), (portal, m2) })
            {
                for (int k = 0; k < m.NewVehicles.Length; k++)
                {
                    var v = w.GetVehicle(m.NewVehicles[k]);
                    v.Drive(k == 1 ? 0.7f : 0.5f, k == 2 ? 0.1f * (float)Math64.Sin((70 + i) * 0.05) : 0f);
                }

                w.GetCharacter(m.NewCharacters[1]).Move(new Vector2F(0.03f, -0.02f), 1f / 60);
                w.Step(1f / 60);
            }

            ctx.Write((long)inPlace.ComputeHash()); ctx.Write((long)portal.ComputeHash());
            ctx.EndStep();
        }
    }
}
