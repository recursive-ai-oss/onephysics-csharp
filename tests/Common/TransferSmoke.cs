using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// Body-set transfer in both worlds: a stack and a ragdoll leave one world as bytes and arrive in another, which then steps like the first; equal at one and three threads.
internal static class TransferSmoke
{
    internal static bool Run(out ulong digest)
    {
        ulong a = Planar(1), b = Planar(3), c = Spatial(1), d = Spatial(3);
        digest = a ^ (c * 31);
        return a == b && c == d && a != 0 && c != 0;
    }

    private static ulong Planar(int threads)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 9, ThreadCount = threads, SleepEnabled = false });
        w.AddBox(new Vector2F(40, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 4; i++) w.AddBox(new Vector2F(1, 1), at: new Vector2D(0.02 * i, 0.5 + 1.001 * i));
        w.AddRagdoll(P2.RagdollDef.Humanoid, new Vector2D(6, 1.2), 1.5f);
        for (int i = 0; i < 30; i++) w.Step(1f / 60);
        var all = new System.Collections.Generic.List<P2.Body>();
        foreach (var body in w.Bodies) { if (body.Type == BodyType.Dynamic) all.Add(body); }
        var packet = BodySetPacket.Read(w.ExtractBodies(all.ToArray(), new BodySetOptions { Move = false }).ToArray());
        var copy = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 9, ThreadCount = threads, SleepEnabled = false });
        copy.AddBox(new Vector2F(40, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var map = copy.InsertBodies(packet);
        for (int i = 0; i < 30; i++) { w.Step(1f / 60); copy.Step(1f / 60); }
        bool same = true;
        for (int i = 0; i < map.NewBodies.Length; i++) { same &= w.GetBody(new BodyHandle(w.Id, map.OldBodies[i].Index, map.OldBodies[i].Generation)).Position == copy.GetBody(map.NewBodies[i]).Position; }
        return same && map.NewRagdolls.Length == 1 ? copy.ComputeHash() : 0;
    }

    private static ulong Spatial(int threads)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 9, ThreadCount = threads, SleepEnabled = false });
        w.AddBox(new Vector3F(40, 1, 40), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 4; i++) w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0.02 * i, 0.5 + 1.001 * i, 0));
        w.AddRagdoll(P3.RagdollDef.Humanoid, new Vector3D(6, 1.2, 0));
        for (int i = 0; i < 30; i++) w.Step(1f / 60);
        var all = new System.Collections.Generic.List<P3.Body>();
        foreach (var body in w.Bodies) { if (body.Type == BodyType.Dynamic) all.Add(body); }
        var packet = BodySetPacket.Read(w.ExtractBodies(all.ToArray(), new BodySetOptions { Move = false }).ToArray());
        var copy = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 9, ThreadCount = threads, SleepEnabled = false });
        copy.AddBox(new Vector3F(40, 1, 40), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var map = copy.InsertBodies(packet);
        for (int i = 0; i < 30; i++) { w.Step(1f / 60); copy.Step(1f / 60); }
        bool same = true;
        for (int i = 0; i < map.NewBodies.Length; i++) { same &= w.GetBody(new BodyHandle(w.Id, map.OldBodies[i].Index, map.OldBodies[i].Generation)).Position == copy.GetBody(map.NewBodies[i]).Position; }
        return same && map.NewRagdolls.Length == 1 ? copy.ComputeHash() : 0;
    }

    // Characters and vehicles in packets: a car with a seated driver and a walker leave one world as bytes and arrive in another, which then drives and steps like the first; equal at one and three threads.
    internal static bool RunEntities(out ulong digest)
    {
        ulong a = Cars2(1), b = Cars2(3), c = Cars3(1), d = Cars3(3);
        digest = a ^ (c * 31);
        return a == b && c == d && a != 0 && c != 0;
    }

    private static ulong Cars2(int threads)
    {
        P2.PhysicsWorld2D World() { var x = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 9, ThreadCount = threads, SleepEnabled = false }); x.AddBox(new Vector2F(400, 1), at: new Vector2D(0, -0.5), type: BodyType.Static); return x; }
        var w = World();
        var car = w.AddCar(new Vector2D(0, 0));
        var driver = w.AddCharacter(new Vector2D(0, 2));
        driver.Mount(car.Chassis, new Vector2F(0.2f, 1.4f));
        var walker = w.AddCharacter(new Vector2D(8, 1));
        for (int i = 0; i < 30; i++) { car.Drive(0.5f); walker.Move(new Vector2F(0.03f, -0.02f), 1f / 60); w.Step(1f / 60); }
        var packet = BodySetPacket.Read(w.ExtractBodies(new[] { car.Chassis, driver.Body, walker.Body }, new BodySetOptions { Move = false }).ToArray());
        var copy = World();
        var map = copy.InsertBodies(packet);
        var car2 = copy.GetVehicle(map.NewVehicles[0]); var walker2 = copy.GetCharacter(map.NewCharacters[1]);
        for (int i = 0; i < 30; i++) { car.Drive(0.5f); walker.Move(new Vector2F(0.03f, -0.02f), 1f / 60); w.Step(1f / 60); car2.Drive(0.5f); walker2.Move(new Vector2F(0.03f, -0.02f), 1f / 60); copy.Step(1f / 60); }
        bool same = car.Chassis.Position == car2.Chassis.Position && walker.Body.Position == walker2.Body.Position && copy.GetCharacter(map.NewCharacters[0]).Mode == CharacterMode.Seated;
        return same ? copy.ComputeHash() : 0;
    }

    private static ulong Cars3(int threads)
    {
        P3.PhysicsWorld3D World() { var x = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 9, ThreadCount = threads, SleepEnabled = false }); x.AddBox(new Vector3F(400, 1, 400), at: new Vector3D(0, -0.5, 0), type: BodyType.Static); return x; }
        var w = World();
        var chassis = w.AddBox(new Vector3F(1.8f, 0.6f, 4.2f), at: new Vector3D(0, 1.0, 0), density: 357f);
        var car = w.AddVehicle(chassis, P3.VehicleDef.Car);
        var driver = w.AddCharacter(new Vector3D(0, 2, 0));
        driver.Mount(chassis, new Vector3F(-0.4f, 0.5f, 0.2f));
        var walker = w.AddCharacter(new Vector3D(8, 1, 0));
        for (int i = 0; i < 30; i++) { car.Drive(0.5f); walker.Move(new Vector3F(0.03f, -0.02f, 0), 1f / 60); w.Step(1f / 60); }
        var packet = BodySetPacket.Read(w.ExtractBodies(new[] { chassis, driver.Body, walker.Body }, new BodySetOptions { Move = false }).ToArray());
        var copy = World();
        var map = copy.InsertBodies(packet);
        var car2 = copy.GetVehicle(map.NewVehicles[0]); var walker2 = copy.GetCharacter(map.NewCharacters[1]);
        for (int i = 0; i < 30; i++) { car.Drive(0.5f); walker.Move(new Vector3F(0.03f, -0.02f, 0), 1f / 60); w.Step(1f / 60); car2.Drive(0.5f); walker2.Move(new Vector3F(0.03f, -0.02f, 0), 1f / 60); copy.Step(1f / 60); }
        bool same = chassis.Position == car2.Chassis.Position && walker.Body.Position == walker2.Body.Position && copy.GetCharacter(map.NewCharacters[0]).Mode == CharacterMode.Seated;
        return same ? copy.ComputeHash() : 0;
    }
}
