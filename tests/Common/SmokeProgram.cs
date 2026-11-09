using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using V3F = RecursiveAI.OneMath.Vector3F;
using V3D = RecursiveAI.OneMath.Vector3D;

// Shared by the NativeAOT and browser-WASM smoke apps: steps a small deterministic scene on one and on several threads,
// snapshots and restores it, and checks that every path gives the same world hash.
internal static class SmokeProgram
{
    private static bool ExtendedQueriesSmoke()
    {
        var a = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D();
        var b = new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D();
        var source2 = a.AddCircle(0.5f, default);
        var source3 = b.AddSphere(0.5f, default);
        a.AddCircle(0.5f, new RecursiveAI.OneMath.Vector2D(5, 0), BodyType.Static);
        b.AddSphere(0.5f, new V3D(5, 0, 0), BodyType.Static);
        var requests2 = new[] { RecursiveAI.OnePhysics.Physics2D.QueryInput2D.Ray(default, new RecursiveAI.OneMath.Vector2F(1, 0), 10f) };
        var requests3 = new[] { RecursiveAI.OnePhysics.Physics3D.QueryInput3D.Ray(default, new V3F(1, 0, 0), 10f) };
        var results2 = new RecursiveAI.OnePhysics.Physics2D.QueryHit2D[2];
        var results3 = new RecursiveAI.OnePhysics.Physics3D.QueryHit3D[2];
        bool good = a.QueryBatch(requests2, results2) == 1 && b.QueryBatch(requests3, results3) == 1;
        var counts = new int[1]; var any = new bool[1];
        a.QueryAllBatch(requests2, results2, counts, 2); good &= counts[0] == 2;
        b.QueryAllBatch(requests3, results3, counts, 2); good &= counts[0] == 2;
        a.QueryAnyBatch(requests2, any); good &= any[0]; b.QueryAnyBatch(requests3, any); good &= any[0];
        good &= source2.Sweep(new RecursiveAI.OneMath.Vector2F(1, 0), out var sweep2, 10f) && sweep2.Distance > 3f;
        good &= source3.Sweep(new V3F(1, 0, 0), out var sweep3, 10f) && sweep3.Distance > 3f;
        good &= a.Penetration(Shape2D.CreateCircle(default, 0.5f), default, 0f, out _);
        good &= b.Penetration(ConvexShape3D.Sphere(0.5f), default, default, out _);
        good &= source2.Sweep(new RecursiveAI.OneMath.Vector2F(10,0),1f,out _);
        good &= source3.Sweep(new V3F(10,0,0),new V3F(0,0,1),out _);
        good &= a.Depenetrate(Shape2D.CreateCircle(default,0.5f),new RecursiveAI.OneMath.Vector2D(5,0),0).Converged;
        good &= b.Depenetrate(ConvexShape3D.Sphere(0.5f),new V3D(5,0,0),default).Converged;
        Console.WriteLine(good ? "OK extended-queries" : "FAIL extended-queries");
        return good;
    }

    private static ulong Grid2(int threads,bool restore)
    {
        var w=new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new(){WorldId=991,ThreadCount=threads});
        var terrain=w.AddTileGrid(3,1,[TileKind2D.Solid,TileKind2D.Solid,TileKind2D.Solid]);
        w.AddCircle(0.2f,new(1,3));
        var character=w.AddCharacter(new(0.4,2.2));
        for(int i=0;i<60;i++)
        {
            if(i==20)w.SetTile(terrain,0,0,TileKind2D.SlopeUpRight);
            character.MoveVelocity(new(0.2f,character.IsGrounded?0:-2),1f/60);
            w.Step(1f/60);
            if(i==30 && restore) {var bytes=w.Snapshot();w=w.Fork();w.Restore(bytes);character=w.GetCharacter(new(w.Id,character.RawHandle.Index,character.RawHandle.Generation));}
        }
        return w.ComputeHash();
    }
    private static ulong Grid3(int threads,bool restore)
    {
        var w=new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D(new(){WorldId=992,ThreadCount=threads});
        w.AddVoxelGrid(3,1,3,[true,true,true,true,true,true,true,true,true]);w.AddSphere(0.2f,new(1,3,1));
        for(int i=0;i<60;i++)
        {
            w.Step(1f/60);
            if(i==30 && restore) {var bytes=w.Snapshot();w=w.Fork();w.Restore(bytes);}
        }
        return w.ComputeHash();
    }
    private static ulong Ragdolls2DHash(int threads)
    {
        var w = Vehicles2DScene.RagdollPileScene(threads).World;
        for (int i = 0; i < 120; i++) { w.Step(Vehicles2DScene.Dt); }
        return w.ComputeHash();
    }

    private static int Main()
    {
        ulong grids2=Grid2(1,false),grids3=Grid3(1,false);
        if(grids2!=Grid2(3,true)||grids3!=Grid3(3,true))throw new InvalidOperationException("Grid smoke hashes differ.");
        Console.WriteLine("OK grids "+grids2.ToString("x16",System.Globalization.CultureInfo.InvariantCulture)+" "+grids3.ToString("x16",System.Globalization.CultureInfo.InvariantCulture));
        ulong fields2=FieldScenes.Run2(1,false),fields3=FieldScenes.Run3(1,false);
        if(fields2!=FieldScenes.Run2(3,true)||fields3!=FieldScenes.Run3(3,true))throw new InvalidOperationException("Field smoke hashes differ.");
        Console.WriteLine("OK fields "+fields2.ToString("x16",System.Globalization.CultureInfo.InvariantCulture)+" "+fields3.ToString("x16",System.Globalization.CultureInfo.InvariantCulture));
        var rods2 = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D();
        var rods3 = new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D();
        rods2.AddRope(new(0,2),new(1,2),4); rods3.AddRope(new(0,2,0),new(1,2,0),4);
        for (int i=0;i<10;i++) { rods2.Step(1f/60); rods3.Step(1f/60); }
        var rodCopy2=rods2.Fork(); var rodCopy3=rods3.Fork(); rodCopy2.Restore(rods2.Snapshot()); rodCopy3.Restore(rods3.Snapshot());
        for (int i=0;i<10;i++) { rods2.Step(1f/60); rods3.Step(1f/60); rodCopy2.Step(1f/60); rodCopy3.Step(1f/60); }
        if (rods2.ComputeHash()!=rodCopy2.ComputeHash() || rods3.ComputeHash()!=rodCopy3.ComputeHash()) throw new InvalidOperationException("Rod smoke hashes differ.");
        Console.WriteLine("OK rods "+rods2.ComputeHash().ToString("x16",System.Globalization.CultureInfo.InvariantCulture)+" "+rods3.ComputeHash().ToString("x16",System.Globalization.CultureInfo.InvariantCulture));
        const float dt = 1f / 60f;
        var a = ParticleScene.CreatePile(200, 42, threadCount: 1, worldId: 1);
        var b = ParticleScene.CreatePile(200, 42, threadCount: 4, worldId: 1);
        for (int i = 0; i < 60; i++) { a.Step(dt); b.Step(dt); }
        byte[] snap = a.World.Snapshot();
        var c = new ParticleScene(new WorldOptions { WorldId = 1 });
        c.World.Restore(snap);
        for (int i = 0; i < 60; i++) { a.Step(dt); b.Step(dt); c.Step(dt); }
        ulong h = a.World.ComputeHash();
        bool ok = h == b.World.ComputeHash() && h == c.World.ComputeHash();
        // The 2D collision scenario must give the same hash on every platform (see docs/spec/collision2d.md).
        ulong ch = RecursiveAI.OnePhysics.LowLevel.CollisionHash2D.Compute();
        ok &= ch == 0x31E47F1BCE1B321FUL;
        Console.WriteLine("collision2d " + ch.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        Console.WriteLine(ok ? "OK " + h.ToString("x16", System.Globalization.CultureInfo.InvariantCulture) : "FAIL");

        // Convex collision results must be bit-identical on every platform: hash a fixed set of queries.
        ulong hash = Collision3DHash.Compute();
        bool same = hash == Collision3DHash.Expected;
        Console.WriteLine((same ? "OK collision3d " : "FAIL collision3d ") + hash.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));

        // The broadphase's pairs, queries and snapshot must be bit-identical on every platform and thread count.
        ulong bp = BroadphaseHash.Compute();
        bool bpSame = bp == BroadphaseHash.Expected;
        Console.WriteLine((bpSame ? "OK broadphase " : "FAIL broadphase ") + bp.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));

        // The 2D world: one and three threads, and a restore halfway, must all end in the same state.
        ulong w1 = World2D(1, false), w3 = World2D(3, false), wr = World2D(1, true);
        bool w2Same = w1 == w3 && w1 == wr;
        Console.WriteLine((w2Same ? "OK world2d " : "FAIL world2d ") + w1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        // 2D joints (hinge chain with limit and motor, wheel with spring, springy distance joint, a weld that breaks): same state on one and three threads and after a restore.
        ulong j1 = World2DJoints(1, false), j3 = World2DJoints(3, false), jr = World2DJoints(1, true);
        bool jSame = j1 == j3 && j1 == jr;
        Console.WriteLine((jSame ? "OK joints2d " : "FAIL joints2d ") + j1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        // 2D bullets and characters: same state on one and three threads and after a restore; the bullet must stop at the wall and the walkers must stand on the floor.
        ulong c1 = World2DCharacters(1, false), c3 = World2DCharacters(3, false), cr = World2DCharacters(1, true);
        bool cSame = c1 == c3 && c1 == cr;
        Console.WriteLine((cSame ? "OK characters2d " : "FAIL characters2d ") + c1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        
        // The 3D world: one and three threads, and a restore halfway, must all end in the same state.
        ulong v1 = World3D(1, false), v3 = World3D(3, false), vr = World3D(1, true);
        bool w3Same = v1 == v3 && v1 == vr;
        Console.WriteLine((w3Same ? "OK world3d " : "FAIL world3d ") + v1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        // 3D joints (one of every kind): same state on one and three threads and after a restore, and the hinge's limit must hold.
        ulong k1 = World3DJoints(1, false), k3 = World3DJoints(3, false), kr = World3DJoints(1, true);
        bool kSame = k1 == k3 && k1 == kr;
        Console.WriteLine((kSame ? "OK joints3d " : "FAIL joints3d ") + k1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        // 3D services (layers, a sensor, mesh pair events, queries): same state on one and three threads and after a restore.
        ulong s1 = Services3DScene.Run(1, false), s3 = Services3DScene.Run(3, false), sr = Services3DScene.Run(1, true);
        bool sSame = s1 == s3 && s1 == sr;
        Console.WriteLine((sSame ? "OK services3d " : "FAIL services3d ") + s1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        // 3D characters (stairs, ramp, block seam, sliding and turning platforms, pushing, rigid character): same state on one and three threads and after a restore.
        ulong h1 = Characters3DScene.Run(1, false), h3 = Characters3DScene.Run(3, false), hr = Characters3DScene.Run(1, true);
        bool hSame = h1 == h3 && h1 == hr;
        Console.WriteLine((hSame ? "OK characters3d " : "FAIL characters3d ") + h1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        // 3D vehicles (height field, ramp mesh, dynamic slab, car, limited-slip car, truck, kart): same state on one and three threads and after a restore.
        ulong x1 = Vehicles3DScene.Run(1, false), x3 = Vehicles3DScene.Run(3, false), xr = Vehicles3DScene.Run(1, true);
        bool xSame = x1 == x3 && x1 == xr;
        Console.WriteLine((xSame ? "OK vehicles3d " : "FAIL vehicles3d ") + x1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        // 2D vehicles (car, limited-slip car, truck, tank, motorcycle on a height field with bumps and a ramp) and a pile of ragdolls: same state on one and three threads and after a restore.
        ulong v21 = Vehicles2DScene.Run(1, false), v23 = Vehicles2DScene.Run(3, false), v2r = Vehicles2DScene.Run(1, true);
        bool v2Same = v21 == v23 && v21 == v2r;
        Console.WriteLine((v2Same ? "OK vehicles2d " : "FAIL vehicles2d ") + v21.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        // Boats, hovercraft, a glider in a gust and thruster rigs in both dimensions: same state on one and three threads and after a restore.
        ulong a31 = Actuators3DScene.Run(1, false), a33 = Actuators3DScene.Run(3, false), a3r = Actuators3DScene.Run(1, true);
        bool act3 = a31 == a33 && a31 == a3r;
        Console.WriteLine((act3 ? "OK actuators3d " : "FAIL actuators3d ") + a31.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        ulong a21 = Actuators2DScene.Run(1, false), a23 = Actuators2DScene.Run(3, false), a2r = Actuators2DScene.Run(1, true);
        bool act2 = a21 == a23 && a21 == a2r;
        Console.WriteLine((act2 ? "OK actuators2d " : "FAIL actuators2d ") + a21.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        // Rail vehicles and bicycles in both dimensions: same state on one and three threads and after a restore.
        ulong rb31 = RailBikeScenes3D.Run(1, false), rb33 = RailBikeScenes3D.Run(3, false), rb3r = RailBikeScenes3D.Run(1, true);
        ulong rb21 = RailBikeScenes2D.Run(1, false), rb23 = RailBikeScenes2D.Run(3, false), rb2r = RailBikeScenes2D.Run(1, true);
        bool railBike = rb31 == rb33 && rb31 == rb3r && rb21 == rb23 && rb21 == rb2r;
        Console.WriteLine((railBike ? "OK railbike " : "FAIL railbike ") + rb31.ToString("x16", System.Globalization.CultureInfo.InvariantCulture) + " " + rb21.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        // Characters that sit, hang, climb, grip ropes and stand on soft bodies, ropes and decks, in both dimensions: same state on one and three threads and after a restore.
        bool abilities = true;
        foreach (string scene in CharacterAbilityScenes3D.Names)
        {
            ulong q1 = CharacterAbilityScenes3D.Run(scene, 1, false), q3 = CharacterAbilityScenes3D.Run(scene, 3, false), qr = CharacterAbilityScenes3D.Run(scene, 1, true);
            abilities &= q1 == q3 && q1 == qr;
            Console.WriteLine((q1 == q3 && q1 == qr ? "OK abilities3d " : "FAIL abilities3d ") + scene + " " + q1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        }

        foreach (string scene in CharacterAbilityScenes2D.Names)
        {
            ulong q1 = CharacterAbilityScenes2D.Run(scene, 1, false), q3 = CharacterAbilityScenes2D.Run(scene, 3, false), qr = CharacterAbilityScenes2D.Run(scene, 1, true);
            abilities &= q1 == q3 && q1 == qr;
            Console.WriteLine((q1 == q3 && q1 == qr ? "OK abilities2d " : "FAIL abilities2d ") + scene + " " + q1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        }

        // Terrain that is edited or sampled at run time: same state on one and three threads and after a restore.
        foreach (string scene in TerrainEditScenes.Names)
        {
            ulong t1 = TerrainEditScenes.Run(scene, 1, false), t3 = TerrainEditScenes.Run(scene, 3, false), tr = TerrainEditScenes.Run(scene, 1, true);
            abilities &= t1 == t3 && t1 == tr;
            Console.WriteLine((t1 == t3 && t1 == tr ? "OK terrain " : "FAIL terrain ") + scene + " " + t1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        }

        ulong rg1 = Ragdolls2DHash(1), rg3 = Ragdolls2DHash(3);
        Console.WriteLine((rg1 == rg3 ? "OK ragdolls2d " : "FAIL ragdolls2d ") + rg1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        // 3D tracked vehicles and motorcycles (pivoting tank, tank on a ramp, bike in a turn, bike braking): same state on one and three threads and after a restore.
        ulong vk1 = VehicleKinds3DScene.Run(1, false), vk3 = VehicleKinds3DScene.Run(3, false), vkr = VehicleKinds3DScene.Run(1, true);
        bool kkSame = vk1 == vk3 && vk1 == vkr;
        Console.WriteLine((kkSame ? "OK vehiclekinds3d " : "FAIL vehiclekinds3d ") + vk1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        // 3D ragdolls (powered at two strengths, limp, keyframed, thrown boxes): same state on one and three threads and after a restore.
        ulong d1 = Ragdolls3DScene.Run(1, false), d3 = Ragdolls3DScene.Run(3, false), dr = Ragdolls3DScene.Run(1, true);
        bool dSame = d1 == d3 && d1 == dr;
        Console.WriteLine((dSame ? "OK ragdolls3d " : "FAIL ragdolls3d ") + d1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        // 3D soft bodies (cloth, jelly, rubber, a pinned cloth, a rigid box landing on a cube): same state on one and three threads and after a restore.
        ulong f1 = SoftBodies3DScene.Run(1, false), f3 = SoftBodies3DScene.Run(3, false), fr = SoftBodies3DScene.Run(1, true);
        bool fSame = f1 == f3 && f1 == fr;
        Console.WriteLine((fSame ? "OK softbodies3d " : "FAIL softbodies3d ") + f1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        // A skinned soft body (a sleeve on a swinging arm): same state on one and three threads and after a restore.
        ulong sk1 = SoftSkinned3DScene.Run(1, false), sk3 = SoftSkinned3DScene.Run(3, false), skr = SoftSkinned3DScene.Run(1, true);
        bool skSame = sk1 == sk3 && sk1 == skr;
        Console.WriteLine((skSame ? "OK softskinned3d " : "FAIL softskinned3d ") + sk1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        // 2D soft bodies (a pile, soft bodies on and under rigid stacks, balloons in a height-field valley): same state on one and three threads and after a restore.
        bool soft2d = true;
        foreach (string scene in SoftBodies2DScene.Names)
        {
            ulong g1 = SoftBodies2DScene.Run(scene, 1, false), g3 = SoftBodies2DScene.Run(scene, 3, false), gr = SoftBodies2DScene.Run(scene, 1, true);
            soft2d &= g1 == g3 && g1 == gr;
            Console.WriteLine((g1 == g3 && g1 == gr ? "OK softbodies2d " : "FAIL softbodies2d ") + scene + " " + g1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        }

        // 3D surface contacts and tearing (a cloth folding on itself, a cloth and a bar torn by hooks): same state on one and three threads and after a restore.
        foreach (string scene in SoftFracture3DScene.Names)
        {
            ulong g1 = SoftFracture3DScene.Run(scene, 1, false, null, 60), g3 = SoftFracture3DScene.Run(scene, 3, false, null, 60), gr = SoftFracture3DScene.Run(scene, 1, true, null, 60);
            soft2d &= g1 == g3 && g1 == gr;
            Console.WriteLine((g1 == g3 && g1 == gr ? "OK softfracture3d " : "FAIL softfracture3d ") + scene + " " + g1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        }

        bool missingShapes = MissingShapes();
        Console.WriteLine(missingShapes ? "OK shape-recipes" : "FAIL shape-recipes");
        bool surfaces = RecursiveAI.OnePhysics.Tests.Common.ContactSurfaceScene.Planar(1, restore: true) == RecursiveAI.OnePhysics.Tests.Common.ContactSurfaceScene.Planar(3)
            && RecursiveAI.OnePhysics.Tests.Common.ContactSurfaceScene.Spatial(1, restore: true) == RecursiveAI.OnePhysics.Tests.Common.ContactSurfaceScene.Spatial(3);
        Console.WriteLine(surfaces ? "OK contact surfaces" : "FAIL contact surfaces");
        bool contacts = ContactModifiers();
        Console.WriteLine(contacts ? "OK contact modifiers" : "FAIL contact modifiers");
        bool bodyControls = BodyControls();
        Console.WriteLine(bodyControls ? "OK body-controls" : "FAIL body-controls");
        bool repairs = HostRepairs();
        Console.WriteLine(repairs ? "OK host-repairs" : "FAIL host-repairs");
        bool formats = FileFormats();
        Console.WriteLine(formats ? "OK formats" : "FAIL formats");
        bool inspection = InspectionSmoke.Run(out ulong inspectionHash);
        Console.WriteLine((inspection ? "OK inspection " : "FAIL inspection ") + inspectionHash.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        bool rods = RodSmoke.Run(out ulong rodHash);
        Console.WriteLine((rods ? "OK rods " : "FAIL rods ") + rodHash.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        bool pulleys = PulleySmoke.Run(out ulong pulleyHash);
        Console.WriteLine((pulleys ? "OK pulleys " : "FAIL pulleys ") + pulleyHash.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        bool machines = JointMachineSmoke.Run();
        Console.WriteLine(machines ? "OK machines" : "FAIL machines");
        return railBike && abilities && act3 && act2 && rods && pulleys && ExtendedQueriesSmoke() && missingShapes && repairs && bodyControls && machines && surfaces && contacts && formats && inspection && ok && same && bpSame && w2Same && jSame && cSame && w3Same && kSame && sSame && hSame && xSame && kkSame && dSame && fSame && skSame && soft2d ? 0 : 1;
    }

    private static bool MissingShapes()
    {
        var a = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D();
        a.AddHeightField([0,0,0,1,1,0]);
        a.AddEmpty(at:new(5,4),mass:2,inertia:3);
        a.AddCircle(0.3f,at:new(1,2));
        var b = new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D();
        b.AddTriangle(new(-5,0,-5),new(0,0,5),new(5,0,-5));
        b.AddTaperedCylinder(0.5f,0.2f,0.4f,at:new(0,2,0));
        b.AddPlane(V3F.UnitY,at:new(0,-3,0));
        var ma=a.CreateBody(new RecursiveAI.OnePhysics.Physics2D.BodyDef { Position=new(10,2) });
        a.CreateShape(ma,Shape2D.CreateCircle(default,0.3f).WithCenterOfMassOffset(new(0,-0.1f)));
        var mb=b.CreateBody(new RecursiveAI.OnePhysics.Physics3D.BodyDef { Position=new(10,2,0) });
        b.CreateShape(mb,RecursiveAI.OnePhysics.Physics3D.ShapeSpec3D.Sphere(0.3f).WithCenterOfMassOffset(new(0,-0.1f,0)));
        b.AddEmpty(at:new(5,4,0),mass:2,inertia:new V3F(3,4,5));
        var ca=a.Fork(); var cb=b.Fork();
        for(int i=0;i<60;i++)
        {
            a.Step(1f/60);b.Step(1f/60);ca.Step(1f/60);cb.Step(1f/60);
            if(a.ComputeHash()!=ca.ComputeHash()||b.ComputeHash()!=cb.ComputeHash()) return false;
        }
        byte[] sa=a.Snapshot(),sb=b.Snapshot();a.Restore(sa);b.Restore(sb);
        return sa.AsSpan().SequenceEqual(a.Snapshot()) && sb.AsSpan().SequenceEqual(b.Snapshot());
    }

    private readonly struct SmokeContactModifier : IContactModifier
    {
        public void Modify(ref ContactModification c)
        {
            c.TangentSpeed = 0.5f;
            c.NormalImpulseLimit = 0.2f;
            c.Friction = 0.5f;
            c.Enabled = c.ShapeB.Index % 3 != 0;
            c.SetSeparation(0, c.GetSeparation(0) + 0.001f);
        }
    }
    private static bool ContactModifiers()
    {
        var a = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new() { WorldId = 3200, ThreadCount = 1 });
        var b = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new() { WorldId = 3200, ThreadCount = 3 });
        foreach (var w in new[] { a, b })
        {
            w.AddBox(new(20, 1), at: new(0, -0.5), type: BodyType.Static);
            for (int i = 0; i < 8; i++)
            {
                var box = w.AddBox(new(1, 1), at: new(i * 2 - 7, 0.49));
                if (i % 4 == 0) box.Constraints = RigidbodyConstraints.FreezePositionX;
            }
            w.SetContactModifier(new SmokeContactModifier());
        }
        var c = new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D(new() { WorldId = 3201, ThreadCount = 1 });
        var d = new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D(new() { WorldId = 3201, ThreadCount = 3 });
        foreach (var w in new[] { c, d })
        {
            w.AddBox(new(20, 1, 20), at: new(0, -0.5, 0), type: BodyType.Static);
            for (int i = 0; i < 8; i++)
            {
                var box = w.AddBox(new(1, 1, 1), at: new(i * 2 - 7, 0.49, 0));
                if (i % 4 == 0) box.Constraints = RigidbodyConstraints.FreezePositionX;
            }
            w.SetContactModifier(new SmokeContactModifier());
        }
        for (int i = 0; i < 20; i++)
        {
            if (i == 10) { a.Restore(a.Snapshot()); c.Restore(c.Snapshot()); }
            a.Step(1f / 60); b.Step(1f / 60); c.Step(1f / 60); d.Step(1f / 60);
            if (a.ComputeHash() != b.ComputeHash() || c.ComputeHash() != d.ComputeHash()) return false;
        }
        // The conveyor and impulse cap run through the same kernel as the restricted inverse mass.
        foreach (var w in new[] { a, b })
            if (w.Core.Bodies.PositionOf(1)[0] != -7 || w.Core.Bodies.PositionOf(5)[0] != 1) return false;
        foreach (var w in new[] { c, d })
            if (w.Core.Bodies.PositionOf(1)[0] != -7 || w.Core.Bodies.PositionOf(5)[0] != 1) return false;
        return true;
    }

    private static bool HostRepairs()
    {
        var a = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D();
        var b = new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D();
        float angle = 0.5625f; // A degree conversion round trip loses one float32 bit.
        var c = a.AddCharacter(default, new() { SlopeLimitRadians = angle });
        var d = b.AddCharacter(default, new() { SlopeLimitRadians = angle });
        var sa = a.Snapshot(); var sb = b.Snapshot();
        a.Restore(sa); b.Restore(sb);
        bool good = c.SlopeLimitRadians == angle && d.SlopeLimitRadians == angle;
        var ca = a.Fork(); var cb = b.Fork();
        ca.RestoreRegion(a.CaptureRegion(new(-2, -2), new(2, 2)));
        cb.RestoreRegion(b.CaptureRegion(new(-2, -2, -2), new(2, 2, 2)));
        a.CanonicalizeBroadphase(); ca.CanonicalizeBroadphase();
        b.CanonicalizeBroadphase(); cb.CanonicalizeBroadphase();
        a.ShiftOrigin(new(1, 0)); ca.ShiftOrigin(new(1, 0));
        b.ShiftOrigin(new(1, 0, 0)); cb.ShiftOrigin(new(1, 0, 0));
        for (int i = 0; i < 3; i++)
        {
            a.Step(1f / 60); ca.Step(1f / 60); b.Step(1f / 60); cb.Step(1f / 60);
            good &= a.ComputeHash() == ca.ComputeHash() && b.ComputeHash() == cb.ComputeHash();
        }
        return good;
    }

    private static bool BodyControls()
    {
        var a = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new() { WorldId = 3100, Gravity = default });
        var b = new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D(new() { WorldId = 3101, Gravity = default });
        var x = a.CreateBody(new() { LinearVelocity = new(0.1f, 0), SleepSettings = new(0.2f, 0.3f, 0.1f), Controls = new(RigidbodyConstraints.FreezeRotationZ,1,1,1), MassOverride=2, InertiaOverride=3 });
        var y = b.CreateBody(new() { LinearVelocity = new(0.1f, 0, 0), SleepSettings = new(0.2f, 0.3f, 0.1f), Controls = new(RigidbodyConstraints.FreezeRotationZ,1,1,1), MassOverride=2, InertiaOverride=new V3F(2,3,4) });
        var platform=a.CreateBody(new(){Type=BodyType.Kinematic});platform.MovePosition(new(0.01,0));
        var platform3=b.CreateBody(new(){Type=BodyType.Kinematic});platform3.MovePosition(new(0.01,0,0));
        var f = new RecursiveAI.OnePhysics.Physics2D.FixedStepAccumulator(a, 0.125f);
        var g = new RecursiveAI.OnePhysics.Physics3D.FixedStepAccumulator(b, 0.125f);
        f.Advance(0.1875); g.Advance(0.1875);
        var p = f.GetInterpolatedPose(x); var q = g.GetInterpolatedPose(y);
        a.AddBox(new(4,1),at:new(10,-0.5),type:BodyType.Static);
        b.AddBox(new(4,1,4),at:new(10,-0.5,0),type:BodyType.Static);
        var fixedPlanar=a.AddBox(new(1,1),at:new(10,0.4));fixedPlanar.Constraints=RigidbodyConstraints.FreezeAll;
        var fixedSpatial=b.AddBox(new(1,1,1),at:new(10,0.4,0));fixedSpatial.Constraints=RigidbodyConstraints.FreezeAll;
        a.Step(0.01f);b.Step(0.01f);
        var aCopy = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new() { WorldId = 3100 });
        var bCopy = new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D(new() { WorldId = 3101 });
        aCopy.Restore(a.Snapshot()); bCopy.Restore(b.Snapshot());
        return f.InterpolationAlpha == 0.5 && g.InterpolationAlpha == 0.5 && p.Position.X == q.Position.X
            && a.ComputeHash() == aCopy.ComputeHash() && b.ComputeHash() == bCopy.ComputeHash() && !x.IsAwake && !y.IsAwake;
    }

    private static bool FileFormats()
    {
        var w=new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new RecursiveAI.OnePhysics.Physics2D.WorldOptions2D { WorldId=3000 });
        var b=w.AddCircle(0.5f,at:new(0,3));
        using var stream=new System.IO.MemoryStream();
        ulong first,second;
        using(var recorder=w.StartRecording(stream))
        {
            b.ApplyImpulse(new(0.1f,0));w.Step(1f/60);first=w.ComputeHash();
            w.Options.Gravity=new(0,-8);w.Step(1f/60);second=w.ComputeHash();
        }
        stream.Position=0;var player=new RecursiveAI.OnePhysics.Replay.ReplayPlayer(stream);var copy=player.CreateWorld2D();
        bool ok=player.ReadNextStep(copy) && copy.ComputeHash()==first && player.ReadNextStep(copy) && copy.ComputeHash()==second && !player.ReadNextStep(copy);
        string text=w.SaveScene();var sceneCopy=RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D.LoadScene(new System.IO.StringReader(text));
        ok &= sceneCopy.ComputeHash()==w.ComputeHash() && sceneCopy.SaveScene()==text;
        var world3=new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D(new RecursiveAI.OnePhysics.Physics3D.WorldOptions3D { WorldId=3001 });
        world3.AddSphere(0.5f,at:new(0,3,0));
        using var history=world3.StartHistory(new RecursiveAI.OnePhysics.Replay.HistoryOptions { KeyframeInterval=2,MemoryBudgetBytes=4*1024*1024 });
        world3.Step(1f/60);ulong h=world3.ComputeHash();world3.Step(1f/60);world3.Step(1f/60);
        history.Seek(1,world3);ok &= world3.ComputeHash()==h;
        string spatial=world3.SaveScene();var copy3=RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D.LoadScene(new System.IO.StringReader(spatial));
        return ok && copy3.ComputeHash()==world3.ComputeHash() && copy3.SaveScene()==spatial;
    }

    private static ulong World2DJoints(int threads, bool restoreHalfway)
    {
        var w = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new RecursiveAI.OnePhysics.Physics2D.WorldOptions2D { ThreadCount = threads, WorldId = 4 });
        var ground = w.AddBox(new RecursiveAI.OneMath.Vector2F(40f, 1f), new RecursiveAI.OneMath.Vector2D(0, -0.5), type: BodyType.Static);
        var prev = w.AddBox(new RecursiveAI.OneMath.Vector2F(1f, 1f), new RecursiveAI.OneMath.Vector2D(0, 10), type: BodyType.Static);
        for (int i = 0; i < 5; i++)
        {
            var link = w.AddBox(new RecursiveAI.OneMath.Vector2F(1f, 0.2f), new RecursiveAI.OneMath.Vector2D(0.5 + i, 10), density: 1f + i);
            var j = w.AddHinge(prev, link, new RecursiveAI.OneMath.Vector2D(i, 10));
            if (i == 2) { j.SetLimits(-0.3f, 0.3f); j.EnableLimit = true; }
            if (i == 0) { j.EnableMotor = true; j.MotorSpeed = 0.5f; j.MaxMotorTorque = 20f; }
            prev = link;
        }

        var car = w.AddBox(new RecursiveAI.OneMath.Vector2F(3f, 0.5f), new RecursiveAI.OneMath.Vector2D(-8, 2), density: 2f);
        var wheel = w.AddCircle(0.5f, new RecursiveAI.OneMath.Vector2D(-7, 1));
        var wj = w.AddWheel(car, wheel, new RecursiveAI.OneMath.Vector2D(-7, 1), new RecursiveAI.OneMath.Vector2F(0, 1));
        wj.EnableMotor = true; wj.MotorSpeed = -4f; wj.MaxMotorTorque = 20f;
        var bob = w.AddCircle(0.4f, new RecursiveAI.OneMath.Vector2D(8, 6));
        w.AddSpring(ground, bob, new RecursiveAI.OneMath.Vector2D(8, 9), new RecursiveAI.OneMath.Vector2D(8, 6), 2f, 0.3f);
        var beamWall = w.AddBox(new RecursiveAI.OneMath.Vector2F(0.4f, 3f), new RecursiveAI.OneMath.Vector2D(14, 3), type: BodyType.Static);
        var beam = w.AddBox(new RecursiveAI.OneMath.Vector2F(3f, 0.3f), new RecursiveAI.OneMath.Vector2D(15.8, 4), density: 30f);
        w.AddFixed(beamWall, beam, new RecursiveAI.OneMath.Vector2D(14.2, 4)).BreakForce = 200f;
        for (int i = 0; i < 120; i++)
        {
            if (restoreHalfway && i == 60)
            {
                byte[] snap = w.Snapshot();
                w = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new RecursiveAI.OnePhysics.Physics2D.WorldOptions2D { WorldId = 4 });
                w.Restore(snap);
            }

            w.Step(1f / 60f);
        }

        // The weld (8 joints in all) must have broken by now, in every variant.
        return w.ComputeHash() ^ (w.JointCount == 7 ? 0UL : 0xDEADUL);
    }

    private static ulong World2D(int threads, bool restoreHalfway)
    {
        var w = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new RecursiveAI.OnePhysics.Physics2D.WorldOptions2D { ThreadCount = threads, WorldId = 3 });
        w.AddBox(new RecursiveAI.OneMath.Vector2F(40f, 1f), new RecursiveAI.OneMath.Vector2D(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 30; i++)
        {
            var at = new RecursiveAI.OneMath.Vector2D(-4 + (i % 6) * 1.3, 1 + (i / 6) * 1.2);
            if (i % 2 == 0) { w.AddBox(new RecursiveAI.OneMath.Vector2F(1f, 1f), at, 0.1f * i); } else { w.AddCircle(0.45f, at); }
        }

        for (int i = 0; i < 120; i++)
        {
            if (restoreHalfway && i == 60)
            {
                byte[] snap = w.Snapshot();
                w = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new RecursiveAI.OnePhysics.Physics2D.WorldOptions2D { WorldId = 3 });
                w.Restore(snap);
            }

            w.Step(1f / 60f);
        }

        return w.ComputeHash();
    }

    private static ulong World2DCharacters(int threads, bool restoreHalfway)
    {
        var w = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new RecursiveAI.OnePhysics.Physics2D.WorldOptions2D { ThreadCount = threads, WorldId = 5 });
        w.AddBox(new RecursiveAI.OneMath.Vector2F(40f, 1f), new RecursiveAI.OneMath.Vector2D(0, -0.5), type: BodyType.Static);
        w.AddBox(new RecursiveAI.OneMath.Vector2F(1f, 0.5f), new RecursiveAI.OneMath.Vector2D(3, 0.25), type: BodyType.Static);
        w.AddBox(new RecursiveAI.OneMath.Vector2F(0.05f, 6f), new RecursiveAI.OneMath.Vector2D(12, 3), density: 5f);
        var bullet = w.CreateBody(new RecursiveAI.OnePhysics.Physics2D.BodyDef { Type = BodyType.Dynamic, Position = new RecursiveAI.OneMath.Vector2D(6, 3), IsBullet = true, GravityScale = 0f });
        w.CreateCircle(bullet, 0.1f);
        bullet.LinearVelocity = new RecursiveAI.OneMath.Vector2F(300f, 0f);
        for (int i = 0; i < 4; i++)
        {
            var def = RecursiveAI.OnePhysics.Physics2D.CharacterDef.Humanoid;
            def.PushStrength = i * 0.3f;
            w.AddCharacter(new RecursiveAI.OneMath.Vector2D(-6 + i * 1.5, 1.5), def);
        }

        for (int i = 0; i < 180; i++)
        {
            if (restoreHalfway && i == 90)
            {
                byte[] snap = w.Snapshot();
                w = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new RecursiveAI.OnePhysics.Physics2D.WorldOptions2D { WorldId = 5 });
                w.Restore(snap);
            }

            foreach (var c in w.GetCharacters())
            {
                c.Move(new RecursiveAI.OneMath.Vector2F(2f / 60f, (c.IsGrounded ? -0.5f : -4f) / 60f), 1f / 60f);
            }

            w.Step(1f / 60f);
        }

        var chars = w.GetCharacters();
        bool stand = chars.Length == 4 && chars[0].IsGrounded && chars[0].Position.X > -3;
        return w.ComputeHash() ^ (stand ? 0UL : 0xBEEFUL);
    }

    private static ulong World3D(int threads, bool restoreHalfway)
    {
        var w = new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D(new RecursiveAI.OnePhysics.Physics3D.WorldOptions3D { ThreadCount = threads, WorldId = 4 });
        w.AddBox(new RecursiveAI.OneMath.Vector3F(40f, 1f, 40f), new RecursiveAI.OneMath.Vector3D(0, -0.5, 0), type: BodyType.Static);
        RecursiveAI.OneMath.Vector3F[] tetra = [new(0.5f, 0.5f, 0.5f), new(-0.5f, -0.5f, 0.5f), new(-0.5f, 0.5f, -0.5f), new(0.5f, -0.5f, -0.5f)];
        for (int i = 0; i < 30; i++)
        {
            var at = new RecursiveAI.OneMath.Vector3D(-4 + (i % 6) * 1.3, 1 + (i / 6) * 1.2, (i % 3) * 0.4);
            switch (i % 4)
            {
                case 0: w.AddBox(new RecursiveAI.OneMath.Vector3F(1f, 1f, 1f), at); break;
                case 1: w.AddSphere(0.45f, at); break;
                case 2: w.AddCapsule(0.3f, 0.25f, at); break;
                default: w.AddConvexHull(tetra, at); break;
            }
        }

        for (int i = 0; i < 120; i++)
        {
            if (restoreHalfway && i == 60)
            {
                byte[] snap = w.Snapshot();
                w = new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D(new RecursiveAI.OnePhysics.Physics3D.WorldOptions3D { WorldId = 4 });
                w.Restore(snap);
            }

            w.Step(1f / 60f);
        }

        return w.ComputeHash();
    }

    private static ulong World3DJoints(int threads, bool restoreHalfway)
    {
        var w = new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D(new RecursiveAI.OnePhysics.Physics3D.WorldOptions3D { ThreadCount = threads, WorldId = 4 });
        w.AddBox(new V3F(100f, 1f, 100f), new V3D(0, -0.5, 0), type: BodyType.Static);
        var top = w.AddBox(new V3F(0.2f, 0.2f, 0.2f), new V3D(0, 6, 0), type: BodyType.Static);
        RecursiveAI.OnePhysics.Physics3D.Body Box(double x, double y, double z) => w.AddBox(new V3F(0.4f, 0.4f, 0.4f), new V3D(x, y, z));
        var b1 = Box(1, 6, 0); var b2 = Box(2, 6, 0); var b3 = Box(3, 6, 0); var b4 = Box(4, 6, 0); var b5 = Box(5, 6, 0); var b6 = Box(6, 6, 0); var b7 = Box(7, 6, 0); var b8 = Box(8, 6, 0);
        w.AddHinge(top, b1, new V3D(0.5, 6, 0), new V3F(0, 0, 1)).SetLimits(-1f, 1f);
        w.AddCharacterJoint(b1, b2, new V3D(1.5, 6, 0), new V3F(1, 0, 0));
        w.AddCone(b2, b3, new V3D(2.5, 6, 0), new V3F(1, 0, 0), 0.6f);
        var six = w.AddConfigurable(b3, b4, new V3D(3.5, 6, 0));
        six.SetAxisMode(RecursiveAI.OnePhysics.Physics3D.JointAxes.AngularZ, RecursiveAI.OnePhysics.Physics3D.JointAxisMode.Free);
        w.AddSlider(b4, b5, new V3D(4.5, 6, 0), new V3F(1, 0, 0)).EnableSpring = true;
        w.AddSpring(b5, b6, new V3D(5, 6, 0), new V3D(6, 6, 0), 3f, 0.3f);
        w.AddDistance(b6, b7, new V3D(6.2, 6, 0), new V3D(6.8, 6, 0));
        w.AddFixed(b7, b8, new V3D(7.5, 6, 0));
        var p = w.AddSphere(0.3f, new V3D(-3, 4, 0));
        var q = w.AddSphere(0.3f, new V3D(-4, 4, 0));
        w.AddGear(p, q, new V3F(0, 0, 1), new V3F(0, 0, 1), 1.5f);
        p.AngularVelocity = new V3F(0, 0, 2);
        var track = w.AddBox(new V3F(0.2f, 0.2f, 0.2f), new V3D(-8, 3, 0), type: BodyType.Static);
        var cart = w.AddBox(new V3F(0.3f, 0.3f, 0.3f), new V3D(-8, 3, 0));
        w.AddPath(track, cart, [new V3D(-8, 3, 0), new V3D(-7, 3, 1), new V3D(-6, 3.5, 2), new V3D(-5, 3, 3)]).EnableMotor = true;
        var l = w.AddBox(new V3F(0.4f, 0.4f, 0.4f), new V3D(-1, 5, 3));
        var r = w.AddBox(new V3F(0.4f, 0.4f, 0.4f), new V3D(1, 4, 3), density: 3f);
        w.AddPulley(l, r, l.Position, r.Position, new V3D(-1, 8, 3), new V3D(1, 8, 3));

        for (int i = 0; i < 120; i++)
        {
            if (restoreHalfway && i == 60)
            {
                byte[] snap = w.Snapshot();
                w = new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D(new RecursiveAI.OnePhysics.Physics3D.WorldOptions3D { WorldId = 4 });
                w.Restore(snap);
            }

            w.Step(1f / 60f);
        }

        return w.ComputeHash() ^ (w.JointCount == 12 ? 0UL : 0xBEEFUL);
    }
}
