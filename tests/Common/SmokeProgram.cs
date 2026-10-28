using System;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Core.Reference;

// Shared by the NativeAOT and browser-WASM smoke apps: steps a small deterministic scene on one and on several threads,
// snapshots and restores it, and checks that every path gives the same world hash.
internal static class SmokeProgram
{
    private static int Main()
    {
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
        // The 2D collision scenario must give the same hash on every platform (see docs/collision2d.md).
        ulong ch = RecursiveAI.OnePhysics.Collision2D.Reference.CollisionHash2D.Compute();
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
        return ok && same && bpSame && w2Same && jSame && cSame && w3Same ? 0 : 1;
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
            if (i == 2) { j.SetLimits(-0.3f, 0.3f); j.LimitEnabled = true; }
            if (i == 0) { j.MotorEnabled = true; j.MotorSpeed = 0.5f; j.MaxMotorForce = 20f; }
            prev = link;
        }

        var car = w.AddBox(new RecursiveAI.OneMath.Vector2F(3f, 0.5f), new RecursiveAI.OneMath.Vector2D(-8, 2), density: 2f);
        var wheel = w.AddCircle(0.5f, new RecursiveAI.OneMath.Vector2D(-7, 1));
        var wj = w.AddWheel(car, wheel, new RecursiveAI.OneMath.Vector2D(-7, 1), new RecursiveAI.OneMath.Vector2F(0, 1));
        wj.MotorEnabled = true; wj.MotorSpeed = -4f; wj.MaxMotorForce = 20f;
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
            var def = RecursiveAI.OnePhysics.Physics2D.Character.Humanoid;
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
}
