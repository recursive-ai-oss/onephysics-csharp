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

        // The 3D world: one and three threads, and a restore halfway, must all end in the same state.
        ulong v1 = World3D(1, false), v3 = World3D(3, false), vr = World3D(1, true);
        bool w3Same = v1 == v3 && v1 == vr;
        Console.WriteLine((w3Same ? "OK world3d " : "FAIL world3d ") + v1.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        return ok && same && bpSame && w2Same && w3Same ? 0 : 1;
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
