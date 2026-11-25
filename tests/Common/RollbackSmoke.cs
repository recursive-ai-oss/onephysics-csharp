using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// Mark and RollbackTo in both worlds: after a rollback the world has the marked snapshot's bytes and goes on bit for bit like a world that never rolled back; a fork steps the same.
internal static class RollbackSmoke
{
    internal static bool Run(out ulong digest)
    {
        ulong a = Planar(1), b = Planar(3), c = Spatial(1), d = Spatial(3);
        digest = a ^ (c * 31);
        return a == b && c == d && a != 0 && c != 0;
    }

    private static ulong Planar(int threads)
    {
        P2.PhysicsWorld2D Make()
        {
            var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 9, ThreadCount = threads });
            w.AddBox(new Vector2F(40, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
            for (int i = 0; i < 8; i++) w.AddBox(new Vector2F(1, 1), at: new Vector2D(0.2 * i, 0.5 + 1.1 * i));
            return w;
        }

        var world = Make(); var never = Make();
        for (int i = 0; i < 30; i++) { world.Step(1f / 60); never.Step(1f / 60); }
        var bytes = world.Snapshot();
        var mark = world.Mark();
        world.AddCircle(0.4f, at: new Vector2D(1, 12));
        for (int i = 0; i < 20; i++) world.Step(1f / 60);
        world.RollbackTo(mark);
        bool ok = bytes.AsSpan().SequenceEqual(world.Snapshot());
        var fork = world.Fork();
        for (int i = 0; i < 30; i++) { world.Step(1f / 60); never.Step(1f / 60); fork.Step(1f / 60); }
        ok &= world.ComputeHash() == never.ComputeHash() && fork.ComputeHash() == never.ComputeHash();
        ok &= world.ReleaseMark(mark) && !world.IsMarkValid(mark);
        return ok ? world.ComputeHash() : 0;
    }

    private static ulong Spatial(int threads)
    {
        P3.PhysicsWorld3D Make()
        {
            var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 9, ThreadCount = threads });
            w.AddBox(new Vector3F(40, 1, 40), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
            for (int i = 0; i < 8; i++) w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0.2 * i, 0.5 + 1.1 * i, 0));
            return w;
        }

        var world = Make(); var never = Make();
        for (int i = 0; i < 30; i++) { world.Step(1f / 60); never.Step(1f / 60); }
        var bytes = world.Snapshot();
        var mark = world.Mark();
        world.AddSphere(0.4f, at: new Vector3D(1, 12, 0));
        for (int i = 0; i < 20; i++) world.Step(1f / 60);
        world.RollbackTo(mark);
        bool ok = bytes.AsSpan().SequenceEqual(world.Snapshot());
        var fork = world.Fork();
        for (int i = 0; i < 30; i++) { world.Step(1f / 60); never.Step(1f / 60); fork.Step(1f / 60); }
        ok &= world.ComputeHash() == never.ComputeHash() && fork.ComputeHash() == never.ComputeHash();
        ok &= world.ReleaseMark(mark) && !world.IsMarkValid(mark);
        return ok ? world.ComputeHash() : 0;
    }
}
