using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// Ropes attached to bodies and swept against a thin wall in both dimensions: threads agree, a snapshot restored
// into a fresh world continues identically, and a fast rope does not pass through the wall.
internal static class RodSmoke
{
    internal static bool Run(out ulong digest)
    {
        ulong a = Planar(1, false), b = Planar(3, true), c = Spatial(1, false), d = Spatial(3, true);
        digest = a ^ (c * 31);
        return a == b && c == d;
    }
    private static ulong Planar(int threads, bool restore)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 5, ThreadCount = threads, SleepEnabled = false });
        w.AddBox(new Vector2F(.02f, 6), at: new Vector2D(0, 0), type: BodyType.Static);
        var weight = w.AddCircle(.1f, at: new Vector2D(-2, 2), density: 5);
        var rope = w.AddRope(new Vector2D(-2, 3), new Vector2D(-2, 2), 8);
        rope.Attach(8, weight);
        var fast = w.AddRope(new Vector2D(-1, 1), new Vector2D(-.2, 1), 8, pinStart: false);
        for (int i = 0; i < 8; i++) fast.SetVelocity(i + 1, new Vector2F(30, 0));
        return Drive(w, restore ? () => { var copy = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 5, ThreadCount = threads, SleepEnabled = false }); copy.Restore(w.Snapshot()); return copy; } : null, x => x.Step(1f / 60), x => x.ComputeHash(), () => rope.GetPosition(0).X < 0.01 && fast.GetPosition(8).X < 0.0);
    }
    private static ulong Spatial(int threads, bool restore)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 5, ThreadCount = threads, SleepEnabled = false });
        w.AddBox(new Vector3F(.02f, 6, 6), at: new Vector3D(0, 0, 0), type: BodyType.Static);
        var weight = w.AddSphere(.1f, at: new Vector3D(-2, 2, 0), density: 5);
        var rope = w.AddRope(new Vector3D(-2, 3, 0), new Vector3D(-2, 2, 0), 8);
        rope.Attach(8, weight);
        var fast = w.AddRope(new Vector3D(-1, 1, 0), new Vector3D(-.2, 1, 0), 8, pinStart: false);
        for (int i = 0; i < 8; i++) fast.SetVelocity(i + 1, new Vector3F(30, 0, 0));
        return Drive(w, restore ? () => { var copy = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 5, ThreadCount = threads, SleepEnabled = false }); copy.Restore(w.Snapshot()); return copy; } : null, x => x.Step(1f / 60), x => x.ComputeHash(), () => rope.GetPosition(0).X < 0.01 && fast.GetPosition(8).X < 0.0);
    }
    private static ulong Drive<T>(T world, Func<T>? fresh, Action<T> step, Func<T, ulong> hash, Func<bool> sane) where T : class
    {
        for (int i = 0; i < 60; i++) step(world);
        if (fresh != null) { var copy = fresh(); for (int i = 0; i < 60; i++) { step(world); step(copy); if (hash(world) != hash(copy)) throw new InvalidOperationException("Restored rod world diverged."); } }
        else for (int i = 0; i < 60; i++) step(world);
        if (!sane()) throw new InvalidOperationException("A rod passed through the wall.");
        return hash(world);
    }
}
