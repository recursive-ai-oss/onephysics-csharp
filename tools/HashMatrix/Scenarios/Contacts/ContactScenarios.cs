using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OnePhysics.LowLevel;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class ContactScenarios
{
    [ModuleInitializer]
    internal static void Register()
    {
        Scenarios.Register("contacts/modify-2d", Planar);
        Scenarios.Register("contacts/modify-3d", Spatial);
    }

    private static void Edit(ref ContactModification c)
    {
        c.Friction = 0.7f;
        c.Restitution = 0.1f;
        c.TangentSpeed = (c.ShapeB.Index % 3 - 1) * 0.4f;
        c.NormalImpulseLimit = 0.2f;
        c.Enabled = c.ShapeB.Index % 5 != 0;
        for (int p = 0; p < c.PointCount; p++) c.SetSeparation(p, c.GetSeparation(p) + 0.001f);
    }

    private static void Planar(ScenarioContext ctx)
    {
        var a = Make2(ctx.Threads, 1);
        var b = Make2(ctx.Threads, 8);
        for (int i = 0; i < 90; i++)
        {
            if (i == 40) { a.Restore(a.Snapshot()); b.Restore(b.Snapshot()); }
            if (i == 60) { a.ContactModifier = null; b.ContactModifier = null; }
            a.Step(1f / 60); b.Step(1f / 60);
            ulong hash = a.ComputeHash();
            if (hash != b.ComputeHash()) throw new InvalidOperationException("Planar contact modifier differs between scalar and wide kernels at step " + i);
            ctx.Write(unchecked((long)hash)); ctx.EndStep();
        }
    }
    private static P2.PhysicsWorld2D Make2(int threads, int width)
    {
        var w = new P2.PhysicsWorld2D(new() { WorldId = 990, ThreadCount = threads, SleepEnabled = false });
        w.Engine.KernelWidth = width;
        w.AddBox(new(100, 1), at: new(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 64; i++) w.AddBox(new(0.8f, 0.8f), at: new(i % 16 * 1.2 - 9, i / 16 + 0.39));
        w.ContactModifier = Edit;
        return w;
    }
    private static void Spatial(ScenarioContext ctx)
    {
        var a = Make3(ctx.Threads, 1);
        var b = Make3(ctx.Threads, 8);
        for (int i = 0; i < 90; i++)
        {
            if (i == 40) { a.Restore(a.Snapshot()); b.Restore(b.Snapshot()); }
            if (i == 60) { a.ContactModifier = null; b.ContactModifier = null; }
            a.Step(1f / 60); b.Step(1f / 60);
            ulong hash = a.ComputeHash();
            if (hash != b.ComputeHash()) throw new InvalidOperationException("Spatial contact modifier differs between scalar and wide kernels at step " + i);
            ctx.Write(unchecked((long)hash)); ctx.EndStep();
        }
    }
    private static P3.PhysicsWorld3D Make3(int threads, int width)
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = 991, ThreadCount = threads, SleepEnabled = false });
        w.Engine.KernelWidth = width;
        w.AddBox(new(100, 1, 100), at: new(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 64; i++) w.AddBox(new(0.8f, 0.8f, 0.8f), at: new(i % 8 * 1.2 - 4, 0.39, i / 8 * 1.2 - 4));
        w.ContactModifier = Edit;
        return w;
    }
}
