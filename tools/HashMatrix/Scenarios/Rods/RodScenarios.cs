using System;
using System.Runtime.CompilerServices;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class RodScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("world2d/rods",ctx=>
        {
            var w = new P2.PhysicsWorld2D(new() { WorldId=783,ThreadCount=ctx.Threads });
            var rope = w.AddRope(new(0,2),new(2,2),12);
            w.AddCable(new(0,4),new(1,4),8);
            for (int i = 0; i < 120; i++)
            {
                if (i == 30) rope.SetTarget(0,new(0.5,2.5));
                if (i == 40) { var snapshot=w.Snapshot(); w=w.Fork(); w.Restore(snapshot); ctx.Write(w.Snapshot()); }
                if (i == 80) w.ShiftOrigin(new(16,-8));
                w.Step(1f/60); ctx.Write((long)w.ComputeHash()); ctx.EndStep();
            }
        });
        Scenarios.Register("world3d/rods",ctx=>
        {
            var w = new P3.PhysicsWorld3D(new() { WorldId=784,ThreadCount=ctx.Threads });
            var rope = w.AddRope(new(0,2,0),new(2,2,0.5),12);
            w.AddCable(new(0,4,0),new(1,4,0),8);
            for (int i = 0; i < 120; i++)
            {
                if (i == 30) rope.SetTarget(0,new(0.5,2.5,0.5));
                if (i == 40) { var snapshot=w.Snapshot(); w=w.Fork(); w.Restore(snapshot); ctx.Write(w.Snapshot()); }
                if (i == 80) w.ShiftOrigin(new(16,-8,2));
                w.Step(1f/60); ctx.Write((long)w.ComputeHash()); ctx.EndStep();
            }
        });
        // The four shared fixtures (contacts, swept collision, body coupling, torsion-free hair roots) with a
        // snapshot restored into a second, freshly created world half way: both worlds' hashes are recorded every step.
        foreach (var (name, steps, strands) in new[] { ("rope-bridge-3d", 120, 0), ("rope-whip-3d", 90, 0), ("hair-3d", 90, 80), ("rope-2d", 120, 0) })
        {
            Scenarios.Register("rods/" + name, ctx =>
            {
                var a = RodScene.Create(name, ctx.Threads, false, strands == 0 ? 1000 : strands);
                RodScene? b = null;
                for (int i = 0; i < steps; i++)
                {
                    if (i == 40)
                    {
                        b = RodScene.Create(name, ctx.Threads, false, strands == 0 ? 1000 : strands);
                        if (a.Spatial != null) { var bytes = a.Spatial.Snapshot(); ctx.Write(bytes); b.Spatial!.Restore(bytes); }
                        else { var bytes = a.Planar!.Snapshot(); ctx.Write(bytes); b.Planar!.Restore(bytes); }
                    }
                    a.Step(i); ctx.Write((long)a.Hash());
                    if (b != null) { b.Step(i); ctx.Write((long)b.Hash()); if (a.Hash() != b.Hash()) throw new InvalidOperationException(name + ": the restored world diverged at step " + i); }
                    ctx.EndStep();
                }
            });
        }
    }
}
