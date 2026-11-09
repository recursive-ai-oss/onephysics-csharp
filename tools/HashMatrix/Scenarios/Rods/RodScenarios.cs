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
    }
}
