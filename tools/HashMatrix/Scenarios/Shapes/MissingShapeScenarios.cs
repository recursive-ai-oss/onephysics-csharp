using System;
using System.Runtime.CompilerServices;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class MissingShapeScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("world2d/heightfield", ctx =>
        {
            var w = new P2.PhysicsWorld2D(new() { WorldId = 993, ThreadCount = ctx.Threads });
            w.AddHeightField([0,0,0,0,1,1,0,0], holes: [false,false,false,false,false,true,false]);
            w.AddBox(new(0.8f,1), at: new(1,0.5)).LinearVelocity = new(1,0);
            w.AddCircle(0.3f, at: new(4,2));
            w.AddEmpty(at:new(8,3),mass:2,inertia:3);
            for (int i=0;i<180;i++)
            {
                w.Step(1f/60);
                if (i==90) { byte[] bytes = w.Snapshot(); w.Restore(bytes); if (!bytes.AsSpan().SequenceEqual(w.Snapshot())) throw new InvalidOperationException("Height-field restore differs."); }
                ctx.Write((long)w.ComputeHash()); ctx.EndStep();
            }
        });
        Scenarios.Register("world3d/frustum-triangle", ctx =>
        {
            var w = new P3.PhysicsWorld3D(new() { WorldId = 994, ThreadCount = ctx.Threads });
            w.AddEmpty(at:new(8,3,0),mass:2,inertia:new RecursiveAI.OneMath.Vector3F(3,4,5));
            w.AddTriangle(new(-10,0,-10),new(0,0,10),new(10,0,-10));
            for(int i=0;i<4;i++) w.AddTaperedCylinder(0.5f,0.2f,0.4f,at:new(i-2,2,0));
            w.AddTriangle(new(-0.2f,0,-0.2f),new(0,0,0.2f),new(0.2f,0,-0.2f),at:new(0,4,0),type:BodyType.Dynamic,convexRadius:0.02f);
            for(int i=0;i<180;i++)
            {
                w.Step(1f/60);
                if(i==90) { byte[] bytes=w.Snapshot(); w.Restore(bytes); if(!bytes.AsSpan().SequenceEqual(w.Snapshot())) throw new InvalidOperationException("Convex recipe restore differs."); }
                ctx.Write((long)w.ComputeHash());ctx.EndStep();
            }
        });
    }
}
