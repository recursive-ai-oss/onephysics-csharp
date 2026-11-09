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
        Scenarios.Register("world2d/mass-offset", ctx =>
        {
            var w = new P2.PhysicsWorld2D(new() { WorldId = 995, ThreadCount = ctx.Threads });
            w.AddBox(new(20,1),at:new(0,-0.5),type:BodyType.Static);
            var body=w.CreateBody(new P2.BodyDef { Position=new(0,2) });
            w.CreateShape(body,Shape2D.CreateCircle(default,0.5f).WithCenterOfMassOffset(new(0,-0.2f)));
            w.CreateCircle(body,0.3f,new(1,0));
            for(int i=0;i<180;i++)
            {
                w.Step(1f/60);
                if(i==90){var bytes=w.Snapshot();w.Restore(bytes);if(!bytes.AsSpan().SequenceEqual(w.Snapshot()))throw new InvalidOperationException("Mass-offset restore differs.");}
                ctx.Write((long)w.ComputeHash());ctx.EndStep();
            }
        });
        Scenarios.Register("world3d/plane-mass-offset", ctx =>
        {
            var w = new P3.PhysicsWorld3D(new() { WorldId = 996, ThreadCount = ctx.Threads });
            w.AddPlane(RecursiveAI.OneMath.Vector3F.UnitY);
            var body=w.CreateBody(new P3.BodyDef { Position=new(0,2,0) });
            w.CreateShape(body,P3.ShapeSpec3D.Box(new(1,1,1)).WithCenterOfMassOffset(new(0,-0.2f,0)));
            w.CreateSphere(body,0.3f,new P3.ShapeDef { LocalPosition=new(1,0,0) });
            w.AddTaperedCylinder(0.4f,0.2f,0.3f,at:new(3,2,0));
            for(int i=0;i<180;i++)
            {
                if(i==60) w.ShiftOrigin(new(1024,0,0));
                w.Step(1f/60);
                if(i==90){var bytes=w.Snapshot();w.Restore(bytes);if(!bytes.AsSpan().SequenceEqual(w.Snapshot()))throw new InvalidOperationException("Plane restore differs.");}
                ctx.Write((long)w.ComputeHash());ctx.EndStep();
            }
        });
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
