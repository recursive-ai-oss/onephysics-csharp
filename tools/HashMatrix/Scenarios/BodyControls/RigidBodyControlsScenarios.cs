using System;
using System.IO;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class RigidBodyControlsScenarios
{
    [ModuleInitializer]
    internal static void Init() { Scenarios.Register("world2d/body-controls", Run2); Scenarios.Register("world3d/body-controls",Run3); }
    private static void Run2(ScenarioContext ctx)
    {
        var w=new P2.PhysicsWorld2D(new(){WorldId=881,ThreadCount=ctx.Threads,MaxAngularSpeed=5});
        var platform=w.AddBox(new(20,1),at:new(0,-0.5),type:BodyType.Kinematic);
        var b=w.AddBox(new(1,1),at:new(0,0.5));b.Mass=2;b.CenterOfMass=new(0.1f,0);b.SetInertia(3);b.Controls=new(RigidbodyConstraints.FreezeRotationZ,2,3,0.2f);
        for(int i=0;i<180;i++)
        {
            platform.MovePosition(new(i*0.005,-0.5));
            if(i==60){b.ResetMass();b.ResetCenterOfMass();b.ResetInertia();b.Constraints=RigidbodyConstraints.FreezePositionY;}
            if(i==120){b.Constraints=RigidbodyConstraints.None;b.ApplyImpulse(new(0.1f,0.2f));}
            w.Step(1f/60);
            if(i%60==0){var copy=w.Fork();copy.Restore(w.Snapshot());if(copy.ComputeHash()!=w.ComputeHash())throw new InvalidOperationException("Control restore differs.");ctx.Write(w.Snapshot());}
            ctx.Write((long)w.ComputeHash());ctx.EndStep();
        }
    }
    private static void Run3(ScenarioContext ctx)
    {
        var w=new P3.PhysicsWorld3D(new(){WorldId=882,ThreadCount=ctx.Threads,MaxAngularSpeed=5});
        var platform=w.AddBox(new(20,1,20),at:new(0,-0.5,0),type:BodyType.Kinematic);
        var b=w.AddBox(new(1,1,1),at:new(0,0.5,0));b.Mass=2;b.CenterOfMass=new(0.1f,0,0);b.SetInertia(new(2,3,4),QuaternionF.CreateFromAxisAngle(new(0,1,0),0.2f));b.Controls=new(RigidbodyConstraints.FreezeRotationX|RigidbodyConstraints.FreezeRotationZ,2,3,0.2f);
        for(int i=0;i<180;i++)
        {
            platform.MovePosition(new(i*0.005,-0.5,0));
            if(i==60){b.ResetMass();b.ResetCenterOfMass();b.ResetInertia();b.Constraints=RigidbodyConstraints.FreezePositionY;}
            if(i==120){b.Constraints=RigidbodyConstraints.None;b.ApplyImpulse(new(0.1f,0.2f,0.05f));}
            w.Step(1f/60);
            if(i%60==0){var copy=w.Fork();copy.Restore(w.Snapshot());if(copy.ComputeHash()!=w.ComputeHash())throw new InvalidOperationException("Control restore differs.");ctx.Write(w.Snapshot());}
            ctx.Write((long)w.ComputeHash());ctx.EndStep();
        }
    }
}
