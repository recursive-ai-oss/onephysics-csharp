using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Fields;
using P2=RecursiveAI.OnePhysics.Physics2D;
using P3=RecursiveAI.OnePhysics.Physics3D;

internal static class FieldScenes
{
    internal static P2.PhysicsWorld2D Planar(int threads)
    {
        var w=new P2.PhysicsWorld2D(new(){WorldId=781,ThreadCount=threads,Gravity=new(0,-10),SleepEnabled=false});
        w.AddBox(new(50,1),at:new(0,-4),type:BodyType.Static);
        for(int i=0;i<12;i++)
        {
            var body=i%3==0?w.AddCircle(0.3f,at:new(i*1.5-8,0.3),density:500):w.AddBox(new(0.8f,0.4f),at:new(i*1.5-8,0.3),angleRadians:0.1f,density:500);
            if(i<2)body.MagneticMoment=new(1,0);
        }
        w.AddBuoyancyEffector(0,damping:3);w.AddMagnetism(0.00001f,3);
        return w;
    }
    internal static P3.PhysicsWorld3D Spatial(int threads)
    {
        var w=new P3.PhysicsWorld3D(new(){WorldId=782,ThreadCount=threads,Gravity=default,SleepEnabled=false});
        var orbit=w.AddSphere(0.1f,at:new(10,0,0));orbit.LinearVelocity=new(0,1,0);
        w.AddRadialGravity(default,10,30);
        var vortex=w.AddSphere(0.1f,at:new(102,0,0));
        w.AddVortex(new(100,0,0),2,1,10);
        w.AddGravityZone(new(0,0,-0.1f),new(99,-3,-3),new(105,3,3));
        var a=w.AddSphere(0.1f,at:new(-30,0,0));var b=w.AddSphere(0.1f,at:new(-28,0,0));a.MagneticMoment=new(1,0,0);b.MagneticMoment=new(1,0,0);w.AddMagnetism(0.00001f,3);
        return w;
    }
    internal static ulong Run2(int threads,bool restore)
    {
        var w=Planar(threads);
        for(int i=0;i<120;i++)
        {
            if(i==40)w.Explode(new(0,1),0.1f,20);
            w.Step(1f/60);
            if(i==60&&restore){var snapshot=w.Snapshot();w=w.Fork();w.Restore(snapshot);}
        }
        return w.ComputeHash();
    }
    internal static ulong Run3(int threads,bool restore)
    {
        var w=Spatial(threads);
        for(int i=0;i<120;i++)
        {
            if(i==40)w.Explode(new(0,1,0),0.001f,20);
            w.Step(1f/60);
            if(i==60&&restore){var snapshot=w.Snapshot();w=w.Fork();w.Restore(snapshot);}
        }
        return w.ComputeHash();
    }
}
