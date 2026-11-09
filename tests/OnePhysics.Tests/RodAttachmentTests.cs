using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Rods;
using P2=RecursiveAI.OnePhysics.Physics2D;
using P3=RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class RodAttachmentTests(ITestOutputHelper output)
{
    [Fact]
    public void BodyRopeBodyConservesLinearMomentumInBothDimensions()
    {
        var a=new P2.PhysicsWorld2D(new(){Gravity=default,SleepEnabled=false});
        var x=a.AddCircle(.1f,at:new(0,0));var y=a.AddCircle(.1f,at:new(1,0));
        var r=a.AddRope(default,new(1,0),8,pinStart:false);
        r.Particles.Damping=0;r.Attach(0,x);r.Attach(8,y);x.LinearVelocity=new(1,2);y.LinearVelocity=new(-.5f,-1);
        var initial=Momentum(a,r);double maxError=0,maxAnchor=0;
        for(int i=0;i<240;i++)
        {
            a.Step(1f/120);var p=Momentum(a,r);maxError=Math.Max(maxError,(p-initial).Length());
            var e=r.GetPosition(0)-x.Position;maxAnchor=Math.Max(maxAnchor,Math.Sqrt(e.X*e.X+e.Y*e.Y));
        }
        output.WriteLine($"2d momentum max error={maxError:R} kg m/s, attachment error={maxAnchor:R} m");
        Assert.InRange(maxError,0,1e-4);Assert.InRange(maxAnchor,0,.02);
        var b=new P3.PhysicsWorld3D(new(){Gravity=default,SleepEnabled=false});
        var u=b.AddSphere(.1f,at:new(0,0,0));var v=b.AddSphere(.1f,at:new(1,0,0));
        var s=b.AddRope(default,new(1,0,0),8,pinStart:false);s.Particles.Damping=0;s.Attach(0,u);s.Attach(8,v);
        u.LinearVelocity=new(1,2,.3f);v.LinearVelocity=new(-.5f,-1,-.15f);
        Vector3D Total()
        {
            var velocity=u.LinearVelocity;var other=v.LinearVelocity;
            var p=new Vector3D(u.Mass*velocity.X+v.Mass*other.X,u.Mass*velocity.Y+v.Mass*other.Y,u.Mass*velocity.Z+v.Mass*other.Z);
            var d=s.Particles;
            for(int i=0;i<=8;i++){var z=s.GetVelocity(i);p+=new Vector3D(z.X,z.Y,z.Z)/d.InverseMass[i];}
            return p;
        }
        var start=Total();double spatial=0;
        for(int i=0;i<240;i++){b.Step(1f/120);spatial=Math.Max(spatial,(Total()-start).Length());}
        output.WriteLine($"3d momentum max error={spatial:R} kg m/s");Assert.InRange(spatial,0,1e-4);
    }
    private static Vector3D Momentum(P2.PhysicsWorld2D w,P2.Rod rod)
    {
        Vector3D p=default;var store=w.Core.Bodies;
        for(int i=0;i<store.HighWater;i++)if(store.IsLive(i)&&store.IsDynamic(i)) {var v=store.LinearVelocityOf(i);p+=new Vector3D(v[0],v[1],0)/store.InverseMass[i];}
        var d=rod.Particles;for(int i=0;i<rod.ParticleCount;i++){var v=rod.GetVelocity(i);p+=new Vector3D(v.X,v.Y,0)/d.InverseMass[i];}
        return p;
    }
    [Fact]
    public void ForceBreakDestroyedGenerationAndSnapshotFork()
    {
        var w=new P3.PhysicsWorld3D(new(){Gravity=default});var body=w.AddSphere(.1f,at:new(0,2,0));
        var rod=w.AddRope(default,new(1,0,0),4,pinStart:false);rod.Attach(0,body,breakForce:1);
        w.Step(1f/60);Assert.True(rod.IsAttachmentBroken(0));
        rod.Attach(0,body);var bytes=w.Snapshot();var copy=w.Fork();Assert.Equal(bytes,copy.Snapshot());
        w.Step(1f/60);copy.Step(1f/60);Assert.Equal(w.ComputeHash(),copy.ComputeHash());
        w.Restore(bytes);body.Destroy();var replacement=w.AddSphere(.1f,at:new(0,2,0));
        w.Step(1f/60);Assert.True(rod.IsAttachmentBroken(0));Assert.True(replacement.IsValid);
    }
}
