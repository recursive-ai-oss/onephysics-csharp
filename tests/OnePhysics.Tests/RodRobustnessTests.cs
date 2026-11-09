using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Rods;
using P2=RecursiveAI.OnePhysics.Physics2D;
using P3=RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class RodRobustnessTests(ITestOutputHelper output)
{
    // A rope whose pinned end sits on a static box's top face (or inside it) used to throw a particle kilometres away: the contact
    // at the pinned end has almost no weight, so undoing its penetration needed an unbounded move of the free end.
    [Theory]
    [InlineData(0.0)]    // pinned end exactly on the box's top face
    [InlineData(-0.2)]   // pinned end inside the box
    [InlineData(0.05)]   // pinned end just above (control)
    public void PinnedRopeTouchingStaticBoxStaysBounded3D(double pinHeight)
    {
        var w=new P3.PhysicsWorld3D(new(){});
        w.AddBox(new(4f,1f,4f),at:new(0,-.5,0),type:BodyType.Static);
        var rod=w.AddRope(new(0,pinHeight,0),new(1.5,pinHeight,0),12,pinStart:true);
        double high=-1e9,low=1e9,reach=0,speed=0;
        for(int i=0;i<480;i++)
        {
            w.Step(1f/60);
            for(int k=0;k<=12;k++){var p=rod.GetPosition(k);high=Math.Max(high,p.Y);low=Math.Min(low,p.Y);reach=Math.Max(reach,Math.Max(Math.Abs(p.X),Math.Abs(p.Z)));}
        }
        for(int k=1;k<=12;k++){var v=rod.GetVelocity(k);speed=Math.Max(speed,Math.Sqrt(v.X*v.X+v.Y*v.Y+v.Z*v.Z));}
        output.WriteLine($"height {low}..{high}, horizontal reach {reach}, final speed {speed}");
        Assert.InRange(high,-1,pinHeight+2);Assert.InRange(reach,0,2);Assert.InRange(speed,0,1);
    }
    [Theory]
    [InlineData(0.0)]
    [InlineData(-0.2)]
    [InlineData(0.05)]
    public void PinnedRopeTouchingStaticBoxStaysBounded2D(double pinHeight)
    {
        var w=new P2.PhysicsWorld2D(new(){});
        w.AddBox(new(4f,1f),at:new(0,-.5),type:BodyType.Static);
        var rod=w.AddRope(new(0,pinHeight),new(1.5,pinHeight),12,pinStart:true);
        double high=-1e9,low=1e9,reach=0,speed=0;
        for(int i=0;i<480;i++)
        {
            w.Step(1f/60);
            for(int k=0;k<=12;k++){var p=rod.GetPosition(k);high=Math.Max(high,p.Y);low=Math.Min(low,p.Y);reach=Math.Max(reach,Math.Abs(p.X));}
        }
        for(int k=1;k<=12;k++){var v=rod.GetVelocity(k);speed=Math.Max(speed,Math.Sqrt(v.X*v.X+v.Y*v.Y));}
        output.WriteLine($"height {low}..{high}, horizontal reach {reach}, final speed {speed}");
        Assert.InRange(high,-1,pinHeight+2);Assert.InRange(reach,0,2);Assert.InRange(speed,0,1);
    }
}
