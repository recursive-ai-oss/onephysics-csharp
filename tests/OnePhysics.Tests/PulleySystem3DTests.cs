using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Cables;
using P3=RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class PulleySystem3DTests(ITestOutputHelper helper)
{
    private readonly Action<string> _log=s=>{helper.WriteLine(s);var f=Environment.GetEnvironmentVariable("PULLEY_LOG");if(f!=null)System.IO.File.AppendAllText(f,s+"\n");};
    private const float Dt=1f/60;

    [Fact]
    public void Atwood3D()
    {
        var w=new P3.PhysicsWorld3D(new(){SleepEnabled=false});
        var a=w.AddBox(new(.4f,.4f,.4f),at:new(-.25,3,0),density:31.25f);
        var b=w.AddBox(new(.4f,.4f,.4f),at:new(.25,3.5,0),density:15.625f);
        var rope=w.AddRopeOverPulleys(P3.RopeEnd.On(a,default),[new P3.Pulley(new(0,5,0),.25f,clockwise:true,inertia:1e-4f)],P3.RopeEnd.On(b,default));
        double total=rope.TotalLength;
        for(int i=0;i<30;i++)w.Step(Dt);
        double expected=(a.Mass-b.Mass)/(a.Mass+b.Mass)*9.81,measured=-a.LinearVelocity.Y/(30*Dt),tension=2*a.Mass*b.Mass/(a.Mass+b.Mass)*9.81;
        _log($"3D Atwood a={measured:F5} (exact {expected:F5}), T={rope.Tension(0):F4} (exact {tension:F4}), masses {a.Mass} {b.Mass}");
        Assert.InRange(measured/expected,.99,1.01);Assert.InRange(rope.Tension(0)/tension,.99,1.01);Assert.InRange(rope.TotalLength-total,-1e-9,1e-9);
    }

    // The rope leaves the wheel's plane (a fleet angle): the load still hangs from it at rest with the right tension.
    [Fact]
    public void FleetAngleHoldsALoad()
    {
        var w=new P3.PhysicsWorld3D(new(){SleepEnabled=false});
        var load=w.AddBox(new(.5f,.5f,.5f),at:new(-.3,2,0),density:80);   // 10 kg, hanging straight below the entry tangent
        var rope=w.AddRopeOverPulleys(P3.RopeEnd.On(load,default),[new P3.Pulley(new(0,5,0),.3f,clockwise:true,friction:0.2f,inertia:float.PositiveInfinity)],P3.RopeEnd.At(new(1.5,5,-.8)));
        for(int i=0;i<300;i++)w.Step(Dt);
        double t=rope.Tension(0);
        _log($"fleet angle: load drift speed {load.LinearVelocity.Length():E2} m/s, tension {t:F4} (weight {load.Mass*9.81:F4})");
        Assert.InRange(load.LinearVelocity.Length(),0,1e-3);Assert.InRange(t/(load.Mass*9.81),.99,1.05);
    }

    // The common tangent of two circles with skew axes: both tangent points lie on their circles and the rope direction is perpendicular to both radii.
    [Fact]
    public void SkewAxisTangentsTouchBothRims()
    {
        var rng=new Random(7);
        int ok=0;double worst=0;
        for(int t=0;t<200;t++)
        {
            Vector3D V()=>new(rng.NextDouble()*4-2,rng.NextDouble()*4-2,rng.NextDouble()*4-2);
            var ca=V();var cb=V()+new Vector3D(6,0,0);var na=V().Normalized();var nb=V().Normalized();
            double ra=.2+rng.NextDouble()*.4,rb=.2+rng.NextDouble()*.4;
            if(!CableGeometry.WheelToWheel(ca,na,ra,1,cb,nb,rb,-1,out var pa,out var pb))continue;
            var d=(pb-pa).Normalized();
            double e1=Math.Abs(Vector3D.Dot(d,(pa-ca)/ra)),e2=Math.Abs(Vector3D.Dot(d,(pb-cb)/rb));
            double r1=Math.Abs((pa-ca).Length()-ra),r2=Math.Abs((pb-cb).Length()-rb),p1=Math.Abs(Vector3D.Dot(pa-ca,na)),p2=Math.Abs(Vector3D.Dot(pb-cb,nb));
            worst=Math.Max(worst,Math.Max(Math.Max(e1,e2),Math.Max(Math.Max(r1,r2),Math.Max(p1,p2))));ok++;
        }
        _log($"skew tangents: {ok} of 200 pairs solved, worst tangency/on-rim error {worst:E2}");
        Assert.True(ok>150);Assert.InRange(worst,0,1e-6);
    }

    private static P3.PhysicsWorld3D Machines(int threads,out List<P3.PulleySystem> systems,out List<P3.Body> watch)
    {
        var w=new P3.PhysicsWorld3D(new(){WorldId=77,ThreadCount=threads});
        systems=[];watch=[];
        var ground=w.AddBox(new(30,1,30),at:new(0,-.5,0),type:BodyType.Static);
        // a 4:1 tackle
        const float R=.1f;
        var block=w.AddBox(new(.9f,.3f,.3f),at:new(2*R,3,0),density:100);
        P3.Pulley Movable(double x)=>new(new(x-2*R,0,0),R,block,clockwise:false,inertia:1e-3f);
        P3.Pulley Fixed(double x)=>new(new(x,6,0),R,clockwise:true,inertia:1e-3f);
        var haul=w.AddBox(new(.1f,.1f,.1f),at:new(7*R,2,0),density:500);
        systems.Add(w.AddRopeOverPulleys(P3.RopeEnd.At(new(-R,6,0)),[Movable(0),Fixed(2*R),Movable(4*R),Fixed(6*R)],P3.RopeEnd.On(haul,default)));
        watch.Add(block);watch.Add(haul);
        // a winch lifting a crate over a friction pulley
        var crate=w.AddBox(new(.5f,.5f,.5f),at:new(5,1,5),density:80);
        var winch=w.AddWinch(default,new(8,6,5),.2f,P3.RopeEnd.On(crate,default),drumInertia:.5f,maxTorque:200);
        winch.SetMotor(0,-3,200);
        systems.Add(winch);watch.Add(crate);
        // a chain drive between two hinged sprockets
        var s1=w.AddCylinder(.12f,.04f,at:new(-5,2,-4),density:200);var s2=w.AddCylinder(.2f,.04f,at:new(-4.5,2,-4),density:200);
        w.AddHinge(ground,s1,new(-5,2,-4),new(0,0,1));w.AddHinge(ground,s2,new(-4.5,2,-4),new(0,0,1));
        systems.Add(w.AddChainDrive([new P3.Sprocket(s1,default,15),new P3.Sprocket(s2,default,25)],.03f));
        watch.Add(s1);watch.Add(s2);
        return w;
    }

    [Fact]
    public void ThreadCountsAndRestoreAgree()
    {
        var one=Machines(1,out var sysOne,out var wOne);var eight=Machines(8,out _,out var wEight);
        for(int i=0;i<100;i++)
        {
            wOne[1].ApplyForce(new(0,-30,0));wEight[1].ApplyForce(new(0,-30,0));wOne[3].ApplyTorque(new(0,0,.2f));wEight[3].ApplyTorque(new(0,0,.2f));
            one.Step(Dt);eight.Step(Dt);
            Assert.True(one.ComputeHash()==eight.ComputeHash(),$"hash differs at step {i}");
        }
        var snap=one.Snapshot();var fork=one.Fork();
        var other=new P3.PhysicsWorld3D(new(){WorldId=77,ThreadCount=3});other.Restore(snap);
        Assert.Equal(one.ComputeHash(),other.ComputeHash());Assert.Equal(snap,other.Snapshot());
        Assert.Equal(sysOne.Count,other.PulleySystemCount);
        for(int k=0;k<200;k++)
        {
            one.Step(Dt);eight.Step(Dt);other.Step(Dt);fork.Step(Dt);
            Assert.True(one.ComputeHash()==eight.ComputeHash()&&one.ComputeHash()==other.ComputeHash()&&one.ComputeHash()==fork.ComputeHash(),$"hash differs {k} steps after the snapshot");
        }
        Assert.Equal(one.Snapshot(),other.Snapshot());
        _log($"3D machines: {sysOne.Count} systems, 1 vs 8 threads, restore and fork equal over 300 steps, hash {one.ComputeHash():X16}, snapshot {snap.Length} bytes");
    }

    private static void Expect<T>(int index,Action a) where T:Exception
    {
        try{a();}catch(T){return;}catch(Exception e){throw new Xunit.Sdk.XunitException($"case {index}: expected {typeof(T).Name}, got {e.GetType().Name}: {e.Message}");}
        throw new Xunit.Sdk.XunitException($"case {index}: nothing thrown");
    }

    [Fact]
    public void HostileDefinitionsAndStaleHandlesFailClearly()
    {
        var w=new P3.PhysicsWorld3D(new(){SleepEnabled=false});
        var a=w.AddBox(new(.4f,.4f,.4f),at:new(0,3,0));
        P3.PulleyNodeDef Wheel(float r=.2f)=>new(){Kind=CableNodeKind.Wheel,Point=new(0,5,0),Radius=r};
        var end=P3.PulleyNodeDef.Anchor(a.RawHandle,default);
        Expect<ArgumentOutOfRangeException>(1,()=>w.AddPulleySystem(new(){Nodes=[end]}));
        Expect<ArgumentException>(2,()=>w.AddPulleySystem(new(){Nodes=[Wheel(),Wheel()]}));
        Expect<ArgumentOutOfRangeException>(3,()=>w.AddPulleySystem(new(){Nodes=[end,Wheel(0),end]}));
        Expect<ArgumentOutOfRangeException>(4,()=>w.AddPulleySystem(new(){Nodes=[end,Wheel(float.NaN),end]}));
        Expect<ArgumentOutOfRangeException>(5,()=>w.AddPulleySystem(new(){Nodes=[P3.PulleyNodeDef.Anchor(a.RawHandle,new(double.NaN,0,0)),Wheel(),end]}));
        Expect<ArgumentOutOfRangeException>(6,()=>w.AddPulleySystem(new(){Nodes=[end,new P3.PulleyNodeDef{Kind=CableNodeKind.Wheel,Point=new(0,5,0),Radius=.2f,Axis=default},end]}));
        Expect<ArgumentException>(7,()=>w.AddPulleySystem(new(){Nodes=[end,new P3.PulleyNodeDef{Kind=CableNodeKind.Wheel,Point=new(0,3.05,0),Radius=1},end]}));   // the rope end lies inside the wheel
        Expect<ArgumentOutOfRangeException>(8,()=>w.AddPulleySystem(new(){Nodes=[end,Wheel(),P3.PulleyNodeDef.Anchor(default,new(3,5,0))],Length=1}));   // shorter than its path
        Expect<ArgumentException>(9,()=>w.AddPulleySystem(new(){Nodes=[end,new P3.PulleyNodeDef{Kind=CableNodeKind.Wheel,Spin=WheelSpin.Body,MotorTorque=1,Body=a.RawHandle,Point=new(0,5,0),Radius=.2f},end]}));
        var other=new P3.PhysicsWorld3D();var foreign=other.AddBox(new(1,1,1));
        Expect<ArgumentException>(10,()=>w.AddPulleySystem(new(){Nodes=[P3.PulleyNodeDef.Anchor(foreign.RawHandle,default),Wheel(),end]}));
        Assert.Equal(0,w.PulleySystemCount);
        var rope=w.AddRopeOverPulleys(P3.RopeEnd.On(a,default),[new P3.Pulley(new(0,5,0),.2f)],P3.RopeEnd.At(new(2,5,0)));
        Assert.True(rope.IsValid);Assert.Throws<ArgumentOutOfRangeException>(()=>rope.Tension(9));Assert.Throws<ArgumentException>(()=>rope.Wrap(0));
        Assert.Throws<ArgumentOutOfRangeException>(()=>{rope.BreakForce=-1;});Assert.Throws<ArgumentException>(()=>rope.SetMotor(0,1,1));
        rope.Destroy();Assert.False(rope.IsValid);Assert.Throws<InvalidOperationException>(()=>rope.Tension(0));Assert.Throws<InvalidOperationException>(rope.Destroy);
        Assert.Throws<InvalidOperationException>(()=>default(P3.PulleySystem).TotalLength);
        // a destroyed carrier breaks its system instead of failing the step
        var rope2=w.AddRopeOverPulleys(P3.RopeEnd.On(a,default),[new P3.Pulley(new(0,5,0),.2f)],P3.RopeEnd.At(new(2,5,0)));
        a.Destroy();w.Step(Dt);Assert.True(rope2.IsBroken);
    }

    private static (P3.PhysicsWorld3D World,P3.Body Load,P3.PulleySystem Rope) Hanging(int id=91,double x=-.3,double slack=.3)
    {
        var w=new P3.PhysicsWorld3D(new(){WorldId=id,SleepEnabled=false});
        var load=w.AddBox(new(.4f,.4f,.4f),at:new(x,2,0),density:100);   // 6.4 kg
        var rope=w.AddRopeOverPulleys(P3.RopeEnd.On(load,default),[new P3.Pulley(new(0,5,0),.3f,clockwise:true,friction:.2f)],P3.RopeEnd.At(new(1.5,5,0)),slack:slack);
        return (w,load,rope);
    }

    [Fact]
    public void BreakingOnTensionReleasesTheLoadForGood()
    {
        var (w,load,rope)=Hanging(91,-.3,0);
        rope.BreakForce=80;   // the load weighs 62.8 N
        for(int i=0;i<60;i++)w.Step(Dt);
        Assert.False(rope.IsBroken);Assert.InRange(rope.MaximumTension,62,64);
        load.ApplyImpulse(new(0,-40,0));   // a jerk
        w.Step(Dt);
        _log($"break: tension at the jerk {rope.MaximumTension:F1} against 80, broken {rope.IsBroken}");
        Assert.True(rope.IsBroken);Assert.Equal(0,rope.Tension(0));
        double y=load.Position.Y,v=load.LinearVelocity.Y;for(int i=0;i<30;i++)w.Step(Dt);
        Assert.True(load.Position.Y<y-1);   // free fall: a broken system exerts nothing
        Assert.True(rope.IsBroken);
    }

    [Fact]
    public void DebugGeometryReportsSpansAndWheels()
    {
        var (w,load,rope)=Hanging();
        for(int i=0;i<60;i++)w.Step(Dt);
        var g=new RecursiveAI.OnePhysics.Debug.DebugGeometry();
        w.GetDebugGeometry(RecursiveAI.OnePhysics.Debug.DebugContent.Pulleys,g);
        Assert.Equal(2,g.PulleySpans.Length);Assert.Single(g.PulleyWheels.ToArray());
        var wheel=g.PulleyWheels[0];
        _log($"debug: span0 {g.PulleySpans[0].Start} -> {g.PulleySpans[0].End} T={g.PulleySpans[0].Tension:F2}, wheel wrap {wheel.Wrap:F4}, radius {wheel.Radius}");
        Assert.InRange(wheel.Wrap,1.5,3.2);Assert.True((wheel.Enter-wheel.Center).Length()>.29&&(wheel.Enter-wheel.Center).Length()<.31);
        Assert.InRange(g.PulleySpans[0].Tension,60,64);
    }

    [Fact]
    public void ShiftedWorldStepsBitIdentically()
    {
        var (a,la,ra)=Hanging(92,-.25);var (b,lb,rb)=Hanging(92,-.25);
        for(int i=0;i<40;i++){a.Step(Dt);b.Step(Dt);}
        b.ShiftOrigin(new(16,-8,2));
        for(int i=0;i<200;i++){la.ApplyForce(new(1,0,.5f));lb.ApplyForce(new(1,0,.5f));a.Step(Dt);b.Step(Dt);}
        var pa=a.Core.Bodies.PositionOf(la.RawHandle.Index);var pb=b.Core.Bodies.PositionOf(lb.RawHandle.Index);
        _log($"shift: unshifted {pa[0]:R} {pa[1]:R} {pa[2]:R}, shifted+offset {pb[0]+16:R} {pb[1]-8:R} {pb[2]+2:R}");
        Assert.Equal(pa[0],pb[0]+16);Assert.Equal(pa[1],pb[1]-8);Assert.Equal(pa[2],pb[2]+2);
        Assert.Equal(ra.Tension(0),rb.Tension(0));
    }

    [Fact]
    public void RecordedSessionReplaysAndForkKeepsSystems()
    {
        var w=new P3.PhysicsWorld3D(new(){WorldId=93,SleepEnabled=false});
        var load=w.AddBox(new(.4f,.4f,.4f),at:new(-.3,2,0),density:100);
        var hashes=new ulong[60];using var stream=new System.IO.MemoryStream();
        P3.PulleySystem rope=default;
        using(var recorder=w.StartRecording(stream))
        {
            for(int i=0;i<hashes.Length;i++)
            {
                if(i==5)rope=w.AddRopeOverPulleys(P3.RopeEnd.On(load,default),[new P3.Pulley(new(0,5,0),.3f,clockwise:true,friction:.2f)],P3.RopeEnd.At(new(1.5,5,0)),slack:.5);
                if(i==30)rope.AdjustLength(-.2);
                if(i==40)rope.BreakForce=5000;
                w.Step(Dt);hashes[i]=w.ComputeHash();
            }
        }
        stream.Position=0;var player=new RecursiveAI.OnePhysics.Replay.ReplayPlayer(stream);var copy=player.CreateWorld3D();
        int n=0;while(player.ReadNextStep(copy))Assert.Equal(hashes[n++],copy.ComputeHash());
        Assert.Equal(hashes.Length,n);Assert.Equal(w.Snapshot(),copy.Snapshot());Assert.Equal(1,copy.PulleySystemCount);
        var fork=w.Fork();Assert.Equal(1,fork.PulleySystemCount);Assert.Equal(w.ComputeHash(),fork.ComputeHash());
    }
}
