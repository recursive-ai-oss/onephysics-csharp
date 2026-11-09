using System;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Fields;
using RecursiveAI.OnePhysics.Replay;
using P2=RecursiveAI.OnePhysics.Physics2D;
using P3=RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class FieldsTests
{
    [Fact]
    public void AreaAccelerationAndPointFalloffsAreAnalyticInBothDimensions()
    {
        foreach(var falloff in new[]{FieldFalloff.Constant,FieldFalloff.InverseLinear,FieldFalloff.InverseSquare})
        {
            var a=new P2.PhysicsWorld2D(new(){Gravity=default,SleepEnabled=false});
            var b=new P3.PhysicsWorld3D(new(){Gravity=default,SleepEnabled=false});
            var x=a.AddCircle(0.1f,at:new(2,0));var y=b.AddSphere(0.1f,at:new(2,0,0));
            a.AddPointEffector(default,8,10,falloff);b.AddPointEffector(default,8,10,falloff);
            a.Step(0.001f);b.Step(0.001f);
            float expected=(falloff==FieldFalloff.Constant?8:falloff==FieldFalloff.InverseLinear?4:2)*0.001f;
            Assert.InRange(x.LinearVelocity.X,expected-1e-6f,expected+1e-6f);Assert.InRange(y.LinearVelocity.X,expected-1e-6f,expected+1e-6f);
        }
        var w=new P2.PhysicsWorld2D(new(){Gravity=default});var body=w.AddBox(new(1,1));
        w.AddAreaEffector(new(2,3),new(-10,-10),new(10,10));w.Step(0.5f);
        Assert.InRange(body.LinearVelocity.X,0.9999f,1.0001f);Assert.InRange(body.LinearVelocity.Y,1.4999f,1.5001f);
    }
    [Fact]
    public void GravityZonesSelectHighestPriorityAndNormalizeEqualPriorityWeights()
    {
        var w=new P3.PhysicsWorld3D(new(){Gravity=new(0,-10,0)});var body=w.AddSphere(0.1f);
        w.AddGravityZone(new(100,0,0),new(-10,-10,-10),new(10,10,10));
        w.AddGravityZone(new(2,0,0),new(-10,-10,-10),new(10,10,10),1,1);
        w.AddGravityZone(new(6,0,0),new(-10,-10,-10),new(10,10,10),1,3);
        w.Step(0.1f);Assert.InRange(body.LinearVelocity.X,0.49999f,0.50001f);Assert.InRange(body.LinearVelocity.Y,-1e-6f,1e-6f);
    }
    [Fact]
    public void RadialInverseSquareGravityKeepsACircularOrbitForOneThousandSteps()
    {
        var w=new P3.PhysicsWorld3D(new(){Gravity=default,SleepEnabled=false});var body=w.AddSphere(0.1f,at:new(10,0,0));
        body.LinearVelocity=new(0,1,0);w.AddRadialGravity(default,10,100);
        for(int i=0;i<1000;i++)w.Step(0.01f);
        var p=body.Position;double r=Math.Sqrt(p.X*p.X+p.Y*p.Y+p.Z*p.Z);Assert.InRange(r,9.98,10.02);
    }
    [Fact]
    public void VortexAndTaggedDipolesHaveTheDocumentedDirections()
    {
        var w=new P2.PhysicsWorld2D(new(){Gravity=default,SleepEnabled=false});var x=w.AddCircle(0.1f,at:new(2,0));
        w.AddVortex(default,4,2,10);w.Step(0.01f);Assert.InRange(x.LinearVelocity.Y,0.0399f,0.0401f);Assert.True(x.LinearVelocity.X<0);
        var m=new P3.PhysicsWorld3D(new(){Gravity=default,SleepEnabled=false});var a=m.AddSphere(0.1f,at:new(-1,0,0));var b=m.AddSphere(0.1f,at:new(1,0,0));var untagged=m.AddSphere(0.1f,at:new(0,3,0));
        a.MagneticMoment=new(1,0,0);b.MagneticMoment=new(1,0,0);m.AddMagnetism(0.0001f,5);m.Step(0.01f);
        Assert.True(a.LinearVelocity.X>0);Assert.True(b.LinearVelocity.X<0);Assert.Equal(default,untagged.LinearVelocity);
    }
    [Fact]
    public void ExplosionIsBlockedByStaticWallAndVisibleImpulseIsAnalytic()
    {
        var w=new P2.PhysicsWorld2D(new(){Gravity=default});var hidden=w.AddCircle(0.2f,at:new(4,0));var visible=w.AddCircle(0.2f,at:new(0,4));
        w.AddBox(new(0.2f,3),at:new(2,0),type:BodyType.Static);w.Explode(default,2,10);
        Assert.Equal(default,hidden.LinearVelocity);Assert.InRange(visible.LinearVelocity.Y,2/visible.Mass-1e-4f,2/visible.Mass+1e-4f);
        var s=new P3.PhysicsWorld3D(new(){Gravity=default});var a=s.AddSphere(0.2f,at:new(4,0,0));var b=s.AddSphere(0.2f,at:new(0,4,0));s.AddBox(new(0.2f,3,3),at:new(2,0,0),type:BodyType.Static);s.Explode(default,2,10);
        Assert.Equal(default,a.LinearVelocity);Assert.InRange(b.LinearVelocity.Y,2/b.Mass-1e-4f,2/b.Mass+1e-4f);
    }
    [Fact]
    public void PolygonAndCircleClipsHaveAnalyticAreaAndCentroid()
    {
        var box=Shape2D.CreateBox(1,1);var cut=P2.Buoyancy2D.ComputeSubmergedArea(in box,new(0,1),0);
        Assert.Equal(2,cut.Area);Assert.Equal(new Vector2F(0,-0.5f),cut.Centroid);
        var circle=Shape2D.CreateCircle(default,1);cut=P2.Buoyancy2D.ComputeSubmergedArea(in circle,new(0,1),0);
        Assert.InRange(cut.Area,3.1415926f/2-1e-6f,3.1415926f/2+1e-6f);Assert.InRange(cut.Centroid.Y,-4f/(3*3.1415926f)-1e-6f,-4f/(3*3.1415926f)+1e-6f);
    }
    [Fact]
    public void BuoyancyFindsTheAnalyticDraftAndRightsARotatedBox()
    {
        var w=new P2.PhysicsWorld2D(new(){Gravity=new(0,-10),SleepEnabled=false});var body=w.AddBox(new(2,1),at:new(0,0.1),angleRadians:0.2f,density:500);
        w.AddBuoyancyEffector(0,damping:3);
        for(int i=0;i<1800;i++)w.Step(1f/120);
        Assert.InRange(body.Position.Y,-0.005,0.005);Assert.InRange(body.Angle,-0.01f,0.01f);Assert.InRange(body.LinearVelocity.Y,-0.01f,0.01f);
    }
    [Fact]
    public void FieldsRoundTripThroughSnapshotForkSceneAndReplay()
    {
        var w=new P2.PhysicsWorld2D(new(){WorldId=701,Gravity=default,SleepEnabled=false});var body=w.AddCircle(0.2f,at:new(1,0));body.MagneticMoment=new(1,0);
        var field=w.AddPointEffector(default,-1,10);var snap=w.Snapshot();var fork=w.Fork();
        Assert.Equal(snap,fork.Snapshot());Assert.Equal(snap,P2.PhysicsWorld2D.LoadScene(new StringReader(w.SaveScene())).Snapshot());
        for(int i=0;i<10;i++){w.Step(0.01f);fork.Step(0.01f);Assert.Equal(w.ComputeHash(),fork.ComputeHash());}
        w.Restore(snap);Assert.Equal(snap,w.Snapshot());
        using var stream=new MemoryStream();using(var recorder=w.StartRecording(stream)) {field.Definition=new(){Kind=FieldKind.Area,Direction=new(1,0,0)};for(int i=0;i<10;i++)w.Step(0.01f);w.Explode(default,0.01f,10,false);}
        stream.Position=0;var player=new ReplayPlayer(stream);var playback=player.CreateWorld2D();while(player.ReadNextStep(playback)){}Assert.Equal(w.ComputeHash(),playback.ComputeHash());
        var g=new DebugGeometry();w.GetDebugGeometry(DebugContent.Fields,g);Assert.Single(g.Fields.ToArray());
        field.Destroy();Assert.Throws<InvalidOperationException>(()=>field.Definition);Assert.False(field.IsValid);
        var restored=new P2.PhysicsWorld2D(new(){WorldId=701});restored.Restore(snap);Assert.Equal(snap,restored.Snapshot());
    }
    [Fact]
    public void CustomGravityBindingsAreExplicitHostContext()
    {
        var w=new P3.PhysicsWorld3D(new(){Gravity=default});var body=w.AddSphere(0.1f);
        w.AddField(new(){Kind=FieldKind.CustomGravity,CallbackId=7});w.BindGravityField(7,static (position,tick)=>new(0,2,0));w.Step(0.1f);
        Assert.InRange(body.LinearVelocity.Y,0.19999f,0.20001f);
        Assert.Throws<InvalidOperationException>(()=>w.Fork().Step(0.1f));
    }
    [Fact]
    public void HostileDefinitionsAreRejectedBeforeMutationAndLayersExcludeBodies()
    {
        var w=new P2.PhysicsWorld2D(new(){Gravity=default});var before=w.Snapshot();
        Assert.Throws<ArgumentOutOfRangeException>(()=>w.AddField(new(){Strength=float.NaN}));Assert.Equal(before,w.Snapshot());
        Assert.Throws<ArgumentOutOfRangeException>(()=>w.AddField(new(){Softening=0}));
        var body=w.AddCircle(0.1f);w.AddField(new(){Direction=new(10,0,0),Layers=0});w.Step(0.1f);Assert.Equal(default,body.LinearVelocity);
    }

    [Fact]
    public void PlatformAllowsPassageFromBelowAndConveyorDrivesOnlyContactingBodies()
    {
        var w=new P2.PhysicsWorld2D(new(){Gravity=default,SleepEnabled=false});
        w.AddBox(new(10,0.2f),type:BodyType.Static);w.AddPlatformEffector(new(0,1),new(-10,-1),new(10,1));
        var body=w.AddCircle(0.2f,at:new(0,-0.5));body.LinearVelocity=new(0,2);
        for(int i=0;i<50;i++)w.Step(0.01f);Assert.True(body.Position.Y>0.4,$"platform body y={body.Position.Y} v={body.LinearVelocity.Y}");
        var c=new P2.PhysicsWorld2D();c.AddBox(new(20,1),at:new(0,-0.5),type:BodyType.Static);
        var rider=c.AddBox(new(0.5f,0.5f),at:new(0,0.26));var free=c.AddBox(new(0.5f,0.5f),at:new(5,5));
        c.AddSurfaceEffector(new(1,0),2,new(-20,-1),new(20,0));
        for(int i=0;i<30;i++)c.Step(1f/60);Assert.True(rider.LinearVelocity.X>1,$"conveyor speed={rider.LinearVelocity.X}");Assert.Equal(0,free.LinearVelocity.X);
    }
    [Fact]
    public void SpatialBuoyancyAndFieldOriginShiftRoundTrip()
    {
        var w=new P3.PhysicsWorld3D(new(){Gravity=new(0,-10,0),SleepEnabled=false});var body=w.AddBox(new(2,1,2),at:new(0,0.1,0),density:500);
        w.AddBuoyancyEffector(0,damping:3);
        for(int i=0;i<1000;i++)w.Step(1f/120);Assert.InRange(body.Position.Y,-0.01,0.01);
        w.ShiftOrigin(new(16,-8,2));var snapshot=w.Snapshot();var copy=w.Fork();copy.Restore(snapshot);Assert.Equal(snapshot,copy.Snapshot());
        for(int i=0;i<30;i++){w.Step(1f/120);copy.Step(1f/120);Assert.Equal(w.ComputeHash(),copy.ComputeHash());}
    }
    [Fact]
    public void UnusedFieldsLeaveSnapshotBytesAndHashesAloneAndSteadyStepsAllocateNothing()
    {
        var a=new P2.PhysicsWorld2D(new(){WorldId=98,Gravity=default});a.AddCircle(0.2f);var before=a.Snapshot();
        Assert.Equal(0,a.FieldCount);Assert.Equal(before,a.Snapshot());
        a.AddAreaEffector(new(1,0),new(-100,-100),new(100,100));
        for(int i=0;i<40;i++)a.Step(0.001f);
        long bytes=GC.GetAllocatedBytesForCurrentThread();for(int i=0;i<40;i++)a.Step(0.001f);Assert.Equal(bytes,GC.GetAllocatedBytesForCurrentThread());
    }

    [Fact]
    public void PlanarFluidApiSupportsRawBuoyancyAndExactFactorRoundTrips()
    {
        var w=new P2.PhysicsWorld2D(new(){Gravity=new(0,-10),SleepEnabled=false});var b=w.AddBox(new(2,1),density:500);
        b.BuoyancyFactor=1e-12f;Assert.Equal(1e-12f,b.BuoyancyFactor);var snap=w.Snapshot();w.Restore(snap);Assert.Equal(1e-12f,b.BuoyancyFactor);
        b.BuoyancyFactor=1;var fluid=w.AddWater(0);Assert.Equal(1,w.FluidCount);fluid.Flow=new(1,0);Assert.Equal(new Vector2F(1,0),fluid.Flow);
        var copy=fluid.Definition;copy.SurfaceOffset=0.2;fluid.SetDefinition(copy);Assert.Equal(0.2,fluid.SurfaceOffset);
        w.ShiftOrigin(new(16,-8));fluid.Flow=new(2,0);Assert.Equal(new Vector2F(2,0),fluid.Flow);w.ShiftOrigin(new(-16,8));
        fluid.Remove();Assert.False(fluid.IsValid);Assert.Equal(0,w.FluidCount);
        float area=w.ApplyBuoyancy(b,new P2.FluidDef {LinearDrag=0,AngularDrag=0},0.01f);Assert.Equal(1,area);w.Step(0.01f);Assert.InRange(b.LinearVelocity.Y,-1e-6f,1e-6f);
    }
    [Fact]
    public void SleepingBodyWakesOnlyAboveItsThresholdAndSensorsAreGenerationChecked()
    {
        var w=new P2.PhysicsWorld2D(new(){Gravity=default});var b=w.AddCircle(0.1f);
        b.SleepSettings=new(0.1f,0.1f,0.1f);b.Sleep();
        var field=w.AddAreaEffector(new(0.01f,0),new(-1,-1),new(1,1));w.Step(0.01f);Assert.False(b.IsAwake);
        field.Definition=new(){Direction=new(100,0,0)};w.Step(0.01f);Assert.True(b.IsAwake);Assert.True(b.LinearVelocity.X>0.9f);
        var sensors=new P2.PhysicsWorld2D(new(){Gravity=default});var subject=sensors.AddCircle(0.1f);var owner=sensors.CreateBody(new(){Type=BodyType.Static});
        var sensor=sensors.CreateBox(owner,new(1,1),default,0,0,new P2.ShapeDef{IsSensor=true});
        var selected=sensors.AddField(new(){Direction=new(1,0,0),Sensor=sensor.RawHandle.Index});sensors.Step(0.01f);Assert.Equal(0,subject.LinearVelocity.X);
        sensors.Step(0.01f);Assert.True(subject.LinearVelocity.X>0);sensor.Destroy();subject.LinearVelocity=default;sensors.Step(0.01f);Assert.Equal(0,subject.LinearVelocity.X);
        Assert.True(selected.Definition.SensorGeneration>0);
    }
}
