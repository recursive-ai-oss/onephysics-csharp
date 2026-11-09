using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fields;
using P2=RecursiveAI.OnePhysics.Physics2D;
using P3=RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

/// <summary>Field results that are not finite or are beyond the field limit are dropped with a count instead of throwing in the middle of a step.</summary>
public sealed class FieldStepRobustnessTests
{
    [Theory]
    [InlineData(float.NaN)]
    [InlineData(float.PositiveInfinity)]
    [InlineData(1e13f)]
    public void GravityCallbackGarbageIsSkippedAndCounted3D(float bad)
    {
        var w=new P3.PhysicsWorld3D(new(){SleepEnabled=false});var plain=new P3.PhysicsWorld3D(new(){SleepEnabled=false});
        var a=w.AddSphere(.5f,at:new(0,10,0));var b=w.AddSphere(.5f,at:new(3,10,0));var pa=plain.AddSphere(.5f,at:new(0,10,0));
        w.AddField(new(){Kind=FieldKind.CustomGravity,CallbackId=1});w.BindGravityField(1,(p,t)=>p.X>1?new(0,bad,0):new(0,-5,0));
        var before=w.ComputeHash();
        w.Step(1f/60);plain.Step(1f/60);
        Assert.NotEqual(before,w.ComputeHash());
        Assert.Equal(1,w.Counters.FieldApplicationsSkipped);
        // The skipped body fell under the world's own gravity, the other under the callback's.
        Assert.Equal(pa.LinearVelocity.Y,b.LinearVelocity.Y);Assert.NotEqual(pa.LinearVelocity.Y,a.LinearVelocity.Y);
        w.Step(1f/60);Assert.Equal(1,w.Counters.FieldApplicationsSkipped);
    }
    [Fact]
    public void GravityCallbackGarbageIsSkippedAndCounted2D()
    {
        var w=new P2.PhysicsWorld2D(new(){SleepEnabled=false});var plain=new P2.PhysicsWorld2D(new(){SleepEnabled=false});
        var a=w.AddCircle(.5f,at:new(0,10));var b=w.AddCircle(.5f,at:new(3,10));var pb=plain.AddCircle(.5f,at:new(3,10));
        w.AddField(new(){Kind=FieldKind.CustomGravity,CallbackId=1});w.BindGravityField(1,(p,t)=>p.X>1?new(0,float.NaN,0):new(0,-5,0));
        w.Step(1f/60);plain.Step(1f/60);
        Assert.Equal(1,w.Counters.FieldApplicationsSkipped);Assert.Equal(pb.LinearVelocity.Y,b.LinearVelocity.Y);Assert.NotEqual(pb.LinearVelocity.Y,a.LinearVelocity.Y);
    }
    // Strength 1e12 over a softening of 1e-30 is a valid definition whose result at the centre overflows.
    [Fact]
    public void OverflowingFalloffAtTheCentreIsSkipped()
    {
        var w=new P3.PhysicsWorld3D(new(){SleepEnabled=false});var body=w.AddSphere(.5f,at:new(0,0,0));
        w.AddField(new(){Kind=FieldKind.RadialGravity,Center=default,Strength=1e12f,Softening=1e-30f,Radius=10,Falloff=FieldFalloff.InverseSquare});
        w.Step(1f/60);
        Assert.Equal(1,w.Counters.FieldApplicationsSkipped);Assert.True(float.IsFinite(body.LinearVelocity.Y));
        var w2=new P2.PhysicsWorld2D(new(){SleepEnabled=false});w2.AddCircle(.5f,at:new(0,0));
        w2.AddField(new(){Kind=FieldKind.RadialGravity,Center=default,Strength=1e12f,Softening=1e-30f,Radius=10,Falloff=FieldFalloff.InverseSquare});
        w2.Step(1f/60);Assert.Equal(1,w2.Counters.FieldApplicationsSkipped);
    }
    [Fact]
    public void WorldWithNoFieldReportsZero()
    {
        var w=new P3.PhysicsWorld3D();w.AddSphere(.5f,at:new(0,3,0));w.Step(1f/60);Assert.Equal(0,w.Counters.FieldApplicationsSkipped);
    }
}
