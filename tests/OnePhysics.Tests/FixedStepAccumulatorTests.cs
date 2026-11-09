using System;
using RecursiveAI.OneMath;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class FixedStepAccumulatorTests
{
    [Fact]
    public void RecordedFrameSequenceStepsExactlyAndCarriesCappedBacklog()
    {
        var a = new P2.PhysicsWorld2D(new() { Gravity = default });
        var b = new P3.PhysicsWorld3D(new() { Gravity = default });
        var x = new P2.FixedStepAccumulator(a, 0.125f, 2); var y = new P3.FixedStepAccumulator(b, 0.125f, 2);
        double[] frames = [0, 0.0625, 0.0625, 0.5, 0, 0.03125]; int[] expected = [0, 0, 1, 2, 2, 0];
        for (int i = 0; i < frames.Length; i++) { Assert.Equal(expected[i], x.Advance(frames[i])); Assert.Equal(expected[i], y.Advance(frames[i])); }
        Assert.Equal(5, a.StepCount); Assert.Equal(5, b.StepCount);
        Assert.Equal(0.03125, x.Remainder); Assert.Equal(x.Remainder, y.Remainder); Assert.Equal(0.25, x.InterpolationAlpha);
        Assert.Equal(2, x.Advance(0.75)); Assert.Equal(1, x.InterpolationAlpha);
        Assert.Equal(0.53125, x.Remainder);
    }

    [Fact]
    public void PositionIsLerpAndOrientationUsesShortestArcWithoutMutatingWorld()
    {
        var a = new P2.PhysicsWorld2D(new() { Gravity = default, MaxRotationPerSubstep = 1 });
        var b = new P3.PhysicsWorld3D(new() { Gravity = default, MaxRotationPerSubstep = 1 });
        var x = a.CreateBody(new() { LinearVelocity = new(2, 0), AngularVelocity = 1, AllowSleep = false });
        var y = b.CreateBody(new() { LinearVelocity = new(2, 0, 0), AngularVelocity = new(0, 0, 1), AllowSleep = false });
        var f = new P2.FixedStepAccumulator(a, 0.125f); var g = new P3.FixedStepAccumulator(b, 0.125f);
        Assert.Equal(1, f.Advance(0.1875)); Assert.Equal(1, g.Advance(0.1875));
        Assert.Equal(0.5, f.InterpolationAlpha); Assert.Equal(0.5, g.InterpolationAlpha);
        var p = f.GetPreviousPose(x); var q = f.GetCurrentPose(x); var r = f.GetInterpolatedPose(x);
        Assert.Equal(p.Position.X + (q.Position.X - p.Position.X) * 0.5, r.Position.X);
        Assert.Equal(q.Position, x.Position);
        double previousAngle = Math64.Atan2(p.Rotation.S, p.Rotation.C), currentAngle = Math64.Atan2(q.Rotation.S, q.Rotation.C);
        double renderedAngle = Math64.Atan2(r.Rotation.S, r.Rotation.C);
        Assert.InRange(Math.Abs(renderedAngle - (previousAngle + currentAngle) * 0.5), 0, 1e-7);
        var previousSpatial = g.GetPreviousPose(y); var currentSpatial = g.GetCurrentPose(y); var renderedSpatial = g.GetInterpolatedPose(y);
        Assert.Equal(previousSpatial.Position.X + (currentSpatial.Position.X - previousSpatial.Position.X) * 0.5, renderedSpatial.Position.X);
        Assert.Equal(QuaternionF.Slerp(previousSpatial.Rotation, currentSpatial.Rotation, 0.5f), renderedSpatial.Rotation);
        var snap = a.Snapshot(); _ = f.GetInterpolatedPose(x); Assert.Equal(snap, a.Snapshot());
        x.Position = new(10, 20); f.Resynchronize(); Assert.Equal(x.Position, f.GetInterpolatedPose(x).Position);
    }

    [Fact]
    public void StaleAndForeignBodiesAndSlotReuseAreHandled()
    {
        var a = new P2.PhysicsWorld2D(new() { Gravity = default }); var x = a.CreateBody(default);
        var f = new P2.FixedStepAccumulator(a, 0.125f);
        x.Destroy(); var replacement = a.CreateBody(new() { Position = new(5, 6) });
        Assert.Equal(replacement.Position, f.GetInterpolatedPose(replacement).Position);
        Assert.Throws<StaleHandleException>(() => f.GetInterpolatedPose(x));
        Assert.Throws<InvalidHandleException>(() => f.GetCurrentPose(default));
        var other = new P2.PhysicsWorld2D().CreateBody(default);
        Assert.Throws<InvalidHandleException>(() => f.GetPreviousPose(other));
    }

    [Fact]
    public void HostileTimeInputsDoNotConsumeRemainderOrStepWorld()
    {
        var w = new P3.PhysicsWorld3D(); var f = new P3.FixedStepAccumulator(w, 0.125f);
        foreach (double bad in new[] { -1d, double.NaN, double.PositiveInfinity }) Assert.Throws<ArgumentOutOfRangeException>(() => f.Advance(bad));
        Assert.Equal(0, f.Remainder); Assert.Equal(0, w.StepCount);
        Assert.Throws<ArgumentOutOfRangeException>(() => new P3.FixedStepAccumulator(w, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new P3.FixedStepAccumulator(w, float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => new P3.FixedStepAccumulator(w, maximumStepsPerFrame: 0));
        Assert.Throws<ArgumentNullException>(() => new P2.FixedStepAccumulator(null!));
    }

    [Fact]
    public void PresentationDoesNotChangeSteppedWorldBytes()
    {
        var a = new P2.PhysicsWorld2D(new() { WorldId = 721 }); var b = new P2.PhysicsWorld2D(new() { WorldId = 721 });
        var x = a.AddBox(new(1, 1), at: new(0, 5)); b.AddBox(new(1, 1), at: new(0, 5));
        var f = new P2.FixedStepAccumulator(a, 0.125f);
        foreach (double frame in new[] { 0.0625, 0.25, 0.1875, 0.125 })
        {
            int n = f.Advance(frame); for (int i = 0; i < n; i++) b.Step(0.125f);
            _ = f.GetInterpolatedPose(x); Assert.Equal(b.Snapshot(), a.Snapshot());
        }
    }
}
