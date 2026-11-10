#pragma warning disable OPHYS0002, OPHYS0003
using System;
using RecursiveAI.OneMath;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.WorldScale;

public sealed class TimeScaleTests
{
    // Period of a pendulum (point joint to a static anchor), from the interpolated upward zero crossings of the bob's offset, over three periods.
    private static double Period2D(float scale, bool region)
    {
        var w = new P2.PhysicsWorld2D(new() { WorldId = 1351 });
        var anchor = w.AddBox(new Vector2F(0.2f, 0.2f), new Vector2D(0, 10), type: BodyType.Static);
        var bob = w.AddCircle(0.2f, new Vector2D(Math.Sin(0.3) * 2, 10 - Math.Cos(0.3) * 2));
        w.AddPoint(anchor, bob, new Vector2D(0, 10));
        if (region) { w.SetTimeScaleRegions([new P2.TimeScaleRegion2D(new(-10, 0), new(10, 20), scale)]); }
        else { bob.TimeScale = scale; }
        var crossings = new System.Collections.Generic.List<double>();
        double prev = bob.Position.X;
        for (int s = 1; s < 6000 && crossings.Count < 4; s++)
        {
            w.Step(1f / 60f);
            double x = bob.Position.X;
            if (prev < 0 && x >= 0) { crossings.Add(s - 1 + (0 - prev) / (x - prev)); }
            prev = x;
        }

        Assert.True(crossings.Count == 4, "four crossings are expected");
        return (crossings[3] - crossings[0]) / 3.0 / 60.0;
    }

    [Theory]
    [InlineData(0.5f, false)]
    [InlineData(0.5f, true)]
    [InlineData(0.25f, false)]
    public void PendulumPeriodScalesWithTime(float scale, bool region)
    {
        double t1 = Period2D(1f, false), ts = Period2D(scale, region);
        Assert.InRange(ts / t1, (1.0 / scale) * 0.995, (1.0 / scale) * 1.005);
    }

    [Fact]
    public void BodiesOfDifferentScalesConserveMomentumWhenTheyCollide()
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = 1352, Gravity = default });
        var a = w.AddSphere(0.5f, new Vector3D(-3, 0, 0));
        var b = w.AddSphere(0.5f, new Vector3D(0, 0, 0), density: 3f);
        b.TimeScale = 0.25f;
        a.LinearVelocity = new Vector3F(5, 0, 0);
        double p0 = a.Mass * a.LinearVelocity.X + b.Mass * b.LinearVelocity.X;
        bool touched = false;
        for (int s = 0; s < 240; s++)
        {
            w.Step(1f / 60f);
            if (a.Position.X + 0.5 > b.Position.X - 0.5) { touched = true; }
        }

        double p1 = a.Mass * a.LinearVelocity.X + b.Mass * b.LinearVelocity.X;
        Assert.True(touched);
        Assert.InRange(Math.Abs(p1 - p0) / Math.Abs(p0), 0.0, 1e-6);
        Assert.True(b.LinearVelocity.X > 0.5f, "the slow body must have been hit");
    }

    [Fact]
    public void ScaleBodyAdvancesAtItsOwnPace()
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = 1353, Gravity = default });
        var slow = w.AddSphere(0.5f, new Vector3D(0, 0, 0)); slow.TimeScale = 0.5f; slow.LinearVelocity = new Vector3F(2, 0, 0);
        var fast = w.AddSphere(0.5f, new Vector3D(0, 10, 0)); fast.TimeScale = 2f; fast.LinearVelocity = new Vector3F(2, 0, 0);
        var norm = w.AddSphere(0.5f, new Vector3D(0, 20, 0)); norm.LinearVelocity = new Vector3F(2, 0, 0);
        for (int s = 0; s < 60; s++) { w.Step(1f / 60f); }
        Assert.InRange(slow.Position.X, 0.99, 1.01);
        Assert.InRange(fast.Position.X, 3.99, 4.01);
        Assert.InRange(norm.Position.X, 1.99, 2.01);
        Assert.Equal(0.5f, slow.TimeScale);
        Assert.Throws<ArgumentOutOfRangeException>(() => slow.TimeScale = 0f);
        Assert.Throws<ArgumentOutOfRangeException>(() => slow.TimeScale = 5f);
    }

    [Fact]
    public void ScaledWorldsKeepBytesHashesAndThreadsEqualThroughSnapshotAndFork()
    {
        ulong Run(int threads, bool snapshotHalfway)
        {
            var w = new P3.PhysicsWorld3D(new() { WorldId = 1354, ThreadCount = threads });
            w.AddBox(new Vector3F(40, 1, 40), new Vector3D(0, -0.5, 0), type: BodyType.Static);
            for (int i = 0; i < 40; i++) { var b = w.AddBox(new Vector3F(1, 1, 1), new Vector3D((i % 5) * 1.3 - 2, 0.5 + (i / 5) * 1.1, ((i * 7) % 5) * 1.2 - 2)); if (i % 3 == 0) { b.TimeScale = 0.5f; } }
            w.SetTimeScaleRegions([new P3.TimeScaleRegion3D(new(-1, -1, -1), new(1, 3, 1), 0.25f)]);
            ulong h = 0;
            for (int s = 0; s < 120; s++)
            {
                if (snapshotHalfway && s == 60) { var snap = w.Snapshot(); w.Restore(snap); var f = w.Fork(); Assert.Equal(w.ComputeHash(), f.ComputeHash()); }
                w.Step(1f / 60f);
                h = h * 1099511628211UL ^ w.ComputeHash();
            }

            return h;
        }

        ulong a = Run(1, false);
        Assert.Equal(a, Run(1, true));
        Assert.Equal(a, Run(8, false));
    }
}
