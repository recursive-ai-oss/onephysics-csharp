#pragma warning disable OPHYS0002, OPHYS0003
using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.WorldScale;

public sealed class LodTests
{
    private static List<P3.Body> Pile(P3.PhysicsWorld3D w, double x0)
    {
        var list = new List<P3.Body>();
        w.AddBox(new Vector3F(60, 1, 20), new Vector3D(20, -0.5, 0), type: BodyType.Static);
        for (int c = 0; c < 3; c++)
        {
            for (int k = 0; k < 6; k++) { list.Add(w.AddBox(new Vector3F(1, 1, 1), new Vector3D(x0 + c * 2.5 + 0.05 * (k % 2), 0.5 + k * 1.02, 0.03 * k))); }
        }

        return list;
    }

    [Theory]
    [InlineData(2)]
    [InlineData(4)]
    [InlineData(8)]
    public void PileInASlowRegionSettlesLikeAtFullRate(int period)
    {
        var full = new P3.PhysicsWorld3D(new() { WorldId = 1360 });
        var slow = new P3.PhysicsWorld3D(new() { WorldId = 1361 });
        var a = Pile(full, 10); var b = Pile(slow, 10);
        slow.SetSimulationLod(default, default, 1, [new P3.LodRegion3D(new(5, -1, -5), new(30, 20, 5), period)]);
        for (int s = 0; s < 900; s++) { full.Step(1f / 60f); slow.Step(1f / 60f); }
        double worstSpeed = 0, worstDrift = 0;
        for (int i = 0; i < a.Count; i++)
        {
            worstSpeed = Math.Max(worstSpeed, b[i].LinearVelocity.Length());
            worstDrift = Math.Max(worstDrift, Math.Abs(b[i].Position.Y - a[i].Position.Y) + Math.Abs(b[i].Position.X - a[i].Position.X));
        }

        Assert.True(worstSpeed < 0.05, $"period {period}: speed {worstSpeed}");
        Assert.True(worstDrift < 0.4, $"period {period}: drift from the full-rate pile {worstDrift}");
        Assert.True(slow.Counters.AsleepIslands >= 3 || worstSpeed < 0.02, $"period {period}: asleep {slow.Counters.AsleepIslands}");
    }

    [Fact]
    public void FreeFlightAcrossRegionsKeepsItsVelocityExactly()
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = 1362, Gravity = default });
        var ball = w.AddSphere(0.5f, new Vector3D(0, 0, 0)); ball.LinearVelocity = new Vector3F(8, 1, -2);
        w.SetSimulationLod(default, default, 1, [new P3.LodRegion3D(new(20, -100, -100), new(60, 100, 100), 4)]);
        var v0 = ball.LinearVelocity;
        int slowSteps = 0;
        for (int s = 0; s < 600; s++)
        {
            w.Step(1f / 60f);
            Assert.Equal(v0, ball.LinearVelocity);
            if (w.Counters.LodIslandsSkipped > 0) { slowSteps++; }
        }

        Assert.True(slowSteps > 100, "the ball must have been in the slow region");
        // An island joins the rhythm of its region at the next step that is due: its clock moves at most period - 1 steps either way.
        Assert.InRange(ball.Position.X, 8 * 10 - 8 * 3 / 60.0 - 1e-3, 8 * 10 + 8 * 3 / 60.0 + 1e-3);
    }

    [Fact]
    public void BounceBetweenARegionAndFullRateConservesMomentumAtEveryCrossing()
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = 1363, Gravity = default });
        var mat = new P3.Material3D(0f, 1f);
        w.AddBox(new Vector3F(1, 20, 20), new Vector3D(45, 0, 0), type: BodyType.Static, material: mat);
        w.AddBox(new Vector3F(1, 20, 20), new Vector3D(-5, 0, 0), type: BodyType.Static, material: mat);
        var ball = w.AddSphere(0.5f, new Vector3D(0, 0, 0), material: mat); ball.LinearVelocity = new Vector3F(12, 0, 0);
        w.SetSimulationLod(default, default, 1, [new P3.LodRegion3D(new(20, -100, -100), new(60, 100, 100), 4)]);
        double mass = ball.Mass;
        double worst = 0;
        double lastX = 0; int crossings = 0; bool inside = false;
        for (int s = 0; s < 2400; s++)
        {
            double pBefore = Math.Abs(ball.LinearVelocity.X);
            w.Step(1f / 60f);
            bool nowInside = ball.Position.X >= 20;
            if (nowInside != inside) { crossings++; inside = nowInside; }
            lastX = ball.Position.X;
            double speed = ball.LinearVelocity.Length();
            worst = Math.Max(worst, Math.Abs(speed - 12) / 12);
        }

        Assert.True(crossings >= 4, $"crossings {crossings}");
        Assert.True(worst < 1e-6, $"speed deviates by {worst} relative (momentum of a restitution-1 bounce)");
    }

    [Fact]
    public void LodWorldsKeepHashesEqualAcrossThreadsSnapshotsAndForks()
    {
        ulong Run(int threads, bool roundTrip)
        {
            var w = new P3.PhysicsWorld3D(new() { WorldId = 1364, ThreadCount = threads });
            Pile(w, 0); Pile(w, 30);
            var lone = w.AddSphere(0.5f, new Vector3D(15, 3, 0)); lone.LinearVelocity = new Vector3F(3, 0, 0);
            w.SetSimulationLod([new Vector3D(0, 0, 0)], [new LodBand(12, 1), new LodBand(25, 2)], 4);
            ulong h = 0;
            for (int s = 0; s < 240; s++)
            {
                if (roundTrip && s == 100) { var snap = w.Snapshot(); w.Restore(snap); var f = w.Fork(); Assert.Equal(w.ComputeHash(), f.ComputeHash()); f.Step(1f / 60f); }
                if (s == 120) { w.SetLodFocus([new Vector3D(30, 0, 0)]); }
                w.Step(1f / 60f);
                h = h * 1099511628211UL ^ w.ComputeHash();
            }

            return h;
        }

        ulong a = Run(1, false);
        Assert.Equal(a, Run(1, true));
        Assert.Equal(a, Run(8, false));
    }

    [Fact]
    public void KinematicContactsAndSleepingComposeWithLod()
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = 1365 });
        w.AddBox(new Vector3F(60, 1, 20), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var platform = w.AddBox(new Vector3F(4, 0.5f, 4), new Vector3D(0, 0.25, 0), type: BodyType.Kinematic);
        platform.LinearVelocity = new Vector3F(1, 0, 0);
        var rider = w.AddBox(new Vector3F(1, 1, 1), new Vector3D(0, 1, 0));
        var far = w.AddBox(new Vector3F(1, 1, 1), new Vector3D(20, 0.5, 0));
        w.SetSimulationLod(default, default, 1, [new P3.LodRegion3D(new(-100, -10, -100), new(100, 100, 100), 8)]);
        for (int s = 0; s < 300; s++) { w.Step(1f / 60f); }
        // The platform moves every step at its own speed and carries its rider: the island touching it keeps period 1.
        Assert.InRange(platform.Position.X, 4.9, 5.1);
        Assert.InRange(rider.Position.X - platform.Position.X, -0.5, 0.5);
        Assert.False(far.IsAwake, "the far box falls asleep at period 8");
    }

    private static double Period3D(int period)
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = 1366 });
        var anchor = w.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), new Vector3D(0, 10, 0), type: BodyType.Static);
        var bob = w.AddSphere(0.2f, new Vector3D(Math.Sin(0.3) * 2, 10 - Math.Cos(0.3) * 2, 0));
        w.AddPoint(anchor, bob, new Vector3D(0, 10, 0));
        w.SetSimulationLod(default, default, 1, [new P3.LodRegion3D(new(-10, 0, -10), new(10, 20, 10), period)]);
        var crossings = new List<double>();
        double prev = bob.Position.X;
        for (int s = 1; s < 4000 && crossings.Count < 4; s++)
        {
            w.Step(1f / 60f);
            double x = bob.Position.X;
            if (prev < 0 && x >= 0) { crossings.Add(s - 1 + (0 - prev) / (x - prev)); }
            prev = x;
        }

        Assert.Equal(4, crossings.Count);
        return (crossings[3] - crossings[0]) / 3.0 / 60.0;
    }

    [Theory]
    [InlineData(2)]
    [InlineData(4)]
    public void JointIslandsKeepTheirPeriodAtASlowerRate(int period)
    {
        double full = Period3D(1), slow = Period3D(period);
        Assert.InRange(slow / full, 0.97, 1.03);
    }

    [Fact]
    public void VehiclesStayAtTheFullRateInsideASlowRegion()
    {
        ulong Run(bool lod)
        {
            var w = new P3.PhysicsWorld3D(new() { WorldId = 1367 });
            w.AddBox(new Vector3F(200, 1, 60), new Vector3D(0, -0.5, 0), type: BodyType.Static);
            var body = w.AddBox(new Vector3F(1.8f, 0.5f, 4.2f), new Vector3D(0, 1.0, 0), density: 600f);
            var car = w.AddVehicle(body, P3.VehicleDef.Car);
            if (lod) { w.SetSimulationLod(default, default, 1, [new P3.LodRegion3D(new(-100, -10, -30), new(100, 20, 30), 8)]); }
            for (int s = 0; s < 150; s++) { car.Drive(0.6f, 0.1f); w.Step(1f / 60f); }
            return (ulong)BitConverter.DoubleToInt64Bits(body.Position.X) * 31 + (ulong)BitConverter.DoubleToInt64Bits(body.Position.Z) * 17 + (ulong)BitConverter.SingleToInt32Bits(body.LinearVelocity.Length());
        }

        Assert.Equal(Run(false), Run(true));
    }
}
