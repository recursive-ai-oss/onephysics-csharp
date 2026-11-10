using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

/// <summary>A restored world must continue exactly like the world it was saved from. A pair that was proved apart keeps a hint that is not saved, so the hint may only skip pairs that really are apart.</summary>
public class RestoreContinuation2DTests
{
    private const float Dt = 1f / 60f;

    [Fact]
    public void ApartHintOfALongShapeNeverSkipsATouchingPair()
    {
        // A box 129 m long (reach about 64 m) turned a little: the chord of a small turn must come from the sine, not from the rounding noise of the cosine,
        // or the bound is far too loose (a 1e-7 error in the cosine becomes 4e-4 of chord, 0.02 m at that reach).
        var a = Shape2D.CreateBox(100f, 0.5f);
        var b = Shape2D.CreateBox(64f, 0.25f);
        const float spec = 0.02f;
        float reach = 64.25f;
        var qa = new Rot2(1f, 0f);
        var rng = new Random(12345);
        int skipped = 0, checkedPairs = 0, started = 0;
        for (int trial = 0; trial < 20000; trial++)
        {
            double angle0 = (rng.NextDouble() - 0.5) * 2e-5;
            // Nearly level, so the ends sit about 64 * angle from the centre's height; the pair starts a few centimetres clear of the speculative distance.
            double gap = 0.25 + 0.5 + spec + 64.25 * Math.Abs(angle0) + 0.002 + rng.NextDouble() * 0.012;
            var q0 = new Rot2((float)Math.Cos(angle0), (float)Math.Sin(angle0));
            var xf0 = new Transform2(new Vec2(0f, (float)gap), q0);
            var m0 = Collide2D.Collide(in a, in b, xf0, spec, out float clearance);
            if (m0.PointCount != 0 || clearance <= 0f) { continue; } started++;
            var skip = new SkipFrame { X = 0f, Y = (float)gap, C = q0.C, S = q0.S, Clear = clearance, Reach = reach };
            double angle1 = angle0 + (rng.NextDouble() - 0.5) * 6e-4;
            double gap1 = gap + (rng.NextDouble() - 0.5) * 0.004;
            var q1 = new Rot2((float)Math.Cos(angle1), (float)Math.Sin(angle1));
            if (!Engine2D.StillApart(in skip, qa, q1, 0f, (float)gap1)) { continue; }
            skipped++;
            var m1 = Collide2D.Collide(in a, in b, new Transform2(new Vec2(0f, (float)gap1), q1), spec, out _);
            checkedPairs++;
            Assert.True(m1.PointCount == 0, $"the hint skipped a touching pair at trial {trial} (turn {angle1 - angle0}, gap {gap1}, clearance {clearance})");
        }

        Assert.True(skipped > 500 && checkedPairs == skipped, $"skipped {skipped} started {started}");
    }

    // A long thin plank turns slowly just above the ground, so its pair is repeatedly proved apart and then skipped; snapshot and restore at step k must continue bit for bit.
    private static PhysicsWorld2D PlankWorld(int id, double angle)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = id, Substeps = 4, Gravity = new Vector2F(0f, 0f) });
        w.AddBox(new Vector2F(400f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        var plank = w.AddBox(new Vector2F(128f, 0.5f), new Vector2D(0, 0.5 + 0.03), angleRadians: (float)angle);
        plank.AngularVelocity = 0.0004f;
        return w;
    }

    [Fact]
    public void RestoredPlankWorldContinuesLikeTheOriginal()
    {
        for (int k = 0; k < 8; k++)
        {
            var w = PlankWorld(31, 0.0003 * k);
            for (int i = 0; i < 20 + 7 * k; i++) { w.Step(Dt); }
            byte[] snap = w.Snapshot();
            var r = new PhysicsWorld2D(new WorldOptions2D { WorldId = 31 });
            r.Restore(snap);
            Assert.Equal(w.ComputeHash(), r.ComputeHash());
            for (int i = 0; i < 60; i++)
            {
                w.Step(Dt); r.Step(Dt);
                Assert.True(w.ComputeHash() == r.ComputeHash(), $"restored plank world diverged {i + 1} steps after restoring at variant {k}");
            }
        }
    }

    private static PhysicsWorld2D VehicleWorld(int id, int kind)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = id });
        w.AddBox(new Vector2F(600f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static, material: new Material2D(0.8f, 0f, 0f));
        for (int i = 0; i < 6; i++) { w.AddBox(new Vector2F(0.5f, 0.5f), at: new Vector2D(14 + 3 * i, 0.5 + 0.9 * i)); }
        switch (kind)
        {
            case 0: w.AddCar(new Vector2D(0, 0)); break;
            case 1: w.AddTank(new Vector2D(0, 0)); break;
            default: w.AddMotorcycle(new Vector2D(0, 0)); break;
        }

        return w;
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void RestoredVehicleWorldContinuesLikeTheOriginal(int kind)
    {
        foreach (int k in new[] { 5, 37, 90 })
        {
            var w = VehicleWorld(32, kind);
            for (int i = 0; i < k; i++) { foreach (var v in w.GetVehicles()) { v.Drive(0.7f, i % 40 < 20 ? 0.3f : -0.3f); } w.Step(Dt); }
            byte[] snap = w.Snapshot();
            var r = new PhysicsWorld2D(new WorldOptions2D { WorldId = 32 });
            r.Restore(snap);
            Assert.Equal(w.ComputeHash(), r.ComputeHash());
            var hashes = new List<ulong>();
            for (int i = 0; i < 120; i++)
            {
                foreach (var v in w.GetVehicles()) { v.Drive(0.7f, i % 40 < 20 ? 0.3f : -0.3f); }
                foreach (var v in r.GetVehicles()) { v.Drive(0.7f, i % 40 < 20 ? 0.3f : -0.3f); }
                w.Step(Dt); r.Step(Dt);
                Assert.True(w.ComputeHash() == r.ComputeHash(), $"restored world (vehicle kind {kind}) diverged {i + 1} steps after restoring at step {k}");
                hashes.Add(r.ComputeHash());
            }

            Assert.Equal(w.Snapshot(), r.Snapshot());
        }
    }
}
