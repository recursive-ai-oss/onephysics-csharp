using System;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Replay;
using RecursiveAI.OnePhysics.Rods;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Settle;

public sealed class SettleTests
{
    private static P3.PhysicsWorld3D Stack3()
    {
        var w = new P3.PhysicsWorld3D();
        w.AddBox(new(40, 1, 40), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 4; i++) w.AddBox(new(1, 1, 1), new Vector3D(0, 0.5 + 1.2 * i, 0));
        return w;
    }

    private static P2.PhysicsWorld2D Stack2()
    {
        var w = new P2.PhysicsWorld2D();
        w.AddBox(new(40, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 4; i++) w.AddBox(new(1, 1), new Vector2D(0, 0.5 + 1.2 * i));
        return w;
    }

    [Fact]
    public void AStackConvergesInBothDimensionsAndEveryBodyIsSupported()
    {
        var w3 = Stack3();
        var r3 = w3.Settle(new P3.SettleOptions3D { ReportBodies = true });
        Assert.True(r3.Converged, r3.ToString());
        Assert.True(r3.IsClean);
        Assert.Equal(4, r3.BodiesJudged);
        Assert.Equal(4, r3.Bodies!.Length);
        Assert.Empty(r3.Unsupported!);
        Assert.All(r3.Bodies, b => Assert.True(b.IsSupported && b.IsInBounds && b.Speed <= 0.01f));
        Assert.True(w3.StepCount >= r3.StepsUsed && r3.StepsUsed >= 20);
        var r2 = Stack2().Settle(new P2.SettleOptions2D { ReportBodies = true });
        Assert.True(r2.Converged, r2.ToString());
        Assert.True(r2.IsClean);
        Assert.Equal(4, r2.BodiesJudged);
    }

    [Fact]
    public void ArraysAppearOnlyWhenAsked()
    {
        var r = Stack3().Settle();
        Assert.Null(r.Bodies); Assert.Null(r.Unsupported); Assert.Null(r.OutOfBounds);
        Assert.Equal(0, r.UnsupportedCount);
    }

    [Fact]
    public void ABodyWithNothingBelowItIsNotConvergedAndLeavesTheBound()
    {
        var w3 = new P3.PhysicsWorld3D();
        w3.AddBox(new(1, 1, 1), new Vector3D(0, 5, 0));
        var open = w3.Settle(new P3.SettleOptions3D { MaxSteps = 120 });
        Assert.False(open.Converged);
        Assert.Equal(120, open.StepsUsed);
        Assert.Equal(1, open.UnsupportedCount);
        Assert.True(open.WorstForce > 5f, open.ToString());
        var w3b = new P3.PhysicsWorld3D();
        w3b.AddBox(new(1, 1, 1), new Vector3D(0, 5, 0));
        var bounded = w3b.Settle(new P3.SettleOptions3D { MaxSteps = 200, ReportBodies = true, Bounds = (new Vector3D(-10, -3, -10), new Vector3D(10, 10, 10)) });
        Assert.Equal(1, bounded.OutOfBoundsCount);
        Assert.Single(bounded.OutOfBounds!);
        Assert.False(bounded.IsClean);
        var w2 = new P2.PhysicsWorld2D();
        w2.AddBox(new(1, 1), new Vector2D(0, 5));
        var r2 = w2.Settle(new P2.SettleOptions2D { MaxSteps = 200, Bounds = (new Vector2D(-10, -3), new Vector2D(10, 10)) });
        Assert.Equal(1, r2.OutOfBoundsCount);
        Assert.False(r2.IsClean);
    }

    [Fact]
    public void TouchingBodiesWithNothingStaticAreUnsupportedEvenAtRest()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = default });
        w.AddBox(new(1, 1, 1), new Vector3D(0, 0, 0));
        w.AddBox(new(1, 1, 1), new Vector3D(0, 1.001, 0));
        var r = w.Settle(new P3.SettleOptions3D { ReportBodies = true });
        Assert.True(r.Converged);
        Assert.Equal(2, r.UnsupportedCount);
        Assert.Equal(2, r.Unsupported!.Length);
    }

    // Catto's friction cone: a block stays on a slope while tan(theta) < mu and slides above it.
    private static (P3.PhysicsWorld3D World, P3.Body Block) Slope3(double slope, float mu)
    {
        float theta = (float)Math64.Atan(slope);
        var w = new P3.PhysicsWorld3D();
        var q = QuaternionF.CreateFromAxisAngle(new Vector3F(0, 0, 1), theta);
        var material = new P3.Material3D(mu, 0f);
        w.AddBox(new(40, 1, 10), default, q, BodyType.Static, material);
        var up = q.Rotate(new Vector3F(0, 1, 0));
        var at = new Vector3D(up.X * 1.0005, up.Y * 1.0005, up.Z * 1.0005);
        return (w, w.AddBox(new(1, 1, 1), at, q, BodyType.Dynamic, material));
    }

    private static (P2.PhysicsWorld2D World, P2.Body Block) Slope2(double slope, float mu)
    {
        float theta = (float)Math64.Atan(slope);
        var w = new P2.PhysicsWorld2D();
        var material = new P2.Material2D(mu, 0f);
        w.AddBox(new(40, 1), default, theta, BodyType.Static, material);
        double c = Math64.Cos(theta), s = Math64.Sin(theta);
        return (w, w.AddBox(new(1, 1), new Vector2D(-s * 1.0005, c * 1.0005), theta, BodyType.Dynamic, material));
    }

    [Theory]
    [InlineData(0.5f, 0.9)]
    [InlineData(0.3f, 0.9)]
    public void ABlockHoldsOnASlopeBelowTheFrictionAngle(float mu, double fraction)
    {
        var (w3, block3) = Slope3(mu * fraction, mu);
        var r3 = w3.Settle(new P3.SettleOptions3D { Bounds = (new Vector3D(-30, -30, -30), new Vector3D(30, 30, 30)) });
        Assert.True(r3.IsClean, r3.ToString());
        Assert.True(block3.LinearVelocity.X * block3.LinearVelocity.X + block3.LinearVelocity.Y * block3.LinearVelocity.Y < 1e-4f);
        var (w2, _) = Slope2(mu * fraction, mu);
        var r2 = w2.Settle(new P2.SettleOptions2D { Bounds = (new Vector2D(-30, -30), new Vector2D(30, 30)) });
        Assert.True(r2.IsClean, r2.ToString());
    }

    [Theory]
    [InlineData(0.5f, 1.1)]
    [InlineData(0.3f, 1.1)]
    public void ABlockSlidesOnASlopeAboveTheFrictionAngle(float mu, double fraction)
    {
        var (w3, _) = Slope3(mu * fraction, mu);
        var open = w3.Settle(new P3.SettleOptions3D { MaxSteps = 240 });
        Assert.False(open.Converged, open.ToString());
        Assert.True(open.WorstSpeed > 0.1f);
        var (w3b, _) = Slope3(mu * fraction, mu);
        var bounded = w3b.Settle(new P3.SettleOptions3D { MaxSteps = 1200, Bounds = (new Vector3D(-9, -30, -30), new Vector3D(9, 30, 30)) });
        Assert.Equal(1, bounded.OutOfBoundsCount);
        Assert.False(bounded.IsClean);
        var (w2, _) = Slope2(mu * fraction, mu);
        var r2 = w2.Settle(new P2.SettleOptions2D { MaxSteps = 240 });
        Assert.False(r2.Converged, r2.ToString());
        var (w2b, _) = Slope2(mu * fraction, mu);
        var b2 = w2b.Settle(new P2.SettleOptions2D { MaxSteps = 1200, Bounds = (new Vector2D(-9, -30), new Vector2D(9, 30)) });
        Assert.Equal(1, b2.OutOfBoundsCount);
    }

    [Fact]
    public void SettleLeavesTheOptionsAndSnapshotsConsistent()
    {
        var w = Stack3();
        bool sleep = w.Options.SleepEnabled;
        w.Settle();
        Assert.Equal(sleep, w.Options.SleepEnabled);
        var bytes = w.Snapshot();
        var copy = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = w.Id });
        copy.Restore(bytes);
        Assert.Equal(w.ComputeHash(), copy.ComputeHash());
        Assert.True(copy.Options.SleepEnabled);
        for (int i = 0; i < 20; i++) { w.Step(1f / 60); copy.Step(1f / 60); }
        Assert.Equal(w.ComputeHash(), copy.ComputeHash());
    }

    [Fact]
    public void SettleIsIdenticalAtAnyThreadCount()
    {
        ulong Run(int threads)
        {
            var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { ThreadCount = threads, WorldId = 7 });
            w.AddBox(new(40, 1, 40), new Vector3D(0, -0.5, 0), type: BodyType.Static);
            for (int i = 0; i < 30; i++) w.AddBox(new(0.5f, 0.5f, 0.5f), new Vector3D(i % 3 * 0.6, 0.3 + 0.55 * (i / 3), (i % 2) * 0.1));
            var r = w.Settle(new P3.SettleOptions3D { ExtraLinearDamping = 0.5f });
            return w.ComputeHash() ^ (ulong)r.StepsUsed;
        }
        Assert.Equal(Run(1), Run(4));
    }

    [Fact]
    public void ExtraDampingShortensTheRun()
    {
        int Steps(float damping)
        {
            var w = Stack3();
            return w.Settle(new P3.SettleOptions3D { ExtraLinearDamping = damping, ExtraAngularDamping = damping }).StepsUsed;
        }
        Assert.True(Steps(3f) < Steps(0f));
    }

    [Fact]
    public void BadOptionsAreRejectedWithTheFieldName()
    {
        var w = Stack3();
        Assert.Equal("MaxSteps", Assert.Throws<ArgumentOutOfRangeException>(() => w.Settle(new P3.SettleOptions3D { MaxSteps = 0 })).ParamName);
        Assert.Equal("Dt", Assert.Throws<ArgumentOutOfRangeException>(() => w.Settle(new P3.SettleOptions3D { Dt = float.NaN })).ParamName);
        Assert.Equal("SpeedTolerance", Assert.Throws<ArgumentOutOfRangeException>(() => w.Settle(new P3.SettleOptions3D { SpeedTolerance = -1 })).ParamName);
        Assert.Equal("Bounds", Assert.Throws<ArgumentOutOfRangeException>(() => w.Settle(new P3.SettleOptions3D { Bounds = (new Vector3D(1, 0, 0), new Vector3D(0, 0, 0)) })).ParamName);
        Assert.Equal("StableSteps", Assert.Throws<ArgumentOutOfRangeException>(() => Stack2().Settle(new P2.SettleOptions2D { StableSteps = 0 })).ParamName);
    }

    [Fact]
    public void SettleFromInsideAStepCallbackIsRefused()
    {
        var w = new P3.PhysicsWorld3D();
        w.AddBox(new(10, 1, 10), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var box = w.AddBox(new(1, 1, 1), new Vector3D(0, 0.7, 0));
        Exception? caught = null;
        box.OnCollisionEnter += _ => { try { w.Settle(); } catch (Exception e) { caught = e; } };
        for (int i = 0; i < 30; i++) w.Step(1f / 60);
        Assert.IsType<InvalidOperationException>(caught);
        Assert.True(w.Settle().Converged);
    }

    // A box 1 wide and 2 tall tips about its bottom edge when the centre of mass, 1 m above it, passes 0.5 m over the edge.
    [Fact]
    public void ATallBoxTipsAtTheAngleAndPushOfTheStaticEstimate()
    {
        var w = new P3.PhysicsWorld3D();
        w.AddBox(new(40, 1, 40), new Vector3D(0, -0.5, 0), type: BodyType.Static, material: new P3.Material3D(2f, 0f));
        var box = w.AddBox(new(1, 2, 1), new Vector3D(0, 1.0005, 0), material: new P3.Material3D(2f, 0f));
        Assert.True(w.Settle().IsClean);
        var m = box.GetStabilityMargin();
        Assert.True(m.HasSupport && m.IsStable);
        double expected = Math64.Atan(0.5 / 1.0);
        Assert.InRange(m.MinTipAngle, expected * 0.99, expected * 1.01);
        double force = box.Mass * 9.81 * 0.5;
        Assert.InRange(m.MinTippingForce, force * 0.99, force * 1.01);
        Assert.InRange(m.Margin, 0.495f, 0.505f);
        Assert.Equal(4, m.SupportPolygon.Length);
        // Diagonal pushes need more: the corner is farther away.
        Assert.True(m.TippingForces[1] > m.MinTippingForce);
        Assert.True(m.SlidingForce > m.MinTippingForce, "friction 2 holds the box until it tips");
        Assert.False(m.SlidesBeforeTipping);
    }

    [Fact]
    public void ThePredictedTippingPushTipsTheBoxAndAnEarlierOneDoesNot()
    {
        bool Tips(double factor)
        {
            var w = new P3.PhysicsWorld3D();
            w.AddBox(new(40, 1, 40), new Vector3D(0, -0.5, 0), type: BodyType.Static, material: new P3.Material3D(2f, 0f));
            var box = w.AddBox(new(1, 2, 1), new Vector3D(0, 1.0005, 0), material: new P3.Material3D(2f, 0f));
            Assert.True(w.Settle().IsClean);
            var m = box.GetStabilityMargin();
            var d = m.Directions[0];
            var f = new Vector3F((float)(d.X * m.MinTippingForce * factor), (float)(d.Y * m.MinTippingForce * factor), (float)(d.Z * m.MinTippingForce * factor));
            for (int i = 0; i < 180; i++) { box.ApplyForce(f); w.Step(1f / 60); }
            return box.Position.Y < 0.9;
        }
        Assert.False(Tips(0.9));
        Assert.True(Tips(1.1));
    }

    [Fact]
    public void ABoxOnAnEdgeOrSlopeReportsItsOwnMarginAndAnAssemblyUsesItsCombinedCentre()
    {
        var w = new P3.PhysicsWorld3D();
        w.AddBox(new(40, 1, 40), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var low = w.AddBox(new(2, 1, 2), new Vector3D(0, 0.5005, 0));
        var high = w.AddBox(new(2, 1, 2), new Vector3D(0.9, 1.502, 0));
        Assert.True(w.Settle().Converged);
        var mLow = low.GetStabilityMargin();
        Assert.True(mLow.IsStable);
        var both = w.GetStabilityMargin([low, high]);
        Assert.True(both.HasSupport);
        // The upper box overhangs: the pair's centre sits 0.45 m toward +x, so the margin shrinks from 1.0 to about 0.55 in that direction.
        Assert.InRange(both.CenterOfMass.X, 0.44, 0.46);
        Assert.InRange(both.Margin, 0.5f, 0.6f);
        Assert.Throws<ArgumentException>(() => w.GetStabilityMargin([]));
        Assert.Throws<ArgumentException>(() => w.GetStabilityMargin([w.AddBox(new(1, 1, 1), type: BodyType.Static)]));
        var floating = new P3.PhysicsWorld3D().AddBox(new(1, 1, 1));
        Assert.False(floating.GetStabilityMargin().HasSupport);
    }

    [Fact]
    public void ATwoDimensionalBoxHasTheSameTipAngleInBothDirections()
    {
        var w = new P2.PhysicsWorld2D();
        w.AddBox(new(40, 1), new Vector2D(0, -0.5), type: BodyType.Static, material: new P2.Material2D(2f, 0f));
        var box = w.AddBox(new(1, 2), new Vector2D(0, 1.0005), material: new P2.Material2D(2f, 0f));
        Assert.True(w.Settle().IsClean);
        var m = box.GetStabilityMargin();
        Assert.True(m.IsStable);
        double expected = Math64.Atan(0.5);
        Assert.InRange(m.TipAngles[0], expected * 0.99, expected * 1.01);
        Assert.InRange(m.TipAngles[1], expected * 0.99, expected * 1.01);
        Assert.InRange(m.MinTippingForce, box.Mass * 9.81 * 0.5 * 0.99, box.Mass * 9.81 * 0.5 * 1.01);
        Assert.Equal(2, m.SupportPolygon.Length);
        Assert.InRange(m.Margin, 0.495f, 0.505f);
        // An off-centre box: lean the box over its edge by moving it.
        var w2 = new P2.PhysicsWorld2D();
        w2.AddBox(new(40, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        var thin = w2.AddBox(new(0.2f, 2), new Vector2D(0, 1.0005));
        Assert.True(w2.Settle().IsClean);
        Assert.True(thin.GetStabilityMargin().MinTipAngle < m.MinTipAngle);
    }

    private static RodMaterial String => new(1e9f, 4e8f, 0.001f, 1000f);

    // The catenary of an inextensible chain of length L hung between two points S apart at the same height.
    private static (double A, double Sag) Catenary(double span, double length)
    {
        double target = length / span, lo = 1e-3, hi = 50;
        for (int i = 0; i < 200; i++)
        {
            double u = 0.5 * (lo + hi);
            double ratio = Math.Sinh(u) / u;
            if (ratio < target) lo = u; else hi = u;
        }
        double uu = 0.5 * (lo + hi), a = span / (2 * uu);
        return (a, a * (Math.Cosh(uu) - 1));
    }

    [Fact]
    public void AHangingStringSettlesToTheCatenaryIn3D()
    {
        var w = new P3.PhysicsWorld3D();
        int n = 64; var pts = new Vector3D[n + 1];
        for (int i = 0; i <= n; i++) { double x = -2 + 4.0 * i / n; pts[i] = new Vector3D(x, -0.8 * (1 - x * x / 4), 0); }
        double length = 0; for (int i = 0; i < n; i++) { var d = pts[i + 1] - pts[i]; length += Math.Sqrt(d.X * d.X + d.Y * d.Y); }
        var rod = w.AddRod(new P3.RodDef { Points = pts, Material = String, PinnedParticles = [0, n], Damping = 1f });
        var r = w.Settle(new P3.SettleOptions3D { ExtraLinearDamping = 1f, SpeedTolerance = 0.002f, MaxSteps = 4000 });
        Assert.True(r.Converged, r.ToString());
        var shape = w.GetRestShape(rod);
        var (a, sag) = Catenary(4, length);
        double lowest = 0, worst = 0;
        foreach (var p in shape.Positions)
        {
            double y = a * (Math.Cosh(p.X / a) - 1) - sag;
            worst = Math.Max(worst, Math.Abs(p.Y - y));
            lowest = Math.Min(lowest, p.Y);
        }
        Assert.InRange(shape.Sag, sag * 0.99, sag * 1.01);
        Assert.True(worst <= 0.01 * sag, $"worst deviation {worst} of sag {sag}");
        Assert.InRange(shape.ArcLength, length * 0.999, length * 1.002);
        Assert.True(shape.MaxSpeed < 0.01);
        Assert.Equal(n, shape.Frames.Length);
        Assert.Equal(2 * n, shape.Edges.Length);
        Assert.Empty(shape.Triangles);
    }

    [Fact]
    public void AHangingStringSettlesToTheCatenaryIn2D()
    {
        var w = new P2.PhysicsWorld2D();
        int n = 64; var pts = new Vector2D[n + 1];
        for (int i = 0; i <= n; i++) { double x = -2 + 4.0 * i / n; pts[i] = new Vector2D(x, -0.8 * (1 - x * x / 4)); }
        double length = 0; for (int i = 0; i < n; i++) { var d = pts[i + 1] - pts[i]; length += Math.Sqrt(d.X * d.X + d.Y * d.Y); }
        var rod = w.AddRod(new P2.RodDef { Points = pts, Material = String, PinnedParticles = [0, n], Damping = 1f });
        var r = w.Settle(new P2.SettleOptions2D { ExtraLinearDamping = 1f, SpeedTolerance = 0.002f, MaxSteps = 4000 });
        Assert.True(r.Converged, r.ToString());
        var shape = w.GetRestShape(rod);
        var (a, sag) = Catenary(4, length);
        double worst = 0;
        foreach (var p in shape.Positions) worst = Math.Max(worst, Math.Abs(p.Y - (a * (Math.Cosh(p.X / a) - 1) - sag)));
        Assert.InRange(shape.Sag, sag * 0.99, sag * 1.01);
        Assert.True(worst <= 0.01 * sag, $"worst deviation {worst} of sag {sag}");
    }

    [Fact]
    public void AClothPinnedAtItsCornersSagsAndTheShapeReadsBack()
    {
        var w = new P3.PhysicsWorld3D();
        var cloth = w.AddCloth(2f, 2f, 8, new Vector3D(0, 2, 0), pinCorners: true);
        var r = w.Settle(new P3.SettleOptions3D { ExtraLinearDamping = 2f, SpeedTolerance = 0.02f, MaxSteps = 3000 });
        Assert.True(r.Converged, r.ToString());
        var shape = w.GetRestShape(cloth);
        Assert.Equal(cloth.ParticleCount, shape.Positions.Length);
        Assert.NotEmpty(shape.Triangles); Assert.NotEmpty(shape.Edges);
        Assert.True(shape.Sag > 0.05, shape.Sag.ToString());
        Assert.True(shape.MaxSpeed < 0.05);
        Assert.True(shape.BoundsMin.Y < shape.BoundsMax.Y);
        Assert.Throws<InvalidHandleException>(() => new P3.PhysicsWorld3D().GetRestShape(cloth));
        cloth.Destroy();
        Assert.Throws<StaleHandleException>(() => w.GetRestShape(cloth));
    }

    [Fact]
    public void AHangingMassOnARopeIsJudgedWithTheRope()
    {
        var w = new P3.PhysicsWorld3D();
        w.AddRope(new Vector3D(0, 5, 0), new Vector3D(0, 3, 0), 16);
        var r = w.Settle(new P3.SettleOptions3D { ExtraLinearDamping = 1f, SpeedTolerance = 0.005f, MaxSteps = 3000 });
        Assert.True(r.Converged, r.ToString());
        Assert.True(r.WorstDeformableSpeed <= 0.005f);
    }
}
