using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

public class SoftBody2DTests
{
    private const float Dt = 1f / 60f;

    private static readonly int[] OneIndex = { 0 };
    private static readonly int[] BadPin = { 99 };
    private static readonly int[] TwoIdx = { 0, 1 };
    private static readonly int[] OutOfRange = { 0, 1, 7 };
    private static readonly int[] Repeated = { 0, 1, 1 };
    private static readonly int[] OneTriangle = { 0, 1, 2 };
    private static readonly int[] BowtieTris = { 0, 1, 2, 0, 3, 4 };

    private static PhysicsWorld2D NewWorld(Action<WorldOptions2D>? tweak = null, int threads = 1)
    {
        var o = new WorldOptions2D { ThreadCount = threads, WorldId = 41 };
        tweak?.Invoke(o);
        return new PhysicsWorld2D(o);
    }

    private static void Run(PhysicsWorld2D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    private static Body Floor(PhysicsWorld2D w) => w.AddBox(new Vector2F(60f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);

    // ---- meshes and definitions

    [Fact]
    public void GeneratedMeshesAreValidAndHaveTheRightArea()
    {
        var w = NewWorld();
        var box = SoftBodyMesh.Box(new Vector2F(2f, 3f), 4, 6);
        Assert.Equal(5 * 7, box.VertexCount);
        Assert.Equal(4 * 6 * 2, box.TriangleCount);
        Assert.True(box.IsClosed);
        var b = w.AddSoftBody(new SoftBodyDef { Mesh = box, Mass = 1f });
        Assert.Equal(6.0, b.RestArea, 4);
        Assert.Equal(6.0, b.Area, 4);
        var ring = SoftBodyMesh.Ring(1f, 32);
        var r = w.AddSoftBody(SoftBodyDef.Balloon(ring));
        Assert.InRange(r.RestArea, 3.10, Math.PI);
        var disc = SoftBodyMesh.Disc(1f, 3, 18);
        Assert.True(disc.IsClosed);
        var poly = SoftBodyMesh.FromPolygon(new[] { new Vector2F(0, 0), new Vector2F(2, 0), new Vector2F(2, 1), new Vector2F(1, 1), new Vector2F(1, 2), new Vector2F(0, 2) });
        Assert.Equal(4, poly.TriangleCount);
        var p = w.AddSoftBody(new SoftBodyDef { Mesh = poly, Mass = 1f });
        Assert.Equal(3.0, p.RestArea, 4);
    }

    [Fact]
    public void HostileMeshesAndDefinitionsAreRefusedWithTypedErrors()
    {
        var v = new[] { new Vector2F(0, 0), new Vector2F(1, 0), new Vector2F(0, 1), new Vector2F(1, 1) };
        Assert.Equal(SoftBodyError.BadCount, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromTriangles(v, Array.Empty<int>())).Error);
        Assert.Equal(SoftBodyError.BadCount, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromTriangles(v, TwoIdx)).Error);
        Assert.Equal(SoftBodyError.BadIndex, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromTriangles(v, OutOfRange)).Error);
        Assert.Equal(SoftBodyError.BadIndex, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromTriangles(v, Repeated)).Error);
        var nan = new[] { new Vector2F(0, 0), new Vector2F(float.NaN, 0), new Vector2F(0, 1) };
        Assert.Equal(SoftBodyError.NonFinite, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromTriangles(nan, OneTriangle)).Error);
        var line = new[] { new Vector2F(0, 0), new Vector2F(1, 0), new Vector2F(2, 0) };
        Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromTriangles(line, OneTriangle));
        Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromPolygon(line));
        Assert.Throws<SoftBodyException>(() => SoftBodyMesh.Box(new Vector2F(0f, 1f), 2, 2));
        Assert.Throws<SoftBodyException>(() => SoftBodyMesh.Box(new Vector2F(1f, 1f), 0, 2));
        Assert.Throws<SoftBodyException>(() => SoftBodyMesh.Ring(1f, 2));

        var w = NewWorld();
        var mesh = SoftBodyMesh.Box(new Vector2F(1f, 1f), 2, 2);
        Assert.Throws<SoftBodyException>(() => w.AddSoftBody(new SoftBodyDef()));
        Assert.Throws<SoftBodyException>(() => w.AddSoftBody(new SoftBodyDef { Mesh = mesh, Mass = -1f }));
        Assert.Throws<SoftBodyException>(() => w.AddSoftBody(new SoftBodyDef { Mesh = mesh, EdgeCompliance = float.NaN }));
        Assert.Throws<SoftBodyException>(() => w.AddSoftBody(new SoftBodyDef { Mesh = mesh, Position = new Vector2D(1e9, 0) }));
        Assert.Throws<SoftBodyException>(() => w.AddSoftBody(new SoftBodyDef { Mesh = mesh, PinnedParticles = BadPin }));
        Assert.Throws<SoftBodyException>(() => w.AddSoftBody(new SoftBodyDef { Mesh = mesh, Layer = 40 }));
        // pressure needs a counter-clockwise closed outline
                Assert.Equal(SoftBodyError.Degenerate, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromOutline(new[] { new Vector2F(0, 0), new Vector2F(0, 1), new Vector2F(1, 1), new Vector2F(1, 0) })).Error);
        var bowtie = SoftBodyMesh.FromTriangles(new[] { new Vector2F(0, 0), new Vector2F(1, 0), new Vector2F(0, 1), new Vector2F(-1, 0), new Vector2F(0, -1) }, BowtieTris);
        Assert.False(bowtie.IsClosed);
        Assert.Equal(SoftBodyError.NotClosed, Assert.Throws<SoftBodyException>(() => w.AddSoftBody(new SoftBodyDef { Mesh = bowtie, InternalPressure = 1f })).Error);
        Assert.Equal(0, w.SoftBodyCount);
    }

    [Fact]
    public void AStaleHandleFailsClearlyAndIndicesAreChecked()
    {
        var w = NewWorld();
        var b = w.AddSoftBox(new Vector2F(1f, 1f), new Vector2D(0, 1));
        Assert.True(b.IsValid);
        Assert.Equal(25, b.ParticleCount);
        Assert.Throws<ArgumentOutOfRangeException>(() => b.GetPosition(25));
        Assert.Throws<ArgumentOutOfRangeException>(() => b.GetPosition(-1));
        Assert.Throws<ArgumentOutOfRangeException>(() => b.SetPosition(0, new Vector2D(double.NaN, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => b.ApplyImpulse(0, new Vector2F(float.PositiveInfinity, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => b.SetTarget(0, new Vector2D(0, 0), -1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => b.EdgeCompliance = -1f);
        b.Destroy();
        Assert.False(b.IsValid);
        Assert.Throws<StaleHandleException>(() => b.GetPosition(0));
        Assert.Throws<StaleHandleException>(() => b.Destroy());
        Assert.Throws<InvalidHandleException>(() => default(SoftBody).ParticleCount);
        var c = w.AddSoftBox(new Vector2F(1f, 1f));
        Assert.Equal(c.Slot, b.Slot);
        Assert.NotEqual(b, c);
        Assert.Throws<StaleHandleException>(() => b.Mass);
        Assert.Equal(c, w.GetSoftBody(c.Slot));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.GetSoftBody(5));
    }

    // ---- correctness

    [Fact]
    public void ABalloonHoldsItsAreaUnderAKnownLoad()
    {
        // A pressurised ring resting on a floor with a heavy rigid slab on top: the enclosed area stays at the target within a few percent while the shape flattens.
        var w = NewWorld(o => o.SoftSubsteps = 8);
        Floor(w);
        var balloon = w.AddBalloon(1f, new Vector2D(0, 1.05), 32);
        double rest = balloon.RestArea;
        var slab = w.AddBox(new Vector2F(1.2f, 0.2f), new Vector2D(0, 2.4), density: 50f);
        Run(w, 360);
        double area = balloon.Area;
        var (mn, mx) = balloon.Bounds;
        Assert.True(Math.Abs(area / rest - 1.0) < 0.03, "area " + area + " against rest " + rest);
        Assert.True(mx.Y - mn.Y < 1.85, "the balloon was not squeezed: height " + (mx.Y - mn.Y));
        Assert.True(mx.X - mn.X > 2.1, "the balloon did not bulge sideways: width " + (mx.X - mn.X));
        Assert.True(slab.Position.Y > 0.6, "the slab sank through: " + slab.Position.Y);
    }

    [Fact]
    public void InflatingToAHigherPressureReachesTheRaisedTarget()
    {
        var w = NewWorld(o => { o.Gravity = default; o.SleepEnabled = false; o.SoftSubsteps = 8; });
        var b = w.AddBalloon(1f, new Vector2D(0, 0), 32);
        double rest = b.RestArea;
        b.InternalPressure = 1.5f;
        Run(w, 240);
        Assert.True(Math.Abs(b.Area / (1.5 * rest) - 1.0) < 0.02, "area " + b.Area + " target " + 1.5 * rest);
        b.InternalPressure = 0f;
        Assert.Equal(0f, b.InternalPressure);
    }

    [Fact]
    public void AShapeMatchedBodyReturnsToItsRestShapeAfterRelease()
    {
        foreach (bool quad in new[] { false, true })
        {
            var w = NewWorld(o => { o.Gravity = default; o.SleepEnabled = false; });
            var def = SoftBodyDef.Blob(SoftBodyMesh.Disc(1f, 3, 18), new Vector2D(0, 0));
            def.MatchQuadratic = quad; def.MatchBlend = 0.2f; def.LinearDamping = 1f; def.MatchCompliance = 1e-4f;
            var b = w.AddSoftBody(def);
            int n = b.ParticleCount;
            var rest = new Vector2D[n];
            b.CopyPositions(rest);
            // squash it flat, then let go
            for (int i = 0; i < n; i++) { var p = rest[i]; b.SetPosition(i, new Vector2D(p.X * 1.3, p.Y * 0.4)); }
            Run(w, 600);
            var now = new Vector2D[n];
            b.CopyPositions(now);
            // centre and compare to the rest shape up to a rotation: the body is free, so only the shape counts
            double cx = 0, cy = 0;
            for (int i = 0; i < n; i++) { cx += now[i].X; cy += now[i].Y; }
            cx /= n; cy /= n;
            double a = 0, bb = 0;
            for (int i = 0; i < n; i++) { double px = now[i].X - cx, py = now[i].Y - cy; a += px * rest[i].X + py * rest[i].Y; bb += py * rest[i].X - px * rest[i].Y; }
            double l = Math.Sqrt(a * a + bb * bb); double c = a / l, s = bb / l;
            double worst = 0;
            for (int i = 0; i < n; i++)
            {
                double px = now[i].X - cx, py = now[i].Y - cy;
                double rx = c * px + s * py, ry = -s * px + c * py;
                worst = Math.Max(worst, Math.Sqrt((rx - rest[i].X) * (rx - rest[i].X) + (ry - rest[i].Y) * (ry - rest[i].Y)));
            }

            Assert.True(worst < 0.05, "quad=" + quad + ": worst particle error " + worst + " after release");
        }
    }

    [Fact]
    public void EdgeStiffnessDoesNotDependOnTheSubstepCount()
    {
        // A strip hung by its top row: the static stretch under gravity is set by the compliance, not by how finely the solver steps.
        var ext = new List<double>();
        foreach (int substeps in new[] { 4, 8, 16, 32 })
        {
            var w = NewWorld(o => { o.SoftSubsteps = substeps; o.SoftIterations = 1; o.SleepEnabled = false; });
            var def = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(0.4f, 2f), 2, 10), new Vector2D(0, 0));
            def.EdgeCompliance = 3e-3f; def.AreaCompliance = 1e6f; def.EdgeDamping = 0.01f; def.LinearDamping = 3f; def.Mass = 4f;
            var pins = new List<int>(); for (int i = 30; i < 33; i++) { pins.Add(i); }
            def.PinnedParticles = pins;
            var b = w.AddSoftBody(def);
            Run(w, 600);
            var (mn, mx) = b.Bounds;
            ext.Add(mx.Y - mn.Y);
        }

        double lo = double.MaxValue, hi = double.MinValue;
        foreach (double e in ext) { lo = Math.Min(lo, e); hi = Math.Max(hi, e); }
        Assert.True(hi > 2.0, "the strip did not stretch: " + string.Join(",", ext));
        Assert.True((hi - lo) / (hi - 2.0) < 0.15, "stretch depends on the substep count: " + string.Join(", ", ext));
    }

    [Fact]
    public void MomentumIsConservedInTwoWayCoupling()
    {
        var w = NewWorld(o => { o.Gravity = default; o.SleepEnabled = false; });
        var def = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(1f, 1f), 4, 4), new Vector2D(0, 0));
        def.Mass = 2f; def.LinearDamping = 0f; def.Friction = 0.3f;
        var soft = w.AddSoftBody(def);
        var ball = w.AddCircle(0.3f, new Vector2D(-2.0, 0.1));
        ball.LinearVelocity = new Vector2F(8f, 0f);
        float mb = ball.Mass;
        double p0 = mb * 8.0, worst = 0;
        bool touched = false;
        for (int i = 0; i < 120; i++)
        {
            w.Step(Dt);
            var ps = soft.Momentum;
            var vb = ball.LinearVelocity;
            double px = ps.X + mb * vb.X, py = ps.Y + mb * vb.Y;
            worst = Math.Max(worst, Math.Abs(px - p0) + Math.Abs(py));
            if (ps.X > 0.5) { touched = true; }
        }

        Assert.True(touched, "the ball never hit the jelly");
        Assert.True(worst < 0.02 * p0, "momentum drifted by " + worst + " of " + p0);
    }

    [Fact]
    public void ABouncingJellyGainsNoEnergyOverTenThousandSteps()
    {
        var w = NewWorld(o => { o.SleepEnabled = false; });
        Floor(w);
        var def = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(1f, 1f), 4, 4), new Vector2D(0, 3));
        def.Mass = 1f; def.LinearDamping = 0f; def.AllowSleep = false;
        var b = w.AddSoftBody(def);
        double e0 = Energy(b, 9.81);
        double worst = 0, emax = e0;
        for (int i = 0; i < 10000; i++)
        {
            w.Step(Dt);
            double e = Energy(b, 9.81);
            emax = Math.Max(emax, e);
            if (i % 50 == 0) { worst = Math.Max(worst, e - e0); }
        }

        Assert.True(worst < 0.02 * e0, "energy grew by " + worst + " from " + e0);
        Assert.True(Energy(b, 9.81) < e0, "no energy was lost at all");
    }

    private static double Energy(SoftBody b, double g)
    {
        // kinetic + potential of the free particles about y = 0; elastic energy is ignored (it only makes the real total larger, so growth of this sum bounds the growth of the real one from below, and the bound is what matters)
        var pos = b.RawPositions; var vel = b.RawVelocities; var inv = b.RawInverseMasses;
        double e = 0;
        for (int i = 0; i < b.ParticleCount; i++)
        {
            if (inv[i] == 0f) { continue; }
            double m = 1.0 / inv[i];
            double vx = vel[2 * i], vy = vel[2 * i + 1];
            e += 0.5 * m * (vx * vx + vy * vy) + m * g * pos[2 * i + 1];
        }

        return e;
    }
}

public class SoftBody2DScenes
{
    // Thresholds are our own (no reference engine has 2D soft bodies): about 1.5 times the values this engine measured, rounded up, recorded in docs/spec/scenes.md.
    // maxSpeed, maxPenetration, maxSoftOverlap, areaError (of pressure bodies, at the end) and finalSpeed99 (the 99th percentile particle speed over the last 3 steps); escapes and nan must be 0.
    [Theory]
    [InlineData("soft-pile-2d", 25.0, 0.08, 0.05, 0.15, 1.5)]
    [InlineData("soft-rigid-2d", 15.0, 0.05, 0.02, 0.10, 1.5)]
    [InlineData("balloon-2d", 20.0, 0.06, 0.02, 0.05, 1.0)]
    public void TheSceneIsStable(string name, double maxSpeed, double maxPenetration, double areaError, double softOverlap, double finalSpeed99)
    {
        var m = SoftBodies2DScene.Measure(name);
        Assert.Equal(0, m.NaN);
        Assert.Equal(0, m.Escapes);
        Assert.True(m.Bodies >= (name == "soft-pile-2d" ? 100 : 5), name + " bodies " + m.Bodies);
        Assert.True(m.MaxSpeed <= maxSpeed, SoftBodies2DScene.Format(m));
        Assert.True(m.MaxPenetration <= maxPenetration, SoftBodies2DScene.Format(m));
        Assert.True(m.AreaError <= areaError, SoftBodies2DScene.Format(m));
        Assert.True(m.MaxSoftOverlap <= softOverlap, SoftBodies2DScene.Format(m));
        Assert.True(m.FinalSpeed99 <= finalSpeed99, SoftBodies2DScene.Format(m));
    }

    [Theory]
    [InlineData("soft-pile-2d")]
    [InlineData("soft-rigid-2d")]
    [InlineData("balloon-2d")]
    public void TheSceneIsTheSameOnAnyThreadCountAndAfterARestore(string name)
    {
        ulong a = SoftBodies2DScene.Run(name, 1, false);
        Assert.Equal(a, SoftBodies2DScene.Run(name, 8, false));
        Assert.Equal(a, SoftBodies2DScene.Run(name, 1, true));
        Assert.Equal(a, SoftBodies2DScene.Run(name, 8, true));
    }
}
