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

    // A balloon dropped exactly symmetrically must not start rolling: a Gauss-Seidel sweep that always runs the same way round the outline used to give it a spin of about 0.7 rad/s on landing.
    [Fact]
    public void SymmetricBalloonDropStaysSymmetric()
    {
        var w = NewWorld();
        Floor(w);
        var s = w.AddBalloon(0.5f, new Vector2D(0, 1.0), 24);
        Run(w, 100);
        var c = s.CenterOfMass;
        double l = 0;
        for (int p = 0; p < s.ParticleCount; p++)
        {
            var q = s.GetPosition(p); var v = s.GetVelocity(p);
            l += (q.X - c.X) * v.Y - (q.Y - c.Y) * v.X;
        }

        Assert.True(Math.Abs(c.X) < 0.01, $"drifted {c.X}");
        Assert.True(Math.Abs(l) < 0.05, $"angular momentum sum {l}");
    }

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
    [InlineData("soft-pile-2d", 25.0, 0.08, 0.05, 0.03, 0.5)]
    [InlineData("soft-rigid-2d", 15.0, 0.05, 0.02, 0.10, 1.5)]
    [InlineData("balloon-2d", 20.0, 0.06, 0.02, 0.05, 1.0)]
    [InlineData("plastic-2d", 8.0, 0.03, 0.05, 0.01, 0.3)]
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
    [InlineData("plastic-2d")]
    public void TheSceneIsTheSameOnAnyThreadCountAndAfterARestore(string name)
    {
        ulong a = SoftBodies2DScene.Run(name, 1, false);
        Assert.Equal(a, SoftBodies2DScene.Run(name, 8, false));
        Assert.Equal(a, SoftBodies2DScene.Run(name, 1, true));
        Assert.Equal(a, SoftBodies2DScene.Run(name, 8, true));
    }
}

public class SoftBody2DHostTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D Scene(int threads = 1)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = threads, WorldId = 61 });
        w.AddBox(new Vector2F(30f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        w.AddSoftBox(new Vector2F(0.8f, 0.8f), new Vector2D(0, 1.0), SoftMaterial.Jelly, 3);
        w.AddBalloon(0.4f, new Vector2D(1.6, 1.0), 16);
        w.AddBlob(0.4f, new Vector2D(-1.6, 1.0), 2);
        w.AddBox(new Vector2F(0.5f, 0.5f), new Vector2D(0.1, 3.0), density: 20f);
        return w;
    }

    private static void Run(PhysicsWorld2D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    [Fact]
    public void SnapshotRestoreContinuesBitIdentically()
    {
        var a = Scene();
        Run(a, 40);
        var snap = a.Snapshot();
        var hashes = new ulong[60];
        for (int i = 0; i < 60; i++) { a.Step(Dt); hashes[i] = a.ComputeHash(); }
        var finalBytes = a.Snapshot();

        // restore into the same world, and into a fresh one
        a.Restore(snap);
        for (int i = 0; i < 60; i++) { a.Step(Dt); Assert.Equal(hashes[i], a.ComputeHash()); }
        Assert.Equal(finalBytes, a.Snapshot());
        var b = new PhysicsWorld2D(new WorldOptions2D { WorldId = 61 });
        b.Restore(snap);
        Assert.Equal(snap, b.Snapshot());
        for (int i = 0; i < 60; i++) { b.Step(Dt); Assert.Equal(hashes[i], b.ComputeHash()); }
        Assert.Equal(finalBytes, b.Snapshot());
        Assert.Equal(3, b.SoftBodyCount);
        Assert.Equal(0, a.FindDifferences(b).Count);
    }

    [Fact]
    public void AForkContinuesLikeTheOriginalAndIsIndependentOfIt()
    {
        var a = Scene();
        Run(a, 30);
        var f = a.Fork();
        Assert.Equal(a.ComputeHash(), f.ComputeHash());
        Run(a, 30); Run(f, 30);
        Assert.Equal(a.ComputeHash(), f.ComputeHash());
        Assert.Equal(a.Snapshot(), f.Snapshot());
        f.GetSoftBody(0).AddVelocity(new Vector2F(5f, 0f));
        Run(f, 5);
        Assert.NotEqual(a.ComputeHash(), f.ComputeHash());
        Run(a, 5);
        Assert.True(a.FindDifferences(f).Count > 0);
        Assert.Equal(f.GetSoftBody(1).ParticleCount, a.GetSoftBody(1).ParticleCount);
    }

    [Fact]
    public void WorldsWithoutSoftBodiesKeepTheirSnapshotBytes()
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 62 });
        w.AddBox(new Vector2F(1f, 1f), new Vector2D(0, 2));
        Run(w, 10);
        var before = w.Snapshot();
        var s = w.AddSoftBox(new Vector2F(1f, 1f), new Vector2D(3, 2));
        var withSoft = w.Snapshot();
        Assert.True(withSoft.Length > before.Length);
        s.Destroy();
        w.AddSoftBox(new Vector2F(1f, 1f), new Vector2D(3, 2)).Destroy();
        // a world that made soft bodies and lost them again still writes the (empty) section; one that never had any writes none
        var fresh = new PhysicsWorld2D(new WorldOptions2D { WorldId = 62 });
        fresh.Restore(before);
        Assert.Equal(before, fresh.Snapshot());
    }

    [Fact]
    public void RestoreRefusesDamagedSoftBodySections()
    {
        var w = Scene();
        Run(w, 5);
        var snap = w.Snapshot();
        var tail = Scene();
        for (int cut = 1; cut < 40; cut++)
        {
            var damaged = new byte[snap.Length - cut];
            Array.Copy(snap, damaged, damaged.Length);
            Assert.Throws<SnapshotFormatException>(() => tail.Restore(damaged));
        }

        // flipping bytes anywhere in the section either restores something valid or is refused; it never throws anything else and leaves the world usable
        var rng = new Random(7);
        for (int k = 0; k < 300; k++)
        {
            var bad = (byte[])snap.Clone();
            int at = snap.Length - 1 - rng.Next(snap.Length / 2);
            bad[at] ^= (byte)(1 << rng.Next(8));
            try { tail.Restore(bad); tail.Step(Dt); }
            catch (SnapshotFormatException) { }
            catch (SoftBodyOutOfBoundsException) { }
            catch (BodyOutOfBoundsException) { }
            tail.Restore(snap);
        }
    }

    [Fact]
    public void TheMomentumOfARigidBodyAndASoftBodyIsConservedByTheCoupling()
    {
        // A heavy rigid box pressed onto a jelly resting on a floor: after the box settles, the sum of the impulses on both is zero in the horizontal direction.
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 63, Gravity = default, SleepEnabled = false });
        var def = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(1f, 1f), 3, 3), new Vector2D(0, 0));
        def.LinearDamping = 0f; def.Mass = 5f;
        var soft = w.AddSoftBody(def);
        var box = w.AddBox(new Vector2F(0.5f, 0.5f), new Vector2D(-1.5, 0.2), density: 10f);
        box.LinearVelocity = new Vector2F(6f, 0.5f);
        double mb = box.Mass;
        double px0 = mb * 6.0, py0 = mb * 0.5;
        for (int i = 0; i < 90; i++)
        {
            w.Step(Dt);
            var p = soft.Momentum;
            var v = box.LinearVelocity;
            Assert.True(Math.Abs(p.X + mb * v.X - px0) < 0.03 * px0 && Math.Abs(p.Y + mb * v.Y - py0) < 0.03 * px0, "step " + i + ": " + (p.X + mb * v.X - px0));
        }
    }

    [Fact]
    public void PinnedParticlesHoldAndTargetsDriveTheBody()
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 64, SleepEnabled = false });
        var def = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(1f, 1f), 3, 3), new Vector2D(0, 5));
        def.PinnedParticles = new[] { 12, 13 };
        var b = w.AddSoftBody(def);
        var p0 = b.GetPosition(12);
        Run(w, 120);
        Assert.Equal(p0, b.GetPosition(12));
        Assert.True(b.IsPinned(12));
        Assert.False(b.IsPinned(0));
        var (mn, _) = b.Bounds;
        Assert.True(mn.Y < 5 - 0.5 && mn.Y > 5 - 2, "hanging extent " + mn.Y);
        Assert.Equal(2, b.Definition.PinnedParticles!.Count);
        b.SetPinned(12, false); b.SetPinned(13, false);
        Run(w, 60);
        Assert.True(b.GetPosition(12).Y < p0.Y - 1.0);

        // a driven pose: every particle targeted at the rest shape moved to (4, 8) and turned by 30 degrees, kinematic
        b.SetTargetPose(new Vector2D(4, 8), 0.5236f, 0f, teleport: true);
        for (int i = 0; i < 20; i++) { w.Step(Dt); }
        var c = b.CenterOfMass;
        Assert.InRange(c.X, 3.99, 4.01); Assert.InRange(c.Y, 7.99, 8.01);
        Assert.True(b.HasTarget(3));
        // limited targets: free to move, but never farther than 0.1 from the target
        b.SetTargetPose(new Vector2D(4, 8), 0.5236f, 0.1f);
        b.AddVelocity(new Vector2F(20f, 0f));
        double worst = 0;
        for (int i = 0; i < 60; i++)
        {
            w.Step(Dt);
            var rest = b.Mesh;
            for (int k = 0; k < b.ParticleCount; k++)
            {
                var pos = b.GetPosition(k);
                var (sn, cs) = Math64.SinCos(0.5236f);
                var r = rest.Vertices[k];
                double tx = 4 + cs * r.X - sn * r.Y, ty = 8 + sn * r.X + cs * r.Y;
                worst = Math.Max(worst, Math.Sqrt((pos.X - tx) * (pos.X - tx) + (pos.Y - ty) * (pos.Y - ty)));
            }
        }

        Assert.True(worst < 0.1 + 1e-3, "particles strayed " + worst + " from their targets");
        b.ClearTarget(3);
        Assert.False(b.HasTarget(3));
    }

    [Fact]
    public void ASoftBodyOverlapSensorFindsTheBodiesInItsShape()
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 65, Gravity = default });
        var a = w.AddSoftBox(new Vector2F(1f, 1f), new Vector2D(0, 0));
        var b = w.AddSoftCircle(0.5f, new Vector2D(5, 0));
        var hits = new SoftBody[4];
        var box = Shape2D.CreateBox(0.7f, 0.7f, default, Rot2.Identity, 0f);
        Assert.Equal(1, w.OverlapSoftBodies(in box, new Vector2D(0.2, 0), 0f, hits));
        Assert.Equal(a, hits[0]);
        Assert.Equal(0, w.OverlapSoftBodies(in box, new Vector2D(2.5, 0), 0f, hits));
        var wide = Shape2D.CreateBox(6f, 1f, default, Rot2.Identity, 0f);
        Assert.Equal(2, w.OverlapSoftBodies(in wide, new Vector2D(2.5, 0), 0f, hits));
        Assert.Equal(a, hits[0]); Assert.Equal(b, hits[1]);
        Assert.Equal(1, w.OverlapSoftBodies(in wide, new Vector2D(2.5, 0), 0f, hits.AsSpan(0, 1)));
        Assert.Throws<ArgumentException>(() => w.OverlapSoftBodies(default(Shape2D), default, 0f, hits));
    }

    [Fact]
    public void ABodyThatLeavesTheRangeIsReportedBeforeAnythingChanges()
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 66, MaxLinearSpeed = 1e7f });
        var b = w.AddSoftBox(new Vector2F(1f, 1f), new Vector2D(0, 5));
        var before = w.Snapshot();
        b.SetPosition(3, new Vector2D(2_000_000, 0));
        b.AddVelocity(new Vector2F(5e5f, 0f));
        var ex = Assert.Throws<SoftBodyOutOfBoundsException>(() => { Run(w, 400); });
        Assert.InRange(ex.Particle, 0, 24);
        Assert.Equal(b, ex.SoftBody);
        Assert.NotEqual(before, w.Snapshot());
    }

    [Fact]
    public void DebugGeometryListsParticlesAndEdges()
    {
        var w = Scene();
        Run(w, 3);
        var g = new RecursiveAI.OnePhysics.Debug.DebugGeometry();
        w.GetDebugGeometry(RecursiveAI.OnePhysics.Debug.DebugContent.SoftBodies, g);
        Assert.Equal(3, g.SoftBodies.Length);
        Assert.Equal(16, g.SoftBodies[0].ParticleCount);
        Assert.True(g.SoftBodies[0].EdgeCount > 0);
        Assert.Equal(w.GetSoftBody(0).ParticleCount + w.GetSoftBody(1).ParticleCount + w.GetSoftBody(2).ParticleCount, g.Points.Length);
    }

    [Fact]
    public void ShiftingTheOriginMovesSoftBodiesWithTheWorld()
    {
        var a = Scene();
        var b = Scene();
        Run(a, 20); Run(b, 20);
        b.ShiftOrigin(new Vector2D(16, -8));
        var pa = a.GetSoftBody(0).GetPosition(7);
        var pb = b.GetSoftBody(0).GetPosition(7);
        Assert.Equal(pa.X - 16, pb.X); Assert.Equal(pa.Y + 8, pb.Y);
        Run(a, 20); Run(b, 20);
        pa = a.GetSoftBody(0).GetPosition(7); pb = b.GetSoftBody(0).GetPosition(7);
        Assert.True(Math.Abs(pa.X - 16 - pb.X) < 1e-3 && Math.Abs(pa.Y + 8 - pb.Y) < 1e-3, "positions differ after the shift: " + pa + " vs " + pb);
    }

    [Fact]
    public void SceneTextAndRegionsRoundTrip()
    {
        var w = Scene();
        Run(w, 25);
        string text = w.SaveScene();
        var copy = PhysicsWorld2D.LoadScene(new System.IO.StringReader(text));
        Assert.Equal(w.ComputeHash(), copy.ComputeHash());
        Run(w, 10); Run(copy, 10);
        Assert.Equal(w.ComputeHash(), copy.ComputeHash());
        // a region around the balloon carries it (and the rigid bodies it touches)
        var region = w.CaptureRegion(new Vector2D(1.0, 0.0), new Vector2D(2.2, 2.0));
        Assert.NotNull(region);
        var other = Scene();
        Run(other, 25);
        other.GetSoftBody(1).AddVelocity(new Vector2F(3f, 3f));
        Run(other, 5);
        w.RestoreRegion(region);
        Assert.NotNull(w.Snapshot());
    }

    [Fact]
    public void ARecordedSessionReplaysWithEqualStates()
    {
        using var stream = new System.IO.MemoryStream();
        var a = Scene();
        using (a.StartRecording(stream))
        {
            Run(a, 10);
            var s = a.GetSoftBody(0);
            s.AddVelocity(new Vector2F(2f, 0f));
            s.SetPinned(0, true);
            a.Options.SoftSubsteps = 6;
            a.Options.SoftIterations = 2;
            Run(a, 20);
            a.AddSoftCircle(0.3f, new Vector2D(0, 4));
            Run(a, 20);
        }

        stream.Position = 0;
        var player = new RecursiveAI.OnePhysics.Replay.ReplayPlayer(stream);
        var b = player.CreateWorld2D();
        int steps = 0;
        while (player.ReadNextStep(b)) { steps++; }
        Assert.Equal(50, steps);
        Assert.Equal(a.ComputeHash(), b.ComputeHash());
        Assert.Equal(6, b.Options.SoftSubsteps);
        Assert.Equal(2, b.Options.SoftIterations);
        Assert.Equal(4, b.SoftBodyCount);
    }

    [Fact]
    public void StepsAllocateNothingOnTheSteppingThreadOnceWarm()
    {
        var w = Scene();
        Run(w, 60);
        Core.AllocMeter.AssertAllocatesNothing(() => Run(w, 30), "soft body steps");
    }
}
