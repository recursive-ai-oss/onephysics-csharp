using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

public class World3DTests
{
    private const float Dt = 1f / 60f;

    internal static PhysicsWorld3D NewWorld(int threads = 1, int id = 7, Action<WorldOptions3D>? tweak = null)
    {
        var o = new WorldOptions3D { ThreadCount = threads, WorldId = id };
        tweak?.Invoke(o);
        return new PhysicsWorld3D(o);
    }

    internal static Body Floor(PhysicsWorld3D w, float half = 50f) => w.AddBox(new Vector3F(2 * half, 1f, 2 * half), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);

    private static QuaternionF AxisAngle(Vector3F axis, float angle)
    {
        double s = Math64.Sin(angle * 0.5), c = Math64.Cos(angle * 0.5);
        return new QuaternionF(axis.X * (float)s, axis.Y * (float)s, axis.Z * (float)s, (float)c);
    }

    /// <summary>A static floor, a stack of boxes, a heap of mixed shapes and a compound: enough contact variety for determinism checks.</summary>
    internal static PhysicsWorld3D BuildScene(int threads, int id = 7, Action<WorldOptions3D>? tweak = null)
    {
        var w = NewWorld(threads, id, tweak);
        Floor(w);
        for (int row = 0; row < 4; row++)
        {
            for (int i = 0; i < 4 - row; i++) { w.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(-2 + row * 0.5 + i * 1.02, 0.5 + row, 0)); }
        }

        Vector3F[] octa = [new(0.5f, 0, 0), new(-0.5f, 0, 0), new(0, 0.5f, 0), new(0, -0.5f, 0), new(0, 0, 0.5f), new(0, 0, -0.5f)];
        for (int i = 0; i < 24; i++)
        {
            double x = -4 + (i % 6) * 1.5, y = 6 + (i / 6) * 1.6, z = 3 + (i % 3) * 0.3;
            var q = AxisAngle(new Vector3F(0, 0, 1), 0.3f * i);
            switch (i % 5)
            {
                case 0: w.AddSphere(0.4f, new Vector3D(x, y, z)); break;
                case 1: w.AddCapsule(0.3f, 0.25f, new Vector3D(x, y, z), q); break;
                case 2: w.AddBox(new Vector3F(0.8f, 0.6f, 0.7f), new Vector3D(x, y, z), q); break;
                case 3: w.AddConvexHull(octa, new Vector3D(x, y, z), q); break;
                default: w.AddCylinder(0.3f, 0.35f, new Vector3D(x, y, z), q); break;
            }
        }

        w.AddCompound([new CompoundPart(ShapeSpec3D.Box(new Vector3F(1.2f, 0.4f, 0.4f)), default), new CompoundPart(ShapeSpec3D.Sphere(0.3f), new Vector3F(0.6f, 0.3f, 0f))], new Vector3D(1, 4, -2));
        return w;
    }

    [Fact]
    public void FreeFallMatchesTheIntegratorFormula()
    {
        // A body with no joint runs the contact-only schedule (2 substeps by default); the world's own schedule has 4.
        foreach (var (contactSubsteps, substeps) in new[] { (0, 4), (2, 2) })
        {
            var w = NewWorld(tweak: o => { o.SleepEnabled = false; o.ContactSubsteps = contactSubsteps; });
            var ball = w.AddSphere(0.5f, new Vector3D(0, 100, 0));
            for (int i = 0; i < 60; i++) { w.Step(Dt); }
            // Semi-implicit Euler: after n substeps v = -g h n and y = y0 - g h^2 n (n + 1) / 2, h = dt / substeps.
            double h = (double)Dt / substeps, n = 60 * substeps;
            Assert.Equal(-9.81 * h * n, ball.LinearVelocity.Y, 3);
            Assert.Equal(100 - 9.81 * h * h * n * (n + 1) / 2, ball.Position.Y, 3);
            Assert.Equal(0.0, ball.Position.X);
            Assert.Equal(0.0, ball.Position.Z);
        }
    }

    [Fact]
    public void ABoxRestsOnTheGroundAndSleeps()
    {
        var w = NewWorld();
        Floor(w);
        var box = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 2, 0));
        for (int i = 0; i < 240; i++) { w.Step(Dt); }
        Assert.Equal(0.5, box.Position.Y, 2);
        Assert.False(box.IsAwake);
    }

    [Fact]
    public void ABoxOnABoxKeepsItsFourPointsAndDoesNotDrift()
    {
        var w = NewWorld(tweak: o => o.SleepEnabled = false);
        Floor(w);
        var low = w.AddBox(new Vector3F(2, 1, 2), at: new Vector3D(0, 0.5, 0));
        var top = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0.1, 1.5, -0.2));
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        var p0 = top.Position;
        int maxPoints = 0;
        for (int i = 0; i < 1000; i++)
        {
            w.Step(Dt);
            var cs = w.Core.Constraints;
            for (int c = 0; c < cs.HighWater; c++)
            {
                if (cs.IsLive(c) && cs.BodyA[c] != cs.BodyB[c] && cs.PointCount[c] > 0 && (w.Core.HandleOfBody(cs.BodyA[c]) == low.RawHandle || w.Core.HandleOfBody(cs.BodyB[c]) == low.RawHandle)
                    && (w.Core.HandleOfBody(cs.BodyA[c]) == top.RawHandle || w.Core.HandleOfBody(cs.BodyB[c]) == top.RawHandle))
                {
                    maxPoints = Math.Max(maxPoints, cs.PointCount[c]);
                }
            }
        }

        Assert.Equal(4, maxPoints);
        var p1 = top.Position;
        Assert.InRange(Math.Abs(p1.X - p0.X) + Math.Abs(p1.Z - p0.Z), 0.0, 1e-3);
        Assert.InRange(Math.Abs(p1.Y - p0.Y), 0.0, 1e-3);
        Assert.InRange(Math.Abs(top.AngularVelocity.X) + Math.Abs(top.AngularVelocity.Y) + Math.Abs(top.AngularVelocity.Z), 0f, 1e-2f);
    }

    [Fact]
    public void ASphereRollsWithoutSlippingOnAPlane()
    {
        var w = NewWorld(tweak: o => o.SleepEnabled = false);
        // No rolling resistance, which would slow the ball below the ideal rolling speed.
        var noRolling = new Material3D(0.6f, 0f, 0f);
        w.AddBox(new Vector3F(100f, 1f, 100f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static, material: noRolling);
        var ball = w.AddSphere(0.5f, new Vector3D(0, 0.5, 0), material: noRolling);
        ball.LinearVelocity = new Vector3F(4f, 0, 0);
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        // A solid sphere sliding at v0 ends rolling at 5/7 v0 with w = v / r (about -z for motion along +x).
        Assert.Equal(4f * 5f / 7f, ball.LinearVelocity.X, 1);
        Assert.Equal(-ball.LinearVelocity.X / 0.5f, ball.AngularVelocity.Z, 1);
        Assert.Equal(0f, ball.AngularVelocity.X, 2);
    }

    [Fact]
    public void ARollingBallComesToRestWithTheDefaultMaterial()
    {
        // The default rolling resistance (0.003) slows a ball rolling at 1 m/s at about c g / (1.4 r) = 0.14 m/s^2, so it stops within about seven seconds (measured: 0.17 m/s left after five).
        var w = NewWorld(tweak: o => o.SleepEnabled = false);
        Floor(w);
        var ball = w.AddSphere(0.15f, new Vector3D(0, 0.15, 0));
        ball.LinearVelocity = new Vector3F(1f, 0, 0);
        ball.AngularVelocity = new Vector3F(0, 0, -1f / 0.15f);
        for (int i = 0; i < 300; i++) { w.Step(Dt); }
        float mid = ball.LinearVelocity.X;
        Assert.InRange(mid, 0.05f, 0.5f);
        for (int i = 0; i < 420; i++) { w.Step(Dt); }
        Assert.True(Math.Abs(ball.LinearVelocity.X) < 0.05f && ball.AngularVelocity.Length() * 0.15f < 0.05f, "the ball still rolls: v = " + ball.LinearVelocity.X);
    }

    [Fact]
    public void ATorqueFreeAsymmetricBodyConservesAngularMomentum()
    {
        var w = NewWorld(tweak: o => { o.SleepEnabled = false; o.Gravity = default; o.ContactSubsteps = 0; });
        var body = w.AddBox(new Vector3F(1f, 2f, 4f), at: new Vector3D(0, 0, 0));
        body.AngularVelocity = new Vector3F(0.2f, 3f, 0.1f); // near the intermediate axis: the tumbling case
        Vector3F L()
        {
            var q = body.Rotation;
            var I = body.Inertia;
            var wl = q.Conjugated().Rotate(body.AngularVelocity);
            return q.Rotate(I * wl);
        }

        var l0 = L();
        float maxDrift = 0f;
        for (int i = 0; i < 600; i++)
        {
            w.Step(Dt);
            var l = L();
            maxDrift = Math.Max(maxDrift, (l - l0).Length() / l0.Length());
        }

        // One implicit gyroscopic Newton step per substep: the measured drift over 10 s of tumbling stays below 1.5 % at 4 substeps (a free body on the contact-only schedule, 2 substeps, drifts 2.9 %).
        Assert.InRange(maxDrift, 0f, 0.015f);
    }

    [Fact]
    public void RestitutionSetsTheBounceHeight()
    {
        var w = NewWorld(tweak: o => o.SleepEnabled = false);
        var floor = Floor(w);
        floor.GetShapes()[0].Restitution = 0.5f;
        var ball = w.AddSphere(0.25f, new Vector3D(0, 5.25, 0));
        ball.GetShapes()[0].Restitution = 0.5f;
        bool bounced = false;
        double apex = 0;
        for (int i = 0; i < 300 && !(bounced && ball.LinearVelocity.Y < 0); i++)
        {
            w.Step(Dt);
            if (ball.LinearVelocity.Y > 1f) { bounced = true; }
            if (bounced) { apex = Math.Max(apex, ball.Position.Y); }
        }

        Assert.True(bounced);
        // Impact speed sqrt(2 g 5) = 9.9 m/s, rebound at 0.5 of it: apex 0.25 * 5 = 1.25 m above the resting height 0.25.
        Assert.InRange(apex - 0.25, 1.25 * 0.9, 1.25 * 1.1);
    }

    [Fact]
    public void AFastSphereDoesNotPassAThinStaticBox()
    {
        var w = NewWorld(tweak: o => { o.Gravity = default; o.SleepEnabled = false; });
        w.AddBox(new Vector3F(0.02f, 10f, 10f), at: new Vector3D(5, 0, 0), type: BodyType.Static);
        var ball = w.AddSphere(0.1f, new Vector3D(0, 0, 0));
        ball.LinearVelocity = new Vector3F(300f, 0, 0);
        for (int i = 0; i < 20; i++) { w.Step(Dt); }
        Assert.True(ball.Position.X < 5.0, "the ball tunnelled to x = " + ball.Position.X);
    }

    [Fact]
    public void AFastSphereDoesNotPassAMesh()
    {
        var w = NewWorld(tweak: o => { o.Gravity = default; o.SleepEnabled = false; });
        Vector3F[] v = [new(5, -10, -10), new(5, -10, 10), new(5, 10, 10), new(5, 10, -10)];
        w.AddMesh(v, [0, 2, 1, 0, 3, 2]);
        var ball = w.AddSphere(0.1f, new Vector3D(0, 0, 0));
        ball.LinearVelocity = new Vector3F(300f, 0, 0);
        for (int i = 0; i < 20; i++) { w.Step(Dt); }
        Assert.True(ball.Position.X < 5.0, "the ball tunnelled to x = " + ball.Position.X);
    }

    // Ice without rolling resistance: the movers start without spin, and the question is the seams, not the resistance.
    private static readonly Material3D Ice = new(0.05f, 0f, 0f);

    /// <summary>A flat two-triangle-per-cell mesh: a box sliding across the seams must not jump.</summary>
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void ABoxSlidingOverInternalMeshEdgesGetsNoGhostBump(int shape)
    {
        var w = NewWorld(tweak: o => o.SleepEnabled = false);
        var verts = new List<Vector3F>();
        var idx = new List<int>();
        const int N = 8;
        for (int j = 0; j <= N; j++) { for (int i = 0; i <= N; i++) { verts.Add(new Vector3F(i * 2f - 8f, 0f, j * 2f - 8f)); } }
        for (int j = 0; j < N; j++)
        {
            for (int i = 0; i < N; i++)
            {
                int a = j * (N + 1) + i, b = a + 1, c = a + N + 1, d = c + 1;
                idx.AddRange([a, c, b, b, c, d]);
            }
        }

        w.AddMesh(verts.ToArray(), idx.ToArray(), material: Ice);
        var ice = Ice;
        Body mover = shape == 0 ? w.AddBox(new Vector3F(0.8f, 0.4f, 0.8f), new Vector3D(-6, 0.2, -5.3), material: ice) : w.AddSphere(0.3f, new Vector3D(-6, 0.3, -5.3), material: ice);
        for (int i = 0; i < 90; i++) { w.Step(Dt); }
        mover.LinearVelocity = new Vector3F(6f, 0f, 0.8f);
        float maxUp = 0f;
        double minY = double.MaxValue;
        for (int i = 0; i < 120; i++)
        {
            w.Step(Dt);
            maxUp = Math.Max(maxUp, mover.LinearVelocity.Y);
            minY = Math.Min(minY, mover.Position.Y);
        }

        // Vertical speed stays at noise level while crossing the seams (a ghost contact kicks the body up by tens of cm/s).
        Assert.InRange(maxUp, -1f, 0.02f);
        Assert.True(mover.Position.X > 2.0, "the mover did not slide: x = " + mover.Position.X);
    }

    /// <summary>The same grid with a spinning sphere: spin adds tangential speed at the contact, which must not turn a seam into a ramp.</summary>
    [Theory]
    [InlineData(0f)]
    [InlineData(20f)]
    [InlineData(-20f)]
    public void ASpinningSphereSlidingOverMeshSeamsIsNotKickedUp(float spin)
    {
        var w = NewWorld(tweak: o => o.SleepEnabled = false);
        var verts = new List<Vector3F>();
        var idx = new List<int>();
        const int N = 8;
        for (int j = 0; j <= N; j++) { for (int i = 0; i <= N; i++) { verts.Add(new Vector3F(i * 2f - 8f, 0f, j * 2f - 8f)); } }
        for (int j = 0; j < N; j++)
        {
            for (int i = 0; i < N; i++)
            {
                int a = j * (N + 1) + i, b = a + 1, c = a + N + 1, d = c + 1;
                idx.AddRange([a, c, b, b, c, d]);
            }
        }

        w.AddMesh(verts.ToArray(), idx.ToArray(), material: Ice);
        var ball = w.AddSphere(0.3f, new Vector3D(-6, 0.3, -5.3), material: Ice);
        for (int i = 0; i < 90; i++) { w.Step(Dt); }
        ball.LinearVelocity = new Vector3F(6f, 0f, 0.8f);
        ball.AngularVelocity = new Vector3F(0f, spin, 0f);
        float maxUp = 0f;
        for (int i = 0; i < 120; i++)
        {
            w.Step(Dt);
            maxUp = Math.Max(maxUp, ball.LinearVelocity.Y);
        }

        Assert.InRange(maxUp, -1f, 0.03f);
        Assert.True(ball.Position.X > 2.0, "the ball did not slide: x = " + ball.Position.X);
    }

    [Theory]
    [InlineData(20f)]
    [InlineData(-20f)]
    public void ASphereSpinningAboutTheSlideDirectionIsNotKickedUp(float spin)
    {
        var w = NewWorld(tweak: o => o.SleepEnabled = false);
        var verts = new List<Vector3F>();
        var idx = new List<int>();
        const int N = 8;
        for (int j = 0; j <= N; j++) { for (int i = 0; i <= N; i++) { verts.Add(new Vector3F(i * 2f - 8f, 0f, j * 2f - 8f)); } }
        for (int j = 0; j < N; j++)
        {
            for (int i = 0; i < N; i++)
            {
                int a = j * (N + 1) + i, b = a + 1, c = a + N + 1, d = c + 1;
                idx.AddRange([a, c, b, b, c, d]);
            }
        }

        w.AddMesh(verts.ToArray(), idx.ToArray(), material: Ice);
        var ball = w.AddSphere(0.3f, new Vector3D(-6, 0.3, -5.3), material: Ice);
        for (int i = 0; i < 90; i++) { w.Step(Dt); }
        ball.LinearVelocity = new Vector3F(6f, 0f, 0.8f);
        ball.AngularVelocity = new Vector3F(0f, 0f, spin);
        float maxUp = 0f;
        for (int i = 0; i < 120; i++)
        {
            w.Step(Dt);
            maxUp = Math.Max(maxUp, ball.LinearVelocity.Y);
        }

        Assert.InRange(maxUp, -1f, 0.03f);
    }

    /// <summary>A flat height field has a seam along every cell edge and diagonal: a spinning sphere sliding over it must not be kicked either.</summary>
    [Theory]
    [InlineData(20f)]
    [InlineData(-20f)]
    public void ASpinningSphereSlidingOverHeightFieldSeamsIsNotKickedUp(float spin)
    {
        var w = NewWorld(tweak: o => o.SleepEnabled = false);
        const int N = 17;
        w.AddHeightField(new float[N * N], N, N, new Vector3F(1f, 1f, 1f), new Vector3D(-8, 0, -8), material: Ice);
        var ball = w.AddSphere(0.3f, new Vector3D(-6, 0.3, -5.3), material: Ice);
        for (int i = 0; i < 90; i++) { w.Step(Dt); }
        ball.LinearVelocity = new Vector3F(6f, 0f, 0.8f);
        ball.AngularVelocity = new Vector3F(0f, 0f, spin);
        float maxUp = 0f;
        for (int i = 0; i < 120; i++)
        {
            w.Step(Dt);
            maxUp = Math.Max(maxUp, ball.LinearVelocity.Y);
        }

        Assert.InRange(maxUp, -1f, 0.03f);
        Assert.True(ball.Position.X > 0.0, "the ball did not slide: x = " + ball.Position.X);
    }

    [Fact]
    public void AStaleBodyFailsClearly()
    {
        var w = NewWorld();
        var b = w.AddSphere(0.5f, new Vector3D(0, 1, 0));
        var s = b.GetShapes()[0];
        b.Destroy();
        Assert.False(b.IsValid);
        var e = Assert.Throws<StaleHandleException>(() => _ = b.Position);
        Assert.Contains("stale", e.Message);
        Assert.Throws<StaleHandleException>(() => b.ApplyImpulse(new Vector3F(1, 0, 0)));
        Assert.Throws<StaleHandleException>(() => s.Destroy());
        Assert.Throws<InvalidHandleException>(() => _ = default(Body).Position);
        var other = NewWorld(id: 99);
        Assert.Throws<ForeignHandleException>(() => other.GetBody(w.AddSphere(0.5f).RawHandle));
    }

    [Fact]
    public void HostileInputGivesTypedErrors()
    {
        var w = NewWorld();
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddSphere(float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddSphere(-1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddBox(new Vector3F(1, 0, 1)));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddSphere(1f, new Vector3D(double.NaN, 0, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddSphere(1f, new Vector3D(1e9, 0, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateBody(new BodyDef { LinearVelocity = new Vector3F(float.PositiveInfinity, 0, 0) }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateBody(new BodyDef { Rotation = new QuaternionF(float.NaN, 0, 0, 1) }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateBody(new BodyDef { LinearDamping = -1f }));
        Assert.ThrowsAny<ArgumentException>(() => w.AddConvexHull([new Vector3F(0, 0, 0), new Vector3F(1, 0, 0), new Vector3F(2, 0, 0), new Vector3F(3, 0, 0)]));
        Assert.ThrowsAny<ArgumentException>(() => w.AddConvexHull([new Vector3F(float.NaN, 0, 0), new Vector3F(1, 0, 0), new Vector3F(0, 1, 0), new Vector3F(0, 0, 1)]));
        Assert.ThrowsAny<ArgumentException>(() => w.AddMesh([new Vector3F(0, 0, 0)], [0, 0, 0]));
        Assert.ThrowsAny<ArgumentException>(() => w.AddCompound([]));
        Assert.Throws<RecursiveAI.OnePhysics.StaticOnlyShapeException>(() =>
        {
            var d = w.AddSphere(1f);
            w.CreateMesh(d, [new Vector3F(0, 0, 0), new Vector3F(1, 0, 0), new Vector3F(0, 1, 0)], [0, 1, 2]);
        });
        var b = w.AddSphere(0.5f);
        Assert.Throws<ArgumentOutOfRangeException>(() => b.LinearVelocity = new Vector3F(float.NaN, 0, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => b.ApplyForce(new Vector3F(0, float.NaN, 0)));
        Assert.Throws<BodyTypeException>(() => w.AddSphere(1f, type: BodyType.Static).LinearVelocity = new Vector3F(1, 0, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Step(0f));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Step(float.NaN));
        // Nothing above left a half-built body behind.
        int live = 0;
        for (int i = 0; i < w.Core.Bodies.HighWater; i++) { if (w.Core.Bodies.IsLive(i)) { live++; } }
        Assert.Equal(3, live); // the failed creations destroyed their bodies; d, b and the static one stay (+ d's mesh failure kept d)
    }

    [Fact]
    public void ABodyOutOfRangeFailsTheStepWithoutChangingTheWorld()
    {
        var w = NewWorld(tweak: o => o.Gravity = default);
        var b = w.AddSphere(0.5f, new Vector3D(2000000, 0, 0), type: BodyType.Kinematic);
        b.LinearVelocity = new Vector3F(1e7f, 0, 0);
        BodyOutOfBoundsException? caught = null;
        for (int i = 0; i < 100 && caught == null; i++)
        {
            try { w.Step(Dt); }
            catch (BodyOutOfBoundsException e) { caught = e; }
        }

        Assert.NotNull(caught);
        Assert.Equal(b, caught!.Body);
        ulong hash = w.ComputeHash();
        Assert.Throws<BodyOutOfBoundsException>(() => w.Step(Dt));
        Assert.Equal(hash, w.ComputeHash());
    }

    [Fact]
    public void ADynamicCompoundAndAHeightFieldWorkTogether()
    {
        var w = NewWorld();
        const int N = 17;
        var h = new float[N * N];
        for (int j = 0; j < N; j++) { for (int i = 0; i < N; i++) { h[j * N + i] = 0.1f * ((i + j) % 2); } }
        var hf = w.AddHeightField(h, N, N, new Vector3F(1f, 1f, 1f), new Vector3D(-8, 0, -8));
        Assert.Equal(BodyType.Static, hf.Type);
        var c = w.AddCompound([new CompoundPart(ShapeSpec3D.Box(new Vector3F(1f, 0.4f, 0.4f)), default), new CompoundPart(ShapeSpec3D.Box(new Vector3F(0.4f, 1f, 0.4f)), new Vector3F(0, 0.5f, 0))], new Vector3D(0, 2, 0));
        for (int i = 0; i < 300; i++) { w.Step(Dt); }
        Assert.InRange(c.Position.Y, -0.1, 1.5);
        Assert.False(c.IsAwake);
    }

    [Fact]
    public void ACollisionEnterCallbackFiresAfterTheStepWithAPointAndNormal()
    {
        var w = NewWorld(tweak: o => o.SleepEnabled = false);
        Floor(w);
        var ball = w.AddSphere(0.5f, new Vector3D(0, 0.6, 0));
        var hits = new List<Collision>();
        ball.OnCollisionEnter += hits.Add;
        for (int i = 0; i < 12; i++) { w.Step(Dt); }
        Assert.Single(hits);
        var c = hits[0];
        Assert.True(c.HasPoint);
        Assert.Equal(ball, c.Body);
        Assert.True(c.Normal.Y < -0.99f); // from the ball towards the floor
        Assert.InRange(c.Point.Y, -0.1, 0.1);
    }

    [Fact]
    public void SnapshotRestoreReplaysIdentically()
    {
        var a = BuildScene(1);
        for (int i = 0; i < 40; i++) { a.Step(Dt); }
        byte[] snap = a.Snapshot();
        var hashes = new List<ulong>();
        for (int i = 0; i < 60; i++) { a.Step(Dt); hashes.Add(a.ComputeHash()); }
        byte[] end = a.Snapshot();

        // Restore into the same world and into a fresh one.
        a.Restore(snap);
        for (int i = 0; i < 60; i++) { a.Step(Dt); Assert.Equal(hashes[i], a.ComputeHash()); }
        Assert.Equal(end, a.Snapshot());

        var b = NewWorld(1, 7);
        b.Restore(snap);
        Assert.Equal(snap, b.Snapshot());
        for (int i = 0; i < 60; i++) { b.Step(Dt); Assert.Equal(hashes[i], b.ComputeHash()); }
        Assert.Equal(end, b.Snapshot());
    }

    [Fact]
    public void ARejectedSnapshotLeavesTheWorldUnchanged()
    {
        var w = BuildScene(1);
        for (int i = 0; i < 20; i++) { w.Step(Dt); }
        byte[] snap = w.Snapshot();
        ulong hash = w.ComputeHash();
        var rnd = new Random(5);
        for (int k = 0; k < 200; k++)
        {
            var bad = (byte[])snap.Clone();
            int flips = 1 + k % 3;
            for (int f = 0; f < flips; f++) { bad[rnd.Next(bad.Length)] ^= (byte)(1 << rnd.Next(8)); }
            try { w.Restore(bad); hash = w.ComputeHash(); snap = w.Snapshot(); }
            catch (SnapshotFormatException) { Assert.Equal(hash, w.ComputeHash()); }
        }

        Assert.Throws<SnapshotFormatException>(() => w.Restore(snap[..(snap.Length / 2)]));
        Assert.Equal(hash, w.ComputeHash());
    }

    [Theory]
    [InlineData(2)]
    [InlineData(5)]
    public void ThreadCountDoesNotChangeTheWorld(int threads)
    {
        var a = BuildScene(1);
        var b = BuildScene(threads);
        for (int i = 0; i < 120; i++)
        {
            a.Step(Dt); b.Step(Dt);
            Assert.Equal(a.ComputeHash(), b.ComputeHash());
        }
    }

    [Fact]
    public void ShiftingTheOriginIsExact()
    {
        // The shift moves positions and stored broadphase boxes by the same offset and nothing else, so the shifted world steps through the same states as the unshifted one, translated.
        // Sleeping is off so the pile keeps moving for the whole run.
        var a = BuildScene(1, 7, o => o.SleepEnabled = false);
        var b = BuildScene(1, 7, o => o.SleepEnabled = false);
        // Positions handed in by a host are 31.32 fixed-point values; snap the scene's decimal ones to that grid so the shift is exact for every body.
        foreach (var w in new[] { a, b })
        {
            var pos = w.Core.Bodies.Position;
            for (int k = 0; k < pos.Length; k++) { pos[k] = Math.Round(pos[k] * 4294967296.0) / 4294967296.0; }
        }
        for (int i = 0; i < 30; i++) { a.Step(Dt); b.Step(Dt); }
        b.ShiftOrigin(new Vector3D(64.0, -8.0, 16.0));
        for (int i = 0; i < 600; i++)
        {
            a.Step(Dt); b.Step(Dt);
            var pa = a.Core.Bodies.Position; var pb = b.Core.Bodies.Position;
            var va = a.Core.Bodies.LinearVelocity; var vb = b.Core.Bodies.LinearVelocity;
            var ra = a.Core.Bodies.Rotation; var rb = b.Core.Bodies.Rotation;
            for (int k = 0; k < a.Core.Bodies.HighWater; k++)
            {
                Assert.True(pa[3 * k] - 64.0 == pb[3 * k] && pa[3 * k + 1] + 8.0 == pb[3 * k + 1] && pa[3 * k + 2] - 16.0 == pb[3 * k + 2], "position of body " + k + " at step " + i);
                Assert.True(va[3 * k] == vb[3 * k] && va[3 * k + 1] == vb[3 * k + 1] && va[3 * k + 2] == vb[3 * k + 2], "velocity of body " + k + " at step " + i);
                Assert.True(ra[4 * k] == rb[4 * k] && ra[4 * k + 1] == rb[4 * k + 1] && ra[4 * k + 2] == rb[4 * k + 2] && ra[4 * k + 3] == rb[4 * k + 3], "rotation of body " + k + " at step " + i);
            }
        }
    }

    [Fact]
    public void ADeadBodyDoesNotLeaveContactsBehind()
    {
        var w = NewWorld();
        Floor(w);
        var a = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0.5, 0));
        var b = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 1.5, 0));
        for (int i = 0; i < 10; i++) { w.Step(Dt); }
        a.Destroy();
        for (int i = 0; i < 30; i++) { w.Step(Dt); }
        Assert.True(b.Position.Y < 1.5);
        byte[] s = w.Snapshot();
        w.Restore(s);
        Assert.Equal(s, w.Snapshot());
    }
}
