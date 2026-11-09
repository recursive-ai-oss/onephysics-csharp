using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

// Surface contacts between soft bodies, tearing and plasticity of 2D soft bodies (docs/spec/softbody.md).
public class SoftFracture2DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D NewWorld(bool gravity = true, int threads = 1)
        => new(new WorldOptions2D { ThreadCount = threads, WorldId = 43, Gravity = gravity ? new Vector2F(0, -9.81f) : new Vector2F(0, 0) });

    private static void Run(PhysicsWorld2D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    private static SoftBody Strip(PhysicsWorld2D w, float tearStrain, float tearForce = 0f, float edgeCompliance = 1e-4f)
    {
        var d = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(2f, 0.2f), 10, 1), default);
        d.EdgeCompliance = edgeCompliance; d.AreaCompliance = 1e-3f; d.EdgeDamping = 0.01f; d.TearStrain = tearStrain; d.TearForce = tearForce; d.Mass = 1f; d.LinearDamping = 0f;
        return w.AddSoftBody(d);
    }

    // A jelly box dropped half a cell off the centre of another one on the floor: the particles of the upper box land between the boundary particles of the lower one.
    // With surface contact they rest on its top edge; without it they pass through the gaps between the discs.
    private static double SinkDepth(bool surface)
    {
        var w = NewWorld();
        w.AddBox(new Vector2F(20f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var lower = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(1.2f, 0.6f), 4, 2), new Vector2D(0, 0.3));
        lower.Mass = 6f; lower.SurfaceCollision = surface;
        var a = w.AddSoftBody(lower);
        var upper = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(0.9f, 0.6f), 3, 2), new Vector2D(0.15, 1.2));
        upper.Mass = 4f; upper.SurfaceCollision = surface;
        var b = w.AddSoftBody(upper);
        Run(w, 240);
        double topA = double.MinValue, bottomB = double.MaxValue;
        for (int i = 0; i < a.ParticleCount; i++) { topA = Math.Max(topA, a.GetPosition(i).Y); }
        for (int i = 0; i < b.ParticleCount; i++) { bottomB = Math.Min(bottomB, b.GetPosition(i).Y); }
        return topA - bottomB;   // how far the lowest upper particle is below the highest lower one
    }

    [Fact]
    public void TheSoftPileSleeps()
    {
        var w = SoftBodies2DScene.Build("soft-pile-2d", 1);
        for (int i = 0; i < 600; i++) { w.Step(Dt); }
        int asleep = 0;
        for (int s = 0; s < w.SoftBodyCount; s++) { if (w.GetSoftBody(s).IsAsleep) { asleep++; } }
        Assert.True(asleep >= 0.9 * w.SoftBodyCount, asleep + " of " + w.SoftBodyCount + " soft bodies asleep");
    }

    [Fact]
    public void SoftBoxesPressedTogetherDoNotInterpenetrate()
    {
        double with = SinkDepth(true), without = SinkDepth(false);
        // The lowest particle of the upper box stays above the lower box's top particles (the boundary is a line of discs plus the edge between them).
        Assert.True(with < 0.02, "with surface contact the upper box sank " + with);
        Assert.True(without > with + 0.01, "without it " + without + " against " + with);
    }

    [Fact]
    public void AStrainTearsAnEdgeWhenItPassesTheThreshold()
    {
        var w = NewWorld(false);
        var b = Strip(w, 0.2f);
        int events = 0;
        w.SoftBodyTorn += e => { Assert.Equal(b, e.Body); events++; };
        double firstStrain = 0;
        double worstBefore = 0;
        for (int step = 1; step <= 4000 && b.BrokenEdgeCount == 0; step++)
        {
            double sep = 0.0002 * step;
            for (int j = 0; j <= 1; j++)
            {
                b.SetTarget(j * 11, new Vector2D(-1 - sep / 2, -0.1 + 0.2 * j), 0);
                b.SetTarget(j * 11 + 10, new Vector2D(1 + sep / 2, -0.1 + 0.2 * j), 0);
            }

            w.Step(Dt);
            double worst = MaxEdgeStrain(b);
            if (b.BrokenEdgeCount == 0) { worstBefore = worst; } else { firstStrain = worstBefore; }
        }

        Assert.True(b.BrokenEdgeCount >= 1);
        Assert.Equal(w.SoftTearEvents.Length, events);
        // Every step before the tear stayed under the threshold, and the step that tore had an edge past it by no more than one step of loading.
        Assert.True(worstBefore <= 0.2 + 1e-4, "strain before the tear " + worstBefore);
        Assert.True(events >= 1);
        // The step before the tear was within a few steps of loading of the threshold (0.0002 m of end separation per step on a 2 m strip).
        Assert.True(firstStrain >= 0.2 - 0.003, "the step before the tear had a worst strain of only " + firstStrain);
    }

    private static double MaxEdgeStrain(SoftBody b)
    {
        double worst = 0;
        var ea = b.RawEdgeA; var eb = b.RawEdgeB; var rest = b.RawCurrentEdgeRestLengths;
        for (int e = 0; e < ea.Length; e++)
        {
            var p = b.GetPosition(ea[e]); var q = b.GetPosition(eb[e]);
            double len = Math.Sqrt((p.X - q.X) * (p.X - q.X) + (p.Y - q.Y) * (p.Y - q.Y));
            worst = Math.Max(worst, (len - rest[e]) / rest[e]);
        }

        return worst;
    }

    [Fact]
    public void ATornStripSplitsIntoPartsAndKeepsMomentum()
    {
        var w = NewWorld(false);
        var b = Strip(w, 0.03f);
        // Throw the two halves apart and give the whole body a drift: the total momentum stays the drift's through the tears.
        for (int i = 0; i < b.ParticleCount; i++)
        {
            double x = b.GetPosition(i).X;
            b.ApplyImpulse(i, new Vector2F((x < 0 ? -1f : 1f) * 0.5f + 0.2f, 0.1f));
        }

        var before = b.Momentum;
        int n0 = b.ParticleCount;
        Run(w, 240);
        var after = b.Momentum;
        Assert.True(b.ParticleCount > n0, "the strip should have split vertices");
        Assert.True(Math.Abs(after.X - before.X) < 2e-3 * Math.Max(1.0, Math.Abs(before.X)) && Math.Abs(after.Y - before.Y) < 2e-3, "momentum " + before + " -> " + after);
    }

    [Fact]
    public void AForceThresholdTearsAnInextensibleEdge()
    {
        var w = NewWorld(false);
        var b = Strip(w, 0f, tearForce: 5f, edgeCompliance: 0f);
        for (int step = 1; step <= 600 && b.BrokenEdgeCount == 0; step++)
        {
            double sep = 0.001 * step;
            for (int j = 0; j <= 1; j++)
            {
                b.SetTarget(j * 11, new Vector2D(-1 - sep / 2, -0.1 + 0.2 * j), 0);
                b.SetTarget(j * 11 + 10, new Vector2D(1 + sep / 2, -0.1 + 0.2 * j), 0);
            }

            w.Step(Dt);
        }

        Assert.True(b.BrokenEdgeCount >= 1);
    }

    // A bar bent into an arc by driven particles, then released: with plasticity it keeps most of the bend, without it springs back flat.
    private static (double Bent, double After) BendBar(float yield)
    {
        var w = NewWorld(false);
        const int nx = 20, ny = 2;
        const float L = 2f;
        var d = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(L, 0.2f), nx, ny), default);
        d.EdgeCompliance = 1e-6f; d.AreaCompliance = 1e-6f; d.EdgeDamping = 0.01f; d.Mass = 1f;
        d.PlasticYield = yield; d.PlasticCreep = 0.3f; d.PlasticMax = 0.5f;
        var b = w.AddSoftBody(d);
        var rest = new Vector2D[b.ParticleCount];
        for (int i = 0; i < rest.Length; i++) { rest[i] = b.GetPosition(i); }
        for (int step = 1; step <= 120; step++)
        {
            double k = Math.Min(1.0, step / 60.0);
            for (int i = 0; i < rest.Length; i++)
            {
                double s = rest[i].X + L / 2, y0 = rest[i].Y, r = 1.0 / Math.Max(k, 1e-9), phi = s / r;
                b.SetTarget(i, k < 1e-9 ? rest[i] : new Vector2D((r - y0) * Math.Sin(phi) - L / 2, r - (r - y0) * Math.Cos(phi)), 0);
            }

            w.Step(Dt);
        }

        int tip = (ny / 2) * (nx + 1) + nx;
        double bent = b.GetPosition(tip).Y;
        for (int i = 0; i < rest.Length; i++) { b.ClearTarget(i); }
        b.SetPinned(0, true); b.SetPinned((ny / 2) * (nx + 1), true); b.SetPinned(ny * (nx + 1), true);
        Run(w, 600);
        return (bent, b.GetPosition(tip).Y);
    }

    [Fact]
    public void ABentPlasticBarKeepsItsShape()
    {
        var plastic = BendBar(0.03f);
        var elastic = BendBar(0f);
        Assert.True(plastic.After > 0.85 * plastic.Bent, "plastic bar tip " + plastic.Bent + " -> " + plastic.After);
        Assert.True(Math.Abs(elastic.After) < 0.3, "elastic bar tip " + elastic.After);
    }

    [Fact]
    public void TearingAndPlasticityAreDeterministicAcrossThreadsAndRestores()
    {
        ulong Run1(int threads, bool restore)
        {
            var w = NewWorld(true, threads);
            w.AddBox(new Vector2F(30f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
            for (int i = 0; i < 3; i++)
            {
                var d = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(1f, 0.5f), 6, 3), new Vector2D(i * 1.5 - 1.5, 1.0 + i * 0.7));
                d.Mass = 3f; d.EdgeCompliance = 2e-4f; d.TearStrain = 0.12f; d.PlasticYield = 0.04f; d.LinearVelocity = new Vector2F(i - 1, 0);
                w.AddSoftBody(d);
            }

            ulong h = 14695981039346656037UL;
            for (int step = 0; step < 150; step++)
            {
                if (restore && step == 70)
                {
                    var snap = w.Snapshot();
                    w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = threads, WorldId = 43, Gravity = new Vector2F(0, -9.81f) });
                    w.Restore(snap);
                }

                w.Step(Dt);
                h = (h ^ w.ComputeHash()) * 1099511628211UL;
            }

            return h;
        }

        ulong a = Run1(1, false);
        Assert.Equal(a, Run1(8, false));
        Assert.Equal(a, Run1(1, true));
        Assert.Equal(a, Run1(8, true));
    }

    [Fact]
    public void ASnapshotAfterATearRestoresTheTornBody()
    {
        var w = NewWorld(false);
        var b = Strip(w, 0.03f);
        for (int i = 0; i < b.ParticleCount; i++) { b.ApplyImpulse(i, new Vector2F((b.GetPosition(i).X < 0 ? -1f : 1f) * 0.5f, 0f)); }
        Run(w, 120);
        Assert.True(b.ParticleCount > 22);
        var snap = w.Snapshot();
        var w2 = new PhysicsWorld2D(new WorldOptions2D { WorldId = 43, Gravity = new Vector2F(0, 0) });
        w2.Restore(snap);
        Assert.Equal(w.ComputeHash(), w2.ComputeHash());
        Assert.Equal(snap, w2.Snapshot());
        Run(w, 60); Run(w2, 60);
        Assert.Equal(w.ComputeHash(), w2.ComputeHash());
    }
}
