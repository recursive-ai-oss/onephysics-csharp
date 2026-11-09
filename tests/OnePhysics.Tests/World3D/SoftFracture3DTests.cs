using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

// Surface contacts, tearing and plasticity of 3D soft bodies (docs/spec/softbody.md).
public class SoftFracture3DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld3D NewWorld(bool gravity = true, int threads = 1)
        => new(new WorldOptions3D { ThreadCount = threads, WorldId = 79, Gravity = gravity ? new Vector3F(0, -9.81f, 0) : default });

    private static void Run(PhysicsWorld3D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    // A jelly box dropped half a cell off the centre of another one on the floor: the particles of the upper box land between the boundary particles of the lower one.
    private static double SinkDepth(bool surface)
    {
        var w = NewWorld();
        w.AddBox(new Vector3F(20f, 1f, 20f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var lower = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector3F(1f, 0.5f, 1f), 4, 2, 4), new Vector3D(0, 0.25, 0));
        lower.Mass = 6f; lower.SurfaceCollision = surface;
        var a = w.AddSoftBody(lower);
        var upper = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector3F(0.75f, 0.5f, 0.75f), 3, 2, 3), new Vector3D(0.125, 1.0, 0.125));
        upper.Mass = 4f; upper.SurfaceCollision = surface;
        var b = w.AddSoftBody(upper);
        Run(w, 240);
        double topA = double.MinValue, bottomB = double.MaxValue;
        for (int i = 0; i < a.ParticleCount; i++) { topA = Math.Max(topA, a.GetPosition(i).Y); }
        for (int i = 0; i < b.ParticleCount; i++) { bottomB = Math.Min(bottomB, b.GetPosition(i).Y); }
        return topA - bottomB;
    }

    [Fact]
    public void SoftBoxesPressedTogetherDoNotInterpenetrate()
    {
        double with = SinkDepth(true), without = SinkDepth(false);
        Assert.True(with < 0.02, "with surface contact the upper box sank " + with);
        Assert.True(without > with + 0.01, "without it " + without + " against " + with);
    }

    [Fact]
    public void AClothFoldingOnItselfNeverIntersectsItself()
    {
        var m = SoftFracture3DScene.Measure("cloth-fold-3d");
        Assert.Equal(0, m.NaN);
        Assert.Equal(0, m.MaxIntersections);
        Assert.True(m.MaxSpeed <= 30.0, SoftFracture3DScene.Format(m));
        // The same fall without self-collision does cross itself, so the count above means something.
        var open = SoftFracture3DScene.Measure("cloth-fold-3d", 300, 1, false);
        Assert.True(open.MaxIntersections > 100, SoftFracture3DScene.Format(open));
    }

    private static double MaxEdgeStrain(SoftBody b)
    {
        double worst = 0;
        var ea = b.RawEdgeA; var eb = b.RawEdgeB; var rest = b.RawEdgeRestLengths;
        for (int e = 0; e < ea.Length; e++)
        {
            var p = b.GetPosition(ea[e]); var q = b.GetPosition(eb[e]);
            double len = Math.Sqrt((p.X - q.X) * (p.X - q.X) + (p.Y - q.Y) * (p.Y - q.Y) + (p.Z - q.Z) * (p.Z - q.Z));
            worst = Math.Max(worst, (len - rest[e]) / rest[e]);
        }

        return worst;
    }

    // A cloth strip held at both ends (pinned particles moved by hand) tears an edge when the strain passes the threshold.
    private static SoftBody Strip(PhysicsWorld3D w, float tearStrain, float tearForce = 0f, float edgeCompliance = 1e-4f)
    {
        var d = SoftBodyDef.Cloth(SoftBodyMesh.Cloth(2f, 0.2f, 10, 1), default);
        d.EdgeCompliance = edgeCompliance; d.EdgeDamping = 0.01f; d.TearStrain = tearStrain; d.TearForce = tearForce; d.Mass = 1f; d.LinearDamping = 0f; d.SelfCollision = false;
        var pins = new List<int>();
        for (int j = 0; j <= 1; j++) { pins.Add(j * 11); pins.Add(j * 11 + 10); }
        d.PinnedParticles = pins;
        return w.AddSoftBody(d);
    }

    private static void Pull(SoftBody b, double sep)
    {
        for (int j = 0; j <= 1; j++)
        {
            b.SetPosition(j * 11, new Vector3D(-1 - sep / 2, 0, -0.1 + 0.2 * j));
            b.SetPosition(j * 11 + 10, new Vector3D(1 + sep / 2, 0, -0.1 + 0.2 * j));
        }
    }

    [Fact]
    public void AStrainTearsAnEdgeWhenItPassesTheThreshold()
    {
        var w = NewWorld(false);
        var b = Strip(w, 0.2f);
        int events = 0;
        w.SoftBodyTorn += e => { Assert.Equal(b, e.Body); events++; };
        double worstBefore = 0;
        for (int step = 1; step <= 4000 && b.BrokenEdgeCount == 0; step++)
        {
            Pull(b, 0.0002 * step);
            w.Step(Dt);
            if (b.BrokenEdgeCount == 0) { worstBefore = MaxEdgeStrain(b); }
        }

        Assert.True(b.BrokenEdgeCount >= 1);
        Assert.Equal(w.SoftTearEvents.Length, events);
        Assert.True(events >= 1);
        Assert.True(worstBefore <= 0.2 + 1e-4, "strain before the tear " + worstBefore);
        Assert.True(worstBefore >= 0.2 - 0.003, "the step before the tear had a worst strain of only " + worstBefore);
    }

    [Fact]
    public void AForceThresholdTearsAnInextensibleEdge()
    {
        var w = NewWorld(false);
        var b = Strip(w, 0f, tearForce: 5f, edgeCompliance: 0f);
        for (int step = 1; step <= 600 && b.BrokenEdgeCount == 0; step++)
        {
            Pull(b, 0.001 * step);
            w.Step(Dt);
        }

        Assert.True(b.BrokenEdgeCount >= 1);
    }

    [Fact]
    public void ATornClothSplitsIntoPartsAndKeepsMomentum()
    {
        var w = NewWorld(false);
        var d = SoftBodyDef.Cloth(SoftBodyMesh.Cloth(2f, 0.2f, 10, 1), default);
        d.EdgeCompliance = 1e-4f; d.EdgeDamping = 0.01f; d.TearStrain = 0.03f; d.Mass = 1f; d.LinearDamping = 0f; d.SelfCollision = false;
        var b = w.AddSoftBody(d);
        for (int i = 0; i < b.ParticleCount; i++) { b.ApplyImpulse(i, new Vector3F((b.GetPosition(i).X < 0 ? -1f : 1f) * 0.5f + 0.2f, 0.1f, 0f)); }
        var before = b.Momentum;
        int n0 = b.ParticleCount;
        Run(w, 240);
        var after = b.Momentum;
        Assert.True(b.ParticleCount > n0, "the strip should have split vertices");
        Assert.True(Math.Abs(after.X - before.X) < 2e-3 * Math.Max(1.0, Math.Abs(before.X)) && Math.Abs(after.Y - before.Y) < 2e-3 && Math.Abs(after.Z - before.Z) < 2e-3, "momentum " + before + " -> " + after);
    }

    // A bar pinned at one end sags under gravity; gravity is then switched off. Elastic, it comes back; with plasticity it keeps most of the sag.
    private static (double Loaded, double After) SagBar(float yield)
    {
        var w = NewWorld();
        var def = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector3F(1.6f, 0.2f, 0.2f), 16, 2, 2), new Vector3D(0, 1, 0));
        def.Mass = 0.5f; def.EdgeCompliance = 5e-4f; def.VolumeCompliance = 1e-5f; def.EdgeDamping = 0.05f; def.LinearDamping = 3f; def.SurfaceCollision = false; def.AllowSleep = false;
        def.PlasticYield = yield; def.PlasticCreep = 0.2f; def.PlasticMax = 0.8f;
        var pins = new List<int>();
        for (int k = 0; k <= 2; k++) { for (int j = 0; j <= 2; j++) { pins.Add((k * 3 + j) * 17); } }
        def.PinnedParticles = pins;
        var b = w.AddSoftBody(def);
        int tip = (1 * 3 + 1) * 17 + 16;
        Run(w, 240);
        double loaded = b.GetPosition(tip).Y;
        w.Options.Gravity = default;
        Run(w, 600);
        return (loaded, b.GetPosition(tip).Y);
    }

    [Fact]
    public void ASaggedPlasticBarKeepsItsShape()
    {
        var plastic = SagBar(0.03f);
        var elastic = SagBar(0f);
        Assert.True(elastic.After > 0.9, "elastic bar tip " + elastic.Loaded + " -> " + elastic.After);
        Assert.True(plastic.After < 0.2, "plastic bar tip " + plastic.Loaded + " -> " + plastic.After);
    }

    [Theory]
    [InlineData("cloth-fold-3d")]
    [InlineData("tear-3d")]
    public void TheSceneIsTheSameOnAnyThreadCountAndAfterARestore(string name)
    {
        ulong a = SoftFracture3DScene.Run(name, 1, false);
        Assert.Equal(a, SoftFracture3DScene.Run(name, 8, false));
        Assert.Equal(a, SoftFracture3DScene.Run(name, 1, true));
        Assert.Equal(a, SoftFracture3DScene.Run(name, 8, true));
    }

    [Fact]
    public void ASnapshotAfterATearRestoresTheTornBodies()
    {
        var w = SoftFracture3DScene.Build("tear-3d", 1);
        for (int step = 0; step < 150; step++) { SoftFracture3DScene.Drive(w, "tear-3d", step); w.Step(Dt); }
        var cloth = w.GetSoftBody(0);
        Assert.True(cloth.ParticleCount > 221, "the cloth should have split vertices");
        var snap = w.Snapshot();
        var w2 = new PhysicsWorld3D(new WorldOptions3D { WorldId = 77 });
        w2.Restore(snap);
        Assert.Equal(w.ComputeHash(), w2.ComputeHash());
        Assert.Equal(snap, w2.Snapshot());
        for (int step = 150; step < 210; step++) { SoftFracture3DScene.Drive(w, "tear-3d", step); w.Step(Dt); w2.Step(Dt); Assert.Equal(w.ComputeHash(), w2.ComputeHash()); }
    }

    [Fact]
    public void ASkinnedBodyCannotTear()
    {
        var def = SoftBodyDef.Cloth(SoftBodyMesh.Cloth(1f, 1f, 4, 4), default);
        int n = def.Mesh!.VertexCount;
        var joints = new int[4 * n]; var weights = new float[4 * n]; var max = new float[n];
        for (int i = 0; i < n; i++) { weights[4 * i] = 1f; max[i] = 0.1f; }
        def.Skin = SoftSkin.Create(def.Mesh, new[] { -1 }, new[] { new Vector3F(0, 0, 0) }, joints, weights, max);
        def.TearStrain = 0.1f;
        Assert.Throws<SoftBodyException>(() => def.Validate());
    }
}
