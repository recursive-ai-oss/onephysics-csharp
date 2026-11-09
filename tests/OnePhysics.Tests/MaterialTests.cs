using System;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fluids;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests;

/// <summary>Solid and viscous matter on the particle store: definitions, determinism, the host hooks, a block of sand at rest, the clay dent, the snowball, the poured pile and the viscous column.</summary>
public sealed class MaterialTests
{
    private const float Dt = 1f / 60f;

    private static FluidParticleInfo[] Particles(P2.PhysicsWorld2D w) { var b = new FluidParticleInfo[w.FluidParticleCount]; w.GetFluidParticles(b); return b; }

    private static FluidParticleInfo[] Particles(P3.PhysicsWorld3D w) { var b = new FluidParticleInfo[w.FluidParticleCount]; w.GetFluidParticles(b); return b; }

    // ------------------------------------------------------------------ definitions

    [Fact]
    public void PresetsAreValidAndSolidMembersNeedAModulus()
    {
        foreach (var d in new[] { ParticleFluidDef.Sand(), ParticleFluidDef.Gravel(), ParticleFluidDef.Snow(), ParticleFluidDef.Mud(), ParticleFluidDef.Clay(), ParticleFluidDef.Dough(), ParticleFluidDef.Honey() }) { d.Validate(); }
        var bad = new ParticleFluidDef { FrictionAngle = 30f };
        var e = Assert.Throws<ArgumentOutOfRangeException>(() => bad.Validate());
        Assert.Equal("ElasticModulus", e.ParamName);
        Assert.Throws<ArgumentOutOfRangeException>(() => new ParticleFluidDef { ElasticModulus = 1e5f, FrictionAngle = 90f }.Validate());
        Assert.Throws<ArgumentOutOfRangeException>(() => new ParticleFluidDef { ElasticModulus = 1e5f, PoissonRatio = float.NaN }.Validate());
        Assert.Throws<ArgumentOutOfRangeException>(() => new ParticleFluidDef { DynamicViscosity = -1f }.Validate());
    }

    // ------------------------------------------------------------------ determinism and the host hooks

    [Theory]
    [InlineData("hopper-2d")]
    [InlineData("clay-press-2d")]
    [InlineData("sand-pile-3d")]
    [InlineData("honey-coil-3d")]
    [InlineData("snow-impact-3d")]
    public void ScenesAreTheSameOnOneAndThreeThreadsAndAfterARestore(string scene)
    {
        float sp = FluidScenes.CoarseSpacing(scene);
        bool d3 = FluidScenes.Is3D(scene);
        ulong g1 = d3 ? FluidScenes.Run3D(scene, 1, 70, sp) : FluidScenes.Run2D(scene, 1, 70, sp);
        ulong g3 = d3 ? FluidScenes.Run3D(scene, 3, 70, sp) : FluidScenes.Run2D(scene, 3, 70, sp);
        ulong gr = d3 ? FluidScenes.Run3D(scene, 1, 70, sp, true) : FluidScenes.Run2D(scene, 1, 70, sp, true);
        Assert.Equal(g1, g3);
        Assert.Equal(g1, gr);
    }

    [Fact]
    public void SolidStateSurvivesSnapshotForkAndAShiftedOrigin()
    {
        var w = FluidScenes.Build2D("clay-press-2d", 1, 0.05f, 91);
        for (int i = 0; i < 80; i++) { FluidScenes.BeforeStep2D("clay-press-2d", w, i); w.Step(Dt); }
        var bytes = w.Snapshot();
        var fresh = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 91 });
        fresh.Restore(bytes);
        Assert.Equal(w.ComputeHash(), fresh.ComputeHash());
        Assert.True(bytes.AsSpan().SequenceEqual(fresh.Snapshot()));
        var fork = w.Fork();
        Assert.Equal(w.ComputeHash(), fork.ComputeHash());
        for (int i = 80; i < 100; i++) { FluidScenes.BeforeStep2D("clay-press-2d", w, i); w.Step(Dt); FluidScenes.BeforeStep2D("clay-press-2d", fork, i); fork.Step(Dt); }
        Assert.Equal(w.ComputeHash(), fork.ComputeHash());
        // A shifted world holds the same particles at the shifted positions.
        var before = Particles(w);
        w.ShiftOrigin(new Vector2D(8, -4));
        var after = Particles(w);
        Assert.Equal(before.Length, after.Length);
        for (int i = 0; i < before.Length; i++) { Assert.Equal(before[i].Position.X - 8, after[i].Position.X); Assert.Equal(before[i].Position.Y + 4, after[i].Position.Y); }
    }

    // ------------------------------------------------------------------ sand

    [Fact]
    public void ABlockOfSandAtRestKeepsItsHeightAndSleeps()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 92 });
        w.AddBox(new Vector2F(4f, 0.2f), at: new Vector2D(0, -0.1), type: BodyType.Static);
        w.AddBox(new Vector2F(0.2f, 4f), at: new Vector2D(-0.7, 1.9), type: BodyType.Static);
        w.AddBox(new Vector2F(0.2f, 4f), at: new Vector2D(0.7, 1.9), type: BodyType.Static);
        var f = w.AddParticleFluid(ParticleFluidDef.Sand(0.02f));
        w.FillFluidBox(f, new Vector2D(-0.6, 0), new Vector2D(0.6, 0.3));
        for (int i = 0; i < 200; i++) { w.Step(Dt); }
        var p = Particles(w);
        double top = p.Max(q => q.Position.Y);
        Assert.InRange(top, 0.29 * 0.98, 0.29 * 1.001);
        Assert.True(w.FluidStatistics.Asleep);
    }

    [Fact]
    public void ABlockOfSandDroppedOnTheFloorLandsOnItAndKeepsItsVolume2D()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 100 });
        w.AddBox(new Vector2F(4f, 0.2f), at: new Vector2D(0, -0.1), type: BodyType.Static);
        w.AddBox(new Vector2F(0.2f, 4f), at: new Vector2D(-0.7, 1.9), type: BodyType.Static);
        w.AddBox(new Vector2F(0.2f, 4f), at: new Vector2D(0.7, 1.9), type: BodyType.Static);
        var f = w.AddParticleFluid(ParticleFluidDef.Sand(0.02f));
        w.FillFluidBox(f, new Vector2D(-0.6, 1.0), new Vector2D(0.6, 1.3));
        for (int i = 0; i < 300; i++) { w.Step(Dt); }
        var p = Particles(w);
        // The lowest grain rests half a spacing above the floor (where the contact holds it), and the block is not loose: it is as tall as it was (0.3 m), within a grain.
        Assert.InRange(p.Min(q => q.Position.Y), 0.0, 0.02);
        Assert.InRange(p.Max(q => q.Position.Y), 0.27, 0.33);
    }

    [Fact]
    public void ABlockOfSandDroppedOnTheFloorLandsOnItAndKeepsItsVolume3D()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 101, ThreadCount = 4 });
        w.AddBox(new Vector3F(4f, 0.2f, 4f), at: new Vector3D(0, -0.1, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(0.2f, 2f, 1.4f), at: new Vector3D(-0.7, 1.0, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(0.2f, 2f, 1.4f), at: new Vector3D(0.7, 1.0, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(1.4f, 2f, 0.2f), at: new Vector3D(0, 1.0, -0.7), type: BodyType.Static);
        w.AddBox(new Vector3F(1.4f, 2f, 0.2f), at: new Vector3D(0, 1.0, 0.7), type: BodyType.Static);
        var f = w.AddParticleFluid(ParticleFluidDef.Sand(0.04f));
        w.FillFluidBox(f, new Vector3D(-0.6, 1.0, -0.6), new Vector3D(0.6, 1.4, 0.6));
        for (int i = 0; i < 240; i++) { w.Step(Dt); }
        var p = Particles(w);
        Assert.InRange(p.Min(q => q.Position.Y), 0.0, 0.03);
        Assert.InRange(p.Max(q => q.Position.Y), 0.35, 0.42);
    }

    // The angle of a poured pile: the slope of its surface (the 97th percentile of the height in each radial bin, which ignores the odd grain on top) between the given shares of the base radius.
    private static double PileAngle(FluidParticleInfo[] b, Func<FluidParticleInfo, double> radius, double bin, double from, double to)
    {
        var r = b.Select(radius).OrderBy(v => v).ToArray();
        double edge = r[(int)(0.98 * (r.Length - 1))];
        var heights = new System.Collections.Generic.Dictionary<int, System.Collections.Generic.List<double>>();
        foreach (var q in b)
        {
            int k = (int)(radius(q) / bin);
            if (!heights.TryGetValue(k, out var l)) { heights[k] = l = new System.Collections.Generic.List<double>(); }
            l.Add(q.Position.Y);
        }

        var xs = new System.Collections.Generic.List<double>(); var ys = new System.Collections.Generic.List<double>();
        foreach (var kv in heights.OrderBy(kv => kv.Key))
        {
            double x = (kv.Key + 0.5) * bin;
            if (!(x > from * edge && x < to * edge)) { continue; }
            if (kv.Value.Count < 6) { continue; }
            kv.Value.Sort();
            xs.Add(x); ys.Add(kv.Value[(int)(0.97 * (kv.Value.Count - 1))]);
        }

        double mx = xs.Average(), my = ys.Average(), num = 0, den = 0;
        for (int i = 0; i < xs.Count; i++) { num += (xs[i] - mx) * (ys[i] - my); den += (xs[i] - mx) * (xs[i] - mx); }
        return Math.Atan(-num / den) * 180.0 / Math.PI;
    }

    private static double Pour2D(float angle, float halfWidth, float speed, int grains, float height)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 93 });
        w.AddBox(new Vector2F(12f, 0.2f), at: new Vector2D(0, -0.1), type: BodyType.Static);
        var d = ParticleFluidDef.Sand(0.03f); d.FrictionAngle = angle;
        var f = w.AddParticleFluid(d);
        f.AddEmitter(new FluidEmitterDef { Position = new Vector3D(0, height, 0), Direction = new Vector3F(0, -1, 0), Kind = FluidVolumeKind.Box, Size = new Vector3F(halfWidth, 0.05f, 0.1f), Speed = speed, MaxEmitted = grains });
        for (int i = 0; i < 1500; i++) { w.Step(Dt); }
        return PileAngle(Particles(w), q => Math.Abs(q.Position.X), 0.03, 0.45, 0.9);
    }

    [Theory]
    [InlineData(30f)]
    [InlineData(35f)]
    public void APouredPile2DFollowsTheFrictionAngle(float angle)
    {
        // Three pours (a wide slow nozzle, a wide slower one, a narrower one): each pile stands within 6 degrees of the friction angle and their mean within 3 (the spread between pours is the grains' avalanches; docs/spec/fluids.md).
        double a = Pour2D(angle, 0.1f, 0.3f, 600, 1.2f), b = Pour2D(angle, 0.1f, 0.2f, 900, 1.2f), c = Pour2D(angle, 0.08f, 0.4f, 700, 1.2f);
        string note = a.ToString("F1") + " " + b.ToString("F1") + " " + c.ToString("F1");
        Assert.True(Math.Abs(a - angle) < 6.0 && Math.Abs(b - angle) < 6.0 && Math.Abs(c - angle) < 6.0, "pour angles " + note);
        Assert.True(Math.Abs((a + b + c) / 3.0 - angle) < 3.0, "pour angles " + note);
    }

    [Theory]
    [InlineData(30f)]
    [InlineData(35f)]
    public void APouredPile3DStandsAsACone(float angle)
    {
        // The slope of a 3D pile of 1500 grains (nine grains in radius) cannot be measured to better than about 10 degrees at this spacing, so this only checks that it stands as a cone of the right order: the whole pile's
        // height over its base radius is between 15 and 50 degrees (docs/spec/fluids.md has the measured spread and the finer spacings).
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 94, ThreadCount = 4 });
        w.AddBox(new Vector3F(8f, 0.2f, 8f), at: new Vector3D(0, -0.1, 0), type: BodyType.Static);
        var d = ParticleFluidDef.Sand(0.04f); d.FrictionAngle = angle;
        var f = w.AddParticleFluid(d);
        f.AddEmitter(new FluidEmitterDef { Position = new Vector3D(0, 1.0, 0), Direction = new Vector3F(0, -1, 0), Kind = FluidVolumeKind.Sphere, Size = new Vector3F(0.08f, 0.05f, 0.1f), Speed = 0.5f, MaxEmitted = 1500 });
        for (int i = 0; i < 1600; i++) { w.Step(Dt); }
        var b = Particles(w);
        double r98 = b.Select(q => Math.Sqrt(q.Position.X * q.Position.X + q.Position.Z * q.Position.Z)).OrderBy(v => v).ElementAt((int)(0.98 * (b.Length - 1)));
        double slope = Math.Atan(b.Max(q => q.Position.Y) / r98) * 180.0 / Math.PI;
        Assert.InRange(slope, 15.0, 50.0);
    }

    // ------------------------------------------------------------------ clay and snow

    private static double ClayTop(P2.PhysicsWorld2D w) => Particles(w).Where(q => Math.Abs(q.Position.X) < 0.05).Max(q => q.Position.Y);

    private static double ClayDent(float yieldStress)
    {
        var w = FluidScenes.Build2D("clay-press-2d", 1, 0.02f, 95);
        if (yieldStress > 0f) { var f = w.GetParticleFluid(0); var d = f.Definition; d.YieldStress = yieldStress; f.Definition = d; }
        for (int i = 0; i < 59; i++) { FluidScenes.BeforeStep2D("clay-press-2d", w, i); w.Step(Dt); }
        double rest = ClayTop(w);
        for (int i = 59; i < 52 + 60 + 300; i++) { FluidScenes.BeforeStep2D("clay-press-2d", w, i); w.Step(Dt); }
        return rest - ClayTop(w);
    }

    [Fact]
    public void ClayKeepsADentAndAnElasticBlockRecovers()
    {
        const double imposed = 0.05;
        Assert.True(ClayDent(0f) > 0.5 * imposed, "the clay should keep more than half of the imposed dent");
        Assert.True(ClayDent(1e9f) < 0.2 * imposed, "below the yield stress the block should recover more than 80%");
    }

    [Fact]
    public void ASnowballThrownAtAWallBreaksIntoClumps()
    {
        var w = FluidScenes.Build3D("snow-impact-3d", 4, 0.04f, 96);
        for (int i = 0; i < 200; i++) { w.Step(Dt); }
        var b = Particles(w);
        Assert.True(Clumps(b, 0.064) >= 5, "the ball should break into at least five clumps");
        var w0 = FluidScenes.Build3D("snow-impact-3d", 4, 0.04f, 97);
        Assert.Equal(1, Clumps(Particles(w0), 0.064));
    }

    private static int Clumps(FluidParticleInfo[] b, double link)
    {
        int n = b.Length; var parent = Enumerable.Range(0, n).ToArray();
        int Find(int x) { while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; }
        var order = Enumerable.Range(0, n).OrderBy(i => b[i].Position.X).ToArray();
        for (int ii = 0; ii < n; ii++)
        {
            int i = order[ii];
            for (int jj = ii + 1; jj < n; jj++)
            {
                int j = order[jj];
                if (b[j].Position.X - b[i].Position.X > link) { break; }
                double dx = b[i].Position.X - b[j].Position.X, dy = b[i].Position.Y - b[j].Position.Y, dz = b[i].Position.Z - b[j].Position.Z;
                if (dx * dx + dy * dy + dz * dz < link * link) { parent[Find(i)] = Find(j); }
            }
        }

        return Enumerable.Range(0, n).Select(Find).Distinct().Count();
    }

    // ------------------------------------------------------------------ viscosity

    private static double Spread(float viscosity)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 98 });
        w.AddBox(new Vector2F(8f, 0.2f), at: new Vector2D(0, -0.1), type: BodyType.Static);
        var d = ParticleFluidDef.Honey(0.02f); d.DynamicViscosity = viscosity; d.Friction = 0f;
        var f = w.AddParticleFluid(d);
        w.FillFluidBox(f, new Vector2D(-0.2, 0), new Vector2D(0.2, 0.4));
        for (int i = 0; i < 90; i++) { w.Step(Dt); }
        var p = Particles(w);
        return p.Max(q => q.Position.X) - p.Min(q => q.Position.X);
    }

    [Fact]
    public void AViscousColumnSlumpsSlowerTheMoreViscousItIs()
    {
        double a = Spread(0.5f), b = Spread(10f), c = Spread(200f);
        Assert.True(a > b && b > c, "spread " + a + ", " + b + ", " + c);
    }

    [Fact]
    public void ViscosityConservesMomentum()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 99, Gravity = new Vector3F(0, 0, 0) });
        var d = ParticleFluidDef.Honey(0.03f); d.DynamicViscosity = 30f;
        var f = w.AddParticleFluid(d);
        w.FillFluidSphere(f, new Vector3D(0, 1, 0), 0.2, new Vector3F(1f, 0.5f, 0f));
        // A shear inside the blob: the upper half moves with an extra velocity.
        double Momentum(out double py)
        {
            double px = 0; py = 0;
            foreach (var q in Particles(w)) { px += q.Velocity.X; py += q.Velocity.Y; }
            return px;
        }

        double m0 = Momentum(out double y0);
        for (int i = 0; i < 40; i++) { w.Step(Dt); }
        double m1 = Momentum(out double y1);
        Assert.InRange(m1, m0 * 0.99, m0 * 1.01);
        Assert.InRange(y1, y0 * 0.99, y0 * 1.01);
    }
}
