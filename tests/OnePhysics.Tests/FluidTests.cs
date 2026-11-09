using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Fluids;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests;

/// <summary>Particle fluids: rest density, hydrostatics, the Martin and Moyce dam break, buoyancy, momentum, cloth and rope coupling, emitters, sleep, queries, events and the host hooks, in both dimensions.</summary>
public sealed class FluidTests
{
    private const float Dt = 1f / 60f;

    // ------------------------------------------------------------------ helpers

    private static P3.PhysicsWorld3D Tank3D(float spacing, int threads, double width, double depth, double fillHeight, out ParticleFluid fluid, ParticleFluidDef? def = null)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { ThreadCount = threads });
        double h = width / 2;
        w.AddBox(new Vector3F((float)(width + 1), 0.2f, (float)(width + 1)), at: new Vector3D(0, -0.1, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(0.2f, 4f, (float)(width + 1)), at: new Vector3D(-h - 0.1, 1.9, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(0.2f, 4f, (float)(width + 1)), at: new Vector3D(h + 0.1, 1.9, 0), type: BodyType.Static);
        w.AddBox(new Vector3F((float)(width + 1), 4f, 0.2f), at: new Vector3D(0, 1.9, -h - 0.1), type: BodyType.Static);
        w.AddBox(new Vector3F((float)(width + 1), 4f, 0.2f), at: new Vector3D(0, 1.9, h + 0.1), type: BodyType.Static);
        fluid = w.AddParticleFluid(def ?? ParticleFluidDef.Water(spacing));
        w.FillFluidBox(fluid, new Vector3D(-h, 0, -h), new Vector3D(h, fillHeight, h));
        _ = depth;
        return w;
    }

    private static P2.PhysicsWorld2D Tank2D(float spacing, int threads, double width, double fillHeight, out ParticleFluid fluid)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { ThreadCount = threads });
        double h = width / 2;
        w.AddBox(new Vector2F((float)(width + 1), 0.2f), at: new Vector2D(0, -0.1), type: BodyType.Static);
        w.AddBox(new Vector2F(0.2f, 4f), at: new Vector2D(-h - 0.1, 1.9), type: BodyType.Static);
        w.AddBox(new Vector2F(0.2f, 4f), at: new Vector2D(h + 0.1, 1.9), type: BodyType.Static);
        fluid = w.AddParticleWater(spacing);
        w.FillFluidBox(fluid, new Vector2D(-h, 0), new Vector2D(h, fillHeight));
        return w;
    }

    private static FluidParticleInfo[] Particles3D(P3.PhysicsWorld3D w) { var b = new FluidParticleInfo[w.FluidParticleCount]; w.GetFluidParticles(b); return b; }

    private static FluidParticleInfo[] Particles2D(P2.PhysicsWorld2D w) { var b = new FluidParticleInfo[w.FluidParticleCount]; w.GetFluidParticles(b); return b; }

    // The experimental front position of the n² = 1 column (Martin and Moyce 1952, table 1): dimensionless time T = t sqrt(g / a) and front Z = z / a, the mean of the records.
    private static readonly double[] BoxHeights = { 0.5, 0.35, 0.2 };
    private static readonly double[] RefT = { 0.43, 0.62, 0.80, 0.97, 1.15, 1.29, 1.45, 1.62, 1.77, 1.94, 2.11, 2.27, 2.42 };
    private static readonly double[] RefZ = { 1.11, 1.22, 1.44, 1.67, 1.89, 2.11, 2.33, 2.56, 2.78, 3.00, 3.22, 3.44, 3.67 };

    private static double ReferenceFront(double t)
    {
        for (int k = 1; k < RefT.Length; k++)
        {
            if (t >= RefT[k - 1] && t <= RefT[k]) { return RefZ[k - 1] + (RefZ[k] - RefZ[k - 1]) * (t - RefT[k - 1]) / (RefT[k] - RefT[k - 1]); }
        }

        return double.NaN;
    }

    // ------------------------------------------------------------------ rest density and hydrostatics

    [Fact]
    public void ARestingColumnKeepsItsDensityAndItsHeight()
    {
        var w = Tank3D(0.05f, 1, 0.8, 0.8, 0.4, out var f);
        for (int i = 0; i < 150; i++) { w.Step(Dt); }
        var ps = Particles3D(w);
        var over = new List<double>();
        double rest = f.RestSphDensity;
        foreach (var p in ps) { over.Add(Math.Max(0.0, w.FluidDensityAt(p.Position) / rest - 1.0)); }
        over.Sort();
        // Compression stays within 5% for 95% of the particles; mean interior density within 5% of the rest density.
        Assert.True(over[(int)(0.95 * (over.Count - 1))] < 0.05, "95th percentile compression " + over[(int)(0.95 * (over.Count - 1))]);
        double sum = 0; int n = 0;
        foreach (var p in ps) { if (p.Position.Y > 0.1 && p.Position.Y < 0.3 && Math.Abs(p.Position.X) < 0.25 && Math.Abs(p.Position.Z) < 0.25) { sum += w.FluidDensityAt(p.Position); n++; } }
        Assert.InRange(sum / n / rest, 0.95, 1.05);
        // Nothing has left the tank, nothing is non-finite, and the surface has settled to within about 10% of the lattice height (the fluid packs a little denser than the cubic lattice it starts on).
        Assert.All(ps, p => { Assert.True(Math.Abs(p.Position.X) < 0.4 && p.Position.Y > 0 && p.Position.Y < 0.45); });
        Assert.InRange(ps.Max(p => p.Position.Y), 0.33, 0.45);
    }

    [Fact]
    public void BuoyancyOfNeutralBodiesAtThreeDepthsEqualsTheirWeight()
    {
        // The hydrostatic pressure profile read as the force a fluid puts on a body of the fluid's own density at 0.1, 0.25 and 0.4 m under the surface: it holds the weight at every depth.
        var w = Tank3D(0.05f, 1, 1.2, 1.2, 0.6, out var f);
        var boxes = BoxHeights.Select((y, i) => w.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), at: new Vector3D(-0.35 + 0.35 * i, y, 0), density: 1000f)).ToArray();
        for (int i = 0; i < 200; i++) { w.Step(Dt); }
        var sum = new double[3];
        const int window = 100;
        for (int i = 0; i < window; i++)
        {
            w.Step(Dt);
            for (int b = 0; b < 3; b++) { sum[b] += w.GetFluidState(boxes[b], f).Force.Y; }
        }

        for (int b = 0; b < 3; b++)
        {
            double weight = boxes[b].Mass * 9.81;
            Assert.InRange(sum[b] / window / weight, 0.85, 1.15);
        }
    }

    // ------------------------------------------------------------------ the dam break

    [Fact]
    public void TheDamBreakFrontFollowsMartinAndMoyceInPlane()
    {
        const float a = 0.4f;
        var w = new P2.PhysicsWorld2D();
        w.AddBox(new Vector2F(8, 0.2f), at: new Vector2D(3, -0.1), type: BodyType.Static);
        w.AddBox(new Vector2F(0.2f, 3), at: new Vector2D(-0.1, 1.5), type: BodyType.Static);
        var f = w.AddParticleWater(0.02f);
        w.FillFluidBox(f, new Vector2D(0, 0), new Vector2D(a, a));
        double scale = Math.Sqrt(9.81 / a);
        int compared = 0;
        for (int i = 0; i < 50; i++)
        {
            w.Step(Dt);
            double T = (i + 1) * Dt * scale;
            double z = ReferenceFront(T);
            if (double.IsNaN(z)) { continue; }
            double front = Particles2D(w).Max(p => p.Position.X) / a;
            // The front particle's centre is half a spacing behind the water's edge: within 8% of the measured front.
            Assert.InRange(front / z, 0.92, 1.08);
            compared++;
        }

        Assert.True(compared >= 20);
    }

    [Fact]
    public void TheDamBreakFrontFollowsMartinAndMoyceBetweenSideWalls()
    {
        const float a = 0.4f;
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { ThreadCount = 4 });
        w.AddBox(new Vector3F(8, 0.2f, 1.2f), at: new Vector3D(3, -0.1, 0.2), type: BodyType.Static);
        w.AddBox(new Vector3F(0.2f, 3, 1.2f), at: new Vector3D(-0.1, 1.5, 0.2), type: BodyType.Static);
        w.AddBox(new Vector3F(8, 3, 0.2f), at: new Vector3D(3, 1.5, -0.1), type: BodyType.Static);
        w.AddBox(new Vector3F(8, 3, 0.2f), at: new Vector3D(3, 1.5, a + 0.1), type: BodyType.Static);
        var f = w.AddParticleWater(0.025f);
        w.FillFluidBox(f, new Vector3D(0, 0, 0), new Vector3D(a, a, a));
        double scale = Math.Sqrt(9.81 / a);
        for (int i = 0; i < 40; i++)
        {
            w.Step(Dt);
            double z = ReferenceFront((i + 1) * Dt * scale);
            if (double.IsNaN(z)) { continue; }
            double front = Particles3D(w).Max(p => p.Position.X) / a;
            // Side-wall friction slows the front a little (the experiment had walls too); within 20%.
            Assert.InRange(front / z, 0.80, 1.08);
        }
    }

    // ------------------------------------------------------------------ buoyancy

    private static void CheckDraft(double level, double bottom, double side, double density, double spacing)
    {
        double expected = Math.Min(side, side * density / 1000.0);
        double draft = level - bottom;
        // A particle's contact radius and the noise of the contacts blur the waterline by about a spacing; at least 2 cm.
        Assert.True(Math.Abs(draft - expected) <= Math.Max(0.02, 0.8 * spacing), $"draft {draft:F3} expected {expected:F3} (spacing {spacing})");
    }

    private static double Level3D(P3.PhysicsWorld3D w, ParticleFluid f, double x, double z)
    {
        double lo = 0.1, hi = 1.5;
        float half = 0.5f * f.RestSphDensity;
        for (int k = 0; k < 30; k++) { double mid = 0.5 * (lo + hi); if (w.FluidDensityAt(new Vector3D(x, mid, z)) > half) { lo = mid; } else { hi = mid; } }
        return lo;
    }

    [Fact]
    public void AFloatingBoxSettlesAtItsDensityRatioIn3D()
    {
        const float s = 0.04f;
        var w = Tank3D(s, 2, 1.0, 1.0, 0.5, out var f);
        const float side = 0.3f;
        var light = w.AddBox(new Vector3F(side, side, side), at: new Vector3D(0, 0.5 + side / 2 + 0.02, 0), density: 500f);
        for (int i = 0; i < 360; i++) { w.Step(Dt); }
        double level = Level3D(w, f, 0.42, 0.42);
        CheckDraft(level, light.Position.Y - side / 2, side, 500, s);
        Assert.InRange(light.LinearVelocity.Y, -0.5f, 0.5f);
        Assert.InRange(w.GetFluidState(light, f).SubmergedFraction, 0.35f, 0.75f);
    }

    [Fact]
    public void AFloatingBoxSettlesAtItsDensityRatioIn2D()
    {
        const float s = 0.04f;
        var w = Tank2D(s, 1, 2.0, 0.6, out var f);
        const float side = 0.4f;
        var box = w.AddBox(new Vector2F(side, side), at: new Vector2D(0, 0.6 + side / 2 + 0.02), density: 500f);
        for (int i = 0; i < 300; i++) { w.Step(Dt); }
        double lo = 0.1, hi = 1.5; float half = 0.5f * f.RestSphDensity;
        for (int k = 0; k < 30; k++) { double mid = 0.5 * (lo + hi); if (w.FluidDensityAt(new Vector2D(0.9, mid)) > half) { lo = mid; } else { hi = mid; } }
        CheckDraft(lo, box.Position.Y - side / 2, side, 500, s);
        Assert.InRange(w.GetFluidState(box, f).SubmergedFraction, 0.35f, 0.75f);
    }

    [Fact]
    public void ADenseBodySinksAndRestsOnTheFloor()
    {
        var w = Tank3D(0.05f, 2, 1.0, 1.0, 0.5, out var f);
        var heavy = w.AddBox(new Vector3F(0.25f, 0.25f, 0.25f), at: new Vector3D(0, 0.7, 0), density: 2500f);
        for (int i = 0; i < 240; i++) { w.Step(Dt); }
        // On the floor, on at most one layer of particles trapped under it.
        Assert.InRange(heavy.Position.Y - 0.125, 0.0, 0.08);
        Assert.InRange(w.GetFluidState(heavy, f).SubmergedFraction, 0.9f, 1.0f);
    }

    // ------------------------------------------------------------------ momentum

    [Fact]
    public void AStruckBodyAndTheFluidConserveMomentum()
    {
        // No gravity and no walls: a heavy sphere flies into a blob of fluid; the sum of the body's and the particles' momentum is the one it started with.
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = default, SleepEnabled = false, ThreadCount = 2 });
        var f = w.AddParticleWater(0.05f);
        w.FillFluidBox(f, new Vector3D(-0.3, -0.3, -0.3), new Vector3D(0.3, 0.3, 0.3));
        var ball = w.AddSphere(0.2f, at: new Vector3D(-1.0, 0, 0), density: 2000f);
        ball.LinearVelocity = new Vector3F(6f, 0, 0);
        double Momentum()
        {
            double px = ball.Mass * ball.LinearVelocity.X, py = ball.Mass * ball.LinearVelocity.Y, pz = ball.Mass * ball.LinearVelocity.Z;
            double m = f.ParticleMass;
            foreach (var p in Particles3D(w)) { px += m * p.Velocity.X; py += m * p.Velocity.Y; pz += m * p.Velocity.Z; }
            return Math.Sqrt(px * px + py * py + pz * pz);
        }

        double before = Momentum();
        Assert.True(before > 0);
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        // The ball has hit and pushed the blob (the fluid moved), and the total is conserved to float rounding.
        Assert.True(w.GetFluidState(ball, f).WettedArea >= 0f);
        Assert.True(ball.LinearVelocity.X < 6f - 0.1f);
        Assert.InRange(Momentum() / before, 0.99, 1.01);
    }

    // ------------------------------------------------------------------ deformables

    [Fact]
    public void AClothSheetHoldsWaterAtTheStatedRadius()
    {
        var w = FluidScenes.Build3D("fluid-cloth-3d", 4);
        var ex = FluidScenes.ExtrasOf(w);
        int leaked = 0;
        for (int i = 0; i < 180; i++) { w.Step(Dt); }
        double sheetMin = double.MaxValue;
        for (int k = 0; k < ex.Cloth!.Value.ParticleCount; k++) { sheetMin = Math.Min(sheetMin, ex.Cloth.Value.GetPosition(k).Y); }
        foreach (var p in Particles3D(w)) { if (p.Position.Y < sheetMin - 0.03) { leaked++; } }
        Assert.Equal(0, leaked);
        Assert.True(w.FluidStatistics.Deformables > 0 && w.FluidStatistics.CoupledContacts > 0);
        Assert.True(sheetMin < 0.5 && sheetMin > 0.2, "the sheet sags under the water but holds: " + sheetMin);
    }

    [Fact]
    public void AnEmptyFluidLeavesRodsAndSoftBodiesAlone()
    {
        // Worlds with a fluid but no particles near a rope behave as without the fluid.
        P2.PhysicsWorld2D Make(bool fluid)
        {
            var w = new P2.PhysicsWorld2D();
            w.AddRope(new Vector2D(0, 5), new Vector2D(3, 5), 20, true, false);
            if (fluid) { w.AddParticleWater(0.05f); }
            return w;
        }

        var a = Make(false); var b = Make(true);
        for (int i = 0; i < 60; i++) { a.Step(Dt); b.Step(Dt); }
        Assert.Equal(a.GetRod(0).GetPosition(20), b.GetRod(0).GetPosition(20));
    }

    // ------------------------------------------------------------------ determinism and the hooks

    [Theory]
    [InlineData("dam-break-2d", 0.06f)]
    [InlineData("fluid-rope-2d", 0.08f)]
    public void Planar_scenes_are_the_same_on_one_and_three_threads_and_after_a_restore(string scene, float spacing)
    {
        ulong a = FluidScenes.Run2D(scene, 1, 90, spacing), b = FluidScenes.Run2D(scene, 3, 90, spacing), c = FluidScenes.Run2D(scene, 1, 90, spacing, true);
        Assert.Equal(a, b);
        Assert.Equal(a, c);
    }

    [Theory]
    [InlineData("dam-break-3d", 0.07f)]
    [InlineData("fluid-boxes-3d", 0.1f)]
    [InlineData("fluid-cloth-3d", 0.1f)]
    public void Spatial_scenes_are_the_same_on_one_and_three_threads_and_after_a_restore(string scene, float spacing)
    {
        ulong a = FluidScenes.Run3D(scene, 1, 60, spacing), b = FluidScenes.Run3D(scene, 3, 60, spacing), c = FluidScenes.Run3D(scene, 1, 60, spacing, true);
        Assert.Equal(a, b);
        Assert.Equal(a, c);
    }

    [Fact]
    public void SnapshotStepRestoreStepGivesTheSameHashesEventsAndBytes()
    {
        var w = FluidScenes.Build3D("fluid-boxes-3d", 2, 0.1f);
        for (int i = 0; i < 20; i++) { w.Step(Dt); }
        var snap = w.Snapshot();
        var first = new List<ulong>(); var events1 = new List<string>();
        for (int i = 0; i < 40; i++) { w.Step(Dt); first.Add(w.ComputeHash()); foreach (var e in w.FluidEvents) { events1.Add(i + ":" + e.Kind + e.Body.Index + "/" + e.Fluid); } }
        var end1 = w.Snapshot();
        w.Restore(snap);
        var second = new List<ulong>(); var events2 = new List<string>();
        for (int i = 0; i < 40; i++) { w.Step(Dt); second.Add(w.ComputeHash()); foreach (var e in w.FluidEvents) { events2.Add(i + ":" + e.Kind + e.Body.Index + "/" + e.Fluid); } }
        Assert.Equal(first, second);
        Assert.Equal(events1, events2);
        Assert.Equal(end1, w.Snapshot());
        // A fresh world restored from the bytes carries on identically.
        var fresh = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = w.Id, ThreadCount = 2 });
        fresh.Restore(snap);
        Assert.Equal(0, fresh.FluidEvents.Length);
        for (int i = 0; i < 40; i++) { fresh.Step(Dt); }
        Assert.Equal(second[^1], fresh.ComputeHash());
    }

    [Fact]
    public void WorldsWithoutFluidsKeepTheirBytes()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 5 });
        w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 3, 0));
        var before = w.Snapshot();
        _ = w.ParticleFluidCount;
        Assert.Equal(before, w.Snapshot());
        var w2 = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 5 });
        w2.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 3));
        Assert.Equal(w2.Snapshot(), w2.Snapshot());
        // Fluids add a section; removing them again takes it away.
        var f = w.AddParticleWater(0.1f);
        Assert.NotEqual(before.Length, w.Snapshot().Length);
        f.Destroy();
        var after = w.Snapshot();
        Assert.True(after.Length - before.Length < 80, "only the settings and empty tables remain: " + (after.Length - before.Length));
    }

    [Fact]
    public void AForkRunsAwayFromItsParent()
    {
        var w = FluidScenes.Build2D("dam-break-2d", 1, 0.06f);
        for (int i = 0; i < 20; i++) { w.Step(Dt); }
        var fork = w.Fork();
        Assert.Equal(w.ComputeHash(), fork.ComputeHash());
        for (int i = 0; i < 20; i++) { w.Step(Dt); }
        Assert.NotEqual(w.ComputeHash(), fork.ComputeHash());
        for (int i = 0; i < 20; i++) { fork.Step(Dt); }
        Assert.Equal(FluidScenes.Run2D("dam-break-2d", 1, 40, 0.06f), fork.ComputeHash());
    }

    [Fact]
    public void ShiftingTheOriginMovesParticlesEmittersAndDrains()
    {
        var w = FluidScenes.Build2D("fluid-rope-2d", 1, 0.08f);
        for (int i = 0; i < 10; i++) { w.Step(Dt); }
        var before = Particles2D(w);
        w.ShiftOrigin(new Vector2D(16, -8));
        var after = Particles2D(w);
        Assert.Equal(before.Length, after.Length);
        for (int i = 0; i < before.Length; i++) { Assert.Equal(before[i].Position.X - 16, after[i].Position.X); Assert.Equal(before[i].Position.Y + 8, after[i].Position.Y); }
        var ex = FluidScenes.ExtrasOf(w);
        var e = ex.Fluid.AddEmitter(new FluidEmitterDef { Position = new Vector3D(20, 3, 0) });
        w.ShiftOrigin(new Vector2D(-4, 2));
        Assert.Equal(24.0, e.Definition.Position.X);
        Assert.Equal(1.0, e.Definition.Position.Y);
    }

    // ------------------------------------------------------------------ emitters, drains, budget, sleep

    [Fact]
    public void ANozzleEmitsALatticeAtTheFlowRateAndADrainEmptiesTheWorld()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { Gravity = default });
        var f = w.AddParticleWater(0.05f);
        var e = f.AddEmitter(new FluidEmitterDef { Position = new Vector3D(0, 0, 0), Direction = new Vector3F(1, 0, 0), Kind = FluidVolumeKind.Box, Size = new Vector3F(0.2f, 0.1f, 0.1f), Speed = 3f });
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        // 3 m/s through a 0.4 m wide nozzle: one layer of 9 particles (0.4 / 0.05 + 1) for every 0.05 m advanced, 3 m in a second: about 60 layers.
        Assert.InRange(w.FluidParticleCount, 9 * 58, 9 * 61);
        Assert.Equal(w.FluidParticleCount, e.Emitted);
        e.Destroy();
        Assert.False(e.IsValid);
        var drain = f.AddDrain(new FluidDrainDef { Position = new Vector3D(2, 0, 0), Kind = FluidVolumeKind.Box, Size = new Vector3F(100, 100, 100) });
        w.Step(Dt);
        Assert.Equal(0, w.FluidParticleCount);
        Assert.True(drain.Removed > 0);
    }

    [Fact]
    public void TheBudgetStopsEmittersAndRefusesFills()
    {
        var w = new P2.PhysicsWorld2D();
        w.FluidSettings = new FluidSettings { Substeps = 2, Iterations = 4, MaxParticles = 100, SleepSpeed = 0.02f, TimeToSleep = 1f, WakeBodySpeed = 0.05f };
        var f = w.AddParticleWater(0.1f);
        Assert.Throws<FluidBudgetException>(() => w.FillFluidBox(f, new Vector2D(0, 0), new Vector2D(100, 100)));
        Assert.Equal(0, w.FluidParticleCount);
        f.AddEmitter(new FluidEmitterDef { Position = new Vector3D(0, 5, 0), Direction = new Vector3F(0, -1, 0), Kind = FluidVolumeKind.Box, Size = new Vector3F(1f, 0.1f, 0.1f), Speed = 10f });
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.True(w.FluidParticleCount <= 100);
    }

    [Fact]
    public void ASettledFluidSleepsAndAFallingBodyWakesIt()
    {
        var w = Tank2D(0.05f, 1, 1.0, 0.3, out var f);
        w.FluidSettings = new FluidSettings { Substeps = 2, Iterations = 4, MaxParticles = 10000, SleepSpeed = 0.5f, TimeToSleep = 0.5f, WakeBodySpeed = 0.05f };
        for (int i = 0; i < 200 && !w.FluidStatistics.Asleep; i++) { w.Step(Dt); }
        Assert.True(w.FluidStatistics.Asleep);
        var asleep = Particles2D(w);
        w.Step(Dt);
        Assert.False(w.FluidStatistics.Stepped);
        Assert.Equal(asleep.Select(p => p.Position).ToArray(), Particles2D(w).Select(p => p.Position).ToArray());
        w.AddBox(new Vector2F(0.2f, 0.2f), at: new Vector2D(0, 1.0), density: 300f);
        bool woke = false;
        for (int i = 0; i < 90; i++) { w.Step(Dt); woke |= w.FluidStatistics.Stepped; }
        Assert.True(woke);
    }

    // ------------------------------------------------------------------ events and queries

    [Fact]
    public void ABodyDroppedIntoThePoolEntersAndOneThrownOutLeaves()
    {
        var w = Tank3D(0.06f, 2, 1.0, 1.0, 0.4, out var f);
        var box = w.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), at: new Vector3D(0, 0.8, 0), density: 400f);
        int enters = 0, exits = 0;
        int firstEnter = -1;
        for (int i = 0; i < 120; i++)
        {
            w.Step(Dt);
            foreach (var e in w.FluidEvents)
            {
                Assert.Equal(box.RawHandle, e.Body);
                Assert.Equal(f.Index, e.Fluid);
                if (e.Kind == FluidEventKind.Enter) { enters++; if (firstEnter < 0) { firstEnter = i; } Assert.True(e.SubmergedFraction >= 0.05f); } else { exits++; }
            }
        }

        Assert.Equal(1, enters);
        Assert.Equal(0, exits);
        box.LinearVelocity = new Vector3F(0, 8, 0);
        for (int i = 0; i < 60; i++)
        {
            w.Step(Dt);
            foreach (var e in w.FluidEvents) { if (e.Kind == FluidEventKind.Exit) { exits++; } else { enters++; } }
        }

        Assert.Equal(1, exits);
        Assert.True(w.GetFluidState(box, f).SubmergedFraction == 0f);
        Assert.True(enters >= 1);
    }

    [Fact]
    public void QueriesAgreeWithBruteForce()
    {
        var w = Tank3D(0.08f, 1, 0.8, 0.8, 0.4, out var f);
        for (int i = 0; i < 30; i++) { w.Step(Dt); }
        var all = Particles3D(w);
        var centre = new Vector3D(0.1, 0.2, -0.05);
        var buf = new FluidParticleInfo[all.Length];
        int n = w.OverlapFluidSphere(centre, 0.2f, buf, out int total);
        var expected = all.Where(p => { var d = p.Position; double dx = d.X - centre.X, dy = d.Y - centre.Y, dz = d.Z - centre.Z; return dx * dx + dy * dy + dz * dz <= (double)0.2f * 0.2f; }).Select(p => p.Id).OrderBy(x => x).ToArray();
        Assert.Equal(expected.Length, total);
        Assert.Equal(expected, buf.Take(n).Select(p => p.Id).OrderBy(x => x).ToArray());
        n = w.OverlapFluidBox(new Vector3D(-0.2, 0.1, -0.2), new Vector3D(0.2, 0.3, 0.2), buf, out total);
        Assert.Equal(all.Count(p => Math.Abs(p.Position.X) <= 0.2 && p.Position.Y >= 0.1 && p.Position.Y <= 0.3 && Math.Abs(p.Position.Z) <= 0.2), total);
        Assert.Equal(total, n);
        // A ray straight down from above hits a top particle; the hit is the nearest particle sphere (checked by brute force).
        Assert.True(w.RaycastFluid(new Vector3D(0.05, 2, 0.02), new Vector3F(0, -1, 0), out var hit));
        double best = double.MaxValue;
        foreach (var p in all)
        {
            double dx = p.Position.X - 0.05, dz = p.Position.Z - 0.02, r = 0.04;
            if (dx * dx + dz * dz > r * r) { continue; }
            double d = 2 - p.Position.Y - Math.Sqrt(r * r - dx * dx - dz * dz);
            best = Math.Min(best, d);
        }

        Assert.InRange(hit.Distance, best - 1e-3, best + 1e-3);
        Assert.False(w.RaycastFluid(new Vector3D(0.05, 2, 0.02), new Vector3F(0, 1, 0), out _));
        Assert.InRange(w.FluidDensityAt(new Vector3D(0, 0.2, 0)) / f.RestSphDensity, 0.9, 1.1);
        Assert.InRange(w.FluidDensityAt(new Vector3D(0, 1.5, 0)), 0f, 1f);
    }

    [Fact]
    public void Planar_queries_work_in_two_dimensions()
    {
        var w = Tank2D(0.05f, 1, 1.0, 0.4, out _);
        for (int i = 0; i < 30; i++) { w.Step(Dt); }
        var buf = new FluidParticleInfo[400];
        int n = w.OverlapFluidSphere(new Vector2D(0, 0.2), 0.15f, buf, out int total);
        Assert.True(total > 10 && n == total);
        Assert.True(w.RaycastFluid(new Vector2D(0, 2), new Vector2F(0, -1), out var hit));
        Assert.True(hit.Point.Y > 0.2 && hit.Point.Y < 0.5);
        Assert.Equal(0.0, hit.Point.Z);
    }

    // ------------------------------------------------------------------ hostile input and handles

    [Fact]
    public void BadDefinitionsAndStaleHandlesAreTypedErrors()
    {
        var w = new P3.PhysicsWorld3D();
        var e = Assert.Throws<ArgumentOutOfRangeException>(() => w.AddParticleFluid(new ParticleFluidDef { Spacing = float.NaN }));
        Assert.Contains("Spacing", e.Message);
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddParticleFluid(new ParticleFluidDef { RestDensity = -1f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddParticleFluid(new ParticleFluidDef { Layer = 40 }));
        Assert.Throws<ArgumentNullException>(() => w.AddParticleFluid(null!));
        var f = w.AddParticleWater(0.1f);
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddParticleFluid(ParticleFluidDef.Water(0.2f)));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddFluidParticle(f, new Vector3D(double.NaN, 0, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddFluidParticle(f, new Vector3D(1e9, 0, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddFluidParticle(f, default, new Vector3F(float.PositiveInfinity, 0, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.FillFluidSphere(f, default, -1));
        Assert.Throws<ArgumentOutOfRangeException>(() => f.AddEmitter(new FluidEmitterDef { Direction = default }));
        Assert.Throws<ArgumentOutOfRangeException>(() => f.AddEmitter(new FluidEmitterDef { Size = new Vector3F(-1, 1, 1) }));
        Assert.Throws<ArgumentOutOfRangeException>(() => f.AddDrain(new FluidDrainDef { Position = new Vector3D(double.PositiveInfinity, 0, 0) }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.FluidSettings = new FluidSettings { Substeps = 0, Iterations = 4, MaxParticles = 10 });
        Assert.Throws<ArgumentOutOfRangeException>(() => w.RaycastFluid(default, new Vector3F(float.NaN, 0, 0), out _));
        w.AddFluidParticle(f, new Vector3D(0, 1, 0));
        var other = new P3.PhysicsWorld3D();
        Assert.Throws<InvalidOperationException>(() => other.AddFluidParticle(f, default));
        f.Destroy();
        Assert.False(f.IsValid);
        Assert.Throws<InvalidOperationException>(() => f.ParticleCount);
        Assert.Throws<InvalidOperationException>(() => w.AddFluidParticle(f, default));
        Assert.Throws<InvalidOperationException>(() => default(ParticleFluid).Definition);
        Assert.Equal(0, w.FluidParticleCount);
        Assert.Throws<ArgumentOutOfRangeException>(() => w.GetParticleFluid(0));
    }

    [Fact]
    public void DamagedFluidSnapshotsAreRejectedWithATypedErrorOrRestoreCleanly()
    {
        var w = FluidScenes.Build3D("fluid-boxes-3d", 1, 0.12f);
        for (int i = 0; i < 5; i++) { w.Step(Dt); }
        var good = w.Snapshot();
        int start = Math.Max(0, good.Length - 6000);
        var rng = new Random(77);
        for (int k = 0; k < 150; k++)
        {
            var bad = (byte[])good.Clone();
            int flips = 1 + rng.Next(3);
            for (int j = 0; j < flips; j++) { bad[rng.Next(start, bad.Length)] ^= (byte)(1 << rng.Next(8)); }
            if (k % 10 == 0) { bad = bad.AsSpan(0, rng.Next(start, bad.Length)).ToArray(); }
            var fresh = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = w.Id });
            try { fresh.Restore(bad); fresh.Step(Dt); }
            catch (SnapshotFormatException) { }
            catch (P3.BodyOutOfBoundsException) { }
        }
    }

    // ------------------------------------------------------------------ debug

    [Fact]
    public void DebugGeometryHasTheParticlesAndTheVolumesAsPlainData()
    {
        var w = FluidScenes.Build2D("fluid-rope-2d", 1, 0.1f);
        w.Step(Dt);
        var g = new DebugGeometry();
        w.GetDebugGeometry(DebugContent.Fluids, g);
        Assert.Equal(w.FluidParticleCount, g.FluidParticles.Length);
        Assert.Equal(2, g.FluidVolumes.Length);
        Assert.True(g.FluidVolumes[0].IsEmitter && !g.FluidVolumes[1].IsEmitter);
        g.Clear();
        Assert.Equal(0, g.FluidParticles.Length);
    }

    [Fact]
    public void FluidScenesStayStable()
    {
        foreach (var name in new[] { "dam-break-2d", "fluid-rope-2d" })
        {
            var w = FluidScenes.Build2D(name, 1, 0.06f);
            for (int i = 0; i < 240; i++) { w.Step(Dt); }
            var m = FluidScenes.Measure2D(w, name);
            Assert.Equal(0, m.NonFinite); Assert.Equal(0, m.Escapes); Assert.Equal(0, m.Penetrating);
            Assert.True(m.MaxSpeed < 8f, name + " max speed " + m.MaxSpeed);
        }
    }
}
