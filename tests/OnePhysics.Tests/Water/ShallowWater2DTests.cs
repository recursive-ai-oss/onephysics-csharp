using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Water;

public class ShallowWater2DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D Tank(out ShallowWater water, int id = 191, float cell = 0.25f, int threads = 1)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = threads, WorldId = id });
        w.AddBox(new Vector2F(60f, 1f), new Vector2D(20, -0.5), type: BodyType.Static);
        water = w.CreateShallowWater(new ShallowWaterDef { Cells = (int)(40 / cell), CellSize = cell, Level = 2.0, FloorHeight = -1.0, Damping = 1f });
        return w;
    }

    private static void Run(PhysicsWorld2D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    [Fact]
    public void ABoxOfHalfTheWatersDensityFloatsAtHalfDepth()
    {
        var w = Tank(out var water);
        var box = w.AddBox(new Vector2F(1f, 1f), new Vector2D(10, 3.0), density: 500f);
        Run(w, 900);
        water.TryGetSample(10, out var dbg);
        Assert.True(box.Position.Y > 1.97 && box.Position.Y < 2.03, "y " + box.Position.Y + " surface " + dbg.Surface + " depth " + dbg.Depth + " vel " + dbg.Velocity + " boxv " + box.LinearVelocity.Y);
        Assert.InRange(box.LinearVelocity.Length(), 0f, 0.02f);
        double sum = 0;
        for (int i = 0; i < 160; i++) { sum += water.CellSurface(i); }
        Assert.InRange(sum / 160, 2.0 + 0.25 / 40 - 3e-4, 2.0 + 0.25 / 40 + 3e-4);
    }

    [Fact]
    public void ALongBargeSettlesToRestInAChannel()
    {
        var w = Tank(out _);
        var barge = w.AddBox(new Vector2F(10f, 1f), new Vector2D(20, 2.0), density: 400f);
        double lo = double.MaxValue, hi = double.MinValue;
        for (int i = 0; i < 2400; i++) { w.Step(Dt); if (i > 1800) { lo = Math.Min(lo, barge.Position.Y); hi = Math.Max(hi, barge.Position.Y); } }
        Assert.True(hi - lo < 0.01, "heave range after settling " + (hi - lo));
        Assert.InRange(Math.Abs(barge.Angle), 0f, 0.01f);
    }

    [Fact]
    public void SnapshotRestoreAndForkContinueBitIdentically()
    {
        var w = Tank(out _, id: 192);
        w.AddBox(new Vector2F(1f, 1f), new Vector2D(10, 3.0), density: 500f);
        w.AddCircle(0.4f, new Vector2D(14, 4.0), density: 300f);
        Run(w, 80);
        var snap = w.Snapshot();
        var fork = w.Fork();
        var hashes = new ulong[60];
        for (int i = 0; i < 60; i++) { w.Step(Dt); hashes[i] = w.ComputeHash(); }
        for (int i = 0; i < 60; i++) { fork.Step(Dt); Assert.Equal(hashes[i], fork.ComputeHash()); }
        var fresh = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = 1, WorldId = 192 });
        fresh.Restore(snap);
        for (int i = 0; i < 60; i++) { fresh.Step(Dt); Assert.Equal(hashes[i], fresh.ComputeHash()); }
        Assert.Equal(w.Snapshot(), fresh.Snapshot());
    }

    [Fact]
    public void TheStepIsTheSameOnEveryThreadCount()
    {
        ulong Hash(int threads)
        {
            var w = Tank(out var water, threads: threads);
            w.AddBox(new Vector2F(1f, 1f), new Vector2D(10, 3.0), density: 500f);
            for (int i = 0; i < 240; i++) { if (i == 30) { water.AddVolume(0, 2, 1f); } w.Step(Dt); }
            return w.ComputeHash();
        }

        Assert.Equal(Hash(1), Hash(8));
    }

    [Fact]
    public void QueriesEventsAndErrors()
    {
        var w = Tank(out var water);
        Assert.True(w.QueryWater(new Vector2D(5, 10), out var s));
        Assert.Equal(2.0, s.Surface, 5);
        Assert.True(w.RaycastWater(new Vector2D(5, 10), new Vector2F(0, -1), out var hit));
        Assert.Equal(8f, hit.Distance, 3);
        Assert.False(w.RaycastWater(new Vector2D(5, 10), new Vector2F(0, 1), out _));
        var seen = new System.Collections.Generic.List<WaterEvent>();
        w.BodyWaterChanged += e => seen.Add(e);
        w.AddBox(new Vector2F(1f, 1f), new Vector2D(10, 3.5), density: 300f);
        Run(w, 120);
        Assert.Contains(seen, e => e.Kind == WaterEventKind.Entered);
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateShallowWater(new ShallowWaterDef { Cells = 1 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateShallowWater(new ShallowWaterDef { Bed = new float[3] }));
        water.Remove();
        Assert.Throws<StaleHandleException>(() => water.CellDepth(0));
    }

    [Fact]
    public void ClosedWaterKeepsItsVolumeWithBodiesComingAndGoing()
    {
        var w = Tank(out var water);
        double v0 = water.Volume;
        var a = w.AddBox(new Vector2F(1f, 1f), new Vector2D(10, 3.5), density: 400f);
        var b = w.AddCircle(0.5f, new Vector2D(20, 5.0), density: 700f);
        Run(w, 200);
        a.Destroy();
        Run(w, 100);
        b.Position = new Vector2D(30, 7.0);
        Run(w, 300);
        Assert.InRange(water.Volume / v0, 1 - 1e-5, 1 + 1e-5);
    }
}
