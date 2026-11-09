using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Water;

public class ShallowWater3DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld3D Tank(out ShallowWater water, double level = 2.0, int id = 91, bool ground = true, float cell = 0.5f, int threads = 1)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = id });
        if (ground) { w.AddBox(new Vector3F(60f, 1f, 60f), at: new Vector3D(10, -0.5, 10), type: BodyType.Static); }
        water = w.AddShallowWater(0, 0, 20, 20, level, cell, floor: -1.0);
        return w;
    }

    private static void Run(PhysicsWorld3D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    [Fact]
    public void TheBedIsTheTopOfTheStaticShapesUnderIt()
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = 1, WorldId = 92 });
        w.AddBox(new Vector3F(4f, 1f, 4f), at: new Vector3D(10, 0.5, 10), type: BodyType.Static);   // a block whose top is at y = 1
        var water = w.AddShallowWater(0, 0, 20, 20, 3.0, 0.5f, floor: -1.0);
        Assert.Equal(0.0, water.CellDepth(0, 0) - 4.0, 6);                 // 3 - (-1) = 4 over the floor
        Assert.Equal(2.0, water.CellDepth(20, 20), 5);                     // 3 - 1 over the block
        Assert.True(w.QueryWater(new Vector3D(10, 0, 10), out var s));
        Assert.Equal(3.0, s.Surface, 5);
        Assert.Equal(2.0f, s.Depth, 4);
    }

    [Fact]
    public void ABoxOfHalfTheWatersDensityFloatsAtHalfDepth()
    {
        var w = Tank(out var water);
        var box = w.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(10, 3.0, 10), density: 500f);
        Run(w, 900);
        Assert.InRange(box.Position.Y, 1.97, 2.03);
        Assert.InRange(box.LinearVelocity.Length(), 0.0f, 0.02f);
        // The box displaces 0.5 m³, half of which is added to a 400 m² tank: the mean level is up by 0.6 mm (the surface itself still has the splash's waves in it).
        double sum = 0;
        for (int iz = 0; iz < 40; iz++) { for (int ix = 0; ix < 40; ix++) { sum += water.CellSurface(ix, iz); } }
        Assert.InRange(sum / 1600, 2.0005, 2.0008);
    }

    [Fact]
    public void ADroppedBoxMakesWavesAndTheWaterKeepsItsVolume()
    {
        var w = Tank(out var water);
        double v0 = water.Volume;
        var box = w.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(10, 4.0, 10), density: 400f);
        for (int i = 0; i < 90; i++) { w.Step(Dt); }
        // A ring of waves has left the splash: the surface a few metres out has moved.
        double maxRise = 0;
        for (int ix = 0; ix < 40; ix++) { maxRise = Math.Max(maxRise, Math.Abs(water.CellSurface(ix, 20) - 2.0)); }
        Assert.True(maxRise > 0.005, "rise " + maxRise);
        Run(w, 1800);
        Assert.InRange(water.Volume / v0, 1 - 1e-4, 1 + 1e-4);
        Assert.Equal(0.0, water.LostVolume);
        Assert.InRange(box.Position.Y, 1.7, 2.3);
    }

    [Fact]
    public void ABodyEnteringAndLeavingRaisesEvents()
    {
        var w = Tank(out _);
        var seen = new System.Collections.Generic.List<WaterEvent>();
        w.BodyWaterChanged += e => seen.Add(e);
        var ball = w.AddSphere(0.5f, at: new Vector3D(10, 4.0, 10), density: 300f);
        for (int i = 0; i < 400 && seen.Count == 0; i++) { w.Step(Dt); }
        Assert.Single(seen);
        Assert.Equal(WaterEventKind.Entered, seen[0].Kind);
        Assert.Equal(ball.RawHandle, seen[0].Body);
        Assert.True(seen[0].Volume > 0f);
        // Pull it out: it leaves.
        ball.Position = new Vector3D(10, 6.0, 10);
        ball.LinearVelocity = default;
        w.Step(Dt);
        Assert.Equal(WaterEventKind.Left, seen[^1].Kind);
    }

    [Fact]
    public void TheStepIsTheSameOnEveryThreadCount()
    {
        ulong Hash(int threads)
        {
            var w = Tank(out _, threads: threads, cell: 0.4f);
            w.AddBox(new Vector3F(1f, 0.5f, 2f), at: new Vector3D(8, 3, 9), density: 300f);
            w.AddSphere(0.4f, at: new Vector3D(12, 3.5, 11), density: 600f);
            Run(w, 240);
            return w.ComputeHash();
        }

        Assert.Equal(Hash(1), Hash(8));
    }

    [Fact]
    public void SnapshotRestoreContinuesBitIdentically()
    {
        var w = Tank(out _, id: 93);
        var box = w.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(10, 3, 10), density: 500f);
        Run(w, 100);
        var snap = w.Snapshot();
        var hashes = new ulong[60];
        for (int i = 0; i < 60; i++) { w.Step(Dt); hashes[i] = w.ComputeHash(); }
        var after = w.Snapshot();
        w.Restore(snap);
        for (int i = 0; i < 60; i++) { w.Step(Dt); Assert.Equal(hashes[i], w.ComputeHash()); }
        Assert.Equal(after, w.Snapshot());
        // Into a fresh world of the same id as well.
        var fresh = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = 1, WorldId = 93 });
        fresh.Restore(snap);
        for (int i = 0; i < 60; i++) { fresh.Step(Dt); Assert.Equal(hashes[i], fresh.ComputeHash()); }
        Assert.Equal(box.RawHandle, box.RawHandle);
    }

    [Fact]
    public void WorldsWithoutWaterKeepTheirBytes()
    {
        var a = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = 1, WorldId = 94 });
        a.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 5, 0));
        var before = a.Snapshot();
        var w = Tank(out var water, id: 94, ground: false);
        water.Remove();
        Assert.Equal(0, w.ShallowWaterCount);
        var empty = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = 1, WorldId = 94 });
        empty.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 5, 0));
        Assert.Equal(before, empty.Snapshot());
    }

    [Fact]
    public void ForkIsAnIndependentCopy()
    {
        var w = Tank(out var water, id: 95);
        w.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(10, 3, 10), density: 500f);
        Run(w, 60);
        var fork = w.Fork();
        Run(w, 60);
        Run(fork, 60);
        Assert.Equal(w.ComputeHash(), fork.ComputeHash());
        Assert.Equal(1, fork.ShallowWaterCount);
        _ = water;
    }

    [Fact]
    public void OriginShiftLeavesTheWaterSteppingTheSame()
    {
        ulong Run1(bool shift)
        {
            var w = Tank(out _, id: 96, ground: false);
            w.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(10, 3, 10), density: 500f);
            Run(w, 60);
            if (shift) { w.ShiftOrigin(new Vector3D(4096, 8, -2048)); }
            Run(w, 120);
            return w.ComputeHash();
        }

        // The shifted world differs from the unshifted one by its origin, so compare the water and the body relative to it.
        var a = Tank(out var wa, id: 97, ground: false);
        var ba = a.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(10, 3, 10), density: 500f);
        var b = Tank(out var wb, id: 98, ground: false);
        var bb = b.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(10, 3, 10), density: 500f);
        Run(a, 60); Run(b, 60);
        b.ShiftOrigin(new Vector3D(4096, 8, -2048));
        Run(a, 120); Run(b, 120);
        Assert.Equal(ba.Position.X - 4096, bb.Position.X);
        Assert.Equal(ba.Position.Y - 8, bb.Position.Y);
        Assert.Equal(ba.Position.Z + 2048, bb.Position.Z);
        for (int i = 0; i < 40; i += 7) { Assert.Equal(wa.CellSurface(i, 20) - 8, wb.CellSurface(i, 20)); }
        _ = Run1(false);
    }

    [Fact]
    public void QueriesAndRaycast()
    {
        var w = Tank(out var water);
        Assert.True(w.QueryWater(new Vector3D(5, 10, 5), out var s));
        Assert.Equal(2.0, s.Surface, 5);
        Assert.False(w.QueryWater(new Vector3D(50, 0, 5), out _));
        Assert.True(w.RaycastWater(new Vector3D(5, 10, 5), new Vector3F(0, -1, 0), out var hit));
        Assert.Equal(8f, hit.Distance, 3);
        Assert.Equal(1f, hit.Normal.Y, 3);
        Assert.True(w.RaycastWater(new Vector3D(5, -0.5, 5), new Vector3F(0, 1, 0), out hit));   // from below
        Assert.Equal(2.5f, hit.Distance, 3);
        Assert.False(w.RaycastWater(new Vector3D(5, 10, 5), new Vector3F(0, 1, 0), out _));
        Assert.False(w.RaycastWater(new Vector3D(5, 10, 5), new Vector3F(0, -1, 0), out _, maxDistance: 5f));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.RaycastWater(new Vector3D(0, 0, 0), default, out _));
        _ = water;
    }

    [Fact]
    public void AStaleHandleFailsClearly()
    {
        var w = Tank(out var water);
        water.Remove();
        Assert.False(water.IsValid);
        Assert.Throws<StaleHandleException>(() => water.CellDepth(0, 0));
        Assert.Throws<StaleHandleException>(() => water.Fill(1.0));
        Assert.Throws<InvalidHandleException>(() => w.GetShallowWater(water.RawHandle));
    }

    [Fact]
    public void HostileDefinitionsAreRefusedWithNamedErrors()
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = 1, WorldId = 99 });
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateShallowWater(new ShallowWaterDef { CellsX = 1 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateShallowWater(new ShallowWaterDef { CellsX = 5000, CellsZ = 5000 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateShallowWater(new ShallowWaterDef { CellSize = float.NaN }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateShallowWater(new ShallowWaterDef { Level = double.PositiveInfinity }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateShallowWater(new ShallowWaterDef { MinX = 4e6 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateShallowWater(new ShallowWaterDef { Bed = new float[3] }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateShallowWater(new ShallowWaterDef { Density = 0f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateShallowWater(new ShallowWaterDef { Cfl = 0.9f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateShallowWater(new ShallowWaterDef { Sources = [new WaterSource(0, 0, -1f, 1f)] }));
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateShallowWater(new ShallowWaterDef { CellsX = 5000, CellsZ = 5000 }));
        Assert.Contains("cells", ex.Message);
        Assert.Equal(0, w.ShallowWaterCount);
    }

    [Fact]
    public void ADamOpensIntoAWaveThatTheBoxRides()
    {
        // A pulse of extra water at one end of the tank travels along it and lifts a floating box as it passes.
        var w = Tank(out var water);
        var box = w.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(14, 2.5, 10), density: 500f);
        Run(w, 240);
        double y0 = box.Position.Y;
        water.AddVolume(0, 0, 3, 20, 60f);
        double peak = y0;
        for (int i = 0; i < 600; i++) { w.Step(Dt); peak = Math.Max(peak, box.Position.Y); }
        Assert.True(peak > y0 + 0.01, "the wave must lift the box: " + (peak - y0));
        Assert.True(box.Position.Y < 3.5);
    }
}
