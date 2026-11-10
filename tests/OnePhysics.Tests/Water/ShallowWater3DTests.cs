using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Replay;
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

    [Fact]
    public void DebugGeometryCarriesTheSurfaceAsPlainData()
    {
        var w = Tank(out _);
        w.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(10, 3, 10), density: 500f);
        Run(w, 60);
        var g = new DebugGeometry();
        w.GetDebugGeometry(DebugContent.ShallowWater, g);
        Assert.Equal(1, g.WaterSurfaces.Length);
        var s = g.WaterSurfaces[0];
        Assert.Equal(40, s.CellsX); Assert.Equal(40, s.CellsZ);
        Assert.Equal(40 * 40, g.WaterHeights.Length);
        Assert.Equal(g.WaterHeights.Length, g.WaterDepths.Length);
        Assert.Equal(-1.0, s.Origin.Y);
        Assert.InRange(g.WaterHeights[0] + s.Origin.Y, 1.9, 2.1);
        w.GetDebugGeometry(DebugContent.Shapes, g);
        Assert.Equal(0, g.WaterSurfaces.Length);
    }

    [Fact]
    public void RecordedMutationsReplayToTheSameWorld()
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = 1, WorldId = 197 });
        w.AddBox(new Vector3F(60f, 1f, 60f), new Vector3D(10, -0.5, 10), type: BodyType.Static);
        using var stream = new System.IO.MemoryStream();
        var expected = new System.Collections.Generic.List<ulong>();
        using (var recorder = w.StartRecording(stream))
        {
            ShallowWater water = default;
            for (int i = 0; i < 40; i++)
            {
                if (i == 2) { water = w.AddShallowWater(0, 0, 20, 20, 2.0, 0.5f, floor: -1.0); }
                if (i == 3) { w.AddBox(new Vector3F(1f, 1f, 1f), new Vector3D(10, 3, 10), density: 500f); }
                if (i == 10) { water.AddVolume(0, 0, 2, 20, 10f); }
                if (i == 20) { water.SetBoundary(1, WaterBoundary.Open); }
                w.Step(Dt); expected.Add(w.ComputeHash());
            }
        }

        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var copy = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = 1, WorldId = 197 });
        int step = 0;
        while (player.ReadNextStep(copy)) { Assert.Equal(expected[step++], copy.ComputeHash()); }
        Assert.Equal(expected.Count, step);
        Assert.Equal(w.Snapshot(), copy.Snapshot());
    }

    [Fact]
    public void RegionSnapshotsKeepTheWholeWater()
    {
        var w = Tank(out var water, id: 198);
        var inside = w.AddBox(new Vector3F(1f, 1f, 1f), new Vector3D(10, 3, 10), density: 500f);
        var outside = w.AddBox(new Vector3F(1f, 1f, 1f), new Vector3D(15, 3, 15), density: 500f);
        Run(w, 90);
        var min = new Vector3D(8, -2, 8); var max = new Vector3D(12, 10, 12);
        var region = w.CaptureRegion(min, max);
        Assert.Contains(region.Fields, f => f.Entity == "shallow water");
        var hash = region.ComputeHash();
        Assert.Equal(hash, w.ComputeRegionHash(min, max));
        outside.Position = new Vector3D(15, 9, 15);
        Assert.Equal(hash, w.ComputeRegionHash(min, max));
        inside.LinearVelocity = new Vector3F(0, 3, 0);
        Assert.NotEqual(hash, w.ComputeRegionHash(min, max));
        w.RestoreRegion(region.ToArray());
        Assert.Equal(hash, w.ComputeRegionHash(min, max));
        _ = inside; _ = water;
    }

    [Fact]
    public void ABoatRidesTheWavesAndStaysUpright()
    {
        var w = Tank(out var water, id: 199);
        var hull = w.CreateBody(BodyType.Dynamic, new Vector3D(6, 2.0, 10));
        w.CreateBox(hull, new Vector3F(1.2f, 0.5f, 3f), 0f, new ShapeDef { Density = 150f / (1.2f * 0.5f * 3f), LocalPosition = new Vector3F(0f, 0.25f - 150f / (1000f * 3f * 1.2f), 0f) });
        var boat = w.AddBoat(hull, BoatDef.Dinghy);
        boat.Throttle = 0f;
        double lowest = 9, highest = -9;
        for (int i = 0; i < 900; i++)
        {
            if (i % 150 == 30) { water.AddVolume(0, 0, 2, 20, 30f); }
            w.Step(Dt);
            if (i > 60) { lowest = Math.Min(lowest, hull.Position.Y); highest = Math.Max(highest, hull.Position.Y); }
            Assert.True(hull.Rotation.Rotate(new Vector3F(0, 1, 0)).Y > 0.3f, "capsized at step " + i);
            if (hull.Position.Y < 1.0) { Assert.Fail("sank at step " + i + " p=" + hull.Position + " v=" + hull.LinearVelocity); }
        }

        Assert.True(highest - lowest > 0.02, "the boat must move with the waves: " + (highest - lowest));
        Assert.InRange(hull.Position.Y, 1.5, 2.8);
    }

    [Fact]
    public void ABodyOnTheBedOfShallowWaterIsPushedByACurrent()
    {
        // A current of 1.5 m/s from an inflow side drags a light box along.
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = 1, WorldId = 200 });
        w.AddBox(new Vector3F(60f, 1f, 30f), new Vector3D(20, -0.5, 5), type: BodyType.Static);
        var water = w.CreateShallowWater(new ShallowWaterDef { CellsX = 80, CellsZ = 20, CellSize = 0.5f, Level = 1.0, FloorHeight = -1.0, West = WaterBoundary.Inflow(1.0, 1.5f), East = WaterBoundary.Open });
        var box = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(10, 1.0, 5), density: 300f);
        var sb = new System.Text.StringBuilder();
        for (int i = 0; i < 600; i++) { w.Step(Dt); if (i % 60 == 0) { water.TryGetSample(box.Position.X, 5, out var q); sb.Append("x" + box.Position.X.ToString("F2") + " vb" + box.LinearVelocity.X.ToString("F2") + " u" + q.Velocity.X.ToString("F2") + " d" + q.Depth.ToString("F2") + "; "); } }
        Assert.True(box.Position.X > 14, sb.ToString() + "the current must carry the box: x = " + box.Position.X);
        Assert.True(water.TryGetSample(5, 5, out var s));
        Assert.InRange(s.Velocity.X, 0.8f, 2.2f);
    }

    [Fact]
    public void TheGuideExampleRuns()
    {
        var world = new PhysicsWorld3D();
        world.AddBox(new Vector3F(60f, 1f, 60f), at: new Vector3D(10, -0.5, 10), type: BodyType.Static);
        var water = world.AddShallowWater(minX: 0, minZ: 0, maxX: 20, maxZ: 20, level: 2.0, cellSize: 0.5f, floor: -1.0);
        var crate = world.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(10, 3, 10), density: 500f);
        int events = 0;
        world.BodyWaterChanged += _ => events++;
        for (int i = 0; i < 600; i++)
        {
            if (i == 200) { water.AddVolume(0, 0, 2, 20, 20f); }
            world.Step(1f / 60f);
        }

        Assert.True(events >= 1, "events " + events + " crate " + crate.Position + " count " + world.ShallowWaterCount);
        Assert.True(world.QueryWater(new Vector3D(10, 0, 10), out var sample));
        Assert.True(sample.Depth > 1.9f);
        Assert.True(world.RaycastWater(new Vector3D(10, 10, 10), new Vector3F(0, -1, 0), out var hit));
        Assert.InRange(hit.Distance, 7f, 8.5f);
        Assert.InRange(crate.Position.Y, 1.8, 2.3);
    }

    [Fact]
    public void SteadyStateStepsAllocateNothing()
    {
        var w = Tank(out var water, id: 201);
        w.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(10, 3, 10), density: 500f);
        w.AddSphere(0.4f, at: new Vector3D(6, 3, 6), density: 300f);
        w.Warmup();
        Run(w, 200);
        water.AddVolume(0, 0, 2, 20, 10f);
        Run(w, 20);
        Core.AllocMeter.AssertAllocatesNothing(() => Run(w, 120), "steady steps of a tank");
    }
}
