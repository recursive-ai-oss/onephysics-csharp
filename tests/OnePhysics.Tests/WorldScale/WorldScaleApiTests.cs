using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Debug;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.WorldScale;

public sealed class WorldScaleApiTests
{
    [Fact]
    public void HostileArgumentsAreRefusedWithNamedErrors()
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = 1450 });
        Assert.Throws<ArgumentOutOfRangeException>(() => w.SetSimulationLod([new Vector3D(0, 0, 0)], [new LodBand(10, 3)]));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.SetSimulationLod([new Vector3D(0, 0, 0)], [new LodBand(float.NaN, 2)]));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.SetSimulationLod([new Vector3D(0, 0, 0)], [new LodBand(10, 2), new LodBand(5, 4)], 8));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.SetSimulationLod([new Vector3D(double.PositiveInfinity, 0, 0)], [new LodBand(10, 2)]));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.SetSimulationLod(default, default, 5));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.SetSimulationLod(default, default, 2, default, 9));
        Assert.Throws<ArgumentException>(() => w.SetSimulationLod(default, default, 2, [new P3.LodRegion3D(new(1, 1, 1), new(0, 0, 0), 2)]));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.SetTimeScaleRegions([new P3.TimeScaleRegion3D(new(0, 0, 0), new(1, 1, 1), 0f)]));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.SetTimeScaleRegions([new P3.TimeScaleRegion3D(new(0, 0, 0), new(1, 1, 1), float.NaN)]));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.SetTimeScaleRegions([new P3.TimeScaleRegion3D(new(0, 0, 0), new(1, 1, 1), 4.5f)]));
        var wall = w.AddBox(new Vector3F(1, 1, 1), type: BodyType.Static);
        Assert.Throws<BodyTypeException>(() => wall.TimeScale = 0.5f);
        var ball = w.AddSphere(0.5f);
        ball.Destroy();
        Assert.Throws<StaleHandleException>(() => ball.TimeScale);
        Assert.Throws<StaleHandleException>(() => ball.TimeScale = 0.5f);
        // nothing above changed the world
        Assert.False(w.Core.ScaleStore.IsPresent);
    }

    [Fact]
    public void ARecycledBodySlotDoesNotInheritATimeScale()
    {
        var w = new P2.PhysicsWorld2D(new() { WorldId = 1451 });
        var a = w.AddCircle(0.5f); a.TimeScale = 0.25f;
        a.Destroy();
        var b = w.AddCircle(0.5f);
        Assert.Equal(1f, b.TimeScale);
    }

    [Fact]
    public void ConfigurationFollowsAnOriginShiftAndKeepsTheSameIslandsSkipped()
    {
        P3.PhysicsWorld3D Make(bool shift)
        {
            var w = new P3.PhysicsWorld3D(new() { WorldId = 1452 });
            w.AddBox(new Vector3F(60, 1, 20), new Vector3D(20, -0.5, 0), type: BodyType.Static);
            for (int i = 0; i < 8; i++) { w.AddBox(new Vector3F(1, 1, 1), new Vector3D(i * 5, 0.5, 0)); }
            w.SetSimulationLod(default, default, 1, [new P3.LodRegion3D(new(18, -1, -5), new(60, 20, 5), 4)]);
            if (shift) { w.ShiftOrigin(new Vector3D(1024, 0, 0)); }
            return w;
        }

        var a = Make(false); var b = Make(true);
        for (int s = 0; s < 40; s++)
        {
            a.Step(1f / 60); b.Step(1f / 60);
            Assert.Equal(a.Counters.LodIslandsSkipped, b.Counters.LodIslandsSkipped);
        }

        Assert.True(a.Counters.LodIslandsSkipped > 0 || a.Counters.AwakeIslands == 0);
        var g = new DebugGeometry();
        b.GetDebugGeometry(DebugContent.WorldScale, g);
        Assert.Equal(1, g.ScaleRegions.Length);
        Assert.Equal(18.0 - 1024.0, g.ScaleRegions[0].Minimum.X, 9);
        Assert.Equal(4, g.ScaleRegions[0].Period);
    }

    [Fact]
    public void DebugGeometryListsTilesRegionsAndFocus()
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = 1453 });
        w.LoadTile(new TileKey(1, 2, 0), WorldScaleScenes.Tile3D(1, 2, true));
        w.LoadTile(new TileKey(0, 0, 0), WorldScaleScenes.Tile3D(0, 0, true));
        w.Step(1f / 60);
        w.SetSimulationLod([new Vector3D(1, 2, 3)], [new LodBand(5, 2)], 8, [new P3.LodRegion3D(new(0, 0, 0), new(1, 1, 1), 2)]);
        w.SetTimeScaleRegions([new P3.TimeScaleRegion3D(new(2, 2, 2), new(3, 3, 3), 0.5f)]);
        var g = new DebugGeometry();
        w.GetDebugGeometry(DebugContent.WorldScale, g);
        Assert.Equal(2, g.Tiles.Length);
        Assert.Equal(new TileKey(0, 0, 0), g.Tiles[0].Key);
        Assert.Equal(new TileKey(1, 2, 0), g.Tiles[1].Key);
        Assert.Equal(2, g.ScaleRegions.Length);
        Assert.Equal(DebugScaleRegionKind.TimeScale, g.ScaleRegions[1].Kind);
        Assert.Equal(1, g.LodFocusPoints.Length);
        w.GetDebugGeometry(DebugContent.Shapes, g);
        Assert.Equal(0, g.Tiles.Length);
    }

    [Fact]
    public void SteadyStateStepsWithLevelOfDetailTimeScaleAndTilesAllocateNothing()
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = 1454, ThreadCount = 1, ContactCapacity = 4096, IslandCapacity = 256, PairCapacity = 4096 });
        w.LoadTile(new TileKey(0, 0), WorldScaleScenes.Tile3D(0, 0, true));
        for (int i = 0; i < 40; i++) { var b = w.AddBox(new Vector3F(1, 1, 1), new Vector3D(2 + (i % 8) * 1.5, WorldScaleScenes.Height(4, 10) + 3 + (i / 8) * 1.2, 10 + (i % 3))); if (i % 4 == 0) { b.TimeScale = 0.5f; b.NeverSleep = true; } }
        w.SetSimulationLod([new Vector3D(0, 0, 0)], [new LodBand(8, 1), new LodBand(16, 2)], 4);
        w.SetTimeScaleRegions([new P3.TimeScaleRegion3D(new(10, -100, 0), new(20, 100, 20), 0.25f)]);
        // The pile lands and settles over the first few hundred steps, and the world's internal buffers grow to fit that (the last growth, 54 KB, comes at step 299).
        // Steady state is what follows, so the window is judged after it.
        for (int s = 0; s < 450; s++) { w.Step(1f / 60); }
        Core.AllocMeter.AssertAllocatesNothing(() => { for (int s = 0; s < 60; s++) { w.Step(1f / 60); } }, "steady steps with level of detail, time scale and tiles");
    }
}
