#pragma warning disable OPHYS0002, OPHYS0003
using System;
using System.IO;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Replay;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.WorldScale;

public sealed class StreamingTests
{
    private static P3.PhysicsWorld3D World3D(int id = 1380, int threads = 1) => new(new() { WorldId = id, ThreadCount = threads });

    [Fact]
    public void LoadOrderWithinAStepDoesNotMatter()
    {
        var keys = new[] { new TileKey(2, 1), new TileKey(0, 0), new TileKey(1, 1), new TileKey(1, 0), new TileKey(0, 1) };
        ulong Run(int[] order, bool flushEach)
        {
            var w = World3D();
            foreach (int i in order)
            {
                w.LoadTile(keys[i], WorldScaleScenes.Tile3D(keys[i].X, keys[i].Y, true));
                if (flushEach) { w.FlushTiles(); }
            }

            var ball = w.AddSphere(0.4f, new Vector3D(10, WorldScaleScenes.Height(10, 20) + 3, 20)); ball.LinearVelocity = new Vector3F(3, 0, 1);
            ulong h = 0;
            for (int s = 0; s < 90; s++) { w.Step(WorldScaleScenes.Dt); h = h * 1099511628211UL ^ w.ComputeHash(); }
            return h;
        }

        ulong sorted = Run(Enumerable.Range(0, keys.Length).OrderBy(i => keys[i]).ToArray(), true);
        Assert.Equal(sorted, Run([0, 1, 2, 3, 4], false));
        Assert.Equal(sorted, Run([4, 2, 0, 3, 1], false));
        Assert.Equal(sorted, Run([3, 0, 4, 1, 2], false));
    }

    [Fact]
    public void TileStateIsQueuedLoadedAndUnloaded()
    {
        var w = World3D(1381);
        var k = new TileKey(3, 4);
        Assert.False(w.IsTileLoaded(k));
        w.LoadTile(k, WorldScaleScenes.Tile3D(3, 4, true));
        Assert.Equal(1, w.PendingTileRequests);
        Assert.False(w.IsTileLoaded(k));
        Assert.Throws<InvalidOperationException>(() => w.LoadTile(k, WorldScaleScenes.Tile3D(3, 4, true)));
        w.Step(WorldScaleScenes.Dt);
        Assert.True(w.IsTileLoaded(k));
        Assert.Equal(new[] { k }, w.GetLoadedTiles());
        Assert.Single(w.GetTileBodies(k));
        Assert.Throws<InvalidOperationException>(() => w.LoadTile(k, WorldScaleScenes.Tile3D(3, 4, true)));
        Assert.True(w.UnloadTile(k));
        w.Step(WorldScaleScenes.Dt);
        Assert.False(w.IsTileLoaded(k));
        Assert.Equal(0, w.TileCount);
        Assert.Equal(0, w.BodyCount);
        Assert.False(w.UnloadTile(k));
        // a load that is cancelled before it happens never builds
        w.LoadTile(k, WorldScaleScenes.Tile3D(3, 4, true));
        Assert.True(w.UnloadTile(k));
        w.Step(WorldScaleScenes.Dt);
        Assert.Equal(0, w.BodyCount);
    }

    [Fact]
    public void UnloadWaitsForTheBodyThatRestsOnTheTile()
    {
        var w = World3D(1382);
        float Flat(int x, int z) => 0f;
        w.LoadTile(new TileKey(0, 0), WorldScaleScenes.Tile3D(0, 0, true, 32, Flat));
        var ball = w.AddSphere(0.4f, new Vector3D(16, 0.41, 16));
        for (int s = 0; s < 30; s++) { w.Step(WorldScaleScenes.Dt); }
        w.UnloadTile(new TileKey(0, 0));
        for (int s = 0; s < 60; s++) { w.Step(WorldScaleScenes.Dt); }
        Assert.Equal(TileState.Unloading, w.GetTileState(new TileKey(0, 0)));
        Assert.InRange(ball.Position.Y, 0.35, 0.45);
        // the body leaves (is moved away): the tile goes
        ball.Position = new Vector3D(16, 20, 16);
        for (int s = 0; s < 5; s++) { w.Step(WorldScaleScenes.Dt); }
        Assert.Equal(TileState.Absent, w.GetTileState(new TileKey(0, 0)));

        var w2 = World3D(1383);
        w2.LoadTile(new TileKey(0, 0), WorldScaleScenes.Tile3D(0, 0, true, 32, Flat));
        var ball2 = w2.AddSphere(0.4f, new Vector3D(16, 0.41, 16));
        for (int s = 0; s < 30; s++) { w2.Step(WorldScaleScenes.Dt); }
        w2.UnloadTile(new TileKey(0, 0), TileUnloadPolicy.Immediate);
        for (int s = 0; s < 90; s++) { w2.Step(WorldScaleScenes.Dt); }
        Assert.False(w2.IsTileLoaded(new TileKey(0, 0)));
        Assert.True(ball2.Position.Y < -1, "without its tile the ball falls");
    }

    [Fact]
    public void LoadedTilesSurviveSnapshotRestoreForkSceneAndAnUnloadInProgress()
    {
        var w = World3D(1384);
        for (int tx = 0; tx < 3; tx++) { w.LoadTile(new TileKey(tx, 0), WorldScaleScenes.Tile3D(tx, 0, true)); }
        w.SetSimulationLod([new Vector3D(10, 0, 10)], [new LodBand(20, 1)], 2);
        var ball = w.AddSphere(0.4f, new Vector3D(40, WorldScaleScenes.Height(40, 16) + 1, 16));
        w.Step(WorldScaleScenes.Dt);
        for (int s = 0; s < 40; s++) { w.Step(WorldScaleScenes.Dt); }
        w.UnloadTile(new TileKey(1, 0));
        for (int s = 0; s < 3; s++) { w.Step(WorldScaleScenes.Dt); }
        var snap = w.Snapshot();
        var copy = World3D(1384); copy.Restore(snap);
        Assert.Equal(snap, copy.Snapshot());
        Assert.Equal(w.GetLoadedTiles(), copy.GetLoadedTiles());
        Assert.Equal(snap, w.Fork().Snapshot());
        Assert.Equal(snap, P3.PhysicsWorld3D.LoadScene(new StringReader(w.SaveScene())).Snapshot());
        for (int s = 0; s < 120; s++) { w.Step(WorldScaleScenes.Dt); copy.Step(WorldScaleScenes.Dt); Assert.Equal(w.ComputeHash(), copy.ComputeHash()); }
        // tiles can still be loaded and unloaded in the restored world
        copy.UnloadTile(new TileKey(0, 0), TileUnloadPolicy.Immediate); copy.Step(WorldScaleScenes.Dt);
        Assert.DoesNotContain(new TileKey(0, 0), copy.GetLoadedTiles());
        // a restore forgets queued requests
        w.LoadTile(new TileKey(9, 9), WorldScaleScenes.Tile3D(9, 9, false));
        w.Restore(snap);
        Assert.Equal(0, w.PendingTileRequests);
    }

    [Fact]
    public void QueuedTilesFollowAnOriginShiftAndBuildAtTheRightPlace()
    {
        var w = World3D(1385);
        var def = WorldScaleScenes.Tile3D(1, 0, false);
        w.LoadTile(new TileKey(1, 0), def);
        w.ShiftOrigin(new Vector3D(1024, 0, 0));
        w.Step(WorldScaleScenes.Dt);
        var tile = w.GetTileBodies(new TileKey(1, 0))[0];
        Assert.Equal(32.0 - 1024.0, tile.Position.X);
        Assert.Equal(32.0, tile.Position.X + w.Core.Origin[0]);
        // a ball dropped at the same host position lands on it before and after shifting
        var ball = w.AddSphere(0.4f, new Vector3D(40 - 1024, WorldScaleScenes.Height(40, 16) + 2, 16));
        for (int s = 0; s < 120; s++) { w.Step(WorldScaleScenes.Dt); }
        Assert.True(ball.Position.Y > WorldScaleScenes.Height(40, 16) - 0.5);
    }

    [Fact]
    public void ReplayOfTileLoadsReproducesEveryHash()
    {
        var w = World3D(1386);
        var hashes = new ulong[40];
        using var stream = new MemoryStream();
        using (var rec = w.StartRecording(stream))
        {
            for (int i = 0; i < hashes.Length; i++)
            {
                if (i == 0) { w.LoadTile(new TileKey(1, 0), WorldScaleScenes.Tile3D(1, 0, true)); w.LoadTile(new TileKey(0, 0), WorldScaleScenes.Tile3D(0, 0, true)); w.AddSphere(0.4f, new Vector3D(20, WorldScaleScenes.Height(20, 16) + 2, 16)); }
                if (i == 20) { w.UnloadTile(new TileKey(0, 0), TileUnloadPolicy.Immediate); }
                if (i == 5) { w.SetSimulationLod([new Vector3D(0, 0, 0)], [new LodBand(10, 1)], 4); }
                w.Step(WorldScaleScenes.Dt); hashes[i] = w.ComputeHash();
            }
        }

        stream.Position = 0;
        var player = new ReplayPlayer(stream); var copy = player.CreateWorld3D();
        int n = 0;
        while (player.ReadNextStep(copy)) { Assert.Equal(hashes[n++], copy.ComputeHash()); }
        Assert.Equal(hashes.Length, n);
        Assert.Equal(w.Snapshot(), copy.Snapshot());
    }

    [Fact]
    public void RegionSnapshotsLeaveTheTileRegistryAndTheLodSettingsAlone()
    {
        var w = World3D(1387);
        w.LoadTile(new TileKey(0, 0), WorldScaleScenes.Tile3D(0, 0, true));
        var ball = w.AddSphere(0.4f, new Vector3D(10, WorldScaleScenes.Height(10, 10) + 1, 10));
        ball.TimeScale = 0.5f;
        w.SetSimulationLod([new Vector3D(0, 0, 0)], [new LodBand(3, 1)], 2);
        for (int s = 0; s < 20; s++) { w.Step(WorldScaleScenes.Dt); }
        var min = new Vector3D(0, -100, 0); var max = new Vector3D(20, 100, 20);
        var region = w.SnapshotRegion(min, max);
        var other = w.Fork();
        // the other world's focus point has moved; focus points, bands and the tile registry are the world's own, not the region's
        other.SetLodFocus([new Vector3D(5, 5, 5)]);
        other.GetBody(ball.RawHandle).Position = new Vector3D(11, WorldScaleScenes.Height(11, 10) + 3, 10);
        other.RestoreRegion(region);
        Assert.Equal(w.ComputeRegionHash(min, max), other.ComputeRegionHash(min, max));
        Assert.Equal(new[] { new TileKey(0, 0) }, other.GetLoadedTiles());
        Assert.Equal(0.5f, other.GetBody(ball.RawHandle).TimeScale);
    }

    [Fact]
    public void DestroyingATileBodyDirectlyUpdatesTheRegistry()
    {
        var w = World3D(1388);
        var def = WorldScaleScenes.Tile3D(0, 0, true);
        def.Boxes.Add(new P3.TileBox3D { Size = new Vector3F(1, 1, 1), Offset = new Vector3F(5, 5, 5) });
        w.LoadTile(new TileKey(0, 0), def);
        w.FlushTiles();
        var bodies = w.GetTileBodies(new TileKey(0, 0));
        Assert.Equal(2, bodies.Length);
        bodies[1].Destroy();
        Assert.Single(w.GetTileBodies(new TileKey(0, 0)));
        bodies[0].Destroy();
        Assert.Equal(0, w.TileCount);
    }

    [Fact]
    public void InvalidTilesAreRejectedOrDroppedWithoutHurtingTheWorld()
    {
        var w = World3D(1389);
        Assert.Throws<ArgumentNullException>(() => w.LoadTile(new TileKey(0, 0), null!));
        var bad = new P3.TileDef3D(); bad.HeightFields.Add(new P3.TileHeightField3D { Heights = new float[3], SamplesX = 2, SamplesZ = 2 });
        Assert.Throws<ArgumentException>(() => w.LoadTile(new TileKey(0, 0), bad));
        var nan = WorldScaleScenes.Tile3D(0, 0, false); nan.HeightFields[0].Heights[5] = float.NaN;
        w.LoadTile(new TileKey(0, 0), nan);
        var huge = new P3.TileDef3D { Origin = new Vector3D(1e12, 0, 0) }; huge.Boxes.Add(new P3.TileBox3D());
        w.LoadTile(new TileKey(1, 0), huge);
        w.LoadTile(new TileKey(2, 0), WorldScaleScenes.Tile3D(2, 0, false));
        w.Step(WorldScaleScenes.Dt);
        Assert.Equal(2, w.FailedTileLoads);
        Assert.NotNull(w.LastTileError);
        Assert.Equal(new[] { new TileKey(2, 0) }, w.GetLoadedTiles());
        Assert.Equal(1, w.BodyCount);
        var edge = WorldScaleScenes.Tile3D(3, 0, true); edge.HeightFields[0].Edges!.West = new float[2];
        w.LoadTile(new TileKey(3, 0), edge);
        w.Step(WorldScaleScenes.Dt);
        Assert.Equal(3, w.FailedTileLoads);
        Assert.Throws<ArgumentOutOfRangeException>(() => w.UnloadTile(new TileKey(2, 0), (TileUnloadPolicy)9));
    }

    [Fact]
    public void PlanarTilesStreamAndKeepTheirGhostsOut()
    {
        float Plane(int x) => -0.12f * x;
        double Roll(bool edges)
        {
            var w = new P2.PhysicsWorld2D(new() { WorldId = 1390 });
            for (int tx = 0; tx < 4; tx++) { w.LoadTile(new TileKey(tx, 0), WorldScaleScenes.Tile2D(tx, edges, 32, Plane)); }
            var ball = w.AddCircle(0.4f, new Vector2D(2, Plane(2) + 0.5)); ball.LinearVelocity = new Vector2F(4, 0);
            double worst = 0, last = 0;
            for (int s = 0; s < 3000; s++)
            {
                w.Step(WorldScaleScenes.Dt);
                double vy = ball.LinearVelocity.Y, x = ball.Position.X;
                if (s > 30 && x > 3 && x < 125 && Math.Abs(x - Math.Round(x / 32) * 32) < 0.5) { worst = Math.Max(worst, Math.Abs(vy - last)); }
                last = vy;
                if (x > 126) { break; }
            }

            return worst;
        }

        double with = Roll(true), without = Roll(false);
        Assert.True(with < 0.02, $"2D seam with ghosts {with}");
        Assert.True(without > 0.05, $"2D seam without ghosts {without}");
        var w2 = new P2.PhysicsWorld2D(new() { WorldId = 1391 });
        w2.LoadTile(new TileKey(5, 0), WorldScaleScenes.Tile2D(5, true));
        w2.Step(WorldScaleScenes.Dt);
        Assert.True(w2.IsTileLoaded(new TileKey(5, 0)));
        var snap = w2.Snapshot(); var c2 = new P2.PhysicsWorld2D(new() { WorldId = 1391 }); c2.Restore(snap);
        Assert.Equal(snap, c2.Snapshot()); Assert.Equal(snap, w2.Fork().Snapshot());
    }
}

public sealed class StreamingSceneTests
{
    [Fact]
    public void StreamingSceneHoldsTogetherAndSeamsAreNoWorseThanOneField()
    {
        var tiled = WorldScaleScenes.BuildStreaming3D(1);
        var whole = WorldScaleScenes.BuildStreaming3D(1, 1401, true);
        for (int i = 0; i < 1200; i++) { tiled.Step(i); whole.Step(i); }
        string report = $"tiled: loads {tiled.Loads} unloads {tiled.Unloads} maxLoaded {tiled.MaxLoaded} maxSpeed {tiled.MaxSpeed:F2} tunnelled {tiled.Tunnelled} seam dvy {tiled.SeamStep:F3}; unsplit: maxSpeed {whole.MaxSpeed:F2} tunnelled {whole.Tunnelled} seam dvy {whole.SeamStep:F3}";
        Assert.True(tiled.Tunnelled == 0 && tiled.NonFinite == 0 && tiled.World.FailedTileLoads == 0 && tiled.Loads > 25, report);
        Assert.True(tiled.SeamStep <= whole.SeamStep * 1.25 + 0.1, report);
        Assert.True(tiled.MaxLoaded <= 30, report);
    }

    [Fact]
    public void StreamingSceneHashesAreEqualOnAnyThreadCountAndThroughARestore()
    {
        ulong a = WorldScaleScenes.RunStreaming3D(1, false, null, 200);
        Assert.Equal(a, WorldScaleScenes.RunStreaming3D(1, true, null, 200));
        Assert.Equal(a, WorldScaleScenes.RunStreaming3D(8, false, null, 200));
    }
}
