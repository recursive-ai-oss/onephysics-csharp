using System;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Replay;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class TerrainHostHookTests
{
    private static P2.PhysicsWorld2D CarvedWorld(int id)
    {
        var w = new P2.PhysicsWorld2D(new() { WorldId = id }); var mask = new bool[32 * 16]; for (int y = 0; y < 8; y++) for (int x = 0; x < 32; x++) mask[y * 32 + x] = true;
        var t = w.AddBitmaskTerrain(32, 16, mask, 0.5f); w.AddBox(new(0.8f, 0.8f), new(8, 6));
        for (int i = 0; i < 20; i++) w.Step(1f / 60);
        t.CarveCircle(new(8, 4), 1.2); w.Step(1f / 60); t.FillCircle(new(12, 4), 0.8); w.Step(1f / 60);
        return w;
    }

    private static P3.PhysicsWorld3D EditedWorld(int id)
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = id }); var occ = new bool[16 * 8 * 16]; for (int z = 0; z < 16; z++) for (int y = 0; y < 4; y++) for (int x = 0; x < 16; x++) occ[(z * 8 + y) * 16 + x] = true;
        var g = w.AddVoxelGrid(16, 8, 16, occ, 0.5f); w.AddBox(new(0.8f, 0.8f, 0.8f), new(4, 3, 4));
        w.AddSdf(SdfGrid3D.FromFunction(new(-2, -2, -2), new(2, 2, 2), 0.5f, SdfGrid3D.Sphere(default, 1f)), new Vector3D(20, 1, 20));
        w.AddSdf(SdfGrid3D.FromFunction(new(-1, -1, -1), new(1, 1, 1), 0.5f, SdfGrid3D.Box(default, new(0.4f, 0.4f, 0.4f))), new Vector3D(20, 4, 20), type: BodyType.Dynamic);
        for (int i = 0; i < 20; i++) w.Step(1f / 60);
        g.Voxels.DigSphere(new(4, 2, 4), 1.2); w.Step(1f / 60); g.Voxels.SetVoxels(2, 0, 2, 2, 4, 2, true); w.Step(1f / 60);
        return w;
    }

    [Fact]
    public void EditedAndFieldWorldsRoundTripThroughScenesAndRegions()
    {
        var a = CarvedWorld(900);
        Assert.Equal(a.Snapshot(), P2.PhysicsWorld2D.LoadScene(new StringReader(a.SaveScene())).Snapshot());
        var b = EditedWorld(901);
        Assert.Equal(b.Snapshot(), P3.PhysicsWorld3D.LoadScene(new StringReader(b.SaveScene())).Snapshot());
        var hash = a.ComputeRegionHash(new(-1, -1), new(40, 20)); a.RestoreRegion(a.CaptureRegion(new(-1, -1), new(40, 20))); Assert.Equal(hash, a.ComputeRegionHash(new(-1, -1), new(40, 20)));
        var rh = b.ComputeRegionHash(new(-1, -1, -1), new(40, 20, 40)); b.RestoreRegion(b.CaptureRegion(new(-1, -1, -1), new(40, 20, 40))); Assert.Equal(rh, b.ComputeRegionHash(new(-1, -1, -1), new(40, 20, 40)));
    }

    [Fact]
    public void EditsReplayAndDebugGeometryDrawsThem()
    {
        using var stream = new MemoryStream(); var w = new P2.PhysicsWorld2D(new() { WorldId = 902 });
        using (var recorder = w.StartRecording(stream))
        {
            var mask = new bool[16 * 8]; for (int i = 0; i < 16 * 4; i++) mask[i] = true;
            var t = w.AddBitmaskTerrain(16, 8, mask, 0.5f); w.AddBox(new(0.8f, 0.8f), new(4, 4)); w.Step(1f / 60);
            t.CarveCircle(new(4, 2), 1.0); w.Step(1f / 60); w.StampMask(t, 8, 0, 2, 2, [true, false, true, true], true); w.Step(1f / 60); t.DetachIslands(); w.Step(1f / 60);
        }

        stream.Position = 0; var player = new ReplayPlayer(stream); var replay = new P2.PhysicsWorld2D(new() { WorldId = 902 });
        int steps = 0; while (player.ReadNextStep(replay)) { steps++; Assert.Equal(replay.ComputeHash(), replay.ComputeHash()); }
        Assert.Equal(4, steps); Assert.Equal(w.Snapshot(), replay.Snapshot());

        using var stream3 = new MemoryStream(); var w3 = new P3.PhysicsWorld3D(new() { WorldId = 903 });
        using (var recorder = w3.StartRecording(stream3))
        {
            var occ = new bool[8 * 8 * 8]; for (int i = 0; i < 8 * 8 * 4; i++) occ[i] = true;
            var g = w3.AddVoxelGrid(8, 8, 8, occ, 0.5f); w3.AddSdf(SdfGrid3D.FromFunction(new(-1, -1, -1), new(1, 1, 1), 0.5f, SdfGrid3D.Sphere(default, 0.6f)), new Vector3D(10, 1, 10)); w3.Step(1f / 60);
            g.Voxels.DigSphere(new(2, 1, 2), 0.8); w3.Step(1f / 60); g.Voxels.StampBox(new(1, 1, 1), new(0.4f, 0.4f, 0.4f), default, true); w3.Step(1f / 60);
        }

        stream3.Position = 0; var player3 = new ReplayPlayer(stream3); var replay3 = new P3.PhysicsWorld3D(new() { WorldId = 903 });
        while (player3.ReadNextStep(replay3)) { }
        Assert.Equal(w3.Snapshot(), replay3.Snapshot());

        var debug = new DebugGeometry(); w.GetDebugGeometry(DebugContent.All, debug); Assert.True(debug.Shapes.Length > 0);
        var debug3 = new DebugGeometry(); w3.GetDebugGeometry(DebugContent.All, debug3); Assert.True(debug3.Shapes.Length > 0);
    }

    [Fact]
    public void OriginShiftKeepsTerrainEditsAndCarvesWorkAtTheirNewCoordinates()
    {
        var w = new P2.PhysicsWorld2D(new() { WorldId = 904 }); var mask = new bool[16 * 8]; for (int i = 0; i < 16 * 4; i++) mask[i] = true;
        var t = w.AddBitmaskTerrain(16, 8, mask, 0.5f, new Vector2D(100, 50));
        Assert.True(t.CarveCircle(new Vector2D(104, 52), 1.0) > 0);
        w.ShiftOrigin(new Vector2D(64, 32));
        Assert.Equal(0, t.CarveCircle(new Vector2D(104, 52), 1.0)); // that place is already carved; its position changed with the shift, so nothing at the old coordinates
        Assert.True(t.CarveCircle(new Vector2D(40, 19), 0.8) > 0); // the same ground, at its coordinates after the shift
    }
}
