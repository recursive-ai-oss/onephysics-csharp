using System;
using System.Linq;
using RecursiveAI.OneMath;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class CarvableTerrain2DTests
{
    private const int W = 128, H = 64, Top = 32; // 0.25 m cells: the surface is at y = 8
    private static bool[] Floor() { var m = new bool[W * H]; for (int y = 0; y < Top; y++) for (int x = 0; x < W; x++) m[y * W + x] = true; return m; }
    private static P2.PhysicsWorld2D World(out P2.Body terrain)
    {
        var w = new P2.PhysicsWorld2D(new() { SleepEnabled = true });
        terrain = w.AddBitmaskTerrain(W, H, Floor(), 0.25f);
        return w;
    }

    [Fact]
    public void FlatContourFloorIsOneSegmentAndRestsABox()
    {
        var w = World(out var terrain); Assert.True(w.GetTileGrid(terrain).IsContour);
        var box = w.AddBox(new(1, 1), new(10, 8.5));
        for (int i = 0; i < 180; i++) w.Step(1f / 60);
        Assert.InRange(box.Position.Y, 8.49, 8.51);
    }

    [Fact]
    public void CarvingAHoleUnderARestingBoxDropsIt()
    {
        var w = World(out var terrain); var box = w.AddBox(new(1, 1), new(10, 8.5));
        for (int i = 0; i < 300; i++) w.Step(1f / 60);
        Assert.False(box.IsAwake); // asleep on the floor
        int changed = w.CarveCircle(terrain, new(10, 8), 2.5);
        Assert.True(changed > 100); Assert.True(box.IsAwake);
        for (int i = 0; i < 120; i++) w.Step(1f / 60);
        Assert.True(box.Position.Y < 7.0, "box fell into the hole: " + box.Position.Y);
        // Refilling the hole restores the original boundary.
        w.FillPolygon(terrain, [new(7, 5), new(13, 5), new(13, 8), new(7, 8)]);
        var fresh = new P2.PhysicsWorld2D(); var t2 = fresh.AddBitmaskTerrain(W, H, Floor(), 0.25f);
        Assert.True(w.GetTileGrid(terrain).SegmentCount >= 1);
        Assert.Equal(fresh.GetTileGrid(t2).SegmentCount, w.GetTileGrid(terrain).SegmentCount);
    }

    [Fact]
    public void BoxBesideACarveKeepsItsMotion()
    {
        // Two separate pieces of ground: an edit of the left piece leaves the right piece's segments (and their
        // handles) alone, so a box resting on it moves bit for bit as it does without the edit.
        (double X, double Y, uint Vx) Run(bool carve)
        {
            var m = new bool[W * H]; for (int y = 0; y < Top; y++) for (int x = 0; x < W; x++) if (x < 50 || x > 60) m[y * W + x] = true;
            var w = new P2.PhysicsWorld2D(new() { SleepEnabled = true }); var terrain = w.AddBitmaskTerrain(W, H, m, 0.25f);
            var box = w.AddBox(new(1, 1), new(24, 8.5)); box.LinearVelocity = new(1, 0);
            for (int i = 0; i < 60; i++) w.Step(1f / 60);
            if (carve) Assert.True(w.CarveCircle(terrain, new(8, 8), 2) > 0);
            for (int i = 0; i < 120; i++) w.Step(1f / 60);
            return (box.Position.X, box.Position.Y, BitConverter.SingleToUInt32Bits(box.LinearVelocity.X));
        }
        Assert.Equal(Run(false), Run(true));
    }

    [Fact]
    public void BoxOnTheSameFlatSegmentAsACarveMovesTheSameWithinRounding()
    {
        // Carving shortens the top segment the box rests on, so it is a new shape in a different slot; contacts are
        // ordered by slot, so the arithmetic order can differ and the motion agrees to rounding, not to the bit.
        (double X, double Y) Run(bool carve)
        {
            var w = World(out var terrain); var box = w.AddBox(new(1, 1), new(30, 8.5)); box.LinearVelocity = new(1, 0);
            for (int i = 0; i < 60; i++) w.Step(1f / 60);
            if (carve) w.CarveCircle(terrain, new(8, 8), 2);
            for (int i = 0; i < 120; i++) w.Step(1f / 60);
            return (box.Position.X, box.Position.Y);
        }
        var a = Run(false); var b = Run(true);
        Assert.InRange(Math.Abs(a.X - b.X), 0, 1e-5); Assert.InRange(Math.Abs(a.Y - b.Y), 0, 1e-5);
    }

    [Fact]
    public void SlidingAcrossACarvedAndRefilledSeamHasNoVerticalKicks()
    {
        var w = new P2.PhysicsWorld2D(new() { SleepEnabled = false });
        var terrain = w.AddBitmaskTerrain(W, H, Floor(), 0.25f, material: new P2.Material2D(0));
        var box = w.AddBox(new(0.8f, 1), new(2, 8.5), material: new P2.Material2D(0));
        for (int i = 0; i < 90; i++) w.Step(1f / 60);
        // Carve a crater, then refill it in two overlapping pieces.
        w.CarveCircle(terrain, new(16, 8), 3);
        w.FillPolygon(terrain, [new(12, 4), new(16.25, 4), new(16.25, 8), new(12, 8)]);
        w.FillPolygon(terrain, [new(15.5, 4), new(20, 4), new(20, 8), new(15.5, 8)]);
        box.LinearVelocity = new(6, 0); float max = 0;
        for (int i = 0; i < 200; i++) { w.Step(1f / 60); max = Math.Max(max, Math.Abs(box.LinearVelocity.Y)); }
        Assert.True(max < 1e-4f, "max |Vy| " + max);
        Assert.True(box.Position.X > 20);
    }

    [Fact]
    public void DetachedIslandBecomesABodyWithTheRightAreaAndMass()
    {
        var mask = Floor();
        // A 3 x 2 cell pillar connected to the floor by a one-cell neck, two cells above it.
        for (int y = Top; y < Top + 3; y++) mask[y * W + 40] = true; // neck, 3 cells tall
        for (int y = Top + 3; y < Top + 5; y++) for (int x = 39; x <= 41; x++) mask[y * W + x] = true; // 6-cell head
        var w = new P2.PhysicsWorld2D(); var terrain = w.AddBitmaskTerrain(W, H, mask, 0.25f);
        Assert.Empty(w.DetachIslands(terrain)); // connected: nothing detaches
        // Carve the neck at its base.
        int changed = w.StampMask(terrain, 40, Top, 1, 1, [true], false);
        Assert.Equal(1, changed);
        var bodies = w.DetachIslands(terrain);
        Assert.Single(bodies);
        float area = 8 * 0.25f * 0.25f; // 2 neck cells + 6 head cells
        Assert.InRange(bodies[0].Mass, area * 0.999f, area * 1.001f);
        Assert.Equal(0, w.GetTileGrid(terrain).Tiles.ToArray().Count(t => t != TileKind2D.Empty) - (W * Top - 0));
        for (int i = 0; i < 120; i++) w.Step(1f / 60);
        Assert.InRange(bodies[0].Position.Y, 8.3, 9.2);
    }

    [Fact]
    public void BitmaskTerrainSnapshotForkAndReplayAreExact()
    {
        var w = World(out var terrain); var box = w.AddBox(new(1, 1), new(10, 8.5));
        for (int i = 0; i < 30; i++) w.Step(1f / 60);
        w.CarveCircle(terrain, new(10, 8), 1.5);
        var bytes = w.Snapshot();
        var other = new P2.PhysicsWorld2D(); other.Restore(bytes);
        Assert.Equal(bytes, other.Snapshot());
        var fork = w.Fork(); 
        for (int i = 0; i < 60; i++) { w.Step(1f / 60); other.Step(1f / 60); fork.Step(1f / 60); Assert.Equal(w.ComputeHash(), other.ComputeHash()); Assert.Equal(w.ComputeHash(), fork.ComputeHash()); }
        bool found = false; foreach (var b in other.Bodies) if (b.Type == BodyType.Static) { Assert.True(other.GetTileGrid(b).IsContour); found = true; }
        Assert.True(found);
    }
}
