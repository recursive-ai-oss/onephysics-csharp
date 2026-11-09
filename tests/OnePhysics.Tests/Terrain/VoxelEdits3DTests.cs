using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class VoxelEdits3DTests
{
    private const int N = 40, H = 8, Top = 4; // 0.5 m cells: the surface is at y = 2
    private static bool[] Floor() { var m = new bool[N * H * N]; for (int z = 0; z < N; z++) for (int y = 0; y < Top; y++) for (int x = 0; x < N; x++) m[(z * H + y) * N + x] = true; return m; }
    private static P3.PhysicsWorld3D World(out P3.Body terrain, bool sleep = true)
    {
        var w = new P3.PhysicsWorld3D(new() { SleepEnabled = sleep });
        terrain = w.AddVoxelGrid(N, H, N, Floor(), 0.5f);
        return w;
    }

    private static P3.Body StaticBody(P3.PhysicsWorld3D w) { foreach (var b in w.Bodies) if (b.Type == BodyType.Static) return b; throw new InvalidOperationException("no static body"); }

    private static void AssertSameGrid(VoxelGrid3D expected, VoxelGrid3D actual, string label)
    {
        Assert.True(expected.Occupancy.SequenceEqual(actual.Occupancy), label + ": occupancy");
        Assert.True(expected.TriangleCount == actual.TriangleCount, label + ": triangle count " + expected.TriangleCount + " vs " + actual.TriangleCount);
        for (int id = 0; id < expected.TriangleIdCount; id++)
        {
            bool a = expected.TryGetTriangle(id, out var ta), b = actual.TryGetTriangle(id, out var tb);
            Assert.True(a == b, label + ": face exposure " + id);
            if (!a) continue;
            Assert.True(ta.V0 == tb.V0 && ta.V1 == tb.V1 && ta.V2 == tb.V2 && ta.CoplanarPatchId == tb.CoplanarPatchId, label + ": triangle " + id + " patch " + ta.CoplanarPatchId + " vs " + tb.CoplanarPatchId);
        }
    }

    [Fact]
    public void IncrementalEditsEqualFreshConstruction()
    {
        const int X = 9, Y = 6, Z = 8; var random = new Random(5); var occ = new bool[X * Y * Z];
        for (int i = 0; i < occ.Length; i++) occ[i] = random.Next(3) != 0;
        var grid = new VoxelGrid3D(X, Y, Z, occ, 0.3f);
        for (int n = 0; n < 150; n++)
        {
            var cells = new SortedSet<int>(); int count = 1 + random.Next(n % 10 == 0 ? 40 : 4);
            for (int i = 0; i < count; i++) cells.Add(random.Next(occ.Length));
            bool solid = random.Next(2) == 0;
            grid.Edit(cells.ToArray(), solid);
            foreach (int c in cells) occ[c] = solid;
            AssertSameGrid(new VoxelGrid3D(X, Y, Z, occ, 0.3f), grid, "edit " + n);
        }
        // A large flat floor: a pit splits and rejoins its patches.
        var big = new bool[24 * 4 * 24]; for (int z = 0; z < 24; z++) for (int y = 0; y < 3; y++) for (int x = 0; x < 24; x++) big[(z * 4 + y) * 24 + x] = true;
        var g2 = new VoxelGrid3D(24, 4, 24, big);
        for (int n = 0; n < 40; n++)
        {
            int x = random.Next(24), z = random.Next(24); var cells = new SortedSet<int>(); for (int k = 0; k < 3; k++) cells.Add((z * 4 + 2) * 24 + Math.Clamp(x + k, 0, 23));
            bool solid = n % 3 == 2; g2.Edit(cells.ToArray(), solid); foreach (int c in cells) big[c] = solid;
            AssertSameGrid(new VoxelGrid3D(24, 4, 24, big), g2, "floor edit " + n);
        }
    }

    [Fact]
    public void EditedGridDoesNotChangeTheOriginalItWasCloned()
    {
        var a = new VoxelGrid3D(4, 4, 4, Enumerable.Repeat(true, 64).ToArray()); var b = a.CloneForEdit();
        b.Edit([21], false);
        Assert.True(a.IsSolid(1, 1, 1)); Assert.False(b.IsSolid(1, 1, 1));
        Assert.Equal(new VoxelGrid3D(4, 4, 4, Enumerable.Repeat(true, 64).ToArray()).TriangleCount, a.TriangleCount);
    }

    [Fact]
    public void DiggingUnderARestingBoxDropsIt()
    {
        var w = World(out var terrain); var box = w.AddBox(new(1, 1, 1), new(10, 2.5, 10));
        for (int i = 0; i < 300; i++) w.Step(1f / 60);
        Assert.False(box.IsAwake); Assert.InRange(box.Position.Y, 2.49, 2.51);
        var voxels = terrain.Voxels;
        int dug = voxels.DigSphere(new(10, 2, 10), 1.8);
        Assert.True(dug > 20); Assert.True(box.IsAwake);
        for (int i = 0; i < 120; i++) w.Step(1f / 60);
        Assert.True(box.Position.Y < 1.5, "box fell into the pit: " + box.Position.Y);
        // Filling the pit again with a block restores the occupancy exactly.
        voxels.SetVoxels(14, 0, 14, 12, Top, 12, true);
        Assert.True(w.GetVoxelOccupancy(voxels).SequenceEqual(w.GetVoxelOccupancy(voxels)));
        var fresh = new P3.PhysicsWorld3D(); var t2 = fresh.AddVoxelGrid(N, H, N, Floor(), 0.5f);
        Assert.Equal(fresh.GetVoxelOccupancy(t2.Voxels).Count(c => c), w.GetVoxelOccupancy(voxels).Count(c => c));
    }

    [Fact]
    public void BodyBesideAnEditKeepsItsMotionBitForBit()
    {
        (double X, double Y, double Z, uint Vy) Run(bool dig)
        {
            var w = World(out var terrain); var box = w.AddBox(new(1, 1, 1), new(15, 2.5, 15)); box.LinearVelocity = new(1, 0, 0);
            for (int i = 0; i < 90; i++) w.Step(1f / 60);
            if (dig) Assert.True(terrain.Voxels.DigSphere(new(4, 2, 4), 1.2) > 0);
            for (int i = 0; i < 120; i++) w.Step(1f / 60);
            return (box.Position.X, box.Position.Y, box.Position.Z, BitConverter.SingleToUInt32Bits(box.LinearVelocity.Y));
        }
        Assert.Equal(Run(false), Run(true));
    }

    [Fact]
    public void CharacterElsewhereOnTheGridIsUnaffectedByAnEdit()
    {
        (double X, double Y, double Z) Run(bool dig)
        {
            var w = World(out var terrain); var c = w.AddCharacter(new Vector3D(15, 2.3, 15), P3.CharacterDef.Humanoid);
            for (int i = 0; i < 240; i++)
            {
                if (i == 120 && dig) Assert.True(terrain.Voxels.DigSphere(new(4, 2, 4), 1.2) > 0);
                c.Move(new Vector3F(0.01f, c.IsGrounded ? -0.01f : -0.1f, 0), 1f / 60); w.Step(1f / 60);
            }
            var p = c.Position; return (p.X, p.Y, p.Z);
        }
        Assert.Equal(Run(false), Run(true));
    }

    [Fact]
    public void SlidingAcrossADugAndRefilledSeamHasNoVerticalKicks()
    {
        var w = new P3.PhysicsWorld3D(new() { SleepEnabled = false });
        var terrain = w.AddVoxelGrid(N, H, N, Floor(), 0.5f, material: new P3.Material3D(0));
        var box = w.AddBox(new(0.8f, 1, 0.8f), new(2, 2.5, 10), material: new P3.Material3D(0));
        for (int i = 0; i < 90; i++) w.Step(1f / 60);
        var v = terrain.Voxels;
        v.DigSphere(new(10, 2, 10), 2.5);
        v.SetVoxels(14, 0, 14, 7, Top, 13, true); v.SetVoxels(19, 0, 14, 8, Top, 13, true);
        box.LinearVelocity = new(4, 0, 0); float max = 0;
        for (int i = 0; i < 200; i++) { w.Step(1f / 60); max = Math.Max(max, Math.Abs(box.LinearVelocity.Y)); }
        Assert.True(max < 1e-4f, "max |Vy| " + max); Assert.True(box.Position.X > 12);
    }

    [Fact]
    public void DetachedPieceBecomesABodyWithTheRightVolumeAndMass()
    {
        var m = Floor(); const int cx = 20;
        for (int y = Top; y < Top + 2; y++) m[(20 * H + y) * N + cx] = true; // a two-cell neck
        for (int y = Top + 2; y < Top + 4; y++) for (int z = 19; z <= 21; z++) for (int x = cx - 1; x <= cx + 1; x++) m[(z * H + y) * N + x] = true; // a 3x2x3 head
        var w = new P3.PhysicsWorld3D(); var terrain = w.AddVoxelGrid(N, H, N, m, 0.5f); var v = terrain.Voxels;
        Assert.Empty(v.DetachIslands());
        Assert.Equal(1, v.SetVoxel(cx, Top, 20, false));
        var bodies = v.DetachIslands();
        Assert.Single(bodies);
        float volume = (1 + 18) * 0.125f; // the remaining neck cell and the head
        Assert.InRange(bodies[0].Mass, volume * 0.999f, volume * 1.001f);
        for (int i = 0; i < 120; i++) w.Step(1f / 60);
        Assert.InRange(bodies[0].Position.Y, 2.0, 3.2);
    }

    [Fact]
    public void EditsSnapshotForkAndReplayExactly()
    {
        var w = World(out var terrain); var box = w.AddBox(new(1, 1, 1), new(10, 2.5, 10));
        for (int i = 0; i < 30; i++) w.Step(1f / 60);
        terrain.Voxels.DigSphere(new(10, 2, 10), 1.6);
        var bytes = w.Snapshot();
        var other = new P3.PhysicsWorld3D(); other.Restore(bytes); Assert.Equal(bytes, other.Snapshot());
        var fork = w.Fork();
        for (int i = 0; i < 90; i++)
        {
            w.Step(1f / 60); other.Step(1f / 60); fork.Step(1f / 60);
            if (i == 40) fork.GetVoxelShape(StaticBody(fork)).DigSphere(new(15, 2, 15), 1.0); // the fork's own edit
            if (i < 40) { Assert.Equal(w.ComputeHash(), other.ComputeHash()); Assert.Equal(w.ComputeHash(), fork.ComputeHash()); }
        }
        Assert.Equal(w.ComputeHash(), other.ComputeHash());
        Assert.NotEqual(w.ComputeHash(), fork.ComputeHash());
        Assert.True(w.GetVoxelOccupancy(terrain.Voxels)[(30 * H + 3) * N + 30]); // the original is untouched by the fork's edit
    }

    [Fact]
    public void HostileEditsAreRejectedBeforeAnyChange()
    {
        var w = World(out var terrain); var v = terrain.Voxels; var before = w.Snapshot();
        Assert.Throws<ArgumentOutOfRangeException>(() => v.SetVoxels(-1, 0, 0, 1, 1, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => v.SetVoxels(0, 0, 0, N + 1, 1, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => v.DigSphere(new(double.NaN, 0, 0), 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => v.DigSphere(new(0, 0, 0), double.PositiveInfinity));
        Assert.Throws<ArgumentOutOfRangeException>(() => v.StampBox(new(0, 0, 0), new(float.NaN, 1, 1)));
        Assert.Equal(before, w.Snapshot());
        Assert.Throws<ArgumentException>(() => w.GetVoxelShape(w.AddBox(new(1, 1, 1), new(0, 10, 0))));
        Assert.Equal(0, v.DigSphere(new(1e6, 0, 0), 1)); // far outside the grid: nothing to change
        Assert.Equal(0, v.DigSphere(new(5, 100, 5), 0)); // zero radius
    }
}
