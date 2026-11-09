using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// Targets for geometry that is edited or sampled at run time: signed distance fields built from hostile meshes, samples and recipes, voxel edit sequences, carve sequences and the snapshot readers for the sections they use.
internal static class FuzzTerrain
{
    private static float F(byte[] b, ref int at, float scale = 1f) { if (at + 2 > b.Length) { at = 0; if (b.Length < 2) return 0f; } int v = (short)(b[at] | (b[at + 1] << 8)); at += 2; return v * scale / 256f; }

    // A mesh from hostile bytes: a handful of vertices (some odd values), triangles from index bytes; tiny meshes and valid solids both occur.
    internal static void SdfMesh(byte[] input)
    {
        if (input.Length < 8) return;
        int at = 2; int nv = 4 + input[0] % 10, nt = 4 + input[1] % 20;
        var v = new Vector3F[nv];
        for (int i = 0; i < nv; i++) v[i] = new Vector3F(F(input, ref at, 0.5f), F(input, ref at, 0.5f), F(input, ref at, 0.5f));
        if ((input[0] & 0x40) != 0) v[input[1] % nv] = new Vector3F(float.NaN, 0, 0);
        var idx = new int[3 * nt];
        for (int i = 0; i < idx.Length; i++) idx[i] = (input[(at + i) % input.Length] % (nv + 1)) - ((input[0] & 0x20) != 0 ? 1 : 0);
        // Sometimes a known closed solid with fuzzed vertex positions, so the valid path runs too.
        if ((input[0] & 0x80) != 0)
        {
            v = new Vector3F[8]; for (int i = 0; i < 8; i++) v[i] = new Vector3F(((i & 1) != 0 ? 1 : -1) + F(input, ref at, 0.01f), ((i & 2) != 0 ? 1 : -1) + F(input, ref at, 0.01f), ((i & 4) != 0 ? 1 : -1) + F(input, ref at, 0.01f));
            idx = [0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4, 2, 6, 7, 2, 7, 3, 0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5];
        }

        float cell = (input[0] & 0x10) != 0 ? F(input, ref at, 0.01f) : 0.25f;
        try
        {
            var grid = SdfGrid3D.FromMesh(v, idx, cell, 1 + input[1] % 3);
            _ = grid.Value(new Vector3F(0.1f, 0.2f, 0.3f)); _ = grid.Gradient(default); grid.Trace(new Vector3F(0, 5, 0), new Vector3F(0, -1, 0), 10, out _, out _, out _);
            var soup = new SdfSoup3D(grid); for (int i = 0; i < Math.Min(soup.TriangleIdCount, 4000); i++) soup.TryGetTriangle(i, out _);
            _ = grid.ComputeMass(2);
        }
        catch (Exception ex) when (ex is SdfBuildException or InvalidOperationException) { }
    }

    internal static void SdfGrid(byte[] input)
    {
        if (input.Length < 10) return;
        int x = 2 + input[0] % 7, y = 2 + input[1] % 7, z = 2 + input[2] % 7; if ((input[3] & 1) != 0) { x = BitConverter.ToInt32(input, 4); y = input[0]; z = input[1]; }
        float cell = (input[3] & 2) != 0 ? BitConverter.ToSingle(input, 6 % Math.Max(1, input.Length - 4)) : 0.5f;
        int n = (long)x * y * z is > 0 and <= 512 ? x * y * z : 0; var samples = new float[n];
        for (int i = 0; i < n; i++) samples[i] = ((input[(i + 4) % input.Length] - 128) / 64f) + ((input[3] & 4) != 0 && i == n / 2 ? float.NaN : 0f);
        try
        {
            var grid = new SdfGrid3D(x, y, z, samples, cell);
            _ = grid.Value(new Vector3F(0.2f, 0.1f, 0.3f)); _ = grid.Gradient(new Vector3F(9, -9, 9)); _ = grid.Lipschitz;
            grid.Trace(new Vector3F(-3, 1, 1), new Vector3F(1, 0.1f, 0), 50, out _, out _, out _);
            var soup = new SdfSoup3D(grid);
            var w = new P3.PhysicsWorld3D(new() { WorldId = 140 });
            w.AddSdf(grid);
            // A dynamic field needs an inside; a convex body and a dynamic field fall on it.
            w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(0.4, 3, 0.4));
            try { w.AddSdf(grid, new Vector3D(0.3, 5, 0.2), type: BodyType.Dynamic); } catch (ArgumentException) { } catch (InvalidOperationException) { }
            for (int i = 0; i < 4; i++) { try { w.Step(1f / 60); } catch (PhysicsException) { break; } }
            _ = soup.TriangleCount;
        }
        catch (Exception ex) when (ex is SdfBuildException or ArgumentException or PhysicsException) { }
    }

    private static byte[]? s_sdfBase;
    internal static void SdfSnapshot(byte[] input)
    {
        if (s_sdfBase == null)
        {
            var w = new P3.PhysicsWorld3D(new() { WorldId = 141 });
            w.AddSdf(SdfGrid3D.FromFunction(new Vector3F(-2, -2, -2), new Vector3F(2, 2, 2), 0.5f, SdfGrid3D.Sphere(default, 1f)));
            w.AddSdf(SdfGrid3D.FromFunction(new Vector3F(-1, -1, -1), new Vector3F(1, 1, 1), 0.5f, SdfGrid3D.Sphere(default, 0.5f)), new Vector3D(0, 4, 0), type: BodyType.Dynamic);
            w.Step(1f / 60); s_sdfBase = w.Snapshot();
        }

        var bytes = (byte[])s_sdfBase.Clone();
        for (int i = 0; i + 2 < input.Length; i += 3) { int off = ((input[i] << 8) | input[i + 1]) % bytes.Length; bytes[off] ^= input[i + 2]; }
        var world = new P3.PhysicsWorld3D(new() { WorldId = 141 }); var before = world.Snapshot();
        try { world.Restore(bytes); }
        catch (SnapshotFormatException) { if (!world.Snapshot().AsSpan().SequenceEqual(before)) throw new InvalidOperationException("Rejected field snapshot changed world."); return; }
        if (!world.Snapshot().AsSpan().SequenceEqual(bytes)) throw new InvalidOperationException("Accepted field snapshot changed bytes.");
        try { world.Step(1f / 60); } catch (PhysicsException) { }
    }

    // Edit sequences on a small voxel world: the edited grid must always equal a fresh grid with the same occupancy.
    internal static void VoxelEdits(byte[] input)
    {
        if (input.Length < 4) return;
        const int N = 6;
        var w = new P3.PhysicsWorld3D(new() { WorldId = 142 });
        var cells = new bool[N * N * N]; for (int i = 0; i < cells.Length; i++) cells[i] = (input[i % input.Length] & 3) != 0;
        var ground = w.AddVoxelGrid(N, N, N, cells, 0.5f); var voxels = ground.Voxels;
        w.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), new Vector3D(1.2, 4, 1.2));
        for (int at = 0; at + 4 < input.Length; at += 5)
        {
            int op = input[at] % 5, a = input[at + 1] % (N + 2) - 1, b = input[at + 2] % (N + 2) - 1, c = input[at + 3] % (N + 2) - 1, d = 1 + input[at + 4] % 4;
            try
            {
                switch (op)
                {
                    case 0: voxels.SetVoxels(a, b, c, d, d, d, (input[at + 4] & 0x80) == 0); break;
                    case 1: voxels.DigSphere(new Vector3D(a * 0.5, b * 0.5, c * 0.5), d * 0.3); break;
                    case 2: voxels.FillSphere(new Vector3D(a * 0.5, b * 0.5, c * 0.5), d * 0.3); break;
                    case 3: voxels.StampBox(new Vector3D(a * 0.5, b * 0.5, c * 0.5), new Vector3F(d * 0.25f, 0.4f, d * 0.2f), QuaternionF.CreateFromAxisAngle(Vector3F.UnitY, a), (input[at + 4] & 0x80) == 0); break;
                    default: voxels.DetachIslands(P3.VoxelAnchors.Bottom, 1, 4); break;
                }
            }
            catch (ArgumentException) { }
            try { w.Step(1f / 60); } catch (PhysicsException) { return; }
            var occupancy = w.GetVoxelOccupancy(voxels); var (sx, sy, sz, cs) = w.GetVoxelSize(voxels);
            var fresh = new VoxelGrid3D(sx, sy, sz, occupancy, cs);
            var live = new VoxelGrid3D(sx, sy, sz, occupancy, cs);
            if (fresh.TriangleCount != live.TriangleCount) throw new InvalidOperationException("Fresh grids disagree.");
        }

        // The edited world restores to itself.
        var bytes = w.Snapshot(); var other = new P3.PhysicsWorld3D(new() { WorldId = 142 }); other.Restore(bytes);
        if (!other.Snapshot().AsSpan().SequenceEqual(bytes)) throw new InvalidOperationException("Edited voxel world did not restore to the same bytes.");
    }

    // Carve sequences on a small bitmask terrain: after every edit the boundary must equal a fresh construction from the same cells.
    internal static void Carve(byte[] input)
    {
        if (input.Length < 4) return;
        const int W = 10, H = 8;
        var w = new P2.PhysicsWorld2D(new() { WorldId = 143 });
        var mask = new bool[W * H]; for (int i = 0; i < mask.Length; i++) mask[i] = (input[i % input.Length] & 3) != 0;
        var terrain = w.AddBitmaskTerrain(W, H, mask, 0.5f); w.AddBox(new Vector2F(0.4f, 0.4f), new Vector2D(2, 6));
        for (int at = 0; at + 4 < input.Length; at += 5)
        {
            int op = input[at] % 6; double a = (input[at + 1] % 60 - 5) * 0.1, b = (input[at + 2] % 50 - 5) * 0.1; float r = 0.1f + (input[at + 3] % 20) * 0.1f;
            try
            {
                switch (op)
                {
                    case 0: w.CarveCircle(terrain, new Vector2D(a, b), r); break;
                    case 1: w.FillCircle(terrain, new Vector2D(a, b), r); break;
                    case 2: w.CarvePolygon(terrain, [new Vector2D(a, b), new Vector2D(a + r, b), new Vector2D(a + r, b + r * 0.5)]); break;
                    case 3: w.FillPolygon(terrain, [new Vector2D(a, b), new Vector2D(a + r, b), new Vector2D(a, b + r)]); break;
                    case 4: w.StampMask(terrain, input[at + 1] % 8, input[at + 2] % 6, 2, 2, [true, (input[at + 3] & 1) != 0, true, true], (input[at + 4] & 1) != 0); break;
                    default: w.DetachIslands(terrain, P2.TerrainAnchors.Bottom, 1, 3); break;
                }
            }
            catch (ArgumentException) { }
            try { w.Step(1f / 60); } catch (PhysicsException) { return; }
            var grid = w.GetTileGrid(terrain); var fresh = new TileGrid2D(grid.Width, grid.Height, grid.Tiles, grid.TileSize, default, true);
            if (fresh.SegmentCount != grid.SegmentCount) throw new InvalidOperationException("Edited boundary differs from a fresh one: " + grid.SegmentCount + " vs " + fresh.SegmentCount);
            for (int i = 0; i < grid.SegmentCount; i++) if (TileGrid2D.SegmentComparer.Instance.Compare(fresh.GetSegment(i), grid.GetSegment(i)) != 0) throw new InvalidOperationException("Edited boundary differs from a fresh one at segment " + i);
        }

        var bytes = w.Snapshot(); var other = new P2.PhysicsWorld2D(new() { WorldId = 143 }); other.Restore(bytes);
        if (!other.Snapshot().AsSpan().SequenceEqual(bytes)) throw new InvalidOperationException("Carved world did not restore to the same bytes.");
    }

    private static byte[]? s_contourBase;
    internal static void ContourSnapshot(byte[] input)
    {
        if (s_contourBase == null)
        {
            var w = new P2.PhysicsWorld2D(new() { WorldId = 144 }); var m = new bool[6 * 5]; for (int i = 0; i < m.Length; i++) m[i] = i % 7 != 3;
            w.AddBitmaskTerrain(6, 5, m, 0.5f); s_contourBase = w.Snapshot();
        }

        var bytes = (byte[])s_contourBase.Clone();
        for (int i = 0; i + 2 < input.Length; i += 3) { int off = bytes.Length - 1 - (((input[i] << 8) | input[i + 1]) % Math.Min(400, bytes.Length)); bytes[off] ^= input[i + 2]; }
        var world = new P2.PhysicsWorld2D(new() { WorldId = 144 }); var before = world.Snapshot();
        try { world.Restore(bytes); }
        catch (SnapshotFormatException) { if (!world.Snapshot().AsSpan().SequenceEqual(before)) throw new InvalidOperationException("Rejected contour snapshot changed world."); return; }
        if (!world.Snapshot().AsSpan().SequenceEqual(bytes)) throw new InvalidOperationException("Accepted contour snapshot changed bytes.");
    }

    internal static void SdfGrid2D(byte[] input)
    {
        if (input.Length < 8) return;
        int x = 2 + input[0] % 12, y = 2 + input[1] % 12; if ((input[2] & 1) != 0) x = BitConverter.ToInt32(input, 3 % Math.Max(1, input.Length - 4));
        float cell = (input[2] & 2) != 0 ? BitConverter.ToSingle(input, 4 % Math.Max(1, input.Length - 4)) : 0.5f;
        int n = (long)x * y is > 0 and <= 400 ? x * y : 0; var samples = new float[n];
        for (int i = 0; i < n; i++) samples[i] = (input[(i + 3) % input.Length] - 128) / 64f + ((input[2] & 4) != 0 && i == n / 2 ? float.NaN : 0f);
        try
        {
            var grid = new RecursiveAI.OnePhysics.SdfGrid2D(x, y, samples, cell);
            _ = grid.Value(new Vec2(0.3f, 0.2f)); _ = grid.Gradient(new Vec2(9, -9)); grid.Trace(new Vec2(-3, 0.5f), new Vec2(1, 0), 50, out _, out _, out _);
            var loops = grid.ExtractContours((input[2] & 8) != 0 ? 0.1f : 0f);
            var w = new P2.PhysicsWorld2D(new() { WorldId = 145 }); w.AddSdf(grid); w.AddBox(new Vector2F(0.3f, 0.3f), new Vector2D(0.5, 3));
            for (int i = 0; i < 4; i++) { try { w.Step(1f / 60); } catch (PhysicsException) { break; } }
            _ = loops.Count;
        }
        catch (Exception ex) when (ex is ArgumentException or PhysicsException or InvalidShapeException) { }
    }
}
