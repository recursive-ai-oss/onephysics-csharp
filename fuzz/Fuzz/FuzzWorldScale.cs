#pragma warning disable OPHYS0002
using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OneMath;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

internal static class FuzzWorldScale
{
    private static float F(byte[] input, int at, float fallback) => at + 4 <= input.Length ? BitConverter.ToSingle(input, at) : fallback;
    private static double D(byte[] input, int at, double fallback) => at + 8 <= input.Length ? BitConverter.ToDouble(input, at) : fallback;

    // A tile definition from hostile bytes: sample counts, heights, scales, offsets, the ring of neighbour heights (some sides wrong-sized), holes, meshes, boxes and voxel grids, loaded and unloaded in a world that steps and
    // round-trips its snapshot. A bad definition is refused at the call or dropped when built; nothing throws out of a step or a flush, and the registry survives the round trip.
    internal static void Tile(byte[] input)
    {
        if (input.Length < 8) return;
        bool planar = (input[0] & 1) == 0;
        int nx = 2 + input[1] % 14, nz = 2 + input[2] % 14;
        try
        {
            if (planar)
            {
                var w = new P2.PhysicsWorld2D(new() { WorldId = 72 });
                var def = new P2.TileDef2D { Origin = new Vector2D(D(input, 8, 0.0), D(input, 16, 0.0)) };
                var h = new float[nx];
                for (int i = 0; i < h.Length; i++) { h[i] = F(input, 24 + 4 * i, 0.2f * (i % 3)); }
                def.HeightFields.Add(new P2.TileHeightField2D { Heights = h, Spacing = F(input, 4, 1f), Offset = new Vector2F(F(input, 20, 0f), 0f), HeightBefore = (input[0] & 2) != 0 ? F(input, 24, 0f) : null, HeightAfter = (input[0] & 4) != 0 ? F(input, 28, 0f) : null, Holes = (input[0] & 8) != 0 ? new bool[(input[3] & 1) != 0 ? nx - 1 : nx] : null });
                if ((input[0] & 16) != 0) { def.Boxes.Add(new P2.TileBox2D { Size = new Vector2F(F(input, 32, 1f), F(input, 36, 1f)), Angle = F(input, 40, 0f) }); }
                w.LoadTile(new TileKey(input[3] % 4 - 2, 0), def);
                w.AddCircle(0.3f, new Vector2D(1, 3));
                for (int i = 0; i < 4; i++) w.Step(1f / 60);
                var bytes = w.Snapshot(); var copy = new P2.PhysicsWorld2D(new() { WorldId = 72 }); copy.Restore(bytes);
                if (!bytes.AsSpan().SequenceEqual(copy.Snapshot())) throw new InvalidOperationException("Tile round trip changed bytes.");
                w.UnloadTile(new TileKey(input[3] % 4 - 2, 0), (input[0] & 32) != 0 ? TileUnloadPolicy.Immediate : TileUnloadPolicy.WaitForContacts);
                w.Step(1f / 60); _ = w.ComputeHash();
            }
            else
            {
                var w = new P3.PhysicsWorld3D(new() { WorldId = 72 });
                var def = new P3.TileDef3D { Origin = new Vector3D(D(input, 8, 0.0), D(input, 16, 0.0), D(input, 24, 0.0)) };
                var h = new float[nx * nz];
                for (int i = 0; i < h.Length; i++) { h[i] = F(input, 32 + 4 * i, 0.2f * (i % 3)); }
                var part = new P3.TileHeightField3D { Heights = h, SamplesX = nx, SamplesZ = (input[3] & 1) != 0 ? nz : nz + 1, Scale = new Vector3F(F(input, 4, 1f), 1f, F(input, 12, 1f)), Offset = new Vector3F(F(input, 28, 0f), 0f, 0f) };
                if ((input[0] & 2) != 0)
                {
                    var e = new RecursiveAI.OnePhysics.LowLevel.HeightFieldEdges3D();
                    if ((input[0] & 4) != 0) e.West = new float[nz + (input[3] >> 7)];
                    if ((input[0] & 8) != 0) { e.East = new float[nz]; Array.Fill(e.East, F(input, 32, 0f)); }
                    if ((input[0] & 16) != 0) { e.South = new float[nx]; Array.Fill(e.South, F(input, 36, 0f)); }
                    if ((input[0] & 32) != 0) e.North = new float[nx];
                    part.Edges = e;
                }

                def.HeightFields.Add(part);
                if ((input[3] & 2) != 0) { def.Boxes.Add(new P3.TileBox3D { Size = new Vector3F(F(input, 40, 1f), F(input, 44, 1f), F(input, 48, 1f)) }); }
                if ((input[3] & 4) != 0) { def.Meshes.Add(new P3.TileMesh3D { Vertices = [new Vector3F(0, 0, 0), new Vector3F(F(input, 40, 1f), 0, 0), new Vector3F(0, 0, F(input, 44, 1f))], Indices = [0, 1, 2 + input[1] % 2] }); }
                w.LoadTile(new TileKey(input[3] % 4 - 2, input[2] % 3 - 1, 0), def);
                w.AddSphere(0.3f, new Vector3D(1, 3, 1));
                for (int i = 0; i < 4; i++) w.Step(1f / 60);
                var bytes = w.Snapshot(); var copy = new P3.PhysicsWorld3D(new() { WorldId = 72 }); copy.Restore(bytes);
                if (!bytes.AsSpan().SequenceEqual(copy.Snapshot())) throw new InvalidOperationException("Tile round trip changed bytes.");
                w.UnloadTile(new TileKey(input[3] % 4 - 2, input[2] % 3 - 1, 0), (input[0] & 64) != 0 ? TileUnloadPolicy.Immediate : TileUnloadPolicy.WaitForContacts);
                w.Step(1f / 60); _ = w.ComputeHash();
            }
        }
        catch (ArgumentException) { }
        catch (InvalidOperationException e) when (e.Message.Contains("already", StringComparison.Ordinal)) { }
    }

    private static P2.TileDef2D Flat2D()
    {
        var d = new P2.TileDef2D();
        d.HeightFields.Add(new P2.TileHeightField2D { Heights = new float[33], Spacing = 1f, HeightBefore = 0f, HeightAfter = 0f });
        return d;
    }

    private static P3.TileDef3D Flat3D()
    {
        var d = new P3.TileDef3D();
        d.HeightFields.Add(new P3.TileHeightField3D { Heights = new float[33 * 33], SamplesX = 33, SamplesZ = 33, Edges = new RecursiveAI.OnePhysics.LowLevel.HeightFieldEdges3D { West = new float[33], East = new float[33], South = new float[33], North = new float[33] } });
        return d;
    }

    private static byte[]? s_planar, s_spatial;

    private static byte[] Baseline(bool planar)
    {
        if (planar)
        {
            if (s_planar != null) return s_planar;
            var w = new P2.PhysicsWorld2D(new() { WorldId = 72 });
            w.LoadTile(new TileKey(0, 0), Flat2D());
            var b = w.AddCircle(0.3f, new Vector2D(3, 1.0)); b.TimeScale = 0.5f;
            w.SetSimulationLod([new Vector2D(0, 0)], [new LodBand(10, 1)], 4, [new P2.LodRegion2D(new(5, -5), new(15, 5), 2)]);
            w.SetTimeScaleRegions([new P2.TimeScaleRegion2D(new(0, -5), new(8, 5), 0.5f)]);
            for (int i = 0; i < 10; i++) w.Step(1f / 60);
            return s_planar = w.Snapshot();
        }

        if (s_spatial != null) return s_spatial;
        var world = new P3.PhysicsWorld3D(new() { WorldId = 72 });
        world.LoadTile(new TileKey(0, 0), Flat3D());
        var ball = world.AddSphere(0.3f, new Vector3D(3, 1.0, 3)); ball.TimeScale = 0.5f;
        world.SetSimulationLod([new Vector3D(0, 0, 0)], [new LodBand(10, 1)], 4, [new P3.LodRegion3D(new(5, -5, -5), new(15, 5, 5), 2)]);
        world.SetTimeScaleRegions([new P3.TimeScaleRegion3D(new(0, -5, -5), new(8, 5, 5), 0.5f)]);
        for (int i = 0; i < 10; i++) world.Step(1f / 60);
        return s_spatial = world.Snapshot();
    }

    // Damaged snapshots with the world-scale section: restore either rejects them with a SnapshotFormatException leaving the world unchanged, or accepts them and writes the same bytes back.
    internal static void Snapshot(byte[] input)
    {
        bool planar = input.Length == 0 || (input[0] & 1) == 0;
        var bytes = (byte[])Baseline(planar).Clone();
        for (int i = 1; i + 2 < input.Length; i += 3)
        {
            // the world-scale section is the last one: bias the damage to the tail
            int offset = bytes.Length - 1 - (((input[i] << 8) | input[i + 1]) % Math.Min(600, bytes.Length)); bytes[offset] ^= input[i + 2];
        }

        if (input.Length > 0 && (input[0] & 15) == 15) Array.Resize(ref bytes, input[^1] * bytes.Length / 256);
        if (planar)
        {
            var world = new P2.PhysicsWorld2D(new() { WorldId = 72 }); var before = world.Snapshot();
            try { world.Restore(bytes); }
            catch (SnapshotFormatException) { if (!before.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Rejected world-scale snapshot changed world."); return; }
            if (!bytes.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Accepted world-scale snapshot changed bytes.");
            for (int i = 0; i < 2; i++) { try { world.Step(1f / 60); } catch (PhysicsException) { return; } }
        }
        else
        {
            var world = new P3.PhysicsWorld3D(new() { WorldId = 72 }); var before = world.Snapshot();
            try { world.Restore(bytes); }
            catch (SnapshotFormatException) { if (!before.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Rejected world-scale snapshot changed world."); return; }
            if (!bytes.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Accepted world-scale snapshot changed bytes.");
            for (int i = 0; i < 2; i++) { try { world.Step(1f / 60); } catch (PhysicsException) { return; } }
        }
    }
}
