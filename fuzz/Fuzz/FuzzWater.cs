#pragma warning disable OPHYS0002
using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OneMath;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

internal static class FuzzWater
{
    private static float F(byte[] input, int at, float fallback) => at + 4 <= input.Length ? BitConverter.ToSingle(input, at) : fallback;
    private static double D(byte[] input, int at, double fallback) => at + 8 <= input.Length ? BitConverter.ToDouble(input, at) : fallback;

    // A shallow-water definition from hostile bytes: sizes, positions, level, bed, edges, sources and coefficients. Creation either fails with an ArgumentOutOfRangeException or produces water that steps with a body in it,
    // snapshots and restores to the same bytes, and answers queries.
    internal static void Definition(byte[] input)
    {
        if (input.Length < 8) return;
        bool planar = (input[0] & 1) == 0;
        int cells = 2 + input[1] % 40, cellsZ = 2 + input[2] % 24;
        float size = F(input, 4, 0.5f);
        var edges = new[] { new WaterBoundary((WaterBoundaryKind)(input[3] & 7), D(input, 8, 1.0), F(input, 16, 1f)), new WaterBoundary((WaterBoundaryKind)((input[3] >> 3) & 7), D(input, 20, 1.0), F(input, 28, 0f)) };
        var sources = (input[0] & 2) != 0 ? new[] { new WaterSource(F(input, 32, 1f), F(input, 36, 1f), F(input, 40, 0.5f), F(input, 44, 1f)) } : Array.Empty<WaterSource>();
        float[]? bed = null;
        if ((input[0] & 4) != 0)
        {
            int n = planar ? cells : cells * cellsZ;
            bed = new float[(input[0] & 8) != 0 ? n : n + 1];
            for (int i = 0; i < bed.Length; i++) { bed[i] = F(input, 48 + 4 * i, 0.1f * (i % 5)); }
        }

        try
        {
            if (planar)
            {
                var w = new P2.PhysicsWorld2D(new() { WorldId = 71 });
                w.AddBox(new(20f, 1f), at: new(5, -0.5), type: BodyType.Static);
                var water = w.CreateShallowWater(new P2.ShallowWaterDef
                {
                    Cells = cells, CellSize = size, MinX = D(input, 8, 0.0), Level = D(input, 20, 1.5), FloorHeight = D(input, 28, -1.0), Bed = bed, West = edges[0], East = edges[1], Sources = sources,
                    Density = F(input, 36, 1000f), Damping = F(input, 40, 0.1f), Displacement = F(input, 44, 0.5f), Cfl = F(input, 12, 0.25f), MaxSubsteps = 1 + input[2],
                });
                w.AddBox(new(0.5f, 0.5f), at: new(2, 2.5), density: 400f);
                for (int i = 0; i < 20; i++) w.Step(1f / 60);
                var bytes = w.Snapshot(); w.Restore(bytes);
                if (!bytes.AsSpan().SequenceEqual(w.Snapshot())) throw new InvalidOperationException("Water round trip changed bytes.");
                w.QueryWater(new(2, 2), out _); w.RaycastWater(new(2, 9), new(0, -1), out _); water.TryGetSample(2, out _);
            }
            else
            {
                var w = new P3.PhysicsWorld3D(new() { WorldId = 71 });
                w.AddBox(new(20f, 1f, 20f), at: new(5, -0.5, 5), type: BodyType.Static);
                var water = w.CreateShallowWater(new P3.ShallowWaterDef
                {
                    CellsX = cells, CellsZ = cellsZ, CellSize = size, MinX = D(input, 8, 0.0), MinZ = D(input, 12, 0.0), Level = D(input, 20, 1.5), FloorHeight = D(input, 28, -1.0), Bed = bed, West = edges[0], East = edges[1], South = edges[1], North = edges[0], Sources = sources,
                    Density = F(input, 36, 1000f), Damping = F(input, 40, 0.1f), Displacement = F(input, 44, 0.5f), Cfl = F(input, 16, 0.25f), MaxSubsteps = 1 + input[2],
                });
                w.AddBox(new(0.5f, 0.5f, 0.5f), at: new(2, 2.5, 2), density: 400f);
                for (int i = 0; i < 20; i++) w.Step(1f / 60);
                var bytes = w.Snapshot(); w.Restore(bytes);
                if (!bytes.AsSpan().SequenceEqual(w.Snapshot())) throw new InvalidOperationException("Water round trip changed bytes.");
                w.QueryWater(new(2, 2, 2), out _); w.RaycastWater(new(2, 9, 2), new(0, -1, 0), out _); water.TryGetSample(2, 2, out _);
            }
        }
        catch (ArgumentException) { }
    }

    private static byte[]? s_planar, s_spatial;
    private static byte[] Baseline(bool planar)
    {
        if (planar)
        {
            if (s_planar != null) return s_planar;
            var w = new P2.PhysicsWorld2D(new() { WorldId = 71 });
            w.AddBox(new(30f, 1f), at: new(5, -0.5), type: BodyType.Static);
            var water = w.AddShallowWater(0, 10, 1.5, 0.5f, floor: -1.0);
            water.SetSources([new WaterSource(1, 0, 0.5f, 0.5f)]);
            w.AddBox(new(0.6f, 0.6f), at: new(4, 2), density: 400f);
            for (int i = 0; i < 20; i++) w.Step(1f / 60);
            return s_planar = w.Snapshot();
        }

        if (s_spatial != null) return s_spatial;
        var world = new P3.PhysicsWorld3D(new() { WorldId = 71 });
        world.AddBox(new(30f, 1f, 30f), at: new(5, -0.5, 5), type: BodyType.Static);
        var water3 = world.AddShallowWater(0, 0, 8, 8, 1.5, 0.5f, floor: -1.0);
        water3.SetBoundary(1, WaterBoundary.Inflow(1.5, 0.5f));
        world.AddBox(new(0.6f, 0.6f, 0.6f), at: new(4, 2, 4), density: 400f);
        for (int i = 0; i < 20; i++) world.Step(1f / 60);
        return s_spatial = world.Snapshot();
    }

    // Damaged snapshots with water: restore either rejects them with a SnapshotFormatException leaving the world unchanged, or accepts them and writes the same bytes back.
    internal static void Snapshot(byte[] input)
    {
        bool planar = input.Length == 0 || (input[0] & 1) == 0;
        var bytes = (byte[])Baseline(planar).Clone();
        for (int i = 1; i + 2 < input.Length; i += 3)
        {
            int offset = bytes.Length - 1 - (((input[i] << 8) | input[i + 1]) % Math.Min(4000, bytes.Length)); bytes[offset] ^= input[i + 2];
        }
        if (input.Length > 0 && (input[0] & 15) == 15) Array.Resize(ref bytes, input[^1] * bytes.Length / 256);
        if (planar)
        {
            var world = new P2.PhysicsWorld2D(new() { WorldId = 71 }); var before = world.Snapshot();
            try { world.Restore(bytes); }
            catch (SnapshotFormatException) { if (!before.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Rejected water snapshot changed world."); return; }
            if (!bytes.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Accepted water snapshot changed bytes.");
            for (int i = 0; i < 2; i++) { try { world.Step(1f / 60); } catch (PhysicsException) { return; } }
        }
        else
        {
            var world = new P3.PhysicsWorld3D(new() { WorldId = 71 }); var before = world.Snapshot();
            try { world.Restore(bytes); }
            catch (SnapshotFormatException) { if (!before.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Rejected water snapshot changed world."); return; }
            if (!bytes.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Accepted water snapshot changed bytes.");
            for (int i = 0; i < 2; i++) { try { world.Step(1f / 60); } catch (PhysicsException) { return; } }
        }
    }
}
