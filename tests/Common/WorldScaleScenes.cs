#pragma warning disable OPHYS0002, OPHYS0003
using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// Scenes for world scale (streamed terrain tiles, simulation level of detail, local time scale). Terrain comes from a function of the position built from integer hashing and
// arithmetic only, so every platform builds the same heights bit for bit.
internal static class WorldScaleScenes
{
    internal const float Dt = 1f / 60f;
    internal const int TileCells = 32;
    internal const float Spacing = 1f;
    internal const double TileSize = TileCells * Spacing;

    internal static readonly string[] Names = { "streaming-3d", "streaming-2d", "lod-3d", "timescale-2d" };

    // ------------------------------------------------------------------ terrain

    private static float Lattice(int ix, int iz)
    {
        uint h = unchecked((uint)(ix * 374761393) + (uint)(iz * 668265263));
        h = unchecked((h ^ (h >> 13)) * 1274126177u);
        h ^= h >> 16;
        return (h & 0xFFFFFF) / 16777216f;
    }

    private static float Smooth(float t) => t * t * (3f - 2f * t);

    /// <summary>Gently rolling ground sloping down along +x: value noise on an 8 m lattice (up to 0.9 m high) minus 12 cm per metre.</summary>
    internal static float Height(int gx, int gz)
    {
        float fx = gx / 8f, fz = gz / 8f;
        int ix = (int)MathF.Floor(fx), iz = (int)MathF.Floor(fz);
        float tx = Smooth(fx - ix), tz = Smooth(fz - iz);
        float a = Lattice(ix, iz), b = Lattice(ix + 1, iz), c = Lattice(ix, iz + 1), d = Lattice(ix + 1, iz + 1);
        float n = (a + (b - a) * tx) + ((c + (d - c) * tx) - (a + (b - a) * tx)) * tz;
        return (n * 0.9f) - (0.12f * gx);
    }

    /// <summary>The 2D ground profile (a function of x only, with the same lattice along z = 0).</summary>
    internal static float Height2D(int gx) => Height(gx, 0);

    internal static P3.TileDef3D Tile3D(int tx, int tz, bool edges, int cells = TileCells, Func<int, int, float>? height = null)
    {
        var Height = height ?? WorldScaleScenes.Height;
        int n = cells + 1;
        var h = new float[n * n];
        int x0 = tx * cells, z0 = tz * cells;
        for (int j = 0; j < n; j++) { for (int i = 0; i < n; i++) { h[j * n + i] = Height(x0 + i, z0 + j); } }
        var def = new P3.TileDef3D { Origin = new Vector3D(x0 * (double)Spacing, 0, z0 * (double)Spacing) };
        var part = new P3.TileHeightField3D { Heights = h, SamplesX = n, SamplesZ = n, Scale = new Vector3F(Spacing, 1f, Spacing) };
        if (edges)
        {
            var e = new RecursiveAI.OnePhysics.LowLevel.HeightFieldEdges3D { West = new float[n], East = new float[n], South = new float[n], North = new float[n] };
            for (int k = 0; k < n; k++)
            {
                e.West![k] = Height(x0 - 1, z0 + k); e.East![k] = Height(x0 + cells + 1, z0 + k);
                e.South![k] = Height(x0 + k, z0 - 1); e.North![k] = Height(x0 + k, z0 + cells + 1);
            }

            part.Edges = e;
        }

        def.HeightFields.Add(part);
        return def;
    }

    internal static P2.TileDef2D Tile2D(int tx, bool edges, int cells = TileCells, Func<int, float>? height = null)
    {
        var Height2D = height ?? WorldScaleScenes.Height2D;
        int n = cells + 1, x0 = tx * cells;
        var h = new float[n];
        for (int i = 0; i < n; i++) { h[i] = Height2D(x0 + i); }
        var def = new P2.TileDef2D { Origin = new Vector2D(x0 * (double)Spacing, 0) };
        def.HeightFields.Add(new P2.TileHeightField2D { Heights = h, Spacing = Spacing, HeightBefore = edges ? Height2D(x0 - 1) : null, HeightAfter = edges ? Height2D(x0 + cells + 1) : null });
        return def;
    }
}
