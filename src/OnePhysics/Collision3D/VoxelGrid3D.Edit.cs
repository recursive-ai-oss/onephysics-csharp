using System;
using System.Buffers;
using System.Collections.Generic;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class VoxelGrid3D
{
    // Runtime edits. Face triangle ids are cell * 12 + face * 2 + half, so an edit never renumbers a face it did not
    // touch. Only faces of the changed cells and of their six neighbours can change exposure, and only coplanar
    // patches containing such a face can change membership or root, so only those are recomputed.

    /// <summary>One world's grid may be shared with a fork; the first edit on either side then works on a private copy.</summary>
    internal bool IsShared => _shared;
    internal void MarkShared() => _shared = true;

    /// <summary>A private deep copy (occupancy, patch ids, counts) that no fork shares.</summary>
    internal VoxelGrid3D CloneForEdit()
    {
        var copy = (VoxelGrid3D)MemberwiseClone();
        copy._shared = false; copy._solid = (bool[])_solid.Clone(); copy._patches = (int[])_patches.Clone();
        return copy;
    }

    /// <summary>The cells an edit really changed, and the inclusive cell box around them.</summary>
    internal readonly record struct EditResult(int[] Changed, int X0, int Y0, int Z0, int X1, int Y1, int Z1);

    /// <summary>Sets every listed cell (indices in x-fastest order, strictly increasing) to <paramref name="solid"/>. Cells already in that state are skipped. Returns the cells that changed, or null when none did.</summary>
    internal EditResult? Edit(ReadOnlySpan<int> cells, bool solid)
    {
        var changed = new List<int>();
        foreach (int c in cells) if (_solid[c] != solid) changed.Add(c);
        if (changed.Count == 0) return null;
        // Affected cells: the changed ones and their face neighbours, in increasing order.
        var affected = new SortedSet<int>();
        int sx = SizeX, sxy = SizeX * SizeY;
        foreach (int c in changed)
        {
            int x = c % sx, y = c / sx % SizeY, z = c / sxy;
            affected.Add(c);
            if (x > 0) affected.Add(c - 1); if (x < SizeX - 1) affected.Add(c + 1);
            if (y > 0) affected.Add(c - sx); if (y < SizeY - 1) affected.Add(c + sx);
            if (z > 0) affected.Add(c - sxy); if (z < SizeZ - 1) affected.Add(c + sxy);
        }
        int before = ExposedFaces(affected);
        foreach (int c in changed) _solid[c] = solid;
        _triangleCount += 2 * (ExposedFaces(affected) - before);
        RefreshPatches(affected);
        int x0 = int.MaxValue, y0 = int.MaxValue, z0 = int.MaxValue, x1 = -1, y1 = -1, z1 = -1;
        foreach (int c in changed)
        {
            int x = c % sx, y = c / sx % SizeY, z = c / sxy;
            if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y; if (z < z0) z0 = z; if (z > z1) z1 = z;
        }
        return new EditResult(changed.ToArray(), x0, y0, z0, x1, y1, z1);
    }

    private int ExposedFaces(SortedSet<int> cells)
    {
        int n = 0;
        foreach (int c in cells)
        {
            int x = c % SizeX, y = c / SizeX % SizeY, z = c / (SizeX * SizeY);
            if (!Solid(x, y, z)) continue;
            for (int f = 0; f < 6; f++) if (Exposed(x, y, z, f)) n++;
        }
        return n;
    }

    private bool FaceExposed(int face) { int cell = face / 6; return Solid(cell % SizeX, cell / SizeX % SizeY, cell / (SizeX * SizeY)) && Exposed(cell % SizeX, cell / SizeX % SizeY, cell / (SizeX * SizeY), face % 6); }

    // Exposed faces of the same orientation that touch this one along the plane (the join rule of the constructor).
    private int Neighbours(int face, Span<int> output)
    {
        int cell = face / 6, f = face % 6, axis = f / 2, n = 0;
        int x = cell % SizeX, y = cell / SizeX % SizeY, z = cell / (SizeX * SizeY);
        for (int t = 0; t < 3; t++)
        {
            if (t == axis) continue;
            for (int step = -1; step <= 1; step += 2)
            {
                int xx = x + (t == 0 ? step : 0), yy = y + (t == 1 ? step : 0), zz = z + (t == 2 ? step : 0);
                if (!Solid(xx, yy, zz) || !Exposed(xx, yy, zz, f)) continue;
                output[n++] = ((zz * SizeY + yy) * SizeX + xx) * 6 + f;
            }
        }
        return n;
    }

    private void RefreshPatches(SortedSet<int> affected)
    {
        // Faces that may have changed exposure start again as their own patch ...
        foreach (int c in affected) for (int f = 0; f < 6; f++) _patches[c * 6 + f] = c * 6 + f;
        // ... and every exposed face of those cells, or touching one in its plane, seeds a flood fill over its whole
        // patch, whose id is the smallest face index in it (what the constructor's union-find produces).
        var seeds = new SortedSet<int>(); var near = new int[4];
        foreach (int c in affected) for (int f = 0; f < 6; f++)
        {
            int face = c * 6 + f;
            if (FaceExposed(face)) seeds.Add(face);
            int cell = c, axis = f / 2, x = cell % SizeX, y = cell / SizeX % SizeY, z = cell / (SizeX * SizeY);
            for (int t = 0; t < 3; t++)
            {
                if (t == axis) continue;
                for (int step = -1; step <= 1; step += 2)
                {
                    int xx = x + (t == 0 ? step : 0), yy = y + (t == 1 ? step : 0), zz = z + (t == 2 ? step : 0);
                    if (Solid(xx, yy, zz) && Exposed(xx, yy, zz, f)) seeds.Add(((zz * SizeY + yy) * SizeX + xx) * 6 + f);
                }
            }
        }
        int words = (_patches.Length + 63) / 64;
        var visited = ArrayPool<ulong>.Shared.Rent(words); Array.Clear(visited, 0, words);
        var queue = new List<int>();
        try
        {
            foreach (int seed in seeds)
            {
                if ((visited[seed >> 6] & (1UL << (seed & 63))) != 0) continue;
                queue.Clear(); queue.Add(seed); visited[seed >> 6] |= 1UL << (seed & 63); int min = seed;
                for (int i = 0; i < queue.Count; i++)
                {
                    int count = Neighbours(queue[i], near);
                    for (int k = 0; k < count; k++)
                    {
                        int n = near[k];
                        if ((visited[n >> 6] & (1UL << (n & 63))) != 0) continue;
                        visited[n >> 6] |= 1UL << (n & 63); queue.Add(n); if (n < min) min = n;
                    }
                }
                foreach (int face in queue) _patches[face] = min;
            }
        }
        finally { ArrayPool<ulong>.Shared.Return(visited); }
    }
}
