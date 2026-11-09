using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>
/// A mesh on a regular grid: surface voxels (a triangle touches them) with the pieces of the surface inside each, and interior voxels found by filling from the outside.
/// This is the voxel stage of hierarchical approximate convex decomposition (Mamou and Ghorbel, ICIP 2009), except that the surface is kept as clipped triangle pieces so a part's hull follows the mesh rather than the voxel staircase.
/// </summary>
internal sealed class VoxelModel
{
    public const byte Empty = 0, Surface = 1, Interior = 2;

    public int Nx, Ny, Nz;
    public double H, Ox, Oy, Oz;
    public byte[] Solid = Array.Empty<byte>();
    public int[] PolyOfVoxel = Array.Empty<int>(); // CSR: the surface polygons of voxel i are PolyOfVoxel[i] .. PolyOfVoxel[i + 1] - 1
    public int[] PolyStart = Array.Empty<int>();   // polygon k has the points Points[PolyStart[k] .. PolyStart[k + 1] - 1]
    public Vector3F[] Points = Array.Empty<Vector3F>();
    public Vector3F[] PolyNormal = Array.Empty<Vector3F>(); // the outward unit normal of polygon k (as the triangle is wound)

    public int Count => Nx * Ny * Nz;

    public int Index(int x, int y, int z) => x + Nx * (y + Ny * z);

    public void Coords(int idx, out int x, out int y, out int z)
    {
        x = idx % Nx;
        int r = idx / Nx;
        y = r % Ny;
        z = r / Ny;
    }

    /// <summary>Voxelizes a clean triangle mesh. <paramref name="resolution"/> is the number of cells along the longest axis.</summary>
    public static VoxelModel Build(Vector3F[] v, int[] tri, int resolution)
    {
        double x0 = v[0].X, x1 = x0, y0 = v[0].Y, y1 = y0, z0 = v[0].Z, z1 = z0;
        for (int i = 1; i < v.Length; i++)
        {
            x0 = Math.Min(x0, v[i].X); x1 = Math.Max(x1, v[i].X);
            y0 = Math.Min(y0, v[i].Y); y1 = Math.Max(y1, v[i].Y);
            z0 = Math.Min(z0, v[i].Z); z1 = Math.Max(z1, v[i].Z);
        }

        double ext = Math.Max(x1 - x0, Math.Max(y1 - y0, z1 - z0));
        var m = new VoxelModel { H = ext / resolution };
        double h = m.H;
        // The mesh starts 1.5 cells inside the grid, so no axis-aligned face of a typical mesh lies on a grid plane and the border layers stay empty.
        m.Ox = x0 - 1.5 * h; m.Oy = y0 - 1.5 * h; m.Oz = z0 - 1.5 * h;
        m.Nx = (int)Math.Ceiling((x1 - x0) / h) + 4;
        m.Ny = (int)Math.Ceiling((y1 - y0) / h) + 4;
        m.Nz = (int)Math.Ceiling((z1 - z0) / h) + 4;
        int nvox = m.Count;
        m.Solid = new byte[nvox];

        var polyA = new double[24 * 3];
        var polyB = new double[24 * 3];
        var polyVox = new List<int>();
        var polyLen = new List<int>();
        var polyNrm = new List<Vector3F>();
        var pts = new List<Vector3F>();
        for (int t = 0; t + 2 < tri.Length; t += 3)
        {
            Vector3F a = v[tri[t]], b = v[tri[t + 1]], c = v[tri[t + 2]];
            double nx = ((double)b.Y - a.Y) * ((double)c.Z - a.Z) - ((double)b.Z - a.Z) * ((double)c.Y - a.Y);
            double ny = ((double)b.Z - a.Z) * ((double)c.X - a.X) - ((double)b.X - a.X) * ((double)c.Z - a.Z);
            double nz = ((double)b.X - a.X) * ((double)c.Y - a.Y) - ((double)b.Y - a.Y) * ((double)c.X - a.X);
            if (nx == 0 && ny == 0 && nz == 0) { continue; }
            double r = 0.5 * h * (Math.Abs(nx) + Math.Abs(ny) + Math.Abs(nz));
            double nlen = Math.Sqrt(nx * nx + ny * ny + nz * nz);
            var unit = new Vector3F((float)(nx / nlen), (float)(ny / nlen), (float)(nz / nlen));
            int i0 = Clamp((int)Math.Floor((Math.Min(a.X, Math.Min(b.X, c.X)) - m.Ox) / h), m.Nx);
            int i1 = Clamp((int)Math.Floor((Math.Max(a.X, Math.Max(b.X, c.X)) - m.Ox) / h), m.Nx);
            int j0 = Clamp((int)Math.Floor((Math.Min(a.Y, Math.Min(b.Y, c.Y)) - m.Oy) / h), m.Ny);
            int j1 = Clamp((int)Math.Floor((Math.Max(a.Y, Math.Max(b.Y, c.Y)) - m.Oy) / h), m.Ny);
            int k0 = Clamp((int)Math.Floor((Math.Min(a.Z, Math.Min(b.Z, c.Z)) - m.Oz) / h), m.Nz);
            int k1 = Clamp((int)Math.Floor((Math.Max(a.Z, Math.Max(b.Z, c.Z)) - m.Oz) / h), m.Nz);
            for (int k = k0; k <= k1; k++)
            {
                for (int j = j0; j <= j1; j++)
                {
                    for (int i = i0; i <= i1; i++)
                    {
                        double bx0 = m.Ox + i * h, by0 = m.Oy + j * h, bz0 = m.Oz + k * h;
                        double d = nx * (bx0 + 0.5 * h - a.X) + ny * (by0 + 0.5 * h - a.Y) + nz * (bz0 + 0.5 * h - a.Z);
                        if (Math.Abs(d) > r * (1 + 1e-12)) { continue; }
                        polyA[0] = a.X; polyA[1] = a.Y; polyA[2] = a.Z; polyA[3] = b.X; polyA[4] = b.Y; polyA[5] = b.Z; polyA[6] = c.X; polyA[7] = c.Y; polyA[8] = c.Z;
                        int n = ClipPolygon(polyA, 3, bx0, by0, bz0, bx0 + h, by0 + h, bz0 + h, polyB, out double[] res);
                        if (n == 0) { continue; }
                        int idx = m.Index(i, j, k);
                        m.Solid[idx] = Surface;
                        polyVox.Add(idx);
                        polyLen.Add(n);
                        polyNrm.Add(unit);
                        for (int q = 0; q < n; q++) { pts.Add(new Vector3F((float)res[3 * q], (float)res[3 * q + 1], (float)res[3 * q + 2])); }
                    }
                }
            }
        }

        // Counting sort of the polygons by voxel keeps the generation order inside a voxel.
        int npoly = polyVox.Count;
        var polyOffset = new int[npoly + 1];
        for (int p = 0; p < npoly; p++) { polyOffset[p + 1] = polyOffset[p] + polyLen[p]; }
        m.PolyOfVoxel = new int[nvox + 1];
        for (int p = 0; p < npoly; p++) { m.PolyOfVoxel[polyVox[p] + 1]++; }
        for (int i = 0; i < nvox; i++) { m.PolyOfVoxel[i + 1] += m.PolyOfVoxel[i]; }
        var fill = new int[nvox];
        var sortedPoly = new int[npoly];
        for (int p = 0; p < npoly; p++)
        {
            int idx = polyVox[p];
            sortedPoly[m.PolyOfVoxel[idx] + fill[idx]++] = p;
        }

        m.PolyStart = new int[npoly + 1];
        m.PolyNormal = new Vector3F[npoly];
        m.Points = new Vector3F[pts.Count];
        int w = 0;
        for (int s = 0; s < npoly; s++)
        {
            int p = sortedPoly[s];
            m.PolyStart[s] = w;
            m.PolyNormal[s] = polyNrm[p];
            for (int q = polyOffset[p]; q < polyOffset[p + 1]; q++) { m.Points[w++] = pts[q]; }
        }

        m.PolyStart[npoly] = w;

        // Fill: everything not reachable from the border through non-surface voxels is inside.
        var outside = new bool[nvox];
        var queue = new int[nvox];
        int head = 0, tail = 0;
        outside[0] = true;
        queue[tail++] = 0;
        while (head < tail)
        {
            int idx = queue[head++];
            m.Coords(idx, out int x, out int y, out int z);
            if (x > 0) { Visit(m, outside, queue, ref tail, idx - 1); }
            if (x + 1 < m.Nx) { Visit(m, outside, queue, ref tail, idx + 1); }
            if (y > 0) { Visit(m, outside, queue, ref tail, idx - m.Nx); }
            if (y + 1 < m.Ny) { Visit(m, outside, queue, ref tail, idx + m.Nx); }
            if (z > 0) { Visit(m, outside, queue, ref tail, idx - m.Nx * m.Ny); }
            if (z + 1 < m.Nz) { Visit(m, outside, queue, ref tail, idx + m.Nx * m.Ny); }
        }

        for (int i = 0; i < nvox; i++) { if (m.Solid[i] == Empty && !outside[i]) { m.Solid[i] = Interior; } }
        return m;
    }

    private static void Visit(VoxelModel m, bool[] outside, int[] queue, ref int tail, int idx)
    {
        if (outside[idx] || m.Solid[idx] != Empty) { return; }
        outside[idx] = true;
        queue[tail++] = idx;
    }

    private static int Clamp(int v, int n) => v < 0 ? 0 : (v >= n ? n - 1 : v);

    /// <summary>
    /// Sutherland-Hodgman clipping of a convex polygon to the closed box [lo, hi] (Sutherland and Hodgman 1974); <paramref name="cur"/> holds n points as x, y, z triples and must be at least 24 * 3 long,
    /// as must <paramref name="spare"/>. Returns the number of points left; they are in <paramref name="res"/>, one of the two buffers.
    /// </summary>
    public static int ClipPolygon(double[] cur, int n, double lox, double loy, double loz, double hix, double hiy, double hiz, double[] spare, out double[] res)
    {
        double[] nxt = spare;
        for (int plane = 0; plane < 6 && n > 0; plane++)
        {
            int axis = plane >> 1;
            bool upper = (plane & 1) == 1;
            double lim = upper ? (axis == 0 ? hix : axis == 1 ? hiy : hiz) : (axis == 0 ? lox : axis == 1 ? loy : loz);
            int m = 0;
            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n;
                double pi = cur[3 * i + axis], pj = cur[3 * j + axis];
                bool inI = upper ? pi <= lim : pi >= lim;
                bool inJ = upper ? pj <= lim : pj >= lim;
                if (inI)
                {
                    nxt[3 * m] = cur[3 * i]; nxt[3 * m + 1] = cur[3 * i + 1]; nxt[3 * m + 2] = cur[3 * i + 2];
                    m++;
                }

                if (inI != inJ)
                {
                    double t = (lim - pi) / (pj - pi);
                    for (int q = 0; q < 3; q++)
                    {
                        nxt[3 * m + q] = q == axis ? lim : cur[3 * i + q] + t * (cur[3 * j + q] - cur[3 * i + q]);
                    }

                    m++;
                }
            }

            (cur, nxt) = (nxt, cur);
            n = m;
        }

        res = cur;
        return n;
    }
}
