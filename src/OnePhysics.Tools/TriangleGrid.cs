using System;
using RecursiveAI.OneMath.Predicates;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>
/// A uniform grid of triangles for two queries: nearest point on the surface, and the parity of the crossings of a ray along +x (inside or outside a closed surface).
/// Triangles are listed in every cell their bounding box touches, in triangle order, so every query visits candidates in a fixed order.
/// </summary>
internal sealed class TriangleGrid
{
    private readonly double[] _x, _y, _z;
    private readonly int[] _tri;
    private readonly int[] _start;
    private readonly int[] _list;
    private readonly int[] _firstCellX;   // per triangle: the lowest cell index along x its bounding box touches

    public readonly double Ox, Oy, Oz, Cell;
    public readonly int Nx, Ny, Nz;

    public int TriangleCount => _tri.Length / 3;

    /// <summary>The most cell-list entries a grid may hold.</summary>
    public const long MaxEntries = 1L << 27;

    public TriangleGrid(double[] x, double[] y, double[] z, int[] tri, double ox, double oy, double oz, double cell, int nx, int ny, int nz)
    {
        _x = x; _y = y; _z = z; _tri = tri;
        Ox = ox; Oy = oy; Oz = oz; Cell = cell; Nx = nx; Ny = ny; Nz = nz;
        int nt = tri.Length / 3;
        long cells = (long)nx * ny * nz;
        _start = new int[cells + 1];
        _firstCellX = new int[nt];
        long entries = 0;
        for (int t = 0; t < nt; t++)
        {
            Range(t, out int i0, out int i1, out int j0, out int j1, out int k0, out int k1);
            _firstCellX[t] = i0;
            entries += (long)(i1 - i0 + 1) * (j1 - j0 + 1) * (k1 - k0 + 1);
            if (entries > MaxEntries) { throw new GeometryToolException(GeometryToolError.OutOfRange, "the mesh covers too many grid cells (more than " + MeshInput.Inv((int)(MaxEntries >> 20)) + " million entries); use a larger edge length", null); }
            for (int k = k0; k <= k1; k++) { for (int j = j0; j <= j1; j++) { for (int i = i0; i <= i1; i++) { _start[CellIndex(i, j, k) + 1]++; } } }
        }

        for (long c = 0; c < cells; c++) { _start[c + 1] += _start[c]; }
        _list = new int[entries];
        var cursor = new int[cells];
        Array.Copy(_start, cursor, cells);
        for (int t = 0; t < nt; t++)
        {
            Range(t, out int i0, out int i1, out int j0, out int j1, out int k0, out int k1);
            for (int k = k0; k <= k1; k++) { for (int j = j0; j <= j1; j++) { for (int i = i0; i <= i1; i++) { _list[cursor[CellIndex(i, j, k)]++] = t; } } }
        }
    }

    public int CellIndex(int i, int j, int k) => i + Nx * (j + Ny * k);

    public bool CellHasTriangles(int i, int j, int k)
    {
        int c = CellIndex(i, j, k);
        return _start[c + 1] > _start[c];
    }

    private void Range(int t, out int i0, out int i1, out int j0, out int j1, out int k0, out int k1)
    {
        int a = _tri[3 * t], b = _tri[3 * t + 1], c = _tri[3 * t + 2];
        i0 = Clamp((Math.Min(_x[a], Math.Min(_x[b], _x[c])) - Ox) / Cell, Nx);
        i1 = Clamp((Math.Max(_x[a], Math.Max(_x[b], _x[c])) - Ox) / Cell, Nx);
        j0 = Clamp((Math.Min(_y[a], Math.Min(_y[b], _y[c])) - Oy) / Cell, Ny);
        j1 = Clamp((Math.Max(_y[a], Math.Max(_y[b], _y[c])) - Oy) / Cell, Ny);
        k0 = Clamp((Math.Min(_z[a], Math.Min(_z[b], _z[c])) - Oz) / Cell, Nz);
        k1 = Clamp((Math.Max(_z[a], Math.Max(_z[b], _z[c])) - Oz) / Cell, Nz);
    }

    private static int Clamp(double v, int n)
    {
        if (!(v > 0)) { return 0; }
        if (v >= n) { return n - 1; }
        return (int)v;
    }

    public int CellOf(double p, double o, int n) => Clamp((p - o) / Cell, n);

    // Is the point inside the closed surface? Crossings of the ray from p along +x are counted; the ray is nudged by an infinitesimal (e, e^2) in (y, z), so a ray through an edge or a vertex
    // counts each of the triangles around it consistently (Edelsbrunner and Muecke, simulation of simplicity).
    public bool Inside(double px, double py, double pz)
    {
        int ci = CellOf(px, Ox, Nx), cj = CellOf(py, Oy, Ny), ck = CellOf(pz, Oz, Nz);
        int crossings = 0;
        for (int i = ci; i < Nx; i++)
        {
            int c = CellIndex(i, cj, ck);
            for (int e = _start[c]; e < _start[c + 1]; e++)
            {
                int t = _list[e];
                if (i != Math.Max(ci, _firstCellX[t])) { continue; }
                if (RayCrosses(t, px, py, pz)) { crossings++; }
            }
        }

        return (crossings & 1) != 0;
    }

    private bool RayCrosses(int t, double px, double py, double pz)
    {
        int a = _tri[3 * t], b = _tri[3 * t + 1], c = _tri[3 * t + 2];
        double ay = _y[a], az = _z[a], by = _y[b], bz = _z[b], cy = _y[c], cz = _z[c];
        int s1 = Side(ay, az, by, bz, py, pz), s2 = Side(by, bz, cy, cz, py, pz), s3 = Side(cy, cz, ay, az, py, pz);
        if (s1 != s2 || s2 != s3) { return false; }
        double ux = _x[b] - _x[a], uy = by - ay, uz = bz - az, vx = _x[c] - _x[a], vy = cy - ay, vz = cz - az;
        double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        if (nx == 0) { return false; }
        double xc = _x[a] - (ny * (py - ay) + nz * (pz - az)) / nx;
        return xc > px;
    }

    // Side of the directed edge (a, b) the nudged point lies on, +1 or -1.
    private static int Side(double ay, double az, double by, double bz, double py, double pz)
    {
        int o = ExactPredicates.Orient2D(ay, az, by, bz, py, pz);
        if (o != 0) { return o; }
        // The determinant's change when p moves by (e, e^2): e * (az - bz) + e^2 * (by - ay).
        if (az != bz) { return az > bz ? 1 : -1; }
        if (by != ay) { return by > ay ? 1 : -1; }
        return 1;
    }

    /// <summary>The point of the surface nearest to p, found among cells within <paramref name="maxRings"/> rings of p's cell. Returns false when nothing was found within the rings that were searched to completion.</summary>
    public bool Nearest(double px, double py, double pz, int maxRings, out int triangle, out double qx, out double qy, out double qz, out double dist2)
    {
        triangle = -1; qx = qy = qz = 0; dist2 = double.PositiveInfinity;
        int ci = CellOf(px, Ox, Nx), cj = CellOf(py, Oy, Ny), ck = CellOf(pz, Oz, Nz);
        for (int r = 0; r <= maxRings; r++)
        {
            int k0 = Math.Max(0, ck - r), k1 = Math.Min(Nz - 1, ck + r), j0 = Math.Max(0, cj - r), j1 = Math.Min(Ny - 1, cj + r), i0 = Math.Max(0, ci - r), i1 = Math.Min(Nx - 1, ci + r);
            for (int k = k0; k <= k1; k++)
            {
                for (int j = j0; j <= j1; j++)
                {
                    for (int i = i0; i <= i1; i++)
                    {
                        // Only the shell of the block: cells at Chebyshev distance exactly r.
                        if (Math.Max(Math.Abs(i - ci), Math.Max(Math.Abs(j - cj), Math.Abs(k - ck))) != r) { continue; }
                        int c = CellIndex(i, j, k);
                        for (int e = _start[c]; e < _start[c + 1]; e++)
                        {
                            int t = _list[e];
                            ClosestOnTriangle(t, px, py, pz, out double cx, out double cy, out double cz);
                            double dx = cx - px, dy = cy - py, dz = cz - pz, d2 = dx * dx + dy * dy + dz * dz;
                            if (d2 < dist2) { dist2 = d2; triangle = t; qx = cx; qy = cy; qz = cz; }
                        }
                    }
                }
            }

            // Everything within r cells of the point has been searched (the point lies inside its own cell).
            double reach = r * Cell;
            if (triangle >= 0 && dist2 <= reach * reach) { return true; }
        }

        return triangle >= 0;
    }

    /// <summary>The closest point of triangle t to p (Ericson, Real-Time Collision Detection, 5.1.5).</summary>
    public void ClosestOnTriangle(int t, double px, double py, double pz, out double qx, out double qy, out double qz)
    {
        int ia = _tri[3 * t], ib = _tri[3 * t + 1], ic = _tri[3 * t + 2];
        double ax = _x[ia], ay = _y[ia], az = _z[ia], bx = _x[ib], by = _y[ib], bz = _z[ib], cx = _x[ic], cy = _y[ic], cz = _z[ic];
        Barycentric(ax, ay, az, bx, by, bz, cx, cy, cz, px, py, pz, out double u, out double v, out double w);
        qx = u * ax + v * bx + w * cx; qy = u * ay + v * by + w * cy; qz = u * az + v * bz + w * cz;
    }

    /// <summary>The barycentric weights (u, v, w) of the closest point of triangle abc to p.</summary>
    public static void Barycentric(double ax, double ay, double az, double bx, double by, double bz, double cx, double cy, double cz, double px, double py, double pz, out double u, out double v, out double w)
    {
        double abx = bx - ax, aby = by - ay, abz = bz - az, acx = cx - ax, acy = cy - ay, acz = cz - az;
        double apx = px - ax, apy = py - ay, apz = pz - az;
        double d1 = abx * apx + aby * apy + abz * apz, d2 = acx * apx + acy * apy + acz * apz;
        if (d1 <= 0 && d2 <= 0) { u = 1; v = 0; w = 0; return; }
        double bpx = px - bx, bpy = py - by, bpz = pz - bz;
        double d3 = abx * bpx + aby * bpy + abz * bpz, d4 = acx * bpx + acy * bpy + acz * bpz;
        if (d3 >= 0 && d4 <= d3) { u = 0; v = 1; w = 0; return; }
        double vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0) { double s = d1 / (d1 - d3); u = 1 - s; v = s; w = 0; return; }
        double cpx = px - cx, cpy = py - cy, cpz = pz - cz;
        double d5 = abx * cpx + aby * cpy + abz * cpz, d6 = acx * cpx + acy * cpy + acz * cpz;
        if (d6 >= 0 && d5 <= d6) { u = 0; v = 0; w = 1; return; }
        double vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0) { double s = d2 / (d2 - d6); u = 1 - s; v = 0; w = s; return; }
        double va = d3 * d6 - d5 * d4;
        if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) { double s = (d4 - d3) / ((d4 - d3) + (d5 - d6)); u = 0; v = 1 - s; w = s; return; }
        double denom = 1.0 / (va + vb + vc);
        v = vb * denom; w = vc * denom; u = 1.0 - v - w;
    }
}
