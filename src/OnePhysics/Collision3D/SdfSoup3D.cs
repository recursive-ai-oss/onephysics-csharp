using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>
/// A signed distance field seen as a triangle soup: the zero set of the trilinear interpolant, triangulated by marching tetrahedra (Doi and Koide 1991) with every cell cut into six
/// tetrahedra the same way. Triangle id = cell * 12 + tetrahedron * 2 + k, so ids never change and every cell's triangles can be built on demand. Queries that work on triangles (characters, particles,
/// rods, depenetration, shape casts) use this view; ray casts walk the field by sphere tracing and convex shapes collide with the field itself (see the narrowphase). It may move: it has mass properties.
/// </summary>
[Experimental("OPHYS0001")]
public sealed class SdfSoup3D : TriangleSoup3D
{
    // The six tetrahedra of a cube (Kuhn/Freudenthal split along the main diagonal); corner index = x + 2y + 4z.
    private static readonly int[] Tets = { 0, 1, 3, 7, 0, 1, 5, 7, 0, 2, 3, 7, 0, 2, 6, 7, 0, 4, 5, 7, 0, 4, 6, 7 };
    private readonly int _triangleCount;
    private MassPropertiesF? _mass;

    /// <summary>Wraps a field.</summary>
    public SdfSoup3D(SdfGrid3D grid)
    {
        ArgumentNullException.ThrowIfNull(grid);
        Grid = grid;
        int count = 0;
        for (int c = 0; c < CellCount; c++) { if (!Crosses(c)) continue; for (int k = 0; k < 12; k++) if (TryGetTriangle(12 * c + k, out _)) count++; }
        _triangleCount = count;
        SurfacePoints = BuildSurfacePoints();
    }

    /// <summary>Points on the zero set (cell centres of crossing cells moved onto the surface by two Newton steps) with the field's unit normal there, in the field's frame. At most 512, taken at an even stride in cell order. Used to test one field against another.</summary>
    internal (Vector3F Point, Vector3F Normal)[] SurfacePoints { get; }

    private (Vector3F, Vector3F)[] BuildSurfacePoints()
    {
        var g = Grid; var all = new List<(Vector3F, Vector3F)>(); int sx = g.SizeX - 1, sy = g.SizeY - 1; float h = g.CellSize;
        for (int c = 0; c < CellCount; c++)
        {
            if (!Crosses(c)) continue;
            int x = c % sx, y = c / sx % sy, z = c / (sx * sy);
            var p = new Vector3F(g.Origin.X + (x + 0.5f) * h, g.Origin.Y + (y + 0.5f) * h, g.Origin.Z + (z + 0.5f) * h);
            for (int it = 0; it < 2; it++) p -= g.Gradient(p) * g.Value(p);
            if (MathF.Abs(g.Value(p)) <= 0.25f * h) all.Add((p, g.Gradient(p)));
        }
        int stride = Math.Max(1, (all.Count + 511) / 512); var result = new List<(Vector3F, Vector3F)>();
        for (int i = 0; i < all.Count; i += stride) result.Add(all[i]);
        return result.ToArray();
    }

    /// <summary>The field.</summary>
    public SdfGrid3D Grid { get; }

    private int CellCount => (Grid.SizeX - 1) * (Grid.SizeY - 1) * (Grid.SizeZ - 1);

    /// <inheritdoc/>
    public override int TriangleIdCount => 12 * CellCount;
    /// <inheritdoc/>
    public override int TriangleCount => _triangleCount;
    /// <inheritdoc/>
    public override Bounds3F Bounds => Grid.Bounds;
    /// <summary>False: a field has mass properties, so it can be a moving body.</summary>
    public override bool IsStaticOnly => false;
    /// <summary>The solid's mass properties at unit density (see <see cref="SdfGrid3D.ComputeMass"/>); computed once.</summary>
    public override MassPropertiesF GetMassProperties() => _mass ??= Grid.ComputeMass();

    private float At(int x, int y, int z) => Grid.Samples[(z * Grid.SizeY + y) * Grid.SizeX + x];

    private bool Crosses(int cell)
    {
        int sx = Grid.SizeX - 1, sy = Grid.SizeY - 1; int x = cell % sx, y = cell / sx % sy, z = cell / (sx * sy);
        bool neg = false, pos = false;
        for (int k = 0; k < 8; k++) { float v = At(x + (k & 1), y + ((k >> 1) & 1), z + (k >> 2)); if (v < 0f) neg = true; else pos = true; }
        return neg && pos;
    }

    /// <inheritdoc/>
    public override bool TryGetTriangle(int id, out SoupTriangle3D triangle)
    {
        triangle = default;
        if ((uint)id >= (uint)TriangleIdCount) return false;
        int cell = id / 12, tet = id % 12 / 2, half = id & 1;
        int sx = Grid.SizeX - 1, sy = Grid.SizeY - 1; int x = cell % sx, y = cell / sx % sy, z = cell / (sx * sy);
        Span<float> v = stackalloc float[4]; Span<Vector3F> p = stackalloc Vector3F[4]; int inside = 0;
        for (int k = 0; k < 4; k++)
        {
            int corner = Tets[4 * tet + k];
            v[k] = At(x + (corner & 1), y + ((corner >> 1) & 1), z + (corner >> 2));
            p[k] = new Vector3F(Grid.Origin.X + (x + (corner & 1)) * Grid.CellSize, Grid.Origin.Y + (y + ((corner >> 1) & 1)) * Grid.CellSize, Grid.Origin.Z + (z + (corner >> 2)) * Grid.CellSize);
            if (v[k] < 0f) inside |= 1 << k;
        }
        if (inside == 0 || inside == 15) return false;
        int n = 0; for (int k = 0; k < 4; k++) if ((inside >> k & 1) != 0) n++;
        Vector3F a, b, c;
        if (n == 1 || n == 3)
        {
            if (half != 0) return false;
            int lone = -1; for (int k = 0; k < 4; k++) if (((inside >> k) & 1) == (n == 1 ? 1 : 0)) lone = k;
            Span<int> others = stackalloc int[3]; int o = 0; for (int k = 0; k < 4; k++) if (k != lone) others[o++] = k;
            a = Cut(p, v, lone, others[0]); b = Cut(p, v, lone, others[1]); c = Cut(p, v, lone, others[2]);
        }
        else
        {
            // Two inside, two outside: the cut is a quad through the four crossing edges, split into two triangles.
            Span<int> ins = stackalloc int[2]; Span<int> outs = stackalloc int[2]; int i0 = 0, o0 = 0;
            for (int k = 0; k < 4; k++) { if ((inside >> k & 1) != 0) ins[i0++] = k; else outs[o0++] = k; }
            var q0 = Cut(p, v, ins[0], outs[0]); var q1 = Cut(p, v, ins[0], outs[1]); var q2 = Cut(p, v, ins[1], outs[1]); var q3 = Cut(p, v, ins[1], outs[0]);
            if (half == 0) { a = q0; b = q1; c = q2; } else { a = q0; b = q2; c = q3; }
        }
        // Face towards the outside (the positive side).
        Vector3F inC = default, outC = default; int ni = 0, no = 0;
        for (int k = 0; k < 4; k++) { if ((inside >> k & 1) != 0) { inC += p[k]; ni++; } else { outC += p[k]; no++; } }
        var toOut = outC * (1f / no) - inC * (1f / ni); var nrm = Vector3F.Cross(b - a, c - a);
        if (Vector3F.Dot(nrm, toOut) < 0f) (b, c) = (c, b);
        if (Vector3F.Cross(b - a, c - a).LengthSquared() <= 0f) return false;
        triangle = new SoupTriangle3D(id, a, b, c, 7);
        return true;
    }

    private static Vector3F Cut(ReadOnlySpan<Vector3F> p, ReadOnlySpan<float> v, int i, int j)
    {
        float t = v[i] / (v[i] - v[j]);
        return p[i] + (p[j] - p[i]) * t;
    }

    internal override void BoxWalk(in Bounds3F box, List<int> ids)
    {
        var g = Grid; float h = g.CellSize; var o = g.Origin;
        int x0 = Math.Clamp((int)MathF.Floor((box.Min.X - o.X) / h) - 0, 0, g.SizeX - 2), x1 = Math.Clamp((int)MathF.Floor((box.Max.X - o.X) / h), 0, g.SizeX - 2);
        int y0 = Math.Clamp((int)MathF.Floor((box.Min.Y - o.Y) / h), 0, g.SizeY - 2), y1 = Math.Clamp((int)MathF.Floor((box.Max.Y - o.Y) / h), 0, g.SizeY - 2);
        int z0 = Math.Clamp((int)MathF.Floor((box.Min.Z - o.Z) / h), 0, g.SizeZ - 2), z1 = Math.Clamp((int)MathF.Floor((box.Max.Z - o.Z) / h), 0, g.SizeZ - 2);
        if (box.Max.X < o.X || box.Max.Y < o.Y || box.Max.Z < o.Z || box.Min.X > o.X + (g.SizeX - 1) * h || box.Min.Y > o.Y + (g.SizeY - 1) * h || box.Min.Z > o.Z + (g.SizeZ - 1) * h) return;
        for (int z = z0; z <= z1; z++) for (int y = y0; y <= y1; y++) for (int x = x0; x <= x1; x++)
        {
            int cell = (z * (g.SizeY - 1) + y) * (g.SizeX - 1) + x;
            if (!Crosses(cell)) continue;
            for (int k = 0; k < 12; k++)
                if (TryGetTriangle(12 * cell + k, out var t) && TriangleTests3D.BoxTriangle(box, t.V0, t.V1, t.V2)) ids.Add(12 * cell + k);
        }
    }

    internal override void RayWalk(in RayD ray, ref RayCollector collector)
    {
        // Sphere tracing in the field; each entry into the solid is offered as a hit on the triangle id of the cell that holds it. The ray's direction need not be unit.
        double len = Math.Sqrt(ray.Dx * ray.Dx + ray.Dy * ray.Dy + ray.Dz * ray.Dz); if (!(len > 0)) return;
        var g = Grid; double tol = 1e-4 * g.CellSize, k = 1.0 / Math.Max(1.0, g.Lipschitz);
        double at = 0, limit = collector.Limit * len; bool inside = false;
        var b = g.Bounds;
        if (!ray.HitBox(b.Min.X, b.Min.Y, b.Min.Z, b.Max.X, b.Max.Y, b.Max.Z, collector.Limit, out double enter)) return;
        at = enter * len;
        for (int i = 0; i < 4096 && at <= limit; i++)
        {
            var p = new Vector3F((float)(ray.Ox + ray.Dx * at / len), (float)(ray.Oy + ray.Dy * at / len), (float)(ray.Oz + ray.Dz * at / len));
            float v = g.Value(p);
            if (!inside && v <= tol)
            {
                double t = at / len; if (t <= collector.Limit) { collector.Offer(CellId(p), t, false); if (collector.Done) return; }
                inside = true; at += tol * 4; continue;
            }
            if (inside)
            {
                if (v > tol) { inside = false; double t = at / len; if (collector.Mode == 2 && t <= collector.Limit) collector.Offer(CellId(p), t, true); }
                at += Math.Max(Math.Abs(v) * k, tol * 4); continue;
            }
            at += Math.Max(v * k, tol * 0.25);
        }
    }

    // A real triangle near a point on the zero set: the first of its cell's triangles, else the first of a neighbouring cell's (the point is within a small tolerance of the surface).
    private int CellId(Vector3F p)
    {
        var g = Grid; float h = g.CellSize; int sx = g.SizeX - 1, sy = g.SizeY - 1, sz = g.SizeZ - 1;
        int cx = Math.Clamp((int)MathF.Floor((p.X - g.Origin.X) / h), 0, sx - 1), cy = Math.Clamp((int)MathF.Floor((p.Y - g.Origin.Y) / h), 0, sy - 1), cz = Math.Clamp((int)MathF.Floor((p.Z - g.Origin.Z) / h), 0, sz - 1);
        for (int r = 0; r <= 2; r++)
            for (int z = Math.Max(0, cz - r); z <= Math.Min(sz - 1, cz + r); z++) for (int y = Math.Max(0, cy - r); y <= Math.Min(sy - 1, cy + r); y++) for (int x = Math.Max(0, cx - r); x <= Math.Min(sx - 1, cx + r); x++)
            {
                int cell = (z * sy + y) * sx + x;
                for (int k = 0; k < 12; k++) if (TryGetTriangle(12 * cell + k, out _)) return 12 * cell + k;
            }
        return 12 * ((cz * sy + cy) * sx + cx);
    }
}
