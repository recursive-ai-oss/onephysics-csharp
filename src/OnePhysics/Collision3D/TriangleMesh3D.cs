using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Collision3D;

/// <summary>Options for <see cref="TriangleMesh3D.Create"/>. The default value chooses every setting automatically.</summary>
public readonly struct MeshBuildOptions
{
    /// <summary>
    /// The sine of the largest bend between two neighbouring triangles that still counts as flat, so the shared edge is inactive. 0 chooses 1e-3 (about 0.06 degrees); otherwise a value in (0, 1].
    /// </summary>
    public double FlatEdgeTolerance { get; init; }
}

/// <summary>
/// An immutable triangle mesh for static geometry: float32 vertices and triangle indices in the mesh's own frame, a static bounding volume hierarchy (<see cref="MeshBvh3D"/>) for queries, and
/// per-triangle active-edge flags so contacts on internal seams do not produce ghost bumps. Meshes are static-only: <see cref="TriangleSoup3D.GetMassProperties"/> throws.
/// <para>
/// Triangle ids are indices into the triangles that were kept; zero-area triangles are dropped at construction and listed in <see cref="DroppedTriangles"/>. Triangle winding defines the front side and should be consistent
/// (outward for closed bodies); active edges are decided from it. Vertices at the same position are treated as one when deciding which edges are shared, so a mesh with split vertices still gets its seams right.
/// </para>
/// </summary>
public sealed class TriangleMesh3D : TriangleSoup3D
{
    /// <summary>Largest coordinate magnitude accepted, 2^21.</summary>
    public const float MaxCoordinate = 2097152f;

    /// <summary>Largest vertex count and largest triangle count accepted, 2^26.</summary>
    public const int MaxElements = 1 << 26;

    private const double DefaultFlatTolerance = 1e-3;

    // A triangle whose doubled area is at most this fraction of its longest edge squared is a zero-area triangle: the cross product is rounding noise far below float32 resolution.
    private const double DegenerateRatio = 1e-10;

    private readonly Vector3F[] _vertices;
    private readonly int[] _indices;
    private readonly byte[] _flags;
    private readonly int[]? _source;
    private readonly int[] _dropped;
    private readonly MeshBvh3D _bvh;
    private readonly Bounds3F _bounds;

    private TriangleMesh3D(Vector3F[] vertices, int[] indices, byte[] flags, int[]? source, int[] dropped, MeshBvh3D bvh, Bounds3F bounds)
    {
        _vertices = vertices;
        _indices = indices;
        _flags = flags;
        _source = source;
        _dropped = dropped;
        _bvh = bvh;
        _bounds = bounds;
    }

    /// <inheritdoc/>
    public override int TriangleIdCount => _flags.Length;

    /// <inheritdoc/>
    public override int TriangleCount => _flags.Length;

    /// <inheritdoc/>
    public override Bounds3F Bounds => _bounds;

    /// <summary>The vertices as given (unreferenced vertices included).</summary>
    public ReadOnlySpan<Vector3F> Vertices => _vertices;

    /// <summary>The indices of the kept triangles, three per triangle, in the order of the input.</summary>
    public ReadOnlySpan<int> Indices => _indices;

    /// <summary>The input triangle numbers (index-array position divided by 3) of the triangles dropped as zero-area, ascending.</summary>
    public ReadOnlySpan<int> DroppedTriangles => _dropped;

    /// <summary>The number of nodes of the bounding volume hierarchy.</summary>
    public int BvhNodeCount => _bvh.NodeCount;

    /// <summary>The input triangle number of a kept triangle (the same as the id when nothing was dropped).</summary>
    /// <param name="id">A triangle id.</param>
    /// <exception cref="ArgumentOutOfRangeException">The id is not in range.</exception>
    public int SourceTriangle(int id)
    {
        if ((uint)id >= (uint)_flags.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(id), id, $"id is {id}; a triangle id in [0, {_flags.Length}) is expected.");
        }

        return _source is null ? id : _source[id];
    }

    /// <summary>The bounding volume hierarchy in its canonical byte form: node count, each node's six bound float bit patterns, offset and count, then the triangle order; little-endian. Identical on every platform for identical input.</summary>
    public byte[] ToBvhBytes() => _bvh.ToBytes();

    private static string F(float v) => v.ToString("R", CultureInfo.InvariantCulture);

    /// <summary>Builds a mesh.</summary>
    /// <param name="vertices">At least one vertex; every coordinate finite with magnitude at most <see cref="MaxCoordinate"/>.</param>
    /// <param name="indices">Three vertex indices per triangle, each in <c>[0, vertices.Length)</c>; at least one triangle, at most <see cref="MaxElements"/>.</param>
    /// <param name="options">Options; the default chooses everything.</param>
    /// <returns>The mesh. The same input gives the same mesh, bit for bit, on every platform.</returns>
    /// <exception cref="MeshBuildException">The input is empty, too large, not a whole number of triangles, has a non-finite or out-of-range coordinate or an index naming no vertex, or has only zero-area triangles; or the options are invalid. The message names the input and what was expected.</exception>
    public static TriangleMesh3D Create(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices, MeshBuildOptions options = default)
    {
        double tol = options.FlatEdgeTolerance;
        if (tol == 0.0)
        {
            tol = DefaultFlatTolerance;
        }
        else if (!(tol > 0.0 && tol <= 1.0))
        {
            throw new MeshBuildException(MeshBuildError.InvalidOptions, $"options.FlatEdgeTolerance is {tol.ToString("R", CultureInfo.InvariantCulture)}; 0 (automatic) or a value in (0, 1] is expected.", nameof(options));
        }

        if (vertices.Length == 0)
        {
            throw new MeshBuildException(MeshBuildError.NoVertices, "vertices is empty; at least one vertex is expected.", nameof(vertices));
        }

        if (indices.Length == 0)
        {
            throw new MeshBuildException(MeshBuildError.NoTriangles, "indices is empty; at least one triangle (three indices) is expected.", nameof(indices));
        }

        if (indices.Length % 3 != 0)
        {
            throw new MeshBuildException(MeshBuildError.InvalidIndexCount, $"indices has {indices.Length} entries; a multiple of 3 is expected.", nameof(indices));
        }

        if (vertices.Length > MaxElements || indices.Length / 3 > MaxElements)
        {
            throw new MeshBuildException(MeshBuildError.TooLarge, $"The mesh has {vertices.Length} vertices and {indices.Length / 3} triangles; at most {MaxElements} of each are supported.", nameof(vertices));
        }

        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3F v = vertices[i];
            for (int a = 0; a < 3; a++)
            {
                float c = v[a];
                if (!float.IsFinite(c))
                {
                    throw new MeshBuildException(MeshBuildError.NonFinite, $"vertices[{i}].{"XYZ"[a]} is {F(c)}; a finite coordinate is expected.", nameof(vertices));
                }

                if (Math.Abs(c) > MaxCoordinate)
                {
                    throw new MeshBuildException(MeshBuildError.OutOfRange, $"vertices[{i}].{"XYZ"[a]} is {F(c)}; a coordinate with magnitude at most {MaxCoordinate} is expected.", nameof(vertices));
                }
            }
        }

        for (int i = 0; i < indices.Length; i++)
        {
            if ((uint)indices[i] >= (uint)vertices.Length)
            {
                throw new MeshBuildException(MeshBuildError.IndexOutOfRange, $"indices[{i}] (corner {i % 3} of triangle {i / 3}) is {indices[i]}; an index in [0, {vertices.Length}) is expected.", nameof(indices));
            }
        }

        int inCount = indices.Length / 3;
        var kept = new List<int>(indices.Length);
        var dropped = new List<int>();
        for (int t = 0; t < inCount; t++)
        {
            Vector3D a = vertices[indices[3 * t]].ToDouble(), b = vertices[indices[(3 * t) + 1]].ToDouble(), c = vertices[indices[(3 * t) + 2]].ToDouble();
            Vector3D n = Vector3D.Cross(b - a, c - a);
            double longest = Math.Max((b - a).LengthSquared(), Math.Max((c - b).LengthSquared(), (a - c).LengthSquared()));
            double area2 = Math.Sqrt(n.LengthSquared());
            if (area2 > DegenerateRatio * longest && area2 > 0.0)
            {
                kept.Add(indices[3 * t]);
                kept.Add(indices[(3 * t) + 1]);
                kept.Add(indices[(3 * t) + 2]);
            }
            else
            {
                dropped.Add(t);
            }
        }

        if (kept.Count == 0)
        {
            throw new MeshBuildException(MeshBuildError.NoTriangles, $"All {inCount} triangles have zero area; at least one triangle with area is expected.", nameof(indices));
        }

        int count = kept.Count / 3;
        var vs = vertices.ToArray();
        int[] idx = kept.ToArray();
        int[]? source = null;
        if (dropped.Count > 0)
        {
            source = new int[count];
            int d = 0, k = 0;
            for (int t = 0; t < inCount; t++)
            {
                if (d < dropped.Count && dropped[d] == t)
                {
                    d++;
                }
                else
                {
                    source[k++] = t;
                }
            }
        }

        var tb = new float[6 * count];
        float x0 = float.PositiveInfinity, y0 = x0, z0 = x0, x1 = float.NegativeInfinity, y1 = x1, z1 = x1;
        for (int t = 0; t < count; t++)
        {
            Vector3F a = vs[idx[3 * t]], b = vs[idx[(3 * t) + 1]], c = vs[idx[(3 * t) + 2]];
            Vector3F lo = Vector3F.Min(a, Vector3F.Min(b, c)), hi = Vector3F.Max(a, Vector3F.Max(b, c));
            tb[6 * t] = lo.X;
            tb[(6 * t) + 1] = lo.Y;
            tb[(6 * t) + 2] = lo.Z;
            tb[(6 * t) + 3] = hi.X;
            tb[(6 * t) + 4] = hi.Y;
            tb[(6 * t) + 5] = hi.Z;
            x0 = Math.Min(x0, lo.X);
            y0 = Math.Min(y0, lo.Y);
            z0 = Math.Min(z0, lo.Z);
            x1 = Math.Max(x1, hi.X);
            y1 = Math.Max(y1, hi.Y);
            z1 = Math.Max(z1, hi.Z);
        }

        byte[] flags = ComputeActiveEdges(vs, idx, count, tol);
        MeshBvh3D bvh = MeshBvh3D.Build(tb, count);
        return new TriangleMesh3D(vs, idx, flags, source, dropped.ToArray(), bvh, new Bounds3F(new Vector3F(x0, y0, z0), new Vector3F(x1, y1, z1)));
    }

    // Edge adjacency by welded vertex identity (equal positions are one vertex): sort the directed edges by their undirected key; an edge with exactly two triangles running opposite ways
    // is active when it is convex; boundary, non-manifold and inconsistently wound edges are always active. The two-triangle case does not depend on the order within a group.
    private static byte[] ComputeActiveEdges(Vector3F[] vs, int[] idx, int count, double tol)
    {
        int[] byPos = new int[vs.Length];
        for (int i = 0; i < byPos.Length; i++)
        {
            byPos[i] = i;
        }

        Array.Sort(byPos, (p, q) =>
        {
            Vector3F a = vs[p], b = vs[q];
            return a.X != b.X ? a.X.CompareTo(b.X) : a.Y != b.Y ? a.Y.CompareTo(b.Y) : a.Z != b.Z ? a.Z.CompareTo(b.Z) : p.CompareTo(q);
        });
        int[] canon = new int[vs.Length];
        for (int i = 0; i < byPos.Length;)
        {
            int j = i + 1;
            Vector3F a = vs[byPos[i]];
            while (j < byPos.Length && vs[byPos[j]] == a)
            {
                j++;
            }

            for (int k = i; k < j; k++)
            {
                canon[byPos[k]] = byPos[i];
            }

            i = j;
        }

        var keys = new ulong[3 * count];
        var slots = new int[3 * count];
        for (int t = 0; t < count; t++)
        {
            for (int e = 0; e < 3; e++)
            {
                int u = canon[idx[(3 * t) + e]], v = canon[idx[(3 * t) + ((e + 1) % 3)]];
                keys[(3 * t) + e] = ((ulong)(uint)Math.Min(u, v) << 32) | (uint)Math.Max(u, v);
                slots[(3 * t) + e] = (3 * t) + e;
            }
        }

        Array.Sort(keys, slots);
        var flags = new byte[count];
        for (int t = 0; t < count; t++)
        {
            flags[t] = 7;
        }

        for (int i = 0; i + 1 < keys.Length; i++)
        {
            if (keys[i] != keys[i + 1] || (i + 2 < keys.Length && keys[i + 2] == keys[i]) || (i > 0 && keys[i - 1] == keys[i]))
            {
                continue;
            }

            int sa = slots[i], sb = slots[i + 1];
            int ta = sa / 3, ea = sa % 3, tbb = sb / 3, eb = sb % 3;
            int fromA = canon[idx[(3 * ta) + ea]], toA = canon[idx[(3 * ta) + ((ea + 1) % 3)]];
            int fromB = canon[idx[(3 * tbb) + eb]], toB = canon[idx[(3 * tbb) + ((eb + 1) % 3)]];
            if (fromA != toB || toA != fromB)
            {
                continue;
            }

            Vector3D a0 = vs[idx[3 * ta]].ToDouble(), a1 = vs[idx[(3 * ta) + 1]].ToDouble(), a2 = vs[idx[(3 * ta) + 2]].ToDouble();
            Vector3D b0 = vs[idx[3 * tbb]].ToDouble(), b1 = vs[idx[(3 * tbb) + 1]].ToDouble(), b2 = vs[idx[(3 * tbb) + 2]].ToDouble();
            if (!TriangleTests3D.IsActive(a0, a1, a2, ea, b0, b1, b2, tol))
            {
                flags[ta] &= (byte)~(1 << ea);
                flags[tbb] &= (byte)~(1 << eb);
            }
        }

        return flags;
    }

    /// <inheritdoc/>
    public override bool TryGetTriangle(int id, out SoupTriangle3D triangle)
    {
        if ((uint)id >= (uint)_flags.Length)
        {
            triangle = default;
            return false;
        }

        triangle = new SoupTriangle3D(id, _vertices[_indices[3 * id]], _vertices[_indices[(3 * id) + 1]], _vertices[_indices[(3 * id) + 2]], _flags[id]);
        return true;
    }

    internal override void RayWalk(in RayD ray, ref RayCollector c)
    {
        BvhNode[] nodes = _bvh.Nodes;
        int[] order = _bvh.Order;
        Span<int> stack = stackalloc int[128];
        Span<double> enters = stackalloc double[128];
        BvhNode root = nodes[0];
        if (!ray.HitBox(root.MinX, root.MinY, root.MinZ, root.MaxX, root.MaxY, root.MaxZ, c.Limit, out double e0))
        {
            return;
        }

        int sp = 0;
        stack[sp] = 0;
        enters[sp++] = e0;
        Vector3D o = ray.Origin, d = ray.Direction;
        while (sp > 0)
        {
            sp--;
            int ni = stack[sp];
            if (enters[sp] > c.Limit)
            {
                continue;
            }

            BvhNode n = nodes[ni];
            if (n.Count > 0)
            {
                for (int k = n.Offset; k < n.Offset + n.Count; k++)
                {
                    int id = order[k];
                    int i3 = 3 * id;
                    if (TriangleTests3D.RayTri(o, d, _vertices[_indices[i3]].ToDouble(), _vertices[_indices[i3 + 1]].ToDouble(), _vertices[_indices[i3 + 2]].ToDouble(), c.CullBack, out double t, out bool back))
                    {
                        c.Offer(id, t, back);
                        if (c.Done)
                        {
                            return;
                        }
                    }
                }

                continue;
            }

            int l = ni + 1, r = n.Offset;
            BvhNode ln = nodes[l], rn = nodes[r];
            bool hl = ray.HitBox(ln.MinX, ln.MinY, ln.MinZ, ln.MaxX, ln.MaxY, ln.MaxZ, c.Limit, out double tl);
            bool hr = ray.HitBox(rn.MinX, rn.MinY, rn.MinZ, rn.MaxX, rn.MaxY, rn.MaxZ, c.Limit, out double tr);
            // The nearer child is visited first (the left one on equal entry fractions), so a closest-hit query tightens its limit early.
            if (hl && hr)
            {
                if (tr < tl)
                {
                    stack[sp] = l;
                    enters[sp++] = tl;
                    stack[sp] = r;
                    enters[sp++] = tr;
                }
                else
                {
                    stack[sp] = r;
                    enters[sp++] = tr;
                    stack[sp] = l;
                    enters[sp++] = tl;
                }
            }
            else if (hl)
            {
                stack[sp] = l;
                enters[sp++] = tl;
            }
            else if (hr)
            {
                stack[sp] = r;
                enters[sp++] = tr;
            }
        }
    }

    internal override void BoxWalk(in Bounds3F box, List<int> ids)
    {
        BvhNode[] nodes = _bvh.Nodes;
        int[] order = _bvh.Order;
        Span<int> stack = stackalloc int[128];
        int sp = 0;
        stack[sp++] = 0;
        while (sp > 0)
        {
            int ni = stack[--sp];
            BvhNode n = nodes[ni];
            if (!(n.MinX <= box.Max.X && box.Min.X <= n.MaxX && n.MinY <= box.Max.Y && box.Min.Y <= n.MaxY && n.MinZ <= box.Max.Z && box.Min.Z <= n.MaxZ))
            {
                continue;
            }

            if (n.Count > 0)
            {
                for (int k = n.Offset; k < n.Offset + n.Count; k++)
                {
                    int id = order[k];
                    int i3 = 3 * id;
                    if (TriangleTests3D.BoxTriangle(box, _vertices[_indices[i3]], _vertices[_indices[i3 + 1]], _vertices[_indices[i3 + 2]]))
                    {
                        ids.Add(id);
                    }
                }

                continue;
            }

            stack[sp++] = n.Offset;
            stack[sp++] = ni + 1;
        }
    }
}
