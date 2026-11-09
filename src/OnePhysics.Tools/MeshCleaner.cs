using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>Options for <see cref="MeshCleaner.Clean"/>. The default (all zeros and false) welds, removes degenerate and duplicate triangles, orients every surface consistently and drops unused vertices.</summary>
public readonly struct MeshCleanOptions
{
    /// <summary>Vertices closer than this are welded into the one with the lowest index, in the vertices' units. 0 chooses 10^-6 times the mesh's longest axis extent; otherwise it must be finite and not negative. To weld only identical positions pass a tiny positive value.</summary>
    public float WeldTolerance { get; init; }

    /// <summary>Keep triangles that repeat a vertex after welding, have zero area, or whose smallest height is within the weld tolerance (slivers). Default false: they are removed.</summary>
    public bool KeepDegenerateTriangles { get; init; }

    /// <summary>Keep every copy of triangles that share the same three vertices (in either winding). Default false: only the first is kept.</summary>
    public bool KeepDuplicateTriangles { get; init; }

    /// <summary>Leave the winding as given. Default false: triangles are flipped so that neighbours agree, and every closed surface points outward.</summary>
    public bool KeepWinding { get; init; }

    /// <summary>Keep vertices no triangle uses (after welding). Default false: they are dropped.</summary>
    public bool KeepUnreferencedVertices { get; init; }
}

/// <summary>An edge shared by a number of triangles other than two.</summary>
public readonly struct MeshEdge
{
    internal MeshEdge(int a, int b, int triangleCount)
    {
        A = a;
        B = b;
        TriangleCount = triangleCount;
    }

    /// <summary>The lower vertex index of the edge.</summary>
    public int A { get; }

    /// <summary>The higher vertex index of the edge.</summary>
    public int B { get; }

    /// <summary>How many triangles use the edge.</summary>
    public int TriangleCount { get; }
}

/// <summary>What <see cref="MeshCleaner.Clean"/> found and changed. Counts of problems refer to the cleaned mesh unless the name says removed.</summary>
public sealed class MeshCleanReport
{
    internal MeshCleanReport() { }

    /// <summary>The weld tolerance used.</summary>
    public double WeldToleranceUsed { get; internal set; }

    /// <summary>Vertices merged into another vertex.</summary>
    public int WeldedVertices { get; internal set; }

    /// <summary>Vertices dropped because no remaining triangle used them.</summary>
    public int UnreferencedVerticesRemoved { get; internal set; }

    /// <summary>Triangles removed as degenerate (repeated vertex, zero area or sliver).</summary>
    public int DegenerateTrianglesRemoved { get; internal set; }

    /// <summary>Triangles removed as copies of an earlier triangle.</summary>
    public int DuplicateTrianglesRemoved { get; internal set; }

    /// <summary>Triangles whose winding was reversed.</summary>
    public int FlippedTriangles { get; internal set; }

    /// <summary>Connected pieces of the surface, linked across edges that exactly two triangles share.</summary>
    public int Components { get; internal set; }

    /// <summary>Edges used by more than two triangles, in ascending vertex-pair order. Such edges are not crossed when orienting.</summary>
    public IReadOnlyList<MeshEdge> NonManifoldEdges { get; internal set; } = Array.Empty<MeshEdge>();

    /// <summary>Edges used by exactly one triangle.</summary>
    public int BoundaryEdgeCount { get; internal set; }

    /// <summary>Holes: connected groups of boundary edges (two holes that touch at a vertex count once).</summary>
    public int HoleCount { get; internal set; }

    /// <summary>True when the mesh has no boundary and no non-manifold edge, so it encloses a volume.</summary>
    public bool IsClosed => BoundaryEdgeCount == 0 && NonManifoldEdges.Count == 0;
}

/// <summary>A cleaned mesh and the report on it.</summary>
public sealed class CleanedMesh
{
    internal CleanedMesh(Vector3F[] vertices, int[] indices, MeshCleanReport report)
    {
        Vertices = vertices;
        Indices = indices;
        Report = report;
    }

    /// <summary>The surviving vertices, in the order of their original indices.</summary>
    public Vector3F[] Vertices { get; }

    /// <summary>Triangle corner indices into <see cref="Vertices"/>, three per triangle, in the order of the surviving input triangles.</summary>
    public int[] Indices { get; }

    /// <summary>What was found and changed.</summary>
    public MeshCleanReport Report { get; }
}

/// <summary>Welds vertices, removes degenerate and duplicate triangles, orients surfaces consistently and reports non-manifold edges and holes. Cleaning a cleaned mesh changes nothing.</summary>
public static class MeshCleaner
{
    /// <summary>Cleans a triangle mesh. The input arrays are not modified.</summary>
    /// <param name="vertices">Mesh vertices, finite, magnitude at most 2^21; at most 2^24.</param>
    /// <param name="indices">Triangle corner indices, three per triangle (may be empty).</param>
    /// <param name="options">What to do; the default does everything.</param>
    /// <returns>The cleaned mesh and the report. Welding links every pair of vertices within the tolerance, so a chain of close vertices collapses into one; the lowest index of a group keeps its position. Identical input and options give identical output on every platform.</returns>
    /// <exception cref="GeometryToolException">A vertex is NaN, infinite or out of range, an index is out of range, the counts are invalid, or the tolerance is invalid.</exception>
    public static CleanedMesh Clean(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices, MeshCleanOptions options = default)
    {
        MeshInput.Check(vertices, indices, requireTriangles: false);
        float wt = options.WeldTolerance;
        if (float.IsNaN(wt) || float.IsInfinity(wt) || wt < 0f)
        {
            throw new GeometryToolException(GeometryToolError.InvalidOptions, "WeldTolerance = " + wt + " must be finite and not negative", nameof(options));
        }

        var report = new MeshCleanReport();
        int nv = vertices.Length;
        double tol = wt != 0f ? wt : 1e-6 * MeshInput.Extent(vertices);
        report.WeldToleranceUsed = tol;

        // Weld: sweep along x and link every pair within the tolerance; the root of a group is its lowest index.
        int[] root = new int[nv];
        for (int i = 0; i < nv; i++) { root[i] = i; }
        var order = new int[nv];
        for (int i = 0; i < nv; i++) { order[i] = i; }
        var vcopy = vertices.ToArray();
        Array.Sort(order, (p, q) =>
        {
            int c = vcopy[p].X.CompareTo(vcopy[q].X);
            return c != 0 ? c : p.CompareTo(q);
        });
        double tol2 = tol * tol;
        for (int oi = 0; oi < nv; oi++)
        {
            int i = order[oi];
            for (int oj = oi + 1; oj < nv; oj++)
            {
                int j = order[oj];
                double dx = (double)vcopy[j].X - vcopy[i].X;
                if (dx > tol) { break; }
                double dy = (double)vcopy[j].Y - vcopy[i].Y, dz = (double)vcopy[j].Z - vcopy[i].Z;
                if (dx * dx + dy * dy + dz * dz <= tol2) { Union(root, i, j); }
            }
        }

        for (int i = 0; i < nv; i++) { root[i] = Find(root, i); }
        int rootCount = 0;
        for (int i = 0; i < nv; i++) { if (root[i] == i) { rootCount++; } }
        report.WeldedVertices = nv - rootCount;

        // Remap triangles; drop degenerate ones.
        int nt = indices.Length / 3;
        var tri = new int[nt * 3];
        var alive = new bool[nt];
        int degenerate = 0;
        for (int t = 0; t < nt; t++)
        {
            int a = root[indices[3 * t]], b = root[indices[3 * t + 1]], c = root[indices[3 * t + 2]];
            tri[3 * t] = a; tri[3 * t + 1] = b; tri[3 * t + 2] = c;
            alive[t] = true;
            if (!options.KeepDegenerateTriangles && IsDegenerate(vcopy, a, b, c, tol))
            {
                alive[t] = false;
                degenerate++;
            }
        }

        report.DegenerateTrianglesRemoved = degenerate;

        // Duplicates: the first triangle (by index) with a given vertex set stays.
        if (!options.KeepDuplicateTriangles)
        {
            var keyed = new List<(int a, int b, int c, int t)>();
            for (int t = 0; t < nt; t++)
            {
                if (!alive[t]) { continue; }
                int a = tri[3 * t], b = tri[3 * t + 1], c = tri[3 * t + 2];
                Sort3(ref a, ref b, ref c);
                keyed.Add((a, b, c, t));
            }

            keyed.Sort((x, y) =>
            {
                int d = x.a.CompareTo(y.a);
                if (d != 0) { return d; }
                d = x.b.CompareTo(y.b);
                if (d != 0) { return d; }
                d = x.c.CompareTo(y.c);
                return d != 0 ? d : x.t.CompareTo(y.t);
            });
            int dup = 0;
            for (int i = 1; i < keyed.Count; i++)
            {
                if (keyed[i].a == keyed[i - 1].a && keyed[i].b == keyed[i - 1].b && keyed[i].c == keyed[i - 1].c)
                {
                    alive[keyed[i].t] = false;
                    dup++;
                }
            }

            report.DuplicateTrianglesRemoved = dup;
        }

        // Compact the vertices (roots, or only referenced roots) and the triangles.
        var used = new bool[nv];
        int outTris = 0;
        for (int t = 0; t < nt; t++)
        {
            if (!alive[t]) { continue; }
            outTris++;
            used[tri[3 * t]] = used[tri[3 * t + 1]] = used[tri[3 * t + 2]] = true;
        }

        var newIndex = new int[nv];
        int outVerts = 0;
        for (int i = 0; i < nv; i++)
        {
            bool keep = root[i] == i && (options.KeepUnreferencedVertices || used[i]);
            newIndex[i] = keep ? outVerts++ : -1;
        }

        report.UnreferencedVerticesRemoved = rootCount - outVerts;
        var ov = new Vector3F[outVerts];
        for (int i = 0; i < nv; i++) { if (newIndex[i] >= 0) { ov[newIndex[i]] = vcopy[i]; } }
        var oi2 = new int[outTris * 3];
        int w = 0;
        for (int t = 0; t < nt; t++)
        {
            if (!alive[t]) { continue; }
            oi2[w++] = newIndex[tri[3 * t]];
            oi2[w++] = newIndex[tri[3 * t + 1]];
            oi2[w++] = newIndex[tri[3 * t + 2]];
        }

        Analyse(ov, oi2, !options.KeepWinding, report);
        return new CleanedMesh(ov, oi2, report);
    }

    private static int Find(int[] p, int i)
    {
        while (p[i] != i)
        {
            p[i] = p[p[i]];
            i = p[i];
        }

        return i;
    }

    private static void Union(int[] p, int a, int b)
    {
        a = Find(p, a);
        b = Find(p, b);
        if (a == b) { return; }
        if (a < b) { p[b] = a; } else { p[a] = b; }
    }

    private static void Sort3(ref int a, ref int b, ref int c)
    {
        if (a > b) { (a, b) = (b, a); }
        if (b > c) { (b, c) = (c, b); }
        if (a > b) { (a, b) = (b, a); }
    }

    private static bool IsDegenerate(Vector3F[] v, int a, int b, int c, double tol)
    {
        if (a == b || b == c || a == c) { return true; }
        double ux = (double)v[b].X - v[a].X, uy = (double)v[b].Y - v[a].Y, uz = (double)v[b].Z - v[a].Z;
        double wx = (double)v[c].X - v[a].X, wy = (double)v[c].Y - v[a].Y, wz = (double)v[c].Z - v[a].Z;
        double cx = uy * wz - uz * wy, cy = uz * wx - ux * wz, cz = ux * wy - uy * wx;
        double twiceArea2 = cx * cx + cy * cy + cz * cz;
        if (twiceArea2 == 0) { return true; }
        if (tol <= 0) { return false; }
        double e0 = ux * ux + uy * uy + uz * uz, e1 = wx * wx + wy * wy + wz * wz;
        double sx = (double)v[c].X - v[b].X, sy = (double)v[c].Y - v[b].Y, sz = (double)v[c].Z - v[b].Z;
        double e2 = sx * sx + sy * sy + sz * sz;
        double longest2 = Math.Max(e0, Math.Max(e1, e2));
        // Smallest height = 2 * area / longest edge; compare squared.
        return twiceArea2 <= tol * tol * longest2;
    }

    // Edge records sorted by (low, high, triangle): lets one pass find shared, boundary and non-manifold edges.
    private static void Analyse(Vector3F[] v, int[] tri, bool orient, MeshCleanReport report)
    {
        int nt = tri.Length / 3;
        var edges = new (long key, int t, bool forward)[nt * 3];
        for (int t = 0; t < nt; t++)
        {
            for (int k = 0; k < 3; k++)
            {
                int p = tri[3 * t + k], q = tri[3 * t + (k + 1) % 3];
                bool fwd = p < q;
                int lo = fwd ? p : q, hi = fwd ? q : p;
                edges[3 * t + k] = (((long)lo << 32) | (uint)hi, t, fwd);
            }
        }

        Array.Sort(edges, (x, y) =>
        {
            int c = x.key.CompareTo(y.key);
            return c != 0 ? c : x.t.CompareTo(y.t);
        });

        // Neighbour links across edges shared by exactly two triangles: link[3*t+slot] = other triangle, with whether the two use the edge in the same direction.
        var nbr = new int[nt * 3];
        var nbrSame = new bool[nt * 3];
        var slotCount = new int[nt];
        var nonManifold = new List<MeshEdge>();
        int boundary = 0;
        var bp = new List<(int a, int b)>();
        for (int i = 0; i < edges.Length;)
        {
            int j = i;
            while (j < edges.Length && edges[j].key == edges[i].key) { j++; }
            int count = j - i;
            int lo = (int)(edges[i].key >> 32), hi = (int)(edges[i].key & 0xFFFFFFFF);
            if (count == 1) { boundary++; bp.Add((lo, hi)); }
            else if (count == 2)
            {
                int t0 = edges[i].t, t1 = edges[i + 1].t;
                bool same = edges[i].forward == edges[i + 1].forward;
                if (t0 != t1)
                {
                    nbr[3 * t0 + slotCount[t0]] = t1; nbrSame[3 * t0 + slotCount[t0]++] = same;
                    nbr[3 * t1 + slotCount[t1]] = t0; nbrSame[3 * t1 + slotCount[t1]++] = same;
                }
            }
            else { nonManifold.Add(new MeshEdge(lo, hi, count)); }
            i = j;
        }

        // Components and orientation by breadth-first search from the lowest unvisited triangle.
        var comp = new int[nt];
        Array.Fill(comp, -1);
        var flip = new bool[nt];
        var queue = new int[nt];
        int components = 0, flipped = 0;
        var compEdgeProblem = new List<bool>();
        for (int seed = 0; seed < nt; seed++)
        {
            if (comp[seed] >= 0) { continue; }
            int head = 0, tail = 0;
            int firstInQueue = tail;
            queue[tail++] = seed;
            comp[seed] = components;
            while (head < tail)
            {
                int t = queue[head++];
                for (int s = 0; s < slotCount[t]; s++)
                {
                    int u = nbr[3 * t + s];
                    if (comp[u] >= 0) { continue; }
                    comp[u] = components;
                    if (orient) { flip[u] = nbrSame[3 * t + s] ? !flip[t] : flip[t]; }
                    queue[tail++] = u;
                }
            }

            if (orient)
            {
                // A component that is a closed manifold must enclose positive volume.
                bool closed = true;
                for (int q = firstInQueue; q < tail && closed; q++) { closed = slotCount[queue[q]] == 3; }
                if (closed)
                {
                    double vol = ComponentVolume(v, tri, queue, firstInQueue, tail, flip);
                    if (vol < 0) { for (int q = firstInQueue; q < tail; q++) { flip[queue[q]] = !flip[queue[q]]; } }
                }
            }

            components++;
        }

        if (orient)
        {
            for (int t = 0; t < nt; t++)
            {
                if (flip[t])
                {
                    (tri[3 * t + 1], tri[3 * t + 2]) = (tri[3 * t + 2], tri[3 * t + 1]);
                    flipped++;
                }
            }
        }

        // Holes: connected groups of boundary edges.
        int holes = 0;
        if (bp.Count > 0)
        {
            var parent = new int[v.Length];
            for (int i = 0; i < parent.Length; i++) { parent[i] = i; }
            foreach (var (a, b) in bp) { Union(parent, a, b); }
            var seen = new bool[v.Length];
            foreach (var (a, _) in bp)
            {
                int r = Find(parent, a);
                if (!seen[r]) { seen[r] = true; holes++; }
            }
        }

        report.FlippedTriangles = flipped;
        report.Components = components;
        report.NonManifoldEdges = nonManifold;
        report.BoundaryEdgeCount = boundary;
        report.HoleCount = holes;
    }

    private static double ComponentVolume(Vector3F[] v, int[] tri, int[] list, int from, int to, bool[] flip)
    {
        double vol = 0;
        for (int q = from; q < to; q++)
        {
            int t = list[q];
            int a = tri[3 * t], b = tri[3 * t + 1], c = tri[3 * t + 2];
            if (flip[t]) { (b, c) = (c, b); }
            double ax = v[a].X, ay = v[a].Y, az = v[a].Z, bx = v[b].X, by = v[b].Y, bz = v[b].Z, cx = v[c].X, cy = v[c].Y, cz = v[c].Z;
            vol += ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
        }

        return vol;
    }
}
