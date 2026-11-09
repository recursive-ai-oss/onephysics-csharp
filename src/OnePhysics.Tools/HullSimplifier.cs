using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>Limits for <see cref="HullSimplifier.Simplify"/>. At least one of the three must be set.</summary>
public readonly struct HullSimplifyOptions
{
    /// <summary>Remove vertices until the hull has at most this many; 0 means no target, otherwise at least 4. The target is always pursued, even if the result then exceeds <see cref="MaxError"/>.</summary>
    public int MaxVertices { get; init; }

    /// <summary>Remove vertices until the hull has at most this many faces; 0 means no target, otherwise at least 4. Pursued like <see cref="MaxVertices"/>.</summary>
    public int MaxFaces { get; init; }

    /// <summary>Beyond the targets, keep removing vertices while the Hausdorff distance between the result and the input hull stays at most this, in the hull's units. 0 means none; otherwise finite and positive.</summary>
    public float MaxError { get; init; }
}

/// <summary>The result of <see cref="HullSimplifier.Simplify"/>.</summary>
public sealed class SimplifiedHull
{
    internal SimplifiedHull(ConvexHull3D hull, float error, int removed, bool targetsMet)
    {
        Hull = hull;
        Error = error;
        RemovedVertices = removed;
        TargetsMet = targetsMet;
    }

    /// <summary>The simplified hull; its vertices are a subset of the input hull's, so it lies inside the input.</summary>
    public ConvexHull3D Hull { get; }

    /// <summary>The Hausdorff distance between the result and the input hull: the largest distance of any input vertex from the result (the result lies inside the input, and the distance from a convex body is largest at a vertex). Accurate to single precision rounding.</summary>
    public float Error { get; }

    /// <summary>How many vertices were removed.</summary>
    public int RemovedVertices { get; }

    /// <summary>False when a vertex or face target could not be reached (the hull cannot go below four vertices, or a removal would degenerate it).</summary>
    public bool TargetsMet { get; }
}

/// <summary>
/// Reduces the vertex or face count of a convex hull by removing vertices and re-hulling the rest, greedily choosing the removal that leaves the smallest Hausdorff error. The error is measured, not estimated,
/// so the bound in <see cref="HullSimplifyOptions.MaxError"/> holds. (Hull simplification by vertex removal: see e.g. Lien and Amato, "Approximate convex decomposition of polyhedra", for the concavity and hull-error measures this follows.)
/// </summary>
public static class HullSimplifier
{
    private const int ExactCandidates = 12;

    /// <summary>Simplifies a hull.</summary>
    /// <param name="hull">The hull to reduce; it is not modified.</param>
    /// <param name="options">Targets and error bound; at least one must be set.</param>
    /// <returns>The reduced hull with its measured error. Identical input gives identical output on every platform.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="hull"/> is null.</exception>
    /// <exception cref="GeometryToolException">No option is set, or an option is invalid.</exception>
    public static SimplifiedHull Simplify(ConvexHull3D hull, HullSimplifyOptions options)
    {
        ArgumentNullException.ThrowIfNull(hull);
        if (options.MaxVertices != 0 && options.MaxVertices < 4)
        {
            throw new GeometryToolException(GeometryToolError.InvalidOptions, "MaxVertices = " + MeshInput.Inv(options.MaxVertices) + " must be 0 or at least 4", nameof(options));
        }

        if (options.MaxFaces != 0 && options.MaxFaces < 4)
        {
            throw new GeometryToolException(GeometryToolError.InvalidOptions, "MaxFaces = " + MeshInput.Inv(options.MaxFaces) + " must be 0 or at least 4", nameof(options));
        }

        float me = options.MaxError;
        if (float.IsNaN(me) || float.IsInfinity(me) || me < 0f)
        {
            throw new GeometryToolException(GeometryToolError.InvalidOptions, "MaxError = " + me + " must be finite and not negative", nameof(options));
        }

        if (options.MaxVertices == 0 && options.MaxFaces == 0 && me == 0f)
        {
            throw new GeometryToolException(GeometryToolError.InvalidOptions, "set at least one of MaxVertices, MaxFaces and MaxError", nameof(options));
        }

        var original = hull.Vertices.ToArray();
        ConvexHull3D cur = hull;
        double err = 0;
        int removed = 0;
        bool targetsMet = true;
        while (true)
        {
            bool needTarget = (options.MaxVertices > 0 && cur.Vertices.Length > options.MaxVertices) || (options.MaxFaces > 0 && cur.Faces.Length > options.MaxFaces);
            if (!needTarget && me <= 0f) { break; }
            if (cur.Vertices.Length <= 4)
            {
                if (needTarget) { targetsMet = false; }
                break;
            }

            if (!BestRemoval(cur, original, out ConvexHull3D? next, out double nextErr))
            {
                if (needTarget) { targetsMet = false; }
                break;
            }

            if (!needTarget && nextErr > me) { break; }
            removed += cur.Vertices.Length - next!.Vertices.Length;
            cur = next;
            err = nextErr;
        }

        return new SimplifiedHull(cur, (float)err, removed, targetsMet);
    }

    private static bool BestRemoval(ConvexHull3D cur, Vector3F[] original, out ConvexHull3D? best, out double bestErr)
    {
        best = null;
        bestErr = double.PositiveInfinity;
        var verts = cur.Vertices.ToArray();
        int n = verts.Length;

        // Rank vertices by how far they stand from the centroid of their neighbours: a flat or barely curved vertex is cheap to remove.
        var ringSum = new double[n * 3];
        var ringCount = new int[n];
        var edges = cur.Edges;
        for (int e = 0; e < edges.Length; e++)
        {
            int o = edges[e].Origin, d = edges[e ^ 1].Origin;
            ringSum[3 * o] += verts[d].X; ringSum[3 * o + 1] += verts[d].Y; ringSum[3 * o + 2] += verts[d].Z;
            ringCount[o]++;
        }

        var proxy = new double[n];
        var order = new int[n];
        for (int i = 0; i < n; i++)
        {
            order[i] = i;
            double k = Math.Max(1, ringCount[i]);
            double dx = verts[i].X - ringSum[3 * i] / k, dy = verts[i].Y - ringSum[3 * i + 1] / k, dz = verts[i].Z - ringSum[3 * i + 2] / k;
            proxy[i] = dx * dx + dy * dy + dz * dz;
        }

        Array.Sort(order, (a, b) =>
        {
            int c = proxy[a].CompareTo(proxy[b]);
            return c != 0 ? c : a.CompareTo(b);
        });

        int tries = Math.Min(n, ExactCandidates);
        var rest = new Vector3F[n - 1];
        for (int oi = 0; oi < tries; oi++)
        {
            int skip = order[oi];
            int w = 0;
            for (int i = 0; i < n; i++) { if (i != skip) { rest[w++] = verts[i]; } }
            ConvexHull3D candidate;
            try { candidate = ConvexHull3D.Build(rest); }
            catch (HullBuildException) { continue; }
            double e = HausdorffFromOriginal(candidate, original);
            // Smaller error wins; on equal error the earlier candidate in rank order (itself ordered by index).
            if (e < bestErr)
            {
                bestErr = e;
                best = candidate;
            }
        }

        return best is not null;
    }

    /// <summary>The largest distance of any of the points from the convex hull (zero for points inside it); for the vertices of a hull that contains another this is the Hausdorff distance between the two.</summary>
    /// <param name="hull">The hull.</param>
    /// <param name="points">The points to measure; finite.</param>
    /// <returns>The distance, in the points' units.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="hull"/> is null.</exception>
    public static double MaxDistanceOutside(ConvexHull3D hull, ReadOnlySpan<Vector3F> points)
    {
        ArgumentNullException.ThrowIfNull(hull);
        return HausdorffFromOriginal(hull, points);
    }

    private static double HausdorffFromOriginal(ConvexHull3D h, ReadOnlySpan<Vector3F> points)
    {
        double worst = 0;
        var faces = h.Faces;
        var edges = h.Edges;
        var v = h.Vertices;
        for (int i = 0; i < points.Length; i++)
        {
            double px = points[i].X, py = points[i].Y, pz = points[i].Z;
            bool outside = false;
            for (int f = 0; f < faces.Length; f++)
            {
                Vector3F nrm = faces[f].Normal;
                if (nrm.X * px + nrm.Y * py + nrm.Z * pz - faces[f].Offset > 1e-7 * (1 + Math.Abs(faces[f].Offset))) { outside = true; break; }
            }

            if (!outside) { continue; }
            double best = double.PositiveInfinity;
            for (int f = 0; f < faces.Length; f++)
            {
                int e0 = faces[f].FirstEdge;
                int a = edges[e0].Origin;
                int e = edges[e0].Next;
                while (edges[e].Next != e0)
                {
                    int b = edges[e].Origin, c = edges[edges[e].Next].Origin;
                    double d2 = TriangleDistanceSquared(px, py, pz, v[a], v[b], v[c]);
                    if (d2 < best) { best = d2; }
                    e = edges[e].Next;
                }
            }

            double d = Math.Sqrt(best);
            if (d > worst) { worst = d; }
        }

        return worst;
    }

    // Closest point on a triangle (Ericson, Real-Time Collision Detection, 5.1.5), squared distance.
    internal static double TriangleDistanceSquared(double px, double py, double pz, Vector3F a, Vector3F b, Vector3F c)
    {
        double abx = (double)b.X - a.X, aby = (double)b.Y - a.Y, abz = (double)b.Z - a.Z;
        double acx = (double)c.X - a.X, acy = (double)c.Y - a.Y, acz = (double)c.Z - a.Z;
        double apx = px - a.X, apy = py - a.Y, apz = pz - a.Z;
        double d1 = abx * apx + aby * apy + abz * apz, d2 = acx * apx + acy * apy + acz * apz;
        if (d1 <= 0 && d2 <= 0) { return Sq(apx, apy, apz); }
        double bpx = px - b.X, bpy = py - b.Y, bpz = pz - b.Z;
        double d3 = abx * bpx + aby * bpy + abz * bpz, d4 = acx * bpx + acy * bpy + acz * bpz;
        if (d3 >= 0 && d4 <= d3) { return Sq(bpx, bpy, bpz); }
        double vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0)
        {
            double t = d1 / (d1 - d3);
            return Sq(apx - t * abx, apy - t * aby, apz - t * abz);
        }

        double cpx = px - c.X, cpy = py - c.Y, cpz = pz - c.Z;
        double d5 = abx * cpx + aby * cpy + abz * cpz, d6 = acx * cpx + acy * cpy + acz * cpz;
        if (d6 >= 0 && d5 <= d6) { return Sq(cpx, cpy, cpz); }
        double vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0)
        {
            double t = d2 / (d2 - d6);
            return Sq(apx - t * acx, apy - t * acy, apz - t * acz);
        }

        double va = d3 * d6 - d5 * d4;
        if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0)
        {
            double t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return Sq(bpx - t * (c.X - (double)b.X), bpy - t * (c.Y - (double)b.Y), bpz - t * (c.Z - (double)b.Z));
        }

        double denom = 1.0 / (va + vb + vc);
        double v2 = vb * denom, w2 = vc * denom;
        return Sq(apx - abx * v2 - acx * w2, apy - aby * v2 - acy * w2, apz - abz * v2 - acz * w2);
    }

    private static double Sq(double x, double y, double z) => x * x + y * y + z * z;
}
