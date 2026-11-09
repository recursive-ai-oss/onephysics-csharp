using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Predicates;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>Validation and exact tests shared by the 2D tools.</summary>
internal static class PolygonInput
{
    /// <summary>The largest coordinate magnitude accepted; the 2D engine's shapes take the same limit.</summary>
    public const float MaxCoordinate = 1048576f;

    /// <summary>The most vertices accepted over a polygon and its holes together.</summary>
    public const int MaxVertices = 16384;

    private const double MinNonZero = 1.0 / (1L << 40) / (1L << 40);

    public static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    /// <summary>Checks a loop's points and returns them as doubles without consecutive duplicates (and without a repeated closing point).</summary>
    public static (double X, double Y)[] Prepare(ReadOnlySpan<Vector2F> pts, string what, string paramName)
    {
        if (pts.Length < 3)
        {
            throw new GeometryToolException(GeometryToolError.Empty, what + " has " + MeshInput.Inv(pts.Length) + " points, at least 3 are needed", paramName);
        }

        if (pts.Length > MaxVertices)
        {
            throw new GeometryToolException(GeometryToolError.OutOfRange, what + " has " + MeshInput.Inv(pts.Length) + " points, the limit is " + MeshInput.Inv(MaxVertices), paramName);
        }

        var list = new List<(double, double)>(pts.Length);
        for (int i = 0; i < pts.Length; i++)
        {
            Vector2F p = pts[i];
            if (!float.IsFinite(p.X) || !float.IsFinite(p.Y))
            {
                throw new GeometryToolException(GeometryToolError.NonFinite, what + "[" + MeshInput.Inv(i) + "] = (" + F(p.X) + ", " + F(p.Y) + ") is not finite", paramName);
            }

            if (Math.Abs(p.X) > MaxCoordinate || Math.Abs(p.Y) > MaxCoordinate || (p.X != 0 && Math.Abs(p.X) < MinNonZero) || (p.Y != 0 && Math.Abs(p.Y) < MinNonZero))
            {
                throw new GeometryToolException(GeometryToolError.OutOfRange, what + "[" + MeshInput.Inv(i) + "] = (" + F(p.X) + ", " + F(p.Y) + ") is beyond ±" + F(MaxCoordinate) + " or too close to zero to test exactly", paramName);
            }

            if (list.Count > 0 && list[^1].Item1 == p.X && list[^1].Item2 == p.Y) { continue; }
            list.Add((p.X, p.Y));
        }

        while (list.Count > 1 && list[0].Item1 == list[^1].Item1 && list[0].Item2 == list[^1].Item2) { list.RemoveAt(list.Count - 1); }
        if (list.Count < 3)
        {
            throw new GeometryToolException(GeometryToolError.Degenerate, what + " has fewer than 3 distinct points", paramName);
        }

        return list.ToArray();
    }

    public static double SignedArea((double X, double Y)[] p)
    {
        double a = 0;
        for (int i = 0; i < p.Length; i++)
        {
            var (x0, y0) = p[i];
            var (x1, y1) = p[(i + 1) % p.Length];
            a += x0 * y1 - x1 * y0;
        }

        return 0.5 * a;
    }

    public static int Orient((double X, double Y) a, (double X, double Y) b, (double X, double Y) c) => ExactPredicates.Orient2D(a.X, a.Y, b.X, b.Y, c.X, c.Y);

    /// <summary>Finds a pair of edges that cross or touch (other than neighbours in one loop meeting at their shared vertex, and neighbours that fold back on each other), or a loop that has no area.</summary>
    /// <returns>A message naming the two loops and edges, or null if the loops are simple and disjoint.</returns>
    public static string? FindIntersection(IReadOnlyList<(double X, double Y)[]> loops)
    {
        for (int a = 0; a < loops.Count; a++)
        {
            var la = loops[a];
            for (int i = 0; i < la.Length; i++)
            {
                var p0 = la[i];
                var p1 = la[(i + 1) % la.Length];
                double minX = Math.Min(p0.X, p1.X), maxX = Math.Max(p0.X, p1.X), minY = Math.Min(p0.Y, p1.Y), maxY = Math.Max(p0.Y, p1.Y);
                for (int b = a; b < loops.Count; b++)
                {
                    var lb = loops[b];
                    for (int j = (b == a ? i + 1 : 0); j < lb.Length; j++)
                    {
                        var q0 = lb[j];
                        var q1 = lb[(j + 1) % lb.Length];
                        if (Math.Max(q0.X, q1.X) < minX || Math.Min(q0.X, q1.X) > maxX || Math.Max(q0.Y, q1.Y) < minY || Math.Min(q0.Y, q1.Y) > maxY) { continue; }
                        var rel = SegmentPredicates.Classify(p0.X, p0.Y, p1.X, p1.Y, q0.X, q0.Y, q1.X, q1.Y);
                        if (rel == SegmentRelation.Disjoint) { continue; }
                        if (a == b)
                        {
                            bool nextTo = j == i + 1 || (i == 0 && j == la.Length - 1);
                            // Neighbours meet at their shared vertex; folding back over each other would be an overlap, not a touch.
                            if (nextTo && rel == SegmentRelation.Touching) { continue; }
                        }

                        return "edge " + MeshInput.Inv(i) + " of loop " + MeshInput.Inv(a) + " meets edge " + MeshInput.Inv(j) + " of loop " + MeshInput.Inv(b);
                    }
                }
            }
        }

        return null;
    }

    /// <summary>Even-odd containment of a point in a loop (the point must not lie on the loop).</summary>
    public static bool Inside((double X, double Y) p, (double X, double Y)[] loop)
    {
        bool inside = false;
        for (int i = 0, j = loop.Length - 1; i < loop.Length; j = i++)
        {
            var a = loop[i];
            var b = loop[j];
            if ((a.Y > p.Y) != (b.Y > p.Y))
            {
                // Which side of ab the point lies on, exactly, oriented so that "left" is the side of the upward edge.
                int o = Orient(a, b, p);
                bool up = b.Y > a.Y;
                if (up ? o > 0 : o < 0) { inside = !inside; }
            }
        }

        return inside;
    }
}
