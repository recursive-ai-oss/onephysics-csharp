using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

/// <summary>Slow, simple double-precision reference answers for convex polyhedra.</summary>
internal static class Brute
{
    public readonly record struct V(double X, double Y, double Z)
    {
        public static V operator +(V a, V b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static V operator -(V a, V b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static V operator *(V a, double s) => new(a.X * s, a.Y * s, a.Z * s);
        public static double Dot(V a, V b) => (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z);
        public static V Cross(V a, V b) => new((a.Y * b.Z) - (a.Z * b.Y), (a.Z * b.X) - (a.X * b.Z), (a.X * b.Y) - (a.Y * b.X));
        public double Len => Math.Sqrt(Dot(this, this));
        public static implicit operator V(Vector3F v) => new(v.X, v.Y, v.Z);
    }

    /// <summary>A convex polyhedron in the frame of A: vertices, unit face normals with plane offsets, and edge directions.</summary>
    public sealed class Poly
    {
        public V[] Verts = Array.Empty<V>();
        public V[] Normals = Array.Empty<V>();
        public double[] Offsets = Array.Empty<double>();
        public List<(int A, int B)> Edges = new();
        public List<(int A, int B, int C)> Tris = new();
    }

    public static Poly FromHull(ConvexHull3D hull, in RigidTransformF xf)
    {
        var p = new Poly();
        var m = xf.Rotation.ToMatrix();
        p.Verts = new V[hull.Vertices.Length];
        for (int i = 0; i < p.Verts.Length; i++)
        {
            p.Verts[i] = xf.TransformPoint(hull.Vertices[i]);
        }

        // Use the double positions of the float-transformed vertices; normals from the transformed hull faces.
        var normals = new List<V>();
        var offsets = new List<double>();
        for (int f = 0; f < hull.Faces.Length; f++)
        {
            HullFace3D face = hull.Faces[f];
            int e = face.FirstEdge;
            int first = hull.Edges[e].Origin;
            int[] ring = new int[face.EdgeCount];
            for (int k = 0; k < face.EdgeCount; k++)
            {
                ring[k] = hull.Edges[e].Origin;
                e = hull.Edges[e].Next;
            }

            // Newell in double on the transformed vertices.
            V c = default;
            foreach (int r in ring)
            {
                c += p.Verts[r];
            }

            c *= 1.0 / ring.Length;
            V n = default;
            for (int k = 0; k < ring.Length; k++)
            {
                V a = p.Verts[ring[k]] - c, b = p.Verts[ring[(k + 1) % ring.Length]] - c;
                n += V.Cross(a, b);
            }

            n *= 1.0 / n.Len;
            normals.Add(n);
            offsets.Add(V.Dot(n, c));
            for (int k = 1; k + 1 < ring.Length; k++)
            {
                p.Tris.Add((ring[0], ring[k], ring[k + 1]));
            }

            _ = first;
        }

        for (int e = 0; e < hull.Edges.Length; e += 2)
        {
            p.Edges.Add((hull.Edges[e].Origin, hull.Edges[e + 1].Origin));
        }

        p.Normals = normals.ToArray();
        p.Offsets = offsets.ToArray();
        _ = m;
        return p;
    }

    public static (double min, double max) Project(V[] verts, V axis)
    {
        double lo = double.MaxValue, hi = -double.MaxValue;
        foreach (V v in verts)
        {
            double d = V.Dot(v, axis);
            lo = Math.Min(lo, d);
            hi = Math.Max(hi, d);
        }

        return (lo, hi);
    }

    /// <summary>
    /// Separating axis test over face normals of both and edge cross products. Returns the best signed separation along an axis
    /// (positive: a gap exists, the shapes are apart by at least that; negative: the least penetration depth) and the axis A -> B.
    /// The second best |separation| is returned too so callers can skip ambiguous cases.
    /// </summary>
    public static (double sep, V axis, double second) Sat(Poly a, Poly b)
    {
        var axes = new List<V>();
        axes.AddRange(a.Normals);
        axes.AddRange(b.Normals);
        foreach (var ea in a.Edges)
        {
            V da = a.Verts[ea.B] - a.Verts[ea.A];
            foreach (var eb in b.Edges)
            {
                V db = b.Verts[eb.B] - b.Verts[eb.A];
                V c = V.Cross(da, db);
                double l = c.Len;
                if (l > 1e-9 * da.Len * db.Len)
                {
                    axes.Add(c * (1.0 / l));
                }
            }
        }

        double best = -double.MaxValue, second = -double.MaxValue;
        V bestAxis = default;
        foreach (V n in axes)
        {
            var (aLo, aHi) = Project(a.Verts, n);
            var (bLo, bHi) = Project(b.Verts, n);
            double g1 = bLo - aHi, g2 = aLo - bHi;
            double g = Math.Max(g1, g2);
            V dir = g1 >= g2 ? n : n * -1.0;
            if (g > best)
            {
                second = best;
                best = g;
                bestAxis = dir;
            }
            else if (g > second)
            {
                second = g;
            }
        }

        return (best, bestAxis, second);
    }

    public static double PointSegment(V p, V a, V b)
    {
        V ab = b - a;
        double den = V.Dot(ab, ab);
        double t = den > 0 ? Math.Clamp(V.Dot(p - a, ab) / den, 0, 1) : 0;
        return (p - (a + (ab * t))).Len;
    }

    public static double PointTriangle(V p, V a, V b, V c)
    {
        V n = V.Cross(b - a, c - a);
        double nl = n.Len;
        double best = Math.Min(PointSegment(p, a, b), Math.Min(PointSegment(p, b, c), PointSegment(p, c, a)));
        if (nl > 0)
        {
            n *= 1.0 / nl;
            double d = V.Dot(p - a, n);
            V q = p - (n * d);
            if (V.Dot(V.Cross(b - a, q - a), n) >= 0 && V.Dot(V.Cross(c - b, q - b), n) >= 0 && V.Dot(V.Cross(a - c, q - c), n) >= 0)
            {
                best = Math.Min(best, Math.Abs(d));
            }
        }

        return best;
    }

    public static double SegmentSegment(V a1, V b1, V a2, V b2)
    {
        double best = Math.Min(Math.Min(PointSegment(a1, a2, b2), PointSegment(b1, a2, b2)), Math.Min(PointSegment(a2, a1, b1), PointSegment(b2, a1, b1)));
        V d1 = b1 - a1, d2 = b2 - a2, r = a1 - a2;
        double a = V.Dot(d1, d1), e = V.Dot(d2, d2), b = V.Dot(d1, d2), c = V.Dot(d1, r), f = V.Dot(d2, r);
        double den = (a * e) - (b * b);
        if (den > 1e-12 * a * e)
        {
            double s = ((b * f) - (c * e)) / den;
            double t = ((a * f) - (b * c)) / den;
            if (s > 0 && s < 1 && t > 0 && t < 1)
            {
                best = Math.Min(best, ((a1 + (d1 * s)) - (a2 + (d2 * t))).Len);
            }
        }

        return best;
    }

    /// <summary>Minimum distance between the surfaces of two disjoint polyhedra: vertex-face and edge-edge pairs.</summary>
    public static double Distance(Poly a, Poly b)
    {
        double best = double.MaxValue;
        foreach (V v in b.Verts)
        {
            foreach (var t in a.Tris)
            {
                best = Math.Min(best, PointTriangle(v, a.Verts[t.A], a.Verts[t.B], a.Verts[t.C]));
            }
        }

        foreach (V v in a.Verts)
        {
            foreach (var t in b.Tris)
            {
                best = Math.Min(best, PointTriangle(v, b.Verts[t.A], b.Verts[t.B], b.Verts[t.C]));
            }
        }

        foreach (var ea in a.Edges)
        {
            foreach (var eb in b.Edges)
            {
                best = Math.Min(best, SegmentSegment(a.Verts[ea.A], a.Verts[ea.B], b.Verts[eb.A], b.Verts[eb.B]));
            }
        }

        return best;
    }

    /// <summary>Ray against a convex polyhedron by clipping with its face planes. Returns the entry parameter or null.</summary>
    public static (double t, V normal)? Ray(Poly p, V origin, V dir, double maxT)
    {
        double t0 = 0, t1 = maxT;
        V nrm = default;
        for (int i = 0; i < p.Normals.Length; i++)
        {
            double denom = V.Dot(p.Normals[i], dir);
            double dist = p.Offsets[i] - V.Dot(p.Normals[i], origin);
            if (Math.Abs(denom) < 1e-14)
            {
                if (dist < 0)
                {
                    return null;
                }
            }
            else
            {
                double t = dist / denom;
                if (denom < 0)
                {
                    if (t > t0)
                    {
                        t0 = t;
                        nrm = p.Normals[i];
                    }
                }
                else if (t < t1)
                {
                    t1 = t;
                }
            }

            if (t0 > t1)
            {
                return null;
            }
        }

        return (t0, nrm);
    }
}
