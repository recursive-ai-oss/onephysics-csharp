using System;
using RecursiveAI.OneMath;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>
/// Exact-as-possible tests of single triangles, shared by <see cref="TriangleMesh3D"/> and <see cref="HeightField3D"/> and usable on their own.
/// They compute in double from float32 inputs, so every result is the same on every platform, and a brute-force loop over these functions is the reference the mid-phase must agree with.
/// </summary>
[Experimental("OPHYS0001")]
public static class TriangleTests3D
{
    /// <summary>
    /// Intersects a ray with a triangle, watertight across shared edges and vertices. The ray's side of each of the three edges is a sign computed from the edge's endpoints in a canonical order
    /// (lexicographic by x, y, z) so two triangles sharing an edge see exactly opposite signs; a ray passing exactly through the edge is resolved as if it were nudged by an infinitesimal step along +x, then +y, then +z,
    /// so it hits exactly one of the triangles meeting there (the idea of Plücker-coordinate ray tests, Ericson, Real-Time Collision Detection, 5.3, with a simulation-of-simplicity tie rule).
    /// </summary>
    /// <param name="origin">Ray origin.</param>
    /// <param name="direction">Ray direction; its length scales <paramref name="fraction"/>. A zero direction never hits.</param>
    /// <param name="a">First vertex; the winding a, b, c defines the front side (the normal is <c>(b - a) x (c - a)</c>).</param>
    /// <param name="b">Second vertex.</param>
    /// <param name="c">Third vertex.</param>
    /// <param name="cullBackFaces">When true a ray hitting the back side misses.</param>
    /// <param name="fraction">The hit as a multiple of <paramref name="direction"/> from the origin, at least 0; unset when false is returned. Not limited above.</param>
    /// <param name="backFace">True when the ray hit the back side (it travels along the normal).</param>
    /// <returns>True when the ray's half line passes through the triangle.</returns>
    public static bool RayTriangle(Vector3F origin, Vector3F direction, Vector3F a, Vector3F b, Vector3F c, bool cullBackFaces, out double fraction, out bool backFace) =>
        RayTri(origin.ToDouble(), direction.ToDouble(), a.ToDouble(), b.ToDouble(), c.ToDouble(), cullBackFaces, out fraction, out backFace);

    internal static bool RayTri(in Vector3D o, in Vector3D d, in Vector3D a, in Vector3D b, in Vector3D c, bool cull, out double t, out bool back)
    {
        t = 0;
        back = false;
        Vector3D n = Vector3D.Cross(b - a, c - a);
        double nd = Vector3D.Dot(n, d);
        if (nd == 0.0)
        {
            return false;
        }

        back = nd > 0.0;
        if (back && cull)
        {
            return false;
        }

        bool first = EdgeSide(a, b, o, d);
        if (first != EdgeSide(b, c, o, d) || first != EdgeSide(c, a, o, d))
        {
            return false;
        }

        t = Vector3D.Dot(n, a - o) / nd;
        return t >= 0.0;
    }

    private static bool Less(in Vector3D p, in Vector3D q) =>
        p.X != q.X ? p.X < q.X : p.Y != q.Y ? p.Y < q.Y : p.Z < q.Z;

    private static bool EdgeSide(in Vector3D p0, in Vector3D q0, in Vector3D o, in Vector3D d)
    {
        bool canonical = Less(p0, q0);
        Vector3D p = canonical ? p0 : q0, q = canonical ? q0 : p0;
        double s = Vector3D.Dot(Vector3D.Cross(p - o, q - o), d);
        bool positive;
        if (s != 0.0)
        {
            positive = s > 0.0;
        }
        else
        {
            Vector3D w = Vector3D.Cross(q - p, d);
            positive = w.X != 0.0 ? w.X > 0.0 : w.Y != 0.0 ? w.Y > 0.0 : w.Z != 0.0 ? w.Z > 0.0 : true;
        }

        return canonical ? positive : !positive;
    }

    /// <summary>
    /// Tests a triangle against an axis-aligned box by the separating-axis theorem (13 axes; Ericson, Real-Time Collision Detection, 5.2.9; Akenine-Möller 2001). Touching counts as overlapping.
    /// The triangle's own bounding box is tested first with exact float comparisons, so a triangle is never reported when its bounds are apart from the box.
    /// </summary>
    /// <param name="box">The box.</param>
    /// <param name="a">First vertex.</param>
    /// <param name="b">Second vertex.</param>
    /// <param name="c">Third vertex.</param>
    /// <returns>True when the closed triangle and the closed box share a point (within double rounding).</returns>
    public static bool BoxTriangle(Bounds3F box, Vector3F a, Vector3F b, Vector3F c)
    {
        Vector3F lo = Vector3F.Min(a, Vector3F.Min(b, c)), hi = Vector3F.Max(a, Vector3F.Max(b, c));
        if (!(lo.X <= box.Max.X && box.Min.X <= hi.X && lo.Y <= box.Max.Y && box.Min.Y <= hi.Y && lo.Z <= box.Max.Z && box.Min.Z <= hi.Z))
        {
            return false;
        }

        Vector3D ctr = (box.Min.ToDouble() + box.Max.ToDouble()) * 0.5;
        Vector3D h = (box.Max.ToDouble() - box.Min.ToDouble()) * 0.5;
        Vector3D v0 = a.ToDouble() - ctr, v1 = b.ToDouble() - ctr, v2 = c.ToDouble() - ctr;
        Vector3D e0 = v1 - v0, e1 = v2 - v1, e2 = v0 - v2;
        for (int i = 0; i < 3; i++)
        {
            Vector3D e = i == 0 ? e0 : i == 1 ? e1 : e2;
            for (int k = 0; k < 3; k++)
            {
                Vector3D axis = k == 0 ? new Vector3D(0, -e.Z, e.Y) : k == 1 ? new Vector3D(e.Z, 0, -e.X) : new Vector3D(-e.Y, e.X, 0);
                double p0 = Vector3D.Dot(axis, v0), p1 = Vector3D.Dot(axis, v1), p2 = Vector3D.Dot(axis, v2);
                double r = (h.X * Math.Abs(axis.X)) + (h.Y * Math.Abs(axis.Y)) + (h.Z * Math.Abs(axis.Z));
                if (Math.Min(p0, Math.Min(p1, p2)) > r || Math.Max(p0, Math.Max(p1, p2)) < -r)
                {
                    return false;
                }
            }
        }

        // The box axes are covered by the bounding-box test above; the triangle's plane remains.
        Vector3D n = Vector3D.Cross(e0, e1);
        double dist = Vector3D.Dot(n, v0);
        double rr = (h.X * Math.Abs(n.X)) + (h.Y * Math.Abs(n.Y)) + (h.Z * Math.Abs(n.Z));
        return Math.Abs(dist) <= rr;
    }

    /// <summary>
    /// Decides whether the edge a triangle shares with a neighbour is <em>active</em>, meaning contacts on it must be kept. Flat edges (the faces within <paramref name="tolerance"/> of coplanar, bending
    /// by less than that sine) and concave edges (the faces fold towards their front sides) are inactive: a shape resting on them touches one of the two faces anyway, and keeping the edge
    /// produces the "ghost bump" of a contact normal along the seam. Convex edges are active, including folds sharper than a right angle.
    /// The faces must be wound consistently (the shared edge runs opposite ways in the two triangles) for the answer to be meaningful.
    /// </summary>
    /// <param name="a0">First vertex of triangle A.</param>
    /// <param name="a1">Second vertex of triangle A.</param>
    /// <param name="a2">Third vertex of triangle A.</param>
    /// <param name="edge">The edge of A: 0 is a0-a1, 1 is a1-a2, 2 is a2-a0.</param>
    /// <param name="b0">First vertex of the neighbour B.</param>
    /// <param name="b1">Second vertex of B.</param>
    /// <param name="b2">Third vertex of B.</param>
    /// <param name="tolerance">Sine of the largest bend still counted as flat, in (0, 1].</param>
    /// <returns>True when the edge is active.</returns>
    public static bool IsEdgeActive(Vector3F a0, Vector3F a1, Vector3F a2, int edge, Vector3F b0, Vector3F b1, Vector3F b2, double tolerance)
    {
        if ((uint)edge > 2u)
        {
            throw new ArgumentOutOfRangeException(nameof(edge), edge, $"edge is {edge}; 0, 1 or 2 is expected.");
        }

        return IsActive(a0.ToDouble(), a1.ToDouble(), a2.ToDouble(), edge, b0.ToDouble(), b1.ToDouble(), b2.ToDouble(), tolerance);
    }

    internal static bool IsActive(in Vector3D a0, in Vector3D a1, in Vector3D a2, int edge, in Vector3D b0, in Vector3D b1, in Vector3D b2, double tol)
    {
        Vector3D nA = Vector3D.Cross(a1 - a0, a2 - a0), nB = Vector3D.Cross(b1 - b0, b2 - b0);
        Vector3D eA = edge == 0 ? a1 - a0 : edge == 1 ? a2 - a1 : a0 - a2;
        double lenA = Math.Sqrt(nA.LengthSquared()), lenB = Math.Sqrt(nB.LengthSquared()), lenE = Math.Sqrt(eA.LengthSquared());
        double denom = lenA * lenB;
        if (!(denom > 0.0) || !(lenE > 0.0))
        {
            return true;
        }

        // For consistently wound faces nA x nB is parallel to the shared edge, along eA when the surface folds away from the viewer (convex).
        double sin = Vector3D.Dot(Vector3D.Cross(nA, nB), eA) / (lenE * denom);
        double cos = Vector3D.Dot(nA, nB) / denom;
        if (sin < -tol)
        {
            return false;
        }

        return !(cos > 0.0 && sin <= tol);
    }
}
