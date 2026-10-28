using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

[InlineArray(4)]
internal struct Vec4
{
    private Vector3F _e;
}

[InlineArray(4)]
internal struct Flt4
{
    private float _e;
}

/// <summary>
/// Up to four points of the Minkowski difference A - B (P), the points of A and B they came from (A, B), the vectors the solver works on (W)
/// and the barycentric weights of the closest point (L). Closest-point regions follow Ericson, Real-Time Collision Detection, sections 5.1.5 and 9.5.
/// </summary>
internal struct Simplex
{
    public int Count;
    public Vec4 P;
    public Vec4 A;
    public Vec4 B;
    public Vec4 W;
    public Flt4 L;

    public void Add(Vector3F p, Vector3F a, Vector3F b)
    {
        P[Count] = p;
        A[Count] = a;
        B[Count] = b;
        W[Count] = p;
        Count++;
    }

    public readonly Vector3F WitnessA()
    {
        Vector3F r = default;
        for (int i = 0; i < Count; i++)
        {
            r += A[i] * L[i];
        }

        return r;
    }

    public readonly Vector3F WitnessB()
    {
        Vector3F r = default;
        for (int i = 0; i < Count; i++)
        {
            r += B[i] * L[i];
        }

        return r;
    }

    // Keeps the vertices whose bit is set in mask, in order, with their weights.
    private void Keep(int mask)
    {
        int n = 0;
        for (int i = 0; i < Count; i++)
        {
            if ((mask & (1 << i)) != 0)
            {
                if (n != i)
                {
                    P[n] = P[i];
                    A[n] = A[i];
                    B[n] = B[i];
                    W[n] = W[i];
                    L[n] = L[i];
                }

                n++;
            }
        }

        Count = n;
    }

    /// <summary>Finds the point of the convex hull of W nearest the origin, stores its weights in L and drops vertices that do not contribute. Returns true when the origin lies inside a full tetrahedron (then v is zero).</summary>
    public bool Solve(out Vector3F v)
    {
        switch (Count)
        {
            case 1:
                L[0] = 1f;
                v = W[0];
                return false;
            case 2:
                {
                    v = ToF(ClosestOnSegment(W[0].ToDouble(), W[1].ToDouble(), out double l0, out double l1));
                    L[0] = (float)l0;
                    L[1] = (float)l1;
                    Keep((l0 > 0.0 ? 1 : 0) | (l1 > 0.0 ? 2 : 0));
                    return false;
                }

            case 3:
                {
                    v = ToF(ClosestOnTriangle(W[0].ToDouble(), W[1].ToDouble(), W[2].ToDouble(), out double l0, out double l1, out double l2));
                    L[0] = (float)l0;
                    L[1] = (float)l1;
                    L[2] = (float)l2;
                    Keep((l0 > 0.0 ? 1 : 0) | (l1 > 0.0 ? 2 : 0) | (l2 > 0.0 ? 4 : 0));
                    return false;
                }

            case 4:
                return SolveTetrahedron(out v);
            default:
                v = default;
                return false;
        }
    }

    private bool SolveTetrahedron(out Vector3F v)
    {
        // Face (i, j, k) is "outside" when the origin is on the opposite side of its plane from the fourth vertex.
        // A flat tetrahedron (fourth vertex in the plane) counts every face as a candidate.
        double scale2 = Math.Max(Math.Max(W[0].LengthSquared(), W[1].LengthSquared()), Math.Max(W[2].LengthSquared(), W[3].LengthSquared()));
        float bestD = float.MaxValue;
        int bi = -1, bj = -1, bk = -1;
        Vector3F bestV = default;
        float b0 = 0, b1 = 0, b2 = 0;
        for (int f = 0; f < 4; f++)
        {
            int i, j, k, d;
            switch (f)
            {
                case 0: i = 0; j = 1; k = 2; d = 3; break;
                case 1: i = 0; j = 2; k = 3; d = 1; break;
                case 2: i = 0; j = 3; k = 1; d = 2; break;
                default: i = 1; j = 3; k = 2; d = 0; break;
            }

            Vector3F a = W[i];

            // The side tests run in double: a thin tetrahedron's face normals lose too much in float32 to decide which side the origin is on.
            double ax = a.X, ay = a.Y, az = a.Z;
            double e1x = W[j].X - ax, e1y = W[j].Y - ay, e1z = W[j].Z - az;
            double e2x = W[k].X - ax, e2y = W[k].Y - ay, e2z = W[k].Z - az;
            double nx = (e1y * e2z) - (e1z * e2y), ny = (e1z * e2x) - (e1x * e2z), nz = (e1x * e2y) - (e1y * e2x);
            double n2 = (nx * nx) + (ny * ny) + (nz * nz);
            double signP = -((nx * ax) + (ny * ay) + (nz * az));
            double signD = (nx * (W[d].X - ax)) + (ny * (W[d].Y - ay)) + (nz * (W[d].Z - az));
            // A sliver face has no reliable normal; a fourth vertex within 1e-6 of its plane counts as in the plane. Both let the face compete on its true distance.
            bool isFlat = n2 <= 1e-10 * scale2 * scale2 || signD * signD <= 1e-12 * scale2 * n2;
            bool outside = isFlat || (signD > 0 ? signP < 0 : signP > 0);
            if (!outside)
            {
                continue;
            }

            Vector3D c = ClosestOnTriangle(a.ToDouble(), W[j].ToDouble(), W[k].ToDouble(), out double l0, out double l1, out double l2);
            float dist = (float)c.LengthSquared();
            if (dist < bestD)
            {
                bestD = dist;
                bi = i; bj = j; bk = k;
                bestV = ToF(c);
                b0 = (float)l0; b1 = (float)l1; b2 = (float)l2;
            }
        }

        if (bi < 0)
        {
            v = default;
            L[0] = L[1] = L[2] = L[3] = 0.25f;
            return true;
        }

        float f0 = 0, f1 = 0, f2 = 0, f3 = 0;
        Set(ref f0, ref f1, ref f2, ref f3, bi, b0);
        Set(ref f0, ref f1, ref f2, ref f3, bj, b1);
        Set(ref f0, ref f1, ref f2, ref f3, bk, b2);
        L[0] = f0;
        L[1] = f1;
        L[2] = f2;
        L[3] = f3;
        Keep((f0 > 0f ? 1 : 0) | (f1 > 0f ? 2 : 0) | (f2 > 0f ? 4 : 0) | (f3 > 0f ? 8 : 0));
        v = bestV;
        return false;
    }

    private static Vector3F ToF(Vector3D v) => new((float)v.X, (float)v.Y, (float)v.Z);

    private static void Set(ref float f0, ref float f1, ref float f2, ref float f3, int index, float value)
    {
        switch (index)
        {
            case 0: f0 = value; break;
            case 1: f1 = value; break;
            case 2: f2 = value; break;
            default: f3 = value; break;
        }
    }

    internal static Vector3D ClosestOnSegment(Vector3D a, Vector3D b, out double l0, out double l1)
    {
        Vector3D ab = b - a;
        double den = Vector3D.Dot(ab, ab);
        double t = 0.0;
        if (den > 0.0)
        {
            t = -Vector3D.Dot(a, ab) / den;
        }

        if (!(t > 0.0))
        {
            l0 = 1.0;
            l1 = 0.0;
            return a;
        }

        if (t >= 1.0)
        {
            l0 = 0.0;
            l1 = 1.0;
            return b;
        }

        l0 = 1.0 - t;
        l1 = t;
        return a + (ab * t);
    }

    // Voronoi regions of the triangle for the origin (Ericson 5.1.5); degenerate triangles fall back to their edges.
    internal static Vector3D ClosestOnTriangle(Vector3D a, Vector3D b, Vector3D c, out double l0, out double l1, out double l2)
    {
        Vector3D ab = b - a, ac = c - a;
        Vector3D ap = -a;
        double d1 = Vector3D.Dot(ab, ap), d2 = Vector3D.Dot(ac, ap);
        if (d1 <= 0.0 && d2 <= 0.0)
        {
            l0 = 1.0; l1 = 0.0; l2 = 0.0;
            return a;
        }

        Vector3D bp = -b;
        double d3 = Vector3D.Dot(ab, bp), d4 = Vector3D.Dot(ac, bp);
        if (d3 >= 0.0 && d4 <= d3)
        {
            l0 = 0.0; l1 = 1.0; l2 = 0.0;
            return b;
        }

        double vc = (d1 * d4) - (d3 * d2);
        if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
        {
            double t = d1 / (d1 - d3);
            l0 = 1.0 - t; l1 = t; l2 = 0.0;
            return a + (ab * t);
        }

        Vector3D cp = -c;
        double d5 = Vector3D.Dot(ab, cp), d6 = Vector3D.Dot(ac, cp);
        if (d6 >= 0.0 && d5 <= d6)
        {
            l0 = 0.0; l1 = 0.0; l2 = 1.0;
            return c;
        }

        double vb = (d5 * d2) - (d1 * d6);
        if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
        {
            double t = d2 / (d2 - d6);
            l0 = 1.0 - t; l1 = 0.0; l2 = t;
            return a + (ac * t);
        }

        double va = (d3 * d6) - (d5 * d4);
        if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
        {
            double t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            l0 = 0.0; l1 = 1.0 - t; l2 = t;
            return b + ((c - b) * t);
        }

        double sum = va + vb + vc;
        if (sum > 0.0 && double.IsFinite(sum))
        {
            double v = vb / sum, w = vc / sum;
            l0 = 1.0 - v - w; l1 = v; l2 = w;
            return a + (ab * v) + (ac * w);
        }

        // Degenerate (collinear or non-finite): nearest of the three edges.
        Vector3D e0 = ClosestOnSegment(a, b, out double p0, out double p1);
        Vector3D e1 = ClosestOnSegment(a, c, out double g0, out double g2);
        Vector3D e2 = ClosestOnSegment(b, c, out double r1, out double r2);
        double m0 = e0.LengthSquared(), m1 = e1.LengthSquared(), m2 = e2.LengthSquared();
        if (m0 <= m1 && m0 <= m2)
        {
            l0 = p0; l1 = p1; l2 = 0.0;
            return e0;
        }

        if (m1 <= m2)
        {
            l0 = g0; l1 = 0.0; l2 = g2;
            return e1;
        }

        l0 = 0.0; l1 = r1; l2 = r2;
        return e2;
    }
}
