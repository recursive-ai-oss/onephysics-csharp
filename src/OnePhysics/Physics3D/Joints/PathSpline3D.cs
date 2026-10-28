using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// A smooth curve through a list of points: a uniform Catmull-Rom spline (each segment is a cubic Hermite curve whose end tangents are half the chord between the neighbouring points; the end points of an open path
/// repeat). Used by path joints. Parameter t runs from 0 to the number of segments; segment i spans [i, i + 1].
/// </summary>
internal static class PathSpline3D
{
    private const int SamplesPerSegment = 8;

    private static Vector3F At(Vector3F[] pts, bool loop, int i)
    {
        int n = pts.Length;
        if (loop) { i %= n; if (i < 0) { i += n; } }
        else if (i < 0) { i = 0; }
        else if (i >= n) { i = n - 1; }
        return pts[i];
    }

    public static int Segments(Vector3F[] pts, bool loop) => loop ? pts.Length : pts.Length - 1;

    /// <summary>The point and its derivative with respect to t.</summary>
    public static void Eval(Vector3F[] pts, bool loop, float t, out Vector3F p, out Vector3F dp)
    {
        int segs = Segments(pts, loop);
        if (!(t > 0f)) { t = 0f; }
        if (t > segs) { t = segs; }
        int i = (int)t;
        if (i >= segs) { i = segs - 1; }
        float u = t - i;
        var p0 = At(pts, loop, i - 1); var p1 = At(pts, loop, i); var p2 = At(pts, loop, i + 1); var p3 = At(pts, loop, i + 2);
        var c1 = p2 - p0;
        var c2 = p0 * 2f - p1 * 5f + p2 * 4f - p3;
        var c3 = p1 * 3f - p0 - p2 * 3f + p3;
        p = (p1 * 2f + c1 * u + c2 * (u * u) + c3 * (u * u * u)) * 0.5f;
        dp = (c1 + c2 * (2f * u) + c3 * (3f * u * u)) * 0.5f;
    }

    /// <summary>
    /// The point of the path nearest to <paramref name="x"/>: a coarse scan of 8 samples per segment (the lowest parameter wins a tie), then a bisection on the sign of (P(t) - x)·P'(t) inside the best sample's neighbourhood.
    /// </summary>
    public static void Closest(Vector3F[] pts, bool loop, Vector3F x, out float t, out Vector3F p, out Vector3F tangent)
    {
        int segs = Segments(pts, loop);
        int total = segs * SamplesPerSegment;
        float best = float.MaxValue, bt = 0f;
        for (int k = 0; k <= total; k++)
        {
            float tt = (float)k / SamplesPerSegment;
            Eval(pts, loop, tt, out var q, out _);
            float d2 = (q - x).LengthSquared();
            if (d2 < best) { best = d2; bt = tt; }
        }

        float lo = bt - 1f / SamplesPerSegment, hi = bt + 1f / SamplesPerSegment;
        if (lo < 0f) { lo = 0f; }
        if (hi > segs) { hi = segs; }
        for (int it = 0; it < 16; it++)
        {
            float mid = 0.5f * (lo + hi);
            Eval(pts, loop, mid, out var q, out var dq);
            if (Vector3F.Dot(q - x, dq) > 0f) { hi = mid; } else { lo = mid; }
        }

        float cand = 0.5f * (lo + hi);
        Eval(pts, loop, cand, out var pc, out var dc);
        if ((pc - x).LengthSquared() <= best) { t = cand; p = pc; tangent = dc; }
        else { t = bt; Eval(pts, loop, bt, out p, out tangent); }
    }

    /// <summary>An orthonormal frame from a tangent and an up hint: x along the tangent, y towards the hint, z = x × y. Degenerate inputs fall back to fixed choices.</summary>
    public static void Frame(Vector3F tangent, Vector3F up, out Vector3F x, out Vector3F y, out Vector3F z)
    {
        float l2 = tangent.LengthSquared();
        x = l2 > 1e-20f && float.IsFinite(l2) ? tangent * (1f / float.Sqrt(l2)) : new Vector3F(1f, 0f, 0f);
        y = up - x * Vector3F.Dot(up, x);
        float yl = y.LengthSquared();
        if (!(yl > 1e-8f)) { y = Perpendicular(x); }
        else { y = y * (1f / float.Sqrt(yl)); }
        z = Vector3F.Cross(x, y);
    }

    /// <summary>A unit vector perpendicular to a unit vector, by a fixed rule.</summary>
    public static Vector3F Perpendicular(Vector3F n)
    {
        var t = Math.Abs(n.X) >= 0.57735f ? new Vector3F(n.Y, -n.X, 0f) : new Vector3F(0f, n.Z, -n.Y);
        float l = t.Length();
        return l > 0f ? t * (1f / l) : new Vector3F(0f, 1f, 0f);
    }

    /// <summary>The rotation whose columns are the given orthonormal axes (a right-handed basis).</summary>
    public static QuaternionF FromBasis(Vector3F x, Vector3F y, Vector3F z)
    {
        float m00 = x.X, m10 = x.Y, m20 = x.Z, m01 = y.X, m11 = y.Y, m21 = y.Z, m02 = z.X, m12 = z.Y, m22 = z.Z;
        float tr = m00 + m11 + m22;
        float qx, qy, qz, qw;
        if (tr > 0f)
        {
            float s = float.Sqrt(tr + 1f) * 2f;
            qw = 0.25f * s; qx = (m21 - m12) / s; qy = (m02 - m20) / s; qz = (m10 - m01) / s;
        }
        else if (m00 > m11 && m00 > m22)
        {
            float s = float.Sqrt(1f + m00 - m11 - m22) * 2f;
            qw = (m21 - m12) / s; qx = 0.25f * s; qy = (m01 + m10) / s; qz = (m02 + m20) / s;
        }
        else if (m11 > m22)
        {
            float s = float.Sqrt(1f + m11 - m00 - m22) * 2f;
            qw = (m02 - m20) / s; qx = (m01 + m10) / s; qy = 0.25f * s; qz = (m12 + m21) / s;
        }
        else
        {
            float s = float.Sqrt(1f + m22 - m00 - m11) * 2f;
            qw = (m10 - m01) / s; qx = (m02 + m20) / s; qy = (m12 + m21) / s; qz = 0.25f * s;
        }

        float n = float.Sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        return n > 0f ? new QuaternionF(qx / n, qy / n, qz / n, qw / n) : QuaternionF.Identity;
    }
}
