using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// A smooth curve through a list of points: a uniform Catmull-Rom spline (each segment is a cubic Hermite curve whose end tangents are half the chord between the neighbouring points; the end points of an open path
/// repeat). Used by path joints. Parameter t runs from 0 to the number of segments; segment i spans [i, i + 1].
/// </summary>
internal static class PathSpline2D
{
    private const int SamplesPerSegment = 8;

    private static Vector2F At(Vector2F[] pts, bool loop, int i)
    {
        int n = pts.Length;
        if (loop) { i %= n; if (i < 0) { i += n; } }
        else if (i < 0) { i = 0; }
        else if (i >= n) { i = n - 1; }
        return pts[i];
    }

    public static int Segments(Vector2F[] pts, bool loop) => loop ? pts.Length : pts.Length - 1;

    /// <summary>The point and its derivative with respect to t.</summary>
    public static void Eval(Vector2F[] pts, bool loop, float t, out Vector2F p, out Vector2F dp)
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
    public static void Closest(Vector2F[] pts, bool loop, Vector2F x, out float t, out Vector2F p, out Vector2F tangent)
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
            if (Vector2F.Dot(q - x, dq) > 0f) { hi = mid; } else { lo = mid; }
        }

        float cand = 0.5f * (lo + hi);
        Eval(pts, loop, cand, out var pc, out var dc);
        if ((pc - x).LengthSquared() <= best) { t = cand; p = pc; tangent = dc; }
        else { t = bt; Eval(pts, loop, bt, out p, out tangent); }
    }
}
