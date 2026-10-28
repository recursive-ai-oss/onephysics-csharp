using System;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

// Ray against a rounded convex core (Ericson, Real-Time Collision Detection ch. 5.3). The rounded shape is the core offset by its radius: its boundary is made of
// the core's edges pushed out along their normals and of circles at the vertices. Every candidate point lies inside or on the shape, so the first one along the ray
// is the entry point.
internal static class RayCasts2D
{
    internal static RayCastOutput2D RayCast(in Shape2D s, Vec2 origin, Vec2 d, float maxFraction)
    {
        if (s._count == 0 || !origin.IsFinite || !d.IsFinite || !(maxFraction >= 0f) || !float.IsFinite(maxFraction)) { return default; }
        int n = s._count;
        float r = s._radius;
        bool segmentLike = s._kind == ShapeKind2D.Segment || s._kind == ShapeKind2D.ChainSegment;

        if (!segmentLike && Inside(in s, origin))
        {
            return new RayCastOutput2D(true, 0f, default, origin);
        }

        float bestT = float.MaxValue;
        Vec2 bestN = default;
        bool hit = false;

        int edges = n == 2 ? (r > 0f ? 2 : 1) : (n >= 3 ? n : 0);
        for (int i = 0; i < edges; i++)
        {
            Vec2 nrm = s._n[i];
            float dn = Vec2.Dot(d, nrm);
            if (segmentLike && i == 0)
            {
                // A segment has two faces; a one-sided chain segment only its front.
                if (dn > 0f)
                {
                    if (s._kind == ShapeKind2D.ChainSegment && (s._flags & Shape2D.FlagTwoSided) == 0) { continue; }
                    nrm = -nrm;
                    dn = -dn;
                }
            }

            if (!(dn < 0f)) { continue; }
            Vec2 a = s._v[i] + r * s._n[i];
            Vec2 b = s._v[(i + 1) % n] + r * s._n[i];
            if (segmentLike && Vec2.Dot(nrm, s._n[i]) < 0f) { a = s._v[i]; b = s._v[(i + 1) % n]; }
            float t = Vec2.Dot(nrm, a - origin) / dn;
            if (!(t >= 0f) || t > maxFraction || !(t < bestT)) { continue; }
            Vec2 q = origin + t * d;
            Vec2 ab = b - a;
            float u = Vec2.Dot(q - a, ab);
            if (u < 0f || u > Vec2.Dot(ab, ab)) { continue; }
            bestT = t;
            bestN = nrm;
            hit = true;
        }

        if (r > 0f)
        {
            float aa = Vec2.Dot(d, d);
            if (aa > 0f)
            {
                for (int i = 0; i < n; i++)
                {
                    Vec2 m = origin - s._v[i];
                    float b = Vec2.Dot(m, d);
                    float c = Vec2.Dot(m, m) - r * r;
                    if (c <= 0f || b >= 0f) { continue; }
                    float disc = b * b - aa * c;
                    if (disc < 0f) { continue; }
                    float t = (-b - float.Sqrt(disc)) / aa;
                    if (!(t >= 0f) || t > maxFraction || !(t < bestT)) { continue; }
                    bestT = t;
                    bestN = (origin + t * d - s._v[i]) * (1f / r);
                    hit = true;
                }
            }
        }

        return hit ? new RayCastOutput2D(true, bestT, bestN, origin + bestT * d) : default;
    }

    // True when the point lies within the rounded shape (distance to the core at most the radius; inside the core counts as distance 0).
    private static bool Inside(in Shape2D s, Vec2 p)
    {
        int n = s._count;
        float r = s._radius;
        if (n >= 3)
        {
            bool inCore = true;
            for (int i = 0; i < n; i++)
            {
                if (Vec2.Dot(s._n[i], p - s._v[i]) > 0f) { inCore = false; break; }
            }

            if (inCore) { return true; }
        }

        if (!(r > 0f)) { return false; }
        float best = float.MaxValue;
        if (n == 1) { best = (p - s._v[0]).LengthSquared; }
        else
        {
            for (int i = 0; i < (n == 2 ? 1 : n); i++)
            {
                float dd = Distance2D.PointSegmentSquared(p, s._v[i], s._v[(i + 1) % n]);
                if (dd < best) { best = dd; }
            }
        }

        return best <= r * r;
    }
}
