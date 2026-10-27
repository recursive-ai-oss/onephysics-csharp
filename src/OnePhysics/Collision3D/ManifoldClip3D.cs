using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Collision3D;

// A contact feature of one shape: a point, a segment or a convex polygon, as points on the shape's surface (core points moved out by the convex radius),
// each with a feature code for the corner and for the edge that leaves it towards the next corner.
internal sealed class Feat
{
    public const int None = 0, Point = 1, Segment = 2, Polygon = 3;

    public int Kind;
    public int N;
    public Vector3F[] P = new Vector3F[16];
    public ushort[] V = new ushort[16];
    public ushort[] E = new ushort[16];
    public ushort Face;

    public void Reset()
    {
        Kind = None;
        N = 0;
        Face = 0;
    }

    public void Add(Vector3F p, ushort vertex, ushort edge)
    {
        if (N == P.Length)
        {
            Array.Resize(ref P, N * 2);
            Array.Resize(ref V, N * 2);
            Array.Resize(ref E, N * 2);
        }

        P[N] = p;
        V[N] = vertex;
        E[N] = edge;
        N++;
    }

    public void Transform(in Matrix3x3F r, Vector3F t)
    {
        for (int i = 0; i < N; i++)
        {
            P[i] = (r * P[i]) + t;
        }
    }

    public float AreaTwice()
    {
        if (Kind != Polygon)
        {
            return 0f;
        }

        Vector3F s = default;
        for (int i = 1; i + 1 < N; i++)
        {
            s += Vector3F.Cross(P[i] - P[0], P[i + 1] - P[0]);
        }

        return ConvexShape3D.Sqrt(s.LengthSquared());
    }

    public ushort Code() => Kind == Polygon ? Face : Kind == Segment ? E[0] : V[0];
}

public sealed partial class ManifoldGenerator3D
{
    private struct ClipPt
    {
        public Vector3F P;
        public ushort Ref, Inc, Out;
    }

    private ConvexContact3D _hint;
    private bool _hasHint;
    private float _closeTol;
    private ClipPt[] _c0 = new ClipPt[32];
    private ClipPt[] _c1 = new ClipPt[32];

    // Turns the contact features of A (outward along n) and B (outward along -n), both in A's frame, into a manifold.
    // A polygon is the reference face and the other feature is clipped against its sides (Sutherland and Hodgman; Ericson, Real-Time Collision Detection, ch. 5); with no polygon the contact is a single closest pair
    // or, for parallel segments, the two ends of their overlap.
    private Manifold3D FromFeatures(Feat fa, Feat fb, Vector3F n, float margin)
    {
        _count = 0;
        if (fa.Kind == Feat.Polygon || fb.Kind == Feat.Polygon)
        {
            bool refIsA = fa.Kind == Feat.Polygon && (fb.Kind != Feat.Polygon || fa.AreaTwice() * 1.05f >= fb.AreaTwice());
            Feat rf = refIsA ? fa : fb;
            Feat inc = refIsA ? fb : fa;
            ClipFace(rf, inc, refIsA ? n : -n, refIsA, margin);
        }
        else
        {
            ClipSimple(fa, fb, n, margin);
        }

        if (_hasHint)
        {
            // Clipping can drop the closest pair (a corner just outside the reference face's sides). The GJK/EPA contact is exact, so it joins the manifold
            // whenever the clipped points are farther apart than it is.
            float min = float.PositiveInfinity;
            for (int i = 0; i < _count; i++)
            {
                min = Math.Min(min, _pts[i].Sep);
            }

            if ((_count == 0 || min > _hint.Separation + _closeTol) && _hint.Separation <= margin)
            {
                Push((_hint.PointA + _hint.PointB) * 0.5f, _hint.Separation, FCode.Pack(FCode.Derived(1), FCode.Derived(1)));
            }

            _hasHint = false;
        }

        return Finish(n);
    }

    private void EnsureClip(int cap)
    {
        if (_c0.Length < cap)
        {
            _c0 = new ClipPt[cap];
            _c1 = new ClipPt[cap];
        }
    }

    // Clips the incident feature against the side planes of the reference polygon; keeps points within the margin of the reference plane.
    // nRef is the outward normal of the reference polygon. Ids pair the reference face with a surviving incident corner, or a reference edge with an incident edge it cut.
    private void ClipFace(Feat rf, Feat inc, Vector3F nRef, bool refIsA, float margin)
    {
        int n = rf.N;
        EnsureClip(inc.N + n + 8);
        Vector3F cen = default;
        for (int i = 0; i < n; i++)
        {
            cen += rf.P[i];
        }

        cen /= n;

        // The reference plane through the polygon (Newell's sum about the centroid). When the polygon is not perpendicular to the contact normal (GJK picked an edge-ish direction)
        // the separation is measured along the contact normal to that plane; a polygon far from perpendicular falls back to the normal's own plane.
        Vector3F planeN = default;
        for (int i = 0; i < n; i++)
        {
            planeN += Vector3F.Cross(rf.P[i] - cen, rf.P[(i + 1) % n] - cen);
        }

        float pl2 = planeN.LengthSquared();
        float cosv = 0f;
        if (pl2 > 0f)
        {
            planeN /= ConvexShape3D.Sqrt(pl2);
            cosv = Vector3F.Dot(planeN, nRef);
            if (cosv < 0f)
            {
                planeN = -planeN;
                cosv = -cosv;
            }
        }

        if (!(cosv >= 0.5f))
        {
            planeN = nRef;
            cosv = 1f;
        }

        float anchor = Vector3F.Dot(planeN, cen);

        ClipPt[] src = _c0, dst = _c1;
        int m = inc.N;
        for (int i = 0; i < m; i++)
        {
            src[i] = new ClipPt { P = inc.P[i], Ref = rf.Face, Inc = inc.V[i], Out = inc.E[i] };
        }

        int cap = src.Length;
        bool segment = inc.Kind == Feat.Segment;
        for (int i = 0; i < n && m > 0; i++)
        {
            Vector3F a = rf.P[i], b = rf.P[(i + 1) % n];
            Vector3F s = Vector3F.Cross(b - a, nRef);
            if (Vector3F.Dot(s, cen - a) < 0f)
            {
                s = -s;
            }

            if (!(s.LengthSquared() > 0f))
            {
                continue;
            }

            ushort refEdge = rf.E[i];
            if (segment)
            {
                float d0 = Vector3F.Dot(s, src[0].P - a), d1 = Vector3F.Dot(s, src[1].P - a);
                if (d0 < 0f && d1 < 0f)
                {
                    m = 0;
                }
                else if (d0 < 0f)
                {
                    src[0] = new ClipPt { P = src[0].P + ((src[1].P - src[0].P) * (d0 / (d0 - d1))), Ref = refEdge, Inc = inc.E[0], Out = inc.E[0] };
                }
                else if (d1 < 0f)
                {
                    src[1] = new ClipPt { P = src[1].P + ((src[0].P - src[1].P) * (d1 / (d1 - d0))), Ref = refEdge, Inc = inc.E[0], Out = inc.E[0] };
                }

                continue;
            }

            if (inc.Kind == Feat.Point)
            {
                // A lone corner counts only where it faces the reference polygon.
                if (Vector3F.Dot(s, src[0].P - a) < 0f)
                {
                    m = 0;
                }

                continue;
            }

            int outCount = 0;
            for (int j = 0; j < m; j++)
            {
                ClipPt cur = src[j], nxt = src[(j + 1) % m];
                float dc = Vector3F.Dot(s, cur.P - a), dn = Vector3F.Dot(s, nxt.P - a);
                bool curIn = dc >= 0f, nxtIn = dn >= 0f;
                if (curIn == nxtIn)
                {
                    if (nxtIn && outCount < cap)
                    {
                        dst[outCount++] = nxt;
                    }

                    continue;
                }

                Vector3F q = cur.P + ((nxt.P - cur.P) * (dc / (dc - dn)));
                if (outCount < cap)
                {
                    dst[outCount++] = new ClipPt { P = q, Ref = refEdge, Inc = cur.Out, Out = curIn ? FCode.Derived(refEdge) : cur.Out };
                }

                if (nxtIn && outCount < cap)
                {
                    dst[outCount++] = nxt;
                }
            }

            (src, dst) = (dst, src);
            m = outCount;
        }

        for (int i = 0; i < m; i++)
        {
            ClipPt c = src[i];
            float sep = (Vector3F.Dot(planeN, c.P) - anchor) / cosv;
            if (sep > margin)
            {
                continue;
            }

            uint id = refIsA ? FCode.Pack(c.Ref, c.Inc) : FCode.Pack(c.Inc, c.Ref);
            Push(c.P - (nRef * (sep * 0.5f)), sep, id);
        }
    }

    // Closest points of segment (p1, q1) and segment (p2, q2) (Ericson 5.1.9); s and t are the parameters along each, exactly 0 or 1 when clamped to an end.
    internal static void ClosestSegmentSegment(Vector3F p1, Vector3F q1, Vector3F p2, Vector3F q2, out float s, out float t)
    {
        Vector3F d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
        float a = d1.LengthSquared(), e = d2.LengthSquared(), f = Vector3F.Dot(d2, r);
        float eps = 1e-12f;
        if (a <= eps && e <= eps)
        {
            s = 0f;
            t = 0f;
            return;
        }

        if (a <= eps)
        {
            s = 0f;
            t = Clamp01(f / e);
            return;
        }

        float c = Vector3F.Dot(d1, r);
        if (e <= eps)
        {
            t = 0f;
            s = Clamp01(-c / a);
            return;
        }

        float b = Vector3F.Dot(d1, d2);
        float denom = (a * e) - (b * b);
        s = denom > 0f ? Clamp01(((b * f) - (c * e)) / denom) : 0f;
        t = ((b * s) + f) / e;
        if (t < 0f)
        {
            t = 0f;
            s = Clamp01(-c / a);
        }
        else if (t > 1f)
        {
            t = 1f;
            s = Clamp01((b - c) / a);
        }
    }

    internal static float Clamp01(float x) => x < 0f ? 0f : x > 1f ? 1f : x;

    // Point and segment combinations without a polygon: one closest pair, or the two ends of the overlap of parallel segments.
    private void ClipSimple(Feat fa, Feat fb, Vector3F n, float margin)
    {
        if (fa.Kind == Feat.Point && fb.Kind == Feat.Point)
        {
            AddPair(fa.P[0], fb.P[0], n, fa.V[0], fb.V[0], margin);
            return;
        }

        if (fa.Kind == Feat.Point || fb.Kind == Feat.Point)
        {
            bool pointIsA = fa.Kind == Feat.Point;
            Feat pt = pointIsA ? fa : fb, sg = pointIsA ? fb : fa;
            // Closest in the view along the normal: the pair that faces each other, so the separation along the normal is the real gap.
            Vector3F d = sg.P[1] - sg.P[0];
            Vector3F dp = d - (n * Vector3F.Dot(n, d));
            float len2 = dp.LengthSquared();
            float t = len2 > 1e-12f * Math.Max(1e-12f, d.LengthSquared()) ? Clamp01(Vector3F.Dot(pt.P[0] - sg.P[0], dp) / len2) : 0.5f;
            Vector3F q = sg.P[0] + (d * t);
            ushort sc = t <= 0f ? sg.V[0] : t >= 1f ? sg.V[1] : sg.E[0];
            if (pointIsA)
            {
                AddPair(pt.P[0], q, n, pt.V[0], sc, margin);
            }
            else
            {
                AddPair(q, pt.P[0], n, sc, pt.V[0], margin);
            }

            return;
        }

        // Segment against segment, in the view along the normal: segments that cross there meet in one point; segments that run parallel there overlap along a stretch,
        // and its two ends are the contacts (a capsule lying along a box edge, or flat on a face's edge). The separation at each is the gap along the normal.
        Vector3F a0 = fa.P[0], a1 = fa.P[1], b0 = fb.P[0], b1 = fb.P[1];
        Vector3F pa0 = Proj(a0, n), pa1 = Proj(a1, n), pb0 = Proj(b0, n), pb1 = Proj(b1, n);
        Vector3F da = pa1 - pa0, db = pb1 - pb0;
        float la2 = da.LengthSquared(), lb2 = db.LengthSquared();
        Vector3F cr = Vector3F.Cross(da, db);
        // Within about half a degree counts as parallel: the crossing point of such segments is ill-conditioned, the overlap's ends are not.
        if (la2 > 0f && lb2 > 0f && cr.LengthSquared() <= 1e-4f * la2 * lb2)
        {
            float la = ConvexShape3D.Sqrt(la2);
            Vector3F ea = da / la;
            float tb0 = Vector3F.Dot(pb0 - pa0, ea), tb1 = Vector3F.Dot(pb1 - pa0, ea);
            float bLo = Math.Min(tb0, tb1), bHi = Math.Max(tb0, tb1);
            float lo = Math.Max(0f, bLo), hi = Math.Min(la, bHi);
            if (hi - lo > 1e-5f * la)
            {
                for (int k = 0; k < 2; k++)
                {
                    float at = k == 0 ? lo : hi;
                    Vector3F pa = a0 + ((a1 - a0) * (at / la));
                    float span = tb1 - tb0;
                    Vector3F pb = b0 + ((b1 - b0) * Clamp01((at - tb0) / span));
                    ushort ca, cb;
                    if (k == 0)
                    {
                        bool fromA = bLo <= 0f;
                        ca = fromA ? fa.V[0] : fa.E[0];
                        cb = fromA ? fb.E[0] : (tb0 < tb1 ? fb.V[0] : fb.V[1]);
                    }
                    else
                    {
                        bool fromA = bHi >= la;
                        ca = fromA ? fa.V[1] : fa.E[0];
                        cb = fromA ? fb.E[0] : (tb0 < tb1 ? fb.V[1] : fb.V[0]);
                    }

                    AddPair(pa, pb, n, ca, cb, margin);
                }

                return;
            }
        }

        ClosestSegmentSegment(pa0, pa1, pb0, pb1, out float s, out float tt);
        ushort sa = s <= 0f ? fa.V[0] : s >= 1f ? fa.V[1] : fa.E[0];
        ushort sb = tt <= 0f ? fb.V[0] : tt >= 1f ? fb.V[1] : fb.E[0];
        AddPair(a0 + ((a1 - a0) * s), b0 + ((b1 - b0) * tt), n, sa, sb, margin);
    }

    private static Vector3F Proj(Vector3F v, Vector3F n) => v - (n * Vector3F.Dot(n, v));

    // A point from surface points on A and B: separation along the normal, position midway.
    private void AddPair(Vector3F pa, Vector3F pb, Vector3F n, ushort ca, ushort cb, float margin)
    {
        float sep = Vector3F.Dot(n, pb - pa);
        if (sep > margin)
        {
            return;
        }

        Push((pa + pb) * 0.5f, sep, FCode.Pack(ca, cb));
    }
}
