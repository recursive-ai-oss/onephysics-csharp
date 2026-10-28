using System;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Collision2D;

/// <summary>One point of a contact manifold.</summary>
public readonly struct ManifoldPoint2D
{
    /// <summary>The contact point, midway between the two surfaces, in the frame of shape A.</summary>
    public Vec2 Point { get; }

    /// <summary>Signed distance between the surfaces along the manifold normal: negative when the shapes overlap, positive for a speculative point that is not touching yet.</summary>
    public float Separation { get; }

    /// <summary>
    /// Feature id, stable while the same pair of geometric features (a vertex or edge of each shape) stays in contact, so warm starting can match points between frames.
    /// The high byte names the feature of shape A and the low byte the feature of shape B: values 0–127 are vertex indices, values 128–255 are edge indices plus 128
    /// (the edge from vertex i to vertex i + 1 of the shape's core; a capsule or segment has edges 0 and 1, the second being the same segment seen from the other side).
    /// </summary>
    public uint Id { get; }

    /// <summary>Creates a point.</summary>
    public ManifoldPoint2D(Vec2 point, float separation, uint id)
    {
        Point = point;
        Separation = separation;
        Id = id;
    }
}

/// <summary>
/// Contact manifold of two convex shapes: a unit normal and up to two points, all in the frame of shape A. Points whose surfaces are
/// farther apart than the speculative distance are not included, so an empty manifold means "not close enough to matter".
/// </summary>
public readonly struct Manifold2D
{
    /// <summary>Unit normal pointing from shape A to shape B, in A's frame. Zero when there are no points.</summary>
    public Vec2 Normal { get; }

    /// <summary>Number of points (0, 1 or 2).</summary>
    public int PointCount { get; }

    /// <summary>First point (valid when <see cref="PointCount"/> ≥ 1).</summary>
    public ManifoldPoint2D Point0 { get; }

    /// <summary>Second point (valid when <see cref="PointCount"/> is 2).</summary>
    public ManifoldPoint2D Point1 { get; }

    internal Manifold2D(Vec2 normal, int count, ManifoldPoint2D p0, ManifoldPoint2D p1)
    {
        Normal = normal;
        PointCount = count;
        Point0 = p0;
        Point1 = p1;
    }

    /// <summary>Returns point <paramref name="i"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="i"/> is not in 0 to <see cref="PointCount"/> − 1.</exception>
    public ManifoldPoint2D this[int i]
    {
        get
        {
            if ((uint)i >= (uint)PointCount) { throw new ArgumentOutOfRangeException(nameof(i), i, "point index must be below PointCount (" + PointCount + ")"); }
            return i == 0 ? Point0 : Point1;
        }
    }
}

// A convex core in the frame of shape A, with its radius. Circles have one vertex, capsules and segments two, polygons three to eight.
internal struct Poly
{
    public int N;
    public float R;
    public Vec2x8 V;
    public Vec2x8 Nm;

    public static Poly From(in Shape2D s)
    {
        var p = new Poly { N = s._count, R = s._radius, V = s._v, Nm = s._n };
        return p;
    }

    // Fill an existing, uncleared Poly in place. Building one by value and copying it out costs more than the transform: scalar stores followed by wide loads of the same bytes stall.
    // Only the first N entries of V and Nm are set (Nm only when N >= 2); nothing reads past N.
    public static void Load(ref Poly p, in Shape2D s)
    {
        p.N = s._count; p.R = s._radius; p.V = s._v; p.Nm = s._n;
    }

    public static void LoadTransformed(ref Poly p, in Shape2D s, Transform2 xf)
    {
        int n = s._count;
        p.N = n; p.R = s._radius;
        for (int i = 0; i < n; i++) { p.V[i] = xf.Apply(s._v[i]); }
        if (n >= 2) { for (int i = 0; i < n; i++) { p.Nm[i] = xf.Q.Rotate(s._n[i]); } }
    }

    public static Poly From(in Shape2D s, Transform2 xf)
    {
        var p = new Poly { N = s._count, R = s._radius };
        for (int i = 0; i < s._count; i++) { p.V[i] = xf.Apply(s._v[i]); }
        if (s._count >= 2) { for (int i = 0; i < s._count; i++) { p.Nm[i] = xf.Q.Rotate(s._n[i]); } }
        return p;
    }
}

/// <summary>
/// Contact manifolds between 2D shapes, up to two points, found by SAT on reference and incident faces with clipping
/// (Gregorius, GDC 2013, "The Separating Axis Test between Convex Polyhedra"; Catto, GDC 2007 and 2010, contact manifolds; Ericson, Real-Time Collision Detection ch. 5).
/// A circle is a one-point core, a capsule or segment a two-point core, so every pair of shapes goes through the same code; rounded cores add their radii to the separation.
/// </summary>
public static class Collide2D
{
    // Reference faces change from A to B only when B's separation is better by this much, which keeps feature ids from flickering.
    private const float FlipTolerance = 0.1f * Constants2D.LinearSlop;

    // Contact normals within this sine of an edge normal count as the edge's own normal when filtering chain contacts (about 0.014 degrees).
    private const float GhostTolerance = 2.44140625e-4f;

    /// <summary>
    /// Computes the manifold of two shapes. Shape B's pose in A's frame is given as one transform so the caller can subtract body positions in double precision first.
    /// Chain segments collide with every shape except other chain segments (static geometry); the ghost-vertex rules are in <c>docs/collision2d.md</c>.
    /// </summary>
    /// <param name="a">Shape A, in its own frame.</param>
    /// <param name="b">Shape B, in its own frame.</param>
    /// <param name="xfBInA">The pose of B's frame in A's frame; must be finite with a unit rotation (otherwise the result is empty).</param>
    /// <param name="speculativeDistance">Points whose surfaces are farther apart than this are dropped (default <see cref="Constants2D.SpeculativeDistance"/>).</param>
    /// <returns>The manifold in A's frame, normal from A to B; empty when the shapes are farther apart than the speculative distance.</returns>
    public static Manifold2D Collide(in Shape2D a, in Shape2D b, Transform2 xfBInA, float speculativeDistance = Constants2D.SpeculativeDistance)
    {
        if (a._count == 0 || b._count == 0 || !xfBInA.IsFinite || !(speculativeDistance >= 0f) || !float.IsFinite(speculativeDistance)) { return default; }
        if (a._kind == ShapeKind2D.ChainSegment)
        {
            return b._kind == ShapeKind2D.ChainSegment ? default : CollideChain(in a, in b, xfBInA, speculativeDistance);
        }

        if (b._kind == ShapeKind2D.ChainSegment)
        {
            // Run it with the chain segment as A, then express the result in the original A's frame.
            Transform2 inv = Transform2.InvMul(xfBInA, Transform2.Identity);
            Manifold2D m = CollideChain(in b, in a, inv, speculativeDistance);
            if (m.PointCount == 0) { return default; }
            Vec2 n = -xfBInA.Q.Rotate(m.Normal);
            var p0 = new ManifoldPoint2D(xfBInA.Apply(m.Point0.Point), m.Point0.Separation, SwapId(m.Point0.Id));
            var p1 = m.PointCount > 1 ? new ManifoldPoint2D(xfBInA.Apply(m.Point1.Point), m.Point1.Separation, SwapId(m.Point1.Id)) : default;
            return new Manifold2D(n, m.PointCount, p0, p1);
        }

        if (a._count == 1 && b._count == 1)
        {
            // Two discs: the same arithmetic as Convex, without building polygons.
            Vec2 ca = a._v[0], cb = xfBInA.Apply(b._v[0]);
            Vec2 d = cb - ca;
            float len = d.Length;
            Vec2 n = len > 1e-12f ? d * (1f / len) : new Vec2(0f, 1f);
            float sep = len - (a._radius + b._radius);
            if (sep > speculativeDistance) { return default; }
            Vec2 mid = 0.5f * ((ca + a._radius * n) + (cb - b._radius * n));
            return new Manifold2D(n, 1, new ManifoldPoint2D(mid, sep, 0), default);
        }

        Mem.SkipInit(out Poly pa);
        Mem.SkipInit(out Poly pb);
        Poly.Load(ref pa, in a);
        Poly.LoadTransformed(ref pb, in b, xfBInA);
        return Convex(in pa, in pb, speculativeDistance, false);
    }

    private static uint SwapId(uint id) => ((id & 0xFFu) << 8) | ((id >> 8) & 0xFFu);

    private static uint Feature(uint a, uint b) => (a << 8) | b;

    private const uint EdgeFlag = 0x80u;

    // Largest over the reference faces of the smallest distance from the face plane to the other core's vertices; ties go to the lowest face index.
    // A face that already separates the shapes by more than the radii plus the margin settles the pair: the answer is then +infinity, which the caller reads as "too far apart".
    // The caller's own test is best - rT > margin and best is at least this face's value, so stopping early cannot change a result.
    private static float FindMaxSeparation(in Poly r, in Poly o, int faces, float rT, float margin, out int edge)
    {
        float best = -float.MaxValue;
        edge = 0;
        ReadOnlySpan<Vec2> rv = r.V, rn = r.Nm, ov = o.V;
        int on = o.N;
        for (int i = 0; i < faces; i++)
        {
            Vec2 nrm = rn[i];
            Vec2 v = rv[i];
            float min = float.MaxValue;
            for (int j = 0; j < on; j++)
            {
                float s = Vec2.Dot(nrm, ov[j] - v);
                if (s < min) { min = s; }
            }

            if (min - rT > margin) { return float.PositiveInfinity; }
            if (min > best) { best = min; edge = i; }
        }

        return best;
    }

    private static Manifold2D Convex(in Poly pa, in Poly pb, float margin, bool onlyFace0OfA, bool forceReferenceA = false)
    {
        float rT = pa.R + pb.R;
        if (pa.N == 1 && pb.N == 1)
        {
            Vec2 d = pb.V[0] - pa.V[0];
            float len = d.Length;
            Vec2 n = len > 1e-12f ? d * (1f / len) : new Vec2(0f, 1f);
            float sep = len - rT;
            if (sep > margin) { return default; }
            Vec2 mid = 0.5f * ((pa.V[0] + pa.R * n) + (pb.V[0] - pb.R * n));
            return new Manifold2D(n, 1, new ManifoldPoint2D(mid, sep, 0), default);
        }

        float sepA = -float.MaxValue, sepB = -float.MaxValue;
        int edgeA = 0, edgeB = 0;
        if (pa.N >= 2) { sepA = FindMaxSeparation(in pa, in pb, onlyFace0OfA ? 1 : pa.N, rT, margin, out edgeA); }
        if (float.IsPositiveInfinity(sepA)) { return default; }
        if (pb.N >= 2) { sepB = FindMaxSeparation(in pb, in pa, pb.N, rT, margin, out edgeB); }
        float best = sepA > sepB ? sepA : sepB;
        if (best - rT > margin) { return default; }

        bool flip = pa.N < 2 || (!forceReferenceA && pb.N >= 2 && sepB > sepA + FlipTolerance);
        return flip ? Face(in pb, in pa, edgeB, true, margin) : Face(in pa, in pb, edgeA, false, margin);
    }

    // Closest points of two segments (Ericson 5.1.9); s and t are exactly 0 or 1 when the closest point is an endpoint.
    private static void SegSeg(Vec2 p1, Vec2 q1, Vec2 p2, Vec2 q2, out float s, out float t, out Vec2 c1, out Vec2 c2)
    {
        Vec2 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
        float a = Vec2.Dot(d1, d1), e = Vec2.Dot(d2, d2), f = Vec2.Dot(d2, r);
        float c = Vec2.Dot(d1, r);
        if (!(e > 0f))
        {
            t = 0f;
            s = a > 0f ? Clamp01(-c / a) : 0f;
        }
        else
        {
            float b = Vec2.Dot(d1, d2);
            float denom = a * e - b * b;
            s = denom > 0f ? Clamp01((b * f - c * e) / denom) : 0f;
            t = (b * s + f) / e;
            if (t < 0f) { t = 0f; s = Clamp01(-c / a); }
            else if (t > 1f) { t = 1f; s = Clamp01((b - c) / a); }
        }

        c1 = p1 + s * d1;
        c2 = p2 + t * d2;
    }

    private static float Clamp01(float x) => x < 0f ? 0f : x > 1f ? 1f : x;

    // Manifold with reference face i of R and the most anti-parallel face of I as the incident edge. Everything is in A's frame; flip says that R is shape B.
    private static Manifold2D Face(in Poly r, in Poly inc, int i, bool flip, float margin)
    {
        int n = r.N;
        int i2 = (i + 1) % n;
        Vec2 v1 = r.V[i], v2 = r.V[i2];
        Vec2 nrm = r.Nm[i];
        Vec2 tan = Vec2.LeftPerp(nrm);
        float rT = r.R + inc.R;

        int ni = inc.N;
        int j1 = 0, j2 = 0;
        if (ni >= 2)
        {
            float minDot = float.MaxValue;
            for (int k = 0; k < ni; k++)
            {
                float dd = Vec2.Dot(nrm, inc.Nm[k]);
                if (dd < minDot) { minDot = dd; j1 = k; }
            }

            j2 = (j1 + 1) % ni;
        }

        Vec2 w1 = inc.V[j1], w2 = inc.V[j2];
        float coreSep = float.MaxValue;
        for (int k = 0; k < ni; k++)
        {
            float s = Vec2.Dot(nrm, inc.V[k] - v1);
            if (s < coreSep) { coreSep = s; }
        }

        Vec2 outNormal = flip ? -nrm : nrm;

        float lower = Vec2.Dot(v1, tan), upper = Vec2.Dot(v2, tan);
        float d0 = Vec2.Dot(w1, tan), d1 = Vec2.Dot(w2, tan);
        bool beside = ni == 1 ? (d0 < lower || d0 > upper) : (d0 < lower && d1 < lower) || (d0 > upper && d1 > upper);

        // The cores are apart along the face normal, or the incident feature lies wholly beside the reference edge without the cores overlapping along it
        // (a point or edge past the end of a segment, whose line distance says nothing about the gap).
        if (coreSep > FlipTolerance || (beside && coreSep >= 0f))
        {
            // The cores are apart. When the closest points of the two edges are both vertices the contact normal is the line between them,
            // not the face normal (vertex-vertex region); otherwise the faces are in contact and clipping gives the points.
            SegSeg(v1, v2, w1, w2, out float s, out float t, out Vec2 c1, out Vec2 c2);
            bool sEnd = s == 0f || s == 1f;
            bool tEnd = ni == 1 || t == 0f || t == 1f;
            if (sEnd && tEnd)
            {
                Vec2 d = c2 - c1;
                float len = d.Length;
                if (len > 0f)
                {
                    Vec2 vn = d * (1f / len);
                    float sep = len - rT;
                    if (sep > margin) { return default; }
                    uint fr = s == 0f ? (uint)i : (uint)i2;
                    uint fi = ni == 1 ? 0u : (t == 0f ? (uint)j1 : (uint)j2);
                    Vec2 mid = 0.5f * ((c1 + r.R * vn) + (c2 - inc.R * vn));
                    return new Manifold2D(flip ? -vn : vn, 1, new ManifoldPoint2D(mid, sep, flip ? Feature(fi, fr) : Feature(fr, fi)), default);
                }
            }
        }

        // Clip the incident edge against the side planes of the reference edge.
        Vec2 pt0 = w1, pt1 = w2;
        uint rf0 = EdgeFlag | (uint)i, rf1 = rf0;
        uint if0 = (uint)j1, if1 = (uint)j2;
        int count = ni == 1 ? 1 : 2;
        if (ni >= 2)
        {
            if (!beside)
            {
                float span = d1 - d0;
                if (d0 < lower) { pt0 = w1 + ((lower - d0) / span) * (w2 - w1); rf0 = (uint)i; if0 = EdgeFlag | (uint)j1; }
                else if (d0 > upper) { pt0 = w1 + ((upper - d0) / span) * (w2 - w1); rf0 = (uint)i2; if0 = EdgeFlag | (uint)j1; }
                if (d1 < lower) { pt1 = w1 + ((lower - d0) / span) * (w2 - w1); rf1 = (uint)i; if1 = EdgeFlag | (uint)j1; }
                else if (d1 > upper) { pt1 = w1 + ((upper - d0) / span) * (w2 - w1); rf1 = (uint)i2; if1 = EdgeFlag | (uint)j1; }
            }
            else
            {
                // The incident edge lies wholly beside the reference edge: keep its nearer vertex, unclipped.
                float s0 = Vec2.Dot(nrm, w1 - v1), s1 = Vec2.Dot(nrm, w2 - v1);
                if (s1 < s0) { pt0 = w2; rf0 = rf1; if0 = if1; }
                count = 1;
            }
        }

        ManifoldPoint2D m0 = default, m1 = default;
        int outCount = 0;
        for (int k = 0; k < count; k++)
        {
            Vec2 p = k == 0 ? pt0 : pt1;
            uint rf = k == 0 ? rf0 : rf1;
            uint ifeat = k == 0 ? if0 : if1;
            float sc = Vec2.Dot(nrm, p - v1);
            float sep = sc - rT;
            if (sep > margin) { continue; }
            Vec2 mid = p + nrm * (0.5f * (r.R - sc - inc.R));
            var mp = new ManifoldPoint2D(mid, sep, flip ? Feature(ifeat, rf) : Feature(rf, ifeat));
            if (outCount == 0) { m0 = mp; } else { m1 = mp; }
            outCount++;
        }

        return outCount == 0 ? default : new Manifold2D(outNormal, outCount, m0, m1);
    }

    // ---------------------------------------------------------------- chain segments

    // A chain segment (A) against any other shape (B, expressed in A's frame by xf). One-sided segments have a solid side (the right of p1 -> p2); a body whose
    // centre is behind collides with nothing. Contacts with the segment's end vertices are kept only when a smooth surface through the ghost vertices
    // could produce that normal, so a body sliding over a row of segments never feels the joints between them.
    private static Manifold2D CollideChain(in Shape2D seg, in Shape2D other, Transform2 xf, float margin)
    {
        Vec2 p1 = seg._v[0], p2 = seg._v[1];
        Vec2 g1 = seg._ghost1, g2 = seg._ghost2;
        bool has1 = (seg._flags & Shape2D.FlagGhost1) != 0, has2 = (seg._flags & Shape2D.FlagGhost2) != 0;
        Vec2 n = seg._n[0];
        Vec2 c = xf.Apply(other._centroid);
        if (Vec2.Dot(c - p1, n) < 0f)
        {
            if ((seg._flags & Shape2D.FlagTwoSided) == 0) { return default; }
            // Two-sided: mirror the segment so the body is on its front side.
            (p1, p2) = (p2, p1);
            (g1, g2) = (g2, g1);
            (has1, has2) = (has2, has1);
            n = -n;
        }

        var pa = new Poly { N = 2, R = 0f };
        pa.V[0] = p1;
        pa.V[1] = p2;
        pa.Nm[0] = n;
        pa.Nm[1] = -n;
        var pb = Poly.From(in other, xf);
        Manifold2D m = Convex(in pa, in pb, margin, true);
        if (m.PointCount == 0) { return m; }
        Manifold2D f = FilterGhosts(in m, p1, p2, g2, has1, has2, n);
        if (f.PointCount == 0)
        {
            // The other shape's own face was the minimum-translation axis, but it only touches the segment's end vertices with a normal a smooth surface
            // could not produce (a tilted body whose corner has sunk through the line of the segment, with the joint poking into its face).
            // Resolve it against the segment's face instead.
            Manifold2D alt = Convex(in pa, in pb, margin, true, true);
            if (alt.PointCount > 0) { f = FilterGhosts(in alt, p1, p2, g2, has1, has2, n); }
        }

        return f;
    }

    // Keeps the points of a chain-segment manifold that a smooth surface through the ghost vertices could produce. A normal equal to the segment's own is always kept.
    private static Manifold2D FilterGhosts(in Manifold2D m, Vec2 p1, Vec2 p2, Vec2 g2, bool has1, bool has2, Vec2 n)
    {
        Vec2 nc = m.Normal;
        if (Vec2.Dot(nc, n) > 0f && Math.Abs(Vec2.Cross(nc, n)) <= GhostTolerance) { return m; }

        Vec2 e1 = p2 - p1;
        float len = e1.Length;
        Vec2 tan = e1 * (1f / len);
        bool wedgeOk = false;
        if (has2)
        {
            Vec2 e2 = g2 - p2;
            if (Vec2.Cross(e1, e2) > 0f)
            {
                Vec2 n2 = Vec2.RightPerp(e2).Normalized();
                wedgeOk = Vec2.Cross(n, nc) >= -GhostTolerance && Vec2.Cross(nc, n2) >= -GhostTolerance;
            }
        }

        ManifoldPoint2D k0 = default, k1 = default;
        int kept = 0;
        for (int k = 0; k < m.PointCount; k++)
        {
            ManifoldPoint2D mp = k == 0 ? m.Point0 : m.Point1;
            bool atEnd2 = Vec2.Dot(mp.Point - p1, tan) >= 0.5f * len;
            // The first vertex belongs to the previous segment when there is one; the last vertex to this segment when the wedge allows it; a free end always.
            bool ok = atEnd2 ? (!has2 || wedgeOk) : !has1;
            if (!ok) { continue; }
            if (kept == 0) { k0 = mp; } else { k1 = mp; }
            kept++;
        }

        return kept == 0 ? default : new Manifold2D(nc, kept, k0, k1);
    }
}
