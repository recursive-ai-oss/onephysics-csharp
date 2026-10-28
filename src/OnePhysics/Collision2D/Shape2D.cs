using System;
using System.Runtime.Intrinsics;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePhysics.Collision2D;

/// <summary>Why a shape could not be built.</summary>
public enum ShapeError
{
    /// <summary>A coordinate or radius is NaN or infinite.</summary>
    NotFinite,

    /// <summary>A coordinate or radius is outside the allowed range (see <see cref="Constants2D.MaxShapeCoordinate"/>), or a size is not positive.</summary>
    OutOfRange,

    /// <summary>Fewer points than the shape needs.</summary>
    TooFewPoints,

    /// <summary>More input points than the builder accepts (65536).</summary>
    TooManyPoints,

    /// <summary>The points do not span an area: all within tolerance of one point or one line, or an edge is too short.</summary>
    Degenerate,

    /// <summary>The convex hull has more than <see cref="Constants2D.MaxPolygonVertices"/> vertices.</summary>
    TooManyVertices,
}

/// <summary>Raised by shape constructors for input that cannot make a valid shape. The message names what was wrong and where.</summary>
public sealed class InvalidShapeException : ArgumentException
{
    /// <summary>The category of the problem.</summary>
    public ShapeError Reason { get; }

    /// <summary>Creates the exception.</summary>
    public InvalidShapeException(ShapeError reason, string message) : base(message) { Reason = reason; }
}

/// <summary>The kind of a <see cref="Shape2D"/>.</summary>
public enum ShapeKind2D : byte
{
    /// <summary>A disc.</summary>
    Circle = 1,

    /// <summary>Two discs joined by a rectangle: a segment with a radius.</summary>
    Capsule = 2,

    /// <summary>A line segment with no thickness; collides from both sides.</summary>
    Segment = 3,

    /// <summary>A convex polygon, optionally rounded by a radius.</summary>
    Polygon = 4,

    /// <summary>A segment of a chain, with the neighbouring vertices needed to avoid snagging on internal edges.</summary>
    ChainSegment = 5,
}

/// <summary>Storage for up to eight 2D points inside a struct without allocating.</summary>
[InlineArray(Constants2D.MaxPolygonVertices)]
public struct Vec2x8
{
    private Vec2 _e0;
}

/// <summary>
/// A 2D collision shape in the local frame of its body. Every kind is stored as a convex core of 1 to 8 points plus a radius:
/// a circle is one point with a radius, a capsule two points with a radius, a segment two points with radius zero, a polygon 3 to 8 points
/// (counter-clockwise) with any radius. Collision, distance and casts work on the core and add the radius, so rounded shapes need no special cases.
/// Create shapes with the static factory methods; a default value has no points and does not collide.
/// </summary>
public struct Shape2D
{
    internal const byte FlagGhost1 = 1;
    internal const byte FlagGhost2 = 2;
    internal const byte FlagTwoSided = 4;

    internal ShapeKind2D _kind;
    internal int _count;
    internal float _radius;
    internal byte _flags;
    internal Vec2 _centroid;
    internal Vec2 _ghost1;
    internal Vec2 _ghost2;
    internal Vec2x8 _v;
    internal Vec2x8 _n;

    /// <summary>The kind of shape. A default value reports 0.</summary>
    public readonly ShapeKind2D Kind => _kind;

    /// <summary>Number of core points (0 for a default value).</summary>
    public readonly int Count => _count;

    /// <summary>The radius added around the core (0 for segments and chain segments).</summary>
    public readonly float Radius => _radius;

    /// <summary>The core's centroid (the area centroid of the core polygon, the midpoint of a segment core).</summary>
    public readonly Vec2 Centroid => _centroid;

    /// <summary>The core points, counter-clockwise for polygons.</summary>
    [UnscopedRef]
    public readonly ReadOnlySpan<Vec2> Vertices => ((ReadOnlySpan<Vec2>)_v).Slice(0, _count);

    /// <summary>Outward unit normals of the core's edges: edge <c>i</c> runs from vertex <c>i</c> to vertex <c>(i + 1) % Count</c>. Empty for a circle; a two-point core has the normal of edge 0 and its opposite.</summary>
    [UnscopedRef]
    public readonly ReadOnlySpan<Vec2> Normals => ((ReadOnlySpan<Vec2>)_n).Slice(0, _count < 2 ? 0 : _count);

    /// <summary>For a chain segment, the vertex before the first point (valid when <see cref="HasGhost1"/>).</summary>
    public readonly Vec2 Ghost1 => _ghost1;

    /// <summary>For a chain segment, the vertex after the second point (valid when <see cref="HasGhost2"/>).</summary>
    public readonly Vec2 Ghost2 => _ghost2;

    /// <summary>True when a chain segment has a neighbour before its first point.</summary>
    public readonly bool HasGhost1 => (_flags & FlagGhost1) != 0;

    /// <summary>True when a chain segment has a neighbour after its second point.</summary>
    public readonly bool HasGhost2 => (_flags & FlagGhost2) != 0;

    /// <summary>True when a chain segment collides from both sides.</summary>
    public readonly bool IsTwoSided => (_flags & FlagTwoSided) != 0;

    // ---------------------------------------------------------------- validation helpers

    internal static void CheckCoordinate(float v, string what)
    {
        if (!float.IsFinite(v))
        {
            throw new InvalidShapeException(ShapeError.NotFinite, what + " is " + v.ToString("R", CultureInfo.InvariantCulture) + ", which is not finite");
        }

        if (v > Constants2D.MaxShapeCoordinate || v < -Constants2D.MaxShapeCoordinate)
        {
            throw new InvalidShapeException(ShapeError.OutOfRange, what + " is " + v.ToString("R", CultureInfo.InvariantCulture) + ", outside the allowed range of +-" + Constants2D.MaxShapeCoordinate.ToString("R", CultureInfo.InvariantCulture));
        }
    }

    internal static void CheckPoint(Vec2 p, string what)
    {
        // The message strings are built only when a coordinate is bad, so valid shapes cost no allocation (queries build shapes on the fly).
        if (!(p.X <= Constants2D.MaxShapeCoordinate && p.X >= -Constants2D.MaxShapeCoordinate)) { CheckCoordinate(p.X, what + ".X"); }
        if (!(p.Y <= Constants2D.MaxShapeCoordinate && p.Y >= -Constants2D.MaxShapeCoordinate)) { CheckCoordinate(p.Y, what + ".Y"); }
    }

    internal static void CheckRadius(float r)
    {
        CheckCoordinate(r, "radius");
        if (r < 0f)
        {
            throw new InvalidShapeException(ShapeError.OutOfRange, "radius is " + r.ToString("R", CultureInfo.InvariantCulture) + ", it must not be negative");
        }
    }

    // ---------------------------------------------------------------- factories

    /// <summary>A circle.</summary>
    /// <param name="center">Center in the body frame.</param>
    /// <param name="radius">Radius, greater than zero and at most 2^20.</param>
    /// <exception cref="InvalidShapeException">A value is not finite, out of range, or the radius is not positive.</exception>
    public static Shape2D CreateCircle(Vec2 center, float radius)
    {
        CheckPoint(center, "center");
        CheckRadius(radius);
        if (radius <= 0f) { throw new InvalidShapeException(ShapeError.OutOfRange, "radius is 0, it must be positive"); }
        var s = new Shape2D { _kind = ShapeKind2D.Circle, _count = 1, _radius = radius, _centroid = center };
        s._v[0] = center;
        return s;
    }

    /// <summary>A capsule: the segment <paramref name="p1"/>–<paramref name="p2"/> with a radius.</summary>
    /// <exception cref="InvalidShapeException">A value is not finite or out of range, the radius is not positive, or the points are closer than <see cref="Constants2D.MinShapeExtent"/>.</exception>
    public static Shape2D CreateCapsule(Vec2 p1, Vec2 p2, float radius)
    {
        CheckPoint(p1, "p1");
        CheckPoint(p2, "p2");
        CheckRadius(radius);
        if (radius <= 0f) { throw new InvalidShapeException(ShapeError.OutOfRange, "radius is 0, it must be positive"); }
        return TwoPoint(ShapeKind2D.Capsule, p1, p2, radius, 0);
    }

    /// <summary>A line segment with no thickness. It collides from both sides.</summary>
    /// <exception cref="InvalidShapeException">A value is not finite or out of range, or the points are closer than <see cref="Constants2D.MinShapeExtent"/>.</exception>
    public static Shape2D CreateSegment(Vec2 p1, Vec2 p2)
    {
        CheckPoint(p1, "p1");
        CheckPoint(p2, "p2");
        return TwoPoint(ShapeKind2D.Segment, p1, p2, 0f, 0);
    }

    /// <summary>
    /// One segment of a chain with the vertices next to it. The solid side is to the right of the direction p1 → p2 (the side the normal (dy, -dx) points to),
    /// unless <paramref name="twoSided"/>. A ghost vertex is the neighbouring chain vertex; a missing one (<c>null</c>) marks a free end of an open chain.
    /// Contacts a body makes with the segment's end vertex are kept only when the contact normal is one a smooth surface through the ghosts could produce (see <c>docs/collision2d.md</c>).
    /// </summary>
    /// <exception cref="InvalidShapeException">A value is not finite or out of range, or two consecutive vertices are closer than <see cref="Constants2D.MinShapeExtent"/>.</exception>
    public static Shape2D CreateChainSegment(Vec2? ghost1, Vec2 p1, Vec2 p2, Vec2? ghost2, bool twoSided = false)
    {
        CheckPoint(p1, "p1");
        CheckPoint(p2, "p2");
        byte flags = twoSided ? FlagTwoSided : (byte)0;
        if (ghost1 is Vec2 g1)
        {
            CheckPoint(g1, "ghost1");
            if ((g1 - p1).Length < Constants2D.MinShapeExtent) { throw new InvalidShapeException(ShapeError.Degenerate, "ghost1 coincides with p1"); }
            flags |= FlagGhost1;
        }

        if (ghost2 is Vec2 g2)
        {
            CheckPoint(g2, "ghost2");
            if ((g2 - p2).Length < Constants2D.MinShapeExtent) { throw new InvalidShapeException(ShapeError.Degenerate, "ghost2 coincides with p2"); }
            flags |= FlagGhost2;
        }

        var s = TwoPoint(ShapeKind2D.ChainSegment, p1, p2, 0f, flags);
        s._ghost1 = ghost1 ?? default;
        s._ghost2 = ghost2 ?? default;
        return s;
    }

    private static Shape2D TwoPoint(ShapeKind2D kind, Vec2 p1, Vec2 p2, float radius, byte flags)
    {
        Vec2 e = p2 - p1;
        float len = e.Length;
        if (!(len >= Constants2D.MinShapeExtent))
        {
            throw new InvalidShapeException(ShapeError.Degenerate, "the two points are " + len.ToString("R", CultureInfo.InvariantCulture) + " apart, less than the minimum of " + Constants2D.MinShapeExtent.ToString("R", CultureInfo.InvariantCulture));
        }

        var s = new Shape2D { _kind = kind, _count = 2, _radius = radius, _flags = flags, _centroid = 0.5f * (p1 + p2) };
        s._v[0] = p1;
        s._v[1] = p2;
        Vec2 n = Vec2.RightPerp(e) * (1f / len);
        s._n[0] = n;
        s._n[1] = -n;
        return s;
    }

    /// <summary>An axis-aligned box centred on the body origin, optionally rounded.</summary>
    /// <param name="halfWidth">Half extent along x, greater than zero.</param>
    /// <param name="halfHeight">Half extent along y, greater than zero.</param>
    /// <param name="radius">Corner radius added around the box (default 0).</param>
    /// <exception cref="InvalidShapeException">A value is not finite, out of range, or an extent is not positive.</exception>
    public static Shape2D CreateBox(float halfWidth, float halfHeight, float radius = 0f) => CreateBox(halfWidth, halfHeight, default, Rot2.Identity, radius);

    /// <summary>A box with a centre and rotation in the body frame, optionally rounded.</summary>
    /// <exception cref="InvalidShapeException">A value is not finite, out of range, or an extent is not positive.</exception>
    public static Shape2D CreateBox(float halfWidth, float halfHeight, Vec2 center, Rot2 rotation, float radius = 0f)
    {
        CheckCoordinate(halfWidth, "halfWidth");
        CheckCoordinate(halfHeight, "halfHeight");
        CheckPoint(center, "center");
        CheckRadius(radius);
        CheckCoordinate(rotation.C, "rotation.C");
        CheckCoordinate(rotation.S, "rotation.S");
        if (!(halfWidth >= Constants2D.MinShapeExtent) || !(halfHeight >= Constants2D.MinShapeExtent))
        {
            throw new InvalidShapeException(ShapeError.OutOfRange, "box half extents are " + halfWidth.ToString("R", CultureInfo.InvariantCulture) + " and " + halfHeight.ToString("R", CultureInfo.InvariantCulture) + ", each must be at least " + Constants2D.MinShapeExtent.ToString("R", CultureInfo.InvariantCulture));
        }

        Rot2 q = Rot2.FromVector(rotation.C, rotation.S);
        var s = new Shape2D { _kind = ShapeKind2D.Polygon, _count = 4, _radius = radius, _centroid = center };
        s._v[0] = center + q.Rotate(new Vec2(-halfWidth, -halfHeight));
        s._v[1] = center + q.Rotate(new Vec2(halfWidth, -halfHeight));
        s._v[2] = center + q.Rotate(new Vec2(halfWidth, halfHeight));
        s._v[3] = center + q.Rotate(new Vec2(-halfWidth, halfHeight));
        s._n[0] = q.Rotate(new Vec2(0f, -1f));
        s._n[1] = q.Rotate(new Vec2(1f, 0f));
        s._n[2] = q.Rotate(new Vec2(0f, 1f));
        s._n[3] = q.Rotate(new Vec2(-1f, 0f));
        return s;
    }

    /// <summary>
    /// A convex polygon from points: their convex hull, counter-clockwise, optionally rounded by <paramref name="radius"/>.
    /// Duplicate and nearly duplicate points and points within a relative tolerance of a hull edge (2^-18 of the point set's extent) are dropped.
    /// </summary>
    /// <param name="points">Between 3 and 65536 points; each coordinate finite and within ±2^20.</param>
    /// <param name="radius">Radius added around the hull (default 0), finite, from 0 to 2^20.</param>
    /// <exception cref="InvalidShapeException">Too few or too many points, a coordinate that is NaN, infinite or out of range, a point set whose extent is below <see cref="Constants2D.MinShapeExtent"/> or that is collinear within tolerance, or a hull of more than <see cref="Constants2D.MaxPolygonVertices"/> vertices.</exception>
    public static Shape2D CreatePolygon(ReadOnlySpan<Vec2> points, float radius = 0f)
    {
        CheckRadius(radius);
        if (points.Length < 3)
        {
            throw new InvalidShapeException(ShapeError.TooFewPoints, "a polygon needs at least 3 points, got " + points.Length.ToString(CultureInfo.InvariantCulture));
        }

        if (points.Length > 65536)
        {
            throw new InvalidShapeException(ShapeError.TooManyPoints, "got " + points.Length.ToString(CultureInfo.InvariantCulture) + " points, the limit is 65536");
        }

        var pts = new Vec2[points.Length];
        float minX = float.MaxValue, minY = float.MaxValue, maxX = -float.MaxValue, maxY = -float.MaxValue;
        for (int i = 0; i < points.Length; i++)
        {
            Vec2 p = points[i];
            string where = "points[" + i.ToString(CultureInfo.InvariantCulture) + "]";
            CheckCoordinate(p.X, where + ".X");
            CheckCoordinate(p.Y, where + ".Y");
            pts[i] = p;
            if (p.X < minX) { minX = p.X; }
            if (p.X > maxX) { maxX = p.X; }
            if (p.Y < minY) { minY = p.Y; }
            if (p.Y > maxY) { maxY = p.Y; }
        }

        double dx = (double)maxX - minX, dy = (double)maxY - minY;
        double extent = dx > dy ? dx : dy;
        if (!(extent >= Constants2D.MinShapeExtent))
        {
            throw new InvalidShapeException(ShapeError.Degenerate, "the points span " + extent.ToString("R", CultureInfo.InvariantCulture) + ", less than the minimum extent of " + Constants2D.MinShapeExtent.ToString("R", CultureInfo.InvariantCulture));
        }

        double tol = extent * (1.0 / 262144.0);
        Vec2[] hull = ConvexHull(pts, tol);
        if (hull.Length < 3)
        {
            throw new InvalidShapeException(ShapeError.Degenerate, "the points are collinear within tolerance (hull has " + hull.Length.ToString(CultureInfo.InvariantCulture) + " distinct vertices)");
        }

        if (hull.Length > Constants2D.MaxPolygonVertices)
        {
            throw new InvalidShapeException(ShapeError.TooManyVertices, "the convex hull has " + hull.Length.ToString(CultureInfo.InvariantCulture) + " vertices, the maximum is " + Constants2D.MaxPolygonVertices.ToString(CultureInfo.InvariantCulture));
        }

        var s = new Shape2D { _kind = ShapeKind2D.Polygon, _count = hull.Length, _radius = radius };
        for (int i = 0; i < hull.Length; i++) { s._v[i] = hull[i]; }
        for (int i = 0; i < hull.Length; i++)
        {
            Vec2 e = hull[(i + 1) % hull.Length] - hull[i];
            Vec2 n = Vec2.RightPerp(e).Normalized();
            if (n.X == 0f && n.Y == 0f)
            {
                throw new InvalidShapeException(ShapeError.Degenerate, "hull edge " + i.ToString(CultureInfo.InvariantCulture) + " is too short to have a direction");
            }

            s._n[i] = n;
        }

        s._centroid = ComputeMass(s).Centroid;
        return s;
    }

    // Andrew's monotone chain in double (Ericson, Real-Time Collision Detection, ch. 3 / Andrew 1979). The sort order, the welding of near points
    // and the removal of near-collinear vertices only depend on the input values, never on the sort algorithm: equal points are identical.
    private static Vec2[] ConvexHull(Vec2[] pts, double tol)
    {
        Array.Sort(pts, static (a, b) => a.X < b.X ? -1 : a.X > b.X ? 1 : a.Y < b.Y ? -1 : a.Y > b.Y ? 1 : 0);
        var u = new Vec2[pts.Length];
        int n = 0;
        for (int i = 0; i < pts.Length; i++)
        {
            if (n > 0 && Math.Abs((double)pts[i].X - u[n - 1].X) <= tol && Math.Abs((double)pts[i].Y - u[n - 1].Y) <= tol) { continue; }
            u[n++] = pts[i];
        }

        if (n < 3) { return Array.Empty<Vec2>(); }
        var h = new Vec2[2 * n];
        int k = 0;
        for (int i = 0; i < n; i++)
        {
            while (k >= 2 && DistanceBelow(h[k - 2], h[k - 1], u[i], tol)) { k--; }
            h[k++] = u[i];
        }

        int lower = k + 1;
        for (int i = n - 2; i >= 0; i--)
        {
            while (k >= lower && DistanceBelow(h[k - 2], h[k - 1], u[i], tol)) { k--; }
            h[k++] = u[i];
        }

        k--; // the last point repeats the first
        var res = new Vec2[k < 0 ? 0 : k];
        Array.Copy(h, res, res.Length);
        return res;
    }

    // True when b is not strictly to the left of the line a -> c by more than tol (so b is dropped from a counter-clockwise hull).
    private static bool DistanceBelow(Vec2 a, Vec2 b, Vec2 c, double tol)
    {
        double abx = (double)b.X - a.X, aby = (double)b.Y - a.Y;
        double acx = (double)c.X - a.X, acy = (double)c.Y - a.Y;
        double cross = abx * acy - aby * acx;
        double len = Math.Sqrt(acx * acx + acy * acy);
        return cross <= tol * len;
    }

    // ---------------------------------------------------------------- bounds

    /// <summary>The axis-aligned bounds of the shape (core plus radius) under a transform.</summary>
    public readonly Aabb2 ComputeAabb(Transform2 xf)
    {
        if (_count == 0) { return new Aabb2(xf.P, xf.P); }
        // The running minimum of (x, y, -x, -y) holds the lower corner and the negated upper corner. Compare and select keep the first of equal values, as the scalar `if (p.X < lx)` does,
        // and negation is exact, so the bounds are bit for bit what four separate comparisons give; there are no branches for the data to mispredict.
        Vec2 p = xf.Apply(_v[0]);
        Vector128<float> m = Vector128.Create(p.X, p.Y, -p.X, -p.Y);
        for (int i = 1; i < _count; i++)
        {
            p = xf.Apply(_v[i]);
            Vector128<float> q = Vector128.Create(p.X, p.Y, -p.X, -p.Y);
            m = Vector128.ConditionalSelect(Vector128.LessThan(q, m), q, m);
        }

        return new Aabb2(new Vec2(m.GetElement(0) - _radius, m.GetElement(1) - _radius), new Vec2(-m.GetElement(2) + _radius, -m.GetElement(3) + _radius));
    }

    /// <summary>The axis-aligned bounds in the body frame.</summary>
    public readonly Aabb2 ComputeAabb() => ComputeAabb(Transform2.Identity);

    // ---------------------------------------------------------------- mass

    /// <summary>
    /// Area, centroid and rotational inertia. Computed in double precision relative to the shape's first vertex, so the inertia about the centroid stays accurate
    /// when the shape sits far from the body origin; the shift to another point is the parallel-axis step in <see cref="MassData2D.InertiaAbout"/>.
    /// Segments and chain segments have no area. Rounded polygons and capsules include the exact area of the rounded rim.
    /// </summary>
    public static MassData2D ComputeMass(in Shape2D s) => MassData2D.Compute(in s);

    /// <summary>Area, centroid and inertia of this shape; see <see cref="ComputeMass(in Shape2D)"/>.</summary>
    public readonly MassData2D ComputeMass() => MassData2D.Compute(in this);

    // ---------------------------------------------------------------- ray cast

    /// <summary>
    /// Casts a ray, given in the shape's local frame, against the shape. The ray is <c>origin + t * translation</c> for t in [0, <paramref name="maxFraction"/>].
    /// A ray that starts inside a solid shape hits at fraction 0 with a zero normal. A one-sided chain segment is hit only from its solid side; a segment is hit from both.
    /// Fraction, normal (pointing against the ray, out of the shape) and point are in the shape frame.
    /// </summary>
    public readonly RayCastOutput2D RayCast(Vec2 origin, Vec2 translation, float maxFraction = 1f) => RayCasts2D.RayCast(in this, origin, translation, maxFraction);

    /// <summary>Like <see cref="RayCast(Vec2, Vec2, float)"/> for a shape placed by <paramref name="xf"/>; the ray and the result are in the parent frame.</summary>
    public readonly RayCastOutput2D RayCast(Transform2 xf, Vec2 origin, Vec2 translation, float maxFraction = 1f)
    {
        RayCastOutput2D o = RayCasts2D.RayCast(in this, xf.ApplyInverse(origin), xf.Q.InverseRotate(translation), maxFraction);
        return o.Hit ? new RayCastOutput2D(true, o.Fraction, xf.Q.Rotate(o.Normal), xf.Apply(o.Point)) : o;
    }
}

/// <summary>A chain: a sequence of points whose consecutive pairs are segments, open or a closed loop, one- or two-sided.</summary>
public sealed class Chain2D
{
    private readonly Vec2[] _points;

    /// <summary>The chain's points (a copy).</summary>
    public ReadOnlySpan<Vec2> Points => _points;

    /// <summary>True when the last point connects to the first.</summary>
    public bool IsLoop { get; }

    /// <summary>True when segments collide from both sides.</summary>
    public bool IsTwoSided { get; }

    /// <summary>Number of segments: the point count for a loop, one less for an open chain.</summary>
    public int SegmentCount => IsLoop ? _points.Length : _points.Length - 1;

    /// <summary>
    /// Builds a chain. The solid side of a one-sided chain is to the right of the direction of travel, so a loop wound clockwise is solid
    /// on its inside and a loop wound counter-clockwise is solid on its outside.
    /// </summary>
    /// <param name="points">At least 2 points (3 for a loop), each coordinate finite and within ±2^20, consecutive points at least <see cref="Constants2D.MinShapeExtent"/> apart.</param>
    /// <param name="loop">Connect the last point to the first.</param>
    /// <param name="twoSided">Collide from both sides.</param>
    /// <exception cref="InvalidShapeException">Too few points, a coordinate that is not finite or out of range, or a zero-length segment.</exception>
    public Chain2D(ReadOnlySpan<Vec2> points, bool loop = false, bool twoSided = false)
    {
        int min = loop ? 3 : 2;
        if (points.Length < min)
        {
            throw new InvalidShapeException(ShapeError.TooFewPoints, "a " + (loop ? "loop" : "chain") + " needs at least " + min.ToString(CultureInfo.InvariantCulture) + " points, got " + points.Length.ToString(CultureInfo.InvariantCulture));
        }

        if (points.Length > (1 << 24))
        {
            throw new InvalidShapeException(ShapeError.TooManyPoints, "got " + points.Length.ToString(CultureInfo.InvariantCulture) + " points, the limit is 16777216");
        }

        _points = points.ToArray();
        for (int i = 0; i < _points.Length; i++)
        {
            Shape2D.CheckPoint(_points[i], "points[" + i.ToString(CultureInfo.InvariantCulture) + "]");
        }

        int segs = loop ? _points.Length : _points.Length - 1;
        for (int i = 0; i < segs; i++)
        {
            if (!((_points[(i + 1) % _points.Length] - _points[i]).Length >= Constants2D.MinShapeExtent))
            {
                throw new InvalidShapeException(ShapeError.Degenerate, "points[" + i.ToString(CultureInfo.InvariantCulture) + "] and the next point are closer than " + Constants2D.MinShapeExtent.ToString("R", CultureInfo.InvariantCulture));
            }
        }

        IsLoop = loop;
        IsTwoSided = twoSided;
    }

    /// <summary>
    /// The segment at <paramref name="index"/> as a shape, with its neighbours as ghost vertices. Use the index as the contact's sub-shape id.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The index is not in 0 to <see cref="SegmentCount"/> − 1.</exception>
    public Shape2D GetSegment(int index)
    {
        int n = _points.Length;
        if ((uint)index >= (uint)SegmentCount)
        {
            throw new ArgumentOutOfRangeException(nameof(index), index, "segment index must be in 0.." + (SegmentCount - 1).ToString(CultureInfo.InvariantCulture));
        }

        Vec2 p1 = _points[index];
        Vec2 p2 = _points[(index + 1) % n];
        Vec2? g1 = IsLoop ? _points[(index + n - 1) % n] : index > 0 ? _points[index - 1] : null;
        Vec2? g2 = IsLoop ? _points[(index + 2) % n] : index + 2 < n ? _points[index + 2] : null;
        return Shape2D.CreateChainSegment(g1, p1, p2, g2, IsTwoSided);
    }
}
