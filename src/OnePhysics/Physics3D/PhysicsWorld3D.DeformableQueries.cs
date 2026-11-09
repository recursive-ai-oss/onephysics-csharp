using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    // Deformable queries: the same ray, shape cast, overlap and closest-point queries as for rigid shapes, against the particles, surface triangles and rod segments of soft bodies, cloth and rods.
    // They read the particles as they are now (between steps); they never change state, so they do not move a hash.

    private static float DeformLength(float maxDistance) => maxDistance < MaxQueryDistance ? maxDistance : MaxQueryDistance;

    private static void RayBox(Vector3D o, Vector3F d, float len, out double lox, out double loy, out double loz, out double hix, out double hiy, out double hiz)
    {
        double ex = o.X + (double)d.X * len, ey = o.Y + (double)d.Y * len, ez = o.Z + (double)d.Z * len;
        lox = Math.Min(o.X, ex) - 1e-3; loy = Math.Min(o.Y, ey) - 1e-3; loz = Math.Min(o.Z, ez) - 1e-3;
        hix = Math.Max(o.X, ex) + 1e-3; hiy = Math.Max(o.Y, ey) + 1e-3; hiz = Math.Max(o.Z, ez) + 1e-3;
    }

    private static float Reach(in ConvexShape3D shape)
    {
        if (shape.Kind == ConvexShapeKind3D.Plane) { throw new ArgumentException("A plane has no finite extent and cannot be used for a deformable query.", nameof(shape)); }
        var h = shape.HalfExtents;
        return float.Sqrt(h.X * h.X + h.Y * h.Y + h.Z * h.Z) * 1.001f + 1e-4f;
    }

    internal int RunDeformableRay(Vector3D origin, Vector3F direction, float maxDistance, in DeformableFilter3D filter, DeformableHit3D[] buffer, int capacity, bool any)
    {
        CheckPosition(origin, nameof(origin));
        CheckCast(direction, maxDistance, out float tx, out float ty, out float tz, out float length);
        float inv = 1f / length;
        var unit = new Vector3F(tx * inv, ty * inv, tz * inv);
        var v = new DeformableVisitor3 { Mode = DeformableVisitor3.Ray, Origin = origin, Direction = unit, Length = length, Buffer = buffer, Capacity = capacity, Any = any };
        RayBox(origin, unit, length, out double a, out double b, out double c, out double d, out double e, out double f);
        DeformableScan3D.Scan(_engine, in filter, a, b, c, d, e, f, ref v);
        return v.Count;
    }

    internal int RunDeformableCast(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, Vector3F direction, float maxDistance, in DeformableFilter3D filter, DeformableHit3D[] buffer, int capacity, bool any)
    {
        CheckPosition(position, nameof(position));
        var q = CheckRotation(rotation);
        CheckCast(direction, maxDistance, out float tx, out float ty, out float tz, out float length);
        float inv = 1f / length;
        var unit = new Vector3F(tx * inv, ty * inv, tz * inv);
        float reach = Reach(in shape);
        var v = new DeformableVisitor3 { Mode = DeformableVisitor3.Cast, Origin = position, Rotation = q, Inverse = q.Conjugated(), Direction = unit, Length = length, Shape = shape, Buffer = buffer, Capacity = capacity, Any = any };
        RayBox(position, unit, length, out double a, out double b, out double c, out double d, out double e, out double f);
        DeformableScan3D.Scan(_engine, in filter, a - reach, b - reach, c - reach, d + reach, e + reach, f + reach, ref v);
        return v.Count;
    }

    internal int RunDeformableOverlap(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, in DeformableFilter3D filter, DeformableHit3D[] buffer, int capacity, bool any)
    {
        CheckPosition(position, nameof(position));
        var q = CheckRotation(rotation);
        float reach = Reach(in shape);
        var v = new DeformableVisitor3 { Mode = DeformableVisitor3.Overlap, Origin = position, Rotation = q, Inverse = q.Conjugated(), Shape = shape, Buffer = buffer, Capacity = capacity, Any = any };
        DeformableScan3D.Scan(_engine, in filter, position.X - reach, position.Y - reach, position.Z - reach, position.X + reach, position.Y + reach, position.Z + reach, ref v);
        return v.Count;
    }

    internal int CountDeformableOverlaps(in ConvexShape3D shape, Vector3D position, in DeformableFilter3D filter)
    {
        float reach = Reach(in shape);
        var v = new DeformableVisitor3 { Mode = DeformableVisitor3.CountOverlap, Origin = position, Rotation = QuaternionF.Identity, Inverse = QuaternionF.Identity, Shape = shape };
        DeformableScan3D.Scan(_engine, in filter, position.X - reach, position.Y - reach, position.Z - reach, position.X + reach, position.Y + reach, position.Z + reach, ref v);
        return v.Total;
    }

    internal bool RunDeformableClosest(Vector3D point, float maxDistance, in DeformableFilter3D filter, DeformableHit3D[] buffer, bool any)
    {
        CheckPosition(point, nameof(point));
        if (float.IsNaN(maxDistance) || !(maxDistance >= 0f)) { throw new ArgumentOutOfRangeException(nameof(maxDistance), maxDistance, "maxDistance must be at least 0 (infinity allowed)."); }
        float length = DeformLength(maxDistance);
        var v = new DeformableVisitor3 { Mode = DeformableVisitor3.Closest, Origin = point, Rotation = QuaternionF.Identity, Inverse = QuaternionF.Identity, Length = length, Shape = ConvexShape3D.Point, Buffer = buffer, Capacity = 1, Any = any };
        double r = length + 1e-3;
        DeformableScan3D.Scan(_engine, in filter, point.X - r, point.Y - r, point.Z - r, point.X + r, point.Y + r, point.Z + r, ref v);
        return v.Count > 0;
    }

    // ------------------------------------------------------------------ rays

    /// <summary>
    /// Casts a ray at the soft bodies, cloth and rods and returns the nearest hit (ties on equal distance go to the lower kind, slot, element and index). Soft bodies and cloth are hit on their surface triangles (the
    /// bare two-sided triangle) or, when they have none, on their particles (spheres of the particle radius); rods on their segments (capsules of the rod's radius). A ray that starts inside an element hits it at distance 0.
    /// Allocates nothing in steady state.
    /// </summary>
    /// <param name="origin">Start point, finite and within +-2^22.</param>
    /// <param name="direction">Direction, finite and not zero; it is normalized.</param>
    /// <param name="hit">The hit: the deformable, the element, the barycentric weights, the point and the normal.</param>
    /// <param name="maxDistance">Ray length, positive; infinity (the default) means <see cref="MaxQueryDistance"/>.</param>
    /// <param name="filter">Which deformables count; the default accepts all.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite, the direction is zero, the length is not positive or the origin is beyond +-2^22.</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public bool RaycastDeformables(Vector3D origin, Vector3F direction, out DeformableHit3D hit, float maxDistance = float.PositiveInfinity, in DeformableFilter3D filter = default)
    {
        var b = Engine3D.DeformScratch(1);
        int n = RunDeformableRay(origin, direction, maxDistance, in filter, b, 1, false);
        hit = n > 0 ? b[0] : default;
        return n > 0;
    }

    /// <summary>True when the ray hits any deformable (stops at the first element found, in slot order).</summary>
    /// <inheritdoc cref="RaycastDeformables"/>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public bool RaycastDeformablesAny(Vector3D origin, Vector3F direction, float maxDistance = float.PositiveInfinity, in DeformableFilter3D filter = default) =>
        RunDeformableRay(origin, direction, maxDistance, in filter, Engine3D.DeformScratch(1), 1, true) > 0;

    /// <summary>All hits of a ray, nearest first, into a caller buffer: the nearest <c>results.Length</c> elements. Returns the count stored.</summary>
    /// <inheritdoc cref="RaycastDeformables"/>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public int RaycastDeformablesAll(Vector3D origin, Vector3F direction, Span<DeformableHit3D> results, float maxDistance = float.PositiveInfinity, in DeformableFilter3D filter = default)
    {
        if (results.Length == 0) { return 0; }
        var b = Engine3D.DeformScratch(results.Length);
        int n = RunDeformableRay(origin, direction, maxDistance, in filter, b, results.Length, false);
        b.AsSpan(0, n).CopyTo(results);
        return n;
    }

    // ------------------------------------------------------------------ shape casts

    /// <summary>
    /// Sweeps a convex shape along a direction and returns the first element it touches. A shape that starts touching or overlapping an element hits it at fraction 0. <see cref="DeformableHit3D.Point"/> is on the element.
    /// </summary>
    /// <param name="shape">The shape (not a plane).</param>
    /// <param name="position">Where the shape's origin starts.</param>
    /// <param name="rotation">The shape's orientation; zero means identity, otherwise normalized.</param>
    /// <param name="direction">Direction, finite and not zero; normalized.</param>
    /// <param name="hit">The hit; its <see cref="DeformableHit3D.Fraction"/> is the fraction of <paramref name="maxDistance"/> travelled.</param>
    /// <param name="maxDistance">Sweep length, positive; infinity means <see cref="MaxQueryDistance"/>.</param>
    /// <param name="filter">Which deformables count.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or out of range.</exception>
    /// <exception cref="ArgumentException">The shape is a plane.</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public bool ShapeCastDeformables(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, Vector3F direction, out DeformableHit3D hit, float maxDistance = float.PositiveInfinity, in DeformableFilter3D filter = default)
    {
        var b = Engine3D.DeformScratch(1);
        int n = RunDeformableCast(in shape, position, rotation, direction, maxDistance, in filter, b, 1, false);
        hit = n > 0 ? b[0] : default;
        return n > 0;
    }

    /// <summary>True when the swept shape touches any deformable.</summary>
    /// <inheritdoc cref="ShapeCastDeformables"/>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public bool ShapeCastDeformablesAny(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, Vector3F direction, float maxDistance = float.PositiveInfinity, in DeformableFilter3D filter = default) =>
        RunDeformableCast(in shape, position, rotation, direction, maxDistance, in filter, Engine3D.DeformScratch(1), 1, true) > 0;

    /// <summary>Every element the swept shape touches, nearest first, into a caller buffer. Returns the count stored.</summary>
    /// <inheritdoc cref="ShapeCastDeformables"/>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public int ShapeCastDeformablesAll(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, Vector3F direction, Span<DeformableHit3D> results, float maxDistance = float.PositiveInfinity, in DeformableFilter3D filter = default)
    {
        if (results.Length == 0) { return 0; }
        var b = Engine3D.DeformScratch(results.Length);
        int n = RunDeformableCast(in shape, position, rotation, direction, maxDistance, in filter, b, results.Length, false);
        b.AsSpan(0, n).CopyTo(results);
        return n;
    }

    /// <summary>Casts a sphere; see <see cref="ShapeCastDeformables"/>.</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public bool SphereCastDeformables(Vector3D center, float radius, Vector3F direction, out DeformableHit3D hit, float maxDistance = float.PositiveInfinity, in DeformableFilter3D filter = default)
    {
        CheckRadius(radius);
        return ShapeCastDeformables(radius > 0f ? ConvexShape3D.Sphere(radius) : ConvexShape3D.Point, center, default, direction, out hit, maxDistance, in filter);
    }

    // ------------------------------------------------------------------ overlaps

    /// <summary>The first element (by kind, slot, element and index) a shape overlaps. An element overlaps when it touches or crosses the shape.</summary>
    /// <param name="shape">The shape (not a plane).</param>
    /// <param name="position">Where the shape's origin is.</param>
    /// <param name="rotation">The shape's orientation; zero means identity.</param>
    /// <param name="hit">The element and a point of contact.</param>
    /// <param name="filter">Which deformables count.</param>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public bool OverlapDeformables(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, out DeformableHit3D hit, in DeformableFilter3D filter = default)
    {
        var b = Engine3D.DeformScratch(1);
        int n = RunDeformableOverlap(in shape, position, rotation, in filter, b, 1, false);
        hit = n > 0 ? b[0] : default;
        return n > 0;
    }

    /// <summary>True when the shape overlaps any deformable.</summary>
    /// <inheritdoc cref="OverlapDeformables"/>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public bool OverlapDeformablesAny(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, in DeformableFilter3D filter = default) =>
        RunDeformableOverlap(in shape, position, rotation, in filter, Engine3D.DeformScratch(1), 1, true) > 0;

    /// <summary>Every element the shape overlaps, ordered by kind, slot, element and index, into a caller buffer (the first <c>results.Length</c>). Returns the count stored.</summary>
    /// <inheritdoc cref="OverlapDeformables"/>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public int OverlapDeformablesAll(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, Span<DeformableHit3D> results, in DeformableFilter3D filter = default)
    {
        if (results.Length == 0) { return 0; }
        var b = Engine3D.DeformScratch(results.Length);
        int n = RunDeformableOverlap(in shape, position, rotation, in filter, b, results.Length, false);
        b.AsSpan(0, n).CopyTo(results);
        return n;
    }

    /// <summary>Every element a sphere overlaps; see <see cref="OverlapDeformablesAll"/>.</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public int OverlapSphereDeformablesAll(Vector3D center, float radius, Span<DeformableHit3D> results, in DeformableFilter3D filter = default)
    {
        CheckRadius(radius);
        return OverlapDeformablesAll(radius > 0f ? ConvexShape3D.Sphere(radius) : ConvexShape3D.Point, center, default, results, in filter);
    }

    // ------------------------------------------------------------------ closest point

    /// <summary>The element nearest to a point within a distance: <see cref="DeformableHit3D.Distance"/> is the gap (0 inside an element) and <see cref="DeformableHit3D.Point"/> the nearest point on it.</summary>
    /// <param name="point">The point.</param>
    /// <param name="hit">The nearest element.</param>
    /// <param name="maxDistance">Search distance, at least 0 (infinity allowed); default 1000.</param>
    /// <param name="filter">Which deformables count.</param>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public bool ClosestDeformable(Vector3D point, out DeformableHit3D hit, float maxDistance = 1000f, in DeformableFilter3D filter = default)
    {
        var b = Engine3D.DeformScratch(1);
        bool found = RunDeformableClosest(point, maxDistance, in filter, b, false);
        hit = found ? b[0] : default;
        return found;
    }

    /// <summary>True when any deformable lies within the distance of the point.</summary>
    /// <inheritdoc cref="ClosestDeformable"/>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public bool ClosestDeformableAny(Vector3D point, float maxDistance = 1000f, in DeformableFilter3D filter = default) =>
        RunDeformableClosest(point, maxDistance, in filter, Engine3D.DeformScratch(1), true);
}
