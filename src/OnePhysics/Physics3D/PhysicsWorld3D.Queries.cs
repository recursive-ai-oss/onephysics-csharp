using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Broadphase;
using RecursiveAI.OnePhysics.Collision3D;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>The longest distance a ray or cast travels; larger values (including infinity) are clamped to it (2^23 metres).</summary>
    public const float MaxQueryDistance = 8388608f;

    internal void InvalidateQueryPad() => _engine.InvalidatePad();

    // ------------------------------------------------------------------ validation

    private static void CheckPosition(Vector3D p, string name)
    {
        double limit = BoxLimits.MaxCoordinate;
        if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !double.IsFinite(p.Z) || Math.Abs(p.X) > limit || Math.Abs(p.Y) > limit || Math.Abs(p.Z) > limit)
        {
            throw new ArgumentOutOfRangeException(name, p, name + " " + p + " must be finite and within +-2^22 (4194304) of the world origin.");
        }
    }

    private static QuaternionF CheckRotation(QuaternionF q)
    {
        if (q.LengthSquared() == 0f) { return QuaternionF.Identity; }
        return Engine3D.Normalize(q, "rotation");
    }

    /// <summary>Normalizes the direction and returns the cast translation and its length.</summary>
    private static void CheckCast(Vector3F direction, float maxDistance, out float tx, out float ty, out float tz, out float length)
    {
        if (!float.IsFinite(direction.X) || !float.IsFinite(direction.Y) || !float.IsFinite(direction.Z)) { throw new ArgumentOutOfRangeException(nameof(direction), direction, "The direction must be finite."); }
        float len = float.Sqrt(direction.X * direction.X + direction.Y * direction.Y + direction.Z * direction.Z);
        if (!(len > 0f) || !float.IsFinite(len)) { throw new ArgumentOutOfRangeException(nameof(direction), direction, "The direction must not be zero (or so large its length overflows)."); }
        if (float.IsNaN(maxDistance) || !(maxDistance > 0f)) { throw new ArgumentOutOfRangeException(nameof(maxDistance), maxDistance, "maxDistance must be positive (infinity allowed)."); }
        length = maxDistance < MaxQueryDistance ? maxDistance : MaxQueryDistance;
        tx = direction.X / len * length;
        ty = direction.Y / len * length;
        tz = direction.Z / len * length;
    }

    private static void CheckRadius(float radius)
    {
        if (!float.IsFinite(radius) || radius < 0f || radius > 1048576f) { throw new ArgumentOutOfRangeException(nameof(radius), radius, "The radius must be finite, at least 0 and at most 2^20."); }
    }

    private static ConvexShape3D SphereShape(float radius)
    {
        CheckRadius(radius);
        return radius > 0f ? ConvexShape3D.Sphere(radius) : ConvexShape3D.Point;
    }

    private static ConvexShape3D BoxShape(Vector3F size)
    {
        if (!(size.X > 0f) || !(size.Y > 0f) || !(size.Z > 0f) || !float.IsFinite(size.X + size.Y + size.Z)) { throw new ArgumentOutOfRangeException(nameof(size), size, "A box size must be finite and positive in all axes."); }
        return ConvexShape3D.Box(new Vector3F(0.5f * size.X, 0.5f * size.Y, 0.5f * size.Z));
    }

    private static ConvexShape3D CapsuleShape(float halfHeight, float radius)
    {
        if (!float.IsFinite(halfHeight) || halfHeight < 0f) { throw new ArgumentOutOfRangeException(nameof(halfHeight), halfHeight, "halfHeight must be finite and at least 0."); }
        if (!(radius > 0f) || !float.IsFinite(radius)) { throw new ArgumentOutOfRangeException(nameof(radius), radius, "A capsule radius must be finite and positive."); }
        return ConvexShape3D.Capsule(halfHeight, radius);
    }

    private QueryHit3D ToHit(in RawHit h, float length) =>
        new(new Shape(this, _core.HandleOfShape(h.Slot)), new Vector3D(h.Px, h.Py, h.Pz), new Vector3F(h.Nx, h.Ny, h.Nz), h.Fraction, h.Fraction * length + h.Distance, h.Sub);

    // ------------------------------------------------------------------ the three forms, once

    private bool QueryOne(in QueryParams q, float length, in QueryFilter3D filter, QueryMode mode, out QueryHit3D hit)
    {
        var s = _engine.Run(in q, in filter, mode, 1);
        try
        {
            if (s.Count == 0) { hit = default; return false; }
            hit = ToHit(in s.Hits[0], length);
            return true;
        }
        finally { s.Release(); }
    }

    private int QueryMany(in QueryParams q, float length, in QueryFilter3D filter, Span<QueryHit3D> results)
    {
        var s = _engine.Run(in q, in filter, QueryMode.All, results.Length);
        try
        {
            for (int i = 0; i < s.Count; i++) { results[i] = ToHit(in s.Hits[i], length); }
            return s.Count;
        }
        finally { s.Release(); }
    }

    private QueryHit3D[] QueryArray(in QueryParams q, float length, in QueryFilter3D filter)
    {
        var s = _engine.Run(in q, in filter, QueryMode.All, int.MaxValue);
        try
        {
            var r = new QueryHit3D[s.Count];
            for (int i = 0; i < r.Length; i++) { r[i] = ToHit(in s.Hits[i], length); }
            return r;
        }
        finally { s.Release(); }
    }

    private static QueryParams RayParams(Vector3D origin, Vector3F direction, float maxDistance, out float length)
    {
        CheckPosition(origin, nameof(origin));
        CheckCast(direction, maxDistance, out float tx, out float ty, out float tz, out length);
        return new QueryParams { Kind = QueryKind.Ray, X = origin.X, Y = origin.Y, Z = origin.Z, Tx = tx, Ty = ty, Tz = tz };
    }

    private static QueryParams CastParams(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, Vector3F direction, float maxDistance, out float length)
    {
        CheckPosition(position, nameof(position));
        var rot = CheckRotation(rotation);
        CheckCast(direction, maxDistance, out float tx, out float ty, out float tz, out length);
        return new QueryParams { Kind = QueryKind.Cast, X = position.X, Y = position.Y, Z = position.Z, Tx = tx, Ty = ty, Tz = tz, Mover = shape, MoverRot = rot };
    }

    private static QueryParams OverlapParams(in ConvexShape3D shape, Vector3D position, QuaternionF rotation)
    {
        CheckPosition(position, nameof(position));
        return new QueryParams { Kind = QueryKind.Overlap, X = position.X, Y = position.Y, Z = position.Z, Mover = shape, MoverRot = CheckRotation(rotation) };
    }

    private static QueryParams ClosestParams(Vector3D point, float maxDistance)
    {
        CheckPosition(point, nameof(point));
        if (float.IsNaN(maxDistance) || !(maxDistance >= 0f)) { throw new ArgumentOutOfRangeException(nameof(maxDistance), maxDistance, "maxDistance must be at least 0 (infinity allowed)."); }
        return new QueryParams { Kind = QueryKind.Closest, X = point.X, Y = point.Y, Z = point.Z, MaxDistance = maxDistance < MaxQueryDistance ? maxDistance : MaxQueryDistance };
    }

    // ------------------------------------------------------------------ rays

    /// <summary>
    /// Casts a ray and returns the nearest hit (ties on equal distance go to the lower shape slot). A ray that starts inside a convex shape hits it at distance 0 with a zero normal.
    /// Sensors are skipped unless the filter includes them. Allocates nothing in steady state.
    /// </summary>
    /// <param name="origin">Start point, finite and within +-2^22.</param>
    /// <param name="direction">Direction, finite and not zero; it is normalized.</param>
    /// <param name="hit">The hit: <see cref="QueryHit3D.Point"/>, <see cref="QueryHit3D.Normal"/>, <see cref="QueryHit3D.Fraction"/> and <see cref="QueryHit3D.Distance"/>.</param>
    /// <param name="maxDistance">Ray length, positive; infinity (the default) means <see cref="MaxQueryDistance"/>.</param>
    /// <param name="filter">Which shapes count; the default accepts all but sensors.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite, the direction is zero, the length is not positive or the origin is beyond +-2^22.</exception>
    public bool Raycast(Vector3D origin, Vector3F direction, out QueryHit3D hit, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
    {
        var q = RayParams(origin, direction, maxDistance, out float len);
        return QueryOne(in q, len, in filter, QueryMode.Closest, out hit);
    }

    /// <summary>True when the ray hits any shape (stops at the first one found, which one is unspecified).</summary>
    /// <inheritdoc cref="Raycast"/>
    public bool RaycastAny(Vector3D origin, Vector3F direction, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
    {
        var q = RayParams(origin, direction, maxDistance, out float len);
        return QueryOne(in q, len, in filter, QueryMode.Any, out _);
    }

    /// <summary>Every shape the ray hits, nearest first (equal distances by shape slot), written to <paramref name="results"/>; when it is too small the nearest hits are kept. Returns how many were written. Allocates nothing in steady state.</summary>
    /// <inheritdoc cref="Raycast"/>
    public int RaycastAll(Vector3D origin, Vector3F direction, Span<QueryHit3D> results, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
    {
        var q = RayParams(origin, direction, maxDistance, out float len);
        return QueryMany(in q, len, in filter, results);
    }

    /// <summary>Every shape the ray hits, nearest first, in a new array.</summary>
    /// <inheritdoc cref="Raycast"/>
    public QueryHit3D[] RaycastAll(Vector3D origin, Vector3F direction, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
    {
        var q = RayParams(origin, direction, maxDistance, out float len);
        return QueryArray(in q, len, in filter);
    }

    // ------------------------------------------------------------------ shape casts

    /// <summary>
    /// Sweeps a convex shape along a direction and returns the first hit (nearest; ties by shape slot). <paramref name="shape"/> is in its own frame, placed at <paramref name="position"/> rotated by <paramref name="rotation"/>.
    /// A mover that already overlaps a shape hits it at distance 0 with a zero normal. The fraction is accurate to about 2e-4 of the translation (see collision3d.md).
    /// </summary>
    /// <param name="shape">The mover, for example <c>ConvexShape3D.Box(...)</c>.</param>
    /// <param name="position">Where the shape's frame starts, finite and within +-2^22.</param>
    /// <param name="rotation">Orientation; zero means identity, other values are normalized.</param>
    /// <param name="direction">Sweep direction, finite and not zero.</param>
    /// <param name="hit">Hit point and normal are on the target shape (normal facing the mover).</param>
    /// <param name="maxDistance">Sweep length, positive; infinity means <see cref="MaxQueryDistance"/>.</param>
    /// <param name="filter">Which shapes count.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite, the direction is zero, or the length is not positive.</exception>
    public bool ShapeCast(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, Vector3F direction, out QueryHit3D hit, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
    {
        var q = CastParams(in shape, position, rotation, direction, maxDistance, out float len);
        return QueryOne(in q, len, in filter, QueryMode.Closest, out hit);
    }

    /// <summary>True when the sweep hits any shape.</summary>
    /// <inheritdoc cref="ShapeCast"/>
    public bool ShapeCastAny(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, Vector3F direction, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
    {
        var q = CastParams(in shape, position, rotation, direction, maxDistance, out float len);
        return QueryOne(in q, len, in filter, QueryMode.Any, out _);
    }

    /// <summary>Every shape the sweep hits, nearest first, into <paramref name="results"/> (the nearest are kept when it is too small). Returns how many were written.</summary>
    /// <inheritdoc cref="ShapeCast"/>
    public int ShapeCastAll(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, Vector3F direction, Span<QueryHit3D> results, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
    {
        var q = CastParams(in shape, position, rotation, direction, maxDistance, out float len);
        return QueryMany(in q, len, in filter, results);
    }

    /// <summary>Every shape the sweep hits, nearest first, in a new array.</summary>
    /// <inheritdoc cref="ShapeCast"/>
    public QueryHit3D[] ShapeCastAll(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, Vector3F direction, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
    {
        var q = CastParams(in shape, position, rotation, direction, maxDistance, out float len);
        return QueryArray(in q, len, in filter);
    }

    /// <summary>Sweeps a sphere; the nearest hit. See <see cref="ShapeCast"/>.</summary>
    public bool SphereCast(Vector3D center, float radius, Vector3F direction, out QueryHit3D hit, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
        => ShapeCast(SphereShape(radius), center, default, direction, out hit, maxDistance, in filter);

    /// <summary>True when a swept sphere hits anything.</summary>
    public bool SphereCastAny(Vector3D center, float radius, Vector3F direction, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
        => ShapeCastAny(SphereShape(radius), center, default, direction, maxDistance, in filter);

    /// <summary>Every hit of a swept sphere, nearest first, into a caller buffer.</summary>
    public int SphereCastAll(Vector3D center, float radius, Vector3F direction, Span<QueryHit3D> results, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
        => ShapeCastAll(SphereShape(radius), center, default, direction, results, maxDistance, in filter);

    /// <summary>Every hit of a swept sphere, nearest first, in a new array.</summary>
    public QueryHit3D[] SphereCastAll(Vector3D center, float radius, Vector3F direction, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
        => ShapeCastAll(SphereShape(radius), center, default, direction, maxDistance, in filter);

    /// <summary>Sweeps a box of full size <paramref name="size"/> centred at <paramref name="center"/>, rotated by <paramref name="rotation"/>; the nearest hit.</summary>
    public bool BoxCast(Vector3D center, Vector3F size, QuaternionF rotation, Vector3F direction, out QueryHit3D hit, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
        => ShapeCast(BoxShape(size), center, rotation, direction, out hit, maxDistance, in filter);

    /// <summary>True when a swept box hits anything.</summary>
    public bool BoxCastAny(Vector3D center, Vector3F size, QuaternionF rotation, Vector3F direction, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
        => ShapeCastAny(BoxShape(size), center, rotation, direction, maxDistance, in filter);

    /// <summary>Every hit of a swept box, nearest first, into a caller buffer.</summary>
    public int BoxCastAll(Vector3D center, Vector3F size, QuaternionF rotation, Vector3F direction, Span<QueryHit3D> results, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
        => ShapeCastAll(BoxShape(size), center, rotation, direction, results, maxDistance, in filter);

    /// <summary>Every hit of a swept box, nearest first, in a new array.</summary>
    public QueryHit3D[] BoxCastAll(Vector3D center, Vector3F size, QuaternionF rotation, Vector3F direction, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
        => ShapeCastAll(BoxShape(size), center, rotation, direction, maxDistance, in filter);

    /// <summary>Sweeps a capsule whose axis runs along y from -halfHeight to halfHeight in its frame, rotated by <paramref name="rotation"/>; the nearest hit.</summary>
    public bool CapsuleCast(Vector3D center, float halfHeight, float radius, QuaternionF rotation, Vector3F direction, out QueryHit3D hit, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
        => ShapeCast(CapsuleShape(halfHeight, radius), center, rotation, direction, out hit, maxDistance, in filter);

    /// <summary>True when a swept capsule hits anything.</summary>
    public bool CapsuleCastAny(Vector3D center, float halfHeight, float radius, QuaternionF rotation, Vector3F direction, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
        => ShapeCastAny(CapsuleShape(halfHeight, radius), center, rotation, direction, maxDistance, in filter);

    /// <summary>Every hit of a swept capsule, nearest first, into a caller buffer.</summary>
    public int CapsuleCastAll(Vector3D center, float halfHeight, float radius, QuaternionF rotation, Vector3F direction, Span<QueryHit3D> results, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
        => ShapeCastAll(CapsuleShape(halfHeight, radius), center, rotation, direction, results, maxDistance, in filter);

    /// <summary>Every hit of a swept capsule, nearest first, in a new array.</summary>
    public QueryHit3D[] CapsuleCastAll(Vector3D center, float halfHeight, float radius, QuaternionF rotation, Vector3F direction, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
        => ShapeCastAll(CapsuleShape(halfHeight, radius), center, rotation, direction, maxDistance, in filter);

    // ------------------------------------------------------------------ overlaps

    /// <summary>
    /// Finds the shapes a convex shape overlaps (touching counts). <c>OverlapShape</c> returns the one whose surface is nearest the query shape's origin (<see cref="QueryHit3D.Distance"/>, 0 when the origin is inside it; ties by shape slot).
    /// </summary>
    /// <param name="shape">The query shape in its own frame.</param>
    /// <param name="position">Where its frame is, finite and within +-2^22.</param>
    /// <param name="rotation">Orientation; zero means identity.</param>
    /// <param name="hit">The overlapped shape.</param>
    /// <param name="filter">Which shapes count.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or the position is beyond +-2^22.</exception>
    public bool OverlapShape(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, out QueryHit3D hit, in QueryFilter3D filter = default)
    {
        var q = OverlapParams(in shape, position, rotation);
        return QueryOne(in q, 0f, in filter, QueryMode.Closest, out hit);
    }

    /// <summary>True when the shape overlaps anything.</summary>
    /// <inheritdoc cref="OverlapShape"/>
    public bool OverlapShapeAny(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, in QueryFilter3D filter = default)
    {
        var q = OverlapParams(in shape, position, rotation);
        return QueryOne(in q, 0f, in filter, QueryMode.Any, out _);
    }

    /// <summary>Every overlapped shape in ascending shape slot order, into <paramref name="results"/> (the lowest slots are kept when it is too small). Returns how many were written.</summary>
    /// <inheritdoc cref="OverlapShape"/>
    public int OverlapShapeAll(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, Span<QueryHit3D> results, in QueryFilter3D filter = default)
    {
        var q = OverlapParams(in shape, position, rotation);
        return QueryMany(in q, 0f, in filter, results);
    }

    /// <summary>Every overlapped shape in ascending shape slot order, in a new array.</summary>
    /// <inheritdoc cref="OverlapShape"/>
    public QueryHit3D[] OverlapShapeAll(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, in QueryFilter3D filter = default)
    {
        var q = OverlapParams(in shape, position, rotation);
        return QueryArray(in q, 0f, in filter);
    }

    /// <summary>The shape containing a point (nearest surface first when several do).</summary>
    public bool OverlapPoint(Vector3D point, out QueryHit3D hit, in QueryFilter3D filter = default) => OverlapShape(ConvexShape3D.Point, point, default, out hit, in filter);

    /// <summary>True when any shape contains the point.</summary>
    public bool OverlapPointAny(Vector3D point, in QueryFilter3D filter = default) => OverlapShapeAny(ConvexShape3D.Point, point, default, in filter);

    /// <summary>Every shape containing the point, by slot, into a caller buffer.</summary>
    public int OverlapPointAll(Vector3D point, Span<QueryHit3D> results, in QueryFilter3D filter = default) => OverlapShapeAll(ConvexShape3D.Point, point, default, results, in filter);

    /// <summary>The overlapped shape nearest the sphere's centre.</summary>
    public bool OverlapSphere(Vector3D center, float radius, out QueryHit3D hit, in QueryFilter3D filter = default) => OverlapShape(SphereShape(radius), center, default, out hit, in filter);

    /// <summary>True when the sphere overlaps anything.</summary>
    public bool OverlapSphereAny(Vector3D center, float radius, in QueryFilter3D filter = default) => OverlapShapeAny(SphereShape(radius), center, default, in filter);

    /// <summary>Every shape the sphere overlaps, by slot, into a caller buffer.</summary>
    public int OverlapSphereAll(Vector3D center, float radius, Span<QueryHit3D> results, in QueryFilter3D filter = default) => OverlapShapeAll(SphereShape(radius), center, default, results, in filter);

    /// <summary>Every shape the sphere overlaps, by slot, in a new array.</summary>
    public QueryHit3D[] OverlapSphereAll(Vector3D center, float radius, in QueryFilter3D filter = default) => OverlapShapeAll(SphereShape(radius), center, default, in filter);

    /// <summary>The overlapped shape nearest the box's centre.</summary>
    public bool OverlapBox(Vector3D center, Vector3F size, QuaternionF rotation, out QueryHit3D hit, in QueryFilter3D filter = default) => OverlapShape(BoxShape(size), center, rotation, out hit, in filter);

    /// <summary>True when the box overlaps anything.</summary>
    public bool OverlapBoxAny(Vector3D center, Vector3F size, QuaternionF rotation, in QueryFilter3D filter = default) => OverlapShapeAny(BoxShape(size), center, rotation, in filter);

    /// <summary>Every shape the box overlaps, by slot, into a caller buffer.</summary>
    public int OverlapBoxAll(Vector3D center, Vector3F size, QuaternionF rotation, Span<QueryHit3D> results, in QueryFilter3D filter = default) => OverlapShapeAll(BoxShape(size), center, rotation, results, in filter);

    /// <summary>Every shape the box overlaps, by slot, in a new array.</summary>
    public QueryHit3D[] OverlapBoxAll(Vector3D center, Vector3F size, QuaternionF rotation, in QueryFilter3D filter = default) => OverlapShapeAll(BoxShape(size), center, rotation, in filter);

    /// <summary>The overlapped shape nearest the capsule's centre; the capsule's axis runs along y from -halfHeight to halfHeight in its frame.</summary>
    public bool OverlapCapsule(Vector3D center, float halfHeight, float radius, QuaternionF rotation, out QueryHit3D hit, in QueryFilter3D filter = default) => OverlapShape(CapsuleShape(halfHeight, radius), center, rotation, out hit, in filter);

    /// <summary>True when the capsule overlaps anything.</summary>
    public bool OverlapCapsuleAny(Vector3D center, float halfHeight, float radius, QuaternionF rotation, in QueryFilter3D filter = default) => OverlapShapeAny(CapsuleShape(halfHeight, radius), center, rotation, in filter);

    /// <summary>Every shape the capsule overlaps, by slot, into a caller buffer.</summary>
    public int OverlapCapsuleAll(Vector3D center, float halfHeight, float radius, QuaternionF rotation, Span<QueryHit3D> results, in QueryFilter3D filter = default) => OverlapShapeAll(CapsuleShape(halfHeight, radius), center, rotation, results, in filter);

    /// <summary>Every shape the capsule overlaps, by slot, in a new array.</summary>
    public QueryHit3D[] OverlapCapsuleAll(Vector3D center, float halfHeight, float radius, QuaternionF rotation, in QueryFilter3D filter = default) => OverlapShapeAll(CapsuleShape(halfHeight, radius), center, rotation, in filter);

    // ------------------------------------------------------------------ closest point

    /// <summary>
    /// The shape whose surface is nearest a point, within <paramref name="maxDistance"/> (ties by shape slot). <see cref="QueryHit3D.Point"/> is the nearest surface point (the point itself when it is inside a shape),
    /// <see cref="QueryHit3D.Distance"/> the distance (0 inside).
    /// </summary>
    /// <param name="point">The query point, finite and within +-2^22.</param>
    /// <param name="hit">The result.</param>
    /// <param name="maxDistance">Search radius, at least 0; infinity means <see cref="MaxQueryDistance"/>. Default 1000.</param>
    /// <param name="filter">Which shapes count.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite, or the distance is negative or NaN.</exception>
    public bool ClosestPoint(Vector3D point, out QueryHit3D hit, float maxDistance = 1000f, in QueryFilter3D filter = default)
    {
        var q = ClosestParams(point, maxDistance);
        return QueryOne(in q, 0f, in filter, QueryMode.Closest, out hit);
    }

    /// <summary>True when any shape is within <paramref name="maxDistance"/> of the point.</summary>
    /// <inheritdoc cref="ClosestPoint"/>
    public bool ClosestPointAny(Vector3D point, float maxDistance = 1000f, in QueryFilter3D filter = default)
    {
        var q = ClosestParams(point, maxDistance);
        return QueryOne(in q, 0f, in filter, QueryMode.Any, out _);
    }

    /// <summary>Every shape within <paramref name="maxDistance"/>, nearest first (equal distances by slot), into <paramref name="results"/> (the nearest are kept when it is too small).</summary>
    /// <inheritdoc cref="ClosestPoint"/>
    public int ClosestPointAll(Vector3D point, Span<QueryHit3D> results, float maxDistance = 1000f, in QueryFilter3D filter = default)
    {
        var q = ClosestParams(point, maxDistance);
        return QueryMany(in q, 0f, in filter, results);
    }

    /// <summary>Every shape within <paramref name="maxDistance"/>, nearest first, in a new array.</summary>
    /// <inheritdoc cref="ClosestPoint"/>
    public QueryHit3D[] ClosestPointAll(Vector3D point, float maxDistance = 1000f, in QueryFilter3D filter = default)
    {
        var q = ClosestParams(point, maxDistance);
        return QueryArray(in q, 0f, in filter);
    }

    // ------------------------------------------------------------------ used by batches and tests

    internal static void ValidateRay(Vector3D origin, Vector3F direction, float maxDistance)
    {
        CheckPosition(origin, nameof(origin));
        CheckCast(direction, maxDistance, out _, out _, out _, out _);
    }

    internal static void ValidateSphere(Vector3D center, float radius)
    {
        CheckPosition(center, nameof(center));
        CheckRadius(radius);
    }

    internal int OverlapSphereCount(Vector3D center, float radius, in QueryFilter3D filter)
    {
        var q = OverlapParams(SphereShape(radius), center, default);
        var s = _engine.Run(in q, in filter, QueryMode.All, int.MaxValue);
        try { return s.Count; }
        finally { s.Release(); }
    }

    internal bool Brute { get => _engine.BruteQueries; set => _engine.BruteQueries = value; }
}
