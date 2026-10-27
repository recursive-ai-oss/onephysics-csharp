using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Broadphase;
using RecursiveAI.OnePhysics.Collision2D;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>The longest distance a ray or cast travels; larger values (including infinity) are clamped to it (2^23 metres).</summary>
    public const float MaxQueryDistance = 8388608f;

    internal void InvalidateQueryPad() => _engine.InvalidatePad();

    // ------------------------------------------------------------------ validation

    private static void CheckPosition(Vector2D p, string name)
    {
        double limit = BoxLimits.MaxCoordinate;
        if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || Math.Abs(p.X) > limit || Math.Abs(p.Y) > limit)
        {
            throw new ArgumentOutOfRangeException(name, p, name + " " + p + " must be finite and within +-2^22 (4194304) of the world origin.");
        }
    }

    private static void CheckAngle(float angle)
    {
        if (!float.IsFinite(angle)) { throw new ArgumentOutOfRangeException(nameof(angle), angle, "Angle must be finite."); }
    }

    /// <summary>Normalizes the direction and returns the cast translation and its length.</summary>
    private static void CheckCast(Vector2F direction, float maxDistance, out float tx, out float ty, out float length)
    {
        if (!float.IsFinite(direction.X) || !float.IsFinite(direction.Y)) { throw new ArgumentOutOfRangeException(nameof(direction), direction, "The direction must be finite."); }
        float len = float.Sqrt(direction.X * direction.X + direction.Y * direction.Y);
        if (!(len > 0f) || !float.IsFinite(len)) { throw new ArgumentOutOfRangeException(nameof(direction), direction, "The direction must not be zero (or so large its length overflows)."); }
        if (float.IsNaN(maxDistance) || !(maxDistance > 0f)) { throw new ArgumentOutOfRangeException(nameof(maxDistance), maxDistance, "maxDistance must be positive (infinity allowed)."); }
        length = maxDistance < MaxQueryDistance ? maxDistance : MaxQueryDistance;
        tx = direction.X / len * length;
        ty = direction.Y / len * length;
    }

    private static void CheckRadius(float radius)
    {
        if (!float.IsFinite(radius) || radius < 0f || radius > Constants2D.MaxShapeCoordinate) { throw new ArgumentOutOfRangeException(nameof(radius), radius, "The radius must be finite, at least 0 and at most 2^20."); }
    }

    private static Shape2D CircleShape(float radius)
    {
        CheckRadius(radius);
        return radius > 0f ? Shape2D.CreateCircle(default, radius) : Engine2D.s_point;
    }

    private static void CheckShape(in Shape2D shape)
    {
        if (shape.Count == 0) { throw new ArgumentException("The shape is empty (the default Shape2D); create one with Shape2D.Create*.", nameof(shape)); }
    }

    private static Rot2 RotationOf(float angle)
    {
        CheckAngle(angle);
        var (s, c) = Math64.SinCos(angle);
        return new Rot2((float)c, (float)s);
    }

    private QueryHit2D ToHit(in RawHit h, float length) => new(new Shape(this, _core.HandleOfShape(h.Slot)), new Vector2D(h.Px, h.Py), new Vector2F(h.Nx, h.Ny), h.Fraction, h.Fraction * length + h.Distance);

    // ------------------------------------------------------------------ the three forms, once

    private bool QueryOne(in QueryParams q, float length, in QueryFilter2D filter, QueryMode mode, out QueryHit2D hit)
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

    private int QueryMany(in QueryParams q, float length, in QueryFilter2D filter, Span<QueryHit2D> results)
    {
        var s = _engine.Run(in q, in filter, QueryMode.All, results.Length);
        try
        {
            for (int i = 0; i < s.Count; i++) { results[i] = ToHit(in s.Hits[i], length); }
            return s.Count;
        }
        finally { s.Release(); }
    }

    private QueryHit2D[] QueryArray(in QueryParams q, float length, in QueryFilter2D filter)
    {
        var s = _engine.Run(in q, in filter, QueryMode.All, int.MaxValue);
        try
        {
            var r = new QueryHit2D[s.Count];
            for (int i = 0; i < r.Length; i++) { r[i] = ToHit(in s.Hits[i], length); }
            return r;
        }
        finally { s.Release(); }
    }

    private static QueryParams RayParams(Vector2D origin, Vector2F direction, float maxDistance, out float length)
    {
        CheckPosition(origin, nameof(origin));
        CheckCast(direction, maxDistance, out float tx, out float ty, out length);
        return new QueryParams { Kind = QueryKind.Ray, X = origin.X, Y = origin.Y, Tx = tx, Ty = ty };
    }

    private static QueryParams CastParams(in Shape2D shape, Vector2D position, float angle, Vector2F direction, float maxDistance, out float length)
    {
        CheckShape(in shape);
        CheckPosition(position, nameof(position));
        var rot = RotationOf(angle);
        CheckCast(direction, maxDistance, out float tx, out float ty, out length);
        return new QueryParams { Kind = QueryKind.Cast, X = position.X, Y = position.Y, Tx = tx, Ty = ty, Mover = shape, MoverRot = rot };
    }

    private static QueryParams OverlapParams(in Shape2D shape, Vector2D position, float angle)
    {
        CheckShape(in shape);
        CheckPosition(position, nameof(position));
        return new QueryParams { Kind = QueryKind.Overlap, X = position.X, Y = position.Y, Mover = shape, MoverRot = RotationOf(angle) };
    }

    private static QueryParams ClosestParams(Vector2D point, float maxDistance)
    {
        CheckPosition(point, nameof(point));
        if (float.IsNaN(maxDistance) || !(maxDistance >= 0f)) { throw new ArgumentOutOfRangeException(nameof(maxDistance), maxDistance, "maxDistance must be at least 0 (infinity allowed)."); }
        return new QueryParams { Kind = QueryKind.Closest, X = point.X, Y = point.Y, MaxDistance = maxDistance < MaxQueryDistance ? maxDistance : MaxQueryDistance };
    }

    // ------------------------------------------------------------------ rays

    /// <summary>
    /// Casts a ray and returns the nearest hit (ties on equal distance go to the lower shape slot). A ray that starts inside a convex shape hits it at distance 0 with a zero normal.
    /// Sensors are skipped unless the filter includes them. Allocates nothing.
    /// </summary>
    /// <param name="origin">Start point, finite and within +-2^22.</param>
    /// <param name="direction">Direction, finite and not zero; it is normalized.</param>
    /// <param name="hit">The hit: <see cref="QueryHit2D.Point"/>, <see cref="QueryHit2D.Normal"/>, <see cref="QueryHit2D.Fraction"/> and <see cref="QueryHit2D.Distance"/>.</param>
    /// <param name="maxDistance">Ray length, positive; infinity (the default) means <see cref="MaxQueryDistance"/>.</param>
    /// <param name="filter">Which shapes count; the default accepts all but sensors.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite, the direction is zero, the length is not positive or the origin is beyond +-2^22.</exception>
    public bool Raycast(Vector2D origin, Vector2F direction, out QueryHit2D hit, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
    {
        var q = RayParams(origin, direction, maxDistance, out float len);
        return QueryOne(in q, len, in filter, QueryMode.Closest, out hit);
    }

    /// <summary>True when the ray hits any shape (stops at the first one found, which one is unspecified).</summary>
    /// <inheritdoc cref="Raycast"/>
    public bool RaycastAny(Vector2D origin, Vector2F direction, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
    {
        var q = RayParams(origin, direction, maxDistance, out float len);
        return QueryOne(in q, len, in filter, QueryMode.Any, out _);
    }

    /// <summary>Every shape the ray hits, nearest first (equal distances by shape slot), written to <paramref name="results"/>; when it is too small the nearest hits are kept. Returns how many were written. Allocates nothing.</summary>
    /// <inheritdoc cref="Raycast"/>
    public int RaycastAll(Vector2D origin, Vector2F direction, Span<QueryHit2D> results, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
    {
        var q = RayParams(origin, direction, maxDistance, out float len);
        return QueryMany(in q, len, in filter, results);
    }

    /// <summary>Every shape the ray hits, nearest first, in a new array.</summary>
    /// <inheritdoc cref="Raycast"/>
    public QueryHit2D[] RaycastAll(Vector2D origin, Vector2F direction, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
    {
        var q = RayParams(origin, direction, maxDistance, out float len);
        return QueryArray(in q, len, in filter);
    }

    // ------------------------------------------------------------------ shape casts

    /// <summary>
    /// Sweeps a shape along a direction and returns the first hit (nearest; ties by shape slot). <paramref name="shape"/> is in its own frame, placed at <paramref name="position"/> rotated by <paramref name="angle"/>.
    /// A mover that already overlaps a shape hits it at distance 0. The hit fraction is within 1e-4 of the translation length short of the exact touch.
    /// </summary>
    /// <param name="shape">The mover, for example <c>Shape2D.CreateBox(...)</c>.</param>
    /// <param name="position">Where the shape's frame starts, finite and within +-2^22.</param>
    /// <param name="angle">Rotation in radians, finite.</param>
    /// <param name="direction">Sweep direction, finite and not zero.</param>
    /// <param name="hit">Hit point and normal are on the target shape (normal facing the mover).</param>
    /// <param name="maxDistance">Sweep length, positive; infinity means <see cref="MaxQueryDistance"/>.</param>
    /// <param name="filter">Which shapes count.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite, the direction is zero, or the length is not positive.</exception>
    /// <exception cref="ArgumentException">The shape is the default (empty) value.</exception>
    public bool ShapeCast(in Shape2D shape, Vector2D position, float angle, Vector2F direction, out QueryHit2D hit, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
    {
        var q = CastParams(in shape, position, angle, direction, maxDistance, out float len);
        return QueryOne(in q, len, in filter, QueryMode.Closest, out hit);
    }

    /// <summary>True when the sweep hits any shape.</summary>
    /// <inheritdoc cref="ShapeCast"/>
    public bool ShapeCastAny(in Shape2D shape, Vector2D position, float angle, Vector2F direction, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
    {
        var q = CastParams(in shape, position, angle, direction, maxDistance, out float len);
        return QueryOne(in q, len, in filter, QueryMode.Any, out _);
    }

    /// <summary>Every shape the sweep hits, nearest first, into <paramref name="results"/> (the nearest are kept when it is too small). Returns how many were written.</summary>
    /// <inheritdoc cref="ShapeCast"/>
    public int ShapeCastAll(in Shape2D shape, Vector2D position, float angle, Vector2F direction, Span<QueryHit2D> results, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
    {
        var q = CastParams(in shape, position, angle, direction, maxDistance, out float len);
        return QueryMany(in q, len, in filter, results);
    }

    /// <summary>Every shape the sweep hits, nearest first, in a new array.</summary>
    /// <inheritdoc cref="ShapeCast"/>
    public QueryHit2D[] ShapeCastAll(in Shape2D shape, Vector2D position, float angle, Vector2F direction, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
    {
        var q = CastParams(in shape, position, angle, direction, maxDistance, out float len);
        return QueryArray(in q, len, in filter);
    }

    /// <summary>Sweeps a circle; the nearest hit. See <see cref="ShapeCast"/>.</summary>
    public bool CircleCast(Vector2D center, float radius, Vector2F direction, out QueryHit2D hit, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
        => ShapeCast(CircleShape(radius), center, 0f, direction, out hit, maxDistance, in filter);

    /// <summary>True when a swept circle hits anything.</summary>
    public bool CircleCastAny(Vector2D center, float radius, Vector2F direction, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
        => ShapeCastAny(CircleShape(radius), center, 0f, direction, maxDistance, in filter);

    /// <summary>Every hit of a swept circle, nearest first, into a caller buffer.</summary>
    public int CircleCastAll(Vector2D center, float radius, Vector2F direction, Span<QueryHit2D> results, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
        => ShapeCastAll(CircleShape(radius), center, 0f, direction, results, maxDistance, in filter);

    /// <summary>Every hit of a swept circle, nearest first, in a new array.</summary>
    public QueryHit2D[] CircleCastAll(Vector2D center, float radius, Vector2F direction, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
        => ShapeCastAll(CircleShape(radius), center, 0f, direction, maxDistance, in filter);

    /// <summary>Sweeps a box of full size <paramref name="size"/> centred at <paramref name="center"/>, rotated by <paramref name="angle"/>; the nearest hit.</summary>
    public bool BoxCast(Vector2D center, Vector2F size, float angle, Vector2F direction, out QueryHit2D hit, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
        => ShapeCast(Box(size), center, angle, direction, out hit, maxDistance, in filter);

    /// <summary>True when a swept box hits anything.</summary>
    public bool BoxCastAny(Vector2D center, Vector2F size, float angle, Vector2F direction, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
        => ShapeCastAny(Box(size), center, angle, direction, maxDistance, in filter);

    /// <summary>Every hit of a swept box, nearest first, into a caller buffer.</summary>
    public int BoxCastAll(Vector2D center, Vector2F size, float angle, Vector2F direction, Span<QueryHit2D> results, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
        => ShapeCastAll(Box(size), center, angle, direction, results, maxDistance, in filter);

    /// <summary>Every hit of a swept box, nearest first, in a new array.</summary>
    public QueryHit2D[] BoxCastAll(Vector2D center, Vector2F size, float angle, Vector2F direction, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
        => ShapeCastAll(Box(size), center, angle, direction, maxDistance, in filter);

    /// <summary>Sweeps a capsule whose axis runs from (-halfLength, 0) to (halfLength, 0) in its frame, rotated by <paramref name="angle"/>; the nearest hit.</summary>
    public bool CapsuleCast(Vector2D center, float halfLength, float radius, float angle, Vector2F direction, out QueryHit2D hit, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
        => ShapeCast(Capsule(halfLength, radius), center, angle, direction, out hit, maxDistance, in filter);

    /// <summary>True when a swept capsule hits anything.</summary>
    public bool CapsuleCastAny(Vector2D center, float halfLength, float radius, float angle, Vector2F direction, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
        => ShapeCastAny(Capsule(halfLength, radius), center, angle, direction, maxDistance, in filter);

    /// <summary>Every hit of a swept capsule, nearest first, into a caller buffer.</summary>
    public int CapsuleCastAll(Vector2D center, float halfLength, float radius, float angle, Vector2F direction, Span<QueryHit2D> results, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
        => ShapeCastAll(Capsule(halfLength, radius), center, angle, direction, results, maxDistance, in filter);

    /// <summary>Every hit of a swept capsule, nearest first, in a new array.</summary>
    public QueryHit2D[] CapsuleCastAll(Vector2D center, float halfLength, float radius, float angle, Vector2F direction, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
        => ShapeCastAll(Capsule(halfLength, radius), center, angle, direction, maxDistance, in filter);

    private static Shape2D Box(Vector2F size)
    {
        if (!(size.X > 0f) || !(size.Y > 0f) || !float.IsFinite(size.X) || !float.IsFinite(size.Y)) { throw new ArgumentOutOfRangeException(nameof(size), size, "A box size must be finite and positive in both axes."); }
        return Shape2D.CreateBox(0.5f * size.X, 0.5f * size.Y);
    }

    private static Shape2D Capsule(float halfLength, float radius)
    {
        if (!float.IsFinite(halfLength) || halfLength < 0f) { throw new ArgumentOutOfRangeException(nameof(halfLength), halfLength, "halfLength must be finite and at least 0."); }
        if (!(radius > 0f) || !float.IsFinite(radius)) { throw new ArgumentOutOfRangeException(nameof(radius), radius, "A capsule radius must be finite and positive."); }
        return Shape2D.CreateCapsule(new Vec2(-halfLength, 0f), new Vec2(halfLength, 0f), radius);
    }

    // ------------------------------------------------------------------ overlaps

    /// <summary>
    /// Finds the shapes a shape overlaps (touching counts). <c>OverlapShape</c> returns the one whose surface is nearest the query shape's centre (<see cref="QueryHit2D.Distance"/>, 0 when the centre is inside it; ties by shape slot).
    /// </summary>
    /// <param name="shape">The query shape in its own frame.</param>
    /// <param name="position">Where its frame is, finite and within +-2^22.</param>
    /// <param name="angle">Rotation in radians.</param>
    /// <param name="hit">The overlapped shape.</param>
    /// <param name="filter">Which shapes count.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or the position is beyond +-2^22.</exception>
    /// <exception cref="ArgumentException">The shape is the default (empty) value.</exception>
    public bool OverlapShape(in Shape2D shape, Vector2D position, float angle, out QueryHit2D hit, in QueryFilter2D filter = default)
    {
        var q = OverlapParams(in shape, position, angle);
        return QueryOne(in q, 0f, in filter, QueryMode.Closest, out hit);
    }

    /// <summary>True when the shape overlaps anything.</summary>
    /// <inheritdoc cref="OverlapShape"/>
    public bool OverlapShapeAny(in Shape2D shape, Vector2D position, float angle, in QueryFilter2D filter = default)
    {
        var q = OverlapParams(in shape, position, angle);
        return QueryOne(in q, 0f, in filter, QueryMode.Any, out _);
    }

    /// <summary>Every overlapped shape in ascending shape slot order, into <paramref name="results"/> (the lowest slots are kept when it is too small). Returns how many were written.</summary>
    /// <inheritdoc cref="OverlapShape"/>
    public int OverlapShapeAll(in Shape2D shape, Vector2D position, float angle, Span<QueryHit2D> results, in QueryFilter2D filter = default)
    {
        var q = OverlapParams(in shape, position, angle);
        return QueryMany(in q, 0f, in filter, results);
    }

    /// <summary>Every overlapped shape in ascending shape slot order, in a new array.</summary>
    /// <inheritdoc cref="OverlapShape"/>
    public QueryHit2D[] OverlapShapeAll(in Shape2D shape, Vector2D position, float angle, in QueryFilter2D filter = default)
    {
        var q = OverlapParams(in shape, position, angle);
        return QueryArray(in q, 0f, in filter);
    }

    /// <summary>The shape containing a point (nearest surface first when several do).</summary>
    public bool OverlapPoint(Vector2D point, out QueryHit2D hit, in QueryFilter2D filter = default) => OverlapShape(Engine2D.s_point, point, 0f, out hit, in filter);

    /// <summary>True when any shape contains the point.</summary>
    public bool OverlapPointAny(Vector2D point, in QueryFilter2D filter = default) => OverlapShapeAny(Engine2D.s_point, point, 0f, in filter);

    /// <summary>Every shape containing the point, by slot, into a caller buffer.</summary>
    public int OverlapPointAll(Vector2D point, Span<QueryHit2D> results, in QueryFilter2D filter = default) => OverlapShapeAll(Engine2D.s_point, point, 0f, results, in filter);

    /// <summary>Every shape containing the point, by slot, in a new array.</summary>
    public QueryHit2D[] OverlapPointAll(Vector2D point, in QueryFilter2D filter = default) => OverlapShapeAll(Engine2D.s_point, point, 0f, in filter);

    /// <summary>The overlapped shape nearest the circle's centre.</summary>
    public bool OverlapCircle(Vector2D center, float radius, out QueryHit2D hit, in QueryFilter2D filter = default) => OverlapShape(CircleShape(radius), center, 0f, out hit, in filter);

    /// <summary>True when the circle overlaps anything.</summary>
    public bool OverlapCircleAny(Vector2D center, float radius, in QueryFilter2D filter = default) => OverlapShapeAny(CircleShape(radius), center, 0f, in filter);

    /// <summary>Every shape the circle overlaps, by slot, into a caller buffer.</summary>
    public int OverlapCircleAll(Vector2D center, float radius, Span<QueryHit2D> results, in QueryFilter2D filter = default) => OverlapShapeAll(CircleShape(radius), center, 0f, results, in filter);

    /// <summary>Every shape the circle overlaps, by slot, in a new array.</summary>
    public QueryHit2D[] OverlapCircleAll(Vector2D center, float radius, in QueryFilter2D filter = default) => OverlapShapeAll(CircleShape(radius), center, 0f, in filter);

    /// <summary>The overlapped shape nearest the box's centre.</summary>
    public bool OverlapBox(Vector2D center, Vector2F size, float angle, out QueryHit2D hit, in QueryFilter2D filter = default) => OverlapShape(Box(size), center, angle, out hit, in filter);

    /// <summary>True when the box overlaps anything.</summary>
    public bool OverlapBoxAny(Vector2D center, Vector2F size, float angle, in QueryFilter2D filter = default) => OverlapShapeAny(Box(size), center, angle, in filter);

    /// <summary>Every shape the box overlaps, by slot, into a caller buffer.</summary>
    public int OverlapBoxAll(Vector2D center, Vector2F size, float angle, Span<QueryHit2D> results, in QueryFilter2D filter = default) => OverlapShapeAll(Box(size), center, angle, results, in filter);

    /// <summary>Every shape the box overlaps, by slot, in a new array.</summary>
    public QueryHit2D[] OverlapBoxAll(Vector2D center, Vector2F size, float angle, in QueryFilter2D filter = default) => OverlapShapeAll(Box(size), center, angle, in filter);

    // ------------------------------------------------------------------ closest point

    /// <summary>
    /// The shape whose surface is nearest a point, within <paramref name="maxDistance"/> (ties by shape slot). <see cref="QueryHit2D.Point"/> is the nearest surface point (the point itself when it is inside a shape),
    /// <see cref="QueryHit2D.Distance"/> the distance (0 inside).
    /// </summary>
    /// <param name="point">The query point, finite and within +-2^22.</param>
    /// <param name="hit">The result.</param>
    /// <param name="maxDistance">Search radius, at least 0; infinity means <see cref="MaxQueryDistance"/>. Default 1000.</param>
    /// <param name="filter">Which shapes count.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite, or the distance is negative or NaN.</exception>
    public bool ClosestPoint(Vector2D point, out QueryHit2D hit, float maxDistance = 1000f, in QueryFilter2D filter = default)
    {
        var q = ClosestParams(point, maxDistance);
        return QueryOne(in q, 0f, in filter, QueryMode.Closest, out hit);
    }

    /// <summary>True when any shape is within <paramref name="maxDistance"/> of the point.</summary>
    /// <inheritdoc cref="ClosestPoint"/>
    public bool ClosestPointAny(Vector2D point, float maxDistance = 1000f, in QueryFilter2D filter = default)
    {
        var q = ClosestParams(point, maxDistance);
        return QueryOne(in q, 0f, in filter, QueryMode.Any, out _);
    }

    /// <summary>Every shape within <paramref name="maxDistance"/>, nearest first (equal distances by slot), into <paramref name="results"/> (the nearest are kept when it is too small).</summary>
    /// <inheritdoc cref="ClosestPoint"/>
    public int ClosestPointAll(Vector2D point, Span<QueryHit2D> results, float maxDistance = 1000f, in QueryFilter2D filter = default)
    {
        var q = ClosestParams(point, maxDistance);
        return QueryMany(in q, 0f, in filter, results);
    }

    /// <summary>Every shape within <paramref name="maxDistance"/>, nearest first, in a new array.</summary>
    /// <inheritdoc cref="ClosestPoint"/>
    public QueryHit2D[] ClosestPointAll(Vector2D point, float maxDistance = 1000f, in QueryFilter2D filter = default)
    {
        var q = ClosestParams(point, maxDistance);
        return QueryArray(in q, 0f, in filter);
    }

    internal QueryHit2D ToPublic(in RawHit h, float length) => ToHit(in h, length);

    internal Engine2D QueryEngine => _engine;

    internal static QueryParams OverlapParamsFor(in Shape2D shape, Vector2D position) => OverlapParams(in shape, position, 0f);

    internal static Shape2D CircleShapeFor(float radius) => CircleShape(radius);

    internal static void ValidateRay(Vector2D origin, Vector2F direction, float maxDistance)
    {
        CheckPosition(origin, nameof(origin));
        CheckCast(direction, maxDistance, out _, out _, out _);
    }

    internal static void ValidateCircle(Vector2D center, float radius)
    {
        CheckPosition(center, nameof(center));
        CheckRadius(radius);
    }

    internal int OverlapCircleCount(Vector2D center, float radius, in QueryFilter2D filter)
    {
        var q = OverlapParams(CircleShape(radius), center, 0f);
        var s = _engine.Run(in q, in filter, QueryMode.All, int.MaxValue);
        try { return s.Count; }
        finally { s.Release(); }
    }
}
