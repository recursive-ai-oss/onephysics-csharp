using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    // Deformable queries: the same ray, shape cast, overlap and closest-point queries as for rigid shapes, against the particles, edges and rod segments of soft bodies, strips and rods.

    private static float Reach(in Shape2D shape)
    {
        if (shape.Kind == ShapeKind2D.Empty) { throw new ArgumentException("The shape is empty and cannot be used for a deformable query.", nameof(shape)); }
        CheckShape(in shape);
        float m = 0f;
        var v = shape.Vertices;
        for (int i = 0; i < v.Length; i++) { m = Math.Max(m, v[i].Length); }
        return (m + shape.Radius) * 1.001f + 1e-4f;
    }

    internal int RunDeformableRay(Vector2D origin, Vector2F direction, float maxDistance, in DeformableFilter2D filter, DeformableHit2D[] buffer, int capacity, bool any)
    {
        CheckPosition(origin, nameof(origin));
        CheckCast(direction, maxDistance, out float tx, out float ty, out float length);
        var unit = new Vec2(tx / length, ty / length);
        var v = new DeformableVisitor2 { Mode = DeformableVisitor2.Ray, Ox = origin.X, Oy = origin.Y, Direction = unit, Length = length, Buffer = buffer, Capacity = capacity, Any = any };
        double ex = origin.X + (double)tx, ey = origin.Y + (double)ty;
        DeformableScan2D.Scan(_engine, in filter, Math.Min(origin.X, ex) - 1e-3, Math.Min(origin.Y, ey) - 1e-3, Math.Max(origin.X, ex) + 1e-3, Math.Max(origin.Y, ey) + 1e-3, ref v);
        return v.Count;
    }

    internal int RunDeformableCast(in Shape2D shape, Vector2D position, float angleRadians, Vector2F direction, float maxDistance, in DeformableFilter2D filter, DeformableHit2D[] buffer, int capacity, bool any)
    {
        CheckPosition(position, nameof(position));
        var rot = RotationOf(angleRadians);
        CheckCast(direction, maxDistance, out float tx, out float ty, out float length);
        float reach = Reach(in shape);
        var v = new DeformableVisitor2 { Mode = DeformableVisitor2.Cast, Ox = position.X, Oy = position.Y, Rotation = rot, Direction = new Vec2(tx / length, ty / length), Length = length, Shape = shape, Buffer = buffer, Capacity = capacity, Any = any };
        double ex = position.X + (double)tx, ey = position.Y + (double)ty;
        DeformableScan2D.Scan(_engine, in filter, Math.Min(position.X, ex) - reach, Math.Min(position.Y, ey) - reach, Math.Max(position.X, ex) + reach, Math.Max(position.Y, ey) + reach, ref v);
        return v.Count;
    }

    internal int RunDeformableOverlap(in Shape2D shape, Vector2D position, float angleRadians, in DeformableFilter2D filter, DeformableHit2D[] buffer, int capacity, bool any)
    {
        CheckPosition(position, nameof(position));
        var rot = RotationOf(angleRadians);
        float reach = Reach(in shape);
        var v = new DeformableVisitor2 { Mode = DeformableVisitor2.Overlap, Ox = position.X, Oy = position.Y, Rotation = rot, Shape = shape, Buffer = buffer, Capacity = capacity, Any = any };
        DeformableScan2D.Scan(_engine, in filter, position.X - reach, position.Y - reach, position.X + reach, position.Y + reach, ref v);
        return v.Count;
    }

    internal int CountDeformableOverlaps(in Shape2D shape, Vector2D position, in DeformableFilter2D filter)
    {
        float reach = Reach(in shape);
        var v = new DeformableVisitor2 { Mode = DeformableVisitor2.CountOverlap, Ox = position.X, Oy = position.Y, Rotation = Rot2.Identity, Shape = shape };
        DeformableScan2D.Scan(_engine, in filter, position.X - reach, position.Y - reach, position.X + reach, position.Y + reach, ref v);
        return v.Total;
    }

    internal bool RunDeformableClosest(Vector2D point, float maxDistance, in DeformableFilter2D filter, DeformableHit2D[] buffer, bool any)
    {
        CheckPosition(point, nameof(point));
        if (float.IsNaN(maxDistance) || !(maxDistance >= 0f)) { throw new ArgumentOutOfRangeException(nameof(maxDistance), maxDistance, "maxDistance must be at least 0 (infinity allowed)."); }
        float length = maxDistance < MaxQueryDistance ? maxDistance : MaxQueryDistance;
        var v = new DeformableVisitor2 { Mode = DeformableVisitor2.Closest, Ox = point.X, Oy = point.Y, Rotation = Rot2.Identity, Length = length, Shape = Engine2D.s_point, Buffer = buffer, Capacity = 1, Any = any };
        double r = length + 1e-3;
        DeformableScan2D.Scan(_engine, in filter, point.X - r, point.Y - r, point.X + r, point.Y + r, ref v);
        return v.Count > 0;
    }

    /// <summary>
    /// Casts a ray at the soft bodies, strips and rods and returns the nearest hit (ties on equal distance go to the lower kind, slot, element and index). Soft bodies are hit on the edges of their outline (strips on their
    /// edges), as capsules of the particle radius, or on their particles when they have none; rods on their segments (capsules of the rod's radius). Allocates nothing in steady state.
    /// </summary>
    /// <param name="origin">Start point, finite and within +-2^22.</param>
    /// <param name="direction">Direction, finite and not zero; it is normalized.</param>
    /// <param name="hit">The hit.</param>
    /// <param name="maxDistance">Ray length, positive; infinity (the default) means <see cref="MaxQueryDistance"/>.</param>
    /// <param name="filter">Which deformables count; the default accepts all.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite, the direction is zero, the length is not positive or the origin is beyond +-2^22.</exception>
    public bool RaycastDeformables(Vector2D origin, Vector2F direction, out DeformableHit2D hit, float maxDistance = float.PositiveInfinity, in DeformableFilter2D filter = default)
    {
        var b = Engine2D.DeformScratch(1);
        int n = RunDeformableRay(origin, direction, maxDistance, in filter, b, 1, false);
        hit = n > 0 ? b[0] : default;
        return n > 0;
    }

    /// <summary>True when the ray hits any deformable.</summary>
    /// <inheritdoc cref="RaycastDeformables"/>
    public bool RaycastDeformablesAny(Vector2D origin, Vector2F direction, float maxDistance = float.PositiveInfinity, in DeformableFilter2D filter = default) =>
        RunDeformableRay(origin, direction, maxDistance, in filter, Engine2D.DeformScratch(1), 1, true) > 0;

    /// <summary>All hits of a ray, nearest first, into a caller buffer: the nearest <c>results.Length</c> elements. Returns the count stored.</summary>
    /// <inheritdoc cref="RaycastDeformables"/>
    public int RaycastDeformablesAll(Vector2D origin, Vector2F direction, Span<DeformableHit2D> results, float maxDistance = float.PositiveInfinity, in DeformableFilter2D filter = default)
    {
        if (results.Length == 0) { return 0; }
        var b = Engine2D.DeformScratch(results.Length);
        int n = RunDeformableRay(origin, direction, maxDistance, in filter, b, results.Length, false);
        b.AsSpan(0, n).CopyTo(results);
        return n;
    }

    /// <summary>Sweeps a shape along a direction and returns the first element it touches; a shape that starts touching an element hits it at fraction 0.</summary>
    /// <param name="shape">The shape.</param>
    /// <param name="position">Where the shape's origin starts.</param>
    /// <param name="angleRadians">The shape's angle.</param>
    /// <param name="direction">Direction, finite and not zero; normalized.</param>
    /// <param name="hit">The hit.</param>
    /// <param name="maxDistance">Sweep length, positive; infinity means <see cref="MaxQueryDistance"/>.</param>
    /// <param name="filter">Which deformables count.</param>
    public bool ShapeCastDeformables(in Shape2D shape, Vector2D position, float angleRadians, Vector2F direction, out DeformableHit2D hit, float maxDistance = float.PositiveInfinity, in DeformableFilter2D filter = default)
    {
        var b = Engine2D.DeformScratch(1);
        int n = RunDeformableCast(in shape, position, angleRadians, direction, maxDistance, in filter, b, 1, false);
        hit = n > 0 ? b[0] : default;
        return n > 0;
    }

    /// <summary>True when the swept shape touches any deformable.</summary>
    /// <inheritdoc cref="ShapeCastDeformables"/>
    public bool ShapeCastDeformablesAny(in Shape2D shape, Vector2D position, float angleRadians, Vector2F direction, float maxDistance = float.PositiveInfinity, in DeformableFilter2D filter = default) =>
        RunDeformableCast(in shape, position, angleRadians, direction, maxDistance, in filter, Engine2D.DeformScratch(1), 1, true) > 0;

    /// <summary>Every element the swept shape touches, nearest first, into a caller buffer. Returns the count stored.</summary>
    /// <inheritdoc cref="ShapeCastDeformables"/>
    public int ShapeCastDeformablesAll(in Shape2D shape, Vector2D position, float angleRadians, Vector2F direction, Span<DeformableHit2D> results, float maxDistance = float.PositiveInfinity, in DeformableFilter2D filter = default)
    {
        if (results.Length == 0) { return 0; }
        var b = Engine2D.DeformScratch(results.Length);
        int n = RunDeformableCast(in shape, position, angleRadians, direction, maxDistance, in filter, b, results.Length, false);
        b.AsSpan(0, n).CopyTo(results);
        return n;
    }

    /// <summary>Casts a circle; see <see cref="ShapeCastDeformables"/>.</summary>
    public bool CircleCastDeformables(Vector2D center, float radius, Vector2F direction, out DeformableHit2D hit, float maxDistance = float.PositiveInfinity, in DeformableFilter2D filter = default)
    {
        CheckRadius(radius);
        return ShapeCastDeformables(radius > 0f ? Shape2D.CreateCircle(default, radius) : Engine2D.s_point, center, 0f, direction, out hit, maxDistance, in filter);
    }

    /// <summary>The first element (by kind, slot, element and index) a shape overlaps.</summary>
    /// <param name="shape">The shape.</param>
    /// <param name="position">Where the shape's origin is.</param>
    /// <param name="angleRadians">The shape's angle.</param>
    /// <param name="hit">The element and a point of contact.</param>
    /// <param name="filter">Which deformables count.</param>
    public bool OverlapDeformables(in Shape2D shape, Vector2D position, float angleRadians, out DeformableHit2D hit, in DeformableFilter2D filter = default)
    {
        var b = Engine2D.DeformScratch(1);
        int n = RunDeformableOverlap(in shape, position, angleRadians, in filter, b, 1, false);
        hit = n > 0 ? b[0] : default;
        return n > 0;
    }

    /// <summary>True when the shape overlaps any deformable.</summary>
    /// <inheritdoc cref="OverlapDeformables"/>
    public bool OverlapDeformablesAny(in Shape2D shape, Vector2D position, float angleRadians, in DeformableFilter2D filter = default) =>
        RunDeformableOverlap(in shape, position, angleRadians, in filter, Engine2D.DeformScratch(1), 1, true) > 0;

    /// <summary>Every element the shape overlaps, ordered by kind, slot, element and index, into a caller buffer. Returns the count stored.</summary>
    /// <inheritdoc cref="OverlapDeformables"/>
    public int OverlapDeformablesAll(in Shape2D shape, Vector2D position, float angleRadians, Span<DeformableHit2D> results, in DeformableFilter2D filter = default)
    {
        if (results.Length == 0) { return 0; }
        var b = Engine2D.DeformScratch(results.Length);
        int n = RunDeformableOverlap(in shape, position, angleRadians, in filter, b, results.Length, false);
        b.AsSpan(0, n).CopyTo(results);
        return n;
    }

    /// <summary>Every element a circle overlaps; see <see cref="OverlapDeformablesAll"/>.</summary>
    public int OverlapCircleDeformablesAll(Vector2D center, float radius, Span<DeformableHit2D> results, in DeformableFilter2D filter = default)
    {
        CheckRadius(radius);
        return OverlapDeformablesAll(radius > 0f ? Shape2D.CreateCircle(default, radius) : Engine2D.s_point, center, 0f, results, in filter);
    }

    /// <summary>The element nearest to a point within a distance: <see cref="DeformableHit2D.Distance"/> is the gap (0 inside an element) and <see cref="DeformableHit2D.Point"/> the nearest point on it.</summary>
    /// <param name="point">The point.</param>
    /// <param name="hit">The nearest element.</param>
    /// <param name="maxDistance">Search distance, at least 0 (infinity allowed); default 1000.</param>
    /// <param name="filter">Which deformables count.</param>
    public bool ClosestDeformable(Vector2D point, out DeformableHit2D hit, float maxDistance = 1000f, in DeformableFilter2D filter = default)
    {
        var b = Engine2D.DeformScratch(1);
        bool found = RunDeformableClosest(point, maxDistance, in filter, b, false);
        hit = found ? b[0] : default;
        return found;
    }

    /// <summary>True when any deformable lies within the distance of the point.</summary>
    /// <inheritdoc cref="ClosestDeformable"/>
    public bool ClosestDeformableAny(Vector2D point, float maxDistance = 1000f, in DeformableFilter2D filter = default) =>
        RunDeformableClosest(point, maxDistance, in filter, Engine2D.DeformScratch(1), true);
}
