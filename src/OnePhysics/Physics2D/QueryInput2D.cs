using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>One validated, immutable query for a mixed batch. Create with the factories; default is invalid. Positions and lengths use the world query limits, metres and radians.</summary>
public readonly struct QueryInput2D
{
    internal readonly QueryParams Params;
    internal readonly float Length;
    internal readonly bool Initialized;

    internal QueryInput2D(QueryParams parameters, float length) { Params = parameters; Length = length; Initialized = true; }

    /// <summary>Creates a ray, normalizing its nonzero direction. Distance is positive; infinity clamps to the world limit. Invalid inputs throw ArgumentOutOfRangeException.</summary>
    public static QueryInput2D Ray(Vector2D origin, Vector2F direction, float maxDistance = float.PositiveInfinity) => PhysicsWorld2D.BatchRay(origin, direction, maxDistance);

    /// <summary>Creates a translation-only shape cast, with the world query's validation and starting-overlap behavior.</summary>
    public static QueryInput2D Cast(in Shape2D shape, Vector2D position, float angleRadians, Vector2F direction, float maxDistance = float.PositiveInfinity) => PhysicsWorld2D.BatchCast(in shape, position, angleRadians, direction, maxDistance);

    /// <summary>Creates an overlap query, with the world query's shape and pose validation.</summary>
    public static QueryInput2D Overlap(in Shape2D shape, Vector2D position, float angleRadians) => PhysicsWorld2D.BatchOverlap(in shape, position, angleRadians);

    /// <summary>Creates a deepest-leaf penetration query, with the world Penetration method's semantics and validation.</summary>
    public static QueryInput2D Penetration(in Shape2D shape, Vector2D position, float angleRadians) => PhysicsWorld2D.BatchPenetration(in shape, position, angleRadians);

    /// <summary>Creates a closest-point query. Search distance is nonnegative (default 1000 metres), with the world query's limits and validation.</summary>
    public static QueryInput2D Closest(Vector2D point, float maxDistance = 1000f) => PhysicsWorld2D.BatchClosest(point, maxDistance);
}

public sealed partial class PhysicsWorld2D
{
    internal static QueryInput2D BatchRay(Vector2D origin, Vector2F direction, float maxDistance)
    { var q = RayParams(origin, direction, maxDistance, out float length); return new QueryInput2D(q, length); }
    internal static QueryInput2D BatchCast(in Shape2D shape, Vector2D position, float angleRadians, Vector2F direction, float maxDistance)
    { var q = CastParams(in shape, position, angleRadians, direction, maxDistance, out float length); return new QueryInput2D(q, length); }
    internal static QueryInput2D BatchOverlap(in Shape2D shape, Vector2D position, float angleRadians) => new(OverlapParams(in shape, position, angleRadians), 0f);
    internal static QueryInput2D BatchPenetration(in Shape2D shape, Vector2D position, float angleRadians)
    { var q = OverlapParams(in shape, position, angleRadians); q.Kind = QueryKind.Penetration; return new QueryInput2D(q, 0f); }
    internal static QueryInput2D BatchClosest(Vector2D point, float maxDistance) => new(ClosestParams(point, maxDistance), 0f);
}
