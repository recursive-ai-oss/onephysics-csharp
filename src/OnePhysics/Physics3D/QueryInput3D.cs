using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>One validated, immutable query for a mixed batch. Create with the factories; default is invalid. Positions and lengths use the world query limits, metres and radians.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct QueryInput3D
{
    internal readonly QueryParams Params;
    internal readonly float Length;
    internal readonly bool Initialized;

    internal QueryInput3D(QueryParams parameters, float length) { Params = parameters; Length = length; Initialized = true; }

    /// <summary>Creates a ray, normalizing its nonzero direction. Distance is positive; infinity clamps to the world limit. Invalid inputs throw ArgumentOutOfRangeException.</summary>
    public static QueryInput3D Ray(Vector3D origin, Vector3F direction, float maxDistance = float.PositiveInfinity) => PhysicsWorld3D.BatchRay(origin, direction, maxDistance);

    /// <summary>Creates a translation-only shape cast, with the world query's validation and starting-overlap behavior.</summary>
    public static QueryInput3D Cast(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, Vector3F direction, float maxDistance = float.PositiveInfinity) => PhysicsWorld3D.BatchCast(in shape, position, rotation, direction, maxDistance);

    /// <summary>Creates an overlap query, with the world query's shape and pose validation.</summary>
    public static QueryInput3D Overlap(in ConvexShape3D shape, Vector3D position, QuaternionF rotation) => PhysicsWorld3D.BatchOverlap(in shape, position, rotation);

    /// <summary>Creates a deepest-leaf penetration query, with the world Penetration method's semantics and validation.</summary>
    public static QueryInput3D Penetration(in ConvexShape3D shape, Vector3D position, QuaternionF rotation) => PhysicsWorld3D.BatchPenetration(in shape, position, rotation);

    /// <summary>Creates a closest-point query. Search distance is nonnegative (default 1000 metres), with the world query's limits and validation.</summary>
    public static QueryInput3D Closest(Vector3D point, float maxDistance = 1000f) => PhysicsWorld3D.BatchClosest(point, maxDistance);
}

public sealed partial class PhysicsWorld3D
{
    internal static QueryInput3D BatchRay(Vector3D origin, Vector3F direction, float maxDistance)
    { var q = RayParams(origin, direction, maxDistance, out float length); return new QueryInput3D(q, length); }
    internal static QueryInput3D BatchCast(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, Vector3F direction, float maxDistance)
    { var q = CastParams(in shape, position, rotation, direction, maxDistance, out float length); return new QueryInput3D(q, length); }
    internal static QueryInput3D BatchOverlap(in ConvexShape3D shape, Vector3D position, QuaternionF rotation) => new(OverlapParams(in shape, position, rotation), 0f);
    internal static QueryInput3D BatchPenetration(in ConvexShape3D shape, Vector3D position, QuaternionF rotation)
    { var q = OverlapParams(in shape, position, rotation); q.Kind = QueryKind.Penetration; return new QueryInput3D(q, 0f); }
    internal static QueryInput3D BatchClosest(Vector3D point, float maxDistance) => new(ClosestParams(point, maxDistance), 0f);
}
