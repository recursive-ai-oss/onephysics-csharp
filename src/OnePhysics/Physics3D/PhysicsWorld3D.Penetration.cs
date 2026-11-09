using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>Returns the deepest penetrating shape. Distance is penetration depth in metres, Normal points from the target towards the query shape, Point is a contact point, and Fraction is zero. Exact touching is not penetration. Ties break by shape slot. For compound or concave targets the deepest leaf contact is reported, not a translation separating the entire union.</summary>
    /// <exception cref="ArgumentException">The shape is invalid.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The pose is invalid; the usual world query limits apply.</exception>
    public bool Penetration(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, out QueryHit3D hit, in QueryFilter3D filter = default)
    {
        var q = OverlapParams(in shape, position, rotation); q.Kind = QueryKind.Penetration;
        return QueryOne(in q, 0f, in filter, QueryMode.Closest, out hit);
    }

    /// <summary>Reports one deepest leaf penetration per target shape, deepest first (ties by shape slot); a short buffer keeps the deepest. See Penetration for result semantics and concave-target limitations.</summary>
    public int PenetrationAll(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, Span<QueryHit3D> results, in QueryFilter3D filter = default)
    {
        var q = OverlapParams(in shape, position, rotation); q.Kind = QueryKind.Penetration;
        return QueryMany(in q, 0f, in filter, results);
    }

    /// <summary>True if any target shape strictly penetrates the query shape; same validation and filtering as Penetration.</summary>
    public bool PenetrationAny(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, in QueryFilter3D filter = default)
    {
        var q = OverlapParams(in shape, position, rotation); q.Kind = QueryKind.Penetration;
        return QueryOne(in q, 0f, in filter, QueryMode.Any, out _);
    }
}
