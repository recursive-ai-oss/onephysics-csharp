using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    internal bool SelectedQuery(QueryParams q, float length, int body, int shape, in QueryFilter3D filter, out QueryHit3D hit)
    {
        q.OnlyBodyPlusOne = body + 1;
        q.OnlyShapePlusOne = shape + 1;
        return QueryOne(in q, length, in filter, QueryMode.Closest, out hit);
    }

    internal bool SelectedRay(int body, int shape, Vector3D origin, Vector3F direction, float maxDistance, in QueryFilter3D filter, out QueryHit3D hit)
    {
        var q = RayParams(origin, direction, maxDistance, out float length);
        return SelectedQuery(q, length, body, shape, in filter, out hit);
    }

    internal bool SelectedCast(int body, int shape, in ConvexShape3D mover, Vector3D position, QuaternionF rotation, Vector3F direction, float maxDistance, in QueryFilter3D filter, out QueryHit3D hit)
    {
        var q = CastParams(in mover, position, rotation, direction, maxDistance, out float length);
        return SelectedQuery(q, length, body, shape, in filter, out hit);
    }

    internal bool SelectedOverlap(int body, int shape, in ConvexShape3D mover, Vector3D position, QuaternionF rotation, in QueryFilter3D filter, out QueryHit3D hit)
    {
        var q = OverlapParams(in mover, position, rotation);
        return SelectedQuery(q, 0f, body, shape, in filter, out hit);
    }

    internal bool SelectedClosest(int body, int shape, Vector3D point, float maxDistance, in QueryFilter3D filter, out QueryHit3D hit)
    {
        var q = ClosestParams(point, maxDistance);
        return SelectedQuery(q, 0f, body, shape, in filter, out hit);
    }
}

public readonly partial struct Body
{
    /// <summary>Casts against only this body; all normal query filtering and distance limits apply. A stale handle throws StaleHandleException.</summary>
    public bool Raycast(Vector3D origin, Vector3F direction, out QueryHit3D hit, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
    {
        var (w, b) = Resolve();
        return w.SelectedRay(b, -1, origin, direction, maxDistance, in filter, out hit);
    }

    /// <summary>Sweeps a query shape against only this body. Uses the same validation, hit convention and filtering as the world query; stale handles throw StaleHandleException.</summary>
    public bool ShapeCast(in ConvexShape3D mover, Vector3D position, QuaternionF rotation, Vector3F direction, out QueryHit3D hit, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
    {
        var (w, b) = Resolve();
        return w.SelectedCast(b, -1, in mover, position, rotation, direction, maxDistance, in filter, out hit);
    }

    /// <summary>Tests a query shape against only this body. Reports the nearest overlapping shape, or false; stale handles throw StaleHandleException.</summary>
    public bool OverlapShape(in ConvexShape3D mover, Vector3D position, QuaternionF rotation, out QueryHit3D hit, in QueryFilter3D filter = default)
    {
        var (w, b) = Resolve();
        return w.SelectedOverlap(b, -1, in mover, position, rotation, in filter, out hit);
    }

    /// <summary>Finds the closest point on only this body. Distance is in metres, nonnegative (default 1000); stale handles throw StaleHandleException.</summary>
    public bool ClosestPoint(Vector3D point, out QueryHit3D hit, float maxDistance = 1000f, in QueryFilter3D filter = default)
    {
        var (w, b) = Resolve();
        return w.SelectedClosest(b, -1, point, maxDistance, in filter, out hit);
    }
}

public readonly partial struct Shape
{
    /// <summary>Casts against only this shape; all normal query filtering and distance limits apply. A stale handle throws StaleHandleException.</summary>
    public bool Raycast(Vector3D origin, Vector3F direction, out QueryHit3D hit, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
    {
        int s = Resolve(); var w = _world!;
        return w.SelectedRay(-1, s, origin, direction, maxDistance, in filter, out hit);
    }

    /// <summary>Sweeps a query shape against only this shape. Uses the same validation, hit convention and filtering as the world query; stale handles throw StaleHandleException.</summary>
    public bool ShapeCast(in ConvexShape3D mover, Vector3D position, QuaternionF rotation, Vector3F direction, out QueryHit3D hit, float maxDistance = float.PositiveInfinity, in QueryFilter3D filter = default)
    {
        int s = Resolve(); var w = _world!;
        return w.SelectedCast(-1, s, in mover, position, rotation, direction, maxDistance, in filter, out hit);
    }

    /// <summary>Tests a query shape against only this shape. Reports the nearest overlapping shape, or false; stale handles throw StaleHandleException.</summary>
    public bool OverlapShape(in ConvexShape3D mover, Vector3D position, QuaternionF rotation, out QueryHit3D hit, in QueryFilter3D filter = default)
    {
        int s = Resolve(); var w = _world!;
        return w.SelectedOverlap(-1, s, in mover, position, rotation, in filter, out hit);
    }

    /// <summary>Finds the closest point on only this shape. Distance is in metres, nonnegative (default 1000); stale handles throw StaleHandleException.</summary>
    public bool ClosestPoint(Vector3D point, out QueryHit3D hit, float maxDistance = 1000f, in QueryFilter3D filter = default)
    {
        int s = Resolve(); var w = _world!;
        return w.SelectedClosest(-1, s, point, maxDistance, in filter, out hit);
    }
}
