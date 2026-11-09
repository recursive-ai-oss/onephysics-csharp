using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    internal bool SelectedQuery(QueryParams q, float length, int body, int shape, in QueryFilter2D filter, out QueryHit2D hit)
    {
        q.OnlyBodyPlusOne = body + 1;
        q.OnlyShapePlusOne = shape + 1;
        return QueryOne(in q, length, in filter, QueryMode.Closest, out hit);
    }

    internal bool SelectedRay(int body, int shape, Vector2D origin, Vector2F direction, float maxDistance, in QueryFilter2D filter, out QueryHit2D hit)
    {
        var q = RayParams(origin, direction, maxDistance, out float length);
        return SelectedQuery(q, length, body, shape, in filter, out hit);
    }

    internal bool SelectedCast(int body, int shape, in Shape2D mover, Vector2D position, float angleRadians, Vector2F direction, float maxDistance, in QueryFilter2D filter, out QueryHit2D hit)
    {
        var q = CastParams(in mover, position, angleRadians, direction, maxDistance, out float length);
        return SelectedQuery(q, length, body, shape, in filter, out hit);
    }

    internal bool SelectedOverlap(int body, int shape, in Shape2D mover, Vector2D position, float angleRadians, in QueryFilter2D filter, out QueryHit2D hit)
    {
        var q = OverlapParams(in mover, position, angleRadians);
        return SelectedQuery(q, 0f, body, shape, in filter, out hit);
    }

    internal bool SelectedClosest(int body, int shape, Vector2D point, float maxDistance, in QueryFilter2D filter, out QueryHit2D hit)
    {
        var q = ClosestParams(point, maxDistance);
        return SelectedQuery(q, 0f, body, shape, in filter, out hit);
    }
}

public readonly partial struct Body
{
    /// <summary>Casts against only this body; all normal query filtering and distance limits apply. A stale handle throws StaleHandleException.</summary>
    public bool Raycast(Vector2D origin, Vector2F direction, out QueryHit2D hit, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
    {
        var (w, b) = Resolve();
        return w.SelectedRay(b, -1, origin, direction, maxDistance, in filter, out hit);
    }

    /// <summary>Sweeps a query shape against only this body. Uses the same validation, hit convention and filtering as the world query; stale handles throw StaleHandleException.</summary>
    public bool ShapeCast(in Shape2D mover, Vector2D position, float angleRadians, Vector2F direction, out QueryHit2D hit, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
    {
        var (w, b) = Resolve();
        return w.SelectedCast(b, -1, in mover, position, angleRadians, direction, maxDistance, in filter, out hit);
    }

    /// <summary>Tests a query shape against only this body. Reports the nearest overlapping shape, or false; stale handles throw StaleHandleException.</summary>
    public bool OverlapShape(in Shape2D mover, Vector2D position, float angleRadians, out QueryHit2D hit, in QueryFilter2D filter = default)
    {
        var (w, b) = Resolve();
        return w.SelectedOverlap(b, -1, in mover, position, angleRadians, in filter, out hit);
    }

    /// <summary>Finds the closest point on only this body. Distance is in metres, nonnegative (default 1000); stale handles throw StaleHandleException.</summary>
    public bool ClosestPoint(Vector2D point, out QueryHit2D hit, float maxDistance = 1000f, in QueryFilter2D filter = default)
    {
        var (w, b) = Resolve();
        return w.SelectedClosest(b, -1, point, maxDistance, in filter, out hit);
    }
}

public readonly partial struct Shape
{
    /// <summary>Casts against only this shape; all normal query filtering and distance limits apply. A stale handle throws StaleHandleException.</summary>
    public bool Raycast(Vector2D origin, Vector2F direction, out QueryHit2D hit, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
    {
        int s = Resolve(); var w = _world!;
        return w.SelectedRay(-1, s, origin, direction, maxDistance, in filter, out hit);
    }

    /// <summary>Sweeps a query shape against only this shape. Uses the same validation, hit convention and filtering as the world query; stale handles throw StaleHandleException.</summary>
    public bool ShapeCast(in Shape2D mover, Vector2D position, float angleRadians, Vector2F direction, out QueryHit2D hit, float maxDistance = float.PositiveInfinity, in QueryFilter2D filter = default)
    {
        int s = Resolve(); var w = _world!;
        return w.SelectedCast(-1, s, in mover, position, angleRadians, direction, maxDistance, in filter, out hit);
    }

    /// <summary>Tests a query shape against only this shape. Reports the nearest overlapping shape, or false; stale handles throw StaleHandleException.</summary>
    public bool OverlapShape(in Shape2D mover, Vector2D position, float angleRadians, out QueryHit2D hit, in QueryFilter2D filter = default)
    {
        int s = Resolve(); var w = _world!;
        return w.SelectedOverlap(-1, s, in mover, position, angleRadians, in filter, out hit);
    }

    /// <summary>Finds the closest point on only this shape. Distance is in metres, nonnegative (default 1000); stale handles throw StaleHandleException.</summary>
    public bool ClosestPoint(Vector2D point, out QueryHit2D hit, float maxDistance = 1000f, in QueryFilter2D filter = default)
    {
        int s = Resolve(); var w = _world!;
        return w.SelectedClosest(-1, s, point, maxDistance, in filter, out hit);
    }
}
