using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>Sweeps every shape of a body along a translation from its current pose without moving the world. Excludes the source body and optionally its direct joint neighbours. Returns the first hit; ties break by target shape slot, then source shape/leaf traversal order. Compounds and multi-shape bodies are swept leaf by leaf. Starting overlaps follow ShapeCast's convention. No angular motion is applied.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The direction or distance is invalid; the ray query limits apply.</exception>
    /// <exception cref="InvalidHandleException">The body is null or belongs to a different world.</exception>
    /// <exception cref="StaleHandleException">The body was destroyed.</exception>
    public bool SweepBody(Body body, Vector2F direction, out QueryHit2D hit, float maxDistance = float.PositiveInfinity, bool excludeJointed = false, in QueryFilter2D filter = default)
    {
        if (!ReferenceEquals(body.World, this)) { throw new InvalidHandleException("The sweep body is null or belongs to a different world."); }
        int slot = _core.Resolve(body.RawHandle);
        var q = RayParams(body.Position, direction, maxDistance, out float length);
        q.Kind = QueryKind.Cast; q.ExcludeBodyPlusOne = slot + 1; q.ExcludeJointed = excludeJointed;
        if (!_engine.SweepBody(slot, in q, in filter, out var raw)) { hit = default; return false; }
        hit = ToHit(in raw, length); return true;
    }
}

public readonly partial struct Body
{
    /// <summary>Sweeps this body's complete shape set along a translation; see the world's SweepBody. A stale handle throws StaleHandleException.</summary>
    public bool Sweep(Vector2F direction, out QueryHit2D hit, float maxDistance = float.PositiveInfinity, bool excludeJointed = false, in QueryFilter2D filter = default)
    {
        var (world, _) = Resolve();
        return world.SweepBody(this, direction, out hit, maxDistance, excludeJointed, in filter);
    }
}

public sealed partial class PhysicsQuery2D
{
    /// <summary>Full-body translation sweep using this query object's filter; see the world's SweepBody.</summary>
    public bool SweepBody(Body body, Vector2F direction, out QueryHit2D hit, float maxDistance = float.PositiveInfinity, bool excludeJointed = false) => _world.SweepBody(body, direction, out hit, maxDistance, excludeJointed, in _filter);
}
