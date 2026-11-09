using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>Continuously sweeps a body's full geometry while its centre of mass translates and it rotates by angularDisplacement (signed angle, radians). Motion has constant linear/angular velocity over fraction [0,1]. Zero translation is valid. Translation length is at most MaxQueryDistance; angular magnitude at most 64 radians. Tolerance is finite in (0,1], default 0.0001 metre, plus the raw solver's relative float margin. Initial overlap reports fraction zero. Fraction is time, Distance is centre translation travelled (zero for pure rotation). No world state changes. Self and optional direct joint neighbours are excluded; target filters apply. Ties use target slot then source/leaf order.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Motion or tolerance is nonfinite or outside its limits.</exception>
    /// <exception cref="InvalidHandleException">The body is null, foreign or stale.</exception>
    /// <exception cref="QueryConvergenceException">Conservative advancement cannot certify progress within its fixed bound. No hit is silently skipped.</exception>
    public bool SweepBody(Body body, Vector2F translation, float angularDisplacement, out QueryHit2D hit, bool excludeJointed = false, in QueryFilter2D filter = default, float tolerance = 0.0001f)
    {
        if (!ReferenceEquals(body.World, this)) { throw new InvalidHandleException("The sweep body is null or belongs to a different world."); }
        int slot = _core.Resolve(body.RawHandle);
        double length = double.Sqrt((double)translation.X * translation.X + (double)translation.Y * translation.Y);
        double angle = Math.Abs((double)angularDisplacement);
        if (!double.IsFinite(length) || length > MaxQueryDistance) { throw new ArgumentOutOfRangeException(nameof(translation), translation, "Translation must be finite and at most MaxQueryDistance metres long."); }
        if (!double.IsFinite(angle) || angle > 64) { throw new ArgumentOutOfRangeException(nameof(angularDisplacement), angularDisplacement, "Angular displacement must be finite with magnitude at most 64 radians."); }
        if (!(tolerance > 0 && tolerance <= 1) || !float.IsFinite(tolerance)) { throw new ArgumentOutOfRangeException(nameof(tolerance), tolerance, "Tolerance must be finite in (0,1] metres."); }
        if (!_engine.AngularSweepBody(slot, translation, angularDisplacement, tolerance, excludeJointed, in filter, out var raw)) { hit = default; return false; }
        hit = ToHit(in raw, (float)length); return true;
    }
}

public readonly partial struct Body
{
    /// <summary>Continuous translation and angular sweep; see PhysicsWorld2D.SweepBody. Does not move or wake the body; invalid handles, uncertifiable convergence raise typed errors.</summary>
    public bool Sweep(Vector2F translation, float angularDisplacement, out QueryHit2D hit, bool excludeJointed = false, in QueryFilter2D filter = default, float tolerance = 0.0001f)
    {
        var (world, _) = Resolve();
        return world.SweepBody(this, translation, angularDisplacement, out hit, excludeJointed, in filter, tolerance);
    }
}

public sealed partial class PhysicsQuery2D
{
    /// <summary>Continuous full-body translation/angular sweep with this object's filter; see the world overload for units, limits, convergence and initial-overlap semantics.</summary>
    public bool SweepBody(Body body, Vector2F translation, float angularDisplacement, out QueryHit2D hit, bool excludeJointed = false, float tolerance = 0.0001f) => _world.SweepBody(body, translation, angularDisplacement, out hit, excludeJointed, in _filter, tolerance);
}
