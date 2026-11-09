using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>Read-only body diagnostics. Read between steps; getters validate the handle and throw for default or stale bodies. No allocation once world capacity is reserved.</summary>

public readonly struct BodyInspection
{
    private readonly RecursiveAI.OnePhysics.BodyInspection _raw;
    internal BodyInspection(RecursiveAI.OnePhysics.BodyInspection raw) { _raw = raw; }
    /// <summary>Gravity before damping, kg m/s. World axes.</summary>
    /// <exception cref="DetailedInspectionDisabledException">Enable Options.DetailedInspection before stepping to collect these values.</exception>
    public Vector2D GravityImpulse => new(_raw.GravityImpulse.X, _raw.GravityImpulse.Y);
    /// <summary>Integrated public force and direct impulse inputs, kg m/s; excludes solver and service-internal impulses. World axes.</summary>
    /// <exception cref="DetailedInspectionDisabledException">Enable Options.DetailedInspection before stepping to collect these values.</exception>
    public Vector2D ExternalImpulse => new(_raw.ExternalImpulse.X, _raw.ExternalImpulse.Y);
    /// <summary>Momentum removed by damping before speed clamps, kg m/s. World axes.</summary>
    /// <exception cref="DetailedInspectionDisabledException">Enable Options.DetailedInspection before stepping to collect these values.</exception>
    public Vector2D DampingImpulse => new(_raw.DampingImpulse.X, _raw.DampingImpulse.Y);
    /// <summary>Integrated public torque and angular impulse inputs, kg m squared/s, about the centre of mass in world axes.</summary>
    /// <exception cref="DetailedInspectionDisabledException">Enable Options.DetailedInspection before stepping to collect these values.</exception>
    public double ExternalAngularImpulse => _raw.ExternalAngularImpulse.Z;
    /// <summary>Angular momentum removed by damping before speed clamps, kg m squared/s, about the centre of mass in world axes.</summary>
    /// <exception cref="DetailedInspectionDisabledException">Enable Options.DetailedInspection before stepping to collect these values.</exception>
    public double DampingAngularImpulse => _raw.DampingAngularImpulse.Z;
    /// <summary>Current dynamic island, or -1.</summary>
    public int Island => _raw.Island;
    /// <summary>Current awake state.</summary>
    public bool IsAwake => _raw.IsAwake;
    /// <summary>Current resting time in seconds.</summary>
    public float SleepTime => _raw.SleepTime;
    /// <summary>Current contact and joint count; scans constraint slots without allocation.</summary>
    public int ConstraintCount => _raw.ConstraintCount;
    /// <summary>Current colour bits 0..23, with overflow in bit 24.</summary>
    public uint ConstraintColorMask => _raw.ConstraintColorMask;

    /// <summary>Live contact and joint views in ascending constraint slot order. Invalidated by world mutation.</summary>
    public BodyConstraintEnumerable Constraints => _raw.Constraints;
}
