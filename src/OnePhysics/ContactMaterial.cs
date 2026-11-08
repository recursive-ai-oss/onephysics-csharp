using System;

namespace RecursiveAI.OnePhysics;

/// <summary>Dimension-neutral scalar coefficients for an indexed triangle or segment material. Friction/rolling resistance are within 0..1e15; restitution is within 0..1.</summary>
public readonly struct ContactMaterial
{
    /// <summary>Creates validated scalar coefficients. Default restitution and rolling resistance are zero.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A coefficient is non-finite or out of range.</exception>
    public ContactMaterial(float friction, float restitution = 0, float rollingResistance = 0)
    {
        if (!(friction >= 0 && friction <= MaterialLimits.MaxCoefficient)) throw new ArgumentOutOfRangeException(nameof(friction), friction, "Friction must be finite within 0..1e15.");
        if (!(restitution >= 0 && restitution <= 1)) throw new ArgumentOutOfRangeException(nameof(restitution), restitution, "Restitution must be within 0..1.");
        if (!(rollingResistance >= 0 && rollingResistance <= MaterialLimits.MaxCoefficient)) throw new ArgumentOutOfRangeException(nameof(rollingResistance), rollingResistance, "Rolling resistance must be finite within 0..1e15.");
        Friction = friction; Restitution = restitution; RollingResistance = rollingResistance;
    }
    /// <summary>Coulomb friction coefficient.</summary>
    public float Friction { get; }
    /// <summary>Restitution in 0..1.</summary>
    public float Restitution { get; }
    /// <summary>Rolling resistance coefficient.</summary>
    public float RollingResistance { get; }
}
