using System;
using System.Globalization;

namespace RecursiveAI.OnePhysics;

/// <summary>How a link of an articulation moves relative to its parent link.</summary>
public enum ArticulationJointKind : byte
{
    /// <summary>No motion: the link is welded to its parent.</summary>
    Fixed = 0,

    /// <summary>Rotation about an axis fixed in the link (1 degree of freedom, radians).</summary>
    Revolute = 1,

    /// <summary>Sliding along an axis fixed in the link (1 degree of freedom, metres).</summary>
    Prismatic = 2,

    /// <summary>Free rotation about the joint point (3 degrees of freedom, a quaternion). 3D only.</summary>
    Spherical = 3,
}

/// <summary>
/// A position and velocity drive on a joint: a spring and a damper towards a target, with a limit on the force. The force is
/// <c>−Stiffness (position − TargetPosition) − Damping (velocity − TargetVelocity)</c>, clamped to ±<see cref="MaxForce"/> when that is above 0. It is integrated implicitly, so any stiffness is stable at any step.
/// For a spherical joint the position error is the rotation vector to the target orientation and the force is a torque.
/// </summary>
public struct ArticulationDrive : IEquatable<ArticulationDrive>
{
    /// <summary>Spring constant: newton-metres per radian or newtons per metre. Finite, at least 0. Default 0 (no spring).</summary>
    public float Stiffness { get; set; }

    /// <summary>Damping: newton-metre-seconds per radian or newton-seconds per metre. Finite, at least 0. Default 0.</summary>
    public float Damping { get; set; }

    /// <summary>Largest force or torque the drive may apply; 0 (default) means no limit. Finite, at least 0.</summary>
    public float MaxForce { get; set; }

    /// <summary>Position the spring pulls towards: radians or metres from the joint's zero. Default 0.</summary>
    public float TargetPosition { get; set; }

    /// <summary>Velocity the damper pulls towards. Default 0.</summary>
    public float TargetVelocity { get; set; }

    /// <summary>A position drive: a stiff spring with critical-ish damping chosen for a joint carrying <paramref name="inertia"/> (kg·m² or kg) to settle in about <paramref name="settleTime"/> seconds.</summary>
    /// <param name="inertia">The effective inertia the joint moves; above 0.</param>
    /// <param name="settleTime">Time to settle, seconds; above 0.</param>
    /// <param name="maxForce">Force limit; 0 for none.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite and above 0.</exception>
    public static ArticulationDrive Position(float inertia, float settleTime = 0.25f, float maxForce = 0f)
    {
        if (!(inertia > 0f) || !float.IsFinite(inertia)) { throw new ArgumentOutOfRangeException(nameof(inertia), inertia, "inertia must be finite and above 0."); }
        if (!(settleTime > 0f) || !float.IsFinite(settleTime)) { throw new ArgumentOutOfRangeException(nameof(settleTime), settleTime, "settleTime must be finite and above 0."); }
        // Critical damping: k = ω² I, c = 2 ω I with ω = 4 / settle time (a critically damped system is within 2% after 4/ω).
        float w = 4f / settleTime;
        return new ArticulationDrive { Stiffness = w * w * inertia, Damping = 2f * w * inertia, MaxForce = maxForce };
    }

    /// <inheritdoc/>
    public readonly bool Equals(ArticulationDrive other) => Stiffness == other.Stiffness && Damping == other.Damping && MaxForce == other.MaxForce && TargetPosition == other.TargetPosition && TargetVelocity == other.TargetVelocity;

    /// <inheritdoc/>
    public override readonly bool Equals(object? obj) => obj is ArticulationDrive d && Equals(d);

    /// <inheritdoc/>
    public override readonly int GetHashCode() => unchecked((BitConverter.SingleToInt32Bits(Stiffness) * 31 + BitConverter.SingleToInt32Bits(Damping)) * 31 + BitConverter.SingleToInt32Bits(TargetPosition));

    /// <summary>Equality of all fields.</summary>
    public static bool operator ==(ArticulationDrive a, ArticulationDrive b) => a.Equals(b);

    /// <summary>Inequality of any field.</summary>
    public static bool operator !=(ArticulationDrive a, ArticulationDrive b) => !a.Equals(b);

    /// <inheritdoc/>
    public override readonly string ToString() => string.Create(CultureInfo.InvariantCulture, $"Drive(k {Stiffness}, d {Damping}, max {MaxForce}, target {TargetPosition})");
}
