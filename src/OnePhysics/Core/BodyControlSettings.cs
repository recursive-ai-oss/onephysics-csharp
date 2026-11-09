using System;

namespace RecursiveAI.OnePhysics;

/// <summary>World-axis rigid-body constraints. In 2D only X/Y position and Z rotation apply. Rotation locks constrain angular velocity, not Euler angles.</summary>
[Flags]
public enum RigidbodyConstraints
{
    /// <summary>No locked axes.</summary>
    None = 0,
    /// <summary>Lock world X translation.</summary>
    FreezePositionX = 1,
    /// <summary>Lock world Y translation.</summary>
    FreezePositionY = 2,
    /// <summary>Lock world Z translation (3D only).</summary>
    FreezePositionZ = 4,
    /// <summary>Lock world X angular velocity (3D only).</summary>
    FreezeRotationX = 8,
    /// <summary>Lock world Y angular velocity (3D only).</summary>
    FreezeRotationY = 16,
    /// <summary>Lock world Z angular velocity.</summary>
    FreezeRotationZ = 32,
    /// <summary>Lock all translation axes; in 2D use X | Y.</summary>
    FreezePosition = 7,
    /// <summary>Lock all rotation axes; in 2D use Z.</summary>
    FreezeRotation = 56,
    /// <summary>Lock every axis; accepted in both dimensions.</summary>
    FreezeAll = 63,
}

/// <summary>Optional per-body world-axis constraints and speed limits. Default means no locks and inherits world limits. Limits in m/s and rad/s are finite and nonnegative; float.MaxValue means no additional per-body limit.</summary>
public readonly struct BodyControlSettings
{
    private readonly uint _linear, _angular, _push;
    /// <summary>Creates settings. Undefined constraint bits or negative/non-finite limits throw ArgumentOutOfRangeException. Dimension-inapplicable axes are rejected when assigned to a body (FreezeAll is accepted in both).</summary>
    public BodyControlSettings(RigidbodyConstraints constraints = RigidbodyConstraints.None, float maxLinearSpeed = float.MaxValue, float maxAngularSpeed = float.MaxValue, float maxDepenetrationSpeed = float.MaxValue)
    {
        if (((int)constraints & ~63) != 0) throw new ArgumentOutOfRangeException(nameof(constraints), constraints, "Undefined constraint bits.");
        ValidateSpeed(maxLinearSpeed, nameof(maxLinearSpeed)); ValidateSpeed(maxAngularSpeed, nameof(maxAngularSpeed)); ValidateSpeed(maxDepenetrationSpeed, nameof(maxDepenetrationSpeed));
        Constraints = constraints; _linear = ZeroDefault.Set(maxLinearSpeed, float.MaxValue); _angular = ZeroDefault.Set(maxAngularSpeed, float.MaxValue); _push = ZeroDefault.Set(maxDepenetrationSpeed, float.MaxValue);
    }
    /// <summary>Locked world axes; default None.</summary>
    public RigidbodyConstraints Constraints { get; }
    /// <summary>Per-body maximum linear speed in m/s, combined with the world's limit by minimum.</summary>
    public float MaxLinearSpeed => ZeroDefault.Get(_linear, float.MaxValue);
    /// <summary>Per-body maximum angular speed in rad/s, combined with the world's limits by minimum.</summary>
    public float MaxAngularSpeed => ZeroDefault.Get(_angular, float.MaxValue);
    /// <summary>Maximum contact push-out speed in m/s, combined by minimum with world PushOutVelocity.</summary>
    public float MaxDepenetrationSpeed => ZeroDefault.Get(_push, float.MaxValue);
    internal bool IsDefault => Constraints == 0 && _linear == 0 && _angular == 0 && _push == 0;
    internal static void ValidateSpeed(float value, string name) { if (!float.IsFinite(value) || value < 0) throw new ArgumentOutOfRangeException(name, value, "Speed must be finite and nonnegative."); }
}
