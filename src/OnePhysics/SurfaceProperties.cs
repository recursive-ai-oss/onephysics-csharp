using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics;

/// <summary>Optional body-frame surface behavior. Default is an ordinary, stationary, two-sided isotropic surface. In a planar world all vector Z components must be zero.</summary>
public struct SurfaceProperties : IEquatable<SurfaceProperties>
{
    /// <summary>Conveyor surface velocity in the owning body's frame, metres/second. Default zero; finite components within ±1e15.</summary>
    public Vector3F Velocity { get; set; }
    /// <summary>Allowed outward direction for a one-way surface, in the owning body's frame. Zero disables one-way behavior; any other finite vector is normalized deterministically. Default zero.</summary>
    public Vector3F OneWayDirection { get; set; }
    /// <summary>Body-frame longitudinal friction axis. Zero selects isotropic friction; a nonzero axis activates LongitudinalFriction and TransverseFriction in 3D. Default zero.</summary>
    public Vector3F FrictionAxis { get; set; }
    /// <summary>Longitudinal Coulomb coefficient when FrictionAxis is nonzero, within 0..1e15. Default zero.</summary>
    public float LongitudinalFriction { get; set; }
    /// <summary>Transverse Coulomb coefficient when FrictionAxis is nonzero, within 0..1e15. Default zero.</summary>
    public float TransverseFriction { get; set; }
    /// <summary>Torsional friction coefficient, metres, limiting torque about the contact normal per unit of normal force. Default zero; within 0..1e15. Applies only in 3D.</summary>
    public float SpinningFriction { get; set; }
    /// <summary>Field-wise equality of surface settings.</summary>
    public readonly bool Equals(SurfaceProperties other) => Velocity == other.Velocity && OneWayDirection == other.OneWayDirection && FrictionAxis == other.FrictionAxis && LongitudinalFriction == other.LongitudinalFriction && TransverseFriction == other.TransverseFriction && SpinningFriction == other.SpinningFriction;
    /// <inheritdoc/>
    public override readonly bool Equals(object? obj) => obj is SurfaceProperties surface && Equals(surface);
    /// <inheritdoc/>
    public override readonly int GetHashCode() => unchecked(Bits(Velocity.X) * 31 + Bits(Velocity.Y) * 17 + Bits(Velocity.Z) + Bits(SpinningFriction));
    /// <summary>Field-wise equality.</summary>
    public static bool operator ==(SurfaceProperties a, SurfaceProperties b) => a.Equals(b);
    /// <summary>Field-wise inequality.</summary>
    public static bool operator !=(SurfaceProperties a, SurfaceProperties b) => !a.Equals(b);
    private static int Bits(float value) => value == 0f ? 0 : BitConverter.SingleToInt32Bits(value);
    internal bool IsDefault => Velocity == default && OneWayDirection == default && FrictionAxis == default && LongitudinalFriction == 0 && TransverseFriction == 0 && SpinningFriction == 0;
    internal void Validate(int dimension)
    {
        CheckVector(Velocity, nameof(Velocity)); CheckVector(OneWayDirection, nameof(OneWayDirection)); CheckVector(FrictionAxis, nameof(FrictionAxis));
        CheckCoefficient(LongitudinalFriction, nameof(LongitudinalFriction)); CheckCoefficient(TransverseFriction, nameof(TransverseFriction)); CheckCoefficient(SpinningFriction, nameof(SpinningFriction));
        if (dimension == 2 && (Velocity.Z != 0 || OneWayDirection.Z != 0 || FrictionAxis != default || SpinningFriction != 0)) throw new ArgumentOutOfRangeException(nameof(dimension), "Planar surfaces require zero Z components, no anisotropic axis and no torsional friction.");
    }
    private static void CheckVector(Vector3F v, string name)
    {
        if (!(Math.Abs(v.X) <= 1e15f && Math.Abs(v.Y) <= 1e15f && Math.Abs(v.Z) <= 1e15f)) throw new ArgumentOutOfRangeException(name, v, "Surface vector components must be finite within ±1e15.");
        float length = Vector3F.Dot(v, v);
        if (v != default && !(length > 0f)) throw new ArgumentOutOfRangeException(name, v, "Surface vector is too small to normalize.");
    }
    private static void CheckCoefficient(float value, string name)
    {
        if (!(value >= 0 && value <= MaterialLimits.MaxCoefficient)) throw new ArgumentOutOfRangeException(name, value, "Surface coefficient must be finite within 0..1e15.");
    }
}
