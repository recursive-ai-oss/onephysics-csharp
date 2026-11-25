using System;
using System.Globalization;

namespace RecursiveAI.OnePhysics;

/// <summary>
/// How long one world unit is, in metres. A world authored in millimetres, centimetres or pixels sets this so that every tolerance, threshold, clamp and default size that is tuned in metres
/// (speculative distance, sleep speed, speed limits, gravity, default radii and the like) keeps its physical meaning instead of being read as that many world units.
/// The default value is one metre per unit, and a world at the default behaves exactly as it always has.
/// </summary>
/// <remarks>
/// Only <em>defaults</em> derive from the unit: a value the host sets explicitly (for example <c>WorldOptions3D.SpeculativeDistance</c>) is in world units and keeps that meaning.
/// Masses, densities, forces and times are not lengths and are not scaled; positions stay on the same 2^-32 world-unit grid within ±2^21 world units whatever the unit.
/// The unit is stored in the snapshot (in an optional section written only when the unit is not one metre) and is covered by the world hash.
/// The value is a small immutable struct; <c>default(LengthUnit)</c> is <see cref="Metres"/>.
/// </remarks>
public readonly struct LengthUnit : IEquatable<LengthUnit>
{
    /// <summary>The smallest accepted <see cref="MetresPerUnit"/>: one micrometre per world unit.</summary>
    public const double MinMetresPerUnit = 1e-6;

    /// <summary>The largest accepted <see cref="MetresPerUnit"/>: one thousand kilometres per world unit.</summary>
    public const double MaxMetresPerUnit = 1e6;

    // Zero stands for one metre, so default(LengthUnit) is metres.
    private readonly double _metresPerUnit;

    /// <summary>Creates a unit of <paramref name="metresPerUnit"/> metres per world unit.</summary>
    /// <param name="metresPerUnit">Metres in one world unit, finite and within <see cref="MinMetresPerUnit"/>..<see cref="MaxMetresPerUnit"/>.</param>
    /// <exception cref="ArgumentOutOfRangeException">The value is NaN, infinite, zero, negative, subnormal or outside the range.</exception>
    public LengthUnit(double metresPerUnit)
    {
        if (!IsValid(metresPerUnit))
        {
            throw new ArgumentOutOfRangeException(nameof(metresPerUnit), metresPerUnit, "Metres per unit must be finite and within " + MinMetresPerUnit.ToString("R", CultureInfo.InvariantCulture) + ".." + MaxMetresPerUnit.ToString("R", CultureInfo.InvariantCulture) + ".");
        }

        _metresPerUnit = metresPerUnit == 1.0 ? 0.0 : metresPerUnit;
    }

    /// <summary>True when <paramref name="metresPerUnit"/> is an accepted value (false for NaN, infinities, zero, negatives, subnormals and anything outside the range).</summary>
    public static bool IsValid(double metresPerUnit) => metresPerUnit >= MinMetresPerUnit && metresPerUnit <= MaxMetresPerUnit;

    /// <summary>One world unit is one metre. The default.</summary>
    public static LengthUnit Metres => default;

    /// <summary>One world unit is one centimetre (0.01 m).</summary>
    public static LengthUnit Centimetres => new(0.01);

    /// <summary>One world unit is one millimetre (0.001 m).</summary>
    public static LengthUnit Millimetres => new(0.001);

    /// <summary>One world unit is one pixel, at <paramref name="pixelsPerMetre"/> pixels to the metre (for example 32 or 100 for a 2D game).</summary>
    /// <exception cref="ArgumentOutOfRangeException">The pixel density is not finite and positive, or the resulting unit is outside the accepted range.</exception>
    public static LengthUnit Pixels(double pixelsPerMetre)
    {
        if (!(pixelsPerMetre > 0.0) || !double.IsFinite(pixelsPerMetre)) { throw new ArgumentOutOfRangeException(nameof(pixelsPerMetre), pixelsPerMetre, "Pixels per metre must be finite and positive."); }
        return new LengthUnit(1.0 / pixelsPerMetre);
    }

    /// <summary>Metres in one world unit. 1 by default.</summary>
    public double MetresPerUnit => _metresPerUnit == 0.0 ? 1.0 : _metresPerUnit;

    /// <summary>World units in one metre (the reciprocal of <see cref="MetresPerUnit"/>). 1 by default.</summary>
    public double UnitsPerMetre => _metresPerUnit == 0.0 ? 1.0 : 1.0 / _metresPerUnit;

    /// <summary>True for the default unit, one metre.</summary>
    public bool IsMetres => _metresPerUnit == 0.0;

    /// <summary>Converts a length in world units to metres.</summary>
    public double ToMetres(double worldUnits) => _metresPerUnit == 0.0 ? worldUnits : worldUnits * _metresPerUnit;

    /// <summary>Converts a length in metres to world units.</summary>
    public double FromMetres(double metres) => _metresPerUnit == 0.0 ? metres : metres / _metresPerUnit;

    // World units per metre as the float the engine multiplies its metre-tuned constants by. Exactly 1 for the default unit, so a world at one metre computes what it always has.
    internal float Scale => _metresPerUnit == 0.0 ? 1f : (float)(1.0 / _metresPerUnit);

    /// <inheritdoc/>
    public bool Equals(LengthUnit other) => BitConverter.DoubleToInt64Bits(_metresPerUnit) == BitConverter.DoubleToInt64Bits(other._metresPerUnit);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is LengthUnit u && Equals(u);

    /// <inheritdoc/>
    public override int GetHashCode() => _metresPerUnit.GetHashCode();

    /// <summary>Equality of the metres-per-unit value.</summary>
    public static bool operator ==(LengthUnit a, LengthUnit b) => a.Equals(b);

    /// <summary>Inequality of the metres-per-unit value.</summary>
    public static bool operator !=(LengthUnit a, LengthUnit b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => MetresPerUnit.ToString("R", CultureInfo.InvariantCulture) + " m/unit";
}

/// <summary>Helpers that move a default tuned in metres to another unit while leaving explicit host values alone.</summary>
internal static class UnitDefaults
{
    // A value still equal to the metre default at the old scale is a default: it follows the unit. Anything else was set by the host and keeps its meaning in world units.
    internal static float Follow(float value, float metreDefault, float oldScale, float newScale) =>
        BitConverter.SingleToInt32Bits(value) == BitConverter.SingleToInt32Bits(metreDefault * oldScale) ? metreDefault * newScale : value;

    internal static double Follow(double value, double metreDefault, float oldScale, float newScale) =>
        BitConverter.DoubleToInt64Bits(value) == BitConverter.DoubleToInt64Bits(metreDefault * oldScale) ? metreDefault * newScale : value;
}
