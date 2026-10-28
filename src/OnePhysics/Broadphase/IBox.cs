using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>
/// An axis-aligned box in world space, in double precision. The tree algorithms are written once over this interface and the JIT specialises them per box type,
/// so the 2D and 3D trees run box-specific code with no virtual calls.
/// </summary>
/// <typeparam name="TSelf">The implementing box type.</typeparam>
[Experimental("OPHYS0001")]
public interface IBox<TSelf> where TSelf : struct, IBox<TSelf>
{
    /// <summary>Number of axes: 2 or 3.</summary>
    static abstract int Axes { get; }

    /// <summary>The smallest box containing both.</summary>
    static abstract TSelf Union(in TSelf a, in TSelf b);

    /// <summary>True when the boxes overlap or touch.</summary>
    static abstract bool Overlaps(in TSelf a, in TSelf b);

    /// <summary>True when <paramref name="outer"/> contains <paramref name="inner"/> (boundaries included).</summary>
    static abstract bool Contains(in TSelf outer, in TSelf inner);

    /// <summary>Reads a box written by <see cref="Write"/>; the values must be finite.</summary>
    static abstract TSelf Read(StateReader reader);

    /// <summary>The surface-area-heuristic measure: perimeter in 2D, surface area in 3D.</summary>
    double Area { get; }

    /// <summary>Lower bound on axis <paramref name="axis"/>.</summary>
    double Min(int axis);

    /// <summary>Upper bound on axis <paramref name="axis"/>.</summary>
    double Max(int axis);

    /// <summary>
    /// The box grown by <paramref name="margin"/> on every side, then stretched by <paramref name="multiplier"/> times <paramref name="displacement"/> in the direction of motion
    /// (Catto, GDC 2019: fat leaf bounds predict where a moving body will be, so most steps need no tree update).
    /// </summary>
    TSelf Fatten(double margin, ReadOnlySpan<double> displacement, double multiplier);

    /// <summary>True when every bound is finite, min is not above max and no bound is larger than <see cref="BoxLimits.MaxCoordinate"/> in magnitude. <paramref name="reason"/> says what was wrong.</summary>
    bool IsValid(out string reason);

    /// <summary>Writes the bounds in axis order, mins then maxes... as min x, min y, (min z), max x, max y, (max z), each a little-endian double.</summary>
    void Write(StateWriter writer);
}

/// <summary>Limits on box coordinates.</summary>
[Experimental("OPHYS0001")]
public static class BoxLimits
{
    /// <summary>
    /// Largest magnitude a box bound may have: 2^22. Body positions are exact within ±2^21; a shape's extent may carry its box past that, and fattening adds a little more.
    /// Beyond 2^22 the host should shift the world origin.
    /// </summary>
    public const double MaxCoordinate = 4194304.0;

    internal static bool CheckAxis(double min, double max, string axis, out string reason)
    {
        if (!double.IsFinite(min) || !double.IsFinite(max))
        {
            reason = "box bound on the " + axis + " axis is not finite (min " + min.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ", max " + max.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ").";
            return false;
        }

        if (min > max)
        {
            reason = "box on the " + axis + " axis has min " + min.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + " above max " + max.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ".";
            return false;
        }

        if (Math.Abs(min) > MaxCoordinate || Math.Abs(max) > MaxCoordinate)
        {
            reason = "box bound on the " + axis + " axis (" + min.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + " to " + max.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ") is beyond the limit of ±" + MaxCoordinate.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ".";
            return false;
        }

        reason = string.Empty;
        return true;
    }
}
