using System;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>Tuning constants of the 2D collision code. Lengths are in the caller's unit (metres by convention).</summary>
[Experimental("OPHYS0001")]
public static class Constants2D
{
    /// <summary>Largest vertex count of a convex polygon core.</summary>
    public const int MaxPolygonVertices = 8;

    /// <summary>Length tolerance: how far shapes may sink into each other before they are considered to be touching, and the gap a time-of-impact search stops at. 0.005.</summary>
    public const float LinearSlop = 0.005f;

    /// <summary>Default distance within which a manifold keeps points that are not yet touching (speculative contacts): four times <see cref="LinearSlop"/>. 0.02.</summary>
    public const float SpeculativeDistance = 4f * LinearSlop;

    /// <summary>Largest absolute coordinate a shape vertex or radius may have (2^20), so float32 arithmetic on shape-local data stays well inside its range.</summary>
    public const float MaxShapeCoordinate = 1048576f;

    /// <summary>Smallest extent (largest distance between two vertices) a polygon, and smallest length a segment, may have.</summary>
    public const float MinShapeExtent = 1e-5f;
}
