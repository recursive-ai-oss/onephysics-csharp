using System;
using System.Runtime.Intrinsics;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using System.Runtime.CompilerServices;

using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>Why a shape could not be built.</summary>
public enum ShapeError
{
    /// <summary>No error; the neutral zero value of the enumeration.</summary>
    None = 0,

    /// <summary>A coordinate or radius is NaN or infinite.</summary>
    NotFinite,

    /// <summary>A coordinate or radius is outside the allowed range (see <see cref="Constants2D.MaxShapeCoordinate"/>), or a size is not positive.</summary>
    OutOfRange,

    /// <summary>Fewer points than the shape needs.</summary>
    TooFewPoints,

    /// <summary>More input points than the builder accepts (65536).</summary>
    TooManyPoints,

    /// <summary>The points do not span an area: all within tolerance of one point or one line, or an edge is too short.</summary>
    Degenerate,

    /// <summary>The convex hull has more than <see cref="Constants2D.MaxPolygonVertices"/> vertices.</summary>
    TooManyVertices,
}
