using System;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;

using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>Why a convex hull could not be built.</summary>
public enum HullBuildError
{
    /// <summary>Fewer than four points were given.</summary>
    TooFewPoints,

    /// <summary>More than <see cref="ConvexHull3D.MaxInputPoints"/> points were given.</summary>
    TooManyPoints,

    /// <summary>A coordinate is NaN or infinite.</summary>
    NonFinite,

    /// <summary>A coordinate's magnitude exceeds <see cref="ConvexHull3D.MaxCoordinate"/>.</summary>
    OutOfRange,

    /// <summary>The points do not span a volume: they are coincident, collinear or coplanar within the tolerance.</summary>
    Degenerate,

    /// <summary>The options are invalid (for example a vertex limit below four).</summary>
    InvalidOptions,
}
