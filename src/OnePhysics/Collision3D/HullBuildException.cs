using System;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;

using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>Thrown by <see cref="ConvexHull3D.Build"/> when the input cannot make a convex hull. The message names the input and what was expected.</summary>
public sealed class HullBuildException : ArgumentException
{
    /// <summary>Creates the exception.</summary>
    /// <param name="error">The kind of failure.</param>
    /// <param name="message">What was wrong and what was expected.</param>
    /// <param name="paramName">The offending parameter.</param>
    public HullBuildException(HullBuildError error, string message, string? paramName = null)
        : base(message, paramName)
    {
        Error = error;
    }

    /// <summary>The kind of failure.</summary>
    public HullBuildError Error { get; }
}
