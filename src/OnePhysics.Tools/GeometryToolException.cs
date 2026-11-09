using System;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>What was wrong with an input to one of the geometry tools.</summary>
public enum GeometryToolError
{
    /// <summary>An argument is null-like or has no elements where some are needed (an empty mesh, a polygon with fewer than three points).</summary>
    Empty,

    /// <summary>A coordinate is NaN or infinite.</summary>
    NonFinite,

    /// <summary>A coordinate or a size is outside the supported range.</summary>
    OutOfRange,

    /// <summary>A triangle index points outside the vertex array, or the index count is not a multiple of three.</summary>
    BadIndex,

    /// <summary>The input has no area or volume (all points collinear or coplanar, or every triangle degenerate).</summary>
    Degenerate,

    /// <summary>The mesh has boundary edges (holes), so it does not enclose a volume.</summary>
    OpenMesh,

    /// <summary>An edge is shared by more than two triangles, or by two triangles that disagree on direction.</summary>
    NonManifold,

    /// <summary>The mesh encloses a negative volume: its triangles wind clockwise seen from outside.</summary>
    InvertedWinding,

    /// <summary>A polygon crosses itself, or a hole crosses another boundary or lies outside its outline.</summary>
    SelfIntersecting,

    /// <summary>An option has an invalid value.</summary>
    InvalidOptions,
}

/// <summary>Thrown by the geometry tools when an input cannot be processed. The message names the input and what was expected; <see cref="ArgumentException.ParamName"/> is the offending parameter.</summary>
public sealed class GeometryToolException : ArgumentException
{
    /// <summary>Creates the exception.</summary>
    /// <param name="error">The kind of failure.</param>
    /// <param name="message">What was wrong and where.</param>
    /// <param name="paramName">The offending parameter.</param>
    public GeometryToolException(GeometryToolError error, string message, string? paramName = null)
        : base(message, paramName)
    {
        Error = error;
    }

    /// <summary>The kind of failure.</summary>
    public GeometryToolError Error { get; }
}
