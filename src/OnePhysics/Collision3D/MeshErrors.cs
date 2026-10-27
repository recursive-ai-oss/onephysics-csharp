using System;

namespace RecursiveAI.OnePhysics.Collision3D;

/// <summary>Why a triangle mesh or height field could not be built.</summary>
public enum MeshBuildError
{
    /// <summary>The vertex array is empty.</summary>
    NoVertices,

    /// <summary>The index array is empty, or every triangle was degenerate and dropped.</summary>
    NoTriangles,

    /// <summary>The index count is not a multiple of three.</summary>
    InvalidIndexCount,

    /// <summary>More vertices or triangles than the supported maximum.</summary>
    TooLarge,

    /// <summary>A coordinate or height is NaN or infinite.</summary>
    NonFinite,

    /// <summary>A coordinate, height, scale or extent is outside the accepted range.</summary>
    OutOfRange,

    /// <summary>A triangle index does not name a vertex.</summary>
    IndexOutOfRange,

    /// <summary>A height-field grid, scale or hole mask has the wrong size or shape.</summary>
    InvalidGrid,

    /// <summary>The build options are invalid.</summary>
    InvalidOptions,
}

/// <summary>Thrown when a triangle mesh or height field is built from input it cannot accept. The message names the input and what was expected.</summary>
public sealed class MeshBuildException : ArgumentException
{
    /// <summary>Creates the exception.</summary>
    /// <param name="error">The kind of failure.</param>
    /// <param name="message">What was wrong and what was expected.</param>
    /// <param name="paramName">The offending parameter.</param>
    public MeshBuildException(MeshBuildError error, string message, string? paramName = null)
        : base(message, paramName)
    {
        Error = error;
    }

    /// <summary>The kind of failure.</summary>
    public MeshBuildError Error { get; }
}

/// <summary>Thrown when a mass property is requested from a shape that can only be static, such as a triangle mesh or a height field.</summary>
public sealed class StaticOnlyShapeException : NotSupportedException
{
    /// <summary>Creates the exception.</summary>
    /// <param name="message">What was asked and why it is not supported.</param>
    public StaticOnlyShapeException(string message)
        : base(message)
    {
    }
}
