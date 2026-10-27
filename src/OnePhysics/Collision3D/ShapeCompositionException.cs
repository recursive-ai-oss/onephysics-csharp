using System;

namespace RecursiveAI.OnePhysics.Collision3D;

/// <summary>What was wrong with the input to a shape decorator or a compound.</summary>
public enum ShapeCompositionError
{
    /// <summary>A scale component is NaN, infinite, zero or negative.</summary>
    InvalidScale,

    /// <summary>The scale is valid but the shape cannot be scaled that way without ceasing to be the same kind of shape (a sphere with unequal scale, a rounded box with non-uniform scale).</summary>
    UnsupportedScale,

    /// <summary>A local transform has a NaN or infinite component, a rotation that is not a unit quaternion, or a translation beyond the limit.</summary>
    InvalidTransform,

    /// <summary>A child shape is the zero-size <see cref="ConvexShape3D.Point"/>.</summary>
    InvalidShape,

    /// <summary>A compound was built with no children, or asked for something that needs at least one.</summary>
    EmptyCompound,

    /// <summary>The compound would hold more than <see cref="CompoundShape3D.MaxChildren"/> children.</summary>
    TooManyChildren,

    /// <summary>A child's bounds, with its transform applied, leave the range the broadphase trees accept.</summary>
    BoundsTooLarge,

    /// <summary>A child handle or id does not name a live child of this compound.</summary>
    UnknownChild,

    /// <summary>The scaled shape could not be built (a dimension left its allowed range, or the scaled hull failed).</summary>
    ShapeBuildFailed,

    /// <summary>A query argument is NaN, infinite or otherwise unusable.</summary>
    InvalidQuery,
}

/// <summary>
/// Raised when a decorator or compound is given input it cannot represent. The message names the input (child index, parameter) and what was expected.
/// It derives from <see cref="ArgumentException"/>.
/// </summary>
public sealed class ShapeCompositionException : ArgumentException
{
    /// <summary>Creates the exception.</summary>
    public ShapeCompositionException(ShapeCompositionError error, string message, string? paramName = null, Exception? inner = null)
        : base(message, paramName, inner)
    {
        Error = error;
    }

    /// <summary>The kind of problem.</summary>
    public ShapeCompositionError Error { get; }
}
