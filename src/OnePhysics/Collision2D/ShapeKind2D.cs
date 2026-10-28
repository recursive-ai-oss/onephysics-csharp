using System;
using System.Runtime.Intrinsics;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using System.Runtime.CompilerServices;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>The kind of a <see cref="Shape2D"/>.</summary>
public enum ShapeKind2D : byte
{
    /// <summary>A disc.</summary>
    Circle = 1,

    /// <summary>Two discs joined by a rectangle: a segment with a radius.</summary>
    Capsule = 2,

    /// <summary>A line segment with no thickness; collides from both sides.</summary>
    Segment = 3,

    /// <summary>A convex polygon, optionally rounded by a radius.</summary>
    Polygon = 4,

    /// <summary>A segment of a chain, with the neighbouring vertices needed to avoid snagging on internal edges.</summary>
    ChainSegment = 5,
}
