using System;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>An axis-aligned bounding box.</summary>
public readonly struct Aabb2
{
    /// <summary>Smallest corner.</summary>
    public Vec2 Lower { get; }

    /// <summary>Largest corner.</summary>
    public Vec2 Upper { get; }

    /// <summary>Creates a box from its corners.</summary>
    public Aabb2(Vec2 lower, Vec2 upper) { Lower = lower; Upper = upper; }

    /// <summary>True when the boxes share at least a point.</summary>
    public static bool Overlaps(Aabb2 a, Aabb2 b) => a.Lower.X <= b.Upper.X && b.Lower.X <= a.Upper.X && a.Lower.Y <= b.Upper.Y && b.Lower.Y <= a.Upper.Y;
}
