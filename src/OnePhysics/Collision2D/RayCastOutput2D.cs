using System;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>Result of a ray cast.</summary>
public readonly struct RayCastOutput2D
{
    /// <summary>True when the ray hit the shape within its length.</summary>
    public bool Hit { get; }

    /// <summary>Where along the ray's translation the hit is, from 0 to the maximum fraction.</summary>
    public float Fraction { get; }

    /// <summary>Unit surface normal at the hit, pointing against the ray; zero when the ray starts inside the shape.</summary>
    public Vec2 Normal { get; }

    /// <summary>The hit point.</summary>
    public Vec2 Point { get; }

    /// <summary>Creates a result.</summary>
    public RayCastOutput2D(bool hit, float fraction, Vec2 normal, Vec2 point)
    {
        Hit = hit;
        Fraction = fraction;
        Normal = normal;
        Point = point;
    }
}
