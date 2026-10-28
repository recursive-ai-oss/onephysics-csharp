using System;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>A rotation stored as cosine and sine of its angle, so no trigonometry is ever needed. A valid rotation has unit length.</summary>
public readonly struct Rot2
{
    /// <summary>Cosine of the angle.</summary>
    public float C { get; }

    /// <summary>Sine of the angle.</summary>
    public float S { get; }

    /// <summary>Creates a rotation from a cosine and a sine; the caller guarantees (or checks) that they form a unit vector.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Rot2(float c, float s) { C = c; S = s; }

    /// <summary>The identity rotation.</summary>
    public static Rot2 Identity => new(1f, 0f);

    /// <summary>The rotation of the vector (<paramref name="c"/>, <paramref name="s"/>) normalised to unit length, or the identity when it has no usable length.</summary>
    public static Rot2 FromVector(float c, float s)
    {
        var v = new Vec2(c, s).Normalized();
        return v.X == 0f && v.Y == 0f ? Identity : new Rot2(v.X, v.Y);
    }

    /// <summary>Rotates a vector.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vec2 Rotate(Vec2 v) => new(C * v.X - S * v.Y, S * v.X + C * v.Y);

    /// <summary>Rotates a vector by the inverse rotation.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vec2 InverseRotate(Vec2 v) => new(C * v.X + S * v.Y, -S * v.X + C * v.Y);

    /// <summary>The composition <c>a ∘ b</c>: apply <paramref name="b"/>, then <paramref name="a"/>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Rot2 Mul(Rot2 a, Rot2 b) => new(a.C * b.C - a.S * b.S, a.S * b.C + a.C * b.S);

    /// <summary>The composition <c>a⁻¹ ∘ b</c>: the rotation of <paramref name="b"/> as seen from <paramref name="a"/>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Rot2 InvMul(Rot2 a, Rot2 b) => new(a.C * b.C + a.S * b.S, a.C * b.S - a.S * b.C);

    /// <summary>The inverse rotation.</summary>
    public Rot2 Inverse => new(C, -S);
}
