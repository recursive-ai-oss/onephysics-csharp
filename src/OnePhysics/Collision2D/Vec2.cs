using System;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>A 2D vector of two float32 components. All operations use only IEEE-exact arithmetic, so results are identical on every platform.</summary>
public readonly struct Vec2
{
    /// <summary>The x component.</summary>
    public float X { get; }

    /// <summary>The y component.</summary>
    public float Y { get; }

    /// <summary>Creates a vector.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vec2(float x, float y) { X = x; Y = y; }

    /// <summary>The zero vector.</summary>
    public static Vec2 Zero => default;

    /// <summary>Component-wise sum.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vec2 operator +(Vec2 a, Vec2 b) => new(a.X + b.X, a.Y + b.Y);

    /// <summary>Component-wise difference.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vec2 operator -(Vec2 a, Vec2 b) => new(a.X - b.X, a.Y - b.Y);

    /// <summary>Negation.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vec2 operator -(Vec2 a) => new(-a.X, -a.Y);

    /// <summary>Scalar product of a vector and a number.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vec2 operator *(float s, Vec2 a) => new(s * a.X, s * a.Y);

    /// <summary>Scalar product of a vector and a number.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vec2 operator *(Vec2 a, float s) => new(s * a.X, s * a.Y);

    /// <summary>Dot product.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Dot(Vec2 a, Vec2 b) => a.X * b.X + a.Y * b.Y;

    /// <summary>The z component of the 3D cross product: positive when <paramref name="b"/> is counter-clockwise from <paramref name="a"/>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Cross(Vec2 a, Vec2 b) => a.X * b.Y - a.Y * b.X;

    /// <summary>The vector turned a quarter turn clockwise: (y, -x). For a counter-clockwise edge this is its outward normal direction.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vec2 RightPerp(Vec2 a) => new(a.Y, -a.X);

    /// <summary>The vector turned a quarter turn counter-clockwise: (-y, x).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vec2 LeftPerp(Vec2 a) => new(-a.Y, a.X);

    /// <summary>Squared length.</summary>
    public float LengthSquared => X * X + Y * Y;

    /// <summary>Length (one correctly rounded square root of the squared length).</summary>
    public float Length => float.Sqrt(X * X + Y * Y);

    /// <summary>The unit vector in this direction, or the zero vector when the length is zero, subnormal or not finite.</summary>
    public Vec2 Normalized()
    {
        float l = Length;
        return l >= 1e-20f && l <= float.MaxValue ? new Vec2(X / l, Y / l) : default;
    }

    /// <summary>True when both components are finite.</summary>
    public bool IsFinite => float.IsFinite(X + Y);

    /// <inheritdoc/>
    public override string ToString() => "(" + X.ToString("R", CultureInfo.InvariantCulture) + ", " + Y.ToString("R", CultureInfo.InvariantCulture) + ")";
}
