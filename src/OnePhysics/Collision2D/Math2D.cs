using System;
using System.Globalization;
using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePhysics.Collision2D;

/// <summary>Tuning constants of the 2D collision code. Lengths are in the caller's unit (metres by convention).</summary>
public static class Constants2D
{
    /// <summary>Largest vertex count of a convex polygon core.</summary>
    public const int MaxPolygonVertices = 8;

    /// <summary>Length tolerance: how far shapes may sink into each other before they are considered to be touching, and the gap a time-of-impact search stops at. 0.005.</summary>
    public const float LinearSlop = 0.005f;

    /// <summary>Default distance within which a manifold keeps points that are not yet touching (speculative contacts): four times <see cref="LinearSlop"/>. 0.02.</summary>
    public const float SpeculativeDistance = 4f * LinearSlop;

    /// <summary>Largest absolute coordinate a shape vertex or radius may have (2^20), so float32 arithmetic on shape-local data stays well inside its range.</summary>
    public const float MaxShapeCoordinate = 1048576f;

    /// <summary>Smallest extent (largest distance between two vertices) a polygon, and smallest length a segment, may have.</summary>
    public const float MinShapeExtent = 1e-5f;
}

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

/// <summary>A rigid transform: rotate, then translate.</summary>
public readonly struct Transform2
{
    /// <summary>Translation.</summary>
    public Vec2 P { get; }

    /// <summary>Rotation.</summary>
    public Rot2 Q { get; }

    /// <summary>Creates a transform.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Transform2(Vec2 p, Rot2 q) { P = p; Q = q; }

    /// <summary>The identity transform.</summary>
    public static Transform2 Identity => new(default, Rot2.Identity);

    /// <summary>Maps a point from the local frame to the parent frame.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vec2 Apply(Vec2 p) => Q.Rotate(p) + P;

    /// <summary>Maps a point from the parent frame to the local frame.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vec2 ApplyInverse(Vec2 p) => Q.InverseRotate(p - P);

    /// <summary>The pose of <paramref name="b"/> in the frame of <paramref name="a"/> (<c>a⁻¹ ∘ b</c>). The caller forms the translation difference of two body positions in double first, then converts to float32, so large worlds keep their precision (the coordinate rule).</summary>
    public static Transform2 InvMul(Transform2 a, Transform2 b) => new(a.Q.InverseRotate(b.P - a.P), Rot2.InvMul(a.Q, b.Q));

    /// <summary>True when the translation is finite and the rotation is finite.</summary>
    public bool IsFinite => float.IsFinite(P.X + P.Y + Q.C + Q.S);
}

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
