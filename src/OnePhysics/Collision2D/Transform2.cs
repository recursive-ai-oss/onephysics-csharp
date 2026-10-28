using System;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

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
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Transform2 InvMul(Transform2 a, Transform2 b) => new(a.Q.InverseRotate(b.P - a.P), Rot2.InvMul(a.Q, b.Q));

    /// <summary>True when the translation is finite and the rotation is finite.</summary>
    public bool IsFinite => float.IsFinite(P.X + P.Y + Q.C + Q.S);
}
