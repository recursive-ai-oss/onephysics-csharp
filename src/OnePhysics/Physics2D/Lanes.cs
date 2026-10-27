using System;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// A group of float32 lanes. Every implementation computes each lane with the same IEEE-754 operations in the same order, so a kernel written once
/// against this interface gives the same bits at every width. Masks are all-ones or all-zeros per lane. Minimum, maximum and negation are built from
/// compare and select so that signed zeros and NaNs follow one rule everywhere instead of whatever each vector type does. There are no reductions
/// across lanes: the lanes of a batch are different constraints on different bodies.
/// </summary>
internal interface ILane<T> where T : struct, ILane<T>
{
    static abstract int Count { get; }

    static abstract T Zero { get; }

    static abstract T Broadcast(float x);

    static abstract T Load(ReadOnlySpan<float> source);

    void Store(Span<float> destination);

    static abstract T operator +(T a, T b);

    static abstract T operator -(T a, T b);

    static abstract T operator *(T a, T b);

    static abstract T GreaterThan(T a, T b);

    static abstract T Select(T mask, T whenSet, T whenClear);
}

internal static class Lane
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static T Max<T>(T a, T b) where T : struct, ILane<T> => T.Select(T.GreaterThan(a, b), a, b);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static T Min<T>(T a, T b) where T : struct, ILane<T> => T.Select(T.GreaterThan(b, a), a, b);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static T Neg<T>(T a) where T : struct, ILane<T> => T.Zero - a;

    /// <summary>The mask of lanes where both masks are set.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static T And<T>(T a, T b) where T : struct, ILane<T> => T.Select(a, b, T.Zero);
}

/// <summary>One lane: the scalar twin of the vector kernels.</summary>
internal readonly struct Lanes1 : ILane<Lanes1>
{
    private readonly float _v;

    private Lanes1(float v) => _v = v;

    public static int Count => 1;

    public static Lanes1 Zero => default;

    public static Lanes1 Broadcast(float x) => new(x);

    public static Lanes1 Load(ReadOnlySpan<float> source) => new(source[0]);

    public void Store(Span<float> destination) => destination[0] = _v;

    public static Lanes1 operator +(Lanes1 a, Lanes1 b) => new(a._v + b._v);

    public static Lanes1 operator -(Lanes1 a, Lanes1 b) => new(a._v - b._v);

    public static Lanes1 operator *(Lanes1 a, Lanes1 b) => new(a._v * b._v);

    public static Lanes1 GreaterThan(Lanes1 a, Lanes1 b) => new(BitConverter.Int32BitsToSingle(a._v > b._v ? -1 : 0));

    public static Lanes1 Select(Lanes1 mask, Lanes1 whenSet, Lanes1 whenClear) =>
        BitConverter.SingleToInt32Bits(mask._v) != 0 ? whenSet : whenClear;
}

/// <summary>Four lanes in a <see cref="Vector128{T}"/>.</summary>
internal readonly struct Lanes128 : ILane<Lanes128>
{
    private readonly Vector128<float> _v;

    private Lanes128(Vector128<float> v) => _v = v;

    public static int Count => 4;

    public static Lanes128 Zero => default;

    public static Lanes128 Broadcast(float x) => new(Vector128.Create(x));

    public static Lanes128 Load(ReadOnlySpan<float> source) => new(Vector128.Create(source));

    public void Store(Span<float> destination) => _v.CopyTo(destination);

    public static Lanes128 operator +(Lanes128 a, Lanes128 b) => new(Vector128.Add(a._v, b._v));

    public static Lanes128 operator -(Lanes128 a, Lanes128 b) => new(Vector128.Subtract(a._v, b._v));

    public static Lanes128 operator *(Lanes128 a, Lanes128 b) => new(Vector128.Multiply(a._v, b._v));

    public static Lanes128 GreaterThan(Lanes128 a, Lanes128 b) => new(Vector128.GreaterThan(a._v, b._v));

    public static Lanes128 Select(Lanes128 mask, Lanes128 whenSet, Lanes128 whenClear) =>
        new(Vector128.ConditionalSelect(mask._v, whenSet._v, whenClear._v));
}

/// <summary>Eight lanes in a <see cref="Vector256{T}"/>: one whole batch of the constraint coloring.</summary>
internal readonly struct Lanes256 : ILane<Lanes256>
{
    private readonly Vector256<float> _v;

    private Lanes256(Vector256<float> v) => _v = v;

    public static int Count => 8;

    public static Lanes256 Zero => default;

    public static Lanes256 Broadcast(float x) => new(Vector256.Create(x));

    public static Lanes256 Load(ReadOnlySpan<float> source) => new(Vector256.Create(source));

    public void Store(Span<float> destination) => _v.CopyTo(destination);

    public static Lanes256 operator +(Lanes256 a, Lanes256 b) => new(Vector256.Add(a._v, b._v));

    public static Lanes256 operator -(Lanes256 a, Lanes256 b) => new(Vector256.Subtract(a._v, b._v));

    public static Lanes256 operator *(Lanes256 a, Lanes256 b) => new(Vector256.Multiply(a._v, b._v));

    public static Lanes256 GreaterThan(Lanes256 a, Lanes256 b) => new(Vector256.GreaterThan(a._v, b._v));

    public static Lanes256 Select(Lanes256 mask, Lanes256 whenSet, Lanes256 whenClear) =>
        new(Vector256.ConditionalSelect(mask._v, whenSet._v, whenClear._v));
}
