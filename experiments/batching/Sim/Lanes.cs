using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;

namespace RecursiveAI.OnePhysics.Experiments.Batching;

/// <summary>
/// A group of float32 lanes. Every implementation computes each lane with the same IEEE-754 operations in the same
/// order, so a kernel written once against this interface must give the same bits at every width. Masks are
/// all-ones or all-zeros per lane. Min, max and negation are built from compare and select so that signed zeros and
/// NaNs follow one rule everywhere instead of whatever each vector type does.
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
    static abstract T operator /(T a, T b);
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
}

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
    public static Lanes1 operator /(Lanes1 a, Lanes1 b) => new(a._v / b._v);
    public static Lanes1 GreaterThan(Lanes1 a, Lanes1 b) => new(BitConverter.Int32BitsToSingle(a._v > b._v ? -1 : 0));
    public static Lanes1 Select(Lanes1 mask, Lanes1 whenSet, Lanes1 whenClear) =>
        BitConverter.SingleToInt32Bits(mask._v) != 0 ? whenSet : whenClear;
}

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
    public static Lanes128 operator /(Lanes128 a, Lanes128 b) => new(Vector128.Divide(a._v, b._v));
    public static Lanes128 GreaterThan(Lanes128 a, Lanes128 b) => new(Vector128.GreaterThan(a._v, b._v));
    public static Lanes128 Select(Lanes128 mask, Lanes128 whenSet, Lanes128 whenClear) =>
        new(Vector128.ConditionalSelect(mask._v, whenSet._v, whenClear._v));
}

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
    public static Lanes256 operator /(Lanes256 a, Lanes256 b) => new(Vector256.Divide(a._v, b._v));
    public static Lanes256 GreaterThan(Lanes256 a, Lanes256 b) => new(Vector256.GreaterThan(a._v, b._v));
    public static Lanes256 Select(Lanes256 mask, Lanes256 whenSet, Lanes256 whenClear) =>
        new(Vector256.ConditionalSelect(mask._v, whenSet._v, whenClear._v));
}

internal readonly struct Lanes512 : ILane<Lanes512>
{
    private readonly Vector512<float> _v;
    private Lanes512(Vector512<float> v) => _v = v;
    public static int Count => 16;
    public static Lanes512 Zero => default;
    public static Lanes512 Broadcast(float x) => new(Vector512.Create(x));
    public static Lanes512 Load(ReadOnlySpan<float> source) => new(Vector512.Create(source));
    public void Store(Span<float> destination) => _v.CopyTo(destination);
    public static Lanes512 operator +(Lanes512 a, Lanes512 b) => new(Vector512.Add(a._v, b._v));
    public static Lanes512 operator -(Lanes512 a, Lanes512 b) => new(Vector512.Subtract(a._v, b._v));
    public static Lanes512 operator *(Lanes512 a, Lanes512 b) => new(Vector512.Multiply(a._v, b._v));
    public static Lanes512 operator /(Lanes512 a, Lanes512 b) => new(Vector512.Divide(a._v, b._v));
    public static Lanes512 GreaterThan(Lanes512 a, Lanes512 b) => new(Vector512.GreaterThan(a._v, b._v));
    public static Lanes512 Select(Lanes512 mask, Lanes512 whenSet, Lanes512 whenClear) =>
        new(Vector512.ConditionalSelect(mask._v, whenSet._v, whenClear._v));
}

/// <summary>The platform's preferred width: 4, 8 or 16 lanes depending on the machine. Used only to test that it makes no difference.</summary>
internal readonly struct LanesT : ILane<LanesT>
{
    private readonly Vector<float> _v;
    private LanesT(Vector<float> v) => _v = v;
    public static int Count => Vector<float>.Count;
    public static LanesT Zero => default;
    public static LanesT Broadcast(float x) => new(new Vector<float>(x));
    public static LanesT Load(ReadOnlySpan<float> source) => new(new Vector<float>(source));
    public void Store(Span<float> destination) => _v.CopyTo(destination);
    public static LanesT operator +(LanesT a, LanesT b) => new(Vector.Add(a._v, b._v));
    public static LanesT operator -(LanesT a, LanesT b) => new(Vector.Subtract(a._v, b._v));
    public static LanesT operator *(LanesT a, LanesT b) => new(Vector.Multiply(a._v, b._v));
    public static LanesT operator /(LanesT a, LanesT b) => new(Vector.Divide(a._v, b._v));
    public static LanesT GreaterThan(LanesT a, LanesT b) => new(Vector.As<int, float>(Vector.GreaterThan(a._v, b._v)));
    public static LanesT Select(LanesT mask, LanesT whenSet, LanesT whenClear) =>
        new(Vector.ConditionalSelect(Vector.As<float, int>(mask._v), whenSet._v, whenClear._v));
}

/// <summary>The kernel widths under test.</summary>
internal enum Width
{
    Scalar,
    V128,
    V256,
    V512,
    VectorT,
}
