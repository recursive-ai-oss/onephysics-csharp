using System;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

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

    /// <summary>Lane i takes the eight floats at <c>source[offsets[i]..]</c>; field k of every lane comes back together as <c>Rk</c> (a transpose: the rows are loaded whole, not float by float).</summary>
    static abstract Rows8<T> GatherRows(ReadOnlySpan<float> source, ReadOnlySpan<int> offsets);

    static abstract T operator +(T a, T b);

    static abstract T operator -(T a, T b);

    static abstract T operator *(T a, T b);

    static abstract T operator /(T a, T b);

    /// <summary>Square root of every lane (IEEE-exact, so every width gives the scalar answer).</summary>
    static abstract T Sqrt(T a);

    /// <summary>Flips the sign bit of every lane, exactly like the scalar unary minus (so zero keeps its sign rule).</summary>
    static abstract T Negate(T a);

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

    /// <summary>The mask of lanes where either mask is set.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static T Or<T>(T a, T b) where T : struct, ILane<T> => T.Select(a, a, b);

    /// <summary>The mask of lanes where both masks are set.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static T And<T>(T a, T b) where T : struct, ILane<T> => T.Select(a, b, T.Zero);
}

/// <summary>Eight vectors, one per field of a gathered row.</summary>
internal struct Rows8<T> where T : struct, ILane<T>
{
    public T R0, R1, R2, R3, R4, R5, R6, R7;
}

/// <summary>One lane: the scalar twin of the vector kernels.</summary>
internal readonly struct Lanes1 : ILane<Lanes1>
{
    private readonly float _v;

    private Lanes1(float v) => _v = v;

    public static int Count => 1;

    public static Lanes1 Zero => default;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes1 Broadcast(float x) => new(x);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes1 Load(ReadOnlySpan<float> source) => new(source[0]);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Store(Span<float> destination) => destination[0] = _v;

    public static Rows8<Lanes1> GatherRows(ReadOnlySpan<float> source, ReadOnlySpan<int> offsets)
    {
        var r = source.Slice(offsets[0], 8);
        return new Rows8<Lanes1> { R0 = new(r[0]), R1 = new(r[1]), R2 = new(r[2]), R3 = new(r[3]), R4 = new(r[4]), R5 = new(r[5]), R6 = new(r[6]), R7 = new(r[7]) };
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes1 operator +(Lanes1 a, Lanes1 b) => new(a._v + b._v);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes1 operator -(Lanes1 a, Lanes1 b) => new(a._v - b._v);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes1 operator *(Lanes1 a, Lanes1 b) => new(a._v * b._v);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes1 operator /(Lanes1 a, Lanes1 b) => new(a._v / b._v);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes1 Sqrt(Lanes1 a) => new(float.Sqrt(a._v));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes1 Negate(Lanes1 a) => new(-a._v);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes1 GreaterThan(Lanes1 a, Lanes1 b) => new(BitConverter.Int32BitsToSingle(a._v > b._v ? -1 : 0));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes128 Broadcast(float x) => new(Vector128.Create(x));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes128 Load(ReadOnlySpan<float> source) => new(Vector128.Create(source));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Store(Span<float> destination) => _v.CopyTo(destination);

    public static Rows8<Lanes128> GatherRows(ReadOnlySpan<float> source, ReadOnlySpan<int> offsets)
    {
        var a = source.Slice(offsets[0], 8); var b = source.Slice(offsets[1], 8); var c = source.Slice(offsets[2], 8); var d = source.Slice(offsets[3], 8);
        return new Rows8<Lanes128>
        {
            R0 = new(Vector128.Create(a[0], b[0], c[0], d[0])), R1 = new(Vector128.Create(a[1], b[1], c[1], d[1])), R2 = new(Vector128.Create(a[2], b[2], c[2], d[2])), R3 = new(Vector128.Create(a[3], b[3], c[3], d[3])),
            R4 = new(Vector128.Create(a[4], b[4], c[4], d[4])), R5 = new(Vector128.Create(a[5], b[5], c[5], d[5])), R6 = new(Vector128.Create(a[6], b[6], c[6], d[6])), R7 = new(Vector128.Create(a[7], b[7], c[7], d[7])),
        };
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes128 operator +(Lanes128 a, Lanes128 b) => new(Vector128.Add(a._v, b._v));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes128 operator -(Lanes128 a, Lanes128 b) => new(Vector128.Subtract(a._v, b._v));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes128 operator *(Lanes128 a, Lanes128 b) => new(Vector128.Multiply(a._v, b._v));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes128 operator /(Lanes128 a, Lanes128 b) => new(Vector128.Divide(a._v, b._v));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes128 Sqrt(Lanes128 a) => new(Vector128.Sqrt(a._v));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes128 Negate(Lanes128 a) => new(Vector128.Xor(a._v, Vector128.Create(-0f)));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes128 GreaterThan(Lanes128 a, Lanes128 b) => new(Vector128.GreaterThan(a._v, b._v));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes256 Broadcast(float x) => new(Vector256.Create(x));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes256 Load(ReadOnlySpan<float> source) => new(Vector256.Create(source));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Store(Span<float> destination) => _v.CopyTo(destination);

    public static Rows8<Lanes256> GatherRows(ReadOnlySpan<float> source, ReadOnlySpan<int> offsets)
    {
        var a = source.Slice(offsets[0], 8); var b = source.Slice(offsets[1], 8); var c = source.Slice(offsets[2], 8); var d = source.Slice(offsets[3], 8);
        var e = source.Slice(offsets[4], 8); var f = source.Slice(offsets[5], 8); var g = source.Slice(offsets[6], 8); var h = source.Slice(offsets[7], 8);
        if (Avx.IsSupported)
        {
            // The usual 8x8 transpose: interleave pairs of rows, then quads, then swap the 128-bit halves. Pure data movement, so the bits are those of the scalar loads below.
            var r0 = Vector256.Create(a); var r1 = Vector256.Create(b); var r2 = Vector256.Create(c); var r3 = Vector256.Create(d);
            var r4 = Vector256.Create(e); var r5 = Vector256.Create(f); var r6 = Vector256.Create(g); var r7 = Vector256.Create(h);
            var t0 = Avx.UnpackLow(r0, r1); var t1 = Avx.UnpackHigh(r0, r1); var t2 = Avx.UnpackLow(r2, r3); var t3 = Avx.UnpackHigh(r2, r3);
            var t4 = Avx.UnpackLow(r4, r5); var t5 = Avx.UnpackHigh(r4, r5); var t6 = Avx.UnpackLow(r6, r7); var t7 = Avx.UnpackHigh(r6, r7);
            var u0 = Avx.Shuffle(t0, t2, 0x44); var u1 = Avx.Shuffle(t0, t2, 0xEE); var u2 = Avx.Shuffle(t1, t3, 0x44); var u3 = Avx.Shuffle(t1, t3, 0xEE);
            var u4 = Avx.Shuffle(t4, t6, 0x44); var u5 = Avx.Shuffle(t4, t6, 0xEE); var u6 = Avx.Shuffle(t5, t7, 0x44); var u7 = Avx.Shuffle(t5, t7, 0xEE);
            return new Rows8<Lanes256>
            {
                R0 = new(Avx.Permute2x128(u0, u4, 0x20)), R1 = new(Avx.Permute2x128(u1, u5, 0x20)), R2 = new(Avx.Permute2x128(u2, u6, 0x20)), R3 = new(Avx.Permute2x128(u3, u7, 0x20)),
                R4 = new(Avx.Permute2x128(u0, u4, 0x31)), R5 = new(Avx.Permute2x128(u1, u5, 0x31)), R6 = new(Avx.Permute2x128(u2, u6, 0x31)), R7 = new(Avx.Permute2x128(u3, u7, 0x31)),
            };
        }

        return new Rows8<Lanes256>
        {
            R0 = new(Vector256.Create(a[0], b[0], c[0], d[0], e[0], f[0], g[0], h[0])), R1 = new(Vector256.Create(a[1], b[1], c[1], d[1], e[1], f[1], g[1], h[1])),
            R2 = new(Vector256.Create(a[2], b[2], c[2], d[2], e[2], f[2], g[2], h[2])), R3 = new(Vector256.Create(a[3], b[3], c[3], d[3], e[3], f[3], g[3], h[3])),
            R4 = new(Vector256.Create(a[4], b[4], c[4], d[4], e[4], f[4], g[4], h[4])), R5 = new(Vector256.Create(a[5], b[5], c[5], d[5], e[5], f[5], g[5], h[5])),
            R6 = new(Vector256.Create(a[6], b[6], c[6], d[6], e[6], f[6], g[6], h[6])), R7 = new(Vector256.Create(a[7], b[7], c[7], d[7], e[7], f[7], g[7], h[7])),
        };
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes256 operator +(Lanes256 a, Lanes256 b) => new(Vector256.Add(a._v, b._v));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes256 operator -(Lanes256 a, Lanes256 b) => new(Vector256.Subtract(a._v, b._v));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes256 operator *(Lanes256 a, Lanes256 b) => new(Vector256.Multiply(a._v, b._v));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes256 operator /(Lanes256 a, Lanes256 b) => new(Vector256.Divide(a._v, b._v));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes256 Sqrt(Lanes256 a) => new(Vector256.Sqrt(a._v));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes256 Negate(Lanes256 a) => new(Vector256.Xor(a._v, Vector256.Create(-0f)));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes256 GreaterThan(Lanes256 a, Lanes256 b) => new(Vector256.GreaterThan(a._v, b._v));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes256 Select(Lanes256 mask, Lanes256 whenSet, Lanes256 whenClear) =>
        new(Vector256.ConditionalSelect(mask._v, whenSet._v, whenClear._v));
}
