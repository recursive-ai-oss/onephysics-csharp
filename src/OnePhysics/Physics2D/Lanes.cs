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

    /// <summary>Loads <see cref="Count"/> floats starting at <paramref name="offset"/> with no bounds check (see <see cref="Mem"/>).</summary>
    static abstract T LoadAt(ReadOnlySpan<float> source, int offset);

    /// <summary>Stores <see cref="Count"/> floats starting at <paramref name="offset"/> with no bounds check (see <see cref="Mem"/>).</summary>
    void StoreAt(Span<float> destination, int offset);

    /// <summary>Lane i takes the eight floats at <c>source[offsets[i]..]</c>; field k of every lane comes back together as <c>Rk</c> (a transpose: the rows are loaded whole, not float by float).</summary>
    static abstract Rows8<T> GatherRows(ReadOnlySpan<float> source, ReadOnlySpan<int> offsets);

    /// <summary>
    /// Lane i writes its three values to <c>destination[offsets[i]..+3]</c>, unless the offset is negative (a body that keeps its velocity). Slot 3 of a row is padding and may be overwritten with zero.
    /// The offsets of different lanes name different rows (the lanes of a batch touch different bodies).
    /// </summary>
    static abstract void Scatter3(Span<float> destination, ReadOnlySpan<int> offsets, T a, T b, T c);

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
internal static class ScatterRow
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Put(Span<float> destination, int offset, float a, float b, float c)
    {
        if (offset < 0) { return; }
        Mem.Range(destination, offset, 3);
        Mem.At(destination, offset) = a; Mem.At(destination, offset + 1) = b; Mem.At(destination, offset + 2) = c;
    }
}

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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes1 LoadAt(ReadOnlySpan<float> source, int offset)
    {
        Mem.Range(source, offset, 1);
        return new(Mem.At(source, offset));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void StoreAt(Span<float> destination, int offset)
    {
        Mem.Range(destination, offset, 1);
        Mem.At(destination, offset) = _v;
    }

    public static void Scatter3(Span<float> destination, ReadOnlySpan<int> offsets, Lanes1 a, Lanes1 b, Lanes1 c) => ScatterRow.Put(destination, offsets[0], a._v, b._v, c._v);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes128 LoadAt(ReadOnlySpan<float> source, int offset)
    {
        Mem.Range(source, offset, 4);
        return new(Vector128.LoadUnsafe(in Mem.At(source, offset)));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void StoreAt(Span<float> destination, int offset)
    {
        Mem.Range(destination, offset, 4);
        _v.StoreUnsafe(ref Mem.At(destination, offset));
    }

    public static void Scatter3(Span<float> destination, ReadOnlySpan<int> offsets, Lanes128 a, Lanes128 b, Lanes128 c)
    {
        ScatterRow.Put(destination, offsets[0], a._v.GetElement(0), b._v.GetElement(0), c._v.GetElement(0));
        ScatterRow.Put(destination, offsets[1], a._v.GetElement(1), b._v.GetElement(1), c._v.GetElement(1));
        ScatterRow.Put(destination, offsets[2], a._v.GetElement(2), b._v.GetElement(2), c._v.GetElement(2));
        ScatterRow.Put(destination, offsets[3], a._v.GetElement(3), b._v.GetElement(3), c._v.GetElement(3));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes256 LoadAt(ReadOnlySpan<float> source, int offset)
    {
        Mem.Range(source, offset, 8);
        return new(Vector256.LoadUnsafe(in Mem.At(source, offset)));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void StoreAt(Span<float> destination, int offset)
    {
        Mem.Range(destination, offset, 8);
        _v.StoreUnsafe(ref Mem.At(destination, offset));
    }

    public static void Scatter3(Span<float> destination, ReadOnlySpan<int> offsets, Lanes256 a, Lanes256 b, Lanes256 c)
    {
        if (Avx.IsSupported)
        {
            // The transpose of GatherRows for three fields: lane i's (a, b, c, 0) becomes one 16-byte row. Pure data movement. Slot 3 of a row is padding, so writing the zero there is harmless.
            var ab0 = Avx.UnpackLow(a._v, b._v); var ab1 = Avx.UnpackHigh(a._v, b._v);
            var c0 = Avx.UnpackLow(c._v, Vector256<float>.Zero); var c1 = Avx.UnpackHigh(c._v, Vector256<float>.Zero);
            var r0 = Avx.Shuffle(ab0, c0, 0x44); var r1 = Avx.Shuffle(ab0, c0, 0xEE); var r2 = Avx.Shuffle(ab1, c1, 0x44); var r3 = Avx.Shuffle(ab1, c1, 0xEE);
            StoreRow(destination, offsets[0], r0.GetLower()); StoreRow(destination, offsets[1], r1.GetLower());
            StoreRow(destination, offsets[2], r2.GetLower()); StoreRow(destination, offsets[3], r3.GetLower());
            StoreRow(destination, offsets[4], r0.GetUpper()); StoreRow(destination, offsets[5], r1.GetUpper());
            StoreRow(destination, offsets[6], r2.GetUpper()); StoreRow(destination, offsets[7], r3.GetUpper());
            return;
        }

        ScatterRow.Put(destination, offsets[0], a._v.GetElement(0), b._v.GetElement(0), c._v.GetElement(0));
        ScatterRow.Put(destination, offsets[1], a._v.GetElement(1), b._v.GetElement(1), c._v.GetElement(1));
        ScatterRow.Put(destination, offsets[2], a._v.GetElement(2), b._v.GetElement(2), c._v.GetElement(2));
        ScatterRow.Put(destination, offsets[3], a._v.GetElement(3), b._v.GetElement(3), c._v.GetElement(3));
        ScatterRow.Put(destination, offsets[4], a._v.GetElement(4), b._v.GetElement(4), c._v.GetElement(4));
        ScatterRow.Put(destination, offsets[5], a._v.GetElement(5), b._v.GetElement(5), c._v.GetElement(5));
        ScatterRow.Put(destination, offsets[6], a._v.GetElement(6), b._v.GetElement(6), c._v.GetElement(6));
        ScatterRow.Put(destination, offsets[7], a._v.GetElement(7), b._v.GetElement(7), c._v.GetElement(7));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void StoreRow(Span<float> destination, int offset, Vector128<float> row)
    {
        if (offset < 0) { return; }
        Mem.Range(destination, offset, 4);
        row.StoreUnsafe(ref Mem.At(destination, offset));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Rows8<Lanes256> GatherRows(ReadOnlySpan<float> source, ReadOnlySpan<int> offsets)
    {
        if (Avx.IsSupported)
        {
            // The usual 8x8 transpose: interleave pairs of rows, then quads, then swap the 128-bit halves. Pure data movement, so the bits are those of the scalar loads below.
            // The offsets name whole body rows that the preparation wrote, so each row is in range.
            Mem.Range(source, offsets[0], 8); Mem.Range(source, offsets[1], 8); Mem.Range(source, offsets[2], 8); Mem.Range(source, offsets[3], 8);
            Mem.Range(source, offsets[4], 8); Mem.Range(source, offsets[5], 8); Mem.Range(source, offsets[6], 8); Mem.Range(source, offsets[7], 8);
            var r0 = Vector256.LoadUnsafe(in Mem.At(source, offsets[0])); var r1 = Vector256.LoadUnsafe(in Mem.At(source, offsets[1]));
            var r2 = Vector256.LoadUnsafe(in Mem.At(source, offsets[2])); var r3 = Vector256.LoadUnsafe(in Mem.At(source, offsets[3]));
            var r4 = Vector256.LoadUnsafe(in Mem.At(source, offsets[4])); var r5 = Vector256.LoadUnsafe(in Mem.At(source, offsets[5]));
            var r6 = Vector256.LoadUnsafe(in Mem.At(source, offsets[6])); var r7 = Vector256.LoadUnsafe(in Mem.At(source, offsets[7]));
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

        var a = source.Slice(offsets[0], 8); var b = source.Slice(offsets[1], 8); var c = source.Slice(offsets[2], 8); var d = source.Slice(offsets[3], 8);
        var e = source.Slice(offsets[4], 8); var f = source.Slice(offsets[5], 8); var g = source.Slice(offsets[6], 8); var h = source.Slice(offsets[7], 8);
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
