using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
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
    /// <see cref="GatherRows(ReadOnlySpan{float}, ReadOnlySpan{int})"/> with the offset of lane i computed here as <c>rows[i] * stride + skip</c>: the row numbers are read once and scaled in registers,
    /// so a caller that has row numbers does not write an offset table first.
    /// </summary>
    static abstract Rows8<T> GatherRows(ReadOnlySpan<float> source, ReadOnlySpan<int> rows, int stride, int skip);

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
    public static Rows8<Lanes1> GatherRows(ReadOnlySpan<float> source, ReadOnlySpan<int> rows, int stride, int skip)
    {
        var r = source.Slice((rows[0] * stride + skip), 8);
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
    public static Rows8<Lanes128> GatherRows(ReadOnlySpan<float> source, ReadOnlySpan<int> rows, int stride, int skip)
    {
        var a = source.Slice((rows[0] * stride + skip), 8); var b = source.Slice((rows[1] * stride + skip), 8); var c = source.Slice((rows[2] * stride + skip), 8); var d = source.Slice((rows[3] * stride + skip), 8);
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

    /// <summary>
    /// Four consecutive floats of eight rows as four lane vectors: lane i of <paramref name="c0"/> is <c>source[rows[i] * stride + skip]</c>, of <paramref name="c1"/> the float after it, and so on.
    /// Each row is read with one exact four-float load (nothing past the fourth float is touched, so the source needs no padding) and the rows are transposed in registers. Pure data movement.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void GatherRows4(ReadOnlySpan<float> source, ReadOnlySpan<int> rows, int stride, int skip, out Lanes256 c0, out Lanes256 c1, out Lanes256 c2, out Lanes256 c3)
    {
        int o0 = rows[0] * stride + skip, o1 = rows[1] * stride + skip, o2 = rows[2] * stride + skip, o3 = rows[3] * stride + skip;
        int o4 = rows[4] * stride + skip, o5 = rows[5] * stride + skip, o6 = rows[6] * stride + skip, o7 = rows[7] * stride + skip;
        Mem.Range(source, o0, 4); Mem.Range(source, o1, 4); Mem.Range(source, o2, 4); Mem.Range(source, o3, 4); Mem.Range(source, o4, 4); Mem.Range(source, o5, 4); Mem.Range(source, o6, 4); Mem.Range(source, o7, 4);
        if (Avx.IsSupported)
        {
            // Rows i and i + 4 share a vector (low and high half); the 4x4 transpose then works inside each half and leaves the lanes in order.
            var y0 = Vector256.Create(Vector128.LoadUnsafe(in Mem.At(source, o0)), Vector128.LoadUnsafe(in Mem.At(source, o4)));
            var y1 = Vector256.Create(Vector128.LoadUnsafe(in Mem.At(source, o1)), Vector128.LoadUnsafe(in Mem.At(source, o5)));
            var y2 = Vector256.Create(Vector128.LoadUnsafe(in Mem.At(source, o2)), Vector128.LoadUnsafe(in Mem.At(source, o6)));
            var y3 = Vector256.Create(Vector128.LoadUnsafe(in Mem.At(source, o3)), Vector128.LoadUnsafe(in Mem.At(source, o7)));
            var t0 = Avx.UnpackLow(y0, y1); var t1 = Avx.UnpackHigh(y0, y1); var t2 = Avx.UnpackLow(y2, y3); var t3 = Avx.UnpackHigh(y2, y3);
            c0 = new(Avx.Shuffle(t0, t2, 0x44)); c1 = new(Avx.Shuffle(t0, t2, 0xEE)); c2 = new(Avx.Shuffle(t1, t3, 0x44)); c3 = new(Avx.Shuffle(t1, t3, 0xEE));
            return;
        }

        c0 = new(Vector256.Create(Mem.At(source, o0), Mem.At(source, o1), Mem.At(source, o2), Mem.At(source, o3), Mem.At(source, o4), Mem.At(source, o5), Mem.At(source, o6), Mem.At(source, o7)));
        c1 = new(Vector256.Create(Mem.At(source, o0 + 1), Mem.At(source, o1 + 1), Mem.At(source, o2 + 1), Mem.At(source, o3 + 1), Mem.At(source, o4 + 1), Mem.At(source, o5 + 1), Mem.At(source, o6 + 1), Mem.At(source, o7 + 1)));
        c2 = new(Vector256.Create(Mem.At(source, o0 + 2), Mem.At(source, o1 + 2), Mem.At(source, o2 + 2), Mem.At(source, o3 + 2), Mem.At(source, o4 + 2), Mem.At(source, o5 + 2), Mem.At(source, o6 + 2), Mem.At(source, o7 + 2)));
        c3 = new(Vector256.Create(Mem.At(source, o0 + 3), Mem.At(source, o1 + 3), Mem.At(source, o2 + 3), Mem.At(source, o3 + 3), Mem.At(source, o4 + 3), Mem.At(source, o5 + 3), Mem.At(source, o6 + 3), Mem.At(source, o7 + 3)));
    }

    /// <summary>One float of eight rows as a lane vector: lane i is <c>source[rows[i] * stride + skip]</c>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Lanes256 GatherOne(ReadOnlySpan<float> source, ReadOnlySpan<int> rows, int stride = 1, int skip = 0) =>
        new(Vector256.Create(Mem.At(source, rows[0] * stride + skip), Mem.At(source, rows[1] * stride + skip), Mem.At(source, rows[2] * stride + skip), Mem.At(source, rows[3] * stride + skip),
            Mem.At(source, rows[4] * stride + skip), Mem.At(source, rows[5] * stride + skip), Mem.At(source, rows[6] * stride + skip), Mem.At(source, rows[7] * stride + skip)));

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
    public static Rows8<Lanes256> GatherRows(ReadOnlySpan<float> source, ReadOnlySpan<int> rows, int stride, int skip)
    {
        if (Avx.IsSupported)
        {
            // The usual 8x8 transpose: interleave pairs of rows, then quads, then swap the 128-bit halves. Pure data movement, so the bits are those of the scalar loads below.
            // The offsets name whole body rows that the preparation wrote, so each row is in range.
            Mem.Range(source, (rows[0] * stride + skip), 8); Mem.Range(source, (rows[1] * stride + skip), 8); Mem.Range(source, (rows[2] * stride + skip), 8); Mem.Range(source, (rows[3] * stride + skip), 8);
            Mem.Range(source, (rows[4] * stride + skip), 8); Mem.Range(source, (rows[5] * stride + skip), 8); Mem.Range(source, (rows[6] * stride + skip), 8); Mem.Range(source, (rows[7] * stride + skip), 8);
            var r0 = Vector256.LoadUnsafe(in Mem.At(source, (rows[0] * stride + skip))); var r1 = Vector256.LoadUnsafe(in Mem.At(source, (rows[1] * stride + skip)));
            var r2 = Vector256.LoadUnsafe(in Mem.At(source, (rows[2] * stride + skip))); var r3 = Vector256.LoadUnsafe(in Mem.At(source, (rows[3] * stride + skip)));
            var r4 = Vector256.LoadUnsafe(in Mem.At(source, (rows[4] * stride + skip))); var r5 = Vector256.LoadUnsafe(in Mem.At(source, (rows[5] * stride + skip)));
            var r6 = Vector256.LoadUnsafe(in Mem.At(source, (rows[6] * stride + skip))); var r7 = Vector256.LoadUnsafe(in Mem.At(source, (rows[7] * stride + skip)));
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

        var a = source.Slice((rows[0] * stride + skip), 8); var b = source.Slice((rows[1] * stride + skip), 8); var c = source.Slice((rows[2] * stride + skip), 8); var d = source.Slice((rows[3] * stride + skip), 8);
        var e = source.Slice((rows[4] * stride + skip), 8); var f = source.Slice((rows[5] * stride + skip), 8); var g = source.Slice((rows[6] * stride + skip), 8); var h = source.Slice((rows[7] * stride + skip), 8);
        return new Rows8<Lanes256>
        {
            R0 = new(Vector256.Create(a[0], b[0], c[0], d[0], e[0], f[0], g[0], h[0])), R1 = new(Vector256.Create(a[1], b[1], c[1], d[1], e[1], f[1], g[1], h[1])),
            R2 = new(Vector256.Create(a[2], b[2], c[2], d[2], e[2], f[2], g[2], h[2])), R3 = new(Vector256.Create(a[3], b[3], c[3], d[3], e[3], f[3], g[3], h[3])),
            R4 = new(Vector256.Create(a[4], b[4], c[4], d[4], e[4], f[4], g[4], h[4])), R5 = new(Vector256.Create(a[5], b[5], c[5], d[5], e[5], f[5], g[5], h[5])),
            R6 = new(Vector256.Create(a[6], b[6], c[6], d[6], e[6], f[6], g[6], h[6])), R7 = new(Vector256.Create(a[7], b[7], c[7], d[7], e[7], f[7], g[7], h[7])),
        };
    }

    /// <summary>
    /// The inverse of <see cref="GatherRows(ReadOnlySpan{float}, ReadOnlySpan{int}, int, int)"/>. Lane i of <paramref name="f0"/> to <paramref name="f7"/> (eight fields of eight bodies) becomes the row
    /// <c>[f0[i], f1[i], ..., f7[i]]</c>, stored whole (one 32-byte store) at <c>dest[rows[i] * stride + skip]</c>, for every lane i below <paramref name="n"/> whose row number is not negative (row −1 means
    /// "this lane's body keeps its row"). The same 8x8 transpose as the gather (a transpose is its own inverse), pure data movement, so the stored bits are the lanes' bits. The next gather of these rows
    /// loads exactly what one store wrote, so its loads forward from the store buffer instead of waiting on several narrow stores. AVX only: the callers test <see cref="Avx.IsSupported"/>.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ScatterRows(Vector256<float> f0, Vector256<float> f1, Vector256<float> f2, Vector256<float> f3, Vector256<float> f4, Vector256<float> f5, Vector256<float> f6, Vector256<float> f7,
        Span<float> dest, ref int rows, int n, int stride, int skip)
    {
        var t0 = Avx.UnpackLow(f0, f1); var t1 = Avx.UnpackHigh(f0, f1); var t2 = Avx.UnpackLow(f2, f3); var t3 = Avx.UnpackHigh(f2, f3);
        var t4 = Avx.UnpackLow(f4, f5); var t5 = Avx.UnpackHigh(f4, f5); var t6 = Avx.UnpackLow(f6, f7); var t7 = Avx.UnpackHigh(f6, f7);
        var u0 = Avx.Shuffle(t0, t2, 0x44); var u1 = Avx.Shuffle(t0, t2, 0xEE); var u2 = Avx.Shuffle(t1, t3, 0x44); var u3 = Avx.Shuffle(t1, t3, 0xEE);
        var u4 = Avx.Shuffle(t4, t6, 0x44); var u5 = Avx.Shuffle(t4, t6, 0xEE); var u6 = Avx.Shuffle(t5, t7, 0x44); var u7 = Avx.Shuffle(t5, t7, 0xEE);
        ref float d = ref Mem.Ref(dest);
        int r;
        // Each stored row is a whole body row of the destination (rows[i] names a body below the high-water mark the destination was sized for); the Checked build verifies it.
        if (n > 0 && (r = rows) >= 0) { Mem.Range(dest, r * stride + skip, 8); Avx.Permute2x128(u0, u4, 0x20).StoreUnsafe(ref Mem.Add(ref d, r * stride + skip)); }
        if (n > 1 && (r = Mem.Add(ref rows, 1)) >= 0) { Mem.Range(dest, r * stride + skip, 8); Avx.Permute2x128(u1, u5, 0x20).StoreUnsafe(ref Mem.Add(ref d, r * stride + skip)); }
        if (n > 2 && (r = Mem.Add(ref rows, 2)) >= 0) { Mem.Range(dest, r * stride + skip, 8); Avx.Permute2x128(u2, u6, 0x20).StoreUnsafe(ref Mem.Add(ref d, r * stride + skip)); }
        if (n > 3 && (r = Mem.Add(ref rows, 3)) >= 0) { Mem.Range(dest, r * stride + skip, 8); Avx.Permute2x128(u3, u7, 0x20).StoreUnsafe(ref Mem.Add(ref d, r * stride + skip)); }
        if (n > 4 && (r = Mem.Add(ref rows, 4)) >= 0) { Mem.Range(dest, r * stride + skip, 8); Avx.Permute2x128(u0, u4, 0x31).StoreUnsafe(ref Mem.Add(ref d, r * stride + skip)); }
        if (n > 5 && (r = Mem.Add(ref rows, 5)) >= 0) { Mem.Range(dest, r * stride + skip, 8); Avx.Permute2x128(u1, u5, 0x31).StoreUnsafe(ref Mem.Add(ref d, r * stride + skip)); }
        if (n > 6 && (r = Mem.Add(ref rows, 6)) >= 0) { Mem.Range(dest, r * stride + skip, 8); Avx.Permute2x128(u2, u6, 0x31).StoreUnsafe(ref Mem.Add(ref d, r * stride + skip)); }
        if (n > 7 && (r = Mem.Add(ref rows, 7)) >= 0) { Mem.Range(dest, r * stride + skip, 8); Avx.Permute2x128(u3, u7, 0x31).StoreUnsafe(ref Mem.Add(ref d, r * stride + skip)); }
    }

    /// <summary>The vector of a lane value (for <see cref="ScatterRows"/>).</summary>
    public Vector256<float> AsVector256 { [MethodImpl(MethodImplOptions.AggressiveInlining)] get => _v; }

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
