using System;
using System.Buffers.Binary;
using System.Globalization;

namespace RecursiveAI.OnePhysics.Bake;

/// <summary>Constants and byte-level helpers of the bake cache format (see docs/spec/bake.md).</summary>
internal static class BakeFormat
{
    internal const uint Magic = 0x4B42504Fu; // "OPBK"
    internal const ushort Version = 1;
    internal const int HeaderFixedSize = 72;
    internal const uint TagRigid = 0x44474952u; // "RIGD"
    internal const uint TagSoft = 0x54464F53u;  // "SOFT"
    internal const uint TagRods = 0x53444F52u;  // "RODS"
    internal const uint TagParticles = 0x54524150u; // "PART"
    internal const uint TagFracture = 0x43415246u;  // "FRAC"
    internal const uint TagEvents = 0x544E5645u;    // "EVNT"
    internal const uint TagUnit = 0x54494E55u;      // "UNIT"
    internal const uint TagHashes = 0x48534148u;    // "HASH"
    internal const uint TagEnd = 0x20444E45u;       // "END "

    internal const int MaxFrames = 1 << 24;
    internal const int MaxBlockFrames = 4096;
    internal const int MaxPoints = 1 << 24;
    internal const int MaxStringBytes = 1024;

    /// <summary>2^e as a double, exactly (e in -1000..1000).</summary>
    internal static double Pow2(int e) => BitConverter.Int64BitsToDouble((long)(1023 + e) << 52);

    /// <summary>The largest exponent e (clamped to -40..0) with 2^e * metresPerUnit at most 2^baseExponent: the quantum in world units that is no coarser than 2^baseExponent metres.</summary>
    internal static int ExponentFor(int baseExponent, double metresPerUnit)
    {
        int e = baseExponent;
        while (e > -300 && Pow2(e) * metresPerUnit > Pow2(baseExponent)) { e--; }
        while (e < 300 && Pow2(e + 1) * metresPerUnit <= Pow2(baseExponent)) { e++; }
        return Math.Clamp(e, -40, 0);
    }

    // FNV-1a over the bytes: a cheap integrity check, not a security measure.
    internal static ulong Checksum(ReadOnlySpan<byte> bytes)
    {
        ulong h = 14695981039346656037UL;
        for (int i = 0; i < bytes.Length; i++) { h ^= bytes[i]; h *= 1099511628211UL; }
        return h;
    }

    internal static ulong ZigZag(long v) => unchecked((ulong)((v << 1) ^ (v >> 63)));
    internal static long UnZigZag(ulong v) => unchecked((long)(v >> 1) ^ -(long)(v & 1));

    /// <summary>Round to nearest, ties to even, of x * scale; scale is a power of two so the product is exact.</summary>
    internal static long Quantize(double x, double scale, string what)
    {
        double s = x * scale;
        if (!(s > -4.0e18 && s < 4.0e18)) { throw new BakeRangeException(what, x); }
        return (long)Math.Round(s, MidpointRounding.ToEven);
    }
}

/// <summary>Growable byte buffer with the writers the format uses (little-endian fixed fields, LEB128 varints).</summary>
internal sealed class ByteSink
{
    private byte[] _buf;
    private int _n;

    internal ByteSink(int capacity = 256) => _buf = new byte[Math.Max(16, capacity)];

    internal int Length => _n;
    internal ReadOnlySpan<byte> Span => _buf.AsSpan(0, _n);
    internal byte[] ToArray() => _buf.AsSpan(0, _n).ToArray();
    internal void Clear() => _n = 0;

    private Span<byte> Reserve(int count)
    {
        if (_n + count > _buf.Length) { Array.Resize(ref _buf, Math.Max(_buf.Length * 2, _n + count)); }
        var s = _buf.AsSpan(_n, count);
        _n += count;
        return s;
    }

    internal void U8(int v) => Reserve(1)[0] = (byte)v;
    internal void U16(ushort v) => BinaryPrimitives.WriteUInt16LittleEndian(Reserve(2), v);
    internal void U32(uint v) => BinaryPrimitives.WriteUInt32LittleEndian(Reserve(4), v);
    internal void I64(long v) => BinaryPrimitives.WriteInt64LittleEndian(Reserve(8), v);
    internal void U64(ulong v) => BinaryPrimitives.WriteUInt64LittleEndian(Reserve(8), v);
    internal void F32(float v) => BinaryPrimitives.WriteInt32LittleEndian(Reserve(4), float.IsNaN(v) ? 0x7fc00000 : BitConverter.SingleToInt32Bits(v));
    internal void F64(double v) => BinaryPrimitives.WriteInt64LittleEndian(Reserve(8), double.IsNaN(v) ? 0x7ff8000000000000L : BitConverter.DoubleToInt64Bits(v));
    internal void Bytes(ReadOnlySpan<byte> b) => b.CopyTo(Reserve(b.Length));

    internal void Var(ulong v)
    {
        while (v >= 0x80) { U8((int)(v | 0x80)); v >>= 7; }
        U8((int)v);
    }

    internal void Str(string s)
    {
        var b = System.Text.Encoding.UTF8.GetBytes(s);
        Var((ulong)b.Length);
        Bytes(b);
    }

    internal void PatchU32(int at, uint v) => BinaryPrimitives.WriteUInt32LittleEndian(_buf.AsSpan(at, 4), v);
}

/// <summary>Bounds-checked reader over a byte range; every failure is a <see cref="BakeFormatException"/> naming the field.</summary>
internal ref struct ByteSource
{
    private readonly ReadOnlySpan<byte> _b;
    private readonly int _baseOffset;
    private int _p;

    internal ByteSource(ReadOnlySpan<byte> bytes, int baseOffset = 0) { _b = bytes; _baseOffset = baseOffset; _p = 0; }

    internal readonly int Position => _p;
    internal readonly int Remaining => _b.Length - _p;
    internal readonly int Offset => _baseOffset + _p;

    internal readonly BakeFormatException Fail(string field, string problem) => new(field, _baseOffset + _p, problem);

    private ReadOnlySpan<byte> Take(int n, string field)
    {
        if (n < 0 || n > _b.Length - _p) { throw Fail(field, "the data ends before the field does"); }
        var s = _b.Slice(_p, n);
        _p += n;
        return s;
    }

    internal ReadOnlySpan<byte> Slice(int n, string field) => Take(n, field);
    internal byte U8(string field) => Take(1, field)[0];
    internal ushort U16(string field) => BinaryPrimitives.ReadUInt16LittleEndian(Take(2, field));
    internal uint U32(string field) => BinaryPrimitives.ReadUInt32LittleEndian(Take(4, field));
    internal long I64(string field) => BinaryPrimitives.ReadInt64LittleEndian(Take(8, field));
    internal ulong U64(string field) => BinaryPrimitives.ReadUInt64LittleEndian(Take(8, field));
    internal float F32(string field) => BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(Take(4, field)));
    internal double F64(string field) => BitConverter.Int64BitsToDouble(BinaryPrimitives.ReadInt64LittleEndian(Take(8, field)));
    internal void Skip(int n, string field) => Take(n, field);

    internal ulong Var(string field)
    {
        ulong v = 0;
        for (int shift = 0; shift < 70; shift += 7)
        {
            if (_p >= _b.Length) { throw Fail(field, "the data ends inside a variable-length integer"); }
            byte c = _b[_p++];
            if (shift == 63 && c > 1) { throw Fail(field, "a variable-length integer does not fit 64 bits"); }
            v |= (ulong)(c & 0x7F) << shift;
            if ((c & 0x80) == 0) { return v; }
        }
        throw Fail(field, "a variable-length integer is longer than 10 bytes");
    }

    /// <summary>A varint that must be at most <paramref name="max"/>.</summary>
    internal int VarInt(string field, int max)
    {
        ulong v = Var(field);
        if (v > (ulong)max) { throw Fail(field, "value " + v.ToString(CultureInfo.InvariantCulture) + " is above the limit " + max.ToString(CultureInfo.InvariantCulture)); }
        return (int)v;
    }

    internal string Str(string field)
    {
        int n = VarInt(field + " length", BakeFormat.MaxStringBytes);
        var s = Take(n, field);
        try { return new System.Text.UTF8Encoding(false, true).GetString(s); }
        catch (System.Text.DecoderFallbackException) { throw Fail(field, "the text is not valid UTF-8"); }
    }
}
