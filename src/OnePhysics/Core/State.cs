using System;
using System.Buffers.Binary;

namespace RecursiveAI.OnePhysics.Core;

/// <summary>
/// Writes world state in the canonical order, either into a byte buffer (a snapshot, little-endian) or into a running 64-bit hash (the per-step world hash).
/// Both modes see exactly the same sequence of primitives, so the hash is a function of the state a snapshot would carry.
/// </summary>
/// <remarks>
/// Hash definition: start with h = 0xCBF29CE484222325. Each primitive is widened to an unsigned 64-bit word w
/// (bytes and 32-bit values zero-extended, floats and doubles by their IEEE bit patterns) and folded as
/// h = (h xor w) * 0x9E3779B97F4A7C15, then h = h xor (h >> 32).
/// </remarks>
public sealed class StateWriter
{
    private const ulong HashSeed = 0xCBF29CE484222325UL;
    private const ulong HashMul = 0x9E3779B97F4A7C15UL;

    private byte[] _buffer;
    private int _length;
    private ulong _hash = HashSeed;
    private readonly bool _hashing;
    private readonly bool _counting;

    private StateWriter(bool hashing, bool counting)
    {
        _hashing = hashing;
        _counting = counting;
        _buffer = hashing || counting ? Array.Empty<byte>() : new byte[4096];
    }

    /// <summary>Creates a writer that produces snapshot bytes.</summary>
    public static StateWriter ToBuffer() => new(false, false);

    /// <summary>Creates a writer that produces only the world hash.</summary>
    public static StateWriter ForHash() => new(true, false);

    /// <summary>Creates a writer that only counts the bytes a snapshot would take (<see cref="Length"/>) and stores nothing.</summary>
    public static StateWriter ForCount() => new(false, true);

    /// <summary>True when this writer feeds the hash and produces no bytes.</summary>
    public bool IsHashing => _hashing;

    /// <summary>True when this writer only counts bytes.</summary>
    public bool IsCounting => _counting;

    /// <summary>The hash of everything written so far (hash mode only).</summary>
    public ulong Hash => _hash;

    /// <summary>Number of bytes written so far (buffer and count modes).</summary>
    public int Length => _length;

    /// <summary>Makes room for at least <paramref name="bytes"/> bytes so writing up to that much allocates nothing. No effect on a hashing or counting writer.</summary>
    internal void EnsureCapacity(int bytes)
    {
        if (!_hashing && !_counting && _buffer.Length < bytes) { Array.Resize(ref _buffer, bytes); }
    }

    /// <summary>Starts over: an empty buffer (its storage is kept), a zero count, or the initial hash. Lets a world reuse one writer for every snapshot or hash.</summary>
    public void Reset()
    {
        _length = 0;
        _hash = HashSeed;
    }

    /// <summary>Copies the bytes written so far (buffer mode).</summary>
    public byte[] ToArray() => _buffer.AsSpan(0, _length).ToArray();

    /// <summary>The bytes written so far (buffer mode); valid until the next write or reset.</summary>
    public ReadOnlySpan<byte> WrittenSpan => _buffer.AsSpan(0, _length);

    /// <summary>The array behind <see cref="WrittenSpan"/>, for a reader over the same bytes.</summary>
    internal byte[] Storage => _buffer;

    private void Mix(ulong w)
    {
        ulong h = (_hash ^ w) * HashMul;
        _hash = h ^ (h >> 32);
    }

    private Span<byte> Reserve(int n)
    {
        if (_length + n > _buffer.Length)
        {
            Array.Resize(ref _buffer, Math.Max(_buffer.Length * 2, _length + n));
        }

        var s = _buffer.AsSpan(_length, n);
        _length += n;
        return s;
    }

    /// <summary>Writes a placeholder 32-bit integer and returns its position for <see cref="PatchInt32"/> (buffer and count modes; in hash mode nothing is written and the result is meaningless).</summary>
    public int ReserveInt32()
    {
        int at = _length;
        if (_hashing) { return at; }
        if (_counting) { _length += 4; } else { Reserve(4).Clear(); }
        return at;
    }

    /// <summary>Fills in a placeholder made by <see cref="ReserveInt32"/> (a no-op in hash and count modes).</summary>
    public void PatchInt32(int at, int v)
    {
        if (_hashing || _counting) { return; }
        BinaryPrimitives.WriteInt32LittleEndian(_buffer.AsSpan(at, 4), v);
    }

    /// <summary>Writes one byte.</summary>
    public void WriteByte(byte v)
    {
        if (_hashing) { Mix(v); } else if (_counting) { _length += 1; } else { Reserve(1)[0] = v; }
    }

    /// <summary>Writes a boolean as one byte, 0 or 1.</summary>
    public void WriteBool(bool v) => WriteByte(v ? (byte)1 : (byte)0);

    /// <summary>Writes a 32-bit integer.</summary>
    public void WriteInt32(int v)
    {
        if (_hashing) { Mix((uint)v); } else if (_counting) { _length += 4; } else { BinaryPrimitives.WriteInt32LittleEndian(Reserve(4), v); }
    }

    /// <summary>Writes an unsigned 32-bit integer.</summary>
    public void WriteUInt32(uint v)
    {
        if (_hashing) { Mix(v); } else if (_counting) { _length += 4; } else { BinaryPrimitives.WriteUInt32LittleEndian(Reserve(4), v); }
    }

    /// <summary>Writes a 64-bit integer.</summary>
    public void WriteInt64(long v)
    {
        if (_hashing) { Mix((ulong)v); } else if (_counting) { _length += 8; } else { BinaryPrimitives.WriteInt64LittleEndian(Reserve(8), v); }
    }

    /// <summary>Writes an unsigned 64-bit integer.</summary>
    public void WriteUInt64(ulong v)
    {
        if (_hashing) { Mix(v); } else if (_counting) { _length += 8; } else { BinaryPrimitives.WriteUInt64LittleEndian(Reserve(8), v); }
    }

    /// <summary>Writes a float32 by its IEEE bit pattern.</summary>
    public void WriteSingle(float v)
    {
        if (_hashing) { Mix(BitConverter.SingleToUInt32Bits(v)); } else if (_counting) { _length += 4; } else { BinaryPrimitives.WriteUInt32LittleEndian(Reserve(4), BitConverter.SingleToUInt32Bits(v)); }
    }

    /// <summary>Writes a double by its IEEE bit pattern.</summary>
    public void WriteDouble(double v)
    {
        if (_hashing) { Mix(BitConverter.DoubleToUInt64Bits(v)); } else if (_counting) { _length += 8; } else { BinaryPrimitives.WriteUInt64LittleEndian(Reserve(8), BitConverter.DoubleToUInt64Bits(v)); }
    }

    /// <summary>Writes the floats in order, without a count.</summary>
    public void WriteSingles(ReadOnlySpan<float> v)
    {
        if (_hashing)
        {
            foreach (float f in v) { Mix(BitConverter.SingleToUInt32Bits(f)); }
        }
        else if (_counting) { _length += v.Length * 4; }
        else
        {
            var s = Reserve(v.Length * 4);
            for (int i = 0; i < v.Length; i++) { BinaryPrimitives.WriteUInt32LittleEndian(s.Slice(i * 4, 4), BitConverter.SingleToUInt32Bits(v[i])); }
        }
    }

    /// <summary>Writes the doubles in order, without a count.</summary>
    public void WriteDoubles(ReadOnlySpan<double> v)
    {
        if (_hashing)
        {
            foreach (double d in v) { Mix(BitConverter.DoubleToUInt64Bits(d)); }
        }
        else if (_counting) { _length += v.Length * 8; }
        else
        {
            var s = Reserve(v.Length * 8);
            for (int i = 0; i < v.Length; i++) { BinaryPrimitives.WriteUInt64LittleEndian(s.Slice(i * 8, 8), BitConverter.DoubleToUInt64Bits(v[i])); }
        }
    }

    /// <summary>Writes the 32-bit integers in order, without a count.</summary>
    public void WriteInt32s(ReadOnlySpan<int> v)
    {
        if (_hashing)
        {
            foreach (int x in v) { Mix((uint)x); }
        }
        else if (_counting) { _length += v.Length * 4; }
        else
        {
            var s = Reserve(v.Length * 4);
            for (int i = 0; i < v.Length; i++) { BinaryPrimitives.WriteInt32LittleEndian(s.Slice(i * 4, 4), v[i]); }
        }
    }

    /// <summary>Writes the unsigned 32-bit integers in order, without a count.</summary>
    public void WriteUInt32s(ReadOnlySpan<uint> v)
    {
        if (_hashing)
        {
            foreach (uint x in v) { Mix(x); }
        }
        else if (_counting) { _length += v.Length * 4; }
        else
        {
            var s = Reserve(v.Length * 4);
            for (int i = 0; i < v.Length; i++) { BinaryPrimitives.WriteUInt32LittleEndian(s.Slice(i * 4, 4), v[i]); }
        }
    }

    /// <summary>Writes the bytes in order, without a count.</summary>
    public void WriteBytes(ReadOnlySpan<byte> v)
    {
        if (_hashing)
        {
            foreach (byte x in v) { Mix(x); }
        }
        else if (_counting) { _length += v.Length; }
        else
        {
            v.CopyTo(Reserve(v.Length));
        }
    }
}

/// <summary>Reads world state written by <see cref="StateWriter"/>. Every read is bounds-checked and failures raise <see cref="SnapshotFormatException"/>.</summary>
public sealed class StateReader
{
    private byte[] _data;
    private int _start;
    private int _end;
    private int _pos;

    /// <summary>Creates a reader over <paramref name="data"/> (not copied).</summary>
    public StateReader(byte[] data)
    {
        ArgumentNullException.ThrowIfNull(data);
        _data = data;
        _end = data.Length;
    }

    /// <summary>Creates a reader over a part of <paramref name="data"/> (not copied).</summary>
    public StateReader(byte[] data, int offset, int length)
    {
        _data = Array.Empty<byte>();
        Reset(data, offset, length);
    }

    /// <summary>Points the reader at a part of <paramref name="data"/> and rewinds it, so one reader serves many snapshots without allocating.</summary>
    internal void Reset(byte[] data, int offset, int length)
    {
        ArgumentNullException.ThrowIfNull(data);
        if (offset < 0 || length < 0 || offset > data.Length - length) { throw new ArgumentOutOfRangeException(nameof(length), "The range is outside the data."); }
        _data = data;
        _start = offset;
        _end = offset + length;
        _pos = offset;
    }

    /// <summary>Current byte offset from the start of the range this reader covers.</summary>
    public int Position => _pos - _start;

    /// <summary>Bytes not yet read.</summary>
    public int Remaining => _end - _pos;

    /// <summary>Skips <paramref name="n"/> bytes and returns the absolute offset (in the underlying array) where they start.</summary>
    internal int Skip(int n)
    {
        int at = _pos;
        Take(n);
        return at;
    }

    /// <summary>Raises a format error at the current offset.</summary>
    public SnapshotFormatException Error(string message) => new(message, _pos - _start);

    private ReadOnlySpan<byte> Take(int n)
    {
        if (n < 0 || n > _end - _pos)
        {
            throw Error("truncated snapshot: " + n.ToString(System.Globalization.CultureInfo.InvariantCulture) + " bytes needed, " + Remaining.ToString(System.Globalization.CultureInfo.InvariantCulture) + " left");
        }

        var s = new ReadOnlySpan<byte>(_data, _pos, n);
        _pos += n;
        return s;
    }

    /// <summary>Reads one byte.</summary>
    public byte ReadByte() => Take(1)[0];

    /// <summary>Reads a boolean written as 0 or 1; any other value is an error.</summary>
    public bool ReadBool()
    {
        byte b = ReadByte();
        if (b > 1) { throw Error("invalid boolean " + b.ToString(System.Globalization.CultureInfo.InvariantCulture)); }
        return b == 1;
    }

    /// <summary>Reads a 32-bit integer.</summary>
    public int ReadInt32() => BinaryPrimitives.ReadInt32LittleEndian(Take(4));

    /// <summary>Reads an unsigned 32-bit integer.</summary>
    public uint ReadUInt32() => BinaryPrimitives.ReadUInt32LittleEndian(Take(4));

    /// <summary>Reads a 64-bit integer.</summary>
    public long ReadInt64() => BinaryPrimitives.ReadInt64LittleEndian(Take(8));

    /// <summary>Reads a float32 (any bit pattern).</summary>
    public float ReadSingle() => BitConverter.UInt32BitsToSingle(BinaryPrimitives.ReadUInt32LittleEndian(Take(4)));

    /// <summary>Reads a double (any bit pattern).</summary>
    public double ReadDouble() => BitConverter.UInt64BitsToDouble(BinaryPrimitives.ReadUInt64LittleEndian(Take(8)));

    /// <summary>Reads a count and checks that <paramref name="perElementBytes"/> bytes per element are actually left in the input, so a hostile count cannot make us allocate more than the input is long.</summary>
    public int ReadCount(int perElementBytes, string what)
    {
        int n = ReadInt32();
        if (n < 0) { throw Error("negative count of " + what); }
        if (perElementBytes > 0 && (long)n * perElementBytes > Remaining)
        {
            throw Error("count of " + what + " (" + n.ToString(System.Globalization.CultureInfo.InvariantCulture) + ") is larger than the remaining bytes can hold");
        }

        return n;
    }

    /// <summary>As <see cref="ReadCount(int, string)"/>, naming the element as <paramref name="what"/> plus <paramref name="suffix"/>; the name is only built when the count is rejected, so a successful read allocates nothing.</summary>
    public int ReadCount(int perElementBytes, string what, string suffix)
    {
        int n = ReadInt32();
        if (n < 0) { throw Error("negative count of " + what + suffix); }
        if (perElementBytes > 0 && (long)n * perElementBytes > Remaining)
        {
            throw Error("count of " + what + suffix + " (" + n.ToString(System.Globalization.CultureInfo.InvariantCulture) + ") is larger than the remaining bytes can hold");
        }

        return n;
    }

    /// <summary>Fills <paramref name="dest"/> with floats; non-finite values are an error when <paramref name="finiteOnly"/> is set.</summary>
    public void ReadSingles(Span<float> dest, bool finiteOnly, string what)
    {
        var s = Take(dest.Length * 4);
        for (int i = 0; i < dest.Length; i++)
        {
            float f = BitConverter.UInt32BitsToSingle(BinaryPrimitives.ReadUInt32LittleEndian(s.Slice(i * 4, 4)));
            if (finiteOnly && !float.IsFinite(f)) { throw new SnapshotFormatException("non-finite value in " + what, _pos - _start - (dest.Length - i) * 4); }
            dest[i] = f;
        }
    }

    /// <summary>Fills <paramref name="dest"/> with doubles; non-finite values are an error when <paramref name="finiteOnly"/> is set.</summary>
    public void ReadDoubles(Span<double> dest, bool finiteOnly, string what)
    {
        var s = Take(dest.Length * 8);
        for (int i = 0; i < dest.Length; i++)
        {
            double d = BitConverter.UInt64BitsToDouble(BinaryPrimitives.ReadUInt64LittleEndian(s.Slice(i * 8, 8)));
            if (finiteOnly && !double.IsFinite(d)) { throw new SnapshotFormatException("non-finite value in " + what, _pos - _start - (dest.Length - i) * 8); }
            dest[i] = d;
        }
    }

    /// <summary>Fills <paramref name="dest"/> with 32-bit integers.</summary>
    public void ReadInt32s(Span<int> dest)
    {
        var s = Take(dest.Length * 4);
        for (int i = 0; i < dest.Length; i++) { dest[i] = BinaryPrimitives.ReadInt32LittleEndian(s.Slice(i * 4, 4)); }
    }

    /// <summary>Fills <paramref name="dest"/> with unsigned 32-bit integers.</summary>
    public void ReadUInt32s(Span<uint> dest)
    {
        var s = Take(dest.Length * 4);
        for (int i = 0; i < dest.Length; i++) { dest[i] = BinaryPrimitives.ReadUInt32LittleEndian(s.Slice(i * 4, 4)); }
    }

    /// <summary>Fills <paramref name="dest"/> with bytes.</summary>
    public void ReadBytes(Span<byte> dest) => Take(dest.Length).CopyTo(dest);
}

/// <summary>
/// A named block of extra state (for example shape geometry or joint parameters owned by an engine) saved and hashed after the core's own state.
/// Sections are written in registration order, each with its id and byte length so a reader can skip nothing and detect a mismatch.
/// </summary>
public interface ISnapshotSection
{
    /// <summary>Stable identifier of the section; unique within a world.</summary>
    int SectionId { get; }

    /// <summary>Writes the section's state. Must write the same primitives in the same order for equal states.</summary>
    void Write(StateWriter writer);

    /// <summary>Reads state written by <see cref="Write"/>, validating it, and either replaces the section's state completely or throws <see cref="SnapshotFormatException"/> leaving it unchanged.</summary>
    void Read(StateReader reader);
}
