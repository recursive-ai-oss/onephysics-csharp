using System;
using System.Buffers.Binary;
using System.IO;
using System.Security.Cryptography;

namespace RecursiveAI.OnePhysics;

/// <summary>
/// Lossless byte-level snapshot transforms shared by both engines. These do not parse or change snapshot v1.
/// Integers are little-endian; compression uses deterministic byte runs, not a platform codec. See docs/spec/snapshots.md.
/// </summary>
public static class SnapshotEncoding
{
    private const uint DeltaMagic = 0x4441504F; // OPAD
    private const uint CompressedMagic = 0x4341504F; // OPAC
    private const int DeltaHeader = 80;
    private const int CompressedHeader = 44;

    /// <summary>Default decoded-output limit, 256 MiB, checked before allocation. Callers may pass a smaller limit.</summary>
    public const int DefaultMaximumDecodedBytes = 256 * 1024 * 1024;

    /// <summary>
    /// Creates a version-1 delta from arbitrary base and target bytes, including all optional snapshot sections.
    /// Unchanged spans are omitted. Neither input is retained. Output can be larger than the target for highly fragmented edits.
    /// </summary>
    public static byte[] CreateDelta(ReadOnlySpan<byte> baseSnapshot, ReadOnlySpan<byte> snapshot) => CreateDeltaCore(baseSnapshot, snapshot, false);

    /// <summary>Creates a version-2 delta whose literals are XORed against the base (zero beyond its end). This often compresses better when float/double values change by a few bits. ApplyDelta accepts both versions.</summary>
    public static byte[] CreateXorDelta(ReadOnlySpan<byte> baseSnapshot, ReadOnlySpan<byte> snapshot) => CreateDeltaCore(baseSnapshot, snapshot, true);

    private static byte[] CreateDeltaCore(ReadOnlySpan<byte> baseSnapshot, ReadOnlySpan<byte> snapshot, bool xor)
    {
        using var output = new MemoryStream();
        WriteInt(output, DeltaMagic);
        WriteInt(output, xor ? 2u : 1u);
        WriteInt(output, (uint)baseSnapshot.Length);
        WriteInt(output, (uint)snapshot.Length);
        output.Write(SHA256.HashData(baseSnapshot));
        output.Write(SHA256.HashData(snapshot));
        int i = 0;
        while (i < snapshot.Length)
        {
            if (i < baseSnapshot.Length && snapshot[i] == baseSnapshot[i]) { i++; continue; }
            int start = i++;
            // Absorb short equal gaps: another record costs eight bytes.
            int end = i;
            while (i < snapshot.Length)
            {
                if (i >= baseSnapshot.Length || snapshot[i] != baseSnapshot[i]) { end = ++i; }
                else if (i - end < 8) { i++; }
                else { break; }
            }
            WriteInt(output, (uint)start);
            WriteInt(output, (uint)(end - start));
            if (!xor) { output.Write(snapshot.Slice(start, end - start)); }
            else { for (int j = start; j < end; j++) { output.WriteByte((byte)(snapshot[j] ^ (j < baseSnapshot.Length ? baseSnapshot[j] : 0))); } }
        }
        return output.ToArray();
    }

    /// <summary>
    /// Applies a delta after checking its base length and SHA-256. Returns exact target bytes; does not restore a world.
    /// Checks payload ranges before allocating output, then checks the target SHA-256. No trailing bytes are permitted.
    /// </summary>
    /// <exception cref="SnapshotEncodingException">Invalid delta, wrong base, or decoded length above the limit.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The output limit is negative.</exception>
    public static byte[] ApplyDelta(ReadOnlySpan<byte> baseSnapshot, ReadOnlySpan<byte> delta, int maximumDecodedBytes = DefaultMaximumDecodedBytes)
    {
        ValidateHeader(delta, DeltaMagic, DeltaHeader, true);
        bool xor = BinaryPrimitives.ReadUInt32LittleEndian(delta.Slice(4)) == 2;
        int size = ReadSize(delta, 12, maximumDecodedBytes);
        if (BinaryPrimitives.ReadUInt32LittleEndian(delta.Slice(8)) != (uint)baseSnapshot.Length ||
            !SHA256.HashData(baseSnapshot).AsSpan().SequenceEqual(delta.Slice(16, 32)))
        {
            throw Invalid("Delta base length or SHA-256 does not match.");
        }
        ValidateRecords(delta, size, baseSnapshot.Length);
        var result = new byte[size];
        baseSnapshot.Slice(0, Math.Min(baseSnapshot.Length, size)).CopyTo(result);
        int cursor = DeltaHeader;
        while (cursor < delta.Length)
        {
            int offset = (int)BinaryPrimitives.ReadUInt32LittleEndian(delta.Slice(cursor));
            int count = (int)BinaryPrimitives.ReadUInt32LittleEndian(delta.Slice(cursor + 4));
            if (!xor) { delta.Slice(cursor + 8, count).CopyTo(result.AsSpan(offset)); }
            else { for (int j = 0; j < count; j++) { result[offset + j] ^= delta[cursor + 8 + j]; } }
            cursor += 8 + count;
        }
        CheckDigest(result, delta.Slice(48, 32));
        return result;
    }

    /// <summary>
    /// Compresses arbitrary bytes, including full snapshots or deltas, with deterministic run-length encoding.
    /// Runs of 3–130 equal bytes use two bytes. Other bytes form literal packets of 1–128 bytes. Neither input nor output is platform-dependent.
    /// </summary>
    public static byte[] Compress(ReadOnlySpan<byte> bytes) => Compress(bytes, false);

    /// <summary>Compresses bytes with optional version-2 byte-plane transposition: each complete 64-byte tile is treated as an 8×8 byte matrix and transposed before run coding. The tail is unchanged. This can group equal high bytes without depending on a snapshot schema. Decode reverses the permutation and verifies the original digest.</summary>
    public static byte[] Compress(ReadOnlySpan<byte> bytes, bool bytePlanes)
    {
        using var output = new MemoryStream();
        WriteInt(output, CompressedMagic);
        WriteInt(output, bytePlanes ? 2u : 1u);
        WriteInt(output, (uint)bytes.Length);
        output.Write(SHA256.HashData(bytes));
        byte[]? transformed = bytePlanes ? bytes.ToArray() : null;
        if (transformed != null) { Transpose(transformed); bytes = transformed; }
        int i = 0;
        while (i < bytes.Length)
        {
            int run = RunLength(bytes, i);
            if (run >= 3)
            {
                output.WriteByte((byte)(128 + run - 3));
                output.WriteByte(bytes[i]);
                i += run;
            }
            else
            {
                int start = i++;
                while (i < bytes.Length && i - start < 128 && RunLength(bytes, i) < 3) { i++; }
                output.WriteByte((byte)(i - start - 1));
                output.Write(bytes.Slice(start, i - start));
            }
        }
        return output.ToArray();
    }

    /// <summary>Decodes a compressed snapshot or delta. Checks packet ranges before allocating, then checks SHA-256.</summary>
    /// <exception cref="SnapshotEncodingException">Invalid encoding, checksum or decoded size above the limit.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The output limit is negative.</exception>
    public static byte[] Decompress(ReadOnlySpan<byte> bytes, int maximumDecodedBytes = DefaultMaximumDecodedBytes)
    {
        ValidateHeader(bytes, CompressedMagic, CompressedHeader, true);
        int size = ReadSize(bytes, 8, maximumDecodedBytes);
        WalkPackets(bytes, size, Span<byte>.Empty);
        var result = new byte[size];
        WalkPackets(bytes, size, result);
        if (BinaryPrimitives.ReadUInt32LittleEndian(bytes.Slice(4)) == 2) { Transpose(result); }
        CheckDigest(result, bytes.Slice(12, 32));
        return result;
    }

    private static void WalkPackets(ReadOnlySpan<byte> bytes, int size, Span<byte> output)
    {
        int cursor = CompressedHeader, written = 0;
        while (cursor < bytes.Length)
        {
            byte token = bytes[cursor++];
            int count = token < 128 ? token + 1 : token - 128 + 3;
            int payload = token < 128 ? count : 1;
            if (count > size - written || payload > bytes.Length - cursor) { throw Invalid("Compressed packet range is invalid."); }
            if (!output.IsEmpty)
            {
                if (token < 128) { bytes.Slice(cursor, count).CopyTo(output.Slice(written)); }
                else { output.Slice(written, count).Fill(bytes[cursor]); }
            }
            cursor += payload;
            written += count;
        }
        if (written != size) { throw Invalid("Compressed decoded length does not match header."); }
    }

    private static void ValidateRecords(ReadOnlySpan<byte> delta, int size, int baseLength)
    {
        int cursor = DeltaHeader;
        uint previousEnd = 0;
        while (cursor < delta.Length)
        {
            if (delta.Length - cursor < 8) { throw Invalid("Delta record header is truncated."); }
            uint offset = BinaryPrimitives.ReadUInt32LittleEndian(delta.Slice(cursor));
            uint count = BinaryPrimitives.ReadUInt32LittleEndian(delta.Slice(cursor + 4));
            cursor += 8;
            if (offset < previousEnd || offset > Math.Max((uint)baseLength, previousEnd) || offset > (uint)size || count == 0 || count > (uint)size - offset || count > (uint)(delta.Length - cursor))
            { throw Invalid("Delta record range or ordering is invalid."); }
            cursor += (int)count;
            previousEnd = offset + count;
        }
        if (size > baseLength && previousEnd != (uint)size) { throw Invalid("Delta extension is not fully specified."); }
    }

    private static int RunLength(ReadOnlySpan<byte> bytes, int start)
    {
        int end = start + 1;
        while (end < bytes.Length && end - start < 130 && bytes[end] == bytes[start]) { end++; }
        return end - start;
    }

    private static int ReadSize(ReadOnlySpan<byte> bytes, int offset, int maximum)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(maximum);
        uint size = BinaryPrimitives.ReadUInt32LittleEndian(bytes.Slice(offset));
        if (size > (uint)maximum) { throw Invalid("Decoded length exceeds maximumDecodedBytes."); }
        return (int)size;
    }

    private static void ValidateHeader(ReadOnlySpan<byte> bytes, uint magic, int length, bool acceptVersion2 = false)
    {
        if (bytes.Length < length || BinaryPrimitives.ReadUInt32LittleEndian(bytes) != magic || (BinaryPrimitives.ReadUInt32LittleEndian(bytes.Slice(4)) != 1 && (!acceptVersion2 || BinaryPrimitives.ReadUInt32LittleEndian(bytes.Slice(4)) != 2)))
        { throw Invalid("Snapshot encoding header is truncated or has unknown magic/version."); }
    }

    // Transposition is its own inverse; complete tiles only, so all reads/writes stay within a fixed 64-byte window.
    private static void Transpose(Span<byte> bytes)
    {
        for (int start = 0; bytes.Length - start >= 64; start += 64)
        {
            for (int row = 0; row < 8; row++)
            {
                for (int col = row + 1; col < 8; col++)
                {
                    int a = start + row * 8 + col, b = start + col * 8 + row;
                    (bytes[a], bytes[b]) = (bytes[b], bytes[a]);
                }
            }
        }
    }

    private static void CheckDigest(ReadOnlySpan<byte> bytes, ReadOnlySpan<byte> digest)
    {
        if (!SHA256.HashData(bytes).AsSpan().SequenceEqual(digest)) { throw Invalid("Decoded SHA-256 does not match."); }
    }

    private static void WriteInt(Stream output, uint value)
    {
        output.WriteByte((byte)value);
        output.WriteByte((byte)(value >> 8));
        output.WriteByte((byte)(value >> 16));
        output.WriteByte((byte)(value >> 24));
    }

    private static SnapshotEncodingException Invalid(string message) => new(message);
}
