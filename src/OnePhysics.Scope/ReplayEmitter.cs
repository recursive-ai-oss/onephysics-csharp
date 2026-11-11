using System;
using System.Buffers;
using System.Buffers.Binary;
using System.Text;

namespace RecursiveAI.OnePhysics.Scope;

/// <summary>
/// Writes a OnePhysics command replay (version 2) to a buffer. The record framing is the published replay format; this class
/// exists because the library's own writer cannot be fed a world's live-option record from outside the library.
/// </summary>
internal sealed class ReplayEmitter
{
    private const uint Magic = 0x5250504F;
    private readonly IBufferWriter<byte> _out;

    public ReplayEmitter(IBufferWriter<byte> output, int dimension, long initialStep)
    {
        _out = output;
        Span<byte> h = output.GetSpan(16);
        BinaryPrimitives.WriteUInt32LittleEndian(h, Magic);
        BinaryPrimitives.WriteUInt16LittleEndian(h[4..], 2);
        h[6] = (byte)dimension;
        h[7] = 1; // every step carries its hash
        BinaryPrimitives.WriteInt64LittleEndian(h[8..], initialStep);
        output.Advance(16);
    }

    public void Snapshot(ReadOnlySpan<byte> snapshot) => Record(1, snapshot);

    public void LiveOptions(ReadOnlySpan<byte> options) => Record(6, options);

    public void BeginStep(float dt)
    {
        Span<byte> p = stackalloc byte[4];
        BinaryPrimitives.WriteSingleLittleEndian(p, dt);
        Record(3, p);
    }

    public void Mutation(string name, long worldStep, ReadOnlySpan<byte> snapshot)
    {
        byte[] n = Encoding.ASCII.GetBytes(name);
        Header(4, 10 + n.Length + snapshot.Length);
        Span<byte> p = stackalloc byte[10];
        BinaryPrimitives.WriteInt64LittleEndian(p, worldStep);
        BinaryPrimitives.WriteUInt16LittleEndian(p[8..], (ushort)n.Length);
        Append(p);
        Append(n);
        Append(snapshot);
    }

    public void EndStep(long worldStep, ulong hash)
    {
        Span<byte> p = stackalloc byte[16];
        BinaryPrimitives.WriteInt64LittleEndian(p, worldStep);
        BinaryPrimitives.WriteUInt64LittleEndian(p[8..], hash);
        Record(5, p);
    }

    public void End() => Record(0, default);

    private void Record(byte kind, ReadOnlySpan<byte> payload)
    {
        Header(kind, payload.Length);
        Append(payload);
    }

    private void Header(byte kind, int length)
    {
        Span<byte> h = _out.GetSpan(5);
        h[0] = kind;
        BinaryPrimitives.WriteInt32LittleEndian(h[1..], length);
        _out.Advance(5);
    }

    private void Append(ReadOnlySpan<byte> bytes)
    {
        while (!bytes.IsEmpty)
        {
            Span<byte> span = _out.GetSpan(Math.Min(bytes.Length, 1 << 16));
            int n = Math.Min(span.Length, bytes.Length);
            bytes[..n].CopyTo(span);
            _out.Advance(n);
            bytes = bytes[n..];
        }
    }
}
