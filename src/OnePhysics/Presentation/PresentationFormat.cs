using System;
using System.Buffers.Binary;

namespace RecursiveAI.OnePhysics.Presentation;

/// <summary>What a presentation recording needs from a follower world, in either dimension.</summary>
internal interface IPresentationHost
{
    int Dimension { get; }
    long StepCount { get; }
    ulong Hash();
    byte[] Snapshot();
    int OptionsMaxSize { get; }
    int WriteOptions(Span<byte> bytes);
    void AttachRecorder(PresentationRecorder? recorder);
}

/// <summary>Layout constants of the presentation recording (docs/spec/presentation.md).</summary>
internal static class PresentationFormat
{
    public const uint Magic = 0x4650504F;      // "OPPF" little-endian
    public const ushort Version = 1;
    public const int HeaderSize = 16;
    public const byte KindEnd = 0, KindSnapshot = 1, KindFrame = 2;
    public const int RecordHeaderSize = 5;
    public const int DefaultMaxPayload = 64 * 1024 * 1024;

    /// <summary>Bytes of the frame prologue: dt, maximum speed, pose count.</summary>
    public const int FramePrologue = 12;

    /// <summary>Bytes of one pose entry: slot, generation, flags, position, rotation.</summary>
    public static int PoseSize(int dimension) => dimension == 3 ? 4 + 4 + 1 + 24 + 16 : 4 + 4 + 1 + 16 + 4;

    public static void WriteHeader(Span<byte> h, int dimension, long stepCount)
    {
        BinaryPrimitives.WriteUInt32LittleEndian(h, Magic);
        BinaryPrimitives.WriteUInt16LittleEndian(h.Slice(4), Version);
        h[6] = (byte)dimension;
        h[7] = 0;
        BinaryPrimitives.WriteInt64LittleEndian(h.Slice(8), stepCount);
    }
}

/// <summary>Reasons a frame from a recording cannot be applied; the player turns it into a <see cref="Replay.ReplayFormatException"/> with the byte offset.</summary>
internal sealed class PresentationFrameException : Exception
{
    public PresentationFrameException(string expected) : base(expected) { }
}
