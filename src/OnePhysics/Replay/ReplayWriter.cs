using System;
using System.Buffers.Binary;
using System.IO;

namespace RecursiveAI.OnePhysics.Replay;

/// <summary>
/// Writes version 1 checkpoint replays to a writable stream without seeking or reflection. The caller owns the stream.
/// After construction, writes allocate no managed memory in this class. The stream may allocate independently.
/// This is a raw transport: callers provide snapshots and steps; it does not intercept world mutations.
/// A stream error makes the output incomplete; discard the writer and its output after an I/O failure.
/// </summary>
public sealed class ReplayWriter
{
    internal const uint Magic = 0x5250504F;
    internal const ushort Version = 1;
    private readonly Stream _stream;
    private readonly byte[] _scratch = new byte[16];
    private readonly bool _hashTrack;
    private bool _ended;
    private long _completedSteps;

    /// <summary>
    /// Writes a header and initial snapshot. Dimension must be 2 or 3; initialStep is nonnegative and must match the snapshot's step count.
    /// Snapshot bytes are opaque here and must be a complete snapshot of that dimension. No bytes are written for invalid arguments.
    /// </summary>
    /// <param name="stream">Writable stream, left open; need not seek.</param>
    /// <param name="dimension">2 or 3.</param>
    /// <param name="initialStep">Absolute count of steps completed by the starting world.</param>
    /// <param name="initialSnapshot">Nonempty snapshot, unchanged in the output.</param>
    /// <param name="hashTrack">True by default: each step carries its resulting 64-bit world hash.</param>
    /// <exception cref="ArgumentException">The stream is not writable, or the snapshot is empty.</exception>
    /// <exception cref="ArgumentOutOfRangeException">Dimension or initial step is invalid.</exception>
    public ReplayWriter(Stream stream, int dimension, long initialStep, ReadOnlySpan<byte> initialSnapshot, bool hashTrack = true)
    {
        ArgumentNullException.ThrowIfNull(stream);
        if (!stream.CanWrite) { throw new ArgumentException("Replay output must be writable.", nameof(stream)); }
        if (dimension != 2 && dimension != 3) { throw new ArgumentOutOfRangeException(nameof(dimension), dimension, "Expected dimension 2 or 3."); }
        ArgumentOutOfRangeException.ThrowIfNegative(initialStep);
        if (initialSnapshot.IsEmpty) { throw new ArgumentException("Expected a nonempty initial snapshot.", nameof(initialSnapshot)); }
        _stream = stream;
        _hashTrack = hashTrack;
        _completedSteps = initialStep;
        BinaryPrimitives.WriteUInt32LittleEndian(_scratch, Magic);
        BinaryPrimitives.WriteUInt16LittleEndian(_scratch.AsSpan(4), Version);
        _scratch[6] = (byte)dimension;
        _scratch[7] = hashTrack ? (byte)1 : (byte)0;
        BinaryPrimitives.WriteInt64LittleEndian(_scratch.AsSpan(8), initialStep);
        _stream.Write(_scratch);
        WriteSnapshot(initialSnapshot);
    }

    /// <summary>The absolute completed step count, initially the constructor's initialStep.</summary>
    public long CompletedSteps => _completedSteps;

    /// <summary>
    /// Writes a full snapshot at the current completed step, for a seek keyframe or to replace state changed between steps.
    /// The bytes are not inspected. The caller must synchronize live options with the world state before taking the snapshot.
    /// </summary>
    /// <exception cref="ArgumentException">Snapshot is empty.</exception>
    /// <exception cref="InvalidOperationException">The terminator was already written.</exception>
    public void WriteSnapshot(ReadOnlySpan<byte> snapshot)
    {
        EnsureOpen();
        if (snapshot.IsEmpty) { throw new ArgumentException("Expected a nonempty snapshot.", nameof(snapshot)); }
        Header(ReplayRecordKind.Snapshot, snapshot.Length);
        _stream.Write(snapshot);
    }

    /// <summary>Writes a completed step (seconds) and its resulting hash. The hash is omitted if hashTrack was false at construction.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Time is not positive and finite.</exception>
    /// <exception cref="InvalidOperationException">The stream is completed or the absolute step count would overflow Int64.</exception>
    public void WriteStep(float dt, ulong hash = 0)
    {
        EnsureOpen();
        if (!float.IsFinite(dt) || dt <= 0) { throw new ArgumentOutOfRangeException(nameof(dt), dt, "Expected positive finite seconds."); }
        if (_completedSteps == long.MaxValue) { throw new InvalidOperationException("Replay step count cannot exceed Int64.MaxValue."); }
        int length = _hashTrack ? 12 : 4;
        Header(ReplayRecordKind.Step, length);
        BinaryPrimitives.WriteSingleLittleEndian(_scratch, dt);
        if (_hashTrack) { BinaryPrimitives.WriteUInt64LittleEndian(_scratch.AsSpan(4), hash); }
        _stream.Write(_scratch.AsSpan(0, length));
        _completedSteps++;
    }

    /// <summary>Writes the mandatory terminator once. Repeated calls do nothing. Does not flush or close the caller's stream.</summary>
    public void Complete()
    {
        if (_ended) { return; }
        Header(ReplayRecordKind.End, 0);
        _ended = true;
    }

    private void Header(ReplayRecordKind kind, int length)
    {
        _scratch[0] = (byte)kind;
        BinaryPrimitives.WriteInt32LittleEndian(_scratch.AsSpan(1), length);
        _stream.Write(_scratch.AsSpan(0, 5));
    }

    private void EnsureOpen()
    {
        if (_ended) { throw new InvalidOperationException("Replay is already completed."); }
    }
}
