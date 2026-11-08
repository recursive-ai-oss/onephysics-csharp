using System;
using System.Buffers.Binary;
using System.IO;

namespace RecursiveAI.OnePhysics.Replay;

/// <summary>
/// Incrementally reads version 1 checkpoint and version 2 command replays, including from non-seekable streams which return short reads.
/// Only one record is buffered. Snapshot payloads are opaque and require the world's snapshot validator before use.
/// No reflection is used. The caller owns the stream. A read or format error makes the reader unusable.
/// </summary>
public sealed class ReplayReader
{
    /// <summary>Default maximum snapshot payload: 64 MiB. Larger trusted worlds may explicitly raise this bound.</summary>
    public const int DefaultMaximumPayloadBytes = 64 * 1024 * 1024;

    private readonly Stream _stream;
    private readonly string _inputName;
    private readonly int _maximumPayloadBytes;
    private readonly byte[] _scratch = new byte[16];
    private byte[] _payload;
    private int _payloadLength;
    private long _offset;
    private bool _initial = true;
    private bool _ended;
    private bool _failed;
    private readonly ushort _version;
    private bool _frameOpen;

    /// <summary>
    /// Reads and validates the 16-byte header. The default payload bound is 64 MiB; initial buffer capacity is 4096 bytes (or the bound if smaller).
    /// Allocation is bounded by the largest accepted record. Repeated reads allocate nothing once the buffer has reached that size.
    /// </summary>
    /// <exception cref="ArgumentException">Stream is not readable.</exception>
    /// <exception cref="ArgumentOutOfRangeException">Maximum payload is less than 12 bytes.</exception>
    /// <exception cref="ReplayFormatException">Header is truncated, unsupported, or invalid.</exception>
    public ReplayReader(Stream stream, string inputName = "replay", int maximumPayloadBytes = DefaultMaximumPayloadBytes)
    {
        ArgumentNullException.ThrowIfNull(stream);
        ArgumentNullException.ThrowIfNull(inputName);
        if (!stream.CanRead) { throw new ArgumentException("Replay input must be readable.", nameof(stream)); }
        if (maximumPayloadBytes < 12) { throw new ArgumentOutOfRangeException(nameof(maximumPayloadBytes), maximumPayloadBytes, "Expected at least 12 bytes."); }
        _stream = stream;
        _inputName = inputName;
        _maximumPayloadBytes = maximumPayloadBytes;
        _payload = new byte[Math.Min(4096, maximumPayloadBytes)];
        ReadExactly(_scratch);
        if (BinaryPrimitives.ReadUInt32LittleEndian(_scratch) != ReplayWriter.Magic) { throw Error(0, "OPPR magic"); }
        _version = BinaryPrimitives.ReadUInt16LittleEndian(_scratch.AsSpan(4));
        if (_version != 1 && _version != 2) { throw Error(4, "replay version 1 or 2"); }
        Dimension = _scratch[6];
        if (Dimension != 2 && Dimension != 3) { throw Error(6, "dimension 2 or 3"); }
        if (_scratch[7] > 1) { throw Error(7, "flags 0 or 1 (optional hash track)"); }
        HasHashTrack = _scratch[7] == 1;
        CompletedSteps = BinaryPrimitives.ReadInt64LittleEndian(_scratch.AsSpan(8));
        if (CompletedSteps < 0) { throw Error(8, "nonnegative initial step"); }
        InitialStep = CompletedSteps;
    }

    /// <summary>The envelope version, 1 or 2.</summary>
    public int FormatVersion => _version;

    /// <summary>2 for planar worlds, 3 for spatial worlds.</summary>
    public int Dimension { get; }

    /// <summary>Whether each step contains its resulting hash.</summary>
    public bool HasHashTrack { get; }

    /// <summary>The starting absolute completed step count from the header.</summary>
    public long InitialStep { get; }

    /// <summary>The completed step count after the current record, or the starting count before any records.</summary>
    public long CompletedSteps { get; private set; }

    /// <summary>The byte offset of the current record header, relative to the start of this replay.</summary>
    public long RecordOffset { get; private set; }

    /// <summary>The current record kind. Valid only after ReadNext returns true.</summary>
    public ReplayRecordKind Kind { get; private set; }

    /// <summary>The current record's payload. Valid until the next ReadNext call. Do not retain it without copying.</summary>
    public ReadOnlySpan<byte> Payload => _payload.AsSpan(0, _payloadLength);

    /// <summary>Current step duration in seconds. Throws InvalidOperationException unless the current record is a Step.</summary>
    public float StepTime => (Kind == ReplayRecordKind.Step || Kind == ReplayRecordKind.BeginStep) && _payloadLength > 0
        ? BinaryPrimitives.ReadSingleLittleEndian(Payload) : throw new InvalidOperationException("Expected a current step record.");

    /// <summary>Current step's expected world hash. Null without a hash track. Throws InvalidOperationException unless the current record is a Step.</summary>
    public ulong? StepHash => (Kind == ReplayRecordKind.Step || Kind == ReplayRecordKind.EndStep) && _payloadLength > 0
        ? HasHashTrack ? BinaryPrimitives.ReadUInt64LittleEndian(Payload.Slice(Kind == ReplayRecordKind.Step ? 4 : 8)) : null
        : throw new InvalidOperationException("Expected a current step record.");

    /// <summary>The snapshot portion of the current Snapshot or Mutation record. Borrowed until the next read; no allocation.</summary>
    public ReadOnlySpan<byte> SnapshotPayload => Kind == ReplayRecordKind.Mutation
        ? Payload.Slice(10 + BinaryPrimitives.ReadUInt16LittleEndian(Payload.Slice(8)))
        : Kind == ReplayRecordKind.Snapshot ? Payload : throw new InvalidOperationException("Expected snapshot or mutation.");

    /// <summary>The current mutation's UTF-8 operation name, borrowed without allocating. Empty for other records.</summary>
    public ReadOnlySpan<byte> OperationName => Kind == ReplayRecordKind.Mutation && _payloadLength > 0
        ? Payload.Slice(10, BinaryPrimitives.ReadUInt16LittleEndian(Payload.Slice(8))) : ReadOnlySpan<byte>.Empty;

    /// <summary>
    /// Reads one complete record. Returns true for snapshots and steps; returns false on the mandatory terminator and on subsequent calls.
    /// End-of-stream without a terminator is an error. Bytes following the terminator are left unread, so replays may be concatenated.
    /// On failure the current payload is invalidated. Lengths are checked before allocating or reading a payload.
    /// </summary>
    /// <exception cref="ReplayFormatException">Truncation, invalid length, kind, order, time step, or step count.</exception>
    /// <exception cref="InvalidOperationException">A prior read failed.</exception>
    public bool ReadNext()
    {
        if (_failed) { throw new InvalidOperationException("Replay reader cannot continue after a failed read."); }
        if (_ended) { return false; }
        _payloadLength = 0;
        try
        {
            RecordOffset = _offset;
            ReadExactly(_scratch.AsSpan(0, 5));
            Kind = (ReplayRecordKind)_scratch[0];
            int length = BinaryPrimitives.ReadInt32LittleEndian(_scratch.AsSpan(1));
            if (length < 0 || length > _maximumPayloadBytes) { throw Error(RecordOffset + 1, $"payload length in [0, {_maximumPayloadBytes}]"); }
            if (_initial && Kind != ReplayRecordKind.Snapshot) { throw Error(RecordOffset, "initial snapshot record"); }
            switch (Kind)
            {
                case ReplayRecordKind.End:
                    if (_frameOpen) throw Error(RecordOffset, "EndStep before terminator");
                    if (length != 0) { throw Error(RecordOffset + 1, "zero-length terminator"); }
                    _ended = true;
                    return false;
                case ReplayRecordKind.Snapshot:
                    if (_frameOpen) throw Error(RecordOffset, "mutation, not checkpoint, inside command frame");
                    if (length == 0) { throw Error(RecordOffset + 1, "nonempty snapshot payload"); }
                    break;
                case ReplayRecordKind.Step:
                    if (_frameOpen) throw Error(RecordOffset, "EndStep before another step");
                    if (length != (HasHashTrack ? 12 : 4)) { throw Error(RecordOffset + 1, HasHashTrack ? "12-byte step payload" : "4-byte step payload"); }
                    if (CompletedSteps == long.MaxValue) { throw Error(RecordOffset, "step count within Int64 range"); }
                    break;
                case ReplayRecordKind.LiveOptions:
                    if (_version != 2 || length != (Dimension == 2 ? ReplayOptions2D.Size : ReplayOptions3D.Size)) throw Error(RecordOffset, "version 2 live option payload for this dimension");
                    break;
                case ReplayRecordKind.BeginStep:
                    if (_version != 2 || _frameOpen || length != 4 || CompletedSteps == long.MaxValue) throw Error(RecordOffset, "version 2 BeginStep with four bytes outside a frame");
                    break;
                case ReplayRecordKind.Mutation:
                    if (_version != 2 || length < 12) throw Error(RecordOffset, "version 2 named mutation with nonempty snapshot");
                    break;
                case ReplayRecordKind.EndStep:
                    if (_version != 2 || !_frameOpen || length != (HasHashTrack ? 16 : 8)) throw Error(RecordOffset, "matching version 2 EndStep");
                    break;
                default: throw Error(RecordOffset, "known record kind");
            }
            if (_payload.Length < length) { _payload = new byte[length]; }
            ReadExactly(_payload.AsSpan(0, length));
            if (Kind == ReplayRecordKind.Step || Kind == ReplayRecordKind.BeginStep)
            {
                float dt = BinaryPrimitives.ReadSingleLittleEndian(_payload);
                if (!float.IsFinite(dt) || dt <= 0) { throw Error(RecordOffset + 5, "positive finite float32 seconds"); }
                CompletedSteps++;
                if (Kind == ReplayRecordKind.BeginStep) _frameOpen = true;
            }
            if (Kind == ReplayRecordKind.Mutation)
            {
                long count = BinaryPrimitives.ReadInt64LittleEndian(_payload);
                int name = BinaryPrimitives.ReadUInt16LittleEndian(_payload.AsSpan(8));
                if (count < 0 || name < 1 || name > 512 || 10 + name >= length) throw Error(RecordOffset + 5, "valid mutation count, name and nonempty snapshot");
                for (int i = 10; i < 10 + name; i++) if (_payload[i] < 32 || _payload[i] > 126) throw Error(RecordOffset + 5 + i, "ASCII operation name");
                CompletedSteps = count;
            }
            if (Kind == ReplayRecordKind.EndStep)
            {
                long count = BinaryPrimitives.ReadInt64LittleEndian(_payload);
                if (count < 0) throw Error(RecordOffset + 5, "nonnegative world count");
                CompletedSteps = count;
                _frameOpen = false;
            }
            _initial = false;
            _payloadLength = length;
            return true;
        }
        catch
        {
            _failed = true;
            _payloadLength = 0;
            throw;
        }
    }

    private void ReadExactly(Span<byte> destination)
    {
        while (!destination.IsEmpty)
        {
            int read = _stream.Read(destination);
            if (read == 0) { throw Error(_offset, "more bytes (truncated replay)"); }
            _offset += read;
            destination = destination.Slice(read);
        }
    }

    private ReplayFormatException Error(long offset, string expected) => new(_inputName, offset, expected);
}
