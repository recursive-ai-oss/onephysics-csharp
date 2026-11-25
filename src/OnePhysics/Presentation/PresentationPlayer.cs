using System;
using System.Buffers.Binary;
using System.IO;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Presentation;

/// <summary>What a follower world offers a player: restoring a recorded snapshot and applying one recorded frame.</summary>
internal interface IPresentationPlayback : IPresentationHost
{
    /// <summary>Restores the snapshot and live options of a snapshot record.</summary>
    void RestoreRecorded(ReadOnlySpan<byte> options, ReadOnlySpan<byte> snapshot);

    /// <summary>Applies the poses of a frame, steps, enforces the speed bound and returns the world hash. Throws <see cref="PresentationFrameException"/> for a frame that does not fit the world.</summary>
    ulong ApplyFrame(ReadOnlySpan<byte> frame);
}

/// <summary>
/// Replays a presentation recording (<see cref="PresentationRecorder"/>) into a follower world built from it, without the authoritative world, and checks the world hash after every step.
/// Create the follower with <see cref="CreateFollower2D"/> or <see cref="CreateFollower3D"/>, then call <see cref="ReadNextStep"/> until it returns false.
/// Malformed input raises <see cref="ReplayFormatException"/> naming the byte offset; a different hash raises <see cref="ReplayHashMismatchException"/> with the step count.
/// Reading is linear in the bytes read, a declared length is checked against <c>maximumPayloadBytes</c> before any allocation, and nothing recurses on input.
/// </summary>
public sealed class PresentationPlayer
{
    private readonly Stream _stream;
    private readonly string _name;
    private readonly int _maxPayload;
    private readonly byte[] _head = new byte[PresentationFormat.HeaderSize];
    private byte[] _payload = new byte[4096];
    private int _payloadLength;
    private long _offset;
    private long _recordOffset;
    private byte _kind;
    private IPresentationPlayback? _host;
    private bool _finished, _failed;

    /// <summary>Reads and validates the 16-byte header.</summary>
    /// <param name="stream">A readable stream positioned at the start of the recording. Left open.</param>
    /// <param name="inputName">Used in error messages. Default "presentation".</param>
    /// <param name="maximumPayloadBytes">The largest record accepted, 1 to 1 GiB. Default 64 MiB.</param>
    /// <exception cref="ReplayFormatException">The header is truncated, has another magic, version, dimension or flags, or a negative step count.</exception>
    public PresentationPlayer(Stream stream, string inputName = "presentation", int maximumPayloadBytes = PresentationFormat.DefaultMaxPayload)
    {
        ArgumentNullException.ThrowIfNull(stream);
        ArgumentNullException.ThrowIfNull(inputName);
        if (!stream.CanRead) throw new ArgumentException("The stream must be readable.", nameof(stream));
        if (maximumPayloadBytes < 1 || maximumPayloadBytes > 1 << 30) throw new ArgumentOutOfRangeException(nameof(maximumPayloadBytes), maximumPayloadBytes, "The payload bound must be 1 to 1073741824.");
        _stream = stream; _name = inputName; _maxPayload = maximumPayloadBytes;
        ReadExactly(_head, 0, "a 16-byte header");
        if (BinaryPrimitives.ReadUInt32LittleEndian(_head) != PresentationFormat.Magic) throw Error(0, "the magic OPPF");
        if (BinaryPrimitives.ReadUInt16LittleEndian(_head.AsSpan(4)) != PresentationFormat.Version) throw Error(4, "version 1");
        Dimension = _head[6];
        if (Dimension != 2 && Dimension != 3) throw Error(6, "dimension 2 or 3");
        if (_head[7] != 0) throw Error(7, "flags 0");
        InitialStepCount = BinaryPrimitives.ReadInt64LittleEndian(_head.AsSpan(8));
        if (InitialStepCount < 0) throw Error(8, "a nonnegative step count");
    }

    /// <summary>2 or 3.</summary>
    public int Dimension { get; }

    /// <summary>The world's completed step count when recording started.</summary>
    public long InitialStepCount { get; }

    /// <summary>Steps replayed so far.</summary>
    public long StepsPlayed { get; private set; }

    /// <summary>The world hash after the last replayed step, 0 before the first.</summary>
    public ulong LastHash { get; private set; }

    /// <summary>The world's completed step count after the last replayed step.</summary>
    public long StepCount => _host?.StepCount ?? InitialStepCount;

    private ReplayFormatException Error(long offset, string expected) { _failed = true; return new ReplayFormatException(_name, offset, expected); }

    private void ReadExactly(byte[] buffer, int count, string what)
    {
        int got = 0;
        int total = count == 0 ? buffer.Length : count;
        while (got < total)
        {
            int n = _stream.Read(buffer, got, total - got);
            if (n <= 0) throw Error(_offset, what + " (the stream ended)");
            got += n; _offset += n;
        }
    }

    private bool ReadRecord()
    {
        _recordOffset = _offset;
        Span<byte> h = _head.AsSpan(0, PresentationFormat.RecordHeaderSize);
        int got = 0;
        while (got < h.Length)
        {
            int n = _stream.Read(_head, got, h.Length - got);
            if (n <= 0) throw Error(_offset, "a record header or the terminator (the stream ended)");
            got += n; _offset += n;
        }

        _kind = h[0];
        int length = BinaryPrimitives.ReadInt32LittleEndian(h.Slice(1));
        if (length < 0 || length > _maxPayload) throw Error(_recordOffset + 1, "a payload length from 0 to " + _maxPayload.ToString(System.Globalization.CultureInfo.InvariantCulture));
        if (_payload.Length < length) _payload = new byte[Math.Min(_maxPayload, Math.Max(length, _payload.Length * 2))];
        int read = 0;
        while (read < length)
        {
            int n = _stream.Read(_payload, read, length - read);
            if (n <= 0) throw Error(_offset, "a record payload of " + length.ToString(System.Globalization.CultureInfo.InvariantCulture) + " bytes (the stream ended)");
            read += n; _offset += n;
        }

        _payloadLength = length;
        return true;
    }

    /// <summary>Reads the first record and builds a planar follower from it, with the recorded world options and the given thread count.</summary>
    /// <exception cref="InvalidOperationException">The recording is spatial, or the follower was already created.</exception>
    /// <exception cref="ReplayFormatException">The first record is not a valid snapshot.</exception>
    public PresentationWorld2D CreateFollower2D(int threadCount = 1)
    {
        Prepare(2);
        var (options, snapshot) = ReadInitial();
        try
        {
            var o = new WorldOptions2D { ThreadCount = threadCount };
            o.BroadphaseMargin = ReplayOptions2D.Read(options, o, out int id); o.WorldId = id;
            ReplayOptions2D.PrepareCreation(o);
            var follower = new PresentationWorld2D(o);
            _host = follower;
            ((IPresentationPlayback)follower).RestoreRecorded(options, snapshot);
            CheckInitial(follower);
            return follower;
        }
        catch (Exception e) when (e is not ReplayFormatException && (e is SnapshotFormatException or ArgumentException or InvalidOperationException)) { throw Error(_recordOffset + 5, "a valid initial snapshot and options: " + e.Message); }
    }

    /// <summary>Reads the first record and builds a spatial follower from it, with the recorded world options and the given thread count.</summary>
    /// <exception cref="InvalidOperationException">The recording is planar, or the follower was already created.</exception>
    /// <exception cref="ReplayFormatException">The first record is not a valid snapshot.</exception>
    public PresentationWorld3D CreateFollower3D(int threadCount = 1)
    {
        Prepare(3);
        var (options, snapshot) = ReadInitial();
        try
        {
            var o = new WorldOptions3D { ThreadCount = threadCount };
            o.BroadphaseMargin = ReplayOptions3D.Read(options, o, out int id); o.WorldId = id;
            ReplayOptions3D.PrepareCreation(o);
            var follower = new PresentationWorld3D(o);
            _host = follower;
            ((IPresentationPlayback)follower).RestoreRecorded(options, snapshot);
            CheckInitial(follower);
            return follower;
        }
        catch (Exception e) when (e is not ReplayFormatException && (e is SnapshotFormatException or ArgumentException or InvalidOperationException)) { throw Error(_recordOffset + 5, "a valid initial snapshot and options: " + e.Message); }
    }

    private void Prepare(int dimension)
    {
        if (_failed || _host != null) throw new InvalidOperationException("Create the follower once, before playback starts, and not after a failure.");
        if (Dimension != dimension) throw new InvalidOperationException("The recording is " + Dimension.ToString(System.Globalization.CultureInfo.InvariantCulture) + "D.");
    }

    private void CheckInitial(IPresentationPlayback follower)
    {
        if (follower.StepCount != InitialStepCount) throw Error(_recordOffset + 5, "an initial snapshot at step " + InitialStepCount.ToString(System.Globalization.CultureInfo.InvariantCulture));
    }

    private (byte[] Options, byte[] Snapshot) ReadInitial()
    {
        try
        {
            ReadRecord();
            if (_kind != PresentationFormat.KindSnapshot) throw Error(_recordOffset, "an initial snapshot record");
            return SplitSnapshot();
        }
        catch { _failed = true; throw; }
    }

    private (byte[] Options, byte[] Snapshot) SplitSnapshot()
    {
        if (_payloadLength < 4) throw Error(_recordOffset + 5, "an options length");
        int optionsSize = BinaryPrimitives.ReadInt32LittleEndian(_payload);
        // The live options come in a few sizes: the base layout and the ones that add soft-body options or the length unit.
        bool valid = Dimension == 3 ? ReplayOptions3D.IsValidSize(optionsSize) : ReplayOptions2D.IsValidSize(optionsSize);
        if (!valid) throw Error(_recordOffset + 5, "a live-options block of a known size");
        if (_payloadLength < 4 + optionsSize + 1) throw Error(_recordOffset + 5, "the live options and a snapshot");
        var options = _payload.AsSpan(4, optionsSize).ToArray();
        var snapshot = _payload.AsSpan(4 + optionsSize, _payloadLength - 4 - optionsSize).ToArray();
        return (options, snapshot);
    }

    /// <summary>Replays records up to and including the next step and checks its hash.</summary>
    /// <returns>True after a step; false at the terminator.</returns>
    /// <exception cref="InvalidOperationException">The follower has not been created, or an earlier call failed.</exception>
    /// <exception cref="ReplayFormatException">The input is malformed or a frame does not fit the world.</exception>
    /// <exception cref="ReplayHashMismatchException">The world hash after a step differs from the recorded one; the world stays available for inspection.</exception>
    public bool ReadNextStep()
    {
        if (_failed || _host == null) throw new InvalidOperationException("Create the follower first, and do not continue after a failure.");
        if (_finished) return false;
        try
        {
            while (true)
            {
                ReadRecord();
                switch (_kind)
                {
                    case PresentationFormat.KindEnd:
                        if (_payloadLength != 0) throw Error(_recordOffset + 1, "a zero-length terminator");
                        _finished = true;
                        return false;
                    case PresentationFormat.KindSnapshot:
                        {
                            var (options, snapshot) = SplitSnapshot();
                            try { _host.RestoreRecorded(options, snapshot); }
                            catch (Exception e) when (e is SnapshotFormatException or ArgumentException) { throw Error(_recordOffset + 5, "a valid snapshot and options: " + e.Message); }
                            break;
                        }
                    case PresentationFormat.KindFrame:
                        {
                            if (_payloadLength < PresentationFormat.FramePrologue + 8) throw Error(_recordOffset + 5, "a frame of at least 20 bytes");
                            ulong expected = BinaryPrimitives.ReadUInt64LittleEndian(_payload.AsSpan(_payloadLength - 8));
                            ulong actual;
                            try { actual = _host.ApplyFrame(_payload.AsSpan(0, _payloadLength - 8)); }
                            catch (PresentationFrameException e) { throw Error(_recordOffset + 5, e.Message); }
                            StepsPlayed++; LastHash = actual;
                            if (actual != expected) { _failed = true; throw new ReplayHashMismatchException(_host.StepCount, expected, actual); }
                            return true;
                        }
                    default:
                        throw Error(_recordOffset, "a record kind of 0, 1 or 2");
                }
            }
        }
        catch (ReplayFormatException) { _failed = true; throw; }
    }
}
