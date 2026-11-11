using System;
using System.Buffers;
using System.Buffers.Binary;
using System.IO;
using OneScope.Sdk;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Scope;

/// <summary>Receives one live record: complete replay records the viewer applies as a unit (a world mutation, or a whole step).</summary>
public delegate void LiveRecordHandler(ReadOnlySpan<byte> record);

/// <summary>
/// The host side of a live attach. Give it to <c>world.StartRecording(feed)</c> and pass <see cref="WriteSnapshot"/> to the probe as its
/// snapshot source; set <see cref="Record"/> to the probe's record call. The feed regroups the recorder's writes into whole records:
/// each world mutation, and each simulated step (from its start to its end, including what callbacks changed in it) arrive as one record,
/// after the host has applied them, which is the moment the probe wants them. The recorder's own header, initial snapshot and
/// option records are not forwarded: the snapshot the probe takes carries that state.
/// </summary>
public sealed class LiveFeedStream : Stream
{
    private const int HeaderBytes = 16;
    private const int RecordHeaderBytes = 5;
    private const int MaxOptionBytes = 4096;
    private static ReadOnlySpan<byte> Magic => "OPLV"u8;

    private readonly Func<byte[]> _snapshot;
    private readonly Func<long> _stepCount;
    private readonly int _dimension;
    private readonly MemoryStream _pending = new();
    private readonly MemoryStream _frame = new();
    private byte[]? _options;
    private bool _headerDone;
    private bool _inFrame;
    private int _skipRecords = 1; // the initial snapshot

    /// <summary>Makes a feed for a planar world.</summary>
    public LiveFeedStream(PhysicsWorld2D world)
        : this(world is null ? throw new ArgumentNullException(nameof(world)) : world.Snapshot, () => world.StepCount, 2)
    {
    }

    /// <summary>Makes a feed for a spatial world.</summary>
    public LiveFeedStream(PhysicsWorld3D world)
        : this(world is null ? throw new ArgumentNullException(nameof(world)) : world.Snapshot, () => world.StepCount, 3)
    {
    }

    private LiveFeedStream(Func<byte[]> snapshot, Func<long> stepCount, int dimension)
    {
        _snapshot = snapshot;
        _stepCount = stepCount;
        _dimension = dimension;
    }

    /// <summary>Where finished records go: the probe's record call. Records are dropped while it is null.</summary>
    public LiveRecordHandler? Record { get; set; }

    /// <summary>
    /// Writes what the module needs to open the stream: the world's snapshot with its step count and the options the recorder wrote.
    /// Call it from the probe's snapshot source; the world is then exactly as it is after the last record passed to <see cref="Record"/>.
    /// </summary>
    /// <exception cref="InvalidOperationException">The world is not being recorded into this feed yet.</exception>
    public void WriteSnapshot(IBufferWriter<byte> writer)
    {
        ArgumentNullException.ThrowIfNull(writer);
        if (_options is null)
        {
            throw new InvalidOperationException("Start the world's recording into this feed before attaching a viewer.");
        }

        byte[] snapshot = _snapshot();
        Span<byte> head = writer.GetSpan(17);
        Magic.CopyTo(head);
        head[4] = (byte)_dimension;
        BinaryPrimitives.WriteInt64LittleEndian(head[5..], _stepCount());
        BinaryPrimitives.WriteInt32LittleEndian(head[13..], _options.Length);
        writer.Advance(17);
        writer.Write(_options);
        writer.Write(snapshot);
    }

    internal static (int Dimension, long Step, byte[] Options, byte[] World) ParseSnapshot(ReadOnlySpan<byte> bytes, string name)
    {
        if (bytes.Length < 18 || !bytes.StartsWith(Magic))
        {
            throw new ScopeFormatException(name, "expected a OnePhysics live snapshot (starting 'OPLV') from the host's feed.");
        }

        int dimension = bytes[4];
        long step = BinaryPrimitives.ReadInt64LittleEndian(bytes[5..]);
        int optionLength = BinaryPrimitives.ReadInt32LittleEndian(bytes[13..]);
        if (dimension is not (2 or 3) || step < 0 || optionLength is < 1 or > MaxOptionBytes || bytes.Length - 17 <= optionLength)
        {
            throw new ScopeFormatException(name, "expected a dimension of 2 or 3, a nonnegative step and option and snapshot lengths that fit, in the first 17 bytes.");
        }

        return (dimension, step, bytes.Slice(17, optionLength).ToArray(), bytes[(17 + optionLength)..].ToArray());
    }

    /// <inheritdoc />
    public override void Write(ReadOnlySpan<byte> buffer)
    {
        _pending.Write(buffer);
        Drain();
    }

    /// <inheritdoc />
    public override void Write(byte[] buffer, int offset, int count) => Write(buffer.AsSpan(offset, count));

    private void Drain()
    {
        while (true)
        {
            ReadOnlySpan<byte> have = _pending.GetBuffer().AsSpan(0, (int)_pending.Length);
            int need = _headerDone ? RecordHeaderBytes : HeaderBytes;
            if (have.Length < need)
            {
                return;
            }

            int consumed;
            if (!_headerDone)
            {
                _headerDone = true;
                consumed = HeaderBytes;
            }
            else
            {
                int length = BinaryPrimitives.ReadInt32LittleEndian(have[1..]);
                if (length < 0 || length > Recording.MaxInputBytes * 4)
                {
                    throw new IOException("The recorder wrote a record longer than a live record may be.");
                }

                if (have.Length < RecordHeaderBytes + length)
                {
                    return;
                }

                consumed = RecordHeaderBytes + length;
                Accept((ReplayRecordKind)have[0], have[..consumed]);
            }

            byte[] rest = _pending.GetBuffer().AsSpan(consumed, (int)_pending.Length - consumed).ToArray();
            _pending.SetLength(0);
            _pending.Write(rest);
        }
    }

    private void Accept(ReplayRecordKind kind, ReadOnlySpan<byte> record)
    {
        if (_skipRecords > 0)
        {
            _skipRecords--;
            return;
        }

        switch (kind)
        {
            case ReplayRecordKind.LiveOptions:
                _options ??= record[RecordHeaderBytes..].ToArray();
                break;
            case ReplayRecordKind.BeginStep:
                _inFrame = true;
                _frame.SetLength(0);
                _frame.Write(record);
                break;
            case ReplayRecordKind.Mutation when _inFrame:
                _frame.Write(record);
                break;
            case ReplayRecordKind.EndStep when _inFrame:
                _frame.Write(record);
                _inFrame = false;
                Forward(_frame.GetBuffer().AsSpan(0, (int)_frame.Length));
                break;
            case ReplayRecordKind.Mutation:
                Forward(record);
                break;
        }
    }

    private void Forward(ReadOnlySpan<byte> record) => Record?.Invoke(record);

    /// <inheritdoc />
    public override bool CanRead => false;

    /// <inheritdoc />
    public override bool CanSeek => false;

    /// <inheritdoc />
    public override bool CanWrite => true;

    /// <inheritdoc />
    public override long Length => throw new NotSupportedException();

    /// <inheritdoc />
    public override long Position
    {
        get => throw new NotSupportedException();
        set => throw new NotSupportedException();
    }

    /// <inheritdoc />
    public override void Flush()
    {
    }

    /// <inheritdoc />
    public override int Read(byte[] buffer, int offset, int count) => throw new NotSupportedException();

    /// <inheritdoc />
    public override long Seek(long offset, SeekOrigin origin) => throw new NotSupportedException();

    /// <inheritdoc />
    public override void SetLength(long value) => throw new NotSupportedException();
}
