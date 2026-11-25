using System;
using System.Buffers.Binary;
using System.IO;

namespace RecursiveAI.OnePhysics.Presentation;

/// <summary>
/// Records a follower world as the inputs it consumed: a full snapshot at the start and whenever something other than a followed pose changed the world between steps (a spawn, a cull, a quality
/// change, a body or rod the host added), and for every step the poses it moved the mirrors to, the step length, the speed bound and the resulting world hash.
/// <see cref="PresentationPlayer"/> replays it without the authoritative world. Obtain one from <c>follower.StartRecording(stream)</c>; dispose it to write the terminator. The caller owns the stream.
/// </summary>
public sealed class PresentationRecorder : IDisposable
{
    private readonly IPresentationHost _host;
    private readonly Stream _stream;
    private readonly byte[] _scratch = new byte[PresentationFormat.RecordHeaderSize + 8];
    private readonly byte[] _header = new byte[PresentationFormat.HeaderSize];
    private readonly byte[] _word = new byte[8];
    private readonly byte[] _options;
    private readonly byte[] _lastOptions;
    private ulong _lastHash;
    private bool _disposed;

    internal PresentationRecorder(IPresentationHost host, Stream stream)
    {
        ArgumentNullException.ThrowIfNull(stream);
        if (!stream.CanWrite) throw new ArgumentException("The stream must be writable.", nameof(stream));
        _host = host; _stream = stream;
        _options = new byte[host.OptionsSize]; _lastOptions = new byte[host.OptionsSize];
        PresentationFormat.WriteHeader(_header, host.Dimension, host.StepCount);
        stream.Write(_header);
        WriteSnapshot();
        host.AttachRecorder(this);
    }

    /// <summary>Number of steps written so far.</summary>
    public long Frames { get; private set; }

    private void WriteRecord(byte kind, ReadOnlySpan<byte> part1, ReadOnlySpan<byte> part2, ReadOnlySpan<byte> part3)
    {
        long length = (long)part1.Length + part2.Length + part3.Length;
        if (length > int.MaxValue) throw new InvalidOperationException("The record is too large to write.");
        _scratch[0] = kind;
        BinaryPrimitives.WriteInt32LittleEndian(_scratch.AsSpan(1), (int)length);
        _stream.Write(_scratch.AsSpan(0, PresentationFormat.RecordHeaderSize));
        if (part1.Length != 0) _stream.Write(part1);
        if (part2.Length != 0) _stream.Write(part2);
        if (part3.Length != 0) _stream.Write(part3);
    }

    private void WriteSnapshot()
    {
        _host.WriteOptions(_options);
        var snapshot = _host.Snapshot();
        BinaryPrimitives.WriteInt32LittleEndian(_word, _options.Length);
        WriteRecord(PresentationFormat.KindSnapshot, _word.AsSpan(0, 4), _options, snapshot);
        _lastHash = _host.Hash();
        Array.Copy(_options, _lastOptions, _options.Length);
    }

    /// <summary>Called by the follower before it applies a frame: writes a snapshot if the world's hash or live options differ from what the last step left.</summary>
    internal void Sync()
    {
        ThrowIfDisposed();
        _host.WriteOptions(_options);
        if (_host.Hash() != _lastHash || !_options.AsSpan().SequenceEqual(_lastOptions)) WriteSnapshot();
    }

    /// <summary>Writes one step: the frame bytes (prologue and poses) and the world hash after it.</summary>
    internal void WriteFrame(ReadOnlySpan<byte> frame, ulong hash)
    {
        ThrowIfDisposed();
        BinaryPrimitives.WriteUInt64LittleEndian(_word, hash);
        WriteRecord(PresentationFormat.KindFrame, frame, _word, default);
        _lastHash = hash;
        Frames++;
    }

    private void ThrowIfDisposed() => ObjectDisposedException.ThrowIf(_disposed, this);

    /// <summary>Writes the terminator and detaches from the follower. The stream stays open.</summary>
    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        _host.AttachRecorder(null);
        WriteRecord(PresentationFormat.KindEnd, default, default, default);
        _stream.Flush();
    }
}
