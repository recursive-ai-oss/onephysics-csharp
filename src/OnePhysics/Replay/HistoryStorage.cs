using System;
using System.IO;

namespace RecursiveAI.OnePhysics.Replay;

/// <summary>A single snapshot or keyframe interval cannot fit the configured history budget. Completed retained steps remain available to seek.</summary>
public sealed class HistoryCapacityException : PhysicsException
{
    /// <summary>Creates a capacity error naming what could not fit.</summary>
    public HistoryCapacityException(string message) : base(message) { }
}

// Keyframe representation is isolated here; storage currently uses complete snapshot bytes.
// A different representation can encode/decode here without changing the history's public API.
internal interface IHistoryKeyframes
{
    ReadOnlySpan<byte> Capture(ReplayRecorder recorder);
    void Restore(IRecordedWorld world, ReadOnlySpan<byte> bytes);
}
internal sealed class FullSnapshotKeyframes : IHistoryKeyframes
{
    public ReadOnlySpan<byte> Capture(ReplayRecorder recorder) => recorder.CaptureKeyframe();
    public void Restore(IRecordedWorld world, ReadOnlySpan<byte> bytes) => world.RestoreReplaySnapshot(bytes);
}

internal sealed class HistoryBufferStream : Stream
{
    private struct Segment
    {
        internal long Start, End, First, Last;
    }
    private readonly byte[] _bytes;
    private readonly Segment[] _segments;
    private int _head, _count;
    private long _written;
    internal HistoryBufferStream(int capacity, int segmentCapacity)
    {
        _bytes = new byte[capacity];
        _segments = new Segment[segmentCapacity];
    }
    internal long FirstStep => _segments[_head].First;
    internal int Capacity => _bytes.Length;
    internal long ActiveBytes => _written - _segments[(_head + _count - 1) % _segments.Length].Start;
    internal long RetainedBytes => _count == 0 ? 0 : _written - _segments[_head].Start;
    internal void BeginSegment(long frame)
    {
        if (_count == _segments.Length) Evict();
        int i = (_head + _count++) % _segments.Length;
        _segments[i] = new Segment { Start = _written, End = _written, First = frame, Last = frame };
    }
    internal void CommitFrame(long frame)
    {
        int i = (_head + _count - 1) % _segments.Length;
        _segments[i].Last = frame; _segments[i].End = _written;
    }
    private void Evict() { _head = (_head + 1) % _segments.Length; _count--; }
    internal HistoryReadStream Open(long frame, out long first)
    {
        for (int j = _count - 1; j >= 0; j--)
        {
            int i = (_head + j) % _segments.Length;
            var s = _segments[i];
            if (s.First <= frame && s.Last >= frame) { first = s.First; return new HistoryReadStream(this,s.Start,s.End); }
        }
        throw new ArgumentOutOfRangeException(nameof(frame), frame, "Step is not retained in this history.");
    }
    public override void Write(ReadOnlySpan<byte> buffer)
    {
        if (buffer.Length > _bytes.Length) throw new HistoryCapacityException("One history record exceeds the data budget.");
        while (_written - _segments[_head].Start + buffer.Length > _bytes.Length)
        {
            if (_count <= 1) throw new HistoryCapacityException("The current keyframe interval exceeds the history budget. Increase the budget or shorten the keyframe interval.");
            Evict();
        }
        int at = (int)(_written % _bytes.Length), first = Math.Min(buffer.Length, _bytes.Length - at);
        buffer.Slice(0,first).CopyTo(_bytes.AsSpan(at));
        buffer.Slice(first).CopyTo(_bytes);
        _written += buffer.Length;
    }
    internal void ReadAt(long absolute, Span<byte> buffer)
    {
        int at = (int)(absolute % _bytes.Length), first = Math.Min(buffer.Length, _bytes.Length-at);
        _bytes.AsSpan(at,first).CopyTo(buffer); _bytes.AsSpan(0,buffer.Length-first).CopyTo(buffer.Slice(first));
    }
    public override bool CanRead => false;
    public override bool CanSeek => false;
    public override bool CanWrite => true;
    public override long Length => _written;
    public override long Position { get => _written; set => throw new NotSupportedException(); }
    public override void Flush() { }
    public override int Read(byte[] buffer,int offset,int count) => throw new NotSupportedException();
    public override long Seek(long offset,SeekOrigin origin) => throw new NotSupportedException();
    public override void SetLength(long value) => throw new NotSupportedException();
    public override void Write(byte[] buffer,int offset,int count) => Write(buffer.AsSpan(offset,count));
}

internal sealed class HistoryReadStream(HistoryBufferStream source,long start,long end) : Stream
{
    private long _read;
    internal void Rewind() => _read=0;
    public override int Read(Span<byte> destination)
    {
        int n=(int)Math.Min(destination.Length,end-start-_read);
        source.ReadAt(start+_read,destination.Slice(0,n));_read+=n;return n;
    }
    public override int Read(byte[] buffer,int offset,int count) => Read(buffer.AsSpan(offset,count));
    public override bool CanRead=>true;
    public override bool CanSeek=>false;
    public override bool CanWrite=>false;
    public override long Length=>end-start;
    public override long Position { get=>_read;set=>throw new NotSupportedException(); }
    public override void Flush()=>throw new NotSupportedException();
    public override long Seek(long offset,SeekOrigin origin)=>throw new NotSupportedException();
    public override void SetLength(long value)=>throw new NotSupportedException();
    public override void Write(byte[] buffer,int offset,int count)=>throw new NotSupportedException();
}
