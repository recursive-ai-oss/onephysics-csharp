using System;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Replay;

/// <summary>History retention settings, copied when a history recorder starts.</summary>
public sealed class HistoryOptions
{
    /// <summary>Maximum steps between full keyframes, default 30; at least 1. Memory pressure may insert a keyframe earlier.</summary>
    public int KeyframeInterval { get; set; } = 30;
    /// <summary>Total owned byte/index/snapshot working-storage budget, default 32 MiB. Allowed range 4 KiB..512 MiB; the initial world and at least one frame must fit.</summary>
    public int MemoryBudgetBytes { get; set; } = 32 * 1024 * 1024;
}

/// <summary>
/// Bounded history: full-snapshot keyframes plus the replay command stream between them. Obtain from world.StartHistory().
/// Retention evicts oldest complete intervals, never arbitrary record bytes. A frame/interval that alone exceeds the budget raises HistoryCapacityException.
/// Step indices count recorded public Step calls starting at 0, independent of explicit restores changing the world's own StepCount.
/// Recording and seeking are single-writer operations. Stop keeps data available; Dispose stops and prevents further seeks.
/// </summary>
public sealed class HistoryRecorder : IDisposable
{
    private readonly HistoryBufferStream _stream;
    private readonly ReplayRecorder _recorder;
    private readonly IHistoryKeyframes _keyframes;
    private readonly int _dimension, _interval;
    private readonly int _snapshotReserve;
    private long _keyframeStep;
    private bool _disposed;

    internal HistoryRecorder(IRecordedWorld world, HistoryOptions options, IHistoryKeyframes? keyframes = null)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(options.KeyframeInterval,1);
        if (options.MemoryBudgetBytes < 4096 || options.MemoryBudgetBytes > 512*1024*1024) throw new ArgumentOutOfRangeException(nameof(options), "History budget must be within 4 KiB..512 MiB.");
        _keyframes = keyframes ?? new FullSnapshotKeyframes();
        MemoryBudgetBytes = options.MemoryBudgetBytes;
        _interval = options.KeyframeInterval;
        _dimension = world.ReplayDimension;
        _snapshotReserve = Math.Max(Math.Max(world.ReplaySnapshotSize,Math.Min(world.ReplaySnapshotBound,ReplayReader.DefaultMaximumPayloadBytes)), MemoryBudgetBytes/4);
        int segments = Math.Clamp(MemoryBudgetBytes/1024,4,4096);
        int data = MemoryBudgetBytes-_snapshotReserve-segments*40-2048;
        if(data < (long)world.ReplaySnapshotSize*2+512) throw new HistoryCapacityException("History budget cannot hold a working snapshot, index, initial keyframe and one frame. Increase MemoryBudgetBytes.");
        world.PrepareHistory();
        _stream = new HistoryBufferStream(data,segments);
        _stream.BeginSegment(0);
        _recorder = new ReplayRecorder(world,_stream,true);
        _recorder.ReserveSnapshotCapacity(_snapshotReserve);
        _recorder.FrameEnded = OnFrame;
        _stream.CommitFrame(0);
    }

    /// <summary>The configured total budget in bytes, including reserved working snapshot storage and the retained data/index storage.</summary>
    public int MemoryBudgetBytes { get; }
    /// <summary>First retained logical step, initially 0. Earlier steps have been evicted.</summary>
    public long FirstStep => _stream.FirstStep;
    /// <summary>Last fully completed logical step, initially 0.</summary>
    public long LastStep => _recorder.RecordedSteps;
    /// <summary>Currently retained replay/keyframe bytes, excluding the already-reserved index and working snapshot storage.</summary>
    public long RetainedBytes => _stream.RetainedBytes;
    /// <summary>True until stopped or disposed.</summary>
    public bool IsRecording => !_recorder.IsStopped;

    private void OnFrame()
    {
        long frame = _recorder.RecordedSteps;
        _stream.CommitFrame(frame);
        if(frame-_keyframeStep < _interval && _stream.ActiveBytes+_snapshotReserve+1024 < _stream.Capacity) return;
        var keyframe = _keyframes.Capture(_recorder);
        if (keyframe.Length + 256 > _stream.Capacity) throw new HistoryCapacityException("A keyframe exceeds the retained data budget.");
        _recorder.CompleteSegment();
        _stream.CommitFrame(frame);
        _stream.BeginSegment(frame);
        _recorder.RestartSegment(keyframe);
        _stream.CommitFrame(frame);
        _keyframeStep = frame;
    }

    /// <summary>Stops recording and keeps retained steps available to seek. Leaves the source world usable. Repeated calls do nothing.</summary>
    public void Stop() => _recorder.Dispose();

    /// <summary>Restores the nearest keyframe and simulates to a retained logical step in a planar destination. Callbacks are suppressed, polled events remain. May allocate reader/restore storage. Passing the source world temporarily suspends its recorder.</summary>
    public void Seek(long step, PhysicsWorld2D destination)
    {
        ArgumentNullException.ThrowIfNull(destination);
        Validate(step,2);
        if(destination.Recorder?.IsInStep == true)throw new InvalidOperationException("Cannot seek a world during its active Step.");
        using var pause=destination.Recorder?.Pause() ?? default;
        using var input=_stream.Open(step,out long first);
        var reader=RestoreKeyframe(input,destination);
        if(first==step)return;
        var player=new ReplayPlayer(reader,"history");
        while(first++<step)if(!player.ReadNextStep(destination))throw new InvalidOperationException("Incomplete retained history frame.");
    }

    /// <summary>Restores the nearest keyframe and simulates to a retained logical step in a spatial destination. Callbacks are suppressed, polled events remain. May allocate reader/restore storage. Passing the source world temporarily suspends its recorder.</summary>
    public void Seek(long step, PhysicsWorld3D destination)
    {
        ArgumentNullException.ThrowIfNull(destination);
        Validate(step,3);
        if(destination.Recorder?.IsInStep == true)throw new InvalidOperationException("Cannot seek a world during its active Step.");
        using var pause=destination.Recorder?.Pause() ?? default;
        using var input=_stream.Open(step,out long first);
        var reader=RestoreKeyframe(input,destination);
        if(first==step)return;
        var player=new ReplayPlayer(reader,"history");
        while(first++<step)if(!player.ReadNextStep(destination))throw new InvalidOperationException("Incomplete retained history frame.");
    }

    private ReplayReader RestoreKeyframe(HistoryReadStream input,IRecordedWorld destination)
    {
        var reader=new ReplayReader(input,"history",maximumPayloadBytes:_stream.Capacity);
        if(!reader.ReadNext() || reader.Kind!=ReplayRecordKind.Snapshot)throw new InvalidOperationException("Missing history keyframe.");
        _keyframes.Restore(destination,reader.SnapshotPayload);
        if (!reader.ReadNext() || reader.Kind != ReplayRecordKind.LiveOptions) throw new InvalidOperationException("Missing live keyframe options.");
        destination.ApplyReplayOptions(reader.Payload);
        return reader;
    }
    private void Validate(long step,int dimension)
    {
        ObjectDisposedException.ThrowIf(_disposed,this);
        if(_dimension!=dimension)throw new ArgumentException("Destination dimension does not match this history.",nameof(dimension));
        if(step<FirstStep || step>LastStep)throw new ArgumentOutOfRangeException(nameof(step),step,$"Retained steps are [{FirstStep}, {LastStep}].");
    }

    /// <summary>Stops recording and invalidates further seeks. Does not dispose the source or any destination world.</summary>
    public void Dispose()
    {
        if(_disposed)return;
        _recorder.FrameEnded=null;
        _recorder.Dispose();
        _disposed=true;
        _stream.Dispose();
    }
}
