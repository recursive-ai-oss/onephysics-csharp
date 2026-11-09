using System;
using System.IO;

namespace RecursiveAI.OnePhysics.Replay;

internal interface IRecordedWorld
{
    int ReplayDimension { get; }
    void PrepareHistory();
    long ReplayStepCount { get; }
    int WriteReplayOptions(Span<byte> bytes);
    void ApplyReplayOptions(ReadOnlySpan<byte> bytes);
    int ReplaySnapshotSize { get; }
    int ReplaySnapshotBound { get; }
    bool WriteReplaySnapshot(Span<byte> bytes, out int written);
    ulong ReplayHash();
    void RestoreReplaySnapshot(ReadOnlySpan<byte> bytes);
    void AttachRecorder(ReplayRecorder? recorder);
}

/// <summary>
/// Opt-in recording of named mutation effects and simulated steps. Obtain it from world.StartRecording(stream).
/// Each outer public mutator records its resulting complete state, so all shape/service kinds use the same reflection-free representation.
/// Steps simulate during playback; callback-issued mutations are applied after simulation and before the final hash check.
/// The caller owns the stream. Initial setup and storage growth allocate; warmed steps and mutations allocate nothing in the recorder.
/// Direct raw span writes are synchronized before each step, or explicitly by CaptureRawChanges. Host code/delegates are not serialized.
/// </summary>
public sealed class ReplayRecorder : IDisposable
{
    private readonly IRecordedWorld _world;
    private readonly ReplayWriter _writer;
    private byte[] _snapshot;
    private readonly byte[] _options = new byte[256];
    private int _depth;
    private string _operation = "Mutation";
    private int _paused;
    internal int MaximumSnapshotCapacity = int.MaxValue;
    private bool _capturing, _simulating, _stepActive, _stopRequested, _stopped;
    internal Action? FrameEnded;
    internal ReplayRecorder(IRecordedWorld world, Stream stream, bool hashes)
    {
        _world = world;
        _capturing = true;
        int size = Math.Max(world.ReplaySnapshotSize, Math.Min(world.ReplaySnapshotBound, ReplayReader.DefaultMaximumPayloadBytes));
        _snapshot = new byte[size];
        world.AttachRecorder(this);
        try
        {
            int n = Snapshot();
            _writer = new ReplayWriter(stream, world.ReplayDimension, world.ReplayStepCount, _snapshot.AsSpan(0,n), hashes, commandTrack: true);
            WriteOptions();
        }
        catch { world.AttachRecorder(null); throw; }
        _capturing = false;
    }

    /// <summary>Number of fully completed recorded Step calls, independent of the world's counter changing through explicit Restore calls.</summary>
    public long RecordedSteps { get; private set; }
    /// <summary>True after disposal or an unrecoverable recording I/O failure.</summary>
    public bool IsStopped => _stopped;

    /// <summary>Records raw store/span writes immediately as a named complete-state effect. Normally each Step synchronizes these automatically. The operation name defaults to RawState and must be printable ASCII, 1..512 characters.</summary>
    public void CaptureRawChanges(string operation = "RawState")
    {
        ObjectDisposedException.ThrowIf(_stopped, this);
        if (_capturing || _simulating) return;
        Capture(operation);
    }

    internal ReplayMutationScope Enter(string operation)
    {
        if (_capturing || _simulating || _stopped || _paused > 0) return default;
        if (_depth++ == 0) _operation = operation;
        return new ReplayMutationScope(this);
    }

    internal void Exit()
    {
        if (--_depth == 0 && !_stopped) Capture(_operation);
    }

    internal void OptionChanged(string operation)
    {
        if (!_capturing && !_simulating && !_stopped && _paused == 0 && _depth == 0) Capture(operation);
    }

    internal ReplayStepScope BeginStep(float dt)
    {
        if (_depth > 0 || _capturing || _stopped || _paused > 0) return default;
        if (_stepActive) throw new InvalidOperationException("A recorded world cannot step recursively.");
        Capture("SynchronizeInputs");
        _writer.BeginStep(dt);
        _stepActive = true;
        _simulating = true;
        return new ReplayStepScope(this);
    }

    internal bool IsInStep => _stepActive;
    internal void SimulationEnded() => _simulating = false;
    internal void SimulationEffect(string operation) { if (_stepActive && !_simulating && !_stopped) Capture(operation); }

    internal void EndStep()
    {
        if (_stopped) return;
        try
        {
            _simulating = false;
            _stepActive = false;
            _writer.EndStep(_world.ReplayStepCount, _world.ReplayHash());
            RecordedSteps++;
            FrameEnded?.Invoke();
            if (_stopRequested) Dispose();
        }
        catch { _stopped = true; _world.AttachRecorder(null); throw; }
    }

    private int Snapshot()
    {
        int needed = _world.ReplaySnapshotSize;
        if (needed > MaximumSnapshotCapacity) throw new HistoryCapacityException("World snapshot exceeds the reserved history scratch capacity.");
        if (_snapshot.Length < needed) Array.Resize(ref _snapshot, needed);
        if (!_world.WriteReplaySnapshot(_snapshot, out int n)) throw new InvalidOperationException("World snapshot size changed while recording.");
        return n;
    }

    private void Capture(string operation)
    {
        _capturing = true;
        try
        {
                int n = Snapshot();
            _writer.WriteMutation(operation, _world.ReplayStepCount, _snapshot.AsSpan(0,n));
            WriteOptions();
        }
        catch { _stopped = true; _world.AttachRecorder(null); throw; }
        finally { _capturing = false; }
    }

    internal ReplayPauseScope Pause() { _paused++; return new ReplayPauseScope(this); }
    internal void Resume() => _paused--;
    internal ReadOnlySpan<byte> CaptureKeyframe() { int n = Snapshot(); return _snapshot.AsSpan(0,n); }
    internal void ReserveSnapshotCapacity(int capacity) { if (_snapshot.Length < capacity) Array.Resize(ref _snapshot, capacity); MaximumSnapshotCapacity = capacity; }
    internal void CompleteSegment() => _writer.Complete();
    internal void RestartSegment(ReadOnlySpan<byte> snapshot) { _writer.Restart(_world.ReplayStepCount, snapshot); WriteOptions(); }
    private void WriteOptions() { int n = _world.WriteReplayOptions(_options); _writer.WriteLiveOptions(_options.AsSpan(0,n)); }

    /// <summary>Writes the terminator and detaches from the world; repeated calls do nothing. When called inside a step callback, completion is deferred until that step ends. Does not close or flush the caller's stream.</summary>
    public void Dispose()
    {
        if (_stopped) return;
        if (_stepActive) { _stopRequested = true; return; }
        try { _writer.Complete(); }
        finally { _stopped = true; _world.AttachRecorder(null); }
    }
}

internal readonly struct ReplayMutationScope(ReplayRecorder? recorder) : IDisposable
{
    public void Dispose() => recorder?.Exit();
}

internal readonly struct ReplayStepScope(ReplayRecorder? recorder) : IDisposable
{
    public void Dispose() => recorder?.EndStep();
}

internal readonly struct ReplayPauseScope(ReplayRecorder? recorder) : IDisposable
{
    public void Dispose() => recorder?.Resume();
}
