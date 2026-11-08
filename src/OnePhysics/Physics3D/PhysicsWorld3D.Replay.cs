using System;
using System.IO;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D : IRecordedWorld
{
    private ReplayRecorder? _recorder;
    private WorldOptions3D? _replayOptionScratch;
    /// <summary>The active recorder, null by default. Recording is opt-in and does not alter simulation state or hashes.</summary>
    public ReplayRecorder? Recorder => _recorder;
    /// <summary>Starts recording all public mutation effects and steps to a caller-owned stream. Snapshot setup and capacity growth may allocate. Only one recorder/history may own a world at a time.</summary>
    public ReplayRecorder StartRecording(Stream stream, bool includeHashes = true)
    {
        if (_recorder != null) throw new InvalidOperationException("This world already has an active recorder.");
        return new ReplayRecorder(this, stream, includeHashes);
    }
    /// <summary>Starts bounded history recording with full-snapshot keyframes. Defaults: 30-step cadence and 32 MiB total storage budget, including reserved working snapshot storage.</summary>
    public HistoryRecorder StartHistory(HistoryOptions? options = null)
    {
        if (_recorder != null) throw new InvalidOperationException("This world already has an active recorder.");
        return new HistoryRecorder(this, options ?? new HistoryOptions());
    }
    internal ReplayMutationScope RecordMutation(string operation) => _recorder?.Enter(operation) ?? default;
    private void RecordPreparedStep(float dt) { using var step = _recorder?.BeginStep(dt) ?? default; StepPrepared(dt); }
    void IRecordedWorld.PrepareHistory() => _remembersDestroyed = true;
    int IRecordedWorld.ReplayDimension => 3;
    long IRecordedWorld.ReplayStepCount => StepCount;
    int IRecordedWorld.WriteReplayOptions(Span<byte> bytes) { ReplayOptions3D.Write(Options, _engine.Bp.Options.Margin, Id, bytes); return ReplayOptions3D.Size; }
    void IRecordedWorld.ApplyReplayOptions(ReadOnlySpan<byte> bytes)
    {
        var options = _replayOptionScratch ??= new WorldOptions3D();
        ReplayOptions3D.Read(bytes, options, out _);
        ReplayOptions3D.Copy(options, Options);
    }
    int IRecordedWorld.ReplaySnapshotSize => GetSnapshotSize();
    int IRecordedWorld.ReplaySnapshotBound => GetSnapshotSizeBound();
    bool IRecordedWorld.WriteReplaySnapshot(Span<byte> bytes, out int written) => _core.TrySnapshot(bytes, out written);
    ulong IRecordedWorld.ReplayHash() => ComputeHash();
    void IRecordedWorld.RestoreReplaySnapshot(ReadOnlySpan<byte> bytes) => Restore(bytes);
    void IRecordedWorld.AttachRecorder(ReplayRecorder? recorder)
    {
        _recorder = recorder;
        _engine.Opt.ReplayChanged = recorder == null ? null : recorder.OptionChanged;
    }
}
