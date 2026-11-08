using System;
using System.IO;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Replay;

/// <summary>The first replay step whose resulting world hash differs from its hash track. The world is left at the divergent step for inspection.</summary>
public sealed class ReplayHashMismatchException : PhysicsException
{
    /// <summary>Creates an error with the absolute completed step count and both hashes.</summary>
    public ReplayHashMismatchException(long step, ulong expected, ulong actual)
        : base($"Replay hash mismatch at step {step}: expected {expected:x16}, actual {actual:x16}.")
    {
        Step = step;
        Expected = expected;
        Actual = actual;
    }

    /// <summary>The absolute completed step count of the first mismatch.</summary>
    public long Step { get; }

    /// <summary>Hash saved in the replay.</summary>
    public ulong Expected { get; }

    /// <summary>Hash computed from the replayed world.</summary>
    public ulong Actual { get; }
}

/// <summary>
/// Incremental playback of a checkpoint replay into an existing world. Snapshots restore full state; Step records simulate rather than restoring results.
/// World callbacks are suppressed while simulating; polled events remain available. Host contact/solver callbacks are not serialized and must be installed by the caller if needed.
/// Use the same destination world throughout. Any error makes this player unusable; the world remains at its last applied state for inspection.
/// The caller owns the input stream. This transport does not yet support a world API command track.
/// </summary>
public sealed class ReplayPlayer
{
    private readonly ReplayReader _reader;
    private readonly string _inputName;
    private bool _failed;
    private object? _world;

    /// <summary>Opens an input; reads its header, not the initial snapshot. Defaults to a 64 MiB maximum record size. The stream need not seek and is left open.</summary>
    /// <exception cref="ReplayFormatException">Invalid header.</exception>
    /// <exception cref="ArgumentException">Input cannot be read.</exception>
    /// <exception cref="ArgumentOutOfRangeException">Maximum payload is less than 12 bytes.</exception>
    public ReplayPlayer(Stream stream, string inputName = "replay", int maximumPayloadBytes = ReplayReader.DefaultMaximumPayloadBytes)
    {
        _reader = new ReplayReader(stream, inputName, maximumPayloadBytes);
        _inputName = inputName;
    }

    internal ReplayPlayer(ReplayReader reader, string inputName) { _reader = reader; _inputName = inputName; }

    /// <summary>The replay's dimension, 2 or 3.</summary>
    public int Dimension => _reader.Dimension;

    /// <summary>The absolute completed step count after the last successfully read record, initially the starting count.</summary>
    public long CompletedSteps => _reader.CompletedSteps;

    /// <summary>
    /// Restores any snapshots up to the next step, then simulates and checks that step in a planar world. Returns false at the terminator.
    /// The initial snapshot replaces the destination state. Snapshot option changes have the same behavior as world.Restore.
    /// No allocations once the reader's payload and the world's restore/step buffers are warm and large enough.
    /// </summary>
    /// <exception cref="ArgumentException">Wrong dimension or a different destination than a previous call.</exception>
    /// <exception cref="ReplayFormatException">Invalid record, snapshot, or absolute step count.</exception>
    /// <exception cref="ReplayHashMismatchException">The first step whose hash differs.</exception>
    /// <exception cref="InvalidOperationException">A prior playback operation failed.</exception>
    public bool ReadNextStep(PhysicsWorld2D world)
    {
        ArgumentNullException.ThrowIfNull(world);
        ValidateWorld(world, 2);
        return Advance(new World2D(world));
    }

    /// <summary>
    /// Restores any snapshots up to the next step, then simulates and checks that step in a spatial world. Returns false at the terminator.
    /// The initial snapshot replaces the destination state. Snapshot option changes have the same behavior as world.Restore.
    /// No allocations once the reader's payload and the world's restore/step buffers are warm and large enough.
    /// </summary>
    /// <exception cref="ArgumentException">Wrong dimension or a different destination than a previous call.</exception>
    /// <exception cref="ReplayFormatException">Invalid record, snapshot, or absolute step count.</exception>
    /// <exception cref="ReplayHashMismatchException">The first step whose hash differs.</exception>
    /// <exception cref="InvalidOperationException">A prior playback operation failed.</exception>
    public bool ReadNextStep(PhysicsWorld3D world)
    {
        ArgumentNullException.ThrowIfNull(world);
        ValidateWorld(world, 3);
        return Advance(new World3D(world));
    }

    /// <summary>Creates a compatible planar destination from the initial snapshot and recorded creation/live options. Intended for recorder-produced version 2 files; version 1 uses default creation options. Continue with ReadNextStep on the returned world.</summary>
    public PhysicsWorld2D CreateWorld2D(int threadCount = 1)
    {
        var initial = ReadInitial(2);
        try
        {
            var options = new WorldOptions2D { ThreadCount = threadCount };
            if (initial.Options != null) { options.BroadphaseMargin = ReplayOptions2D.Read(initial.Options,options,out int id); options.WorldId=id; }
            ReplayOptions2D.PrepareCreation(options);
            var world = new PhysicsWorld2D(options);
            world.Restore(initial.Snapshot);
            if(initial.Options!=null)((IRecordedWorld)world).ApplyReplayOptions(initial.Options);
            CheckStep(world.StepCount,_reader.CompletedSteps);
            _world=world;return world;
        }
        catch (SnapshotFormatException e) { _failed=true;throw new ReplayFormatException(_inputName,21+e.Offset,"valid initial snapshot: "+e.Message); }
        catch (ArgumentException e) { _failed=true;throw new ReplayFormatException(_inputName,_reader.RecordOffset+5,"valid initial live options: "+e.Message); }
        catch { _failed=true;throw; }
    }

    /// <summary>Creates a compatible spatial destination from the initial snapshot and recorded creation/live options. Intended for recorder-produced version 2 files; version 1 uses default creation options. Continue with ReadNextStep on the returned world.</summary>
    public PhysicsWorld3D CreateWorld3D(int threadCount = 1)
    {
        var initial = ReadInitial(3);
        try
        {
            var options = new WorldOptions3D { ThreadCount = threadCount };
            if (initial.Options != null) { options.BroadphaseMargin = ReplayOptions3D.Read(initial.Options,options,out int id); options.WorldId=id; }
            ReplayOptions3D.PrepareCreation(options);
            var world = new PhysicsWorld3D(options);
            world.Restore(initial.Snapshot);
            if(initial.Options!=null)((IRecordedWorld)world).ApplyReplayOptions(initial.Options);
            CheckStep(world.StepCount,_reader.CompletedSteps);
            _world=world;return world;
        }
        catch (SnapshotFormatException e) { _failed=true;throw new ReplayFormatException(_inputName,21+e.Offset,"valid initial snapshot: "+e.Message); }
        catch (ArgumentException e) { _failed=true;throw new ReplayFormatException(_inputName,_reader.RecordOffset+5,"valid initial live options: "+e.Message); }
        catch { _failed=true;throw; }
    }

    private (byte[] Snapshot,byte[]? Options) ReadInitial(int dimension)
    {
        if(_failed || _world!=null)throw new InvalidOperationException("Create the destination only before playback starts.");
        if(Dimension!=dimension)throw new ArgumentException("Replay dimension does not match the requested destination.",nameof(dimension));
        try
        {
            if(!_reader.ReadNext() || _reader.Kind!=ReplayRecordKind.Snapshot)throw new ReplayFormatException(_inputName,_reader.RecordOffset,"initial snapshot");
            byte[] snapshot=_reader.Payload.ToArray();
            byte[]? options=null;
            if(_reader.FormatVersion==2)
            {
                if(!_reader.ReadNext() || _reader.Kind!=ReplayRecordKind.LiveOptions)throw new ReplayFormatException(_inputName,_reader.RecordOffset,"initial live options for automatic world construction");
                options=_reader.Payload.ToArray();
            }
            return(snapshot,options);
        }
        catch {_failed=true;throw;}
    }

    private void ValidateWorld(object world, int dimension)
    {
        if (_failed) { throw new InvalidOperationException("Replay player cannot continue after a failure."); }
        if (Dimension != dimension) { throw new ArgumentException($"Replay dimension is {Dimension}, not {dimension}.", nameof(world)); }
        if (_world != null && !ReferenceEquals(_world, world)) { throw new ArgumentException("Continue playback in the same destination world.", nameof(world)); }
        _world = world;
    }

    private bool Advance<TWorld>(TWorld world) where TWorld : struct, IWorld
    {
        try
        {
            while (_reader.ReadNext())
            {
                if (_reader.Kind == ReplayRecordKind.Snapshot || _reader.Kind == ReplayRecordKind.Mutation)
                {
                    try { world.Restore(_reader.SnapshotPayload); }
                    catch (SnapshotFormatException e) { throw new ReplayFormatException(_inputName, _reader.RecordOffset + 5 + e.Offset, "valid snapshot: " + e.Message); }
                }
                else if (_reader.Kind == ReplayRecordKind.LiveOptions)
                {
                    try { world.Options(_reader.Payload); }
                    catch (ArgumentException e) { throw new ReplayFormatException(_inputName,_reader.RecordOffset+5,"valid live options: "+e.Message); }
                    continue;
                }
                else
                {
                    if (_reader.Kind != ReplayRecordKind.EndStep)
                    {
                        CheckStep(world.StepCount, _reader.CompletedSteps - 1);
                        world.Step(_reader.StepTime);
                        CheckStep(world.StepCount, _reader.CompletedSteps);
                        if (_reader.Kind == ReplayRecordKind.BeginStep) continue;
                    }
                    CheckStep(world.StepCount, _reader.CompletedSteps);
                    ulong? expected = _reader.StepHash;
                    if (expected.HasValue)
                    {
                        ulong actual = world.Hash();
                        if (actual != expected.Value) { throw new ReplayHashMismatchException(world.StepCount, expected.Value, actual); }
                    }
                    return true;
                }
                CheckStep(world.StepCount, _reader.CompletedSteps);
            }
            return false;
        }
        catch { _failed = true; throw; }
    }

    private void CheckStep(long actual, long expected)
    {
        if (actual != expected) { throw new ReplayFormatException(_inputName, _reader.RecordOffset, $"world step count {expected}, found {actual}"); }
    }

    private interface IWorld
    {
        long StepCount { get; }
        void Restore(ReadOnlySpan<byte> bytes);
        void Step(float dt);
        ulong Hash();
        void Options(ReadOnlySpan<byte> bytes);
    }

    private readonly struct World2D(PhysicsWorld2D world) : IWorld
    {
        public long StepCount => world.StepCount;
        public void Restore(ReadOnlySpan<byte> bytes) => world.Restore(bytes);
        public void Step(float dt) { using var suppression = world.SuppressCallbacks(); world.Step(dt); }
        public ulong Hash() => world.ComputeHash();
        public void Options(ReadOnlySpan<byte> bytes) => ((IRecordedWorld)world).ApplyReplayOptions(bytes);
    }

    private readonly struct World3D(PhysicsWorld3D world) : IWorld
    {
        public long StepCount => world.StepCount;
        public void Restore(ReadOnlySpan<byte> bytes) => world.Restore(bytes);
        public void Step(float dt) { using var suppression = world.SuppressCallbacks(); world.Step(dt); }
        public ulong Hash() => world.ComputeHash();
        public void Options(ReadOnlySpan<byte> bytes) => ((IRecordedWorld)world).ApplyReplayOptions(bytes);
    }
}
