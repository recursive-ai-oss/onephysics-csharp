using System;
using System.Buffers;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.IO;
using OneScope.Sdk;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Replay;
using SceneFile = RecursiveAI.OnePhysics.Scene.Scene;

namespace RecursiveAI.OnePhysics.Scope;

/// <summary>How a world is made at step 0.</summary>
internal enum SetupKind : byte
{
    /// <summary>A full snapshot plus the live options a recorded replay begins with.</summary>
    SnapshotWithOptions = 0,

    /// <summary>Scene text, authored or exact-state.</summary>
    SceneText = 1,

    /// <summary>A bare snapshot.</summary>
    Snapshot = 2,
}

/// <summary>A full world state that a step input refers to by number (a replay's mutation effects can be large, and an input holds at most 1 MiB).</summary>
internal sealed record StoredState(string Name, byte[] Bytes);

/// <summary>Everything a world is built from. Immutable and shared by a source and its forks.</summary>
internal sealed class WorldSetup
{
    public required string Name { get; init; }

    public required int Dimension { get; init; }

    public required SetupKind Kind { get; init; }

    /// <summary>The snapshot bytes, or the UTF-8 scene text.</summary>
    public required byte[] Data { get; init; }

    /// <summary>The recorded live options of a replay's first record, for <see cref="SetupKind.SnapshotWithOptions"/>.</summary>
    public byte[]? Options { get; init; }

    /// <summary>The initial completed step count of the replay the snapshot came from.</summary>
    public long InitialStep { get; init; }

    /// <summary>The time step a world uses until an input says otherwise.</summary>
    public required float DefaultDt { get; init; }

    public required IReadOnlyList<StoredState> States { get; init; }

    /// <summary>The small, fixed header kept as the recording's setup bytes.</summary>
    public byte[] Header()
    {
        var h = new byte[12];
        "OPSC"u8.CopyTo(h);
        h[4] = 1;
        h[5] = (byte)Dimension;
        h[6] = (byte)Kind;
        BinaryPrimitives.WriteSingleLittleEndian(h.AsSpan(8), DefaultDt);
        return h;
    }

    /// <summary>Makes a world at its first state.</summary>
    /// <exception cref="ScopeFormatException">The data does not make a valid world.</exception>
    public ScopeWorld CreateWorld(int threads)
    {
        try
        {
            WorldHost host = Kind switch
            {
                SetupKind.SnapshotWithOptions => FromReplayPrefix(threads),
                SetupKind.SceneText => FromScene(SceneFile.Read(new StringReader(System.Text.Encoding.UTF8.GetString(Data)), Name), threads),
                _ => FromScene(SceneFile.FromSnapshot(Dimension, 1, Data), threads),
            };
            return new ScopeWorld(host, this);
        }
        catch (Exception e) when (e is PhysicsException or ArgumentException or InvalidDataException)
        {
            throw new ScopeFormatException(Name, "expected a valid OnePhysics world: " + e.Message);
        }
    }

    private static WorldHost FromScene(SceneFile scene, int threads) =>
        scene.Dim == 3 ? new Host3D(scene.CreateWorld3D(threads)) : new Host2D(scene.CreateWorld2D(threads));

    // The library builds a world with its original options only from a replay stream, so the first records are re-framed as one.
    private WorldHost FromReplayPrefix(int threads)
    {
        var buffer = new ArrayBufferWriter<byte>();
        var e = new ReplayEmitter(buffer, Dimension, InitialStep);
        e.Snapshot(Data);
        e.LiveOptions(Options!);
        e.End();
        var player = new ReplayPlayer(new MemoryStream(buffer.WrittenSpan.ToArray()), Name);
        return Dimension == 3 ? new Host3D(player.CreateWorld3D(threads)) : new Host2D(player.CreateWorld2D(threads));
    }
}

/// <summary>An opened recording: the world setup, the per-step inputs and the declarations the viewer shows.</summary>
internal sealed class PhysicsSource : IScopeSource
{
    private readonly WorldSetup _setup;
    private readonly int _threads;

    public PhysicsSource(WorldSetup setup, Recording recording, int threads)
    {
        _setup = setup;
        _threads = threads;
        Recording = recording;
        Info = ScopeInfo.Make(setup.Name, setup.Dimension);
    }

    public SourceInfo Info { get; }

    public Recording Recording { get; }

    public IScopeWorld CreateWorld() => _setup.CreateWorld(_threads);

    public IScopeSource WithRecording(Recording recording) => new PhysicsSource(_setup, recording, _threads);

    /// <summary>
    /// Writes the recording as a OnePhysics replay (version 2): the world's initial records, then each step as its stored or poked
    /// state changes followed by a step with its world hash. The hashes come from running the recording, so the file is
    /// self-checking and reading it back and writing it again gives the same bytes.
    /// </summary>
    public void WriteReplay(IBufferWriter<byte> writer)
    {
        ArgumentNullException.ThrowIfNull(writer);
        ScopeWorld world = _setup.CreateWorld(_threads);
        (byte[] snapshot, byte[]? options, long initialStep) = world.InitialRecords();
        var emit = new ReplayEmitter(writer, _setup.Dimension, initialStep);
        emit.Snapshot(snapshot);
        if (options is not null)
        {
            emit.LiveOptions(options);
        }

        for (long s = 0; s < Recording.StepCount; s++)
        {
            world.Run(Recording.InputAt(s).Span, TimelineSink.None, emit);
        }

        emit.End();
    }
}
