using System;
using System.Text;
using OneScope.Sdk;
using SceneFile = RecursiveAI.OnePhysics.Scene.Scene;

namespace RecursiveAI.OnePhysics.Scope;

/// <summary>
/// The OnePhysics module for the OneScope viewer. It opens a OnePhysics replay (checkpoint or command, 2D or 3D), a world snapshot
/// or a scene (authored or exact-state), and plays it with the library's own stepping, snapshots and world hash.
/// A step's input is any pokes made on a fork plus the recorded state changes; the time step is carried in the input only when it changes.
/// </summary>
public sealed class OnePhysicsModule : IScopeModule
{
    /// <summary>The module id, <c>"onephysics"</c>.</summary>
    public const string ModuleId = "onephysics";

    private readonly int _threads;

    /// <summary>Creates the module.</summary>
    /// <param name="threadCount">Threads the worlds it makes may use. The result never depends on it; default 1.</param>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="threadCount"/> is less than 1.</exception>
    public OnePhysicsModule(int threadCount = 1)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(threadCount, 1);
        _threads = threadCount;
    }

    /// <inheritdoc />
    public string Id => ModuleId;

    /// <inheritdoc />
    public string Name => "OnePhysics";

    /// <inheritdoc />
    /// <exception cref="ScopeFormatException">The bytes are not a replay, snapshot or scene, or do not make a valid world.</exception>
    public IScopeSource Open(ReadOnlyMemory<byte> data, string name)
    {
        ArgumentNullException.ThrowIfNull(name);
        ReadOnlySpan<byte> s = data.Span;
        if (s.StartsWith("OPPR"u8))
        {
            return ReplayImport.Open(data.ToArray(), name, _threads);
        }

        if (s.StartsWith("OPSN"u8))
        {
            return OpenSnapshot(data.ToArray(), name);
        }

        return OpenScene(data, name);
    }

    private PhysicsSource OpenSnapshot(byte[] bytes, string name)
    {
        // A snapshot does not say which engine wrote it, so try the spatial engine and then the planar one.
        ScopeFormatException? last = null;
        foreach (int dimension in new[] { 3, 2 })
        {
            WorldSetup setup = Setup(name, dimension, SetupKind.Snapshot, bytes);
            try
            {
                setup.CreateWorld(_threads);
                return Make(setup);
            }
            catch (ScopeFormatException e)
            {
                last = e;
            }
        }

        throw new ScopeFormatException(name, "expected a OnePhysics replay, snapshot or scene; the snapshot is not valid for either engine (" + last!.Message + ")");
    }

    private PhysicsSource OpenScene(ReadOnlyMemory<byte> data, string name)
    {
        string text;
        try
        {
            text = new UTF8Encoding(false, true).GetString(data.Span);
        }
        catch (ArgumentException)
        {
            throw new ScopeFormatException(name, "expected a OnePhysics replay (starting 'OPPR'), snapshot (starting 'OPSN') or scene text.");
        }

        int dimension;
        try
        {
            dimension = SceneFile.Read(new System.IO.StringReader(text), name).Dim;
        }
        catch (Exception e) when (e is PhysicsException or ArgumentException or System.IO.InvalidDataException)
        {
            throw new ScopeFormatException(name, "expected a OnePhysics replay, snapshot or scene: " + e.Message);
        }

        WorldSetup setup = Setup(name, dimension, SetupKind.SceneText, data.ToArray());
        setup.CreateWorld(_threads);
        return Make(setup);
    }

    private static WorldSetup Setup(string name, int dimension, SetupKind kind, byte[] data) => new()
    {
        Name = name,
        Dimension = dimension,
        Kind = kind,
        Data = data,
        DefaultDt = 1f / 60f,
        States = [],
    };

    private PhysicsSource Make(WorldSetup setup) => new(setup, Recording.Create(setup.Header(), []), _threads);

    internal PhysicsSource Make(WorldSetup setup, Recording recording) => new(setup, recording, _threads);
}
