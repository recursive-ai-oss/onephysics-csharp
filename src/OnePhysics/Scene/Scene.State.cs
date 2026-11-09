using System;
using System.IO;
using System.Text;
using System.Collections.Generic;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Scene;

/// <summary>Malformed scene input. Line numbers are one-based and input names are supplied by the caller.</summary>
public sealed class SceneFormatException : PhysicsException
{
    /// <summary>Creates an error identifying the input, line and expected record or value.</summary>
    public SceneFormatException(string inputName, int line, string expected)
        : base($"Scene '{inputName}', line {line}: expected {expected}.") { InputName = inputName; Line = line; }
    /// <summary>The source name, such as a file path.</summary>
    public string InputName { get; }
    /// <summary>One-based line number, or 1 for an authoring object not read from text.</summary>
    public int Line { get; }
}

public sealed partial class Scene
{
    private const int MaximumSnapshotBytes = 64 * 1024 * 1024;
    private byte[]? _state;
    private byte[]? _liveOptions;
    internal void SetLiveOptions(byte[] bytes) => _liveOptions = bytes;
    private int _stateWritten;
    private bool _stateEnd, _dimSeen, _idSeen;
    private int _worldId = 1;
    internal string InputName { get; set; } = "scene";
    internal int ActiveLine { get; set; } = 1;
    /// <summary>The text version, 1 for authoring records, 2 for an exact full-state scene. New authoring objects default to 1.</summary>
    public int Version { get; private set; } = 1;
    /// <summary>True if this scene contains complete serialized world state rather than authoring records.</summary>
    public bool HasWorldState => _state != null;
    /// <summary>The snapshot bytes of a full-state scene, empty for authoring scenes. Borrowed, read-only data.</summary>
    public ReadOnlySpan<byte> WorldState => _state;

    /// <summary>Wraps complete snapshot bytes in a full-state scene. Dimension must be 2 or 3, id positive, payload nonempty and at most 64 MiB. Snapshot contents are validated on world creation.</summary>
    public static Scene FromSnapshot(int dimension, int worldId, ReadOnlySpan<byte> snapshot)
    {
        if (dimension != 2 && dimension != 3) throw new ArgumentOutOfRangeException(nameof(dimension));
        ArgumentOutOfRangeException.ThrowIfLessThan(worldId, 1);
        if (snapshot.IsEmpty || snapshot.Length > MaximumSnapshotBytes) throw new ArgumentOutOfRangeException(nameof(snapshot));
        return new Scene { Version = 2, Dim = dimension, _worldId = worldId, _state = snapshot.ToArray(), _stateWritten = snapshot.Length, _stateEnd = true, _dimSeen = true, _idSeen = true };
    }

    /// <summary>Builds a planar world from authoring records or restores the full saved world. Thread count defaults to 1 and is not part of scene state. Invalid content throws SceneFormatException.</summary>
    public P2.PhysicsWorld2D CreateWorld2D(int threadCount = 1)
    {
        if (Dim != 2) throw Error("dimension 2");
        Validate();
        try
        {
            if (_state != null)
            {
                var options = new P2.WorldOptions2D { ThreadCount = threadCount, WorldId = _worldId };
                if (_liveOptions != null) options.BroadphaseMargin = Replay.ReplayOptions2D.Read(_liveOptions, options, out _);
                Replay.ReplayOptions2D.PrepareCreation(options);
                var w = new P2.PhysicsWorld2D(options); w.Restore(_state);
                if (_liveOptions != null) ((Replay.IRecordedWorld)w).ApplyReplayOptions(_liveOptions);
                return w;
            }
            return SceneBuilder2D.Build(this, threadCount).World;
        }
        catch (Exception e) when (e is ArgumentException or PhysicsException or InvalidDataException) { throw Error("valid planar world: " + e.Message); }
    }

    /// <summary>Builds a spatial world including authored characters, vehicles, fluid volumes and soft bodies, or restores all saved state. Thread count defaults to 1. Invalid content throws SceneFormatException.</summary>
    public P3.PhysicsWorld3D CreateWorld3D(int threadCount = 1)
    {
        if (Dim != 3) throw Error("dimension 3");
        Validate();
        try
        {
            if (_state != null)
            {
                var options = new P3.WorldOptions3D { ThreadCount = threadCount, WorldId = _worldId };
                if (_liveOptions != null) options.BroadphaseMargin = Replay.ReplayOptions3D.Read(_liveOptions, options, out _);
                Replay.ReplayOptions3D.PrepareCreation(options);
                var w = new P3.PhysicsWorld3D(options); w.Restore(_state);
                if (_liveOptions != null) ((Replay.IRecordedWorld)w).ApplyReplayOptions(_liveOptions);
                return w;
            }
            var world = SceneBuilder3D.Build(this, threadCount).World;
            SceneBuilder3D.AddSofts(this, world);
            return world;
        }
        catch (Exception e) when (e is ArgumentException or PhysicsException or InvalidDataException) { throw Error("valid spatial world: " + e.Message); }
    }

    private SceneFormatException Error(string expected) => new(InputName, Math.Max(1, ActiveLine), expected);

    private static IEnumerable<string> ReadLines(TextReader input, string name)
    {
        var line = new StringBuilder(); int number = 1, total = 0;
        for (int c; (c = input.Read()) >= 0;)
        {
            if (++total > 128 * 1024 * 1024) throw new SceneFormatException(name, number, "at most 128 MiB of scene characters");
            if (c == '\n') { yield return line.ToString().TrimEnd('\r'); line.Clear(); number++; }
            else { if (line.Length >= 1024 * 1024) throw new SceneFormatException(name, number, "line at most 1 MiB"); line.Append((char)c); }
        }
        if (line.Length > 0) yield return line.ToString().TrimEnd('\r');
    }

    private void ReadStateRecord(string[] tokens)
    {
        if (_stateEnd) throw new FormatException("no records after end");
        if (tokens[0] == "dim" && tokens.Length == 2 && !_dimSeen && _state == null) { Dim = I(tokens[1]); if (Dim != 2 && Dim != 3) throw new FormatException("dimension 2 or 3"); _dimSeen = true; }
        else if (tokens[0] == "world_id" && tokens.Length == 2 && !_idSeen && _state == null) { _worldId = I(tokens[1]); if (_worldId < 1) throw new FormatException("positive world id"); _idSeen = true; }
        else if (tokens[0] == "options" && tokens.Length == 2 && _liveOptions == null && _state == null && _dimSeen && _idSeen)
        {
            int size = Dim == 2 ? Replay.ReplayOptions2D.SizeWithSoft : Replay.ReplayOptions3D.Size;
            var bytes = new byte[size];
            if (!Convert.TryFromBase64String(tokens[1], bytes, out int n) || (n != size && !(Dim == 2 && n == Replay.ReplayOptions2D.Size))) throw new FormatException("complete live option payload");
            _liveOptions = n == size ? bytes : bytes.AsSpan(0, n).ToArray();
        }
        else if (tokens[0] == "state" && tokens.Length == 2 && _state == null && _dimSeen && _idSeen)
        {
            int length = I(tokens[1]); if (length < 1 || length > MaximumSnapshotBytes) throw new FormatException("state length in [1, 67108864]");
            _state = new byte[length];
        }
        else if (tokens[0] == "data" && tokens.Length == 2 && _state != null)
        {
            if (tokens[1].Length > 4096 || !Convert.TryFromBase64String(tokens[1], _state.AsSpan(_stateWritten), out int n)) throw new FormatException("base64 data fitting declared state length (at most 4096 characters per chunk)");
            _stateWritten += n;
        }
        else if (tokens[0] == "end" && tokens.Length == 1 && _state != null && _stateWritten == _state.Length) _stateEnd = true;
        else throw new FormatException("dim, world_id, state, data, or end record in order");
    }

    private void WriteState(TextWriter output)
    {
        Validate();
        output.Write("scene 2\ndim "); output.Write(Dim.ToString(System.Globalization.CultureInfo.InvariantCulture));
        output.Write("\nworld_id "); output.Write(_worldId.ToString(System.Globalization.CultureInfo.InvariantCulture));
        output.Write('\n');
        if (_liveOptions != null) { output.Write("options "); output.Write(Convert.ToBase64String(_liveOptions)); output.Write('\n'); }
        output.Write("state "); output.Write(_state!.Length.ToString(System.Globalization.CultureInfo.InvariantCulture)); output.Write('\n');
        for (int i = 0; i < _state.Length; i += 3072) { output.Write("data "); output.Write(Convert.ToBase64String(_state, i, Math.Min(3072, _state.Length - i))); output.Write('\n'); }
        output.Write("end\n");
    }
}
