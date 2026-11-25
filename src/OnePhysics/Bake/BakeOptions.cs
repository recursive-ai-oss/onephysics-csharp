using System;
using System.Globalization;

namespace RecursiveAI.OnePhysics.Bake;

/// <summary>Which tracks a bake records. Flags combine.</summary>
[Flags]
public enum BakeTracks
{
    /// <summary>Nothing.</summary>
    None = 0,
    /// <summary>Position and rotation of every body, every frame.</summary>
    Rigid = 1,
    /// <summary>Particle positions of every soft body and cloth, with their surface triangles once.</summary>
    Soft = 2,
    /// <summary>Particle positions of every rod, rope, cable and hair strand.</summary>
    Rods = 4,
    /// <summary>Position of every fluid and granular particle.</summary>
    Particles = 8,
    /// <summary>Fracture records (when, which body, which fragments) and the geometry of each fragment, once.</summary>
    Fracture = 16,
    /// <summary>The world's events (contacts, sensors, joint breaks, sleep and wake), by step.</summary>
    Events = 32,
    /// <summary>Every track.</summary>
    All = 63,
}

/// <summary>
/// How a bake is recorded: which tracks, how often, and how coarsely the numbers are stored. The defaults keep a long clip compact
/// (positions to about 8 micrometres, rotation components to about 8e-6) and are documented in docs/spec/bake.md.
/// </summary>
public sealed class BakeOptions
{
    /// <summary>The tracks to record. Default <see cref="BakeTracks.All"/>.</summary>
    public BakeTracks Tracks { get; set; } = BakeTracks.All;

    /// <summary>One frame is recorded every this many steps (1 to 1024). Default 1. Events are recorded for every step whatever the stride.</summary>
    public int FrameStride { get; set; } = 1;

    /// <summary>The fixed step in seconds the world is stepped with, finite and positive. Default 1/60. A step with another dt marks the bake faulted (<see cref="BakeRecorder.Finish()"/> then throws).</summary>
    public float Dt { get; set; } = 1f / 60f;

    /// <summary>Rigid-body positions are stored as multiples of 2^PositionExponent metres (-40 to 0). Default -16 (about 15 micrometres, error at most half of that). The world's own positions lie on a 2^-32 grid (a body origin is that grid point minus a rotated float offset), so -40 reproduces a position to about 5e-13 m, which is as exact as a double holds at the world's extent.</summary>
    public int PositionExponent { get; set; } = -16;

    /// <summary>Soft, cloth, rod and particle positions are stored as multiples of 2^VertexExponent metres (-40 to 0). Default -14 (about 61 micrometres, error at most half of that).</summary>
    public int VertexExponent { get; set; } = -14;

    /// <summary>Rotation components (quaternion or cosine and sine) are stored as multiples of 2^-RotationBits (1 to 30), or as exact float32 bit patterns when 0. Default 16 (error at most 7.7e-6 per component, below 0.001 degrees).</summary>
    public int RotationBits { get; set; } = 16;

    /// <summary>Frames per independently coded block (1 to 4096). A reader decodes one block per entity to sample a frame, so smaller blocks sample faster and compress slightly worse. Default 64.</summary>
    public int BlockFrames { get; set; } = 64;

    /// <summary>Which events are recorded when <see cref="BakeTracks.Events"/> is on. Default <see cref="EventMask.Default"/> (everything but contact persistence). The world must also be collecting them (its own event mask).</summary>
    public EventMask EventKinds { get; set; } = EventMask.Default;

    /// <summary>The name of the scene, stored in the header (at most 1024 UTF-8 bytes). Default empty.</summary>
    public string SceneName { get; set; } = "";

    /// <summary>The SHA-256 of the replay file that reproduces this bake (see <see cref="BakeRecorder.DigestOf"/>), 32 bytes, or null for none. Stored in the header.</summary>
    public byte[]? ReplayDigest { get; set; }

    /// <summary>The library version stored in the header. Null (default) stores this assembly's version; a fixed text makes the bytes independent of the version.</summary>
    public string? LibraryVersion { get; set; }

    /// <summary>Checks every member; the recorder calls it.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A member is out of range, or the replay digest is not 32 bytes.</exception>
    public void Validate()
    {
        if ((Tracks & ~BakeTracks.All) != 0) { throw new ArgumentOutOfRangeException(nameof(Tracks), Tracks, "Tracks has unknown bits."); }
        if (FrameStride < 1 || FrameStride > 1024) { throw new ArgumentOutOfRangeException(nameof(FrameStride), FrameStride, "FrameStride must be 1 to 1024."); }
        if (!(Dt > 0f) || !float.IsFinite(Dt)) { throw new ArgumentOutOfRangeException(nameof(Dt), Dt, "Dt must be finite and positive."); }
        if (PositionExponent < -40 || PositionExponent > 0) { throw new ArgumentOutOfRangeException(nameof(PositionExponent), PositionExponent, "PositionExponent must be -40 to 0."); }
        if (VertexExponent < -40 || VertexExponent > 0) { throw new ArgumentOutOfRangeException(nameof(VertexExponent), VertexExponent, "VertexExponent must be -40 to 0."); }
        if (RotationBits < 0 || RotationBits > 30) { throw new ArgumentOutOfRangeException(nameof(RotationBits), RotationBits, "RotationBits must be 0 to 30."); }
        if (BlockFrames < 1 || BlockFrames > BakeFormat.MaxBlockFrames) { throw new ArgumentOutOfRangeException(nameof(BlockFrames), BlockFrames, "BlockFrames must be 1 to 4096."); }
        ArgumentNullException.ThrowIfNull(SceneName);
        if (System.Text.Encoding.UTF8.GetByteCount(SceneName) > BakeFormat.MaxStringBytes) { throw new ArgumentOutOfRangeException(nameof(SceneName), "SceneName is longer than 1024 UTF-8 bytes."); }
        if (ReplayDigest != null && ReplayDigest.Length != 32) { throw new ArgumentOutOfRangeException(nameof(ReplayDigest), ReplayDigest.Length, "ReplayDigest must be 32 bytes."); }
        if (LibraryVersion != null && System.Text.Encoding.UTF8.GetByteCount(LibraryVersion) > BakeFormat.MaxStringBytes) { throw new ArgumentOutOfRangeException(nameof(LibraryVersion), "LibraryVersion is longer than 1024 UTF-8 bytes."); }
    }

    internal BakeOptions Copy() => new()
    {
        Tracks = Tracks, FrameStride = FrameStride, Dt = Dt, PositionExponent = PositionExponent, VertexExponent = VertexExponent, RotationBits = RotationBits,
        BlockFrames = BlockFrames, EventKinds = EventKinds, SceneName = SceneName, ReplayDigest = (byte[]?)ReplayDigest?.Clone(), LibraryVersion = LibraryVersion,
    };
}

/// <summary>A bake cache file is malformed. Names the field and the byte offset where it went wrong.</summary>
public sealed class BakeFormatException : FormatException
{
    /// <summary>Creates the exception.</summary>
    public BakeFormatException(string field, long offset, string problem)
        : base("Bake cache: " + field + " at byte " + offset.ToString(CultureInfo.InvariantCulture) + ": " + problem + ".")
    {
        Field = field; Offset = offset; Problem = problem;
    }

    /// <summary>The field that was wrong.</summary>
    public string Field { get; }

    /// <summary>Byte offset in the file where the problem was found.</summary>
    public long Offset { get; }

    /// <summary>What was wrong.</summary>
    public string Problem { get; }
}

/// <summary>A value the bake cannot store: not finite, or beyond the range the format holds.</summary>
public sealed class BakeRangeException : ArgumentOutOfRangeException
{
    /// <summary>Creates the exception.</summary>
    public BakeRangeException(string what, double value)
        : base(what, value, "Bake: " + what + " is " + value.ToString("R", CultureInfo.InvariantCulture) + ", which the format cannot store (not finite or too large for the chosen quantum).") { }
}
