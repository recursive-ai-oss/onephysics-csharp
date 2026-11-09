using System;
using System.Globalization;

namespace RecursiveAI.OnePhysics;

/// <summary>What happens at a side of a shallow-water grid.</summary>
public enum WaterBoundaryKind : byte
{
    /// <summary>A closed wall: nothing passes, waves reflect. The default.</summary>
    Wall = 0,

    /// <summary>Open: waves leave without reflecting much and the water level is not held (zero-gradient outflow, and inflow of whatever is just inside).</summary>
    Open = 1,

    /// <summary>Inflow: water enters at a fixed level and speed, like a reservoir pushing in at a fixed discharge (depth over the bed times speed). Waves that reach it reflect; use <see cref="Level"/> for an edge waves leave through.</summary>
    Inflow = 2,

    /// <summary>A held water level, such as the sea or a reservoir: the surface at the side stays at the boundary's level and the flow through it follows from the water just inside, in or out.</summary>
    Level = 3,
}

/// <summary>One side of a shallow-water grid and what happens there.</summary>
public readonly struct WaterBoundary : IEquatable<WaterBoundary>
{
    /// <summary>Creates a boundary.</summary>
    public WaterBoundary(WaterBoundaryKind kind, double level = 0.0, float speed = 0f) { Kind = kind; Level = level; Speed = speed; }

    /// <summary>The kind.</summary>
    public WaterBoundaryKind Kind { get; }

    /// <summary>The surface height held (<see cref="WaterBoundaryKind.Level"/>) or fed in (<see cref="WaterBoundaryKind.Inflow"/>), metres (world y).</summary>
    public double Level { get; }

    /// <summary>For <see cref="WaterBoundaryKind.Inflow"/>: the speed of the incoming water into the grid, m/s (negative flows out). Within ±1000.</summary>
    public float Speed { get; }

    /// <summary>A closed wall.</summary>
    public static WaterBoundary Wall => default;

    /// <summary>An open edge.</summary>
    public static WaterBoundary Open => new(WaterBoundaryKind.Open);

    /// <summary>An inflow edge: water of surface height <paramref name="level"/> enters at <paramref name="speed"/> (m/s into the grid), a discharge of the depth over the bed there times the speed.</summary>
    public static WaterBoundary Inflow(double level, float speed) => new(WaterBoundaryKind.Inflow, level, speed);

    /// <summary>An edge that holds the water surface at <paramref name="level"/> (metres, world y), letting the flow through it follow.</summary>
    public static WaterBoundary HeldLevel(double level) => new(WaterBoundaryKind.Level, level);

    /// <inheritdoc/>
    public bool Equals(WaterBoundary other) => Kind == other.Kind && Level.Equals(other.Level) && Speed.Equals(other.Speed);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is WaterBoundary b && Equals(b);

    /// <inheritdoc/>
    public override int GetHashCode() => HashCode.Combine(Kind, Level, Speed);

    /// <summary>Equality of all parts.</summary>
    public static bool operator ==(WaterBoundary a, WaterBoundary b) => a.Equals(b);

    /// <summary>Inequality of any part.</summary>
    public static bool operator !=(WaterBoundary a, WaterBoundary b) => !a.Equals(b);
}

/// <summary>A source (positive <see cref="Rate"/>) or drain (negative) of water.</summary>
public readonly struct WaterSource
{
    /// <summary>Creates a source.</summary>
    public WaterSource(double x, double z, float radius, float rate) { X = x; Z = z; Radius = radius; Rate = rate; }

    /// <summary>World x of the source, metres.</summary>
    public double X { get; }

    /// <summary>World z of the source, metres. Ignored in a 2D world, whose water lies along x.</summary>
    public double Z { get; }

    /// <summary>Radius of the cells it acts on, metres; 0 acts on the one cell holding the point.</summary>
    public float Radius { get; }

    /// <summary>Cubic metres per second (square metres per second in 2D). A drain takes only the water that is there.</summary>
    public float Rate { get; }
}

/// <summary>Handle to a body of shallow water: the id of the world that created it, the slot index and the slot's generation. The default value is the null handle and is never valid.</summary>
public readonly struct WaterHandle : IEquatable<WaterHandle>
{
    /// <summary>Creates a handle from its parts.</summary>
    public WaterHandle(int world, int index, int generation) { World = world; Index = index; Generation = generation; }

    /// <summary>Id of the owning world.</summary>
    public int World { get; }

    /// <summary>Slot index in the world's water table.</summary>
    public int Index { get; }

    /// <summary>Generation of the slot when the water was created (at least 1 for a real handle).</summary>
    public int Generation { get; }

    /// <summary>True for the default (null) handle.</summary>
    public bool IsNull => Generation == 0;

    /// <inheritdoc/>
    public bool Equals(WaterHandle other) => World == other.World && Index == other.Index && Generation == other.Generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is WaterHandle h && Equals(h);

    /// <inheritdoc/>
    public override int GetHashCode() => HashCode.Combine(World, Index, Generation);

    /// <inheritdoc/>
    public override string ToString() => "Water(world " + World.ToString(CultureInfo.InvariantCulture) + ", #" + Index.ToString(CultureInfo.InvariantCulture) + ", gen " + Generation.ToString(CultureInfo.InvariantCulture) + ")";

    /// <summary>Equality of all three parts.</summary>
    public static bool operator ==(WaterHandle a, WaterHandle b) => a.Equals(b);

    /// <summary>Inequality of any part.</summary>
    public static bool operator !=(WaterHandle a, WaterHandle b) => !a.Equals(b);
}

/// <summary>Whether a body went into the water or came out of it.</summary>
public enum WaterEventKind : byte
{
    /// <summary>The body's submerged volume went from zero to positive.</summary>
    Entered = 0,

    /// <summary>The body's submerged volume went from positive to zero.</summary>
    Left = 1,
}

/// <summary>A body entering or leaving a body of shallow water during a step.</summary>
public readonly struct WaterEvent
{
    /// <summary>Creates an event.</summary>
    public WaterEvent(WaterEventKind kind, BodyHandle body, WaterHandle water, float volume) { Kind = kind; Body = body; Water = water; Volume = volume; }

    /// <summary>Entered or left.</summary>
    public WaterEventKind Kind { get; }

    /// <summary>The body.</summary>
    public BodyHandle Body { get; }

    /// <summary>The water.</summary>
    public WaterHandle Water { get; }

    /// <summary>Entering: the volume submerged at the end of the step's start pose; leaving: the volume that was submerged before. Cubic metres (square metres in 2D).</summary>
    public float Volume { get; }
}
