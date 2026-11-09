using System;
using System.Globalization;

namespace RecursiveAI.OnePhysics;


/// <summary>Handle to a bicycle: the id of the world that created it, the slot index and the slot's generation. The default value is the null handle and is never valid. Get it from <c>Bicycle.RawHandle</c>; turn it back into the bicycle with <c>world.GetVehicle</c>.</summary>
public readonly struct BicycleHandle : IEquatable<BicycleHandle>
{
    /// <summary>Id of the owning world (at least 1 for a real handle).</summary>
    public int World { get; }

    /// <summary>Slot index in the world's bicycle table.</summary>
    public int Index { get; }

    /// <summary>Generation of the slot when the bicycle was created (at least 1 for a real handle).</summary>
    public int Generation { get; }

    /// <summary>Creates a handle from its parts.</summary>
    public BicycleHandle(int world, int index, int generation) { World = world; Index = index; Generation = generation; }

    /// <summary>True for the default (null) handle.</summary>
    public bool IsNull => Generation == 0;

    /// <inheritdoc/>
    public bool Equals(BicycleHandle other) => World == other.World && Index == other.Index && Generation == other.Generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is BicycleHandle h && Equals(h);

    /// <inheritdoc/>
    public override int GetHashCode() => unchecked(World * 73856093 ^ Index * 19349663 ^ Generation * 83492791);

    /// <inheritdoc/>
    public override string ToString() => "Bicycle(world " + World.ToString(CultureInfo.InvariantCulture) + ", #" + Index.ToString(CultureInfo.InvariantCulture) + ", gen " + Generation.ToString(CultureInfo.InvariantCulture) + ")";

    /// <summary>Equality of all three parts.</summary>
    public static bool operator ==(BicycleHandle a, BicycleHandle b) => a.Equals(b);

    /// <summary>Inequality of any part.</summary>
    public static bool operator !=(BicycleHandle a, BicycleHandle b) => !a.Equals(b);
}


/// <summary>Handle to a track: the id of the world that created it, the slot index and the slot's generation. The default value is the null handle and is never valid. Get it from <c>Track.RawHandle</c>; turn it back into the track with <c>world.GetVehicle</c>.</summary>
public readonly struct TrackHandle : IEquatable<TrackHandle>
{
    /// <summary>Id of the owning world (at least 1 for a real handle).</summary>
    public int World { get; }

    /// <summary>Slot index in the world's track table.</summary>
    public int Index { get; }

    /// <summary>Generation of the slot when the track was created (at least 1 for a real handle).</summary>
    public int Generation { get; }

    /// <summary>Creates a handle from its parts.</summary>
    public TrackHandle(int world, int index, int generation) { World = world; Index = index; Generation = generation; }

    /// <summary>True for the default (null) handle.</summary>
    public bool IsNull => Generation == 0;

    /// <inheritdoc/>
    public bool Equals(TrackHandle other) => World == other.World && Index == other.Index && Generation == other.Generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is TrackHandle h && Equals(h);

    /// <inheritdoc/>
    public override int GetHashCode() => unchecked(World * 73856093 ^ Index * 19349663 ^ Generation * 83492791);

    /// <inheritdoc/>
    public override string ToString() => "Track(world " + World.ToString(CultureInfo.InvariantCulture) + ", #" + Index.ToString(CultureInfo.InvariantCulture) + ", gen " + Generation.ToString(CultureInfo.InvariantCulture) + ")";

    /// <summary>Equality of all three parts.</summary>
    public static bool operator ==(TrackHandle a, TrackHandle b) => a.Equals(b);

    /// <summary>Inequality of any part.</summary>
    public static bool operator !=(TrackHandle a, TrackHandle b) => !a.Equals(b);
}


/// <summary>Handle to a rail vehicle: the id of the world that created it, the slot index and the slot's generation. The default value is the null handle and is never valid. Get it from <c>RailVehicle.RawHandle</c>; turn it back into the rail vehicle with <c>world.GetVehicle</c>.</summary>
public readonly struct RailVehicleHandle : IEquatable<RailVehicleHandle>
{
    /// <summary>Id of the owning world (at least 1 for a real handle).</summary>
    public int World { get; }

    /// <summary>Slot index in the world's rail vehicle table.</summary>
    public int Index { get; }

    /// <summary>Generation of the slot when the rail vehicle was created (at least 1 for a real handle).</summary>
    public int Generation { get; }

    /// <summary>Creates a handle from its parts.</summary>
    public RailVehicleHandle(int world, int index, int generation) { World = world; Index = index; Generation = generation; }

    /// <summary>True for the default (null) handle.</summary>
    public bool IsNull => Generation == 0;

    /// <inheritdoc/>
    public bool Equals(RailVehicleHandle other) => World == other.World && Index == other.Index && Generation == other.Generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is RailVehicleHandle h && Equals(h);

    /// <inheritdoc/>
    public override int GetHashCode() => unchecked(World * 73856093 ^ Index * 19349663 ^ Generation * 83492791);

    /// <inheritdoc/>
    public override string ToString() => "RailVehicle(world " + World.ToString(CultureInfo.InvariantCulture) + ", #" + Index.ToString(CultureInfo.InvariantCulture) + ", gen " + Generation.ToString(CultureInfo.InvariantCulture) + ")";

    /// <summary>Equality of all three parts.</summary>
    public static bool operator ==(RailVehicleHandle a, RailVehicleHandle b) => a.Equals(b);

    /// <summary>Inequality of any part.</summary>
    public static bool operator !=(RailVehicleHandle a, RailVehicleHandle b) => !a.Equals(b);
}
