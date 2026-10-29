using System;
using System.Globalization;

namespace RecursiveAI.OnePhysics;

/// <summary>Handle to a vehicle: the id of the world that created it, the slot index and the slot's generation. The default value is the null handle and is never valid. Get it from <c>Vehicle.RawHandle</c>; turn it back into the vehicle with <c>world.GetVehicle</c>.</summary>
public readonly struct VehicleHandle : IEquatable<VehicleHandle>
{
    /// <summary>Id of the owning world (at least 1 for a real handle).</summary>
    public int World { get; }

    /// <summary>Slot index in the world's vehicle table.</summary>
    public int Index { get; }

    /// <summary>Generation of the slot when the vehicle was created (at least 1 for a real handle).</summary>
    public int Generation { get; }

    /// <summary>Creates a handle from its parts.</summary>
    public VehicleHandle(int world, int index, int generation) { World = world; Index = index; Generation = generation; }

    /// <summary>True for the default (null) handle.</summary>
    public bool IsNull => Generation == 0;

    /// <inheritdoc/>
    public bool Equals(VehicleHandle other) => World == other.World && Index == other.Index && Generation == other.Generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is VehicleHandle h && Equals(h);

    /// <inheritdoc/>
    public override int GetHashCode() => unchecked(World * 73856093 ^ Index * 19349663 ^ Generation * 83492791);

    /// <inheritdoc/>
    public override string ToString() => "Vehicle(world " + World.ToString(CultureInfo.InvariantCulture) + ", #" + Index.ToString(CultureInfo.InvariantCulture) + ", gen " + Generation.ToString(CultureInfo.InvariantCulture) + ")";

    /// <summary>Equality of all three parts.</summary>
    public static bool operator ==(VehicleHandle a, VehicleHandle b) => a.Equals(b);

    /// <summary>Inequality of any part.</summary>
    public static bool operator !=(VehicleHandle a, VehicleHandle b) => !a.Equals(b);
}
