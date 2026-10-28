using System;
using System.Collections.Generic;
using System.Globalization;

using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>Handle to a shape; same layout and rules as <see cref="BodyHandle"/>.</summary>
public readonly struct ShapeHandle : IEquatable<ShapeHandle>
{
    /// <summary>Id of the owning world.</summary>
    public int World { get; }

    /// <summary>Slot index in the world's shape arrays.</summary>
    public int Index { get; }

    /// <summary>Generation of the slot when the shape was created.</summary>
    public int Generation { get; }

    /// <summary>Creates a handle from its parts.</summary>
    public ShapeHandle(int world, int index, int generation) { World = world; Index = index; Generation = generation; }

    /// <summary>True for the default (null) handle.</summary>
    public bool IsNull => Generation == 0;

    /// <inheritdoc/>
    public bool Equals(ShapeHandle other) => World == other.World && Index == other.Index && Generation == other.Generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is ShapeHandle h && Equals(h);

    /// <inheritdoc/>
    public override int GetHashCode() => unchecked(World * 73856093 ^ Index * 19349663 ^ Generation * 83492791);

    /// <inheritdoc/>
    public override string ToString() => "Shape(world " + World.ToString(CultureInfo.InvariantCulture) + ", #" + Index.ToString(CultureInfo.InvariantCulture) + ", gen " + Generation.ToString(CultureInfo.InvariantCulture) + ")";

    /// <summary>Equality of all three parts.</summary>
    public static bool operator ==(ShapeHandle a, ShapeHandle b) => a.Equals(b);

    /// <summary>Inequality of any part.</summary>
    public static bool operator !=(ShapeHandle a, ShapeHandle b) => !a.Equals(b);
}
