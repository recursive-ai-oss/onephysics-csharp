using System;
using System.Collections.Generic;
using System.Globalization;

using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>Handle to a body: the id of the world that created it, the slot index and the slot's generation. The default value is the null handle and is never valid.</summary>
public readonly struct BodyHandle : IEquatable<BodyHandle>
{
    /// <summary>Id of the owning world (at least 1 for a real handle).</summary>
    public int World { get; }

    /// <summary>Slot index in the world's body arrays.</summary>
    public int Index { get; }

    /// <summary>Generation of the slot when the body was created (at least 1 for a real handle).</summary>
    public int Generation { get; }

    /// <summary>Creates a handle from its parts.</summary>
    public BodyHandle(int world, int index, int generation) { World = world; Index = index; Generation = generation; }

    /// <summary>True for the default (null) handle.</summary>
    public bool IsNull => Generation == 0;

    /// <inheritdoc/>
    public bool Equals(BodyHandle other) => World == other.World && Index == other.Index && Generation == other.Generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is BodyHandle h && Equals(h);

    /// <inheritdoc/>
    public override int GetHashCode() => unchecked(World * 73856093 ^ Index * 19349663 ^ Generation * 83492791);

    /// <inheritdoc/>
    public override string ToString() => "Body(world " + World.ToString(CultureInfo.InvariantCulture) + ", #" + Index.ToString(CultureInfo.InvariantCulture) + ", gen " + Generation.ToString(CultureInfo.InvariantCulture) + ")";

    /// <summary>Equality of all three parts.</summary>
    public static bool operator ==(BodyHandle a, BodyHandle b) => a.Equals(b);

    /// <summary>Inequality of any part.</summary>
    public static bool operator !=(BodyHandle a, BodyHandle b) => !a.Equals(b);
}
