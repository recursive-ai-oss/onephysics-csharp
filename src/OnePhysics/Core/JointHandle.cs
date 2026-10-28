using System;
using System.Collections.Generic;
using System.Globalization;

using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>Handle to a joint (a constraint of kind <see cref="ConstraintKind.Joint"/>); same layout and rules as <see cref="BodyHandle"/>. The index is the constraint index.</summary>
public readonly struct JointHandle : IEquatable<JointHandle>
{
    /// <summary>Id of the owning world.</summary>
    public int World { get; }

    /// <summary>Constraint index.</summary>
    public int Index { get; }

    /// <summary>Generation of the slot when the joint was created.</summary>
    public int Generation { get; }

    /// <summary>Creates a handle from its parts.</summary>
    public JointHandle(int world, int index, int generation) { World = world; Index = index; Generation = generation; }

    /// <summary>True for the default (null) handle.</summary>
    public bool IsNull => Generation == 0;

    /// <inheritdoc/>
    public bool Equals(JointHandle other) => World == other.World && Index == other.Index && Generation == other.Generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is JointHandle h && Equals(h);

    /// <inheritdoc/>
    public override int GetHashCode() => unchecked(World * 73856093 ^ Index * 19349663 ^ Generation * 83492791);

    /// <inheritdoc/>
    public override string ToString() => "Joint(world " + World.ToString(CultureInfo.InvariantCulture) + ", #" + Index.ToString(CultureInfo.InvariantCulture) + ", gen " + Generation.ToString(CultureInfo.InvariantCulture) + ")";

    /// <summary>Equality of all three parts.</summary>
    public static bool operator ==(JointHandle a, JointHandle b) => a.Equals(b);

    /// <summary>Inequality of any part.</summary>
    public static bool operator !=(JointHandle a, JointHandle b) => !a.Equals(b);
}
