using System;
using System.Globalization;

namespace RecursiveAI.OnePhysics;

/// <summary>Handle to a character: the id of the world that created it, the slot index and the slot's generation. The default value is the null handle and is never valid. Get it from <c>Character.RawHandle</c>; turn it back into the character with <c>world.GetCharacter</c>.</summary>
public readonly struct CharacterHandle : IEquatable<CharacterHandle>
{
    /// <summary>Id of the owning world (at least 1 for a real handle).</summary>
    public int World { get; }

    /// <summary>Slot index in the world's character table.</summary>
    public int Index { get; }

    /// <summary>Generation of the slot when the character was created (at least 1 for a real handle).</summary>
    public int Generation { get; }

    /// <summary>Creates a handle from its parts.</summary>
    public CharacterHandle(int world, int index, int generation) { World = world; Index = index; Generation = generation; }

    /// <summary>True for the default (null) handle.</summary>
    public bool IsNull => Generation == 0;

    /// <inheritdoc/>
    public bool Equals(CharacterHandle other) => World == other.World && Index == other.Index && Generation == other.Generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is CharacterHandle h && Equals(h);

    /// <inheritdoc/>
    public override int GetHashCode() => unchecked(World * 73856093 ^ Index * 19349663 ^ Generation * 83492791);

    /// <inheritdoc/>
    public override string ToString() => "Character(world " + World.ToString(CultureInfo.InvariantCulture) + ", #" + Index.ToString(CultureInfo.InvariantCulture) + ", gen " + Generation.ToString(CultureInfo.InvariantCulture) + ")";

    /// <summary>Equality of all three parts.</summary>
    public static bool operator ==(CharacterHandle a, CharacterHandle b) => a.Equals(b);

    /// <summary>Inequality of any part.</summary>
    public static bool operator !=(CharacterHandle a, CharacterHandle b) => !a.Equals(b);
}
