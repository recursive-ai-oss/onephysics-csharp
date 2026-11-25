using System;

namespace RecursiveAI.OnePhysics;

/// <summary>
/// A saved state of one world, taken by <c>Mark()</c> and returned to by <c>RollbackTo(mark)</c>. A small value (world id, slot, generation); it holds no memory itself, the world keeps the saved state until the mark is released.
/// A mark stays valid across any number of rollbacks to it. It becomes invalid when it is released, when a rollback to an earlier mark of the same world passes it, or when the world drops its marks;
/// using an invalid mark throws <see cref="StaleHandleException"/>. The default value is a null mark and throws <see cref="InvalidHandleException"/>.
/// </summary>
public readonly struct WorldMark : IEquatable<WorldMark>
{
    private readonly int _world;
    private readonly int _slot;
    private readonly int _generation;
    private readonly long _step;

    internal WorldMark(int world, int slot, int generation, long step)
    {
        _world = world; _slot = slot + 1; _generation = generation; _step = step;
    }

    /// <summary>True for the default value, which names no state.</summary>
    public bool IsNull => _slot == 0;

    /// <summary>The id of the world that took the mark.</summary>
    public int World => _world;

    /// <summary>The world's <c>StepCount</c> when the mark was taken: the step counter a rollback returns to.</summary>
    public long Step => _step;

    internal int Slot => _slot - 1;

    internal int Generation => _generation;

    /// <inheritdoc />
    public bool Equals(WorldMark other) => _world == other._world && _slot == other._slot && _generation == other._generation;

    /// <inheritdoc />
    public override bool Equals(object? obj) => obj is WorldMark m && Equals(m);

    /// <inheritdoc />
    public override int GetHashCode() => unchecked((((_world * 31) + _slot) * 31) + _generation);

    /// <summary>Equality of two marks (same world, slot and generation).</summary>
    public static bool operator ==(WorldMark a, WorldMark b) => a.Equals(b);

    /// <summary>Inequality of two marks.</summary>
    public static bool operator !=(WorldMark a, WorldMark b) => !a.Equals(b);

    /// <inheritdoc />
    public override string ToString() => IsNull ? "WorldMark(null)" : FormattableString.Invariant($"WorldMark(world {_world}, step {_step}, slot {_slot - 1}, generation {_generation})");
}
