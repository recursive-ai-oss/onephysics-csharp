using System.Collections.Generic;

namespace RecursiveAI.OnePhysics;

/// <summary>
/// Holds the callback sets of destroyed bodies, joints and characters so that a restore which brings the same handle back can give them back.
/// Bounded: when it is full the oldest entry is forgotten. A world only fills it once it has taken a snapshot (a world that never snapshots has nothing to restore).
/// </summary>
internal sealed class HandlerGraveyard<T> where T : class
{
    /// <summary>Most destroyed handles whose callbacks are remembered.</summary>
    internal const int Capacity = 1024;

    private readonly List<(int Index, int Generation, T Set)> _entries = new();

    internal int Count => _entries.Count;

    internal (int Index, int Generation, T Set) this[int i] => _entries[i];

    internal void RemoveAt(int i) => _entries.RemoveAt(i);

    internal void Add(int index, int generation, T set)
    {
        for (int i = 0; i < _entries.Count; i++)
        {
            if (_entries[i].Index == index && _entries[i].Generation == generation) { _entries[i] = (index, generation, set); return; }
        }

        if (_entries.Count == Capacity) { _entries.RemoveAt(0); }
        _entries.Add((index, generation, set));
    }
}
