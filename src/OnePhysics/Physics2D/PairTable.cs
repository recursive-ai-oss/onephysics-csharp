using System;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// A map from a pair key to its slot in the engine's pair table: open addressing with linear probing and backward-shift deletion, so lookups touch one or two cache lines and nothing allocates once it has grown.
/// It is only ever probed by key, never enumerated, so its internal order cannot reach a result.
/// </summary>
internal sealed class PairTable
{
    private long[] _keys = new long[64];
    private int[] _values = new int[64];
    private bool[] _used = new bool[64];
    private int _count;
    private int _shift = 64 - 6;

    public int Count => _count;

    private int Slot(long key) => (int)(((ulong)key * 0x9E3779B97F4A7C15UL) >> _shift);

    public bool TryGetValue(long key, out int value)
    {
        int mask = _keys.Length - 1;
        for (int i = Slot(key); _used[i]; i = (i + 1) & mask)
        {
            if (_keys[i] == key) { value = _values[i]; return true; }
        }

        value = 0;
        return false;
    }

    public bool ContainsKey(long key) => TryGetValue(key, out _);

    /// <summary>Adds or replaces.</summary>
    public void Set(long key, int value)
    {
        if ((_count + 1) * 2 > _keys.Length) { Grow(); }
        int mask = _keys.Length - 1;
        int i = Slot(key);
        while (_used[i])
        {
            if (_keys[i] == key) { _values[i] = value; return; }
            i = (i + 1) & mask;
        }

        _used[i] = true; _keys[i] = key; _values[i] = value;
        _count++;
    }

    public void Remove(long key)
    {
        int mask = _keys.Length - 1;
        int i = Slot(key);
        while (_used[i] && _keys[i] != key) { i = (i + 1) & mask; }
        if (!_used[i]) { return; }
        // Backward-shift deletion: pull later entries of the probe run into the hole when their home slot allows it.
        int hole = i;
        for (int j = (i + 1) & mask; _used[j]; j = (j + 1) & mask)
        {
            int home = Slot(_keys[j]);
            // Entry j may move to the hole unless its home lies cyclically in (hole, j].
            bool inRange = hole <= j ? (home > hole && home <= j) : (home > hole || home <= j);
            if (inRange) { continue; }
            _keys[hole] = _keys[j]; _values[hole] = _values[j];
            hole = j;
        }

        _used[hole] = false;
        _count--;
    }

    public void Clear()
    {
        Array.Clear(_used);
        _count = 0;
    }

    private void Grow()
    {
        var keys = _keys; var values = _values; var used = _used;
        int cap = keys.Length * 2;
        _keys = new long[cap]; _values = new int[cap]; _used = new bool[cap];
        _shift--;
        _count = 0;
        for (int i = 0; i < keys.Length; i++)
        {
            if (used[i]) { Set(keys[i], values[i]); }
        }
    }
}
