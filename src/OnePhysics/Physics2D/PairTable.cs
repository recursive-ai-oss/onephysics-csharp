using System;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// A map from a pair key to its slot in the engine's pair table: open addressing with linear probing and backward-shift deletion, so a lookup touches one or two cache lines and nothing allocates once it has grown.
/// Key and value share one 16-byte entry, so a probe is one memory access rather than one per parallel array. Keys are never negative (two non-negative shape indices packed), and -1 marks an empty entry.
/// It is only ever probed by key, never enumerated, so its internal order cannot reach a result.
/// </summary>
internal sealed class PairTable
{
    private struct Entry
    {
        public long Key;
        public int Value;
    }

    private const long Empty = -1;

    private Entry[] _entries = NewEntries(64);
    private int _count;
    private int _shift = 64 - 6;

    public int Count => _count;

    private static Entry[] NewEntries(int n)
    {
        var e = new Entry[n];
        Array.Fill(e, new Entry { Key = Empty });
        return e;
    }

    private int Slot(long key) => (int)(((ulong)key * 0x9E3779B97F4A7C15UL) >> _shift);

    /// <summary>Makes room for <paramref name="count"/> entries without growing.</summary>
    public void Reserve(int count)
    {
        while (count * 2 > _entries.Length) { Grow(); }
    }

    public bool TryGetValue(long key, out int value)
    {
        var entries = _entries;
        int mask = entries.Length - 1;
        for (int i = Slot(key); ; i = (i + 1) & mask)
        {
            ref Entry e = ref entries[i];
            if (e.Key == key) { value = e.Value; return true; }
            if (e.Key == Empty) { value = 0; return false; }
        }
    }

    public bool ContainsKey(long key) => TryGetValue(key, out _);

    /// <summary>Adds or replaces. The key must not be negative.</summary>
    public void Set(long key, int value)
    {
        if ((_count + 1) * 2 > _entries.Length) { Grow(); }
        var entries = _entries;
        int mask = entries.Length - 1;
        int i = Slot(key);
        while (entries[i].Key != Empty)
        {
            if (entries[i].Key == key) { entries[i].Value = value; return; }
            i = (i + 1) & mask;
        }

        entries[i].Key = key; entries[i].Value = value;
        _count++;
    }

    public void Remove(long key)
    {
        var entries = _entries;
        int mask = entries.Length - 1;
        int i = Slot(key);
        while (entries[i].Key != Empty && entries[i].Key != key) { i = (i + 1) & mask; }
        if (entries[i].Key == Empty) { return; }
        // Backward-shift deletion: pull later entries of the probe run into the hole when their home slot allows it.
        int hole = i;
        for (int j = (i + 1) & mask; entries[j].Key != Empty; j = (j + 1) & mask)
        {
            int home = Slot(entries[j].Key);
            // Entry j may move to the hole unless its home lies cyclically in (hole, j].
            bool inRange = hole <= j ? (home > hole && home <= j) : (home > hole || home <= j);
            if (inRange) { continue; }
            entries[hole] = entries[j];
            hole = j;
        }

        entries[hole].Key = Empty;
        _count--;
    }

    public void Clear()
    {
        Array.Fill(_entries, new Entry { Key = Empty });
        _count = 0;
    }

    private void Grow()
    {
        var old = _entries;
        _entries = NewEntries(old.Length * 2);
        _shift--;
        _count = 0;
        for (int i = 0; i < old.Length; i++)
        {
            if (old[i].Key != Empty) { Set(old[i].Key, old[i].Value); }
        }
    }
}
