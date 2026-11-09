using System;

namespace RecursiveAI.OnePhysics;

/// <summary>
/// A sorted cell grid for the soft-body surface search: items are keyed by the hash of their cell (shifted up) and their number (low 24 bits), sorted ascending, cut into runs of equal cell hash and looked up through
/// an open-addressing table. The order of the items inside a run and the order a caller visits cells in are fixed, so a search over the grid lists its hits in an order that depends only on the items.
/// </summary>
internal sealed class SoftCellGrid
{
    /// <summary>Most items: the item number takes 24 bits of the key.</summary>
    internal const int MaxItems = 1 << 24;

    /// <summary>Keys the caller fills with <c>(Hash(cell) &lt;&lt; 24) | item</c> before calling <see cref="Build"/>.</summary>
    public long[] Keys = new long[64];
    private long[] _cell = new long[65];
    private int[] _start = new int[65];
    private int[] _table = new int[128];
    private int _mask;
    private int _runs;

    public void Grow(int n)
    {
        if (Keys.Length < n) { Array.Resize(ref Keys, Math.Max(n, 2 * Keys.Length)); }
    }

    /// <summary>Hash of an integer cell, 39 bits.</summary>
    public static long Hash(long cx, long cy, long cz)
    {
        ulong h = (ulong)cx * 0x9E3779B97F4A7C15UL ^ (ulong)cy * 0xC2B2AE3D27D4EB4FUL ^ (ulong)cz * 0x165667B19E3779F9UL;
        h ^= h >> 31; h *= 0x94D049BB133111EBUL; h ^= h >> 29;
        return (long)(h & ((1UL << 39) - 1));
    }

    /// <summary>Sorts the first <paramref name="n"/> keys and builds the table.</summary>
    public void Build(int n)
    {
        Array.Sort(Keys, 0, n);
        if (_cell.Length < n + 1) { _cell = new long[n + 1 + n / 2]; _start = new int[_cell.Length]; }
        int runs = 0;
        for (int e = 0; e < n; e++)
        {
            long h = Keys[e] >> 24;
            if (e == 0 || h != (Keys[e - 1] >> 24)) { _cell[runs] = h; _start[runs] = e; runs++; }
        }

        _start[runs] = n;
        _runs = runs;
        int size = 16;
        while (size < 2 * runs) { size <<= 1; }
        if (_table.Length < size) { _table = new int[size]; }
        Array.Fill(_table, -1, 0, size);
        _mask = size - 1;
        for (int u = 0; u < runs; u++)
        {
            int slot = (int)(_cell[u] & _mask);
            while (_table[slot] >= 0) { slot = (slot + 1) & _mask; }
            _table[slot] = u;
        }
    }

    /// <summary>The keys <c>[start, end)</c> of the run with the given cell hash; false when there is none.</summary>
    public bool Find(long hash, out int start, out int end)
    {
        int slot = (int)(hash & _mask);
        while (_table[slot] >= 0)
        {
            int run = _table[slot];
            if (_cell[run] == hash) { start = _start[run]; end = _start[run + 1]; return true; }
            slot = (slot + 1) & _mask;
        }

        start = end = 0;
        return false;
    }
}
