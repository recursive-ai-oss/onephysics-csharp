using System;
using System.Collections.Generic;
using System.Globalization;

namespace RecursiveAI.OnePhysics.Core;

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

/// <summary>
/// Slot allocator with generations. Freed slots are reused lowest index first (a binary min-heap), so the arrays that hang off the table
/// stay compact and the allocation sequence is a pure function of the create/destroy sequence.
/// A slot whose generation would overflow is retired and never reused.
/// </summary>
public sealed class HandleTable
{
    private int[] _generation = new int[16];
    private bool[] _live = new bool[16];
    private int[] _heap = new int[16];
    private int _heapCount;
    private int _highWater;
    private int _liveCount;

    /// <summary>Number of live slots.</summary>
    public int Count => _liveCount;

    /// <summary>Makes room for slots up to <paramref name="capacity"/> (and as many free slots to reuse) so that allocating and freeing below it never grows an array.</summary>
    internal void Reserve(int capacity)
    {
        if (_generation.Length < capacity) { Array.Resize(ref _generation, capacity); Array.Resize(ref _live, capacity); }
        if (_heap.Length < capacity) { Array.Resize(ref _heap, capacity); }
    }

    /// <summary>One more than the highest slot index ever allocated; arrays indexed by slot are valid below this.</summary>
    public int HighWater => _highWater;

    /// <summary>Allocates the lowest free slot, or a new one above the high-water mark, and returns its index.</summary>
    public int Allocate()
    {
        int index;
        if (_heapCount > 0)
        {
            index = PopMin();
        }
        else
        {
            index = _highWater++;
            if (index == _generation.Length)
            {
                Array.Resize(ref _generation, index * 2);
                Array.Resize(ref _live, index * 2);
            }

            _generation[index] = 1;
        }

        _live[index] = true;
        _liveCount++;
        return index;
    }

    /// <summary>Generation of a slot below <see cref="HighWater"/>.</summary>
    public int GenerationOf(int index) => _generation[index];

    /// <summary>True when the slot is below the high-water mark and live.</summary>
    public bool IsLive(int index) => (uint)index < (uint)_highWater && _live[index];

    /// <summary>True when the slot is live with exactly this generation.</summary>
    public bool IsAlive(int index, int generation) => IsLive(index) && _generation[index] == generation;

    /// <summary>Frees a live slot: its generation increments so older handles go stale, and the slot becomes available for reuse.</summary>
    public void Free(int index)
    {
        if (!IsLive(index)) { throw new InvalidOperationException("slot " + index.ToString(CultureInfo.InvariantCulture) + " is not live"); }
        _live[index] = false;
        _liveCount--;
        if (_generation[index] == int.MaxValue)
        {
            return; // retired: the generation cannot advance, so the slot is never reused
        }

        _generation[index]++;
        Push(index);
    }

    private void Push(int v)
    {
        if (_heapCount == _heap.Length) { Array.Resize(ref _heap, _heapCount * 2); }
        int i = _heapCount++;
        while (i > 0)
        {
            int p = (i - 1) >> 1;
            if (_heap[p] <= v) { break; }
            _heap[i] = _heap[p];
            i = p;
        }

        _heap[i] = v;
    }

    private int PopMin()
    {
        int min = _heap[0];
        int last = _heap[--_heapCount];
        int i = 0;
        while (true)
        {
            int c = 2 * i + 1;
            if (c >= _heapCount) { break; }
            if (c + 1 < _heapCount && _heap[c + 1] < _heap[c]) { c++; }
            if (_heap[c] >= last) { break; }
            _heap[i] = _heap[c];
            i = c;
        }

        if (_heapCount > 0) { _heap[i] = last; }
        return min;
    }

    /// <summary>Writes the table: high-water mark, per-slot generation and liveness, and the free heap in its exact layout (the layout decides future allocations).</summary>
    public void Write(StateWriter w)
    {
        w.WriteInt32(_highWater);
        w.WriteInt32s(_generation.AsSpan(0, _highWater));
        for (int i = 0; i < _highWater; i++) { w.WriteBool(_live[i]); }
        w.WriteInt32(_heapCount);
        w.WriteInt32s(_heap.AsSpan(0, _heapCount));
    }

    /// <summary>Reads and validates a table written by <see cref="Write"/>: generations at least 1, free list a valid min-heap of distinct dead slots, every dead slot either free or retired.</summary>
    public static HandleTable Read(StateReader r, string what)
    {
        var t = new HandleTable();
        t.ReadInto(r, what);
        return t;
    }

    private bool[] _seenScratch = Array.Empty<bool>();

    /// <summary>Forgets every slot, as in a new table (storage is kept).</summary>
    internal void Reset()
    {
        _liveCount = 0;
        _heapCount = 0;
        _highWater = 0;
    }

    /// <summary>
    /// Like <see cref="Read"/> but fills this table, keeping its arrays when they are big enough, so a world restoring a snapshot into a spare table allocates nothing once warm.
    /// The table's contents are undefined after a failure; only a table nobody uses (a spare) may be read into.
    /// </summary>
    internal void ReadInto(StateReader r, string what)
    {
        _liveCount = 0;
        _heapCount = 0;
        _highWater = 0;
        int high = r.ReadCount(5, what, " slots");
        if (_generation.Length < Math.Max(16, high))
        {
            int cap = Math.Max(16, high);
            _generation = new int[cap];
            _live = new bool[cap];
        }

        r.ReadInt32s(_generation.AsSpan(0, high));
        for (int i = 0; i < high; i++)
        {
            if (_generation[i] < 1) { throw r.Error("invalid generation in " + what + " table"); }
            _live[i] = r.ReadBool();
            if (_live[i]) { _liveCount++; }
        }

        _highWater = high;
        int free = r.ReadCount(4, what, " free list");
        if (_heap.Length < Math.Max(16, free)) { _heap = new int[Math.Max(16, free)]; }
        r.ReadInt32s(_heap.AsSpan(0, free));
        _heapCount = free;
        if (_seenScratch.Length < high) { _seenScratch = new bool[Math.Max(high, _seenScratch.Length * 2)]; }
        var seen = _seenScratch.AsSpan(0, high);
        seen.Clear();
        for (int i = 0; i < free; i++)
        {
            int v = _heap[i];
            if ((uint)v >= (uint)high || _live[v] || seen[v]) { throw r.Error("invalid free-list entry in " + what + " table"); }
            seen[v] = true;
            if (i > 0 && _heap[(i - 1) >> 1] > v) { throw r.Error("free list of " + what + " table is not a heap"); }
        }

        for (int i = 0; i < high; i++)
        {
            if (!_live[i] && !seen[i] && _generation[i] != int.MaxValue) { throw r.Error("dead slot missing from the free list of " + what + " table"); }
        }
    }

    /// <summary>Appends the indices of all live slots in ascending order to <paramref name="into"/>.</summary>
    public void CollectLive(List<int> into)
    {
        for (int i = 0; i < _highWater; i++) { if (_live[i]) { into.Add(i); } }
    }
}
