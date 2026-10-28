using System;

namespace RecursiveAI.OnePhysics.Core;

/// <summary>
/// Deterministic graph coloring of the constraint graph. Two constraints that share a dynamic body never share a color, so all constraints of one color can be solved in any order, or in parallel,
/// and give the same bits. Static and kinematic bodies do not take part in the coloring (many constraints touch the ground), the solver must only read them.
/// </summary>
/// <remarks>
/// The assignment is a pure function of the order in which constraints were added and removed: a new constraint takes the lowest free color on both its bodies
/// (Catto, Solver2D / Box2D v3 graph coloring posts). Constraints touching one dynamic body prefer the last <c>ColorCount - DynamicColors</c> colors so that
/// contacts with the ground are solved late within an iteration, which improves stacking. Colors are stored per constraint and saved in snapshots.
/// A constraint that finds no free color goes to the overflow set, solved serially before the colors.
/// Nothing here refers to thread count or vector width: <see cref="Lanes"/> only says how many consecutive constraints of one color form a batch, and kernels of any hardware width process a batch.
/// </remarks>
public sealed class ConstraintColoring
{
    /// <summary>Number of colors before constraints overflow.</summary>
    public const int ColorCount = 24;

    /// <summary>Colors below this index are preferred by constraints between two dynamic bodies; the rest are preferred by constraints touching a single dynamic body.</summary>
    public const int DynamicColors = 20;

    /// <summary>Logical lane count of a batch: consecutive constraints of one color, in ascending constraint index. A constant of the format, not of the hardware.</summary>
    public const int Lanes = 8;

    /// <summary>Batches handed to one parallel task. Fixed, so the split of work never depends on the thread count.</summary>
    public const int BatchesPerTask = 4;

    private ulong[] _mask = new ulong[64];
    private int[] _list = new int[64];
    private readonly int[] _start = new int[ColorCount + 2];
    private readonly int[] _cursor = new int[ColorCount + 1];

    internal void EnsureBodies(int highWater)
    {
        if (highWater > _mask.Length) { Array.Resize(ref _mask, Math.Max(highWater, _mask.Length * 2)); }
    }

    internal void ResetBody(int body) => _mask[body] = 0;

    internal void CopyFrom(ConstraintColoring other)
    {
        _mask = other._mask;
        Array.Clear(_start);
    }

    /// <summary>The colors in use on a body (bit c set: some constraint of color c touches it); zero for non-dynamic bodies.</summary>
    public ulong ColorMask(int body) => _mask[body];

    internal byte Assign(int a, bool aDynamic, int b, bool bDynamic)
    {
        if (!aDynamic && !bDynamic) { return ConstraintStore.Uncolored; }
        ulong used = (aDynamic ? _mask[a] : 0UL) | (bDynamic ? _mask[b] : 0UL);
        int color = -1;
        if (aDynamic && bDynamic)
        {
            color = FirstFree(used, 0, ColorCount);
        }
        else
        {
            color = FirstFree(used, DynamicColors, ColorCount);
            if (color < 0) { color = FirstFree(used, 0, DynamicColors); }
        }

        if (color < 0) { return ConstraintStore.Overflow; }
        if (aDynamic) { _mask[a] |= 1UL << color; }
        if (bDynamic) { _mask[b] |= 1UL << color; }
        return (byte)color;
    }

    private static int FirstFree(ulong used, int from, int to)
    {
        for (int c = from; c < to; c++)
        {
            if ((used & (1UL << c)) == 0) { return c; }
        }

        return -1;
    }

    internal void Release(byte color, int a, bool aDynamic, int b, bool bDynamic)
    {
        if (color >= ColorCount) { return; }
        ulong bit = ~(1UL << color);
        if (aDynamic) { _mask[a] &= bit; }
        if (bDynamic) { _mask[b] &= bit; }
    }

    /// <summary>Rebuilds the per-body masks from stored colors (after a restore). Returns false when two constraints on one dynamic body share a color.</summary>
    internal bool RebuildMasks(BodyStore bodies, ConstraintStore cons)
    {
        EnsureBodies(bodies.HighWater);
        Array.Clear(_mask);
        for (int c = 0; c < cons.HighWater; c++)
        {
            if (!cons.IsLive(c)) { continue; }
            byte col = cons.Color[c];
            if (col >= ColorCount) { continue; }
            ulong bit = 1UL << col;
            int a = cons.BodyA[c], b = cons.BodyB[c];
            if (bodies.IsDynamic(a)) { if ((_mask[a] & bit) != 0) { return false; } _mask[a] |= bit; }
            if (bodies.IsDynamic(b)) { if ((_mask[b] & bit) != 0) { return false; } _mask[b] |= bit; }
        }

        return true;
    }

    /// <summary>Buckets the active constraints (not sensors, not uncolored, at least one awake dynamic body) by color, each bucket in ascending constraint index.</summary>
    internal void BuildActive(BodyStore bodies, ConstraintStore cons, TaskExecutor? executor = null, bool anyAwake = true)
    {
        Array.Clear(_start);
        // A constraint is active only through an awake dynamic body; with no awake or kinematic body at all (a world that has fallen asleep) nothing is, and the scan can be skipped.
        if (!anyAwake) { return; }
        int n = cons.HighWater;
        if (_bucketOf.Length < n) { _bucketOf = new byte[Math.Max(n, _bucketOf.Length * 2)]; }
        var bucketOf = _bucketOf;

        // Whether a constraint is active is looked up (through its two bodies) once, in blocks that write only their own slice of bucketOf; the counting and the fill then read the answers.
        int blocks = (n + ActiveBlock - 1) / ActiveBlock;
        var task = new ActiveBlockTask(bodies, cons, bucketOf);
        if (executor != null && blocks > 1) { executor.For(blocks, task); }
        else { for (int blk = 0; blk < blocks; blk++) { task.Run(blk); } }

        int total = 0;
        for (int c = 0; c < n; c++)
        {
            int bucket = bucketOf[c];
            if (bucket == Inactive) { continue; }
            _start[bucket + 1]++;
            total++;
        }

        for (int i = 1; i < _start.Length; i++) { _start[i] += _start[i - 1]; }
        if (total > _list.Length) { _list = new int[Math.Max(total, _list.Length * 2)]; }
        var cursor = _cursor;
        for (int i = 0; i < cursor.Length; i++) { cursor[i] = _start[i]; }
        for (int c = 0; c < n; c++)
        {
            int bucket = bucketOf[c];
            if (bucket == Inactive) { continue; }
            _list[cursor[bucket]++] = c;
        }
    }

    private const byte Inactive = 255;
    private const int ActiveBlock = 4096;
    private byte[] _bucketOf = new byte[64];

    private readonly struct ActiveBlockTask : IParallelBody
    {
        private readonly BodyStore _bodies; private readonly ConstraintStore _cons; private readonly byte[] _bucketOf;

        public ActiveBlockTask(BodyStore bodies, ConstraintStore cons, byte[] bucketOf) { _bodies = bodies; _cons = cons; _bucketOf = bucketOf; }

        public void Run(int block)
        {
            var cons = _cons;
            var kind = cons.Kind; var color = cons.Color; var ba = cons.BodyA; var bb = cons.BodyB;
            int lo = block * ActiveBlock, hi = Math.Min(lo + ActiveBlock, cons.HighWater);
            for (int c = lo; c < hi; c++)
            {
                _bucketOf[c] = IsActive(_bodies, kind[c], color[c], ba[c], bb[c]) ? (byte)Bucket(color[c]) : Inactive;
            }
        }
    }

    private static int Bucket(byte color) => color == ConstraintStore.Overflow ? 0 : 1 + color;

    private static bool IsActive(BodyStore bodies, byte kind, byte color, int a, int b)
    {
        if (kind == 0 || kind == (byte)ConstraintKind.Sensor || color == ConstraintStore.Uncolored) { return false; }
        return (bodies.IsDynamic(a) && bodies.IsAwake(a)) || (bodies.IsDynamic(b) && bodies.IsAwake(b));
    }

    /// <summary>Total number of active constraints from the last <c>BuildActive</c>.</summary>
    public int ActiveCount => _start[ColorCount + 1];

    /// <summary>All active constraints in solve order: the overflow set, then color 0, 1, and so on; ascending index within each.</summary>
    public ReadOnlySpan<int> Active => _list.AsSpan(0, _start[ColorCount + 1]);

    /// <summary>The overflow constraints, solved serially.</summary>
    public ReadOnlySpan<int> Overflow => _list.AsSpan(_start[0], _start[1] - _start[0]);

    /// <summary>The active constraints of one color, ascending by index.</summary>
    public ReadOnlySpan<int> Color(int color) => _list.AsSpan(_start[1 + color], _start[2 + color] - _start[1 + color]);

    internal int ColorStart(int color) => _start[1 + color];

    internal int OverflowStart => _start[0];

    internal int[] ListArray => _list;
}
