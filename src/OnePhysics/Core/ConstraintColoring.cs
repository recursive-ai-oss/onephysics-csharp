using System;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

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
[Experimental("OPHYS0001")]
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

    /// <summary>Sizes the lists a step builds for up to <paramref name="constraints"/> constraints and <paramref name="bodies"/> bodies, so that steps within those counts allocate nothing.</summary>
    internal void Reserve(int constraints, int bodies)
    {
        EnsureBodies(bodies);
        if (_list.Length < constraints) { Array.Resize(ref _list, constraints); }
        if (_bucketOf.Length < constraints) { Array.Resize(ref _bucketOf, constraints); }
        int hist = ((constraints + ActiveBlock - 1) / ActiveBlock) * HistogramStride;
        if (_hist.Length < hist) { Array.Resize(ref _hist, hist); }
    }

    internal void ResetBody(int body) => _mask[body] = 0;

    internal void CopyFrom(ConstraintColoring other)
    {
        (_mask, other._mask) = (other._mask, _mask);
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
    internal void BuildActive(BodyStore bodies, ConstraintStore cons, byte[] births, byte[] newPoints, TaskExecutor? executor = null, bool anyAwake = true)
    {
        Array.Clear(_start); _activeBlocks = 0;
        // A constraint is active only through an awake dynamic body; with no awake or kinematic body at all (a world that has fallen asleep) nothing is, and the scan can be skipped.
        if (!anyAwake) { return; }
        int n = cons.HighWater;
        if (_bucketOf.Length < n) { _bucketOf = new byte[Math.Max(n, _bucketOf.Length * 2)]; }
        var bucketOf = _bucketOf;

        // Whether a constraint is active is looked up (through its two bodies) once, in blocks that write only their own slice of bucketOf and their own row of the histogram.
        // The row of a block says how many of its constraints fall in each bucket; the buckets are then laid out block by block, so every bucket lists its constraints in ascending index
        // whatever the number of threads, and a second pass fills each block's share of the list.
        int blocks = (n + ActiveBlock - 1) / ActiveBlock;
        const int Buckets = ColorCount + 1;
        _activeBlocks = blocks;
        if (_hist.Length < blocks * HistogramStride) { _hist = new int[Math.Max(blocks * HistogramStride, _hist.Length * 2)]; }
        var count = new ActiveBlockTask(bodies, cons, bucketOf, _hist, births, newPoints);
        if (executor != null && blocks > 1) { executor.For(blocks, count); }
        else { for (int blk = 0; blk < blocks; blk++) { count.Run(blk); } }

        int total = 0;
        for (int bucket = 0; bucket < Buckets; bucket++)
        {
            _start[bucket] = total;
            for (int blk = 0; blk < blocks; blk++)
            {
                int c = _hist[blk * HistogramStride + bucket];
                _hist[blk * HistogramStride + bucket] = total; // from here on: where this block's first constraint of the bucket goes
                total += c;
            }
        }

        _start[Buckets] = total;
        if (total > _list.Length) { _list = new int[Math.Max(total, _list.Length * 2)]; }
        var fill = new ActiveFillTask(bucketOf, _hist, _list, n);
        if (executor != null && blocks > 1) { executor.For(blocks, fill); }
        else { for (int blk = 0; blk < blocks; blk++) { fill.Run(blk); } }
    }

    private int[] _hist = new int[HistogramStride * 8];
    private int _activeBlocks;

    // Fold diagnostic counts into the existing active scan rather than walking the solve list again serially.
    internal void CaptureCounts(StepCounters counters)
    {
        int joints = 0, contacts = 0, points = 0, births = 0, newPoints = 0;
        for (int b = 0; b < _activeBlocks; b++)
        {
            int o = b * HistogramStride + ColorCount + 1;
            joints += _hist[o]; contacts += _hist[o + 1]; points += _hist[o + 2];
            births += _hist[o + 3]; newPoints += _hist[o + 4];
        }
        counters.Joints = joints; counters.Manifolds = contacts; counters.ContactPoints = points;
        counters.NewManifolds = births; counters.NewContactPoints = newPoints;
    }

    private readonly struct ActiveFillTask : IParallelBody
    {
        private readonly byte[] _bucketOf; private readonly int[] _hist; private readonly int[] _list; private readonly int _n;

        public ActiveFillTask(byte[] bucketOf, int[] hist, int[] list, int n) { _bucketOf = bucketOf; _hist = hist; _list = list; _n = n; }

        public void Run(int block)
        {
            const int Buckets = ColorCount + 1;
            var cursor = _hist.AsSpan(block * HistogramStride, Buckets);
            int lo = block * ActiveBlock, hi = Math.Min(lo + ActiveBlock, _n);
            for (int c = lo; c < hi; c++)
            {
                int bucket = _bucketOf[c];
                if (bucket == Inactive) { continue; }
                _list[cursor[bucket]++] = c;
            }
        }
    }

    private const byte Inactive = 255;
    private const int ActiveBlock = 512;
    private const int HistogramStride = ColorCount + 1 + 5;
    private byte[] _bucketOf = new byte[64];

    private readonly struct ActiveBlockTask : IParallelBody
    {
        private readonly BodyStore _bodies; private readonly ConstraintStore _cons; private readonly byte[] _bucketOf, _births, _newPoints; private readonly int[] _hist;

        public ActiveBlockTask(BodyStore bodies, ConstraintStore cons, byte[] bucketOf, int[] hist, byte[] births, byte[] newPoints) { _bodies = bodies; _cons = cons; _bucketOf = bucketOf; _hist = hist; _births = births; _newPoints = newPoints; }

        public void Run(int block)
        {
            var cons = _cons;
            var kind = cons.Kind; var color = cons.Color; var ba = cons.BodyA; var bb = cons.BodyB;
            var row = _hist.AsSpan(block * HistogramStride, HistogramStride);
            row.Clear();
            var points = cons.PointCount;
            int joints = 0, contacts = 0, pointCount = 0, births = 0, newPoints = 0;
            int lo = block * ActiveBlock, hi = Math.Min(lo + ActiveBlock, cons.HighWater);
            for (int c = lo; c < hi; c++)
            {
                if (IsActive(_bodies, kind[c], color[c], ba[c], bb[c]))
                {
                    int bucket = Bucket(color[c]); _bucketOf[c] = (byte)bucket; row[bucket]++;
                    if (kind[c] == (byte)ConstraintKind.Joint) { joints++; }
                    else { contacts++; pointCount += points[c]; births += _births[c]; newPoints += _newPoints[c]; }
                }
                else { _bucketOf[c] = Inactive; }
            }
            row[ColorCount + 1] = joints; row[ColorCount + 2] = contacts; row[ColorCount + 3] = pointCount;
            row[ColorCount + 4] = births; row[ColorCount + 5] = newPoints;
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

    internal int ColorSize(int color) => _start[2 + color] - _start[1 + color];

    internal int OverflowStart => _start[0];

    internal int[] ListArray => _list;
}
