using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsQuery3D
{
    // Arrays, rather than pinned spans, keep worker lifetimes and bounds checks explicit.
    // One object owns and reuses its buffers; independent objects can run concurrently.
    private readonly object _batchLock = new();
    private bool _batchActive;
    private QueryInput3D[] _batchInputs = Array.Empty<QueryInput3D>();
    private QueryHit3D[] _batchHits = Array.Empty<QueryHit3D>();
    private int[] _batchCounts = Array.Empty<int>();

    /// <summary>Broadphase-only box selection; see the world method for limits and slot ordering.</summary>
    public int QueryAabb(Vector3D minimum, Vector3D maximum, Span<Shape> results) => _world.QueryAabb(minimum, maximum, results, in _filter);

    /// <summary>Deepest contact penetration; see the world's Penetration method for leaf and direction semantics.</summary>
    public bool Penetration(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, out QueryHit3D hit) => _world.Penetration(in shape, position, rotation, out hit, in _filter);

    private readonly struct MixedBatch : IParallelBody
    {
        private readonly PhysicsWorld3D _world;
        private readonly QueryInput3D[] _inputs;
        private readonly QueryHit3D[] _hits;
        private readonly int[] _counts;
        private readonly QueryFilter3D _filter;
        private readonly int _count, _stride;
        private readonly QueryMode _mode;

        public MixedBatch(PhysicsWorld3D world, QueryInput3D[] inputs, QueryHit3D[] hits, int[] counts, QueryFilter3D filter, int count, int stride, QueryMode mode)
        { _world = world; _inputs = inputs; _hits = hits; _counts = counts; _filter = filter; _count = count; _stride = stride; _mode = mode; }

        public void Run(int task)
        {
            int start = task * 64;
            int end = start + Math.Min(64, _count - start);
            for (int i = start; i < end; i++)
            {
                ref readonly var input = ref _inputs[i];
                var scratch = _world.QueryEngine.Run(in input.Params, in _filter, _mode, _stride);
                try
                {
                    _counts[i] = scratch.Count;
                    int offset = i * _stride;
                    for (int j = 0; j < scratch.Count; j++) { _hits[offset + j] = _world.ToPublic(in scratch.Hits[j], input.Length); }
                }
                finally { scratch.Release(); }
            }
        }
    }

    /// <summary>Runs mixed ray, cast, overlap and closest-point queries in parallel. Each output slot is the nearest result for the same input slot, or default on a miss. Returns the number of inputs with a hit. Predicates must be pure and thread-safe. Concurrent or callback-reentrant batches on the same query object throw InvalidOperationException. Inputs are copied before execution; output changes only after success. Buffers are reused after capacity growth.</summary>
    /// <exception cref="ArgumentException">The output is too short or an input is default.</exception>
    /// <exception cref="InvalidOperationException">A predicate reenters a batch on this query object.</exception>
    public int QueryBatch(ReadOnlySpan<QueryInput3D> inputs, Span<QueryHit3D> hits)
    {
        if (hits.Length < inputs.Length) { throw new ArgumentException("One output slot is required per input.", nameof(hits)); }
        if (!System.Threading.Monitor.TryEnter(_batchLock)) { throw new InvalidOperationException("This query object is already running a batch; use a separate query object for concurrent batches."); }
        try
        {
            ExecuteMixed(inputs, 1, QueryMode.Closest);
            _batchHits.AsSpan(0, inputs.Length).CopyTo(hits);
            int count = 0;
            for (int i = 0; i < inputs.Length; i++) { if (_batchCounts[i] != 0) { count++; } }
            return count;
        }
        finally { System.Threading.Monitor.Exit(_batchLock); }
    }

    /// <summary>Runs mixed queries as existence tests in parallel, storing one boolean per input. Same validation, concurrency and allocation contract as QueryBatch.</summary>
    public void QueryAnyBatch(ReadOnlySpan<QueryInput3D> inputs, Span<bool> hits)
    {
        if (hits.Length < inputs.Length) { throw new ArgumentException("One output slot is required per input.", nameof(hits)); }
        if (!System.Threading.Monitor.TryEnter(_batchLock)) { throw new InvalidOperationException("This query object is already running a batch; use a separate query object for concurrent batches."); }
        try
        {
            ExecuteMixed(inputs, 1, QueryMode.Any);
            for (int i = 0; i < inputs.Length; i++) { hits[i] = _batchCounts[i] != 0; }
        }
        finally { System.Threading.Monitor.Exit(_batchLock); }
    }

    /// <summary>Runs mixed queries in parallel with a fixed number of result slots per input. Input i writes its first counts[i] hits at i * hitsPerQuery; unused slots become default. Counts are stored, not total available, and truncated hits follow each scalar All method's ordering (overlaps by slot, others by distance then slot). Zero slots is valid. Same validation and concurrency contract as QueryBatch.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The slot count is negative.</exception>
    /// <exception cref="ArgumentException">A buffer is too short, the slot product exceeds Int32.MaxValue, or an input is default.</exception>
    public void QueryAllBatch(ReadOnlySpan<QueryInput3D> inputs, Span<QueryHit3D> hits, Span<int> counts, int hitsPerQuery)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(hitsPerQuery);
        long length = (long)inputs.Length * hitsPerQuery;
        if (length > hits.Length || counts.Length < inputs.Length) { throw new ArgumentException("Buffers must hold every fixed-stride result and one count per input."); }
        if (!System.Threading.Monitor.TryEnter(_batchLock)) { throw new InvalidOperationException("This query object is already running a batch; use a separate query object for concurrent batches."); }
        try
        {
            ExecuteMixed(inputs, hitsPerQuery, QueryMode.All);
            _batchHits.AsSpan(0, (int)length).CopyTo(hits);
            _batchCounts.AsSpan(0, inputs.Length).CopyTo(counts);
        }
        finally { System.Threading.Monitor.Exit(_batchLock); }
    }

    private void ExecuteMixed(ReadOnlySpan<QueryInput3D> inputs, int stride, QueryMode mode)
    {
        if (_batchActive) { throw new InvalidOperationException("A batch predicate cannot reenter the same query object."); }
        for (int i = 0; i < inputs.Length; i++)
        {
            if (!inputs[i].Initialized) { throw new ArgumentException("Query input at index " + i.ToString(System.Globalization.CultureInfo.InvariantCulture) + " is default; use a query factory.", nameof(inputs)); }
        }
        _batchActive = true;
        try
        {
            int n = inputs.Length, total = checked(n * stride);
            if (_batchInputs.Length < n) { Array.Resize(ref _batchInputs, n); }
            if (_batchCounts.Length < n) { Array.Resize(ref _batchCounts, n); }
            if (_batchHits.Length < total) { Array.Resize(ref _batchHits, total); }
            inputs.CopyTo(_batchInputs);
            // Publish lazy world-wide proxy padding before worker tasks read it.
            _world.QueryEngine.PrepareQueries();
            _batchHits.AsSpan(0, total).Clear();
            _world.Core.Executor.For(n / 64 + (n % 64 == 0 ? 0 : 1), new MixedBatch(_world, _batchInputs, _batchHits, _batchCounts, _filter, n, stride, mode));
        }
        finally { _batchActive = false; }
    }
}
