using System;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    private readonly PhysicsQuery2D _mixedQueries;

    /// <summary>Runs mixed queries with the default filter, using reusable world-owned buffers. See PhysicsQuery2D.QueryBatch for limits and ordering. Concurrent batches on the same world are rejected; use separate query objects for independent filters and buffers.</summary>
    public int QueryBatch(ReadOnlySpan<QueryInput2D> inputs, Span<QueryHit2D> hits) => _mixedQueries.QueryBatch(inputs, hits);

    /// <summary>Mixed query existence tests with the default filter, in input order; uses reusable buffers. Invalid inputs and short outputs throw ArgumentException.</summary>
    public void QueryAnyBatch(ReadOnlySpan<QueryInput2D> inputs, Span<bool> hits) => _mixedQueries.QueryAnyBatch(inputs, hits);

    /// <summary>Mixed queries with fixed-stride output and the default filter; see PhysicsQuery2D.QueryAllBatch for truncation, validation and ordering.</summary>
    public void QueryAllBatch(ReadOnlySpan<QueryInput2D> inputs, Span<QueryHit2D> hits, Span<int> counts, int hitsPerQuery) => _mixedQueries.QueryAllBatch(inputs, hits, counts, hitsPerQuery);
}
