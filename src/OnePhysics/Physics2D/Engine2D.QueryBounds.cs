using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    private struct BoundsVisitor : IOverlapCallback
    {
        public Engine2D Engine;
        public QueryFilter2D Filter;
        public QueryScratch Scratch;

        public bool Report(int proxy, int userId)
        {
            if (Engine.Passes(in Filter, userId)) { Scratch.Insert(new RawHit { Slot = userId, Key = userId }); }
            return true;
        }
    }

    internal QueryScratch QueryBounds(in Aabb2D bounds, in QueryFilter2D filter, int capacity)
    {
        var scratch = QueryScratch.Acquire(Math.Min(capacity, Core.Shapes.HighWater));
        try
        {
            if (scratch.Capacity != 0)
            {
                var visitor = new BoundsVisitor { Engine = this, Filter = filter, Scratch = scratch };
                Bp.QueryAabb(in bounds, ref visitor);
            }
            return scratch;
        }
        catch { scratch.Release(); throw; }
    }
}
