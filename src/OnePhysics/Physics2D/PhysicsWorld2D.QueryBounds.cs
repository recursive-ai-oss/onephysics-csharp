using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>Returns shapes whose stored broadphase boxes intersect the inclusive box, in shape-slot order. No narrowphase runs; boxes can be fat or stale until the next step. When the buffer is short, the lowest slots are kept. No allocation after thread scratch warmup.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A bound is nonfinite, outside +-2^22, or a minimum exceeds its maximum.</exception>
    public int QueryAabb(Vector2D minimum, Vector2D maximum, Span<Shape> results, in QueryFilter2D filter = default)
    {
        CheckPosition(minimum, nameof(minimum));
        CheckPosition(maximum, nameof(maximum));
        if (minimum.X > maximum.X || minimum.Y > maximum.Y) { throw new ArgumentOutOfRangeException(nameof(maximum), maximum, "Each maximum must be at least its minimum."); }
        var scratch = _engine.QueryBounds(new Aabb2D(minimum.X, minimum.Y, maximum.X, maximum.Y), in filter, results.Length);
        try
        {
            for (int i = 0; i < scratch.Count; i++) { results[i] = new Shape(this, _core.HandleOfShape(scratch.Hits[i].Slot)); }
            return scratch.Count;
        }
        finally { scratch.Release(); }
    }
}
