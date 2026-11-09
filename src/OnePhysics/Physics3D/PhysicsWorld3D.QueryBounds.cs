using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>Returns shapes whose stored broadphase boxes intersect the inclusive box, in shape-slot order. No narrowphase runs; boxes can be fat or stale until the next step. When the buffer is short, the lowest slots are kept. No allocation after thread scratch warmup.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A bound is nonfinite, outside +-2^22, or a minimum exceeds its maximum.</exception>
    public int QueryAabb(Vector3D minimum, Vector3D maximum, Span<Shape> results, in QueryFilter3D filter = default)
    {
        CheckPosition(minimum, nameof(minimum));
        CheckPosition(maximum, nameof(maximum));
        if (minimum.X > maximum.X || minimum.Y > maximum.Y || minimum.Z > maximum.Z) { throw new ArgumentOutOfRangeException(nameof(maximum), maximum, "Each maximum must be at least its minimum."); }
        var scratch = _engine.QueryBounds(new Aabb3D(minimum.X, minimum.Y, minimum.Z, maximum.X, maximum.Y, maximum.Z), in filter, results.Length);
        try
        {
            for (int i = 0; i < scratch.Count; i++) { results[i] = new Shape(this, _core.HandleOfShape(scratch.Hits[i].Slot)); }
            return scratch.Count;
        }
        finally { scratch.Release(); }
    }
}
