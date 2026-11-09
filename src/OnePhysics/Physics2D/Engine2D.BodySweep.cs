using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    private bool ExcludedFromSweep(int candidate, int source, bool jointed)
    {
        if (candidate == source) { return true; }
        if (!jointed) { return false; }
        var constraints = Core.Constraints;
        for (int i = 0; i < constraints.HighWater; i++)
        {
            if (!constraints.Handles.IsLive(i) || constraints.KindOf(i) != ConstraintKind.Joint) { continue; }
            int a = constraints.BodyA[i], b = constraints.BodyB[i];
            if ((a == source && b == candidate) || (b == source && a == candidate)) { return true; }
        }
        return false;
    }

    private void KeepSweep(in QueryParams query, in QueryFilter2D filter, ref RawHit best, ref bool found)
    {
        var scratch = Run(in query, in filter, QueryMode.Closest, 1);
        try
        {
            if (scratch.Count == 0) { return; }
            var hit = scratch.Hits[0];
            if (!found || hit.Key < best.Key || (hit.Key == best.Key && hit.Slot < best.Slot)) { best = hit; found = true; }
        }
        finally { scratch.Release(); }
    }

    internal bool SweepBody(int body, in QueryParams template, in QueryFilter2D filter, out RawHit best)
    {
        best = default; bool found = false;
        for (int slot = ShapeHead[body]; slot >= 0; slot = NextShape[slot])
        {
            FrameOf(body, out double x, out double y, out Rot2 rotation);
            var query = template;
            query.X = x; query.Y = y; query.Mover = Geom[slot]; query.MoverRot = rotation;
            KeepSweep(in query, in filter, ref best, ref found);
        }
        return found;
    }
}
