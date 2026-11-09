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
            if (Geom[slot].Kind == ShapeKind2D.Empty) { continue; }
            FrameOf(body, out double x, out double y, out Rot2 rotation);
            var query = template;
            query.X = x; query.Y = y; query.Mover = Geom[slot]; query.MoverRot = rotation;
            KeepSweep(in query, in filter, ref best, ref found);
        }
        return found;
    }

    internal bool AngularSweepBody(int body, RecursiveAI.OneMath.Vector2F translation, float angular, float tolerance, bool excludeJointed, in QueryFilter2D filter, out RawHit best)
    {
        best = default; bool found = false;
        var centre = Core.Bodies.PositionOf(body);
        FrameOf(body, out double sx, out double sy, out var sq);
        for (int source = ShapeHead[body]; source >= 0; source = NextShape[source])
        {
            if (Geom[source].Kind == ShapeKind2D.Empty) { continue; }
            for (int target = 0; target < Core.Shapes.HighWater; target++)
            {
                if (!Core.Shapes.Handles.IsLive(target) || Geom[target].Kind == ShapeKind2D.Empty || ExcludedFromSweep(Core.Shapes.Owner[target], body, excludeJointed) || !Passes(in filter, target)) { continue; }
                FrameOf(Core.Shapes.Owner[target], out double tx, out double ty, out var tq);
                var relative = new Transform2(tq.InverseRotate(new Vec2((float)(sx - tx), (float)(sy - ty))), Rot2.InvMul(tq, sq));
                var c = tq.InverseRotate(new Vec2((float)(centre[0] - tx), (float)(centre[1] - ty)));
                var motion = tq.InverseRotate(new Vec2(translation.X, translation.Y));
                if (!AngularCast2D.Test(Geom[target], Geom[source], relative, c, motion, angular, out float fraction, out var point, out var normal, tolerance)) { continue; }
                if (found && (fraction > best.Fraction || (fraction == best.Fraction && target >= best.Slot))) { continue; }
                var wp = tq.Rotate(point); var wn = tq.Rotate(normal);
                best = new RawHit { Slot = target, Fraction = fraction, Key = fraction, Px = tx + wp.X, Py = ty + wp.Y, Nx = wn.X, Ny = wn.Y };
                found = true;
            }
        }
        return found;
    }
}
