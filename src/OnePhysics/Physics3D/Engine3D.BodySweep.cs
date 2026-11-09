using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    [ThreadStatic] private static SourceTriangles? t_sourceTriangles;

    private sealed class SourceTriangles
    {
        internal readonly TriangleShapeBuffer3D Buffer = new();
        internal bool InUse;
    }

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

    private void KeepSweep(in QueryParams query, in QueryFilter3D filter, ref RawHit best, ref bool found)
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

    internal bool SweepBody(int body, in QueryParams template, in QueryFilter3D filter, out RawHit best)
    {
        best = default; bool found = false;
        for (int slot = ShapeHead[body]; slot >= 0; slot = NextShape[slot])
        {
            ShapeFrame(slot, out double x, out double y, out double z, out QuaternionF rotation);
            var geometry = Geoms[GeomOf[slot]]!;
            var query = template;
            query.X = x; query.Y = y; query.Z = z; query.MoverRot = rotation;
            if (geometry.IsConvex)
            {
                query.Mover = geometry.Convex;
                KeepSweep(in query, in filter, ref best, ref found);
            }
            else if (geometry.Compound is { } compound)
            {
                for (int child = 0; child < compound.SlotCount; child++)
                {
                    if (!compound.IsLive(child)) { continue; }
                    var placed = compound.GetChild(child);
                    var offset = rotation.Rotate(placed.Local.Translation);
                    query.X = x + offset.X; query.Y = y + offset.Y; query.Z = z + offset.Z;
                    query.Mover = placed.Shape; query.MoverRot = rotation * placed.Local.Rotation;
                    KeepSweep(in query, in filter, ref best, ref found);
                }
            }
            else
            {
                // A soup is static-only, but sweeping it as a query is still useful for editing.
                // Separate storage prevents target triangle tests from changing the source leaf.
                var source = t_sourceTriangles ??= new SourceTriangles();
                if (source.InUse) { source = new SourceTriangles(); }
                source.InUse = true;
                try
                {
                    var soup = geometry.Soup!;
                    for (int triangle = 0; triangle < soup.TriangleIdCount; triangle++)
                    {
                        if (!soup.TryGetTriangle(triangle, out var tri)) { continue; }
                        query.Mover = source.Buffer.Set(tri.V0, tri.V1, tri.V2);
                        KeepSweep(in query, in filter, ref best, ref found);
                    }
                }
                finally { source.InUse = false; }
            }
        }
        return found;
    }
}
