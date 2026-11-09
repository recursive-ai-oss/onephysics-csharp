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
        internal readonly TriangleShapeBuffer3D TargetBuffer = new();
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

    private struct LeafCursor
    {
        private readonly Geom3D _geometry;
        private readonly TriangleShapeBuffer3D _triangles;
        private int _next;
        internal int CurrentId { get; private set; }
        internal LeafCursor(Geom3D geometry, TriangleShapeBuffer3D triangles) { _geometry = geometry; _triangles = triangles; _next = 0; CurrentId = 0; }
        internal bool MoveNext(out ConvexShape3D shape, out RigidTransformF local)
        {
            shape = default; local = RigidTransformF.Identity;
            if (_geometry.IsConvex)
            {
                if (_next++ != 0 || _geometry.Convex.Kind == ConvexShapeKind3D.Empty) { return false; }
                shape = _geometry.Convex; return true;
            }
            if (_geometry.Compound is { } compound)
            {
                while (_next < compound.SlotCount)
                {
                    int id = _next++;
                    if (!compound.IsLive(id)) { continue; }
                    var child = compound.GetChild(id);
                    if (child.Shape.Kind == ConvexShapeKind3D.Empty) { continue; }
                    shape = child.Shape; local = child.Local; CurrentId = id; return true;
                }
                return false;
            }
            var soup = _geometry.Soup!;
            while (_next < soup.TriangleIdCount)
            {
                if (!soup.TryGetTriangle(_next++, out var tri)) { continue; }
                shape = _triangles.Set(tri.V0, tri.V1, tri.V2); CurrentId = tri.Id; return true;
            }
            return false;
        }
    }

    private static bool CastPlaneMover(Geom3D geometry, in ConvexShape3D plane, in RigidTransformF relative, Vector3F motion, QueryScratch scratch, out ConvexCastHit3D result, out int sub)
    {
        result = default; sub = 0; bool found = false;
        var leaves = new LeafCursor(geometry, scratch.TriBuf);
        while (leaves.MoveNext(out var target, out var local))
        {
            var childMotion = local.Rotation.Conjugated().Rotate(motion);
            if (!Gjk3D.ShapeCast(target, plane, local.Inverted() * relative, -childMotion, out var hit)) { continue; }
            if (found && (hit.Fraction > result.Fraction || (hit.Fraction == result.Fraction && leaves.CurrentId >= sub))) { continue; }
            var point = local.TransformPoint(hit.PointA + childMotion * hit.Fraction);
            result = new ConvexCastHit3D(hit.Fraction, local.Rotation.Rotate(hit.Normal), point, point, hit.StartsOverlapping);
            sub = leaves.CurrentId; found = true;
        }
        return found;
    }

    private static SourceTriangles AcquireSweepTriangles()
    {
        var scratch = t_sourceTriangles ??= new SourceTriangles();
        if (scratch.InUse) { scratch = new SourceTriangles(); }
        scratch.InUse = true; return scratch;
    }

    internal bool SweepBody(int body, in QueryParams template, in QueryFilter3D filter, out RawHit best)
    {
        best = default; bool found = false;
        var scratch = AcquireSweepTriangles();
        try
        {
            for (int slot = ShapeHead[body]; slot >= 0; slot = NextShape[slot])
            {
                ShapeFrame(slot, out double x, out double y, out double z, out QuaternionF rotation);
                var leaves = new LeafCursor(Geoms[GeomOf[slot]]!, scratch.Buffer);
                while (leaves.MoveNext(out var shape, out var local))
                {
                    var offset = rotation.Rotate(local.Translation);
                    var query = template;
                    query.X = x + offset.X; query.Y = y + offset.Y; query.Z = z + offset.Z;
                    query.Mover = shape; query.MoverRot = rotation * local.Rotation;
                    KeepSweep(in query, in filter, ref best, ref found);
                }
            }
            return found;
        }
        finally { scratch.InUse = false; }
    }

    internal bool AngularSweepBody(int body, Vector3F translation, Vector3F angular, float tolerance, bool excludeJointed, in QueryFilter3D filter, out RawHit best)
    {
        best = default; bool found = false;
        var centre = Core.Bodies.PositionOf(body);
        var scratch = AcquireSweepTriangles();
        try
        {
            for (int source = ShapeHead[body]; source >= 0; source = NextShape[source])
            {
                ShapeFrame(source, out double sx, out double sy, out double sz, out var sq);
                var sources = new LeafCursor(Geoms[GeomOf[source]]!, scratch.Buffer);
                while (sources.MoveNext(out var mover, out var sourceLocal))
                {
                    var sourceOffset = sq.Rotate(sourceLocal.Translation);
                    var sourceRotation = sq * sourceLocal.Rotation;
                    for (int target = 0; target < Core.Shapes.HighWater; target++)
                    {
                        if (!Core.Shapes.Handles.IsLive(target) || ExcludedFromSweep(Core.Shapes.Owner[target], body, excludeJointed) || !Passes(in filter, target)) { continue; }
                        ShapeFrame(target, out double tx, out double ty, out double tz, out var tq);
                        var targets = new LeafCursor(Geoms[GeomOf[target]]!, scratch.TargetBuffer);
                        while (targets.MoveNext(out var obstacle, out var targetLocal))
                        {
                            var offset = tq.Rotate(targetLocal.Translation);
                            var rotation = tq * targetLocal.Rotation;
                            var inv = rotation.Conjugated();
                            double ox = tx + offset.X, oy = ty + offset.Y, oz = tz + offset.Z;
                            var relative = new RigidTransformF(inv * sourceRotation, inv.Rotate(new Vector3F((float)((sx - ox) + sourceOffset.X), (float)((sy - oy) + sourceOffset.Y), (float)((sz - oz) + sourceOffset.Z))));
                            var c = inv.Rotate(new Vector3F((float)(centre[0] - ox), (float)(centre[1] - oy), (float)(centre[2] - oz)));
                            if (!AngularCast3D.Test(obstacle, mover, relative, c, inv.Rotate(translation), inv.Rotate(angular), out float fraction, out var point, out var normal, tolerance)) { continue; }
                            if (found && (fraction > best.Fraction || (fraction == best.Fraction && target >= best.Slot))) { continue; }
                            var wp = rotation.Rotate(point); var wn = rotation.Rotate(normal);
                            best = new RawHit { Slot = target, Sub = targets.CurrentId, Fraction = fraction, Key = fraction, Px = ox + wp.X, Py = oy + wp.Y, Pz = oz + wp.Z, Nx = wn.X, Ny = wn.Y, Nz = wn.Z };
                            found = true;
                        }
                    }
                }
            }
            return found;
        }
        finally { scratch.InUse = false; }
    }
}
