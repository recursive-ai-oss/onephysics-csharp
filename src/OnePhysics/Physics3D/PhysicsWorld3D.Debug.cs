using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Debug;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    // The box's twelve triangles over corners whose index bits are (x, y, z): outward winding.
    private static readonly int[] s_boxTriangles = [0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4, 2, 6, 7, 2, 7, 3, 0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5];

    /// <summary>
    /// Fills <paramref name="geometry"/> with plain data for drawing the world, choosing what to emit with <paramref name="flags"/>; the buffer is cleared first and keeps its capacity.
    /// Nothing is called back and no simulation state changes. Positions are in world coordinates relative to <see cref="Origin"/>. Order: shapes, boxes by shape slot (a compound emits one entry per child, in child order);
    /// contacts by constraint index; bodies by body slot; joints by constraint slot. Shape points are <c>shape origin + (double)(rotation * local point)</c>; convex shapes list their features (see <see cref="DebugShapeKind"/>),
    /// meshes and height fields their triangles only with <see cref="DebugContent.MeshTriangles"/>. Contact points are those of the last step. Joints report the anchors, frame A's x axis, frame B's x axis, the limits and the cone angle.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="geometry"/> is null.</exception>
    public void GetDebugGeometry(DebugContent flags, DebugGeometry geometry)
    {
        ArgumentNullException.ThrowIfNull(geometry);
        geometry.Clear();
        var shapes = _core.Shapes;
        var bodies = _core.Bodies;
        var e = _engine;
        if ((flags & (DebugContent.Shapes | DebugContent.Bounds | DebugContent.BroadphaseBounds)) != 0)
        {
            for (int s = 0; s < shapes.HighWater; s++)
            {
                if (!shapes.Handles.IsLive(s)) { continue; }
                int b = shapes.Owner[s];
                var bh = _core.HandleOfBody(b);
                var sh = _core.HandleOfShape(s);
                if ((flags & DebugContent.Shapes) != 0) { AddDebugShape(geometry, s, bh, sh, (flags & DebugContent.MeshTriangles) != 0); }
                if ((flags & DebugContent.Bounds) != 0)
                {
                    var t = e.TightBox(b, s);
                    geometry.AddBounds(new DebugBox(bh, sh, new Vector3D(t.MinX, t.MinY, t.MinZ), new Vector3D(t.MaxX, t.MaxY, t.MaxZ)));
                }

                if ((flags & DebugContent.BroadphaseBounds) != 0)
                {
                    var f = e.Bp.GetFatBounds(e.Proxy[s]);
                    geometry.AddBroadphaseBounds(new DebugBox(bh, sh, new Vector3D(f.MinX, f.MinY, f.MinZ), new Vector3D(f.MaxX, f.MaxY, f.MaxZ)));
                }
            }
        }

        if ((flags & DebugContent.Contacts) != 0)
        {
            var cs = _core.Constraints;
            for (int c = 0; c < cs.HighWater; c++)
            {
                if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Contact || cs.PointCount[c] == 0) { continue; }
                var pl = cs.Payload(c);
                var posA = bodies.PositionOf(cs.BodyA[c]);
                var ba = _core.HandleOfBody(cs.BodyA[c]);
                var bb = _core.HandleOfBody(cs.BodyB[c]);
                var sa = _core.HandleOfShape(cs.ShapeA[c]);
                var sb = _core.HandleOfShape(cs.ShapeB[c]);
                var n = new Vector3F(pl[ManifoldLayout.Normal], pl[ManifoldLayout.Normal + 1], pl[ManifoldLayout.Normal + 2]);
                for (int k = 0; k < cs.PointCount[c]; k++)
                {
                    int a = ManifoldLayout.AnchorA(k);
                    geometry.Add(new DebugContact(ba, bb, sa, sb, new Vector3D(posA[0] + (double)pl[a], posA[1] + (double)pl[a + 1], posA[2] + (double)pl[a + 2]), n, pl[ManifoldLayout.Separation(k)], pl[ManifoldLayout.NormalImpulse(k)]));
                }
            }
        }

        if ((flags & DebugContent.Bodies) != 0)
        {
            for (int b = 0; b < bodies.HighWater; b++)
            {
                if (!bodies.IsLive(b)) { continue; }
                var type = bodies.TypeOf(b);
                var state = type == BodyType.Static ? DebugBodyState.Static : type == BodyType.Kinematic ? DebugBodyState.Kinematic : bodies.IsAwake(b) ? DebugBodyState.Awake : DebugBodyState.Asleep;
                var p = bodies.PositionOf(b);
                var off = e.RotationOfBody(b).Rotate(e.LocalCenterOf(b));
                var origin = new Vector3D(p[0] - (double)off.X, p[1] - (double)off.Y, p[2] - (double)off.Z);
                geometry.Add(new DebugBody(_core.HandleOfBody(b), state, origin, new Vector3D(p[0], p[1], p[2]), type == BodyType.Dynamic ? bodies.Island[b] : -1));
            }
        }

        if ((flags & DebugContent.Joints) != 0) { AddDebugJoints(geometry); }
        if ((flags & DebugContent.Characters) != 0) { _engine.AddDebugCharacters(geometry); }
        if ((flags & DebugContent.Vehicles) != 0) { _engine.AddDebugVehicles(geometry); }
        if ((flags & DebugContent.SoftBodies) != 0) { _engine.AddDebugSoftBodies(geometry); }
    }

    private void AddDebugJoints(DebugGeometry geometry)
    {
        var cs = _core.Constraints;
        var bodies = _core.Bodies;
        var e = _engine;
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Joint || e.Joints[c].Type == 0) { continue; }
            ref readonly var j = ref e.Joints[c];
            int a = cs.BodyA[c], b = cs.BodyB[c];
            var qa = e.RotationOfBody(a);
            var qb = e.RotationOfBody(b);
            var pa = bodies.PositionOf(a);
            var pb = bodies.PositionOf(b);
            var ra = qa.Rotate(new Vector3F(j.LAx, j.LAy, j.LAz) - e.LocalCenterOf(a));
            var rb = qb.Rotate(new Vector3F(j.LBx, j.LBy, j.LBz) - e.LocalCenterOf(b));
            var anchorA = new Vector3D(pa[0] + (double)ra.X, pa[1] + (double)ra.Y, pa[2] + (double)ra.Z);
            var anchorB = new Vector3D(pb[0] + (double)rb.X, pb[1] + (double)rb.Y, pb[2] + (double)rb.Z);
            var ax = (qa * j.QA).Rotate(new Vector3F(1f, 0f, 0f));
            var bx = (qb * j.QB).Rotate(new Vector3F(1f, 0f, 0f));
            var type = (JointType3D)j.Type;
            float lower = 0f, upper = 0f, cone = 0f;
            if (j.Limit) { lower = j.Lower; upper = j.Upper; }
            if (type == JointType3D.Cone || type == JointType3D.SwingTwist) { cone = j.Swing1; }
            geometry.Add(new DebugJoint(_core.HandleOfJoint(c), _core.HandleOfBody(a), _core.HandleOfBody(b), anchorA, anchorB, new Vector3D(ax.X, ax.Y, ax.Z), new Vector3D(bx.X, bx.Y, bx.Z), j.Type, lower, upper, cone));
        }
    }

    private void AddDebugShape(DebugGeometry g, int s, BodyHandle bh, ShapeHandle sh, bool triangles)
    {
        var e = _engine;
        int b = _core.Shapes.Owner[s];
        var geom = e.Geoms[e.GeomOf[s]]!;
        var pose = e.ShapePoseFromCom(b, s);
        var pos = _core.Bodies.PositionOf(b);
        // The shape origin in world coordinates: the centre of mass plus the rotated offset (the translation of the pose).
        double ox = pos[0] + (double)pose.Translation.X, oy = pos[1] + (double)pose.Translation.Y, oz = pos[2] + (double)pose.Translation.Z;
        bool sensor = e.IsSensor(s);
        if (geom.IsConvex) { AddConvex(g, bh, sh, sensor, geom.Convex, pose.Rotation, ox, oy, oz); return; }
        if (geom.Compound != null)
        {
            var comp = geom.Compound;
            for (int j = 0; j < comp.SlotCount; j++)
            {
                if (!comp.IsLive(j)) { continue; }
                var child = comp.GetChild(j);
                var rot = pose.Rotation * child.Local.Rotation;
                var t = pose.Rotation.Rotate(child.Local.Translation);
                AddConvex(g, bh, sh, sensor, child.Shape, rot, ox + (double)t.X, oy + (double)t.Y, oz + (double)t.Z);
            }

            return;
        }

        var soup = geom.Soup!;
        int first = g.Points.Length;
        int firstIndex = g.Indices.Length;
        if (!triangles)
        {
            var bb = geom.LocalBounds;
            for (int i = 0; i < 8; i++)
            {
                var c = new Vector3F((i & 1) != 0 ? bb.Max.X : bb.Min.X, (i & 2) != 0 ? bb.Max.Y : bb.Min.Y, (i & 4) != 0 ? bb.Max.Z : bb.Min.Z);
                var w = pose.Rotation.Rotate(c);
                g.AddPoint(new Vector3D(ox + (double)w.X, oy + (double)w.Y, oz + (double)w.Z));
            }

            g.Add(new DebugShape(bh, sh, geom.Kind == GeomKind.Mesh ? DebugShapeKind.Mesh : DebugShapeKind.HeightField, sensor, 0f, first, 8, 0f, firstIndex, 0));
            return;
        }

        // Triangles in id order; each triangle owns three points, so the indices are 0, 1, 2, 3, ...
        int count = 0;
        for (int id = 0; id < soup.TriangleIdCount; id++)
        {
            if (!soup.TryGetTriangle(id, out var tri)) { continue; }
            foreach (var v in new[] { tri.V0, tri.V1, tri.V2 })
            {
                var w = pose.Rotation.Rotate(v);
                g.AddPoint(new Vector3D(ox + (double)w.X, oy + (double)w.Y, oz + (double)w.Z));
                g.AddIndex(count++);
            }
        }

        g.Add(new DebugShape(bh, sh, geom.Kind == GeomKind.Mesh ? DebugShapeKind.Mesh : DebugShapeKind.HeightField, sensor, 0f, first, count, 0f, firstIndex, count));
    }

    private static void AddConvex(DebugGeometry g, BodyHandle bh, ShapeHandle sh, bool sensor, in ConvexShape3D shape, QuaternionF rot, double ox, double oy, double oz)
    {
        int first = g.Points.Length;
        int firstIndex = g.Indices.Length;
        int AddPoint(Vector3F local)
        {
            var w = rot.Rotate(local);
            return g.AddPoint(new Vector3D(ox + (double)w.X, oy + (double)w.Y, oz + (double)w.Z));
        }

        switch (shape.Kind)
        {
            case ConvexShapeKind3D.Sphere:
                AddPoint(default);
                g.Add(new DebugShape(bh, sh, DebugShapeKind.Circle, sensor, shape.ConvexRadius, first, 1));
                break;
            case ConvexShapeKind3D.Box:
            {
                var h = shape.HalfExtents;
                float r = shape.ConvexRadius;
                var core = new Vector3F(h.X - r, h.Y - r, h.Z - r);
                for (int i = 0; i < 8; i++) { AddPoint(new Vector3F((i & 1) != 0 ? core.X : -core.X, (i & 2) != 0 ? core.Y : -core.Y, (i & 4) != 0 ? core.Z : -core.Z)); }
                foreach (int t in s_boxTriangles) { g.AddIndex(t); }
                g.Add(new DebugShape(bh, sh, DebugShapeKind.Box, sensor, r, first, 8, 0f, firstIndex, s_boxTriangles.Length));
                break;
            }

            case ConvexShapeKind3D.Capsule:
            {
                var s = shape.Support(new Vector3F(0f, 1f, 0f));
                float hh = s.Y - shape.ConvexRadius;
                AddPoint(new Vector3F(0f, -hh, 0f)); AddPoint(new Vector3F(0f, hh, 0f));
                g.Add(new DebugShape(bh, sh, DebugShapeKind.Capsule, sensor, shape.ConvexRadius, first, 2));
                break;
            }

            case ConvexShapeKind3D.TaperedCapsule:
            case ConvexShapeKind3D.Cylinder:
            {
                // The extreme supports along y and the widest supports along x give the half height and the radii.
                var top = shape.CoreSupport(new Vector3F(0f, 1f, 0f));
                var bottom = shape.CoreSupport(new Vector3F(0f, -1f, 0f));
                bool cyl = shape.Kind == ConvexShapeKind3D.Cylinder;
                float rt = shape.CoreSupport(new Vector3F(1f, 1e-3f, 0f)).X, rb = shape.CoreSupport(new Vector3F(1f, -1e-3f, 0f)).X;
                AddPoint(new Vector3F(0f, bottom.Y, 0f)); AddPoint(new Vector3F(0f, top.Y, 0f));
                g.Add(new DebugShape(bh, sh, cyl ? DebugShapeKind.Cylinder : DebugShapeKind.TaperedCapsule, sensor, cyl ? Math.Max(rt, rb) + shape.ConvexRadius : rt, first, 2, cyl ? 0f : rb, firstIndex, 0));
                break;
            }

            default:
            {
                var hull = shape.Hull!;
                var verts = hull.Vertices;
                for (int i = 0; i < verts.Length; i++) { AddPoint(verts[i]); }
                var faces = hull.Faces;
                var edges = hull.Edges;
                int n = 0;
                for (int f = 0; f < faces.Length; f++)
                {
                    int e0 = faces[f].FirstEdge;
                    int v0 = edges[e0].Origin;
                    int e1 = edges[e0].Next;
                    for (int k = 2; k < faces[f].EdgeCount; k++)
                    {
                        int e2 = edges[e1].Next;
                        g.AddIndex(v0); g.AddIndex(edges[e1].Origin); g.AddIndex(edges[e2].Origin);
                        n += 3;
                        e1 = e2;
                    }
                }

                g.Add(new DebugShape(bh, sh, DebugShapeKind.Hull, sensor, shape.ConvexRadius, first, verts.Length, 0f, firstIndex, n));
                break;
            }
        }
    }
}
