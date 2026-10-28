using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Debug;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>
    /// Fills <paramref name="geometry"/> with plain data for drawing the world, choosing what to emit with <paramref name="flags"/>; the buffer is cleared first and keeps its capacity.
    /// Nothing is called back and no simulation state changes. Positions are in world coordinates relative to <see cref="Origin"/>. Order: shapes, boxes by shape slot; contacts by constraint index; bodies by body slot.
    /// Contact points are those of the last step (sleeping contacts keep theirs). <see cref="DebugContent.Joints"/> lists each joint (by constraint slot) with both anchors, and the world axis of prismatic and wheel joints.
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
                if ((flags & DebugContent.Shapes) != 0) { AddDebugShape(geometry, b, s, bh, sh); }
                if ((flags & DebugContent.Bounds) != 0)
                {
                    var t = e.TightBox(b, in e.Geom[s]);
                    geometry.AddBounds(new DebugBox(bh, sh, new Vector3D(t.MinX, t.MinY, 0.0), new Vector3D(t.MaxX, t.MaxY, 0.0)));
                }

                if ((flags & DebugContent.BroadphaseBounds) != 0)
                {
                    var f = e.Bp.GetFatBounds(e.Proxy[s]);
                    geometry.AddBroadphaseBounds(new DebugBox(bh, sh, new Vector3D(f.MinX, f.MinY, 0.0), new Vector3D(f.MaxX, f.MaxY, 0.0)));
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
                var n = new Vector3F(pl[ManifoldLayout.Normal], pl[ManifoldLayout.Normal + 1], 0f);
                for (int k = 0; k < cs.PointCount[c]; k++)
                {
                    int a = ManifoldLayout.AnchorA(k);
                    geometry.Add(new DebugContact(ba, bb, sa, sb, new Vector3D(posA[0] + (double)pl[a], posA[1] + (double)pl[a + 1], 0.0), n, pl[ManifoldLayout.Separation(k)], pl[ManifoldLayout.NormalImpulse(k)]));
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
                var r = bodies.RotationOf(b);
                float lx = e.LocalCenter[2 * b], ly = e.LocalCenter[2 * b + 1];
                var origin = new Vector3D(p[0] - (double)(r[0] * lx - r[1] * ly), p[1] - (double)(r[1] * lx + r[0] * ly), 0.0);
                geometry.Add(new DebugBody(_core.HandleOfBody(b), state, origin, new Vector3D(p[0], p[1], 0.0), type == BodyType.Dynamic ? bodies.Island[b] : -1));
            }
        }

        if ((flags & DebugContent.Joints) != 0) { _engine.AddDebugJoints(geometry); }
        if ((flags & DebugContent.Characters) != 0) { _engine.AddDebugCharacters(geometry); }
    }

    private void AddDebugShape(DebugGeometry g, int b, int s, BodyHandle bh, ShapeHandle sh)
    {
        var e = _engine;
        ref readonly var geom = ref e.Geom[s];
        var r = _core.Bodies.RotationOf(b);
        var q = new Rot2(r[0], r[1]);
        var lc = new Vec2(e.LocalCenter[2 * b], e.LocalCenter[2 * b + 1]);
        var w = q.Rotate(lc);
        var pos = _core.Bodies.PositionOf(b);
        double ox = pos[0] - (double)w.X, oy = pos[1] - (double)w.Y;
        var kind = geom.Kind switch
        {
            ShapeKind2D.Circle => DebugShapeKind.Circle,
            ShapeKind2D.Capsule => DebugShapeKind.Capsule,
            ShapeKind2D.Segment => DebugShapeKind.Segment,
            ShapeKind2D.ChainSegment => DebugShapeKind.ChainSegment,
            _ => DebugShapeKind.Polygon,
        };
        var v = geom.Vertices;
        int first = -1;
        for (int i = 0; i < v.Length; i++)
        {
            var p = q.Rotate(v[i]);
            int idx = g.AddPoint(new Vector3D(ox + (double)p.X, oy + (double)p.Y, 0.0));
            if (i == 0) { first = idx; }
        }

        g.Add(new DebugShape(bh, sh, kind, e.IsSensor(s), geom.Radius, first, v.Length));
    }
}
