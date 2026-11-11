using System;
using System.Collections.Generic;
using System.Numerics;
using OneScope.Sdk;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Debug;

namespace RecursiveAI.OnePhysics.Scope;

internal sealed partial class ScopeWorld
{
    private static readonly Color32 StaticColor = new(150, 150, 165);
    private static readonly Color32 KinematicColor = new(90, 140, 230);
    private static readonly Color32 AwakeColor = new(90, 200, 110);
    private static readonly Color32 AsleepColor = new(80, 120, 140);
    private static readonly Color32 SensorColor = new(230, 200, 60, 120);

    private readonly Dictionary<int, DebugBodyState> _states = [];
    private Vector3[] _scratch = new Vector3[64];

    public void Draw(DrawList into, in DrawRequest request)
    {
        ArgumentNullException.ThrowIfNull(into);
        DebugContent content = DebugContent.None;
        if (request.IsOn(ScopeInfo.OverlayShapes))
        {
            content |= DebugContent.Shapes | DebugContent.Bodies;
        }

        if (request.IsOn(ScopeInfo.OverlayBounds))
        {
            content |= DebugContent.Bounds;
        }

        if (request.IsOn(ScopeInfo.OverlayContacts))
        {
            content |= DebugContent.Contacts | DebugContent.Bounds; // the bounds give the arrows their size
        }

        if (request.IsOn(ScopeInfo.OverlayJoints) || request.IsOn(ScopeInfo.OverlayLimits))
        {
            content |= DebugContent.Joints;
        }

        if (request.IsOn(ScopeInfo.OverlayCentres))
        {
            content |= DebugContent.Bodies;
        }

        if (content == DebugContent.None)
        {
            return;
        }

        _host.CollectDebug(content, _geometry);
        (double ox, double oy, double oz) = _host.Origin;
        _states.Clear();
        foreach (DebugBody b in _geometry.Bodies)
        {
            _states[b.Body.Index] = b.State;
        }

        ReadOnlySpan<Vector3D> points = _geometry.Points;
        if (request.IsOn(ScopeInfo.OverlayShapes))
        {
            foreach (DebugShape s in _geometry.Shapes)
            {
                DrawShape(into, s, points, ox, oy, oz);
            }
        }

        if (request.IsOn(ScopeInfo.OverlayBounds))
        {
            foreach (DebugBox b in _geometry.Bounds)
            {
                var style = new DrawStyle(new Color32(200, 200, 90, 160), ScopeInfo.OverlayBounds, 1, DrawFlags.Wireframe, EntityIds.Of(b.Body));
                Vector3 lo = P(b.Min, ox, oy, oz);
                Vector3 hi = P(b.Max, ox, oy, oz);
                into.Box((lo + hi) * 0.5f, (hi - lo) * 0.5f, Quaternion.Identity, style);
            }
        }

        if (request.IsOn(ScopeInfo.OverlayContacts))
        {
            // Arrows scale with the bodies: the geometric mean of the bounds' longest sides (a few huge statics hardly move it), 1 when there are none.
            float unit = BodyUnit();
            foreach (DebugContact c in _geometry.Contacts)
            {
                var style = new DrawStyle(new Color32(240, 80, 60), ScopeInfo.OverlayContacts, 2, DrawFlags.OnTop, EntityIds.Of(c.BodyA));
                Vector3 at = P(c.Point, ox, oy, oz);
                // The normal points from A to B. Its length grows with the impulse the solver applied: 0.15 to 0.65 body units, a unit being the typical body size.
                float impulse = c.NormalImpulse > 0f ? c.NormalImpulse : 0f;
                float length = unit * (0.15f + 0.5f * (impulse / (impulse + 1f)));
                into.Arrow(at, at + new Vector3(c.Normal.X, c.Normal.Y, c.Normal.Z) * length, 0.05f * unit, style);
                into.Point(at, 4f, style);
            }
        }

        if (request.IsOn(ScopeInfo.OverlayJoints))
        {
            foreach (DebugJoint j in _geometry.Joints)
            {
                var style = new DrawStyle(new Color32(250, 170, 40), ScopeInfo.OverlayJoints, 2, DrawFlags.OnTop, EntityIds.Of(j.Joint));
                Vector3 a = P(j.AnchorA, ox, oy, oz);
                Vector3 b = P(j.AnchorB, ox, oy, oz);
                into.Line(a, b, style);
                into.Point(a, 5f, style);
                into.Point(b, 5f, style);
            }
        }

        if (request.IsOn(ScopeInfo.OverlayLimits))
        {
            foreach (DebugJoint j in _geometry.Joints)
            {
                DrawLimits(into, j, ox, oy, oz);
            }
        }

        if (request.IsOn(ScopeInfo.OverlayCentres))
        {
            foreach (DebugBody b in _geometry.Bodies)
            {
                var style = new DrawStyle(new Color32(255, 255, 255), ScopeInfo.OverlayCentres, 2, DrawFlags.OnTop, EntityIds.Of(b.Body));
                into.Point(P(b.CenterOfMass, ox, oy, oz), 4f, style);
            }
        }
    }

    private void DrawShape(DrawList into, in DebugShape s, ReadOnlySpan<Vector3D> points, double ox, double oy, double oz)
    {
        Color32 color = SensorColor;
        if (!s.IsSensor)
        {
            _states.TryGetValue(s.Body.Index, out DebugBodyState state);
            color = state switch
            {
                DebugBodyState.Static => StaticColor,
                DebugBodyState.Kinematic => KinematicColor,
                DebugBodyState.Awake => AwakeColor,
                _ => AsleepColor,
            };
        }

        EntityId owner = EntityIds.Of(s.Body);
        var style = new DrawStyle(color, ScopeInfo.OverlayShapes, 0, s.IsSensor ? DrawFlags.Wireframe : DrawFlags.None, owner);
        var line = new DrawStyle(color, ScopeInfo.OverlayShapes, 0, DrawFlags.Wireframe, owner);
        if (s.PointCount == 0 || s.FirstPoint < 0 || s.FirstPoint + s.PointCount > points.Length)
        {
            return;
        }

        ReadOnlySpan<Vector3D> p = points.Slice(s.FirstPoint, s.PointCount);
        switch (s.Kind)
        {
            case DebugShapeKind.Circle:
                into.Sphere(P(p[0], ox, oy, oz), s.Radius, style);
                break;
            case DebugShapeKind.Capsule when p.Length >= 2:
            case DebugShapeKind.TaperedCapsule when p.Length >= 2:
                into.Capsule(P(p[0], ox, oy, oz), P(p[1], ox, oy, oz), (s.Radius > s.BottomRadius ? s.Radius : s.BottomRadius), style);
                break;
            case DebugShapeKind.Cylinder when p.Length >= 2:
            case DebugShapeKind.TaperedCylinder when p.Length >= 2:
                into.Cylinder(P(p[0], ox, oy, oz), P(p[1], ox, oy, oz), (s.Radius > s.BottomRadius ? s.Radius : s.BottomRadius), style);
                break;
            case DebugShapeKind.Polygon:
                for (int i = 0; i < p.Length; i++)
                {
                    into.Line(P(p[i], ox, oy, oz), P(p[(i + 1) % p.Length], ox, oy, oz), line);
                }

                break;
            case DebugShapeKind.Segment or DebugShapeKind.ChainSegment or DebugShapeKind.Plane when p.Length >= 2:
                into.Line(P(p[0], ox, oy, oz), P(p[1], ox, oy, oz), line);
                break;
            case DebugShapeKind.Empty:
                break;
            default:
                DrawTriangles(into, s, p, style, ox, oy, oz);
                break;
        }
    }

    // Boxes, hulls, meshes, height fields and voxel grids come as points plus triangle indices; without indices only the points are drawn.
    private void DrawTriangles(DrawList into, in DebugShape s, ReadOnlySpan<Vector3D> p, in DrawStyle style, double ox, double oy, double oz)
    {
        if (_scratch.Length < p.Length)
        {
            _scratch = new Vector3[Math.Max(p.Length, _scratch.Length * 2)];
        }

        for (int i = 0; i < p.Length; i++)
        {
            _scratch[i] = P(p[i], ox, oy, oz);
        }

        ReadOnlySpan<int> indices = _geometry.Indices;
        if (s.IndexCount >= 3 && s.FirstIndex >= 0 && s.FirstIndex + s.IndexCount <= indices.Length)
        {
            ReadOnlySpan<int> tri = indices.Slice(s.FirstIndex, s.IndexCount);
            bool valid = true;
            foreach (int index in tri)
            {
                valid &= (uint)index < (uint)p.Length;
            }

            if (valid)
            {
                into.Mesh(_scratch.AsSpan(0, p.Length), tri, style);
                return;
            }
        }

        for (int i = 0; i < p.Length; i++)
        {
            into.Point(_scratch[i], 3f, style);
        }
    }

    private const int ArcSegments = 24;
    private const int ConeRays = 12;

    // Limits as the joint's debug entry reports them (3D only; the planar engine's debug entry carries no limit or joint type).
    // Hinge: an arc of the limit range around the axis. The entry has no zero direction of the hinge angle, so the arc starts at a fixed
    // direction perpendicular to the axis: its span is the limit range, its orientation is not the joint's.
    // Slider: the allowed travel along the axis. Cone and swing-twist: the swing cone about the axis.
    private static void DrawLimits(DrawList into, in DebugJoint j, double ox, double oy, double oz)
    {
        Vector3 axis = new((float)j.Axis.X, (float)j.Axis.Y, (float)j.Axis.Z);
        float axisLength = axis.Length();
        if (j.Type == 0 || axisLength < 0.5f)
        {
            return;
        }

        axis /= axisLength;
        Vector3 at = P(j.AnchorA, ox, oy, oz);
        var style = new DrawStyle(new Color32(255, 120, 200), ScopeInfo.OverlayLimits, 3, DrawFlags.OnTop, EntityIds.Of(j.Joint));
        const float Radius = 0.4f;
        switch ((Physics3D.JointType3D)j.Type)
        {
            case Physics3D.JointType3D.Hinge when j.UpperLimit > j.LowerLimit:
                Vector3 u = Perpendicular(axis);
                Vector3 v = Vector3.Cross(axis, u);
                Vector3 previous = at + u * Radius;
                for (int i = 0; i <= ArcSegments; i++)
                {
                    float angle = j.LowerLimit + (j.UpperLimit - j.LowerLimit) * i / ArcSegments;
                    Vector3 p = at + (u * Cos(angle) + v * Sin(angle)) * Radius;
                    into.Line(previous, p, style);
                    previous = p;
                }

                into.Line(at, at + (u * Cos(j.LowerLimit) + v * Sin(j.LowerLimit)) * Radius, style);
                into.Line(at, previous, style);
                break;
            case Physics3D.JointType3D.Slider when j.UpperLimit > j.LowerLimit:
                Vector3 lo = at + axis * j.LowerLimit;
                Vector3 hi = at + axis * j.UpperLimit;
                Vector3 side = Perpendicular(axis) * 0.1f;
                into.Line(lo, hi, style);
                into.Line(lo - side, lo + side, style);
                into.Line(hi - side, hi + side, style);
                break;
            case Physics3D.JointType3D.Cone or Physics3D.JointType3D.SwingTwist when j.ConeAngle > 0f:
                Vector3 a = Perpendicular(axis);
                Vector3 b = Vector3.Cross(axis, a);
                float rim = Sin(j.ConeAngle) * Radius;
                float height = Cos(j.ConeAngle) * Radius;
                Vector3 last = default;
                for (int i = 0; i <= ConeRays * 2; i++)
                {
                    float angle = 2f * (float)Math.PI * i / (ConeRays * 2);
                    Vector3 p = at + axis * height + (a * Cos(angle) + b * Sin(angle)) * rim;
                    if (i > 0)
                    {
                        into.Line(last, p, style);
                    }

                    if (i % 2 == 0)
                    {
                        into.Line(at, p, style);
                    }

                    last = p;
                }

                break;
        }
    }

    private static Vector3 Perpendicular(Vector3 axis)
    {
        Vector3 other = float.Abs(axis.X) < 0.9f ? Vector3.UnitX : Vector3.UnitY;
        return Vector3.Normalize(Vector3.Cross(axis, other));
    }

    // Onemath's correctly rounded functions keep the drawing identical on every platform.
    private static float Cos(float x) => (float)RecursiveAI.OneMath.Math64.Cos(x);

    private static float Sin(float x) => (float)RecursiveAI.OneMath.Math64.Sin(x);

    private float BodyUnit()
    {
        float sumLog = 0f;
        int count = 0;
        foreach (DebugBox b in _geometry.Bounds)
        {
            double size = Math.Max(b.Max.X - b.Min.X, Math.Max(b.Max.Y - b.Min.Y, b.Max.Z - b.Min.Z));
            if (size > 1e-4 && size < 1e9)
            {
                sumLog += MathF.Log((float)size);
                count++;
            }
        }

        return count == 0 ? 1f : MathF.Exp(sumLog / count);
    }

    private static Vector3 P(Vector3D v, double ox, double oy, double oz) => new((float)(v.X + ox), (float)(v.Y + oy), (float)(v.Z + oz));
}
