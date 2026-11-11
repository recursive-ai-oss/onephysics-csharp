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
            content |= DebugContent.Contacts;
        }

        if (request.IsOn(ScopeInfo.OverlayJoints))
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
            var style = new DrawStyle(new Color32(200, 200, 90, 160), ScopeInfo.OverlayBounds, 1, DrawFlags.Wireframe);
            foreach (DebugBox b in _geometry.Bounds)
            {
                Vector3 lo = P(b.Min, ox, oy, oz);
                Vector3 hi = P(b.Max, ox, oy, oz);
                into.Box((lo + hi) * 0.5f, (hi - lo) * 0.5f, Quaternion.Identity, style);
            }
        }

        if (request.IsOn(ScopeInfo.OverlayContacts))
        {
            var style = new DrawStyle(new Color32(240, 80, 60), ScopeInfo.OverlayContacts, 2, DrawFlags.OnTop);
            foreach (DebugContact c in _geometry.Contacts)
            {
                Vector3 at = P(c.Point, ox, oy, oz);
                into.Arrow(at, at + new Vector3(c.Normal.X, c.Normal.Y, c.Normal.Z) * 0.25f, 0.05f, style);
            }
        }

        if (request.IsOn(ScopeInfo.OverlayJoints))
        {
            var style = new DrawStyle(new Color32(250, 170, 40), ScopeInfo.OverlayJoints, 2, DrawFlags.OnTop);
            foreach (DebugJoint j in _geometry.Joints)
            {
                Vector3 a = P(j.AnchorA, ox, oy, oz);
                Vector3 b = P(j.AnchorB, ox, oy, oz);
                into.Line(a, b, style);
                into.Point(a, 5f, style);
                into.Point(b, 5f, style);
            }
        }

        if (request.IsOn(ScopeInfo.OverlayCentres))
        {
            var style = new DrawStyle(new Color32(255, 255, 255), ScopeInfo.OverlayCentres, 2, DrawFlags.OnTop);
            foreach (DebugBody b in _geometry.Bodies)
            {
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

        var style = new DrawStyle(color, ScopeInfo.OverlayShapes, 0, s.IsSensor ? DrawFlags.Wireframe : DrawFlags.None);
        var line = new DrawStyle(color, ScopeInfo.OverlayShapes, 0, DrawFlags.Wireframe);
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

    private static Vector3 P(Vector3D v, double ox, double oy, double oz) => new((float)(v.X + ox), (float)(v.Y + oy), (float)(v.Z + oz));
}
