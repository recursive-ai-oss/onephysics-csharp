using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Destruction;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D : IDestructionHost
{
    private DestructionStore? _destruction;
    internal DestructionStore Destruction => _destruction ??= new DestructionStore(this);
    DestructionStore IDestructionHost.Destruction => Destruction;
    WorldCore IDestructionHost.DestructionCore => Core;
    int IDestructionHost.EngineSectionId => Id;
    bool IDestructionHost.Planar => true;
    int IDestructionHost.MaxPieceVertices => Constants2D.MaxPolygonVertices;
    Vector3F IDestructionHost.Gravity => new(Opt.Gravity.X, Opt.Gravity.Y, 0f);
    float IDestructionHost.GravityScaleOf(int body) => GravityScale[body];
    long IDestructionHost.StepCount => Core.StepCount;
    float IDestructionHost.MassOf(int body) => Mass[body];
    Vector3F IDestructionHost.LocalCenter(int body) => new(LocalCenter[2 * body], LocalCenter[2 * body + 1], 0f);

    Vector3F IDestructionHost.ToWorld(int body, Vector3F v)
    {
        var r = Core.Bodies.RotationOf(body);
        return new Vector3F(r[0] * v.X - r[1] * v.Y, r[1] * v.X + r[0] * v.Y, 0f);
    }

    Vector3F IDestructionHost.ToBody(int body, Vector3F v)
    {
        var r = Core.Bodies.RotationOf(body);
        return new Vector3F(r[0] * v.X + r[1] * v.Y, r[0] * v.Y - r[1] * v.X, 0f);
    }

    Vector3F IDestructionHost.InertiaTimes(int body, Vector3F v) => new(0f, 0f, Inertia[body] * v.Z);

    Vector3F IDestructionHost.InverseInertiaTimes(int body, Vector3F v) => Inertia[body] > 0f ? new Vector3F(0f, 0f, v.Z / Inertia[body]) : default;

    bool IDestructionHost.CanReplace(int body)
    {
        if (CharacterOfBody(body) >= 0 || VehicleOfBody(body) >= 0 || RagdollOfBody(body) >= 0) return false;
        var cs = Core.Constraints; var bodies = Core.Bodies;
        if (bodies.IsDynamic(body))
        {
            for (int e = bodies.FirstEdge(body); e >= 0; e = cs.NextEdge(e)) if (cs.KindOf(e >> 1) != ConstraintKind.Contact) return false;
        }
        else
        {
            for (int c = 0; c < cs.HighWater; c++)
                if (cs.IsLive(c) && (cs.BodyA[c] == body || cs.BodyB[c] == body) && cs.KindOf(c) != ConstraintKind.Contact) return false;
        }
        return true;
    }

    bool IDestructionHost.ReadShapes(int body, List<ShapeInfo> into)
    {
        for (int s = ShapeHead[body]; s >= 0; s = NextShape[s])
        {
            if (IsSensor(s)) continue;
            ref readonly var g = ref Geom[s];
            if (g.Kind != ShapeKind2D.Polygon || g.Radius != 0f) return false;
            var v = g.Vertices;
            var pts = new Vector3F[v.Length];
            for (int i = 0; i < pts.Length; i++) pts[i] = new Vector3F(v[i].X, v[i].Y, 0f);
            var piece = Piece.FromPoints2D(pts);
            if (piece == null) return false;
            into.Add(new ShapeInfo
            {
                Shape = s, Generation = Core.Shapes.Handles.GenerationOf(s), Piece = piece,
                Template = new ShapeTemplate(Density[s], Friction[s], Restitution[s], Rolling[s], Category[s], Mask[s], Group[s], ShapeLayer[s]),
            });
        }
        return true;
    }

    BodyHandle IDestructionHost.CloneBody(int source, BodyType type)
    {
        var bodies = Core.Bodies;
        var p = bodies.PositionOf(source);
        var r = bodies.RotationOf(source);
        float lx = LocalCenter[2 * source], ly = LocalCenter[2 * source + 1];
        var def = new BodyDef
        {
            Type = type,
            Position = new Vector2D(p[0] - (double)(r[0] * lx - r[1] * ly), p[1] - (double)(r[1] * lx + r[0] * ly)),
            LinearDamping = LinearDamping[source], AngularDamping = AngularDamping[source], GravityScale = GravityScale[source],
            FixedRotation = (BodyFlags[source] & 1) != 0,
            IsBullet = (bodies.Flags[source] & (byte)BodyBits.Bullet) != 0,
            AllowSleep = (bodies.Flags[source] & (byte)BodyBits.NoSleep) == 0,
        };
        if (type == BodyType.Dynamic && bodies.TypeOf(source) == BodyType.Dynamic) def.SleepSettings = Core.GetBodySleepSettings(Core.HandleOfBody(source));
        var handle = CreateBody(in def);
        // Keep the source's rotation bit for bit instead of rebuilding it from an angle.
        var nr = bodies.RotationOf(handle.Index);
        nr[0] = r[0]; nr[1] = r[1];
        return handle;
    }

    ShapeHandle IDestructionHost.AddPiece(BodyHandle body, Vector3F[] points, in ShapeTemplate t)
    {
        var v = new Vec2[points.Length];
        for (int i = 0; i < v.Length; i++) v[i] = new Vec2(points[i].X, points[i].Y);
        var def = new ShapeDef { Density = t.Density, Friction = t.Friction, Restitution = t.Restitution, RollingResistance = t.Rolling, CategoryBits = t.Category, MaskBits = t.Mask, GroupIndex = t.Group, Layer = t.Layer };
        var geometry = Shape2D.CreatePolygon(v, 0f);
        return AddShape(body, in geometry, in def);
    }

    void IDestructionHost.DestroyBody(BodyHandle body) => World!.DestroyBody(body);
}
