using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Destruction;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D : IDestructionHost
{
    private DestructionStore? _destruction;
    internal DestructionStore Destruction => _destruction ??= new DestructionStore(this);
    DestructionStore IDestructionHost.Destruction => Destruction;
    WorldCore IDestructionHost.DestructionCore => Core;
    int IDestructionHost.EngineSectionId => Id;
    bool IDestructionHost.Planar => false;
    Vector3F IDestructionHost.Gravity => Opt.Gravity;
    float IDestructionHost.GravityScaleOf(int body) => GravityScale[body];
    long IDestructionHost.StepCount => Core.StepCount;
    float IDestructionHost.MassOf(int body) => Mass[body];
    Vector3F IDestructionHost.LocalCenter(int body) => LocalCenterOf(body);
    Vector3F IDestructionHost.ToWorld(int body, Vector3F v) => RotationOfBody(body).Rotate(v);
    Vector3F IDestructionHost.ToBody(int body, Vector3F v) => RotationOfBody(body).Conjugated().Rotate(v);

    private Matrix3x3F LocalInertia(int b)
    {
        int o = 6 * b;
        return new Matrix3x3F(Inertia[o], Inertia[o + 1], Inertia[o + 2], Inertia[o + 1], Inertia[o + 3], Inertia[o + 4], Inertia[o + 2], Inertia[o + 4], Inertia[o + 5]);
    }

    Vector3F IDestructionHost.InertiaTimes(int body, Vector3F v)
    {
        var q = RotationOfBody(body);
        return q.Rotate(LocalInertia(body) * q.Conjugated().Rotate(v));
    }

    Vector3F IDestructionHost.InverseInertiaTimes(int body, Vector3F v)
    {
        var q = RotationOfBody(body);
        if (!LocalInertia(body).TryInvert(out var inv)) return default;
        return q.Rotate(inv * q.Conjugated().Rotate(v));
    }

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
        bool all = true;
        for (int s = ShapeHead[body]; s >= 0; s = NextShape[s])
        {
            if (IsSensor(s)) continue;
            var g = Geoms[GeomOf[s]]!;
            if (!g.IsConvex || g.Convex.ConvexRadius != 0f) { all = false; break; }
            Piece? piece;
            var local = Local[s];
            var convex = g.Convex;
            if (convex.Kind == ConvexShapeKind3D.Box)
            {
                var h = convex.CoreSize;
                piece = Piece.Box(new Vector3D(-h.X, -h.Y, -h.Z), new Vector3D(h.X, h.Y, h.Z));
                if (local != RigidTransformF.Identity) piece = Piece.FromPoints3D(Transform(piece.FloatPoints(), local));
            }
            else if (convex.Kind == ConvexShapeKind3D.Hull && convex.Hull != null)
            {
                var verts = new Vector3F[convex.Hull.Vertices.Length];
                convex.Hull.Vertices.CopyTo(verts);
                piece = local == RigidTransformF.Identity ? Piece.FromHull(convex.Hull) : Piece.FromPoints3D(Transform(verts, local));
            }
            else { all = false; break; }
            if (piece == null) { all = false; break; }
            into.Add(new ShapeInfo
            {
                Shape = s, Generation = Core.Shapes.Handles.GenerationOf(s), Piece = piece,
                Template = new ShapeTemplate(Density[s], Friction[s], Restitution[s], Rolling[s], Category[s], Mask[s], Group[s], ShapeLayer[s]),
            });
        }
        return all;
    }

    private static Vector3F[] Transform(Vector3F[] points, RigidTransformF t)
    {
        var r = new Vector3F[points.Length];
        for (int i = 0; i < r.Length; i++) r[i] = t.TransformPoint(points[i]);
        return r;
    }

    BodyHandle IDestructionHost.CloneBody(int source, BodyType type)
    {
        var rot = RotationOfBody(source);
        var p = Core.Bodies.PositionOf(source);
        var off = rot.Rotate(LocalCenterOf(source));
        var handle = Core.HandleOfBody(source);
        var def = new BodyDef
        {
            Type = type,
            Position = new Vector3D(p[0] - (double)off.X, p[1] - (double)off.Y, p[2] - (double)off.Z),
            Rotation = rot,
            LinearDamping = LinearDamping[source], AngularDamping = AngularDamping[source], GravityScale = GravityScale[source],
            FixedRotation = (BodyFlags[source] & 1) != 0,
            IsBullet = (Core.Bodies.Flags[source] & (byte)BodyBits.Bullet) != 0,
            AllowSleep = (Core.Bodies.Flags[source] & (byte)BodyBits.NoSleep) == 0,
        };
        if (type == BodyType.Dynamic && Core.Bodies.TypeOf(source) == BodyType.Dynamic) def.SleepSettings = Core.GetBodySleepSettings(handle);
        return CreateBody(in def);
    }

    ShapeHandle IDestructionHost.AddPiece(BodyHandle body, Vector3F[] points, in ShapeTemplate t)
    {
        var def = new ShapeDef { Density = t.Density, Friction = t.Friction, Restitution = t.Restitution, RollingResistance = t.Rolling, CategoryBits = t.Category, MaskBits = t.Mask, GroupIndex = t.Group, Layer = t.Layer };
        return AddShape(body, ShapeSpec3D.ConvexHull(points).Recipe.ToArray(), in def);
    }

    void IDestructionHost.DestroyBody(BodyHandle body)
    {
        World!.DestroyBody(body);
    }
}
