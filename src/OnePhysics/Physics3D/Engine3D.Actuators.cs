using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Aero;
using RecursiveAI.OnePhysics.Fields;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D : IActuatorHost
{
    private ActuatorStore? _actuators;

    /// <summary>Lift and drag of shapes, thrusters, boats and hovercraft; created on first use and registered with the world.</summary>
    internal ActuatorStore Actuators => _actuators ??= new ActuatorStore(this);

    WorldCore IActuatorHost.Core => Core;
    bool IActuatorHost.Planar => false;
    int IActuatorHost.EngineSectionId => Id;
    ActuatorStore IActuatorHost.Actuators => Actuators;
    FieldStore IActuatorHost.FieldsOf => Fields;
    float IActuatorHost.MassOf(int body) => Mass[body];
    float IActuatorHost.ReachOf(int body) => MaxExtent[body];
    Vector3F IActuatorHost.LocalCenterOf(int body) => LocalCenterOf(body);

    Vector3D IActuatorHost.PositionOf(int body)
    {
        var p = Core.Bodies.PositionOf(body);
        return new Vector3D(p[0], p[1], p[2]);
    }

    Vector3F IActuatorHost.Rotate(int body, Vector3F v) => RotationOfBody(body).Rotate(v);
    Vector3F IActuatorHost.RotateInverse(int body, Vector3F v) => RotationOfBody(body).Conjugated().Rotate(v);

    Vector3F IActuatorHost.LinearVelocityOf(int body)
    {
        var v = Core.Bodies.LinearVelocityOf(body);
        return new Vector3F(v[0], v[1], v[2]);
    }

    Vector3F IActuatorHost.AngularVelocityOf(int body)
    {
        var w = Core.Bodies.AngularVelocityOf(body);
        return new Vector3F(w[0], w[1], w[2]);
    }

    void IActuatorHost.AddForce(int body, Vector3F force, Vector3F torque)
    {
        Force[3 * body] += force.X; Force[3 * body + 1] += force.Y; Force[3 * body + 2] += force.Z;
        Torque[3 * body] += torque.X; Torque[3 * body + 1] += torque.Y; Torque[3 * body + 2] += torque.Z;
    }

    float IActuatorHost.InverseMassAt(int body, Vector3F arm, Vector3F direction)
    {
        float m = Core.Bodies.InverseMass[body];
        var rxu = Aero.V3.Cross(arm, direction);
        RefreshWorldInertia(body);
        var iw = ApplyWorldInertia(body, rxu);
        return m + Aero.V3.Dot(rxu, iw);
    }

    float IActuatorHost.InverseInertiaAbout(int body, Vector3F axis)
    {
        RefreshWorldInertia(body);
        return Aero.V3.Dot(axis, ApplyWorldInertia(body, axis));
    }

    bool IActuatorHost.ShapeIsSensor(int shape) => IsSensor(shape);
    Vector3F IActuatorHost.ShapeToBody(int shape, Vector3F v) => Local[shape].Rotation.Rotate(v);
    Vector3F IActuatorHost.ShapeOffset(int shape) => Local[shape].Translation;

    int IActuatorHost.FluidCount => FluidHandles.HighWater;

    bool IActuatorHost.GetFluid(int index, out WaterSample water)
    {
        water = default;
        if (!FluidHandles.IsLive(index)) { return false; }
        var f = Fluids[index]!;
        water = new WaterSample { Normal = f.SurfaceNormal, Offset = f.SurfaceOffset, Density = f.Density, Flow = f.Flow, Limited = f.LimitToRegion, Min = f.RegionMin, Max = f.RegionMax };
        return true;
    }

    private int _castIgnore = -1;
    private Func<Shape, bool>? _castPredicate;

    bool IActuatorHost.Cast(Vector3D origin, Vector3F direction, float maxDistance, int ignoreBody, out float distance, out Vector3F normal, out int hitBody)
    {
        distance = 0f; normal = default; hitBody = -1;
        double lim = BoxLimits.MaxCoordinate;
        if (!(Math.Abs(origin.X) <= lim && Math.Abs(origin.Y) <= lim && Math.Abs(origin.Z) <= lim)) { return false; }
        _castPredicate ??= sh => Core.Shapes.Owner[sh.Handle.Index] != _castIgnore;
        _castIgnore = ignoreBody;
        var filter = new QueryFilter3D { Predicate = _castPredicate };
        bool hit = World!.Raycast(origin, direction, out var h, maxDistance, in filter);
        _castIgnore = -1;
        if (!hit) { return false; }
        distance = h.Distance; normal = h.Normal; hitBody = h.Body.Handle.Index;
        return true;
    }

    /// <summary>
    /// The silhouette of a shape seen along a direction: the buoyancy cutter with the whole shape below its plane, so the projected area of the "submerged" part is the shape's own. Spheres are a disc; convex shapes use their polyhedron;
    /// compounds sum their children (overlapping children count twice); meshes and height fields have none.
    /// </summary>
    float IActuatorHost.SilhouetteArea(int shape, Vector3F directionInBodyFrame)
    {
        var geom = Geoms[GeomOf[shape]]!;
        if (geom.Kind != GeomKind.Convex && geom.Kind != GeomKind.Compound) { return 0f; }
        var l = Local[shape];
        var dir = l.Rotation.Conjugated().Rotate(directionInBodyFrame);
        float len2 = dir.X * dir.X + dir.Y * dir.Y + dir.Z * dir.Z;
        if (!(len2 > 0.25f)) { return 0f; }
        int body = Core.Shapes.Owner[shape];
        // The shape frame's origin sits at the plane's height h0 below the surface by more than any vertex can reach.
        float reach = MaxExtent[body] + 2f * (float)Math.Sqrt(l.Translation.X * l.Translation.X + l.Translation.Y * l.Translation.Y + l.Translation.Z * l.Translation.Z) + 1f;
        float h0 = -reach;
        var n = new Vector3F(0f, 1f, 0f);
        var cache = geom.Buoyancy ??= new BuoyancyCache();
        if (geom.Kind == GeomKind.Convex) { return SilhouettePiece(geom.Convex, cache, 0, 1, RigidTransformF.Identity, n, h0, dir); }
        var comp = geom.Compound!;
        float area = 0f;
        for (int i = 0; i < comp.SlotCount; i++)
        {
            if (!comp.IsLive(i)) { continue; }
            var child = comp.GetChild(i);
            area += SilhouettePiece(child.Shape, cache, i, comp.SlotCount, child.Local, n, h0, dir);
        }

        return area;
    }

    private float SilhouettePiece(in ConvexShape3D shape, BuoyancyCache cache, int slot, int slots, in RigidTransformF pose, Vector3F n, float h0, Vector3F dir)
    {
        CutResult r;
        if (shape.Kind == ConvexShapeKind3D.Sphere)
        {
            if (!Buoyancy3D.CutSphere(shape.ConvexRadius, pose.Translation, n, h0, out r)) { return 0f; }
        }
        else
        {
            if (cache.Meshes.Length < slots) { Array.Resize(ref cache.Meshes, slots); Array.Resize(ref cache.Built, slots); }
            if (!cache.Built[slot]) { cache.Meshes[slot] = BuoyancyMesh.For(shape); cache.Built[slot] = true; }
            var mesh = cache.Meshes[slot];
            if (mesh == null || !_cutter.Cut(mesh, pose, n, h0, dir, out r)) { return 0f; }
        }

        return r.ProjectedArea;
    }

    /// <summary>The planar and spatial engines present the same hooks; the 3D engine has nothing to move when the origin shifts, since the store keeps body-frame data only.</summary>
    internal bool ActuatorsConsistent() => _actuators == null || _actuators.Consistent();
}
