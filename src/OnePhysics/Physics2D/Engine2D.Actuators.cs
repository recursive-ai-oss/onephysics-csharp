using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Aero;
using RecursiveAI.OnePhysics.Fields;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D : IActuatorHost
{
    private ActuatorStore? _actuators;

    /// <summary>Lift and drag of shapes, thrusters, boats and hovercraft; created on first use and registered with the world.</summary>
    internal ActuatorStore Actuators => _actuators ??= new ActuatorStore(this);

    WorldCore IActuatorHost.Core => Core;
    bool IActuatorHost.Planar => true;
    int IActuatorHost.EngineSectionId => Id;
    ActuatorStore IActuatorHost.Actuators => Actuators;
    FieldStore IActuatorHost.FieldsOf => Fields;
    float IActuatorHost.MassOf(int body) => Mass[body];
    float IActuatorHost.ReachOf(int body) => MaxExtent[body];
    Vector3F IActuatorHost.LocalCenterOf(int body) => new(LocalCenter[2 * body], LocalCenter[2 * body + 1], 0f);

    Vector3D IActuatorHost.PositionOf(int body)
    {
        var p = Core.Bodies.PositionOf(body);
        return new Vector3D(p[0], p[1], 0.0);
    }

    Vector3F IActuatorHost.Rotate(int body, Vector3F v)
    {
        var r = Core.Bodies.RotationOf(body);
        return new Vector3F(r[0] * v.X - r[1] * v.Y, r[1] * v.X + r[0] * v.Y, v.Z);
    }

    Vector3F IActuatorHost.RotateInverse(int body, Vector3F v)
    {
        var r = Core.Bodies.RotationOf(body);
        return new Vector3F(r[0] * v.X + r[1] * v.Y, -r[1] * v.X + r[0] * v.Y, v.Z);
    }

    Vector3F IActuatorHost.LinearVelocityOf(int body)
    {
        var v = Core.Bodies.LinearVelocityOf(body);
        return new Vector3F(v[0], v[1], 0f);
    }

    Vector3F IActuatorHost.AngularVelocityOf(int body) => new(0f, 0f, Core.Bodies.AngularVelocityOf(body)[0]);

    void IActuatorHost.AddForce(int body, Vector3F force, Vector3F torque)
    {
        Force[2 * body] += force.X; Force[2 * body + 1] += force.Y;
        Torque[body] += torque.Z;
    }

    float IActuatorHost.InverseMassAt(int body, Vector3F arm, Vector3F direction)
    {
        float cross = arm.X * direction.Y - arm.Y * direction.X;
        return Core.Bodies.InverseMass[body] + cross * cross * Core.Bodies.InverseInertia[body];
    }

    float IActuatorHost.InverseInertiaAbout(int body, Vector3F axis) => axis.Z * axis.Z * Core.Bodies.InverseInertia[body];

    bool IActuatorHost.ShapeIsSensor(int shape) => IsSensor(shape);

    // Planar shapes have no frame of their own: their points are in the body's frame, and a surface's reference point is the shape's centroid.
    Vector3F IActuatorHost.ShapeToBody(int shape, Vector3F v) => v;

    Vector3F IActuatorHost.ShapeOffset(int shape)
    {
        ref readonly var g = ref Geom[shape];
        var c = g.Centroid;
        return new Vector3F(c.X, c.Y, 0f);
    }

    /// <summary>The width of a shape across a direction, per metre of depth: the extent of its points along the perpendicular, plus the rounding on both sides. Segments and chains have none.</summary>
    float IActuatorHost.SilhouetteArea(int shape, Vector3F directionInBodyFrame)
    {
        ref readonly var g = ref Geom[shape];
        if (g.Kind != ShapeKind2D.Circle && g.Kind != ShapeKind2D.Capsule && g.Kind != ShapeKind2D.Polygon) { return 0f; }
        float dx = directionInBodyFrame.X, dy = directionInBodyFrame.Y;
        float l2 = dx * dx + dy * dy;
        if (!(l2 > 0.25f)) { return 0f; }
        float px = -dy, py = dx;
        var v = g.Vertices;
        float lo = float.MaxValue, hi = -float.MaxValue;
        for (int i = 0; i < v.Length; i++)
        {
            float p = v[i].X * px + v[i].Y * py;
            lo = p < lo ? p : lo; hi = p > hi ? p : hi;
        }

        return hi - lo + 2f * g.Radius;
    }

    Water.WaterStore? IActuatorHost.ShallowWater => _water != null && _water.Count > 0 ? _water : null;

    int IActuatorHost.FluidCount => 0;

    bool IActuatorHost.GetFluid(int index, out WaterSample water) { water = default; return false; }

    private int _castIgnore = -1;
    private Func<Shape, bool>? _castPredicate;

    bool IActuatorHost.Cast(Vector3D origin, Vector3F direction, float maxDistance, int ignoreBody, out float distance, out Vector3F normal, out int hitBody)
    {
        distance = 0f; normal = default; hitBody = -1;
        double lim = BoxLimits.MaxCoordinate;
        if (!(Math.Abs(origin.X) <= lim && Math.Abs(origin.Y) <= lim)) { return false; }
        _castPredicate ??= sh => Core.Shapes.Owner[sh.Handle.Index] != _castIgnore;
        _castIgnore = ignoreBody;
        var filter = new QueryFilter2D { Predicate = _castPredicate };
        bool hit = World!.Raycast(new Vector2D(origin.X, origin.Y), new Vector2F(direction.X, direction.Y), out var h, maxDistance, in filter);
        _castIgnore = -1;
        if (!hit) { return false; }
        distance = h.Distance; normal = new Vector3F(h.Normal.X, h.Normal.Y, 0f); hitBody = h.Body.Handle.Index;
        return true;
    }

    internal bool ActuatorsConsistent() => _actuators == null || _actuators.Consistent();
}
