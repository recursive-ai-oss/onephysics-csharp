using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>Adds a body with one mass-centre-decorated shape. Geometry does not move; offset is local metres within ±2^21. Defaults: origin, identity rotation, dynamic, default material, density 1 kg/m³ (kg/m² for triangles). Invalid pose, geometry, density or offsets throw ArgumentException; static-only geometry on non-static bodies throws StaticOnlyShapeException. Massless shapes stay massless contributors.</summary>
    public Body AddWithCenterOfMassOffset(ShapeSpec3D shape, Vector3F offset, Vector3D at = default, QuaternionF rotation = default, BodyType type = BodyType.Dynamic, Material3D? material = null, float density = 1f)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.AddWithCenterOfMassOffset");
        return AddOne(type, at, rotation, shape.WithCenterOfMassOffset(offset), material, density);
    }

    /// <summary>Adds an infinite static half-space, dot(normal, local point) ≤ offset, at the given body origin and rotation. Default material, offset zero. Normal is normalized; invalid values throw ArgumentOutOfRangeException. Debug geometry emits a surface point and outward normal endpoint.</summary>
    public Body AddPlane(Vector3F normal, float offset = 0f, Vector3D at = default, QuaternionF rotation = default, Material3D? material = null)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.AddPlane");
        return AddOne(BodyType.Static, at, rotation, ShapeSpec3D.Plane(normal, offset), material, 0f);
    }

    /// <summary>Attaches an empty placeholder, without collision geometry, mass contribution or sensor overlap. Body mass/inertia overrides are unaffected. Throws ArgumentException for a foreign body and the usual definition errors.</summary>
    public Shape CreateEmpty(Body body, ShapeDef? def = null)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.CreateEmpty");
        return CreateShape(body, ShapeSpec3D.Empty, def);
    }

    /// <summary>Adds a dynamic non-colliding placeholder with explicit mass and inertia overrides. Defaults: origin, mass 1 kg and principal inertia (1,1,1) kg m². Sensor overlap has no meaning for empty geometry and is always false. Throws ArgumentOutOfRangeException for invalid position or overrides (finite 1e-18..1e30).</summary>
    public Body AddEmpty(Vector3D at = default, float mass = 1f, Vector3F? inertia = null)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.AddEmpty");
        var body = CreateBody(BodyType.Dynamic, at);
        try { CreateEmpty(body); body.Mass = mass; body.SetInertia(inertia ?? new Vector3F(1,1,1), QuaternionF.Identity); }
        catch (Exception) { body.Destroy(); throw; }
        return body;
    }

    /// <summary>Adds a double-sided triangular lamina. Vertices are metres in the body frame, finite within ±2^21 and non-collinear. Default: static body, zero inflation and unit area density; inflated mass ignores the skin. Throws ArgumentOutOfRangeException for invalid dimensions.</summary>
    public Body AddTriangle(Vector3F a, Vector3F b, Vector3F c, Vector3D at = default, QuaternionF rotation = default, BodyType type = BodyType.Static, Material3D? material = null, float density = 1f, float convexRadius = 0f)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.AddTriangle");
        return AddOne(type, at, rotation, ShapeSpec3D.Triangle(a, b, c, convexRadius), material, density);
    }

    /// <summary>Adds a circular frustum along Y. Lengths are metres; half height must be positive, radii non-negative and not both zero, finite and at most 2^21. Defaults: dynamic body, unit density and default material. Throws ArgumentOutOfRangeException for invalid dimensions.</summary>
    public Body AddTaperedCylinder(float halfHeight, float topRadius, float bottomRadius, Vector3D at = default, QuaternionF rotation = default, BodyType type = BodyType.Dynamic, Material3D? material = null, float density = 1f)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.AddTaperedCylinder");
        return AddOne(type, at, rotation, ShapeSpec3D.TaperedCylinder(halfHeight, topRadius, bottomRadius), material, density);
    }
}
