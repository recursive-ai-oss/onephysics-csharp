using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>Adds a body with one mass-centre-decorated shape. Geometry does not move; offset is local metres within ±2^20. Defaults: origin, zero angle, dynamic, default material, density 1 kg/m². Throws InvalidShapeException for invalid offsets, ArgumentOutOfRangeException for invalid pose/density and ArgumentException for default geometry. Zero-area shapes remain massless contributors.</summary>
    public Body AddWithCenterOfMassOffset(in Shape2D geometry, Vector2F offset, Vector2D at = default, float angleRadians = 0f, BodyType type = BodyType.Dynamic, Material2D? material = null, float density = 1f)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddWithCenterOfMassOffset");
        var decorated = geometry.WithCenterOfMassOffset(new(offset.X, offset.Y));
        var body = CreateBody(type, at, angleRadians);
        try { CreateShape(body, decorated, ShapeFor(material, density)); }
        catch (Exception) { body.Destroy(); throw; }
        return body;
    }

    /// <summary>Attaches validated local geometry, including a per-shape mass-centre offset. Default definition uses unit density and the default material. A default Shape2D is rejected; use Shape2D.Empty for a placeholder. Throws ArgumentException for default geometry or a foreign body and InvalidHandleException for a stale body.</summary>
    public Shape CreateShape(Body body, in Shape2D geometry, ShapeDef? def = null)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.CreateShape");
        if (geometry.Count == 0 && geometry.Kind != ShapeKind2D.Empty) throw new ArgumentException("Default geometry has no shape kind; use Shape2D.Empty.", nameof(geometry));
        return AddShape(body, geometry, def);
    }
}
