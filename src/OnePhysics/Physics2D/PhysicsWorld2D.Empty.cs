using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>Attaches an empty placeholder, with no geometry, mass contribution or sensor overlap. Body mass/inertia overrides are unaffected. Throws ArgumentException for a foreign body and the usual shape-definition errors.</summary>
    public Shape CreateEmpty(Body body, ShapeDef? def = null)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.CreateEmpty");
        return AddShape(body, Shape2D.Empty, def);
    }

    /// <summary>Adds a dynamic, non-colliding placeholder with explicit mass and inertia overrides. Defaults: origin, mass 1 kg, inertia 1 kg m². Sensor overlap has no meaning for empty geometry and is always false. Throws ArgumentOutOfRangeException for invalid position or overrides (finite 1e-18..1e30).</summary>
    public Body AddEmpty(Vector2D at = default, float mass = 1f, float inertia = 1f)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddEmpty");
        var body = CreateBody(BodyType.Dynamic, at);
        try { CreateEmpty(body); body.Mass = mass; body.SetInertia(inertia); }
        catch (Exception) { body.Destroy(); throw; }
        return body;
    }
}
