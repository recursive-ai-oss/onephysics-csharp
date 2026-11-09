namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    private void BindBodyControlOptions()
    {
        _core.Controls.DefaultAngularSpeed = _engine.Opt.MaxAngularSpeed;
        _engine.Opt.ReadAngularSpeed = () => _core.Controls.DefaultAngularSpeed;
        _engine.Opt.WriteAngularSpeed = value => { using var mutation = RecordMutation("WorldOptions2D.MaxAngularSpeed"); _core.Controls.DefaultAngularSpeed = value; };
    }
    /// <summary>Raw-handle mass override in kg; dynamic bodies only. Same validation and recording as Body.SetMass.</summary>
    public void SetBodyMass(RecursiveAI.OnePhysics.BodyHandle body, float mass) => GetBody(body).SetMass(mass);
    /// <summary>Raw-handle local center override in metres; dynamic bodies only. Same validation and recording as Body.CenterOfMass.</summary>
    public void SetBodyCenterOfMass(RecursiveAI.OnePhysics.BodyHandle body, RecursiveAI.OneMath.Vector2F center) { var b = GetBody(body); b.CenterOfMass = center; }
    /// <summary>Raw-handle inertia override in kg m²; dynamic bodies only. Same validation and recording as Body.SetInertia.</summary>
    public void SetBodyInertia(RecursiveAI.OnePhysics.BodyHandle body, float inertia) => GetBody(body).SetInertia(inertia);
    /// <summary>Raw-handle reset of all shape-derived mass properties; dynamic bodies only.</summary>
    public void ResetBodyMassProperties(RecursiveAI.OnePhysics.BodyHandle body) { var b = GetBody(body); b.ResetMass(); b.ResetCenterOfMass(); b.ResetInertia(); }
    /// <summary>Raw-handle queued kinematic target. Same validation, units and behavior as Body.MoveKinematic.</summary>
    public void MoveKinematic(RecursiveAI.OnePhysics.BodyHandle body, RecursiveAI.OneMath.Vector2D position, float rotation) => GetBody(body).MoveKinematic(position, rotation);
}
