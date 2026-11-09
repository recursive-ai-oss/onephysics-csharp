namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    private void BindBodyControlOptions()
    {
        _core.Controls.DefaultAngularSpeed = _engine.Opt.MaxAngularSpeed;
        _engine.Opt.ReadAngularSpeed = () => _core.Controls.DefaultAngularSpeed;
        _engine.Opt.WriteAngularSpeed = value => { using var mutation = RecordMutation("WorldOptions3D.MaxAngularSpeed"); _core.Controls.DefaultAngularSpeed = value; };
    }
    /// <summary>Raw-handle mass override in kg; dynamic bodies only. Same validation and recording as Body.SetMass.</summary>
    public void SetBodyMass(RecursiveAI.OnePhysics.BodyHandle body, float mass) => GetBody(body).SetMass(mass);
    /// <summary>Raw-handle local center override in metres; dynamic bodies only. Same validation and recording as Body.CenterOfMass.</summary>
    public void SetBodyCenterOfMass(RecursiveAI.OnePhysics.BodyHandle body, RecursiveAI.OneMath.Vector3F center) { var b = GetBody(body); b.CenterOfMass = center; }
    /// <summary>Raw-handle inertia override in kg m²; dynamic bodies only. Same validation and recording as Body.SetInertia.</summary>
    public void SetBodyInertia(RecursiveAI.OnePhysics.BodyHandle body, RecursiveAI.OneMath.Vector3F inertia, RecursiveAI.OneMath.QuaternionF rotation) => GetBody(body).SetInertia(inertia, rotation);
    /// <summary>Raw-handle reset of all shape-derived mass properties; dynamic bodies only.</summary>
    public void ResetBodyMassProperties(RecursiveAI.OnePhysics.BodyHandle body) { var b = GetBody(body); b.ResetMass(); b.ResetCenterOfMass(); b.ResetInertia(); }
    /// <summary>Raw-handle queued kinematic target. Same validation, units and behavior as Body.MoveKinematic.</summary>
    public void MoveKinematic(RecursiveAI.OnePhysics.BodyHandle body, RecursiveAI.OneMath.Vector3D position, RecursiveAI.OneMath.QuaternionF rotation) => GetBody(body).MoveKinematic(position, rotation);
}
