using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>Creates a suspension with frame x travel and frame y spin; invalid bodies, limits or non-finite parameters throw before mutation.</summary>
    public WheelJoint CreateJoint(in WheelJointDef def)
    {
        using var mutation = RecordMutation("PhysicsWorld3D.CreateJoint");
        var d = Base(JointType3D.Wheel, Flags(def.EnableLimit, def.EnableMotor, def.EnableSpring, def.CollideConnected), def.LocalFrameA, def.LocalFrameB, def.BreakForce, def.BreakTorque, def.LifetimeSteps);
        d.Lower = def.LowerTranslation; d.Upper = def.UpperTranslation; d.MotorSpeed = def.MotorSpeed; d.MaxMotor = def.MaxMotorTorque;
        d.Hertz = def.SpringHertz; d.Damping = def.SpringDampingRatio;
        return new(Make(def.BodyA, def.BodyB, d));
    }

    /// <summary>Creates a capped relative-pose drive. Invalid bodies, negative caps or correction outside [0,1] throw before mutation.</summary>
    public RelativeJoint CreateJoint(in MotorJointDef def)
    {
        using var mutation = RecordMutation("PhysicsWorld3D.CreateJoint");
        var d = Base(JointType3D.Motor, Flags(false, false, false, def.CollideConnected), new JointFrame3D(def.LinearOffset, def.AngularOffset), default, def.BreakForce, def.BreakTorque, def.LifetimeSteps);
        d.MaxMotor = def.MaxForce; d.Ratio = def.MaxTorque; d.Length = def.CorrectionFactor;
        return new(Make(def.BodyA, def.BodyB, d));
    }

    /// <summary>Creates a capped spring pulling B's anchor to a world target. Invalid bodies, target or negative/non-finite spring parameters throw before mutation.</summary>
    public TargetJoint CreateJoint(in MouseJointDef def)
    {
        using var mutation = RecordMutation("PhysicsWorld3D.CreateJoint");
        var d = Base(JointType3D.Mouse, Flags(false, false, true, def.CollideConnected), default, new JointFrame3D(def.LocalAnchorB, QuaternionF.Identity), def.BreakForce, def.BreakTorque, def.LifetimeSteps);
        d.FxA = def.Target.X; d.FyA = def.Target.Y; d.FzA = def.Target.Z;
        d.MaxMotor = def.MaxForce; d.Hertz = def.Hertz; d.Damping = def.DampingRatio;
        return new(Make(def.BodyA, def.BodyB, d));
    }

    /// <summary>Adds a wheel at a world anchor, suspension along axis and spin along the deterministically chosen perpendicular frame y axis. Spring defaults to 5 Hz, damping ratio 0.7; collision disabled.</summary>
    public WheelJoint AddWheel(Body chassis, Body wheel, Vector3D anchor, Vector3F axis, bool collideConnected = false)
    {
        var frame = WorldFrame(axis, nameof(axis));
        return CreateJoint(new WheelJointDef { BodyA = chassis, BodyB = wheel, LocalFrameA = FrameOn(chassis, anchor, frame, nameof(anchor)), LocalFrameB = FrameOn(wheel, anchor, frame, nameof(anchor)), EnableSpring = true, SpringHertz = 5f, SpringDampingRatio = 0.7f, CollideConnected = collideConnected });
    }

    /// <summary>Drives B to a pose relative to A, with finite nonnegative force and torque caps (N and N m); invalid values throw.</summary>
    public RelativeJoint AddMotor(Body a, Body b, Vector3F linearOffset, QuaternionF angularOffset, float maxForce, float maxTorque)
        => CreateJoint(new MotorJointDef { BodyA = a, BodyB = b, LinearOffset = linearOffset, AngularOffset = angularOffset, MaxForce = maxForce, MaxTorque = maxTorque });

    /// <summary>Pulls B's origin towards a finite world target within +-2^21; maxForce is a finite nonnegative cap in N. Default drag spring 5 Hz, damping ratio 0.7.</summary>
    public TargetJoint AddMouse(Body a, Body b, Vector3D target, float maxForce)
        => CreateJoint(new MouseJointDef { BodyA = a, BodyB = b, Target = target, MaxForce = maxForce });
    /// <summary>Softly drags the local point currently at target; target is finite within +-2^21. Spring defaults to 5 Hz and 0.7 damping. A nonpositive maxForce selects at least 1000 N, scaled by body mass.</summary>
    public TargetJoint AddTarget(Body ground, Body body, Vector3D target, float hertz = 5f, float dampingRatio = 0.7f, float maxForce = 0f)
    {
        if (float.IsNaN(maxForce)) { throw new System.ArgumentOutOfRangeException(nameof(maxForce), maxForce, "Target force cap must not be NaN."); }
        return CreateJoint(new MouseJointDef { BodyA = ground, BodyB = body, LocalAnchorB = ToLocal(body, target, nameof(target)), Target = target, Hertz = hertz, DampingRatio = dampingRatio, MaxForce = maxForce > 0 ? maxForce : 1000f * System.Math.Max(1f, body.Mass) });
    }

    /// <summary>Holds the current relative pose with force/torque caps, default 1000 N and 1000 N m. Invalid bodies or negative/non-finite caps throw.</summary>
    public RelativeJoint AddRelative(Body a, Body b, float maxForce = 1000f, float maxTorque = 1000f, bool collideConnected = false)
    {
        var delta = b.Position - a.Position;
        return CreateJoint(new MotorJointDef { BodyA = a, BodyB = b, LinearOffset = a.Rotation.Conjugated().Rotate(new Vector3F((float)delta.X, (float)delta.Y, (float)delta.Z)), AngularOffset = a.Rotation.Conjugated() * b.Rotation, MaxForce = maxForce, MaxTorque = maxTorque, CollideConnected = collideConnected });
    }

}

public readonly partial struct Joint
{
    /// <summary>Sets the motor joint's desired relative pose; local translation in metres within +-2^21, finite nonzero orientation normalized on edit. Other types or invalid values throw before mutation.</summary>
    public void SetOffset(Vector3F linearOffset, QuaternionF angularOffset)
    {
        using var mutation = _world?.RecordMutation("Joint.SetOffset");
        ref var d = ref Rec(out var w, out int c); Need(d, "offset", 1u << (int)JointType3D.Motor);
        var e = d; e.LAx = linearOffset.X; e.LAy = linearOffset.Y; e.LAz = linearOffset.Z; e.QA = Engine3D.Normalize(angularOffset, nameof(angularOffset));
        Commit(w, c, ref d, e);
    }

    /// <summary>Mouse joint's world target in metres within +-2^21. Other types throw; invalid values leave state unchanged.</summary>
    public Vector3D Target
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "target", 1u << (int)JointType3D.Mouse); return new(d.FxA, d.FyA, d.FzA); }
        set { using var mutation = _world?.RecordMutation("Joint.Target"); ref var d = ref Rec(out var w, out int c); Need(in d, "target", 1u << (int)JointType3D.Mouse); var e = d; e.FxA = value.X; e.FyA = value.Y; e.FzA = value.Z; Commit(w, c, ref d, e); }
    }

    /// <summary>Mouse or motor joint's finite nonnegative force cap, N. Other types or invalid values throw.</summary>
    public float MaxForce
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "maximum force", (1u << (int)JointType3D.Mouse) | (1u << (int)JointType3D.Motor)); return d.MaxMotor; }
        set { using var mutation = _world?.RecordMutation("Joint.MaxForce"); ref var d = ref Rec(out var w, out int c); Need(in d, "maximum force", (1u << (int)JointType3D.Mouse) | (1u << (int)JointType3D.Motor)); var e = d; e.MaxMotor = value; Commit(w, c, ref d, e); }
    }

    /// <summary>Motor joint's finite nonnegative torque cap, N m. Other types or invalid values throw.</summary>
    public float MaxTorque
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "maximum torque", 1u << (int)JointType3D.Motor); return d.Ratio; }
        set { using var mutation = _world?.RecordMutation("Joint.MaxTorque"); ref var d = ref Rec(out var w, out int c); Need(in d, "maximum torque", 1u << (int)JointType3D.Motor); var e = d; e.Ratio = value; Commit(w, c, ref d, e); }
    }
}
