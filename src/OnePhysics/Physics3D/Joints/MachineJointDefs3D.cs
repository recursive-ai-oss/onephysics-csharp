using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Suspension with independent wheel spin. Values are validated on creation; defaults apply to the zero struct.</summary>
public struct WheelJointDef : IJointDef3D
{
    /// <summary>First body, in the destination world.</summary>
    public Body BodyA { get; set; }
    /// <summary>Second body, in the destination world.</summary>
    public Body BodyB { get; set; }
    /// <summary>Whether connected bodies collide. Default false.</summary>
    public bool CollideConnected { get; set; }
    /// <summary>Whole-step mean COM force threshold, N; infinity by default.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }
    private uint _breakForce;
    /// <summary>Whole-step mean COM torque threshold, N m; infinity by default.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }
    private uint _breakTorque;
    /// <summary>Whole steps until expiry; zero by default means unlimited, negative values throw.</summary>
    public int LifetimeSteps { get; set; }
    /// <summary>Frame on A; x is suspension, y is spin. Default identity.</summary>
    public JointFrame3D LocalFrameA { get; set; }
    /// <summary>Frame on B; its y axis is kept aligned with A's y axis. Default identity.</summary>
    public JointFrame3D LocalFrameB { get; set; }
    /// <summary>Enable suspension travel limits. Default false.</summary>
    public bool EnableLimit { get; set; }
    /// <summary>Lower suspension travel, metres. Default zero; must not exceed upper.</summary>
    public float LowerTranslation { get; set; }
    /// <summary>Upper suspension travel, metres. Default zero.</summary>
    public float UpperTranslation { get; set; }
    /// <summary>Enable spin motor. Default false.</summary>
    public bool EnableMotor { get; set; }
    /// <summary>Relative spin rate about frame y, rad/s. Default zero.</summary>
    public float MotorSpeed { get; set; }
    /// <summary>Motor torque cap, N m; finite and nonnegative. Default zero.</summary>
    public float MaxMotorTorque { get; set; }
    /// <summary>Enable suspension spring towards zero travel. Default false.</summary>
    public bool EnableSpring { get; set; }
    /// <summary>Suspension frequency, Hz; finite and nonnegative. Default zero.</summary>
    public float SpringHertz { get; set; }
    /// <summary>Damping ratio; finite and nonnegative, one is critical. Default zero.</summary>
    public float SpringDampingRatio { get; set; }
}

/// <summary>Capped relative-pose motor. Values are validated on creation; defaults apply to the zero struct.</summary>
public struct MotorJointDef : IJointDef3D
{
    /// <summary>First body, in the destination world.</summary>
    public Body BodyA { get; set; }
    /// <summary>Second body, in the destination world.</summary>
    public Body BodyB { get; set; }
    /// <summary>Whether connected bodies collide. Default false.</summary>
    public bool CollideConnected { get; set; }
    /// <summary>Whole-step mean COM force threshold, N; infinity by default.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }
    private uint _breakForce;
    /// <summary>Whole-step mean COM torque threshold, N m; infinity by default.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }
    private uint _breakTorque;
    /// <summary>Whole steps until expiry; zero by default means unlimited, negative values throw.</summary>
    public int LifetimeSteps { get; set; }
    /// <summary>B's desired origin offset in A's local frame, metres. Default zero.</summary>
    public Vector3F LinearOffset { get; set; }
    /// <summary>B's desired orientation relative to A; default identity. Finite nonzero quaternions are normalized.</summary>
    public QuaternionF AngularOffset { readonly get => _offset.Get(QuaternionF.Identity); set => _offset.Set(value, QuaternionF.Identity); }
    private ZeroDefaultQuat _offset;
    /// <summary>Linear force vector magnitude cap, N; finite and nonnegative. Default zero.</summary>
    public float MaxForce { get; set; }
    /// <summary>Angular torque vector magnitude cap, N m; finite and nonnegative. Default zero.</summary>
    public float MaxTorque { get; set; }
    /// <summary>Pose error fraction corrected per substep, in [0,1]. Default 0.3.</summary>
    public float CorrectionFactor { readonly get => ZeroDefault.Get(_correction, 0.3f); set => _correction = ZeroDefault.Set(value, 0.3f); }
    private uint _correction;
}

/// <summary>Soft capped drag to a world point. Body A is a graph endpoint and receives no impulse. Values are validated on creation; defaults apply to the zero struct.</summary>
public struct MouseJointDef : IJointDef3D
{
    /// <summary>First body, in the destination world.</summary>
    public Body BodyA { get; set; }
    /// <summary>Second body, in the destination world.</summary>
    public Body BodyB { get; set; }
    /// <summary>Whether connected bodies collide. Default false.</summary>
    public bool CollideConnected { get; set; }
    /// <summary>Whole-step mean COM force threshold, N; infinity by default.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }
    private uint _breakForce;
    /// <summary>Whole-step mean COM torque threshold, N m; infinity by default.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }
    private uint _breakTorque;
    /// <summary>Whole steps until expiry; zero by default means unlimited, negative values throw.</summary>
    public int LifetimeSteps { get; set; }
    /// <summary>Dragged anchor in B's local origin frame, metres. Default zero.</summary>
    public Vector3F LocalAnchorB { get; set; }
    /// <summary>World target, metres within +-2^21, finite. Default zero.</summary>
    public Vector3D Target { get; set; }
    /// <summary>Force vector magnitude cap, N; finite and nonnegative. Default zero.</summary>
    public float MaxForce { get; set; }
    /// <summary>Drag spring frequency, Hz; finite and nonnegative. Default 5.</summary>
    public float Hertz { readonly get => ZeroDefault.Get(_hertz, 5f); set => _hertz = ZeroDefault.Set(value, 5f); }
    private uint _hertz;
    /// <summary>Drag damping ratio; finite and nonnegative. Default 0.7.</summary>
    public float DampingRatio { readonly get => ZeroDefault.Get(_damping, 0.7f); set => _damping = ZeroDefault.Set(value, 0.7f); }
    private uint _damping;
}

