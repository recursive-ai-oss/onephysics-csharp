using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>The kinds of 2D joint.</summary>
public enum JointType2D : byte
{
    /// <summary>A hinge: two anchors held together, rotation free (limit, motor and spring optional).</summary>
    Revolute = 1,

    /// <summary>A slider: B moves along an axis fixed in A, rotation locked (limit, motor and spring optional).</summary>
    Prismatic = 2,

    /// <summary>Welds two bodies together; optionally springy.</summary>
    Weld = 3,

    /// <summary>A suspension: B's anchor slides along an axis in A with a spring, and B spins freely (motor and limit optional).</summary>
    Wheel = 4,

    /// <summary>Keeps two anchors a distance apart: rigid, or a spring with a rope-like length range.</summary>
    Distance = 5,

    /// <summary>Drives B's pose relative to A towards an offset, with capped force and torque.</summary>
    Motor = 6,

    /// <summary>Pulls an anchor of B towards a world target like a spring, with capped force (a dragging handle).</summary>
    Mouse = 7,
}

/// <summary>Creation parameters of a revolute (hinge) joint. Anchors are in each body's own frame (measured from the body origin); the angle is B's rotation minus A's rotation minus <see cref="ReferenceAngle"/>.</summary>
public struct RevoluteJointDef
{
    /// <summary>The first body.</summary>
    public Body BodyA { get; set; }

    /// <summary>The second body.</summary>
    public Body BodyB { get; set; }

    /// <summary>Anchor in A's frame, metres. Default (0, 0).</summary>
    public Vector2F LocalAnchorA { get; set; }

    /// <summary>Anchor in B's frame, metres. Default (0, 0).</summary>
    public Vector2F LocalAnchorB { get; set; }

    /// <summary>Relative angle (B minus A) at which the joint angle reads zero, radians. Default 0.</summary>
    public float ReferenceAngle { get; set; }

    /// <summary>Whether the angle is limited to <see cref="LowerAngle"/>..<see cref="UpperAngle"/>. Default false.</summary>
    public bool EnableLimit { get; set; }

    /// <summary>Lower angle limit, radians, within -pi..pi and at most the upper limit. Default 0.</summary>
    public float LowerAngle { get; set; }

    /// <summary>Upper angle limit, radians. Default 0.</summary>
    public float UpperAngle { get; set; }

    /// <summary>Whether the motor drives the relative angular velocity. Default false.</summary>
    public bool EnableMotor { get; set; }

    /// <summary>Target relative angular velocity (B minus A), radians per second. Default 0.</summary>
    public float MotorSpeed { get; set; }

    /// <summary>Largest torque the motor applies, N·m, at least 0. Default 0.</summary>
    public float MaxMotorTorque { get; set; }

    /// <summary>Whether an angular spring pulls the joint angle to zero. Default false.</summary>
    public bool EnableSpring { get; set; }

    /// <summary>Spring frequency, Hz, at least 0. Default 0.</summary>
    public float SpringHertz { get; set; }

    /// <summary>Spring damping ratio (1 is critical), at least 0. Default 0.</summary>
    public float SpringDampingRatio { get; set; }

    /// <summary>When true the two bodies keep colliding with each other. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>The joint breaks at the end of a step in which its reaction force (N) exceeds this. Default infinity (never).</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>The joint breaks when its reaction torque (N·m) exceeds this. Default infinity (never).</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}

/// <summary>Creation parameters of a prismatic (slider) joint. The axis is a direction in A's frame; the translation is measured from the anchors' coincidence along it.</summary>
public struct PrismaticJointDef
{
    /// <summary>The first body.</summary>
    public Body BodyA { get; set; }

    /// <summary>The second body.</summary>
    public Body BodyB { get; set; }

    /// <summary>Anchor in A's frame. Default (0, 0).</summary>
    public Vector2F LocalAnchorA { get; set; }

    /// <summary>Anchor in B's frame. Default (0, 0).</summary>
    public Vector2F LocalAnchorB { get; set; }

    /// <summary>Slide direction in A's frame; normalized on creation, must be non-zero and finite. Default (1, 0).</summary>
    public Vector2F LocalAxisA { readonly get => _localAxisA.Get(new Vector2F(1f, 0f)); set => _localAxisA.Set(value, new Vector2F(1f, 0f)); }

    private ZeroDefault2 _localAxisA;

    /// <summary>Relative angle (B minus A) the joint holds, radians. Default 0.</summary>
    public float ReferenceAngle { get; set; }

    /// <summary>Whether the translation is limited. Default false.</summary>
    public bool EnableLimit { get; set; }

    /// <summary>Lower translation limit, metres, at most the upper limit. Default 0.</summary>
    public float LowerTranslation { get; set; }

    /// <summary>Upper translation limit, metres. Default 0.</summary>
    public float UpperTranslation { get; set; }

    /// <summary>Whether the motor drives the sliding speed. Default false.</summary>
    public bool EnableMotor { get; set; }

    /// <summary>Target sliding speed along the axis, metres per second. Default 0.</summary>
    public float MotorSpeed { get; set; }

    /// <summary>Largest force the motor applies, newtons, at least 0. Default 0.</summary>
    public float MaxMotorForce { get; set; }

    /// <summary>Whether a spring pulls the translation to zero. Default false.</summary>
    public bool EnableSpring { get; set; }

    /// <summary>Spring frequency, Hz, at least 0. Default 0.</summary>
    public float SpringHertz { get; set; }

    /// <summary>Spring damping ratio, at least 0. Default 0.</summary>
    public float SpringDampingRatio { get; set; }

    /// <summary>When true the two bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold for the reaction force, N. Default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>Break threshold for the reaction torque, N·m. Default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}

/// <summary>Creation parameters of a weld joint. With both hertz values 0 the weld is rigid; a positive value makes that part a spring.</summary>
public struct WeldJointDef
{
    /// <summary>The first body.</summary>
    public Body BodyA { get; set; }

    /// <summary>The second body.</summary>
    public Body BodyB { get; set; }

    /// <summary>Anchor in A's frame. Default (0, 0).</summary>
    public Vector2F LocalAnchorA { get; set; }

    /// <summary>Anchor in B's frame. Default (0, 0).</summary>
    public Vector2F LocalAnchorB { get; set; }

    /// <summary>Relative angle (B minus A) the weld holds, radians. Default 0.</summary>
    public float ReferenceAngle { get; set; }

    /// <summary>Linear spring frequency, Hz; 0 makes the position rigid. Default 0.</summary>
    public float LinearHertz { get; set; }

    /// <summary>Linear damping ratio, at least 0. Default 0.</summary>
    public float LinearDampingRatio { get; set; }

    /// <summary>Angular spring frequency, Hz; 0 makes the angle rigid. Default 0.</summary>
    public float AngularHertz { get; set; }

    /// <summary>Angular damping ratio, at least 0. Default 0.</summary>
    public float AngularDampingRatio { get; set; }

    /// <summary>When true the two bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold for the reaction force, N. Default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>Break threshold for the reaction torque, N·m. Default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}

/// <summary>Creation parameters of a wheel joint: a suspension along an axis of A, with B free to spin.</summary>
public struct WheelJointDef
{
    /// <summary>The chassis.</summary>
    public Body BodyA { get; set; }

    /// <summary>The wheel.</summary>
    public Body BodyB { get; set; }

    /// <summary>Anchor in A's frame. Default (0, 0).</summary>
    public Vector2F LocalAnchorA { get; set; }

    /// <summary>Anchor in B's frame. Default (0, 0).</summary>
    public Vector2F LocalAnchorB { get; set; }

    /// <summary>Suspension direction in A's frame; normalized on creation. Default (0, 1).</summary>
    public Vector2F LocalAxisA { readonly get => _localAxisA.Get(new Vector2F(0f, 1f)); set => _localAxisA.Set(value, new Vector2F(0f, 1f)); }

    private ZeroDefault2 _localAxisA;

    /// <summary>Whether the suspension is limited. Default false.</summary>
    public bool EnableLimit { get; set; }

    /// <summary>Lower translation limit, metres. Default 0.</summary>
    public float LowerTranslation { get; set; }

    /// <summary>Upper translation limit, metres. Default 0.</summary>
    public float UpperTranslation { get; set; }

    /// <summary>Whether the motor spins the wheel. Default false.</summary>
    public bool EnableMotor { get; set; }

    /// <summary>Target relative angular velocity (B minus A), radians per second. Default 0.</summary>
    public float MotorSpeed { get; set; }

    /// <summary>Largest motor torque, N·m. Default 0.</summary>
    public float MaxMotorTorque { get; set; }

    /// <summary>Whether the suspension spring is on. Default true.</summary>
    public bool EnableSpring { readonly get => !_enableSpringOff; set => _enableSpringOff = !value; }

    private bool _enableSpringOff;

    /// <summary>Suspension frequency, Hz. Default 1.</summary>
    public float SpringHertz { readonly get => ZeroDefault.Get(_springHertz, 1f); set => _springHertz = ZeroDefault.Set(value, 1f); }

    private uint _springHertz;

    /// <summary>Suspension damping ratio. Default 0.7.</summary>
    public float SpringDampingRatio { readonly get => ZeroDefault.Get(_springDampingRatio, 0.7f); set => _springDampingRatio = ZeroDefault.Set(value, 0.7f); }

    private uint _springDampingRatio;

    /// <summary>When true the two bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold for the reaction force, N. Default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>Break threshold for the reaction torque, N·m. Default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}

/// <summary>Creation parameters of a distance joint. Rigid at <see cref="Length"/> unless <see cref="EnableSpring"/> is set and the length range is not degenerate.</summary>
public struct DistanceJointDef
{
    /// <summary>The first body.</summary>
    public Body BodyA { get; set; }

    /// <summary>The second body.</summary>
    public Body BodyB { get; set; }

    /// <summary>Anchor in A's frame. Default (0, 0).</summary>
    public Vector2F LocalAnchorA { get; set; }

    /// <summary>Anchor in B's frame. Default (0, 0).</summary>
    public Vector2F LocalAnchorB { get; set; }

    /// <summary>Rest length, metres, at least 0.005 (the linear slop). Default 1.</summary>
    public float Length { readonly get => ZeroDefault.Get(_length, 1f); set => _length = ZeroDefault.Set(value, 1f); }

    private uint _length;

    /// <summary>Whether a spring acts along the joint. Default false (rigid).</summary>
    public bool EnableSpring { get; set; }

    /// <summary>Spring frequency, Hz. Default 0.</summary>
    public float SpringHertz { get; set; }

    /// <summary>Spring damping ratio. Default 0.</summary>
    public float SpringDampingRatio { get; set; }

    /// <summary>Whether the length is limited to <see cref="MinLength"/>..<see cref="MaxLength"/> while the spring is on. Default false.</summary>
    public bool EnableLimit { get; set; }

    /// <summary>Shortest length when limited, metres, at least 0. Default 0.</summary>
    public float MinLength { get; set; }

    /// <summary>Longest length when limited, metres, at least <see cref="MinLength"/>. Default very large.</summary>
    public float MaxLength { readonly get => ZeroDefault.Get(_maxLength, float.MaxValue); set => _maxLength = ZeroDefault.Set(value, float.MaxValue); }

    private uint _maxLength;

    /// <summary>Whether a motor changes the length at <see cref="MotorSpeed"/> while the spring is on. Default false.</summary>
    public bool EnableMotor { get; set; }

    /// <summary>Target rate of change of the length, metres per second. Default 0.</summary>
    public float MotorSpeed { get; set; }

    /// <summary>Largest motor force, N. Default 0.</summary>
    public float MaxMotorForce { get; set; }

    /// <summary>When true the two bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold for the reaction force, N. Default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>Break threshold for the reaction torque, N·m (always 0 for this joint, so only infinity makes sense). Default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}

/// <summary>Creation parameters of a motor joint: B's origin is driven to <see cref="LinearOffset"/> (in A's frame) from A's origin and B's angle to <see cref="AngularOffset"/> from A's, with capped force and torque.</summary>
public struct MotorJointDef
{
    /// <summary>The first body (the reference frame).</summary>
    public Body BodyA { get; set; }

    /// <summary>The driven body.</summary>
    public Body BodyB { get; set; }

    /// <summary>Target position of B's origin relative to A's origin, in A's frame, metres. Default (0, 0).</summary>
    public Vector2F LinearOffset { get; set; }

    /// <summary>Target angle of B relative to A, radians. Default 0.</summary>
    public float AngularOffset { get; set; }

    /// <summary>Largest force, N, at least 0. Default 0.</summary>
    public float MaxForce { get; set; }

    /// <summary>Largest torque, N·m, at least 0. Default 0.</summary>
    public float MaxTorque { get; set; }

    /// <summary>Fraction of the pose error removed per substep, within 0..1. Default 0.3.</summary>
    public float CorrectionFactor { readonly get => ZeroDefault.Get(_correctionFactor, 0.3f); set => _correctionFactor = ZeroDefault.Set(value, 0.3f); }

    private uint _correctionFactor;

    /// <summary>When true the two bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold for the reaction force, N. Default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>Break threshold for the reaction torque, N·m. Default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}

/// <summary>Creation parameters of a mouse (target) joint: an anchor on <see cref="BodyB"/> is pulled to <see cref="Target"/> by a spring with capped force. <see cref="BodyA"/> only anchors the joint in the constraint graph (a static ground body is typical) and is not moved by it.</summary>
public struct MouseJointDef
{
    /// <summary>The reference body, usually a static ground body.</summary>
    public Body BodyA { get; set; }

    /// <summary>The dragged body.</summary>
    public Body BodyB { get; set; }

    /// <summary>Anchor in B's frame, metres. Default (0, 0).</summary>
    public Vector2F LocalAnchorB { get; set; }

    /// <summary>World position the anchor is pulled to (within +-2^21).</summary>
    public Vector2D Target { get; set; }

    /// <summary>Spring frequency, Hz, at least 0. Default 5.</summary>
    public float Hertz { readonly get => ZeroDefault.Get(_hertz, 5f); set => _hertz = ZeroDefault.Set(value, 5f); }

    private uint _hertz;

    /// <summary>Spring damping ratio, at least 0. Default 0.7.</summary>
    public float DampingRatio { readonly get => ZeroDefault.Get(_dampingRatio, 0.7f); set => _dampingRatio = ZeroDefault.Set(value, 0.7f); }

    private uint _dampingRatio;

    /// <summary>Largest force, N, at least 0. Default 1000.</summary>
    public float MaxForce { readonly get => ZeroDefault.Get(_maxForce, 1000f); set => _maxForce = ZeroDefault.Set(value, 1000f); }

    private uint _maxForce;

    /// <summary>When true the two bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold for the reaction force, N. Default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>Break threshold for the reaction torque, N·m. Default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}
