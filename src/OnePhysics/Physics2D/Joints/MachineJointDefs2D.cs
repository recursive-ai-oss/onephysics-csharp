using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>How one planar configurable axis moves.</summary>
public enum JointAxisMode : byte
{
    /// <summary>Held at zero.</summary>
    Locked,
    /// <summary>Restricted to a finite range.</summary>
    Limited,
    /// <summary>Unconstrained except optional motors and springs.</summary>
    Free,
}

/// <summary>Planar configurable axes, using the corresponding spatial axis bits. Select exactly one axis for edits.</summary>
[System.Flags]
public enum JointAxes : byte
{
    /// <summary>No axis; invalid for an axis edit.</summary>
    None = 0,
    /// <summary>Frame x translation.</summary>
    LinearX = 1,
    /// <summary>Frame y translation.</summary>
    LinearY = 2,
    /// <summary>Rotation about the plane normal.</summary>
    AngularZ = 32,
}

/// <summary>Settings for one configurable axis, in metres or radians, N or N m. Zero struct locks the axis without a drive. All scalar settings must be finite; force, frequency and damping must be nonnegative.</summary>
public struct JointAxisSettings
{
    /// <summary>Axis mode; default locked.</summary>
    public JointAxisMode Mode { get; set; }
    /// <summary>Lower limited value; default zero, at most upper.</summary>
    public float Lower { get; set; }
    /// <summary>Upper limited value; default zero.</summary>
    public float Upper { get; set; }
    /// <summary>Enable speed motor; default false.</summary>
    public bool EnableMotor { get; set; }
    /// <summary>Speed target, metres/s or rad/s; default zero.</summary>
    public float MotorSpeed { get; set; }
    /// <summary>Motor force or torque cap; default zero.</summary>
    public float MaxMotor { get; set; }
    /// <summary>Enable spring to zero; default false.</summary>
    public bool EnableSpring { get; set; }
    /// <summary>Spring frequency, Hz; default zero.</summary>
    public float SpringHertz { get; set; }
    /// <summary>Damping ratio; default zero, one is critical.</summary>
    public float SpringDampingRatio { get; set; }
}

/// <summary>Angular velocity coupling; supporting hinges are separate joints. Invalid parameters throw on creation.</summary>
public struct GearJointDef
{
    /// <summary>First body in the destination world.</summary>
    public Body BodyA { get; set; }
    /// <summary>Second body in the destination world; distinct from A, at least one dynamic.</summary>
    public Body BodyB { get; set; }
    /// <summary>Whether connected bodies collide; default false.</summary>
    public bool CollideConnected { get; set; }
    /// <summary>Whole-step force threshold, N; default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }
    private uint _breakForce;
    /// <summary>Whole-step COM torque threshold, N m; default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }
    private uint _breakTorque;
    /// <summary>Steps until expiry, including sleeping steps. Default zero means unlimited; negative values throw.</summary>
    public int LifetimeSteps { get; set; }
    /// <summary>Finite nonzero coupling ratio; default one. Positive gear ratios turn in opposite directions.</summary>
    public float Ratio { readonly get => ZeroDefault.Get(_ratio, 1f); set => _ratio = ZeroDefault.Set(value, 1f); }
    private uint _ratio;
}

/// <summary>Pinion A drives rack B along an axis fixed in B; supporting hinge/slider joints are separate. Invalid parameters throw on creation.</summary>
public struct RackAndPinionJointDef
{
    /// <summary>First body in the destination world.</summary>
    public Body BodyA { get; set; }
    /// <summary>Second body in the destination world; distinct from A, at least one dynamic.</summary>
    public Body BodyB { get; set; }
    /// <summary>Whether connected bodies collide; default false.</summary>
    public bool CollideConnected { get; set; }
    /// <summary>Whole-step force threshold, N; default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }
    private uint _breakForce;
    /// <summary>Whole-step COM torque threshold, N m; default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }
    private uint _breakTorque;
    /// <summary>Steps until expiry, including sleeping steps. Default zero means unlimited; negative values throw.</summary>
    public int LifetimeSteps { get; set; }
    /// <summary>Finite nonzero coupling ratio; default one. Positive gear ratios turn in opposite directions.</summary>
    public float Ratio { readonly get => ZeroDefault.Get(_ratio, 1f); set => _ratio = ZeroDefault.Set(value, 1f); }
    private uint _ratio;
    /// <summary>Rack translation direction in B's frame; finite nonzero, normalized on creation; default (1,0).</summary>
    public Vector2F LocalAxisB { readonly get => _axis.Get(new(1, 0)); set => _axis.Set(value, new(1, 0)); }
    private ZeroDefault2 _axis;
}

/// <summary>Ratio-weighted rope length conserved between two anchors and fixed world points. Invalid parameters throw on creation.</summary>
public struct PulleyJointDef
{
    /// <summary>First body in the destination world.</summary>
    public Body BodyA { get; set; }
    /// <summary>Second body in the destination world; distinct from A, at least one dynamic.</summary>
    public Body BodyB { get; set; }
    /// <summary>Whether connected bodies collide; default false.</summary>
    public bool CollideConnected { get; set; }
    /// <summary>Whole-step force threshold, N; default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }
    private uint _breakForce;
    /// <summary>Whole-step COM torque threshold, N m; default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }
    private uint _breakTorque;
    /// <summary>Steps until expiry, including sleeping steps. Default zero means unlimited; negative values throw.</summary>
    public int LifetimeSteps { get; set; }
    /// <summary>Anchor in A's origin frame, metres; default zero.</summary>
    public Vector2F LocalAnchorA { get; set; }
    /// <summary>Anchor in B's origin frame, metres; default zero.</summary>
    public Vector2F LocalAnchorB { get; set; }
    /// <summary>Finite nonzero coupling ratio; default one. Positive gear ratios turn in opposite directions.</summary>
    public float Ratio { readonly get => ZeroDefault.Get(_ratio, 1f); set => _ratio = ZeroDefault.Set(value, 1f); }
    private uint _ratio;
    /// <summary>Fixed world point for A; metres within +-2^21. Default zero.</summary>
    public Vector2D FixedPointA { get; set; }
    /// <summary>Fixed world point for B; metres within +-2^21. Default zero.</summary>
    public Vector2D FixedPointB { get; set; }
    /// <summary>Minimum ratio-weighted rope length; default -1 derives from Length or the initial pose. Must be finite, nonnegative or -1.</summary>
    public float MinLength { readonly get => ZeroDefault.Get(_minLength, -1f); set => _minLength = ZeroDefault.Set(value, -1f); }
    private uint _minLength;
    /// <summary>Maximum ratio-weighted rope length; default -1 derives from Length or the initial pose. Must be finite, nonnegative or -1, and not below the minimum.</summary>
    public float MaxLength { readonly get => ZeroDefault.Get(_maxLength, -1f); set => _maxLength = ZeroDefault.Set(value, -1f); }
    private uint _maxLength;
    /// <summary>Ratio-weighted total rope length, metres; zero derives it from the initial pose. Must be finite and nonnegative.</summary>
    public float Length { get; set; }
}

/// <summary>Anchor B follows a spline in A's origin frame. Invalid parameters throw on creation.</summary>
public struct PathJointDef
{
    /// <summary>First body in the destination world.</summary>
    public Body BodyA { get; set; }
    /// <summary>Second body in the destination world; distinct from A, at least one dynamic.</summary>
    public Body BodyB { get; set; }
    /// <summary>Whether connected bodies collide; default false.</summary>
    public bool CollideConnected { get; set; }
    /// <summary>Whole-step force threshold, N; default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }
    private uint _breakForce;
    /// <summary>Whole-step COM torque threshold, N m; default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }
    private uint _breakTorque;
    /// <summary>Steps until expiry, including sleeping steps. Default zero means unlimited; negative values throw.</summary>
    public int LifetimeSteps { get; set; }
    /// <summary>Two to 256 finite local points within +-2^21. Copied on creation; null throws.</summary>
    public Vector2F[]? Points { get; set; }
    /// <summary>Closed spline when true, otherwise endpoints clamp travel. Default false.</summary>
    public bool Loop { get; set; }
    /// <summary>Follower anchor in B's origin frame, metres; default zero.</summary>
    public Vector2F LocalAnchorB { get; set; }
    /// <summary>Align B's local x axis to the tangent when true; default false means free rotation.</summary>
    public bool AlignTangent { get; set; }
    /// <summary>Enable path speed drive. Default false.</summary>
    public bool EnableMotor { get; set; }
    /// <summary>Target speed along the tangent, metres/s; finite, default zero.</summary>
    public float MotorSpeed { get; set; }
    /// <summary>Finite nonnegative path motor force cap, N; default zero.</summary>
    public float MaxMotorForce { get; set; }
}

/// <summary>Planar three-degree-of-freedom constraint in a frame fixed to A. Invalid parameters throw on creation.</summary>
public struct ConfigurableJointDef
{
    /// <summary>First body in the destination world.</summary>
    public Body BodyA { get; set; }
    /// <summary>Second body in the destination world; distinct from A, at least one dynamic.</summary>
    public Body BodyB { get; set; }
    /// <summary>Whether connected bodies collide; default false.</summary>
    public bool CollideConnected { get; set; }
    /// <summary>Whole-step force threshold, N; default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }
    private uint _breakForce;
    /// <summary>Whole-step COM torque threshold, N m; default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }
    private uint _breakTorque;
    /// <summary>Steps until expiry, including sleeping steps. Default zero means unlimited; negative values throw.</summary>
    public int LifetimeSteps { get; set; }
    /// <summary>Anchor in A's origin frame, metres; default zero.</summary>
    public Vector2F LocalAnchorA { get; set; }
    /// <summary>Anchor in B's origin frame, metres; default zero.</summary>
    public Vector2F LocalAnchorB { get; set; }
    /// <summary>Frame angle in A, radians; finite, default zero.</summary>
    public float LocalFrameAngleA { get; set; }
    /// <summary>B minus A rotation at zero joint angle, radians; finite, default zero.</summary>
    public float ReferenceAngle { get; set; }
    /// <summary>Translation along frame x; default locked at zero.</summary>
    public JointAxisSettings X { get; set; }
    /// <summary>Translation along frame y; default locked at zero.</summary>
    public JointAxisSettings Y { get; set; }
    /// <summary>Relative rotation; default locked at zero.</summary>
    public JointAxisSettings Angle { get; set; }
}

