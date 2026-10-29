using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>The kinds of 3D joint.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public enum JointType3D : byte
{
    /// <summary>Locks position and rotation.</summary>
    Fixed = 1,

    /// <summary>Ball and socket: the two anchors stay together, rotation is free.</summary>
    Point = 2,

    /// <summary>Keeps two anchors at a distance (rigid, or a spring within a range).</summary>
    Distance = 3,

    /// <summary>Rotation about one axis, with limit, motor and spring.</summary>
    Hinge = 4,

    /// <summary>Translation along one axis with rotation locked, with limit, motor and spring.</summary>
    Slider = 5,

    /// <summary>Ball and socket with a cone limit on the angle between two axes.</summary>
    Cone = 6,

    /// <summary>Ball and socket with a twist range and an elliptical swing cone: the joint of a limb.</summary>
    SwingTwist = 7,

    /// <summary>Each of the six degrees of freedom locked, limited or free, with motors and springs.</summary>
    SixDof = 8,

    /// <summary>One body's anchor follows a curve attached to the other.</summary>
    Path = 9,

    /// <summary>Two rotating bodies turn at a fixed ratio.</summary>
    Gear = 10,

    /// <summary>A rotating body drives a sliding one at a fixed ratio.</summary>
    RackAndPinion = 11,

    /// <summary>A rope over two fixed points: the two lengths stay in a fixed ratio.</summary>
    Pulley = 12,
}

/// <summary>What a six-degrees-of-freedom joint does about one axis.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public enum JointAxisMode : byte
{
    /// <summary>The axis is held (the default).</summary>
    Locked = 0,

    /// <summary>The axis moves within limits.</summary>
    Limited = 1,

    /// <summary>The axis is unconstrained.</summary>
    Free = 2,
}

/// <summary>How a path joint holds the follower's orientation.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public enum PathRotation : byte
{
    /// <summary>The follower turns freely.</summary>
    Free = 0,

    /// <summary>The follower's frame x axis stays along the path tangent; it may spin about it.</summary>
    AlignTangent = 1,

    /// <summary>The follower's frame is held to the path frame (x along the tangent, y towards the up vector).</summary>
    Full = 2,
}

/// <summary>Axes of a six-degrees-of-freedom joint, as flags: translation along x, y, z of the joint frame, rotation about them.</summary>
[Flags]
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public enum JointAxes : byte
{
    /// <summary>None.</summary>
    None = 0,

    /// <summary>Translation along x.</summary>
    LinearX = 1,

    /// <summary>Translation along y.</summary>
    LinearY = 2,

    /// <summary>Translation along z.</summary>
    LinearZ = 4,

    /// <summary>Rotation about x (twist).</summary>
    AngularX = 8,

    /// <summary>Rotation about y (swing).</summary>
    AngularY = 16,

    /// <summary>Rotation about z (swing).</summary>
    AngularZ = 32,
}

/// <summary>A joint's frame on one body: a point and an orientation in the body's own frame (measured from the body origin). The frame's x axis is the joint's main axis.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct JointFrame3D
{
    /// <summary>Creates a frame.</summary>
    public JointFrame3D(Vector3F position, QuaternionF rotation)
    {
        Position = position;
        _rotation = default;
        _rotation.Set(rotation, QuaternionF.Identity);
    }

    // Stored relative to the identity so that default(JointFrame3D) is the frame at the body origin with no rotation.
    private readonly ZeroDefaultQuat _rotation;

    /// <summary>Position in the body's origin frame.</summary>
    public Vector3F Position { get; }

    /// <summary>Orientation in the body's frame; any finite non-zero quaternion (normalized on creation). The zero value of the struct is the identity.</summary>
    public QuaternionF Rotation => _rotation.Get(QuaternionF.Identity);

    /// <summary>The frame at the body origin with no rotation.</summary>
    public static JointFrame3D Identity => new(default, QuaternionF.Identity);
}

/// <summary>Fields shared by every 3D joint definition; the concrete definitions add their own. Each definition is a plain struct with defaults.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public interface IJointDef3D
{
    /// <summary>The first body.</summary>
    Body BodyA { get; set; }

    /// <summary>The second body.</summary>
    Body BodyB { get; set; }
}

/// <summary>Fixed joint: locks the relative pose of the two frames. Defaults: frames at the body origins, no collision between the bodies, never breaks.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public struct FixedJointDef : IJointDef3D
{
    /// <summary>The first body.</summary>
    public Body BodyA { get; set; }

    /// <summary>The second body.</summary>
    public Body BodyB { get; set; }

    /// <summary>Frame on A.</summary>
    public JointFrame3D LocalFrameA { get; set; }

    /// <summary>Frame on B.</summary>
    public JointFrame3D LocalFrameB { get; set; }

    /// <summary>When true the bodies keep colliding with each other. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Reaction force (N) above which the joint breaks at the end of a step. Default infinity (never).</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>Reaction torque (N·m) above which the joint breaks. Default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}

/// <summary>Point (ball-and-socket) joint: the two frame positions coincide, rotation is free.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public struct PointJointDef : IJointDef3D
{
    /// <summary>The first body.</summary>
    public Body BodyA { get; set; }

    /// <summary>The second body.</summary>
    public Body BodyB { get; set; }

    /// <summary>Frame on A (only the position matters).</summary>
    public JointFrame3D LocalFrameA { get; set; }

    /// <summary>Frame on B (only the position matters).</summary>
    public JointFrame3D LocalFrameB { get; set; }

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}

/// <summary>Distance joint between the two frame positions. Rigid at <see cref="Length"/> unless the spring is on; with a spring the length may range between <see cref="MinLength"/> and <see cref="MaxLength"/>.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public struct DistanceJointDef : IJointDef3D
{
    /// <summary>The first body.</summary>
    public Body BodyA { get; set; }

    /// <summary>The second body.</summary>
    public Body BodyB { get; set; }

    /// <summary>Frame on A (only the position matters).</summary>
    public JointFrame3D LocalFrameA { get; set; }

    /// <summary>Frame on B (only the position matters).</summary>
    public JointFrame3D LocalFrameB { get; set; }

    /// <summary>Rest length, metres, at least 0.005. Default 1.</summary>
    public float Length { readonly get => ZeroDefault.Get(_length, 1f); set => _length = ZeroDefault.Set(value, 1f); }

    private uint _length;

    /// <summary>Shortest length when the limit is on or the spring is on, at least 0. Default 0.</summary>
    public float MinLength { get; set; }

    /// <summary>Longest length, at least <see cref="MinLength"/>. Default infinity.</summary>
    public float MaxLength { readonly get => ZeroDefault.Get(_maxLength, float.PositiveInfinity); set => _maxLength = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _maxLength;

    /// <summary>Enforces <see cref="MinLength"/> and <see cref="MaxLength"/> as hard limits.</summary>
    public bool EnableLimit { get; set; }

    /// <summary>Replaces the rigid rod by a spring.</summary>
    public bool EnableSpring { get; set; }

    /// <summary>Spring frequency, Hz, at least 0. Default 0.</summary>
    public float SpringHertz { get; set; }

    /// <summary>Spring damping ratio. Default 0.5.</summary>
    public float SpringDampingRatio { readonly get => ZeroDefault.Get(_springDampingRatio, 0.5f); set => _springDampingRatio = ZeroDefault.Set(value, 0.5f); }

    private uint _springDampingRatio;

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}

/// <summary>Hinge joint about the x axis of the frames. The angle is zero when the frames' y axes agree.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public struct HingeJointDef : IJointDef3D
{
    /// <summary>The first body.</summary>
    public Body BodyA { get; set; }

    /// <summary>The second body.</summary>
    public Body BodyB { get; set; }

    /// <summary>Frame on A.</summary>
    public JointFrame3D LocalFrameA { get; set; }

    /// <summary>Frame on B.</summary>
    public JointFrame3D LocalFrameB { get; set; }

    /// <summary>Enforces the angle limits.</summary>
    public bool EnableLimit { get; set; }

    /// <summary>Lower angle, radians within -pi..pi. Default -pi.</summary>
    public float LowerAngle { readonly get => ZeroDefault.Get(_lowerAngle, -3.1415927f); set => _lowerAngle = ZeroDefault.Set(value, -3.1415927f); }

    private uint _lowerAngle;

    /// <summary>Upper angle, radians. Default pi.</summary>
    public float UpperAngle { readonly get => ZeroDefault.Get(_upperAngle, 3.1415927f); set => _upperAngle = ZeroDefault.Set(value, 3.1415927f); }

    private uint _upperAngle;

    /// <summary>Turns the motor on.</summary>
    public bool EnableMotor { get; set; }

    /// <summary>Target relative angular speed about the axis, rad/s.</summary>
    public float MotorSpeed { get; set; }

    /// <summary>Largest motor torque, N·m, at least 0. Default 0.</summary>
    public float MaxMotorTorque { get; set; }

    /// <summary>Turns on a torsional spring towards angle 0.</summary>
    public bool EnableSpring { get; set; }

    /// <summary>Spring frequency, Hz.</summary>
    public float SpringHertz { get; set; }

    /// <summary>Spring damping ratio. Default 0.5.</summary>
    public float SpringDampingRatio { readonly get => ZeroDefault.Get(_springDampingRatio, 0.5f); set => _springDampingRatio = ZeroDefault.Set(value, 0.5f); }

    private uint _springDampingRatio;

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}

/// <summary>Slider joint along the x axis of the frames, rotation locked. The translation is zero when the frame positions coincide.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public struct SliderJointDef : IJointDef3D
{
    /// <summary>The first body.</summary>
    public Body BodyA { get; set; }

    /// <summary>The second body.</summary>
    public Body BodyB { get; set; }

    /// <summary>Frame on A.</summary>
    public JointFrame3D LocalFrameA { get; set; }

    /// <summary>Frame on B.</summary>
    public JointFrame3D LocalFrameB { get; set; }

    /// <summary>Enforces the translation limits.</summary>
    public bool EnableLimit { get; set; }

    /// <summary>Lower translation, metres.</summary>
    public float LowerTranslation { get; set; }

    /// <summary>Upper translation, metres.</summary>
    public float UpperTranslation { get; set; }

    /// <summary>Turns the motor on.</summary>
    public bool EnableMotor { get; set; }

    /// <summary>Target relative speed along the axis, m/s.</summary>
    public float MotorSpeed { get; set; }

    /// <summary>Largest motor force, N, at least 0.</summary>
    public float MaxMotorForce { get; set; }

    /// <summary>Turns on a spring towards translation 0.</summary>
    public bool EnableSpring { get; set; }

    /// <summary>Spring frequency, Hz.</summary>
    public float SpringHertz { get; set; }

    /// <summary>Spring damping ratio. Default 0.5.</summary>
    public float SpringDampingRatio { readonly get => ZeroDefault.Get(_springDampingRatio, 0.5f); set => _springDampingRatio = ZeroDefault.Set(value, 0.5f); }

    private uint _springDampingRatio;

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}

/// <summary>Cone joint: a point joint that keeps the two frame x axes within <see cref="HalfConeAngle"/> of each other.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public struct ConeJointDef : IJointDef3D
{
    /// <summary>The first body.</summary>
    public Body BodyA { get; set; }

    /// <summary>The second body.</summary>
    public Body BodyB { get; set; }

    /// <summary>Frame on A.</summary>
    public JointFrame3D LocalFrameA { get; set; }

    /// <summary>Frame on B.</summary>
    public JointFrame3D LocalFrameB { get; set; }

    /// <summary>Largest angle between the two x axes, radians within (0, pi]. Default pi/4.</summary>
    public float HalfConeAngle { readonly get => ZeroDefault.Get(_halfConeAngle, 0.7853982f); set => _halfConeAngle = ZeroDefault.Set(value, 0.7853982f); }

    private uint _halfConeAngle;

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}

/// <summary>Swing-twist joint (a ball-and-socket for limbs): twist about the frame x axis within a range, swing of that axis inside an elliptical cone.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public struct SwingTwistJointDef : IJointDef3D
{
    /// <summary>The first body.</summary>
    public Body BodyA { get; set; }

    /// <summary>The second body.</summary>
    public Body BodyB { get; set; }

    /// <summary>Frame on A.</summary>
    public JointFrame3D LocalFrameA { get; set; }

    /// <summary>Frame on B.</summary>
    public JointFrame3D LocalFrameB { get; set; }

    /// <summary>Lower twist angle, radians within -pi..pi. Default -pi/4.</summary>
    public float LowerTwist { readonly get => ZeroDefault.Get(_lowerTwist, -0.7853982f); set => _lowerTwist = ZeroDefault.Set(value, -0.7853982f); }

    private uint _lowerTwist;

    /// <summary>Upper twist angle. Default pi/4.</summary>
    public float UpperTwist { readonly get => ZeroDefault.Get(_upperTwist, 0.7853982f); set => _upperTwist = ZeroDefault.Set(value, 0.7853982f); }

    private uint _upperTwist;

    /// <summary>Half angle of the swing about the frame y axis, radians within (0, pi]. Default pi/4.</summary>
    public float SwingYHalfAngle { readonly get => ZeroDefault.Get(_swingYHalfAngle, 0.7853982f); set => _swingYHalfAngle = ZeroDefault.Set(value, 0.7853982f); }

    private uint _swingYHalfAngle;

    /// <summary>Half angle of the swing about the frame z axis, radians within (0, pi]. Default pi/4.</summary>
    public float SwingZHalfAngle { readonly get => ZeroDefault.Get(_swingZHalfAngle, 0.7853982f); set => _swingZHalfAngle = ZeroDefault.Set(value, 0.7853982f); }

    private uint _swingZHalfAngle;

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}

/// <summary>
/// Six-degrees-of-freedom joint. Axes are those of frame A: translation along x, y, z and rotation about them (x is the twist, y and z the swing). Each axis is locked, limited or free;
/// limited rotation about y and z bounds the swing components independently (accurate for moderate swing). Defaults: everything locked, no motors or springs.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public struct SixDofJointDef : IJointDef3D
{
    /// <summary>The first body.</summary>
    public Body BodyA { get; set; }

    /// <summary>The second body.</summary>
    public Body BodyB { get; set; }

    /// <summary>Frame on A.</summary>
    public JointFrame3D LocalFrameA { get; set; }

    /// <summary>Frame on B.</summary>
    public JointFrame3D LocalFrameB { get; set; }

    /// <summary>Mode of translation along x.</summary>
    public JointAxisMode LinearX { get; set; }

    /// <summary>Mode of translation along y.</summary>
    public JointAxisMode LinearY { get; set; }

    /// <summary>Mode of translation along z.</summary>
    public JointAxisMode LinearZ { get; set; }

    /// <summary>Mode of rotation about x (twist).</summary>
    public JointAxisMode AngularX { get; set; }

    /// <summary>Mode of rotation about y (swing).</summary>
    public JointAxisMode AngularY { get; set; }

    /// <summary>Mode of rotation about z (swing).</summary>
    public JointAxisMode AngularZ { get; set; }

    /// <summary>Lower translation limits, metres (used by limited axes).</summary>
    public Vector3F LowerLinearLimit { get; set; }

    /// <summary>Upper translation limits, metres.</summary>
    public Vector3F UpperLinearLimit { get; set; }

    /// <summary>Lower rotation limits, radians within -pi..pi.</summary>
    public Vector3F LowerAngularLimit { get; set; }

    /// <summary>Upper rotation limits, radians.</summary>
    public Vector3F UpperAngularLimit { get; set; }

    /// <summary>Axes that have a motor.</summary>
    public JointAxes MotorAxes { get; set; }

    /// <summary>Target relative translation speeds along x, y, z, m/s.</summary>
    public Vector3F LinearMotorSpeed { get; set; }

    /// <summary>Largest motor forces along x, y, z, N.</summary>
    public Vector3F MaxLinearMotorForce { get; set; }

    /// <summary>Target relative angular speeds about x, y, z, rad/s.</summary>
    public Vector3F AngularMotorSpeed { get; set; }

    /// <summary>Largest motor torques about x, y, z, N·m.</summary>
    public Vector3F MaxAngularMotorTorque { get; set; }

    /// <summary>Axes that have a spring towards zero translation or angle.</summary>
    public JointAxes SpringAxes { get; set; }

    /// <summary>Translation spring frequencies, Hz.</summary>
    public Vector3F SpringHertz { get; set; }

    /// <summary>Translation spring damping ratios. Default 0.5.</summary>
    public Vector3F SpringDampingRatio { readonly get => _springDampingRatio.Get(new Vector3F(0.5f, 0.5f, 0.5f)); set => _springDampingRatio.Set(value, new Vector3F(0.5f, 0.5f, 0.5f)); }

    private ZeroDefault3 _springDampingRatio;

    /// <summary>Rotation spring frequencies, Hz.</summary>
    public Vector3F AngularSpringHertz { get; set; }

    /// <summary>Rotation spring damping ratios. Default 0.5.</summary>
    public Vector3F AngularSpringDampingRatio { readonly get => _angularSpringDampingRatio.Get(new Vector3F(0.5f, 0.5f, 0.5f)); set => _angularSpringDampingRatio.Set(value, new Vector3F(0.5f, 0.5f, 0.5f)); }

    private ZeroDefault3 _angularSpringDampingRatio;

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}

/// <summary>
/// Path joint: <see cref="LocalFrameB"/>'s position follows a smooth curve (a Catmull-Rom spline through <see cref="Points"/>) fixed to body A. Points are in A's origin frame. With a motor the follower is driven along the path;
/// an open path has hard ends.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public struct PathJointDef : IJointDef3D
{
    /// <summary>The body that carries the path.</summary>
    public Body BodyA { get; set; }

    /// <summary>The follower.</summary>
    public Body BodyB { get; set; }

    /// <summary>Frame on the follower: the position rides the path, the x axis is its forward direction.</summary>
    public JointFrame3D LocalFrameB { get; set; }

    /// <summary>The path's points in A's origin frame: 2 to 256, all finite and within ±2^21. Copied on creation.</summary>
    public Vector3F[]? Points { get; set; }

    /// <summary>Closes the path into a loop. Default false.</summary>
    public bool Loop { get; set; }

    /// <summary>Direction (in A's frame) that the path frame's y axis leans towards. Default (0, 1, 0).</summary>
    public Vector3F Up { readonly get => _up.Get(new Vector3F(0f, 1f, 0f)); set => _up.Set(value, new Vector3F(0f, 1f, 0f)); }

    private ZeroDefault3 _up;

    /// <summary>How the follower's orientation is held. Default free.</summary>
    public PathRotation Rotation { get; set; }

    /// <summary>Turns the motor on.</summary>
    public bool EnableMotor { get; set; }

    /// <summary>Target speed along the path, m/s.</summary>
    public float MotorSpeed { get; set; }

    /// <summary>Largest motor force, N.</summary>
    public float MaxMotorForce { get; set; }

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}

/// <summary>Gear joint: the angular speeds about the two frames' x axes satisfy <c>ωA·axisA + Ratio·ωB·axisB = 0</c>. Joint frames' rotations give the axes; positions are ignored.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public struct GearJointDef : IJointDef3D
{
    /// <summary>The first gear.</summary>
    public Body BodyA { get; set; }

    /// <summary>The second gear.</summary>
    public Body BodyB { get; set; }

    /// <summary>Frame on A (its x axis is the gear axis).</summary>
    public JointFrame3D LocalFrameA { get; set; }

    /// <summary>Frame on B.</summary>
    public JointFrame3D LocalFrameB { get; set; }

    /// <summary>Speed ratio: B's angular speed is -1/Ratio times A's. Finite, absolute value within 1e-6..1e6. Default 1.</summary>
    public float Ratio { readonly get => ZeroDefault.Get(_ratio, 1f); set => _ratio = ZeroDefault.Set(value, 1f); }

    private uint _ratio;

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}

/// <summary>Rack-and-pinion joint: <c>Ratio·ωA·axisA = vB·axisB</c> with A the pinion (rotating about its frame x axis) and B the rack (moving along its frame x axis); Ratio is metres per radian.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public struct RackAndPinionJointDef : IJointDef3D
{
    /// <summary>The pinion.</summary>
    public Body BodyA { get; set; }

    /// <summary>The rack.</summary>
    public Body BodyB { get; set; }

    /// <summary>Frame on the pinion (its x axis is the rotation axis).</summary>
    public JointFrame3D LocalFrameA { get; set; }

    /// <summary>Frame on the rack (its x axis is the sliding direction).</summary>
    public JointFrame3D LocalFrameB { get; set; }

    /// <summary>Rack travel per radian of pinion rotation, metres; finite, absolute value within 1e-6..1e6. Default 1.</summary>
    public float Ratio { readonly get => ZeroDefault.Get(_ratio, 1f); set => _ratio = ZeroDefault.Set(value, 1f); }

    private uint _ratio;

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}

/// <summary>Pulley joint: a rope from body A's anchor up to <see cref="FixedPointA"/>, over to <see cref="FixedPointB"/> and down to B's anchor, with <c>lengthA + Ratio·lengthB</c> held between the limits.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public struct PulleyJointDef : IJointDef3D
{
    /// <summary>The first body.</summary>
    public Body BodyA { get; set; }

    /// <summary>The second body.</summary>
    public Body BodyB { get; set; }

    /// <summary>Anchor on A (only the position matters).</summary>
    public JointFrame3D LocalFrameA { get; set; }

    /// <summary>Anchor on B.</summary>
    public JointFrame3D LocalFrameB { get; set; }

    /// <summary>The fixed point above A, world coordinates within ±2^21.</summary>
    public Vector3D FixedPointA { get; set; }

    /// <summary>The fixed point above B.</summary>
    public Vector3D FixedPointB { get; set; }

    /// <summary>Length ratio, within 1e-6..1e6. Default 1.</summary>
    public float Ratio { readonly get => ZeroDefault.Get(_ratio, 1f); set => _ratio = ZeroDefault.Set(value, 1f); }

    private uint _ratio;

    /// <summary>Smallest value of <c>lengthA + Ratio·lengthB</c>, at least 0; a negative value (the default for both limits) means the current value, making the rope rigid.</summary>
    public float MinLength { readonly get => ZeroDefault.Get(_minLength, -1f); set => _minLength = ZeroDefault.Set(value, -1f); }

    private uint _minLength;

    /// <summary>Largest value of <c>lengthA + Ratio·lengthB</c>; negative means the current value.</summary>
    public float MaxLength { readonly get => ZeroDefault.Get(_maxLength, -1f); set => _maxLength = ZeroDefault.Set(value, -1f); }

    private uint _maxLength;

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { readonly get => ZeroDefault.Get(_breakForce, float.PositiveInfinity); set => _breakForce = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakForce;

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { readonly get => ZeroDefault.Get(_breakTorque, float.PositiveInfinity); set => _breakTorque = ZeroDefault.Set(value, float.PositiveInfinity); }

    private uint _breakTorque;
}
