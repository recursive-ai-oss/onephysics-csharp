using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>The kinds of 3D joint.</summary>
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
public readonly struct JointFrame3D
{
    /// <summary>Creates a frame.</summary>
    public JointFrame3D(Vector3F position, QuaternionF rotation)
    {
        Position = position;
        Rotation = rotation;
    }

    /// <summary>Position in the body's origin frame.</summary>
    public Vector3F Position { get; }

    /// <summary>Orientation in the body's frame; any finite non-zero quaternion (normalized on creation).</summary>
    public QuaternionF Rotation { get; }

    /// <summary>The frame at the body origin with no rotation.</summary>
    public static JointFrame3D Identity => new(default, QuaternionF.Identity);
}

/// <summary>Fields shared by every 3D joint definition; the concrete definitions add their own. Each definition is a plain struct with defaults.</summary>
public interface IJointDef3D
{
    /// <summary>The first body.</summary>
    Body BodyA { get; set; }

    /// <summary>The second body.</summary>
    Body BodyB { get; set; }
}

/// <summary>Fixed joint: locks the relative pose of the two frames. Defaults: frames at the body origins, no collision between the bodies, never breaks.</summary>
public struct FixedJointDef : IJointDef3D
{
    /// <summary>Creates a definition with the defaults.</summary>
    public FixedJointDef()
    {
        LocalFrameA = LocalFrameB = JointFrame3D.Identity;
        BreakForce = BreakTorque = float.PositiveInfinity;
    }

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
    public float BreakForce { get; set; }

    /// <summary>Reaction torque (N·m) above which the joint breaks. Default infinity.</summary>
    public float BreakTorque { get; set; }
}

/// <summary>Point (ball-and-socket) joint: the two frame positions coincide, rotation is free.</summary>
public struct PointJointDef : IJointDef3D
{
    /// <summary>Creates a definition with the defaults.</summary>
    public PointJointDef()
    {
        LocalFrameA = LocalFrameB = JointFrame3D.Identity;
        BreakForce = BreakTorque = float.PositiveInfinity;
    }

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
    public float BreakForce { get; set; }

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { get; set; }
}

/// <summary>Distance joint between the two frame positions. Rigid at <see cref="Length"/> unless the spring is on; with a spring the length may range between <see cref="MinLength"/> and <see cref="MaxLength"/>.</summary>
public struct DistanceJointDef : IJointDef3D
{
    /// <summary>Creates a definition with the defaults.</summary>
    public DistanceJointDef()
    {
        LocalFrameA = LocalFrameB = JointFrame3D.Identity;
        Length = 1f;
        MaxLength = float.PositiveInfinity;
        SpringDampingRatio = 0.5f;
        BreakForce = BreakTorque = float.PositiveInfinity;
    }

    /// <summary>The first body.</summary>
    public Body BodyA { get; set; }

    /// <summary>The second body.</summary>
    public Body BodyB { get; set; }

    /// <summary>Frame on A (only the position matters).</summary>
    public JointFrame3D LocalFrameA { get; set; }

    /// <summary>Frame on B (only the position matters).</summary>
    public JointFrame3D LocalFrameB { get; set; }

    /// <summary>Rest length, metres, at least 0.005. Default 1.</summary>
    public float Length { get; set; }

    /// <summary>Shortest length when the limit is on or the spring is on, at least 0. Default 0.</summary>
    public float MinLength { get; set; }

    /// <summary>Longest length, at least <see cref="MinLength"/>. Default infinity.</summary>
    public float MaxLength { get; set; }

    /// <summary>Enforces <see cref="MinLength"/> and <see cref="MaxLength"/> as hard limits.</summary>
    public bool EnableLimit { get; set; }

    /// <summary>Replaces the rigid rod by a spring.</summary>
    public bool EnableSpring { get; set; }

    /// <summary>Spring frequency, Hz, at least 0. Default 0.</summary>
    public float SpringHertz { get; set; }

    /// <summary>Spring damping ratio. Default 0.5.</summary>
    public float SpringDampingRatio { get; set; }

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { get; set; }

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { get; set; }
}

/// <summary>Hinge joint about the x axis of the frames. The angle is zero when the frames' y axes agree.</summary>
public struct HingeJointDef : IJointDef3D
{
    /// <summary>Creates a definition with the defaults.</summary>
    public HingeJointDef()
    {
        LocalFrameA = LocalFrameB = JointFrame3D.Identity;
        LowerAngle = -3.1415927f; UpperAngle = 3.1415927f;
        SpringDampingRatio = 0.5f;
        BreakForce = BreakTorque = float.PositiveInfinity;
    }

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
    public float LowerAngle { get; set; }

    /// <summary>Upper angle, radians. Default pi.</summary>
    public float UpperAngle { get; set; }

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
    public float SpringDampingRatio { get; set; }

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { get; set; }

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { get; set; }
}

/// <summary>Slider joint along the x axis of the frames, rotation locked. The translation is zero when the frame positions coincide.</summary>
public struct SliderJointDef : IJointDef3D
{
    /// <summary>Creates a definition with the defaults.</summary>
    public SliderJointDef()
    {
        LocalFrameA = LocalFrameB = JointFrame3D.Identity;
        SpringDampingRatio = 0.5f;
        BreakForce = BreakTorque = float.PositiveInfinity;
    }

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
    public float SpringDampingRatio { get; set; }

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { get; set; }

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { get; set; }
}

/// <summary>Cone joint: a point joint that keeps the two frame x axes within <see cref="HalfConeAngle"/> of each other.</summary>
public struct ConeJointDef : IJointDef3D
{
    /// <summary>Creates a definition with the defaults.</summary>
    public ConeJointDef()
    {
        LocalFrameA = LocalFrameB = JointFrame3D.Identity;
        HalfConeAngle = 0.7853982f;
        BreakForce = BreakTorque = float.PositiveInfinity;
    }

    /// <summary>The first body.</summary>
    public Body BodyA { get; set; }

    /// <summary>The second body.</summary>
    public Body BodyB { get; set; }

    /// <summary>Frame on A.</summary>
    public JointFrame3D LocalFrameA { get; set; }

    /// <summary>Frame on B.</summary>
    public JointFrame3D LocalFrameB { get; set; }

    /// <summary>Largest angle between the two x axes, radians within (0, pi]. Default pi/4.</summary>
    public float HalfConeAngle { get; set; }

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { get; set; }

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { get; set; }
}

/// <summary>Swing-twist joint (a ball-and-socket for limbs): twist about the frame x axis within a range, swing of that axis inside an elliptical cone.</summary>
public struct SwingTwistJointDef : IJointDef3D
{
    /// <summary>Creates a definition with the defaults.</summary>
    public SwingTwistJointDef()
    {
        LocalFrameA = LocalFrameB = JointFrame3D.Identity;
        LowerTwist = -0.7853982f; UpperTwist = 0.7853982f;
        SwingYHalfAngle = SwingZHalfAngle = 0.7853982f;
        BreakForce = BreakTorque = float.PositiveInfinity;
    }

    /// <summary>The first body.</summary>
    public Body BodyA { get; set; }

    /// <summary>The second body.</summary>
    public Body BodyB { get; set; }

    /// <summary>Frame on A.</summary>
    public JointFrame3D LocalFrameA { get; set; }

    /// <summary>Frame on B.</summary>
    public JointFrame3D LocalFrameB { get; set; }

    /// <summary>Lower twist angle, radians within -pi..pi. Default -pi/4.</summary>
    public float LowerTwist { get; set; }

    /// <summary>Upper twist angle. Default pi/4.</summary>
    public float UpperTwist { get; set; }

    /// <summary>Half angle of the swing about the frame y axis, radians within (0, pi]. Default pi/4.</summary>
    public float SwingYHalfAngle { get; set; }

    /// <summary>Half angle of the swing about the frame z axis, radians within (0, pi]. Default pi/4.</summary>
    public float SwingZHalfAngle { get; set; }

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { get; set; }

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { get; set; }
}

/// <summary>
/// Six-degrees-of-freedom joint. Axes are those of frame A: translation along x, y, z and rotation about them (x is the twist, y and z the swing). Each axis is locked, limited or free;
/// limited rotation about y and z bounds the swing components independently (accurate for moderate swing). Defaults: everything locked, no motors or springs.
/// </summary>
public struct SixDofJointDef : IJointDef3D
{
    /// <summary>Creates a definition with the defaults.</summary>
    public SixDofJointDef()
    {
        LocalFrameA = LocalFrameB = JointFrame3D.Identity;
        SpringDampingRatio = new Vector3F(0.5f, 0.5f, 0.5f);
        AngularSpringDampingRatio = new Vector3F(0.5f, 0.5f, 0.5f);
        BreakForce = BreakTorque = float.PositiveInfinity;
    }

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
    public Vector3F SpringDampingRatio { get; set; }

    /// <summary>Rotation spring frequencies, Hz.</summary>
    public Vector3F AngularSpringHertz { get; set; }

    /// <summary>Rotation spring damping ratios. Default 0.5.</summary>
    public Vector3F AngularSpringDampingRatio { get; set; }

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { get; set; }

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { get; set; }
}

/// <summary>
/// Path joint: <see cref="LocalFrameB"/>'s position follows a smooth curve (a Catmull-Rom spline through <see cref="Points"/>) fixed to body A. Points are in A's origin frame. With a motor the follower is driven along the path;
/// an open path has hard ends.
/// </summary>
public struct PathJointDef : IJointDef3D
{
    /// <summary>Creates a definition with the defaults.</summary>
    public PathJointDef()
    {
        LocalFrameB = JointFrame3D.Identity;
        Up = new Vector3F(0f, 1f, 0f);
        BreakForce = BreakTorque = float.PositiveInfinity;
    }

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
    public Vector3F Up { get; set; }

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
    public float BreakForce { get; set; }

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { get; set; }
}

/// <summary>Gear joint: the angular speeds about the two frames' x axes satisfy <c>ωA·axisA + Ratio·ωB·axisB = 0</c>. Joint frames' rotations give the axes; positions are ignored.</summary>
public struct GearJointDef : IJointDef3D
{
    /// <summary>Creates a definition with the defaults.</summary>
    public GearJointDef()
    {
        LocalFrameA = LocalFrameB = JointFrame3D.Identity;
        Ratio = 1f;
        BreakForce = BreakTorque = float.PositiveInfinity;
    }

    /// <summary>The first gear.</summary>
    public Body BodyA { get; set; }

    /// <summary>The second gear.</summary>
    public Body BodyB { get; set; }

    /// <summary>Frame on A (its x axis is the gear axis).</summary>
    public JointFrame3D LocalFrameA { get; set; }

    /// <summary>Frame on B.</summary>
    public JointFrame3D LocalFrameB { get; set; }

    /// <summary>Speed ratio: B's angular speed is -1/Ratio times A's. Finite, absolute value within 1e-6..1e6. Default 1.</summary>
    public float Ratio { get; set; }

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { get; set; }

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { get; set; }
}

/// <summary>Rack-and-pinion joint: <c>Ratio·ωA·axisA = vB·axisB</c> with A the pinion (rotating about its frame x axis) and B the rack (moving along its frame x axis); Ratio is metres per radian.</summary>
public struct RackAndPinionJointDef : IJointDef3D
{
    /// <summary>Creates a definition with the defaults.</summary>
    public RackAndPinionJointDef()
    {
        LocalFrameA = LocalFrameB = JointFrame3D.Identity;
        Ratio = 1f;
        BreakForce = BreakTorque = float.PositiveInfinity;
    }

    /// <summary>The pinion.</summary>
    public Body BodyA { get; set; }

    /// <summary>The rack.</summary>
    public Body BodyB { get; set; }

    /// <summary>Frame on the pinion (its x axis is the rotation axis).</summary>
    public JointFrame3D LocalFrameA { get; set; }

    /// <summary>Frame on the rack (its x axis is the sliding direction).</summary>
    public JointFrame3D LocalFrameB { get; set; }

    /// <summary>Rack travel per radian of pinion rotation, metres; finite, absolute value within 1e-6..1e6. Default 1.</summary>
    public float Ratio { get; set; }

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { get; set; }

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { get; set; }
}

/// <summary>Pulley joint: a rope from body A's anchor up to <see cref="FixedPointA"/>, over to <see cref="FixedPointB"/> and down to B's anchor, with <c>lengthA + Ratio·lengthB</c> held between the limits.</summary>
public struct PulleyJointDef : IJointDef3D
{
    /// <summary>Creates a definition with the defaults.</summary>
    public PulleyJointDef()
    {
        LocalFrameA = LocalFrameB = JointFrame3D.Identity;
        Ratio = 1f;
        MinLength = MaxLength = -1f;
        BreakForce = BreakTorque = float.PositiveInfinity;
    }

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
    public float Ratio { get; set; }

    /// <summary>Smallest value of <c>lengthA + Ratio·lengthB</c>, at least 0; a negative value (the default for both limits) means the current value, making the rope rigid.</summary>
    public float MinLength { get; set; }

    /// <summary>Largest value of <c>lengthA + Ratio·lengthB</c>; negative means the current value.</summary>
    public float MaxLength { get; set; }

    /// <summary>When true the bodies keep colliding. Default false.</summary>
    public bool CollideConnected { get; set; }

    /// <summary>Break threshold, N. Default infinity.</summary>
    public float BreakForce { get; set; }

    /// <summary>Break threshold, N·m. Default infinity.</summary>
    public float BreakTorque { get; set; }
}
