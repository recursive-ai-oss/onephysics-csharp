using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// A typed view of a revolute (hinge) joint: a point on each body stays together, rotation is free until limited or driven. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddHinge, RevoluteJointDef.
/// </summary>
public readonly struct HingeJoint : IEquatable<HingeJoint>
{
    private readonly Joint _joint;

    internal HingeJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(HingeJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator HingeJoint(Joint joint)
    {
        if (joint.Type != JointType2D.Revolute) { throw new InvalidJointException("A " + joint.Type + " joint is not a Revolute joint (HingeJoint).", nameof(joint)); }
        return new HingeJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType2D.Revolute); fails when the joint is destroyed.</summary>
    public JointType2D Type => _joint.Type;

    /// <summary>True while the joint exists.</summary>
    public bool IsValid => _joint.IsValid;

    /// <summary>The first body.</summary>
    public Body BodyA => _joint.BodyA;

    /// <summary>The second body.</summary>
    public Body BodyB => _joint.BodyB;

    /// <summary>Destroys the joint. Every copy of this handle becomes stale; no break event is raised.</summary>
    public void Destroy() => _joint.Destroy();

    /// <summary>Raised after the step in which this joint broke; see <see cref="Joint.OnJointBreak"/>.</summary>
    public event Action<JointBreak> OnJointBreak { add { _joint.OnJointBreak += value; } remove { _joint.OnJointBreak -= value; } }

    /// <summary>Whether the two bodies collide with each other (fixed at creation).</summary>
    public bool CollideConnected => _joint.CollideConnected;

    /// <summary>Reaction force above which the joint breaks at the end of a step, newtons; infinity (the default) never breaks.</summary>
    public float BreakForce { get => _joint.BreakForce; set => _joint.BreakForce = value; }

    /// <summary>Reaction torque above which the joint breaks, N·m; infinity never breaks.</summary>
    public float BreakTorque { get => _joint.BreakTorque; set => _joint.BreakTorque = value; }

    /// <summary>The magnitude of the joint's linear reaction force at the end of the last step it was solved, newtons.</summary>
    public float ReactionForce => _joint.ReactionForce;

    /// <summary>The magnitude of the joint's reaction torque at the end of the last step it was solved, N·m.</summary>
    public float ReactionTorque => _joint.ReactionTorque;

    /// <summary>Whether the limit is on.</summary>
    public bool EnableLimit { get => _joint.EnableLimit; set => _joint.EnableLimit = value; }

    /// <summary>Lower angle limit, radians.</summary>
    public float LowerAngle => _joint.LowerAngle;

    /// <summary>Upper angle limit, radians.</summary>
    public float UpperAngle => _joint.UpperAngle;

    /// <summary>Sets both limits (radians within -pi..pi) without turning the limit on; lower at most upper.
    /// <exception cref="InvalidJointException">The limits are the wrong way round.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or out of range.</exception></summary>
    public void SetLimits(float lower, float upper) => _joint.SetLimits(lower, upper);

    /// <summary>Whether the motor is on.</summary>
    public bool EnableMotor { get => _joint.EnableMotor; set => _joint.EnableMotor = value; }

    /// <summary>Target speed of the motor: radians per second for a rotational motor, metres per second for a linear one.</summary>
    public float MotorSpeed { get => _joint.MotorSpeed; set => _joint.MotorSpeed = value; }

    /// <summary>Largest motor torque, N·m, at least 0.</summary>
    public float MaxMotorTorque { get => _joint.MaxMotorTorque; set => _joint.MaxMotorTorque = value; }

    /// <summary>Whether the spring is on.</summary>
    public bool EnableSpring { get => _joint.EnableSpring; set => _joint.EnableSpring = value; }

    /// <summary>Spring frequency in Hz, at least 0 (0 means no spring force).</summary>
    public float SpringHertz { get => _joint.SpringHertz; set => _joint.SpringHertz = value; }

    /// <summary>Spring damping ratio, at least 0 (1 is critical).</summary>
    public float SpringDampingRatio { get => _joint.SpringDampingRatio; set => _joint.SpringDampingRatio = value; }

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(HingeJoint other) => _joint.Equals(other._joint);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is HingeJoint v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => _joint.GetHashCode();

    /// <summary>Handle equality.</summary>
    public static bool operator ==(HingeJoint a, HingeJoint b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(HingeJoint a, HingeJoint b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _joint.ToString();
}

/// <summary>
/// A typed view of a prismatic (slider) joint: B moves along an axis fixed in A, rotation locked. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddSlider, PrismaticJointDef.
/// </summary>
public readonly struct SliderJoint : IEquatable<SliderJoint>
{
    private readonly Joint _joint;

    internal SliderJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(SliderJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator SliderJoint(Joint joint)
    {
        if (joint.Type != JointType2D.Prismatic) { throw new InvalidJointException("A " + joint.Type + " joint is not a Prismatic joint (SliderJoint).", nameof(joint)); }
        return new SliderJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType2D.Prismatic); fails when the joint is destroyed.</summary>
    public JointType2D Type => _joint.Type;

    /// <summary>True while the joint exists.</summary>
    public bool IsValid => _joint.IsValid;

    /// <summary>The first body.</summary>
    public Body BodyA => _joint.BodyA;

    /// <summary>The second body.</summary>
    public Body BodyB => _joint.BodyB;

    /// <summary>Destroys the joint. Every copy of this handle becomes stale; no break event is raised.</summary>
    public void Destroy() => _joint.Destroy();

    /// <summary>Raised after the step in which this joint broke; see <see cref="Joint.OnJointBreak"/>.</summary>
    public event Action<JointBreak> OnJointBreak { add { _joint.OnJointBreak += value; } remove { _joint.OnJointBreak -= value; } }

    /// <summary>Whether the two bodies collide with each other (fixed at creation).</summary>
    public bool CollideConnected => _joint.CollideConnected;

    /// <summary>Reaction force above which the joint breaks at the end of a step, newtons; infinity (the default) never breaks.</summary>
    public float BreakForce { get => _joint.BreakForce; set => _joint.BreakForce = value; }

    /// <summary>Reaction torque above which the joint breaks, N·m; infinity never breaks.</summary>
    public float BreakTorque { get => _joint.BreakTorque; set => _joint.BreakTorque = value; }

    /// <summary>The magnitude of the joint's linear reaction force at the end of the last step it was solved, newtons.</summary>
    public float ReactionForce => _joint.ReactionForce;

    /// <summary>The magnitude of the joint's reaction torque at the end of the last step it was solved, N·m.</summary>
    public float ReactionTorque => _joint.ReactionTorque;

    /// <summary>Whether the limit is on.</summary>
    public bool EnableLimit { get => _joint.EnableLimit; set => _joint.EnableLimit = value; }

    /// <summary>Lower translation limit along the axis, metres.</summary>
    public float LowerTranslation => _joint.LowerTranslation;

    /// <summary>Upper translation limit along the axis, metres.</summary>
    public float UpperTranslation => _joint.UpperTranslation;

    /// <summary>Sets both limits (metres) without turning the limit on; lower at most upper.
    /// <exception cref="InvalidJointException">The limits are the wrong way round.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or out of range.</exception></summary>
    public void SetLimits(float lower, float upper) => _joint.SetLimits(lower, upper);

    /// <summary>Whether the motor is on.</summary>
    public bool EnableMotor { get => _joint.EnableMotor; set => _joint.EnableMotor = value; }

    /// <summary>Target speed of the motor: radians per second for a rotational motor, metres per second for a linear one.</summary>
    public float MotorSpeed { get => _joint.MotorSpeed; set => _joint.MotorSpeed = value; }

    /// <summary>Largest motor force, newtons, at least 0.</summary>
    public float MaxMotorForce { get => _joint.MaxMotorForce; set => _joint.MaxMotorForce = value; }

    /// <summary>Whether the spring is on.</summary>
    public bool EnableSpring { get => _joint.EnableSpring; set => _joint.EnableSpring = value; }

    /// <summary>Spring frequency in Hz, at least 0 (0 means no spring force).</summary>
    public float SpringHertz { get => _joint.SpringHertz; set => _joint.SpringHertz = value; }

    /// <summary>Spring damping ratio, at least 0 (1 is critical).</summary>
    public float SpringDampingRatio { get => _joint.SpringDampingRatio; set => _joint.SpringDampingRatio = value; }

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(SliderJoint other) => _joint.Equals(other._joint);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is SliderJoint v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => _joint.GetHashCode();

    /// <summary>Handle equality.</summary>
    public static bool operator ==(SliderJoint a, SliderJoint b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(SliderJoint a, SliderJoint b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _joint.ToString();
}

/// <summary>
/// A typed view of a weld joint: position and rotation locked, optionally springy. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddFixed, WeldJointDef.
/// </summary>
public readonly struct FixedJoint : IEquatable<FixedJoint>
{
    private readonly Joint _joint;

    internal FixedJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(FixedJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator FixedJoint(Joint joint)
    {
        if (joint.Type != JointType2D.Weld) { throw new InvalidJointException("A " + joint.Type + " joint is not a Weld joint (FixedJoint).", nameof(joint)); }
        return new FixedJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType2D.Weld); fails when the joint is destroyed.</summary>
    public JointType2D Type => _joint.Type;

    /// <summary>True while the joint exists.</summary>
    public bool IsValid => _joint.IsValid;

    /// <summary>The first body.</summary>
    public Body BodyA => _joint.BodyA;

    /// <summary>The second body.</summary>
    public Body BodyB => _joint.BodyB;

    /// <summary>Destroys the joint. Every copy of this handle becomes stale; no break event is raised.</summary>
    public void Destroy() => _joint.Destroy();

    /// <summary>Raised after the step in which this joint broke; see <see cref="Joint.OnJointBreak"/>.</summary>
    public event Action<JointBreak> OnJointBreak { add { _joint.OnJointBreak += value; } remove { _joint.OnJointBreak -= value; } }

    /// <summary>Whether the two bodies collide with each other (fixed at creation).</summary>
    public bool CollideConnected => _joint.CollideConnected;

    /// <summary>Reaction force above which the joint breaks at the end of a step, newtons; infinity (the default) never breaks.</summary>
    public float BreakForce { get => _joint.BreakForce; set => _joint.BreakForce = value; }

    /// <summary>Reaction torque above which the joint breaks, N·m; infinity never breaks.</summary>
    public float BreakTorque { get => _joint.BreakTorque; set => _joint.BreakTorque = value; }

    /// <summary>The magnitude of the joint's linear reaction force at the end of the last step it was solved, newtons.</summary>
    public float ReactionForce => _joint.ReactionForce;

    /// <summary>The magnitude of the joint's reaction torque at the end of the last step it was solved, N·m.</summary>
    public float ReactionTorque => _joint.ReactionTorque;

    /// <summary>Sets the linear and angular spring frequencies (Hz, 0 = rigid) and damping ratios.</summary>
    public void SetWeldSprings(float linearHertz, float linearDampingRatio, float angularHertz, float angularDampingRatio) => _joint.SetWeldSprings(linearHertz, linearDampingRatio, angularHertz, angularDampingRatio);

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(FixedJoint other) => _joint.Equals(other._joint);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is FixedJoint v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => _joint.GetHashCode();

    /// <summary>Handle equality.</summary>
    public static bool operator ==(FixedJoint a, FixedJoint b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(FixedJoint a, FixedJoint b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _joint.ToString();
}

/// <summary>
/// A typed view of a wheel joint: a suspension along an axis fixed in the chassis, with the wheel free to spin. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddWheel, WheelJointDef.
/// </summary>
public readonly struct WheelJoint : IEquatable<WheelJoint>
{
    private readonly Joint _joint;

    internal WheelJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(WheelJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator WheelJoint(Joint joint)
    {
        if (joint.Type != JointType2D.Wheel) { throw new InvalidJointException("A " + joint.Type + " joint is not a Wheel joint (WheelJoint).", nameof(joint)); }
        return new WheelJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType2D.Wheel); fails when the joint is destroyed.</summary>
    public JointType2D Type => _joint.Type;

    /// <summary>True while the joint exists.</summary>
    public bool IsValid => _joint.IsValid;

    /// <summary>The first body.</summary>
    public Body BodyA => _joint.BodyA;

    /// <summary>The second body.</summary>
    public Body BodyB => _joint.BodyB;

    /// <summary>Destroys the joint. Every copy of this handle becomes stale; no break event is raised.</summary>
    public void Destroy() => _joint.Destroy();

    /// <summary>Raised after the step in which this joint broke; see <see cref="Joint.OnJointBreak"/>.</summary>
    public event Action<JointBreak> OnJointBreak { add { _joint.OnJointBreak += value; } remove { _joint.OnJointBreak -= value; } }

    /// <summary>Whether the two bodies collide with each other (fixed at creation).</summary>
    public bool CollideConnected => _joint.CollideConnected;

    /// <summary>Reaction force above which the joint breaks at the end of a step, newtons; infinity (the default) never breaks.</summary>
    public float BreakForce { get => _joint.BreakForce; set => _joint.BreakForce = value; }

    /// <summary>Reaction torque above which the joint breaks, N·m; infinity never breaks.</summary>
    public float BreakTorque { get => _joint.BreakTorque; set => _joint.BreakTorque = value; }

    /// <summary>The magnitude of the joint's linear reaction force at the end of the last step it was solved, newtons.</summary>
    public float ReactionForce => _joint.ReactionForce;

    /// <summary>The magnitude of the joint's reaction torque at the end of the last step it was solved, N·m.</summary>
    public float ReactionTorque => _joint.ReactionTorque;

    /// <summary>Whether the limit is on.</summary>
    public bool EnableLimit { get => _joint.EnableLimit; set => _joint.EnableLimit = value; }

    /// <summary>Lower translation limit along the axis, metres.</summary>
    public float LowerTranslation => _joint.LowerTranslation;

    /// <summary>Upper translation limit along the axis, metres.</summary>
    public float UpperTranslation => _joint.UpperTranslation;

    /// <summary>Sets both limits (metres) without turning the limit on; lower at most upper.
    /// <exception cref="InvalidJointException">The limits are the wrong way round.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or out of range.</exception></summary>
    public void SetLimits(float lower, float upper) => _joint.SetLimits(lower, upper);

    /// <summary>Whether the motor is on.</summary>
    public bool EnableMotor { get => _joint.EnableMotor; set => _joint.EnableMotor = value; }

    /// <summary>Target speed of the motor: radians per second for a rotational motor, metres per second for a linear one.</summary>
    public float MotorSpeed { get => _joint.MotorSpeed; set => _joint.MotorSpeed = value; }

    /// <summary>Largest motor torque, N·m, at least 0.</summary>
    public float MaxMotorTorque { get => _joint.MaxMotorTorque; set => _joint.MaxMotorTorque = value; }

    /// <summary>Whether the spring is on.</summary>
    public bool EnableSpring { get => _joint.EnableSpring; set => _joint.EnableSpring = value; }

    /// <summary>Spring frequency in Hz, at least 0 (0 means no spring force).</summary>
    public float SpringHertz { get => _joint.SpringHertz; set => _joint.SpringHertz = value; }

    /// <summary>Spring damping ratio, at least 0 (1 is critical).</summary>
    public float SpringDampingRatio { get => _joint.SpringDampingRatio; set => _joint.SpringDampingRatio = value; }

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(WheelJoint other) => _joint.Equals(other._joint);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is WheelJoint v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => _joint.GetHashCode();

    /// <summary>Handle equality.</summary>
    public static bool operator ==(WheelJoint a, WheelJoint b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(WheelJoint a, WheelJoint b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _joint.ToString();
}

/// <summary>
/// A typed view of a distance joint: a rod or spring between two points, with an optional length limit and motor. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddDistance, AddSpring, DistanceJointDef.
/// </summary>
public readonly struct DistanceJoint : IEquatable<DistanceJoint>
{
    private readonly Joint _joint;

    internal DistanceJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(DistanceJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator DistanceJoint(Joint joint)
    {
        if (joint.Type != JointType2D.Distance) { throw new InvalidJointException("A " + joint.Type + " joint is not a Distance joint (DistanceJoint).", nameof(joint)); }
        return new DistanceJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType2D.Distance); fails when the joint is destroyed.</summary>
    public JointType2D Type => _joint.Type;

    /// <summary>True while the joint exists.</summary>
    public bool IsValid => _joint.IsValid;

    /// <summary>The first body.</summary>
    public Body BodyA => _joint.BodyA;

    /// <summary>The second body.</summary>
    public Body BodyB => _joint.BodyB;

    /// <summary>Destroys the joint. Every copy of this handle becomes stale; no break event is raised.</summary>
    public void Destroy() => _joint.Destroy();

    /// <summary>Raised after the step in which this joint broke; see <see cref="Joint.OnJointBreak"/>.</summary>
    public event Action<JointBreak> OnJointBreak { add { _joint.OnJointBreak += value; } remove { _joint.OnJointBreak -= value; } }

    /// <summary>Whether the two bodies collide with each other (fixed at creation).</summary>
    public bool CollideConnected => _joint.CollideConnected;

    /// <summary>Reaction force above which the joint breaks at the end of a step, newtons; infinity (the default) never breaks.</summary>
    public float BreakForce { get => _joint.BreakForce; set => _joint.BreakForce = value; }

    /// <summary>Reaction torque above which the joint breaks, N·m; infinity never breaks.</summary>
    public float BreakTorque { get => _joint.BreakTorque; set => _joint.BreakTorque = value; }

    /// <summary>The magnitude of the joint's linear reaction force at the end of the last step it was solved, newtons.</summary>
    public float ReactionForce => _joint.ReactionForce;

    /// <summary>The magnitude of the joint's reaction torque at the end of the last step it was solved, N·m.</summary>
    public float ReactionTorque => _joint.ReactionTorque;

    /// <summary>Rest length, metres, at least 0.005.</summary>
    public float Length { get => _joint.Length; set => _joint.Length = value; }

    /// <summary>Whether the limit is on.</summary>
    public bool EnableLimit { get => _joint.EnableLimit; set => _joint.EnableLimit = value; }

    /// <summary>Shortest length the limit allows, metres.</summary>
    public float MinLength => _joint.MinLength;

    /// <summary>Longest length the limit allows, metres.</summary>
    public float MaxLength => _joint.MaxLength;

    /// <summary>Sets both limits (lengths in metres, at least 0) without turning the limit on; lower at most upper.
    /// <exception cref="InvalidJointException">The limits are the wrong way round.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or out of range.</exception></summary>
    public void SetLimits(float lower, float upper) => _joint.SetLimits(lower, upper);

    /// <summary>Whether the motor is on.</summary>
    public bool EnableMotor { get => _joint.EnableMotor; set => _joint.EnableMotor = value; }

    /// <summary>Target speed of the motor: radians per second for a rotational motor, metres per second for a linear one.</summary>
    public float MotorSpeed { get => _joint.MotorSpeed; set => _joint.MotorSpeed = value; }

    /// <summary>Largest motor force, newtons, at least 0.</summary>
    public float MaxMotorForce { get => _joint.MaxMotorForce; set => _joint.MaxMotorForce = value; }

    /// <summary>Whether the spring is on.</summary>
    public bool EnableSpring { get => _joint.EnableSpring; set => _joint.EnableSpring = value; }

    /// <summary>Spring frequency in Hz, at least 0 (0 means no spring force).</summary>
    public float SpringHertz { get => _joint.SpringHertz; set => _joint.SpringHertz = value; }

    /// <summary>Spring damping ratio, at least 0 (1 is critical).</summary>
    public float SpringDampingRatio { get => _joint.SpringDampingRatio; set => _joint.SpringDampingRatio = value; }

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(DistanceJoint other) => _joint.Equals(other._joint);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is DistanceJoint v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => _joint.GetHashCode();

    /// <summary>Handle equality.</summary>
    public static bool operator ==(DistanceJoint a, DistanceJoint b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(DistanceJoint a, DistanceJoint b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _joint.ToString();
}

/// <summary>
/// A typed view of a mouse (target) joint: pulls a point of a body towards a moving world target. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddTarget, MouseJointDef.
/// </summary>
public readonly struct TargetJoint : IEquatable<TargetJoint>
{
    private readonly Joint _joint;

    internal TargetJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(TargetJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator TargetJoint(Joint joint)
    {
        if (joint.Type != JointType2D.Mouse) { throw new InvalidJointException("A " + joint.Type + " joint is not a Mouse joint (TargetJoint).", nameof(joint)); }
        return new TargetJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType2D.Mouse); fails when the joint is destroyed.</summary>
    public JointType2D Type => _joint.Type;

    /// <summary>True while the joint exists.</summary>
    public bool IsValid => _joint.IsValid;

    /// <summary>The first body.</summary>
    public Body BodyA => _joint.BodyA;

    /// <summary>The second body.</summary>
    public Body BodyB => _joint.BodyB;

    /// <summary>Destroys the joint. Every copy of this handle becomes stale; no break event is raised.</summary>
    public void Destroy() => _joint.Destroy();

    /// <summary>Raised after the step in which this joint broke; see <see cref="Joint.OnJointBreak"/>.</summary>
    public event Action<JointBreak> OnJointBreak { add { _joint.OnJointBreak += value; } remove { _joint.OnJointBreak -= value; } }

    /// <summary>Whether the two bodies collide with each other (fixed at creation).</summary>
    public bool CollideConnected => _joint.CollideConnected;

    /// <summary>Reaction force above which the joint breaks at the end of a step, newtons; infinity (the default) never breaks.</summary>
    public float BreakForce { get => _joint.BreakForce; set => _joint.BreakForce = value; }

    /// <summary>Reaction torque above which the joint breaks, N·m; infinity never breaks.</summary>
    public float BreakTorque { get => _joint.BreakTorque; set => _joint.BreakTorque = value; }

    /// <summary>The magnitude of the joint's linear reaction force at the end of the last step it was solved, newtons.</summary>
    public float ReactionForce => _joint.ReactionForce;

    /// <summary>The magnitude of the joint's reaction torque at the end of the last step it was solved, N·m.</summary>
    public float ReactionTorque => _joint.ReactionTorque;

    /// <summary>The world position the joint pulls its anchor to. Setting it wakes the dragged body.</summary>
    public Vector2D Target { get => _joint.Target; set => _joint.Target = value; }

    /// <summary>Largest pulling force, newtons.</summary>
    public float MaxForce { get => _joint.MaxForce; set => _joint.MaxForce = value; }

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(TargetJoint other) => _joint.Equals(other._joint);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is TargetJoint v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => _joint.GetHashCode();

    /// <summary>Handle equality.</summary>
    public static bool operator ==(TargetJoint a, TargetJoint b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(TargetJoint a, TargetJoint b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _joint.ToString();
}

/// <summary>
/// A typed view of a motor (relative) joint: drives B towards a pose relative to A with limited force and torque. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddRelative, MotorJointDef.
/// </summary>
public readonly struct RelativeJoint : IEquatable<RelativeJoint>
{
    private readonly Joint _joint;

    internal RelativeJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(RelativeJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator RelativeJoint(Joint joint)
    {
        if (joint.Type != JointType2D.Motor) { throw new InvalidJointException("A " + joint.Type + " joint is not a Motor joint (RelativeJoint).", nameof(joint)); }
        return new RelativeJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType2D.Motor); fails when the joint is destroyed.</summary>
    public JointType2D Type => _joint.Type;

    /// <summary>True while the joint exists.</summary>
    public bool IsValid => _joint.IsValid;

    /// <summary>The first body.</summary>
    public Body BodyA => _joint.BodyA;

    /// <summary>The second body.</summary>
    public Body BodyB => _joint.BodyB;

    /// <summary>Destroys the joint. Every copy of this handle becomes stale; no break event is raised.</summary>
    public void Destroy() => _joint.Destroy();

    /// <summary>Raised after the step in which this joint broke; see <see cref="Joint.OnJointBreak"/>.</summary>
    public event Action<JointBreak> OnJointBreak { add { _joint.OnJointBreak += value; } remove { _joint.OnJointBreak -= value; } }

    /// <summary>Whether the two bodies collide with each other (fixed at creation).</summary>
    public bool CollideConnected => _joint.CollideConnected;

    /// <summary>Reaction force above which the joint breaks at the end of a step, newtons; infinity (the default) never breaks.</summary>
    public float BreakForce { get => _joint.BreakForce; set => _joint.BreakForce = value; }

    /// <summary>Reaction torque above which the joint breaks, N·m; infinity never breaks.</summary>
    public float BreakTorque { get => _joint.BreakTorque; set => _joint.BreakTorque = value; }

    /// <summary>The magnitude of the joint's linear reaction force at the end of the last step it was solved, newtons.</summary>
    public float ReactionForce => _joint.ReactionForce;

    /// <summary>The magnitude of the joint's reaction torque at the end of the last step it was solved, N·m.</summary>
    public float ReactionTorque => _joint.ReactionTorque;

    /// <summary>Largest force, newtons.</summary>
    public float MaxForce { get => _joint.MaxForce; set => _joint.MaxForce = value; }

    /// <summary>Largest torque, N·m.</summary>
    public float MaxTorque { get => _joint.MaxTorque; set => _joint.MaxTorque = value; }

    /// <summary>Sets the target pose of B relative to A: the origin offset in A's frame (metres) and the angle (radians).</summary>
    public void SetOffset(Vector2F linear, float angular) => _joint.SetOffset(linear, angular);

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(RelativeJoint other) => _joint.Equals(other._joint);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is RelativeJoint v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => _joint.GetHashCode();

    /// <summary>Handle equality.</summary>
    public static bool operator ==(RelativeJoint a, RelativeJoint b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(RelativeJoint a, RelativeJoint b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _joint.ToString();
}
