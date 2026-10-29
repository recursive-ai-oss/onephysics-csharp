using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// A typed view of a fixed joint: the two frames are locked together. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddFixed, FixedJointDef.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
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
        if (joint.Type != JointType3D.Fixed) { throw new InvalidJointException("A " + joint.Type + " joint is not a Fixed joint (FixedJoint).", nameof(joint)); }
        return new FixedJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType3D.Fixed); fails when the joint is destroyed.</summary>
    public JointType3D Type => _joint.Type;

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
/// A typed view of a point (ball-and-socket) joint: the two frame origins stay together, rotation is free. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddPoint, PointJointDef.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct PointJoint : IEquatable<PointJoint>
{
    private readonly Joint _joint;

    internal PointJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(PointJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator PointJoint(Joint joint)
    {
        if (joint.Type != JointType3D.Point) { throw new InvalidJointException("A " + joint.Type + " joint is not a Point joint (PointJoint).", nameof(joint)); }
        return new PointJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType3D.Point); fails when the joint is destroyed.</summary>
    public JointType3D Type => _joint.Type;

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

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(PointJoint other) => _joint.Equals(other._joint);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is PointJoint v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => _joint.GetHashCode();

    /// <summary>Handle equality.</summary>
    public static bool operator ==(PointJoint a, PointJoint b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(PointJoint a, PointJoint b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _joint.ToString();
}

/// <summary>
/// A typed view of a distance joint: a rod or spring between two points, with an optional length limit. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddDistance, AddSpring, DistanceJointDef.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
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
        if (joint.Type != JointType3D.Distance) { throw new InvalidJointException("A " + joint.Type + " joint is not a Distance joint (DistanceJoint).", nameof(joint)); }
        return new DistanceJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType3D.Distance); fails when the joint is destroyed.</summary>
    public JointType3D Type => _joint.Type;

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
/// A typed view of a hinge joint: rotation about one axis, with optional limits, motor and spring. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddHinge, HingeJointDef.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
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
        if (joint.Type != JointType3D.Hinge) { throw new InvalidJointException("A " + joint.Type + " joint is not a Hinge joint (HingeJoint).", nameof(joint)); }
        return new HingeJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType3D.Hinge); fails when the joint is destroyed.</summary>
    public JointType3D Type => _joint.Type;

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
/// A typed view of a slider joint: translation along one axis, with optional limits, motor and spring. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddSlider, SliderJointDef.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
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
        if (joint.Type != JointType3D.Slider) { throw new InvalidJointException("A " + joint.Type + " joint is not a Slider joint (SliderJoint).", nameof(joint)); }
        return new SliderJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType3D.Slider); fails when the joint is destroyed.</summary>
    public JointType3D Type => _joint.Type;

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
/// A typed view of a cone joint: the two x axes stay within a cone. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddCone, ConeJointDef.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct ConeJoint : IEquatable<ConeJoint>
{
    private readonly Joint _joint;

    internal ConeJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(ConeJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator ConeJoint(Joint joint)
    {
        if (joint.Type != JointType3D.Cone) { throw new InvalidJointException("A " + joint.Type + " joint is not a Cone joint (ConeJoint).", nameof(joint)); }
        return new ConeJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType3D.Cone); fails when the joint is destroyed.</summary>
    public JointType3D Type => _joint.Type;

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

    /// <summary>The largest angle between the two x axes, radians within (0, pi].</summary>
    public float HalfConeAngle { get => _joint.HalfConeAngle; set => _joint.HalfConeAngle = value; }

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(ConeJoint other) => _joint.Equals(other._joint);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is ConeJoint v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => _joint.GetHashCode();

    /// <summary>Handle equality.</summary>
    public static bool operator ==(ConeJoint a, ConeJoint b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(ConeJoint a, ConeJoint b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _joint.ToString();
}

/// <summary>
/// A typed view of a swing-twist joint (a ragdoll joint): twist about one axis and swing about the other two, within limits, optionally driven to a pose. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddCharacterJoint, SwingTwistJointDef.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct SwingTwistJoint : IEquatable<SwingTwistJoint>
{
    private readonly Joint _joint;

    internal SwingTwistJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(SwingTwistJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator SwingTwistJoint(Joint joint)
    {
        if (joint.Type != JointType3D.SwingTwist) { throw new InvalidJointException("A " + joint.Type + " joint is not a SwingTwist joint (SwingTwistJoint).", nameof(joint)); }
        return new SwingTwistJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType3D.SwingTwist); fails when the joint is destroyed.</summary>
    public JointType3D Type => _joint.Type;

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

    /// <summary>Sets the twist range (radians within -pi..pi) and the swing half angles about the frame y and z axes (within (0, pi]).</summary>
    public void SetSwingTwistLimits(float lowerTwist, float upperTwist, float swingYHalfAngle, float swingZHalfAngle) => _joint.SetSwingTwistLimits(lowerTwist, upperTwist, swingYHalfAngle, swingZHalfAngle);

    /// <summary>Drives frame B towards <paramref name="target"/> relative to frame A with an angular spring-damper (see <see cref="Joint.SetPoseDrive"/>).</summary>
    public void SetPoseDrive(QuaternionF target, float stiffness, float damping, float maxTorque) => _joint.SetPoseDrive(target, stiffness, damping, maxTorque);

    /// <summary>Removes the pose drive. Does nothing when there is none.</summary>
    public void ClearPoseDrive() => _joint.ClearPoseDrive();

    /// <summary>True when the joint has a pose drive.</summary>
    public bool IsPoseDriven => _joint.IsPoseDriven;

    /// <summary>The drive's target rotation of frame B relative to frame A; the identity when there is no drive.</summary>
    public QuaternionF PoseDriveTarget => _joint.PoseDriveTarget;

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(SwingTwistJoint other) => _joint.Equals(other._joint);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is SwingTwistJoint v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => _joint.GetHashCode();

    /// <summary>Handle equality.</summary>
    public static bool operator ==(SwingTwistJoint a, SwingTwistJoint b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(SwingTwistJoint a, SwingTwistJoint b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _joint.ToString();
}

/// <summary>
/// A typed view of a six-degrees-of-freedom joint: each of the three translation and three rotation axes is free, locked, limited, driven or sprung. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddConfigurable, SixDofJointDef.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct SixDofJoint : IEquatable<SixDofJoint>
{
    private readonly Joint _joint;

    internal SixDofJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(SixDofJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator SixDofJoint(Joint joint)
    {
        if (joint.Type != JointType3D.SixDof) { throw new InvalidJointException("A " + joint.Type + " joint is not a SixDof joint (SixDofJoint).", nameof(joint)); }
        return new SixDofJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType3D.SixDof); fails when the joint is destroyed.</summary>
    public JointType3D Type => _joint.Type;

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

    /// <summary>Sets how one axis is treated. Clears the accumulated impulses of that axis.</summary>
    public void SetAxisMode(JointAxes axis, JointAxisMode mode) => _joint.SetAxisMode(axis, mode);

    /// <summary>Sets the limits of one axis (metres or radians; used while the axis is limited).</summary>
    public void SetAxisLimits(JointAxes axis, float lower, float upper) => _joint.SetAxisLimits(axis, lower, upper);

    /// <summary>Turns the motor of one axis on or off and sets its target speed and largest force (a translation axis) or torque (a rotation axis).</summary>
    public void SetAxisMotor(JointAxes axis, bool enabled, float speed = 0f, float maxForceOrTorque = 0f) => _joint.SetAxisMotor(axis, enabled, speed, maxForceOrTorque);

    /// <summary>Turns the spring of one axis on or off (towards zero translation or angle) and sets its frequency (Hz) and damping ratio.</summary>
    public void SetAxisSpring(JointAxes axis, bool enabled, float hertz = 0f, float dampingRatio = 0.5f) => _joint.SetAxisSpring(axis, enabled, hertz, dampingRatio);

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(SixDofJoint other) => _joint.Equals(other._joint);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is SixDofJoint v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => _joint.GetHashCode();

    /// <summary>Handle equality.</summary>
    public static bool operator ==(SixDofJoint a, SixDofJoint b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(SixDofJoint a, SixDofJoint b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _joint.ToString();
}

/// <summary>
/// A typed view of a path joint: the follower is constrained to a curve fixed in the path body, with an optional motor along it. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddPath, PathJointDef.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct PathJoint : IEquatable<PathJoint>
{
    private readonly Joint _joint;

    internal PathJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(PathJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator PathJoint(Joint joint)
    {
        if (joint.Type != JointType3D.Path) { throw new InvalidJointException("A " + joint.Type + " joint is not a Path joint (PathJoint).", nameof(joint)); }
        return new PathJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType3D.Path); fails when the joint is destroyed.</summary>
    public JointType3D Type => _joint.Type;

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

    /// <summary>Whether the motor is on.</summary>
    public bool EnableMotor { get => _joint.EnableMotor; set => _joint.EnableMotor = value; }

    /// <summary>Target speed of the motor: radians per second for a rotational motor, metres per second for a linear one.</summary>
    public float MotorSpeed { get => _joint.MotorSpeed; set => _joint.MotorSpeed = value; }

    /// <summary>Largest motor force, newtons, at least 0.</summary>
    public float MaxMotorForce { get => _joint.MaxMotorForce; set => _joint.MaxMotorForce = value; }

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(PathJoint other) => _joint.Equals(other._joint);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is PathJoint v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => _joint.GetHashCode();

    /// <summary>Handle equality.</summary>
    public static bool operator ==(PathJoint a, PathJoint b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(PathJoint a, PathJoint b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _joint.ToString();
}

/// <summary>
/// A typed view of a gear joint: couples the rotation of two bodies. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddGear, GearJointDef.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct GearJoint : IEquatable<GearJoint>
{
    private readonly Joint _joint;

    internal GearJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(GearJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator GearJoint(Joint joint)
    {
        if (joint.Type != JointType3D.Gear) { throw new InvalidJointException("A " + joint.Type + " joint is not a Gear joint (GearJoint).", nameof(joint)); }
        return new GearJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType3D.Gear); fails when the joint is destroyed.</summary>
    public JointType3D Type => _joint.Type;

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

    /// <summary>The gear ratio.</summary>
    public float Ratio { get => _joint.Ratio; set => _joint.Ratio = value; }

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(GearJoint other) => _joint.Equals(other._joint);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is GearJoint v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => _joint.GetHashCode();

    /// <summary>Handle equality.</summary>
    public static bool operator ==(GearJoint a, GearJoint b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(GearJoint a, GearJoint b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _joint.ToString();
}

/// <summary>
/// A typed view of a rack-and-pinion joint: couples a rotation to a translation. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddRackAndPinion, RackAndPinionJointDef.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct RackAndPinionJoint : IEquatable<RackAndPinionJoint>
{
    private readonly Joint _joint;

    internal RackAndPinionJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(RackAndPinionJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator RackAndPinionJoint(Joint joint)
    {
        if (joint.Type != JointType3D.RackAndPinion) { throw new InvalidJointException("A " + joint.Type + " joint is not a RackAndPinion joint (RackAndPinionJoint).", nameof(joint)); }
        return new RackAndPinionJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType3D.RackAndPinion); fails when the joint is destroyed.</summary>
    public JointType3D Type => _joint.Type;

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

    /// <summary>The ratio of translation to rotation.</summary>
    public float Ratio { get => _joint.Ratio; set => _joint.Ratio = value; }

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(RackAndPinionJoint other) => _joint.Equals(other._joint);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is RackAndPinionJoint v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => _joint.GetHashCode();

    /// <summary>Handle equality.</summary>
    public static bool operator ==(RackAndPinionJoint a, RackAndPinionJoint b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(RackAndPinionJoint a, RackAndPinionJoint b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _joint.ToString();
}

/// <summary>
/// A typed view of a pulley joint: the sum of two rope lengths (one per body) stays constant. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddPulley, PulleyJointDef.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct PulleyJoint : IEquatable<PulleyJoint>
{
    private readonly Joint _joint;

    internal PulleyJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(PulleyJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator PulleyJoint(Joint joint)
    {
        if (joint.Type != JointType3D.Pulley) { throw new InvalidJointException("A " + joint.Type + " joint is not a Pulley joint (PulleyJoint).", nameof(joint)); }
        return new PulleyJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType3D.Pulley); fails when the joint is destroyed.</summary>
    public JointType3D Type => _joint.Type;

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

    /// <summary>The pulley ratio.</summary>
    public float Ratio { get => _joint.Ratio; set => _joint.Ratio = value; }

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(PulleyJoint other) => _joint.Equals(other._joint);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is PulleyJoint v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => _joint.GetHashCode();

    /// <summary>Handle equality.</summary>
    public static bool operator ==(PulleyJoint a, PulleyJoint b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(PulleyJoint a, PulleyJoint b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _joint.ToString();
}
