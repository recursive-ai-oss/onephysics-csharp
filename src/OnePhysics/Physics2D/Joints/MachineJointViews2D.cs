using System;
using RecursiveAI.OnePhysics;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// A typed view of a path joint: the follower is constrained to a curve fixed in the path body, with an optional motor along it. It converts implicitly to <see cref="Joint"/> and has only the members this kind of joint has,
/// so a member of another kind does not compile. A small struct that allocates nothing; once the joint is destroyed every copy fails with a <see cref="StaleHandleException"/>.
/// Created by: AddPath, PathJointDef.
/// </summary>
public readonly struct PathJoint : IEquatable<PathJoint>
{
    private readonly Joint _joint;

    internal PathJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Whole-step force/torque and last-pass solver errors, without allocation.</summary>
    public JointInspection Inspect() => _joint.Inspect();

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(PathJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator PathJoint(Joint joint)
    {
        if (joint.Type != JointType2D.Path) { throw new InvalidJointException("A " + joint.Type + " joint is not a Path joint (PathJoint).", nameof(joint)); }
        return new PathJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType2D.Path); fails when the joint is destroyed.</summary>
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

    /// <summary>Steps remaining until breaking; zero means unlimited. Negative values throw.</summary>
    public int LifetimeStepsRemaining { get => _joint.LifetimeStepsRemaining; set => _joint.LifetimeStepsRemaining = value; }

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
public readonly struct GearJoint : IEquatable<GearJoint>
{
    private readonly Joint _joint;

    internal GearJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Whole-step force/torque and last-pass solver errors, without allocation.</summary>
    public JointInspection Inspect() => _joint.Inspect();

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(GearJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator GearJoint(Joint joint)
    {
        if (joint.Type != JointType2D.Gear) { throw new InvalidJointException("A " + joint.Type + " joint is not a Gear joint (GearJoint).", nameof(joint)); }
        return new GearJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType2D.Gear); fails when the joint is destroyed.</summary>
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

    /// <summary>Steps remaining until breaking; zero means unlimited. Negative values throw.</summary>
    public int LifetimeStepsRemaining { get => _joint.LifetimeStepsRemaining; set => _joint.LifetimeStepsRemaining = value; }

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
public readonly struct RackAndPinionJoint : IEquatable<RackAndPinionJoint>
{
    private readonly Joint _joint;

    internal RackAndPinionJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Whole-step force/torque and last-pass solver errors, without allocation.</summary>
    public JointInspection Inspect() => _joint.Inspect();

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(RackAndPinionJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator RackAndPinionJoint(Joint joint)
    {
        if (joint.Type != JointType2D.RackAndPinion) { throw new InvalidJointException("A " + joint.Type + " joint is not a RackAndPinion joint (RackAndPinionJoint).", nameof(joint)); }
        return new RackAndPinionJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType2D.RackAndPinion); fails when the joint is destroyed.</summary>
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

    /// <summary>Steps remaining until breaking; zero means unlimited. Negative values throw.</summary>
    public int LifetimeStepsRemaining { get => _joint.LifetimeStepsRemaining; set => _joint.LifetimeStepsRemaining = value; }

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
public readonly struct PulleyJoint : IEquatable<PulleyJoint>
{
    private readonly Joint _joint;

    internal PulleyJoint(Joint joint) { _joint = joint; }

    /// <summary>The general <see cref="Joint"/> handle for this joint (the same as the implicit conversion).</summary>
    public Joint Joint => _joint;

    /// <summary>Whole-step force/torque and last-pass solver errors, without allocation.</summary>
    public JointInspection Inspect() => _joint.Inspect();

    /// <summary>Converts to the general handle.</summary>
    public static implicit operator Joint(PulleyJoint view) => view._joint;

    /// <summary>Converts a general handle to this view.</summary>
    /// <exception cref="InvalidJointException">The joint is of another kind.</exception>
    public static explicit operator PulleyJoint(Joint joint)
    {
        if (joint.Type != JointType2D.Pulley) { throw new InvalidJointException("A " + joint.Type + " joint is not a Pulley joint (PulleyJoint).", nameof(joint)); }
        return new PulleyJoint(joint);
    }

    /// <summary>The raw core handle (world id, constraint slot, generation).</summary>
    public JointHandle RawHandle => _joint.RawHandle;

    /// <summary>The kind of joint (always JointType2D.Pulley); fails when the joint is destroyed.</summary>
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

    /// <summary>Steps remaining until breaking; zero means unlimited. Negative values throw.</summary>
    public int LifetimeStepsRemaining { get => _joint.LifetimeStepsRemaining; set => _joint.LifetimeStepsRemaining = value; }

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
