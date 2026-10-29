using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>What a joint-break callback is told. The joint is already destroyed, so <see cref="Joint"/> is stale and only identifies it (compare handles, or read <see cref="Joint.RawHandle"/>).</summary>
[System.Diagnostics.CodeAnalysis.SuppressMessage("Design", "CA1051:Do not declare visible instance fields", Justification = "Handle fields, not properties, so that `hit.Body.LinearVelocity = v` compiles: a property result is not a variable (CS1612), a readonly field is.")]
public readonly struct JointBreak
{
    internal JointBreak(Joint joint, Body bodyA, Body bodyB, float value)
    {
        Joint = joint;
        BodyA = bodyA;
        BodyB = bodyB;
        Value = value;
    }

    /// <summary>The joint that broke (destroyed by the break).</summary>
    public readonly Joint Joint;

    /// <summary>The joint's first body (stale if that body was destroyed in the meantime).</summary>
    public readonly Body BodyA;

    /// <summary>The joint's second body.</summary>
    public readonly Body BodyB;

    /// <summary>The reaction force (N) or torque (N·m) that exceeded the threshold, as measured at the end of the step.</summary>
    public float Value { get; }

    /// <inheritdoc/>
    public override string ToString() =>
        "JointBreak(" + Joint + ", " + BodyA + " - " + BodyB + ", value " + Value.ToString(System.Globalization.CultureInfo.InvariantCulture) + ")";
}

/// <summary>
/// A handle to a joint: world, slot and generation. A small struct that allocates nothing; once the joint is destroyed (by <see cref="Destroy"/>, by breaking, or because a body was destroyed)
/// every copy fails with a <see cref="StaleHandleException"/> naming the joint. Properties that a joint type does not have throw <see cref="InvalidJointException"/>.
/// Edits take effect at the next step and wake the joint's bodies. The <c>Add*</c> one-liners and <c>CreateJoint</c> return a typed view (for example <c>HingeJoint</c>) that has only the members its kind has and converts implicitly to <see cref="Joint"/>; use <see cref="Joint"/> to hold joints of any kind in one collection (the explicit conversion back checks the kind).
/// </summary>
public readonly struct Joint : IEquatable<Joint>
{
    private readonly PhysicsWorld2D? _world;
    private readonly JointHandle _handle;

    internal Joint(PhysicsWorld2D world, JointHandle handle)
    {
        _world = world;
        _handle = handle;
    }

    /// <summary>The raw core handle (world id, constraint slot, generation) for the raw tier and for events.</summary>
    public JointHandle RawHandle => _handle;

    /// <summary>True while the joint exists.</summary>
    public bool IsValid => _world != null && _handle.World == _world.Id && _world.Core.Constraints.Handles.IsAlive(_handle.Index, _handle.Generation) && _world.Core.Constraints.KindOf(_handle.Index) == ConstraintKind.Joint;

    private ref JointData2D Rec(out PhysicsWorld2D w, out int c)
    {
        if (_world == null) { throw new InvalidHandleException("The default Joint is not attached to a world."); }
        w = _world;
        c = w.Core.Resolve(_handle);
        w.Engine.BindJoints();
        return ref w.Engine.Joints[c];
    }

    private static void Need(in JointData2D d, string what, params JointType2D[] types)
    {
        foreach (var t in types) { if ((byte)t == d.Type) { return; } }
        throw new InvalidJointException("A " + JointData2D.Name((JointType2D)d.Type) + " joint has no " + what + ".");
    }

    private static void Commit(PhysicsWorld2D w, int c, ref JointData2D d, in JointData2D edited)
    {
        edited.Validate();
        d = edited;
        var cs = w.Core.Constraints;
        w.Core.Wake(w.Core.HandleOfBody(cs.BodyA[c]));
        w.Core.Wake(w.Core.HandleOfBody(cs.BodyB[c]));
    }

    /// <summary>The kind of joint.</summary>
    public JointType2D Type { get { ref var d = ref Rec(out _, out _); return (JointType2D)d.Type; } }

    /// <summary>The first body.</summary>
    public Body BodyA { get { Rec(out var w, out int c); return new Body(w, w.Core.HandleOfBody(w.Core.Constraints.BodyA[c])); } }

    /// <summary>The second body.</summary>
    public Body BodyB { get { Rec(out var w, out int c); return new Body(w, w.Core.HandleOfBody(w.Core.Constraints.BodyB[c])); } }

    /// <summary>Whether the two bodies collide with each other (fixed at creation).</summary>
    public bool CollideConnected { get { ref var d = ref Rec(out _, out _); return d.Collide; } }

    /// <summary>Destroys the joint. Every copy of this handle becomes stale; no break event is raised.</summary>
    public void Destroy() { Rec(out var w, out _); w.DestroyJoint(_handle); }

    // ---- limits ----

    /// <summary>Whether the limit is on (revolute, prismatic, wheel, distance).</summary>
    public bool EnableLimit
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "limit", JointType2D.Revolute, JointType2D.Prismatic, JointType2D.Wheel, JointType2D.Distance); return d.Limit; }
        set
        {
            ref var d = ref Rec(out var w, out int c);
            Need(in d, "limit", JointType2D.Revolute, JointType2D.Prismatic, JointType2D.Wheel, JointType2D.Distance);
            var e = d;
            e.Flags = value ? (byte)(e.Flags | JointData2D.FlagLimit) : (byte)(e.Flags & ~JointData2D.FlagLimit);
            Commit(w, c, ref d, in e);
            if (!value) { var p = w.Core.Constraints.Payload(c); p[JPay.Lower] = 0f; p[JPay.Upper] = 0f; }
        }
    }

    private float LimitValue(bool upper, string what, params JointType2D[] types)
    {
        ref var d = ref Rec(out _, out _);
        Need(in d, what, types);
        return upper ? d.Upper : d.Lower;
    }

    /// <summary>Hinge (revolute) joints: the lower angle limit, radians.</summary>
    /// <exception cref="InvalidJointException">The joint is not a revolute joint.</exception>
    public float LowerAngle => LimitValue(false, "angle limit", JointType2D.Revolute);

    /// <summary>Hinge (revolute) joints: the upper angle limit, radians.</summary>
    /// <exception cref="InvalidJointException">The joint is not a revolute joint.</exception>
    public float UpperAngle => LimitValue(true, "angle limit", JointType2D.Revolute);

    /// <summary>Slider (prismatic) and wheel joints: the lower translation limit along the axis, metres.</summary>
    /// <exception cref="InvalidJointException">The joint is not a prismatic or wheel joint.</exception>
    public float LowerTranslation => LimitValue(false, "translation limit", JointType2D.Prismatic, JointType2D.Wheel);

    /// <summary>Slider (prismatic) and wheel joints: the upper translation limit along the axis, metres.</summary>
    /// <exception cref="InvalidJointException">The joint is not a prismatic or wheel joint.</exception>
    public float UpperTranslation => LimitValue(true, "translation limit", JointType2D.Prismatic, JointType2D.Wheel);

    /// <summary>Distance joints: the shortest length the limit allows, metres.</summary>
    /// <exception cref="InvalidJointException">The joint is not a distance joint.</exception>
    public float MinLength => LimitValue(false, "length limit", JointType2D.Distance);

    /// <summary>Distance joints: the longest length the limit allows, metres.</summary>
    /// <exception cref="InvalidJointException">The joint is not a distance joint.</exception>
    public float MaxLength => LimitValue(true, "length limit", JointType2D.Distance);

    /// <summary>Sets both limits without turning the limit on: angles in radians within -pi..pi (revolute), translations in metres (prismatic, wheel) or lengths in metres, at least 0 (distance); lower at most upper.</summary>
    /// <exception cref="InvalidJointException">The limits are the wrong way round, or the joint has none.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or out of range.</exception>
    public void SetLimits(float lower, float upper)
    {
        ref var d = ref Rec(out var w, out int c);
        Need(in d, "limit", JointType2D.Revolute, JointType2D.Prismatic, JointType2D.Wheel, JointType2D.Distance);
        var e = d;
        e.Lower = lower; e.Upper = upper;
        bool wasLimit = e.Limit;
        e.Flags |= JointData2D.FlagLimit; // validate as an active limit
        e.Validate();
        if (!wasLimit) { e.Flags = (byte)(e.Flags & ~JointData2D.FlagLimit); }
        Commit(w, c, ref d, in e);
    }

    // ---- motor ----

    /// <summary>Whether the motor is on (revolute, prismatic, wheel, distance).</summary>
    public bool EnableMotor
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "motor", JointType2D.Revolute, JointType2D.Prismatic, JointType2D.Wheel, JointType2D.Distance); return d.Motor; }
        set
        {
            ref var d = ref Rec(out var w, out int c);
            Need(in d, "motor", JointType2D.Revolute, JointType2D.Prismatic, JointType2D.Wheel, JointType2D.Distance);
            var e = d;
            e.Flags = value ? (byte)(e.Flags | JointData2D.FlagMotor) : (byte)(e.Flags & ~JointData2D.FlagMotor);
            Commit(w, c, ref d, in e);
            if (!value) { w.Core.Constraints.Payload(c)[JPay.Motor] = 0f; }
        }
    }

    /// <summary>Target speed: radians per second (revolute, wheel) or metres per second (prismatic, distance).</summary>
    public float MotorSpeed
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "motor", JointType2D.Revolute, JointType2D.Prismatic, JointType2D.Wheel, JointType2D.Distance); return d.MotorSpeed; }
        set { ref var d = ref Rec(out var w, out int c); Need(in d, "motor", JointType2D.Revolute, JointType2D.Prismatic, JointType2D.Wheel, JointType2D.Distance); var e = d; e.MotorSpeed = value; Commit(w, c, ref d, in e); }
    }

    /// <summary>Largest motor force in newtons of a slider (prismatic) or distance joint, at least 0. Rotational motors take a torque: <see cref="MaxMotorTorque"/>.</summary>
    /// <exception cref="InvalidJointException">The joint's motor is rotational (revolute, wheel) or the joint has none.</exception>
    public float MaxMotorForce
    {
        get { ref var d = ref Rec(out _, out _); NeedForceMotor(in d); return d.MaxMotor; }
        set { ref var d = ref Rec(out var w, out int c); NeedForceMotor(in d); var e = d; e.MaxMotor = value; Commit(w, c, ref d, in e); }
    }

    /// <summary>Largest motor torque in N·m of a hinge (revolute) or wheel joint, at least 0. Linear motors take a force: <see cref="MaxMotorForce"/>.</summary>
    /// <exception cref="InvalidJointException">The joint's motor is linear (prismatic, distance) or the joint has none.</exception>
    public float MaxMotorTorque
    {
        get { ref var d = ref Rec(out _, out _); NeedTorqueMotor(in d); return d.MaxMotor; }
        set { ref var d = ref Rec(out var w, out int c); NeedTorqueMotor(in d); var e = d; e.MaxMotor = value; Commit(w, c, ref d, in e); }
    }

    private static void NeedForceMotor(in JointData2D d)
    {
        if (d.Type == (byte)JointType2D.Revolute || d.Type == (byte)JointType2D.Wheel) { throw new InvalidJointException("A " + JointData2D.Name((JointType2D)d.Type) + " joint's motor is rotational and takes a torque: use MaxMotorTorque."); }
        Need(in d, "motor force", JointType2D.Prismatic, JointType2D.Distance);
    }

    private static void NeedTorqueMotor(in JointData2D d)
    {
        if (d.Type == (byte)JointType2D.Prismatic || d.Type == (byte)JointType2D.Distance) { throw new InvalidJointException("A " + JointData2D.Name((JointType2D)d.Type) + " joint's motor is linear and takes a force: use MaxMotorForce."); }
        Need(in d, "motor torque", JointType2D.Revolute, JointType2D.Wheel);
    }

    // ---- spring ----

    /// <summary>Whether the spring is on (revolute, prismatic, wheel, distance).</summary>
    public bool EnableSpring
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "spring", JointType2D.Revolute, JointType2D.Prismatic, JointType2D.Wheel, JointType2D.Distance); return d.Spring; }
        set
        {
            ref var d = ref Rec(out var w, out int c);
            Need(in d, "spring", JointType2D.Revolute, JointType2D.Prismatic, JointType2D.Wheel, JointType2D.Distance);
            var e = d;
            e.Flags = value ? (byte)(e.Flags | JointData2D.FlagSpring) : (byte)(e.Flags & ~JointData2D.FlagSpring);
            Commit(w, c, ref d, in e);
            if (!value) { var p = w.Core.Constraints.Payload(c); p[JPay.Spring] = 0f; if (e.Type == (byte)JointType2D.Distance) { p[JPay.P0] = 0f; } }
        }
    }

    /// <summary>Spring frequency in Hz, at least 0 (0 means no spring force).</summary>
    public float SpringHertz
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "spring", JointType2D.Revolute, JointType2D.Prismatic, JointType2D.Wheel, JointType2D.Distance); return d.Hertz; }
        set { ref var d = ref Rec(out var w, out int c); Need(in d, "spring", JointType2D.Revolute, JointType2D.Prismatic, JointType2D.Wheel, JointType2D.Distance); var e = d; e.Hertz = value; Commit(w, c, ref d, in e); }
    }

    /// <summary>Spring damping ratio, at least 0 (1 is critical).</summary>
    public float SpringDampingRatio
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "spring", JointType2D.Revolute, JointType2D.Prismatic, JointType2D.Wheel, JointType2D.Distance); return d.Damping; }
        set { ref var d = ref Rec(out var w, out int c); Need(in d, "spring", JointType2D.Revolute, JointType2D.Prismatic, JointType2D.Wheel, JointType2D.Distance); var e = d; e.Damping = value; Commit(w, c, ref d, in e); }
    }

    // ---- type specific ----

    /// <summary>Rest length of a distance joint, metres, at least 0.005.</summary>
    public float Length
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "length", JointType2D.Distance); return d.Length; }
        set { ref var d = ref Rec(out var w, out int c); Need(in d, "length", JointType2D.Distance); var e = d; e.Length = value; Commit(w, c, ref d, in e); }
    }

    /// <summary>Weld: sets the linear and angular spring frequencies (Hz, 0 = rigid) and damping ratios.</summary>
    public void SetWeldSprings(float linearHertz, float linearDampingRatio, float angularHertz, float angularDampingRatio)
    {
        ref var d = ref Rec(out var w, out int c);
        Need(in d, "weld spring", JointType2D.Weld);
        var e = d;
        e.Hertz = linearHertz; e.Damping = linearDampingRatio; e.Hertz2 = angularHertz; e.Damping2 = angularDampingRatio;
        Commit(w, c, ref d, in e);
    }

    /// <summary>The world position a mouse joint pulls its anchor to. Setting it wakes the dragged body.</summary>
    public Vector2D Target
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "target", JointType2D.Mouse); return new Vector2D(d.TargetX, d.TargetY); }
        set { ref var d = ref Rec(out var w, out int c); Need(in d, "target", JointType2D.Mouse); var e = d; e.TargetX = value.X; e.TargetY = value.Y; Commit(w, c, ref d, in e); }
    }

    /// <summary>Mouse joint: largest pulling force, N. Motor joint: largest force, N.</summary>
    public float MaxForce
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "maximum force", JointType2D.Mouse, JointType2D.Motor); return d.MaxMotor; }
        set { ref var d = ref Rec(out var w, out int c); Need(in d, "maximum force", JointType2D.Mouse, JointType2D.Motor); var e = d; e.MaxMotor = value; Commit(w, c, ref d, in e); }
    }

    /// <summary>Motor joint: largest torque, N·m.</summary>
    public float MaxTorque
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "maximum torque", JointType2D.Motor); return d.MaxMotor2; }
        set { ref var d = ref Rec(out var w, out int c); Need(in d, "maximum torque", JointType2D.Motor); var e = d; e.MaxMotor2 = value; Commit(w, c, ref d, in e); }
    }

    /// <summary>Motor joint: the target pose of B relative to A (origin offset in A's frame, and angle).</summary>
    public void SetOffset(Vector2F linear, float angular)
    {
        ref var d = ref Rec(out var w, out int c);
        Need(in d, "offset", JointType2D.Motor);
        var e = d;
        e.OffX = linear.X; e.OffY = linear.Y; e.OffAngle = angular;
        e.Validate();
        e.SetReference(angular);
        Commit(w, c, ref d, in e);
    }

    // ---- breaking and readouts ----

    /// <summary>Reaction force above which the joint breaks at the end of a step, newtons; infinity (the default) never breaks.</summary>
    public float BreakForce
    {
        get { ref var d = ref Rec(out _, out _); return d.BreakForce; }
        set { ref var d = ref Rec(out var w, out int c); var e = d; e.BreakForce = value; Commit(w, c, ref d, in e); }
    }

    /// <summary>Reaction torque above which the joint breaks, N·m; infinity never breaks.</summary>
    public float BreakTorque
    {
        get { ref var d = ref Rec(out _, out _); return d.BreakTorque; }
        set { ref var d = ref Rec(out var w, out int c); var e = d; e.BreakTorque = value; Commit(w, c, ref d, in e); }
    }

    /// <summary>The magnitude of the joint's linear reaction force at the end of the last step it was solved, newtons (0 before the first step, and after a restore until the next step).</summary>
    public float ReactionForce { get { Rec(out var w, out int c); return w.Engine.JointForce[c]; } }

    /// <summary>The magnitude of the joint's reaction torque at the end of the last step it was solved, N·m.</summary>
    public float ReactionTorque { get { Rec(out var w, out int c); return w.Engine.JointTorque[c]; } }

    /// <summary>
    /// Raised after the step in which this joint broke (its reaction exceeded <see cref="BreakForce"/> or <see cref="BreakTorque"/>), in event order, exactly once. A plain delegate; the joint is already destroyed when it runs.
    /// </summary>
    public event Action<JointBreak> OnJointBreak
    {
        add { Subscribe(value); }
        remove { Unsubscribe(value); }
    }

    private void Subscribe(Action<JointBreak> handler)
    {
        ArgumentNullException.ThrowIfNull(handler);
        if (_world == null) { throw new InvalidHandleException("The default Joint is not attached to a world."); }
        _world.AddJointHandler(_handle, handler);
    }

    private void Unsubscribe(Action<JointBreak> handler)
    {
        if (_world == null || handler == null) { return; }
        _world.RemoveJointHandler(_handle, handler);
    }

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(Joint other) => ReferenceEquals(_world, other._world) && _handle == other._handle;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Joint j && Equals(j);

    /// <inheritdoc/>
    public override int GetHashCode() => _handle.GetHashCode();

    /// <summary>Handle equality.</summary>
    public static bool operator ==(Joint a, Joint b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(Joint a, Joint b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _handle.ToString();
}
