using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>What a joint-break callback is told. The joint is already destroyed, so <see cref="Joint"/> is stale and only identifies it (compare handles, or read <see cref="Joint.RawHandle"/>).</summary>
[System.Diagnostics.CodeAnalysis.SuppressMessage("Design", "CA1051:Do not declare visible instance fields", Justification = "Handle fields, not properties, so that `hit.Body.LinearVelocity = v` compiles: a property result is not a variable (CS1612), a readonly field is.")]
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
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
/// A handle to a 3D joint: world, slot and generation. A small struct that allocates nothing; once the joint is destroyed (by <see cref="Destroy"/>, by breaking, or because a body was destroyed)
/// every copy fails with a <see cref="StaleHandleException"/> naming the joint. Properties that a joint type does not have throw <see cref="InvalidJointException"/>.
/// Edits take effect at the next step and wake the joint's bodies. The <c>Add*</c> one-liners and <c>CreateJoint</c> return a typed view (for example <c>HingeJoint</c>) that has only the members its kind has and converts implicitly to <see cref="Joint"/>; use <see cref="Joint"/> to hold joints of any kind in one collection (the explicit conversion back checks the kind).
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct Joint : IEquatable<Joint>
{
    private const uint Hinge = 1u << (int)JointType3D.Hinge, Slider = 1u << (int)JointType3D.Slider, Distance = 1u << (int)JointType3D.Distance, Path = 1u << (int)JointType3D.Path;
    private const uint LimitTypes = Hinge | Slider | Distance, MotorTypes = Hinge | Slider | Path, SpringTypes = Hinge | Slider | Distance;

    private readonly PhysicsWorld3D? _world;
    private readonly JointHandle _handle;

    internal Joint(PhysicsWorld3D world, JointHandle handle)
    {
        _world = world;
        _handle = handle;
    }

    /// <summary>The raw core handle (world id, constraint slot, generation) for the raw tier and for events.</summary>
    public JointHandle RawHandle => _handle;

    /// <summary>True while the joint exists.</summary>
    public bool IsValid => _world != null && _handle.World == _world.Id && _world.Core.Constraints.Handles.IsAlive(_handle.Index, _handle.Generation) && _world.Core.Constraints.KindOf(_handle.Index) == ConstraintKind.Joint;

    private ref JointData3D Rec(out PhysicsWorld3D w, out int c)
    {
        if (_world == null) { throw new InvalidHandleException("The default Joint is not attached to a world."); }
        w = _world;
        c = w.Core.Resolve(_handle);
        w.Engine.BindJoints();
        return ref w.Engine.Joints[c];
    }

    private static void Need(in JointData3D d, string what, uint types)
    {
        if ((types & (1u << d.Type)) != 0) { return; }
        throw new InvalidJointException("A " + JointData3D.Name((JointType3D)d.Type) + " joint has no " + what + ".");
    }

    private static void Commit(PhysicsWorld3D w, int c, ref JointData3D d, in JointData3D edited)
    {
        edited.Validate();
        d = edited;
        var cs = w.Core.Constraints;
        w.Core.Wake(w.Core.HandleOfBody(cs.BodyA[c]));
        w.Core.Wake(w.Core.HandleOfBody(cs.BodyB[c]));
    }

    private static byte Set(byte flags, byte bit, bool on) => on ? (byte)(flags | bit) : (byte)(flags & ~bit);

    /// <summary>The kind of joint.</summary>
    public JointType3D Type { get { ref var d = ref Rec(out _, out _); return (JointType3D)d.Type; } }

    /// <summary>The first body.</summary>
    public Body BodyA { get { Rec(out var w, out int c); return new Body(w, w.Core.HandleOfBody(w.Core.Constraints.BodyA[c])); } }

    /// <summary>The second body.</summary>
    public Body BodyB { get { Rec(out var w, out int c); return new Body(w, w.Core.HandleOfBody(w.Core.Constraints.BodyB[c])); } }

    /// <summary>Whether the two bodies collide with each other (fixed at creation).</summary>
    public bool CollideConnected { get { ref var d = ref Rec(out _, out _); return d.Collide; } }

    /// <summary>Destroys the joint. Every copy of this handle becomes stale; no break event is raised.</summary>
    public void Destroy() { Rec(out var w, out _); w.DestroyJoint(_handle); }

    // ---- limits ----

    /// <summary>Whether the limit is on (hinge, slider, distance).</summary>
    public bool EnableLimit
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "limit", LimitTypes); return d.Limit; }
        set
        {
            ref var d = ref Rec(out var w, out int c);
            Need(in d, "limit", LimitTypes);
            var e = d;
            e.Flags = Set(e.Flags, JointData3D.FlagLimit, value);
            Commit(w, c, ref d, in e);
            if (!value) { var p = w.Core.Constraints.Payload(c); p[JP.Lower] = 0f; p[JP.Upper] = 0f; }
        }
    }

    private float LimitValue(bool upper, string what, uint types)
    {
        ref var d = ref Rec(out _, out _);
        Need(in d, what, types);
        return upper ? d.Upper : d.Lower;
    }

    /// <summary>Hinge joints: the lower angle limit, radians.</summary>
    /// <exception cref="InvalidJointException">The joint is not a hinge.</exception>
    public float LowerAngle => LimitValue(false, "angle limit", Hinge);

    /// <summary>Hinge joints: the upper angle limit, radians.</summary>
    /// <exception cref="InvalidJointException">The joint is not a hinge.</exception>
    public float UpperAngle => LimitValue(true, "angle limit", Hinge);

    /// <summary>Slider joints: the lower translation limit along the axis, metres.</summary>
    /// <exception cref="InvalidJointException">The joint is not a slider.</exception>
    public float LowerTranslation => LimitValue(false, "translation limit", Slider);

    /// <summary>Slider joints: the upper translation limit along the axis, metres.</summary>
    /// <exception cref="InvalidJointException">The joint is not a slider.</exception>
    public float UpperTranslation => LimitValue(true, "translation limit", Slider);

    /// <summary>Distance joints: the shortest length the limit allows, metres.</summary>
    /// <exception cref="InvalidJointException">The joint is not a distance joint.</exception>
    public float MinLength => LimitValue(false, "length limit", Distance);

    /// <summary>Distance joints: the longest length the limit allows, metres.</summary>
    /// <exception cref="InvalidJointException">The joint is not a distance joint.</exception>
    public float MaxLength => LimitValue(true, "length limit", Distance);

    /// <summary>Sets both limits without turning the limit on: angles in radians within -pi..pi (hinge), translations in metres (slider) or lengths in metres, at least 0 (distance); lower at most upper.</summary>
    /// <exception cref="InvalidJointException">The limits are the wrong way round, or the joint has none.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or out of range.</exception>
    public void SetLimits(float lower, float upper)
    {
        ref var d = ref Rec(out var w, out int c);
        Need(in d, "limit", LimitTypes);
        var e = d;
        e.Lower = lower; e.Upper = upper;
        bool wasLimit = e.Limit;
        e.Flags |= JointData3D.FlagLimit; // validate as an active limit
        e.Validate();
        if (!wasLimit) { e.Flags = (byte)(e.Flags & ~JointData3D.FlagLimit); }
        Commit(w, c, ref d, in e);
    }

    // ---- motor ----

    /// <summary>Whether the motor is on (hinge, slider, path).</summary>
    public bool EnableMotor
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "motor", MotorTypes); return d.Motor; }
        set
        {
            ref var d = ref Rec(out var w, out int c);
            Need(in d, "motor", MotorTypes);
            var e = d;
            e.Flags = Set(e.Flags, JointData3D.FlagMotor, value);
            Commit(w, c, ref d, in e);
            if (!value) { w.Core.Constraints.Payload(c)[JP.Motor] = 0f; }
        }
    }

    /// <summary>Target speed: radians per second (hinge) or metres per second (slider, path).</summary>
    public float MotorSpeed
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "motor", MotorTypes); return d.MotorSpeed; }
        set { ref var d = ref Rec(out var w, out int c); Need(in d, "motor", MotorTypes); var e = d; e.MotorSpeed = value; Commit(w, c, ref d, in e); }
    }

    /// <summary>Largest motor force in newtons of a slider or path joint, at least 0. A hinge's motor takes a torque: <see cref="MaxMotorTorque"/>.</summary>
    /// <exception cref="InvalidJointException">The joint's motor is rotational (hinge) or the joint has none.</exception>
    public float MaxMotorForce
    {
        get { ref var d = ref Rec(out _, out _); NeedForceMotor(in d); return d.MaxMotor; }
        set { ref var d = ref Rec(out var w, out int c); NeedForceMotor(in d); var e = d; e.MaxMotor = value; Commit(w, c, ref d, in e); }
    }

    /// <summary>Largest motor torque in N·m of a hinge, at least 0. Linear motors take a force: <see cref="MaxMotorForce"/>.</summary>
    /// <exception cref="InvalidJointException">The joint is not a hinge.</exception>
    public float MaxMotorTorque
    {
        get { ref var d = ref Rec(out _, out _); NeedTorqueMotor(in d); return d.MaxMotor; }
        set { ref var d = ref Rec(out var w, out int c); NeedTorqueMotor(in d); var e = d; e.MaxMotor = value; Commit(w, c, ref d, in e); }
    }

    private static void NeedForceMotor(in JointData3D d)
    {
        if (d.Type == (byte)JointType3D.Hinge) { throw new InvalidJointException("A hinge joint's motor is rotational and takes a torque: use MaxMotorTorque."); }
        Need(in d, "motor force", Slider | Path);
    }

    private static void NeedTorqueMotor(in JointData3D d)
    {
        if (d.Type == (byte)JointType3D.Slider || d.Type == (byte)JointType3D.Path) { throw new InvalidJointException("A " + JointData3D.Name((JointType3D)d.Type) + " joint's motor is linear and takes a force: use MaxMotorForce."); }
        Need(in d, "motor torque", Hinge);
    }

    // ---- spring ----

    /// <summary>Whether the spring is on (hinge, slider, distance).</summary>
    public bool EnableSpring
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "spring", SpringTypes); return d.Spring; }
        set
        {
            ref var d = ref Rec(out var w, out int c);
            Need(in d, "spring", SpringTypes);
            var e = d;
            e.Flags = Set(e.Flags, JointData3D.FlagSpring, value);
            Commit(w, c, ref d, in e);
            if (!value) { var p = w.Core.Constraints.Payload(c); p[JP.Spring] = 0f; if (e.Type == (byte)JointType3D.Distance) { p[0] = 0f; } }
        }
    }

    /// <summary>Spring frequency in Hz, at least 0 (0 means no spring force).</summary>
    public float SpringHertz
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "spring", SpringTypes); return d.Hertz; }
        set { ref var d = ref Rec(out var w, out int c); Need(in d, "spring", SpringTypes); var e = d; e.Hertz = value; Commit(w, c, ref d, in e); }
    }

    /// <summary>Spring damping ratio, at least 0 (1 is critical).</summary>
    public float SpringDampingRatio
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "spring", SpringTypes); return d.Damping; }
        set { ref var d = ref Rec(out var w, out int c); Need(in d, "spring", SpringTypes); var e = d; e.Damping = value; Commit(w, c, ref d, in e); }
    }

    // ---- type specific ----

    /// <summary>Rest length of a distance joint, metres, at least 0.005.</summary>
    public float Length
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "length", Distance); return d.Length; }
        set { ref var d = ref Rec(out var w, out int c); Need(in d, "length", Distance); var e = d; e.Length = value; Commit(w, c, ref d, in e); }
    }

    /// <summary>Ratio of a gear, rack-and-pinion or pulley joint.</summary>
    public float Ratio
    {
        get { ref var d = ref Rec(out _, out _); NeedRatio(in d); return d.Ratio; }
        set { ref var d = ref Rec(out var w, out int c); NeedRatio(in d); var e = d; e.Ratio = value; Commit(w, c, ref d, in e); }
    }

    private static void NeedRatio(in JointData3D d) => Need(in d, "ratio", (1u << (int)JointType3D.Gear) | (1u << (int)JointType3D.RackAndPinion) | (1u << (int)JointType3D.Pulley));

    /// <summary>Cone joint: the largest angle between the two x axes, radians within (0, pi].</summary>
    public float HalfConeAngle
    {
        get { ref var d = ref Rec(out _, out _); Need(in d, "cone angle", 1u << (int)JointType3D.Cone); return d.Swing1; }
        set { ref var d = ref Rec(out var w, out int c); Need(in d, "cone angle", 1u << (int)JointType3D.Cone); var e = d; e.Swing1 = value; Commit(w, c, ref d, in e); }
    }

    /// <summary>Swing-twist joint: sets the twist range (radians within -pi..pi) and the swing half angles about the frame y and z axes (within (0, pi]).</summary>
    public void SetSwingTwistLimits(float lowerTwist, float upperTwist, float swingYHalfAngle, float swingZHalfAngle)
    {
        ref var d = ref Rec(out var w, out int c);
        Need(in d, "swing and twist limits", 1u << (int)JointType3D.SwingTwist);
        var e = d;
        e.Lower = lowerTwist; e.Upper = upperTwist; e.Swing1 = swingYHalfAngle; e.Swing2 = swingZHalfAngle;
        Commit(w, c, ref d, in e);
    }

    private static int AxisIndex(JointAxes axis, string what)
    {
        for (int i = 0; i < 6; i++) { if (axis == (JointAxes)(1 << i)) { return i; } }
        throw new ArgumentOutOfRangeException(what, axis, what + " must name exactly one axis.");
    }

    /// <summary>Six-dof joint: sets how one axis is treated. Clears the accumulated impulses of that axis.</summary>
    public void SetAxisMode(JointAxes axis, JointAxisMode mode)
    {
        int i = AxisIndex(axis, nameof(axis));
        ref var d = ref Rec(out var w, out int c);
        Need(in d, "axes", 1u << (int)JointType3D.SixDof);
        var e = d;
        e.Six.Mode[i] = (byte)mode;
        Commit(w, c, ref d, in e);
        ClearSix(w, c);
    }

    private static void ClearSix(PhysicsWorld3D w, int c) => w.Core.Constraints.Payload(c).Slice(0, JP.SixAxis + 24).Clear();

    /// <summary>Six-dof joint: sets the limits of one axis (metres or radians; used while the axis is limited).</summary>
    public void SetAxisLimits(JointAxes axis, float lower, float upper)
    {
        int i = AxisIndex(axis, nameof(axis));
        ref var d = ref Rec(out var w, out int c);
        Need(in d, "axes", 1u << (int)JointType3D.SixDof);
        var e = d;
        e.Six.Lo[i] = lower; e.Six.Hi[i] = upper;
        bool limited = e.Six.Mode[i] == 1;
        if (!limited) { e.Six.Mode[i] = 1; }
        e.Validate();
        if (!limited) { e.Six.Mode[i] = d.Six.Mode[i]; }
        Commit(w, c, ref d, in e);
    }

    /// <summary>Six-dof joint: turns the motor of one axis on or off and sets its target speed (m/s or rad/s) and largest force or torque.</summary>
    public void SetAxisMotor(JointAxes axis, bool enabled, float speed = 0f, float maxForceOrTorque = 0f)
    {
        int i = AxisIndex(axis, nameof(axis));
        ref var d = ref Rec(out var w, out int c);
        Need(in d, "axes", 1u << (int)JointType3D.SixDof);
        var e = d;
        e.Six.MotorMask = (byte)(enabled ? e.Six.MotorMask | (1 << i) : e.Six.MotorMask & ~(1 << i));
        e.Six.MotorSpeed[i] = speed; e.Six.MaxMotor[i] = maxForceOrTorque;
        Commit(w, c, ref d, in e);
        if (!enabled) { w.Core.Constraints.Payload(c)[JP.SixAxis + 4 * i] = 0f; }
    }

    /// <summary>Six-dof joint: turns the spring of one axis on or off (towards zero translation or angle) and sets its frequency (Hz) and damping ratio.</summary>
    public void SetAxisSpring(JointAxes axis, bool enabled, float hertz = 0f, float dampingRatio = 0.5f)
    {
        int i = AxisIndex(axis, nameof(axis));
        ref var d = ref Rec(out var w, out int c);
        Need(in d, "axes", 1u << (int)JointType3D.SixDof);
        var e = d;
        e.Six.SpringMask = (byte)(enabled ? e.Six.SpringMask | (1 << i) : e.Six.SpringMask & ~(1 << i));
        e.Six.Hertz[i] = hertz; e.Six.Damping[i] = dampingRatio;
        Commit(w, c, ref d, in e);
        if (!enabled) { w.Core.Constraints.Payload(c)[JP.SixAxis + 4 * i + 1] = 0f; }
    }

    /// <summary>
    /// Swing-twist joint: drives frame B towards a rotation relative to frame A with an angular spring-damper of absolute stiffness and damping and a torque limit (the joint of a powered ragdoll). The torque is
    /// <c>-stiffness · error - damping · relative angular velocity</c>, integrated implicitly over the substep, so any stiffness is stable, and capped at <paramref name="maxTorque"/>. The drive acts beside the joint's limits, which still hold.
    /// </summary>
    /// <param name="target">Wanted rotation of frame B relative to frame A (a unit quaternion; it is normalized). The identity is the pose in which the two frames coincide.</param>
    /// <param name="stiffness">Torque per radian of error, N·m/rad, finite and at least 0.</param>
    /// <param name="damping">Torque per rad/s of relative angular velocity, N·m·s/rad, finite and at least 0.</param>
    /// <param name="maxTorque">Largest torque the drive may apply, N·m, finite and at least 0. 0 makes the joint limp (the drive is removed).</param>
    /// <exception cref="InvalidJointException">The joint is not a swing-twist joint.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or out of range; the joint is unchanged.</exception>
    public void SetPoseDrive(QuaternionF target, float stiffness, float damping, float maxTorque)
    {
        ref var d = ref Rec(out var w, out int c);
        Need(in d, "pose drive", 1u << (int)JointType3D.SwingTwist);
        var e = d;
        e.Flags |= JointData3D.FlagDrive;
        e.DriveTarget = target; e.Length = stiffness; e.Ratio = damping; e.MaxMotor = maxTorque;
        e.Validate();
        if (!(maxTorque > 0f)) { ClearPoseDrive(); return; }
        e.DriveTarget = Engine3D.Normalize(target, "target");
        if (d.Drive && SameQuaternion(d.DriveTarget, e.DriveTarget) && d.Length == e.Length && d.Ratio == e.Ratio && d.MaxMotor == e.MaxMotor) { return; }
        Commit(w, c, ref d, in e);
    }

    private static bool SameQuaternion(QuaternionF a, QuaternionF b) => a.X == b.X && a.Y == b.Y && a.Z == b.Z && a.W == b.W;

    /// <summary>Swing-twist joint: removes the pose drive (the joint is limp inside its limits). Does nothing when there is none.</summary>
    public void ClearPoseDrive()
    {
        ref var d = ref Rec(out var w, out int c);
        Need(in d, "pose drive", 1u << (int)JointType3D.SwingTwist);
        if (!d.Drive) { return; }
        var e = d;
        e.Flags = (byte)(e.Flags & ~JointData3D.FlagDrive);
        e.DriveTarget = default; e.Length = 0f; e.Ratio = 0f; e.MaxMotor = 0f;
        Commit(w, c, ref d, in e);
        var p = w.Core.Constraints.Payload(c);
        p[JP.Drive] = 0f; p[JP.Drive + 1] = 0f; p[JP.Drive + 2] = 0f;
    }

    /// <summary>True when the joint has a pose drive (<see cref="SetPoseDrive"/>).</summary>
    public bool IsPoseDriven { get { ref var d = ref Rec(out _, out _); return d.Drive; } }

    /// <summary>The drive's target rotation of frame B relative to frame A; the identity when there is no drive.</summary>
    public QuaternionF PoseDriveTarget { get { ref var d = ref Rec(out _, out _); return d.Drive ? d.DriveTarget : QuaternionF.Identity; } }

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
