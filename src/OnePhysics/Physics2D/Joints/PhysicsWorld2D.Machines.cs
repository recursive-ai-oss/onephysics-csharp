using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    private Joint MakeMachine(Body a, Body b, JointData2D d, MachineData2D machine)
    {
        machine.Validate((JointType2D)d.Type);
        var joint = Make(a, b, d);
        _engine.SetMachine(joint.RawHandle.Index, machine);
        return joint;
    }
    private static JointData2D MachineBase(JointType2D type, bool collide, float force, float torque, int lifetime) => new()
    { Type = (byte)type, Flags = Flags(false, false, false, collide), BreakForce = force, BreakTorque = torque, LifetimeSteps = lifetime };

    /// <summary>Creates angular coupling omegaA + ratio * omegaB = 0. Invalid bodies or a zero/non-finite ratio throw before mutation.</summary>
    public Joint CreateJoint(in GearJointDef def)
    {
        using var mutation = RecordMutation("PhysicsWorld2D.CreateJoint");
        var d = MachineBase(JointType2D.Gear, def.CollideConnected, def.BreakForce, def.BreakTorque, def.LifetimeSteps);
        d.OffAngle = def.Ratio;
        return Make(def.BodyA, def.BodyB, d);
    }

    /// <summary>Creates rack coupling ratio * omegaA = velocityB along B's axis. Ratio is metres/radian. Invalid bodies, ratio or axis throw before mutation.</summary>
    public Joint CreateJoint(in RackAndPinionJointDef def)
    {
        using var mutation = RecordMutation("PhysicsWorld2D.CreateJoint");
        var d = MachineBase(JointType2D.RackAndPinion, def.CollideConnected, def.BreakForce, def.BreakTorque, def.LifetimeSteps);
        d.OffAngle = def.Ratio; d.AxX = def.LocalAxisB.X; d.AxY = def.LocalAxisB.Y;
        return Make(def.BodyA, def.BodyB, d);
    }

    /// <summary>Creates a pulley conserving lengthA + ratio * lengthB. Length zero derives from the initial anchors. Invalid bodies, points, length or nonpositive ratio throw.</summary>
    public Joint CreateJoint(in PulleyJointDef def)
    {
        using var mutation = RecordMutation("PhysicsWorld2D.CreateJoint");
        var d = MachineBase(JointType2D.Pulley, def.CollideConnected, def.BreakForce, def.BreakTorque, def.LifetimeSteps);
        var m = new MachineData2D { FixedA = def.FixedPointA, FixedB = def.FixedPointB };
        d.LAx = def.LocalAnchorA.X; d.LAy = def.LocalAnchorA.Y; d.LBx = def.LocalAnchorB.X; d.LBy = def.LocalAnchorB.Y; d.OffAngle = def.Ratio; d.Length = def.Length;
        if (d.Length == 0f)
        {
            var ra = def.BodyA.Rotation.Rotate(new Vec2(d.LAx, d.LAy)); var rb = def.BodyB.Rotation.Rotate(new Vec2(d.LBx, d.LBy));
            var pa = def.BodyA.Position + new Vector2D(ra.X, ra.Y); var pb = def.BodyB.Position + new Vector2D(rb.X, rb.Y);
            d.Length = (float)((pa - m.FixedA).Length() + def.Ratio * (pb - m.FixedB).Length());
        }
        if (!float.IsFinite(def.MinLength) || !float.IsFinite(def.MaxLength) || def.MinLength < -1f || def.MaxLength < -1f) { throw new ArgumentOutOfRangeException(nameof(def), "Pulley limits must be finite, nonnegative or -1 for derived length."); }
        d.Lower = def.MinLength < 0f ? d.Length : def.MinLength; d.Upper = def.MaxLength < 0f ? d.Length : def.MaxLength;
        return MakeMachine(def.BodyA, def.BodyB, d, m);
    }

    /// <summary>Creates a path joint; copies 2..256 finite local points. Invalid bodies, points or motor settings throw before mutation.</summary>
    public Joint CreateJoint(in PathJointDef def)
    {
        using var mutation = RecordMutation("PhysicsWorld2D.CreateJoint");
        ArgumentNullException.ThrowIfNull(def.Points);
        var d = MachineBase(JointType2D.Path, def.CollideConnected, def.BreakForce, def.BreakTorque, def.LifetimeSteps);
        d.Flags = Flags(false, def.EnableMotor, false, def.CollideConnected); d.LBx = def.LocalAnchorB.X; d.LBy = def.LocalAnchorB.Y;
        d.MotorSpeed = def.MotorSpeed; d.MaxMotor = def.MaxMotorForce;
        return MakeMachine(def.BodyA, def.BodyB, d, new MachineData2D { Points = (Vector2F[])def.Points.Clone(), Loop = def.Loop, Align = def.AlignTangent });
    }

    /// <summary>Creates a configurable joint, with x, y and angle axes. Invalid bodies, anchors, modes or axis settings throw before mutation.</summary>
    public Joint CreateJoint(in ConfigurableJointDef def)
    {
        using var mutation = RecordMutation("PhysicsWorld2D.CreateJoint");
        var d = MachineBase(JointType2D.Configurable, def.CollideConnected, def.BreakForce, def.BreakTorque, def.LifetimeSteps);
        d.LAx = def.LocalAnchorA.X; d.LAy = def.LocalAnchorA.Y; d.LBx = def.LocalAnchorB.X; d.LBy = def.LocalAnchorB.Y;
        d.RefAngle = def.ReferenceAngle; d.OffAngle = def.LocalFrameAngleA;
        return MakeMachine(def.BodyA, def.BodyB, d, new MachineData2D { X = def.X, Y = def.Y, Angle = def.Angle });
    }

    /// <summary>Couples two angular speeds; ratio defaults to one, positive means opposite rotation. Invalid bodies or ratio throw.</summary>
    public Joint AddGear(Body a, Body b, float ratio = 1f, bool collideConnected = false)
        => CreateJoint(new GearJointDef { BodyA = a, BodyB = b, Ratio = ratio, CollideConnected = collideConnected });

    /// <summary>Couples pinion rotation to rack translation along a finite nonzero world direction; ratio defaults to one metre/radian. Invalid bodies or parameters throw.</summary>
    public Joint AddRackAndPinion(Body pinion, Body rack, Vector2F rackAxis, float ratio = 1f, bool collideConnected = false)
        => CreateJoint(new RackAndPinionJointDef { BodyA = pinion, BodyB = rack, LocalAxisB = DirToLocal(rack, rackAxis, nameof(rackAxis)), Ratio = ratio, CollideConnected = collideConnected });

    /// <summary>Conserves ratio-weighted rope length derived from initial world anchors and fixed points. Ratio defaults to one and must be positive; points must be finite within +-2^21.</summary>
    public Joint AddPulley(Body a, Body b, Vector2D anchorA, Vector2D anchorB, Vector2D fixedA, Vector2D fixedB, float ratio = 1f, bool collideConnected = false)
        => CreateJoint(new PulleyJointDef { BodyA = a, BodyB = b, LocalAnchorA = ToLocal(a, anchorA, nameof(anchorA)), LocalAnchorB = ToLocal(b, anchorB, nameof(anchorB)), FixedPointA = fixedA, FixedPointB = fixedB, Ratio = ratio, CollideConnected = collideConnected });

    /// <summary>Follows a spline through 2..256 finite world points within +-2^21, attached to pathBody. Points are copied. Rotation is free; open endpoints clamp travel.</summary>
    public Joint AddPath(Body pathBody, Body follower, ReadOnlySpan<Vector2D> worldPoints, bool loop = false, bool collideConnected = false)
    {
        if (worldPoints.Length < 2 || worldPoints.Length > 256) { throw new ArgumentOutOfRangeException(nameof(worldPoints), worldPoints.Length, "Path requires 2..256 points."); }
        var points = new Vector2F[worldPoints.Length];
        for (int i = 0; i < points.Length; i++) { points[i] = ToLocal(pathBody, worldPoints[i], nameof(worldPoints)); }
        return CreateJoint(new PathJointDef { BodyA = pathBody, BodyB = follower, Points = points, Loop = loop, CollideConnected = collideConnected });
    }

    /// <summary>Locks x, y and angle at the current pose through a world anchor. Use SetAxis to free, limit or drive each axis. Invalid bodies or anchor throw.</summary>
    public Joint AddConfigurable(Body a, Body b, Vector2D anchor, bool collideConnected = false)
        => CreateJoint(new ConfigurableJointDef { BodyA = a, BodyB = b, LocalAnchorA = ToLocal(a, anchor, nameof(anchor)), LocalAnchorB = ToLocal(b, anchor, nameof(anchor)), ReferenceAngle = RelativeAngle(a, b), CollideConnected = collideConnected });

    /// <summary>The planar cone analogue: a hinge with symmetric relative angle limits about its initial pose, default half angle pi/4. Invalid bodies, anchor or half angle outside (0,pi] throw.</summary>
    public HingeJoint AddCone(Body a, Body b, Vector2D anchor, float halfAngle = 0.7853982f, bool collideConnected = false)
    {
        if (!(halfAngle > 0f && halfAngle <= 3.1415927f)) { throw new ArgumentOutOfRangeException(nameof(halfAngle), halfAngle, "Half angle must lie in (0,pi]."); }
        return CreateJoint(new RevoluteJointDef { BodyA = a, BodyB = b, LocalAnchorA = ToLocal(a, anchor, nameof(anchor)), LocalAnchorB = ToLocal(b, anchor, nameof(anchor)), ReferenceAngle = RelativeAngle(a, b), EnableLimit = true, LowerAngle = -halfAngle, UpperAngle = halfAngle, CollideConnected = collideConnected });
    }
    /// <summary>Drives B to a pose relative to A, using finite local metres/radians and nonnegative finite force/torque caps. Invalid inputs throw.</summary>
    public RelativeJoint AddMotor(Body a, Body b, Vector2F linearOffset, float angularOffset, float maxForce, float maxTorque)
        => CreateJoint(new MotorJointDef { BodyA = a, BodyB = b, LinearOffset = linearOffset, AngularOffset = angularOffset, MaxForce = maxForce, MaxTorque = maxTorque });

    /// <summary>Pulls B's origin to a finite world target within +-2^21, with a finite nonnegative force cap. Defaults: 5 Hz, damping ratio 0.7.</summary>
    public TargetJoint AddMouse(Body a, Body b, Vector2D target, float maxForce)
        => CreateJoint(new MouseJointDef { BodyA = a, BodyB = b, Target = target, MaxForce = maxForce, Hertz = 5f, DampingRatio = 0.7f });

    /// <summary>Planar ball-and-socket analogue: holds the two anchors together and leaves rotation free. Invalid bodies or world anchor throw.</summary>
    public HingeJoint AddPoint(Body a, Body b, Vector2D anchor, bool collideConnected = false) => AddHinge(a, b, anchor, collideConnected);

}

public readonly partial struct Joint
{
    /// <summary>Gear, rack or pulley ratio. Finite and nonzero, positive for pulleys. Other types or invalid values throw.</summary>
    public float Ratio
    {
        get { ref var d = ref Rec(out _, out _); Need(d, "ratio", JointType2D.Gear, JointType2D.RackAndPinion, JointType2D.Pulley); return d.OffAngle; }
        set { using var mutation = _world?.RecordMutation("Joint.Ratio"); ref var d = ref Rec(out var w, out int c); Need(d, "ratio", JointType2D.Gear, JointType2D.RackAndPinion, JointType2D.Pulley); var e = d; e.OffAngle = value; Commit(w, c, ref d, e); }
    }

    private JointAxisSettings AxisSettings(JointAxes axis, out int index)
    {
        index = axis switch { JointAxes.LinearX => 0, JointAxes.LinearY => 1, JointAxes.AngularZ => 2, _ => throw new ArgumentOutOfRangeException(nameof(axis), axis, "Select exactly one of LinearX, LinearY or AngularZ.") };
        ref var d = ref Rec(out var w, out int c); Need(d, "axis", JointType2D.Configurable);
        var m = w.Engine.Machines[c]!;
        return index == 0 ? m.X : index == 1 ? m.Y : m.Angle;
    }

    /// <summary>Sets one configurable axis to locked, limited or free, clears its warm start and wakes the bodies. Invalid axis, mode or joint type throws before mutation.</summary>
    public void SetAxisMode(JointAxes axis, JointAxisMode mode)
    {
        using var mutation = _world?.RecordMutation("Joint.SetAxisMode");
        var settings = AxisSettings(axis, out int index); settings.Mode = mode; SetAxis(index, settings);
    }

    /// <summary>Sets limits for one configurable axis, metres or radians; does not turn limits on. Limits must be finite and ordered, angular limits within -pi..pi. Invalid input throws before mutation.</summary>
    public void SetAxisLimits(JointAxes axis, float lower, float upper)
    {
        using var mutation = _world?.RecordMutation("Joint.SetAxisLimits");
        var settings = AxisSettings(axis, out int index); settings.Lower = lower; settings.Upper = upper; SetAxis(index, settings);
    }

    /// <summary>Sets one configurable axis's speed motor, target in metres/s or rad/s and nonnegative force/torque cap in N or N m. Default speed/cap zero. Non-finite values or invalid axis/type throw before mutation.</summary>
    public void SetAxisMotor(JointAxes axis, bool enabled, float speed = 0f, float maxForceOrTorque = 0f)
    {
        using var mutation = _world?.RecordMutation("Joint.SetAxisMotor");
        var settings = AxisSettings(axis, out int index); settings.EnableMotor = enabled; settings.MotorSpeed = speed; settings.MaxMotor = maxForceOrTorque; SetAxis(index, settings);
    }

    /// <summary>Sets one configurable axis's spring to zero; frequency in Hz and damping ratio must be finite and nonnegative. Defaults 0 Hz, damping 0.5. Invalid axis/type or values throw before mutation.</summary>
    public void SetAxisSpring(JointAxes axis, bool enabled, float hertz = 0f, float dampingRatio = 0.5f)
    {
        using var mutation = _world?.RecordMutation("Joint.SetAxisSpring");
        var settings = AxisSettings(axis, out int index); settings.EnableSpring = enabled; settings.SpringHertz = hertz; settings.SpringDampingRatio = dampingRatio; SetAxis(index, settings);
    }

    /// <summary>Replaces one configurable axis (0=x, 1=y, 2=angle), clears its warm-start slots and wakes the bodies. Invalid index, joint type or settings throw before mutation.</summary>
    public void SetAxis(int axis, JointAxisSettings settings)
    {
        using var mutation = _world?.RecordMutation("Joint.SetAxis");
        ref var d = ref Rec(out var w, out int c); Need(d, "axis", JointType2D.Configurable);
        if ((uint)axis > 2) { throw new ArgumentOutOfRangeException(nameof(axis), axis, "Axis must be 0, 1 or 2."); }
        MachineData2D.ValidateAxis(settings, axis == 2);
        var m = w.Engine.Machines[c]!;
        if (axis == 0) { m.X = settings; } else if (axis == 1) { m.Y = settings; } else { m.Angle = settings; }
        w.Core.Constraints.Payload(c).Slice(5 * axis, 5).Clear();
        var edited = d; Commit(w, c, ref d, edited);
    }
}
