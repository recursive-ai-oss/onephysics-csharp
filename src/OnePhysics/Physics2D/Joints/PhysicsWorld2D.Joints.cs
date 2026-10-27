using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Collision2D;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    private sealed class JointHandlers
    {
        public int Generation;
        public Action<JointBreak>? Break;
    }

    private readonly Dictionary<int, JointHandlers> _jointHandlers = new();

    // ------------------------------------------------------------------ break events

    internal void AddJointHandler(JointHandle h, Action<JointBreak> handler)
    {
        _core.Resolve(h);
        if (!_jointHandlers.TryGetValue(h.Index, out var set) || set.Generation != h.Generation)
        {
            set = new JointHandlers { Generation = h.Generation };
            _jointHandlers[h.Index] = set;
        }

        set.Break += handler;
    }

    internal void RemoveJointHandler(JointHandle h, Action<JointBreak> handler)
    {
        if (!_jointHandlers.TryGetValue(h.Index, out var set) || set.Generation != h.Generation) { return; }
        set.Break -= handler;
        if (set.Break == null) { _jointHandlers.Remove(h.Index); }
    }

    /// <summary>Called from the one delivery loop, so a break handler runs at the event's place in the sorted order, between the contact and sleep events around it.</summary>
    private void FireJointBreak(in PhysicsEvent e)
    {
        if (!_jointHandlers.TryGetValue(e.Joint.Index, out var h) || h.Generation != e.Joint.Generation) { return; }
        var action = h.Break;
        _jointHandlers.Remove(e.Joint.Index);
        action?.Invoke(new JointBreak(new Joint(this, e.Joint), new Body(this, e.BodyA), new Body(this, e.BodyB), e.Value));
    }

    internal void DestroyJoint(JointHandle h)
    {
        _core.DestroyJoint(h);
        _jointHandlers.Remove(h.Index);
        _engine.ReconcileJoints();
    }

    /// <summary>The joint for a raw handle (from an event or the raw tier).</summary>
    /// <exception cref="InvalidHandleException">The handle is null, belongs to another world, refers to a destroyed joint or to a constraint that is not a joint.</exception>
    public Joint GetJoint(JointHandle handle)
    {
        _core.Resolve(handle);
        return new Joint(this, handle);
    }

    /// <summary>Number of live joints.</summary>
    public int JointCount
    {
        get
        {
            var cs = _core.Constraints;
            int n = 0;
            for (int c = 0; c < cs.HighWater; c++) { if (cs.IsLive(c) && cs.KindOf(c) == ConstraintKind.Joint) { n++; } }
            return n;
        }
    }

    // ------------------------------------------------------------------ middle tier: definitions

    private BodyHandle H(Body b, string field)
    {
        if (b.World == null) { throw new InvalidHandleException(field + " is the default Body; give the joint two bodies of this world."); }
        if (b.World != this) { throw new ForeignHandleException(field + " " + b.Handle + " belongs to another world."); }
        return b.Handle;
    }

    private Joint Make(Body a, Body b, JointData2D d)
    {
        var h = _engine.CreateJoint(H(a, "BodyA"), H(b, "BodyB"), d);
        return new Joint(this, h);
    }

    private static byte Flags(bool limit, bool motor, bool spring, bool collide) =>
        (byte)((limit ? JointData2D.FlagLimit : 0) | (motor ? JointData2D.FlagMotor : 0) | (spring ? JointData2D.FlagSpring : 0) | (collide ? JointData2D.FlagCollide : 0));

    /// <summary>Creates a revolute (hinge) joint.</summary>
    /// <exception cref="InvalidJointException">Same body twice, no dynamic body, or limits the wrong way round.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or out of range (the message names it).</exception>
    /// <exception cref="InvalidHandleException">A body is the default value, destroyed (<see cref="StaleHandleException"/>) or from another world.</exception>
    public Joint CreateJoint(in RevoluteJointDef def) => Make(def.BodyA, def.BodyB, new JointData2D
    {
        Type = (byte)JointType2D.Revolute, Flags = Flags(def.EnableLimit, def.EnableMotor, def.EnableSpring, def.CollideConnected),
        LAx = def.LocalAnchorA.X, LAy = def.LocalAnchorA.Y, LBx = def.LocalAnchorB.X, LBy = def.LocalAnchorB.Y, RefAngle = def.ReferenceAngle,
        Lower = def.LowerAngle, Upper = def.UpperAngle, MotorSpeed = def.MotorSpeed, MaxMotor = def.MaxMotorTorque,
        Hertz = def.SpringHertz, Damping = def.SpringDampingRatio, BreakForce = def.BreakForce, BreakTorque = def.BreakTorque,
    });

    /// <summary>Creates a prismatic (slider) joint. Throws as <see cref="CreateJoint(in RevoluteJointDef)"/>, and <see cref="InvalidJointException"/> for a zero axis.</summary>
    public Joint CreateJoint(in PrismaticJointDef def) => Make(def.BodyA, def.BodyB, new JointData2D
    {
        Type = (byte)JointType2D.Prismatic, Flags = Flags(def.EnableLimit, def.EnableMotor, def.EnableSpring, def.CollideConnected),
        LAx = def.LocalAnchorA.X, LAy = def.LocalAnchorA.Y, LBx = def.LocalAnchorB.X, LBy = def.LocalAnchorB.Y, AxX = def.LocalAxisA.X, AxY = def.LocalAxisA.Y, RefAngle = def.ReferenceAngle,
        Lower = def.LowerTranslation, Upper = def.UpperTranslation, MotorSpeed = def.MotorSpeed, MaxMotor = def.MaxMotorForce,
        Hertz = def.SpringHertz, Damping = def.SpringDampingRatio, BreakForce = def.BreakForce, BreakTorque = def.BreakTorque,
    });

    /// <summary>Creates a weld joint. Throws as <see cref="CreateJoint(in RevoluteJointDef)"/>.</summary>
    public Joint CreateJoint(in WeldJointDef def) => Make(def.BodyA, def.BodyB, new JointData2D
    {
        Type = (byte)JointType2D.Weld, Flags = Flags(false, false, false, def.CollideConnected),
        LAx = def.LocalAnchorA.X, LAy = def.LocalAnchorA.Y, LBx = def.LocalAnchorB.X, LBy = def.LocalAnchorB.Y, RefAngle = def.ReferenceAngle,
        Hertz = def.LinearHertz, Damping = def.LinearDampingRatio, Hertz2 = def.AngularHertz, Damping2 = def.AngularDampingRatio,
        BreakForce = def.BreakForce, BreakTorque = def.BreakTorque,
    });

    /// <summary>Creates a wheel joint. Throws as <see cref="CreateJoint(in PrismaticJointDef)"/>.</summary>
    public Joint CreateJoint(in WheelJointDef def) => Make(def.BodyA, def.BodyB, new JointData2D
    {
        Type = (byte)JointType2D.Wheel, Flags = Flags(def.EnableLimit, def.EnableMotor, def.EnableSpring, def.CollideConnected),
        LAx = def.LocalAnchorA.X, LAy = def.LocalAnchorA.Y, LBx = def.LocalAnchorB.X, LBy = def.LocalAnchorB.Y, AxX = def.LocalAxisA.X, AxY = def.LocalAxisA.Y,
        Lower = def.LowerTranslation, Upper = def.UpperTranslation, MotorSpeed = def.MotorSpeed, MaxMotor = def.MaxMotorTorque,
        Hertz = def.SpringHertz, Damping = def.SpringDampingRatio, BreakForce = def.BreakForce, BreakTorque = def.BreakTorque,
    });

    /// <summary>Creates a distance joint. Throws as <see cref="CreateJoint(in RevoluteJointDef)"/>.</summary>
    public Joint CreateJoint(in DistanceJointDef def) => Make(def.BodyA, def.BodyB, new JointData2D
    {
        Type = (byte)JointType2D.Distance, Flags = Flags(def.EnableLimit, def.EnableMotor, def.EnableSpring, def.CollideConnected),
        LAx = def.LocalAnchorA.X, LAy = def.LocalAnchorA.Y, LBx = def.LocalAnchorB.X, LBy = def.LocalAnchorB.Y, Length = def.Length,
        Lower = def.MinLength, Upper = def.MaxLength, MotorSpeed = def.MotorSpeed, MaxMotor = def.MaxMotorForce,
        Hertz = def.SpringHertz, Damping = def.SpringDampingRatio, BreakForce = def.BreakForce, BreakTorque = def.BreakTorque,
    });

    /// <summary>Creates a motor (relative pose) joint. Throws as <see cref="CreateJoint(in RevoluteJointDef)"/>.</summary>
    public Joint CreateJoint(in MotorJointDef def) => Make(def.BodyA, def.BodyB, new JointData2D
    {
        Type = (byte)JointType2D.Motor, Flags = Flags(false, false, false, def.CollideConnected),
        OffX = def.LinearOffset.X, OffY = def.LinearOffset.Y, OffAngle = def.AngularOffset, MaxMotor = def.MaxForce, MaxMotor2 = def.MaxTorque, Correction = def.CorrectionFactor,
        BreakForce = def.BreakForce, BreakTorque = def.BreakTorque,
    });

    /// <summary>Creates a mouse (target) joint. Throws as <see cref="CreateJoint(in RevoluteJointDef)"/>.</summary>
    public Joint CreateJoint(in MouseJointDef def) => Make(def.BodyA, def.BodyB, new JointData2D
    {
        Type = (byte)JointType2D.Mouse, Flags = Flags(false, false, false, def.CollideConnected),
        LBx = def.LocalAnchorB.X, LBy = def.LocalAnchorB.Y, TargetX = def.Target.X, TargetY = def.Target.Y,
        Hertz = def.Hertz, Damping = def.DampingRatio, MaxMotor = def.MaxForce, BreakForce = def.BreakForce, BreakTorque = def.BreakTorque,
    });

    // ------------------------------------------------------------------ happy path

    /// <summary>A world point in a body's own frame (measured from the body origin).</summary>
    private static Vector2F ToLocal(Body body, Vector2D world, string what)
    {
        if (!double.IsFinite(world.X) || !double.IsFinite(world.Y) || Math.Abs(world.X) > 2097152.0 || Math.Abs(world.Y) > 2097152.0)
        {
            throw new ArgumentOutOfRangeException(what, world, what + " " + world + " is not finite or is outside the supported range of +-2^21.");
        }

        var origin = body.Position;
        var q = body.Rotation;
        var v = q.InverseRotate(new Vec2((float)(world.X - origin.X), (float)(world.Y - origin.Y)));
        return new Vector2F(v.X, v.Y);
    }

    private static Vector2F DirToLocal(Body body, Vector2F dir, string what)
    {
        if (!float.IsFinite(dir.X) || !float.IsFinite(dir.Y)) { throw new ArgumentOutOfRangeException(what, dir, what + " " + dir + " is not finite."); }
        var v = body.Rotation.InverseRotate(new Vec2(dir.X, dir.Y));
        return new Vector2F(v.X, v.Y);
    }

    /// <summary>B's rotation relative to A's now, radians, so a joint created from the current poses holds them.</summary>
    private static float RelativeAngle(Body a, Body b)
    {
        var qa = a.Rotation; var qb = b.Rotation;
        float c = qa.C * qb.C + qa.S * qb.S, s = qa.C * qb.S - qa.S * qb.C;
        return (float)Math64.Atan2(s, c);
    }

    /// <summary>A hinge: the world point <paramref name="anchor"/> (on both bodies as they are now) stays together, rotation is free. Set limits, a motor or a spring through the returned <see cref="Joint"/>.</summary>
    /// <param name="a">The first body.</param>
    /// <param name="b">The second body.</param>
    /// <param name="anchor">The hinge point in world coordinates (within +-2^21).</param>
    /// <param name="collideConnected">When true the bodies keep colliding with each other.</param>
    /// <exception cref="InvalidJointException">Same body twice, or neither body dynamic.</exception>
    public Joint AddHinge(Body a, Body b, Vector2D anchor, bool collideConnected = false)
    {
        var d = new RevoluteJointDef { BodyA = a, BodyB = b, LocalAnchorA = ToLocal(a, anchor, "anchor"), LocalAnchorB = ToLocal(b, anchor, "anchor"), CollideConnected = collideConnected };
        d.ReferenceAngle = RelativeAngle(a, b);
        return CreateJoint(in d);
    }

    /// <summary>A slider: B moves along <paramref name="axis"/> (a world direction, normalized) through <paramref name="anchor"/>, rotation locked as it is now.</summary>
    public Joint AddSlider(Body a, Body b, Vector2D anchor, Vector2F axis, bool collideConnected = false)
    {
        var d = new PrismaticJointDef { BodyA = a, BodyB = b, LocalAnchorA = ToLocal(a, anchor, "anchor"), LocalAnchorB = ToLocal(b, anchor, "anchor"), LocalAxisA = DirToLocal(a, axis, "axis"), CollideConnected = collideConnected };
        d.ReferenceAngle = RelativeAngle(a, b);
        return CreateJoint(in d);
    }

    /// <summary>Welds the bodies together as they are now: position and rotation locked at <paramref name="anchor"/>. Make it springy with <see cref="Joint.SetWeldSprings"/>.</summary>
    public Joint AddFixed(Body a, Body b, Vector2D anchor, bool collideConnected = false)
    {
        var d = new WeldJointDef { BodyA = a, BodyB = b, LocalAnchorA = ToLocal(a, anchor, "anchor"), LocalAnchorB = ToLocal(b, anchor, "anchor"), CollideConnected = collideConnected };
        d.ReferenceAngle = RelativeAngle(a, b);
        return CreateJoint(in d);
    }

    /// <summary>A spring between two world points (one on each body): rest length is their current distance, frequency <paramref name="hertz"/> and damping ratio <paramref name="dampingRatio"/>; the length is free to change.</summary>
    public Joint AddSpring(Body a, Body b, Vector2D anchorA, Vector2D anchorB, float hertz = 4f, float dampingRatio = 0.5f, bool collideConnected = false)
    {
        var la = ToLocal(a, anchorA, "anchorA"); var lb = ToLocal(b, anchorB, "anchorB");
        double dx = anchorB.X - anchorA.X, dy = anchorB.Y - anchorA.Y;
        float len = (float)Math.Sqrt(dx * dx + dy * dy);
        var d = new DistanceJointDef
        {
            BodyA = a, BodyB = b, LocalAnchorA = la, LocalAnchorB = lb, Length = len < 0.005f ? 0.005f : len,
            EnableSpring = true, SpringHertz = hertz, SpringDampingRatio = dampingRatio, CollideConnected = collideConnected,
        };
        return CreateJoint(in d);
    }

    /// <summary>A rigid rod between two world points, at their current distance.</summary>
    public Joint AddDistance(Body a, Body b, Vector2D anchorA, Vector2D anchorB, bool collideConnected = false)
    {
        var la = ToLocal(a, anchorA, "anchorA"); var lb = ToLocal(b, anchorB, "anchorB");
        double dx = anchorB.X - anchorA.X, dy = anchorB.Y - anchorA.Y;
        float len = (float)Math.Sqrt(dx * dx + dy * dy);
        var d = new DistanceJointDef { BodyA = a, BodyB = b, LocalAnchorA = la, LocalAnchorB = lb, Length = len < 0.005f ? 0.005f : len, CollideConnected = collideConnected };
        return CreateJoint(in d);
    }

    /// <summary>A suspension: <paramref name="wheel"/> slides along <paramref name="axis"/> (a world direction) through <paramref name="anchor"/> on <paramref name="chassis"/> with a spring (default 1 Hz, damping ratio 0.7) and spins freely; add a motor through the <see cref="Joint"/>.</summary>
    public Joint AddWheel(Body chassis, Body wheel, Vector2D anchor, Vector2F axis, bool collideConnected = false)
    {
        var d = new WheelJointDef { BodyA = chassis, BodyB = wheel, LocalAnchorA = ToLocal(chassis, anchor, "anchor"), LocalAnchorB = ToLocal(wheel, anchor, "anchor"), LocalAxisA = DirToLocal(chassis, axis, "axis"), CollideConnected = collideConnected };
        return CreateJoint(in d);
    }

    /// <summary>
    /// Drags <paramref name="body"/> towards <paramref name="target"/> like a mouse: the point of the body that is at <paramref name="target"/> now is pulled to wherever <see cref="Joint.Target"/> is later.
    /// <paramref name="ground"/> is any other body (usually a static one); it is not moved. <paramref name="maxForce"/> at most 0 means 1000 N per kilogram of the body (at least 1000 N).
    /// </summary>
    public Joint AddTarget(Body ground, Body body, Vector2D target, float hertz = 5f, float dampingRatio = 0.7f, float maxForce = 0f)
    {
        float mass = body.Mass;
        var d = new MouseJointDef
        {
            BodyA = ground, BodyB = body, LocalAnchorB = ToLocal(body, target, "target"), Target = target, Hertz = hertz, DampingRatio = dampingRatio,
            MaxForce = maxForce > 0f ? maxForce : 1000f * (mass > 1f ? mass : 1f),
        };
        return CreateJoint(in d);
    }

    /// <summary>A motor joint that keeps B's pose relative to A as it is now, with force and torque capped at <paramref name="maxForce"/> and <paramref name="maxTorque"/>.</summary>
    public Joint AddRelative(Body a, Body b, float maxForce = 1000f, float maxTorque = 1000f, bool collideConnected = false)
    {
        var pa = a.Position; var pb = b.Position;
        var off = a.Rotation.InverseRotate(new Vec2((float)(pb.X - pa.X), (float)(pb.Y - pa.Y)));
        var d = new MotorJointDef { BodyA = a, BodyB = b, LinearOffset = new Vector2F(off.X, off.Y), AngularOffset = RelativeAngle(a, b), MaxForce = maxForce, MaxTorque = maxTorque, CollideConnected = collideConnected };
        return CreateJoint(in d);
    }
}
