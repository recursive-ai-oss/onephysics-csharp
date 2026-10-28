using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
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

    /// <summary>Called from the one delivery loop, so a break handler runs at the event's place in the sorted order, between the contact events around it.</summary>
    private void FireJointBreak(in PhysicsEvent e)
    {
        if (!_jointHandlers.TryGetValue(e.Joint.Index, out var h) || h.Generation != e.Joint.Generation) { return; }
        var action = h.Break;
        _jointHandlers.Remove(e.Joint.Index);
        action?.Invoke(new JointBreak(new Joint(this, e.Joint), new Body(this, e.BodyA), new Body(this, e.BodyB), e.Value));
    }

    private void DropStaleJointHandlers()
    {
        if (_jointHandlers.Count == 0) { return; }
        var dead = new List<int>();
        var cs = _core.Constraints;
        foreach (var kv in _jointHandlers)
        {
            if (!cs.Handles.IsAlive(kv.Key, kv.Value.Generation)) { dead.Add(kv.Key); }
        }

        foreach (int k in dead) { _jointHandlers.Remove(k); }
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

    private Joint Make(Body a, Body b, JointData3D d)
    {
        var h = _engine.CreateJoint(H(a, "BodyA"), H(b, "BodyB"), d);
        return new Joint(this, h);
    }

    private static byte Flags(bool limit, bool motor, bool spring, bool collide) =>
        (byte)((limit ? JointData3D.FlagLimit : 0) | (motor ? JointData3D.FlagMotor : 0) | (spring ? JointData3D.FlagSpring : 0) | (collide ? JointData3D.FlagCollide : 0));

    private static JointData3D Base(JointType3D type, byte flags, JointFrame3D fa, JointFrame3D fb, float breakForce, float breakTorque) => new()
    {
        Type = (byte)type, Flags = flags,
        LAx = fa.Position.X, LAy = fa.Position.Y, LAz = fa.Position.Z, LBx = fb.Position.X, LBy = fb.Position.Y, LBz = fb.Position.Z,
        QA = fa.Rotation, QB = fb.Rotation, BreakForce = breakForce, BreakTorque = breakTorque,
    };

    /// <summary>Creates a fixed joint.</summary>
    /// <exception cref="InvalidJointException">Same body twice, no dynamic body, or values the type does not allow.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or out of range (the message names it).</exception>
    /// <exception cref="InvalidHandleException">A body is the default value, destroyed (<see cref="StaleHandleException"/>) or from another world.</exception>
    public Joint CreateJoint(in FixedJointDef def) => Make(def.BodyA, def.BodyB, Base(JointType3D.Fixed, Flags(false, false, false, def.CollideConnected), def.LocalFrameA, def.LocalFrameB, def.BreakForce, def.BreakTorque));

    /// <summary>Creates a point (ball-and-socket) joint. Throws as <see cref="CreateJoint(in FixedJointDef)"/>.</summary>
    public Joint CreateJoint(in PointJointDef def) => Make(def.BodyA, def.BodyB, Base(JointType3D.Point, Flags(false, false, false, def.CollideConnected), def.LocalFrameA, def.LocalFrameB, def.BreakForce, def.BreakTorque));

    /// <summary>Creates a distance joint. Throws as <see cref="CreateJoint(in FixedJointDef)"/>.</summary>
    public Joint CreateJoint(in DistanceJointDef def)
    {
        var d = Base(JointType3D.Distance, Flags(def.EnableLimit, false, def.EnableSpring, def.CollideConnected), def.LocalFrameA, def.LocalFrameB, def.BreakForce, def.BreakTorque);
        d.Length = def.Length; d.Lower = def.MinLength; d.Upper = def.MaxLength; d.Hertz = def.SpringHertz; d.Damping = def.SpringDampingRatio;
        return Make(def.BodyA, def.BodyB, d);
    }

    /// <summary>Creates a hinge joint. Throws as <see cref="CreateJoint(in FixedJointDef)"/>.</summary>
    public Joint CreateJoint(in HingeJointDef def)
    {
        var d = Base(JointType3D.Hinge, Flags(def.EnableLimit, def.EnableMotor, def.EnableSpring, def.CollideConnected), def.LocalFrameA, def.LocalFrameB, def.BreakForce, def.BreakTorque);
        d.Lower = def.LowerAngle; d.Upper = def.UpperAngle; d.MotorSpeed = def.MotorSpeed; d.MaxMotor = def.MaxMotorTorque; d.Hertz = def.SpringHertz; d.Damping = def.SpringDampingRatio;
        return Make(def.BodyA, def.BodyB, d);
    }

    /// <summary>Creates a slider joint. Throws as <see cref="CreateJoint(in FixedJointDef)"/>.</summary>
    public Joint CreateJoint(in SliderJointDef def)
    {
        var d = Base(JointType3D.Slider, Flags(def.EnableLimit, def.EnableMotor, def.EnableSpring, def.CollideConnected), def.LocalFrameA, def.LocalFrameB, def.BreakForce, def.BreakTorque);
        d.Lower = def.LowerTranslation; d.Upper = def.UpperTranslation; d.MotorSpeed = def.MotorSpeed; d.MaxMotor = def.MaxMotorForce; d.Hertz = def.SpringHertz; d.Damping = def.SpringDampingRatio;
        return Make(def.BodyA, def.BodyB, d);
    }

    /// <summary>Creates a cone joint. Throws as <see cref="CreateJoint(in FixedJointDef)"/>.</summary>
    public Joint CreateJoint(in ConeJointDef def)
    {
        var d = Base(JointType3D.Cone, Flags(false, false, false, def.CollideConnected), def.LocalFrameA, def.LocalFrameB, def.BreakForce, def.BreakTorque);
        d.Swing1 = def.HalfConeAngle;
        return Make(def.BodyA, def.BodyB, d);
    }

    /// <summary>Creates a swing-twist joint. Throws as <see cref="CreateJoint(in FixedJointDef)"/>.</summary>
    public Joint CreateJoint(in SwingTwistJointDef def)
    {
        var d = Base(JointType3D.SwingTwist, Flags(false, false, false, def.CollideConnected), def.LocalFrameA, def.LocalFrameB, def.BreakForce, def.BreakTorque);
        d.Lower = def.LowerTwist; d.Upper = def.UpperTwist; d.Swing1 = def.SwingYHalfAngle; d.Swing2 = def.SwingZHalfAngle;
        return Make(def.BodyA, def.BodyB, d);
    }

    /// <summary>Creates a six-degrees-of-freedom joint. Throws as <see cref="CreateJoint(in FixedJointDef)"/>.</summary>
    public Joint CreateJoint(in SixDofJointDef def)
    {
        var d = Base(JointType3D.SixDof, Flags(false, false, false, def.CollideConnected), def.LocalFrameA, def.LocalFrameB, def.BreakForce, def.BreakTorque);
        ref var s = ref d.Six;
        s.Mode[0] = (byte)def.LinearX; s.Mode[1] = (byte)def.LinearY; s.Mode[2] = (byte)def.LinearZ;
        s.Mode[3] = (byte)def.AngularX; s.Mode[4] = (byte)def.AngularY; s.Mode[5] = (byte)def.AngularZ;
        Set3(ref s.Lo, 0, def.LowerLinearLimit); Set3(ref s.Hi, 0, def.UpperLinearLimit); Set3(ref s.Lo, 3, def.LowerAngularLimit); Set3(ref s.Hi, 3, def.UpperAngularLimit);
        Set3(ref s.MotorSpeed, 0, def.LinearMotorSpeed); Set3(ref s.MaxMotor, 0, def.MaxLinearMotorForce); Set3(ref s.MotorSpeed, 3, def.AngularMotorSpeed); Set3(ref s.MaxMotor, 3, def.MaxAngularMotorTorque);
        Set3(ref s.Hertz, 0, def.SpringHertz); Set3(ref s.Damping, 0, def.SpringDampingRatio); Set3(ref s.Hertz, 3, def.AngularSpringHertz); Set3(ref s.Damping, 3, def.AngularSpringDampingRatio);
        s.MotorMask = (byte)def.MotorAxes; s.SpringMask = (byte)def.SpringAxes;
        return Make(def.BodyA, def.BodyB, d);
    }

    private static void Set3(ref JF6 a, int o, Vector3F v) { a[o] = v.X; a[o + 1] = v.Y; a[o + 2] = v.Z; }

    /// <summary>Creates a path joint. Throws as <see cref="CreateJoint(in FixedJointDef)"/>, and <see cref="InvalidJointException"/> for fewer than 2 or more than 256 points.</summary>
    public Joint CreateJoint(in PathJointDef def)
    {
        var d = Base(JointType3D.Path, (byte)(Flags(false, def.EnableMotor, false, def.CollideConnected) | (def.Loop ? JointData3D.FlagLoop : 0)), JointFrame3D.Identity, def.LocalFrameB, def.BreakForce, def.BreakTorque);
        d.Path = def.Points == null ? null : (Vector3F[])def.Points.Clone();
        d.UpX = def.Up.X; d.UpY = def.Up.Y; d.UpZ = def.Up.Z; d.Aux = (byte)def.Rotation; d.MotorSpeed = def.MotorSpeed; d.MaxMotor = def.MaxMotorForce;
        return Make(def.BodyA, def.BodyB, d);
    }

    /// <summary>Creates a gear joint. Throws as <see cref="CreateJoint(in FixedJointDef)"/>.</summary>
    public Joint CreateJoint(in GearJointDef def)
    {
        var d = Base(JointType3D.Gear, Flags(false, false, false, def.CollideConnected), def.LocalFrameA, def.LocalFrameB, def.BreakForce, def.BreakTorque);
        d.Ratio = def.Ratio;
        return Make(def.BodyA, def.BodyB, d);
    }

    /// <summary>Creates a rack-and-pinion joint. Throws as <see cref="CreateJoint(in FixedJointDef)"/>.</summary>
    public Joint CreateJoint(in RackAndPinionJointDef def)
    {
        var d = Base(JointType3D.RackAndPinion, Flags(false, false, false, def.CollideConnected), def.LocalFrameA, def.LocalFrameB, def.BreakForce, def.BreakTorque);
        d.Ratio = def.Ratio;
        return Make(def.BodyA, def.BodyB, d);
    }

    /// <summary>Creates a pulley joint. Negative min and max lengths mean the current rope length. Throws as <see cref="CreateJoint(in FixedJointDef)"/>.</summary>
    public Joint CreateJoint(in PulleyJointDef def)
    {
        var d = Base(JointType3D.Pulley, Flags(false, false, false, def.CollideConnected), def.LocalFrameA, def.LocalFrameB, def.BreakForce, def.BreakTorque);
        d.Ratio = def.Ratio;
        d.FxA = def.FixedPointA.X; d.FyA = def.FixedPointA.Y; d.FzA = def.FixedPointA.Z; d.FxB = def.FixedPointB.X; d.FyB = def.FixedPointB.Y; d.FzB = def.FixedPointB.Z;
        d.Lower = def.MinLength; d.Upper = def.MaxLength;
        if (def.MinLength < 0f || def.MaxLength < 0f)
        {
            var probe = d; probe.Lower = 0f; probe.Upper = 0f;
            probe.Validate(); // rejects non-finite fixed points before they are used below
            H(def.BodyA, "BodyA"); H(def.BodyB, "BodyB");
            double la = Dist(AnchorWorld(def.BodyA, def.LocalFrameA.Position), def.FixedPointA), lb = Dist(AnchorWorld(def.BodyB, def.LocalFrameB.Position), def.FixedPointB);
            float len = (float)(la + (double)def.Ratio * lb);
            if (def.MinLength < 0f) { d.Lower = len; }
            if (def.MaxLength < 0f) { d.Upper = len; }
            if (d.Upper < d.Lower) { throw new InvalidJointException("pulley joint: MaxLength " + d.Upper.ToString(System.Globalization.CultureInfo.InvariantCulture) + " is below MinLength " + d.Lower.ToString(System.Globalization.CultureInfo.InvariantCulture) + "."); }
        }

        return Make(def.BodyA, def.BodyB, d);
    }

    private static double Dist(Vector3D a, Vector3D b)
    {
        double x = a.X - b.X, y = a.Y - b.Y, z = a.Z - b.Z;
        return Math.Sqrt(x * x + y * y + z * z);
    }

    private static Vector3D AnchorWorld(Body body, Vector3F local)
    {
        var o = body.Position;
        var r = body.Rotation.Rotate(local);
        return new Vector3D(o.X + (double)r.X, o.Y + (double)r.Y, o.Z + (double)r.Z);
    }

    // ------------------------------------------------------------------ happy path

    private static void CheckWorldPoint(Vector3D world, string what)
    {
        if (!double.IsFinite(world.X) || !double.IsFinite(world.Y) || !double.IsFinite(world.Z) || Math.Abs(world.X) > 2097152.0 || Math.Abs(world.Y) > 2097152.0 || Math.Abs(world.Z) > 2097152.0)
        {
            throw new ArgumentOutOfRangeException(what, world, what + " " + world + " is not finite or is outside the supported range of +-2^21.");
        }
    }

    /// <summary>A world point in a body's own frame (measured from the body origin).</summary>
    private static Vector3F ToLocal(Body body, Vector3D world, string what)
    {
        CheckWorldPoint(world, what);
        var o = body.Position;
        return body.Rotation.Conjugate().Rotate(new Vector3F((float)(world.X - o.X), (float)(world.Y - o.Y), (float)(world.Z - o.Z)));
    }

    /// <summary>A world orientation whose x axis is <paramref name="axis"/> (a finite, non-zero world direction), with an arbitrary but fixed choice of the other two axes.</summary>
    private static QuaternionF WorldFrame(Vector3F axis, string what)
    {
        float l2 = axis.LengthSquared();
        if (!float.IsFinite(l2) || !(l2 > 1e-20f)) { throw new ArgumentOutOfRangeException(what, axis, what + " " + axis + " must be finite and non-zero."); }
        PathSpline3D.Frame(axis, default, out var x, out var y, out var z);
        return PathSpline3D.FromBasis(x, y, z);
    }

    /// <summary>The frame on a body that coincides with a world frame at <paramref name="anchor"/> now.</summary>
    private static JointFrame3D FrameOn(Body body, Vector3D anchor, QuaternionF worldFrame, string what)
        => new(ToLocal(body, anchor, what), body.Rotation.Conjugate() * worldFrame);

    /// <summary>A hinge: the world point <paramref name="anchor"/> (on both bodies as they are now) stays together and the bodies turn about <paramref name="axis"/> (a world direction). Angle 0 is the current pose. Set limits, a motor or a spring through the returned <see cref="Joint"/>.</summary>
    /// <param name="a">The first body.</param>
    /// <param name="b">The second body.</param>
    /// <param name="anchor">The hinge point in world coordinates (within +-2^21).</param>
    /// <param name="axis">The hinge axis, a finite non-zero world direction.</param>
    /// <param name="collideConnected">When true the bodies keep colliding with each other.</param>
    /// <exception cref="InvalidJointException">Same body twice, or neither body dynamic.</exception>
    public Joint AddHinge(Body a, Body b, Vector3D anchor, Vector3F axis, bool collideConnected = false)
    {
        var w = WorldFrame(axis, nameof(axis));
        var d = new HingeJointDef { BodyA = a, BodyB = b, LocalFrameA = FrameOn(a, anchor, w, "anchor"), LocalFrameB = FrameOn(b, anchor, w, "anchor"), CollideConnected = collideConnected };
        return CreateJoint(in d);
    }

    /// <summary>A slider: B moves along <paramref name="axis"/> (a world direction) through <paramref name="anchor"/>, rotation locked as it is now; translation 0 is the current pose.</summary>
    public Joint AddSlider(Body a, Body b, Vector3D anchor, Vector3F axis, bool collideConnected = false)
    {
        var w = WorldFrame(axis, nameof(axis));
        var d = new SliderJointDef { BodyA = a, BodyB = b, LocalFrameA = FrameOn(a, anchor, w, "anchor"), LocalFrameB = FrameOn(b, anchor, w, "anchor"), CollideConnected = collideConnected };
        return CreateJoint(in d);
    }

    /// <summary>Welds the bodies together as they are now: position and rotation locked at <paramref name="anchor"/>.</summary>
    public Joint AddFixed(Body a, Body b, Vector3D anchor, bool collideConnected = false)
    {
        var d = new FixedJointDef { BodyA = a, BodyB = b, LocalFrameA = FrameOn(a, anchor, QuaternionF.Identity, "anchor"), LocalFrameB = FrameOn(b, anchor, QuaternionF.Identity, "anchor"), CollideConnected = collideConnected };
        return CreateJoint(in d);
    }

    /// <summary>A ball-and-socket: the world point <paramref name="anchor"/> stays on both bodies, rotation is free.</summary>
    public Joint AddPoint(Body a, Body b, Vector3D anchor, bool collideConnected = false)
    {
        var d = new PointJointDef { BodyA = a, BodyB = b, LocalFrameA = FrameOn(a, anchor, QuaternionF.Identity, "anchor"), LocalFrameB = FrameOn(b, anchor, QuaternionF.Identity, "anchor"), CollideConnected = collideConnected };
        return CreateJoint(in d);
    }

    /// <summary>A spring between two world points (one on each body): rest length is their current distance, frequency <paramref name="hertz"/> and damping ratio <paramref name="dampingRatio"/>; the length is free to change.</summary>
    public Joint AddSpring(Body a, Body b, Vector3D anchorA, Vector3D anchorB, float hertz = 4f, float dampingRatio = 0.5f, bool collideConnected = false)
    {
        var fa = new JointFrame3D(ToLocal(a, anchorA, "anchorA"), QuaternionF.Identity); var fb = new JointFrame3D(ToLocal(b, anchorB, "anchorB"), QuaternionF.Identity);
        float len = (float)Dist(anchorA, anchorB);
        var d = new DistanceJointDef { BodyA = a, BodyB = b, LocalFrameA = fa, LocalFrameB = fb, Length = len < 0.005f ? 0.005f : len, EnableSpring = true, SpringHertz = hertz, SpringDampingRatio = dampingRatio, CollideConnected = collideConnected };
        return CreateJoint(in d);
    }

    /// <summary>A rigid rod between two world points, at their current distance.</summary>
    public Joint AddDistance(Body a, Body b, Vector3D anchorA, Vector3D anchorB, bool collideConnected = false)
    {
        var fa = new JointFrame3D(ToLocal(a, anchorA, "anchorA"), QuaternionF.Identity); var fb = new JointFrame3D(ToLocal(b, anchorB, "anchorB"), QuaternionF.Identity);
        float len = (float)Dist(anchorA, anchorB);
        var d = new DistanceJointDef { BodyA = a, BodyB = b, LocalFrameA = fa, LocalFrameB = fb, Length = len < 0.005f ? 0.005f : len, CollideConnected = collideConnected };
        return CreateJoint(in d);
    }

    /// <summary>A cone joint: a ball-and-socket at <paramref name="anchor"/> that keeps the two bodies' copies of <paramref name="axis"/> within <paramref name="halfAngle"/> radians of each other.</summary>
    public Joint AddCone(Body a, Body b, Vector3D anchor, Vector3F axis, float halfAngle = 0.7853982f, bool collideConnected = false)
    {
        var w = WorldFrame(axis, nameof(axis));
        var d = new ConeJointDef { BodyA = a, BodyB = b, LocalFrameA = FrameOn(a, anchor, w, "anchor"), LocalFrameB = FrameOn(b, anchor, w, "anchor"), HalfConeAngle = halfAngle, CollideConnected = collideConnected };
        return CreateJoint(in d);
    }

    /// <summary>A limb joint (Unity's CharacterJoint): a ball-and-socket at <paramref name="anchor"/> with twist about <paramref name="twistAxis"/> limited to the given range and swing limited to an elliptical cone (half angles about the two perpendicular axes). Angles in radians.</summary>
    public Joint AddCharacterJoint(Body parent, Body child, Vector3D anchor, Vector3F twistAxis, float lowTwist = -0.7853982f, float highTwist = 0.7853982f, float swingYHalfAngle = 0.7853982f, float swingZHalfAngle = 0.7853982f, bool collideConnected = false)
    {
        var w = WorldFrame(twistAxis, nameof(twistAxis));
        var d = new SwingTwistJointDef
        {
            BodyA = parent, BodyB = child, LocalFrameA = FrameOn(parent, anchor, w, "anchor"), LocalFrameB = FrameOn(child, anchor, w, "anchor"),
            LowerTwist = lowTwist, UpperTwist = highTwist, SwingYHalfAngle = swingYHalfAngle, SwingZHalfAngle = swingZHalfAngle, CollideConnected = collideConnected,
        };
        return CreateJoint(in d);
    }

    /// <summary>A six-degrees-of-freedom joint (Unity's ConfigurableJoint) with every axis locked at the current pose and the frame axes along the world axes; free or limit axes through <see cref="Joint.SetAxisMode"/> and <see cref="Joint.SetAxisLimits"/>.</summary>
    public Joint AddConfigurable(Body a, Body b, Vector3D anchor, bool collideConnected = false)
    {
        var d = new SixDofJointDef { BodyA = a, BodyB = b, LocalFrameA = FrameOn(a, anchor, QuaternionF.Identity, "anchor"), LocalFrameB = FrameOn(b, anchor, QuaternionF.Identity, "anchor"), CollideConnected = collideConnected };
        return CreateJoint(in d);
    }

    /// <summary>A gear: A turns about world axis <paramref name="axisA"/> and B about <paramref name="axisB"/> so that <c>ωA·axisA + ratio·ωB·axisB = 0</c>.</summary>
    public Joint AddGear(Body a, Body b, Vector3F axisA, Vector3F axisB, float ratio = 1f, bool collideConnected = false)
    {
        var d = new GearJointDef
        {
            BodyA = a, BodyB = b, LocalFrameA = new JointFrame3D(default, a.Rotation.Conjugate() * WorldFrame(axisA, nameof(axisA))), LocalFrameB = new JointFrame3D(default, b.Rotation.Conjugate() * WorldFrame(axisB, nameof(axisB))),
            Ratio = ratio, CollideConnected = collideConnected,
        };
        return CreateJoint(in d);
    }

    /// <summary>A rack and pinion: the pinion turns about world axis <paramref name="pinionAxis"/> and the rack slides along <paramref name="rackAxis"/>, <paramref name="ratio"/> metres per radian.</summary>
    public Joint AddRackAndPinion(Body pinion, Body rack, Vector3F pinionAxis, Vector3F rackAxis, float ratio = 1f, bool collideConnected = false)
    {
        var d = new RackAndPinionJointDef
        {
            BodyA = pinion, BodyB = rack, LocalFrameA = new JointFrame3D(default, pinion.Rotation.Conjugate() * WorldFrame(pinionAxis, nameof(pinionAxis))), LocalFrameB = new JointFrame3D(default, rack.Rotation.Conjugate() * WorldFrame(rackAxis, nameof(rackAxis))),
            Ratio = ratio, CollideConnected = collideConnected,
        };
        return CreateJoint(in d);
    }

    /// <summary>A pulley: a rope from <paramref name="anchorA"/> over <paramref name="fixedA"/> and <paramref name="fixedB"/> to <paramref name="anchorB"/> at its current length, <c>lengthA + ratio·lengthB</c> held constant.</summary>
    public Joint AddPulley(Body a, Body b, Vector3D anchorA, Vector3D anchorB, Vector3D fixedA, Vector3D fixedB, float ratio = 1f, bool collideConnected = false)
    {
        var d = new PulleyJointDef
        {
            BodyA = a, BodyB = b, LocalFrameA = new JointFrame3D(ToLocal(a, anchorA, "anchorA"), QuaternionF.Identity), LocalFrameB = new JointFrame3D(ToLocal(b, anchorB, "anchorB"), QuaternionF.Identity),
            FixedPointA = fixedA, FixedPointB = fixedB, Ratio = ratio, CollideConnected = collideConnected,
        };
        return CreateJoint(in d);
    }

    /// <summary>A path: the origin of <paramref name="follower"/> rides a smooth curve through <paramref name="worldPoints"/> (world coordinates as the bodies are now) fixed to <paramref name="pathBody"/>. Add a motor through the <see cref="Joint"/>.</summary>
    public Joint AddPath(Body pathBody, Body follower, ReadOnlySpan<Vector3D> worldPoints, bool loop = false, bool collideConnected = false)
    {
        var pts = new Vector3F[worldPoints.Length];
        for (int i = 0; i < pts.Length; i++) { pts[i] = ToLocal(pathBody, worldPoints[i], "worldPoints"); }
        var d = new PathJointDef { BodyA = pathBody, BodyB = follower, Points = pts, Loop = loop, CollideConnected = collideConnected };
        return CreateJoint(in d);
    }
}
