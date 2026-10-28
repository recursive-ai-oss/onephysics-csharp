using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics3D;

// Fuzz targets for 3D joints. "world3d-joints" turns bytes into a program: bodies, hostile joint definitions of every kind (NaN, infinity, huge, tiny, negative, the same body twice, default and destroyed handles,
// zero-length axes, degenerate and non-unit frame rotations, short and repeated path points), edits of live joints through their properties, destroying joints and bodies, stepping, and snapshot round trips.
// Only argument errors and typed physics errors may come out; anything else is a finding.
// "world3d-joint-snapshot" flips bytes in the snapshot of a jointed world: the restore must be rejected with the world unchanged, or accepted and then re-serialize to the same bytes and step.
internal static class FuzzJoints3D
{
    private static int s_made, s_refused, s_edits, s_editsRefused, s_steps, s_accepted, s_rejected;
    private static byte[]? s_base;

    static FuzzJoints3D()
    {
        AppDomain.CurrentDomain.ProcessExit += (_, _) => Console.WriteLine("3d joint targets: joints created " + s_made + ", refused " + s_refused + ", edits " + s_edits + ", edits refused " + s_editsRefused + ", steps " + s_steps + "; snapshots accepted " + s_accepted + ", rejected " + s_rejected);
    }

    private sealed class Reader
    {
        private readonly byte[] _d;
        private int _i;

        public Reader(byte[] d) { _d = d; Calm = d.Length > 0 && (d[0] & 0x40) != 0; }

        /// <summary>Half of the inputs use no special values and small magnitudes, so that most joints are accepted and the solver sees them.</summary>
        public bool Calm { get; }

        public bool More => _i < _d.Length;

        public byte Byte() => _i < _d.Length ? _d[_i++] : (byte)0;

        public float Float()
        {
            byte b = Byte();
            if (Calm) { return (short)(b | (Byte() << 8)) / 2048f; }
            switch (b)
            {
                case 250: return float.NaN;
                case 251: return float.PositiveInfinity;
                case 252: return float.NegativeInfinity;
                case 253: return 3e6f;
                case 254: return 1e-9f;
                case 255: return -0f;
            }

            int v = (short)(b | (Byte() << 8));
            return v / 256f;
        }

        public double Double()
        {
            byte b = Byte();
            switch (b)
            {
                case 250: return double.NaN;
                case 251: return double.PositiveInfinity;
                case 252: return double.NegativeInfinity;
                case 253: return 3e6;
                case 254: return 1e-300;
                case 255: return -0.0;
            }

            int v = (short)(b | (Byte() << 8));
            return v / 64.0;
        }

        public Vector3F V3F() => new(Float(), Float(), Float());

        public Vector3D V3D() => new(Double(), Double(), Double());

        public JointFrame3D Frame()
        {
            // Mostly valid frames (identity or a normalized random rotation) so the joints get built and stepped; one in eight is raw and hostile.
            byte mode = Byte();
            var p = V3F();
            if (mode % 8 == 0) { return new JointFrame3D(p, new QuaternionF(Float(), Float(), Float(), Float())); }
            if (mode % 2 == 0) { return new JointFrame3D(p, QuaternionF.Identity); }
            float x = Byte() / 128f - 1f, y = Byte() / 128f - 1f, z = Byte() / 128f - 1f, w = Byte() / 128f - 1f;
            float n = float.Sqrt(x * x + y * y + z * z + w * w);
            return n > 0.1f ? new JointFrame3D(p, new QuaternionF(x / n, y / n, z / n, w / n)) : new JointFrame3D(p, QuaternionF.Identity);
        }

        /// <summary>A breaking threshold: mostly the default (never), sometimes a low finite one so joints break, sometimes whatever the bytes say.</summary>
        public float Break()
        {
            byte b = Byte();
            return b < 160 ? float.PositiveInfinity : b < 224 ? 20f + b : Float();
        }

        public bool Bool() => (Byte() & 1) != 0;
    }

    private static bool Allowed(Exception e) => e is ArgumentException || e is PhysicsException;

    private static void StepTolerant(PhysicsWorld3D w)
    {
        try { w.Step(1f / 60f); s_steps++; }
        catch (BodyOutOfBoundsException e) { w.GetBody(e.Body).Destroy(); }
    }

    private static Body Pick(Body[] bodies, Reader r)
    {
        byte b = r.Byte();
        if (b >= 248) { return default; }
        return bodies[b % bodies.Length];
    }

    private static PhysicsWorld3D Scene(int substeps, out Body[] bodies)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { WorldId = 9, Substeps = substeps });
        var list = new System.Collections.Generic.List<Body>
        {
            w.AddBox(new Vector3F(40f, 1f, 40f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static),
            w.AddSphere(0.2f, new Vector3D(0, 8, 0), BodyType.Static),
        };
        for (int i = 0; i < 5; i++) { list.Add(w.AddBox(new Vector3F(1f, 0.6f, 0.6f), new Vector3D(-4 + 2 * i, 3 + 0.2 * i, 0.1 * i))); }
        list.Add(w.AddSphere(0.5f, new Vector3D(1, 6, 0)));
        bodies = list.ToArray();
        return w;
    }

    public static void Run(byte[] data)
    {
        var r = new Reader(data);
        var w = Scene(1 + (r.Byte() & 3), out var bodies);
        var joints = new System.Collections.Generic.List<Joint>();
        int ops = 4 + r.Byte() % 40;
        for (int n = 0; n < ops && r.More; n++)
        {
            try { Op(w, bodies, joints, r); }
            catch (Exception e) when (Allowed(e)) { s_refused++; }
        }

        for (int i = 0; i < 4; i++) { StepTolerant(w); }
        byte[] snap = w.Snapshot();
        var w2 = new PhysicsWorld3D(new WorldOptions3D { WorldId = 9, Substeps = w.Options.Substeps });
        // A body can end the last step outside the supported range or not finite (the next step raises a typed error); the snapshot then holds it and a restore refuses it, which is the documented behavior.
        try { w2.Restore(snap); }
        catch (SnapshotFormatException e) when (e.Message.Contains("out of range", StringComparison.Ordinal) || e.Message.Contains("non-finite", StringComparison.Ordinal)) { return; }
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world hash differs"); }
        if (!w2.Snapshot().AsSpan().SequenceEqual(snap)) { throw new InvalidOperationException("restored world re-serializes differently"); }
        StepTolerant(w); StepTolerant(w2);
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world diverged after a step"); }
    }

    private static void Add(System.Collections.Generic.List<Joint> joints, Joint j)
    {
        joints.Add(j);
        s_made++;
    }

    private static void Op(PhysicsWorld3D w, Body[] bodies, System.Collections.Generic.List<Joint> joints, Reader r)
    {
        switch (r.Byte() % 18)
        {
            case 0: { var d = new FixedJointDef { BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LocalFrameA = r.Frame(), LocalFrameB = r.Frame(), CollideConnected = r.Bool(), BreakForce = r.Break(), BreakTorque = r.Break() }; Add(joints, w.CreateJoint(in d)); break; }
            case 1: { var d = new PointJointDef { BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LocalFrameA = r.Frame(), LocalFrameB = r.Frame(), CollideConnected = r.Bool(), BreakForce = r.Break(), BreakTorque = r.Break() }; Add(joints, w.CreateJoint(in d)); break; }
            case 2: { var d = new DistanceJointDef { BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LocalFrameA = r.Frame(), LocalFrameB = r.Frame(), Length = r.Float(), MinLength = r.Float(), MaxLength = r.Float(), EnableLimit = r.Bool(), EnableSpring = r.Bool(), SpringHertz = r.Float(), SpringDampingRatio = r.Float(), CollideConnected = r.Bool(), BreakForce = r.Break() }; Add(joints, w.CreateJoint(in d)); break; }
            case 3: { var d = new HingeJointDef { BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LocalFrameA = r.Frame(), LocalFrameB = r.Frame(), EnableLimit = r.Bool(), LowerAngle = r.Float(), UpperAngle = r.Float(), EnableMotor = r.Bool(), MotorSpeed = r.Float(), MaxMotorTorque = r.Float(), EnableSpring = r.Bool(), SpringHertz = r.Float(), SpringDampingRatio = r.Float(), CollideConnected = r.Bool(), BreakForce = r.Break(), BreakTorque = r.Break() }; Add(joints, w.CreateJoint(in d)); break; }
            case 4: { var d = new SliderJointDef { BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LocalFrameA = r.Frame(), LocalFrameB = r.Frame(), EnableLimit = r.Bool(), LowerTranslation = r.Float(), UpperTranslation = r.Float(), EnableMotor = r.Bool(), MotorSpeed = r.Float(), MaxMotorForce = r.Float(), EnableSpring = r.Bool(), SpringHertz = r.Float(), SpringDampingRatio = r.Float(), CollideConnected = r.Bool(), BreakForce = r.Break(), BreakTorque = r.Break() }; Add(joints, w.CreateJoint(in d)); break; }
            case 5: { var d = new ConeJointDef { BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LocalFrameA = r.Frame(), LocalFrameB = r.Frame(), HalfConeAngle = r.Float(), CollideConnected = r.Bool(), BreakForce = r.Break() }; Add(joints, w.CreateJoint(in d)); break; }
            case 6: { var d = new SwingTwistJointDef { BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LocalFrameA = r.Frame(), LocalFrameB = r.Frame(), LowerTwist = r.Float(), UpperTwist = r.Float(), SwingYHalfAngle = r.Float(), SwingZHalfAngle = r.Float(), CollideConnected = r.Bool(), BreakForce = r.Break() }; Add(joints, w.CreateJoint(in d)); break; }
            case 7:
                {
                    var d = new SixDofJointDef
                    {
                        BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LocalFrameA = r.Frame(), LocalFrameB = r.Frame(),
                        LinearX = (JointAxisMode)(r.Byte() % 5), LinearY = (JointAxisMode)(r.Byte() % 5), LinearZ = (JointAxisMode)(r.Byte() % 5),
                        AngularX = (JointAxisMode)(r.Byte() % 5), AngularY = (JointAxisMode)(r.Byte() % 5), AngularZ = (JointAxisMode)(r.Byte() % 5),
                        LowerLinearLimit = r.V3F(), UpperLinearLimit = r.V3F(), LowerAngularLimit = r.V3F(), UpperAngularLimit = r.V3F(),
                        MotorAxes = (JointAxes)r.Byte(), LinearMotorSpeed = r.V3F(), MaxLinearMotorForce = r.V3F(), AngularMotorSpeed = r.V3F(), MaxAngularMotorTorque = r.V3F(),
                        SpringAxes = (JointAxes)r.Byte(), SpringHertz = r.V3F(), SpringDampingRatio = r.V3F(), AngularSpringHertz = r.V3F(), AngularSpringDampingRatio = r.V3F(),
                        CollideConnected = r.Bool(), BreakForce = r.Break(),
                    };
                    Add(joints, w.CreateJoint(in d)); break;
                }
            case 8:
                {
                    int n = r.Byte() % 7; var pts = n == 6 ? null : new Vector3F[n];
                    if (pts != null) { for (int i = 0; i < n; i++) { pts[i] = r.V3F(); } }
                    var d = new PathJointDef { BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LocalFrameB = r.Frame(), Points = pts, Loop = r.Bool(), Up = r.V3F(), Rotation = (PathRotation)(r.Byte() % 4), EnableMotor = r.Bool(), MotorSpeed = r.Float(), MaxMotorForce = r.Float(), CollideConnected = r.Bool() };
                    Add(joints, w.CreateJoint(in d)); break;
                }
            case 9: { var d = new GearJointDef { BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LocalFrameA = r.Frame(), LocalFrameB = r.Frame(), Ratio = r.Float(), CollideConnected = r.Bool(), BreakForce = r.Break() }; Add(joints, w.CreateJoint(in d)); break; }
            case 10: { var d = new RackAndPinionJointDef { BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LocalFrameA = r.Frame(), LocalFrameB = r.Frame(), Ratio = r.Float(), CollideConnected = r.Bool(), BreakForce = r.Break() }; Add(joints, w.CreateJoint(in d)); break; }
            case 11: { var d = new PulleyJointDef { BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LocalFrameA = r.Frame(), LocalFrameB = r.Frame(), FixedPointA = r.V3D(), FixedPointB = r.V3D(), Ratio = r.Float(), MinLength = r.Float(), MaxLength = r.Float(), CollideConnected = r.Bool(), BreakForce = r.Break() }; Add(joints, w.CreateJoint(in d)); break; }
            case 12: { int n = 1 + r.Byte() % 5; for (int i = 0; i < n; i++) { StepTolerant(w); } break; }
            case 13: Edit(joints, r); break;
            case 14: { if (joints.Count > 0) { joints[r.Byte() % joints.Count].Destroy(); } break; }
            case 15: { var b = Pick(bodies, r); if (b.IsValid && b != bodies[0] && r.Byte() < 40) { b.Destroy(); } break; }
            case 16:
                {
                    byte[] snap = w.Snapshot();
                    var w2 = new PhysicsWorld3D(new WorldOptions3D { WorldId = 9, Substeps = w.Options.Substeps });
                    try { w2.Restore(snap); }
                    catch (SnapshotFormatException e) when (e.Message.Contains("out of range", StringComparison.Ordinal) || e.Message.Contains("non-finite", StringComparison.Ordinal)) { break; }
                    if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world hash differs"); }
                    break;
                }
            default:
                {
                    var a = Pick(bodies, r); var b = Pick(bodies, r);
                    var p = r.V3D();
                    switch (r.Byte() % 12)
                    {
                        case 0: Add(joints, w.AddHinge(a, b, p, r.V3F())); break;
                        case 1: Add(joints, w.AddSlider(a, b, p, r.V3F())); break;
                        case 2: Add(joints, w.AddFixed(a, b, p)); break;
                        case 3: Add(joints, w.AddPoint(a, b, p)); break;
                        case 4: Add(joints, w.AddSpring(a, b, p, r.V3D(), r.Float(), r.Float())); break;
                        case 5: Add(joints, w.AddDistance(a, b, p, r.V3D())); break;
                        case 6: Add(joints, w.AddCone(a, b, p, r.V3F(), r.Float())); break;
                        case 7: Add(joints, w.AddCharacterJoint(a, b, p, r.V3F(), r.Float(), r.Float(), r.Float(), r.Float())); break;
                        case 8: Add(joints, w.AddGear(a, b, r.V3F(), r.V3F(), r.Float())); break;
                        case 9: Add(joints, w.AddRackAndPinion(a, b, r.V3F(), r.V3F(), r.Float())); break;
                        case 10: Add(joints, w.AddPulley(a, b, p, r.V3D(), r.V3D(), r.V3D(), r.Float())); break;
                        default: Add(joints, w.AddConfigurable(a, b, p)); break;
                    }

                    break;
                }
        }
    }

    private static void Edit(System.Collections.Generic.List<Joint> joints, Reader r)
    {
        if (joints.Count == 0) { return; }
        var j = joints[r.Byte() % joints.Count];
        s_edits++;
        try
        {
            switch (r.Byte() % 18)
            {
                case 0: j.LimitEnabled = r.Bool(); break;
                case 1: j.SetLimits(r.Float(), r.Float()); break;
                case 2: j.MotorEnabled = r.Bool(); break;
                case 3: j.MotorSpeed = r.Float(); break;
                case 4: j.MaxMotorForce = r.Float(); break;
                case 5: j.SpringEnabled = r.Bool(); break;
                case 6: j.SpringHertz = r.Float(); break;
                case 7: j.SpringDampingRatio = r.Float(); break;
                case 8: j.Length = r.Float(); break;
                case 9: j.Ratio = r.Float(); break;
                case 10: j.HalfConeAngle = r.Float(); break;
                case 11: j.SetSwingTwistLimits(r.Float(), r.Float(), r.Float(), r.Float()); break;
                case 12: j.SetAxisMode((JointAxes)r.Byte(), (JointAxisMode)(r.Byte() % 5)); break;
                case 13: j.SetAxisLimits((JointAxes)r.Byte(), r.Float(), r.Float()); break;
                case 14: j.SetAxisMotor((JointAxes)r.Byte(), r.Bool(), r.Float(), r.Float()); break;
                case 15: j.SetAxisSpring((JointAxes)r.Byte(), r.Bool(), r.Float(), r.Float()); break;
                case 16: j.BreakForce = r.Break(); j.BreakTorque = r.Break(); break;
                default: _ = j.ReactionForce + j.ReactionTorque + (float)j.Type + (j.BodyA.IsValid ? 1f : 0f); break;
            }
        }
        catch (Exception e) when (Allowed(e)) { s_editsRefused++; }
    }

    /// <summary>A jointed world snapshot with one joint of every kind, so every record layout is mutated.</summary>
    private static byte[] Base()
    {
        if (s_base != null) { return s_base; }
        var w = Scene(4, out var b);
        w.AddHinge(b[0], b[2], new Vector3D(-4, 3, 0), new Vector3F(0, 0, 1)).SetLimits(-0.5f, 0.5f);
        w.AddSlider(b[1], b[3], new Vector3D(-2, 3.2, 0), new Vector3F(1, 0, 0)).SpringEnabled = true;
        w.AddFixed(b[2], b[4], new Vector3D(0, 3.4, 0));
        w.AddCharacterJoint(b[4], b[5], new Vector3D(3, 3.6, 0), new Vector3F(1, 0, 0));
        w.AddCone(b[5], b[6], new Vector3D(4.5, 6, 0), new Vector3F(1, 0, 0), 0.6f);
        var six = w.AddConfigurable(b[3], b[5], new Vector3D(1, 3.5, 0));
        six.SetAxisMode(JointAxes.AngularZ, JointAxisMode.Free); six.SetAxisMode(JointAxes.LinearX, JointAxisMode.Limited); six.SetAxisLimits(JointAxes.LinearX, -0.1f, 0.1f);
        w.AddSpring(b[1], b[6], new Vector3D(0, 8, 0), new Vector3D(1, 6, 0), 2f, 0.3f);
        w.AddDistance(b[5], b[6], new Vector3D(2, 3.8, 0), new Vector3D(1, 6, 0));
        w.AddGear(b[6], b[3], new Vector3F(0, 0, 1), new Vector3F(0, 0, 1), 1.5f);
        w.AddRackAndPinion(b[2], b[4], new Vector3F(0, 0, 1), new Vector3F(1, 0, 0), 0.5f);
        w.AddPulley(b[3], b[4], b[3].Position, b[4].Position, new Vector3D(-2, 12, 0), new Vector3D(2, 12, 0));
        var path = w.AddPath(b[0], b[6], [new Vector3D(1, 6, 0), new Vector3D(2, 6, 1), new Vector3D(3, 6.5, 2)]);
        path.MotorEnabled = true; path.MotorSpeed = 1f; path.MaxMotorForce = 50f;
        for (int i = 0; i < 20; i++) { w.Step(1f / 60f); }
        s_base = w.Snapshot();
        return s_base;
    }

    public static void SnapshotTarget(byte[] data)
    {
        var bytes = (byte[])Base().Clone();
        int mutations = data.Length > 0 ? 1 + data[0] % 5 : 0;
        for (int i = 1, m = 0; i + 2 < data.Length && m < mutations; i += 3, m++)
        {
            int at = ((data[i] << 8) | data[i + 1]) % bytes.Length;
            bytes[at] ^= data[i + 2];
        }

        if (data.Length > 0 && (data[0] & 0x0F) == 0x0F) { Array.Resize(ref bytes, (data[^1] * bytes.Length) / 256); }

        var w = new PhysicsWorld3D(new WorldOptions3D { WorldId = 9, Substeps = 4 });
        byte[] empty = w.Snapshot();
        try { w.Restore(bytes); }
        catch (SnapshotFormatException)
        {
            s_rejected++;
            if (!w.Snapshot().AsSpan().SequenceEqual(empty)) { throw new InvalidOperationException("a rejected snapshot changed the world"); }
            return;
        }

        s_accepted++;
        if (!w.Snapshot().AsSpan().SequenceEqual(bytes)) { throw new InvalidOperationException("accepted snapshot does not re-serialize to the same bytes"); }
        StepTolerant(w);
    }
}
