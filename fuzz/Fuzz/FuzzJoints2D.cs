using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;

// Fuzz targets for 2D joints. "world2d-joints" turns bytes into a program: bodies, hostile joint definitions (NaN, infinity, huge, tiny, negative, the same body twice, default and destroyed handles),
// edits of live joints through their properties, destroying joints and bodies, stepping, and snapshot round trips. Only argument errors and typed physics errors may come out; anything else is a finding.
// "world2d-joint-snapshot" flips bytes in the snapshot of a jointed world: the restore must be rejected with the world unchanged, or accepted and then re-serialize to the same bytes and step.
internal static class FuzzJoints2D
{
    private static int s_made, s_refused, s_edits, s_editsRefused, s_steps, s_accepted, s_rejected, s_breaks;
    private static byte[]? s_base;

    static FuzzJoints2D()
    {
        AppDomain.CurrentDomain.ProcessExit += (_, _) => Console.WriteLine("joint targets: joints created " + s_made + ", refused " + s_refused + ", edits " + s_edits + ", edits refused " + s_editsRefused + ", steps " + s_steps + ", breaks " + s_breaks + "; snapshots accepted " + s_accepted + ", rejected " + s_rejected);
    }

    private sealed class Reader
    {
        private readonly byte[] _d;
        private int _i;

        public Reader(byte[] d) { _d = d; }

        public bool More => _i < _d.Length;

        public byte Byte() => _i < _d.Length ? _d[_i++] : (byte)0;

        public float Float()
        {
            byte b = Byte();
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

        public Vector2F V2F() => new(Float(), Float());

        public bool Bool() => (Byte() & 1) != 0;
    }

    private static bool Allowed(Exception e) => e is ArgumentException || e is PhysicsException;

    private static void StepTolerant(PhysicsWorld2D w)
    {
        try { w.Step(1f / 60f); s_steps++; }
        catch (BodyOutOfBoundsException e)
        {
            e.Body.Destroy();
        }
    }

    private static Body Pick(Body[] bodies, Reader r)
    {
        byte b = r.Byte();
        if (b >= 240) { return default; }
        return bodies[b % bodies.Length];
    }

    private static PhysicsWorld2D Scene(int substeps, out Body[] bodies)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 9, Substeps = substeps });
        var list = new System.Collections.Generic.List<Body>
        {
            w.AddBox(new Vector2F(40f, 1f), new Vector2D(0, -0.5), type: BodyType.Static),
            w.AddCircle(0.2f, new Vector2D(0, 8), BodyType.Static),
        };
        for (int i = 0; i < 5; i++) { list.Add(w.AddBox(new Vector2F(1f, 0.6f), new Vector2D(-4 + 2 * i, 3 + 0.2 * i), angleRadians: 0.1f * i)); }
        list.Add(w.AddCircle(0.5f, new Vector2D(1, 6)));
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
        var w2 = new PhysicsWorld2D(new WorldOptions2D { WorldId = 9, Substeps = w.Options.Substeps });
        // A body can end the last step outside the supported range or not finite (the next step raises a typed error); the snapshot then holds it and a restore refuses it, which is the documented behavior.
        try { w2.Restore(snap); }
        catch (SnapshotFormatException e) when (e.Message.Contains("out of range", StringComparison.Ordinal) || e.Message.Contains("non-finite", StringComparison.Ordinal)) { return; }
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world hash differs"); }
        if (!w2.Snapshot().AsSpan().SequenceEqual(snap)) { throw new InvalidOperationException("restored world re-serializes differently"); }
        StepTolerant(w); StepTolerant(w2);
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world diverged after a step"); }
    }

    private static void Op(PhysicsWorld2D w, Body[] bodies, System.Collections.Generic.List<Joint> joints, Reader r)
    {
        switch (r.Byte() % 14)
        {
            case 0: { var d = new RevoluteJointDef { BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LocalAnchorA = r.V2F(), LocalAnchorB = r.V2F(), ReferenceAngle = r.Float(), EnableLimit = r.Bool(), LowerAngle = r.Float(), UpperAngle = r.Float(), EnableMotor = r.Bool(), MotorSpeed = r.Float(), MaxMotorTorque = r.Float(), EnableSpring = r.Bool(), SpringHertz = r.Float(), SpringDampingRatio = r.Float(), CollideConnected = r.Bool(), BreakForce = r.Float(), BreakTorque = r.Float() }; Add(joints, w.CreateJoint(in d)); break; }
            case 1: { var d = new PrismaticJointDef { BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LocalAnchorA = r.V2F(), LocalAnchorB = r.V2F(), LocalAxisA = r.V2F(), ReferenceAngle = r.Float(), EnableLimit = r.Bool(), LowerTranslation = r.Float(), UpperTranslation = r.Float(), EnableMotor = r.Bool(), MotorSpeed = r.Float(), MaxMotorForce = r.Float(), EnableSpring = r.Bool(), SpringHertz = r.Float(), SpringDampingRatio = r.Float(), CollideConnected = r.Bool(), BreakForce = r.Float(), BreakTorque = r.Float() }; Add(joints, w.CreateJoint(in d)); break; }
            case 2: { var d = new WeldJointDef { BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LocalAnchorA = r.V2F(), LocalAnchorB = r.V2F(), ReferenceAngle = r.Float(), LinearHertz = r.Float(), LinearDampingRatio = r.Float(), AngularHertz = r.Float(), AngularDampingRatio = r.Float(), CollideConnected = r.Bool(), BreakForce = r.Float(), BreakTorque = r.Float() }; Add(joints, w.CreateJoint(in d)); break; }
            case 3: { var d = new WheelJointDef { BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LocalAnchorA = r.V2F(), LocalAnchorB = r.V2F(), LocalAxisA = r.V2F(), EnableLimit = r.Bool(), LowerTranslation = r.Float(), UpperTranslation = r.Float(), EnableMotor = r.Bool(), MotorSpeed = r.Float(), MaxMotorTorque = r.Float(), EnableSpring = r.Bool(), SpringHertz = r.Float(), SpringDampingRatio = r.Float(), CollideConnected = r.Bool(), BreakForce = r.Float() }; Add(joints, w.CreateJoint(in d)); break; }
            case 4: { var d = new DistanceJointDef { BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LocalAnchorA = r.V2F(), LocalAnchorB = r.V2F(), Length = r.Float(), EnableSpring = r.Bool(), SpringHertz = r.Float(), SpringDampingRatio = r.Float(), EnableLimit = r.Bool(), MinLength = r.Float(), MaxLength = r.Float(), EnableMotor = r.Bool(), MotorSpeed = r.Float(), MaxMotorForce = r.Float(), CollideConnected = r.Bool(), BreakForce = r.Float() }; Add(joints, w.CreateJoint(in d)); break; }
            case 5: { var d = new MotorJointDef { BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LinearOffset = r.V2F(), AngularOffset = r.Float(), MaxForce = r.Float(), MaxTorque = r.Float(), CorrectionFactor = r.Float(), CollideConnected = r.Bool(), BreakForce = r.Float() }; Add(joints, w.CreateJoint(in d)); break; }
            case 6: { var d = new MouseJointDef { BodyA = Pick(bodies, r), BodyB = Pick(bodies, r), LocalAnchorB = r.V2F(), Target = new Vector2D(r.Double(), r.Double()), Hertz = r.Float(), DampingRatio = r.Float(), MaxForce = r.Float(), BreakForce = r.Float() }; Add(joints, w.CreateJoint(in d)); break; }
            case 7: { int n = 1 + r.Byte() % 5; for (int i = 0; i < n; i++) { StepTolerant(w); } break; }
            case 8: Edit(joints, r); break;
            case 9: { if (joints.Count > 0) { joints[r.Byte() % joints.Count].Destroy(); } break; }
            case 10: { var b = Pick(bodies, r); if (b.IsValid && b != bodies[0]) { b.Destroy(); } break; }
            case 11: { var b = Pick(bodies, r); switch (r.Byte() % 3) { case 0: b.SetTransform(new Vector2D(r.Double(), r.Double()), r.Float()); break; case 1: b.Wake(); break; default: b.Sleep(); break; } break; }
            case 12:
                {
                    byte[] snap = w.Snapshot();
                    var w2 = new PhysicsWorld2D(new WorldOptions2D { WorldId = 9, Substeps = w.Options.Substeps });
                    try { w2.Restore(snap); }
                    catch (SnapshotFormatException e) when (e.Message.Contains("out of range", StringComparison.Ordinal) || e.Message.Contains("non-finite", StringComparison.Ordinal)) { break; }
                    if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world hash differs"); }
                    break;
                }
            default:
                {
                    var a = Pick(bodies, r); var b = Pick(bodies, r);
                    var p = new Vector2D(r.Double(), r.Double());
                    switch (r.Byte() % 7)
                    {
                        case 0: Add(joints, w.AddHinge(a, b, p)); break;
                        case 1: Add(joints, w.AddSlider(a, b, p, r.V2F())); break;
                        case 2: Add(joints, w.AddFixed(a, b, p)); break;
                        case 3: Add(joints, w.AddSpring(a, b, p, new Vector2D(r.Double(), r.Double()), r.Float(), r.Float())); break;
                        case 4: Add(joints, w.AddWheel(a, b, p, r.V2F())); break;
                        case 5: Add(joints, w.AddTarget(a, b, p, r.Float(), r.Float(), r.Float())); break;
                        default: Add(joints, w.AddRelative(a, b, r.Float(), r.Float())); break;
                    }

                    break;
                }
        }
    }

    private static void Add(System.Collections.Generic.List<Joint> joints, Joint j)
    {
        joints.Add(j);
        s_made++;
    }

    private static void Edit(System.Collections.Generic.List<Joint> joints, Reader r)
    {
        if (joints.Count == 0) { return; }
        var j = joints[r.Byte() % joints.Count];
        s_edits++;
        try
        {
            switch (r.Byte() % 16)
            {
                case 0: j.EnableLimit = r.Bool(); break;
                case 1: j.SetLimits(r.Float(), r.Float()); break;
                case 2: j.EnableMotor = r.Bool(); break;
                case 3: j.MotorSpeed = r.Float(); break;
                case 4: j.MaxMotorForce = r.Float(); break;
                case 5: j.EnableSpring = r.Bool(); break;
                case 6: j.SpringHertz = r.Float(); break;
                case 7: j.SpringDampingRatio = r.Float(); break;
                case 8: j.Length = r.Float(); break;
                case 9: j.Target = new Vector2D(r.Double(), r.Double()); break;
                case 10: j.MaxForce = r.Float(); break;
                case 11: j.MaxTorque = r.Float(); break;
                case 12: j.SetOffset(r.V2F(), r.Float()); break;
                case 13: j.BreakForce = r.Float(); break;
                case 14: j.SetWeldSprings(r.Float(), r.Float(), r.Float(), r.Float()); break;
                default: _ = j.ReactionForce + j.ReactionTorque + (float)j.Type + (j.BodyA.IsValid ? 1f : 0f); break;
            }
        }
        catch (Exception e) when (Allowed(e)) { s_editsRefused++; }
    }

    private static byte[]? s_baseServices;

    /// <summary>A jointed world snapshot, with or without services state (a layer rule and a sensor), so both encodings of the joint section version are mutated.</summary>
    private static byte[] Base(bool services)
    {
        if (services ? s_baseServices != null : s_base != null) { return services ? s_baseServices! : s_base!; }
        var w = Scene(4, out var b);
        if (services)
        {
            w.IgnoreLayerCollision(2, 5);
            var sensor = w.CreateBody(BodyType.Static, new Vector2D(0, 2));
            w.CreateBox(sensor, new Vector2F(1f, 1f), default, 0f, 0f, new ShapeDef { IsSensor = true, Layer = 2 });
        }
        w.AddHinge(b[0], b[2], new Vector2D(-4, 3)).SetLimits(-0.5f, 0.5f);
        w.AddSlider(b[1], b[3], new Vector2D(-2, 3.2), new Vector2F(1, 0));
        w.AddFixed(b[2], b[4], new Vector2D(0, 3.4));
        var wheel = w.AddWheel(b[4], b[6], new Vector2D(1, 6), new Vector2F(0, 1));
        wheel.EnableMotor = true; wheel.MaxMotorTorque = 5f;
        w.AddSpring(b[1], b[5], new Vector2D(0, 8), new Vector2D(2, 3.8), 2f, 0.3f);
        w.AddDistance(b[5], b[6], new Vector2D(2, 3.8), new Vector2D(1, 6));
        var mouse = w.AddTarget(b[0], b[6], new Vector2D(1, 6));
        mouse.Target = new Vector2D(3, 7);
        w.AddRelative(b[3], b[4], 50f, 50f).SetOffset(new Vector2F(1f, 0f), 0.1f);
        for (int i = 0; i < 20; i++) { w.Step(1f / 60f); }
        var snap = w.Snapshot();
        if (services) { s_baseServices = snap; } else { s_base = snap; }
        return snap;
    }

    public static void SnapshotTarget(byte[] data)
    {
        var bytes = (byte[])Base(data.Length > 0 && (data[0] & 0x10) != 0).Clone();
        int mutations = data.Length > 0 ? 1 + data[0] % 5 : 0;
        for (int i = 1, m = 0; i + 2 < data.Length && m < mutations; i += 3, m++)
        {
            int at = ((data[i] << 8) | data[i + 1]) % bytes.Length;
            bytes[at] ^= data[i + 2];
        }

        if (data.Length > 0 && (data[0] & 0x0F) == 0x0F) { Array.Resize(ref bytes, (data[^1] * bytes.Length) / 256); }

        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 9, Substeps = 4 });
        // A world to restore into: same sections, empty; then compare after a rejected restore.
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
        int before = w.JointCount;
        StepTolerant(w);
        StepTolerant(w);
        if (w.JointCount > before) { throw new InvalidOperationException("stepping created joints"); }
        if (w.JointCount < before) { s_breaks++; }
    }
}
