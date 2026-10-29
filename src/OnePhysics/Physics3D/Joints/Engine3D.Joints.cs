using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Indices into the per-joint scratch row that <c>PrepareJoint</c> fills once per step and the solver reads.</summary>
internal static class JRow3
{
    public const int Stride = 80;
    public const int RA = 0, RB = 3, D = 6;                      // anchors from the centres of mass at the start-of-step poses; centre of mass of B minus that of A
    public const int FA = 9, FB = 13;                            // joint frames in world axes at the start of the step (quaternions x y z w)
    public const int JBias = 17, JMass = 18, JImp = 19;          // soft coefficients of the joint's rigid rows
    public const int SBias = 20, SMass = 21, SImp = 22;          // user spring
    public const int MaxMotorImp = 23;                           // force/torque limit times the substep
    public const int PulleyA = 24, PulleyB = 27;                 // start-of-step centre of mass minus the fixed point
    public const int PathP = 30, PathT = 33, PathN = 36, PathB = 39, PathEnd = 42;   // path: point (from A's centre of mass), frame in world axes, which end is clamped (-1, 0, 1)
    public const int SixSpring = 46;                             // 3 floats per axis: stiffness, mass scale, impulse scale
    public const int SixMaxMotor = 64;                           // 6 floats
    public const int SwingLimit = 70;                            // sin(half angle / 2) of swing y and z
}

internal sealed partial class Engine3D
{
    public JointData3D[] Joints = new JointData3D[16];
    public float[] JScratch = new float[JRow3.Stride * 16];
    public float[] JointForce = new float[16];     // reaction force and torque of the last step the joint was solved, N and N m; not state
    public float[] JointTorque = new float[16];
    private int _noCollideJoints;
    private int _trackedJoints;
    private bool _needBind;
    private float _pendingJointHertz, _pendingJointDamping;

    private const uint VersionJoints = 2;
    private const uint BlockJoints = 1;

    private int LiveJointCount()
    {
        var cs = Core.Constraints;
        int n = 0;
        for (int c = 0; c < cs.HighWater; c++) { if (cs.IsLive(c) && cs.KindOf(c) == ConstraintKind.Joint) { n++; } }
        return n;
    }

    private void EnsureJointSlots(int n)
    {
        if (Joints.Length < n) { Array.Resize(ref Joints, Math.Max(n, Joints.Length * 2)); }
        if (JointForce.Length < n) { Array.Resize(ref JointForce, Joints.Length); Array.Resize(ref JointTorque, Joints.Length); }
    }

    // ------------------------------------------------------------------ creation

    /// <summary>Creates a joint between two live bodies. The record is validated before anything changes.</summary>
    public JointHandle CreateJoint(BodyHandle ha, BodyHandle hb, JointData3D d)
    {
        EnsureBound();
        int a = Core.Resolve(ha), b = Core.Resolve(hb);
        string name = JointData3D.Name((JointType3D)d.Type);
        if (a == b) { throw new InvalidJointException(name + " joint: both bodies are " + ha + "; a joint needs two different bodies."); }
        var bodies = Core.Bodies;
        if (!bodies.IsDynamic(a) && !bodies.IsDynamic(b)) { throw new InvalidJointException(name + " joint between " + ha + " and " + hb + ": at least one body must be dynamic."); }
        d.Validate();
        d.Prepare();
        var desc = new ConstraintDesc { Kind = ConstraintKind.Joint, BodyA = ha, BodyB = hb };
        int c = Core.AddConstraint(desc);
        EnsureJointSlots(c + 1);
        if (Joints[c].Gen != 0) { JointGone(c); }
        d.BodyA = a; d.BodyB = b; d.GenA = ha.Generation; d.GenB = hb.Generation;
        d.Gen = Core.Constraints.Handles.GenerationOf(c);
        Joints[c] = d;
        _trackedJoints++;
        if (!d.Collide)
        {
            _noCollideJoints++;
            DropContactsBetween(a, b);
        }

        return Core.HandleOfJoint(c);
    }

    /// <summary>Removes the contacts between all shape pairs of two bodies (they stop colliding because a joint now connects them).</summary>
    private void DropContactsBetween(int a, int b)
    {
        for (int sa = ShapeHead[a]; sa >= 0; sa = NextShape[sa])
        {
            for (int sb = ShapeHead[b]; sb >= 0; sb = NextShape[sb])
            {
                int lo = Math.Min(sa, sb), hi = Math.Max(sa, sb);
                if (!PairMap.TryGetValue(PairKey(lo, hi), out int p)) { continue; }
                for (int c = PairCons[p]; c >= 0;)
                {
                    int next = ConsNext[c];
                    Core.RemoveConstraint(c);
                    c = next;
                }

                FreePair(p);
            }
        }
    }

    /// <summary>The core removed this joint (it broke, a body was destroyed, or it was destroyed): if it kept its bodies from colliding, let them find each other again.</summary>
    private void JointGone(int c)
    {
        ref JointData3D j = ref Joints[c];
        j.Gen = 0;
        _trackedJoints--;
        if (j.Collide) { return; }
        _noCollideJoints--;
        var h = Core.Bodies.Handles;
        if (h.IsAlive(j.BodyA, j.GenA)) { TouchBody(j.BodyA); }
        if (h.IsAlive(j.BodyB, j.GenB)) { TouchBody(j.BodyB); }
    }

    private void TouchBody(int b)
    {
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s]) { Bp.Touch(Proxy[s]); }
    }

    /// <summary>
    /// Notices joints the core removed (a break, a destroyed body or joint) and lets the bodies of a joint that kept them from colliding find each other again. It runs as soon as such a removal can have happened
    /// (the end of a step, destroying a joint or a body) and at the start of a step, for removals made through the raw tier.
    /// </summary>
    internal void ReconcileJoints()
    {
        EnsureBound();
        if (_trackedJoints == 0) { return; }
        var cs = Core.Constraints;
        int n = Math.Min(cs.HighWater, Joints.Length);
        for (int c = 0; c < n; c++)
        {
            int g = Joints[c].Gen;
            if (g == 0) { continue; }
            if (cs.IsLive(c) && cs.KindOf(c) == ConstraintKind.Joint && cs.Handles.GenerationOf(c) == g) { continue; }
            JointGone(c);
        }
    }

    /// <summary>Sizes the per-step joint storage; called at the end of the contact stage.</summary>
    private void SizeJointStorage()
    {
        int ch = Core.Constraints.HighWater;
        if (JScratch.Length < ch * JRow3.Stride) { JScratch = new float[Math.Max(ch, JScratch.Length / JRow3.Stride * 2) * JRow3.Stride]; }
        EnsureJointSlots(ch);
    }

    internal void BindJoints() => EnsureBound();

    private void EnsureBound()
    {
        if (!_needBind) { return; }
        _needBind = false;
        var cs = Core.Constraints;
        var bodies = Core.Bodies;
        _noCollideJoints = 0;
        _trackedJoints = 0;
        EnsureJointSlots(cs.HighWater);
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Joint || Joints[c].Type == 0) { Joints[c].Gen = 0; continue; }
            ref JointData3D j = ref Joints[c];
            j.BodyA = cs.BodyA[c]; j.BodyB = cs.BodyB[c];
            j.GenA = bodies.Handles.GenerationOf(j.BodyA); j.GenB = bodies.Handles.GenerationOf(j.BodyB);
            j.Gen = cs.Handles.GenerationOf(c);
            _trackedJoints++;
            if (!j.Collide) { _noCollideJoints++; }
        }
    }

    /// <summary>The pair filter's question: does a joint between these two bodies keep them from colliding? Read-only, so it may run on worker threads.</summary>
    private bool JointsPreventCollision(int a, int b)
    {
        if (_noCollideJoints == 0) { return false; }
        var bodies = Core.Bodies;
        if (!bodies.IsDynamic(a)) { (a, b) = (b, a); }
        if (!bodies.IsDynamic(a)) { return false; }
        var cs = Core.Constraints;
        for (int e = bodies.FirstEdge(a); e >= 0; e = cs.NextEdge(e))
        {
            int c = e >> 1;
            if (cs.KindOf(c) != ConstraintKind.Joint || Joints[c].Collide) { continue; }
            int other = (e & 1) == 0 ? cs.BodyB[c] : cs.BodyA[c];
            if (other == b) { return true; }
        }

        return false;
    }

    /// <summary>After the core shifted the origin: pulley fixed points are world positions, so they shift with it.</summary>
    internal void ShiftJoints(ReadOnlySpan<double> offset)
    {
        EnsureBound();
        var cs = Core.Constraints;
        for (int c = 0; c < cs.HighWater && c < Joints.Length; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Joint || Joints[c].Type != (byte)JointType3D.Pulley) { continue; }
            ref JointData3D j = ref Joints[c];
            j.FxA -= offset[0]; j.FyA -= offset[1]; j.FzA -= offset[2];
            j.FxB -= offset[0]; j.FyB -= offset[1]; j.FzB -= offset[2];
        }
    }

    // ------------------------------------------------------------------ snapshot

    private void WriteJoints(StateWriter w)
    {
        var cs = Core.Constraints;
        w.WriteSingle(Opt.JointHertz); w.WriteSingle(Opt.JointDampingRatio);
        int n = cs.HighWater;
        w.WriteInt32(n);
        for (int c = 0; c < n; c++)
        {
            bool live = cs.IsLive(c) && cs.KindOf(c) == ConstraintKind.Joint && Joints[c].Type != 0;
            w.WriteBool(live);
            if (live) { Joints[c].Write(w); }
        }
    }

    private JointData3D[] ReadJoints(StateReader r)
    {
        float hertz = r.ReadSingle(), zeta = r.ReadSingle();
        if (!(hertz >= 0f && zeta >= 0f) || !float.IsFinite(hertz + zeta)) { throw r.Error("invalid joint tunables"); }
        int n = r.ReadCount(1, "joint slots");
        if (n > 1 << 24) { throw r.Error("too many joint slots"); }
        var joints = _sJoints;
        if (joints.Length < Math.Max(n, 16)) { joints = _sJoints = new JointData3D[Math.Max(n, 16)]; } else { Array.Clear(joints); }
        int live = 0;
        for (int c = 0; c < n; c++)
        {
            if (!r.ReadBool()) { continue; }
            if (r.Remaining < JointData3D.MinRecordBytes) { throw r.Error("joint record is longer than the remaining bytes"); }
            joints[c] = JointData3D.Read(r);
            live++;
        }

        if (live == 0) { throw r.Error("3D section holds a joints block with no joints"); }
        _pendingJointHertz = hertz; _pendingJointDamping = zeta;
        return joints;
    }

    /// <summary>Checks the joint records against the core's constraints after a restore.</summary>
    private bool JointsConsistentWithCore()
    {
        EnsureBound();
        var cs = Core.Constraints;
        for (int c = 0; c < cs.HighWater; c++)
        {
            bool isJoint = cs.IsLive(c) && cs.KindOf(c) == ConstraintKind.Joint;
            bool has = c < Joints.Length && Joints[c].Type != 0;
            if (isJoint != has) { return false; }
        }

        return true;
    }

    // ------------------------------------------------------------------ prepare

    /// <summary>
    /// Coefficients of a user spring (frequency <paramref name="hertz"/>, damping ratio <paramref name="zeta"/>) for one solve per substep: impulse = -m (massScale * relativeVelocity + stiffness * error) - impulseScale * accumulated.
    /// Contact softness adds numerical damping of about the substep times the frequency (Catto, "Soft Constraints", GDC 2011), which is wrong for a spring someone tuned. Here the per-substep map (x, v) to (x + h v', v'),
    /// with v' = (1 - massScale) v - stiffness x, is made to have exactly the eigenvalues of the continuous oscillator over one substep: determinant e^(-2 zeta w h) and trace 2 e^(-zeta w h) cos(w h sqrt(1 - zeta^2))
    /// (cosh when zeta is above 1), which fixes massScale = 1 - determinant and stiffness = (2 - massScale - trace) / h.
    /// </summary>
    private static void MakeSpring(float hertz, float zeta, float h, out float stiffness, out float massScale, out float impulseScale)
    {
        if (!(hertz > 0f)) { stiffness = 0f; massScale = 1f; impulseScale = 0f; return; }
        double wh = 2.0 * Math.PI * hertz * h;
        if (wh > Math.PI) { wh = Math.PI; } // a spring faster than the substep rate means nothing: stay at half a turn per substep
        double zh = zeta * wh;
        double det = Math64.Exp(-2.0 * zh);
        double decay = Math64.Exp(-zh);
        double trace;
        if (zeta < 1f) { trace = 2.0 * decay * Math64.Cos(wh * Math.Sqrt(1.0 - (double)zeta * zeta)); }
        else
        {
            double x = wh * Math.Sqrt((double)zeta * zeta - 1.0);
            trace = Math64.Exp(x - zh) + Math64.Exp(-x - zh);
        }

        double ms = 1.0 - det;
        massScale = (float)ms;
        impulseScale = (float)det;
        stiffness = (float)((2.0 - ms - trace) / h);
    }

    private void PrepareJoint(int c, in StepContext ctx)
    {
        var core = Core;
        var cs = core.Constraints;
        var bodies = core.Bodies;
        ref JointData3D j = ref Joints[c];
        int a = cs.BodyA[c], b = cs.BodyB[c];
        var s = JScratch.AsSpan(c * JRow3.Stride, JRow3.Stride);
        s.Clear();
        var pA = bodies.PositionOf(a); var pB = bodies.PositionOf(b);
        var qA = RotationOfBody(a); var qB = RotationOfBody(b);
        var lcA = LocalCenterOf(a); var lcB = LocalCenterOf(b);
        var rA = qA.Rotate(new Vector3F(j.LAx - lcA.X, j.LAy - lcA.Y, j.LAz - lcA.Z));
        var rB = qB.Rotate(new Vector3F(j.LBx - lcB.X, j.LBy - lcB.Y, j.LBz - lcB.Z));
        Put(s, JRow3.RA, rA); Put(s, JRow3.RB, rB);
        Put(s, JRow3.D, new Vector3F((float)(pB[0] - pA[0]), (float)(pB[1] - pA[1]), (float)(pB[2] - pA[2])));
        var fa = qA * j.QA; var fb = qB * j.QB;
        s[JRow3.FA] = fa.X; s[JRow3.FA + 1] = fa.Y; s[JRow3.FA + 2] = fa.Z; s[JRow3.FA + 3] = fa.W;
        s[JRow3.FB] = fb.X; s[JRow3.FB + 1] = fb.Y; s[JRow3.FB + 2] = fb.Z; s[JRow3.FB + 3] = fb.W;

        float h = ctx.SubstepDt;
        float cap = 0.25f * ctx.InverseSubstepDt;
        float jh = Opt.JointHertz > cap ? cap : Opt.JointHertz;
        MakeSoft(jh, Opt.JointDampingRatio, h, out float bias, out float ms, out float isc);
        s[JRow3.JBias] = bias; s[JRow3.JMass] = ms; s[JRow3.JImp] = isc;
        MakeSpring(j.Hertz, j.Damping, h, out bias, out ms, out isc);
        s[JRow3.SBias] = bias; s[JRow3.SMass] = ms; s[JRow3.SImp] = isc;
        s[JRow3.MaxMotorImp] = h * j.MaxMotor;
        switch ((JointType3D)j.Type)
        {
            case JointType3D.SwingTwist:
                s[JRow3.SwingLimit] = (float)Math64.Sin(0.5 * j.Swing1);
                s[JRow3.SwingLimit + 1] = (float)Math64.Sin(0.5 * j.Swing2);
                break;
            case JointType3D.SixDof:
                for (int i = 0; i < 6; i++)
                {
                    MakeSpring(j.Six.Hertz[i], j.Six.Damping[i], h, out bias, out ms, out isc);
                    s[JRow3.SixSpring + 3 * i] = bias; s[JRow3.SixSpring + 3 * i + 1] = ms; s[JRow3.SixSpring + 3 * i + 2] = isc;
                    s[JRow3.SixMaxMotor + i] = h * j.Six.MaxMotor[i];
                }

                break;
            case JointType3D.Pulley:
                Put(s, JRow3.PulleyA, new Vector3F((float)(pA[0] - j.FxA), (float)(pA[1] - j.FyA), (float)(pA[2] - j.FzA)));
                Put(s, JRow3.PulleyB, new Vector3F((float)(pB[0] - j.FxB), (float)(pB[1] - j.FyB), (float)(pB[2] - j.FzB)));
                break;
            case JointType3D.Path:
                PreparePath(ref j, s, qA, lcA, new Vector3F((float)(pB[0] - pA[0]), (float)(pB[1] - pA[1]), (float)(pB[2] - pA[2])), rB);
                break;
        }
    }

    private void ClearJointIdx(int group)
    {
        var idx = WideJIdx.AsSpan(group * J3D.IdxPerGroup, J3D.IdxPerGroup);
        for (int l = 0; l < J3D.Lanes; l++) { idx[l] = 0; idx[8 + l] = 0; idx[16 + l] = -1; idx[24 + l] = -1; }
    }

    /// <summary>Once per step: a point joint's start-of-step anchors, centre-of-mass offset, inverse masses and warm-start impulse into one lane of its batch's joint group.</summary>
    private void PreparePointJoint(int c, int group, int lane)
    {
        var core = Core;
        var cs = core.Constraints;
        var bodies = core.Bodies;
        ref JointData3D j = ref Joints[c];
        int a = cs.BodyA[c], b = cs.BodyB[c];
        var w = WideJ.AsSpan(group * J3D.GroupFloats, J3D.GroupFloats);
        var idx = WideJIdx.AsSpan(group * J3D.IdxPerGroup, J3D.IdxPerGroup);
        SlotOf[c] = group * J3D.Lanes + lane;
        float mA = bodies.InverseMass[a], mB = bodies.InverseMass[b];
        idx[lane] = a; idx[8 + lane] = b;
        idx[16 + lane] = mA > 0f ? a : -1; idx[24 + lane] = mB > 0f ? b : -1;
        var pA = bodies.PositionOf(a); var pB = bodies.PositionOf(b);
        var lcA = LocalCenterOf(a); var lcB = LocalCenterOf(b);
        var rA = RotationOfBody(a).Rotate(new Vector3F(j.LAx - lcA.X, j.LAy - lcA.Y, j.LAz - lcA.Z));
        var rB = RotationOfBody(b).Rotate(new Vector3F(j.LBx - lcB.X, j.LBy - lcB.Y, j.LBz - lcB.Z));
        Put3(w, J3D.RA, lane, rA); Put3(w, J3D.RB, lane, rB);
        Put3(w, J3D.D, lane, new Vector3F((float)(pB[0] - pA[0]), (float)(pB[1] - pA[1]), (float)(pB[2] - pA[2])));
        w[J3D.InvMassA * 8 + lane] = mA; w[J3D.InvMassB * 8 + lane] = mB;
        var p = cs.Payload(c);
        for (int k = 0; k < 3; k++) { w[(J3D.Impulse + k) * 8 + lane] = p[k]; }
    }

    private static void PreparePath(ref JointData3D j, Span<float> s, QuaternionF qA, Vector3F lcA, Vector3F d, Vector3F rB)
    {
        // The follower's anchor in A's origin frame: the vector from A's origin to it, in A's axes.
        var anchor = qA.Conjugated().Rotate(d + qA.Rotate(lcA) + rB);
        PathSpline3D.Closest(j.Path!, (j.Flags & JointData3D.FlagLoop) != 0, anchor, out float t, out var pos, out var tangent);
        var up = new Vector3F(j.UpX, j.UpY, j.UpZ);
        PathSpline3D.Frame(tangent, up, out var tn, out var nn, out var bn);
        int end = 0;
        if ((j.Flags & JointData3D.FlagLoop) == 0)
        {
            float last = j.Path!.Length - 1;
            float along = Vector3F.Dot(anchor - pos, tn);
            if (t <= 1e-6f && along < 0f) { end = -1; }
            else if (t >= last - 1e-6f && along > 0f) { end = 1; }
        }

        Put(s, JRow3.PathP, qA.Rotate(pos - lcA));
        Put(s, JRow3.PathT, qA.Rotate(tn)); Put(s, JRow3.PathN, qA.Rotate(nn)); Put(s, JRow3.PathB, qA.Rotate(bn));
        s[JRow3.PathEnd] = end;
    }

    // ------------------------------------------------------------------ results and breaking

    /// <summary>After the last substep: reaction force and torque of each joint from its accumulated impulses (impulse of the last substep divided by the substep), and a break request when a threshold is exceeded.</summary>
    public void StoreJointResults(ReadOnlySpan<int> constraints, in StepContext ctx)
    {
        var cs = Core.Constraints;
        float inv = ctx.InverseSubstepDt;
        foreach (int c in constraints)
        {
            if (cs.KindOf(c) != ConstraintKind.Joint) { continue; }
            ref JointData3D j = ref Joints[c];
            var p = cs.Payload(c);
            JointReaction(in j, p, out float lin, out float ang);
            float force = lin * inv, torque = ang * inv;
            JointForce[c] = force; JointTorque[c] = torque;
            if (force > j.BreakForce) { cs.Flags[c] |= (byte)ConstraintBits.BreakRequested; cs.BreakValue[c] = force; }
            else if (torque > j.BreakTorque) { cs.Flags[c] |= (byte)ConstraintBits.BreakRequested; cs.BreakValue[c] = torque; }
        }
    }

    private static float Mag(float x, float y, float z) => float.Sqrt(x * x + y * y + z * z);

    private static void JointReaction(in JointData3D j, Span<float> p, out float lin, out float ang)
    {
        float axial = p[JP.Spring] + p[JP.Motor] + p[JP.Lower] - p[JP.Upper];
        switch ((JointType3D)j.Type)
        {
            case JointType3D.Fixed: lin = Mag(p[0], p[1], p[2]); ang = Mag(p[3], p[4], p[5]); break;
            case JointType3D.Point: lin = Mag(p[0], p[1], p[2]); ang = 0f; break;
            case JointType3D.Distance: lin = p[0] + axial; lin = lin < 0f ? -lin : lin; ang = 0f; break;
            case JointType3D.Hinge: lin = Mag(p[0], p[1], p[2]); ang = Mag(p[3], p[4], axial); break;
            case JointType3D.Slider: lin = Mag(p[0], p[1], axial); ang = Mag(p[3], p[4], p[5]); break;
            case JointType3D.Cone: lin = Mag(p[0], p[1], p[2]); ang = p[JP.Lower] < 0f ? -p[JP.Lower] : p[JP.Lower]; break;
            case JointType3D.SwingTwist: lin = Mag(p[0], p[1], p[2]); ang = Mag(p[JP.Lower] - p[JP.Upper], p[JP.Swing], j.Drive ? Mag(p[JP.Drive], p[JP.Drive + 1], p[JP.Drive + 2]) : 0f); break;
            case JointType3D.SixDof:
            {
                float lx = SixAxial(p, 0), ly = SixAxial(p, 1), lz = SixAxial(p, 2);
                float ax = SixAxial(p, 3), ay = SixAxial(p, 4), az = SixAxial(p, 5);
                lin = Mag(lx, ly, lz); ang = Mag(ax, ay, az);
                break;
            }

            case JointType3D.Path: lin = Mag(p[0], p[1], axial); ang = Mag(p[3], p[4], p[5]); break;
            case JointType3D.Gear: lin = 0f; ang = p[0] < 0f ? -p[0] : p[0]; break;
            case JointType3D.RackAndPinion: lin = 0f; ang = p[0] < 0f ? -p[0] : p[0]; break;
            default: lin = p[0] + p[JP.Lower] - p[JP.Upper]; lin = lin < 0f ? -lin : lin; ang = 0f; break; // pulley
        }
    }

    private static float SixAxial(Span<float> p, int i)
    {
        int o = JP.SixAxis + 4 * i;
        return p[i] + p[o] + p[o + 1] - p[o + 2] + p[o + 3];
    }
}
