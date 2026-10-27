using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>Indices into the per-joint scratch row that <c>PrepareJoint</c> fills once per step and the solver reads.</summary>
internal static class JRow
{
    public const int Stride = 40;
    public const int RAx = 0, RAy = 1, RBx = 2, RBy = 3;      // anchors from the centres of mass, rotated to the start-of-step poses
    public const int Dx = 4, Dy = 5;                          // centre of mass of B minus that of A
    public const int MA = 6, IA = 7, MB = 8, IB = 9;
    public const int AxX = 10, AxY = 11;                      // axis (prismatic, wheel) at the start of the step
    public const int Q0C = 12, Q0S = 13;                      // relative rotation (B relative to A, minus the reference) at the start of the step
    public const int JBias = 14, JMass = 15, JImp = 16;       // soft coefficients of the joint's rigid rows
    public const int SBias = 17, SMass = 18, SImp = 19;       // spring (or weld linear spring)
    public const int ABias = 20, AMass = 21, AImp = 22;       // weld angular spring
    public const int MaxMotorImp = 23, MaxMotorImp2 = 24;    // force/torque limits times the substep
    public const int AngMass = 25;                            // 1 / (iA + iB)
    public const int TgtX = 26, TgtY = 27;                    // mouse target, or the motor joint's offset, relative to the start-of-step centre of mass of B / rotated into the world
}

/// <summary>Layout of a joint's persistent float payload (accumulated impulses, one slot per row; at most six rows fit in the core's payload).</summary>
internal static class JPay
{
    public const int P0 = 0, P1 = 1, P2 = 2;                 // point impulse (x, y) / perpendicular and angular rows / angular row
    public const int Spring = 3, Motor = 4, Lower = 5, Upper = 6;
    public const int Used = 7;
}

internal sealed partial class Engine2D
{
    public JointData2D[] Joints = new JointData2D[16];
    public float[] JScratch = new float[JRow.Stride * 16];
    public float[] JointForce = new float[16];     // reaction force and torque of the last step the joint was solved, N and N m; not state
    public float[] JointTorque = new float[16];
    private int _noCollideJoints;
    private int _trackedJoints;
    private bool _needBind;

    /// <summary>True when any joint exists or ever existed in this section's data; decides the snapshot section version.</summary>
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
    public JointHandle CreateJoint(BodyHandle ha, BodyHandle hb, JointData2D d)
    {
        EnsureBound();
        int a = Core.Resolve(ha), b = Core.Resolve(hb);
        string name = JointData2D.Name((JointType2D)d.Type);
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
                if (PairCons[p] >= 0) { Core.RemoveConstraint(PairCons[p]); }
                FreePair(p);
            }
        }
    }

    /// <summary>The core removed this joint (it broke, a body was destroyed, or it was destroyed): if it kept its bodies from colliding, let them find each other again.</summary>
    private void JointGone(int c)
    {
        ref JointData2D j = ref Joints[c];
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
    /// (the end of a step, destroying a joint or a body), so a snapshot never holds a removal whose effect is still pending; it runs at the start of a step too, for removals made through the raw tier.
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

    /// <summary>Sizes the per-step joint storage; called by the contact stage.</summary>
    private void SizeJointStorage()
    {
        var cs = Core.Constraints;
        int ch = cs.HighWater;
        if (JScratch.Length < ch * JRow.Stride) { JScratch = new float[Math.Max(ch, JScratch.Length / JRow.Stride * 2) * JRow.Stride]; }
        EnsureJointSlots(ch);
    }

    /// <summary>After a restore, rebuilds the runtime fields of every joint record from the core's constraints.</summary>
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
            ref JointData2D j = ref Joints[c];
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

    // ------------------------------------------------------------------ snapshot

    /// <summary>Section version 2 carries the joint records; a world with no joints still writes version 1, so its snapshot and hash are what they were before joints existed.</summary>
    private uint SectionVersion() => LiveJointCount() > 0 ? 2u : 1u;

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

    private void ReadJoints(StateReader r, out JointData2D[] joints)
    {
        float hertz = r.ReadSingle(), zeta = r.ReadSingle();
        if (!(hertz >= 0f && zeta >= 0f) || !float.IsFinite(hertz + zeta)) { throw r.Error("invalid joint tunables"); }
        int n = r.ReadCount(1, "joint slots");
        if (n > 1 << 24) { throw r.Error("too many joint slots"); }
        joints = new JointData2D[Math.Max(n, 16)];
        for (int c = 0; c < n; c++)
        {
            if (!r.ReadBool()) { continue; }
            if (r.Remaining < JointData2D.RecordBytes) { throw r.Error("joint record is longer than the remaining bytes"); }
            joints[c] = JointData2D.Read(r);
        }

        _pendingJointHertz = hertz; _pendingJointDamping = zeta;
    }

    private float _pendingJointHertz, _pendingJointDamping;

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

    // ------------------------------------------------------------------ solving a batch that holds joints

    [System.Runtime.CompilerServices.InlineArray(ConstraintColoring.Lanes)]
    private struct BatchBuffer
    {
        private int _e;
    }

    /// <summary>
    /// A batch of one color that holds joints and contacts: joints are solved one by one, the contacts go through the lane kernel together. They share no dynamic body, so doing the joints first changes nothing.
    /// The overflow set shares bodies and is solved strictly one constraint at a time in list order.
    /// </summary>
    private void SolveMixed(ReadOnlySpan<int> constraints, int joints, bool overflow, SolvePhase phase, in StepContext ctx, in KernelParams kp)
    {
        var cs = Core.Constraints;
        if (overflow)
        {
            foreach (int c in constraints)
            {
                if (cs.KindOf(c) == ConstraintKind.Joint) { SolveJoint(c, phase, in ctx); }
                else { ContactKernel2D.Run<Lanes1>(this, new ReadOnlySpan<int>(in c), phase, in kp); }
            }

            return;
        }

        BatchBuffer buffer = default;
        Span<int> contacts = buffer;
        int n = 0;
        foreach (int c in constraints)
        {
            if (cs.KindOf(c) == ConstraintKind.Joint) { SolveJoint(c, phase, in ctx); }
            else { contacts[n++] = c; }
        }

        if (n == 0) { return; }
        var list = contacts.Slice(0, n);
        if (KernelWidth == 1)
        {
            for (int i = 0; i < n; i++) { ContactKernel2D.Run<Lanes1>(this, list.Slice(i, 1), phase, in kp); }
        }
        else if (KernelWidth == 4)
        {
            for (int i = 0; i < n; i += 4) { ContactKernel2D.Run<Lanes128>(this, list.Slice(i, Math.Min(4, n - i)), phase, in kp); }
        }
        else
        {
            ContactKernel2D.Run<Lanes256>(this, list, phase, in kp);
        }
    }

    // ------------------------------------------------------------------ prepare

    /// <summary>
    /// Coefficients of a user spring (frequency <paramref name="hertz"/>, damping ratio <paramref name="zeta"/>) for one solve per substep: impulse = -m (massScale * relativeVelocity + stiffness * error) - impulseScale * accumulated.
    /// The soft-contact coefficients of <see cref="MakeSoft"/> add numerical damping of about the substep times the frequency (implicit integration is dissipative; Catto, "Soft Constraints", GDC 2011), which is
    /// fine for contacts and wrong for a spring someone tuned. Here the per-substep map (x, v) to (x + h v', v'), with v' = (1 - massScale) v - stiffness x, is made to have exactly the eigenvalues of the
    /// continuous oscillator over one substep: determinant e^(-2 zeta w h) and trace 2 e^(-zeta w h) cos(w h sqrt(1 - zeta^2)) (cosh when zeta is above 1), which fixes massScale = 1 - determinant and
    /// stiffness = (2 - massScale - trace) / h. The impulse scale is 1 - massScale, so repeated solves converge to the same impulse.
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
            // 2 e^(-zh) cosh(x) with x = wh sqrt(zeta^2 - 1) <= zh, written as two decaying exponentials so a huge damping ratio cannot overflow.
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
        ref JointData2D j = ref Joints[c];
        int a = cs.BodyA[c], b = cs.BodyB[c];
        var s = JScratch.AsSpan(c * JRow.Stride, JRow.Stride);
        var pA = bodies.PositionOf(a); var pB = bodies.PositionOf(b);
        var qA = bodies.RotationOf(a); var qB = bodies.RotationOf(b);
        float qAc = qA[0], qAs = qA[1], qBc = qB[0], qBs = qB[1];
        float xa = j.LAx - LocalCenter[2 * a], ya = j.LAy - LocalCenter[2 * a + 1];
        float xb = j.LBx - LocalCenter[2 * b], yb = j.LBy - LocalCenter[2 * b + 1];
        s[JRow.RAx] = qAc * xa - qAs * ya; s[JRow.RAy] = qAs * xa + qAc * ya;
        s[JRow.RBx] = qBc * xb - qBs * yb; s[JRow.RBy] = qBs * xb + qBc * yb;
        s[JRow.Dx] = (float)(pB[0] - pA[0]); s[JRow.Dy] = (float)(pB[1] - pA[1]);
        s[JRow.MA] = bodies.InverseMass[a]; s[JRow.IA] = bodies.InverseInertia[a]; s[JRow.MB] = bodies.InverseMass[b]; s[JRow.IB] = bodies.InverseInertia[b];
        s[JRow.AxX] = qAc * j.AxX - qAs * j.AxY; s[JRow.AxY] = qAs * j.AxX + qAc * j.AxY;
        // Relative rotation of B in A's frame, with the reference angle taken out: conj(qA) * qB * conj(ref).
        float relC = qAc * qBc + qAs * qBs, relS = qAc * qBs - qAs * qBc;
        s[JRow.Q0C] = relC * j.RefC + relS * j.RefS; s[JRow.Q0S] = relS * j.RefC - relC * j.RefS;

        float h = ctx.SubstepDt;
        float cap = 0.25f * ctx.InverseSubstepDt;
        float jh = Opt.JointHertz > cap ? cap : Opt.JointHertz;
        MakeSoft(jh, Opt.JointDampingRatio, h, out float bias, out float ms, out float isc);
        s[JRow.JBias] = bias; s[JRow.JMass] = ms; s[JRow.JImp] = isc;
        MakeSpring(j.Hertz, j.Damping, h, out bias, out ms, out isc);
        s[JRow.SBias] = bias; s[JRow.SMass] = ms; s[JRow.SImp] = isc;
        MakeSpring(j.Hertz2, j.Damping2, h, out bias, out ms, out isc);
        s[JRow.ABias] = bias; s[JRow.AMass] = ms; s[JRow.AImp] = isc;
        s[JRow.MaxMotorImp] = h * j.MaxMotor; s[JRow.MaxMotorImp2] = h * j.MaxMotor2;
        float k = s[JRow.IA] + s[JRow.IB];
        s[JRow.AngMass] = k > 0f ? 1f / k : 0f;
        if (j.Type == (byte)JointType2D.Mouse)
        {
            s[JRow.TgtX] = (float)(j.TargetX - pB[0]); s[JRow.TgtY] = (float)(j.TargetY - pB[1]);
        }
        else if (j.Type == (byte)JointType2D.Motor)
        {
            s[JRow.TgtX] = qAc * j.OffX - qAs * j.OffY; s[JRow.TgtY] = qAs * j.OffX + qAc * j.OffY;
        }
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
            ref JointData2D j = ref Joints[c];
            var p = cs.Payload(c);
            float lx, ly, torque;
            float axial = p[JPay.Spring] + p[JPay.Motor] + p[JPay.Lower] - p[JPay.Upper];
            switch ((JointType2D)j.Type)
            {
                case JointType2D.Revolute: lx = p[JPay.P0]; ly = p[JPay.P1]; torque = axial; break;
                case JointType2D.Weld: lx = p[JPay.P0]; ly = p[JPay.P1]; torque = p[JPay.P2]; break;
                case JointType2D.Motor: lx = p[JPay.P0]; ly = p[JPay.P1]; torque = p[JPay.P2]; break;
                case JointType2D.Mouse: lx = p[JPay.P0]; ly = p[JPay.P1]; torque = 0f; break;
                case JointType2D.Prismatic: lx = p[JPay.P0]; ly = axial; torque = p[JPay.P1]; break;
                case JointType2D.Wheel: lx = p[JPay.P0]; ly = axial; torque = p[JPay.Motor]; break;
                default: lx = p[JPay.P0] + axial; ly = 0f; torque = 0f; break; // distance
            }

            float force = float.Sqrt(lx * lx + ly * ly) * inv;
            torque = (torque < 0f ? -torque : torque) * inv;
            JointForce[c] = force; JointTorque[c] = torque;
            if (force > j.BreakForce) { cs.Flags[c] |= (byte)ConstraintBits.BreakRequested; cs.BreakValue[c] = force; }
            else if (torque > j.BreakTorque) { cs.Flags[c] |= (byte)ConstraintBits.BreakRequested; cs.BreakValue[c] = torque; }
        }
    }
}
