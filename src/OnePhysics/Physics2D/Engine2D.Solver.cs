using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Broadphase;
using RecursiveAI.OnePhysics.Collision2D;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    // ------------------------------------------------------------------ soft constraint coefficients

    /// <summary>
    /// Soft constraint coefficients (Catto, "Soft Step Solver", GDC 2024; "Solver2D"): a spring of frequency <paramref name="hertz"/> and damping ratio <paramref name="zeta"/>
    /// integrated implicitly over <paramref name="h"/>, expressed as a bias rate and scales for the constraint mass and the accumulated impulse.
    /// </summary>
    private static void MakeSoft(float hertz, float zeta, float h, out float biasRate, out float massScale, out float impulseScale)
    {
        if (hertz == 0f) { biasRate = 0f; massScale = 1f; impulseScale = 0f; return; }
        float omega = 6.2831855f * hertz;
        float a1 = 2f * zeta + h * omega;
        float a2 = h * omega * a1;
        float a3 = 1f / (1f + a2);
        biasRate = omega / a1;
        massScale = a2 * a3;
        impulseScale = a3;
    }

    // ------------------------------------------------------------------ prepare

    /// <summary>Once per step before anything is prepared: sizes the per-step solver storage and copies every live body's velocity into <see cref="BodyState"/> with zero accumulated motion.</summary>
    public void BeginSolve(int batches)
    {
        var core = Core;
        var bodies = core.Bodies;
        int nb = bodies.HighWater;
        if (BodyState.Length < (nb + 1) * BS2D.Stride) { BodyState = new float[Math.Max(nb + 1, BodyState.Length / BS2D.Stride * 2) * BS2D.Stride]; }
        if (Wide.Length < batches * W2D.GroupFloats)
        {
            int cap = Math.Max(batches, Wide.Length / W2D.GroupFloats * 2);
            Wide = new float[cap * W2D.GroupFloats]; WideIdx = new int[cap * W2D.IdxPerGroup]; WideCons = new int[cap * W2D.Lanes];
        }

        if (JWide.Length < batches * J2D.GroupFloats && LiveJointCount() > 0)
        {
            int cap = Math.Max(batches, JWide.Length / J2D.GroupFloats * 2);
            JWide = new float[cap * J2D.GroupFloats]; JIdx = new int[cap * W2D.IdxPerGroup];
        }

        int nc = core.Constraints.HighWater;
        if (SlotOf.Length < nc) { SlotOf = new int[Math.Max(nc, SlotOf.Length * 2)]; JSlotOf = new int[SlotOf.Length]; BatchSimple = new byte[SlotOf.Length]; }
        ReadOnlySpan<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        var st = BodyState.AsSpan();
        _movingKinematic = false;
        for (int b = 0; b < nb; b++)
        {
            if (!bodies.IsLive(b)) { continue; }
            if (bodies.TypeOf(b) == BodyType.Kinematic && IsActive(bodies, b)) { _movingKinematic = true; }
            int o = (b + 1) * BS2D.Stride;
            st[o + BS2D.Vx] = lin[2 * b]; st[o + BS2D.Vy] = lin[2 * b + 1]; st[o + BS2D.W] = ang[b];
            st[o + BS2D.Dpx] = 0f; st[o + BS2D.Dpy] = 0f; st[o + BS2D.DqC] = 1f; st[o + BS2D.DqS] = 0f;
        }
    }

    /// <summary>Prepares the joints among a chunk of constraints (contacts are prepared per batch by <see cref="PrepareBatch"/>).</summary>
    public void PrepareJoints(ReadOnlySpan<int> constraints, in StepContext ctx)
    {
        var cs = Core.Constraints;
        foreach (int c in constraints)
        {
            // A plain revolute joint that the wide kernel will solve is prepared straight into its group by PrepareBatch.
            if (cs.KindOf(c) == ConstraintKind.Joint && !IsPlainRevolute(c, cs)) { PrepareJoint(c, in ctx); }
        }
    }

    /// <summary>Once per step: effective masses, adjusted separations, relative velocities for restitution and soft coefficients of the contacts of one batch, from the manifolds stored in the payloads, into one wide group.</summary>
    public void PrepareBatch(ReadOnlySpan<int> constraints, int group, in StepContext ctx)
    {
        var core = Core;
        var cs = core.Constraints;
        var bodies = core.Bodies;
        ReadOnlySpan<int> bodyA = cs.BodyA, bodyB = cs.BodyB;
        ReadOnlySpan<float> invMass = bodies.InverseMass, invI = bodies.InverseInertia;
        ReadOnlySpan<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        ReadOnlySpan<byte> type = bodies.Type;
        float hertz = Opt.ContactHertz;
        float cap = 0.25f * ctx.InverseSubstepDt;
        if (hertz > cap) { hertz = cap; }
        MakeSoft(hertz, Opt.ContactDampingRatio, ctx.SubstepDt, out float dynBias, out float dynMass, out float dynImpulse);
        MakeSoft(2f * hertz, Opt.ContactDampingRatio, ctx.SubstepDt, out float stBias, out float stMass, out float stImpulse);

        var w = Wide.AsSpan(group * W2D.GroupFloats, W2D.GroupFloats);
        var idx = WideIdx.AsSpan(group * W2D.IdxPerGroup, W2D.IdxPerGroup);
        var ids = WideCons.AsSpan(group * W2D.Lanes, W2D.Lanes);
        for (int l = 0; l < W2D.Lanes; l++) { idx[l] = 0; idx[8 + l] = 0; idx[16 + l] = -1; idx[24 + l] = -1; ids[l] = -1; }
        int lane = 0;
        int jlane = 0;
        var jw = JWide.Length >= (group + 1) * J2D.GroupFloats ? JWide.AsSpan(group * J2D.GroupFloats, J2D.GroupFloats) : default;
        var jidx = JIdx.Length >= (group + 1) * W2D.IdxPerGroup ? JIdx.AsSpan(group * W2D.IdxPerGroup, W2D.IdxPerGroup) : default;
        if (jidx.Length != 0)
        {
            for (int l = 0; l < W2D.Lanes; l++) { jidx[l] = 0; jidx[8 + l] = 0; jidx[16 + l] = -1; jidx[24 + l] = -1; }
        }

        float jointRigidBias = 0f, jointRigidMass = 1f, jointRigidImpulse = 0f;
        if (jidx.Length != 0) { MakeSoft(Opt.JointHertz > cap ? cap : Opt.JointHertz, Opt.JointDampingRatio, ctx.SubstepDt, out jointRigidBias, out jointRigidMass, out jointRigidImpulse); }
        bool anyJoint = false;
        foreach (int c in constraints)
        {
            if (cs.KindOf(c) == ConstraintKind.Joint)
            {
                anyJoint = true;
                if (IsPlainRevolute(c, cs))
                {
                    if (jidx.Length != 0) { PrepareWideJoint(c, cs, jw, jidx, jlane++, group, in ctx, jointRigidBias, jointRigidMass, jointRigidImpulse); continue; }
                    PrepareJoint(c, in ctx); // no wide group to put it in (cannot happen while joints exist): solve it scalar
                }

                JSlotOf[c] = -1;
                continue;
            }

            var pl = cs.Payload(c);
            int a = bodyA[c], b = bodyB[c];
            ids[lane] = c;
            SlotOf[c] = group * W2D.Lanes + lane;
            idx[lane] = (a + 1) * BS2D.Stride; idx[8 + lane] = (b + 1) * BS2D.Stride;
            idx[16 + lane] = type[a] == (byte)BodyType.Dynamic ? (a + 1) * BS2D.Stride : -1;
            idx[24 + lane] = type[b] == (byte)BodyType.Dynamic ? (b + 1) * BS2D.Stride : -1;
            float mA = invMass[a], iA = invI[a], mB = invMass[b], iB = invI[b];
            float nx = pl[ManifoldLayout.Normal], ny = pl[ManifoldLayout.Normal + 1];
            float tx = ny, ty = -nx;
            w[W2D.NormalX * 8 + lane] = nx; w[W2D.NormalY * 8 + lane] = ny;
            w[W2D.Friction * 8 + lane] = pl[ManifoldLayout.Friction];
            w[W2D.Restitution * 8 + lane] = pl[ManifoldLayout.Restitution];
            w[W2D.Rolling * 8 + lane] = pl[ManifoldLayout.RollingResistance];
            float k = iA + iB;
            w[W2D.RollingMass * 8 + lane] = k > 0f ? 1f / k : 0f;
            w[W2D.InvMassA * 8 + lane] = mA; w[W2D.InvInertiaA * 8 + lane] = iA; w[W2D.InvMassB * 8 + lane] = mB; w[W2D.InvInertiaB * 8 + lane] = iB;
            if (mA == 0f || mB == 0f) { w[W2D.BiasRate * 8 + lane] = stBias; w[W2D.MassScale * 8 + lane] = stMass; w[W2D.ImpulseScale * 8 + lane] = stImpulse; }
            else { w[W2D.BiasRate * 8 + lane] = dynBias; w[W2D.MassScale * 8 + lane] = dynMass; w[W2D.ImpulseScale * 8 + lane] = dynImpulse; }
            w[W2D.Roll * 8 + lane] = pl[ManifoldLayout.RollingImpulse];
            w[(W2D.Impulse0 + 2) * 8 + lane] = 0f; w[(W2D.Impulse1 + 2) * 8 + lane] = 0f; // the largest normal impulse of the step starts at zero

            float vAx = lin[2 * a], vAy = lin[2 * a + 1], wA = ang[a], vBx = lin[2 * b], vBy = lin[2 * b + 1], wB = ang[b];
            int count = cs.PointCount[c];
            for (int i = 0; i < count; i++)
            {
                float rAx = pl[ManifoldLayout.AnchorA(i)], rAy = pl[ManifoldLayout.AnchorA(i) + 1];
                float rBx = pl[ManifoldLayout.AnchorB(i)], rBy = pl[ManifoldLayout.AnchorB(i) + 1];
                float sep = pl[ManifoldLayout.Separation(i)];
                int o = W2D.Point0 + i * W2D.PointStride;
                w[(o + W2D.AnchorAX) * 8 + lane] = rAx; w[(o + W2D.AnchorAY) * 8 + lane] = rAy; w[(o + W2D.AnchorBX) * 8 + lane] = rBx; w[(o + W2D.AnchorBY) * 8 + lane] = rBy;
                w[(o + W2D.AdjustedSeparation) * 8 + lane] = sep - ((rBx - rAx) * nx + (rBy - rAy) * ny);
                float rnA = rAx * ny - rAy * nx, rnB = rBx * ny - rBy * nx;
                float kn = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                w[(o + W2D.NormalMass) * 8 + lane] = kn > 0f ? 1f / kn : 0f;
                float rtA = rAx * ty - rAy * tx, rtB = rBx * ty - rBy * tx;
                float kt = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                w[(o + W2D.TangentMass) * 8 + lane] = kt > 0f ? 1f / kt : 0f;
                float dvx = (vBx - wB * rBy) - (vAx - wA * rAy);
                float dvy = (vBy + wB * rBx) - (vAy + wA * rAx);
                w[(o + W2D.RelativeVelocity) * 8 + lane] = dvx * nx + dvy * ny;
                int ib = i == 0 ? W2D.Impulse0 : W2D.Impulse1;
                w[ib * 8 + lane] = pl[ManifoldLayout.NormalImpulse(i)];
                w[(ib + 1) * 8 + lane] = pl[ManifoldLayout.Tangent1Impulse(i)];
            }

            // A manifold with one point leaves the second point's fields zero.
            for (int i = count; i < 2; i++)
            {
                int o = W2D.Point0 + i * W2D.PointStride;
                for (int f = 0; f < W2D.PointStride; f++) { w[(o + f) * 8 + lane] = 0f; }
                int ib = i == 0 ? W2D.Impulse0 : W2D.Impulse1;
                w[ib * 8 + lane] = 0f; w[(ib + 1) * 8 + lane] = 0f; w[(ib + 2) * 8 + lane] = 0f;
            }

            lane++;
        }

        if (anyJoint && jlane > 0 && jlane < J2D.Lanes)
        {
            for (int l = jlane; l < J2D.Lanes; l++) { for (int f = 0; f < J2D.Fields; f++) { jw[f * 8 + l] = 0f; } }
        }

        // Remembered on the batch's first constraint so the solve passes can pick the wide path without looking at every constraint: contacts only, not the overflow set.
        if (constraints.Length > 0) { BatchSimple[constraints[0]] = (byte)(!anyJoint && cs.Color[constraints[0]] != ConstraintStore.Overflow ? 1 : 0); }

        // Unused lanes of a short group read as zeros.
        for (int l = lane; l < W2D.Lanes; l++)
        {
            for (int f = 0; f < W2D.Fields; f++) { w[f * 8 + l] = 0f; }
        }
    }

    /// <summary>A revolute joint with no spring, motor or limit, whose stored spring, motor and limit impulses are all zero: the wide kernel handles exactly this.</summary>
    internal static bool WideRevoluteJoints { get; set; } = true;

    private bool IsPlainRevolute(int c, ConstraintStore cs)
    {
        if (!WideRevoluteJoints) { return false; }
        ref JointData2D d = ref Joints[c];
        if (d.Type != (byte)JointType2D.Revolute || (cs.Color[c] == ConstraintStore.Overflow) || d.Motor || d.Limit || (d.Spring && d.Hertz > 0f)) { return false; }
        var p = cs.Payload(c);
        return p[JPay.Spring] == 0f && p[JPay.Motor] == 0f && p[JPay.Lower] == 0f && p[JPay.Upper] == 0f;
    }

    // The scalar PrepareJoint for a plain revolute joint, written straight into the wide group: same operations in the same order, without the scratch row.
    private void PrepareWideJoint(int c, ConstraintStore cs, Span<float> w, Span<int> idx, int lane, int group, in StepContext ctx, float jBias, float jMass, float jImp)
    {
        var bodies = Core.Bodies;
        ref JointData2D j = ref Joints[c];
        var p = cs.Payload(c);
        int a = cs.BodyA[c], b = cs.BodyB[c];
        JSlotOf[c] = group * J2D.Lanes + lane;
        idx[lane] = (a + 1) * BS2D.Stride; idx[8 + lane] = (b + 1) * BS2D.Stride;
        idx[16 + lane] = bodies.TypeOf(a) == BodyType.Dynamic ? (a + 1) * BS2D.Stride : -1;
        idx[24 + lane] = bodies.TypeOf(b) == BodyType.Dynamic ? (b + 1) * BS2D.Stride : -1;
        var pA = bodies.PositionOf(a); var pB = bodies.PositionOf(b);
        var qA = bodies.RotationOf(a); var qB = bodies.RotationOf(b);
        float qAc = qA[0], qAs = qA[1], qBc = qB[0], qBs = qB[1];
        float xa = j.LAx - LocalCenter[2 * a], ya = j.LAy - LocalCenter[2 * a + 1];
        float xb = j.LBx - LocalCenter[2 * b], yb = j.LBy - LocalCenter[2 * b + 1];
        w[J2D.MA * 8 + lane] = bodies.InverseMass[a]; w[J2D.IA * 8 + lane] = bodies.InverseInertia[a]; w[J2D.MB * 8 + lane] = bodies.InverseMass[b]; w[J2D.IB * 8 + lane] = bodies.InverseInertia[b];
        w[J2D.RAx * 8 + lane] = qAc * xa - qAs * ya; w[J2D.RAy * 8 + lane] = qAs * xa + qAc * ya;
        w[J2D.RBx * 8 + lane] = qBc * xb - qBs * yb; w[J2D.RBy * 8 + lane] = qBs * xb + qBc * yb;
        w[J2D.Dx * 8 + lane] = (float)(pB[0] - pA[0]); w[J2D.Dy * 8 + lane] = (float)(pB[1] - pA[1]);
        w[J2D.Rigid * 8 + lane] = jMass * jBias; w[J2D.Mass * 8 + lane] = jMass; w[J2D.Impulse * 8 + lane] = jImp;
        w[J2D.P0 * 8 + lane] = p[JPay.P0]; w[J2D.P1 * 8 + lane] = p[JPay.P1];
    }

    /// <summary>After the last substep: copies the solved impulses of the contacts among a chunk of constraints back into their payloads for the next step's warm start.</summary>
    public void StoreContactResults(ReadOnlySpan<int> constraints)
    {
        var cs = Core.Constraints;
        ReadOnlySpan<float> wide = Wide;
        Span<float> payload = cs.PayloadAll;
        int pf = cs.PayloadFloats;
        foreach (int c in constraints)
        {
            if (cs.KindOf(c) == ConstraintKind.Joint)
            {
                int js = JSlotOf[c];
                if (js >= 0)
                {
                    int jb = (js >> 3) * J2D.GroupFloats + (js & 7);
                    payload[c * pf + JPay.P0] = JWide[jb + J2D.P0 * 8]; payload[c * pf + JPay.P1] = JWide[jb + J2D.P1 * 8];
                }

                continue;
            }

            int slot = SlotOf[c];
            int wb = (slot >> 3) * W2D.GroupFloats + (slot & 7);
            int pb = c * pf;
            payload[pb + ManifoldLayout.RollingImpulse] = wide[wb + W2D.Roll * 8];
            payload[pb + ManifoldLayout.NormalImpulse(0)] = wide[wb + W2D.Impulse0 * 8];
            payload[pb + ManifoldLayout.Tangent1Impulse(0)] = wide[wb + (W2D.Impulse0 + 1) * 8];
            payload[pb + ManifoldLayout.MaxNormalImpulse(0)] = wide[wb + (W2D.Impulse0 + 2) * 8];
            payload[pb + ManifoldLayout.NormalImpulse(1)] = wide[wb + W2D.Impulse1 * 8];
            payload[pb + ManifoldLayout.Tangent1Impulse(1)] = wide[wb + (W2D.Impulse1 + 1) * 8];
            payload[pb + ManifoldLayout.MaxNormalImpulse(1)] = wide[wb + (W2D.Impulse1 + 2) * 8];
        }
    }

    // ------------------------------------------------------------------ solve

    public void SolveBatch(ReadOnlySpan<int> constraints, SolvePhase phase, in StepContext ctx)
    {
        var kp = new KernelParams(ctx.InverseSubstepDt, Opt.PushOutVelocity, Opt.ContactSlop, Opt.RestitutionThreshold);
        if (constraints.Length == 0) { return; }
        if (BatchSimple[constraints[0]] == 1) { RunContacts(SlotOf[constraints[0]] >> 3, constraints.Length, phase, in kp); return; }
        var cons = Core.Constraints;
        bool overflow = cons.Color[constraints[0]] == ConstraintStore.Overflow;
        int joints = 0;
        for (int i = 0; i < constraints.Length; i++) { if (cons.KindOf(constraints[i]) == ConstraintKind.Joint) { joints++; } }
        if (joints > 0)
        {
            SolveMixed(constraints, joints, overflow, phase, ctx, in kp);
            return;
        }

        if (overflow)
        {
            // The overflow set may share bodies inside a group, so it is solved one constraint at a time, in list order.
            for (int i = 0; i < constraints.Length; i++) { RunLane(constraints[i], phase, in kp); }
        }
        else { RunContacts(SlotOf[constraints[0]] >> 3, constraints.Length, phase, in kp); }
    }

    /// <summary>Runs one contact alone (its lane of its group) through the one-lane kernel.</summary>
    private void RunLane(int c, SolvePhase phase, in KernelParams kp)
    {
        int slot = SlotOf[c];
        ContactKernel2D.Run<Lanes1>(this, slot >> 3, slot & 7, 1, phase, in kp);
    }

    /// <summary>Runs the first <paramref name="n"/> lanes of a group at the configured kernel width.</summary>
    private void RunContacts(int group, int n, SolvePhase phase, in KernelParams kp)
    {
        if (KernelWidth == 1) { ContactKernel2D.Run<Lanes1>(this, group, 0, n, phase, in kp); }
        else if (KernelWidth == 4) { ContactKernel2D.Run<Lanes128>(this, group, 0, n, phase, in kp); }
        else { ContactKernel2D.Run<Lanes256>(this, group, 0, n, phase, in kp); }
    }

    // ------------------------------------------------------------------ integration

    private static void IntegrateRotation(ref float c, ref float s, float angle)
    {
        // Catto: advance the rotation by a small angle linearly, then renormalize.
        float nc = c - angle * s;
        float ns = s + angle * c;
        float mag = float.Sqrt(nc * nc + ns * ns);
        float inv = mag > 0f ? 1f / mag : 0f;
        c = nc * inv;
        s = ns * inv;
    }

    public void IntegrateVelocities(ReadOnlySpan<int> list, in StepContext ctx)
    {
        var bodies = Core.Bodies;
        Span<float> st = BodyState;
        ReadOnlySpan<float> invMass = bodies.InverseMass, invI = bodies.InverseInertia;
        ReadOnlySpan<byte> type = bodies.Type;
        float h = ctx.SubstepDt;
        var g = Core.Options.Gravity;
        float maxSpeed = Opt.MaxLinearSpeed, maxSpeed2 = maxSpeed * maxSpeed;
        float maxSpin = Opt.MaxRotationPerSubstep / h;
        foreach (int b in list)
        {
            if (type[b] != (byte)BodyType.Dynamic) { continue; }
            int o = (b + 1) * BS2D.Stride;
            float gs = GravityScale[b];
            float vx = st[o + BS2D.Vx] + h * (gs * g.X + invMass[b] * Force[2 * b]);
            float vy = st[o + BS2D.Vy] + h * (gs * g.Y + invMass[b] * Force[2 * b + 1]);
            float w = st[o + BS2D.W] + h * (invI[b] * Torque[b]);
            float dl = 1f / (1f + h * LinearDamping[b]);
            float da = 1f / (1f + h * AngularDamping[b]);
            vx *= dl; vy *= dl; w *= da;
            float v2 = vx * vx + vy * vy;
            if (v2 > maxSpeed2)
            {
                float scale = maxSpeed / float.Sqrt(v2);
                vx *= scale; vy *= scale;
            }

            if (w > maxSpin) { w = maxSpin; }
            else if (w < -maxSpin) { w = -maxSpin; }
            st[o + BS2D.Vx] = vx; st[o + BS2D.Vy] = vy; st[o + BS2D.W] = w;
        }
    }

    public void IntegratePositions(ReadOnlySpan<int> list, in StepContext ctx)
    {
        var bodies = Core.Bodies;
        Span<float> st = BodyState;
        Span<double> pos = bodies.Position;
        Span<float> rot = bodies.Rotation;
        float h = ctx.SubstepDt;
        foreach (int b in list)
        {
            int o = (b + 1) * BS2D.Stride;
            float dx = h * st[o + BS2D.Vx], dy = h * st[o + BS2D.Vy];
            st[o + BS2D.Dpx] += dx; st[o + BS2D.Dpy] += dy;
            pos[2 * b] += (double)dx; pos[2 * b + 1] += (double)dy;
            float w = st[o + BS2D.W];
            if (w != 0f)
            {
                float a = h * w;
                IntegrateRotation(ref st[o + BS2D.DqC], ref st[o + BS2D.DqS], a);
                IntegrateRotation(ref rot[2 * b], ref rot[2 * b + 1], a);
            }
        }
    }

    // ------------------------------------------------------------------ finalize: forces and continuous collision

    public void FinalizeBodies(ReadOnlySpan<int> list, in StepContext ctx)
    {
        var bodies = Core.Bodies;
        Span<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        foreach (int b in list)
        {
            int o = (b + 1) * BS2D.Stride;
            Dp[2 * b] = BodyState[o + BS2D.Dpx]; Dp[2 * b + 1] = BodyState[o + BS2D.Dpy];
            if (bodies.TypeOf(b) != BodyType.Dynamic) { continue; }
            lin[2 * b] = BodyState[o + BS2D.Vx]; lin[2 * b + 1] = BodyState[o + BS2D.Vy]; ang[b] = BodyState[o + BS2D.W];
            Force[2 * b] = 0f; Force[2 * b + 1] = 0f; Torque[b] = 0f;
            // Bullets are swept after every body has finished (RunBullets), against static and moving shapes alike.
            if (Opt.ContinuousEnabled && !IsBullet(b)) { Continuous(b); }
        }
    }

    private struct ToiQuery : IOverlapCallback
    {
        public Engine2D E;
        public int Shape;      // the fast shape
        public Sweep2D Sweep;  // its body's sweep, relative to the start position
        public double Cx, Cy;  // start position of the fast body's centre of mass
        public Vec2 End;       // end position of the fast body's centre of mass, relative to its start
        public float MinExtent;
        public float Best;     // smallest fraction so far
        public bool Hit;

        public bool Report(int key, int userId)
        {
            var e = E;
            var core = e.Core;
            int other = userId;
            if (e.IsSensor(Shape) || e.IsSensor(other) || !e.CanCollide(Shape, other)) { return true; }
            int sb = core.Shapes.Owner[other];
            var bodies = core.Bodies;
            var p = bodies.PositionOf(sb);
            var r = bodies.RotationOf(sb);
            var q = new Rot2(r[0], r[1]);
            var c = new Vec2((float)(p[0] - Cx), (float)(p[1] - Cy));
            ref readonly var g = ref e.Geom[other];
            if (g.Kind == ShapeKind2D.Segment || g.Kind == ShapeKind2D.ChainSegment)
            {
                // Catto, continuous collision: against a segment only count a body whose centre really moves towards it. This keeps a body sliding
                // along a rough surface (it stays within a few millimetres of it) from being pulled back to its start pose every step.
                var v = g.Vertices;
                Vec2 edge = v[1] - v[0];
                float len = edge.Length;
                if (!(len > 0f)) { return true; }
                Vec2 l0 = q.InverseRotate(new Vec2(-c.X, -c.Y)) - v[0];
                Vec2 l1 = q.InverseRotate(new Vec2(End.X - c.X, End.Y - c.Y)) - v[0];
                float d0 = (edge.X * l0.Y - edge.Y * l0.X) / len; // positive on the left of v0 -> v1
                float d1 = (edge.X * l1.Y - edge.Y * l1.X) / len;
                if (g.Kind == ShapeKind2D.ChainSegment && !g.IsTwoSided)
                {
                    // Solid on the right only: a mover that started behind passes.
                    if (d0 > 0f) { return true; }
                    d0 = -d0; d1 = -d1;
                }
                else if (d0 < 0f) { d0 = -d0; d1 = -d1; }

                if (d0 - d1 < 0.25f * MinExtent) { return true; }
            }

            var sweepStatic = new Sweep2D(default, c, c, q, q);
            var toi = Distance2D.TimeOfImpact(in g, sweepStatic, in e.Geom[Shape], Sweep, Best, Constants2D.LinearSlop);
            if ((toi.State == ToiState.Hit || toi.State == ToiState.Failed) && toi.Fraction > 0f && toi.Fraction < Best)
            {
                Best = toi.Fraction;
                Hit = true;
            }

            return true;
        }
    }

    /// <summary>
    /// Continuous collision against static shapes (Catto, GDC 2013, conservative advancement): a body that moved more than half its smallest dimension this step is swept
    /// from its start pose against every static shape its path touches; if it hit one it is pulled back to the first time of impact, so the next step sees a speculative contact
    /// instead of a body on the far side of the wall.
    /// </summary>
    private void Continuous(int b)
    {
        var bodies = Core.Bodies;
        var pos = bodies.PositionOf(b);
        var rot = bodies.RotationOf(b);
        double c1x = StartPos[2 * b], c1y = StartPos[2 * b + 1];
        float mx = (float)(pos[0] - c1x), my = (float)(pos[1] - c1y);
        float q1c = StartRot[2 * b], q1s = StartRot[2 * b + 1], q2c = rot[0], q2s = rot[1];
        float spin = Math.Abs(q1c * q2s - q1s * q2c) * MaxExtent[b];
        float moved = float.Sqrt(mx * mx + my * my) + spin;
        if (!(moved > 0.5f * MinExtent[b])) { return; }

        var lc = new Vec2(LocalCenter[2 * b], LocalCenter[2 * b + 1]);
        var sweep = new Sweep2D(lc, default, new Vec2(mx, my), new Rot2(q1c, q1s), new Rot2(q2c, q2s));
        float best = 1f;
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            // The box of the whole sweep: the shape's bounds at the start and at the end.
            var xf1 = new Transform2(-new Rot2(q1c, q1s).Rotate(lc), new Rot2(q1c, q1s));
            var xf2 = new Transform2(new Vec2(mx, my) - new Rot2(q2c, q2s).Rotate(lc), new Rot2(q2c, q2s));
            var a1 = Geom[s].ComputeAabb(xf1);
            var a2 = Geom[s].ComputeAabb(xf2);
            var box = new Aabb2D(
                c1x + (double)Math.Min(a1.Lower.X, a2.Lower.X), c1y + (double)Math.Min(a1.Lower.Y, a2.Lower.Y),
                c1x + (double)Math.Max(a1.Upper.X, a2.Upper.X), c1y + (double)Math.Max(a1.Upper.Y, a2.Upper.Y));
            var q = new ToiQuery { E = this, Shape = s, Sweep = sweep, Cx = c1x, Cy = c1y, End = new Vec2(mx, my), MinExtent = MinExtent[b], Best = best };
            Bp.QueryAabb(in box, ref q, BodyTypeMask.Static);
            if (q.Hit && q.Best < best) { best = q.Best; }
        }

        if (best >= 1f) { return; }
        float t = best;
        pos[0] = c1x + (double)(t * mx);
        pos[1] = c1y + (double)(t * my);
        float nc = (1f - t) * q1c + t * q2c, ns = (1f - t) * q1s + t * q2s;
        float mag = float.Sqrt(nc * nc + ns * ns);
        if (mag > 0f) { rot[0] = nc / mag; rot[1] = ns / mag; }
        Dp[2 * b] = (float)(pos[0] - c1x); Dp[2 * b + 1] = (float)(pos[1] - c1y);
    }

    // ------------------------------------------------------------------ sleeping

    private bool _movingKinematic; // set once per step in BeginSolve: whether any kinematic body moves, so BodyMotion can skip looking at contact partners when none does

    public float BodyMotion(int b)
    {
        var bodies = Core.Bodies;
        // A body touching a moving kinematic body is not at rest.
        var cs = Core.Constraints;
        for (int e = _movingKinematic ? bodies.FirstEdge(b) : -1; e >= 0; e = cs.NextEdge(e))
        {
            int c = e >> 1;
            int other = (e & 1) == 0 ? cs.BodyB[c] : cs.BodyA[c];
            if (bodies.TypeOf(other) == BodyType.Kinematic && cs.PointCount[c] > 0 && IsActive(bodies, other)) { return float.MaxValue; }
        }

        var v = bodies.LinearVelocityOf(b);
        return float.Sqrt(v[0] * v[0] + v[1] * v[1]) + Math.Abs(bodies.AngularVelocityOf(b)[0]) * MaxExtent[b];
    }
}
