using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Broadphase;
using RecursiveAI.OnePhysics.Collision3D;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    // ------------------------------------------------------------------ small math

    /// <summary>Applies a symmetric tensor stored as xx xy xz yy yz zz to a vector.</summary>
    private static Vector3F Apply(ReadOnlySpan<float> m, Vector3F v) => new(
        m[0] * v.X + m[1] * v.Y + m[2] * v.Z,
        m[1] * v.X + m[3] * v.Y + m[4] * v.Z,
        m[2] * v.X + m[4] * v.Y + m[5] * v.Z);

    private static Vector3F V(ReadOnlySpan<float> a, int i) => new(a[i], a[i + 1], a[i + 2]);

    private static void Put(Span<float> a, int i, Vector3F v) { a[i] = v.X; a[i + 1] = v.Y; a[i + 2] = v.Z; }

    /// <summary>Copies the displacement, accumulated rotation and world inverse inertia of a body into its packed row (see <see cref="Pk"/>).</summary>
    public void PackBody(int b)
    {
        var r = Pk.AsSpan(16 * b, 16);
        r[0] = Dp[3 * b]; r[1] = Dp[3 * b + 1]; r[2] = Dp[3 * b + 2];
        r[3] = Dq[4 * b]; r[4] = Dq[4 * b + 1]; r[5] = Dq[4 * b + 2]; r[6] = Dq[4 * b + 3];
        var w = WInvI.AsSpan(6 * b, 6);
        r[7] = w[0]; r[8] = w[1]; r[9] = w[2]; r[10] = w[3]; r[11] = w[4]; r[12] = w[5];
    }

    /// <summary>Refreshes one body's world-axes inverse inertia from the current rotation (for impulses applied between steps).</summary>
    public void RefreshWorldInertia(int b)
    {
        EnsureBodies(b + 1);
        var bodies = Core.Bodies;
        RefreshWorldInertia(b, bodies.InverseMass, bodies.InverseInertia, bodies.Rotation);
    }

    /// <summary>Multiplies a vector by the body's world-axes inverse inertia (as last refreshed).</summary>
    public Vector3F ApplyWorldInertia(int b, Vector3F v) => Apply(WInvI.AsSpan(6 * b, 6), v);

    /// <summary>Inverse inertia in world axes, R · I⁻¹ · Rᵀ, from the body-frame tensor and the body's rotation. Static and kinematic bodies get zeros.</summary>
    private void RefreshWorldInertia(int b, ReadOnlySpan<float> invMass, ReadOnlySpan<float> invI, ReadOnlySpan<float> rot)
    {
        var w = WInvI.AsSpan(6 * b, 6);
        if (!(invMass[b] > 0f)) { w.Clear(); return; }
        float x = rot[4 * b], y = rot[4 * b + 1], z = rot[4 * b + 2], s = rot[4 * b + 3];
        float xx = x * x, yy = y * y, zz = z * z, xy = x * y, xz = x * z, yz = y * z, sx = s * x, sy = s * y, sz = s * z;
        float r00 = 1f - 2f * (yy + zz), r01 = 2f * (xy - sz), r02 = 2f * (xz + sy);
        float r10 = 2f * (xy + sz), r11 = 1f - 2f * (xx + zz), r12 = 2f * (yz - sx);
        float r20 = 2f * (xz - sy), r21 = 2f * (yz + sx), r22 = 1f - 2f * (xx + yy);
        float ia = invI[6 * b], ib = invI[6 * b + 1], ic = invI[6 * b + 2], id = invI[6 * b + 3], ie = invI[6 * b + 4], ifz = invI[6 * b + 5];
        float t00 = r00 * ia + r01 * ib + r02 * ic, t01 = r00 * ib + r01 * id + r02 * ie, t02 = r00 * ic + r01 * ie + r02 * ifz;
        float t10 = r10 * ia + r11 * ib + r12 * ic, t11 = r10 * ib + r11 * id + r12 * ie, t12 = r10 * ic + r11 * ie + r12 * ifz;
        float t20 = r20 * ia + r21 * ib + r22 * ic, t21 = r20 * ib + r21 * id + r22 * ie, t22 = r20 * ic + r21 * ie + r22 * ifz;
        w[0] = t00 * r00 + t01 * r01 + t02 * r02;
        w[1] = t00 * r10 + t01 * r11 + t02 * r12;
        w[2] = t00 * r20 + t01 * r21 + t02 * r22;
        w[3] = t10 * r10 + t11 * r11 + t12 * r12;
        w[4] = t10 * r20 + t11 * r21 + t12 * r22;
        w[5] = t20 * r20 + t21 * r21 + t22 * r22;
    }

    /// <summary>Two unit vectors perpendicular to a unit normal and to each other, chosen by a fixed rule so the same normal always gives the same pair.</summary>
    private static void Tangents(Vector3F n, out Vector3F t1, out Vector3F t2)
    {
        // Pick the axis the normal is least aligned with; cross products and square roots only.
        Vector3F t;
        if (Math.Abs(n.X) >= 0.57735f) { t = new Vector3F(n.Y, -n.X, 0f); }
        else { t = new Vector3F(0f, n.Z, -n.Y); }
        float len = float.Sqrt(t.X * t.X + t.Y * t.Y + t.Z * t.Z);
        t1 = len > 0f ? t * (1f / len) : new Vector3F(1f, 0f, 0f);
        t2 = Vector3F.Cross(n, t1);
    }

    // ------------------------------------------------------------------ soft constraint coefficients

    /// <summary>
    /// Soft constraint coefficients (Catto, "Soft Step Solver", GDC 2024; "Solver2D", 2024): a spring of frequency <paramref name="hertz"/> and damping ratio <paramref name="zeta"/>
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

    private float _dynBias, _dynMass, _dynImpulse, _stBias, _stMass, _stImpulse;
    private JointKernelParams _jointParams;

    /// <summary>Once per step before anything is prepared: sizes the wide groups (one per batch) and the constraint-to-lane map, and computes the soft-constraint coefficients.</summary>
    public void BeginSolve(int batches, in StepContext ctx)
    {
        if (Wide.Length < batches * W3D.GroupFloats)
        {
            int cap = Math.Max(batches, Wide.Length / W3D.GroupFloats * 2);
            Wide = new float[cap * W3D.GroupFloats]; WideIdx = new int[cap * W3D.IdxPerGroup];
        }

        int nc = Core.Constraints.HighWater;
        if (SlotOf.Length < nc) { SlotOf = new int[Math.Max(nc, SlotOf.Length * 2)]; }
        Array.Fill(SlotOf, -1, 0, nc);
        float hertz = Opt.ContactHertz;
        float cap2 = 0.25f * ctx.InverseSubstepDt;
        if (hertz > cap2) { hertz = cap2; }
        MakeSoft(hertz, Opt.ContactDampingRatio, ctx.SubstepDt, out _dynBias, out _dynMass, out _dynImpulse);
        MakeSoft(2f * hertz, Opt.ContactDampingRatio, ctx.SubstepDt, out _stBias, out _stMass, out _stImpulse);

        GroupColorsByPointCount();

        // Point joints solve in their own wide groups, one per batch, only when the world has joints.
        EnsureBound();
        if (_trackedJoints > 0 && WideJ.Length < batches * J3D.GroupFloats)
        {
            int jcap = Math.Max(batches, WideJ.Length / J3D.GroupFloats * 2);
            WideJ = new float[jcap * J3D.GroupFloats]; WideJIdx = new int[jcap * J3D.IdxPerGroup];
        }

        float jh = Opt.JointHertz > cap2 ? cap2 : Opt.JointHertz;
        MakeSoft(jh, Opt.JointDampingRatio, ctx.SubstepDt, out float jbias, out float jmass, out float jimp);
        _jointParams = new JointKernelParams(jmass, jmass * jbias, jimp);
    }

    private int[] _sortTmp = new int[64];

    /// <summary>
    /// Reorders each color's constraints, stably, by contact point count (most points first, joints last), so the contacts of a batch have similar point counts: a group runs as many point passes as its
    /// largest lane needs, and a pile is mostly one-point contacts with a few four-point ones that would otherwise stretch every batch they land in. The constraints of a color share no dynamic body,
    /// so the order they are solved in cannot change a result; the new order depends only on the data.
    /// </summary>
    private void GroupColorsByPointCount()
    {
        var coloring = Core.Coloring;
        var list = coloring.ListArray;
        var cs = Core.Constraints;
        ReadOnlySpan<byte> points = cs.PointCount;
        Span<int> start = stackalloc int[W3D.MaxPoints + 3];
        for (int g = 0; g < ConstraintColoring.ColorCount; g++)
        {
            int n = coloring.Color(g).Length;
            if (n <= ConstraintColoring.Lanes) { continue; }
            int off = coloring.ColorStart(g);
            // Bucket 0 holds four points, 1 three, 2 two, 3 one, 4 none, 5 joints (and any other kind).
            start.Clear();
            for (int i = 0; i < n; i++) { start[1 + Bucket(cs, points, list[off + i])]++; }
            if (start[1 + 5] == n || start[1] == n || start[1 + 3] == n) { continue; }
            for (int k = 1; k < start.Length; k++) { start[k] += start[k - 1]; }
            if (_sortTmp.Length < n) { _sortTmp = new int[Math.Max(n, _sortTmp.Length * 2)]; }
            for (int i = 0; i < n; i++) { int c = list[off + i]; _sortTmp[start[Bucket(cs, points, c)]++] = c; }
            Array.Copy(_sortTmp, 0, list, off, n);
        }
    }

    private static int Bucket(ConstraintStore cs, ReadOnlySpan<byte> points, int c) => cs.KindOf(c) == ConstraintKind.Contact ? W3D.MaxPoints - Math.Min((int)points[c], W3D.MaxPoints) : 5;

    /// <summary>
    /// Once per step: effective masses, adjusted separations, relative velocities for restitution and soft coefficients of the contacts of one batch, from the manifolds stored in the payloads, into one wide group.
    /// The group also holds the impulses while the step runs; <see cref="StoreResults"/> writes them back to the payloads.
    /// </summary>
    public void PrepareBatch(ReadOnlySpan<int> constraints, int group, in StepContext ctx)
    {
        var core = Core;
        var cs = core.Constraints;
        var bodies = core.Bodies;
        ReadOnlySpan<int> bodyA = cs.BodyA, bodyB = cs.BodyB;
        ReadOnlySpan<float> invMass = bodies.InverseMass;
        ReadOnlySpan<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        var w = Wide.AsSpan(group * W3D.GroupFloats, W3D.GroupFloats);
        var idx = WideIdx.AsSpan(group * W3D.IdxPerGroup, W3D.IdxPerGroup);
        for (int l = 0; l < W3D.Lanes; l++) { idx[l] = 0; idx[8 + l] = 0; idx[16 + l] = -1; idx[24 + l] = -1; }
        int maxPoints = 1, anyRolling = 0;
        float allOnes = BitConverter.Int32BitsToSingle(-1);
        int lane = 0, jlane = 0;
        Span<float> rm = stackalloc float[6];
        foreach (int c in constraints)
        {
            if (cs.KindOf(c) == ConstraintKind.Joint)
            {
                byte jt = Joints[c].Type;
                if (WidePointBlock(jt))
                {
                    if (jlane == 0) { ClearJointIdx(group); }
                    PreparePointJoint(c, group, jlane++);
                }

                if (jt != (byte)JointType3D.Point) { PrepareJoint(c, in ctx); }

                continue;
            }

            var pl = cs.Payload(c);
            int a = bodyA[c], b = bodyB[c];
            SlotOf[c] = group * W3D.Lanes + lane;
            float mA = invMass[a], mB = invMass[b];
            idx[lane] = a; idx[8 + lane] = b;
            idx[16 + lane] = mA > 0f ? a : -1; idx[24 + lane] = mB > 0f ? b : -1;
            ReadOnlySpan<float> iA = WInvI.AsSpan(6 * a, 6), iB = WInvI.AsSpan(6 * b, 6);
            var n = V(pl, ManifoldLayout.Normal);
            Tangents(n, out var t1, out var t2);
            Put3(w, W3D.N, lane, n); Put3(w, W3D.T1, lane, t1); Put3(w, W3D.T2, lane, t2);
            w[W3D.Friction * 8 + lane] = pl[ManifoldLayout.Friction];
            w[W3D.Restitution * 8 + lane] = pl[ManifoldLayout.Restitution];
            float rolling = pl[ManifoldLayout.RollingResistance];
            w[W3D.Rolling * 8 + lane] = rolling;
            w[W3D.InvMassA * 8 + lane] = mA; w[W3D.InvMassB * 8 + lane] = mB;
            if (mA == 0f || mB == 0f) { w[W3D.BiasRate * 8 + lane] = _stBias; w[W3D.MassScale * 8 + lane] = _stMass; w[W3D.ImpulseScale * 8 + lane] = _stImpulse; }
            else { w[W3D.BiasRate * 8 + lane] = _dynBias; w[W3D.MassScale * 8 + lane] = _dynMass; w[W3D.ImpulseScale * 8 + lane] = _dynImpulse; }
            if (rolling > 0f) { anyRolling = 1; RollingMass(iA, iB, rm); for (int k = 0; k < 6; k++) { w[(W3D.RollMass + k) * 8 + lane] = rm[k]; } }
            Put3(w, W3D.RollImpulse, lane, V(pl, ManifoldLayout.RollingImpulse));

            // The effective masses, adjusted separations and relative velocities are computed for the whole group at once afterwards (ContactKernel3D.PrepareMasses); the inputs wait in the group's scratch rows.
            int st = W3D.StageOffset;
            for (int k = 0; k < 6; k++) { w[st + k * 8 + lane] = iA[k]; w[st + (6 + k) * 8 + lane] = iB[k]; }
            Put3(w, st / 8 + 12, lane, V(lin, 3 * a)); Put3(w, st / 8 + 15, lane, V(ang, 3 * a)); Put3(w, st / 8 + 18, lane, V(lin, 3 * b)); Put3(w, st / 8 + 21, lane, V(ang, 3 * b));
            int count = cs.PointCount[c];
            if (count > maxPoints) { maxPoints = count; }
            for (int i = 0; i < count; i++)
            {
                var rA = V(pl, ManifoldLayout.AnchorA(i));
                var rB = V(pl, ManifoldLayout.AnchorB(i));
                int f = W3D.Point0 + i * W3D.PointStride;
                Put3(w, f + W3D.ArmA, lane, rA); Put3(w, f + W3D.ArmB, lane, rB);
                w[(f + W3D.AdjustedSeparation) * 8 + lane] = pl[ManifoldLayout.Separation(i)];
                w[(f + W3D.NormalImpulse) * 8 + lane] = pl[ManifoldLayout.NormalImpulse(i)];
                w[(f + W3D.Tangent1Impulse) * 8 + lane] = pl[ManifoldLayout.Tangent1Impulse(i)];
                w[(f + W3D.Tangent2Impulse) * 8 + lane] = pl[ManifoldLayout.Tangent2Impulse(i)];
                w[(f + W3D.MaxNormalImpulse) * 8 + lane] = 0f;
                w[(f + W3D.Mask) * 8 + lane] = allOnes;
            }

            // Slots past the manifold's points take no part: their mask is clear and their impulse reads as zero in the rolling-resistance total. Everything else in the slot is ignored under the mask.
            for (int i = count; i < W3D.MaxPoints; i++)
            {
                int f = W3D.Point0 + i * W3D.PointStride;
                w[(f + W3D.Mask) * 8 + lane] = 0f; w[(f + W3D.NormalImpulse) * 8 + lane] = 0f;
            }

            lane++;
        }

        if (lane > 0)
        {
            if (KernelWidth == 1) { ContactKernel3D.PrepareMasses<Lanes1>(w, maxPoints); }
            else if (KernelWidth == 4) { ContactKernel3D.PrepareMasses<Lanes128>(w, maxPoints); }
            else { ContactKernel3D.PrepareMasses<Lanes256>(w, maxPoints); }
        }

        idx[W3D.MaxPointsSlot] = maxPoints; idx[W3D.AnyRollingSlot] = anyRolling;
    }

    /// <summary>The joint types whose three-row point block (always their last rows) is solved by <see cref="JointKernel3D"/>; their other rows stay in <see cref="SolveJoint"/>.</summary>
    private static bool WidePointBlock(byte type) =>
        type == (byte)JointType3D.Point || type == (byte)JointType3D.Fixed || type == (byte)JointType3D.Hinge || type == (byte)JointType3D.Cone || type == (byte)JointType3D.SwingTwist;

    private static void Put3(Span<float> w, int field, int lane, Vector3F v) { w[field * 8 + lane] = v.X; w[(field + 1) * 8 + lane] = v.Y; w[(field + 2) * 8 + lane] = v.Z; }

    /// <summary>After the last substep: the accumulated impulses go back to the payloads, where the next step's warm start and the events read them.</summary>
    public void StoreResults(ReadOnlySpan<int> constraints)
    {
        var cs = Core.Constraints;
        foreach (int c in constraints)
        {
            int slot = SlotOf[c];
            if (slot < 0) { continue; }
            if (cs.KindOf(c) == ConstraintKind.Joint)
            {
                var jw = WideJ.AsSpan((slot >> 3) * J3D.GroupFloats, J3D.GroupFloats);
                var jp = cs.Payload(c);
                for (int k = 0; k < 3; k++) { jp[k] = jw[(J3D.Impulse + k) * 8 + (slot & 7)]; }
                continue;
            }

            var w = Wide.AsSpan((slot >> 3) * W3D.GroupFloats, W3D.GroupFloats);
            int lane = slot & 7;
            var pl = cs.Payload(c);
            int count = cs.PointCount[c];
            for (int i = 0; i < count; i++)
            {
                int f = W3D.Point0 + i * W3D.PointStride;
                pl[ManifoldLayout.NormalImpulse(i)] = w[(f + W3D.NormalImpulse) * 8 + lane];
                pl[ManifoldLayout.Tangent1Impulse(i)] = w[(f + W3D.Tangent1Impulse) * 8 + lane];
                pl[ManifoldLayout.Tangent2Impulse(i)] = w[(f + W3D.Tangent2Impulse) * 8 + lane];
                pl[ManifoldLayout.MaxNormalImpulse(i)] = w[(f + W3D.MaxNormalImpulse) * 8 + lane];
            }

            for (int k = 0; k < 3; k++) { pl[ManifoldLayout.RollingImpulse + k] = w[(W3D.RollImpulse + k) * 8 + lane]; }
        }
    }

    private static float EffectiveMass(float mA, float mB, ReadOnlySpan<float> iA, ReadOnlySpan<float> iB, Vector3F rA, Vector3F rB, Vector3F d)
    {
        var rnA = Vector3F.Cross(rA, d);
        var rnB = Vector3F.Cross(rB, d);
        float k = mA + mB + Vector3F.Dot(rnA, Apply(iA, rnA)) + Vector3F.Dot(rnB, Apply(iB, rnB));
        return k > 0f ? 1f / k : 0f;
    }

    /// <summary>Inverse of (IA + IB), the mass seen by a relative angular velocity; zero when the sum is singular.</summary>
    private static void RollingMass(ReadOnlySpan<float> iA, ReadOnlySpan<float> iB, Span<float> dest)
    {
        float a = iA[0] + iB[0], b = iA[1] + iB[1], c = iA[2] + iB[2], d = iA[3] + iB[3], e = iA[4] + iB[4], f = iA[5] + iB[5];
        float c00 = d * f - e * e, c01 = c * e - b * f, c02 = b * e - c * d;
        float det = a * c00 + b * c01 + c * c02;
        if (!(det > 0f) || !float.IsFinite(det)) { dest.Clear(); return; }
        float inv = 1f / det;
        dest[0] = c00 * inv; dest[1] = c01 * inv; dest[2] = c02 * inv;
        dest[3] = (a * f - c * c) * inv; dest[4] = (b * c - a * e) * inv; dest[5] = (a * d - b * b) * inv;
    }

    // ------------------------------------------------------------------ solve

    private static Vector3F Rotate(float x, float y, float z, float w, Vector3F v)
    {
        // q v q⁻¹ for a unit quaternion: v + w·t + u × t with t = 2 u × v.
        float tx = 2f * (y * v.Z - z * v.Y), ty = 2f * (z * v.X - x * v.Z), tz = 2f * (x * v.Y - y * v.X);
        return new Vector3F(v.X + w * tx + (y * tz - z * ty), v.Y + w * ty + (z * tx - x * tz), v.Z + w * tz + (x * ty - y * tx));
    }

    /// <summary>
    /// Solves one batch of contacts through the wide kernel. A colored batch has no dynamic body in common between constraints, so its lanes are independent; the overflow set may share bodies inside a batch,
    /// so it runs one lane at a time in list order. Every width gives the same bits.
    /// </summary>
    public void SolveBatch(ReadOnlySpan<int> constraints, SolvePhase phase, in StepContext ctx)
    {
        if (constraints.Length == 0) { return; }
        var kp = new KernelParams(ctx.InverseSubstepDt, Opt.PushOutVelocity, Opt.ContactSlop, Opt.RestitutionThreshold);
        var cons = Core.Constraints;
        if (cons.Color[constraints[0]] == ConstraintStore.Overflow)
        {
            for (int i = 0; i < constraints.Length; i++)
            {
                int c = constraints[i];
                int slot = SlotOf[c];
                if (cons.KindOf(c) == ConstraintKind.Joint)
                {
                    // A joint's own rows first, then its point block, as the joint's rows have always been ordered.
                    if (Joints[c].Type != (byte)JointType3D.Point) { SolveJoint(c, phase, in ctx); }
                    if (slot >= 0) { JointKernel3D.Run<Lanes1>(this, slot >> 3, slot & 7, 1, phase, in _jointParams); }
                }
                else if (slot >= 0) { ContactKernel3D.Run<Lanes1>(this, slot >> 3, slot & 7, 1, phase, in kp); }
            }

            return;
        }

        // Contacts were packed into the first lanes of the batch's group in list order; joints (which share no dynamic body with anything else in the batch) solve one by one.
        int first = -1, count = 0, jfirst = -1, jcount = 0;
        for (int i = 0; i < constraints.Length; i++)
        {
            int c = constraints[i];
            if (cons.KindOf(c) == ConstraintKind.Joint)
            {
                if (Joints[c].Type != (byte)JointType3D.Point) { SolveJoint(c, phase, in ctx); }
                if (SlotOf[c] >= 0)
                {
                    if (jcount == 0) { jfirst = SlotOf[c]; }
                    jcount++;
                }

                continue;
            }

            if (count == 0) { first = SlotOf[c]; }
            count++;
        }

        if (jcount > 0)
        {
            int jgroup = jfirst >> 3;
            if (KernelWidth == 1) { JointKernel3D.Run<Lanes1>(this, jgroup, 0, jcount, phase, in _jointParams); }
            else if (KernelWidth == 4) { JointKernel3D.Run<Lanes128>(this, jgroup, 0, jcount, phase, in _jointParams); }
            else { JointKernel3D.Run<Lanes256>(this, jgroup, 0, jcount, phase, in _jointParams); }
        }

        if (count == 0 || first < 0) { return; }
        int group = first >> 3;
        if (KernelWidth == 1) { ContactKernel3D.Run<Lanes1>(this, group, 0, count, phase, in kp); }
        else if (KernelWidth == 4) { ContactKernel3D.Run<Lanes128>(this, group, 0, count, phase, in kp); }
        else { ContactKernel3D.Run<Lanes256>(this, group, 0, count, phase, in kp); }
    }

    // ------------------------------------------------------------------ integration

    public void IntegrateVelocities(ReadOnlySpan<int> list, in StepContext ctx)
    {
        var bodies = Core.Bodies;
        Span<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        ReadOnlySpan<float> invMass = bodies.InverseMass, rot = bodies.Rotation;
        ReadOnlySpan<byte> type = bodies.Type;
        float h = ctx.SubstepDt;
        var g = Core.Options.Gravity;
        float maxSpeed = Opt.MaxLinearSpeed, maxSpeed2 = maxSpeed * maxSpeed;
        float maxSpin = Opt.MaxRotationPerSubstep / h;
        foreach (int b in list)
        {
            if (type[b] != (byte)BodyType.Dynamic) { continue; }
            float gs = GravityScale[b], im = invMass[b];
            float vx = lin[3 * b] + h * (gs * g.X + im * Force[3 * b]);
            float vy = lin[3 * b + 1] + h * (gs * g.Y + im * Force[3 * b + 1]);
            float vz = lin[3 * b + 2] + h * (gs * g.Z + im * Force[3 * b + 2]);
            var w = V(ang, 3 * b);
            var tq = V(Torque, 3 * b);
            if (tq.X != 0f || tq.Y != 0f || tq.Z != 0f) { w += Apply(WInvI.AsSpan(6 * b, 6), tq) * h; }
            if ((BodyFlags[b] & 2) != 0) { w = Gyroscopic(b, w, h, rot); }
            float dl = 1f / (1f + h * LinearDamping[b]);
            float da = 1f / (1f + h * AngularDamping[b]);
            vx *= dl; vy *= dl; vz *= dl; w *= da;
            float v2 = vx * vx + vy * vy + vz * vz;
            if (v2 > maxSpeed2)
            {
                float scale = maxSpeed / float.Sqrt(v2);
                vx *= scale; vy *= scale; vz *= scale;
            }

            float w2 = w.X * w.X + w.Y * w.Y + w.Z * w.Z;
            if (w2 > maxSpin * maxSpin) { w *= maxSpin / float.Sqrt(w2); }
            lin[3 * b] = vx; lin[3 * b + 1] = vy; lin[3 * b + 2] = vz;
            Put(ang, 3 * b, w);
        }
    }

    /// <summary>
    /// The gyroscopic term ω × Iω as one implicit step (Catto, "Numerical Methods", GDC 2015): in the body frame, one Newton iteration on f(ω) = I(ω − ω₀) + h ω × Iω with the exact Jacobian
    /// J = I + h (ω̃ I − (Iω)~). It conserves angular momentum much better than an explicit step and, unlike a full solve, never blows up for a fast spin because the step is clamped by the speed limit.
    /// </summary>
    private Vector3F Gyroscopic(int b, Vector3F w, float h, ReadOnlySpan<float> rot)
    {
        float qx = rot[4 * b], qy = rot[4 * b + 1], qz = rot[4 * b + 2], qw = rot[4 * b + 3];
        var wl = Rotate(-qx, -qy, -qz, qw, w);
        ReadOnlySpan<float> I = Inertia.AsSpan(6 * b, 6);
        var iw = Apply(I, wl);
        var f = Vector3F.Cross(wl, iw) * h;
        // J = I + h (skew(wl) I − skew(iw)); rows below.
        float a = I[0], bb = I[1], c = I[2], d = I[3], e = I[4], ff = I[5];
        float x = wl.X, y = wl.Y, z = wl.Z;
        // skew(wl) · I: rows (−z·I[1,:] + y·I[2,:]), (z·I[0,:] − x·I[2,:]), (−y·I[0,:] + x·I[1,:])
        float m00 = a + h * (-z * bb + y * c), m01 = bb + h * (-z * d + y * e), m02 = c + h * (-z * e + y * ff);
        float m10 = bb + h * (z * a - x * c), m11 = d + h * (z * bb - x * e), m12 = e + h * (z * c - x * ff);
        float m20 = c + h * (-y * a + x * bb), m21 = e + h * (-y * bb + x * d), m22 = ff + h * (-y * c + x * e);
        // − skew(iw)
        m01 -= h * (-iw.Z); m02 -= h * iw.Y; m10 -= h * iw.Z; m12 -= h * (-iw.X); m20 -= h * (-iw.Y); m21 -= h * iw.X;
        float c00 = m11 * m22 - m12 * m21, c01 = m12 * m20 - m10 * m22, c02 = m10 * m21 - m11 * m20;
        float det = m00 * c00 + m01 * c01 + m02 * c02;
        if (!(Math.Abs(det) > 1e-30f) || !float.IsFinite(det)) { return w; }
        float inv = 1f / det;
        float dx = ((m11 * m22 - m12 * m21) * f.X + (m02 * m21 - m01 * m22) * f.Y + (m01 * m12 - m02 * m11) * f.Z) * inv;
        float dy = (c01 * f.X + (m00 * m22 - m02 * m20) * f.Y + (m02 * m10 - m00 * m12) * f.Z) * inv;
        float dz = (c02 * f.X + (m01 * m20 - m00 * m21) * f.Y + (m00 * m11 - m01 * m10) * f.Z) * inv;
        var nl = new Vector3F(wl.X - dx, wl.Y - dy, wl.Z - dz);
        if (!float.IsFinite(nl.X + nl.Y + nl.Z)) { return w; }
        return Rotate(qx, qy, qz, qw, nl);
    }

    private static void IntegrateRotation(Span<float> q, int o, Vector3F w, float h)
    {
        // Baraff: q' = q + h/2 · (w, 0) · q, renormalised. The product (w, 0)·q is written out.
        float qx = q[o], qy = q[o + 1], qz = q[o + 2], qw = q[o + 3];
        float dx = w.X * qw + w.Y * qz - w.Z * qy;
        float dy = w.Y * qw + w.Z * qx - w.X * qz;
        float dz = w.Z * qw + w.X * qy - w.Y * qx;
        float dw = -(w.X * qx + w.Y * qy + w.Z * qz);
        float hh = 0.5f * h;
        float nx = qx + hh * dx, ny = qy + hh * dy, nz = qz + hh * dz, nw = qw + hh * dw;
        float mag = float.Sqrt(nx * nx + ny * ny + nz * nz + nw * nw);
        if (!(mag > 0f) || !float.IsFinite(mag)) { return; } // a runaway state (damaged or extreme input) keeps the last valid orientation instead of becoming a zero quaternion
        float inv = 1f / mag;
        q[o] = nx * inv; q[o + 1] = ny * inv; q[o + 2] = nz * inv; q[o + 3] = nw * inv;
    }

    public void IntegratePositions(ReadOnlySpan<int> list, in StepContext ctx)
    {
        var bodies = Core.Bodies;
        ReadOnlySpan<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        ReadOnlySpan<float> invMass = bodies.InverseMass, invI = bodies.InverseInertia;
        Span<double> pos = bodies.Position;
        Span<float> rot = bodies.Rotation;
        float h = ctx.SubstepDt;
        foreach (int b in list)
        {
            float dx = h * lin[3 * b], dy = h * lin[3 * b + 1], dz = h * lin[3 * b + 2];
            Dp[3 * b] += dx; Dp[3 * b + 1] += dy; Dp[3 * b + 2] += dz;
            pos[3 * b] += (double)dx; pos[3 * b + 1] += (double)dy; pos[3 * b + 2] += (double)dz;
            var w = V(ang, 3 * b);
            if (w.X != 0f || w.Y != 0f || w.Z != 0f)
            {
                IntegrateRotation(Dq, 4 * b, w, h);
                IntegrateRotation(rot, 4 * b, w, h);
                RefreshWorldInertia(b, invMass, invI, rot);
            }

            PackBody(b);
        }
    }

    // ------------------------------------------------------------------ finalize: forces and continuous collision

    public void FinalizeBodies(ReadOnlySpan<int> list, in StepContext ctx)
    {
        var bodies = Core.Bodies;
        foreach (int b in list)
        {
            if (bodies.TypeOf(b) != BodyType.Dynamic) { continue; }
            Force[3 * b] = 0f; Force[3 * b + 1] = 0f; Force[3 * b + 2] = 0f; Torque[3 * b] = 0f; Torque[3 * b + 1] = 0f; Torque[3 * b + 2] = 0f;
            if (Opt.ContinuousEnabled) { Continuous(b); }
        }
    }

    private struct ToiQuery : IOverlapCallback
    {
        public Engine3D E;
        public int MoverShape;
        public ConvexShape3D Shape;       // the moving convex shape (or compound child)
        public RigidTransformF ShapePose; // its frame at the start: world rotation, translation relative to the start centre of mass (float)
        public double Cx, Cy, Cz;         // start centre of mass
        public Vector3F Motion;           // centre of mass motion over the step, world axes
        public float DepthTolerance;      // hits that would end the step shallower than this are left to the contact solver
        public float Best;                // smallest fraction so far
        public bool Hit;

        public bool Report(int key, int userId)
        {
            var e = E;
            if (e.IsSensor(MoverShape) || e.IsSensor(userId) || !e.CanCollide(MoverShape, userId)) { return true; }
            var core = e.Core;
            int sb = core.Shapes.Owner[userId];
            var bodies = core.Bodies;
            var p = bodies.PositionOf(sb);
            var other = e.Geoms[e.GeomOf[userId]]!;
            // The static shape's frame in world axes and the mover's shape frame in it.
            var pose = e.ShapePoseFromCom(sb, userId);
            var rs = pose.Rotation;
            var inv = rs.Conjugate();
            var off = new Vector3F(
                (float)((Cx + (double)ShapePose.Translation.X) - (p[0] + (double)pose.Translation.X)),
                (float)((Cy + (double)ShapePose.Translation.Y) - (p[1] + (double)pose.Translation.Y)),
                (float)((Cz + (double)ShapePose.Translation.Z) - (p[2] + (double)pose.Translation.Z)));
            var rel = new RigidTransformF(inv * ShapePose.Rotation, inv.Rotate(off));
            var motion = inv.Rotate(Motion);
            // The hit only matters if the body would end the step well inside the surface: shallower overlaps are the contact solver's job, and clamping them would pin a body that is resting on,
            // or rolling over, a surface (its centre of mass moves along the surface while the shape, held at its start orientation, brushes it).
            float t, approach;
            if (other.IsConvex)
            {
                if (!Gjk3D.ShapeCast(other.Convex, Shape, rel, -motion, out var hit) || hit.StartsOverlapping) { return true; }
                t = hit.Fraction; approach = -Vector3F.Dot(motion, hit.Normal);
            }
            else if (other.Compound != null)
            {
                if (!other.Compound.ShapeCast(Shape, rel, motion, out var hit) || hit.StartsOverlapping) { return true; }
                t = hit.Fraction; approach = Vector3F.Dot(motion, hit.Normal);
            }
            else
            {
                if (!other.Soup!.ShapeCast(Shape, rel, motion, out var hit, CastIds()) || hit.StartsOverlapping) { return true; }
                t = hit.Fraction; approach = -Vector3F.Dot(motion, hit.Normal);
            }

            if ((1f - t) * approach < DepthTolerance) { return true; }
            // A surface already within speculative range has a contact of its own; clamping against it would only pin the body at a place where no contact exists yet.
            if (t * float.Sqrt(Motion.X * Motion.X + Motion.Y * Motion.Y + Motion.Z * Motion.Z) < 2f * e.Opt.SpeculativeDistance) { return true; }
            if (t > 0f && t < Best) { Best = t; Hit = true; }
            return true;
        }
    }

    private struct AnyCandidate : IOverlapCallback
    {
        public bool Found;

        public bool Report(int key, int userId)
        {
            Found = true;
            return false;
        }
    }

    // Continuous collision runs on whichever thread finalizes the body, so the id list its soup casts reuse is per thread; it only holds candidates while one cast runs and never decides a result.
    [ThreadStatic]
    private static List<int>? t_castIds;

    private static List<int> CastIds() => t_castIds ??= new List<int>();

    /// <summary>
    /// Continuous collision against static geometry for a body that moved far compared with its size this step: a linear shape cast of each shape along the centre-of-mass motion, with the shape held at its
    /// start orientation, finds the first touch; the body is moved back to just before it (a gap of 5 mm along the motion) with its rotation interpolated. Velocity is unchanged: the next step finds a
    /// speculative contact. Rotation during the step is not part of the cast; it only counts towards deciding that the body moved far enough to need one.
    /// </summary>
    private void Continuous(int b)
    {
        var bodies = Core.Bodies;
        var pos = bodies.PositionOf(b);
        var rot = bodies.RotationOf(b);
        double cx = StartPos[3 * b], cy = StartPos[3 * b + 1], cz = StartPos[3 * b + 2];
        var m = new Vector3F((float)(pos[0] - cx), (float)(pos[1] - cy), (float)(pos[2] - cz));
        var qStart = new QuaternionF(StartRot[4 * b], StartRot[4 * b + 1], StartRot[4 * b + 2], StartRot[4 * b + 3]);
        var qEnd = new QuaternionF(rot[0], rot[1], rot[2], rot[3]);
        var rel = qStart.Conjugate() * qEnd;
        float spin = 2f * float.Sqrt(rel.X * rel.X + rel.Y * rel.Y + rel.Z * rel.Z) * MaxExtent[b];
        float len = float.Sqrt(m.X * m.X + m.Y * m.Y + m.Z * m.Z);
        if (!float.IsFinite(len + spin) || !(len + spin > 0.5f * MinExtent[b]) || !(len > 0f)) { return; } // non-finite motion is reported by the bounds check, not cast

        // Nothing static near the whole sweep: no shape can touch anything, so skip the per-shape bounds and casts. The box holds every point of every shape at the start and the end pose
        // (each within MaxExtent of the centre of mass, padded for float rounding), which contains every per-shape sweep box below, so the answer is the same as without this test.
        double reach = (double)(MaxExtent[b] * 1.01f + 1e-4f);
        var coarse = new Aabb3D(cx + Math.Min(0.0, (double)m.X) - reach, cy + Math.Min(0.0, (double)m.Y) - reach, cz + Math.Min(0.0, (double)m.Z) - reach,
            cx + Math.Max(0.0, (double)m.X) + reach, cy + Math.Max(0.0, (double)m.Y) + reach, cz + Math.Max(0.0, (double)m.Z) + reach);
        var near = new AnyCandidate();
        Bp.QueryAabb(in coarse, ref near, BodyTypeMask.Static);
        if (!near.Found) { return; }

        var lc = LocalCenterOf(b);
        float best = 1f;
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            var g = Geoms[GeomOf[s]]!;
            if (g.Soup != null) { continue; }
            var l = Local[s];
            // Box of the whole sweep: the shape's bounds at the start and at the end.
            var startPose = new RigidTransformF(qStart * l.Rotation, qStart.Rotate(l.Translation - lc));
            var endPose = new RigidTransformF(qEnd * l.Rotation, qEnd.Rotate(l.Translation - lc));
            Vector3F a0, a1, c0, c1;
            if (g.IsConvex) { ShapeBounds3D.Compute(g.Convex, startPose, out a0, out a1); ShapeBounds3D.Compute(g.Convex, endPose, out c0, out c1); }
            else { BoundsOfBox(g.LocalBounds, startPose, out a0, out a1); BoundsOfBox(g.LocalBounds, endPose, out c0, out c1); }
            var box = new Aabb3D(
                cx + (double)Math.Min(a0.X, c0.X + m.X), cy + (double)Math.Min(a0.Y, c0.Y + m.Y), cz + (double)Math.Min(a0.Z, c0.Z + m.Z),
                cx + (double)Math.Max(a1.X, c1.X + m.X), cy + (double)Math.Max(a1.Y, c1.Y + m.Y), cz + (double)Math.Max(a1.Z, c1.Z + m.Z));

            if (g.IsConvex)
            {
                var q = new ToiQuery { E = this, MoverShape = s, Shape = g.Convex, ShapePose = startPose, Cx = cx, Cy = cy, Cz = cz, Motion = m, Best = best, DepthTolerance = 0.25f * MinExtent[b] };
                Bp.QueryAabb(in box, ref q, BodyTypeMask.Static);
                if (q.Hit && q.Best < best) { best = q.Best; }
            }
            else
            {
                var comp = g.Compound!;
                for (int j = 0; j < comp.SlotCount; j++)
                {
                    if (!comp.IsLive(j)) { continue; }
                    var child = comp.GetChild(j);
                    var childPose = new RigidTransformF(startPose.Rotation * child.Local.Rotation, startPose.Rotation.Rotate(child.Local.Translation) + startPose.Translation);
                    var q = new ToiQuery { E = this, MoverShape = s, Shape = child.Shape, ShapePose = childPose, Cx = cx, Cy = cy, Cz = cz, Motion = m, Best = best, DepthTolerance = 0.25f * MinExtent[b] };
                    Bp.QueryAabb(in box, ref q, BodyTypeMask.Static);
                    if (q.Hit && q.Best < best) { best = q.Best; }
                }
            }
        }

        if (best >= 1f) { return; }
        float t = best - 0.005f / len;
        if (t < 0f) { t = 0f; }
        pos[0] = cx + (double)(t * m.X); pos[1] = cy + (double)(t * m.Y); pos[2] = cz + (double)(t * m.Z);
        // Normalised lerp along the shorter arc.
        float dot = qStart.X * qEnd.X + qStart.Y * qEnd.Y + qStart.Z * qEnd.Z + qStart.W * qEnd.W;
        float sgn = dot < 0f ? -1f : 1f;
        float nx = (1f - t) * qStart.X + t * sgn * qEnd.X, ny = (1f - t) * qStart.Y + t * sgn * qEnd.Y, nz = (1f - t) * qStart.Z + t * sgn * qEnd.Z, nw = (1f - t) * qStart.W + t * sgn * qEnd.W;
        float mag = float.Sqrt(nx * nx + ny * ny + nz * nz + nw * nw);
        if (mag > 0f) { rot[0] = nx / mag; rot[1] = ny / mag; rot[2] = nz / mag; rot[3] = nw / mag; }
        Dp[3 * b] = (float)(pos[0] - cx); Dp[3 * b + 1] = (float)(pos[1] - cy); Dp[3 * b + 2] = (float)(pos[2] - cz);
    }

    // ------------------------------------------------------------------ sleeping

    public float BodyMotion(int b)
    {
        var bodies = Core.Bodies;
        // A body touching a moving kinematic body is not at rest.
        var cs = Core.Constraints;
        for (int e = bodies.FirstEdge(b); e >= 0; e = cs.NextEdge(e))
        {
            int c = e >> 1;
            int other = (e & 1) == 0 ? cs.BodyB[c] : cs.BodyA[c];
            if (bodies.TypeOf(other) == BodyType.Kinematic && cs.PointCount[c] > 0 && IsActive(bodies, other)) { return float.MaxValue; }
        }

        var v = bodies.LinearVelocityOf(b);
        var w = bodies.AngularVelocityOf(b);
        return float.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) + float.Sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) * MaxExtent[b];
    }
}
