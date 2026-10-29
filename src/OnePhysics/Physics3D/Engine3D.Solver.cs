using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
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
        // Every per-body array was sized for the world's high-water mark before any step or call that reaches here (EnsureBodies), so the unchecked accessors stay in range; the Checked build verifies each access.
        Span<float> r = Pk;
        ReadOnlySpan<float> dp = Dp, dq = Dq, w = WInvI;
        int o = 16 * b, p3 = 3 * b, p4 = 4 * b, p6 = 6 * b;
        Mem.At(r, o) = Mem.At(dp, p3); Mem.At(r, o + 1) = Mem.At(dp, p3 + 1); Mem.At(r, o + 2) = Mem.At(dp, p3 + 2);
        Mem.At(r, o + 3) = Mem.At(dq, p4); Mem.At(r, o + 4) = Mem.At(dq, p4 + 1); Mem.At(r, o + 5) = Mem.At(dq, p4 + 2); Mem.At(r, o + 6) = Mem.At(dq, p4 + 3);
        Mem.At(r, o + 7) = Mem.At(w, p6); Mem.At(r, o + 8) = Mem.At(w, p6 + 1); Mem.At(r, o + 9) = Mem.At(w, p6 + 2); Mem.At(r, o + 10) = Mem.At(w, p6 + 3); Mem.At(r, o + 11) = Mem.At(w, p6 + 4); Mem.At(r, o + 12) = Mem.At(w, p6 + 5);
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
        // Callers pass arrays sized for body b (see PackBody).
        Span<float> w = WInvI;
        int p6 = 6 * b;
        if (!(Mem.At(invMass, b) > 0f)) { for (int k = 0; k < 6; k++) { Mem.At(w, p6 + k) = 0f; } return; }
        int p4 = 4 * b;
        float x = Mem.At(rot, p4), y = Mem.At(rot, p4 + 1), z = Mem.At(rot, p4 + 2), s = Mem.At(rot, p4 + 3);
        float xx = x * x, yy = y * y, zz = z * z, xy = x * y, xz = x * z, yz = y * z, sx = s * x, sy = s * y, sz = s * z;
        float r00 = 1f - 2f * (yy + zz), r01 = 2f * (xy - sz), r02 = 2f * (xz + sy);
        float r10 = 2f * (xy + sz), r11 = 1f - 2f * (xx + zz), r12 = 2f * (yz - sx);
        float r20 = 2f * (xz - sy), r21 = 2f * (yz + sx), r22 = 1f - 2f * (xx + yy);
        float ia = Mem.At(invI, p6), ib = Mem.At(invI, p6 + 1), ic = Mem.At(invI, p6 + 2), id = Mem.At(invI, p6 + 3), ie = Mem.At(invI, p6 + 4), ifz = Mem.At(invI, p6 + 5);
        float t00 = r00 * ia + r01 * ib + r02 * ic, t01 = r00 * ib + r01 * id + r02 * ie, t02 = r00 * ic + r01 * ie + r02 * ifz;
        float t10 = r10 * ia + r11 * ib + r12 * ic, t11 = r10 * ib + r11 * id + r12 * ie, t12 = r10 * ic + r11 * ie + r12 * ifz;
        float t20 = r20 * ia + r21 * ib + r22 * ic, t21 = r20 * ib + r21 * id + r22 * ie, t22 = r20 * ic + r21 * ie + r22 * ifz;
        Mem.At(w, p6) = t00 * r00 + t01 * r01 + t02 * r02;
        Mem.At(w, p6 + 1) = t00 * r10 + t01 * r11 + t02 * r12;
        Mem.At(w, p6 + 2) = t00 * r20 + t01 * r21 + t02 * r22;
        Mem.At(w, p6 + 3) = t10 * r10 + t11 * r11 + t12 * r12;
        Mem.At(w, p6 + 4) = t10 * r20 + t11 * r21 + t12 * r22;
        Mem.At(w, p6 + 5) = t20 * r20 + t21 * r21 + t22 * r22;
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
        if (BatchInfo.Length < 4 * nc) { BatchInfo = new int[Math.Max(4 * nc, BatchInfo.Length * 2)]; }
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
        var w = Wide.AsSpan(group * W3D.GroupFloats, W3D.GroupFloats);
        var idx = WideIdx.AsSpan(group * W3D.IdxPerGroup, W3D.IdxPerGroup);
        for (int l = 0; l < W3D.Lanes; l++) { idx[l] = 0; idx[8 + l] = 0; idx[16 + l] = -1; idx[24 + l] = -1; }
        int maxPoints = 1, anyRolling = 0, anyRestitution = 0;
        float allOnes = BitConverter.Int32BitsToSingle(-1);
        int lane = 0, jlane = 0, scalarJoints = 0;
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

                if (jt != (byte)JointType3D.Point) { PrepareJoint(c, in ctx); scalarJoints = 1; }

                continue;
            }

            var pl = cs.Payload(c);
            int a = bodyA[c], b = bodyB[c];
            SlotOf[c] = group * W3D.Lanes + lane;
            float mA = invMass[a], mB = invMass[b];
            idx[lane] = a; idx[8 + lane] = b;
            idx[16 + lane] = mA > 0f ? a : -1; idx[24 + lane] = mB > 0f ? b : -1;
            var n = V(pl, ManifoldLayout.Normal);
            Tangents(n, out var t1, out var t2);
            Put3(w, W3D.N, lane, n); Put3(w, W3D.T1, lane, t1); Put3(w, W3D.T2, lane, t2);
            w[W3D.Friction * 8 + lane] = pl[ManifoldLayout.Friction];
            float restitution = pl[ManifoldLayout.Restitution];
            w[W3D.Restitution * 8 + lane] = restitution;
            if (restitution != 0f) { anyRestitution = 1; }
            float rolling = pl[ManifoldLayout.RollingResistance];
            w[W3D.Rolling * 8 + lane] = rolling;
            w[W3D.InvMassA * 8 + lane] = mA; w[W3D.InvMassB * 8 + lane] = mB;
            if (mA == 0f || mB == 0f) { w[W3D.BiasRate * 8 + lane] = _stBias; w[W3D.MassScale * 8 + lane] = _stMass; w[W3D.ImpulseScale * 8 + lane] = _stImpulse; }
            else { w[W3D.BiasRate * 8 + lane] = _dynBias; w[W3D.MassScale * 8 + lane] = _dynMass; w[W3D.ImpulseScale * 8 + lane] = _dynImpulse; }
            if (rolling > 0f) { anyRolling = 1; RollingMass(WInvI.AsSpan(6 * a, 6), WInvI.AsSpan(6 * b, 6), rm); for (int k = 0; k < 6; k++) { w[(W3D.RollMass + k) * 8 + lane] = rm[k]; } }
            Put3(w, W3D.RollImpulse, lane, V(pl, ManifoldLayout.RollingImpulse));

            // The effective masses, adjusted separations and relative velocities are computed for the whole group at once afterwards (ContactKernel3D.PrepareMasses), which gathers the bodies itself.
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
            ReadOnlySpan<float> linP = bodies.LinearVelocityPadded, angP = bodies.AngularVelocityPadded;
            if (KernelWidth == 1) { ContactKernel3D.PrepareMasses<Lanes1>(w, idx, linP, angP, Pk, maxPoints); }
            else if (KernelWidth == 4) { ContactKernel3D.PrepareMasses<Lanes128>(w, idx, linP, angP, Pk, maxPoints); }
            else { ContactKernel3D.PrepareMasses<Lanes256>(w, idx, linP, angP, Pk, maxPoints); }
        }

        if (constraints.Length > 0) { var bi = BatchInfo.AsSpan(4 * constraints[0], 4); bi[0] = group; bi[1] = lane; bi[2] = jlane; bi[3] = scalarJoints; }
        idx[W3D.MaxPointsSlot] = maxPoints; idx[W3D.AnyRollingSlot] = anyRolling; idx[W3D.AnyRestitutionSlot] = anyRestitution;
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

        // Contacts were packed into the first lanes of the batch's group in list order, point joints into the first lanes of its joint group; both counts were worked out when the batch was prepared.
        // Joints that are not point blocks (which share no dynamic body with anything else in the batch) solve one by one.
        var info = BatchInfo.AsSpan(4 * constraints[0], 4);
        int group = info[0], count = info[1], jcount = info[2];
        if (info[3] != 0)
        {
            for (int i = 0; i < constraints.Length; i++)
            {
                int c = constraints[i];
                if (cons.KindOf(c) == ConstraintKind.Joint && Joints[c].Type != (byte)JointType3D.Point) { SolveJoint(c, phase, in ctx); }
            }
        }

        if (jcount > 0)
        {
            int jgroup = group;
            if (KernelWidth == 1) { JointKernel3D.Run<Lanes1>(this, jgroup, 0, jcount, phase, in _jointParams); }
            else if (KernelWidth == 4) { JointKernel3D.Run<Lanes128>(this, jgroup, 0, jcount, phase, in _jointParams); }
            else { JointKernel3D.Run<Lanes256>(this, jgroup, 0, jcount, phase, in _jointParams); }
        }

        if (count == 0) { return; }
        if (KernelWidth == 1) { ContactKernel3D.Run<Lanes1>(this, group, 0, count, phase, in kp); }
        else if (KernelWidth == 4) { ContactKernel3D.Run<Lanes128>(this, group, 0, count, phase, in kp); }
        else { ContactKernel3D.Run<Lanes256>(this, group, 0, count, phase, in kp); }
    }

    // ------------------------------------------------------------------ integration

    /// <summary>Gravity, forces, torque, the gyroscopic term, damping and the speed limits for the bodies of the list, eight at a time (see <see cref="IntegrateKernel3D"/>).</summary>
    public void IntegrateVelocities(ReadOnlySpan<int> list, in StepContext ctx)
    {
        if (KernelWidth == 1) { IntegrateKernel3D.Velocities<Lanes1>(this, list, ctx.SubstepDt); }
        else if (KernelWidth == 4) { IntegrateKernel3D.Velocities<Lanes128>(this, list, ctx.SubstepDt); }
        else { IntegrateKernel3D.Velocities<Lanes256>(this, list, ctx.SubstepDt); }
    }

    /// <summary>Displacement, rotations, world inverse inertia and the packed rows of the bodies of the list, eight at a time (see <see cref="IntegrateKernel3D"/>).</summary>
    public void IntegratePositions(ReadOnlySpan<int> list, in StepContext ctx)
    {
        if (KernelWidth == 1) { IntegrateKernel3D.Positions<Lanes1>(this, list, ctx.SubstepDt); }
        else if (KernelWidth == 4) { IntegrateKernel3D.Positions<Lanes128>(this, list, ctx.SubstepDt); }
        else { IntegrateKernel3D.Positions<Lanes256>(this, list, ctx.SubstepDt); }
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
            var inv = rs.Conjugated();
            var off = new Vector3F(
                (float)((Cx - p[0]) + ((double)ShapePose.Translation.X - (double)pose.Translation.X)),
                (float)((Cy - p[1]) + ((double)ShapePose.Translation.Y - (double)pose.Translation.Y)),
                (float)((Cz - p[2]) + ((double)ShapePose.Translation.Z - (double)pose.Translation.Z)));
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
                // The compound refuses a sweep that leaves its coordinate range (a fast body far from the compound), so the sweep is cut to the range first, as the query path does.
                if (!ClipToReach(rel.Translation, motion, out var co, out var cd, out float t0, out float t1)) { return true; }
                if (!other.Compound.ShapeCast(Shape, new RigidTransformF(rel.Rotation, co), cd, out var hit) || hit.StartsOverlapping) { return true; }
                t = t0 + (hit.Fraction * (t1 - t0)); approach = Vector3F.Dot(motion, hit.Normal);
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
        var rel = qStart.Conjugated() * qEnd;
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
