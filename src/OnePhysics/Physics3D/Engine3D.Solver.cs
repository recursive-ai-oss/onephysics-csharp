using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    // ------------------------------------------------------------------ small math

    /// <summary>Applies a symmetric tensor stored as xx xy xz yy yz zz to a vector.</summary>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
    private static Vector3F Apply(ReadOnlySpan<float> m, Vector3F v) => new(
        m[0] * v.X + m[1] * v.Y + m[2] * v.Z,
        m[1] * v.X + m[3] * v.Y + m[4] * v.Z,
        m[2] * v.X + m[4] * v.Y + m[5] * v.Z);

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
    private static Vector3F V(ReadOnlySpan<float> a, int i) => new(a[i], a[i + 1], a[i + 2]);

    private static void Put(Span<float> a, int i, Vector3F v) { a[i] = v.X; a[i + 1] = v.Y; a[i + 2] = v.Z; }

    /// <summary>Copies the velocities, world inverse inertia, displacement and accumulated rotation of a body into its packed rows (see <see cref="Pk"/>); from here to FinalizeBodies row 0 is the body's velocity.</summary>
    public void PackBody(int b)
    {
        // Every per-body array was sized for the world's high-water mark before any step or call that reaches here (EnsureBodies), so the unchecked accessors stay in range; the Checked build verifies each access.
        Span<float> r = Pk;
        var bodies = Core.Bodies;
        ReadOnlySpan<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity, dp = Dp, dq = Dq, w = WInvI;
        int o = PkStride * b, p3 = 3 * b, p4 = 4 * b, p6 = 6 * b;
        Mem.At(r, o) = Mem.At(lin, p3); Mem.At(r, o + 1) = Mem.At(lin, p3 + 1); Mem.At(r, o + 2) = Mem.At(lin, p3 + 2);
        Mem.At(r, o + 3) = Mem.At(ang, p3); Mem.At(r, o + 4) = Mem.At(ang, p3 + 1); Mem.At(r, o + 5) = Mem.At(ang, p3 + 2); Mem.At(r, o + 6) = 0f; Mem.At(r, o + 7) = 0f;
        o += PkInertia;
        Mem.At(r, o) = Mem.At(w, p6); Mem.At(r, o + 1) = Mem.At(w, p6 + 1); Mem.At(r, o + 2) = Mem.At(w, p6 + 2); Mem.At(r, o + 3) = Mem.At(w, p6 + 3); Mem.At(r, o + 4) = Mem.At(w, p6 + 4); Mem.At(r, o + 5) = Mem.At(w, p6 + 5); Mem.At(r, o + 6) = 0f; Mem.At(r, o + 7) = 0f;
        o += PkPose - PkInertia;
        Mem.At(r, o) = Mem.At(dp, p3); Mem.At(r, o + 1) = Mem.At(dp, p3 + 1); Mem.At(r, o + 2) = Mem.At(dp, p3 + 2);
        Mem.At(r, o + 3) = Mem.At(dq, p4); Mem.At(r, o + 4) = Mem.At(dq, p4 + 1); Mem.At(r, o + 5) = Mem.At(dq, p4 + 2); Mem.At(r, o + 6) = Mem.At(dq, p4 + 3); Mem.At(r, o + 7) = 0f;
    }

    /// <summary>Copies a body's velocities from the body store into row 0 of its packed rows: for code that changes a velocity in the body store, or wakes a body, between the contact stage's body initialisation and the solve (see <see cref="Pk"/>).</summary>
    internal void PackVelocity(int b)
    {
        Span<float> r = Pk;
        var bodies = Core.Bodies;
        ReadOnlySpan<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        int o = PkStride * b, p3 = 3 * b;
        Mem.At(r, o) = Mem.At(lin, p3); Mem.At(r, o + 1) = Mem.At(lin, p3 + 1); Mem.At(r, o + 2) = Mem.At(lin, p3 + 2);
        Mem.At(r, o + 3) = Mem.At(ang, p3); Mem.At(r, o + 4) = Mem.At(ang, p3 + 1); Mem.At(r, o + 5) = Mem.At(ang, p3 + 2); Mem.At(r, o + 6) = 0f; Mem.At(r, o + 7) = 0f;
    }

    /// <summary>Copies a body's world inverse inertia into row 1 of its packed rows, after something other than the position pass changed it (axis locks).</summary>
    private void PackInertia(int b)
    {
        Span<float> r = Pk;
        ReadOnlySpan<float> w = WInvI;
        int o = PkStride * b + PkInertia, p6 = 6 * b;
        Mem.At(r, o) = Mem.At(w, p6); Mem.At(r, o + 1) = Mem.At(w, p6 + 1); Mem.At(r, o + 2) = Mem.At(w, p6 + 2); Mem.At(r, o + 3) = Mem.At(w, p6 + 3); Mem.At(r, o + 4) = Mem.At(w, p6 + 4); Mem.At(r, o + 5) = Mem.At(w, p6 + 5);
    }

    /// <summary>A body's velocity while a step's solve runs: the linear (0 to 2) and angular (3 to 5) lanes of row 0 of its packed rows (see <see cref="Pk"/>). The scalar parts of the solve read and write it here.</summary>
    internal Span<float> SolveVelocity(int b) => Pk.AsSpan(PkStride * b + PkVelocity, 6);

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
        if (Core.Controls.HasLocks) ProjectInertia(b);
    }

    /// <summary>Two unit vectors perpendicular to a unit normal and to each other, chosen by a fixed rule so the same normal always gives the same pair.</summary>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
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
    private float _liteDynBias, _liteDynMass, _liteDynImpulse, _liteStBias, _liteStMass, _liteStImpulse;
    private JointKernelParams _jointParams, _liteJointParams;

    /// <summary>Once per step before anything is prepared: sizes the wide groups (one per batch) and the constraint-to-lane map, and computes the soft-constraint coefficients.</summary>
    public void BeginSolve(int batches, in StepContext ctx)
    {
        if (ctx.Schedule != 0)
        {
            // The contact-only schedule has its own substep length, so its contacts get their own soft-constraint coefficients; nothing else is sized or ordered twice.
            float liteCap = 0.25f * ctx.InverseSubstepDt;
            float liteHertz = Opt.ContactHertz > liteCap ? liteCap : Opt.ContactHertz;
            MakeSoft(liteHertz, Opt.ContactDampingRatio, ctx.SubstepDt, out _liteDynBias, out _liteDynMass, out _liteDynImpulse);
            MakeSoft(2f * liteHertz, Opt.ContactDampingRatio, ctx.SubstepDt, out _liteStBias, out _liteStMass, out _liteStImpulse);
            float liteJh = Opt.JointHertz > liteCap ? liteCap : Opt.JointHertz;
            MakeSoft(liteJh, Opt.JointDampingRatio, ctx.SubstepDt, out float lbias, out float lmass, out float limp);
            _liteJointParams = new JointKernelParams(lmass, lmass * lbias, limp);
            return;
        }

        if (Wide.Length < batches * ContactGroupFloats)
        {
            int cap = Math.Max(batches, Wide.Length / ContactGroupFloats * 2);
            Wide = new float[cap * ContactGroupFloats]; WideIdx = new int[cap * W3D.IdxPerGroup];
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

        if (Core.JointReadback.Enabled && _trackedJoints > 0) { Core.JointReadback.ReserveWide(batches); }
        float jh = Opt.JointHertz > cap2 ? cap2 : Opt.JointHertz;
        MakeSoft(jh, Opt.JointDampingRatio, ctx.SubstepDt, out float jbias, out float jmass, out float jimp);
        _jointParams = new JointKernelParams(jmass, jmass * jbias, jimp);
    }

    private int[] _sortTmp = new int[64];

    /// <summary>
    /// Reorders each color's constraints, stably, by contact point count (most points first, joints last), so the contacts of a batch have similar point counts: a group runs as many point passes as its
    /// largest lane needs, and a pile is mostly one-point contacts with a few four-point ones that would otherwise stretch every batch they land in. The constraints of a color share no dynamic body,
    /// so the order they are solved in cannot change a result; the new order depends only on the data.
    /// r4 h09: one task per color (Executor.For over ColorCount). A color sorts through its own range of <see cref="_sortTmp"/> (the same offsets as in the list), so tasks share no scratch.
    /// </summary>
    private void GroupColorsByPointCount()
    {
        var list = Core.Coloring.ListArray;
        if (_sortTmp.Length < list.Length) { _sortTmp = new int[Math.Max(list.Length, _sortTmp.Length * 2)]; }
        Core.Executor.For(ConstraintColoring.ColorCount, new GroupColorTask(this, list));
    }

    private readonly struct GroupColorTask : IParallelBody
    {
        private readonly Engine3D _e;
        private readonly int[] _list;
        public GroupColorTask(Engine3D e, int[] list) { _e = e; _list = list; }
        public void Run(int g) => _e.GroupColorByPointCount(g, _list);
    }

    private void GroupColorByPointCount(int g, int[] list)
    {
        var coloring = Core.Coloring;
        var cs = Core.Constraints;
        ReadOnlySpan<byte> points = cs.PointCount;
        int n = coloring.Color(g).Length;
        if (n <= ConstraintColoring.Lanes) { return; }
        int off = coloring.ColorStart(g);
        // Bucket 0 holds four points, 1 three, 2 two, 3 one, 4 none, 5 joints (and any other kind).
        Span<int> start = stackalloc int[W3D.MaxPoints + 3];
        start.Clear();
        for (int i = 0; i < n; i++) { start[1 + Bucket(cs, points, list[off + i])]++; }
        if (start[1 + 5] == n || start[1] == n || start[1 + 3] == n) { return; }
        for (int k = 1; k < start.Length; k++) { start[k] += start[k - 1]; }
        var tmp = _sortTmp.AsSpan(off, n);
        for (int i = 0; i < n; i++) { int c = list[off + i]; tmp[start[Bucket(cs, points, c)]++] = c; }
        tmp.CopyTo(list.AsSpan(off, n));
    }

    private static int Bucket(ConstraintStore cs, ReadOnlySpan<byte> points, int c) => cs.KindOf(c) == ConstraintKind.Contact ? W3D.MaxPoints - Math.Min((int)points[c], W3D.MaxPoints) : 5;

    /// <summary>
    /// Once per step: effective masses, adjusted separations, relative velocities for restitution and soft coefficients of the contacts of one batch, from the manifolds stored in the payloads, into one wide group.
    /// The group also holds the impulses while the step runs; <see cref="StoreResults"/> writes them back to the payloads.
    /// </summary>
    public void PrepareBatch(ReadOnlySpan<int> constraints, int group, in StepContext ctx)
    {
        // At the 8-lane width the group is built by whole-window gathers and 32-byte field stores (PrepareBatch256); the narrower widths keep the per-lane writes below, and so does a step with
        // contact modification data (its extra fields and tangent overrides are written per lane).
        if (KernelWidth == 8 && Avx2.IsSupported && !ContactModifications.Extended) { PrepareBatch256(constraints, group, in ctx); return; }
        var core = Core;
        var cs = core.Constraints;
        var bodies = core.Bodies;
        ReadOnlySpan<int> bodyA = cs.BodyA, bodyB = cs.BodyB;
        ReadOnlySpan<float> invMass = bodies.InverseMass;
        var w = Wide.AsSpan(group * ContactGroupFloats, ContactGroupFloats);
        var idx = WideIdx.AsSpan(group * W3D.IdxPerGroup, W3D.IdxPerGroup);
        for (int l = 0; l < W3D.Lanes; l++) { idx[l] = 0; idx[8 + l] = 0; idx[16 + l] = -1; idx[24 + l] = -1; }
        int maxPoints = 1, anyRolling = 0, anyRestitution = 0;
        float allOnes = BitConverter.Int32BitsToSingle(-1);
        int lane = 0, jlane = 0, scalarJoints = 0;
        Span<float> rm = stackalloc float[6];
        // Accessor bounds: a batch holds at most W3D.Lanes contacts, so lane < 8 and every field offset below is a constant inside the group (ContactGroupFloats); payload offsets are fixed layout offsets and
        // point indices stay below the point count, which restore validates against MaxManifoldPoints. The Checked configuration verifies each access.
        foreach (int c in constraints)
        {
            if (cs.KindOf(c) == ConstraintKind.Joint)
            {
                byte jt = Joints[c].Type;
                bool springFixed = jt == (byte)JointType3D.Fixed && Joints[c].Hertz > 0f;
                if (JointIsLocked(c) || springFixed) SlotOf[c] = -1;
                if (!JointIsLocked(c) && !springFixed && WidePointBlock(jt))
                {
                    if (jlane == 0) { ClearJointIdx(group); }
                    PreparePointJoint(c, group, jlane++);
                }

                if (jt != (byte)JointType3D.Point || JointIsLocked(c)) { PrepareJoint(c, in ctx); scalarJoints = 1; }

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
            if (ContactModifications.Extended)
            {
                var extra = ContactModifications.ExtraOf(c);
                if (extra[9] != 0) { t1 = new Vector3F(extra[4], extra[5], extra[6]); t2 = Vector3F.Cross(n, t1); }
            }
            Put3(w, W3D.N, lane, n); Put3(w, W3D.T1, lane, t1); Put3(w, W3D.T2, lane, t2);
            Mem.At(w, W3D.Friction * 8 + lane) = Mem.At(pl, ManifoldLayout.Friction);
            if (ContactModifications.Extended)
            {
                Mem.At(w, W3D.TangentSpeed * 8 + lane) = ContactModifications.TangentSpeed(c);
                Mem.At(w, W3D.ImpulseLimit * 8 + lane) = ContactModifications.ImpulseLimit(c);
                var extra = ContactModifications.ExtraOf(c);
                Mem.At(w, W3D.TangentSpeed2 * 8 + lane) = extra[2];
                Mem.At(w, W3D.Friction2 * 8 + lane) = extra[3];
                Mem.At(w, W3D.Spin * 8 + lane) = extra[7];
                Mem.At(w, W3D.SpinImpulse * 8 + lane) = 0;
                Mem.At(w, W3D.Anisotropic * 8 + lane) = extra[9];
            }
            float restitution = Mem.At(pl, ManifoldLayout.Restitution);
            Mem.At(w, W3D.Restitution * 8 + lane) = restitution;
            if (restitution != 0f) { anyRestitution = 1; }
            float rolling = Mem.At(pl, ManifoldLayout.RollingResistance);
            Mem.At(w, W3D.Rolling * 8 + lane) = rolling;
            Mem.At(w, W3D.InvMassA * 8 + lane) = mA; Mem.At(w, W3D.InvMassB * 8 + lane) = mB;
            if (ctx.Schedule != 0)
            {
                if (mA == 0f || mB == 0f) { Mem.At(w, W3D.BiasRate * 8 + lane) = _liteStBias; Mem.At(w, W3D.MassScale * 8 + lane) = _liteStMass; Mem.At(w, W3D.ImpulseScale * 8 + lane) = _liteStImpulse; }
                else { Mem.At(w, W3D.BiasRate * 8 + lane) = _liteDynBias; Mem.At(w, W3D.MassScale * 8 + lane) = _liteDynMass; Mem.At(w, W3D.ImpulseScale * 8 + lane) = _liteDynImpulse; }
            }
            else if (mA == 0f || mB == 0f) { Mem.At(w, W3D.BiasRate * 8 + lane) = _stBias; Mem.At(w, W3D.MassScale * 8 + lane) = _stMass; Mem.At(w, W3D.ImpulseScale * 8 + lane) = _stImpulse; }
            else { Mem.At(w, W3D.BiasRate * 8 + lane) = _dynBias; Mem.At(w, W3D.MassScale * 8 + lane) = _dynMass; Mem.At(w, W3D.ImpulseScale * 8 + lane) = _dynImpulse; }
            if (rolling > 0f) { anyRolling = 1; RollingMass(WInvI.AsSpan(6 * a, 6), WInvI.AsSpan(6 * b, 6), rm); for (int k = 0; k < 6; k++) { Mem.At(w, (W3D.RollMass + k) * 8 + lane) = rm[k]; } }
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
                Mem.At(w, (f + W3D.AdjustedSeparation) * 8 + lane) = Mem.At(pl, ManifoldLayout.Separation(i));
                Mem.At(w, (f + W3D.NormalImpulse) * 8 + lane) = Mem.At(pl, ManifoldLayout.NormalImpulse(i));
                Mem.At(w, (f + W3D.Tangent1Impulse) * 8 + lane) = Mem.At(pl, ManifoldLayout.Tangent1Impulse(i));
                Mem.At(w, (f + W3D.Tangent2Impulse) * 8 + lane) = Mem.At(pl, ManifoldLayout.Tangent2Impulse(i));
                Mem.At(w, (f + W3D.MaxNormalImpulse) * 8 + lane) = 0f;
                Mem.At(w, (f + W3D.Mask) * 8 + lane) = allOnes;
            }

            // Slots past the manifold's points take no part: their mask is clear and their impulse reads as zero in the rolling-resistance total. Everything else in the slot is ignored under the mask.
            for (int i = count; i < W3D.MaxPoints; i++)
            {
                int f = W3D.Point0 + i * W3D.PointStride;
                Mem.At(w, (f + W3D.Mask) * 8 + lane) = 0f; Mem.At(w, (f + W3D.NormalImpulse) * 8 + lane) = 0f;
            }

            lane++;
        }

        if (lane > 0)
        {
            // The velocities come from the packed rows, which hold the start-of-step velocities here (no solve pass has run yet).
            if (KernelWidth == 1) { ContactKernel3D.PrepareMasses<Lanes1>(w, idx, Pk, maxPoints); }
            else if (KernelWidth == 4) { ContactKernel3D.PrepareMasses<Lanes128>(w, idx, Pk, maxPoints); }
            else { ContactKernel3D.PrepareMasses<Lanes256>(w, idx, Pk, maxPoints); }
        }

        if (Core.Controls.Active) PrepareControlledContacts(constraints);
        if (constraints.Length > 0) { var bi = BatchInfo.AsSpan(4 * constraints[0], 4); bi[0] = group; bi[1] = lane; bi[2] = jlane; bi[3] = scalarJoints; }
        idx[W3D.MaxPointsSlot] = maxPoints; idx[W3D.AnyRollingSlot] = anyRolling; idx[W3D.AnyRestitutionSlot] = anyRestitution;
    }

    /// <summary>
    /// r4 h10: <see cref="PrepareBatch"/> at the 8-lane width. The scalar loop keeps only what is per constraint (the joints, the slots, the body rows, the point counts and payload offsets); the group's fields
    /// are then built for all eight lanes at once (<see cref="BuildContacts256"/>): the eight payloads are read as 8-float windows at the same <see cref="ManifoldLayout"/> offsets (eight 32-byte loads and an
    /// 8x8 transpose per window; the windows at 0, 8, ..., 48 cover the header and the points up to the group's largest point count, and a 3D payload is 104 floats, so the last window ends inside its own
    /// payload and needs no padding) and every field is written with one 32-byte store, so <see cref="PrepareMasses256"/> (the register-resident form of ContactKernel3D.PrepareMasses) and the solver
    /// load each field from one store instead of eight.
    /// Every value is a copy of the scalar path's or the same IEEE operation on the same inputs (the tangents, the soft-coefficient choice by a select on mA == 0 || mB == 0, the point masks and zeroed
    /// impulses by a compare with the point count, the rolling mass), so the bits are the scalar path's in every lane the scalar path writes. Lanes past the batch read the first constraint's payload with a
    /// point count of zero; nothing reads them back (as before, when they held stale values). Masked point slots (past a lane's point count) and the rolling mass of lanes without rolling resistance hold
    /// computed values instead of stale ones; every reader selects them out (the normal impulse, which the rolling total reads unmasked, is zeroed as before).
    /// The point joints of the batch get the same treatment: the loop writes their slots and body rows, and <see cref="BuildJoints256"/> builds their group's fields eight lanes at a time (this replaces
    /// <see cref="PreparePointJoint"/>, which the narrower widths still use).
    /// </summary>
    [System.Runtime.CompilerServices.SkipLocalsInit]
    private void PrepareBatch256(ReadOnlySpan<int> constraints, int group, in StepContext ctx)
    {
        var core = Core;
        var cs = core.Constraints;
        var bodies = core.Bodies;
        ReadOnlySpan<int> bodyA = cs.BodyA, bodyB = cs.BodyB;
        ReadOnlySpan<float> invMass = bodies.InverseMass;
        ReadOnlySpan<byte> pointCount = cs.PointCount;
        var idx = WideIdx.AsSpan(group * W3D.IdxPerGroup, W3D.IdxPerGroup);
        Vector256<int>.Zero.StoreUnsafe(ref idx[0]); Vector256<int>.Zero.StoreUnsafe(ref idx[8]); Vector256<int>.AllBitsSet.StoreUnsafe(ref idx[16]); Vector256<int>.AllBitsSet.StoreUnsafe(ref idx[24]);
        // Per-lane scalars in fixed buffers (no GC references, so nothing to zero in the prolog); the vectors are built element by element from them, each element load forwarding from its own store.
        // Every lane of them is written (by the loop, then the padding) before it is read.
        Unsafe.SkipInit(out Lane8Int off); Unsafe.SkipInit(out Lane8Int cnt); Unsafe.SkipInit(out Lane8Float msA); Unsafe.SkipInit(out Lane8Float msB);
        Unsafe.SkipInit(out Lane8Int jc); Unsafe.SkipInit(out Lane8Float jmA); Unsafe.SkipInit(out Lane8Float jmB);
        int ps = cs.PayloadStride;
        int maxPoints = 1, lane = 0, jlane = 0, scalarJoints = 0;
        foreach (int c in constraints)
        {
            if (cs.KindOf(c) == ConstraintKind.Joint)
            {
                byte jt = Joints[c].Type;
                bool springFixed = jt == (byte)JointType3D.Fixed && Joints[c].Hertz > 0f;
                if (JointIsLocked(c) || springFixed) SlotOf[c] = -1;
                if (!JointIsLocked(c) && !springFixed && WidePointBlock(jt))
                {
                    // PreparePointJoint's per-joint part (slot, readback slot, body rows); its fields are built for the whole joint group by BuildJoints256 after the loop.
                    if (jlane == 0) { ClearJointIdx(group); }
                    var jidx = WideJIdx.AsSpan(group * J3D.IdxPerGroup, J3D.IdxPerGroup);
                    int ja = bodyA[c], jb = bodyB[c];
                    SlotOf[c] = group * J3D.Lanes + jlane;
                    if (core.JointReadback.Enabled) { core.JointReadback.WideSlots[group * 8 + jlane] = c; }
                    float jA = invMass[ja], jB = invMass[jb];
                    jidx[jlane] = ja; jidx[8 + jlane] = jb;
                    jidx[16 + jlane] = jA > 0f ? ja : -1; jidx[24 + jlane] = jB > 0f ? jb : -1;
                    jc[jlane] = c; jmA[jlane] = jA; jmB[jlane] = jB;
                    jlane++;
                }

                if (jt != (byte)JointType3D.Point || JointIsLocked(c)) { PrepareJoint(c, in ctx); scalarJoints = 1; }

                continue;
            }

            int a = bodyA[c], b = bodyB[c];
            SlotOf[c] = group * W3D.Lanes + lane;
            float mA = invMass[a], mB = invMass[b];
            idx[lane] = a; idx[8 + lane] = b;
            idx[16 + lane] = mA > 0f ? a : -1; idx[24 + lane] = mB > 0f ? b : -1;
            int count = pointCount[c];
            if (count > maxPoints) { maxPoints = count; }
            off[lane] = c * ps; cnt[lane] = count; msA[lane] = mA; msB[lane] = mB;
            lane++;
        }

        if (jlane > 0) { BuildJoints256(group, jlane, ref jc, ref jmA, ref jmB); }

        int anyRolling = 0, anyRestitution = 0;
        if (lane > 0)
        {
            for (int l = lane; l < W3D.Lanes; l++) { off[l] = off[0]; cnt[l] = 0; msA[l] = msA[0]; msB[l] = msB[0]; }
            var soft = ctx.Schedule != 0
                ? new SoftPair(_liteStBias, _liteStMass, _liteStImpulse, _liteDynBias, _liteDynMass, _liteDynImpulse)
                : new SoftPair(_stBias, _stMass, _stImpulse, _dynBias, _dynMass, _dynImpulse);
            var w = Wide.AsSpan(group * ContactGroupFloats, ContactGroupFloats);
            int flags = BuildContacts256(ref MemoryMarshal.GetReference(w), ref idx[0], cs.PayloadArray, Pk, ref off, ref cnt, ref msA, ref msB, lane, maxPoints, in soft);
            anyRestitution = flags & 1; anyRolling = flags >> 1;

            // The velocities come from the packed rows, which hold the start-of-step velocities here (no solve pass has run yet).
            PrepareMasses256(ref MemoryMarshal.GetReference(w), idx, Pk, maxPoints);
        }

        if (Core.Controls.Active) PrepareControlledContacts(constraints);

        if (constraints.Length > 0) { var bi = BatchInfo.AsSpan(4 * constraints[0], 4); bi[0] = group; bi[1] = lane; bi[2] = jlane; bi[3] = scalarJoints; }
        idx[W3D.MaxPointsSlot] = maxPoints; idx[W3D.AnyRollingSlot] = anyRolling; idx[W3D.AnyRestitutionSlot] = anyRestitution;
    }

    /// <summary>
    /// r4 h10: <see cref="PreparePointJoint"/> at the 8-lane width, for the <paramref name="n"/> point joints <paramref name="jc"/> of joint group <paramref name="group"/> (their slots, readback slots and
    /// body rows were written by the caller, <paramref name="jmA"/> and <paramref name="jmB"/> are their inverse masses). Every lane's inputs are loaded straight into vector lanes (the rotations by 16-byte
    /// row loads and a 4x4 transpose, the anchors, centres of mass and positions element by element), the anchor arms are rotated with the same operations as <see cref="QuaternionF.Rotate"/>, the
    /// separation is the same double difference rounded to float (one conversion per four lanes), and each of the fourteen fields is written with one 32-byte store. Lanes past the batch repeat lane 0's
    /// joint and read body 0 (their body rows are 0 and −1, so nothing is written back from them).
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    [System.Runtime.CompilerServices.SkipLocalsInit]
    private void BuildJoints256(int group, int n, ref Lane8Int jc, ref Lane8Float jmA, ref Lane8Float jmB)
    {
        for (int l = n; l < J3D.Lanes; l++) { jc[l] = jc[0]; jmA[l] = jmA[0]; jmB[l] = jmB[0]; }
        var core = Core;
        var bodies = core.Bodies;
        var cs = core.Constraints;
        ReadOnlySpan<int> jidx = WideJIdx.AsSpan(group * J3D.IdxPerGroup, J3D.IdxPerGroup);
        int a0 = jidx[0], a1 = jidx[1], a2 = jidx[2], a3 = jidx[3], a4 = jidx[4], a5 = jidx[5], a6 = jidx[6], a7 = jidx[7];
        int b0 = jidx[8], b1 = jidx[9], b2 = jidx[10], b3 = jidx[11], b4 = jidx[12], b5 = jidx[13], b6 = jidx[14], b7 = jidx[15];
        ref JointData3D jd = ref MemoryMarshal.GetArrayDataReference(Joints);
        ref JointData3D j0 = ref Unsafe.Add(ref jd, jc[0]), j1 = ref Unsafe.Add(ref jd, jc[1]), j2 = ref Unsafe.Add(ref jd, jc[2]), j3 = ref Unsafe.Add(ref jd, jc[3]);
        ref JointData3D j4 = ref Unsafe.Add(ref jd, jc[4]), j5 = ref Unsafe.Add(ref jd, jc[5]), j6 = ref Unsafe.Add(ref jd, jc[6]), j7 = ref Unsafe.Add(ref jd, jc[7]);
        ReadOnlySpan<float> lc = LocalCenter;

        // Anchor minus centre of mass, in each body's frame (the scalar path's j.LAx - lcA.X, ...).
        var vAx = Vector256.Create(j0.LAx, j1.LAx, j2.LAx, j3.LAx, j4.LAx, j5.LAx, j6.LAx, j7.LAx) - Lc8(lc, a0, a1, a2, a3, a4, a5, a6, a7, 0);
        var vAy = Vector256.Create(j0.LAy, j1.LAy, j2.LAy, j3.LAy, j4.LAy, j5.LAy, j6.LAy, j7.LAy) - Lc8(lc, a0, a1, a2, a3, a4, a5, a6, a7, 1);
        var vAz = Vector256.Create(j0.LAz, j1.LAz, j2.LAz, j3.LAz, j4.LAz, j5.LAz, j6.LAz, j7.LAz) - Lc8(lc, a0, a1, a2, a3, a4, a5, a6, a7, 2);
        var vBx = Vector256.Create(j0.LBx, j1.LBx, j2.LBx, j3.LBx, j4.LBx, j5.LBx, j6.LBx, j7.LBx) - Lc8(lc, b0, b1, b2, b3, b4, b5, b6, b7, 0);
        var vBy = Vector256.Create(j0.LBy, j1.LBy, j2.LBy, j3.LBy, j4.LBy, j5.LBy, j6.LBy, j7.LBy) - Lc8(lc, b0, b1, b2, b3, b4, b5, b6, b7, 1);
        var vBz = Vector256.Create(j0.LBz, j1.LBz, j2.LBz, j3.LBz, j4.LBz, j5.LBz, j6.LBz, j7.LBz) - Lc8(lc, b0, b1, b2, b3, b4, b5, b6, b7, 2);

        ReadOnlySpan<float> rot = bodies.Rotation;
        Rot8(rot, a0, a1, a2, a3, a4, a5, a6, a7, out var qAx, out var qAy, out var qAz, out var qAw);
        Rot8(rot, b0, b1, b2, b3, b4, b5, b6, b7, out var qBx, out var qBy, out var qBz, out var qBw);

        ref float w = ref WideJ[group * J3D.GroupFloats];
        Mem.Range(WideJ, group * J3D.GroupFloats, J3D.GroupFloats);
        RotateStore(ref w, J3D.RA, qAx, qAy, qAz, qAw, vAx, vAy, vAz);
        RotateStore(ref w, J3D.RB, qBx, qBy, qBz, qBw, vBx, vBy, vBz);

        // pB - pA in double, rounded to float (the scalar path's (float)(pB[k] - pA[k])).
        ReadOnlySpan<double> pos = bodies.Position;
        for (int k = 0; k < 3; k++)
        {
            var lo = Vector256.Create(pos[3 * b0 + k], pos[3 * b1 + k], pos[3 * b2 + k], pos[3 * b3 + k]) - Vector256.Create(pos[3 * a0 + k], pos[3 * a1 + k], pos[3 * a2 + k], pos[3 * a3 + k]);
            var hi = Vector256.Create(pos[3 * b4 + k], pos[3 * b5 + k], pos[3 * b6 + k], pos[3 * b7 + k]) - Vector256.Create(pos[3 * a4 + k], pos[3 * a5 + k], pos[3 * a6 + k], pos[3 * a7 + k]);
            St(ref w, J3D.D + k, Vector256.Create(Avx.ConvertToVector128Single(lo), Avx.ConvertToVector128Single(hi)));
        }

        St(ref w, J3D.InvMassA, Vector256.Create(jmA[0], jmA[1], jmA[2], jmA[3], jmA[4], jmA[5], jmA[6], jmA[7]));
        St(ref w, J3D.InvMassB, Vector256.Create(jmB[0], jmB[1], jmB[2], jmB[3], jmB[4], jmB[5], jmB[6], jmB[7]));

        // The warm-start impulse: the first three payload floats, from one 8-float window per lane.
        int ps = cs.PayloadStride;
        ReadOnlySpan<float> payload = cs.PayloadArray;
        Window(payload, ref MemoryMarshal.GetReference(payload), jc[0] * ps, jc[1] * ps, jc[2] * ps, jc[3] * ps, jc[4] * ps, jc[5] * ps, jc[6] * ps, jc[7] * ps, 0,
            out var i0, out var i1, out var i2, out _, out _, out _, out _, out _);
        St(ref w, J3D.Impulse, i0); St(ref w, J3D.Impulse + 1, i1); St(ref w, J3D.Impulse + 2, i2);
    }

    /// <summary>r4 h10: component <paramref name="k"/> of eight bodies' local centres of mass as one vector.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector256<float> Lc8(ReadOnlySpan<float> lc, int b0, int b1, int b2, int b3, int b4, int b5, int b6, int b7, int k) =>
        Vector256.Create(lc[3 * b0 + k], lc[3 * b1 + k], lc[3 * b2 + k], lc[3 * b3 + k], lc[3 * b4 + k], lc[3 * b5 + k], lc[3 * b6 + k], lc[3 * b7 + k]);

    /// <summary>r4 h10: eight bodies' rotations (x, y, z, w) as four vectors: one exact 16-byte load per body and a 4x4 transpose in each half, as <see cref="Lanes256.GatherRows4"/>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Rot8(ReadOnlySpan<float> rot, int b0, int b1, int b2, int b3, int b4, int b5, int b6, int b7, out Vector256<float> x, out Vector256<float> y, out Vector256<float> z, out Vector256<float> w)
    {
        Mem.Range(rot, 4 * b0, 4); Mem.Range(rot, 4 * b1, 4); Mem.Range(rot, 4 * b2, 4); Mem.Range(rot, 4 * b3, 4); Mem.Range(rot, 4 * b4, 4); Mem.Range(rot, 4 * b5, 4); Mem.Range(rot, 4 * b6, 4); Mem.Range(rot, 4 * b7, 4);
        ref float r = ref MemoryMarshal.GetReference(rot);
        var y0 = Vector256.Create(Vector128.LoadUnsafe(ref r, (nuint)(4 * b0)), Vector128.LoadUnsafe(ref r, (nuint)(4 * b4)));
        var y1 = Vector256.Create(Vector128.LoadUnsafe(ref r, (nuint)(4 * b1)), Vector128.LoadUnsafe(ref r, (nuint)(4 * b5)));
        var y2 = Vector256.Create(Vector128.LoadUnsafe(ref r, (nuint)(4 * b2)), Vector128.LoadUnsafe(ref r, (nuint)(4 * b6)));
        var y3 = Vector256.Create(Vector128.LoadUnsafe(ref r, (nuint)(4 * b3)), Vector128.LoadUnsafe(ref r, (nuint)(4 * b7)));
        var t0 = Avx.UnpackLow(y0, y1); var t1 = Avx.UnpackHigh(y0, y1); var t2 = Avx.UnpackLow(y2, y3); var t3 = Avx.UnpackHigh(y2, y3);
        x = Avx.Shuffle(t0, t2, 0x44); y = Avx.Shuffle(t0, t2, 0xEE); z = Avx.Shuffle(t1, t3, 0x44); w = Avx.Shuffle(t1, t3, 0xEE);
    }

    /// <summary>r4 h10: q v q⁻¹ per lane with the operations of <see cref="QuaternionF.Rotate"/> (t = 2 (q × v); v + t·w + q × t), stored as three fields from <paramref name="field"/>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void RotateStore(ref float g, int field, Vector256<float> qx, Vector256<float> qy, Vector256<float> qz, Vector256<float> qw, Vector256<float> vx, Vector256<float> vy, Vector256<float> vz)
    {
        var two = Vector256.Create(2f);
        var tx = (qy * vz - qz * vy) * two; var ty = (qz * vx - qx * vz) * two; var tz = (qx * vy - qy * vx) * two;
        St(ref g, field, vx + tx * qw + (qy * tz - qz * ty));
        St(ref g, field + 1, vy + ty * qw + (qz * tx - qx * tz));
        St(ref g, field + 2, vz + tz * qw + (qx * ty - qy * tx));
    }

    /// <summary>
    /// r4 h10: <see cref="ContactKernel3D.PrepareMasses{T}"/> at the 8-lane width with everything in registers: the bodies' velocity and inverse-inertia rows by the same row gathers as GatherBody, the
    /// group's fields by 32-byte loads of what <see cref="BuildContacts256"/> just stored whole (so each load forwards from one store), and per point the adjusted separation, the three effective masses and
    /// the relative normal velocity with the generic kernel's operations in the generic kernel's order (PreparePoint, EffectiveMass, Sym.Apply, V3.Cross, V3.Dot, RelativeNormalVelocity). The generic
    /// version passes about twenty-five vectors by value to each of its per-direction calls (EffectiveMassAt), which is where its store-to-load stalls were.
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    [System.Runtime.CompilerServices.SkipLocalsInit]
    private static void PrepareMasses256(ref float g, ReadOnlySpan<int> idx, ReadOnlySpan<float> pk, int maxPoints)
    {
        ref float p = ref MemoryMarshal.GetReference(pk);
        const int s = PkStride;
        int a0 = idx[0] * s, a1 = idx[1] * s, a2 = idx[2] * s, a3 = idx[3] * s, a4 = idx[4] * s, a5 = idx[5] * s, a6 = idx[6] * s, a7 = idx[7] * s;
        int b0 = idx[8] * s, b1 = idx[9] * s, b2 = idx[10] * s, b3 = idx[11] * s, b4 = idx[12] * s, b5 = idx[13] * s, b6 = idx[14] * s, b7 = idx[15] * s;
        Window(pk, ref p, a0, a1, a2, a3, a4, a5, a6, a7, PkVelocity, out var vAx, out var vAy, out var vAz, out var wAx, out var wAy, out var wAz, out _, out _);
        Window(pk, ref p, a0, a1, a2, a3, a4, a5, a6, a7, PkInertia, out var iA0, out var iA1, out var iA2, out var iA3, out var iA4, out var iA5, out _, out _);
        Window(pk, ref p, b0, b1, b2, b3, b4, b5, b6, b7, PkVelocity, out var vBx, out var vBy, out var vBz, out var wBx, out var wBy, out var wBz, out _, out _);
        Window(pk, ref p, b0, b1, b2, b3, b4, b5, b6, b7, PkInertia, out var iB0, out var iB1, out var iB2, out var iB3, out var iB4, out var iB5, out _, out _);
        var mAB = Ld(ref g, W3D.InvMassA) + Ld(ref g, W3D.InvMassB);
        Vector256<float> nx = Ld(ref g, W3D.N), ny = Ld(ref g, W3D.N + 1), nz = Ld(ref g, W3D.N + 2);
        for (int i = 0; i < maxPoints; i++)
        {
            int f = W3D.Point0 + i * W3D.PointStride;
            Vector256<float> rAx = Ld(ref g, f + W3D.ArmA), rAy = Ld(ref g, f + W3D.ArmA + 1), rAz = Ld(ref g, f + W3D.ArmA + 2);
            Vector256<float> rBx = Ld(ref g, f + W3D.ArmB), rBy = Ld(ref g, f + W3D.ArmB + 1), rBz = Ld(ref g, f + W3D.ArmB + 2);
            St(ref g, f + W3D.AdjustedSeparation, Ld(ref g, f + W3D.AdjustedSeparation) - ((rBx - rAx) * nx + (rBy - rAy) * ny + (rBz - rAz) * nz));
            St(ref g, f + W3D.NormalMass, Mass256(mAB, iA0, iA1, iA2, iA3, iA4, iA5, iB0, iB1, iB2, iB3, iB4, iB5, rAx, rAy, rAz, rBx, rBy, rBz, nx, ny, nz));
            St(ref g, f + W3D.Tangent1Mass, Mass256(mAB, iA0, iA1, iA2, iA3, iA4, iA5, iB0, iB1, iB2, iB3, iB4, iB5, rAx, rAy, rAz, rBx, rBy, rBz, Ld(ref g, W3D.T1), Ld(ref g, W3D.T1 + 1), Ld(ref g, W3D.T1 + 2)));
            St(ref g, f + W3D.Tangent2Mass, Mass256(mAB, iA0, iA1, iA2, iA3, iA4, iA5, iB0, iB1, iB2, iB3, iB4, iB5, rAx, rAy, rAz, rBx, rBy, rBz, Ld(ref g, W3D.T2), Ld(ref g, W3D.T2 + 1), Ld(ref g, W3D.T2 + 2)));
            // (vB + wB × rB) − (vA + wA × rA), dotted with the normal.
            var dvx = (vBx + (wBy * rBz - wBz * rBy)) - (vAx + (wAy * rAz - wAz * rAy));
            var dvy = (vBy + (wBz * rBx - wBx * rBz)) - (vAy + (wAz * rAx - wAx * rAz));
            var dvz = (vBz + (wBx * rBy - wBy * rBx)) - (vAz + (wAx * rAy - wAy * rAx));
            St(ref g, f + W3D.RelativeVelocity, dvx * nx + dvy * ny + dvz * nz);
        }
    }

    /// <summary>r4 h10: ContactKernel3D.EffectiveMass on Vector256: k = mA + mB + (rA × d)·IA(rA × d) + (rB × d)·IB(rB × d), and 1/k where k &gt; 0, else 0.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector256<float> Mass256(Vector256<float> mAB, Vector256<float> iA0, Vector256<float> iA1, Vector256<float> iA2, Vector256<float> iA3, Vector256<float> iA4, Vector256<float> iA5,
        Vector256<float> iB0, Vector256<float> iB1, Vector256<float> iB2, Vector256<float> iB3, Vector256<float> iB4, Vector256<float> iB5,
        Vector256<float> rAx, Vector256<float> rAy, Vector256<float> rAz, Vector256<float> rBx, Vector256<float> rBy, Vector256<float> rBz, Vector256<float> dx, Vector256<float> dy, Vector256<float> dz)
    {
        var ax = rAy * dz - rAz * dy; var ay = rAz * dx - rAx * dz; var az = rAx * dy - rAy * dx;
        var bx = rBy * dz - rBz * dy; var by = rBz * dx - rBx * dz; var bz = rBx * dy - rBy * dx;
        var k = mAB
            + (ax * (iA0 * ax + iA1 * ay + iA2 * az) + ay * (iA1 * ax + iA3 * ay + iA4 * az) + az * (iA2 * ax + iA4 * ay + iA5 * az))
            + (bx * (iB0 * bx + iB1 * by + iB2 * bz) + by * (iB1 * bx + iB3 * by + iB4 * bz) + bz * (iB2 * bx + iB4 * by + iB5 * bz));
        return Vector256.ConditionalSelect(Vector256.GreaterThan(k, Vector256<float>.Zero), Vector256.Create(1f) / k, Vector256<float>.Zero);
    }

    /// <summary>r4 h10: one 32-byte load of a group field (eight lanes).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector256<float> Ld(ref float g, int field) => Vector256.LoadUnsafe(ref g, (nuint)(field * 8));

    [InlineArray(8)]
    private struct Lane8Int { private int _e; }

    [InlineArray(8)]
    private struct Lane8Float { private float _e; }

    /// <summary>The soft coefficients of a contact against a static or kinematic body (S) and between two dynamic bodies (D), of the schedule the batch runs on.</summary>
    private readonly struct SoftPair(float sBias, float sMass, float sImp, float dBias, float dMass, float dImp)
    {
        public readonly float SBias = sBias, SMass = sMass, SImp = sImp, DBias = dBias, DMass = dMass, DImp = dImp;
    }

    /// <summary>
    /// r4 h10: the fields of one contact group from the lanes' payloads (see <see cref="PrepareBatch256"/>). <paramref name="g"/> is the group's first float, <paramref name="idx"/> its body rows (A at 0,
    /// B at 8), <paramref name="off"/> the lanes' payload offsets, <paramref name="cnt"/> their point counts, <paramref name="msA"/> and <paramref name="msB"/> their inverse masses. Returns bit 0 = some
    /// lane of the batch has restitution, bit 1 = some lane has rolling resistance. Plain refs, no span temporaries, so the prolog zeroes nothing.
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    [System.Runtime.CompilerServices.SkipLocalsInit]
    private static int BuildContacts256(ref float g, ref int idx, ReadOnlySpan<float> payload, ReadOnlySpan<float> pk, ref Lane8Int off, ref Lane8Int cnt, ref Lane8Float msA, ref Lane8Float msB, int lane, int maxPoints, in SoftPair soft)
    {
        ref float pay = ref MemoryMarshal.GetReference(payload);
        var counts = Vector256.Create(cnt[0], cnt[1], cnt[2], cnt[3], cnt[4], cnt[5], cnt[6], cnt[7]);
        var vmA = Vector256.Create(msA[0], msA[1], msA[2], msA[3], msA[4], msA[5], msA[6], msA[7]);
        var vmB = Vector256.Create(msB[0], msB[1], msB[2], msB[3], msB[4], msB[5], msB[6], msB[7]);
        int o0 = off[0], o1 = off[1], o2 = off[2], o3 = off[3], o4 = off[4], o5 = off[5], o6 = off[6], o7 = off[7];
        Vector256<float> zero = Vector256<float>.Zero, one = Vector256.Create(1f), sign = Vector256.Create(-0f);
        int real = (1 << lane) - 1;

        // Window 0: normal xyz, friction, restitution, rolling resistance, rolling impulse x y. Window 1: rolling impulse z, point 0's anchors on A and B, its separation.
        Window(payload, ref pay, o0, o1, o2, o3, o4, o5, o6, o7, 0, out var nx, out var ny, out var nz, out var friction, out var restitution, out var rolling, out var rix, out var riy);
        Window(payload, ref pay, o0, o1, o2, o3, o4, o5, o6, o7, 8, out var riz, out var p0ax, out var p0ay, out var p0az, out var p0bx, out var p0by, out var p0bz, out var p0s);
        St(ref g, W3D.N, nx); St(ref g, W3D.N + 1, ny); St(ref g, W3D.N + 2, nz);

        // Tangents (the scalar Tangents' rule and operations, per lane; negation flips the sign bit as the scalar unary minus does).
        var pickX = Vector256.GreaterThanOrEqual(Vector256.Abs(nx), Vector256.Create(0.57735f));
        var tx = Vector256.ConditionalSelect(pickX, ny, zero);
        var ty = Vector256.ConditionalSelect(pickX, Vector256.Xor(nx, sign), nz);
        var tz = Vector256.ConditionalSelect(pickX, zero, Vector256.Xor(ny, sign));
        var len = Vector256.Sqrt(tx * tx + ty * ty + tz * tz);
        var inv = one / len;
        var pos = Vector256.GreaterThan(len, zero);
        var t1x = Vector256.ConditionalSelect(pos, tx * inv, one);
        var t1y = Vector256.ConditionalSelect(pos, ty * inv, zero);
        var t1z = Vector256.ConditionalSelect(pos, tz * inv, zero);
        St(ref g, W3D.T1, t1x); St(ref g, W3D.T1 + 1, t1y); St(ref g, W3D.T1 + 2, t1z);
        St(ref g, W3D.T2, ny * t1z - nz * t1y); St(ref g, W3D.T2 + 1, nz * t1x - nx * t1z); St(ref g, W3D.T2 + 2, nx * t1y - ny * t1x);

        St(ref g, W3D.Friction, friction); St(ref g, W3D.Restitution, restitution); St(ref g, W3D.Rolling, rolling);
        St(ref g, W3D.InvMassA, vmA); St(ref g, W3D.InvMassB, vmB);
        var stat = Vector256.Equals(vmA, zero) | Vector256.Equals(vmB, zero);
        St(ref g, W3D.BiasRate, Vector256.ConditionalSelect(stat, Vector256.Create(soft.SBias), Vector256.Create(soft.DBias)));
        St(ref g, W3D.MassScale, Vector256.ConditionalSelect(stat, Vector256.Create(soft.SMass), Vector256.Create(soft.DMass)));
        St(ref g, W3D.ImpulseScale, Vector256.ConditionalSelect(stat, Vector256.Create(soft.SImp), Vector256.Create(soft.DImp)));
        St(ref g, W3D.RollImpulse, rix); St(ref g, W3D.RollImpulse + 1, riy); St(ref g, W3D.RollImpulse + 2, riz);
        int flags = ((int)(~Vector256.Equals(restitution, zero)).ExtractMostSignificantBits() & real) != 0 ? 1 : 0;
        if (((int)Vector256.GreaterThan(rolling, zero).ExtractMostSignificantBits() & real) != 0)
        {
            // RollingMass for all lanes: the inverse of IA + IB from the bodies' packed inertia rows (the same floats as WInvI), zero where the sum is singular. Only the lanes with rolling resistance read it.
            flags |= 2;
            ref float pkr = ref MemoryMarshal.GetReference(pk);
            const int s = Engine3D.PkStride, k = Engine3D.PkInertia;
            Window(pk, ref pkr, idx * s + k, Unsafe.Add(ref idx, 1) * s + k, Unsafe.Add(ref idx, 2) * s + k, Unsafe.Add(ref idx, 3) * s + k, Unsafe.Add(ref idx, 4) * s + k, Unsafe.Add(ref idx, 5) * s + k,
                Unsafe.Add(ref idx, 6) * s + k, Unsafe.Add(ref idx, 7) * s + k, 0, out var a0, out var a1, out var a2, out var a3, out var a4, out var a5, out _, out _);
            Window(pk, ref pkr, Unsafe.Add(ref idx, 8) * s + k, Unsafe.Add(ref idx, 9) * s + k, Unsafe.Add(ref idx, 10) * s + k, Unsafe.Add(ref idx, 11) * s + k, Unsafe.Add(ref idx, 12) * s + k,
                Unsafe.Add(ref idx, 13) * s + k, Unsafe.Add(ref idx, 14) * s + k, Unsafe.Add(ref idx, 15) * s + k, 0, out var b0, out var b1, out var b2, out var b3, out var b4, out var b5, out _, out _);
            Vector256<float> ia = a0 + b0, ib = a1 + b1, ic = a2 + b2, id = a3 + b3, ie = a4 + b4, iff = a5 + b5;
            var c00 = id * iff - ie * ie; var c01 = ic * ie - ib * iff; var c02 = ib * ie - ic * id;
            var det = ia * c00 + ib * c01 + ic * c02;
            var ok = Vector256.GreaterThan(det, zero) & Vector256.LessThan(Vector256.Abs(det), Vector256.Create(float.PositiveInfinity));
            var dinv = one / det;
            St(ref g, W3D.RollMass, Vector256.ConditionalSelect(ok, c00 * dinv, zero));
            St(ref g, W3D.RollMass + 1, Vector256.ConditionalSelect(ok, c01 * dinv, zero));
            St(ref g, W3D.RollMass + 2, Vector256.ConditionalSelect(ok, c02 * dinv, zero));
            St(ref g, W3D.RollMass + 3, Vector256.ConditionalSelect(ok, (ia * iff - ic * ic) * dinv, zero));
            St(ref g, W3D.RollMass + 4, Vector256.ConditionalSelect(ok, (ib * ic - ia * ie) * dinv, zero));
            St(ref g, W3D.RollMass + 5, Vector256.ConditionalSelect(ok, (ia * id - ib * ib) * dinv, zero));
        }

        // Points: point i's payload floats start at 9 + 12i (anchor A, anchor B, separation, normal, tangent 1 and tangent 2 impulse); points 0 and 2 start one float into a window, 1 and 3 five.
        Window(payload, ref pay, o0, o1, o2, o3, o4, o5, o6, o7, 16, out var p0n, out var p0t1, out var p0t2, out _, out _, out var p1ax, out var p1ay, out var p1az);
        StorePoint256(ref g, 0, p0ax, p0ay, p0az, p0bx, p0by, p0bz, p0s, p0n, p0t1, p0t2, counts);
        if (maxPoints >= 2)
        {
            Window(payload, ref pay, o0, o1, o2, o3, o4, o5, o6, o7, 24, out var p1bx, out var p1by, out var p1bz, out var p1s, out var p1n, out var p1t1, out var p1t2, out _);
            StorePoint256(ref g, 1, p1ax, p1ay, p1az, p1bx, p1by, p1bz, p1s, p1n, p1t1, p1t2, counts);
            if (maxPoints >= 3)
            {
                Window(payload, ref pay, o0, o1, o2, o3, o4, o5, o6, o7, 32, out _, out var p2ax, out var p2ay, out var p2az, out var p2bx, out var p2by, out var p2bz, out var p2s);
                Window(payload, ref pay, o0, o1, o2, o3, o4, o5, o6, o7, 40, out var p2n, out var p2t1, out var p2t2, out _, out _, out var p3ax, out var p3ay, out var p3az);
                StorePoint256(ref g, 2, p2ax, p2ay, p2az, p2bx, p2by, p2bz, p2s, p2n, p2t1, p2t2, counts);
                if (maxPoints >= 4)
                {
                    Window(payload, ref pay, o0, o1, o2, o3, o4, o5, o6, o7, 48, out var p3bx, out var p3by, out var p3bz, out var p3s, out var p3n, out var p3t1, out var p3t2, out _);
                    StorePoint256(ref g, 3, p3ax, p3ay, p3az, p3bx, p3by, p3bz, p3s, p3n, p3t1, p3t2, counts);
                }
            }
        }

        // Points past the group's largest count: mask clear and impulse zero in every lane (the rolling total reads the impulse).
        for (int i = maxPoints; i < W3D.MaxPoints; i++)
        {
            int f = W3D.Point0 + i * W3D.PointStride;
            St(ref g, f + W3D.Mask, zero); St(ref g, f + W3D.NormalImpulse, zero);
        }

        return flags;
    }

    /// <summary>
    /// r4 h10: eight floats at <c>offset_i + skip</c> of <paramref name="source"/> for the eight lanes i, transposed: <paramref name="f0"/> holds the first float of every lane's window, and so on. One 32-byte
    /// load per lane and the 8x8 transpose of <see cref="Lanes256.GatherRows(ReadOnlySpan{float}, ReadOnlySpan{int})"/>; pure data movement.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Window(ReadOnlySpan<float> source, ref float src, int o0, int o1, int o2, int o3, int o4, int o5, int o6, int o7, int skip,
        out Vector256<float> f0, out Vector256<float> f1, out Vector256<float> f2, out Vector256<float> f3, out Vector256<float> f4, out Vector256<float> f5, out Vector256<float> f6, out Vector256<float> f7)
    {
        // Every window lies inside its lane's payload (or packed body row); the Checked build verifies it.
        Mem.Range(source, o0 + skip, 8); Mem.Range(source, o1 + skip, 8); Mem.Range(source, o2 + skip, 8); Mem.Range(source, o3 + skip, 8);
        Mem.Range(source, o4 + skip, 8); Mem.Range(source, o5 + skip, 8); Mem.Range(source, o6 + skip, 8); Mem.Range(source, o7 + skip, 8);
        var r0 = Vector256.LoadUnsafe(ref src, (nuint)(o0 + skip)); var r1 = Vector256.LoadUnsafe(ref src, (nuint)(o1 + skip));
        var r2 = Vector256.LoadUnsafe(ref src, (nuint)(o2 + skip)); var r3 = Vector256.LoadUnsafe(ref src, (nuint)(o3 + skip));
        var r4 = Vector256.LoadUnsafe(ref src, (nuint)(o4 + skip)); var r5 = Vector256.LoadUnsafe(ref src, (nuint)(o5 + skip));
        var r6 = Vector256.LoadUnsafe(ref src, (nuint)(o6 + skip)); var r7 = Vector256.LoadUnsafe(ref src, (nuint)(o7 + skip));
        var t0 = Avx.UnpackLow(r0, r1); var t1 = Avx.UnpackHigh(r0, r1); var t2 = Avx.UnpackLow(r2, r3); var t3 = Avx.UnpackHigh(r2, r3);
        var t4 = Avx.UnpackLow(r4, r5); var t5 = Avx.UnpackHigh(r4, r5); var t6 = Avx.UnpackLow(r6, r7); var t7 = Avx.UnpackHigh(r6, r7);
        var u0 = Avx.Shuffle(t0, t2, 0x44); var u1 = Avx.Shuffle(t0, t2, 0xEE); var u2 = Avx.Shuffle(t1, t3, 0x44); var u3 = Avx.Shuffle(t1, t3, 0xEE);
        var u4 = Avx.Shuffle(t4, t6, 0x44); var u5 = Avx.Shuffle(t4, t6, 0xEE); var u6 = Avx.Shuffle(t5, t7, 0x44); var u7 = Avx.Shuffle(t5, t7, 0xEE);
        f0 = Avx.Permute2x128(u0, u4, 0x20); f1 = Avx.Permute2x128(u1, u5, 0x20); f2 = Avx.Permute2x128(u2, u6, 0x20); f3 = Avx.Permute2x128(u3, u7, 0x20);
        f4 = Avx.Permute2x128(u0, u4, 0x31); f5 = Avx.Permute2x128(u1, u5, 0x31); f6 = Avx.Permute2x128(u2, u6, 0x31); f7 = Avx.Permute2x128(u3, u7, 0x31);
    }

    /// <summary>r4 h10: one 32-byte store of a group field (eight lanes).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void St(ref float g, int field, Vector256<float> v) => v.StoreUnsafe(ref g, (nuint)(field * 8));

    /// <summary>r4 h10: point <paramref name="i"/> of all eight lanes from its gathered payload rows; the mask is set where the lane's point count exceeds <paramref name="i"/>, and the normal impulse is zero where it is clear.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void StorePoint256(ref float g, int i, Vector256<float> ax, Vector256<float> ay, Vector256<float> az, Vector256<float> bx, Vector256<float> by, Vector256<float> bz, Vector256<float> sep,
        Vector256<float> nImp, Vector256<float> t1Imp, Vector256<float> t2Imp, Vector256<int> counts)
    {
        int f = W3D.Point0 + i * W3D.PointStride;
        var mask = Vector256.GreaterThan(counts, Vector256.Create(i)).AsSingle();
        St(ref g, f + W3D.ArmA, ax); St(ref g, f + W3D.ArmA + 1, ay); St(ref g, f + W3D.ArmA + 2, az);
        St(ref g, f + W3D.ArmB, bx); St(ref g, f + W3D.ArmB + 1, by); St(ref g, f + W3D.ArmB + 2, bz);
        St(ref g, f + W3D.AdjustedSeparation, sep);
        St(ref g, f + W3D.NormalImpulse, Vector256.ConditionalSelect(mask, nImp, Vector256<float>.Zero));
        St(ref g, f + W3D.Tangent1Impulse, t1Imp); St(ref g, f + W3D.Tangent2Impulse, t2Imp);
        St(ref g, f + W3D.MaxNormalImpulse, Vector256<float>.Zero);
        St(ref g, f + W3D.Mask, mask);
    }

    /// <summary>The joint types whose three-row point block (always their last rows) is solved by <see cref="JointKernel3D"/>; their other rows stay in <see cref="SolveJoint"/>.</summary>
    private static bool WidePointBlock(byte type) =>
        type == (byte)JointType3D.Point || type == (byte)JointType3D.Fixed || type == (byte)JointType3D.Hinge || type == (byte)JointType3D.Cone || type == (byte)JointType3D.SwingTwist;

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
    private static void Put3(Span<float> w, int field, int lane, Vector3F v) { Mem.At(w, field * 8 + lane) = v.X; Mem.At(w, (field + 1) * 8 + lane) = v.Y; Mem.At(w, (field + 2) * 8 + lane) = v.Z; }

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

            var w = Wide.AsSpan((slot >> 3) * ContactGroupFloats, ContactGroupFloats);
            int lane = slot & 7;
            var pl = cs.Payload(c);
            int count = cs.PointCount[c];
            for (int i = 0; i < count; i++)
            {
                int f = W3D.Point0 + i * W3D.PointStride;
                Mem.At(pl, ManifoldLayout.NormalImpulse(i)) = Mem.At(w, (f + W3D.NormalImpulse) * 8 + lane);
                Mem.At(pl, ManifoldLayout.Tangent1Impulse(i)) = Mem.At(w, (f + W3D.Tangent1Impulse) * 8 + lane);
                Mem.At(pl, ManifoldLayout.Tangent2Impulse(i)) = Mem.At(w, (f + W3D.Tangent2Impulse) * 8 + lane);
                Mem.At(pl, ManifoldLayout.MaxNormalImpulse(i)) = Mem.At(w, (f + W3D.MaxNormalImpulse) * 8 + lane);
            }

            for (int k = 0; k < 3; k++) { Mem.At(pl, ManifoldLayout.RollingImpulse + k) = Mem.At(w, (W3D.RollImpulse + k) * 8 + lane); }
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
        var jp = ctx.Schedule != 0 ? _liteJointParams : _jointParams;
        var cons = Core.Constraints;
        if (cons.Color[constraints[0]] == ConstraintStore.Overflow)
        {
            for (int i = 0; i < constraints.Length; i++)
            {
                int c = constraints[i];
                int slot = SlotOf[c];
                if (cons.KindOf(c) == ConstraintKind.Joint)
                {
                    if (Opt.DetailedInspection && phase != SolvePhase.Restitution) { Core.JointReadback.ClearRows(c); }
                    // A joint's own rows first, then its point block, as the joint's rows have always been ordered.
                    if ((Joints[c].Type != (byte)JointType3D.Point || JointIsLocked(c))) { SolveJoint(c, phase, in ctx); }
                    if (slot >= 0) { JointKernel3D.Run<Lanes1>(this, slot >> 3, slot & 7, 1, phase, in jp); }
                }
                else if (slot >= 0) { ContactKernel3D.Run<Lanes1>(this, slot >> 3, slot & 7, 1, phase, in kp); }
            }

            return;
        }

        // Contacts were packed into the first lanes of the batch's group in list order, point joints into the first lanes of its joint group; both counts were worked out when the batch was prepared.
        // Joints that are not point blocks (which share no dynamic body with anything else in the batch) solve one by one.
        var info = BatchInfo.AsSpan(4 * constraints[0], 4);
        int group = info[0], count = info[1], jcount = info[2];
        if (Opt.DetailedInspection && phase != SolvePhase.Restitution && (jcount != 0 || info[3] != 0)) { foreach (int c in constraints) { if (cons.KindOf(c) == ConstraintKind.Joint) { Core.JointReadback.ClearRows(c); } } }
        if (info[3] != 0)
        {
            for (int i = 0; i < constraints.Length; i++)
            {
                int c = constraints[i];
                if (cons.KindOf(c) == ConstraintKind.Joint && (Joints[c].Type != (byte)JointType3D.Point || JointIsLocked(c))) { SolveJoint(c, phase, in ctx); }
            }
        }

        if (jcount > 0)
        {
            int jgroup = group;
            if (KernelWidth == 1) { JointKernel3D.Run<Lanes1>(this, jgroup, 0, jcount, phase, in jp); }
            else if (KernelWidth == 4) { JointKernel3D.Run<Lanes128>(this, jgroup, 0, jcount, phase, in jp); }
            else { JointKernel3D.Run<Lanes256>(this, jgroup, 0, jcount, phase, in jp); }
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
        if (RailHandles.Count != 0) { RailSubstep(list, ctx.SubstepDt); }
        if (Core.Controls.Active || Opt.MaxLinearSpeed != 400f) foreach (int b in list) LimitSolveVelocity(b);
    }

    /// <summary>Displacement, rotations, world inverse inertia and the packed rows of the bodies of the list, eight at a time (see <see cref="IntegrateKernel3D"/>).</summary>
    public void IntegratePositions(ReadOnlySpan<int> list, in StepContext ctx)
    {
        if (Core.Controls.Active || Opt.MaxLinearSpeed != 400f) foreach (int b in list) LimitSolveVelocity(b);
        if (KernelWidth == 1) { IntegrateKernel3D.Positions<Lanes1>(this, list, ctx.SubstepDt); }
        else if (KernelWidth == 4) { IntegrateKernel3D.Positions<Lanes128>(this, list, ctx.SubstepDt); }
        else { IntegrateKernel3D.Positions<Lanes256>(this, list, ctx.SubstepDt); }
        if (Core.Controls.HasLocks) foreach (int b in list) { ProjectInertia(b); PackInertia(b); }
    }

    // ------------------------------------------------------------------ finalize: forces and continuous collision

    public void FinalizeBodies(ReadOnlySpan<int> list, in StepContext ctx)
    {
        if (Core.Controls.Active || Opt.MaxLinearSpeed != 400f) foreach (int b in list) LimitSolveVelocity(b);
        var bodies = Core.Bodies;
        // r4 h06: the solve kept every awake body's velocity in its packed row 0 (see Pk); it goes back to the body store here, before anything after the solve reads it.
        Span<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        ReadOnlySpan<float> pk = Pk;
        foreach (int b in list)
        {
            int b3 = 3 * b, o = PkStride * b;
            Mem.At(lin, b3) = Mem.At(pk, o); Mem.At(lin, b3 + 1) = Mem.At(pk, o + 1); Mem.At(lin, b3 + 2) = Mem.At(pk, o + 2);
            Mem.At(ang, b3) = Mem.At(pk, o + 3); Mem.At(ang, b3 + 1) = Mem.At(pk, o + 4); Mem.At(ang, b3 + 2) = Mem.At(pk, o + 5);
        }

        foreach (int b in list)
        {
            if (bodies.TypeOf(b) != BodyType.Dynamic) { continue; }
            if (Opt.DetailedInspection) { Core.BodyReadback.Inputs(b, bodies.InverseMass[b], GravityScale[b], new(Force[3 * b], Force[3 * b + 1], Force[3 * b + 2]), new(Torque[3 * b], Torque[3 * b + 1], Torque[3 * b + 2]), Core.IntegratedTimeOfBody(b)); }
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
            if (core.Surfaces.IsPresent && !SurfaceContacts.SweepApproaches(core, MoverShape, userId, Motion)) return true;
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
            float t, approach; Vector3F outwardOther;
            if (other.IsConvex)
            {
                if (!Gjk3D.ShapeCast(other.Convex, Shape, rel, -motion, out var hit) || hit.StartsOverlapping) { return true; }
                t = hit.Fraction; approach = -Vector3F.Dot(motion, hit.Normal); outwardOther = rs.Rotate(hit.Normal);
            }
            else if (other.Compound != null)
            {
                // The compound refuses a sweep that leaves its coordinate range (a fast body far from the compound), so the sweep is cut to the range first, as the query path does.
                if (!ClipToReach(rel.Translation, motion, out var co, out var cd, out float t0, out float t1)) { return true; }
                if (!other.Compound.ShapeCast(Shape, new RigidTransformF(rel.Rotation, co), cd, out var hit) || hit.StartsOverlapping) { return true; }
                t = t0 + (hit.Fraction * (t1 - t0)); approach = Vector3F.Dot(motion, hit.Normal); outwardOther = -rs.Rotate(hit.Normal);
            }
            else
            {
                if (!other.Soup!.ShapeCast(Shape, rel, motion, out var hit, CastIds()) || hit.StartsOverlapping) { return true; }
                t = hit.Fraction; approach = -Vector3F.Dot(motion, hit.Normal); outwardOther = rs.Rotate(hit.Normal);
            }

            if (core.Surfaces.IsPresent && !SurfaceContacts.SweepNormalAllowed(core, MoverShape, userId, outwardOther)) return true;
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
    /// speculative contact. The rotation is not part of these casts (it counts towards deciding that the body moved far enough to need one); the corners of boxes and hulls are swept through the turn
    /// separately, see <see cref="CornerSweep"/>.
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
        var rel = qStart.Conjugated().Times(qEnd);
        float spin = 2f * float.Sqrt(rel.X * rel.X + rel.Y * rel.Y + rel.Z * rel.Z) * MaxExtent[b];
        float len = float.Sqrt(m.X * m.X + m.Y * m.Y + m.Z * m.Z);
        if (!float.IsFinite(len + spin) || !(len + spin > 0.5f * MinExtent[b]) || !(len > 0f)) { return; } // non-finite motion is reported by the bounds check, not cast

        Core.BodyReadback.CcdState[b] = (byte)(1 | ((Core.Bodies.Flags[b] & (byte)BodyBits.Bullet) != 0 ? 4 : 0));

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

        // A turning body also sweeps the corners of its boxes and hulls, which the linear casts above (shapes held at their start orientation) cannot see. A vehicle's chassis is left out: its wheels
        // are suspension casts from the chassis pose, and clamping the pose back by the hull's corners can leave a wheel inside the geometry (the suspension then pushes it out hard).
        if (spin > Opt.SpeculativeDistance && VehicleOfBody(b) < 0) { best = CornerSweep(b, cx, cy, cz, m, qStart, qEnd, spin, best); }
        if (best >= 1f) { return; }
        Core.BodyReadback.CcdState[b] |= 2;
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
