using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// Layout of one group of up to eight contact constraints in <see cref="Engine3D.Wide"/>: <see cref="Fields"/> fields of eight floats (one per lane), so every field loads as one vector.
/// Constants first (normal and tangents, material, inverse masses, soft-constraint coefficients, rolling mass), the rolling impulse, then four points of <see cref="PointStride"/> fields each.
/// </summary>
internal static class W3D
{
    public const int N = 0, T1 = 3, T2 = 6, Friction = 9, Restitution = 10, Rolling = 11, InvMassA = 12, InvMassB = 13, BiasRate = 14, MassScale = 15, ImpulseScale = 16;
    public const int RollMass = 17, RollImpulse = 23;
    public const int Point0 = 26, PointStride = 17;

    // Offsets inside a point.
    public const int ArmA = 0, ArmB = 3, AdjustedSeparation = 6, NormalMass = 7, Tangent1Mass = 8, Tangent2Mass = 9, RelativeVelocity = 10;
    public const int NormalImpulse = 11, Tangent1Impulse = 12, Tangent2Impulse = 13, MaxNormalImpulse = 14, Mask = 15;

    public const int MaxPoints = 4;
    public const int Fields = Point0 + MaxPoints * PointStride;
    public const int Lanes = 8;
    /// <summary>Scratch after the fields: gathered body state (38 rows of eight) and the velocities on their way out (12 rows of eight), so the kernel keeps no large locals.</summary>
    public const int StageOffset = Fields * Lanes, StageFloats = 0, OutOffset = StageOffset + StageFloats, OutFloats = 12 * 8;
    public const int GroupFloats = OutOffset + OutFloats;

    /// <summary>Integers per group in <see cref="Engine3D.WideIdx"/>: body rows of A and B, then the same rows for bodies the kernel writes (or -1), eight lanes each, then the largest point count in the group (points past it are skipped for every lane), whether any lane has rolling resistance, and whether any lane has restitution.</summary>
    public const int IdxPerGroup = 35, MaxPointsSlot = 32, AnyRollingSlot = 33, AnyRestitutionSlot = 34;
}

/// <summary>
/// The 3D contact solver kernel: soft step (Catto, "Solver2D" and the soft-step talk), the same arithmetic as a scalar contact solver, written once over <see cref="ILane{T}"/>.
/// A batch of up to eight constraints of one color is gathered into lanes, solved with the same operations in the same order at every width, and scattered back.
/// The constraints of a batch touch disjoint dynamic bodies, so lanes never depend on each other; static and kinematic bodies are only read. A lane with fewer than four points masks the
/// unused points out with selects, so it computes exactly what a loop over its own points would.
/// </summary>
// The kernels' locals are large vector structs (a batch's bodies and points, several kilobytes of frame); clearing them at every call cost about a twelfth of a step. Every local is assigned before it is read
// (the compiler checks that for ordinary locals), and the one stack buffer, the gather offsets, is written for every lane before it is read, so nothing depends on what the stack held.
[System.Runtime.CompilerServices.SkipLocalsInit]
internal static class ContactKernel3D
{
    private struct V3<T> where T : struct, ILane<T>
    {
        public T X, Y, Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public V3(T x, T y, T z) { X = x; Y = y; Z = z; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static V3<T> operator +(V3<T> a, V3<T> b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static V3<T> operator -(V3<T> a, V3<T> b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static V3<T> operator *(V3<T> a, T s) => new(a.X * s, a.Y * s, a.Z * s);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static V3<T> Neg(V3<T> a) => new(T.Negate(a.X), T.Negate(a.Y), T.Negate(a.Z));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static V3<T> Cross(V3<T> a, V3<T> b) => new(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static T Dot(V3<T> a, V3<T> b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static V3<T> Select(T mask, V3<T> whenSet, V3<T> whenClear) => new(T.Select(mask, whenSet.X, whenClear.X), T.Select(mask, whenSet.Y, whenClear.Y), T.Select(mask, whenSet.Z, whenClear.Z));
    }

    /// <summary>A symmetric 3×3 tensor stored as xx xy xz yy yz zz.</summary>
    private struct Sym<T> where T : struct, ILane<T>
    {
        public T M0, M1, M2, M3, M4, M5;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public V3<T> Apply(V3<T> v) => new(M0 * v.X + M1 * v.Y + M2 * v.Z, M1 * v.X + M3 * v.Y + M4 * v.Z, M2 * v.X + M4 * v.Y + M5 * v.Z);
    }

    private struct BodyLanes<T> where T : struct, ILane<T>
    {
        public V3<T> V, W, Dp;
        public T Qx, Qy, Qz, Qw;
        public Sym<T> I;
        public T InvMass;
    }

    private struct PointLanes<T> where T : struct, ILane<T>
    {
        public V3<T> RA, RB;
        public T AdjSep, NMass, T1Mass, T2Mass, RelVel, NImp, T1Imp, T2Imp, MaxN, Mask;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T Field<T>(ReadOnlySpan<float> wide, int f, int lane0) where T : struct, ILane<T> => T.LoadAt(wide, f * 8 + lane0);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static V3<T> Vec<T>(ReadOnlySpan<float> wide, int f, int lane0) where T : struct, ILane<T> => new(Field<T>(wide, f, lane0), Field<T>(wide, f + 1, lane0), Field<T>(wide, f + 2, lane0));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T Stage<T>(ReadOnlySpan<float> stage, int f) where T : struct, ILane<T> => T.Load(stage.Slice(f * T.Count, T.Count));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static V3<T> StageVec<T>(ReadOnlySpan<float> stage, int f) where T : struct, ILane<T> => new(Stage<T>(stage, f), Stage<T>(stage, f + 1), Stage<T>(stage, f + 2));

    /// <summary>
    /// The state of the bodies on one side of a batch, one body per lane, by whole-row loads and a transpose: velocities from the velocity arrays (three floats a body, read through an eight-float window whose
    /// first three are the ones used), displacement, accumulated rotation and world inverse inertia from the packed rows (<see cref="Engine3D.Pk"/>).
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void GatherBody<T>(out BodyLanes<T> r, ReadOnlySpan<float> lin, ReadOnlySpan<float> ang, ReadOnlySpan<float> pk, ReadOnlySpan<int> bodyIds, Span<int> offs, T invMass) where T : struct, ILane<T>
    {
        int width = T.Count;
        for (int l = 0; l < width; l++) { offs[l] = 3 * bodyIds[l]; }
        var v = T.GatherRows(lin, offs);
        var w = T.GatherRows(ang, offs);
        for (int l = 0; l < width; l++) { offs[l] = 16 * bodyIds[l]; }
        var g0 = T.GatherRows(pk, offs);
        for (int l = 0; l < width; l++) { offs[l] += 8; }
        var g1 = T.GatherRows(pk, offs);
        r = new BodyLanes<T>
        {
            V = new V3<T>(v.R0, v.R1, v.R2), W = new V3<T>(w.R0, w.R1, w.R2), Dp = new V3<T>(g0.R0, g0.R1, g0.R2),
            Qx = g0.R3, Qy = g0.R4, Qz = g0.R5, Qw = g0.R6,
            I = new Sym<T> { M0 = g0.R7, M1 = g1.R0, M2 = g1.R1, M3 = g1.R2, M4 = g1.R3, M5 = g1.R4 },
            InvMass = invMass,
        };
    }

    /// <summary>q v q⁻¹ for a unit quaternion: v + w·t + u × t with t = 2 u × v.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static V3<T> Rotate<T>(T x, T y, T z, T w, V3<T> v, T two) where T : struct, ILane<T>
    {
        T tx = two * (y * v.Z - z * v.Y), ty = two * (z * v.X - x * v.Z), tz = two * (x * v.Y - y * v.X);
        return new V3<T>(v.X + w * tx + (y * tz - z * ty), v.Y + w * ty + (z * tx - x * tz), v.Z + w * tz + (x * ty - y * tx));
    }

    /// <summary>Applies an impulse <paramref name="p"/> at the anchors to both bodies in the lanes where <paramref name="mask"/> is set; the other lanes keep their bits.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ApplyImpulse<T>(ref BodyLanes<T> a, ref BodyLanes<T> b, V3<T> rA, V3<T> rB, V3<T> p, T mask) where T : struct, ILane<T>
    {
        var vA = a.V - p * a.InvMass;
        var wA = a.W - a.I.Apply(V3<T>.Cross(rA, p));
        var vB = b.V + p * b.InvMass;
        var wB = b.W + b.I.Apply(V3<T>.Cross(rB, p));
        a.V = V3<T>.Select(mask, vA, a.V); a.W = V3<T>.Select(mask, wA, a.W);
        b.V = V3<T>.Select(mask, vB, b.V); b.W = V3<T>.Select(mask, wB, b.W);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static V3<T> RelativeVelocity<T>(in BodyLanes<T> a, in BodyLanes<T> b, V3<T> rA, V3<T> rB) where T : struct, ILane<T> =>
        (b.V + V3<T>.Cross(b.W, rB)) - (a.V + V3<T>.Cross(a.W, rA));

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void LoadPoint<T>(out PointLanes<T> p, ReadOnlySpan<float> wide, int i, int lane0) where T : struct, ILane<T>
    {
        int f = W3D.Point0 + i * W3D.PointStride;
        p = new PointLanes<T>
        {
            RA = Vec<T>(wide, f + W3D.ArmA, lane0), RB = Vec<T>(wide, f + W3D.ArmB, lane0),
            AdjSep = Field<T>(wide, f + W3D.AdjustedSeparation, lane0), NMass = Field<T>(wide, f + W3D.NormalMass, lane0),
            T1Mass = Field<T>(wide, f + W3D.Tangent1Mass, lane0), T2Mass = Field<T>(wide, f + W3D.Tangent2Mass, lane0), RelVel = Field<T>(wide, f + W3D.RelativeVelocity, lane0),
            NImp = Field<T>(wide, f + W3D.NormalImpulse, lane0), T1Imp = Field<T>(wide, f + W3D.Tangent1Impulse, lane0), T2Imp = Field<T>(wide, f + W3D.Tangent2Impulse, lane0),
            MaxN = Field<T>(wide, f + W3D.MaxNormalImpulse, lane0), Mask = Field<T>(wide, f + W3D.Mask, lane0),
        };
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T Clamp0<T>(T x) where T : struct, ILane<T> => T.Select(T.GreaterThan(x, T.Zero), x, T.Zero);

    /// <summary>
    /// The per-point quantities of a freshly prepared group, for all eight lanes at once: the separation adjusted for the anchors' offset along the normal, the effective masses along the normal and both
    /// tangents, and the relative normal velocity. The bodies come from the same gathers the solver uses; the separation as made sits in the adjusted-separation row. Every lane does what one scalar lane does, in the same order.
    /// </summary>
    public static void PrepareMasses<T>(Span<float> wide, ReadOnlySpan<int> idx, ReadOnlySpan<float> lin, ReadOnlySpan<float> ang, ReadOnlySpan<float> pk, int maxPoints) where T : struct, ILane<T>
    {
        int width = T.Count;
        T zero = T.Zero, one = T.Broadcast(1f);
        Span<int> offs = stackalloc int[8];
        for (int l0 = 0; l0 < W3D.Lanes; l0 += width)
        {
            // The bodies' world inverse inertias and velocities, gathered as the solver gathers them.
            GatherBody<T>(out var ba, lin, ang, pk, idx.Slice(l0, width), offs, zero);
            GatherBody<T>(out var bb, lin, ang, pk, idx.Slice(8 + l0, width), offs, zero);
            var iA = ba.I; var iB = bb.I;
            V3<T> vA = ba.V, wA = ba.W, vB = bb.V, wB = bb.W;
            T mA = Field<T>(wide, W3D.InvMassA, l0), mB = Field<T>(wide, W3D.InvMassB, l0);
            V3<T> n = Vec<T>(wide, W3D.N, l0), t1 = Vec<T>(wide, W3D.T1, l0), t2 = Vec<T>(wide, W3D.T2, l0);
            for (int i = 0; i < maxPoints; i++) { PreparePoint(wide, W3D.Point0 + i * W3D.PointStride, l0, mA, mB, iA, iB, vA, wA, vB, wB, n, t1, t2, zero, one); }
        }
    }

    /// <summary>One point's adjusted separation, effective masses and relative normal velocity for the lanes of a group (see <see cref="PrepareMasses{T}"/>).</summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void PreparePoint<T>(Span<float> wide, int f, int l0, T mA, T mB, Sym<T> iA, Sym<T> iB, V3<T> vA, V3<T> wA, V3<T> vB, V3<T> wB, V3<T> n, V3<T> t1, V3<T> t2, T zero, T one) where T : struct, ILane<T>
    {
        V3<T> rA = Vec<T>(wide, f + W3D.ArmA, l0), rB = Vec<T>(wide, f + W3D.ArmB, l0);
        T sep = Field<T>(wide, f + W3D.AdjustedSeparation, l0);
        (sep - V3<T>.Dot(rB - rA, n)).StoreAt(wide, (f + W3D.AdjustedSeparation) * 8 + l0);
        // One method per direction keeps each under the JIT's inlining budget, so the lane operations inline instead of becoming calls.
        EffectiveMassAt(wide, (f + W3D.NormalMass) * 8 + l0, mA, mB, iA, iB, rA, rB, n, zero, one);
        EffectiveMassAt(wide, (f + W3D.Tangent1Mass) * 8 + l0, mA, mB, iA, iB, rA, rB, t1, zero, one);
        EffectiveMassAt(wide, (f + W3D.Tangent2Mass) * 8 + l0, mA, mB, iA, iB, rA, rB, t2, zero, one);
        RelativeNormalVelocity(wide, (f + W3D.RelativeVelocity) * 8 + l0, vA, wA, vB, wB, rA, rB, n);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void EffectiveMassAt<T>(Span<float> wide, int at, T mA, T mB, Sym<T> iA, Sym<T> iB, V3<T> rA, V3<T> rB, V3<T> d, T zero, T one) where T : struct, ILane<T> =>
        EffectiveMass(mA, mB, iA, iB, rA, rB, d, zero, one).StoreAt(wide, at);

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void RelativeNormalVelocity<T>(Span<float> wide, int at, V3<T> vA, V3<T> wA, V3<T> vB, V3<T> wB, V3<T> rA, V3<T> rB, V3<T> n) where T : struct, ILane<T>
    {
        var dv = (vB + V3<T>.Cross(wB, rB)) - (vA + V3<T>.Cross(wA, rA));
        V3<T>.Dot(dv, n).StoreAt(wide, at);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T EffectiveMass<T>(T mA, T mB, Sym<T> iA, Sym<T> iB, V3<T> rA, V3<T> rB, V3<T> d, T zero, T one) where T : struct, ILane<T>
    {
        var rnA = V3<T>.Cross(rA, d);
        var rnB = V3<T>.Cross(rB, d);
        T k = mA + mB + V3<T>.Dot(rnA, iA.Apply(rnA)) + V3<T>.Dot(rnB, iB.Apply(rnB));
        return T.Select(T.GreaterThan(k, zero), one / k, zero);
    }

    /// <summary>
    /// Runs lanes <paramref name="lane0"/> to <paramref name="lane0"/> + <paramref name="n"/> - 1 of a group, <c>T.Count</c> at a time. Lanes past the end of the batch compute on whatever the group holds and are never written back.
    /// </summary>
    public static void Run<T>(Engine3D e, int group, int lane0, int n, SolvePhase phase, in KernelParams kp) where T : struct, ILane<T>
    {
        int width = T.Count;
        var idx = e.WideIdx.AsSpan(group * W3D.IdxPerGroup, W3D.IdxPerGroup);
        // With no restitution in any lane the pass changes nothing (every lane's mask is clear and the selects keep the old bits), so the gather and scatter are skipped.
        if (phase == SolvePhase.Restitution && idx[W3D.AnyRestitutionSlot] == 0) { return; }
        var wide = e.Wide.AsSpan(group * W3D.GroupFloats, W3D.GroupFloats);
        var bodies = e.Core.Bodies;
        ReadOnlySpan<float> lin = bodies.LinearVelocityPadded, ang = bodies.AngularVelocityPadded;
        ReadOnlySpan<float> pk = e.Pk;
        Span<int> offs = stackalloc int[8];
        Span<float> linW = bodies.LinearVelocity, angW = bodies.AngularVelocity;
        Span<float> os = e.Wide.AsSpan(group * W3D.GroupFloats + W3D.OutOffset, W3D.OutFloats);
        T two = T.Broadcast(2f), zero = T.Zero;

        for (int l0 = lane0; l0 < lane0 + n; l0 += width)
        {
            GatherBody<T>(out var a, lin, ang, pk, idx.Slice(l0, width), offs, Field<T>(wide, W3D.InvMassA, l0));
            GatherBody<T>(out var b2, lin, ang, pk, idx.Slice(8 + l0, width), offs, Field<T>(wide, W3D.InvMassB, l0));
            var nrm = Vec<T>(wide, W3D.N, l0);
            var t1 = Vec<T>(wide, W3D.T1, l0);
            var t2 = Vec<T>(wide, W3D.T2, l0);
            V3<T> roll = Vec<T>(wide, W3D.RollImpulse, l0);
            int maxPts = idx[W3D.MaxPointsSlot];
            LoadPoint<T>(out var pts0, wide, 0, l0);
            PointLanes<T> pts1 = default, pts2 = default, pts3 = default;
            if (maxPts > 1) { LoadPoint<T>(out pts1, wide, 1, l0); }
            if (maxPts > 2) { LoadPoint<T>(out pts2, wide, 2, l0); }
            if (maxPts > 3) { LoadPoint<T>(out pts3, wide, 3, l0); }

            if (phase == SolvePhase.WarmStart)
            {
                WarmPoint(ref a, ref b2, in pts0, nrm, t1, t2);
                if (maxPts > 1) { WarmPoint(ref a, ref b2, in pts1, nrm, t1, t2); }
                if (maxPts > 2) { WarmPoint(ref a, ref b2, in pts2, nrm, t1, t2); }
                if (maxPts > 3) { WarmPoint(ref a, ref b2, in pts3, nrm, t1, t2); }
                ApplyRoll(ref a, ref b2, roll);
            }
            else if (phase == SolvePhase.Restitution)
            {
                T e0 = Field<T>(wide, W3D.Restitution, l0);
                T lane = Lane.Or(T.GreaterThan(e0, zero), T.GreaterThan(zero, e0));
                T negThr = T.Broadcast(-kp.RestitutionThreshold);
                Restitute(ref a, ref b2, ref pts0, nrm, e0, lane, negThr, zero);
                if (maxPts > 1) { Restitute(ref a, ref b2, ref pts1, nrm, e0, lane, negThr, zero); }
                if (maxPts > 2) { Restitute(ref a, ref b2, ref pts2, nrm, e0, lane, negThr, zero); }
                if (maxPts > 3) { Restitute(ref a, ref b2, ref pts3, nrm, e0, lane, negThr, zero); }
            }
            else
            {
                bool useBias = phase == SolvePhase.Solve;
                T biasRate = Field<T>(wide, W3D.BiasRate, l0), massScale = Field<T>(wide, W3D.MassScale, l0), impulseScale = Field<T>(wide, W3D.ImpulseScale, l0);
                T friction = Field<T>(wide, W3D.Friction, l0), rolling = Field<T>(wide, W3D.Rolling, l0);

                // Rolling resistance first, then friction, then non-penetration last: the last constraint solved wins the argument, and keeping bodies apart matters most.
                // A group with no rolling resistance in any lane would leave every lane unchanged (the selects keep the old bits), so it skips the work.
                if (idx[W3D.AnyRollingSlot] != 0) { RollingResistance(ref a, ref b2, nrm, ref roll, wide, l0, rolling, in pts0, in pts1, in pts2, in pts3); }

                FrictionPoint(ref a, ref b2, ref pts0, t1, t2, friction);
                if (maxPts > 1) { FrictionPoint(ref a, ref b2, ref pts1, t1, t2, friction); }
                if (maxPts > 2) { FrictionPoint(ref a, ref b2, ref pts2, t1, t2, friction); }
                if (maxPts > 3) { FrictionPoint(ref a, ref b2, ref pts3, t1, t2, friction); }

                T invH = T.Broadcast(kp.InverseSubstepDt), slop = T.Broadcast(kp.LinearSlop), negPush = T.Broadcast(-kp.PushOutVelocity);
                NormalPoint(ref a, ref b2, ref pts0, nrm, useBias, biasRate, massScale, impulseScale, invH, slop, negPush, zero, two);
                if (maxPts > 1) { NormalPoint(ref a, ref b2, ref pts1, nrm, useBias, biasRate, massScale, impulseScale, invH, slop, negPush, zero, two); }
                if (maxPts > 2) { NormalPoint(ref a, ref b2, ref pts2, nrm, useBias, biasRate, massScale, impulseScale, invH, slop, negPush, zero, two); }
                if (maxPts > 3) { NormalPoint(ref a, ref b2, ref pts3, nrm, useBias, biasRate, massScale, impulseScale, invH, slop, negPush, zero, two); }
            }

            // Velocities back out.
            a.V.X.StoreAt(os, 0 * width); a.V.Y.StoreAt(os, 1 * width); a.V.Z.StoreAt(os, 2 * width);
            a.W.X.StoreAt(os, 3 * width); a.W.Y.StoreAt(os, 4 * width); a.W.Z.StoreAt(os, 5 * width);
            b2.V.X.StoreAt(os, 6 * width); b2.V.Y.StoreAt(os, 7 * width); b2.V.Z.StoreAt(os, 8 * width);
            b2.W.X.StoreAt(os, 9 * width); b2.W.Y.StoreAt(os, 10 * width); b2.W.Z.StoreAt(os, 11 * width);
            for (int l = 0; l < width && l0 + l < lane0 + n; l++)
            {
                // Scatter offsets are body indices of dynamic bodies, or -1 for a body that keeps its velocity.
                int ia = idx[16 + l0 + l];
                if (ia >= 0) { ScatterBody(linW, angW, os, 3 * ia, l, width); }
                int ib = idx[24 + l0 + l];
                if (ib >= 0) { ScatterBody(linW, angW, os, 3 * ib, l + 6 * width, width); }
            }

            if (phase == SolvePhase.WarmStart) { continue; }
            // Impulses back to the group. Restitution leaves the friction and rolling impulses alone.
            StorePoint(wide, 0, l0, in pts0, phase);
            if (maxPts > 1) { StorePoint(wide, 1, l0, in pts1, phase); }
            if (maxPts > 2) { StorePoint(wide, 2, l0, in pts2, phase); }
            if (maxPts > 3) { StorePoint(wide, 3, l0, in pts3, phase); }
            if (phase != SolvePhase.Restitution)
            {
                roll.X.StoreAt(wide, (W3D.RollImpulse + 0) * 8 + l0); roll.Y.StoreAt(wide, (W3D.RollImpulse + 1) * 8 + l0); roll.Z.StoreAt(wide, (W3D.RollImpulse + 2) * 8 + l0);
            }
        }
    }

    /// <summary>Applies the rolling impulse carried over from the last step to both bodies' angular velocity.</summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void ApplyRoll<T>(ref BodyLanes<T> a, ref BodyLanes<T> b2, V3<T> roll) where T : struct, ILane<T>
    {
        a.W = a.W - a.I.Apply(roll);
        b2.W = b2.W + b2.I.Apply(roll);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void RollingResistance<T>(ref BodyLanes<T> a, ref BodyLanes<T> b2, V3<T> nrm, ref V3<T> roll, ReadOnlySpan<float> wide, int l0, T rolling, in PointLanes<T> p0, in PointLanes<T> p1, in PointLanes<T> p2, in PointLanes<T> p3) where T : struct, ILane<T>
    {
        T zero = T.Zero;
        T total = T.Zero + p0.NImp + p1.NImp + p2.NImp + p3.NImp;
        T rollMask = T.GreaterThan(rolling, zero);
        T maxRoll = rolling * total;
        var dw = b2.W - a.W;
        dw = dw - nrm * V3<T>.Dot(dw, nrm);
        var rm = new Sym<T> { M0 = Field<T>(wide, W3D.RollMass, l0), M1 = Field<T>(wide, W3D.RollMass + 1, l0), M2 = Field<T>(wide, W3D.RollMass + 2, l0), M3 = Field<T>(wide, W3D.RollMass + 3, l0), M4 = Field<T>(wide, W3D.RollMass + 4, l0), M5 = Field<T>(wide, W3D.RollMass + 5, l0) };
        var lam = V3<T>.Neg(rm.Apply(dw));
        lam = lam - nrm * V3<T>.Dot(lam, nrm);
        var newR = roll + lam;
        T len2 = V3<T>.Dot(newR, newR);
        T over = T.GreaterThan(len2, maxRoll * maxRoll);
        var scaled = newR * (maxRoll / T.Sqrt(len2));
        newR = V3<T>.Select(over, scaled, newR);
        lam = newR - roll;
        var aW = a.W - a.I.Apply(lam);
        var bW = b2.W + b2.I.Apply(lam);
        roll = V3<T>.Select(rollMask, newR, roll);
        a.W = V3<T>.Select(rollMask, aW, a.W);
        b2.W = V3<T>.Select(rollMask, bW, b2.W);
    }

    /// <summary>Writes the velocities of one lane from the staged rows (<paramref name="first"/> is the lane's slot in the linear rows; the angular rows follow three rows later) to a body's three-float slots. The offset is a live dynamic body's, set when the batch was prepared.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ScatterBody(Span<float> lin, Span<float> ang, ReadOnlySpan<float> os, int at, int first, int width)
    {
        // at is a live dynamic body's slot (3 floats a body, set when the batch was prepared) and first + 5 * width + 1 stays inside the group's output rows.
        Mem.Range(lin, at, 3); Mem.Range(ang, at, 3); Mem.Range(os, first, 5 * width + 1);
        Mem.At(lin, at) = Mem.At(os, first); Mem.At(lin, at + 1) = Mem.At(os, first + width); Mem.At(lin, at + 2) = Mem.At(os, first + 2 * width);
        Mem.At(ang, at) = Mem.At(os, first + 3 * width); Mem.At(ang, at + 1) = Mem.At(os, first + 4 * width); Mem.At(ang, at + 2) = Mem.At(os, first + 5 * width);
    }

    private static void StorePoint<T>(Span<float> wide, int i, int l0, in PointLanes<T> p, SolvePhase phase) where T : struct, ILane<T>
    {
        int f = W3D.Point0 + i * W3D.PointStride;
        int w = T.Count;
        p.NImp.StoreAt(wide, (f + W3D.NormalImpulse) * 8 + l0); p.MaxN.StoreAt(wide, (f + W3D.MaxNormalImpulse) * 8 + l0);
        if (phase != SolvePhase.Restitution)
        {
            p.T1Imp.StoreAt(wide, (f + W3D.Tangent1Impulse) * 8 + l0); p.T2Imp.StoreAt(wide, (f + W3D.Tangent2Impulse) * 8 + l0);
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void WarmPoint<T>(ref BodyLanes<T> a, ref BodyLanes<T> b, in PointLanes<T> p, V3<T> n, V3<T> t1, V3<T> t2) where T : struct, ILane<T>
    {
        var imp = n * p.NImp + t1 * p.T1Imp + t2 * p.T2Imp;
        ApplyImpulse(ref a, ref b, p.RA, p.RB, imp, p.Mask);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void Restitute<T>(ref BodyLanes<T> a, ref BodyLanes<T> b, ref PointLanes<T> p, V3<T> n, T e, T laneMask, T negThreshold, T zero) where T : struct, ILane<T>
    {
        var dv = RelativeVelocity(in a, in b, p.RA, p.RB);
        T vn = V3<T>.Dot(dv, n);
        T lambda = T.Negate(p.NMass) * (vn + e * p.RelVel);
        T notFast = T.Select(T.GreaterThan(p.RelVel, negThreshold), zero, p.Mask); // relVel <= -threshold, and the point exists
        T mask = Lane.And(Lane.And(Lane.And(notFast, T.GreaterThan(p.MaxN, zero)), laneMask), p.Mask);
        T nw = Clamp0(p.NImp + lambda);
        lambda = nw - p.NImp;
        p.NImp = T.Select(mask, nw, p.NImp);
        p.MaxN = T.Select(Lane.And(mask, T.GreaterThan(nw, p.MaxN)), nw, p.MaxN);
        ApplyImpulse(ref a, ref b, p.RA, p.RB, n * lambda, mask);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void FrictionPoint<T>(ref BodyLanes<T> a, ref BodyLanes<T> b, ref PointLanes<T> p, V3<T> t1, V3<T> t2, T friction) where T : struct, ILane<T>
    {
        var imp = FrictionImpulse(in a, in b, ref p, t1, t2, friction);
        ApplyImpulse(ref a, ref b, p.RA, p.RB, imp, p.Mask);
    }

    /// <summary>Accumulates the two tangent impulses of a point, clamped to the friction cone, and returns the change as an impulse.</summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static V3<T> FrictionImpulse<T>(in BodyLanes<T> a, in BodyLanes<T> b, ref PointLanes<T> p, V3<T> t1, V3<T> t2, T friction) where T : struct, ILane<T>
    {
        var dv = RelativeVelocity(in a, in b, p.RA, p.RB);
        T l1 = p.T1Imp, l2 = p.T2Imp;
        T n1 = l1 - p.T1Mass * V3<T>.Dot(dv, t1);
        T n2 = l2 - p.T2Mass * V3<T>.Dot(dv, t2);
        T maxF = friction * p.NImp;
        T m2 = n1 * n1 + n2 * n2;
        T clamp = T.GreaterThan(m2, maxF * maxF);
        T k = maxF / T.Sqrt(m2);
        n1 = T.Select(clamp, n1 * k, n1);
        n2 = T.Select(clamp, n2 * k, n2);
        p.T1Imp = T.Select(p.Mask, n1, l1); p.T2Imp = T.Select(p.Mask, n2, l2);
        return t1 * (n1 - l1) + t2 * (n2 - l2);
    }

    // The runtime inlines at most a few hundred calls into one method and then leaves every remaining vector operation as a call that returns its result through memory, so the larger pieces of the
    // kernel are methods of their own.

    /// <summary>Current separation: the anchors move with their bodies' accumulated translation and rotation.</summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static T Separation<T>(in BodyLanes<T> a, in BodyLanes<T> b, in PointLanes<T> p, V3<T> n, T two) where T : struct, ILane<T>
    {
        var sepVec = (b.Dp - a.Dp) + (Rotate(b.Qx, b.Qy, b.Qz, b.Qw, p.RB, two) - Rotate(a.Qx, a.Qy, a.Qz, a.Qw, p.RA, two));
        return V3<T>.Dot(sepVec, n) + p.AdjSep;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void NormalPoint<T>(ref BodyLanes<T> a, ref BodyLanes<T> b, ref PointLanes<T> p, V3<T> n, bool useBias, T biasRate, T massScale, T impulseScale, T invH, T slop, T negPush, T zero, T two) where T : struct, ILane<T>
    {
        T lambda = NormalImpulse(in a, in b, ref p, n, useBias, biasRate, massScale, impulseScale, invH, slop, negPush, zero, two);
        ApplyImpulse(ref a, ref b, p.RA, p.RB, n * lambda, p.Mask);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static T NormalImpulse<T>(in BodyLanes<T> a, in BodyLanes<T> b, ref PointLanes<T> p, V3<T> n, bool useBias, T biasRate, T massScale, T impulseScale, T invH, T slop, T negPush, T zero, T two) where T : struct, ILane<T>
    {
        T sep = Separation(in a, in b, in p, n, two);
        T apart = T.GreaterThan(sep, zero);
        T one = T.Broadcast(1f);
        T bias, ms, isc;
        T biasApart = sep * invH;
        if (useBias)
        {
            T x = sep + slop;
            x = T.Select(T.GreaterThan(x, zero), zero, x);
            T soft = biasRate * x;
            soft = T.Select(T.GreaterThan(negPush, soft), negPush, soft);
            bias = T.Select(apart, biasApart, soft);
            ms = T.Select(apart, one, massScale);
            isc = T.Select(apart, zero, impulseScale);
        }
        else
        {
            bias = T.Select(apart, biasApart, zero);
            ms = one;
            isc = zero;
        }

        var dv = RelativeVelocity(in a, in b, p.RA, p.RB);
        T vn = V3<T>.Dot(dv, n);
        T old = p.NImp;
        T lambda = T.Negate(p.NMass) * ms * (vn + bias) - isc * old;
        T nw = Clamp0(old + lambda);
        lambda = nw - old;
        p.NImp = T.Select(p.Mask, nw, old);
        p.MaxN = T.Select(Lane.And(p.Mask, T.GreaterThan(nw, p.MaxN)), nw, p.MaxN);
        return lambda;
    }
}
