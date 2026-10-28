using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OnePhysics.Core;
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
    public const int StageOffset = Fields * Lanes, StageFloats = 38 * 8, OutOffset = StageOffset + StageFloats, OutFloats = 12 * 8;
    public const int GroupFloats = OutOffset + OutFloats;

    /// <summary>Integers per group in <see cref="Engine3D.WideIdx"/>: body rows of A and B, then the same rows for bodies the kernel writes (or -1), eight lanes each, then the largest point count in the group (points past it are skipped for every lane).</summary>
    public const int IdxPerGroup = 33, MaxPointsSlot = 32;
}

/// <summary>
/// The 3D contact solver kernel: soft step (Catto, "Solver2D" and the soft-step talk), the same arithmetic as a scalar contact solver, written once over <see cref="ILane{T}"/>.
/// A batch of up to eight constraints of one color is gathered into lanes, solved with the same operations in the same order at every width, and scattered back.
/// The constraints of a batch touch disjoint dynamic bodies, so lanes never depend on each other; static and kinematic bodies are only read. A lane with fewer than four points masks the
/// unused points out with selects, so it computes exactly what a loop over its own points would.
/// </summary>
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
    private static T Field<T>(ReadOnlySpan<float> wide, int f, int lane0) where T : struct, ILane<T> => T.Load(wide.Slice(f * 8 + lane0, T.Count));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static V3<T> Vec<T>(ReadOnlySpan<float> wide, int f, int lane0) where T : struct, ILane<T> => new(Field<T>(wide, f, lane0), Field<T>(wide, f + 1, lane0), Field<T>(wide, f + 2, lane0));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T Stage<T>(ReadOnlySpan<float> stage, int f) where T : struct, ILane<T> => T.Load(stage.Slice(f * T.Count, T.Count));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static V3<T> StageVec<T>(ReadOnlySpan<float> stage, int f) where T : struct, ILane<T> => new(Stage<T>(stage, f), Stage<T>(stage, f + 1), Stage<T>(stage, f + 2));

    private static BodyLanes<T> LoadBody<T>(ReadOnlySpan<float> stage, int f0, T invMass) where T : struct, ILane<T> => new()
    {
        V = StageVec<T>(stage, f0), W = StageVec<T>(stage, f0 + 3), Dp = StageVec<T>(stage, f0 + 6),
        Qx = Stage<T>(stage, f0 + 9), Qy = Stage<T>(stage, f0 + 10), Qz = Stage<T>(stage, f0 + 11), Qw = Stage<T>(stage, f0 + 12),
        I = new Sym<T> { M0 = Stage<T>(stage, f0 + 13), M1 = Stage<T>(stage, f0 + 14), M2 = Stage<T>(stage, f0 + 15), M3 = Stage<T>(stage, f0 + 16), M4 = Stage<T>(stage, f0 + 17), M5 = Stage<T>(stage, f0 + 18) },
        InvMass = invMass,
    };

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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static PointLanes<T> LoadPoint<T>(ReadOnlySpan<float> wide, int i, int lane0) where T : struct, ILane<T>
    {
        int f = W3D.Point0 + i * W3D.PointStride;
        return new PointLanes<T>
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
    /// Runs lanes <paramref name="lane0"/> to <paramref name="lane0"/> + <paramref name="n"/> - 1 of a group, <c>T.Count</c> at a time. Lanes past the end of the batch compute on whatever the group holds and are never written back.
    /// </summary>
    public static void Run<T>(Engine3D e, int group, int lane0, int n, SolvePhase phase, in KernelParams kp) where T : struct, ILane<T>
    {
        int width = T.Count;
        var wide = e.Wide.AsSpan(group * W3D.GroupFloats, W3D.GroupFloats);
        var idx = e.WideIdx.AsSpan(group * W3D.IdxPerGroup, W3D.IdxPerGroup);
        var bodies = e.Core.Bodies;
        ReadOnlySpan<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        ReadOnlySpan<float> dp = e.Dp, dq = e.Dq, winv = e.WInvI;
        Span<float> linW = bodies.LinearVelocity, angW = bodies.AngularVelocity;
        Span<float> stage = e.Wide.AsSpan(group * W3D.GroupFloats + W3D.StageOffset, W3D.StageFloats);
        Span<float> os = e.Wide.AsSpan(group * W3D.GroupFloats + W3D.OutOffset, W3D.OutFloats);
        T two = T.Broadcast(2f), zero = T.Zero;

        for (int l0 = lane0; l0 < lane0 + n; l0 += width)
        {
            // Gather the bodies of each lane into field-major rows (a transpose done with scalar loads).
            for (int l = 0; l < width; l++)
            {
                for (int side = 0; side < 2; side++)
                {
                    int b = idx[8 * side + l0 + l];
                    int f = 19 * side;
                    stage[(f + 0) * width + l] = lin[3 * b]; stage[(f + 1) * width + l] = lin[3 * b + 1]; stage[(f + 2) * width + l] = lin[3 * b + 2];
                    stage[(f + 3) * width + l] = ang[3 * b]; stage[(f + 4) * width + l] = ang[3 * b + 1]; stage[(f + 5) * width + l] = ang[3 * b + 2];
                    stage[(f + 6) * width + l] = dp[3 * b]; stage[(f + 7) * width + l] = dp[3 * b + 1]; stage[(f + 8) * width + l] = dp[3 * b + 2];
                    stage[(f + 9) * width + l] = dq[4 * b]; stage[(f + 10) * width + l] = dq[4 * b + 1]; stage[(f + 11) * width + l] = dq[4 * b + 2]; stage[(f + 12) * width + l] = dq[4 * b + 3];
                    for (int k = 0; k < 6; k++) { stage[(f + 13 + k) * width + l] = winv[6 * b + k]; }
                }
            }

            var a = LoadBody<T>(stage, 0, Field<T>(wide, W3D.InvMassA, l0));
            var b2 = LoadBody<T>(stage, 19, Field<T>(wide, W3D.InvMassB, l0));
            var nrm = Vec<T>(wide, W3D.N, l0);
            var t1 = Vec<T>(wide, W3D.T1, l0);
            var t2 = Vec<T>(wide, W3D.T2, l0);
            V3<T> roll = Vec<T>(wide, W3D.RollImpulse, l0);
            int maxPts = idx[W3D.MaxPointsSlot];
            var pts0 = LoadPoint<T>(wide, 0, l0); var pts1 = maxPts > 1 ? LoadPoint<T>(wide, 1, l0) : default; var pts2 = maxPts > 2 ? LoadPoint<T>(wide, 2, l0) : default; var pts3 = maxPts > 3 ? LoadPoint<T>(wide, 3, l0) : default;

            if (phase == SolvePhase.WarmStart)
            {
                WarmPoint(ref a, ref b2, in pts0, nrm, t1, t2);
                if (maxPts > 1) { WarmPoint(ref a, ref b2, in pts1, nrm, t1, t2); }
                if (maxPts > 2) { WarmPoint(ref a, ref b2, in pts2, nrm, t1, t2); }
                if (maxPts > 3) { WarmPoint(ref a, ref b2, in pts3, nrm, t1, t2); }
                a.W = a.W - a.I.Apply(roll);
                b2.W = b2.W + b2.I.Apply(roll);
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
                T rollMask = T.GreaterThan(rolling, zero);
                T total = zero + pts0.NImp + pts1.NImp + pts2.NImp + pts3.NImp;
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
            a.V.X.Store(os.Slice(0 * width, width)); a.V.Y.Store(os.Slice(1 * width, width)); a.V.Z.Store(os.Slice(2 * width, width));
            a.W.X.Store(os.Slice(3 * width, width)); a.W.Y.Store(os.Slice(4 * width, width)); a.W.Z.Store(os.Slice(5 * width, width));
            b2.V.X.Store(os.Slice(6 * width, width)); b2.V.Y.Store(os.Slice(7 * width, width)); b2.V.Z.Store(os.Slice(8 * width, width));
            b2.W.X.Store(os.Slice(9 * width, width)); b2.W.Y.Store(os.Slice(10 * width, width)); b2.W.Z.Store(os.Slice(11 * width, width));
            for (int l = 0; l < width && l0 + l < lane0 + n; l++)
            {
                // Scatter offsets are body indices of dynamic bodies, or -1 for a body that keeps its velocity.
                int ia = idx[16 + l0 + l];
                if (ia >= 0)
                {
                    linW[3 * ia] = os[l]; linW[3 * ia + 1] = os[width + l]; linW[3 * ia + 2] = os[2 * width + l];
                    angW[3 * ia] = os[3 * width + l]; angW[3 * ia + 1] = os[4 * width + l]; angW[3 * ia + 2] = os[5 * width + l];
                }

                int ib = idx[24 + l0 + l];
                if (ib >= 0)
                {
                    linW[3 * ib] = os[6 * width + l]; linW[3 * ib + 1] = os[7 * width + l]; linW[3 * ib + 2] = os[8 * width + l];
                    angW[3 * ib] = os[9 * width + l]; angW[3 * ib + 1] = os[10 * width + l]; angW[3 * ib + 2] = os[11 * width + l];
                }
            }

            if (phase == SolvePhase.WarmStart) { continue; }
            // Impulses back to the group. Restitution leaves the friction and rolling impulses alone.
            StorePoint(wide, 0, l0, in pts0, phase);
            if (maxPts > 1) { StorePoint(wide, 1, l0, in pts1, phase); }
            if (maxPts > 2) { StorePoint(wide, 2, l0, in pts2, phase); }
            if (maxPts > 3) { StorePoint(wide, 3, l0, in pts3, phase); }
            if (phase != SolvePhase.Restitution)
            {
                roll.X.Store(wide.Slice((W3D.RollImpulse + 0) * 8 + l0, width)); roll.Y.Store(wide.Slice((W3D.RollImpulse + 1) * 8 + l0, width)); roll.Z.Store(wide.Slice((W3D.RollImpulse + 2) * 8 + l0, width));
            }
        }
    }

    private static void StorePoint<T>(Span<float> wide, int i, int l0, in PointLanes<T> p, SolvePhase phase) where T : struct, ILane<T>
    {
        int f = W3D.Point0 + i * W3D.PointStride;
        int w = T.Count;
        p.NImp.Store(wide.Slice((f + W3D.NormalImpulse) * 8 + l0, w)); p.MaxN.Store(wide.Slice((f + W3D.MaxNormalImpulse) * 8 + l0, w));
        if (phase != SolvePhase.Restitution)
        {
            p.T1Imp.Store(wide.Slice((f + W3D.Tangent1Impulse) * 8 + l0, w)); p.T2Imp.Store(wide.Slice((f + W3D.Tangent2Impulse) * 8 + l0, w));
        }
    }

    private static void WarmPoint<T>(ref BodyLanes<T> a, ref BodyLanes<T> b, in PointLanes<T> p, V3<T> n, V3<T> t1, V3<T> t2) where T : struct, ILane<T>
    {
        var imp = n * p.NImp + t1 * p.T1Imp + t2 * p.T2Imp;
        ApplyImpulse(ref a, ref b, p.RA, p.RB, imp, p.Mask);
    }

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

    private static void FrictionPoint<T>(ref BodyLanes<T> a, ref BodyLanes<T> b, ref PointLanes<T> p, V3<T> t1, V3<T> t2, T friction) where T : struct, ILane<T>
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
        var imp = t1 * (n1 - l1) + t2 * (n2 - l2);
        ApplyImpulse(ref a, ref b, p.RA, p.RB, imp, p.Mask);
    }

    private static void NormalPoint<T>(ref BodyLanes<T> a, ref BodyLanes<T> b, ref PointLanes<T> p, V3<T> n, bool useBias, T biasRate, T massScale, T impulseScale, T invH, T slop, T negPush, T zero, T two) where T : struct, ILane<T>
    {
        // Current separation: the anchors move with their bodies' accumulated translation and rotation.
        var sepVec = (b.Dp - a.Dp) + (Rotate(b.Qx, b.Qy, b.Qz, b.Qw, p.RB, two) - Rotate(a.Qx, a.Qy, a.Qz, a.Qw, p.RA, two));
        T sep = V3<T>.Dot(sepVec, n) + p.AdjSep;
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
        ApplyImpulse(ref a, ref b, p.RA, p.RB, n * lambda, p.Mask);
    }
}
