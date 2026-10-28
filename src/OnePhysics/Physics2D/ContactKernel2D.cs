using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>Per-step constants of the contact kernel.</summary>
internal readonly struct KernelParams
{
    public readonly float InverseSubstepDt;
    public readonly float PushOutVelocity;
    public readonly float LinearSlop;
    public readonly float RestitutionThreshold;

    public KernelParams(float inverseSubstepDt, float pushOutVelocity, float linearSlop, float restitutionThreshold)
    {
        InverseSubstepDt = inverseSubstepDt;
        PushOutVelocity = pushOutVelocity;
        LinearSlop = linearSlop;
        RestitutionThreshold = restitutionThreshold;
    }
}

/// <summary>
/// Layout of one group of up to eight contact constraints in <see cref="Engine2D.Wide"/>: <see cref="Fields"/> fields of eight floats (one per lane) each, so every field loads as one vector.
/// Constants first (normal, material, inverse masses, soft-constraint coefficients), then two points of eight fields, then the impulses that change while solving.
/// </summary>
internal static class W2D
{
    public const int NormalX = 0, NormalY = 1, Friction = 2, Restitution = 3, Rolling = 4, RollingMass = 5;
    public const int InvMassA = 6, InvInertiaA = 7, InvMassB = 8, InvInertiaB = 9;
    public const int BiasRate = 10, MassScale = 11, ImpulseScale = 12;
    public const int Point0 = 13, PointStride = 8;

    // Offsets inside a point.
    public const int AnchorAX = 0, AnchorAY = 1, AnchorBX = 2, AnchorBY = 3, AdjustedSeparation = 4, NormalMass = 5, TangentMass = 6, RelativeVelocity = 7;

    // Impulses: rolling, then normal, tangent and largest normal impulse of each point.
    public const int Roll = 29, Impulse0 = 30, Impulse1 = 33;
    public const int Fields = 36;
    public const int Lanes = 8;
    public const int GroupFloats = Fields * Lanes;
    /// <summary>Integers per group in <see cref="Engine2D.WideIdx"/>: gather offsets of A and B, then scatter offsets of A and B, eight lanes each.</summary>
    public const int IdxPerGroup = 32;
}

/// <summary>Layout of one body's solver state in <see cref="Engine2D.BodyState"/>: velocity and the motion accumulated over the step, eight floats so a body is half a cache line.</summary>
internal static class BS2D
{
    public const int Stride = 8;
    public const int Vx = 0, Vy = 1, W = 2, Dpx = 4, Dpy = 5, DqC = 6, DqS = 7;
}

/// <summary>
/// The 2D contact solver kernel: soft step (Catto, "Solver2D" and the soft-step talk): substeps with soft contact constraints, a bias-free relax pass, and restitution once at the end.
/// It is written once over <see cref="ILane{T}"/>. A batch of up to eight constraints of one color is gathered into lanes, solved with the same operations in the same order at
/// every width, and scattered back. The constraints of a batch touch disjoint dynamic bodies, so lanes never depend on each other, and static and kinematic bodies are only read.
/// </summary>
internal static class ContactKernel2D
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T W<T>(ReadOnlySpan<float> wide, int f, int lane0) where T : struct, ILane<T> => T.LoadAt(wide, f * 8 + lane0);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T Clamp<T>(T x, T lo, T hi) where T : struct, ILane<T> => Lane.Max(lo, Lane.Min(hi, x));

    // The solver works on plain local vectors rather than structs of vectors: the JIT keeps a struct with many fields in memory, so the same arithmetic runs several times slower.
    // The two points of a manifold go through the same code in a loop of two; points are independent of each other except through the body velocities, which stay in locals.

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Apply<T>(ref T vAx, ref T vAy, ref T wA, ref T vBx, ref T vBy, ref T wB, T mA, T iA, T mB, T iB, T rAx, T rAy, T rBx, T rBy, T px, T py) where T : struct, ILane<T>
    {
        vAx = vAx - mA * px; vAy = vAy - mA * py;
        wA = wA - iA * (rAx * py - rAy * px);
        vBx = vBx + mB * px; vBy = vBy + mB * py;
        wB = wB + iB * (rBx * py - rBy * px);
    }

    /// <summary>
    /// Solves lanes <paramref name="lane0"/>..<paramref name="lane0"/>+<paramref name="lanes"/>-1 of one group in <paramref name="phase"/>. Body velocities are gathered from <see cref="Engine2D.BodyState"/>
    /// and scattered back; static and kinematic bodies (and unused lanes) have no scatter offset, so they are only read.
    /// Each phase has its own method: one method holding all three needs a large frame that is cleared on every call, which costs more than a group's arithmetic.
    /// </summary>
    public static void Run<T>(Engine2D e, int group, int lane0, int lanes, SolvePhase phase, in KernelParams kp) where T : struct, ILane<T>
    {
        var wide = e.Wide.AsSpan(group * W2D.GroupFloats, W2D.GroupFloats);
        var idx = e.WideIdx.AsSpan(group * W2D.IdxPerGroup, W2D.IdxPerGroup);
        if (phase == SolvePhase.WarmStart) { RunWarm<T>(e.BodyState, wide, idx, lane0, lanes); }
        else if (phase == SolvePhase.Restitution) { RunRestitution<T>(e.BodyState, wide, idx, lane0, lanes, in kp); }
        else { RunSolve<T>(e.BodyState, wide, idx, lane0, lanes, phase == SolvePhase.Solve, in kp); }
    }

    private static void RunWarm<T>(Span<float> state, Span<float> wide, ReadOnlySpan<int> idx, int lane0, int lanes) where T : struct, ILane<T>
    {
        int width = T.Count;
        for (int l0 = lane0; l0 < lane0 + lanes; l0 += width)
        {
            var ra = T.GatherRows(state, idx.Slice(l0, width));
            var rb = T.GatherRows(state, idx.Slice(8 + l0, width));
            T vAx = ra.R0, vAy = ra.R1, wA = ra.R2, vBx = rb.R0, vBy = rb.R1, wB = rb.R2;
            T nx = W<T>(wide, W2D.NormalX, l0), ny = W<T>(wide, W2D.NormalY, l0);
            T mA = W<T>(wide, W2D.InvMassA, l0), iA = W<T>(wide, W2D.InvInertiaA, l0), mB = W<T>(wide, W2D.InvMassB, l0), iB = W<T>(wide, W2D.InvInertiaB, l0);
            T tx = ny, ty = Lane.Neg(nx); // the tangent is the normal turned clockwise
            T rollImp = W<T>(wide, W2D.Roll, l0);
            for (int pt = 0; pt < 2; pt++)
            {
                int pb = W2D.Point0 + pt * W2D.PointStride, ib = W2D.Impulse0 + 3 * pt;
                T nImp = W<T>(wide, ib, l0), tImp = W<T>(wide, ib + 1, l0);
                T px = nx * nImp + tx * tImp;
                T py = ny * nImp + ty * tImp;
                Apply(ref vAx, ref vAy, ref wA, ref vBx, ref vBy, ref wB, mA, iA, mB, iB,
                    W<T>(wide, pb + W2D.AnchorAX, l0), W<T>(wide, pb + W2D.AnchorAY, l0), W<T>(wide, pb + W2D.AnchorBX, l0), W<T>(wide, pb + W2D.AnchorBY, l0), px, py);
            }

            wA = wA - iA * rollImp;
            wB = wB + iB * rollImp;
            Scatter(state, idx, l0, vAx, vAy, wA, vBx, vBy, wB);
        }
    }

    private static void RunRestitution<T>(Span<float> state, Span<float> wide, ReadOnlySpan<int> idx, int lane0, int lanes, in KernelParams kp) where T : struct, ILane<T>
    {
        int width = T.Count;
        T zero = T.Zero, one = T.Broadcast(1f);
        T maskTrue = T.GreaterThan(one, zero);
        T negThreshold = T.Broadcast(-kp.RestitutionThreshold);
        for (int l0 = lane0; l0 < lane0 + lanes; l0 += width)
        {
            var ra = T.GatherRows(state, idx.Slice(l0, width));
            var rb = T.GatherRows(state, idx.Slice(8 + l0, width));
            T vAx = ra.R0, vAy = ra.R1, wA = ra.R2, vBx = rb.R0, vBy = rb.R1, wB = rb.R2;
            T nx = W<T>(wide, W2D.NormalX, l0), ny = W<T>(wide, W2D.NormalY, l0);
            T mA = W<T>(wide, W2D.InvMassA, l0), iA = W<T>(wide, W2D.InvInertiaA, l0), mB = W<T>(wide, W2D.InvMassB, l0), iB = W<T>(wide, W2D.InvInertiaB, l0);
            T restitution = W<T>(wide, W2D.Restitution, l0);
            for (int pt = 0; pt < 2; pt++)
            {
                int pb = W2D.Point0 + pt * W2D.PointStride, ib = W2D.Impulse0 + 3 * pt;
                T rAx = W<T>(wide, pb + W2D.AnchorAX, l0), rAy = W<T>(wide, pb + W2D.AnchorAY, l0), rBx = W<T>(wide, pb + W2D.AnchorBX, l0), rBy = W<T>(wide, pb + W2D.AnchorBY, l0);
                T nMass = W<T>(wide, pb + W2D.NormalMass, l0), relVel = W<T>(wide, pb + W2D.RelativeVelocity, l0);
                T nImp = W<T>(wide, ib, l0), maxN = W<T>(wide, ib + 2, l0);
                T dvx = (vBx - wB * rBy) - (vAx - wA * rAy);
                T dvy = (vBy + wB * rBx) - (vAy + wA * rAx);
                T vn = dvx * nx + dvy * ny;
                T notFast = T.Select(T.GreaterThan(relVel, negThreshold), zero, maskTrue); // relVel <= -threshold
                T mask = Lane.And(Lane.And(notFast, T.GreaterThan(maxN, zero)), T.GreaterThan(restitution, zero));
                T raw = Lane.Neg(nMass * (vn + restitution * relVel));
                T imp = T.Select(mask, raw, zero);
                T newImp = Lane.Max(nImp + imp, zero);
                imp = newImp - nImp;
                maxN = Lane.Max(maxN, newImp);
                Apply(ref vAx, ref vAy, ref wA, ref vBx, ref vBy, ref wB, mA, iA, mB, iB, rAx, rAy, rBx, rBy, nx * imp, ny * imp);
                // Restitution leaves the friction and rolling impulses alone.
                newImp.StoreAt(wide, ib * 8 + l0); maxN.StoreAt(wide, (ib + 2) * 8 + l0);
            }

            Scatter(state, idx, l0, vAx, vAy, wA, vBx, vBy, wB);
        }
    }

    private static void RunSolve<T>(Span<float> state, Span<float> wide, ReadOnlySpan<int> idx, int lane0, int lanes, bool useBias, in KernelParams kp) where T : struct, ILane<T>
    {
        int width = T.Count;
        for (int l0 = lane0; l0 < lane0 + lanes; l0 += width)
        {
            var ra = T.GatherRows(state, idx.Slice(l0, width));
            var rb = T.GatherRows(state, idx.Slice(8 + l0, width));
            T vAx = ra.R0, vAy = ra.R1, wA = ra.R2, vBx = rb.R0, vBy = rb.R1, wB = rb.R2;
            T mA = W<T>(wide, W2D.InvMassA, l0), iA = W<T>(wide, W2D.InvInertiaA, l0), mB = W<T>(wide, W2D.InvMassB, l0), iB = W<T>(wide, W2D.InvInertiaB, l0);
            T rolling = W<T>(wide, W2D.Rolling, l0), rollingMass = W<T>(wide, W2D.RollingMass, l0);
            T rollImp = W<T>(wide, W2D.Roll, l0);

            // Rolling resistance, then friction, then non-penetration last: the last constraint solved wins the argument, and keeping bodies apart matters most.
            T total = W<T>(wide, W2D.Impulse0, l0) + W<T>(wide, W2D.Impulse1, l0);
            T maxRoll = rolling * total;
            T lambda = Lane.Neg(rollingMass * (wB - wA));
            T newRoll = Clamp(rollImp + lambda, Lane.Neg(maxRoll), maxRoll);
            lambda = newRoll - rollImp;
            rollImp = newRoll;
            wA = wA - iA * lambda;
            wB = wB + iB * lambda;
            rollImp.StoreAt(wide, W2D.Roll * 8 + l0);

            FrictionPoints<T>(wide, l0, ref vAx, ref vAy, ref wA, ref vBx, ref vBy, ref wB);
            NormalPoints<T>(wide, l0, useBias, in kp, in ra, in rb, ref vAx, ref vAy, ref wA, ref vBx, ref vBy, ref wB);
            Scatter(state, idx, l0, vAx, vAy, wA, vBx, vBy, wB);
        }
    }

    // The friction and the non-penetration passes are separate methods, not inlined: each is its own unit for the JIT's inlining budget, which one method holding all of the lane operations exhausts
    // (the operators then stay calls). The velocities pass through references and are held in locals inside.
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void FrictionPoints<T>(Span<float> wide, int l0, ref T rvAx, ref T rvAy, ref T rwA, ref T rvBx, ref T rvBy, ref T rwB) where T : struct, ILane<T>
    {
        T vAx = rvAx, vAy = rvAy, wA = rwA, vBx = rvBx, vBy = rvBy, wB = rwB;
        T nx = W<T>(wide, W2D.NormalX, l0), ny = W<T>(wide, W2D.NormalY, l0);
        T mA = W<T>(wide, W2D.InvMassA, l0), iA = W<T>(wide, W2D.InvInertiaA, l0), mB = W<T>(wide, W2D.InvMassB, l0), iB = W<T>(wide, W2D.InvInertiaB, l0);
        T tx = ny, ty = Lane.Neg(nx); // the tangent is the normal turned clockwise
        T friction = W<T>(wide, W2D.Friction, l0);
        for (int pt = 0; pt < 2; pt++)
        {
            int pb = W2D.Point0 + pt * W2D.PointStride, ib = W2D.Impulse0 + 3 * pt;
            T rAx = W<T>(wide, pb + W2D.AnchorAX, l0), rAy = W<T>(wide, pb + W2D.AnchorAY, l0), rBx = W<T>(wide, pb + W2D.AnchorBX, l0), rBy = W<T>(wide, pb + W2D.AnchorBY, l0);
            T tMass = W<T>(wide, pb + W2D.TangentMass, l0);
            T nImp = W<T>(wide, ib, l0), tImp = W<T>(wide, ib + 1, l0);
            T dvx = (vBx - wB * rBy) - (vAx - wA * rAy);
            T dvy = (vBy + wB * rBx) - (vAy + wA * rAx);
            T vt = dvx * tx + dvy * ty;
            T imp = Lane.Neg(tMass * vt);
            T maxF = friction * nImp;
            T newT = Clamp(tImp + imp, Lane.Neg(maxF), maxF);
            imp = newT - tImp;
            newT.StoreAt(wide, (ib + 1) * 8 + l0);
            Apply(ref vAx, ref vAy, ref wA, ref vBx, ref vBy, ref wB, mA, iA, mB, iB, rAx, rAy, rBx, rBy, tx * imp, ty * imp);
        }

        rvAx = vAx; rvAy = vAy; rwA = wA; rvBx = vBx; rvBy = vBy; rwB = wB;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void NormalPoints<T>(Span<float> wide, int l0, bool useBias, in KernelParams kp, in Rows8<T> ra, in Rows8<T> rb, ref T rvAx, ref T rvAy, ref T rwA, ref T rvBx, ref T rvBy, ref T rwB) where T : struct, ILane<T>
    {
        T zero = T.Zero, one = T.Broadcast(1f);
        T negPush = T.Broadcast(-kp.PushOutVelocity);
        T slop = T.Broadcast(kp.LinearSlop);
        T invH = T.Broadcast(kp.InverseSubstepDt);
        T vAx = rvAx, vAy = rvAy, wA = rwA, vBx = rvBx, vBy = rvBy, wB = rwB;
        T dpAx = ra.R4, dpAy = ra.R5, dqAc = ra.R6, dqAs = ra.R7;
        T dpBx = rb.R4, dpBy = rb.R5, dqBc = rb.R6, dqBs = rb.R7;
        T nx = W<T>(wide, W2D.NormalX, l0), ny = W<T>(wide, W2D.NormalY, l0);
        T mA = W<T>(wide, W2D.InvMassA, l0), iA = W<T>(wide, W2D.InvInertiaA, l0), mB = W<T>(wide, W2D.InvMassB, l0), iB = W<T>(wide, W2D.InvInertiaB, l0);
        T biasRate = default, massScale = default, impulseScale = default;
        if (useBias) { biasRate = W<T>(wide, W2D.BiasRate, l0); massScale = W<T>(wide, W2D.MassScale, l0); impulseScale = W<T>(wide, W2D.ImpulseScale, l0); }
        for (int pt = 0; pt < 2; pt++)
        {
            int pb = W2D.Point0 + pt * W2D.PointStride, ib = W2D.Impulse0 + 3 * pt;
            T rAx = W<T>(wide, pb + W2D.AnchorAX, l0), rAy = W<T>(wide, pb + W2D.AnchorAY, l0), rBx = W<T>(wide, pb + W2D.AnchorBX, l0), rBy = W<T>(wide, pb + W2D.AnchorBY, l0);
            T adjSep = W<T>(wide, pb + W2D.AdjustedSeparation, l0), nMass = W<T>(wide, pb + W2D.NormalMass, l0);
            T nImp = W<T>(wide, ib, l0), maxN = W<T>(wide, ib + 2, l0);
            // Current separation from the accumulated body motion of this step: anchors turn with the bodies' delta rotations.
            T prAx = dqAc * rAx - dqAs * rAy, prAy = dqAs * rAx + dqAc * rAy;
            T prBx = dqBc * rBx - dqBs * rBy, prBy = dqBs * rBx + dqBc * rBy;
            T dx = (dpBx - dpAx) + (prBx - prAx);
            T dy = (dpBy - dpAy) + (prBy - prAy);
            T s = (dx * nx + dy * ny) + adjSep;
            T sPos = T.GreaterThan(s, zero);
            T bias, ms, isc;
            if (useBias)
            {
                T soft = Lane.Max(biasRate * Lane.Min(s + slop, zero), negPush);
                bias = T.Select(sPos, s * invH, soft);
                ms = T.Select(sPos, one, massScale);
                isc = T.Select(sPos, zero, impulseScale);
            }
            else
            {
                bias = T.Select(sPos, s * invH, zero);
                ms = one;
                isc = zero;
            }

            T dvx = (vBx - wB * rBy) - (vAx - wA * rAy);
            T dvy = (vBy + wB * rBx) - (vAy + wA * rAx);
            T vn = dvx * nx + dvy * ny;
            T imp = Lane.Neg(nMass * ms * (vn + bias)) - isc * nImp;
            T newImp = Lane.Max(nImp + imp, zero);
            imp = newImp - nImp;
            maxN = Lane.Max(maxN, newImp);
            Apply(ref vAx, ref vAy, ref wA, ref vBx, ref vBy, ref wB, mA, iA, mB, iB, rAx, rAy, rBx, rBy, nx * imp, ny * imp);
            newImp.StoreAt(wide, ib * 8 + l0); maxN.StoreAt(wide, (ib + 2) * 8 + l0);
        }

        rvAx = vAx; rvAy = vAy; rwA = wA; rvBx = vBx; rvBy = vBy; rwB = wB;
    }


    // Writes the solved velocities back. Scatter offsets are body rows the preparation wrote (a body index plus one, times the row stride), or -1 for a body that keeps its velocity.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Scatter<T>(Span<float> state, ReadOnlySpan<int> idx, int l0, T vAx, T vAy, T wA, T vBx, T vBy, T wB) where T : struct, ILane<T>
    {
        int width = T.Count;
        T.Scatter3(state, idx.Slice(16 + l0, width), vAx, vAy, wA);
        T.Scatter3(state, idx.Slice(24 + l0, width), vBx, vBy, wB);
    }
}
