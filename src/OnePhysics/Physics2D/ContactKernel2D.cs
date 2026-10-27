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
        private const int Out = 6;
    private const int MaxLanes = 8;

    [InlineArray(Out * MaxLanes)]
    private struct OutStage
    {
        private float _e;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T W<T>(ReadOnlySpan<float> wide, int f, int lane0) where T : struct, ILane<T> => T.LoadAt(wide, f * 8 + lane0);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T Clamp<T>(T x, T lo, T hi) where T : struct, ILane<T> => Lane.Max(lo, Lane.Min(hi, x));

    private struct Vel<T> where T : struct, ILane<T>
    {
        public T Ax, Ay, Wa, Bx, By, Wb;
    }

    private struct Con<T> where T : struct, ILane<T>
    {
        public T DpAx, DpAy, DqAc, DqAs, DpBx, DpBy, DqBc, DqBs;
        public T Nx, Ny, Tx, Ty, Friction, Restitution, Rolling, RollingMass, MA, IA, MB, IB, BiasRate, MassScale, ImpulseScale;
    }

    private struct Pt<T> where T : struct, ILane<T>
    {
        public T RAx, RAy, RBx, RBy, AdjSep, NMass, TMass, RelVel, NImp, TImp, MaxN;
    }

    private static Pt<T> LoadPoint<T>(ReadOnlySpan<float> wide, int pbase, int ibase, int l0) where T : struct, ILane<T> => new()
    {
        RAx = W<T>(wide, pbase + W2D.AnchorAX, l0), RAy = W<T>(wide, pbase + W2D.AnchorAY, l0), RBx = W<T>(wide, pbase + W2D.AnchorBX, l0), RBy = W<T>(wide, pbase + W2D.AnchorBY, l0),
        AdjSep = W<T>(wide, pbase + W2D.AdjustedSeparation, l0), NMass = W<T>(wide, pbase + W2D.NormalMass, l0), TMass = W<T>(wide, pbase + W2D.TangentMass, l0), RelVel = W<T>(wide, pbase + W2D.RelativeVelocity, l0),
        NImp = W<T>(wide, ibase, l0), TImp = W<T>(wide, ibase + 1, l0), MaxN = W<T>(wide, ibase + 2, l0),
    };

    private static Pt<T> LoadWarm<T>(ReadOnlySpan<float> wide, int pbase, int ibase, int l0) where T : struct, ILane<T> => new()
    {
        RAx = W<T>(wide, pbase + W2D.AnchorAX, l0), RAy = W<T>(wide, pbase + W2D.AnchorAY, l0), RBx = W<T>(wide, pbase + W2D.AnchorBX, l0), RBy = W<T>(wide, pbase + W2D.AnchorBY, l0),
        NImp = W<T>(wide, ibase, l0), TImp = W<T>(wide, ibase + 1, l0),
    };

    private static Pt<T> LoadRestitution<T>(ReadOnlySpan<float> wide, int pbase, int ibase, int l0) where T : struct, ILane<T> => new()
    {
        RAx = W<T>(wide, pbase + W2D.AnchorAX, l0), RAy = W<T>(wide, pbase + W2D.AnchorAY, l0), RBx = W<T>(wide, pbase + W2D.AnchorBX, l0), RBy = W<T>(wide, pbase + W2D.AnchorBY, l0),
        NMass = W<T>(wide, pbase + W2D.NormalMass, l0), RelVel = W<T>(wide, pbase + W2D.RelativeVelocity, l0),
        NImp = W<T>(wide, ibase, l0), MaxN = W<T>(wide, ibase + 2, l0),
    };

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Apply<T>(ref Vel<T> v, in Con<T> k, in Pt<T> p, T px, T py) where T : struct, ILane<T>
    {
        v.Ax = v.Ax - k.MA * px; v.Ay = v.Ay - k.MA * py;
        v.Wa = v.Wa - k.IA * (p.RAx * py - p.RAy * px);
        v.Bx = v.Bx + k.MB * px; v.By = v.By + k.MB * py;
        v.Wb = v.Wb + k.IB * (p.RBx * py - p.RBy * px);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void RelativeVelocity<T>(in Vel<T> v, in Pt<T> p, out T dvx, out T dvy) where T : struct, ILane<T>
    {
        dvx = (v.Bx - v.Wb * p.RBy) - (v.Ax - v.Wa * p.RAy);
        dvy = (v.By + v.Wb * p.RBx) - (v.Ay + v.Wa * p.RAx);
    }

    private static void WarmStart<T>(ref Vel<T> v, in Con<T> k, in Pt<T> p) where T : struct, ILane<T>
    {
        T px = k.Nx * p.NImp + k.Tx * p.TImp;
        T py = k.Ny * p.NImp + k.Ty * p.TImp;
        Apply(ref v, in k, in p, px, py);
    }

    private static void Restitute<T>(ref Vel<T> v, in Con<T> k, ref Pt<T> p, T maskTrue, T negThreshold) where T : struct, ILane<T>
    {
        T zero = T.Zero;
        RelativeVelocity(in v, in p, out T dvx, out T dvy);
        T vn = dvx * k.Nx + dvy * k.Ny;
        T notFast = T.Select(T.GreaterThan(p.RelVel, negThreshold), zero, maskTrue); // relVel <= -threshold
        T mask = Lane.And(Lane.And(notFast, T.GreaterThan(p.MaxN, zero)), T.GreaterThan(k.Restitution, zero));
        T raw = Lane.Neg(p.NMass * (vn + k.Restitution * p.RelVel));
        T imp = T.Select(mask, raw, zero);
        T newImp = Lane.Max(p.NImp + imp, zero);
        imp = newImp - p.NImp;
        p.NImp = newImp;
        p.MaxN = Lane.Max(p.MaxN, newImp);
        Apply(ref v, in k, in p, k.Nx * imp, k.Ny * imp);
    }

    private static void Friction<T>(ref Vel<T> v, in Con<T> k, ref Pt<T> p) where T : struct, ILane<T>
    {
        RelativeVelocity(in v, in p, out T dvx, out T dvy);
        T vt = dvx * k.Tx + dvy * k.Ty;
        T imp = Lane.Neg(p.TMass * vt);
        T maxF = k.Friction * p.NImp;
        T newT = Clamp(p.TImp + imp, Lane.Neg(maxF), maxF);
        imp = newT - p.TImp;
        p.TImp = newT;
        Apply(ref v, in k, in p, k.Tx * imp, k.Ty * imp);
    }

    private static void Normal<T>(ref Vel<T> v, in Con<T> k, ref Pt<T> p, bool useBias, T slop, T invH, T negPush) where T : struct, ILane<T>
    {
        T zero = T.Zero, one = T.Broadcast(1f);
        // Current separation from the accumulated body motion of this step: anchors turn with the bodies' delta rotations.
        T prAx = k.DqAc * p.RAx - k.DqAs * p.RAy, prAy = k.DqAs * p.RAx + k.DqAc * p.RAy;
        T prBx = k.DqBc * p.RBx - k.DqBs * p.RBy, prBy = k.DqBs * p.RBx + k.DqBc * p.RBy;
        T dx = (k.DpBx - k.DpAx) + (prBx - prAx);
        T dy = (k.DpBy - k.DpAy) + (prBy - prAy);
        T s = (dx * k.Nx + dy * k.Ny) + p.AdjSep;
        T sPos = T.GreaterThan(s, zero);
        T bias, ms, isc;
        if (useBias)
        {
            T soft = Lane.Max(k.BiasRate * Lane.Min(s + slop, zero), negPush);
            bias = T.Select(sPos, s * invH, soft);
            ms = T.Select(sPos, one, k.MassScale);
            isc = T.Select(sPos, zero, k.ImpulseScale);
        }
        else
        {
            bias = T.Select(sPos, s * invH, zero);
            ms = one;
            isc = zero;
        }

        RelativeVelocity(in v, in p, out T dvx, out T dvy);
        T vn = dvx * k.Nx + dvy * k.Ny;
        T imp = Lane.Neg(p.NMass * ms * (vn + bias)) - isc * p.NImp;
        T newImp = Lane.Max(p.NImp + imp, zero);
        imp = newImp - p.NImp;
        p.NImp = newImp;
        p.MaxN = Lane.Max(p.MaxN, newImp);
        Apply(ref v, in k, in p, k.Nx * imp, k.Ny * imp);
    }

    /// <summary>
    /// Solves lanes <paramref name="lane0"/>..<paramref name="lane0"/>+<paramref name="lanes"/>-1 of one group in <paramref name="phase"/>. Body velocities are gathered from <see cref="Engine2D.BodyState"/>
    /// and scattered back; static and kinematic bodies (and unused lanes) have no scatter offset, so they are only read.
    /// </summary>
    public static void Run<T>(Engine2D e, int group, int lane0, int lanes, SolvePhase phase, in KernelParams kp) where T : struct, ILane<T>
    {
        int width = T.Count;
        var wide = e.Wide.AsSpan(group * W2D.GroupFloats, W2D.GroupFloats);
        var idx = e.WideIdx.AsSpan(group * W2D.IdxPerGroup, W2D.IdxPerGroup);
        Span<float> state = e.BodyState;

        OutStage outs = default;
        Span<float> os = outs;
        T zero = T.Zero, one = T.Broadcast(1f);
        T maskTrue = T.GreaterThan(one, zero);
        T negPush = T.Broadcast(-kp.PushOutVelocity);
        T slop = T.Broadcast(kp.LinearSlop);
        T invH = T.Broadcast(kp.InverseSubstepDt);
        T negThreshold = T.Broadcast(-kp.RestitutionThreshold);

        for (int l0 = lane0; l0 < lane0 + lanes; l0 += width)
        {
            var gA = idx.Slice(l0, width);
            var gB = idx.Slice(8 + l0, width);
            var ra = T.GatherRows(state, gA);
            var rb = T.GatherRows(state, gB);
            var v = new Vel<T> { Ax = ra.R0, Ay = ra.R1, Wa = ra.R2, Bx = rb.R0, By = rb.R1, Wb = rb.R2 };
            var k = new Con<T>
            {
                Nx = W<T>(wide, W2D.NormalX, l0), Ny = W<T>(wide, W2D.NormalY, l0),
                MA = W<T>(wide, W2D.InvMassA, l0), IA = W<T>(wide, W2D.InvInertiaA, l0), MB = W<T>(wide, W2D.InvMassB, l0), IB = W<T>(wide, W2D.InvInertiaB, l0),
            };
            k.Tx = k.Ny; k.Ty = Lane.Neg(k.Nx); // the tangent is the normal turned clockwise
            T rollImp = W<T>(wide, W2D.Roll, l0);
            Pt<T> p0 = default, p1 = default;

            if (phase == SolvePhase.WarmStart)
            {
                p0 = LoadWarm<T>(wide, W2D.Point0, W2D.Impulse0, l0);
                p1 = LoadWarm<T>(wide, W2D.Point0 + W2D.PointStride, W2D.Impulse1, l0);
                WarmStart(ref v, in k, in p0);
                WarmStart(ref v, in k, in p1);
                v.Wa = v.Wa - k.IA * rollImp;
                v.Wb = v.Wb + k.IB * rollImp;
            }
            else if (phase == SolvePhase.Restitution)
            {
                k.Restitution = W<T>(wide, W2D.Restitution, l0);
                p0 = LoadRestitution<T>(wide, W2D.Point0, W2D.Impulse0, l0);
                p1 = LoadRestitution<T>(wide, W2D.Point0 + W2D.PointStride, W2D.Impulse1, l0);
                Restitute(ref v, in k, ref p0, maskTrue, negThreshold);
                Restitute(ref v, in k, ref p1, maskTrue, negThreshold);
            }
            else
            {
                k.Friction = W<T>(wide, W2D.Friction, l0); k.Rolling = W<T>(wide, W2D.Rolling, l0); k.RollingMass = W<T>(wide, W2D.RollingMass, l0);
                k.BiasRate = W<T>(wide, W2D.BiasRate, l0); k.MassScale = W<T>(wide, W2D.MassScale, l0); k.ImpulseScale = W<T>(wide, W2D.ImpulseScale, l0);
                k.DpAx = ra.R4; k.DpAy = ra.R5; k.DqAc = ra.R6; k.DqAs = ra.R7;
                k.DpBx = rb.R4; k.DpBy = rb.R5; k.DqBc = rb.R6; k.DqBs = rb.R7;
                p0 = LoadPoint<T>(wide, W2D.Point0, W2D.Impulse0, l0);
                p1 = LoadPoint<T>(wide, W2D.Point0 + W2D.PointStride, W2D.Impulse1, l0);
                bool useBias = phase == SolvePhase.Solve;
                // Rolling resistance, then friction, then non-penetration last: the last constraint solved wins the argument, and keeping bodies apart matters most.
                T total = p0.NImp + p1.NImp;
                T maxRoll = k.Rolling * total;
                T lambda = Lane.Neg(k.RollingMass * (v.Wb - v.Wa));
                T newRoll = Clamp(rollImp + lambda, Lane.Neg(maxRoll), maxRoll);
                lambda = newRoll - rollImp;
                rollImp = newRoll;
                v.Wa = v.Wa - k.IA * lambda;
                v.Wb = v.Wb + k.IB * lambda;
                Friction(ref v, in k, ref p0);
                Friction(ref v, in k, ref p1);
                Normal(ref v, in k, ref p0, useBias, slop, invH, negPush);
                Normal(ref v, in k, ref p1, useBias, slop, invH, negPush);
            }

            T vAx = v.Ax, vAy = v.Ay, wA = v.Wa, vBx = v.Bx, vBy = v.By, wB = v.Wb;
            vAx.StoreAt(os, 0 * width); vAy.StoreAt(os, 1 * width); wA.StoreAt(os, 2 * width);
            vBx.StoreAt(os, 3 * width); vBy.StoreAt(os, 4 * width); wB.StoreAt(os, 5 * width);
            for (int l = 0; l < width; l++)
            {
                // Scatter offsets are body rows the preparation wrote (a body index plus one, times the row stride), or -1 for a body that keeps its velocity.
                int oa = idx[16 + l0 + l];
                if (oa >= 0)
                {
                    Mem.Range(state, oa, BS2D.W + 1);
                    Mem.At(state, oa + BS2D.Vx) = os[l]; Mem.At(state, oa + BS2D.Vy) = os[width + l]; Mem.At(state, oa + BS2D.W) = os[2 * width + l];
                }

                int ob = idx[24 + l0 + l];
                if (ob >= 0)
                {
                    Mem.Range(state, ob, BS2D.W + 1);
                    Mem.At(state, ob + BS2D.Vx) = os[3 * width + l]; Mem.At(state, ob + BS2D.Vy) = os[4 * width + l]; Mem.At(state, ob + BS2D.W) = os[5 * width + l];
                }
            }

            if (phase == SolvePhase.Restitution)
            {
                // Restitution leaves the friction and rolling impulses alone.
                p0.NImp.StoreAt(wide, W2D.Impulse0 * 8 + l0); p0.MaxN.StoreAt(wide, (W2D.Impulse0 + 2) * 8 + l0);
                p1.NImp.StoreAt(wide, W2D.Impulse1 * 8 + l0); p1.MaxN.StoreAt(wide, (W2D.Impulse1 + 2) * 8 + l0);
            }
            else if (phase != SolvePhase.WarmStart)
            {
                rollImp.StoreAt(wide, W2D.Roll * 8 + l0);
                p0.NImp.StoreAt(wide, W2D.Impulse0 * 8 + l0); p0.TImp.StoreAt(wide, (W2D.Impulse0 + 1) * 8 + l0); p0.MaxN.StoreAt(wide, (W2D.Impulse0 + 2) * 8 + l0);
                p1.NImp.StoreAt(wide, W2D.Impulse1 * 8 + l0); p1.TImp.StoreAt(wide, (W2D.Impulse1 + 1) * 8 + l0); p1.MaxN.StoreAt(wide, (W2D.Impulse1 + 2) * 8 + l0);
            }
        }
    }
}
