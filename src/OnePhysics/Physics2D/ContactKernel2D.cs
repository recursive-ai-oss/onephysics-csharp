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
/// Indices into the per-constraint scratch rows that <c>Prepare</c> fills and the kernel reads (floats, <see cref="ContactKernel2D.Stride"/> per constraint).
/// Row layout: normal (2), friction, restitution, rolling resistance, rolling mass, inverse masses and inertias of A and B, soft-constraint coefficients;
/// then two points of eight floats starting at <see cref="Point0"/>.
/// </summary>
internal static class Row2D
{
    public const int NormalX = 0, NormalY = 1, Friction = 2, Restitution = 3, Rolling = 4, RollingMass = 5;
    public const int InvMassA = 6, InvInertiaA = 7, InvMassB = 8, InvInertiaB = 9;
    public const int BiasRate = 10, MassScale = 11, ImpulseScale = 12;
    public const int Point0 = 16, PointStride = 8;

    // Offsets inside a point.
    public const int AnchorAX = 0, AnchorAY = 1, AnchorBX = 2, AnchorBY = 3, AdjustedSeparation = 4, NormalMass = 5, TangentMass = 6, RelativeVelocity = 7;
}

/// <summary>
/// The 2D contact solver kernel: soft step (Catto, "Solver2D" and the soft-step talk): substeps with soft contact constraints, a bias-free relax pass, and restitution once at the end.
/// It is written once over <see cref="ILane{T}"/>. A batch of up to eight constraints of one color is gathered into lanes, solved with the same operations in the same order at
/// every width, and scattered back. The constraints of a batch touch disjoint dynamic bodies, so lanes never depend on each other, and static and kinematic bodies are only read.
/// </summary>
internal static class ContactKernel2D
{
    public const int Stride = 32;

    private const int Fields = 50;
    private const int Out = 13;
    private const int MaxLanes = 8;

    [InlineArray(Fields * MaxLanes)]
    private struct Stage
    {
        private float _e;
    }

    [InlineArray(Out * MaxLanes)]
    private struct OutStage
    {
        private float _e;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T L<T>(ReadOnlySpan<float> st, int f) where T : struct, ILane<T> => T.Load(st.Slice(f * T.Count, T.Count));

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

    private static Pt<T> LoadPoint<T>(ReadOnlySpan<float> st, int pbase, int ibase) where T : struct, ILane<T> => new()
    {
        RAx = L<T>(st, pbase + Row2D.AnchorAX), RAy = L<T>(st, pbase + Row2D.AnchorAY), RBx = L<T>(st, pbase + Row2D.AnchorBX), RBy = L<T>(st, pbase + Row2D.AnchorBY),
        AdjSep = L<T>(st, pbase + Row2D.AdjustedSeparation), NMass = L<T>(st, pbase + Row2D.NormalMass), TMass = L<T>(st, pbase + Row2D.TangentMass), RelVel = L<T>(st, pbase + Row2D.RelativeVelocity),
        NImp = L<T>(st, ibase), TImp = L<T>(st, ibase + 1), MaxN = L<T>(st, ibase + 2),
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

    /// <summary>Solves the given constraints (at most eight of one color, or a slice of the overflow set) in <paramref name="phase"/>.</summary>
    public static void Run<T>(Engine2D e, ReadOnlySpan<int> cons, SolvePhase phase, in KernelParams kp) where T : struct, ILane<T>
    {
        int width = T.Count;
        var core = e.Core;
        var cs = core.Constraints;
        ReadOnlySpan<int> bodyA = cs.BodyA, bodyB = cs.BodyB;
        Span<float> payload = cs.PayloadAll;
        int pf = cs.PayloadFloats;
        Span<float> lin = core.Bodies.LinearVelocity;
        Span<float> ang = core.Bodies.AngularVelocity;
        Span<byte> type = core.Bodies.Type;
        float[] dp = e.Dp, dq = e.Dq, scratch = e.Scratch;

        Stage stage = default;
        OutStage outs = default;
        Span<float> st = stage;
        Span<float> os = outs;
        T zero = T.Zero, one = T.Broadcast(1f);
        T maskTrue = T.GreaterThan(one, zero);
        T negPush = T.Broadcast(-kp.PushOutVelocity);
        T slop = T.Broadcast(kp.LinearSlop);
        T invH = T.Broadcast(kp.InverseSubstepDt);
        T negThreshold = T.Broadcast(-kp.RestitutionThreshold);

        for (int g = 0; g < cons.Length; g += width)
        {
            int active = Math.Min(width, cons.Length - g);
            if (active < width) { st.Clear(); }
            for (int l = 0; l < active; l++)
            {
                int c = cons[g + l];
                int a = bodyA[c], b = bodyB[c];
                st[0 * width + l] = lin[2 * a]; st[1 * width + l] = lin[2 * a + 1]; st[2 * width + l] = ang[a];
                st[3 * width + l] = dp[2 * a]; st[4 * width + l] = dp[2 * a + 1]; st[5 * width + l] = dq[2 * a]; st[6 * width + l] = dq[2 * a + 1];
                st[7 * width + l] = lin[2 * b]; st[8 * width + l] = lin[2 * b + 1]; st[9 * width + l] = ang[b];
                st[10 * width + l] = dp[2 * b]; st[11 * width + l] = dp[2 * b + 1]; st[12 * width + l] = dq[2 * b]; st[13 * width + l] = dq[2 * b + 1];
                int sb = c * Stride;
                for (int f = 0; f < 13; f++) { st[(14 + f) * width + l] = scratch[sb + f]; }
                for (int f = 0; f < 16; f++) { st[(27 + f) * width + l] = scratch[sb + Row2D.Point0 + f]; }
                int pb = c * pf;
                st[43 * width + l] = payload[pb + ManifoldLayout.RollingImpulse];
                st[44 * width + l] = payload[pb + ManifoldLayout.NormalImpulse(0)];
                st[45 * width + l] = payload[pb + ManifoldLayout.Tangent1Impulse(0)];
                st[46 * width + l] = payload[pb + ManifoldLayout.MaxNormalImpulse(0)];
                st[47 * width + l] = payload[pb + ManifoldLayout.NormalImpulse(1)];
                st[48 * width + l] = payload[pb + ManifoldLayout.Tangent1Impulse(1)];
                st[49 * width + l] = payload[pb + ManifoldLayout.MaxNormalImpulse(1)];
            }

            var v = new Vel<T>
            {
                Ax = L<T>(st, 0), Ay = L<T>(st, 1), Wa = L<T>(st, 2), Bx = L<T>(st, 7), By = L<T>(st, 8), Wb = L<T>(st, 9),
            };
            var k = new Con<T>
            {
                DpAx = L<T>(st, 3), DpAy = L<T>(st, 4), DqAc = L<T>(st, 5), DqAs = L<T>(st, 6),
                DpBx = L<T>(st, 10), DpBy = L<T>(st, 11), DqBc = L<T>(st, 12), DqBs = L<T>(st, 13),
                Nx = L<T>(st, 14), Ny = L<T>(st, 15), Friction = L<T>(st, 16), Restitution = L<T>(st, 17), Rolling = L<T>(st, 18), RollingMass = L<T>(st, 19),
                MA = L<T>(st, 20), IA = L<T>(st, 21), MB = L<T>(st, 22), IB = L<T>(st, 23),
                BiasRate = L<T>(st, 24), MassScale = L<T>(st, 25), ImpulseScale = L<T>(st, 26),
            };
            k.Tx = k.Ny; k.Ty = Lane.Neg(k.Nx); // the tangent is the normal turned clockwise
            T rollImp = L<T>(st, 43);
            var p0 = LoadPoint<T>(st, 27, 44);
            var p1 = LoadPoint<T>(st, 35, 47);

            if (phase == SolvePhase.WarmStart)
            {
                WarmStart(ref v, in k, in p0);
                WarmStart(ref v, in k, in p1);
                v.Wa = v.Wa - k.IA * rollImp;
                v.Wb = v.Wb + k.IB * rollImp;
            }
            else if (phase == SolvePhase.Restitution)
            {
                Restitute(ref v, in k, ref p0, maskTrue, negThreshold);
                Restitute(ref v, in k, ref p1, maskTrue, negThreshold);
            }
            else
            {
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
            T nImp0 = p0.NImp, tImp0 = p0.TImp, maxN0 = p0.MaxN, nImp1 = p1.NImp, tImp1 = p1.TImp, maxN1 = p1.MaxN;
            vAx.Store(os.Slice(0 * width, width)); vAy.Store(os.Slice(1 * width, width)); wA.Store(os.Slice(2 * width, width));
            vBx.Store(os.Slice(3 * width, width)); vBy.Store(os.Slice(4 * width, width)); wB.Store(os.Slice(5 * width, width));
            rollImp.Store(os.Slice(6 * width, width));
            nImp0.Store(os.Slice(7 * width, width)); tImp0.Store(os.Slice(8 * width, width)); maxN0.Store(os.Slice(9 * width, width));
            nImp1.Store(os.Slice(10 * width, width)); tImp1.Store(os.Slice(11 * width, width)); maxN1.Store(os.Slice(12 * width, width));

            for (int l = 0; l < active; l++)
            {
                int c = cons[g + l];
                int a = bodyA[c], b = bodyB[c];
                if (type[a] == (byte)BodyType.Dynamic)
                {
                    lin[2 * a] = os[0 * width + l]; lin[2 * a + 1] = os[1 * width + l]; ang[a] = os[2 * width + l];
                }

                if (type[b] == (byte)BodyType.Dynamic)
                {
                    lin[2 * b] = os[3 * width + l]; lin[2 * b + 1] = os[4 * width + l]; ang[b] = os[5 * width + l];
                }

                if (phase != SolvePhase.WarmStart)
                {
                    int pb = c * pf;
                    payload[pb + ManifoldLayout.RollingImpulse] = os[6 * width + l];
                    payload[pb + ManifoldLayout.NormalImpulse(0)] = os[7 * width + l];
                    payload[pb + ManifoldLayout.Tangent1Impulse(0)] = os[8 * width + l];
                    payload[pb + ManifoldLayout.MaxNormalImpulse(0)] = os[9 * width + l];
                    payload[pb + ManifoldLayout.NormalImpulse(1)] = os[10 * width + l];
                    payload[pb + ManifoldLayout.Tangent1Impulse(1)] = os[11 * width + l];
                    payload[pb + ManifoldLayout.MaxNormalImpulse(1)] = os[12 * width + l];
                }
            }
        }
    }
}
