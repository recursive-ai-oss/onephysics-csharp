using System;
using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePhysics.Experiments.Batching;

internal enum KernelMode
{
    WarmStart,
    Solve,
    Relax,
}

/// <summary>Soft-contact coefficients for one substep (Catto, soft constraints).</summary>
internal readonly record struct SoftParams(float BiasRate, float MassScale, float ImpulseScale, float InvH, float MaxBiasVelocity);

/// <summary>
/// The contact solver kernel, written once over <see cref="ILane{T}"/>. A group of <c>T.Count</c> consecutive slots is
/// gathered into lanes, solved with the same operations in the same order whatever the width, and scattered back. Slots in
/// one batch touch different bodies, so lanes never depend on each other.
/// </summary>
internal static class Kernels
{
    private const int Gather = 11; // body fields per body that the kernel reads
    private const int Max = Pile.L;

    /// <summary>Solves slots <c>[start, end)</c>; the range holds a whole number of <c>T.Count</c> slots.</summary>
    public static void Run<T>(Pile pile, int start, int end, KernelMode mode, SoftParams soft) where T : struct, ILane<T>
    {
        int count = T.Count;
        if (Pile.L % count != 0 || (end - start) % count != 0)
        {
            throw new InvalidOperationException($"A range of {end - start} slots does not divide into groups of {count} lanes.");
        }
        Span<float> scratch = stackalloc float[2 * Gather * Max];
        float[] body = pile.Body;
        float[] data = pile.Data;
        int[] ia = pile.IA, ib = pile.IB;
        int stride = pile.Stride;
        int n = pile.N;

        for (int g = start; g < end; g += count)
        {
            for (int k = 0; k < count; k++)
            {
                int a = ia[g + k] * Pile.BodyStride, b = ib[g + k] * Pile.BodyStride;
                for (int f = 0; f < Gather; f++)
                {
                    scratch[f * count + k] = body[a + f];
                    scratch[(Gather + f) * count + k] = body[b + f];
                }
            }
            T vAx = Ld<T>(scratch, 0, count), vAy = Ld<T>(scratch, 1, count), vAz = Ld<T>(scratch, 2, count);
            T wAx = Ld<T>(scratch, 3, count), wAy = Ld<T>(scratch, 4, count), wAz = Ld<T>(scratch, 5, count);
            T dAx = Ld<T>(scratch, 6, count), dAy = Ld<T>(scratch, 7, count), dAz = Ld<T>(scratch, 8, count);
            T mA = Ld<T>(scratch, 9, count), iA = Ld<T>(scratch, 10, count);
            T vBx = Ld<T>(scratch, Gather + 0, count), vBy = Ld<T>(scratch, Gather + 1, count), vBz = Ld<T>(scratch, Gather + 2, count);
            T wBx = Ld<T>(scratch, Gather + 3, count), wBy = Ld<T>(scratch, Gather + 4, count), wBz = Ld<T>(scratch, Gather + 5, count);
            T dBx = Ld<T>(scratch, Gather + 6, count), dBy = Ld<T>(scratch, Gather + 7, count), dBz = Ld<T>(scratch, Gather + 8, count);
            T mB = Ld<T>(scratch, Gather + 9, count), iB = Ld<T>(scratch, Gather + 10, count);

            T nx = F<T>(data, Pile.NX, stride, g), ny = F<T>(data, Pile.NX + 1, stride, g), nz = F<T>(data, Pile.NX + 2, stride, g);
            T lamN = F<T>(data, Pile.LAMN, stride, g);
            T lamT1 = F<T>(data, Pile.LAMT1, stride, g), lamT2 = F<T>(data, Pile.LAMT2, stride, g);

            if (mode == KernelMode.WarmStart)
            {
                T t1x = F<T>(data, Pile.T1X, stride, g), t1y = F<T>(data, Pile.T1X + 1, stride, g), t1z = F<T>(data, Pile.T1X + 2, stride, g);
                T t2x = F<T>(data, Pile.T2X, stride, g), t2y = F<T>(data, Pile.T2X + 1, stride, g), t2z = F<T>(data, Pile.T2X + 2, stride, g);
                Apply(lamN, nx, ny, nz, F<T>(data, Pile.RAN, stride, g), F<T>(data, Pile.RAN + 1, stride, g), F<T>(data, Pile.RAN + 2, stride, g),
                    F<T>(data, Pile.RBN, stride, g), F<T>(data, Pile.RBN + 1, stride, g), F<T>(data, Pile.RBN + 2, stride, g),
                    mA, iA, mB, iB, ref vAx, ref vAy, ref vAz, ref wAx, ref wAy, ref wAz, ref vBx, ref vBy, ref vBz, ref wBx, ref wBy, ref wBz);
                Apply(lamT1, t1x, t1y, t1z, F<T>(data, Pile.RAT1, stride, g), F<T>(data, Pile.RAT1 + 1, stride, g), F<T>(data, Pile.RAT1 + 2, stride, g),
                    F<T>(data, Pile.RBT1, stride, g), F<T>(data, Pile.RBT1 + 1, stride, g), F<T>(data, Pile.RBT1 + 2, stride, g),
                    mA, iA, mB, iB, ref vAx, ref vAy, ref vAz, ref wAx, ref wAy, ref wAz, ref vBx, ref vBy, ref vBz, ref wBx, ref wBy, ref wBz);
                Apply(lamT2, t2x, t2y, t2z, F<T>(data, Pile.RAT2, stride, g), F<T>(data, Pile.RAT2 + 1, stride, g), F<T>(data, Pile.RAT2 + 2, stride, g),
                    F<T>(data, Pile.RBT2, stride, g), F<T>(data, Pile.RBT2 + 1, stride, g), F<T>(data, Pile.RBT2 + 2, stride, g),
                    mA, iA, mB, iB, ref vAx, ref vAy, ref vAz, ref wAx, ref wAy, ref wAz, ref vBx, ref vBy, ref vBz, ref wBx, ref wBy, ref wBz);
            }
            else
            {
                bool useBias = mode == KernelMode.Solve;
                T zero = T.Zero, one = T.Broadcast(1f);
                T fric = F<T>(data, Pile.FRIC, stride, g);

                // Friction first, so the normal constraint has the last word (Catto, Solver2D).
                T maxF = fric * lamN;
                T negMaxF = Lane.Neg(maxF);
                lamT1 = Tangent(data, stride, g, Pile.T1X, Pile.RAT1, Pile.RBT1, Pile.MASST1, lamT1, maxF, negMaxF,
                    mA, iA, mB, iB, ref vAx, ref vAy, ref vAz, ref wAx, ref wAy, ref wAz, ref vBx, ref vBy, ref vBz, ref wBx, ref wBy, ref wBz);
                lamT2 = Tangent(data, stride, g, Pile.T2X, Pile.RAT2, Pile.RBT2, Pile.MASST2, lamT2, maxF, negMaxF,
                    mA, iA, mB, iB, ref vAx, ref vAy, ref vAz, ref wAx, ref wAy, ref wAz, ref vBx, ref vBy, ref vBz, ref wBx, ref wBy, ref wBz);

                T raNx = F<T>(data, Pile.RAN, stride, g), raNy = F<T>(data, Pile.RAN + 1, stride, g), raNz = F<T>(data, Pile.RAN + 2, stride, g);
                T rbNx = F<T>(data, Pile.RBN, stride, g), rbNy = F<T>(data, Pile.RBN + 1, stride, g), rbNz = F<T>(data, Pile.RBN + 2, stride, g);

                // Current separation: the value at the start of the step plus the relative motion along the normal since.
                T s = F<T>(data, Pile.Sep0, stride, g) + (((dBx - dAx) * nx + (dBy - dAy) * ny) + (dBz - dAz) * nz);
                T vn = (((vBx - vAx) * nx + (vBy - vAy) * ny) + (vBz - vAz) * nz)
                    + ((wBx * rbNx + wBy * rbNy) + wBz * rbNz) - ((wAx * raNx + wAy * raNy) + wAz * raNz);

                T speculative = T.GreaterThan(s, zero);
                T bias, massScale, impulseScale;
                if (useBias)
                {
                    T soft_ = Lane.Max(T.Broadcast(soft.BiasRate) * s, T.Broadcast(-soft.MaxBiasVelocity));
                    bias = T.Select(speculative, s * T.Broadcast(soft.InvH), soft_);
                    massScale = T.Select(speculative, one, T.Broadcast(soft.MassScale));
                    impulseScale = T.Select(speculative, zero, T.Broadcast(soft.ImpulseScale));
                }
                else
                {
                    // Relax: no push-out, but a separated contact still may not close faster than it gap allows.
                    bias = T.Select(speculative, s * T.Broadcast(soft.InvH), zero);
                    massScale = one;
                    impulseScale = zero;
                }
                T massN = F<T>(data, Pile.MASSN, stride, g);
                T impulse = Lane.Neg(massN * massScale * (vn + bias)) - impulseScale * lamN;
                T newLam = Lane.Max(lamN + impulse, zero);
                impulse = newLam - lamN;
                lamN = newLam;
                Apply(impulse, nx, ny, nz, raNx, raNy, raNz, rbNx, rbNy, rbNz,
                    mA, iA, mB, iB, ref vAx, ref vAy, ref vAz, ref wAx, ref wAy, ref wAz, ref vBx, ref vBy, ref vBz, ref wBx, ref wBy, ref wBz);

                lamN.Store(data.AsSpan(Pile.LAMN * stride + g, count));
                lamT1.Store(data.AsSpan(Pile.LAMT1 * stride + g, count));
                lamT2.Store(data.AsSpan(Pile.LAMT2 * stride + g, count));
            }

            // Scatter velocities of dynamic bodies back. Lanes of a batch hold different bodies; the static body is never written.
            vAx.Store(scratch.Slice(0, count)); vAy.Store(scratch.Slice(count, count)); vAz.Store(scratch.Slice(2 * count, count));
            wAx.Store(scratch.Slice(3 * count, count)); wAy.Store(scratch.Slice(4 * count, count)); wAz.Store(scratch.Slice(5 * count, count));
            vBx.Store(scratch.Slice(6 * count, count)); vBy.Store(scratch.Slice(7 * count, count)); vBz.Store(scratch.Slice(8 * count, count));
            wBx.Store(scratch.Slice(9 * count, count)); wBy.Store(scratch.Slice(10 * count, count)); wBz.Store(scratch.Slice(11 * count, count));
            for (int k = 0; k < count; k++)
            {
                int a = ia[g + k], b = ib[g + k];
                if (a < n)
                {
                    int o = a * Pile.BodyStride;
                    for (int f = 0; f < 6; f++) { body[o + f] = scratch[f * count + k]; }
                }
                if (b < n)
                {
                    int o = b * Pile.BodyStride;
                    for (int f = 0; f < 6; f++) { body[o + f] = scratch[(6 + f) * count + k]; }
                }
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T Ld<T>(Span<float> scratch, int field, int count) where T : struct, ILane<T> => T.Load(scratch.Slice(field * count, count));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T F<T>(float[] data, int field, int stride, int slot) where T : struct, ILane<T> => T.Load(data.AsSpan(field * stride + slot, T.Count));

    // One friction direction: relative velocity along the tangent, clamped by the friction cone, then applied.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T Tangent<T>(float[] data, int stride, int g, int fT, int fRa, int fRb, int fMass, T lam, T maxF, T negMaxF,
        T mA, T iA, T mB, T iB,
        ref T vAx, ref T vAy, ref T vAz, ref T wAx, ref T wAy, ref T wAz, ref T vBx, ref T vBy, ref T vBz, ref T wBx, ref T wBy, ref T wBz)
        where T : struct, ILane<T>
    {
        T tx = F<T>(data, fT, stride, g), ty = F<T>(data, fT + 1, stride, g), tz = F<T>(data, fT + 2, stride, g);
        T rax = F<T>(data, fRa, stride, g), ray = F<T>(data, fRa + 1, stride, g), raz = F<T>(data, fRa + 2, stride, g);
        T rbx = F<T>(data, fRb, stride, g), rby = F<T>(data, fRb + 1, stride, g), rbz = F<T>(data, fRb + 2, stride, g);
        T vt = (((vBx - vAx) * tx + (vBy - vAy) * ty) + (vBz - vAz) * tz)
            + ((wBx * rbx + wBy * rby) + wBz * rbz) - ((wAx * rax + wAy * ray) + wAz * raz);
        T impulse = Lane.Neg(F<T>(data, fMass, stride, g) * vt);
        T newLam = Lane.Max(Lane.Min(lam + impulse, maxF), negMaxF);
        impulse = newLam - lam;
        Apply(impulse, tx, ty, tz, rax, ray, raz, rbx, rby, rbz, mA, iA, mB, iB,
            ref vAx, ref vAy, ref vAz, ref wAx, ref wAy, ref wAz, ref vBx, ref vBy, ref vBz, ref wBx, ref wBy, ref wBz);
        return newLam;
    }

    // Applies an impulse of size lam along (dx, dy, dz) at the anchors whose cross products with the direction are ra and rb.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Apply<T>(T lam, T dx, T dy, T dz, T rax, T ray, T raz, T rbx, T rby, T rbz, T mA, T iA, T mB, T iB,
        ref T vAx, ref T vAy, ref T vAz, ref T wAx, ref T wAy, ref T wAz, ref T vBx, ref T vBy, ref T vBz, ref T wBx, ref T wBy, ref T wBz)
        where T : struct, ILane<T>
    {
        T px = lam * dx, py = lam * dy, pz = lam * dz;
        vAx = vAx - mA * px; vAy = vAy - mA * py; vAz = vAz - mA * pz;
        wAx = wAx - iA * (lam * rax); wAy = wAy - iA * (lam * ray); wAz = wAz - iA * (lam * raz);
        vBx = vBx + mB * px; vBy = vBy + mB * py; vBz = vBz + mB * pz;
        wBx = wBx + iB * (lam * rbx); wBy = wBy + iB * (lam * rby); wBz = wBz + iB * (lam * rbz);
    }
}
