using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// Layout of one group of up to eight plain revolute joints (no spring, motor or limit) in <see cref="Engine2D.JWide"/>: fields of eight floats, one per lane.
/// Anchors, the centre-of-mass offset and the soft coefficients come from the scalar joint preparation; the two point impulses are the only fields the solver changes.
/// </summary>
internal static class J2D
{
    public const int MA = 0, IA = 1, MB = 2, IB = 3;
    public const int RAx = 4, RAy = 5, RBx = 6, RBy = 7;
    public const int Dx = 8, Dy = 9;
    public const int Rigid = 10, Mass = 11, Impulse = 12; // bias rate times mass scale, mass scale and impulse scale of the rigid point row
    public const int P0 = 13, P1 = 14;
    public const int Fields = 15;
    public const int Lanes = 8;
    public const int GroupFloats = Fields * Lanes;
}

/// <summary>
/// The wide solver for plain revolute joints. Each lane does exactly what the scalar revolute solver does for such a joint (soft point row of Catto's soft step: the 2x2 point mass is rebuilt from the
/// anchors at the current rotation, solved, and applied), in the same order of operations, so a lane gives the bits of the scalar code at any width.
/// </summary>
internal static class JointKernel2D
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T W<T>(ReadOnlySpan<float> wide, int f, int lane0) where T : struct, ILane<T> => T.LoadAt(wide, f * 8 + lane0);

    /// <summary>
    /// Solves lanes <paramref name="lane0"/>.. <paramref name="lane0"/>+<paramref name="lanes"/>-1 of one group. The warm start and the solve are separate methods: one method holding every lane operation
    /// exhausts the JIT's inlining budget and the operators stay calls.
    /// </summary>
    public static void Run<T>(Engine2D e, int group, int lane0, int lanes, SolvePhase phase) where T : struct, ILane<T>
    {
        if (phase == SolvePhase.Restitution) { return; } // joints have no restitution pass
        var wide = e.JWide.AsSpan(group * J2D.GroupFloats, J2D.GroupFloats);
        var idx = e.JIdx.AsSpan(group * W2D.IdxPerGroup, W2D.IdxPerGroup);
        if (phase == SolvePhase.WarmStart) { RunWarm<T>(e.BodyState, wide, idx, lane0, lanes); }
        else { RunSolve<T>(e.BodyState, wide, idx, lane0, lanes, phase == SolvePhase.Solve); }
    }

    private static void RunWarm<T>(Span<float> state, Span<float> wide, ReadOnlySpan<int> idx, int lane0, int lanes) where T : struct, ILane<T>
    {
        int width = T.Count;
        T zero = T.Zero;
        for (int l0 = lane0; l0 < lane0 + lanes; l0 += width)
        {
            var ra = T.GatherRows(state, idx.Slice(l0, width));
            var rb = T.GatherRows(state, idx.Slice(8 + l0, width));
            T vAx = ra.R0, vAy = ra.R1, wA = ra.R2, vBx = rb.R0, vBy = rb.R1, wB = rb.R2;
            T mA = W<T>(wide, J2D.MA, l0), iA = W<T>(wide, J2D.IA, l0), mB = W<T>(wide, J2D.MB, l0), iB = W<T>(wide, J2D.IB, l0);
            // Anchors at the current rotation: the start-of-step anchors turned by the rotation accumulated this step.
            T rx = W<T>(wide, J2D.RAx, l0), ry = W<T>(wide, J2D.RAy, l0);
            T rAx = ra.R6 * rx - ra.R7 * ry, rAy = ra.R7 * rx + ra.R6 * ry;
            rx = W<T>(wide, J2D.RBx, l0); ry = W<T>(wide, J2D.RBy, l0);
            T rBx = rb.R6 * rx - rb.R7 * ry, rBy = rb.R7 * rx + rb.R6 * ry;
            T lx = W<T>(wide, J2D.P0, l0), ly = W<T>(wide, J2D.P1, l0);
            Lin(ref vAx, ref vAy, ref wA, ref vBx, ref vBy, ref wB, mA, iA, mB, iB, rAx, rAy, rBx, rBy, lx, ly);
            // The scalar solver also applies the (zero) sum of the spring, motor and limit impulses as an angular impulse; adding a signed zero is not always a no-op.
            wA = wA - iA * zero; wB = wB + iB * zero;
            T.Scatter3(state, idx.Slice(16 + l0, width), vAx, vAy, wA);
            T.Scatter3(state, idx.Slice(24 + l0, width), vBx, vBy, wB);
        }
    }

    private static void RunSolve<T>(Span<float> state, Span<float> wide, ReadOnlySpan<int> idx, int lane0, int lanes, bool useBias) where T : struct, ILane<T>
    {
        int width = T.Count;
        T zero = T.Zero;
        for (int l0 = lane0; l0 < lane0 + lanes; l0 += width)
        {
            var ra = T.GatherRows(state, idx.Slice(l0, width));
            var rb = T.GatherRows(state, idx.Slice(8 + l0, width));
            T vAx = ra.R0, vAy = ra.R1, wA = ra.R2, vBx = rb.R0, vBy = rb.R1, wB = rb.R2;
            T mA = W<T>(wide, J2D.MA, l0), iA = W<T>(wide, J2D.IA, l0), mB = W<T>(wide, J2D.MB, l0), iB = W<T>(wide, J2D.IB, l0);
            T rx = W<T>(wide, J2D.RAx, l0), ry = W<T>(wide, J2D.RAy, l0);
            T rAx = ra.R6 * rx - ra.R7 * ry, rAy = ra.R7 * rx + ra.R6 * ry;
            rx = W<T>(wide, J2D.RBx, l0); ry = W<T>(wide, J2D.RBy, l0);
            T rBx = rb.R6 * rx - rb.R7 * ry, rBy = rb.R7 * rx + rb.R6 * ry;
            T lx = W<T>(wide, J2D.P0, l0), ly = W<T>(wide, J2D.P1, l0);
            T dx = W<T>(wide, J2D.Dx, l0) + (rb.R4 - ra.R4), dy = W<T>(wide, J2D.Dy, l0) + (rb.R5 - ra.R5);
            T cx = dx + rBx - rAx, cy = dy + rBy - rAy;
            T k, ms, isc;
            if (useBias) { k = W<T>(wide, J2D.Rigid, l0); ms = W<T>(wide, J2D.Mass, l0); isc = W<T>(wide, J2D.Impulse, l0); }
            else { k = zero; ms = T.Broadcast(1f); isc = zero; }

            T dvx = (vBx - wB * rBy) - (vAx - wA * rAy);
            T dvy = (vBy + wB * rBx) - (vAy + wA * rAx);
            T k11 = mA + mB + iA * rAy * rAy + iB * rBy * rBy;
            T k12 = T.Negate(iA) * rAx * rAy - iB * rBx * rBy;
            T k22 = mA + mB + iA * rAx * rAx + iB * rBx * rBx;
            T qx = ms * dvx + k * cx, qy = ms * dvy + k * cy;
            T det = k11 * k22 - k12 * k12;
            T nonZero = Lane.Or(T.GreaterThan(det, zero), T.GreaterThan(zero, det));
            det = T.Select(nonZero, T.Broadcast(1f) / det, det);
            T bx = det * (k22 * qx - k12 * qy);
            T by = det * (k11 * qy - k12 * qx);
            T ix = T.Negate(bx) - isc * lx, iy = T.Negate(by) - isc * ly;
            lx = lx + ix; ly = ly + iy;
            Lin(ref vAx, ref vAy, ref wA, ref vBx, ref vBy, ref wB, mA, iA, mB, iB, rAx, rAy, rBx, rBy, ix, iy);
            lx.StoreAt(wide, J2D.P0 * 8 + l0); ly.StoreAt(wide, J2D.P1 * 8 + l0);
            T.Scatter3(state, idx.Slice(16 + l0, width), vAx, vAy, wA);
            T.Scatter3(state, idx.Slice(24 + l0, width), vBx, vBy, wB);
        }
    }

    // A linear impulse (px, py) at the anchors: pushes B along it and A against it.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Lin<T>(ref T vAx, ref T vAy, ref T wA, ref T vBx, ref T vBy, ref T wB, T mA, T iA, T mB, T iB, T rAx, T rAy, T rBx, T rBy, T px, T py) where T : struct, ILane<T>
    {
        vAx = vAx - mA * px; vAy = vAy - mA * py; wA = wA - iA * (rAx * py - rAy * px);
        vBx = vBx + mB * px; vBy = vBy + mB * py; wB = wB + iB * (rBx * py - rBy * px);
    }
}
