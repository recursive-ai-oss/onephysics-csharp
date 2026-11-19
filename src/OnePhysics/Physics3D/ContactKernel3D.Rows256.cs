// The rows of the generic kernel in ContactKernel3D.cs, expanded by hand for eight lanes on Vector256<float>; edit both together (the hash matrix's scalar and jit columns differ if they drift apart).
using System;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Physics3D;

internal static partial class ContactKernel3D
{
    /// <summary>
    /// The warm-start, solve and relax rows of one group of eight ordinary contacts (scalar inverse masses), written directly on <see cref="Vector256{T}"/>: the bodies' velocities stay in registers from
    /// the first row to the last; everything else (inertia, masses, normal, tangents, the group's coefficients) is read from memory where it is used, at constant offsets, so the
    /// method needs few registers and no spills. The generic kernel reaches the same rows through methods that each take the bodies by reference, because the runtime
    /// stops inlining lane operations in a method that large; here there are no lane calls to inline. Every expression is the generic kernel's, operation for operation in the same order, so the bits are
    /// those of every other width (the hash matrix compares them). <paramref name="a"/> and <paramref name="b"/> are the gathered bodies (the twenty vectors of BodyLanes in declaration order);
    /// impulses go to the group <paramref name="g"/>. r4 h06: the velocities go straight back to the bodies' packed velocity rows in <paramref name="pk"/> (<see cref="Engine3D.Pk"/>): the twelve velocity
    /// vectors are transposed in registers and every written body (scatter row numbers from <paramref name="scatter"/>: A's eight, then B's eight; −1 keeps the body's row) takes one 32-byte store.
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void Rows256(ref Vector256<float> a, ref Vector256<float> b, ref float g, Span<float> pk, ref int scatter, int lanes, int maxPts, SolvePhase phase, bool anyRolling, float inverseSubstepDt, float linearSlop, float pushOutVelocity)
    {
        Vector256<float> aVx = a, aVy = Mem.Row(ref a, 1), aVz = Mem.Row(ref a, 2), aWx = Mem.Row(ref a, 3), aWy = Mem.Row(ref a, 4), aWz = Mem.Row(ref a, 5);
        Vector256<float> bVx = b, bVy = Mem.Row(ref b, 1), bVz = Mem.Row(ref b, 2), bWx = Mem.Row(ref b, 3), bWy = Mem.Row(ref b, 4), bWz = Mem.Row(ref b, 5);
        Vector256<float> rx = Vector256.LoadUnsafe(ref g, W3D.RollImpulse * 8), ry = Vector256.LoadUnsafe(ref g, (W3D.RollImpulse + 1) * 8), rz = Vector256.LoadUnsafe(ref g, (W3D.RollImpulse + 2) * 8);
        Vector256<float> zero = Vector256<float>.Zero;

        if (phase == SolvePhase.WarmStart)
        {
            for (int i = 0; i < maxPts; i++)
            {
                ref float p = ref Mem.Row(ref g, W3D.Point0 + i * W3D.PointStride);
                Vector256<float> ni = Vector256.LoadUnsafe(ref p, W3D.NormalImpulse * 8), i1 = Vector256.LoadUnsafe(ref p, W3D.Tangent1Impulse * 8), i2 = Vector256.LoadUnsafe(ref p, W3D.Tangent2Impulse * 8);
                // n * normal + t1 * tangent1 + t2 * tangent2
                Vector256<float> px = Vector256.LoadUnsafe(ref g, (W3D.N + 0) * 8) * ni + Vector256.LoadUnsafe(ref g, (W3D.T1 + 0) * 8) * i1 + Vector256.LoadUnsafe(ref g, (W3D.T2 + 0) * 8) * i2, py = Vector256.LoadUnsafe(ref g, (W3D.N + 1) * 8) * ni + Vector256.LoadUnsafe(ref g, (W3D.T1 + 1) * 8) * i1 + Vector256.LoadUnsafe(ref g, (W3D.T2 + 1) * 8) * i2, pz = Vector256.LoadUnsafe(ref g, (W3D.N + 2) * 8) * ni + Vector256.LoadUnsafe(ref g, (W3D.T1 + 2) * 8) * i1 + Vector256.LoadUnsafe(ref g, (W3D.T2 + 2) * 8) * i2;
                Vector256<float> rax = Vector256.LoadUnsafe(ref p, W3D.ArmA * 8), ray = Vector256.LoadUnsafe(ref p, (W3D.ArmA + 1) * 8), raz = Vector256.LoadUnsafe(ref p, (W3D.ArmA + 2) * 8);
                Vector256<float> rbx = Vector256.LoadUnsafe(ref p, W3D.ArmB * 8), rby = Vector256.LoadUnsafe(ref p, (W3D.ArmB + 1) * 8), rbz = Vector256.LoadUnsafe(ref p, (W3D.ArmB + 2) * 8);
                Vector256<float> pm = Vector256.LoadUnsafe(ref p, W3D.Mask * 8);
                Vector256<float> cax = (ray * pz - raz * py), cay = (raz * px - rax * pz), caz = (rax * py - ray * px);
                Vector256<float> cbx = (rby * pz - rbz * py), cby = (rbz * px - rbx * pz), cbz = (rbx * py - rby * px);
                aVx = Vector256.ConditionalSelect(pm, aVx - px * Mem.Row(ref a, 19), aVx);
                aVy = Vector256.ConditionalSelect(pm, aVy - py * Mem.Row(ref a, 19), aVy);
                aVz = Vector256.ConditionalSelect(pm, aVz - pz * Mem.Row(ref a, 19), aVz);
                aWx = Vector256.ConditionalSelect(pm, aWx - ((Mem.Row(ref a, 13) * cax + Mem.Row(ref a, 14) * cay) + Mem.Row(ref a, 15) * caz), aWx);
                aWy = Vector256.ConditionalSelect(pm, aWy - ((Mem.Row(ref a, 14) * cax + Mem.Row(ref a, 16) * cay) + Mem.Row(ref a, 17) * caz), aWy);
                aWz = Vector256.ConditionalSelect(pm, aWz - ((Mem.Row(ref a, 15) * cax + Mem.Row(ref a, 17) * cay) + Mem.Row(ref a, 18) * caz), aWz);
                bVx = Vector256.ConditionalSelect(pm, bVx + px * Mem.Row(ref b, 19), bVx);
                bVy = Vector256.ConditionalSelect(pm, bVy + py * Mem.Row(ref b, 19), bVy);
                bVz = Vector256.ConditionalSelect(pm, bVz + pz * Mem.Row(ref b, 19), bVz);
                bWx = Vector256.ConditionalSelect(pm, bWx + ((Mem.Row(ref b, 13) * cbx + Mem.Row(ref b, 14) * cby) + Mem.Row(ref b, 15) * cbz), bWx);
                bWy = Vector256.ConditionalSelect(pm, bWy + ((Mem.Row(ref b, 14) * cbx + Mem.Row(ref b, 16) * cby) + Mem.Row(ref b, 17) * cbz), bWy);
                bWz = Vector256.ConditionalSelect(pm, bWz + ((Mem.Row(ref b, 15) * cbx + Mem.Row(ref b, 17) * cby) + Mem.Row(ref b, 18) * cbz), bWz);
            }

            // ApplyRoll
            Vector256<float> wax = aWx - ((Mem.Row(ref a, 13) * rx + Mem.Row(ref a, 14) * ry) + Mem.Row(ref a, 15) * rz), way = aWy - ((Mem.Row(ref a, 14) * rx + Mem.Row(ref a, 16) * ry) + Mem.Row(ref a, 17) * rz), waz = aWz - ((Mem.Row(ref a, 15) * rx + Mem.Row(ref a, 17) * ry) + Mem.Row(ref a, 18) * rz);
            Vector256<float> wbx = bWx + ((Mem.Row(ref b, 13) * rx + Mem.Row(ref b, 14) * ry) + Mem.Row(ref b, 15) * rz), wby = bWy + ((Mem.Row(ref b, 14) * rx + Mem.Row(ref b, 16) * ry) + Mem.Row(ref b, 17) * rz), wbz = bWz + ((Mem.Row(ref b, 15) * rx + Mem.Row(ref b, 17) * ry) + Mem.Row(ref b, 18) * rz);
            aWx = wax; aWy = way; aWz = waz; bWx = wbx; bWy = wby; bWz = wbz;
        }
        else
        {
            bool useBias = phase == SolvePhase.Solve;
            Vector256<float> two = Vector256.Create(2f), one = Vector256.Create(1f), signBit = Vector256.Create(-0f);

            if (anyRolling)
            {
                // RollingResistance
                Vector256<float> total = zero;
                for (int i = 0; i < maxPts; i++) { total = total + Vector256.LoadUnsafe(ref Mem.Row(ref g, W3D.Point0 + i * W3D.PointStride), W3D.NormalImpulse * 8); }
                Vector256<float> rollMask = Vector256.GreaterThan(Vector256.LoadUnsafe(ref g, W3D.Rolling * 8), zero);
                Vector256<float> maxRoll = Vector256.LoadUnsafe(ref g, W3D.Rolling * 8) * total;
                Vector256<float> dwx = bWx - aWx, dwy = bWy - aWy, dwz = bWz - aWz;
                Vector256<float> dn = dwx * Vector256.LoadUnsafe(ref g, (W3D.N + 0) * 8) + dwy * Vector256.LoadUnsafe(ref g, (W3D.N + 1) * 8) + dwz * Vector256.LoadUnsafe(ref g, (W3D.N + 2) * 8);
                dwx = dwx - Vector256.LoadUnsafe(ref g, (W3D.N + 0) * 8) * dn; dwy = dwy - Vector256.LoadUnsafe(ref g, (W3D.N + 1) * 8) * dn; dwz = dwz - Vector256.LoadUnsafe(ref g, (W3D.N + 2) * 8) * dn;
                Vector256<float> lx = Vector256.Xor(((Vector256.LoadUnsafe(ref g, (W3D.RollMass + 0) * 8) * dwx + Vector256.LoadUnsafe(ref g, (W3D.RollMass + 1) * 8) * dwy) + Vector256.LoadUnsafe(ref g, (W3D.RollMass + 2) * 8) * dwz), signBit), ly = Vector256.Xor(((Vector256.LoadUnsafe(ref g, (W3D.RollMass + 1) * 8) * dwx + Vector256.LoadUnsafe(ref g, (W3D.RollMass + 3) * 8) * dwy) + Vector256.LoadUnsafe(ref g, (W3D.RollMass + 4) * 8) * dwz), signBit), lz = Vector256.Xor(((Vector256.LoadUnsafe(ref g, (W3D.RollMass + 2) * 8) * dwx + Vector256.LoadUnsafe(ref g, (W3D.RollMass + 4) * 8) * dwy) + Vector256.LoadUnsafe(ref g, (W3D.RollMass + 5) * 8) * dwz), signBit);
                Vector256<float> ln = lx * Vector256.LoadUnsafe(ref g, (W3D.N + 0) * 8) + ly * Vector256.LoadUnsafe(ref g, (W3D.N + 1) * 8) + lz * Vector256.LoadUnsafe(ref g, (W3D.N + 2) * 8);
                lx = lx - Vector256.LoadUnsafe(ref g, (W3D.N + 0) * 8) * ln; ly = ly - Vector256.LoadUnsafe(ref g, (W3D.N + 1) * 8) * ln; lz = lz - Vector256.LoadUnsafe(ref g, (W3D.N + 2) * 8) * ln;
                Vector256<float> nrx = rx + lx, nry = ry + ly, nrz = rz + lz;
                Vector256<float> len2 = nrx * nrx + nry * nry + nrz * nrz;
                Vector256<float> over = Vector256.GreaterThan(len2, maxRoll * maxRoll);
                Vector256<float> scale = maxRoll / Vector256.Sqrt(len2);
                nrx = Vector256.ConditionalSelect(over, nrx * scale, nrx); nry = Vector256.ConditionalSelect(over, nry * scale, nry); nrz = Vector256.ConditionalSelect(over, nrz * scale, nrz);
                lx = nrx - rx; ly = nry - ry; lz = nrz - rz;
                Vector256<float> wax = aWx - ((Mem.Row(ref a, 13) * lx + Mem.Row(ref a, 14) * ly) + Mem.Row(ref a, 15) * lz), way = aWy - ((Mem.Row(ref a, 14) * lx + Mem.Row(ref a, 16) * ly) + Mem.Row(ref a, 17) * lz), waz = aWz - ((Mem.Row(ref a, 15) * lx + Mem.Row(ref a, 17) * ly) + Mem.Row(ref a, 18) * lz);
                Vector256<float> wbx = bWx + ((Mem.Row(ref b, 13) * lx + Mem.Row(ref b, 14) * ly) + Mem.Row(ref b, 15) * lz), wby = bWy + ((Mem.Row(ref b, 14) * lx + Mem.Row(ref b, 16) * ly) + Mem.Row(ref b, 17) * lz), wbz = bWz + ((Mem.Row(ref b, 15) * lx + Mem.Row(ref b, 17) * ly) + Mem.Row(ref b, 18) * lz);
                rx = Vector256.ConditionalSelect(rollMask, nrx, rx); ry = Vector256.ConditionalSelect(rollMask, nry, ry); rz = Vector256.ConditionalSelect(rollMask, nrz, rz);
                aWx = Vector256.ConditionalSelect(rollMask, wax, aWx); aWy = Vector256.ConditionalSelect(rollMask, way, aWy); aWz = Vector256.ConditionalSelect(rollMask, waz, aWz);
                bWx = Vector256.ConditionalSelect(rollMask, wbx, bWx); bWy = Vector256.ConditionalSelect(rollMask, wby, bWy); bWz = Vector256.ConditionalSelect(rollMask, wbz, bWz);
            }

            // FrictionPoint
            for (int i = 0; i < maxPts; i++)
            {
                ref float p = ref Mem.Row(ref g, W3D.Point0 + i * W3D.PointStride);
                Vector256<float> rax = Vector256.LoadUnsafe(ref p, W3D.ArmA * 8), ray = Vector256.LoadUnsafe(ref p, (W3D.ArmA + 1) * 8), raz = Vector256.LoadUnsafe(ref p, (W3D.ArmA + 2) * 8);
                Vector256<float> rbx = Vector256.LoadUnsafe(ref p, W3D.ArmB * 8), rby = Vector256.LoadUnsafe(ref p, (W3D.ArmB + 1) * 8), rbz = Vector256.LoadUnsafe(ref p, (W3D.ArmB + 2) * 8);
                Vector256<float> pm = Vector256.LoadUnsafe(ref p, W3D.Mask * 8);
                // (b.V + Cross(b.W, rB)) - (a.V + Cross(a.W, rA))
                Vector256<float> dvx = (bVx + (bWy * rbz - bWz * rby)) - (aVx + (aWy * raz - aWz * ray));
                Vector256<float> dvy = (bVy + (bWz * rbx - bWx * rbz)) - (aVy + (aWz * rax - aWx * raz));
                Vector256<float> dvz = (bVz + (bWx * rby - bWy * rbx)) - (aVz + (aWx * ray - aWy * rax));
                Vector256<float> l1 = Vector256.LoadUnsafe(ref p, W3D.Tangent1Impulse * 8), l2 = Vector256.LoadUnsafe(ref p, W3D.Tangent2Impulse * 8);
                Vector256<float> velocity1 = dvx * Vector256.LoadUnsafe(ref g, (W3D.T1 + 0) * 8) + dvy * Vector256.LoadUnsafe(ref g, (W3D.T1 + 1) * 8) + dvz * Vector256.LoadUnsafe(ref g, (W3D.T1 + 2) * 8);
                Vector256<float> n1 = l1 - Vector256.LoadUnsafe(ref p, W3D.Tangent1Mass * 8) * velocity1;
                Vector256<float> velocity2 = dvx * Vector256.LoadUnsafe(ref g, (W3D.T2 + 0) * 8) + dvy * Vector256.LoadUnsafe(ref g, (W3D.T2 + 1) * 8) + dvz * Vector256.LoadUnsafe(ref g, (W3D.T2 + 2) * 8);
                Vector256<float> n2 = l2 - Vector256.LoadUnsafe(ref p, W3D.Tangent2Mass * 8) * velocity2;
                Vector256<float> maxF = Vector256.LoadUnsafe(ref g, W3D.Friction * 8) * Vector256.LoadUnsafe(ref p, W3D.NormalImpulse * 8);
                Vector256<float> m2 = n1 * n1 + n2 * n2;
                Vector256<float> clamp = Vector256.GreaterThan(m2, maxF * maxF);
                Vector256<float> k = maxF / Vector256.Sqrt(m2);
                n1 = Vector256.ConditionalSelect(pm, Vector256.ConditionalSelect(clamp, n1 * k, n1), l1);
                n2 = Vector256.ConditionalSelect(pm, Vector256.ConditionalSelect(clamp, n2 * k, n2), l2);
                n1.StoreUnsafe(ref p, W3D.Tangent1Impulse * 8); n2.StoreUnsafe(ref p, W3D.Tangent2Impulse * 8);
                Vector256<float> d1 = n1 - l1, d2 = n2 - l2;
                Vector256<float> px = Vector256.LoadUnsafe(ref g, (W3D.T1 + 0) * 8) * d1 + Vector256.LoadUnsafe(ref g, (W3D.T2 + 0) * 8) * d2, py = Vector256.LoadUnsafe(ref g, (W3D.T1 + 1) * 8) * d1 + Vector256.LoadUnsafe(ref g, (W3D.T2 + 1) * 8) * d2, pz = Vector256.LoadUnsafe(ref g, (W3D.T1 + 2) * 8) * d1 + Vector256.LoadUnsafe(ref g, (W3D.T2 + 2) * 8) * d2;
                Vector256<float> cax = (ray * pz - raz * py), cay = (raz * px - rax * pz), caz = (rax * py - ray * px);
                Vector256<float> cbx = (rby * pz - rbz * py), cby = (rbz * px - rbx * pz), cbz = (rbx * py - rby * px);
                aVx = Vector256.ConditionalSelect(pm, aVx - px * Mem.Row(ref a, 19), aVx);
                aVy = Vector256.ConditionalSelect(pm, aVy - py * Mem.Row(ref a, 19), aVy);
                aVz = Vector256.ConditionalSelect(pm, aVz - pz * Mem.Row(ref a, 19), aVz);
                aWx = Vector256.ConditionalSelect(pm, aWx - ((Mem.Row(ref a, 13) * cax + Mem.Row(ref a, 14) * cay) + Mem.Row(ref a, 15) * caz), aWx);
                aWy = Vector256.ConditionalSelect(pm, aWy - ((Mem.Row(ref a, 14) * cax + Mem.Row(ref a, 16) * cay) + Mem.Row(ref a, 17) * caz), aWy);
                aWz = Vector256.ConditionalSelect(pm, aWz - ((Mem.Row(ref a, 15) * cax + Mem.Row(ref a, 17) * cay) + Mem.Row(ref a, 18) * caz), aWz);
                bVx = Vector256.ConditionalSelect(pm, bVx + px * Mem.Row(ref b, 19), bVx);
                bVy = Vector256.ConditionalSelect(pm, bVy + py * Mem.Row(ref b, 19), bVy);
                bVz = Vector256.ConditionalSelect(pm, bVz + pz * Mem.Row(ref b, 19), bVz);
                bWx = Vector256.ConditionalSelect(pm, bWx + ((Mem.Row(ref b, 13) * cbx + Mem.Row(ref b, 14) * cby) + Mem.Row(ref b, 15) * cbz), bWx);
                bWy = Vector256.ConditionalSelect(pm, bWy + ((Mem.Row(ref b, 14) * cbx + Mem.Row(ref b, 16) * cby) + Mem.Row(ref b, 17) * cbz), bWy);
                bWz = Vector256.ConditionalSelect(pm, bWz + ((Mem.Row(ref b, 15) * cbx + Mem.Row(ref b, 17) * cby) + Mem.Row(ref b, 18) * cbz), bWz);
            }

            // NormalPoint
            Vector256<float> invH = Vector256.Create(inverseSubstepDt), slop = Vector256.Create(linearSlop), negPush = Vector256.Create(-pushOutVelocity), speedCap = Vector256.Create(ImpulseSpeedCap);
            for (int i = 0; i < maxPts; i++)
            {
                ref float p = ref Mem.Row(ref g, W3D.Point0 + i * W3D.PointStride);
                Vector256<float> rax = Vector256.LoadUnsafe(ref p, W3D.ArmA * 8), ray = Vector256.LoadUnsafe(ref p, (W3D.ArmA + 1) * 8), raz = Vector256.LoadUnsafe(ref p, (W3D.ArmA + 2) * 8);
                Vector256<float> rbx = Vector256.LoadUnsafe(ref p, W3D.ArmB * 8), rby = Vector256.LoadUnsafe(ref p, (W3D.ArmB + 1) * 8), rbz = Vector256.LoadUnsafe(ref p, (W3D.ArmB + 2) * 8);
                Vector256<float> pm = Vector256.LoadUnsafe(ref p, W3D.Mask * 8);
                // Separation: (b.Dp - a.Dp) + (Rotate(qB, rB) - Rotate(qA, rA)), along the normal, plus the adjusted separation
                Vector256<float> qbtx = two * (Mem.Row(ref b, 10) * rbz - Mem.Row(ref b, 11) * rby), qbty = two * (Mem.Row(ref b, 11) * rbx - Mem.Row(ref b, 9) * rbz), qbtz = two * (Mem.Row(ref b, 9) * rby - Mem.Row(ref b, 10) * rbx);
                Vector256<float> qbx = rbx + Mem.Row(ref b, 12) * qbtx + (Mem.Row(ref b, 10) * qbtz - Mem.Row(ref b, 11) * qbty), qby = rby + Mem.Row(ref b, 12) * qbty + (Mem.Row(ref b, 11) * qbtx - Mem.Row(ref b, 9) * qbtz), qbz = rbz + Mem.Row(ref b, 12) * qbtz + (Mem.Row(ref b, 9) * qbty - Mem.Row(ref b, 10) * qbtx);
                Vector256<float> qatx = two * (Mem.Row(ref a, 10) * raz - Mem.Row(ref a, 11) * ray), qaty = two * (Mem.Row(ref a, 11) * rax - Mem.Row(ref a, 9) * raz), qatz = two * (Mem.Row(ref a, 9) * ray - Mem.Row(ref a, 10) * rax);
                Vector256<float> qax = rax + Mem.Row(ref a, 12) * qatx + (Mem.Row(ref a, 10) * qatz - Mem.Row(ref a, 11) * qaty), qay = ray + Mem.Row(ref a, 12) * qaty + (Mem.Row(ref a, 11) * qatx - Mem.Row(ref a, 9) * qatz), qaz = raz + Mem.Row(ref a, 12) * qatz + (Mem.Row(ref a, 9) * qaty - Mem.Row(ref a, 10) * qatx);
                Vector256<float> sx = (Mem.Row(ref b, 6) - Mem.Row(ref a, 6)) + (qbx - qax), sy = (Mem.Row(ref b, 7) - Mem.Row(ref a, 7)) + (qby - qay), sz = (Mem.Row(ref b, 8) - Mem.Row(ref a, 8)) + (qbz - qaz);
                Vector256<float> sep = sx * Vector256.LoadUnsafe(ref g, (W3D.N + 0) * 8) + sy * Vector256.LoadUnsafe(ref g, (W3D.N + 1) * 8) + sz * Vector256.LoadUnsafe(ref g, (W3D.N + 2) * 8) + Vector256.LoadUnsafe(ref p, W3D.AdjustedSeparation * 8);
                Vector256<float> apart = Vector256.GreaterThan(sep, zero);
                Vector256<float> biasApart = sep * invH;
                Vector256<float> bias, ms, isc;
                if (useBias)
                {
                    Vector256<float> x = sep + slop;
                    x = Vector256.ConditionalSelect(Vector256.GreaterThan(x, zero), zero, x);
                    Vector256<float> soft = Vector256.LoadUnsafe(ref g, W3D.BiasRate * 8) * x;
                    soft = Vector256.ConditionalSelect(Vector256.GreaterThan(negPush, soft), negPush, soft);
                    bias = Vector256.ConditionalSelect(apart, biasApart, soft);
                    ms = Vector256.ConditionalSelect(apart, one, Vector256.LoadUnsafe(ref g, W3D.MassScale * 8));
                    isc = Vector256.ConditionalSelect(apart, zero, Vector256.LoadUnsafe(ref g, W3D.ImpulseScale * 8));
                }
                else
                {
                    bias = Vector256.ConditionalSelect(apart, biasApart, zero);
                    ms = one;
                    isc = zero;
                }

                // (b.V + Cross(b.W, rB)) - (a.V + Cross(a.W, rA))
                Vector256<float> dvx = (bVx + (bWy * rbz - bWz * rby)) - (aVx + (aWy * raz - aWz * ray));
                Vector256<float> dvy = (bVy + (bWz * rbx - bWx * rbz)) - (aVy + (aWz * rax - aWx * raz));
                Vector256<float> dvz = (bVz + (bWx * rby - bWy * rbx)) - (aVz + (aWx * ray - aWy * rax));
                Vector256<float> vn = dvx * Vector256.LoadUnsafe(ref g, (W3D.N + 0) * 8) + dvy * Vector256.LoadUnsafe(ref g, (W3D.N + 1) * 8) + dvz * Vector256.LoadUnsafe(ref g, (W3D.N + 2) * 8);
                Vector256<float> old = Vector256.LoadUnsafe(ref p, W3D.NormalImpulse * 8), normalMass = Vector256.LoadUnsafe(ref p, W3D.NormalMass * 8);
                Vector256<float> lambda = Vector256.Xor(normalMass, signBit) * ms * (vn + bias) - isc * old;
                Vector256<float> nw = old + lambda;
                nw = Vector256.ConditionalSelect(Vector256.GreaterThan(nw, zero), nw, zero);
                Vector256<float> cap = normalMass * speedCap;
                nw = Vector256.ConditionalSelect(Vector256.GreaterThan(cap, nw), nw, cap);
                lambda = nw - old;
                Vector256.ConditionalSelect(pm, nw, old).StoreUnsafe(ref p, W3D.NormalImpulse * 8);
                Vector256<float> maxN = Vector256.LoadUnsafe(ref p, W3D.MaxNormalImpulse * 8);
                Vector256.ConditionalSelect(Vector256.BitwiseAnd(pm, Vector256.GreaterThan(nw, maxN)), nw, maxN).StoreUnsafe(ref p, W3D.MaxNormalImpulse * 8);
                Vector256<float> px = Vector256.LoadUnsafe(ref g, (W3D.N + 0) * 8) * lambda, py = Vector256.LoadUnsafe(ref g, (W3D.N + 1) * 8) * lambda, pz = Vector256.LoadUnsafe(ref g, (W3D.N + 2) * 8) * lambda;
                Vector256<float> cax = (ray * pz - raz * py), cay = (raz * px - rax * pz), caz = (rax * py - ray * px);
                Vector256<float> cbx = (rby * pz - rbz * py), cby = (rbz * px - rbx * pz), cbz = (rbx * py - rby * px);
                aVx = Vector256.ConditionalSelect(pm, aVx - px * Mem.Row(ref a, 19), aVx);
                aVy = Vector256.ConditionalSelect(pm, aVy - py * Mem.Row(ref a, 19), aVy);
                aVz = Vector256.ConditionalSelect(pm, aVz - pz * Mem.Row(ref a, 19), aVz);
                aWx = Vector256.ConditionalSelect(pm, aWx - ((Mem.Row(ref a, 13) * cax + Mem.Row(ref a, 14) * cay) + Mem.Row(ref a, 15) * caz), aWx);
                aWy = Vector256.ConditionalSelect(pm, aWy - ((Mem.Row(ref a, 14) * cax + Mem.Row(ref a, 16) * cay) + Mem.Row(ref a, 17) * caz), aWy);
                aWz = Vector256.ConditionalSelect(pm, aWz - ((Mem.Row(ref a, 15) * cax + Mem.Row(ref a, 17) * cay) + Mem.Row(ref a, 18) * caz), aWz);
                bVx = Vector256.ConditionalSelect(pm, bVx + px * Mem.Row(ref b, 19), bVx);
                bVy = Vector256.ConditionalSelect(pm, bVy + py * Mem.Row(ref b, 19), bVy);
                bVz = Vector256.ConditionalSelect(pm, bVz + pz * Mem.Row(ref b, 19), bVz);
                bWx = Vector256.ConditionalSelect(pm, bWx + ((Mem.Row(ref b, 13) * cbx + Mem.Row(ref b, 14) * cby) + Mem.Row(ref b, 15) * cbz), bWx);
                bWy = Vector256.ConditionalSelect(pm, bWy + ((Mem.Row(ref b, 14) * cbx + Mem.Row(ref b, 16) * cby) + Mem.Row(ref b, 17) * cbz), bWy);
                bWz = Vector256.ConditionalSelect(pm, bWz + ((Mem.Row(ref b, 15) * cbx + Mem.Row(ref b, 17) * cby) + Mem.Row(ref b, 18) * cbz), bWz);
            }

            rx.StoreUnsafe(ref g, W3D.RollImpulse * 8); ry.StoreUnsafe(ref g, (W3D.RollImpulse + 1) * 8); rz.StoreUnsafe(ref g, (W3D.RollImpulse + 2) * 8);
        }

        // Rows [v.xyz, w.xyz, 0, 0], as PackBody wrote them.
        Lanes256.ScatterRows(aVx, aVy, aVz, aWx, aWy, aWz, zero, zero, pk, ref scatter, lanes, Engine3D.PkStride, Engine3D.PkVelocity);
        Lanes256.ScatterRows(bVx, bVy, bVz, bWx, bWy, bWz, zero, zero, pk, ref Unsafe.Add(ref scatter, 8), lanes, Engine3D.PkStride, Engine3D.PkVelocity);
    }
}
