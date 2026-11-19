// The rows of the generic kernel in JointKernel3D.cs, expanded by hand for eight lanes on Vector256<float>; edit both together (the hash matrix's scalar and jit columns differ if they drift apart).
using System;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Physics3D;

internal static partial class JointKernel3D
{
    /// <summary>
    /// The warm-start pass (with the inverse of the block K) or one solve or relax pass of a group of eight point joints, written directly on <see cref="Vector256{T}"/> like the contact kernel's
    /// <c>Rows256</c>: the bodies' velocities and the turned anchors stay in registers, everything else (inertia, masses, rotation, displacement, the group's fields) is read from memory where it is
    /// used. Every expression is the generic kernel's (<c>WarmStart</c>, <c>BuildInverse</c>, <c>Matrix</c>, <c>SolveBlock</c>, <c>Anchors</c>, <c>Block</c>, <c>Apply</c>), operation for operation in
    /// the same order, so the bits are those of every other width. <paramref name="a"/> and <paramref name="b"/> are the gathered bodies (the twenty vectors of BodyLanes in declaration order); the
    /// inverse block or the accumulated impulse go to the group <paramref name="g"/>. The joint readback is not written here: the caller takes the generic rows when it is on.
    /// The velocities go straight back to the bodies' packed velocity rows in <paramref name="pk"/> (<see cref="Engine3D.Pk"/>), one 32-byte store per written body (scatter row numbers from
    /// <paramref name="scatter"/>: A's eight, then B's eight; −1 keeps the body's row), as in the contact kernel's rows.
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void Rows256(ref Vector256<float> a, ref Vector256<float> b, ref float g, Span<float> pk, ref int scatter, int lanes, bool warm, float massScale, float biasTerm, float impulseScale)
    {
        Vector256<float> aVx = a, aVy = Mem.Row(ref a, 1), aVz = Mem.Row(ref a, 2), aWx = Mem.Row(ref a, 3), aWy = Mem.Row(ref a, 4), aWz = Mem.Row(ref a, 5);
        Vector256<float> bVx = b, bVy = Mem.Row(ref b, 1), bVz = Mem.Row(ref b, 2), bWx = Mem.Row(ref b, 3), bWy = Mem.Row(ref b, 4), bWz = Mem.Row(ref b, 5);
        Vector256<float> two = Vector256.Create(2f);

        // rA = Rotate(a.Qx, a.Qy, a.Qz, a.Qw, RA, two) and rB likewise: t = two * (u x v); v + w * t + (u x t). Rows 9 to 12 of a body are its rotation x y z w.
        Vector256<float> rax, ray, raz, rbx, rby, rbz;
        {
            Vector256<float> vx = Mem.LoadLanes(ref g, J3D.RA), vy = Mem.LoadLanes(ref g, J3D.RA + 1), vz = Mem.LoadLanes(ref g, J3D.RA + 2);
            Vector256<float> tx = two * (Mem.Row(ref a, 10) * vz - Mem.Row(ref a, 11) * vy), ty = two * (Mem.Row(ref a, 11) * vx - Mem.Row(ref a, 9) * vz), tz = two * (Mem.Row(ref a, 9) * vy - Mem.Row(ref a, 10) * vx);
            rax = vx + Mem.Row(ref a, 12) * tx + (Mem.Row(ref a, 10) * tz - Mem.Row(ref a, 11) * ty);
            ray = vy + Mem.Row(ref a, 12) * ty + (Mem.Row(ref a, 11) * tx - Mem.Row(ref a, 9) * tz);
            raz = vz + Mem.Row(ref a, 12) * tz + (Mem.Row(ref a, 9) * ty - Mem.Row(ref a, 10) * tx);
        }

        {
            Vector256<float> vx = Mem.LoadLanes(ref g, J3D.RB), vy = Mem.LoadLanes(ref g, J3D.RB + 1), vz = Mem.LoadLanes(ref g, J3D.RB + 2);
            Vector256<float> tx = two * (Mem.Row(ref b, 10) * vz - Mem.Row(ref b, 11) * vy), ty = two * (Mem.Row(ref b, 11) * vx - Mem.Row(ref b, 9) * vz), tz = two * (Mem.Row(ref b, 9) * vy - Mem.Row(ref b, 10) * vx);
            rbx = vx + Mem.Row(ref b, 12) * tx + (Mem.Row(ref b, 10) * tz - Mem.Row(ref b, 11) * ty);
            rby = vy + Mem.Row(ref b, 12) * ty + (Mem.Row(ref b, 11) * tx - Mem.Row(ref b, 9) * tz);
            rbz = vz + Mem.Row(ref b, 12) * tz + (Mem.Row(ref b, 9) * ty - Mem.Row(ref b, 10) * tx);
        }

        // The impulse applied at the anchors: the accumulated one (warm start) or this pass's step.
        Vector256<float> px = Mem.LoadLanes(ref g, J3D.Impulse), py = Mem.LoadLanes(ref g, J3D.Impulse + 1), pz = Mem.LoadLanes(ref g, J3D.Impulse + 2);
        if (!warm)
        {
            Vector256<float> ms = Vector256.Create(massScale), kk = Vector256.Create(biasTerm), isc = Vector256.Create(impulseScale), signBit = Vector256.Create(-0f);
            // Anchors: d = D + (b.Dp - a.Dp), c = (d + rB) - rA, dv = (b.V + Cross(b.W, rB)) - (a.V + Cross(a.W, rA)). Rows 6 to 8 of a body are its displacement.
            Vector256<float> cx = ((Mem.LoadLanes(ref g, J3D.D) + (Mem.Row(ref b, 6) - Mem.Row(ref a, 6))) + rbx) - rax;
            Vector256<float> cy = ((Mem.LoadLanes(ref g, J3D.D + 1) + (Mem.Row(ref b, 7) - Mem.Row(ref a, 7))) + rby) - ray;
            Vector256<float> cz = ((Mem.LoadLanes(ref g, J3D.D + 2) + (Mem.Row(ref b, 8) - Mem.Row(ref a, 8))) + rbz) - raz;
            Vector256<float> dvx = (bVx + (bWy * rbz - bWz * rby)) - (aVx + (aWy * raz - aWz * ray));
            Vector256<float> dvy = (bVy + (bWz * rbx - bWx * rbz)) - (aVy + (aWz * rax - aWx * raz));
            Vector256<float> dvz = (bVz + (bWx * rby - bWy * rbx)) - (aVz + (aWx * ray - aWy * rax));
            // Block: step = -K⁻¹ (ms dv + kk c) - isc imp; imp += step.
            Vector256<float> r0 = ms * dvx + kk * cx, r1 = ms * dvy + kk * cy, r2 = ms * dvz + kk * cz;
            Vector256<float> x0 = Mem.LoadLanes(ref g, J3D.KInv) * r0 + Mem.LoadLanes(ref g, J3D.KInv + 1) * r1 + Mem.LoadLanes(ref g, J3D.KInv + 2) * r2;
            Vector256<float> x1 = Mem.LoadLanes(ref g, J3D.KInv + 1) * r0 + Mem.LoadLanes(ref g, J3D.KInv + 3) * r1 + Mem.LoadLanes(ref g, J3D.KInv + 4) * r2;
            Vector256<float> x2 = Mem.LoadLanes(ref g, J3D.KInv + 2) * r0 + Mem.LoadLanes(ref g, J3D.KInv + 4) * r1 + Mem.LoadLanes(ref g, J3D.KInv + 5) * r2;
            Vector256<float> sx = Vector256.Xor(x0, signBit) - isc * px, sy = Vector256.Xor(x1, signBit) - isc * py, sz = Vector256.Xor(x2, signBit) - isc * pz;
            Mem.StoreLanes(ref g, J3D.Impulse, px + sx); Mem.StoreLanes(ref g, J3D.Impulse + 1, py + sy); Mem.StoreLanes(ref g, J3D.Impulse + 2, pz + sz);
            px = sx; py = sy; pz = sz;
        }

        // Apply: a.V -= p * a.InvMass, a.W -= a.I (rA x p), b.V += p * b.InvMass, b.W += b.I (rB x p). Rows 13 to 18 of a body are its inverse inertia xx xy xz yy yz zz, row 19 its inverse mass.
        {
            Vector256<float> cax = ray * pz - raz * py, cay = raz * px - rax * pz, caz = rax * py - ray * px;
            Vector256<float> cbx = rby * pz - rbz * py, cby = rbz * px - rbx * pz, cbz = rbx * py - rby * px;
            aVx = aVx - px * Mem.Row(ref a, 19); aVy = aVy - py * Mem.Row(ref a, 19); aVz = aVz - pz * Mem.Row(ref a, 19);
            aWx = aWx - (Mem.Row(ref a, 13) * cax + Mem.Row(ref a, 14) * cay + Mem.Row(ref a, 15) * caz);
            aWy = aWy - (Mem.Row(ref a, 14) * cax + Mem.Row(ref a, 16) * cay + Mem.Row(ref a, 17) * caz);
            aWz = aWz - (Mem.Row(ref a, 15) * cax + Mem.Row(ref a, 17) * cay + Mem.Row(ref a, 18) * caz);
            bVx = bVx + px * Mem.Row(ref b, 19); bVy = bVy + py * Mem.Row(ref b, 19); bVz = bVz + pz * Mem.Row(ref b, 19);
            bWx = bWx + (Mem.Row(ref b, 13) * cbx + Mem.Row(ref b, 14) * cby + Mem.Row(ref b, 15) * cbz);
            bWy = bWy + (Mem.Row(ref b, 14) * cbx + Mem.Row(ref b, 16) * cby + Mem.Row(ref b, 17) * cbz);
            bWz = bWz + (Mem.Row(ref b, 15) * cbx + Mem.Row(ref b, 17) * cby + Mem.Row(ref b, 18) * cbz);
        }

        if (warm)
        {
            // BuildInverse: K = (mA + mB) 1 + [rA]ᵀ IA [rA] + [rB]ᵀ IB [rB] (Matrix, AddInertiaTerm), then its closed-form inverse.
            Vector256<float> zero = Vector256<float>.Zero;
            Vector256<float> k00 = Mem.Row(ref a, 19) + Mem.Row(ref b, 19), k11 = k00, k22 = k00, k01 = zero, k02 = zero, k12 = zero;
            {
                Vector256<float> xx = rax * rax, yy = ray * ray, zz = raz * raz, xy = rax * ray, xz = rax * raz, yz = ray * raz;
                Vector256<float> eyz = Mem.Row(ref a, 17) * yz, cxz = Mem.Row(ref a, 15) * xz, bxy = Mem.Row(ref a, 14) * xy;
                k00 = k00 + (Mem.Row(ref a, 16) * zz - (eyz + eyz) + Mem.Row(ref a, 18) * yy);
                k11 = k11 + (Mem.Row(ref a, 13) * zz - (cxz + cxz) + Mem.Row(ref a, 18) * xx);
                k22 = k22 + (Mem.Row(ref a, 13) * yy - (bxy + bxy) + Mem.Row(ref a, 16) * xx);
                k01 = k01 + (Mem.Row(ref a, 17) * xz + Mem.Row(ref a, 15) * yz - Mem.Row(ref a, 14) * zz - Mem.Row(ref a, 18) * xy);
                k02 = k02 + (Mem.Row(ref a, 14) * yz + Mem.Row(ref a, 17) * xy - Mem.Row(ref a, 16) * xz - Mem.Row(ref a, 15) * yy);
                k12 = k12 + (Mem.Row(ref a, 14) * xz + Mem.Row(ref a, 15) * xy - Mem.Row(ref a, 13) * yz - Mem.Row(ref a, 17) * xx);
            }

            {
                Vector256<float> xx = rbx * rbx, yy = rby * rby, zz = rbz * rbz, xy = rbx * rby, xz = rbx * rbz, yz = rby * rbz;
                Vector256<float> eyz = Mem.Row(ref b, 17) * yz, cxz = Mem.Row(ref b, 15) * xz, bxy = Mem.Row(ref b, 14) * xy;
                k00 = k00 + (Mem.Row(ref b, 16) * zz - (eyz + eyz) + Mem.Row(ref b, 18) * yy);
                k11 = k11 + (Mem.Row(ref b, 13) * zz - (cxz + cxz) + Mem.Row(ref b, 18) * xx);
                k22 = k22 + (Mem.Row(ref b, 13) * yy - (bxy + bxy) + Mem.Row(ref b, 16) * xx);
                k01 = k01 + (Mem.Row(ref b, 17) * xz + Mem.Row(ref b, 15) * yz - Mem.Row(ref b, 14) * zz - Mem.Row(ref b, 18) * xy);
                k02 = k02 + (Mem.Row(ref b, 14) * yz + Mem.Row(ref b, 17) * xy - Mem.Row(ref b, 16) * xz - Mem.Row(ref b, 15) * yy);
                k12 = k12 + (Mem.Row(ref b, 14) * xz + Mem.Row(ref b, 15) * xy - Mem.Row(ref b, 13) * yz - Mem.Row(ref b, 17) * xx);
            }

            Vector256<float> c00 = k11 * k22 - k12 * k12, c01 = k02 * k12 - k01 * k22, c02 = k01 * k12 - k02 * k11;
            Vector256<float> det = k00 * c00 + k01 * c01 + k02 * c02;
            Vector256<float> positive = Vector256.GreaterThan(det, zero);
            Vector256<float> nonZero = Vector256.ConditionalSelect(positive, positive, Vector256.GreaterThan(zero, det));
            Vector256<float> inv = Vector256.ConditionalSelect(nonZero, Vector256.Create(1f) / det, zero);
            Vector256<float> c11 = k00 * k22 - k02 * k02, c12 = k01 * k02 - k00 * k12, c22 = k00 * k11 - k01 * k01;
            Mem.StoreLanes(ref g, J3D.KInv, inv * c00); Mem.StoreLanes(ref g, J3D.KInv + 1, inv * c01); Mem.StoreLanes(ref g, J3D.KInv + 2, inv * c02);
            Mem.StoreLanes(ref g, J3D.KInv + 3, inv * c11); Mem.StoreLanes(ref g, J3D.KInv + 4, inv * c12); Mem.StoreLanes(ref g, J3D.KInv + 5, inv * c22);
        }

        // Rows [v.xyz, w.xyz, 0, 0], as PackBody wrote them.
        Vector256<float> z = Vector256<float>.Zero;
        Lanes256.ScatterRows(aVx, aVy, aVz, aWx, aWy, aWz, z, z, pk, ref scatter, lanes, Engine3D.PkStride, Engine3D.PkVelocity);
        Lanes256.ScatterRows(bVx, bVy, bVz, bWx, bWy, bWz, z, z, pk, ref Mem.Add(ref scatter, 8), lanes, Engine3D.PkStride, Engine3D.PkVelocity);
    }
}
