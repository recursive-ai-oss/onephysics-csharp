using System;
using System.Runtime.Intrinsics;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    // Extended position-based dynamics in small steps: Müller et al. 2007 (position based dynamics), Macklin, Müller, Chentanez 2016 (XPBD: compliance and the multiplier update),
    // Macklin et al. 2019 (small steps: many substeps with a single Gauss–Seidel pass each beats many iterations of one large step).
    // The kernels work on the float32 copies X / XP of the positions (relative to the body's origin for the step). Constraints are grouped by colour (SoftBodyColoring): the colours are solved one
    // after another, the constraints of a colour in any order or at once in vector lanes, and every path does the same float operations in the same order per constraint, so the bits are the same.

    /// <summary>Whether constraint kernels may use <see cref="Vector128"/> lanes. The lanes and the scalar loops give the same bits; this switch exists so a test can compare them.</summary>
    internal static bool SoftLanes = Vector128.IsHardwareAccelerated;

    private static void PredictSoft(SoftBodyData d, float h, Vector3F g)
    {
        float gs = d.Def.GravityScale;
        float damp = 1f / (1f + d.Def.LinearDamping * h);
        float gx = g.X * gs * h, gy = g.Y * gs * h, gz = g.Z * gs * h;
        var x = d.X; var xp = d.XP; var vel = d.Vel; var inv = d.InvMass;
        for (int i = 0; i < d.N; i++)
        {
            int k = 3 * i;
            xp[k] = x[k]; xp[k + 1] = x[k + 1]; xp[k + 2] = x[k + 2];
            if (inv[i] == 0f) { continue; }
            float vx = (vel[k] + gx) * damp, vy = (vel[k + 1] + gy) * damp, vz = (vel[k + 2] + gz) * damp;
            vel[k] = vx; vel[k + 1] = vy; vel[k + 2] = vz;
            x[k] += vx * h; x[k + 1] += vy * h; x[k + 2] += vz * h;
        }
    }

    private void FinishSubstep(SoftBodyData d, float invH)
    {
        var x = d.X; var xp = d.XP; var vel = d.Vel; var inv = d.InvMass;
        float vmax = Opt.MaxLinearSpeed;
        float vmax2 = vmax * vmax;
        for (int i = 0; i < d.N; i++)
        {
            int k = 3 * i;
            if (inv[i] == 0f) { vel[k] = 0f; vel[k + 1] = 0f; vel[k + 2] = 0f; continue; }
            float vx = (x[k] - xp[k]) * invH, vy = (x[k + 1] - xp[k + 1]) * invH, vz = (x[k + 2] - xp[k + 2]) * invH;
            float s2 = vx * vx + vy * vy + vz * vz;
            if (s2 > vmax2)
            {
                float f = vmax / float.Sqrt(s2);
                vx *= f; vy *= f; vz *= f;
            }

            vel[k] = vx; vel[k + 1] = vy; vel[k + 2] = vz;
        }
    }

    // ------------------------------------------------------------------ constraints

    /// <summary>Solves constraints <c>first..end-1</c> of one kind (0 edge, 1 bend, 2 tetrahedron, 3 pressure) of a body; they belong to one colour, or to the overflow set (<paramref name="lanes"/> false), whose constraints share particles and are solved one after another in index order.</summary>
    internal static void SolveRange(SoftBodyData d, int kind, int first, int end, float h, bool lanes = true)
    {
        var def = d.Def;
        float ih2 = 1f / (h * h), ih = 1f / h;
        var x = d.X; var xp = d.XP; var inv = d.InvMass; var lam = d.Lambda;
        int ne = d.SEdgeA.Length, nh = d.SBendS.Length;
        switch (kind)
        {
            case 0:
            {
                float alpha = def.EdgeCompliance * ih2, gamma = def.EdgeDamping * ih;
                int e = first;
                if (SoftLanes && lanes)
                {
                    for (; e + 4 <= end; e += 4) { EdgeLanes(x, xp, inv, d.SEdgeA, d.SEdgeB, d.SEdgeRest, lam, e, alpha, gamma); }
                }

                for (; e < end; e++) { XpbdDistance(x, xp, inv, d.SEdgeA[e], d.SEdgeB[e], d.SEdgeRest[e], alpha, gamma, ref lam[e]); }
                break;
            }
            case 1:
            {
                float alpha = def.BendCompliance * ih2, gamma = def.BendDamping * ih;
                int e = first;
                if (SoftLanes && lanes)
                {
                    for (; e + 4 <= end; e += 4) { BendLanes(d, e, alpha, gamma, ne); }
                }

                for (; e < end; e++) { XpbdBend(d, e, alpha, gamma, ref lam[ne + e]); }
                break;
            }
            case 2:
            {
                float alpha = def.VolumeCompliance * ih2, gamma = def.VolumeDamping * ih;
                for (int e = first; e < end; e++) { XpbdVolume(d, e, alpha, gamma, ref lam[ne + nh + e]); }
                break;
            }
            default:
            {
                float alpha = def.VolumeCompliance * ih2, gamma = def.VolumeDamping * ih;
                XpbdPressure(d, alpha, gamma, ref lam[ne + nh + d.STetRest.Length]);
                break;
            }
        }
    }

    // Distance constraint C = |xa - xb| - rest, with the XPBD multiplier update and Rayleigh-style damping (Macklin et al. 2016, eq. 18 with gamma = damping time / substep).
    private static void XpbdDistance(float[] x, float[] xp, float[] inv, int a, int b, float rest, float alpha, float gamma, ref float lambda)
    {
        float wa = inv[a], wb = inv[b];
        float wsum = wa + wb;
        if (wsum == 0f) { return; }
        int ka = 3 * a, kb = 3 * b;
        float dx = x[ka] - x[kb], dy = x[ka + 1] - x[kb + 1], dz = x[ka + 2] - x[kb + 2];
        float len = float.Sqrt(dx * dx + dy * dy + dz * dz);
        if (!(len > 1e-9f)) { return; }
        float il = 1f / len;
        float nx = dx * il, ny = dy * il, nz = dz * il;
        float c = len - rest;
        float damp = 0f;
        if (gamma != 0f)
        {
            damp = gamma * (nx * ((x[ka] - xp[ka]) - (x[kb] - xp[kb])) + ny * ((x[ka + 1] - xp[ka + 1]) - (x[kb + 1] - xp[kb + 1])) + nz * ((x[ka + 2] - xp[ka + 2]) - (x[kb + 2] - xp[kb + 2])));
        }

        float dl = (-c - alpha * lambda - damp) / ((1f + gamma) * wsum + alpha);
        lambda += dl;
        float sa = wa * dl, sb = wb * dl;
        x[ka] += nx * sa; x[ka + 1] += ny * sa; x[ka + 2] += nz * sa;
        x[kb] -= nx * sb; x[kb + 1] -= ny * sb; x[kb + 2] -= nz * sb;
    }

    // Four edges of one colour at once: the same operations as XpbdDistance on each lane (selects instead of early returns), so the result is the same bits.
    private static void EdgeLanes(float[] x, float[] xp, float[] inv, int[] ea, int[] eb, float[] rest, float[] lam, int e, float alpha, float gamma)
    {
        int a0 = 3 * ea[e], a1 = 3 * ea[e + 1], a2 = 3 * ea[e + 2], a3 = 3 * ea[e + 3];
        int b0 = 3 * eb[e], b1 = 3 * eb[e + 1], b2 = 3 * eb[e + 2], b3 = 3 * eb[e + 3];
        var wa = Vector128.Create(inv[ea[e]], inv[ea[e + 1]], inv[ea[e + 2]], inv[ea[e + 3]]);
        var wb = Vector128.Create(inv[eb[e]], inv[eb[e + 1]], inv[eb[e + 2]], inv[eb[e + 3]]);
        var wsum = wa + wb;
        var axv = Vector128.Create(x[a0], x[a1], x[a2], x[a3]);
        var ayv = Vector128.Create(x[a0 + 1], x[a1 + 1], x[a2 + 1], x[a3 + 1]);
        var azv = Vector128.Create(x[a0 + 2], x[a1 + 2], x[a2 + 2], x[a3 + 2]);
        var bxv = Vector128.Create(x[b0], x[b1], x[b2], x[b3]);
        var byv = Vector128.Create(x[b0 + 1], x[b1 + 1], x[b2 + 1], x[b3 + 1]);
        var bzv = Vector128.Create(x[b0 + 2], x[b1 + 2], x[b2 + 2], x[b3 + 2]);
        var dx = axv - bxv; var dy = ayv - byv; var dz = azv - bzv;
        var len = Vector128.Sqrt(dx * dx + dy * dy + dz * dz);
        var valid = Vector128.AndNot(Vector128.GreaterThan(len, Vector128.Create(1e-9f)), Vector128.Equals(wsum, Vector128<float>.Zero));
        var il = Vector128<float>.One / len;
        var nx = dx * il; var ny = dy * il; var nz = dz * il;
        var c = len - Vector128.Create(rest[e], rest[e + 1], rest[e + 2], rest[e + 3]);
        var damp = Vector128<float>.Zero;
        if (gamma != 0f)
        {
            var uxv = (axv - Vector128.Create(xp[a0], xp[a1], xp[a2], xp[a3])) - (bxv - Vector128.Create(xp[b0], xp[b1], xp[b2], xp[b3]));
            var uyv = (ayv - Vector128.Create(xp[a0 + 1], xp[a1 + 1], xp[a2 + 1], xp[a3 + 1])) - (byv - Vector128.Create(xp[b0 + 1], xp[b1 + 1], xp[b2 + 1], xp[b3 + 1]));
            var uzv = (azv - Vector128.Create(xp[a0 + 2], xp[a1 + 2], xp[a2 + 2], xp[a3 + 2])) - (bzv - Vector128.Create(xp[b0 + 2], xp[b1 + 2], xp[b2 + 2], xp[b3 + 2]));
            damp = Vector128.Create(gamma) * (nx * uxv + ny * uyv + nz * uzv);
        }

        var lv = Vector128.Create(lam[e], lam[e + 1], lam[e + 2], lam[e + 3]);
        var dl = (-c - Vector128.Create(alpha) * lv - damp) / (Vector128.Create(1f + gamma) * wsum + Vector128.Create(alpha));
        var sa = wa * dl; var sb = wb * dl;
        var lnew = Vector128.ConditionalSelect(valid, lv + dl, lv);
        var axn = Vector128.ConditionalSelect(valid, axv + nx * sa, axv);
        var ayn = Vector128.ConditionalSelect(valid, ayv + ny * sa, ayv);
        var azn = Vector128.ConditionalSelect(valid, azv + nz * sa, azv);
        var bxn = Vector128.ConditionalSelect(valid, bxv - nx * sb, bxv);
        var byn = Vector128.ConditionalSelect(valid, byv - ny * sb, byv);
        var bzn = Vector128.ConditionalSelect(valid, bzv - nz * sb, bzv);
        // Constraints of one colour share no particle, so the stores cannot overlap.
        lam[e] = lnew.GetElement(0); lam[e + 1] = lnew.GetElement(1); lam[e + 2] = lnew.GetElement(2); lam[e + 3] = lnew.GetElement(3);
        x[a0] = axn.GetElement(0); x[a0 + 1] = ayn.GetElement(0); x[a0 + 2] = azn.GetElement(0);
        x[a1] = axn.GetElement(1); x[a1 + 1] = ayn.GetElement(1); x[a1 + 2] = azn.GetElement(1);
        x[a2] = axn.GetElement(2); x[a2 + 1] = ayn.GetElement(2); x[a2 + 2] = azn.GetElement(2);
        x[a3] = axn.GetElement(3); x[a3 + 1] = ayn.GetElement(3); x[a3 + 2] = azn.GetElement(3);
        x[b0] = bxn.GetElement(0); x[b0 + 1] = byn.GetElement(0); x[b0 + 2] = bzn.GetElement(0);
        x[b1] = bxn.GetElement(1); x[b1 + 1] = byn.GetElement(1); x[b1 + 2] = bzn.GetElement(1);
        x[b2] = bxn.GetElement(2); x[b2 + 1] = byn.GetElement(2); x[b2 + 2] = bzn.GetElement(2);
        x[b3] = bxn.GetElement(3); x[b3 + 1] = byn.GetElement(3); x[b3 + 2] = bzn.GetElement(3);
    }

    // Isometric bend constraint C = s (|m|^2 - |m0|^2), m = sum K_i x_i, so the gradient is 2 s K_i m (see SoftBodyData.BendCoefficients).
    private static void XpbdBend(SoftBodyData d, int e, float alpha, float gamma, ref float lambda)
    {
        var x = d.X; var xp = d.XP; var inv = d.InvMass;
        int a = d.SBendIdx[4 * e], b = d.SBendIdx[4 * e + 1], c = d.SBendIdx[4 * e + 2], dd = d.SBendIdx[4 * e + 3];
        int ka = 3 * a, kb = 3 * b, kc = 3 * c, kd = 3 * dd;
        float k1 = d.SBendK[4 * e + 1], k2 = d.SBendK[4 * e + 2], k3 = d.SBendK[4 * e + 3];
        float k0 = d.SBendK[4 * e];
        float s = d.SBendS[e];
        float mx = k1 * (x[kb] - x[ka]) + k2 * (x[kc] - x[ka]) + k3 * (x[kd] - x[ka]);
        float my = k1 * (x[kb + 1] - x[ka + 1]) + k2 * (x[kc + 1] - x[ka + 1]) + k3 * (x[kd + 1] - x[ka + 1]);
        float mz = k1 * (x[kb + 2] - x[ka + 2]) + k2 * (x[kc + 2] - x[ka + 2]) + k3 * (x[kd + 2] - x[ka + 2]);
        float m2 = mx * mx + my * my + mz * mz;
        float cv = s * (m2 - d.SBendM0[e]);
        float f = 2f * s;
        float wa = inv[a], wb = inv[b], wc = inv[c], wd = inv[dd];
        float wsum = f * f * m2 * (wa * k0 * k0 + wb * k1 * k1 + wc * k2 * k2 + wd * k3 * k3);
        float damp = 0f;
        if (gamma != 0f)
        {
            float dxm = k0 * (x[ka] - xp[ka]) + k1 * (x[kb] - xp[kb]) + k2 * (x[kc] - xp[kc]) + k3 * (x[kd] - xp[kd]);
            float dym = k0 * (x[ka + 1] - xp[ka + 1]) + k1 * (x[kb + 1] - xp[kb + 1]) + k2 * (x[kc + 1] - xp[kc + 1]) + k3 * (x[kd + 1] - xp[kd + 1]);
            float dzm = k0 * (x[ka + 2] - xp[ka + 2]) + k1 * (x[kb + 2] - xp[kb + 2]) + k2 * (x[kc + 2] - xp[kc + 2]) + k3 * (x[kd + 2] - xp[kd + 2]);
            damp = gamma * f * (mx * dxm + my * dym + mz * dzm);
        }

        float denom = (1f + gamma) * wsum + alpha;
        if (!(denom > 1e-30f)) { return; }
        float dl = (-cv - alpha * lambda - damp) / denom;
        lambda += dl;
        float t;
        t = wa * dl * f * k0; x[ka] += mx * t; x[ka + 1] += my * t; x[ka + 2] += mz * t;
        t = wb * dl * f * k1; x[kb] += mx * t; x[kb + 1] += my * t; x[kb + 2] += mz * t;
        t = wc * dl * f * k2; x[kc] += mx * t; x[kc + 1] += my * t; x[kc + 2] += mz * t;
        t = wd * dl * f * k3; x[kd] += mx * t; x[kd + 1] += my * t; x[kd + 2] += mz * t;
    }

    private static Vector128<float> Gather(float[] a, int i0, int i1, int i2, int i3) => Vector128.Create(a[i0], a[i1], a[i2], a[i3]);

    // Four bend constraints of one colour at once: the operations of XpbdBend on each lane, in the same order.
    private static void BendLanes(SoftBodyData d, int e, float alpha, float gamma, int ne)
    {
        var x = d.X; var xp = d.XP; var inv = d.InvMass; var bi = d.SBendIdx; var bk = d.SBendK;
        int a0 = bi[4 * e], a1 = bi[4 * e + 4], a2 = bi[4 * e + 8], a3 = bi[4 * e + 12];
        int b0 = bi[4 * e + 1], b1 = bi[4 * e + 5], b2 = bi[4 * e + 9], b3 = bi[4 * e + 13];
        int c0 = bi[4 * e + 2], c1 = bi[4 * e + 6], c2 = bi[4 * e + 10], c3 = bi[4 * e + 14];
        int d0 = bi[4 * e + 3], d1 = bi[4 * e + 7], d2 = bi[4 * e + 11], d3 = bi[4 * e + 15];
        var k0 = Vector128.Create(bk[4 * e], bk[4 * e + 4], bk[4 * e + 8], bk[4 * e + 12]);
        var k1 = Vector128.Create(bk[4 * e + 1], bk[4 * e + 5], bk[4 * e + 9], bk[4 * e + 13]);
        var k2 = Vector128.Create(bk[4 * e + 2], bk[4 * e + 6], bk[4 * e + 10], bk[4 * e + 14]);
        var k3 = Vector128.Create(bk[4 * e + 3], bk[4 * e + 7], bk[4 * e + 11], bk[4 * e + 15]);
        var s = Vector128.Create(d.SBendS[e], d.SBendS[e + 1], d.SBendS[e + 2], d.SBendS[e + 3]);
        var m0 = Vector128.Create(d.SBendM0[e], d.SBendM0[e + 1], d.SBendM0[e + 2], d.SBendM0[e + 3]);
        int ka0 = 3 * a0, ka1 = 3 * a1, ka2 = 3 * a2, ka3 = 3 * a3, kb0 = 3 * b0, kb1 = 3 * b1, kb2 = 3 * b2, kb3 = 3 * b3;
        int kc0 = 3 * c0, kc1 = 3 * c1, kc2 = 3 * c2, kc3 = 3 * c3, kd0 = 3 * d0, kd1 = 3 * d1, kd2 = 3 * d2, kd3 = 3 * d3;
        var xa = Gather(x, ka0, ka1, ka2, ka3); var ya = Gather(x, ka0 + 1, ka1 + 1, ka2 + 1, ka3 + 1); var za = Gather(x, ka0 + 2, ka1 + 2, ka2 + 2, ka3 + 2);
        var xb = Gather(x, kb0, kb1, kb2, kb3); var yb = Gather(x, kb0 + 1, kb1 + 1, kb2 + 1, kb3 + 1); var zb = Gather(x, kb0 + 2, kb1 + 2, kb2 + 2, kb3 + 2);
        var xc = Gather(x, kc0, kc1, kc2, kc3); var yc = Gather(x, kc0 + 1, kc1 + 1, kc2 + 1, kc3 + 1); var zc = Gather(x, kc0 + 2, kc1 + 2, kc2 + 2, kc3 + 2);
        var xd = Gather(x, kd0, kd1, kd2, kd3); var yd = Gather(x, kd0 + 1, kd1 + 1, kd2 + 1, kd3 + 1); var zd = Gather(x, kd0 + 2, kd1 + 2, kd2 + 2, kd3 + 2);
        var mx = k1 * (xb - xa) + k2 * (xc - xa) + k3 * (xd - xa);
        var my = k1 * (yb - ya) + k2 * (yc - ya) + k3 * (yd - ya);
        var mz = k1 * (zb - za) + k2 * (zc - za) + k3 * (zd - za);
        var m2 = mx * mx + my * my + mz * mz;
        var cv = s * (m2 - m0);
        var f = Vector128.Create(2f) * s;
        var wa = Gather(inv, a0, a1, a2, a3); var wb = Gather(inv, b0, b1, b2, b3); var wc = Gather(inv, c0, c1, c2, c3); var wd = Gather(inv, d0, d1, d2, d3);
        var wsum = f * f * m2 * (wa * k0 * k0 + wb * k1 * k1 + wc * k2 * k2 + wd * k3 * k3);
        var damp = Vector128<float>.Zero;
        if (gamma != 0f)
        {
            var dxm = k0 * (xa - Gather(xp, ka0, ka1, ka2, ka3)) + k1 * (xb - Gather(xp, kb0, kb1, kb2, kb3)) + k2 * (xc - Gather(xp, kc0, kc1, kc2, kc3)) + k3 * (xd - Gather(xp, kd0, kd1, kd2, kd3));
            var dym = k0 * (ya - Gather(xp, ka0 + 1, ka1 + 1, ka2 + 1, ka3 + 1)) + k1 * (yb - Gather(xp, kb0 + 1, kb1 + 1, kb2 + 1, kb3 + 1)) + k2 * (yc - Gather(xp, kc0 + 1, kc1 + 1, kc2 + 1, kc3 + 1)) + k3 * (yd - Gather(xp, kd0 + 1, kd1 + 1, kd2 + 1, kd3 + 1));
            var dzm = k0 * (za - Gather(xp, ka0 + 2, ka1 + 2, ka2 + 2, ka3 + 2)) + k1 * (zb - Gather(xp, kb0 + 2, kb1 + 2, kb2 + 2, kb3 + 2)) + k2 * (zc - Gather(xp, kc0 + 2, kc1 + 2, kc2 + 2, kc3 + 2)) + k3 * (zd - Gather(xp, kd0 + 2, kd1 + 2, kd2 + 2, kd3 + 2));
            damp = Vector128.Create(gamma) * f * (mx * dxm + my * dym + mz * dzm);
        }

        var denom = Vector128.Create(1f + gamma) * wsum + Vector128.Create(alpha);
        var valid = Vector128.GreaterThan(denom, Vector128.Create(1e-30f));
        var lam = d.Lambda;
        var lv = Vector128.Create(lam[ne + e], lam[ne + e + 1], lam[ne + e + 2], lam[ne + e + 3]);
        var dl = (-cv - Vector128.Create(alpha) * lv - damp) / denom;
        var lnew = Vector128.ConditionalSelect(valid, lv + dl, lv);
        var ta = wa * dl * f * k0; var tb = wb * dl * f * k1; var tc = wc * dl * f * k2; var td = wd * dl * f * k3;
        var xan = Vector128.ConditionalSelect(valid, xa + mx * ta, xa); var yan = Vector128.ConditionalSelect(valid, ya + my * ta, ya); var zan = Vector128.ConditionalSelect(valid, za + mz * ta, za);
        var xbn = Vector128.ConditionalSelect(valid, xb + mx * tb, xb); var ybn = Vector128.ConditionalSelect(valid, yb + my * tb, yb); var zbn = Vector128.ConditionalSelect(valid, zb + mz * tb, zb);
        var xcn = Vector128.ConditionalSelect(valid, xc + mx * tc, xc); var ycn = Vector128.ConditionalSelect(valid, yc + my * tc, yc); var zcn = Vector128.ConditionalSelect(valid, zc + mz * tc, zc);
        var xdn = Vector128.ConditionalSelect(valid, xd + mx * td, xd); var ydn = Vector128.ConditionalSelect(valid, yd + my * td, yd); var zdn = Vector128.ConditionalSelect(valid, zd + mz * td, zd);
        // Constraints of one colour share no particle, so the stores cannot overlap.
        for (int l = 0; l < 4; l++) { lam[ne + e + l] = lnew.GetElement(l); }
        x[ka0] = xan.GetElement(0); x[ka0 + 1] = yan.GetElement(0); x[ka0 + 2] = zan.GetElement(0);
        x[ka1] = xan.GetElement(1); x[ka1 + 1] = yan.GetElement(1); x[ka1 + 2] = zan.GetElement(1);
        x[ka2] = xan.GetElement(2); x[ka2 + 1] = yan.GetElement(2); x[ka2 + 2] = zan.GetElement(2);
        x[ka3] = xan.GetElement(3); x[ka3 + 1] = yan.GetElement(3); x[ka3 + 2] = zan.GetElement(3);
        x[kb0] = xbn.GetElement(0); x[kb0 + 1] = ybn.GetElement(0); x[kb0 + 2] = zbn.GetElement(0);
        x[kb1] = xbn.GetElement(1); x[kb1 + 1] = ybn.GetElement(1); x[kb1 + 2] = zbn.GetElement(1);
        x[kb2] = xbn.GetElement(2); x[kb2 + 1] = ybn.GetElement(2); x[kb2 + 2] = zbn.GetElement(2);
        x[kb3] = xbn.GetElement(3); x[kb3 + 1] = ybn.GetElement(3); x[kb3 + 2] = zbn.GetElement(3);
        x[kc0] = xcn.GetElement(0); x[kc0 + 1] = ycn.GetElement(0); x[kc0 + 2] = zcn.GetElement(0);
        x[kc1] = xcn.GetElement(1); x[kc1 + 1] = ycn.GetElement(1); x[kc1 + 2] = zcn.GetElement(1);
        x[kc2] = xcn.GetElement(2); x[kc2 + 1] = ycn.GetElement(2); x[kc2 + 2] = zcn.GetElement(2);
        x[kc3] = xcn.GetElement(3); x[kc3 + 1] = ycn.GetElement(3); x[kc3 + 2] = zcn.GetElement(3);
        x[kd0] = xdn.GetElement(0); x[kd0 + 1] = ydn.GetElement(0); x[kd0 + 2] = zdn.GetElement(0);
        x[kd1] = xdn.GetElement(1); x[kd1 + 1] = ydn.GetElement(1); x[kd1 + 2] = zdn.GetElement(1);
        x[kd2] = xdn.GetElement(2); x[kd2 + 1] = ydn.GetElement(2); x[kd2 + 2] = zdn.GetElement(2);
        x[kd3] = xdn.GetElement(3); x[kd3 + 1] = ydn.GetElement(3); x[kd3 + 2] = zdn.GetElement(3);
    }

    // Tetrahedron volume: C = 6V - 6V0 with the gradients of the triple product (Müller et al. 2007 sec. 3.5; Müller, Heidelberger, Hennix, Ratcliff 2007 for the tetrahedral version).
    private static void XpbdVolume(SoftBodyData d, int e, float alpha, float gamma, ref float lambda)
    {
        var x = d.X; var xp = d.XP; var inv = d.InvMass;
        int a = d.STets[4 * e], b = d.STets[4 * e + 1], c = d.STets[4 * e + 2], dd = d.STets[4 * e + 3];
        int ka = 3 * a, kb = 3 * b, kc = 3 * c, kd = 3 * dd;
        float e1x = x[kb] - x[ka], e1y = x[kb + 1] - x[ka + 1], e1z = x[kb + 2] - x[ka + 2];
        float e2x = x[kc] - x[ka], e2y = x[kc + 1] - x[ka + 1], e2z = x[kc + 2] - x[ka + 2];
        float e3x = x[kd] - x[ka], e3y = x[kd + 1] - x[ka + 1], e3z = x[kd + 2] - x[ka + 2];
        // gradients of 6V with respect to b, c, d (a gets minus their sum)
        float gbx = e2y * e3z - e2z * e3y, gby = e2z * e3x - e2x * e3z, gbz = e2x * e3y - e2y * e3x;
        float gcx = e3y * e1z - e3z * e1y, gcy = e3z * e1x - e3x * e1z, gcz = e3x * e1y - e3y * e1x;
        float gdx = e1y * e2z - e1z * e2y, gdy = e1z * e2x - e1x * e2z, gdz = e1x * e2y - e1y * e2x;
        float gax = -(gbx + gcx + gdx), gay = -(gby + gcy + gdy), gaz = -(gbz + gcz + gdz);
        float wa = inv[a], wb = inv[b], wc = inv[c], wd = inv[dd];
        float wsum = wa * (gax * gax + gay * gay + gaz * gaz) + wb * (gbx * gbx + gby * gby + gbz * gbz) + wc * (gcx * gcx + gcy * gcy + gcz * gcz) + wd * (gdx * gdx + gdy * gdy + gdz * gdz);
        float vol6 = e1x * gbx + e1y * gby + e1z * gbz;
        float cv = vol6 - d.STetRest[e];
        float damp = 0f;
        if (gamma != 0f)
        {
            float ux = x[ka] - xp[ka], uy = x[ka + 1] - xp[ka + 1], uz = x[ka + 2] - xp[ka + 2];
            float vx = x[kb] - xp[kb], vy = x[kb + 1] - xp[kb + 1], vz = x[kb + 2] - xp[kb + 2];
            float wx = x[kc] - xp[kc], wy = x[kc + 1] - xp[kc + 1], wz = x[kc + 2] - xp[kc + 2];
            float sx = x[kd] - xp[kd], sy = x[kd + 1] - xp[kd + 1], sz = x[kd + 2] - xp[kd + 2];
            damp = gamma * (gax * ux + gay * uy + gaz * uz + gbx * vx + gby * vy + gbz * vz + gcx * wx + gcy * wy + gcz * wz + gdx * sx + gdy * sy + gdz * sz);
        }

        float denom = (1f + gamma) * wsum + alpha;
        if (!(denom > 1e-30f)) { return; }
        float dl = (-cv - alpha * lambda - damp) / denom;
        lambda += dl;
        float s;
        s = wa * dl; x[ka] += gax * s; x[ka + 1] += gay * s; x[ka + 2] += gaz * s;
        s = wb * dl; x[kb] += gbx * s; x[kb + 1] += gby * s; x[kb + 2] += gbz * s;
        s = wc * dl; x[kc] += gcx * s; x[kc + 1] += gcy * s; x[kc + 2] += gcz * s;
        s = wd * dl; x[kd] += gdx * s; x[kd + 1] += gdy * s; x[kd + 2] += gdz * s;
    }

    // Pressure: one constraint on the volume enclosed by a closed surface, C = V - pressure * V0 (Müller et al. 2007, sec. 3.7, cloth balloon).
    private static void XpbdPressure(SoftBodyData d, float alpha, float gamma, ref float lambda)
    {
        int n = d.N;
        var gp = d.GradP;
        Array.Clear(gp, 0, 3 * n);
        var x = d.X; var xp = d.XP; var inv = d.InvMass;
        float ox = x[0], oy = x[1], oz = x[2];
        float v6 = 0f;
        var tris = d.Tris;
        for (int e = 0; e < tris.Length / 3; e++)
        {
            int a = tris[3 * e], b = tris[3 * e + 1], c = tris[3 * e + 2];
            float ax = x[3 * a] - ox, ay = x[3 * a + 1] - oy, az = x[3 * a + 2] - oz;
            float bx = x[3 * b] - ox, by = x[3 * b + 1] - oy, bz = x[3 * b + 2] - oz;
            float cx = x[3 * c] - ox, cy = x[3 * c + 1] - oy, cz = x[3 * c + 2] - oz;
            float bcx = by * cz - bz * cy, bcy = bz * cx - bx * cz, bcz = bx * cy - by * cx;
            float cax = cy * az - cz * ay, cay = cz * ax - cx * az, caz = cx * ay - cy * ax;
            float abx = ay * bz - az * by, aby = az * bx - ax * bz, abz = ax * by - ay * bx;
            v6 += ax * bcx + ay * bcy + az * bcz;
            gp[3 * a] += bcx; gp[3 * a + 1] += bcy; gp[3 * a + 2] += bcz;
            gp[3 * b] += cax; gp[3 * b + 1] += cay; gp[3 * b + 2] += caz;
            gp[3 * c] += abx; gp[3 * c + 1] += aby; gp[3 * c + 2] += abz;
        }

        // Volume does not change when everything moves together, so the reference particle's gradient is minus the sum of the others.
        float sx = 0f, sy = 0f, sz = 0f;
        for (int i = 1; i < n; i++) { sx += gp[3 * i]; sy += gp[3 * i + 1]; sz += gp[3 * i + 2]; }
        gp[0] = -sx; gp[1] = -sy; gp[2] = -sz;
        // The gradients above are those of 6V; work with 6V throughout.
        float target = d.Def.InternalPressure * d.PressureRest * 6f;
        float cv = v6 - target;
        float wsum = 0f, damp = 0f;
        for (int i = 0; i < n; i++)
        {
            int k = 3 * i;
            wsum += inv[i] * (gp[k] * gp[k] + gp[k + 1] * gp[k + 1] + gp[k + 2] * gp[k + 2]);
            if (gamma != 0f) { damp += gp[k] * (x[k] - xp[k]) + gp[k + 1] * (x[k + 1] - xp[k + 1]) + gp[k + 2] * (x[k + 2] - xp[k + 2]); }
        }

        float denom = (1f + gamma) * wsum + alpha;
        if (!(denom > 1e-30f)) { return; }
        float dl = (-cv - alpha * lambda - gamma * damp) / denom;
        lambda += dl;
        for (int i = 0; i < n; i++)
        {
            float s = inv[i] * dl;
            if (s == 0f) { continue; }
            int k = 3 * i;
            x[k] += gp[k] * s; x[k + 1] += gp[k + 1] * s; x[k + 2] += gp[k + 2] * s;
        }
    }
}
