using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    // Extended position-based dynamics in small steps: Müller et al. 2007 (position based dynamics), Macklin, Müller, Chentanez 2016 (XPBD: compliance and the multiplier update),
    // Macklin et al. 2019 (small steps: many substeps with a single Gauss–Seidel pass each beats many iterations of one large step).
    // The kernels work on the float32 copies X / XP of the positions (relative to the body's origin for the step). Constraints are stored by region (SoftBodyBlocks) and solved in that order.

    private static void SolveEdges(SoftBodyData d, int first, int end, float h)
    {
        if (first == end) { return; }
        var def = d.Def;
        float alpha = def.EdgeCompliance / (h * h), gamma = def.EdgeDamping / h;
        for (int e = first; e < end; e++) { XpbdDistance(d.X, d.XP, d.InvMass, d.SEdgeA[e], d.SEdgeB[e], d.SEdgeRest[e], alpha, gamma, ref d.Lambda[e]); }
    }

    private static void SolveBends(SoftBodyData d, int first, int end, float h)
    {
        if (first == end) { return; }
        var def = d.Def;
        float alpha = def.BendCompliance / (h * h), gamma = def.BendDamping / h;
        int ne = d.SEdgeA.Length;
        for (int e = first; e < end; e++) { XpbdDistance(d.X, d.XP, d.InvMass, d.SBendA[e], d.SBendB[e], d.SBendRest[e], alpha, gamma, ref d.Lambda[ne + e]); }
    }

    private static void SolveAreas(SoftBodyData d, int first, int end, float h)
    {
        if (first == end) { return; }
        var def = d.Def;
        float alpha = def.AreaCompliance / (h * h), gamma = def.AreaDamping / h;
        int off = d.SEdgeA.Length + d.SBendA.Length;
        for (int e = first; e < end; e++) { XpbdArea(d, e, alpha, gamma, ref d.Lambda[off + e]); }
    }

    // Distance constraint C = |xa - xb| - rest, with the XPBD multiplier update and Rayleigh-style damping (Macklin et al. 2016, eq. 18 with gamma = damping time / substep).
    private static void XpbdDistance(float[] x, float[] xp, float[] inv, int a, int b, float rest, float alpha, float gamma, ref float lambda)
    {
        float wa = inv[a], wb = inv[b];
        float wsum = wa + wb;
        if (wsum == 0f) { return; }
        int ka = 2 * a, kb = 2 * b;
        float dx = x[ka] - x[kb], dy = x[ka + 1] - x[kb + 1];
        float len = float.Sqrt(dx * dx + dy * dy);
        if (!(len > 1e-9f)) { return; }
        float il = 1f / len;
        float nx = dx * il, ny = dy * il;
        float c = len - rest;
        float damp = 0f;
        if (gamma != 0f)
        {
            damp = gamma * (nx * ((x[ka] - xp[ka]) - (x[kb] - xp[kb])) + ny * ((x[ka + 1] - xp[ka + 1]) - (x[kb + 1] - xp[kb + 1])));
        }

        float dl = (-c - alpha * lambda - damp) / ((1f + gamma) * wsum + alpha);
        lambda += dl;
        float sa = wa * dl, sb = wb * dl;
        x[ka] += nx * sa; x[ka + 1] += ny * sa;
        x[kb] -= nx * sb; x[kb + 1] -= ny * sb;
    }

    // Triangle area: C = 2A - 2A0 (twice the signed area, the cross product of two edges) with the gradients of the cross product: the 2D analogue of the tetrahedron volume constraint
    // (Müller et al. 2007, sec. 3.5).
    private static void XpbdArea(SoftBodyData d, int e, float alpha, float gamma, ref float lambda)
    {
        var x = d.X; var xp = d.XP; var inv = d.InvMass;
        int a = d.STris[3 * e], b = d.STris[3 * e + 1], c = d.STris[3 * e + 2];
        int ka = 2 * a, kb = 2 * b, kc = 2 * c;
        float ax = x[ka], ay = x[ka + 1], bx = x[kb], by = x[kb + 1], cx = x[kc], cy = x[kc + 1];
        // gradients of the doubled area with respect to a, b, c (they sum to zero)
        float gax = by - cy, gay = cx - bx;
        float gbx = cy - ay, gby = ax - cx;
        float gcx = ay - by, gcy = bx - ax;
        float wa = inv[a], wb = inv[b], wc = inv[c];
        float wsum = wa * (gax * gax + gay * gay) + wb * (gbx * gbx + gby * gby) + wc * (gcx * gcx + gcy * gcy);
        float area2 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        float cv = area2 - d.STriRest2[e];
        float damp = 0f;
        if (gamma != 0f)
        {
            damp = gamma * (gax * (ax - xp[ka]) + gay * (ay - xp[ka + 1]) + gbx * (bx - xp[kb]) + gby * (by - xp[kb + 1]) + gcx * (cx - xp[kc]) + gcy * (cy - xp[kc + 1]));
        }

        float denom = (1f + gamma) * wsum + alpha;
        if (!(denom > 1e-30f)) { return; }
        float dl = (-cv - alpha * lambda - damp) / denom;
        lambda += dl;
        float s;
        s = wa * dl; x[ka] += gax * s; x[ka + 1] += gay * s;
        s = wb * dl; x[kb] += gbx * s; x[kb + 1] += gby * s;
        s = wc * dl; x[kc] += gcx * s; x[kc + 1] += gcy * s;
    }

    // Pressure: one constraint on the area enclosed by the outline, C = 2A - 2 * pressure * A0 (Müller et al. 2007, sec. 3.7, the cloth balloon, in two dimensions). The doubled area is the sum over the outline of
    // x_i cross x_{i+1}, taken relative to the first outline vertex so large coordinates do not cancel; its gradient at x_i is (y_next - y_prev, x_prev - x_next).
    private static void SolvePressure(SoftBodyData d, float h)
    {
        var def = d.Def;
        float alpha = def.AreaCompliance / (h * h), gamma = def.AreaDamping / h;
        var x = d.X; var xp = d.XP; var inv = d.InvMass; var gp = d.GradP; var ol = d.Outline;
        int m = ol.Length;
        float ox = x[2 * ol[0]], oy = x[2 * ol[0] + 1];
        float area2 = 0f;
        for (int i = 0; i < m; i++)
        {
            int a = ol[i], b = ol[i + 1 == m ? 0 : i + 1], p = ol[i == 0 ? m - 1 : i - 1];
            float ax = x[2 * a] - ox, ay = x[2 * a + 1] - oy, bx = x[2 * b] - ox, by = x[2 * b + 1] - oy;
            area2 += ax * by - ay * bx;
            float px = x[2 * p] - ox, py = x[2 * p + 1] - oy;
            gp[2 * a] = by - py; gp[2 * a + 1] = px - bx;
        }

        float target = def.InternalPressure * d.PressureRest * 2f;
        float cv = area2 - target;
        float wsum = 0f, damp = 0f;
        for (int i = 0; i < m; i++)
        {
            int a = ol[i];
            int k = 2 * a;
            wsum += inv[a] * (gp[k] * gp[k] + gp[k + 1] * gp[k + 1]);
            if (gamma != 0f) { damp += gp[k] * (x[k] - xp[k]) + gp[k + 1] * (x[k + 1] - xp[k + 1]); }
        }

        float denom = (1f + gamma) * wsum + alpha;
        if (!(denom > 1e-30f)) { return; }
        ref float lambda = ref d.Lambda[d.SEdgeA.Length + d.SBendA.Length + d.STriRest2.Length];
        float dl = (-cv - alpha * lambda - gamma * damp) / denom;
        lambda += dl;
        for (int i = 0; i < m; i++)
        {
            int a = ol[i];
            float s = inv[a] * dl;
            if (s == 0f) { continue; }
            int k = 2 * a;
            x[k] += gp[k] * s; x[k + 1] += gp[k + 1] * s;
        }
    }

    // Shape matching (Müller, Heidelberger, Teschner, Gross 2005, "Meshless deformations based on shape matching"), in the position-based form of their section on clusters and with XPBD's multiplier so the
    // pull does not depend on the number of substeps or passes. Per cluster: the best rotation R of the rest offsets q_i onto the current offsets p_i = x_i - c comes from the 2 x 2 matrix A = sum m_i p_i q_i^T
    // (the polar factor: atan2(a10 - a01, a00 + a11) as a unit vector, no trigonometry); the goal is g_i = c + R q_i. A quadratic cluster also fits A~ = A_pq~ A_qq~^-1 over (q, q_x^2, q_y^2, q_x q_y) and aims at
    // c + blend * A~ q~_i + (1 - blend) * R q_i. Each member is then a soft constraint x_i = g_i weighted by the square root of its mass, which makes the pull a mass-proportional spring with compliance
    // MatchCompliance (seconds squared) and momentum-neutral inside the cluster: mu_i accumulates the position correction, dmu = -((x_i - g_i) + a mu_i) / (1 + a), a = compliance / h^2.
    private static void SolveClusters(SoftBodyData d, float h)
    {
        var def = d.Def;
        float alpha = def.MatchCompliance / (h * h);
        float keep = 1f / (1f + alpha);
        float blend = def.MatchBlend;
        var x = d.X; var inv = d.InvMass; var mu = d.MatchMu;
        Span<double> at = stackalloc double[10];
        Span<double> apq = stackalloc double[10];
        for (int c = 0; c < d.ClusterCount; c++)
        {
            int m0 = d.ClusterStart[c], m1 = d.ClusterStart[c + 1];
            double mass = d.ClusterTotalMass[c];
            double cx = 0.0, cy = 0.0;
            for (int m = m0; m < m1; m++)
            {
                int k = 2 * d.ClusterMember[m];
                double mi = d.ClusterMass[m];
                cx += mi * x[k]; cy += mi * x[k + 1];
            }

            cx /= mass; cy /= mass;
            double a00 = 0.0, a01 = 0.0, a10 = 0.0, a11 = 0.0;
            for (int m = m0; m < m1; m++)
            {
                int k = 2 * d.ClusterMember[m];
                double mi = d.ClusterMass[m];
                double px = x[k] - cx, py = x[k + 1] - cy, qx = d.ClusterQ[2 * m], qy = d.ClusterQ[2 * m + 1];
                a00 += mi * px * qx; a01 += mi * px * qy; a10 += mi * py * qx; a11 += mi * py * qy;
            }

            double rc = a00 + a11, rs = a10 - a01;
            double rl = Math.Sqrt(rc * rc + rs * rs);
            if (rl > 1e-30) { rc /= rl; rs /= rl; } else { rc = 1.0; rs = 0.0; }
            float scale = d.ClusterScale[c];
            bool quad = scale > 0f;
            // Ã (2 x 5): row 0 and row 1 of A_pq~ times A_qq~^-1, with offsets scaled by 1 / scale.
            at.Clear();
            if (quad)
            {
                apq.Clear();
                double inv5 = 1.0 / scale;
                for (int m = m0; m < m1; m++)
                {
                    int k = 2 * d.ClusterMember[m];
                    double mi = d.ClusterMass[m];
                    double px = x[k] - cx, py = x[k + 1] - cy, qx = d.ClusterQ[2 * m] * inv5, qy = d.ClusterQ[2 * m + 1] * inv5;
                    double q0 = qx, q1 = qy, q2 = qx * qx, q3 = qy * qy, q4 = qx * qy;
                    apq[0] += mi * px * q0; apq[1] += mi * px * q1; apq[2] += mi * px * q2; apq[3] += mi * px * q3; apq[4] += mi * px * q4;
                    apq[5] += mi * py * q0; apq[6] += mi * py * q1; apq[7] += mi * py * q2; apq[8] += mi * py * q3; apq[9] += mi * py * q4;
                }

                for (int r = 0; r < 2; r++)
                {
                    for (int col = 0; col < 5; col++)
                    {
                        double sum = 0.0;
                        for (int j = 0; j < 5; j++) { sum += apq[5 * r + j] * d.ClusterAinv[25 * c + 5 * j + col]; }
                        at[5 * r + col] = sum;
                    }
                }
            }

            double inv5s = quad ? 1.0 / scale : 0.0;
            for (int m = m0; m < m1; m++)
            {
                int i = d.ClusterMember[m];
                if (inv[i] == 0f) { continue; }
                int k = 2 * i;
                double qx = d.ClusterQ[2 * m], qy = d.ClusterQ[2 * m + 1];
                double gx = rc * qx - rs * qy, gy = rs * qx + rc * qy;
                if (quad)
                {
                    double sx = qx * inv5s, sy = qy * inv5s;
                    double q2 = sx * sx, q3 = sy * sy, q4 = sx * sy;
                    double ex = at[0] * sx + at[1] * sy + at[2] * q2 + at[3] * q3 + at[4] * q4;
                    double ey = at[5] * sx + at[6] * sy + at[7] * q2 + at[8] * q3 + at[9] * q4;
                    gx = blend * ex + (1.0 - blend) * gx; gy = blend * ey + (1.0 - blend) * gy;
                }

                float tx = (float)(cx + gx), ty = (float)(cy + gy);
                float mux = mu[2 * m], muy = mu[2 * m + 1];
                float dmx = -((x[k] - tx) + alpha * mux) * keep, dmy = -((x[k + 1] - ty) + alpha * muy) * keep;
                x[k] += dmx; x[k + 1] += dmy;
                mu[2 * m] = mux + dmx; mu[2 * m + 1] = muy + dmy;
            }
        }
    }
}
