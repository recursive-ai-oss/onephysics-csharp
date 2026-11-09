using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Aero;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    // Fabric: dihedral bending (Bridson, Marino, Fedkiw 2003; Müller et al. 2007), strain limiting and tethers as hard projections after the contacts of every pass (Provot 1995; Goldenthal et al. 2007; Kim, Chentanez,
    // Müller 2012), and aerodynamic forces on every triangle of the cloth (flat-plate lift and drag, the library's wing model).

    // Bend constraint on the signed dihedral angle of a hinge: C = angle - rest angle, gradient from the heights of the two triangles over the shared edge (the apex gradients are the normals over |N|^2 times the edge length,
    // the edge ends take the rest so that force and torque balance).
    private static void XpbdBendDihedral(SoftBodyData d, int e, float alpha, float gamma, ref float lambda) =>
        DihedralKernel(d.X, d.XP, d.InvMass, d.SBendIdx[4 * e], d.SBendIdx[4 * e + 1], d.SBendIdx[4 * e + 2], d.SBendIdx[4 * e + 3], d.SBendSin0![e], d.SBendCos0![e], alpha, gamma, ref lambda);

    internal static void DihedralKernel(float[] x, float[] xp, float[] inv, int a, int b, int c, int dd, float s0, float c0, float alpha, float gamma, ref float lambda)
    {
        int ka = 3 * a, kb = 3 * b, kc = 3 * c, kd = 3 * dd;
        float ex = x[kb] - x[ka], ey = x[kb + 1] - x[ka + 1], ez = x[kb + 2] - x[ka + 2];
        float cx = x[kc] - x[ka], cy = x[kc + 1] - x[ka + 1], cz = x[kc + 2] - x[ka + 2];
        float dx = x[kd] - x[ka], dy = x[kd + 1] - x[ka + 1], dz = x[kd + 2] - x[ka + 2];
        float n1x = ey * cz - ez * cy, n1y = ez * cx - ex * cz, n1z = ex * cy - ey * cx;
        float n2x = dy * ez - dz * ey, n2y = dz * ex - dx * ez, n2z = dx * ey - dy * ex;
        float q1 = n1x * n1x + n1y * n1y + n1z * n1z, q2 = n2x * n2x + n2y * n2y + n2z * n2z, e2 = ex * ex + ey * ey + ez * ez;
        if (!(q1 > 1e-20f) || !(q2 > 1e-20f) || !(e2 > 1e-20f)) { return; }
        float l1 = float.Sqrt(q1), l2 = float.Sqrt(q2), el = float.Sqrt(e2);
        float i1 = 1f / l1, i2 = 1f / l2, iel = 1f / el;
        float m1x = n1x * i1, m1y = n1y * i1, m1z = n1z * i1, m2x = n2x * i2, m2y = n2y * i2, m2z = n2z * i2;
        float cosT = m1x * m2x + m1y * m2y + m1z * m2z;
        float sinT = ((m1y * m2z - m1z * m2y) * ex + (m1z * m2x - m1x * m2z) * ey + (m1x * m2y - m1y * m2x) * ez) * iel;
        // The angle error, from the sine and cosine of the difference.
        float sd = sinT * c0 - cosT * s0, cd = cosT * c0 + sinT * s0;
        float cv = ClothMath.Atan2(sd, cd);
        // Gradients of the signed angle: apexes move along their own normals, the edge ends share the balance by where the apexes project on the edge.
        float f3 = -el / q1, f4 = -el / q2;
        float g3x = n1x * f3, g3y = n1y * f3, g3z = n1z * f3, g4x = n2x * f4, g4y = n2y * f4, g4z = n2z * f4;
        float ie2 = 1f / e2;
        float t3 = (cx * ex + cy * ey + cz * ez) * ie2, t4 = (dx * ex + dy * ey + dz * ez) * ie2;
        float u3 = 1f - t3, u4 = 1f - t4;
        float g1x = -(u3 * g3x + u4 * g4x), g1y = -(u3 * g3y + u4 * g4y), g1z = -(u3 * g3z + u4 * g4z);
        float g2x = -(t3 * g3x + t4 * g4x), g2y = -(t3 * g3y + t4 * g4y), g2z = -(t3 * g3z + t4 * g4z);
        float w1 = inv[a], w2 = inv[b], w3 = inv[c], w4 = inv[dd];
        float wsum = w1 * (g1x * g1x + g1y * g1y + g1z * g1z) + w2 * (g2x * g2x + g2y * g2y + g2z * g2z) + w3 * (g3x * g3x + g3y * g3y + g3z * g3z) + w4 * (g4x * g4x + g4y * g4y + g4z * g4z);
        float damp = 0f;
        if (gamma != 0f)
        {
            damp = gamma * (g1x * (x[ka] - xp[ka]) + g1y * (x[ka + 1] - xp[ka + 1]) + g1z * (x[ka + 2] - xp[ka + 2])
                + g2x * (x[kb] - xp[kb]) + g2y * (x[kb + 1] - xp[kb + 1]) + g2z * (x[kb + 2] - xp[kb + 2])
                + g3x * (x[kc] - xp[kc]) + g3y * (x[kc + 1] - xp[kc + 1]) + g3z * (x[kc + 2] - xp[kc + 2])
                + g4x * (x[kd] - xp[kd]) + g4y * (x[kd + 1] - xp[kd + 1]) + g4z * (x[kd + 2] - xp[kd + 2]));
        }

        float denom = (1f + gamma) * wsum + alpha;
        if (!(denom > 1e-30f)) { return; }
        float dl = (-cv - alpha * lambda - damp) / denom;
        lambda += dl;
        float s = w1 * dl; x[ka] += g1x * s; x[ka + 1] += g1y * s; x[ka + 2] += g1z * s;
        s = w2 * dl; x[kb] += g2x * s; x[kb + 1] += g2y * s; x[kb + 2] += g2z * s;
        s = w3 * dl; x[kc] += g3x * s; x[kc + 1] += g3y * s; x[kc + 2] += g3z * s;
        s = w4 * dl; x[kd] += g4x * s; x[kd + 1] += g4y * s; x[kd + 2] += g4z * s;
    }

    private const int ClothLimitSweeps = 4;

    // The limits of a cloth after a pass: edges and tethers are pulled back to their longest allowed length, by inverse-mass shares. Sweeps alternate direction with the pass number so no end of the cloth is always last.
    private static void ClothLimits(SoftBodyData d, int pass)
    {
        var cl = d.Def.Fabric!;
        var x = d.X; var inv = d.InvMass;
        if (cl.StrainLimit > 0f)
        {
            float k = 1f + cl.StrainLimit;
            int ne = d.SEdgeA.Length;
            for (int sweep = 0; sweep < ClothLimitSweeps; sweep++)
            {
                bool rev = ((pass + sweep) & 1) != 0;
                for (int i = 0; i < ne; i++)
                {
                    int e = rev ? ne - 1 - i : i;
                    LimitDistance(x, inv, d.SEdgeA[e], d.SEdgeB[e], d.SEdgeRest[e] * k);
                }
            }
        }

        for (int t = 0; t < d.TethP.Length; t++) { LimitDistance(x, inv, d.TethP[t], d.TethA[t], d.TethLen[t]); }
    }

    private static void LimitDistance(float[] x, float[] inv, int a, int b, float max)
    {
        float wa = inv[a], wb = inv[b];
        float wsum = wa + wb;
        if (wsum == 0f) { return; }
        int ka = 3 * a, kb = 3 * b;
        float dx = x[ka] - x[kb], dy = x[ka + 1] - x[kb + 1], dz = x[ka + 2] - x[kb + 2];
        float l2 = dx * dx + dy * dy + dz * dz;
        if (!(l2 > max * max)) { return; }
        float len = float.Sqrt(l2);
        float f = (len - max) / (len * wsum);
        float sa = wa * f, sb = wb * f;
        x[ka] -= dx * sa; x[ka + 1] -= dy * sa; x[ka + 2] -= dz * sa;
        x[kb] += dx * sb; x[kb + 1] += dy * sb; x[kb + 2] += dz * sb;
    }

    // Aerodynamic force on every triangle of a cloth for one substep, applied to the particles' velocities before they are predicted: the relative flow (wind of the area fields minus the triangle's velocity) splits
    // into a part along the surface (the chord) and a part through it, and the plate model gives the force along each. A third of the force goes to each corner, and the velocity change is capped at the
    // relative speed so a light cloth cannot be thrown past the flow's own velocity in one substep.
    private void ClothAero(SoftBodyData d, float h)
    {
        var cl = d.Def.Fabric!;
        var coef = ClothCoefficients(cl);
        float rho = Actuators.AirDensity;
        var tris = d.Tris;
        var x = d.X; var vel = d.Vel; var inv = d.InvMass;
        for (int t = 0; t < tris.Length / 3; t++)
        {
            int a = tris[3 * t], b = tris[3 * t + 1], c = tris[3 * t + 2];
            int ka = 3 * a, kb = 3 * b, kc = 3 * c;
            float e1x = x[kb] - x[ka], e1y = x[kb + 1] - x[ka + 1], e1z = x[kb + 2] - x[ka + 2];
            float e2x = x[kc] - x[ka], e2y = x[kc + 1] - x[ka + 1], e2z = x[kc + 2] - x[ka + 2];
            float nx = e1y * e2z - e1z * e2y, ny = e1z * e2x - e1x * e2z, nz = e1x * e2y - e1y * e2x;
            float nl = float.Sqrt(nx * nx + ny * ny + nz * nz);
            if (!(nl > 1e-12f)) { continue; }
            float area = 0.5f * nl;
            float inl = 1f / nl;
            nx *= inl; ny *= inl; nz *= inl;
            double cxw = d.Ox + (x[ka] + x[kb] + x[kc]) * (1.0 / 3.0), cyw = d.Oy + (x[ka + 1] + x[kb + 1] + x[kc + 1]) * (1.0 / 3.0), czw = d.Oz + (x[ka + 2] + x[kb + 2] + x[kc + 2]) * (1.0 / 3.0);
            var wind = Actuators.WindAt(new Vector3D(cxw, cyw, czw));
            float vx = (vel[ka] + vel[kb] + vel[kc]) * (1f / 3f), vy = (vel[ka + 1] + vel[kb + 1] + vel[kc + 1]) * (1f / 3f), vz = (vel[ka + 2] + vel[kb + 2] + vel[kc + 2]) * (1f / 3f);
            float ux = wind.X - vx, uy = wind.Y - vy, uz = wind.Z - vz;
            float un = ux * nx + uy * ny + uz * nz;
            float tx = ux - un * nx, ty = uy - un * ny, tz = uz - un * nz;
            float tl = float.Sqrt(tx * tx + ty * ty + tz * tz);
            float chx = 0f, chy = 0f, chz = 0f;
            if (tl > 1e-9f) { float itl = 1f / tl; chx = tx * itl; chy = ty * itl; chz = tz * itl; }
            coef.Force(tl, un, area, rho, out float fc, out float fn);
            float fx = fc * chx + fn * nx, fy = fc * chy + fn * ny, fz = fc * chz + fn * nz;
            // The velocity change is capped at the relative speed: a very light cloth in a strong flow ends the substep moving with the flow, not past it.
            float wa = inv[a], wb = inv[b], wc = inv[c];
            float mTri = (wa > 0f ? 1f / wa : 0f) + (wb > 0f ? 1f / wb : 0f) + (wc > 0f ? 1f / wc : 0f);
            float fl = float.Sqrt(fx * fx + fy * fy + fz * fz), ul = float.Sqrt(ux * ux + uy * uy + uz * uz);
            if (fl * h > ul * mTri && fl > 0f) { float sc = ul * mTri / (fl * h); fx *= sc; fy *= sc; fz *= sc; }
            // A third of the force goes to each corner.
            for (int k = 0; k < 3; k++)
            {
                int p = k == 0 ? a : (k == 1 ? b : c);
                float w = inv[p];
                if (w == 0f) { continue; }
                float s = w * h * (1f / 3f);
                int kp = 3 * p;
                vel[kp] += fx * s; vel[kp + 1] += fy * s; vel[kp + 2] += fz * s;
            }
        }
    }

    private static AeroCoefficients ClothCoefficients(ClothParams cl)
    {
        var c = AeroCoefficients.Plate(2f);
        c.LiftSlope = cl.LiftSlope; c.PlateDrag = cl.Drag; c.MinDrag = Math.Min(c.MinDrag, cl.Drag);
        return c;
    }
}
