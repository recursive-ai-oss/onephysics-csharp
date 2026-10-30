using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    // Extended position-based dynamics in small steps: Müller et al. 2007 (position based dynamics), Macklin, Müller, Chentanez 2016 (XPBD: compliance and the multiplier update),
    // Macklin et al. 2019 (small steps: many substeps with a single Gauss–Seidel pass each beats many iterations of one large step).

    private static void PredictSoft(SoftBodyData d, float h, Vector3F g)
    {
        float gs = d.Def.GravityScale;
        float damp = 1f / (1f + d.Def.LinearDamping * h);
        float gx = g.X * gs * h, gy = g.Y * gs * h, gz = g.Z * gs * h;
        var pos = d.Pos; var prev = d.Prev; var vel = d.Vel; var inv = d.InvMass;
        for (int i = 0; i < d.N; i++)
        {
            int k = 3 * i;
            prev[k] = pos[k]; prev[k + 1] = pos[k + 1]; prev[k + 2] = pos[k + 2];
            if (inv[i] == 0f) { continue; }
            float vx = (vel[k] + gx) * damp, vy = (vel[k + 1] + gy) * damp, vz = (vel[k + 2] + gz) * damp;
            vel[k] = vx; vel[k + 1] = vy; vel[k + 2] = vz;
            pos[k] += vx * h; pos[k + 1] += vy * h; pos[k + 2] += vz * h;
        }
    }

    private void FinishSubstep(SoftBodyData d, float invH)
    {
        var pos = d.Pos; var prev = d.Prev; var vel = d.Vel; var inv = d.InvMass;
        float vmax = Opt.MaxLinearSpeed;
        float vmax2 = vmax * vmax;
        for (int i = 0; i < d.N; i++)
        {
            int k = 3 * i;
            if (inv[i] == 0f) { vel[k] = 0f; vel[k + 1] = 0f; vel[k + 2] = 0f; continue; }
            float vx = (float)(pos[k] - prev[k]) * invH, vy = (float)(pos[k + 1] - prev[k + 1]) * invH, vz = (float)(pos[k + 2] - prev[k + 2]) * invH;
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

    private void SolveConstraints(SoftBodyData d, float h)
    {
        var def = d.Def;
        float ih2 = 1f / (h * h), ih = 1f / h;
        var pos = d.Pos; var prev = d.Prev; var inv = d.InvMass; var lam = d.Lambda;
        int ne = d.EdgeA.Length;

        float alpha = def.EdgeCompliance * ih2, gamma = def.EdgeDamping * ih;
        for (int e = 0; e < ne; e++) { XpbdDistance(pos, prev, inv, d.EdgeA[e], d.EdgeB[e], d.EdgeRest[e], alpha, gamma, ref lam[e]); }
        alpha = def.BendCompliance * ih2; gamma = def.BendDamping * ih;
        int nh = d.BendCount;
        for (int e = 0; e < nh; e++) { XpbdBend(d, e, alpha, gamma, ref lam[ne + e]); }
        int nt = d.Tets.Length / 4;
        if (nt > 0)
        {
            alpha = def.VolumeCompliance * ih2; gamma = def.VolumeDamping * ih;
            for (int e = 0; e < nt; e++) { XpbdVolume(d, e, alpha, gamma, ref lam[ne + nh + e]); }
        }

        if (d.PressureRest > 0f && def.InternalPressure > 0f)
        {
            alpha = def.VolumeCompliance * ih2; gamma = def.VolumeDamping * ih;
            XpbdPressure(d, alpha, gamma, ref lam[ne + nh + nt]);
        }
    }

    // Distance constraint C = |xa - xb| - rest, with the XPBD multiplier update and Rayleigh-style damping (Macklin et al. 2016, eq. 18 with gamma = damping time / substep).
    private static void XpbdDistance(double[] pos, double[] prev, float[] inv, int a, int b, float rest, float alpha, float gamma, ref float lambda)
    {
        float wa = inv[a], wb = inv[b];
        float wsum = wa + wb;
        if (wsum == 0f) { return; }
        int ka = 3 * a, kb = 3 * b;
        float dx = (float)(pos[ka] - pos[kb]), dy = (float)(pos[ka + 1] - pos[kb + 1]), dz = (float)(pos[ka + 2] - pos[kb + 2]);
        float len = float.Sqrt(dx * dx + dy * dy + dz * dz);
        if (!(len > 1e-9f)) { return; }
        float il = 1f / len;
        float nx = dx * il, ny = dy * il, nz = dz * il;
        float c = len - rest;
        float damp = 0f;
        if (gamma != 0f)
        {
            damp = gamma * (nx * ((float)(pos[ka] - prev[ka]) - (float)(pos[kb] - prev[kb])) + ny * ((float)(pos[ka + 1] - prev[ka + 1]) - (float)(pos[kb + 1] - prev[kb + 1])) + nz * ((float)(pos[ka + 2] - prev[ka + 2]) - (float)(pos[kb + 2] - prev[kb + 2])));
        }

        float dl = (-c - alpha * lambda - damp) / ((1f + gamma) * wsum + alpha);
        lambda += dl;
        float sa = wa * dl, sb = wb * dl;
        pos[ka] += nx * sa; pos[ka + 1] += ny * sa; pos[ka + 2] += nz * sa;
        pos[kb] -= nx * sb; pos[kb + 1] -= ny * sb; pos[kb + 2] -= nz * sb;
    }

    // Isometric bend constraint C = s (|m|^2 - |m0|^2), m = sum K_i x_i, so the gradient is 2 s K_i m (see SoftBodyData.BendCoefficients).
    private static void XpbdBend(SoftBodyData d, int e, float alpha, float gamma, ref float lambda)
    {
        var pos = d.Pos; var prev = d.Prev; var inv = d.InvMass;
        int a = d.BendIdx[4 * e], b = d.BendIdx[4 * e + 1], c = d.BendIdx[4 * e + 2], dd = d.BendIdx[4 * e + 3];
        int ka = 3 * a, kb = 3 * b, kc = 3 * c, kd = 3 * dd;
        float k1 = d.BendK[4 * e + 1], k2 = d.BendK[4 * e + 2], k3 = d.BendK[4 * e + 3];
        float k0 = d.BendK[4 * e];
        float s = d.BendS[e];
        float mx = k1 * (float)(pos[kb] - pos[ka]) + k2 * (float)(pos[kc] - pos[ka]) + k3 * (float)(pos[kd] - pos[ka]);
        float my = k1 * (float)(pos[kb + 1] - pos[ka + 1]) + k2 * (float)(pos[kc + 1] - pos[ka + 1]) + k3 * (float)(pos[kd + 1] - pos[ka + 1]);
        float mz = k1 * (float)(pos[kb + 2] - pos[ka + 2]) + k2 * (float)(pos[kc + 2] - pos[ka + 2]) + k3 * (float)(pos[kd + 2] - pos[ka + 2]);
        float m2 = mx * mx + my * my + mz * mz;
        float cv = s * (m2 - d.BendM0[e]);
        float f = 2f * s;
        float wa = inv[a], wb = inv[b], wc = inv[c], wd = inv[dd];
        float wsum = f * f * m2 * (wa * k0 * k0 + wb * k1 * k1 + wc * k2 * k2 + wd * k3 * k3);
        float damp = 0f;
        if (gamma != 0f)
        {
            float dxm = k0 * (float)(pos[ka] - prev[ka]) + k1 * (float)(pos[kb] - prev[kb]) + k2 * (float)(pos[kc] - prev[kc]) + k3 * (float)(pos[kd] - prev[kd]);
            float dym = k0 * (float)(pos[ka + 1] - prev[ka + 1]) + k1 * (float)(pos[kb + 1] - prev[kb + 1]) + k2 * (float)(pos[kc + 1] - prev[kc + 1]) + k3 * (float)(pos[kd + 1] - prev[kd + 1]);
            float dzm = k0 * (float)(pos[ka + 2] - prev[ka + 2]) + k1 * (float)(pos[kb + 2] - prev[kb + 2]) + k2 * (float)(pos[kc + 2] - prev[kc + 2]) + k3 * (float)(pos[kd + 2] - prev[kd + 2]);
            damp = gamma * f * (mx * dxm + my * dym + mz * dzm);
        }

        float denom = (1f + gamma) * wsum + alpha;
        if (!(denom > 1e-30f)) { return; }
        float dl = (-cv - alpha * lambda - damp) / denom;
        lambda += dl;
        float t;
        t = wa * dl * f * k0; pos[ka] += mx * t; pos[ka + 1] += my * t; pos[ka + 2] += mz * t;
        t = wb * dl * f * k1; pos[kb] += mx * t; pos[kb + 1] += my * t; pos[kb + 2] += mz * t;
        t = wc * dl * f * k2; pos[kc] += mx * t; pos[kc + 1] += my * t; pos[kc + 2] += mz * t;
        t = wd * dl * f * k3; pos[kd] += mx * t; pos[kd + 1] += my * t; pos[kd + 2] += mz * t;
    }

    // Tetrahedron volume: C = 6V - 6V0 with the gradients of the triple product (Müller et al. 2007 sec. 3.5; Müller, Heidelberger, Hennix, Ratcliff 2007 for the tetrahedral version).
    private static void XpbdVolume(SoftBodyData d, int e, float alpha, float gamma, ref float lambda)
    {
        var pos = d.Pos; var prev = d.Prev; var inv = d.InvMass;
        int a = d.Tets[4 * e], b = d.Tets[4 * e + 1], c = d.Tets[4 * e + 2], dd = d.Tets[4 * e + 3];
        int ka = 3 * a, kb = 3 * b, kc = 3 * c, kd = 3 * dd;
        float e1x = (float)(pos[kb] - pos[ka]), e1y = (float)(pos[kb + 1] - pos[ka + 1]), e1z = (float)(pos[kb + 2] - pos[ka + 2]);
        float e2x = (float)(pos[kc] - pos[ka]), e2y = (float)(pos[kc + 1] - pos[ka + 1]), e2z = (float)(pos[kc + 2] - pos[ka + 2]);
        float e3x = (float)(pos[kd] - pos[ka]), e3y = (float)(pos[kd + 1] - pos[ka + 1]), e3z = (float)(pos[kd + 2] - pos[ka + 2]);
        // gradients of 6V with respect to b, c, d (a gets minus their sum)
        float gbx = e2y * e3z - e2z * e3y, gby = e2z * e3x - e2x * e3z, gbz = e2x * e3y - e2y * e3x;
        float gcx = e3y * e1z - e3z * e1y, gcy = e3z * e1x - e3x * e1z, gcz = e3x * e1y - e3y * e1x;
        float gdx = e1y * e2z - e1z * e2y, gdy = e1z * e2x - e1x * e2z, gdz = e1x * e2y - e1y * e2x;
        float gax = -(gbx + gcx + gdx), gay = -(gby + gcy + gdy), gaz = -(gbz + gcz + gdz);
        float wa = inv[a], wb = inv[b], wc = inv[c], wd = inv[dd];
        float wsum = wa * (gax * gax + gay * gay + gaz * gaz) + wb * (gbx * gbx + gby * gby + gbz * gbz) + wc * (gcx * gcx + gcy * gcy + gcz * gcz) + wd * (gdx * gdx + gdy * gdy + gdz * gdz);
        float vol6 = e1x * gbx + e1y * gby + e1z * gbz;
        float cv = vol6 - d.TetRest[e];
        float damp = 0f;
        if (gamma != 0f)
        {
            float ux = (float)(pos[ka] - prev[ka]), uy = (float)(pos[ka + 1] - prev[ka + 1]), uz = (float)(pos[ka + 2] - prev[ka + 2]);
            float vx = (float)(pos[kb] - prev[kb]), vy = (float)(pos[kb + 1] - prev[kb + 1]), vz = (float)(pos[kb + 2] - prev[kb + 2]);
            float wx = (float)(pos[kc] - prev[kc]), wy = (float)(pos[kc + 1] - prev[kc + 1]), wz = (float)(pos[kc + 2] - prev[kc + 2]);
            float sx = (float)(pos[kd] - prev[kd]), sy = (float)(pos[kd + 1] - prev[kd + 1]), sz = (float)(pos[kd + 2] - prev[kd + 2]);
            damp = gamma * (gax * ux + gay * uy + gaz * uz + gbx * vx + gby * vy + gbz * vz + gcx * wx + gcy * wy + gcz * wz + gdx * sx + gdy * sy + gdz * sz);
        }

        float denom = (1f + gamma) * wsum + alpha;
        if (!(denom > 1e-30f)) { return; }
        float dl = (-cv - alpha * lambda - damp) / denom;
        lambda += dl;
        float s;
        s = wa * dl; pos[ka] += gax * s; pos[ka + 1] += gay * s; pos[ka + 2] += gaz * s;
        s = wb * dl; pos[kb] += gbx * s; pos[kb + 1] += gby * s; pos[kb + 2] += gbz * s;
        s = wc * dl; pos[kc] += gcx * s; pos[kc + 1] += gcy * s; pos[kc + 2] += gcz * s;
        s = wd * dl; pos[kd] += gdx * s; pos[kd + 1] += gdy * s; pos[kd + 2] += gdz * s;
    }

    // Pressure: one constraint on the volume enclosed by a closed surface, C = V - pressure * V0 (Müller et al. 2007, sec. 3.7, cloth balloon).
    private void XpbdPressure(SoftBodyData d, float alpha, float gamma, ref float lambda)
    {
        int n = d.N;
        GrowTo(ref _gradP, 3 * n);
        var gp = _gradP;
        Array.Clear(gp, 0, 3 * n);
        var pos = d.Pos; var prev = d.Prev; var inv = d.InvMass;
        double ox = pos[0], oy = pos[1], oz = pos[2];
        double v6 = 0.0;
        var tris = d.Tris;
        for (int e = 0; e < tris.Length / 3; e++)
        {
            int a = tris[3 * e], b = tris[3 * e + 1], c = tris[3 * e + 2];
            float ax = (float)(pos[3 * a] - ox), ay = (float)(pos[3 * a + 1] - oy), az = (float)(pos[3 * a + 2] - oz);
            float bx = (float)(pos[3 * b] - ox), by = (float)(pos[3 * b + 1] - oy), bz = (float)(pos[3 * b + 2] - oz);
            float cx = (float)(pos[3 * c] - ox), cy = (float)(pos[3 * c + 1] - oy), cz = (float)(pos[3 * c + 2] - oz);
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
        float cv = (float)v6 - target;
        float wsum = 0f, damp = 0f;
        for (int i = 0; i < n; i++)
        {
            int k = 3 * i;
            wsum += inv[i] * (gp[k] * gp[k] + gp[k + 1] * gp[k + 1] + gp[k + 2] * gp[k + 2]);
            if (gamma != 0f) { damp += gp[k] * (float)(pos[k] - prev[k]) + gp[k + 1] * (float)(pos[k + 1] - prev[k + 1]) + gp[k + 2] * (float)(pos[k + 2] - prev[k + 2]); }
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
            pos[k] += gp[k] * s; pos[k + 1] += gp[k + 1] * s; pos[k + 2] += gp[k + 2] * s;
        }
    }

    // ------------------------------------------------------------------ contacts with shapes

    private static bool BoxesOverlap(double[] box, int o, double x0, double y0, double z0, double x1, double y1, double z1) =>
        !(x0 > box[o + 3] || x1 < box[o] || y0 > box[o + 4] || y1 < box[o + 1] || z0 > box[o + 5] || z1 < box[o + 2]);

    private void DetectContacts(SoftBodyData d, float h)
    {
        float r = d.Radius;
        float spec = Opt.SpeculativeDistance;
        float fastSq = 0.25f * r * r;
        var pos = d.Pos; var prev = d.Prev;
        d.HadMovingContact = false;
        for (int i = 0; i < d.N; i++)
        {
            if (d.InvMass[i] == 0f) { continue; }
            int k = 3 * i;
            double x = pos[k], y = pos[k + 1], z = pos[k + 2], px = prev[k], py = prev[k + 1], pz = prev[k + 2];
            float mx = (float)(x - px), my = (float)(y - py), mz = (float)(z - pz);
            float m2 = mx * mx + my * my + mz * mz;
            bool fast = m2 > fastSq;
            // A particle that moves far in a substep may be pushed much farther by the constraints, so contacts are made speculatively for twice the distance it moved: a plane that is not reached stays inactive.
            float specP = spec + 2f * float.Sqrt(m2);
            double reach = r + specP;
            double x0 = Math.Min(x, px) - reach, y0 = Math.Min(y, py) - reach, z0 = Math.Min(z, pz) - reach;
            double x1 = Math.Max(x, px) + reach, y1 = Math.Max(y, py) + reach, z1 = Math.Max(z, pz) + reach;
            int first = _scCount;
            for (int c = d.CandStart; c < d.CandStart + d.CandCount; c++)
            {
                if (!BoxesOverlap(_cBox, 6 * c, x0, y0, z0, x1, y1, z1)) { continue; }
                CollideParticle(d, i, c, first, fast, specP, x, y, z, px, py, pz);
            }
        }
    }

    private void AddContact(int first, in SoftContact sc)
    {
        if (_scCount - first < 4)
        {
            GrowTo(ref _sc, _scCount + 1);
            _sc[_scCount++] = sc;
            return;
        }

        // Keep the four deepest contacts of the particle; ties keep the earlier one.
        int worst = first;
        for (int j = first + 1; j < _scCount; j++) { if (_sc[j].C0 > _sc[worst].C0) { worst = j; } }
        if (sc.C0 < _sc[worst].C0) { _sc[worst] = sc; }
    }

    private void TouchSoftBody(int b)
    {
        if (_rbStamp[b] == _softStamp) { return; }
        _rbStamp[b] = _softStamp;
        int k = 3 * b;
        _rbJ[k] = 0f; _rbJ[k + 1] = 0f; _rbJ[k + 2] = 0f; _rbL[k] = 0f; _rbL[k + 1] = 0f; _rbL[k + 2] = 0f;
        _rbDp[k] = 0f; _rbDp[k + 1] = 0f; _rbDp[k + 2] = 0f; _rbDr[k] = 0f; _rbDr[k + 1] = 0f; _rbDr[k + 2] = 0f;
        WorldInverseInertia(b, _rbIw.AsSpan(6 * b, 6));
        GrowTo(ref _rbTouched, _rbTouchedCount + 1);
        _rbTouched[_rbTouchedCount++] = b;
    }

    private void WorldInverseInertia(int b, Span<float> w)
    {
        var bodies = Core.Bodies;
        var rot = bodies.RotationOf(b);
        var invI = bodies.InverseInertiaOf(b);
        float x = rot[0], y = rot[1], z = rot[2], s = rot[3];
        float xx = x * x, yy = y * y, zz = z * z, xy = x * y, xz = x * z, yz = y * z, sx = s * x, sy = s * y, sz = s * z;
        float r00 = 1f - 2f * (yy + zz), r01 = 2f * (xy - sz), r02 = 2f * (xz + sy);
        float r10 = 2f * (xy + sz), r11 = 1f - 2f * (xx + zz), r12 = 2f * (yz - sx);
        float r20 = 2f * (xz - sy), r21 = 2f * (yz + sx), r22 = 1f - 2f * (xx + yy);
        float ia = invI[0], ib = invI[1], ic = invI[2], id = invI[3], ie = invI[4], ifz = invI[5];
        float t00 = r00 * ia + r01 * ib + r02 * ic, t01 = r00 * ib + r01 * id + r02 * ie, t02 = r00 * ic + r01 * ie + r02 * ifz;
        float t10 = r10 * ia + r11 * ib + r12 * ic, t11 = r10 * ib + r11 * id + r12 * ie, t12 = r10 * ic + r11 * ie + r12 * ifz;
        float t20 = r20 * ia + r21 * ib + r22 * ic, t21 = r20 * ib + r21 * id + r22 * ie, t22 = r20 * ic + r21 * ie + r22 * ifz;
        w[0] = t00 * r00 + t01 * r01 + t02 * r02;
        w[1] = t00 * r10 + t01 * r11 + t02 * r12;
        w[2] = t00 * r20 + t01 * r21 + t02 * r22;
        w[3] = t10 * r10 + t11 * r11 + t12 * r12;
        w[4] = t10 * r20 + t11 * r21 + t12 * r22;
        w[5] = t20 * r20 + t21 * r21 + t22 * r22;
    }

    // Records a contact from a normal pointing from the shape to the particle and the separation of the surfaces.
    private void RecordContact(SoftBodyData d, int i, int c, int first, float nx, float ny, float nz, float sep, double cx, double cy, double cz, double qx, double qy, double qz)
    {
        int s = _cShape[c], b = _cBody[c];
        var bodies = Core.Bodies;
        var sc = new SoftContact { Body = d, Particle = i, Rigid = -1, Nx = nx, Ny = ny, Nz = nz, C0 = sep, X = cx, Y = cy, Z = cz, Mu = float.Sqrt(d.Def.Friction * Friction[s]) };
        if (_cMoving[c])
        {
            sc.Rigid = b;
            d.HadMovingContact = true;
            sc.Coupled = bodies.IsDynamic(b) && bodies.IsAwake(b) && bodies.InverseMass[b] > 0f;
            // Contact point on the shape surface relative to the body's centre of mass at the end of the substep.
            BodyPoseAt(b, 1f, out double bx, out double by, out double bz, out _);
            // (qx, qy, qz) is the surface point
            sc.Rx = (float)(qx - bx); sc.Ry = (float)(qy - by); sc.Rz = (float)(qz - bz);
            if (sc.Coupled) { TouchSoftBody(b); }
        }

        AddContact(first, sc);
    }

    private void CollideParticle(SoftBodyData d, int i, int c, int first, bool fast, float spec, double x, double y, double z, double px, double py, double pz)
    {
        int s = _cShape[c];
        var g = Geoms[GeomOf[s]]!;
        var rw = new QuaternionF(_cRot[4 * c], _cRot[4 * c + 1], _cRot[4 * c + 2], _cRot[4 * c + 3]);
        var inv = rw.Conjugated();
        double sx = _cPos[3 * c], sy = _cPos[3 * c + 1], sz = _cPos[3 * c + 2];
        var sphere = d.SphereShape;

        if (fast)
        {
            // Continuous: sweep the particle from where it started the substep, relative to the shape's own motion in the substep, against the shape's pose at the end.
            double ex = sx - _cPos0[3 * c], ey = sy - _cPos0[3 * c + 1], ez = sz - _cPos0[3 * c + 2];
            double s0x = px - ex, s0y = py - ey, s0z = pz - ez;
            var motion = new Vector3F((float)(x - s0x), (float)(y - s0y), (float)(z - s0z));
            if (TrySweep(d, i, c, first, g, rw, inv, sphere, s0x, s0y, s0z, motion, sx, sy, sz)) { return; }
        }

        var rel = inv.Rotate(new Vector3F((float)(x - sx), (float)(y - sy), (float)(z - sz)));
        var gen = NarrowScratch.Gen;
        var pose = new RigidTransformF(QuaternionF.Identity, rel);
        if (g.IsConvex)
        {
            var m = gen.Collide(g.Convex, sphere, pose, spec);
            FromManifold(d, i, c, first, m, rw, sx, sy, sz, x, y, z);
        }
        else if (g.Compound != null)
        {
            var children = NarrowScratch.Children;
            gen.CollideCompoundConvex(g.Compound, sphere, pose, children, spec);
            for (int k = 0; k < children.Count; k++) { FromManifold(d, i, c, first, children[k].Manifold, rw, sx, sy, sz, x, y, z); }
        }
        else if (g.Soup != null)
        {
            var tris = _softTris;
            tris.Clear();
            g.Soup.CollectCandidates(sphere, pose, spec, tris, _softIds);
            var triBuf = NarrowScratch.TriBuf;
            gen.BeginPin(2);
            for (int t = 0; t < tris.Count; t++)
            {
                var tri = tris[t];
                var tshape = triBuf.Set(tri.V0, tri.V1, tri.V2);
                var m = gen.Collide(tshape, sphere, pose, spec);
                FromManifold(d, i, c, first, m, rw, sx, sy, sz, x, y, z);
            }

            gen.EndPin();
        }
    }

    private void FromManifold(SoftBodyData d, int i, int c, int first, in Manifold3D m, QuaternionF rw, double sx, double sy, double sz, double x, double y, double z)
    {
        if (m.PointCount == 0) { return; }
        int best = 0;
        for (int k = 1; k < m.PointCount; k++) { if (m[k].Separation < m[best].Separation) { best = k; } }
        var p = m[best];
        var n = rw.Rotate(m.Normal);
        var pt = rw.Rotate(p.Point);
        RecordContact(d, i, c, first, n.X, n.Y, n.Z, p.Separation, x, y, z, sx + pt.X, sy + pt.Y, sz + pt.Z);
    }

    private bool TrySweep(SoftBodyData d, int i, int c, int first, Geom3D g, QuaternionF rw, QuaternionF inv, in ConvexShape3D sphere, double s0x, double s0y, double s0z, Vector3F motion, double sx, double sy, double sz)
    {
        float bestF = 2f;
        Vector3F bestN = default;
        if (g.IsConvex || g.Compound != null)
        {
            var bpos = new Vector3F((float)(sx - s0x), (float)(sy - s0y), (float)(sz - s0z));
            if (g.IsConvex)
            {
                if (Gjk3D.ShapeCast(sphere, g.Convex, new RigidTransformF(rw, bpos), motion, out var hit) && !hit.StartsOverlapping && hit.Fraction < bestF) { bestF = hit.Fraction; bestN = -hit.Normal; }
            }
            else
            {
                var comp = g.Compound!;
                for (int j = 0; j < comp.SlotCount; j++)
                {
                    if (!comp.IsLive(j)) { continue; }
                    var child = comp.GetChild(j);
                    var l = child.Local;
                    var place = new RigidTransformF(rw * l.Rotation, bpos + rw.Rotate(l.Translation));
                    if (Gjk3D.ShapeCast(sphere, child.Shape, place, motion, out var hit) && !hit.StartsOverlapping && hit.Fraction < bestF) { bestF = hit.Fraction; bestN = -hit.Normal; }
                }
            }
        }
        else if (g.Soup != null)
        {
            var rel0 = inv.Rotate(new Vector3F((float)(s0x - sx), (float)(s0y - sy), (float)(s0z - sz)));
            if (g.Soup.ShapeCast(sphere, new RigidTransformF(QuaternionF.Identity, rel0), inv.Rotate(motion), out var hit, _softIds) && !hit.StartsOverlapping && hit.Fraction < bestF)
            {
                bestF = hit.Fraction; bestN = rw.Rotate(hit.Normal);
            }
        }

        if (!(bestF < 1f)) { return false; }
        double hx = s0x + motion.X * bestF, hy = s0y + motion.Y * bestF, hz = s0z + motion.Z * bestF;
        // The sphere touches the surface at the hit, so the surface point is a radius back along the normal.
        double r = d.Radius;
        RecordContact(d, i, c, first, bestN.X, bestN.Y, bestN.Z, 0f, hx, hy, hz, hx - bestN.X * r, hy - bestN.Y * r, hz - bestN.Z * r);
        return true;
    }

    private void SolveContacts(float h, float invH)
    {
        var bodies = Core.Bodies;
        for (int ci = 0; ci < _scCount; ci++)
        {
            ref SoftContact c = ref _sc[ci];
            var d = c.Body;
            int k = 3 * c.Particle;
            float wp = d.InvMass[c.Particle];
            float nx = c.Nx, ny = c.Ny, nz = c.Nz;
            float wb = 0f, invM = 0f;
            float rnx = 0f, rny = 0f, rnz = 0f, iwx = 0f, iwy = 0f, iwz = 0f;
            int b = c.Rigid;
            if (c.Coupled)
            {
                invM = bodies.InverseMass[b];
                // r x n and I⁻¹ (r x n)
                rnx = c.Ry * nz - c.Rz * ny; rny = c.Rz * nx - c.Rx * nz; rnz = c.Rx * ny - c.Ry * nx;
                ApplyIw(b, rnx, rny, rnz, out iwx, out iwy, out iwz);
                wb = invM + rnx * iwx + rny * iwy + rnz * iwz;
            }

            float dx = (float)(d.Pos[k] - c.X), dy = (float)(d.Pos[k + 1] - c.Y), dz = (float)(d.Pos[k + 2] - c.Z);
            // The body has already moved away by wb times what earlier passes applied (c.Applied), which opens the gap as well.
            float cv = c.C0 + c.Nx * dx + c.Ny * dy + c.Nz * dz + wb * c.Applied;
            if (!(cv < 0f)) { continue; }
            float wsum = wp + wb;
            if (!(wsum > 0f)) { continue; }
            float lam = -cv / wsum;
            c.Applied += lam;
            float sp = wp * lam;
            d.Pos[k] += nx * sp; d.Pos[k + 1] += ny * sp; d.Pos[k + 2] += nz * sp;
            if (c.Coupled)
            {
                int kb = 3 * b;
                float li = lam * invH;
                // The body is pushed the other way: momentum -n·lam/h, with the matching virtual displacement for the rest of the step.
                _rbJ[kb] -= nx * li; _rbJ[kb + 1] -= ny * li; _rbJ[kb + 2] -= nz * li;
                _rbL[kb] -= rnx * li; _rbL[kb + 1] -= rny * li; _rbL[kb + 2] -= rnz * li;
                float sb = lam * invM;
                _rbDp[kb] -= nx * sb; _rbDp[kb + 1] -= ny * sb; _rbDp[kb + 2] -= nz * sb;
                _rbDr[kb] -= iwx * lam; _rbDr[kb + 1] -= iwy * lam; _rbDr[kb + 2] -= iwz * lam;
            }

            if (c.Mu > 0f) { Friction3(ref c, d, k, wp, lam, h, invH); }
        }
    }

    // Split-impulse style push-out: a particle that starts a substep inside a surface (a body placed resting on a floor, a cloth laid on a box) is only brought out at SoftPushOutSpeed, so the
    // neighbours it is connected to are not dragged along. The contact's constant term is raised by the depth that is not allowed to be removed this substep.
    private const float SoftPushOutSpeed = 1f;

    private void SoftenDepenetration(float h)
    {
        float allowed = SoftPushOutSpeed * h;
        for (int ci = 0; ci < _scCount; ci++)
        {
            ref SoftContact c = ref _sc[ci];
            if (c.Coupled) { continue; }
            var d = c.Body;
            int k = 3 * c.Particle;
            if (d.InvMass[c.Particle] == 0f) { continue; }
            float dx = (float)(d.Prev[k] - c.X), dy = (float)(d.Prev[k + 1] - c.Y), dz = (float)(d.Prev[k + 2] - c.Z);
            float cp = c.C0 + c.Nx * dx + c.Ny * dy + c.Nz * dz;
            if (cp < -allowed) { c.C0 += -cp - allowed; }
        }
    }

    // A particle that started the substep inside a surface is pushed out by the contact; that push is a correction of position, not a motion, so it must not become velocity (a block resting inside a floor
    // by a few millimetres would otherwise leave it at metres per second). The start of the substep is moved out along the normal by the depth it was in, for contacts not coupled to a dynamic body.
    private void ExcludeDepenetration()
    {
        for (int ci = 0; ci < _scCount; ci++)
        {
            ref SoftContact c = ref _sc[ci];
            if (c.Coupled) { continue; }
            var d = c.Body;
            int k = 3 * c.Particle;
            if (d.InvMass[c.Particle] == 0f) { continue; }
            float dx = (float)(d.Prev[k] - c.X), dy = (float)(d.Prev[k + 1] - c.Y), dz = (float)(d.Prev[k + 2] - c.Z);
            float cp = c.C0 + c.Nx * dx + c.Ny * dy + c.Nz * dz;
            if (!(cp < 0f)) { continue; }
            d.Prev[k] -= c.Nx * cp; d.Prev[k + 1] -= c.Ny * cp; d.Prev[k + 2] -= c.Nz * cp;
        }
    }

    // Restitution zero against surfaces that are not coupled to a dynamic body: a particle that rests on a contact keeps no velocity into it. (Against a coupled body the exchange of momentum
    // in the position solve already decides both velocities, and changing the particle's here would break the balance.)
    private void RemoveApproachVelocity()
    {
        var bodies = Core.Bodies;
        for (int ci = 0; ci < _scCount; ci++)
        {
            ref SoftContact c = ref _sc[ci];
            if (c.Coupled) { continue; }
            var d = c.Body;
            int k = 3 * c.Particle;
            if (d.InvMass[c.Particle] == 0f) { continue; }
            float dx = (float)(d.Pos[k] - c.X), dy = (float)(d.Pos[k + 1] - c.Y), dz = (float)(d.Pos[k + 2] - c.Z);
            float cv = c.C0 + c.Nx * dx + c.Ny * dy + c.Nz * dz;
            if (!(cv < 0.1f * d.Radius)) { continue; }
            float svx = 0f, svy = 0f, svz = 0f;
            if (c.Rigid >= 0)
            {
                var v = bodies.LinearVelocityOf(c.Rigid);
                var w = bodies.AngularVelocityOf(c.Rigid);
                svx = v[0] + (w[1] * c.Rz - w[2] * c.Ry); svy = v[1] + (w[2] * c.Rx - w[0] * c.Rz); svz = v[2] + (w[0] * c.Ry - w[1] * c.Rx);
            }

            float vn = (d.Vel[k] - svx) * c.Nx + (d.Vel[k + 1] - svy) * c.Ny + (d.Vel[k + 2] - svz) * c.Nz;
            if (vn < 0f) { d.Vel[k] -= c.Nx * vn; d.Vel[k + 1] -= c.Ny * vn; d.Vel[k + 2] -= c.Nz * vn; }
        }
    }

    private void ApplyIw(int b, float x, float y, float z, out float ox, out float oy, out float oz)
    {
        int q = 6 * b;
        var w = _rbIw;
        ox = w[q] * x + w[q + 1] * y + w[q + 2] * z;
        oy = w[q + 1] * x + w[q + 3] * y + w[q + 4] * z;
        oz = w[q + 2] * x + w[q + 4] * y + w[q + 5] * z;
    }

    // Coulomb friction as a position correction (Müller et al. 2007 / Macklin et al. 2014): the particle's displacement relative to the surface it touches is removed up to mu times the normal correction.
    private void Friction3(ref SoftContact c, SoftBodyData d, int k, float wp, float normalLambda, float h, float invH)
    {
        var bodies = Core.Bodies;
        float nx = c.Nx, ny = c.Ny, nz = c.Nz;
        float ux = (float)(d.Pos[k] - d.Prev[k]), uy = (float)(d.Pos[k + 1] - d.Prev[k + 1]), uz = (float)(d.Pos[k + 2] - d.Prev[k + 2]);
        if (c.Rigid >= 0)
        {
            var v = bodies.LinearVelocityOf(c.Rigid);
            var w = bodies.AngularVelocityOf(c.Rigid);
            float sx = v[0] + (w[1] * c.Rz - w[2] * c.Ry), sy = v[1] + (w[2] * c.Rx - w[0] * c.Rz), sz = v[2] + (w[0] * c.Ry - w[1] * c.Rx);
            ux -= sx * h; uy -= sy * h; uz -= sz * h;
        }

        float un = ux * nx + uy * ny + uz * nz;
        float tx = ux - nx * un, ty = uy - ny * un, tz = uz - nz * un;
        float len = float.Sqrt(tx * tx + ty * ty + tz * tz);
        if (!(len > 1e-12f)) { return; }
        float il = 1f / len;
        tx *= il; ty *= il; tz *= il;
        float wt = wp, invM = 0f, rtx = 0f, rty = 0f, rtz = 0f, iwx = 0f, iwy = 0f, iwz = 0f;
        if (c.Coupled)
        {
            int b = c.Rigid;
            invM = bodies.InverseMass[b];
            rtx = c.Ry * tz - c.Rz * ty; rty = c.Rz * tx - c.Rx * tz; rtz = c.Rx * ty - c.Ry * tx;
            ApplyIw(b, rtx, rty, rtz, out iwx, out iwy, out iwz);
            wt += invM + rtx * iwx + rty * iwy + rtz * iwz;
        }

        if (!(wt > 0f)) { return; }
        float lt = len / wt;
        float cap = c.Mu * normalLambda;
        if (lt > cap) { lt = cap; }
        float sp = wp * lt;
        d.Pos[k] -= tx * sp; d.Pos[k + 1] -= ty * sp; d.Pos[k + 2] -= tz * sp;
        if (c.Coupled)
        {
            int kb = 3 * c.Rigid;
            float li = lt * invH;
            _rbJ[kb] += tx * li; _rbJ[kb + 1] += ty * li; _rbJ[kb + 2] += tz * li;
            _rbL[kb] += rtx * li; _rbL[kb + 1] += rty * li; _rbL[kb + 2] += rtz * li;
            float sb = lt * invM;
            _rbDp[kb] += tx * sb; _rbDp[kb + 1] += ty * sb; _rbDp[kb + 2] += tz * sb;
            _rbDr[kb] += iwx * lt; _rbDr[kb + 1] += iwy * lt; _rbDr[kb + 2] += iwz * lt;
        }
    }

    private void ApplyRigidReactions()
    {
        if (_rbTouchedCount == 0) { return; }
        var bodies = Core.Bodies;
        Array.Sort(_rbTouched, 0, _rbTouchedCount);
        for (int t = 0; t < _rbTouchedCount; t++)
        {
            int b = _rbTouched[t];
            int kb = 3 * b;
            float invM = bodies.InverseMass[b];
            var v = bodies.LinearVelocityOf(b);
            var w = bodies.AngularVelocityOf(b);
            v[0] += _rbJ[kb] * invM; v[1] += _rbJ[kb + 1] * invM; v[2] += _rbJ[kb + 2] * invM;
            ApplyIw(b, _rbL[kb], _rbL[kb + 1], _rbL[kb + 2], out float wx, out float wy, out float wz);
            w[0] += wx; w[1] += wy; w[2] += wz;
            bodies.SleepTime[b] = 0f;
            // The body ends the step where the particles met it: the displacement the reactions gave it is applied to its pose as well, so that no overlap is left to be pushed out next step
            // (an overlap resolved there would turn into velocity and give the body energy).
            var pos = bodies.PositionOf(b);
            pos[0] += WorldCore.Grid(_rbDp[kb]); pos[1] += WorldCore.Grid(_rbDp[kb + 1]); pos[2] += WorldCore.Grid(_rbDp[kb + 2]);
            var rot = bodies.RotationOf(b);
            float dx = _rbDr[kb], dy = _rbDr[kb + 1], dz = _rbDr[kb + 2];
            float qx = rot[0], qy = rot[1], qz = rot[2], qw = rot[3];
            // q += ½ (ω_virtual, 0) ⊗ q
            float nx = qw * dx + (dy * qz - dz * qy), ny = qw * dy + (dz * qx - dx * qz), nz = qw * dz + (dx * qy - dy * qx), nw = -(dx * qx + dy * qy + dz * qz);
            qx += 0.5f * nx; qy += 0.5f * ny; qz += 0.5f * nz; qw += 0.5f * nw;
            float len = float.Sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
            if (len > 0f) { rot[0] = qx / len; rot[1] = qy / len; rot[2] = qz / len; rot[3] = qw / len; }
        }
    }

    // ------------------------------------------------------------------ particle against particle

    private bool SoftPairAllowed(SoftBodyDef a, SoftBodyDef b)
    {
        if (((LayerMatrix[a.Layer] >> b.Layer) & 1u) == 0) { return false; }
        if (a.GroupIndex == b.GroupIndex && a.GroupIndex != 0) { return a.GroupIndex > 0; }
        return (a.MaskBits & b.CategoryBits) != 0 && (a.CategoryBits & b.MaskBits) != 0;
    }

    private static long CellHash(long cx, long cy, long cz)
    {
        ulong h = (ulong)cx * 0x9E3779B97F4A7C15UL ^ (ulong)cy * 0xC2B2AE3D27D4EB4FUL ^ (ulong)cz * 0x165667B19E3779F9UL;
        h ^= h >> 31; h *= 0x94D049BB133111EBUL; h ^= h >> 29;
        return (long)(h & ((1UL << 39) - 1));
    }

    private readonly long[] _gridSeen = new long[8];

    private void BuildPairs()
    {
        _pairCount = 0;
        int total = 0;
        float rmax = 0f;
        bool any = _awakeSoftCount > 1;
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            var d = _awakeSoft[a];
            total += d.N;
            if (d.Radius > rmax) { rmax = d.Radius; }
            if (d.Def.SelfCollision) { any = true; }
        }

        if (!any || total < 2) { return; }
        double cs = 6.0 * rmax, icsz = 1.0 / cs;
        var keys = _gridKeys;
        for (int g = 0; g < total; g++)
        {
            var d = _gData[g];
            int k = 3 * _gPart[g];
            long cx = (long)Math.Floor(d.Pos[k] * icsz), cy = (long)Math.Floor(d.Pos[k + 1] * icsz), cz = (long)Math.Floor(d.Pos[k + 2] * icsz);
            keys[g] = (CellHash(cx, cy, cz) << 24) | (long)g;
        }

        Array.Sort(keys, 0, total);
        // Runs of equal cell hash and a table from hash to run.
        int runs = 0;
        GrowTo(ref _gridCell, total + 1);
        for (int e = 0; e < total; e++)
        {
            long hsh = keys[e] >> 24;
            if (e == 0 || hsh != (keys[e - 1] >> 24)) { _gridCell[runs] = hsh; _gridStart[runs] = e; runs++; }
        }

        _gridStart[runs] = total;
        int size = 16;
        while (size < 2 * runs) { size <<= 1; }
        GrowTo(ref _gridTable, size);
        Array.Fill(_gridTable, -1, 0, size);
        int mask = size - 1;
        for (int u = 0; u < runs; u++)
        {
            int slot = (int)(_gridCell[u] & mask);
            while (_gridTable[slot] >= 0) { slot = (slot + 1) & mask; }
            _gridTable[slot] = u;
        }

        var seen = _gridSeen;
        for (int g = 0; g < total; g++)
        {
            var di = _gData[g];
            int pi = _gPart[g];
            int k = 3 * pi;
            double fx = di.Pos[k] * icsz, fy = di.Pos[k + 1] * icsz, fz = di.Pos[k + 2] * icsz;
            long cx = (long)Math.Floor(fx), cy = (long)Math.Floor(fy), cz = (long)Math.Floor(fz);
            int sx = fx - cx < 0.5 ? -1 : 1, sy = fy - cy < 0.5 ? -1 : 1, sz = fz - cz < 0.5 ? -1 : 1;
            int nseen = 0;
            for (int q = 0; q < 8; q++)
            {
                long hsh = CellHash(cx + ((q & 1) != 0 ? sx : 0), cy + ((q & 2) != 0 ? sy : 0), cz + ((q & 4) != 0 ? sz : 0));
                bool dup = false;
                for (int u = 0; u < nseen; u++) { if (seen[u] == hsh) { dup = true; break; } }
                if (dup) { continue; }
                seen[nseen++] = hsh;
                int slot = (int)(hsh & mask);
                int run = -1;
                while (_gridTable[slot] >= 0)
                {
                    if (_gridCell[_gridTable[slot]] == hsh) { run = _gridTable[slot]; break; }
                    slot = (slot + 1) & mask;
                }

                if (run < 0) { continue; }
                for (int e = _gridStart[run]; e < _gridStart[run + 1]; e++)
                {
                    int j = (int)(keys[e] & 0xFFFFFF);
                    if (j <= g) { continue; }
                    var dj = _gData[j];
                    int pj = _gPart[j];
                    if (di.InvMass[pi] == 0f && dj.InvMass[pj] == 0f) { continue; }
                    if (di == dj)
                    {
                        if (!di.Def.SelfCollision || Neighbours(di, pi, pj)) { continue; }
                    }
                    else if (!SoftPairAllowed(di.Def, dj.Def)) { continue; }

                    int kj = 3 * pj;
                    float ddx = (float)(di.Pos[k] - dj.Pos[kj]), ddy = (float)(di.Pos[k + 1] - dj.Pos[kj + 1]), ddz = (float)(di.Pos[k + 2] - dj.Pos[kj + 2]);
                    float rr = (di.Radius + dj.Radius) * 1.5f;
                    if (ddx * ddx + ddy * ddy + ddz * ddz >= rr * rr) { continue; }
                    GrowTo(ref _pairA, _pairCount + 1); GrowTo(ref _pairB, _pairCount + 1);
                    _pairA[_pairCount] = g; _pairB[_pairCount] = j; _pairCount++;
                }
            }
        }
    }

    // Particles that share an edge or a hinge, or rest closer than their spheres, never collide with each other.
    private static bool Neighbours(SoftBodyData d, int a, int b)
    {
        long key = a < b ? ((long)a << 20) | (long)b : ((long)b << 20) | (long)a;
        if (Array.BinarySearch(d.EdgeKey, key) >= 0 || Array.BinarySearch(d.HingeKey, key) >= 0) { return true; }
        float dx = d.Rest[3 * a] - d.Rest[3 * b], dy = d.Rest[3 * a + 1] - d.Rest[3 * b + 1], dz = d.Rest[3 * a + 2] - d.Rest[3 * b + 2];
        float rr = 2f * d.Radius * 1.05f;
        return dx * dx + dy * dy + dz * dz < rr * rr;
    }

    private void SolvePairs()
    {
        for (int p = 0; p < _pairCount; p++)
        {
            int gi = _pairA[p], gj = _pairB[p];
            var di = _gData[gi]; var dj = _gData[gj];
            int pi = _gPart[gi], pj = _gPart[gj];
            float wi = di.InvMass[pi], wj = dj.InvMass[pj];
            float wsum = wi + wj;
            if (wsum == 0f) { continue; }
            int ki = 3 * pi, kj = 3 * pj;
            float dx = (float)(di.Pos[ki] - dj.Pos[kj]), dy = (float)(di.Pos[ki + 1] - dj.Pos[kj + 1]), dz = (float)(di.Pos[ki + 2] - dj.Pos[kj + 2]);
            float d2 = dx * dx + dy * dy + dz * dz;
            float rr = di.Radius + dj.Radius;
            if (!(d2 < rr * rr)) { continue; }
            float len = float.Sqrt(d2);
            float nx, ny, nz;
            if (len > 1e-9f) { float il = 1f / len; nx = dx * il; ny = dy * il; nz = dz * il; }
            else { nx = 0f; ny = 1f; nz = 0f; }
            float lam = (rr - len) / wsum;
            float si = wi * lam, sj = wj * lam;
            di.Pos[ki] += nx * si; di.Pos[ki + 1] += ny * si; di.Pos[ki + 2] += nz * si;
            dj.Pos[kj] -= nx * sj; dj.Pos[kj + 1] -= ny * sj; dj.Pos[kj + 2] -= nz * sj;
        }
    }

    // ------------------------------------------------------------------ sleep

    private void UpdateSoftSleep(float dt)
    {
        float thr2 = Opt.SleepThreshold * Opt.SleepThreshold;
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            var d = _awakeSoft[a];
            float vmax2 = 0f;
            for (int i = 0; i < d.N; i++)
            {
                float vx = d.Vel[3 * i], vy = d.Vel[3 * i + 1], vz = d.Vel[3 * i + 2];
                float v2 = vx * vx + vy * vy + vz * vz;
                if (v2 > vmax2) { vmax2 = v2; }
            }

            if (!Opt.SleepEnabled || !d.Def.AllowSleep || d.HadMovingContact || !(vmax2 < thr2)) { d.SleepTime = 0f; continue; }
            d.SleepTime += dt;
            if (d.SleepTime >= Opt.TimeToSleep)
            {
                d.Asleep = true;
                Array.Clear(d.Vel);
            }
        }
    }
}
