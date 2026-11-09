using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Fluids;

internal sealed partial class FluidStore
{
    private float[] _epsA = new float[MaxFluids];
    private float _spacing2;

    // ------------------------------------------------------------------ predict

    private struct PredictTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task)
        {
            var s = S;
            int from = task * Chunk, to = Math.Min(s.Count, from + Chunk);
            float h = s._h, gx = s._g.X * h, gy = s._g.Y * h, gz = s._g.Z * h;
            for (int i = from; i < to; i++)
            {
                int f = s.PFluid[i];
                float gs = s._gscale[f];
                float vx = s.PVx[i] + gx * gs, vy = s.PVy[i] + gy * gs, vz = s.PVz[i] + gz * gs;
                float m = s._maxSpeed[f];
                float sq = vx * vx + vy * vy + vz * vz;
                if (sq > m * m) { float k = m / float.Sqrt(sq); vx *= k; vy *= k; vz *= k; }
                s.PVx[i] = vx; s.PVy[i] = vy; s.PVz[i] = vz;
                s._x0[i] = s._x[i]; s._y0[i] = s._y[i]; s._z0[i] = s._z[i];
                s._x[i] += vx * h; s._y[i] += vy * h; s._z[i] += vz * h;
            }
        }
    }

    // ------------------------------------------------------------------ density constraint

    // Macklin and Müller 2013 (Position Based Fluids): C_i = rho_i / rho_0 - 1 and lambda_i = -C_i / (sum_k |grad_k C_i|^2 + eps), with the particles' own masses so that two fluids may meet.
    // Only compression is corrected (C > 0); a deficit at the free surface is not pulled together.
    private struct LambdaTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task)
        {
            var s = S;
            s.LambdaRange(task * Chunk, Math.Min(s.Count, task * Chunk + Chunk));
        }
    }

    private void LambdaRange(int from, int to)
    {
        float h2 = _k.H2, c6 = _k.Poly6, sp = _k.Spiky, hh = _k.H;
        float selfW = c6 * h2 * h2 * h2;
        int stride = _stride;
        for (int i = from; i < to; i++)
        {
            int f = PFluid[i];
            float xi = _x[i], yi = _y[i], zi = _z[i];
            float n = _mass[f] * selfW;
            float gsx = 0f, gsy = 0f, gsz = 0f, sumSq = 0f;
            int bas = i * stride, cnt = _nbrCount[i];
            for (int a = 0; a < cnt; a++)
            {
                int j = _nbr[bas + a];
                float dx = xi - _x[j], dy = yi - _y[j], dz = zi - _z[j];
                float r2 = dx * dx + dy * dy + dz * dz;
                if (!(r2 < h2)) { continue; }
                float mj = _mass[PFluid[j]];
                float t = h2 - r2;
                n += mj * c6 * t * t * t;
                float r = float.Sqrt(r2);
                if (r > 1e-9f)
                {
                    float q = hh - r;
                    float coef = -sp * q * q / r * mj;
                    float gx = coef * dx, gy = coef * dy, gz = coef * dz;
                    gsx += gx; gsy += gy; gsz += gz;
                    sumSq += gx * gx + gy * gy + gz * gz;
                }
            }

            _rho[i] = n;
            float c = n / _n0[f] - 1f;
            if (!(c > 0f)) { _lam[i] = 0f; continue; }
            float inv = 1f / _n0[f];
            float denom = inv * inv * (gsx * gsx + gsy * gsy + gsz * gsz + sumSq) + _epsA[f];
            _lam[i] = -c / denom;
        }
    }

    private struct DeltaTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task)
        {
            var s = S;
            s.DeltaRange(task * Chunk, Math.Min(s.Count, task * Chunk + Chunk));
        }
    }

    private void DeltaRange(int from, int to)
    {
        float h2 = _k.H2, c6 = _k.Poly6, sp = _k.Spiky, hh = _k.H, invDq = _k.InvPoly6Dq;
        int stride = _stride;
        float cap = 0.25f * hh, cap2 = cap * cap;
        for (int i = from; i < to; i++)
        {
            int f = PFluid[i];
            float xi = _x[i], yi = _y[i], zi = _z[i];
            float mi = _mass[f], n0i = _n0[f], lami = _lam[i];
            float tens = _tens[f] * _spacing2;
            float dpx = 0f, dpy = 0f, dpz = 0f;
            int bas = i * stride, cnt = _nbrCount[i];
            for (int a = 0; a < cnt; a++)
            {
                int j = _nbr[bas + a];
                float dx = xi - _x[j], dy = yi - _y[j], dz = zi - _z[j];
                float r2 = dx * dx + dy * dy + dz * dz;
                if (!(r2 < h2)) { continue; }
                float r = float.Sqrt(r2);
                if (!(r > 1e-9f)) { continue; }
                int fj = PFluid[j];
                float mj = _mass[fj];
                float q = hh - r;
                float coef = -sp * q * q / r;
                float t = h2 - r2;
                // Artificial pressure, s_corr = -k (W(r) / W(dq))^4 (Macklin and Müller 2013, equation 13): keeps particles from clumping when the neighbourhood is thin.
                float ratio = c6 * t * t * t * invDq;
                float r4 = ratio * ratio; r4 *= r4;
                float scorr = -tens * r4;
                float w = (lami + scorr) * mj / n0i + _lam[j] * mi / _n0[fj];
                float k = w * coef;
                dpx += k * dx; dpy += k * dy; dpz += k * dz;
            }

            float l2 = dpx * dpx + dpy * dpy + dpz * dpz;
            if (l2 > cap2) { float k = cap / float.Sqrt(l2); dpx *= k; dpy *= k; dpz *= k; }
            _dx[i] = dpx; _dy[i] = dpy; _dz[i] = dpz;
        }
    }

    // ------------------------------------------------------------------ apply the corrections and the contacts that need no reaction

    private struct ApplyTask : IParallelBody
    {
        public FluidStore S;
        public bool Move;
        public void Run(int task)
        {
            var s = S;
            int from = task * Chunk, to = Math.Min(s.Count, from + Chunk);
            for (int i = from; i < to; i++)
            {
                if (Move) { s._x[i] += s._dx[i]; s._y[i] += s._dy[i]; s._z[i] += s._dz[i]; }
                int cf = s._cFirst[i];
                if (cf >= 0) { s.SolveStatic(i, cf, s._cCount[i]); }
            }
        }
    }

    // ------------------------------------------------------------------ velocity update, viscosity, vorticity

    private struct FinishTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task)
        {
            var s = S;
            int from = task * Chunk, to = Math.Min(s.Count, from + Chunk);
            float inv = 1f / s._h;
            for (int i = from; i < to; i++)
            {
                float vx = (s._x[i] - s._x0[i]) * inv, vy = (s._y[i] - s._y0[i]) * inv, vz = (s._z[i] - s._z0[i]) * inv;
                float m = s._maxSpeed[s.PFluid[i]];
                float sq = vx * vx + vy * vy + vz * vz;
                if (sq > m * m) { float k = m / float.Sqrt(sq); vx *= k; vy *= k; vz *= k; }
                s.PVx[i] = vx; s.PVy[i] = vy; s.PVz[i] = vz;
            }
        }
    }

    // XSPH viscosity (Schechter and Bridson 2012 as used by Macklin and Müller): v_i += c sum_j (m_j / rho_j) (v_j - v_i) W_ij, and the curl omega_i of the velocity field for vorticity confinement.
    private struct SmoothTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task)
        {
            var s = S;
            s.SmoothRange(task * Chunk, Math.Min(s.Count, task * Chunk + Chunk));
        }
    }

    private void SmoothRange(int from, int to)
    {
        float h2 = _k.H2, c6 = _k.Poly6, sp = _k.Spiky, hh = _k.H;
        int stride = _stride;
        for (int i = from; i < to; i++)
        {
            int f = PFluid[i];
            float c = _visc[f];
            float vxi = PVx[i], vyi = PVy[i], vzi = PVz[i];
            float xi = _x[i], yi = _y[i], zi = _z[i];
            float ax = 0f, ay = 0f, az = 0f, ox = 0f, oy = 0f, oz = 0f;
            int bas = i * stride, cnt = _nbrCount[i];
            for (int a = 0; a < cnt; a++)
            {
                int j = _nbr[bas + a];
                float dx = xi - _x[j], dy = yi - _y[j], dz = zi - _z[j];
                float r2 = dx * dx + dy * dy + dz * dz;
                if (!(r2 < h2)) { continue; }
                float vol = _mass[PFluid[j]] / _rho[j];
                float wvx = PVx[j] - vxi, wvy = PVy[j] - vyi, wvz = PVz[j] - vzi;
                float t = h2 - r2;
                float w = c6 * t * t * t * vol;
                ax += wvx * w; ay += wvy * w; az += wvz * w;
                if (_anyVort)
                {
                    float r = float.Sqrt(r2);
                    if (r > 1e-9f)
                    {
                        float q = hh - r;
                        float coef = -sp * q * q / r * vol;
                        float gx = coef * dx, gy = coef * dy, gz = coef * dz;
                        // (v_j - v_i) x grad W
                        ox += wvy * gz - wvz * gy; oy += wvz * gx - wvx * gz; oz += wvx * gy - wvy * gx;
                    }
                }
            }

            _nvx[i] = vxi + c * ax; _nvy[i] = vyi + c * ay; _nvz[i] = vzi + c * az;
            _omx[i] = ox; _omy[i] = oy; _omz[i] = oz;
        }
    }

    private struct ConfineTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task)
        {
            var s = S;
            s.ConfineRange(task * Chunk, Math.Min(s.Count, task * Chunk + Chunk));
        }
    }

    // Vorticity confinement (Fedkiw, Stam, Jensen 2001; Macklin and Müller 2013): a force eps (N x omega) with N the direction of the gradient of |omega| puts back the small-scale swirl that the solver damps.
    private void ConfineRange(int from, int to)
    {
        float h2 = _k.H2, sp = _k.Spiky, hh = _k.H;
        int stride = _stride;
        for (int i = from; i < to; i++)
        {
            int f = PFluid[i];
            float eps = _vort[f];
            float vx = _nvx[i], vy = _nvy[i], vz = _nvz[i];
            if (eps > 0f)
            {
                float xi = _x[i], yi = _y[i], zi = _z[i];
                float ex = 0f, ey = 0f, ez = 0f;
                int bas = i * stride, cnt = _nbrCount[i];
                for (int a = 0; a < cnt; a++)
                {
                    int j = _nbr[bas + a];
                    float dx = xi - _x[j], dy = yi - _y[j], dz = zi - _z[j];
                    float r2 = dx * dx + dy * dy + dz * dz;
                    if (!(r2 < h2)) { continue; }
                    float r = float.Sqrt(r2);
                    if (!(r > 1e-9f)) { continue; }
                    float oj = float.Sqrt(_omx[j] * _omx[j] + _omy[j] * _omy[j] + _omz[j] * _omz[j]);
                    float q = hh - r;
                    float coef = -sp * q * q / r * (_mass[PFluid[j]] / _rho[j]) * oj;
                    ex += coef * dx; ey += coef * dy; ez += coef * dz;
                }

                float el = float.Sqrt(ex * ex + ey * ey + ez * ez);
                if (el > 1e-9f)
                {
                    float nx = ex / el, ny = ey / el, nz = ez / el;
                    float ox = _omx[i], oy = _omy[i], oz = _omz[i];
                    // N x omega
                    float fx = ny * oz - nz * oy, fy = nz * ox - nx * oz, fz = nx * oy - ny * ox;
                    float k = eps * _h;
                    vx += fx * k; vy += fy * k; vz += fz * k;
                }
            }

            PVx[i] = vx; PVy[i] = vy; PVz[i] = vz;
        }
    }
}
