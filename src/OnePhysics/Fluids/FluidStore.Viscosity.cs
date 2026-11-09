using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Fluids;

// Implicit viscosity of fluids (honey, lava): before the particles move in a substep, the velocity field is replaced by the solution of
//
//     (M + h R) v' = M v,      R_ij = c_ij x_ij x_ij^T,
//
// the implicit Euler step of dv/dt = (1/rho) div(eta (grad v + grad v^T)) with the pairwise central-force SPH form of the Laplacian (Weiler, Koschier, Brand, Bender 2018, section 4; Peer, Ihmsen, Cornelis, Teschner 2015):
// a_i = 2 (d + 2) sum_j (m_j / rho_j) (eta_ij / rho_i) (v_ij . x_ij) / (|x_ij|^2 + 0.01 h_k^2) grad W_ij. The force is central and equal and opposite for the pair, so linear and angular momentum are conserved, and the matrix is
// symmetric positive definite, so a Jacobi-preconditioned conjugate-gradient solve converges. The solve runs a fixed maximum number of iterations with every dot product summed over chunks of 256 particles in chunk order, so its
// result does not depend on the thread count. Positions are then rebuilt from the solved velocities so the density constraint sees the strain the viscosity allowed.
internal sealed partial class FluidStore
{
    private bool _anyViscous;
    private float[] _etaF = new float[MaxFluids];

    // Per neighbour slot (same indexing as _nbr): the viscous coupling coefficient of the pair.
    private float[] _gk = Array.Empty<float>();

    private const int ViscosityMaxIterations = 32;
    private const double ViscosityTolerance = 1e-3;

    private float[] _ux = new float[64], _uy = new float[64], _uz = new float[64];
    private float[] _rx = new float[64], _ry = new float[64], _rz = new float[64];
    private float[] _px = new float[64], _py = new float[64], _pz = new float[64];
    private float[] _qx = new float[64], _qy = new float[64], _qz = new float[64];
    private float[] _zx = new float[64], _zy = new float[64], _zz = new float[64];
    private float[] _ix = new float[64], _iy = new float[64], _iz = new float[64];
    private double _cgAlpha, _cgBeta;
    internal int LastViscosityIterations;

    private void SetUpViscosity()
    {
        _anyViscous = false;
        for (int f = 0; f < FluidHandles.HighWater; f++)
        {
            if (!FluidHandles.IsLive(f)) { continue; }
            var d = Fluids[f]!.Def;
            _etaF[f] = d.DynamicViscosity;
            if (d.DynamicViscosity > 0f && !Fluids[f]!.Mpm) { _anyViscous = true; }
        }

        if (_anyViscous)
        {
            if (_gk.Length < _nbr.Length) { _gk = new float[_nbr.Length]; }
            int n = Count;
            Fit(ref _ux, n); Fit(ref _uy, n); Fit(ref _uz, n);
            Fit(ref _rx, n); Fit(ref _ry, n); Fit(ref _rz, n);
            Fit(ref _px, n); Fit(ref _py, n); Fit(ref _pz, n);
            Fit(ref _qx, n); Fit(ref _qy, n); Fit(ref _qz, n);
            Fit(ref _zx, n); Fit(ref _zy, n); Fit(ref _zz, n);
            Fit(ref _ix, n); Fit(ref _iy, n); Fit(ref _iz, n);
        }
    }

    private double SumChunks(int which)
    {
        double s = 0.0;
        for (int t = 0; t < _taskCount; t++)
        {
            var b = _tb[t];
            s += which == 0 ? b.D0 : b.D1;
        }

        return s;
    }

    /// <summary>Solves for the velocities after viscosity, stores them in <c>PVx..</c> and moves the particles with them.</summary>
    private void SolveViscosity()
    {
        var ex = Host.Core.Executor;
        ex.For(_taskCount, new ViscSetupTask { S = this });
        double rz = SumChunks(0), rr0 = SumChunks(1);
        LastViscosityIterations = 0;
        if (rr0 > 1e-30)
        {
            double rr = rr0;
            double tol2 = ViscosityTolerance * ViscosityTolerance * rr0;
            for (int it = 0; it < ViscosityMaxIterations; it++)
            {
                ex.For(_taskCount, new ViscMultiplyTask { S = this });
                double pq = SumChunks(0);
                if (!(pq > 1e-60)) { break; }
                _cgAlpha = rz / pq;
                ex.For(_taskCount, new ViscUpdateTask { S = this });
                double rzNew = SumChunks(0);
                rr = SumChunks(1);
                LastViscosityIterations = it + 1;
                if (rr <= tol2) { break; }
                _cgBeta = rzNew / rz;
                rz = rzNew;
                ex.For(_taskCount, new ViscDirectionTask { S = this });
            }
        }

        ex.For(_taskCount, new ViscStoreTask { S = this });
    }

    private struct ViscSetupTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task) { S.ViscSetupRange(task, task * Chunk, Math.Min(S.Count, task * Chunk + Chunk)); }
    }

    private void ViscSetupRange(int task, int from, int to)
    {
        float h2 = _k.H2, sp = _k.Spiky, hh = _k.H, eps = 0.01f * h2;
        float pref = 2f * (Dim + 2);
        int stride = _stride;
        double rz = 0.0, rr = 0.0;
        for (int i = from; i < to; i++)
        {
            int f = PFluid[i];
            float vxi = PVx[i], vyi = PVy[i], vzi = PVz[i];
            _ux[i] = vxi; _uy[i] = vyi; _uz[i] = vzi;
            if (_isMpm[f])
            {
                _rx[i] = 0f; _ry[i] = 0f; _rz[i] = 0f; _ix[i] = 1f; _iy[i] = 1f; _iz[i] = 1f;
                _zx[i] = 0f; _zy[i] = 0f; _zz[i] = 0f; _px[i] = 0f; _py[i] = 0f; _pz[i] = 0f;
                int b0 = i * stride;
                for (int a = 0; a < _nbrCount[i]; a++) { _gk[b0 + a] = 0f; }
                continue;
            }

            float xi = _x[i], yi = _y[i], zi = _z[i];
            float vi = _mass[f] / _n0[f];
            float etai = _etaF[f];
            float dgx = _mass[f], dgy = _mass[f], dgz = _mass[f];
            float rxi = 0f, ryi = 0f, rzi = 0f;
            int bas = i * stride, cnt = _nbrCount[i];
            for (int a = 0; a < cnt; a++)
            {
                int j = _nbr[bas + a];
                float dx = xi - _x[j], dy = yi - _y[j], dz = zi - _z[j];
                float r2 = dx * dx + dy * dy + dz * dz;
                float c = 0f;
                if (r2 < h2 && r2 > 1e-18f)
                {
                    int fj = PFluid[j];
                    float etaij = 0.5f * (etai + _etaF[fj]);
                    if (etaij > 0f)
                    {
                        float r = float.Sqrt(r2);
                        float q = hh - r;
                        float vj = _mass[fj] / _n0[fj];
                        c = pref * vi * vj * etaij * (sp * q * q / r) / (r2 + eps);
                        float dvx = vxi - PVx[j], dvy = vyi - PVy[j], dvz = vzi - PVz[j];
                        float dot = dx * dvx + dy * dvy + dz * dvz;
                        rxi -= _h * c * dot * dx; ryi -= _h * c * dot * dy; rzi -= _h * c * dot * dz;
                        dgx += _h * c * dx * dx; dgy += _h * c * dy * dy; dgz += _h * c * dz * dz;
                    }
                }

                _gk[bas + a] = c;
            }

            float ix = 1f / dgx, iy = 1f / dgy, iz = 1f / dgz;
            _ix[i] = ix; _iy[i] = iy; _iz[i] = iz;
            _rx[i] = rxi; _ry[i] = ryi; _rz[i] = rzi;
            float zx = ix * rxi, zy = iy * ryi, zz = iz * rzi;
            _zx[i] = zx; _zy[i] = zy; _zz[i] = zz;
            _px[i] = zx; _py[i] = zy; _pz[i] = zz;
            rz += (double)(rxi * zx + ryi * zy + rzi * zz);
            rr += (double)(rxi * rxi + ryi * ryi + rzi * rzi);
        }

        var b = _tb[task]; b.D0 = rz; b.D1 = rr;
    }

    private struct ViscMultiplyTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task) { S.ViscMultiplyRange(task, task * Chunk, Math.Min(S.Count, task * Chunk + Chunk)); }
    }

    // q = (M + h R) p, and the chunk's part of p . q.
    private void ViscMultiplyRange(int task, int from, int to)
    {
        int stride = _stride;
        double pq = 0.0;
        for (int i = from; i < to; i++)
        {
            float m = _mass[PFluid[i]];
            float xi = _x[i], yi = _y[i], zi = _z[i];
            float pxi = _px[i], pyi = _py[i], pzi = _pz[i];
            float qx = m * pxi, qy = m * pyi, qz = m * pzi;
            int bas = i * stride, cnt = _nbrCount[i];
            for (int a = 0; a < cnt; a++)
            {
                float c = _gk[bas + a];
                if (c == 0f) { continue; }
                int j = _nbr[bas + a];
                float dx = xi - _x[j], dy = yi - _y[j], dz = zi - _z[j];
                float dot = dx * (pxi - _px[j]) + dy * (pyi - _py[j]) + dz * (pzi - _pz[j]);
                float k = _h * c * dot;
                qx += k * dx; qy += k * dy; qz += k * dz;
            }

            _qx[i] = qx; _qy[i] = qy; _qz[i] = qz;
            pq += (double)(pxi * qx + pyi * qy + pzi * qz);
        }

        _tb[task].D0 = pq;
    }

    private struct ViscUpdateTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task) { S.ViscUpdateRange(task, task * Chunk, Math.Min(S.Count, task * Chunk + Chunk)); }
    }

    // u += alpha p; r -= alpha q; z = D^-1 r; the chunk's parts of r . z and r . r.
    private void ViscUpdateRange(int task, int from, int to)
    {
        float al = (float)_cgAlpha;
        double rz = 0.0, rr = 0.0;
        for (int i = from; i < to; i++)
        {
            _ux[i] += al * _px[i]; _uy[i] += al * _py[i]; _uz[i] += al * _pz[i];
            float rx = _rx[i] - al * _qx[i], ry = _ry[i] - al * _qy[i], rzv = _rz[i] - al * _qz[i];
            _rx[i] = rx; _ry[i] = ry; _rz[i] = rzv;
            float zx = _ix[i] * rx, zy = _iy[i] * ry, zz = _iz[i] * rzv;
            _zx[i] = zx; _zy[i] = zy; _zz[i] = zz;
            rz += (double)(rx * zx + ry * zy + rzv * zz);
            rr += (double)(rx * rx + ry * ry + rzv * rzv);
        }

        var b = _tb[task]; b.D0 = rz; b.D1 = rr;
    }

    private struct ViscDirectionTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task)
        {
            var s = S;
            int from = task * Chunk, to = Math.Min(s.Count, from + Chunk);
            float be = (float)s._cgBeta;
            for (int i = from; i < to; i++)
            {
                s._px[i] = s._zx[i] + be * s._px[i]; s._py[i] = s._zy[i] + be * s._py[i]; s._pz[i] = s._zz[i] + be * s._pz[i];
            }
        }
    }

    private struct ViscStoreTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task)
        {
            var s = S;
            int from = task * Chunk, to = Math.Min(s.Count, from + Chunk);
            bool d3 = s.Dim == 3;
            for (int i = from; i < to; i++)
            {
                if (s._isMpm[s.PFluid[i]]) { continue; }
                float vx = s._ux[i], vy = s._uy[i], vz = d3 ? s._uz[i] : 0f;
                s.PVx[i] = vx; s.PVy[i] = vy; s.PVz[i] = vz;
                // The solved velocity is the one the particle moves with: positions are rebuilt from the start of the substep.
                s._x[i] = s._x0[i] + vx * s._h; s._y[i] = s._y0[i] + vy * s._h; s._z[i] = s._z0[i] + vz * s._h;
            }
        }
    }
}
