using System;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Fluids;

// Solid matter (sand, gravel, snow, clay, dough, mud) with the material point method on the grid the particles are already sorted into.
//
//  * Transfers: moving least squares MPM (Hu, Fang, Ge, Qu, Zhu, Pradhana, Jiang 2018) with quadratic B-spline weights; the grid nodes sit at the cell centres, so a node gathers from the 3^d cells around it (node-centric, in
//    the fixed order of the sorted particles) instead of scattering from the particles. Every node sums its own particles in the same order whatever the thread count.
//  * Elasticity: the elastic left Cauchy-Green tensor b = F_e F_e^T is tracked (not F), its eigenvalues are the squared principal stretches, and the Kirchhoff stress is Hencky's: tau_i = 2 mu eps_i + lambda tr(eps), eps_i = ln(s_i).
//  * Plasticity by return mapping in the principal stresses: Drucker-Prager for granular matter (Klár, Gast, Pradhana, Fu, Batty, Teran, Sifakis, Jiang 2016, with the cone matched to Mohr-Coulomb), von Mises for clay and dough
//    (Gao et al. 2017), and the singular-value clamp with hardening for snow (Stomakhin, Schroeder, Chai, Teran, Selle 2013).
//  * The stress is explicit, so the substep follows the elastic wave speed (a CFL number of 0.3 on the cell width); the world takes as many substeps as that needs. Contacts with rigid bodies, soft bodies and rods are the particle contacts
//    of the fluids, solved on the positions after the transfer, and their reaction is the coupling.
internal sealed partial class FluidStore
{
    private bool[] _isMpm = new bool[MaxFluids];
    private bool _anyMpm, _anyPbf;
    private ulong _mpmMask;
    private float _mpmFriction;

    // Per fluid.
    private float[] _sLame1 = new float[MaxFluids], _sLame2 = new float[MaxFluids], _sAlpha = new float[MaxFluids], _sApex = new float[MaxFluids], _sVm = new float[MaxFluids];
    private float[] _rhoRest = new float[MaxFluids], _sBulk = new float[MaxFluids], _sPic = new float[MaxFluids];
    // Calibrations of the granular model, fitted to poured piles in both dimensions (the angle of repose follows FrictionAngle within a few degrees):
    // (The PIC share applies to granular matter only and the bulk damping not to clay and dough, which have a yield stress and keep their elastic recovery undamped.)
    //  * BulkDamping: bulk viscosity as a share of rho c dx, which takes the bounce out of the elastic compression of a stream that lands on a pile;
    //  * PicBlend: the share of the affine velocity field that the transfer drops (0 is APIC, 1 is PIC), which damps the sub-cell shear that grains lose to collisions;
    //  * PressureFloor: the pressure a particle always carries, in units of the weight of one particle layer (rho g spacing), so the thin sheet of grains at a free surface has some friction to hold it on a slope.
    private const float BulkDamping = 0.25f, PicBlend = 0.3f, PressureFloor = 1f;
    private float _gFloor;
    private float[] _sSin = new float[MaxFluids], _sDSin = new float[MaxFluids];
    private float[] _sCritC = new float[MaxFluids], _sCritS = new float[MaxFluids], _sXi = new float[MaxFluids], _sVol = new float[MaxFluids];

    // Per step.
    private float[] _aff = new float[9], _fx = new float[1], _fy = new float[1], _fz = new float[1];
    private float[] _rhoP = new float[1];
    private float[] _nm = new float[1], _gvx = new float[1], _gvy = new float[1], _gvz = new float[1];
    private int _nnx, _nny, _nnz;

    private const float SolidCfl = 0.3f;

    /// <summary>Grid nodes within this many cell widths outside a surface are held from moving into it, so a particle never finds the nodes under it free to fall.</summary>
    private const float BoundaryMargin = 0.25f;

    /// <summary>The inertial number at which the friction of flowing granular matter is half way to its fast-flow value (mu(I) rheology: Jop, Forterre, Pouliquen 2006), and the rise of the friction angle in fast flow, in degrees.</summary>
    private const float InertialI0 = 0.279f, FlowFrictionRise = 12f;
    private const double PlanarConeCalibration = 1.2;
    private const float MaxHardening = 4f;
    private const float OverpackRatio = 1.1f;
    /// <summary>The most log-volume of expansion a granular particle keeps for later (a factor of about 20 in volume).</summary>
    private const float MaxVolumeCorrection = 3f;
    private const int MaxSolidSubsteps = 64;

    internal void DeriveSolid(FluidState s)
    {
        var d = s.Def;
        s.Mpm = d.ElasticModulus > 0f;
        s.Lame1 = 0f; s.Lame2 = 0f; s.DpAlpha = 0f; s.DpApex = 0f; s.VmLimit = 0f;
        double sp = Spacing;
        s.Volume = (float)(Dim == 3 ? sp * sp * sp : sp * sp);
        if (!s.Mpm) { return; }
        double e = d.ElasticModulus, nu = d.PoissonRatio;
        s.Lame2 = (float)(e / (2.0 * (1.0 + nu)));
        s.Lame1 = (float)(e * nu / ((1.0 + nu) * (1.0 - 2.0 * nu)));
        // Drucker-Prager cone |tau_dev| <= slope (p + p0) matched to Mohr-Coulomb in plane strain, the state of the surface layer of a slope (2D exactly, 3D with the middle principal stress halfway between the others):
        // slope = sqrt(2) sin(phi), p0 = c / tan(phi). The triaxial-compression match of Klár et al. 2016 is stronger by about a third at 30 degrees and holds piles at steeper angles than the friction angle.
        double alpha = 0.0, apex = 0.0;
        s.FlowSin = 0f; s.FlowDSin = 0f;
        if (d.FrictionAngle > 0f)
        {
            double rad = (double)d.FrictionAngle * (Math.PI / 180.0);
            // Calibration of the cone to poured piles: the pile of a planar world stands about 5 degrees under the friction angle with the exact cone, a 3D pile on it (fits in docs/spec/fluids.md).
            double cal = Dim == 2 ? PlanarConeCalibration : 1.0;
            double sinTrue = Math64.Sin(rad);
            double sin0 = Math.Min(sinTrue * cal, 0.95);
            alpha = Math.Sqrt(2.0) * sin0;
            if (d.Cohesion > 0f) { apex = d.Cohesion * Math64.Cos(rad) / sinTrue; }
            double sin1 = Math.Max(sin0, Math.Min(Math64.Sin(Math.Min((double)d.FrictionAngle + FlowFrictionRise, 85.0) * (Math.PI / 180.0)) * cal, 0.95));
            s.FlowSin = (float)sin0; s.FlowDSin = (float)(sin1 - sin0);
        }

        s.DpAlpha = (float)alpha;
        s.DpApex = (float)apex;
        double vm = d.YieldStress > 0f ? Math.Sqrt(2.0 / 3.0) * d.YieldStress : 0.0;
        // Cohesion without friction is a pressure-independent shear strength: the largest shear stress equals it, which is a deviatoric norm of sqrt(2) times as much.
        if (alpha == 0.0 && d.Cohesion > 0f) { double c = Math.Sqrt(2.0) * d.Cohesion; vm = vm > 0.0 ? Math.Min(vm, c) : c; }
        s.VmLimit = (float)vm;
    }

    private void EnsureSolid()
    {
        if (PBe != null) { return; }
        int c = PId.Length;
        PBe = new float[c * 6]; PCm = new float[c * 9]; PAux = new float[c];
    }

    private void InitSolid(int p)
    {
        var be = PBe!;
        be[p * 6] = 1f; be[p * 6 + 1] = 1f; be[p * 6 + 2] = 1f; be[p * 6 + 3] = 0f; be[p * 6 + 4] = 0f; be[p * 6 + 5] = 0f;
        var cm = PCm!;
        for (int k = 0; k < 9; k++) { cm[p * 9 + k] = 0f; }
        PAux![p] = 1f;
    }

    private void SetUpSolids()
    {
        _anyMpm = false; _anyPbf = false; _mpmMask = 0;
        _gFloor = PressureFloor * Math.Max(Math.Abs(Host.Gravity.Y), Math.Max(Math.Abs(Host.Gravity.X), Math.Abs(Host.Gravity.Z)));
        for (int f = 0; f < FluidHandles.HighWater; f++)
        {
            if (!FluidHandles.IsLive(f)) { continue; }
            var s = Fluids[f]!; var d = s.Def;
            _isMpm[f] = s.Mpm;
            bool granular = d.FrictionAngle > 0f || d.Cohesion > 0f;
            _sPic[f] = granular ? PicBlend : 0f;
            _sBulk[f] = (d.YieldStress == 0f ? BulkDamping : 0f) * d.RestDensity * float.Sqrt((s.Lame1 + 2f * s.Lame2) / d.RestDensity) * Spacing * KernelScale;
            _rhoRest[f] = d.RestDensity; _sSin[f] = s.FlowSin; _sDSin[f] = s.FlowDSin;
            _sLame1[f] = s.Lame1; _sLame2[f] = s.Lame2; _sAlpha[f] = s.DpAlpha; _sApex[f] = s.DpApex; _sVm[f] = s.VmLimit;
            if (s.Mpm) { if (_mpmMask == 0) { _mpmFriction = d.Friction; } _mpmMask |= 1UL << f; }
            _sCritC[f] = d.CriticalCompression; _sCritS[f] = d.CriticalStretch; _sXi[f] = d.Hardening; _sVol[f] = s.Volume;
        }

        for (int p = 0; p < Count; p++)
        {
            if (_isMpm[PFluid[p]]) { _anyMpm = true; } else { _anyPbf = true; }
            if (_anyMpm && _anyPbf) { break; }
        }

    }

    /// <summary>The number of substeps of the step: the world setting, raised to what the stiffest solid needs to keep a wave from crossing more than 0.3 of a cell per substep (at most 64).</summary>
    private int SubstepCount(float dt, float vmax)
    {
        int n = Settings.Substeps;
        if (!_anyMpm) { return n; }
        float dx = Spacing * KernelScale;
        float need = 0f;
        for (int f = 0; f < FluidHandles.HighWater; f++)
        {
            if (!FluidHandles.IsLive(f) || !_isMpm[f]) { continue; }
            var d = Fluids[f]!.Def;
            float hard = d.Hardening > 0f ? MaxHardening : 1f;
            float c = float.Sqrt(hard * (_sLame1[f] + 2f * _sLame2[f]) / d.RestDensity);
            need = Math.Max(need, dt * (c + vmax) / (SolidCfl * dx));
            if (d.DynamicViscosity > 0f) { need = Math.Max(need, dt * d.DynamicViscosity / (0.2f * d.RestDensity * dx * dx)); }
        }

        int m = (int)Math.Ceiling(need);
        return Math.Min(Math.Max(n, m), Math.Max(n, MaxSolidSubsteps));
    }

    // ------------------------------------------------------------------ one substep of the transfer

    private void SolidTransfer()
    {
        int n = Count;
        _nnx = _gnx + 2; _nny = _gny + 2; _nnz = Dim == 3 ? _gnz + 2 : 1;
        long nodes = (long)_nnx * _nny * _nnz;
        if (nodes > 1L << 26) { throw new FluidBudgetException("The solid matter's grid would have " + nodes.ToString(System.Globalization.CultureInfo.InvariantCulture) + " nodes."); }
        Fit(ref _aff, n * 9); Fit(ref _rhoP, n); Fit(ref _fx, n); Fit(ref _fy, n); Fit(ref _fz, n);
        int nn = (int)nodes;
        Fit(ref _nm, nn); Fit(ref _gvx, nn); Fit(ref _gvy, nn); Fit(ref _gvz, nn);
        var ex = Host.Core.Executor;
        // Density first (mass on the nodes, then back to the particles), because a particle's stress needs it: the elastic strain follows the velocity gradient, which cannot see grains packing closer than a cell.
        ex.For(_taskCount, new SolidFracTask { S = this });
        ex.For((nn + Chunk - 1) / Chunk, new SolidMassTask { S = this, Nodes = nn });
        ex.For(_taskCount, new SolidDensityTask { S = this });
        ex.For(_taskCount, new SolidPrepareTask { S = this });
        ex.For((nn + Chunk - 1) / Chunk, new SolidGridTask { S = this, Nodes = nn });
        SolidBoundary();
        ex.For(_taskCount, new SolidG2PTask { S = this });
    }

    private struct SolidFracTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task)
        {
            var s = S; int d = s.Dim; float invDx = s._invH;
            for (int i = task * Chunk, to = Math.Min(s.Count, task * Chunk + Chunk); i < to; i++)
            {
                if (!s._isMpm[s.PFluid[i]]) { continue; }
                float ux = (s._x[i] - s._gMinX) * invDx, uy = (s._y[i] - s._gMinY) * invDx, uz = d == 3 ? (s._z[i] - s._gMinZ) * invDx : 0f;
                s._fx[i] = ux - (float)Math.Floor(ux) - 0.5f; s._fy[i] = uy - (float)Math.Floor(uy) - 0.5f; s._fz[i] = d == 3 ? uz - (float)Math.Floor(uz) - 0.5f : 0f;
            }
        }
    }

    // Mass of every node alone (the momentum pass repeats it): the same node-centric gather in the sorted order of the particles.
    private struct SolidMassTask : IParallelBody
    {
        public FluidStore S;
        public int Nodes;
        public void Run(int task)
        {
            var s = S; int d = s.Dim; var cs = s._cellStart;
            int nnx = s._nnx, nny = s._nny, gnx = s._gnx, gny = s._gny, gnz = s._gnz, zr = d == 3 ? 1 : 0;
            for (int n = task * Chunk, to = Math.Min(Nodes, task * Chunk + Chunk); n < to; n++)
            {
                int nx = n % nnx, ny = (n / nnx) % nny, nz = n / (nnx * nny);
                int cxn = nx - 1, cyn = ny - 1, czn = d == 3 ? nz - 1 : 0;
                float m = 0f;
                for (int oz = -zr; oz <= zr; oz++)
                {
                    int cz = czn + oz;
                    if (cz < 0 || cz >= gnz) { continue; }
                    for (int oy = -1; oy <= 1; oy++)
                    {
                        int cy = cyn + oy;
                        if (cy < 0 || cy >= gny) { continue; }
                        for (int ox = -1; ox <= 1; ox++)
                        {
                            int cx = cxn + ox;
                            if (cx < 0 || cx >= gnx) { continue; }
                            int cell = cx + gnx * (cy + gny * cz);
                            for (int j = cs[cell], b = cs[cell + 1]; j < b; j++)
                            {
                                int f = s.PFluid[j];
                                if (!s._isMpm[f]) { continue; }
                                float w = Spline(-ox - s._fx[j]) * Spline(-oy - s._fy[j]) * (d == 3 ? Spline(-oz - s._fz[j]) : 1f);
                                m += w * s._mass[f];
                            }
                        }
                    }
                }

                s._nm[n] = m;
            }
        }
    }

    // The particle's density: the node masses interpolated back and divided by the cell volume.
    private struct SolidDensityTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task)
        {
            var s = S; int d = s.Dim; int nnx = s._nnx, nny = s._nny, zr = d == 3 ? 1 : 0;
            float dx = s._k.H, inv = d == 3 ? 1f / (dx * dx * dx) : 1f / (dx * dx);
            for (int i = task * Chunk, to = Math.Min(s.Count, task * Chunk + Chunk); i < to; i++)
            {
                if (!s._isMpm[s.PFluid[i]]) { continue; }
                float ux = (s._x[i] - s._gMinX) * s._invH, uy = (s._y[i] - s._gMinY) * s._invH, uz = d == 3 ? (s._z[i] - s._gMinZ) * s._invH : 0f;
                int cx = (int)Math.Floor(ux), cy = (int)Math.Floor(uy), cz = d == 3 ? (int)Math.Floor(uz) : 0;
                float fx = s._fx[i], fy = s._fy[i], fz = s._fz[i];
                float rho = 0f;
                for (int oz = -zr; oz <= zr; oz++)
                {
                    for (int oy = -1; oy <= 1; oy++)
                    {
                        for (int ox = -1; ox <= 1; ox++)
                        {
                            float w = Spline(ox - fx) * Spline(oy - fy) * (d == 3 ? Spline(oz - fz) : 1f);
                            if (w == 0f) { continue; }
                            rho += w * s._nm[(cx + ox + 1) + nnx * ((cy + oy + 1) + nny * (d == 3 ? cz + oz + 1 : 0))];
                        }
                    }
                }

                s._rhoP[i] = rho * inv;
            }
        }
    }

    private struct SolidPrepareTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task) { S.PrepareRange(task * Chunk, Math.Min(S.Count, task * Chunk + Chunk)); }
    }

    // Quadratic B-spline of the distance d (in cell widths) between a node and a particle.
    private static float Spline(float d)
    {
        float a = Math.Abs(d);
        if (a < 0.5f) { return 0.75f - a * a; }
        if (a < 1.5f) { float t = 1.5f - a; return 0.5f * t * t; }
        return 0f;
    }

    // Symmetric 2x2 eigen decomposition by one Jacobi rotation (only + - * / sqrt): returns the eigenvalues and the eigenvector columns (c, -s) and (s, c) of the rotation.
    private static void Eigen2(float a, float b, float c, out float e0, out float e1, out float cs, out float sn)
    {
        if (Math.Abs(b) <= 1e-30f * (Math.Abs(a) + Math.Abs(c)) || b == 0f) { e0 = a; e1 = c; cs = 1f; sn = 0f; return; }
        float theta = (c - a) / (2f * b);
        float t = (theta >= 0f ? 1f : -1f) / (Math.Abs(theta) + float.Sqrt(theta * theta + 1f));
        cs = 1f / float.Sqrt(t * t + 1f); sn = t * cs;
        e0 = a - t * b; e1 = c + t * b;
    }

    // Everything a particle needs before the transfer: its position inside its cell, the plastic return of the elastic strain left by the last substep, the stress, and the affine matrix
    // A = m C - h V (4 / dx^2) tau that the grid adds to the momentum at (node - particle).
    private void PrepareRange(int from, int to)
    {
        int d = Dim;
        float invDx = _invH, dx = _k.H, h = _h;
        float four = 4f * invDx * invDx;
        var be = PBe!; var cm = PCm!; var aux = PAux!;
        for (int i = from; i < to; i++)
        {
            int f = PFluid[i];
            if (!_isMpm[f]) { continue; }
            float ux = (_x[i] - _gMinX) * invDx, uy = (_y[i] - _gMinY) * invDx, uz = d == 3 ? (_z[i] - _gMinZ) * invDx : 0f;
            _fx[i] = ux - (float)Math.Floor(ux) - 0.5f; _fy[i] = uy - (float)Math.Floor(uy) - 0.5f; _fz[i] = d == 3 ? uz - (float)Math.Floor(uz) - 0.5f : 0f;

            int b0 = i * 6, c0 = i * 9;
            float bxx = be[b0], byy = be[b0 + 1], bzz = be[b0 + 2], bxy = be[b0 + 3], bxz = be[b0 + 4], byz = be[b0 + 5];
            // Principal stretches and directions: U[:,k] = (u0k, u1k, u2k).
            float e0, e1, e2;
            float u00, u01, u02, u10, u11, u12, u20, u21, u22;
            if (d == 3)
            {
                var eig = SymmetricEigenF.Decompose(new Matrix3x3F(bxx, bxy, bxz, bxy, byy, byz, bxz, byz, bzz));
                e0 = eig.Values.X; e1 = eig.Values.Y; e2 = eig.Values.Z;
                var v = eig.Vectors;
                u00 = v.M11; u01 = v.M12; u02 = v.M13; u10 = v.M21; u11 = v.M22; u12 = v.M23; u20 = v.M31; u21 = v.M32; u22 = v.M33;
            }
            else
            {
                Eigen2(bxx, bxy, byy, out e0, out e1, out float cs, out float sn);
                e2 = bzz;
                u00 = cs; u01 = sn; u02 = 0f; u10 = -sn; u11 = cs; u12 = 0f; u20 = 0f; u21 = 0f; u22 = 1f;
            }

            if (!(float.IsFinite(e0) && float.IsFinite(e1) && float.IsFinite(e2)))
            {
                // A particle whose strain left the numbers is reset to rest.
                InitSolid(i);
                e0 = 1f; e1 = 1f; e2 = 1f; u00 = 1f; u01 = 0f; u02 = 0f; u10 = 0f; u11 = 1f; u12 = 0f; u20 = 0f; u21 = 0f; u22 = 1f;
            }

            float st0 = float.Sqrt(Math.Max(e0, 0.01f)), st1 = float.Sqrt(Math.Max(e1, 0.01f)), st2 = d == 3 ? float.Sqrt(Math.Max(e2, 0.01f)) : 1f;
            float mu = _sLame2[f], lam = _sLame1[f];

            // Snow: the elastic stretches are clamped, the plastic volume ratio follows and both Lamé parameters harden with its compaction (Stomakhin et al. 2013, equations 4 to 6).
            float critC = _sCritC[f], critS = _sCritS[f];
            if (critC > 0f || critS > 0f)
            {
                float lo = critC > 0f ? 1f - critC : 0.1f, hi = critS > 0f ? 1f + critS : 10f;
                float c0s = Math.Min(Math.Max(st0, lo), hi), c1s = Math.Min(Math.Max(st1, lo), hi), c2s = d == 3 ? Math.Min(Math.Max(st2, lo), hi) : 1f;
                float ratio = (st0 * st1 * st2) / (c0s * c1s * c2s);
                st0 = c0s; st1 = c1s; st2 = c2s;
                float jp = Math.Min(Math.Max(aux[i] * ratio, 0.05f), 20f);
                aux[i] = jp;
                float xi = _sXi[f];
                if (xi > 0f)
                {
                    float hard = Math.Min(Math.Max(Math32.Exp(Math.Min(xi * (1f - jp), 1.4f)), 0.25f), MaxHardening);
                    mu *= hard; lam *= hard;
                }

            }

            float ep0 = Math32.Log(st0), ep1 = Math32.Log(st1), ep2 = d == 3 ? Math32.Log(st2) : 0f;
            float tr = d == 3 ? ep0 + ep1 + ep2 : ep0 + ep1;
            float third = tr / d;

            // von Mises: the deviatoric stress is limited in norm; the volumetric part stays elastic.
            float vm = _sVm[f];
            if (vm > 0f)
            {
                float d0 = ep0 - third, d1 = ep1 - third, d2 = d == 3 ? ep2 - third : 0f;
                float norm = 2f * mu * float.Sqrt(d0 * d0 + d1 * d1 + d2 * d2);
                if (norm > vm)
                {
                    float k = vm / norm;
                    ep0 = third + d0 * k; ep1 = third + d1 * k; ep2 = d == 3 ? third + d2 * k : 0f;
                    }
            }

            // Drucker-Prager: the stress is brought back onto the cone |tau_dev| <= alpha d (p + p0), or to its tip when the pressure is below -p0.
            if (_sSin[f] > 0f)
            {
                // Volume correction (Klár et al. 2016, section 4.4): returning an expanded state to the tip of the cone discards the expansion from the elastic strain, so the particles would stay loose for ever.
                // The log-volume that was discarded is kept (aux holds one plus it) and added back to the strain, so a loose pile has no stress until it is compacted to its rest density again.
                bool corrected = !(critC > 0f || critS > 0f);
                float vcSaved = corrected ? aux[i] - 1f : 0f;
                if (vcSaved != 0f)
                {
                    float share = vcSaved / d;
                    ep0 += share; ep1 += share; if (d == 3) { ep2 += share; }
                    tr += vcSaved; third = tr / d;
                }

                float lam2 = 2f * mu + d * lam;
                float t0 = 2f * mu * ep0 + lam * tr, t1 = 2f * mu * ep1 + lam * tr, t2 = d == 3 ? 2f * mu * ep2 + lam * tr : 0f;
                float p = -(d == 3 ? t0 + t1 + t2 : t0 + t1) / d;
                // The friction angle rises with the inertial number I = shear rate x grain size / sqrt(p / rho), from its static value towards the fast-flow one.
                float cth = (d == 3 ? cm[c0] + cm[c0 + 4] + cm[c0 + 8] : cm[c0] + cm[c0 + 4]) / d;
                float rxx = cm[c0] - cth, ryy = cm[c0 + 4] - cth, rzz = d == 3 ? cm[c0 + 8] - cth : 0f;
                float rxy = 0.5f * (cm[c0 + 1] + cm[c0 + 3]), rxz = 0.5f * (cm[c0 + 2] + cm[c0 + 6]), ryz = 0.5f * (cm[c0 + 5] + cm[c0 + 7]);
                float shear = float.Sqrt(2f * (rxx * rxx + ryy * ryy + rzz * rzz + 2f * (rxy * rxy + rxz * rxz + ryz * ryz)));
                float inertial = p > 1e-3f ? shear * Spacing * float.Sqrt(_rhoRest[f] / p) : 1e3f;
                float sinEff = _sSin[f] + _sDSin[f] * (inertial / (inertial + InertialI0));
                float slope = 1.4142135f * sinEff;
                float apex = _sApex[f];
                if (p + apex <= 0f)
                {
                    float e = apex / lam2;
                    // The strain left is the tip's; the expansion beyond it is the volume held back.
                    if (corrected) { aux[i] = 1f + Math.Min(Math.Max(tr - d * e, 0f), MaxVolumeCorrection); }
                    ep0 = e; ep1 = e; ep2 = d == 3 ? e : 0f;
                }
                else
                {
                    if (corrected) { aux[i] = 1f; }
                    float h0 = t0 + p, h1 = t1 + p, h2 = d == 3 ? t2 + p : 0f;
                    float norm = float.Sqrt(h0 * h0 + h1 * h1 + h2 * h2);
                    float limit = slope * (Math.Max(p, _gFloor * _rhoRest[f] * Spacing) + apex);
                    if (norm > limit)
                    {
                        float k = limit / norm;
                        ep0 = h0 * k / (2f * mu) + third; ep1 = h1 * k / (2f * mu) + third; ep2 = d == 3 ? h2 * k / (2f * mu) + third : 0f;
                            }
                }
            }

            // Stress and the new elastic strain, from the final principal strains.
            float tr2 = d == 3 ? ep0 + ep1 + ep2 : ep0 + ep1;
            float ta = 2f * mu * ep0 + lam * tr2, tb = 2f * mu * ep1 + lam * tr2, tc = d == 3 ? 2f * mu * ep2 + lam * tr2 : 0f;
            // Overpacking: a strain that follows the velocity gradient cannot see grains pressed closer than a cell (they move as one smooth field), so a stream that lands on a pile packs several times its rest density with no
            // stress. The density the grid holds (node masses interpolated back) exceeds what the elastic volume change accounts for, ln(rho / rho0) + tr(eps), only by that excess, and the excess is resisted by the bulk modulus
            // of the particle, in compression only and beyond a 10% dead zone that the lattice and the free surface stay inside.
            if (_rhoP[i] > OverpackRatio * _rhoRest[f])
            {
                float excess = Math32.Log(_rhoP[i] / _rhoRest[f]) + tr2;
                if (excess > 0f)
                {
                    float iso = -(lam + 2f * mu / d) * Math.Min(excess, 1f);
                    ta += iso; tb += iso; if (d == 3) { tc += iso; }
                }
            }

            {
                float f0 = Math32.Exp(2f * ep0), f1 = Math32.Exp(2f * ep1), f2 = d == 3 ? Math32.Exp(2f * ep2) : 1f;
                be[b0] = u00 * u00 * f0 + u01 * u01 * f1 + u02 * u02 * f2; be[b0 + 1] = u10 * u10 * f0 + u11 * u11 * f1 + u12 * u12 * f2; be[b0 + 2] = u20 * u20 * f0 + u21 * u21 * f1 + u22 * u22 * f2;
                be[b0 + 3] = u00 * u10 * f0 + u01 * u11 * f1 + u02 * u12 * f2; be[b0 + 4] = u00 * u20 * f0 + u01 * u21 * f1 + u02 * u22 * f2; be[b0 + 5] = u10 * u20 * f0 + u11 * u21 * f1 + u12 * u22 * f2;
            }

            float sxx = u00 * u00 * ta + u01 * u01 * tb + u02 * u02 * tc, syy = u10 * u10 * ta + u11 * u11 * tb + u12 * u12 * tc, szz = u20 * u20 * ta + u21 * u21 * tb + u22 * u22 * tc;
            float sxy = u00 * u10 * ta + u01 * u11 * tb + u02 * u12 * tc, sxz = u00 * u20 * ta + u01 * u21 * tb + u02 * u22 * tc, syz = u10 * u20 * ta + u11 * u21 * tb + u12 * u22 * tc;

            // Viscous stress of the velocity gradient of the last transfer: 2 eta times the deviatoric rate of strain.
            float eta = _etaF[f];
            float c00 = cm[c0 + 0], c01 = cm[c0 + 1], c02 = cm[c0 + 2], c10 = cm[c0 + 3], c11 = cm[c0 + 4], c12 = cm[c0 + 5], c20 = cm[c0 + 6], c21 = cm[c0 + 7], c22 = cm[c0 + 8];
            {
                // Bulk viscosity (a Neumann-Richtmyer damping of the volumetric waves): the elastic compression of a stream that hits a pile would otherwise come back as a bounce, which grains do not do.
                float divC = d == 3 ? c00 + c11 + c22 : c00 + c11;
                // Compression only: a viscous resistance to expansion would hold a loose pile up like a tension.
                float bulk = _sBulk[f] * (divC < 0f ? divC : 0f);
                sxx += bulk; syy += bulk; szz += d == 3 ? bulk : 0f;
            }

            if (eta > 0f)
            {
                float thirdC = (d == 3 ? c00 + c11 + c22 : c00 + c11) / d;
                sxx += 2f * eta * (c00 - thirdC); syy += 2f * eta * (c11 - thirdC); szz += d == 3 ? 2f * eta * (c22 - thirdC) : 0f;
                sxy += eta * (c01 + c10); sxz += eta * (c02 + c20); syz += eta * (c12 + c21);
            }

            float m = _mass[f];
            float k0 = -h * _sVol[f] * four;
            int a0 = i * 9;
            float ma = m * (1f - _sPic[f]);
            _aff[a0 + 0] = ma * c00 + k0 * sxx; _aff[a0 + 1] = ma * c01 + k0 * sxy; _aff[a0 + 2] = ma * c02 + k0 * sxz;
            _aff[a0 + 3] = ma * c10 + k0 * sxy; _aff[a0 + 4] = ma * c11 + k0 * syy; _aff[a0 + 5] = ma * c12 + k0 * syz;
            _aff[a0 + 6] = ma * c20 + k0 * sxz; _aff[a0 + 7] = ma * c21 + k0 * syz; _aff[a0 + 8] = ma * c22 + k0 * szz;
        }

        _ = dx;
    }

    private struct SolidGridTask : IParallelBody
    {
        public FluidStore S;
        public int Nodes;
        public void Run(int task) { S.GridRange(task * Chunk, Math.Min(Nodes, task * Chunk + Chunk)); }
    }

    // Particles to grid, one node at a time: mass and momentum m (v + g h) + A (x_node - x_particle) of every particle whose cell is next to the node's, then the velocity.
    private void GridRange(int from, int to)
    {
        int d = Dim;
        float dx = _k.H, h = _h;
        var cs = _cellStart;
        int nnx = _nnx, nny = _nny, gnx = _gnx, gny = _gny, gnz = _gnz;
        int zr = d == 3 ? 1 : 0;
        for (int n = from; n < to; n++)
        {
            int nx = n % nnx, ny = (n / nnx) % nny, nz = n / (nnx * nny);
            int cxn = nx - 1, cyn = ny - 1, czn = d == 3 ? nz - 1 : 0;
            float m = 0f, mx = 0f, my = 0f, mz = 0f;
            for (int oz = -zr; oz <= zr; oz++)
            {
                int cz = czn + oz;
                if (cz < 0 || cz >= gnz) { continue; }
                for (int oy = -1; oy <= 1; oy++)
                {
                    int cy = cyn + oy;
                    if (cy < 0 || cy >= gny) { continue; }
                    for (int ox = -1; ox <= 1; ox++)
                    {
                        int cx = cxn + ox;
                        if (cx < 0 || cx >= gnx) { continue; }
                        int cell = cx + gnx * (cy + gny * cz);
                        int a = cs[cell], b = cs[cell + 1];
                        // The node is at offset (-ox, -oy, -oz) from the cell the particle is in.
                        for (int j = a; j < b; j++)
                        {
                            int f = PFluid[j];
                            if (!_isMpm[f]) { continue; }
                            float dX = -ox - _fx[j], dY = -oy - _fy[j], dZ = d == 3 ? -oz - _fz[j] : 0f;
                            float w = Spline(dX) * Spline(dY) * (d == 3 ? Spline(dZ) : 1f);
                            if (w == 0f) { continue; }
                            float mp = _mass[f], gs = _gscale[f] * h;
                            int a0 = j * 9;
                            float px = dX * dx, py = dY * dx, pz = dZ * dx;
                            m += w * mp;
                            mx += w * (mp * (PVx[j] + _g.X * gs) + _aff[a0] * px + _aff[a0 + 1] * py + _aff[a0 + 2] * pz);
                            my += w * (mp * (PVy[j] + _g.Y * gs) + _aff[a0 + 3] * px + _aff[a0 + 4] * py + _aff[a0 + 5] * pz);
                            mz += w * (mp * (PVz[j] + _g.Z * gs) + _aff[a0 + 6] * px + _aff[a0 + 7] * py + _aff[a0 + 8] * pz);
                        }
                    }
                }
            }

            _nm[n] = m;
            if (m > 1e-20f) { float inv = 1f / m; _gvx[n] = mx * inv; _gvy[n] = my * inv; _gvz[n] = d == 3 ? mz * inv : 0f; }
            else { _gvx[n] = 0f; _gvy[n] = 0f; _gvz[n] = 0f; }
        }
    }

    /// <summary>
    /// Grid nodes inside a shape cannot be left to fall through it: a node at or under a surface loses its velocity into the surface (relative to the surface's own motion), and the tangential velocity loses up to the Coulomb limit of the
    /// normal change. What a node loses a body that moves freely gains, as momentum and angular momentum about its centre, so the node boundary is also the two-way coupling (Gao et al. 2017). Serial, in candidate and node order.
    /// </summary>
    private void SolidBoundary()
    {
        if (_candCount == 0) { return; }
        int d = Dim;
        float dx = _k.H, invDx = _invH;
        int nnx = _nnx, nny = _nny, nnz = _nnz;
        for (int c = 0; c < _candCount; c++)
        {
            if ((_cAllow[c] & _mpmMask) == 0) { continue; }
            int o = 6 * c;
            double u0 = (_cBox[o] - _ox - _gMinX) * invDx + 0.5 - 1.0, u1 = (_cBox[o + 3] - _ox - _gMinX) * invDx + 0.5 + 1.0;
            double v0 = (_cBox[o + 1] - _oy - _gMinY) * invDx + 0.5 - 1.0, v1 = (_cBox[o + 4] - _oy - _gMinY) * invDx + 0.5 + 1.0;
            double w0 = d == 3 ? (_cBox[o + 2] - _oz - _gMinZ) * invDx + 0.5 - 1.0 : 0.0, w1 = d == 3 ? (_cBox[o + 5] - _oz - _gMinZ) * invDx + 0.5 + 1.0 : 0.0;
            if (u1 < 0.0 || v1 < 0.0 || u0 > nnx - 1 || v0 > nny - 1 || (d == 3 && (w1 < 0.0 || w0 > nnz - 1))) { continue; }
            int x0 = (int)Math.Max(0.0, Math.Floor(u0)), x1 = (int)Math.Min(nnx - 1, Math.Ceiling(u1));
            int y0 = (int)Math.Max(0.0, Math.Floor(v0)), y1 = (int)Math.Min(nny - 1, Math.Ceiling(v1));
            int z0 = d == 3 ? (int)Math.Max(0.0, Math.Floor(w0)) : 0, z1 = d == 3 ? (int)Math.Min(nnz - 1, Math.Ceiling(w1)) : 0;
            int bi = _cBody[c];
            for (int nz = z0; nz <= z1; nz++)
            {
                for (int ny = y0; ny <= y1; ny++)
                {
                    for (int nx = x0; nx <= x1; nx++)
                    {
                        int n = nx + nnx * (ny + nny * nz);
                        float m = _nm[n];
                        if (!(m > 0f)) { continue; }
                        double wx = _ox + _gMinX + (nx - 0.5) * dx, wy = _oy + _gMinY + (ny - 0.5) * dx, wz = d == 3 ? _oz + _gMinZ + (nz - 0.5) * dx : 0.0;
                        if (!Host.CollideSphere(_cShape[c], in _cFrame[c], wx, wy, wz, 1e-4f * dx, BoundaryMargin * dx, out FluidHit hit)) { continue; }
                        float nxn = hit.Nx, nyn = hit.Ny, nzn = d == 3 ? hit.Nz : 0f;
                        float svx = 0f, svy = 0f, svz = 0f, rx = 0f, ry = 0f, rz = 0f;
                        bool dyn = false;
                        if (bi >= 0)
                        {
                            ref readonly var b = ref _bodies[bi];
                            rx = (float)(wx - b.X); ry = (float)(wy - b.Y); rz = (float)(wz - b.Z);
                            svx = b.Vx + (b.Wy * rz - b.Wz * ry); svy = b.Vy + (b.Wz * rx - b.Wx * rz); svz = b.Vz + (b.Wx * ry - b.Wy * rx);
                            dyn = b.Dynamic;
                        }

                        float ovx = _gvx[n], ovy = _gvy[n], ovz = _gvz[n];
                        float rvx = ovx - svx, rvy = ovy - svy, rvz = ovz - svz;
                        float vn = rvx * nxn + rvy * nyn + rvz * nzn;
                        if (!(vn < 0f)) { continue; }
                        rvx -= vn * nxn; rvy -= vn * nyn; rvz -= vn * nzn;
                        float mu = float.Sqrt(_mpmFriction * hit.Friction);
                        if (mu > 0f)
                        {
                            float tl = float.Sqrt(rvx * rvx + rvy * rvy + rvz * rvz);
                            if (tl > 0f) { float k = Math.Max(0f, 1f - mu * -vn / tl); rvx *= k; rvy *= k; rvz *= k; }
                        }

                        float nvx = rvx + svx, nvy = rvy + svy, nvz = rvz + svz;
                        _gvx[n] = nvx; _gvy[n] = nvy; _gvz[n] = nvz;
                        if (dyn)
                        {
                            // The node's momentum change is the body's with the opposite sign.
                            float px = m * (nvx - ovx), py = m * (nvy - ovy), pz = m * (nvz - ovz);
                            ref var b = ref _bodies[bi];
                            b.Jx -= px; b.Jy -= py; b.Jz -= pz;
                            b.Lx -= ry * pz - rz * py; b.Ly -= rz * px - rx * pz; b.Lz -= rx * py - ry * px;
                        }
                    }
                }
            }
        }
    }

    private struct SolidG2PTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task) { S.GatherRange(task * Chunk, Math.Min(S.Count, task * Chunk + Chunk)); }
    }

    // Grid to particles: velocity, the affine velocity C = (4 / dx^2) sum w v (x_node - x_p)^T, the new position and the new elastic strain b <- (I + h C) b (I + h C)^T.
    private void GatherRange(int from, int to)
    {
        int d = Dim;
        float dx = _k.H, invDx = _invH, h = _h;
        float four = 4f * invDx * invDx;
        int nnx = _nnx, nny = _nny;
        int zr = d == 3 ? 1 : 0;
        var be = PBe!; var cm = PCm!;
        for (int i = from; i < to; i++)
        {
            int f = PFluid[i];
            if (!_isMpm[f]) { continue; }
            float ux = (_x[i] - _gMinX) * invDx, uy = (_y[i] - _gMinY) * invDx, uz = d == 3 ? (_z[i] - _gMinZ) * invDx : 0f;
            int cx = (int)Math.Floor(ux), cy = (int)Math.Floor(uy), cz = d == 3 ? (int)Math.Floor(uz) : 0;
            float fx = ux - cx - 0.5f, fy = uy - cy - 0.5f, fz = d == 3 ? uz - cz - 0.5f : 0f;
            float vx = 0f, vy = 0f, vz = 0f;
            float c00 = 0f, c01 = 0f, c02 = 0f, c10 = 0f, c11 = 0f, c12 = 0f, c20 = 0f, c21 = 0f, c22 = 0f;
            for (int oz = -zr; oz <= zr; oz++)
            {
                for (int oy = -1; oy <= 1; oy++)
                {
                    for (int ox = -1; ox <= 1; ox++)
                    {
                        float dX = ox - fx, dY = oy - fy, dZ = d == 3 ? oz - fz : 0f;
                        float w = Spline(dX) * Spline(dY) * (d == 3 ? Spline(dZ) : 1f);
                        if (w == 0f) { continue; }
                        int n = (cx + ox + 1) + nnx * ((cy + oy + 1) + nny * (d == 3 ? cz + oz + 1 : 0));
                        float gx = _gvx[n] * w, gy = _gvy[n] * w, gz = _gvz[n] * w;
                        vx += gx; vy += gy; vz += gz;
                        float px = dX * dx, py = dY * dx, pz = dZ * dx;
                        c00 += gx * px; c01 += gx * py; c02 += gx * pz;
                        c10 += gy * px; c11 += gy * py; c12 += gy * pz;
                        c20 += gz * px; c21 += gz * py; c22 += gz * pz;
                    }
                }
            }

            c00 *= four; c01 *= four; c02 *= four; c10 *= four; c11 *= four; c12 *= four; c20 *= four; c21 *= four; c22 *= four;
            float m = _maxSpeed[f];
            float sq = vx * vx + vy * vy + vz * vz;
            if (sq > m * m) { float k = m / float.Sqrt(sq); vx *= k; vy *= k; vz *= k; }
            PVx[i] = vx; PVy[i] = vy; PVz[i] = vz;
            _x[i] += vx * h; _y[i] += vy * h; if (d == 3) { _z[i] += vz * h; }
            int c0 = i * 9;
            cm[c0] = c00; cm[c0 + 1] = c01; cm[c0 + 2] = c02; cm[c0 + 3] = c10; cm[c0 + 4] = c11; cm[c0 + 5] = c12; cm[c0 + 6] = c20; cm[c0 + 7] = c21; cm[c0 + 8] = c22;

            // b' = F b F^T with F = I + h C (the unit z row of a planar world stays).
            float f00 = 1f + h * c00, f01 = h * c01, f02 = h * c02, f10 = h * c10, f11 = 1f + h * c11, f12 = h * c12, f20 = h * c20, f21 = h * c21, f22 = 1f + h * c22;
            int b0 = i * 6;
            float bxx = be[b0], byy = be[b0 + 1], bzz = be[b0 + 2], bxy = be[b0 + 3], bxz = be[b0 + 4], byz = be[b0 + 5];
            // G = F b
            float g00 = f00 * bxx + f01 * bxy + f02 * bxz, g01 = f00 * bxy + f01 * byy + f02 * byz, g02 = f00 * bxz + f01 * byz + f02 * bzz;
            float g10 = f10 * bxx + f11 * bxy + f12 * bxz, g11 = f10 * bxy + f11 * byy + f12 * byz, g12 = f10 * bxz + f11 * byz + f12 * bzz;
            float g20 = f20 * bxx + f21 * bxy + f22 * bxz, g21 = f20 * bxy + f21 * byy + f22 * byz, g22 = f20 * bxz + f21 * byz + f22 * bzz;
            be[b0] = g00 * f00 + g01 * f01 + g02 * f02; be[b0 + 1] = g10 * f10 + g11 * f11 + g12 * f12; be[b0 + 2] = g20 * f20 + g21 * f21 + g22 * f22;
            be[b0 + 3] = g00 * f10 + g01 * f11 + g02 * f12; be[b0 + 4] = g00 * f20 + g01 * f21 + g02 * f22; be[b0 + 5] = g10 * f20 + g11 * f21 + g12 * f22;
        }

        _ = dx;
    }
}
