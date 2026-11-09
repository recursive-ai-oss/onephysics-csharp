using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Water;

internal sealed partial class WaterGrid
{
    private float[] _lost = Array.Empty<float>();
    private float _dtStage, _g;

    /// <summary>
    /// Advances the water by <paramref name="dt"/> seconds under gravity <paramref name="g"/>: sources first, then as many substeps as the wave speed needs (never more than <see cref="MaxSubsteps"/>; the last one takes whatever time is left).
    /// The substep length comes from the state alone, so every thread count and platform takes the same ones. Returns the number of substeps.
    /// </summary>
    public int Advance(float dt, float g, TaskExecutor executor)
    {
        ApplySources(dt);
        if (!(g > 0f) || !(dt > 0f) || Nx == 0) { return 0; }
        int blocks = Blocks;
        if (_lost.Length != blocks) { _lost = new float[blocks]; }
        _g = g;
        int n = 0;
        float t = 0f;
        bool last = false;
        while (!last)
        {
            Array.Copy(H, _h0, H.Length); Array.Copy(Hu, _u0, Hu.Length); Array.Copy(Hv, _v0, Hv.Length);
            Reconstruct(executor, blocks);
            float ax = 0f, az = 0f;
            for (int b = 0; b < blocks; b++) { if (_amaxX[b] > ax) { ax = _amaxX[b]; } if (_amaxZ[b] > az) { az = _amaxZ[b]; } }
            float rate = ax / Dx + (Planar ? 0f : az / Dx);
            float ds = rate > 0f ? Cfl / rate : dt;
            float rem = dt - t;
            n++;
            if (ds >= rem || n >= MaxSubsteps) { ds = rem; last = true; }
            t += ds;
            _dtStage = ds;
            // Stage one (forward Euler), then stage two from its result, then the average with the start (Heun's method, the strong-stability-preserving two-stage Runge-Kutta).
            executor.For(blocks, new UpdateTask(this));
            Reconstruct(executor, blocks);
            executor.For(blocks, new UpdateTask(this));
            executor.For(blocks, new CombineTask(this));
            double lost = 0;
            for (int b = 0; b < blocks; b++) { lost += _lost[b]; _lost[b] = 0f; }
            LostVolume += lost * CellArea;
        }

        return n;
    }

    private void Reconstruct(TaskExecutor executor, int blocks)
    {
        executor.For(blocks, new SlopeTask(this));
        executor.For(blocks, new FaceTask(this));
    }

    private readonly struct SlopeTask(WaterGrid grid) : IParallelBody { public void Run(int task) => grid.Slopes(task); }
    private readonly struct FaceTask(WaterGrid grid) : IParallelBody { public void Run(int task) => grid.Faces(task); }
    private readonly struct UpdateTask(WaterGrid grid) : IParallelBody { public void Run(int task) => grid.Update(task); }
    private readonly struct CombineTask(WaterGrid grid) : IParallelBody { public void Run(int task) => grid.Combine(task); }

    // ---------------------------------------------------------------- slopes

    // Limited slopes of the surface and of the momenta, as differences across a cell. Next to a dry cell the scheme is first order (slope zero), which keeps shores robust.
    private void Slopes(int block)
    {
        int k0 = block * RowsPerTask, k1 = Math.Min(k0 + RowsPerTask, Nz);
        for (int k = k0; k < k1; k++)
        {
            for (int i = 0; i < Nx; i++)
            {
                int c = k * Nx + i;
                float h = H[c];
                if (i == 0 || i == Nx - 1 || !(h > DryDepth) || !(H[c - 1] > DryDepth) || !(H[c + 1] > DryDepth))
                {
                    _sxw[c] = 0f; _sxn[c] = 0f; _sxt[c] = 0f;
                }
                else
                {
                    _sxw[c] = Slope(H[c - 1] + Bed[c - 1], h + Bed[c], H[c + 1] + Bed[c + 1]);
                    _sxn[c] = Slope(Hu[c - 1], Hu[c], Hu[c + 1]);
                    _sxt[c] = Slope(Hv[c - 1], Hv[c], Hv[c + 1]);
                }

                if (Planar || k == 0 || k == Nz - 1 || !(h > DryDepth) || !(H[c - Nx] > DryDepth) || !(H[c + Nx] > DryDepth))
                {
                    _szw[c] = 0f; _szn[c] = 0f; _szt[c] = 0f;
                }
                else
                {
                    _szw[c] = Slope(H[c - Nx] + Bed[c - Nx], h + Bed[c], H[c + Nx] + Bed[c + Nx]);
                    _szn[c] = Slope(Hv[c - Nx], Hv[c], Hv[c + Nx]);
                    _szt[c] = Slope(Hu[c - Nx], Hu[c], Hu[c + Nx]);
                }
            }
        }
    }

    // ---------------------------------------------------------------- face fluxes

    private void Faces(int block)
    {
        int k0 = block * RowsPerTask;
        float ax = 0f, az = 0f;
        int nw = Nx + 1;
        int kx1 = Math.Min(k0 + RowsPerTask, Nz);
        for (int k = k0; k < kx1; k++)
        {
            for (int i = 0; i <= Nx; i++)
            {
                int f = k * nw + i;
                float a = Face(i > 0 ? k * Nx + i - 1 : -1, i < Nx ? k * Nx + i : -1, true, 0, 1, out _fxh[f], out _fxl[f], out _fxu[f], out _fxr[f]);
                if (a > ax) { ax = a; }
            }
        }

        if (!Planar)
        {
            int kz1 = Math.Min(k0 + RowsPerTask, Nz + 1);
            for (int k = k0; k < kz1; k++)
            {
                for (int i = 0; i < Nx; i++)
                {
                    int f = k * Nx + i;
                    float a = Face(k > 0 ? (k - 1) * Nx + i : -1, k < Nz ? k * Nx + i : -1, false, 2, 3, out _fzh[f], out _fzl[f], out _fzu[f], out _fzr[f]);
                    if (a > az) { az = a; }
                }
            }
        }

        _amaxX[block] = ax; _amaxZ[block] = az;
    }

    /// <summary>
    /// The state a cell presents at one of its faces: its own depth there, the surface there, and the velocities normal to and along the face.
    /// A dry cell has no water at its faces and its surface is its bed.
    /// </summary>
    private void Side(int c, bool east, bool xDirection, out float hOwn, out float w, out float un, out float ut)
    {
        float h = H[c];
        if (!(h > DryDepth)) { hOwn = 0f; w = Bed[c]; un = 0f; ut = 0f; return; }
        float sgn = east ? 0.5f : -0.5f;
        float sw, qn, qt;
        if (xDirection) { sw = _sxw[c]; qn = Hu[c] + sgn * _sxn[c]; qt = Hv[c] + sgn * _sxt[c]; }
        else { sw = _szw[c]; qn = Hv[c] + sgn * _szn[c]; qt = Hu[c] + sgn * _szt[c]; }
        w = h + Bed[c] + sgn * sw;
        hOwn = w - Bed[c];
        if (hOwn < 0f) { hOwn = 0f; }
        un = Vel(hOwn, qn); ut = Vel(hOwn, qt);
    }

    /// <summary>
    /// The flux across the face between a cell on the low side (index -1 at the grid's edge) and one on the high side. The two states are made hydrostatic over the higher of the two beds
    /// (Audusse et al. 2004) and exchanged by the central-upwind flux; each cell then takes the flux plus the difference between the pressure of its own depth at the face and of the reconstructed depth,
    /// which is what makes a lake at rest exactly at rest. Returns the fastest signal seen.
    /// </summary>
    private float Face(int cl, int cr, bool xDirection, int lowSide, int highSide, out float fh, out float fqLow, out float fqHigh, out float fr)
    {
        float g = _g;
        float hl = 0f, ul = 0f, vl = 0f, hr = 0f, ur = 0f, vr = 0f, hOwnL = 0f, hOwnR = 0f;
        float a;
        if (cl >= 0 && cr >= 0)
        {
            Side(cl, true, xDirection, out hOwnL, out float wl, out ul, out vl);
            Side(cr, false, xDirection, out hOwnR, out float wr, out ur, out vr);
            float bf = Math.Max(Bed[cl], Bed[cr]);
            hl = wl - bf; if (hl < 0f) { hl = 0f; }
            hr = wr - bf; if (hr < 0f) { hr = 0f; }
            CentralUpwind(hl, ul, vl, hr, ur, vr, g, out fh, out float fn, out fr, out a);
            fqLow = (fn - Pressure(hl, g)) + Pressure(hOwnL, g);
            fqHigh = (fn - Pressure(hr, g)) + Pressure(hOwnR, g);
            return a;
        }

        bool lowEdge = cl < 0;
        int inside = lowEdge ? cr : cl;
        Side(inside, !lowEdge, xDirection, out float hIn, out _, out float uIn, out float vIn);
        var kind = Edges[lowEdge ? lowSide : highSide];
        if (kind == WaterEdge.Wall)
        {
            fh = 0f; fr = 0f;
            fqLow = fqHigh = Pressure(hIn, g);
            return Math.Abs(uIn) + float.Sqrt(g * hIn);
        }

        float hOut, uOut, vOut;
        if (kind == WaterEdge.Open) { hOut = hIn; uOut = uIn; vOut = vIn; }
        else
        {
            float d = EdgeLevel[lowEdge ? lowSide : highSide] - Bed[inside];
            hOut = d > 0f ? d : 0f;
            float speed = EdgeSpeed[lowEdge ? lowSide : highSide];
            uOut = lowEdge ? speed : -speed; vOut = 0f;
        }

        float fq1, fr1;
        if (lowEdge) { CentralUpwind(hOut, uOut, vOut, hIn, uIn, vIn, g, out fh, out fq1, out fr1, out a); }
        else { CentralUpwind(hIn, uIn, vIn, hOut, uOut, vOut, g, out fh, out fq1, out fr1, out a); }
        fqLow = fqHigh = fq1; fr = fr1;
        return a;
    }

    // ---------------------------------------------------------------- update

    private void Update(int block)
    {
        int k0 = block * RowsPerTask, k1 = Math.Min(k0 + RowsPerTask, Nz);
        float dt = _dtStage;
        float rdx = 1f / Dx;
        int nw = Nx + 1;
        float lost = 0f;
        for (int k = k0; k < k1; k++)
        {
            for (int i = 0; i < Nx; i++)
            {
                int c = k * Nx + i;
                int fw = k * nw + i, fe = fw + 1;
                float dh = -(_fxh[fe] - _fxh[fw]) * rdx;
                float dq = -(_fxl[fe] - _fxu[fw]) * rdx;
                float dr = -(_fxr[fe] - _fxr[fw]) * rdx;
                if (!Planar)
                {
                    int fs = k * Nx + i, fn = fs + Nx;
                    dh -= (_fzh[fn] - _fzh[fs]) * rdx;
                    dq -= (_fzr[fn] - _fzr[fs]) * rdx;
                    dr -= (_fzl[fn] - _fzu[fs]) * rdx;
                }

                float nh = H[c] + dt * dh;
                if (nh < 0f) { lost -= nh; nh = 0f; }
                H[c] = nh;
                Hu[c] = Hu[c] + dt * dq;
                Hv[c] = Hv[c] + dt * dr;
            }
        }

        _lost[block] += lost;
    }

    private void Combine(int block)
    {
        int k0 = block * RowsPerTask, k1 = Math.Min(k0 + RowsPerTask, Nz);
        float dt = _dtStage;
        float cf = BottomFriction;
        for (int k = k0; k < k1; k++)
        {
            for (int i = 0; i < Nx; i++)
            {
                int c = k * Nx + i;
                float h = 0.5f * (_h0[c] + H[c]);
                float q = 0.5f * (_u0[c] + Hu[c]), r = 0.5f * (_v0[c] + Hv[c]);
                if (!(h > DryDepth)) { q = 0f; r = 0f; }
                else if (cf > 0f)
                {
                    // Quadratic bottom drag, implicit in the momentum so it can only slow the water down.
                    float u = q / h, v = r / h;
                    float f = 1f / (1f + dt * cf * float.Sqrt(u * u + v * v) / h);
                    q *= f; r *= f;
                }

                if (Planar) { r = 0f; }
                H[c] = h; Hu[c] = q; Hv[c] = r;
            }
        }
    }

    // ---------------------------------------------------------------- sources

    private void ApplySources(float dt)
    {
        for (int s = 0; s < Sources.Length; s++)
        {
            var src = Sources[s];
            if (src.Rate == 0f) { continue; }
            float inv = 1f / Dx;
            float cx = src.X * inv, cz = Planar ? 0.5f : src.Z * inv, rad = src.Radius * inv;
            int i0 = (int)Math.Floor(cx - rad), i1 = (int)Math.Floor(cx + rad), k0 = (int)Math.Floor(cz - rad), k1 = (int)Math.Floor(cz + rad);
            if (Planar) { k0 = 0; k1 = 0; }
            i0 = Math.Max(i0, 0); i1 = Math.Min(i1, Nx - 1); k0 = Math.Max(k0, 0); k1 = Math.Min(k1, Nz - 1);
            if (i0 > i1 || k0 > k1) { continue; }
            int count = (i1 - i0 + 1) * (k1 - k0 + 1);
            float dv = src.Rate * dt / (count * CellArea);
            for (int k = k0; k <= k1; k++) { for (int i = i0; i <= i1; i++) { Apply(k * Nx + i, dv); } }
        }
    }
}
