using System;
using System.Collections.Generic;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Fluids;

/// <summary>A contact of one particle with a shape or a deformable's particle during a substep.</summary>
internal struct FluidContact
{
    public int P;              // particle (index in the substep's sorted order)
    public int Body;           // index in the step's body table, or -1
    public int Def;            // index in the step's deformables, or -1
    public bool Coupled;       // the other side takes the reaction
    public float Nx, Ny, Nz;   // normal, towards the particle
    public float C0;           // separation of the surfaces at detection (negative when overlapping)
    public float X, Y, Z;      // particle centre at detection, local coordinates
    public float Mu;
    public float Applied;      // sum of the contact multipliers applied so far in the substep
    public float Rx, Ry, Rz;   // contact point relative to the body's centre of mass
    public float Sx, Sy, Sz;   // velocity of the other surface at the contact
}

internal sealed class FluidTaskBuf
{
    public FluidContact[] C = new FluidContact[64];
    public int Count;
    public float MaxSq, MinX, MinY, MinZ, MaxX, MaxY, MaxZ;
    public double D0, D1;
}

internal sealed partial class FluidStore
{
    private const int Chunk = 256;
    private const int MaxDenseCells = 1 << 22;
    private const int HashBuckets = 1 << 20;

    // ---- working storage for a step, sized to the particle capacity
    private float[] _x = new float[64], _y = new float[64], _z = new float[64];
    private float[] _x0 = new float[64], _y0 = new float[64], _z0 = new float[64];
    private float[] _lam = new float[64], _rho = new float[64];
    private float[] _dx = new float[64], _dy = new float[64], _dz = new float[64];
    private float[] _nvx = new float[64], _nvy = new float[64], _nvz = new float[64];
    private float[] _omx = new float[64], _omy = new float[64], _omz = new float[64];
    private int[] _sId = new int[64];
    private byte[] _sFluid = new byte[64];
    private int[] _order = new int[64], _cellOf = new int[64];
    private int[] _cellStart = new int[16];
    private int[] _nbr = new int[64];
    private int[] _nbrCount = new int[64];
    private int[] _cFirst = new int[64];
    private byte[] _cCount = new byte[64];
    private int _stride;

    // grid of the current substep
    private float _gMinX, _gMinY, _gMinZ, _invH;
    private int _gnx, _gny, _gnz;
    private bool _hashed;
    private int _cells;

    private FluidKernels _k;
    private float _h;                 // substep length
    private float _invDt;
    private double _ox, _oy, _oz;     // local origin of the step
    private float[] _mass = new float[MaxFluids], _n0 = new float[MaxFluids], _invMass = new float[MaxFluids];
    private float[] _visc = new float[MaxFluids], _vort = new float[MaxFluids], _tens = new float[MaxFluids], _fric = new float[MaxFluids];
    private float[] _maxSpeed = new float[MaxFluids], _gscale = new float[MaxFluids];
    private bool _anyVisc, _anyVort;
    private Vector3Ref _g;

    private struct Vector3Ref { public float X, Y, Z; }

    private FluidTaskBuf[] _tb = Array.Empty<FluidTaskBuf>();
    private int _taskCount;

    // Each array is checked on its own: the persistent and scratch arrays are swapped after every sort and
    // grow separately (particles are appended between sorts), so their lengths can differ.
    private static void Fit<T>(ref T[] a, int n) { if (a.Length < n) { Array.Resize(ref a, Math.Max(n, a.Length * 2)); } }

    private void EnsureWork(int n)
    {
        Fit(ref _x, n); Fit(ref _y, n); Fit(ref _z, n);
        Fit(ref _x0, n); Fit(ref _y0, n); Fit(ref _z0, n);
        Fit(ref _lam, n); Fit(ref _rho, n);
        Fit(ref _dx, n); Fit(ref _dy, n); Fit(ref _dz, n);
        Fit(ref _nvx, n); Fit(ref _nvy, n); Fit(ref _nvz, n);
        Fit(ref _omx, n); Fit(ref _omy, n); Fit(ref _omz, n);
        Fit(ref _sId, n); Fit(ref _sFluid, n);
        Fit(ref _order, n); Fit(ref _cellOf, n);
        Fit(ref _nbrCount, n); Fit(ref _cFirst, n); Fit(ref _cCount, n);
    }

    // ------------------------------------------------------------------ grid

    private struct BoundsTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task)
        {
            var s = S; var b = s._tb[task];
            int from = task * Chunk, to = Math.Min(s.Count, from + Chunk);
            float mnx = float.MaxValue, mny = float.MaxValue, mnz = float.MaxValue, mxx = float.MinValue, mxy = float.MinValue, mxz = float.MinValue;
            for (int i = from; i < to; i++)
            {
                float x = s._x[i], y = s._y[i], z = s._z[i];
                if (x < mnx) { mnx = x; } if (x > mxx) { mxx = x; }
                if (y < mny) { mny = y; } if (y > mxy) { mxy = y; }
                if (z < mnz) { mnz = z; } if (z > mxz) { mxz = z; }
            }

            b.MinX = mnx; b.MinY = mny; b.MinZ = mnz; b.MaxX = mxx; b.MaxY = mxy; b.MaxZ = mxz;
        }
    }

    private struct CellTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task)
        {
            var s = S;
            int from = task * Chunk, to = Math.Min(s.Count, from + Chunk);
            for (int i = from; i < to; i++) { s._cellOf[i] = s.CellOfParticle(i); }
        }
    }

    private struct GatherTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task)
        {
            var s = S;
            int from = task * Chunk, to = Math.Min(s.Count, from + Chunk);
            var o = s._order;
            // The permuted arrays are written into the scratch arrays and swapped in afterwards (see Permute).
            for (int i = from; i < to; i++)
            {
                int j = o[i];
                s._dx[i] = s._x[j]; s._dy[i] = s._y[j]; s._dz[i] = s._z[j];
                s._nvx[i] = s._x0[j]; s._nvy[i] = s._y0[j]; s._nvz[i] = s._z0[j];
                s._omx[i] = s.PVx[j]; s._omy[i] = s.PVy[j]; s._omz[i] = s.PVz[j];
                s._sId[i] = s.PId[j]; s._sFluid[i] = s.PFluid[j];
            }

            if (s.PBe != null)
            {
                for (int i = from; i < to; i++)
                {
                    int j = o[i];
                    for (int k = 0; k < 6; k++) { s._sBe[i * 6 + k] = s.PBe[j * 6 + k]; }
                    for (int k = 0; k < 9; k++) { s._sCm[i * 9 + k] = s.PCm![j * 9 + k]; }
                    s._sAux[i] = s.PAux![j];
                }
            }
        }
    }

    private int CellCoord(float v, float min, int n)
    {
        long c = (long)((v - min) * _invH);
        if (c < 0) { c = 0; }
        if (!_hashed && c >= n) { c = n - 1; }
        return (int)Math.Min(c, int.MaxValue);
    }

    private int CellOfParticle(int i)
    {
        int ix = CellCoord(_x[i], _gMinX, _gnx), iy = CellCoord(_y[i], _gMinY, _gny), iz = CellCoord(_z[i], _gMinZ, _gnz);
        return _hashed ? Bucket(ix, iy, iz) : ix + _gnx * (iy + _gny * iz);
    }

    private static int Bucket(int ix, int iy, int iz)
    {
        unchecked
        {
            uint h = (uint)ix * 73856093u ^ (uint)iy * 19349663u ^ (uint)iz * 83492791u;
            h ^= h >> 15; h *= 2246822519u; h ^= h >> 13;
            return (int)(h & (HashBuckets - 1));
        }
    }

    /// <summary>Sorts the particles by grid cell (a stable counting sort: ties keep their earlier order) and builds the cell table. The arrays are permuted so neighbouring cells are neighbours in memory.</summary>
    private void SortParticles()
    {
        int n = Count;
        _taskCount = (n + Chunk - 1) / Chunk;
        if (_tb.Length < _taskCount) { int old = _tb.Length; Array.Resize(ref _tb, Math.Max(_taskCount, old * 2)); for (int t = old; t < _tb.Length; t++) { _tb[t] = new FluidTaskBuf(); } }
        Core().Executor.For(_taskCount, new BoundsTask { S = this });
        float mnx = float.MaxValue, mny = float.MaxValue, mnz = float.MaxValue, mxx = float.MinValue, mxy = float.MinValue, mxz = float.MinValue;
        for (int t = 0; t < _taskCount; t++)
        {
            var b = _tb[t];
            if (b.MinX < mnx) { mnx = b.MinX; } if (b.MaxX > mxx) { mxx = b.MaxX; }
            if (b.MinY < mny) { mny = b.MinY; } if (b.MaxY > mxy) { mxy = b.MaxY; }
            if (b.MinZ < mnz) { mnz = b.MinZ; } if (b.MaxZ > mxz) { mxz = b.MaxZ; }
        }

        _gMinX = mnx; _gMinY = mny; _gMinZ = mnz;
        _invH = 1f / _k.H;
        double sx = (double)(mxx - mnx) * _invH, sy = (double)(mxy - mny) * _invH, sz = (double)(mxz - mnz) * _invH;
        double cells = (Math.Floor(sx) + 1.0) * (Math.Floor(sy) + 1.0) * (Math.Floor(sz) + 1.0);
        _hashed = !(cells >= 1.0 && cells <= MaxDenseCells);
        if (_hashed && _anyMpm) { throw new FluidBudgetException("The solid matter spans " + cells.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + " grid cells; the material point grid is dense and holds at most " + MaxDenseCells.ToString(System.Globalization.CultureInfo.InvariantCulture) + " (bring the particles closer together or use a larger KernelRadiusScale)."); }
        if (_hashed) { _gnx = _gny = _gnz = int.MaxValue; _cells = HashBuckets; }
        else { _gnx = (int)Math.Floor(sx) + 1; _gny = (int)Math.Floor(sy) + 1; _gnz = (int)Math.Floor(sz) + 1; _cells = _gnx * _gny * _gnz; }
        if (_cellStart.Length < _cells + 1) { _cellStart = new int[Math.Max(_cells + 1, _cellStart.Length * 2)]; }
        Core().Executor.For(_taskCount, new CellTask { S = this });
        var cs = _cellStart;
        Array.Clear(cs, 0, _cells + 1);
        for (int i = 0; i < n; i++) { cs[_cellOf[i] + 1]++; }
        for (int c = 0; c < _cells; c++) { cs[c + 1] += cs[c]; }
        // Scatter: positions in cs[] advance, so build the order with a copy of the starts held in the cell table's own slack: use _nbrCount as a cursor store is wrong (indexed by particle); use a second pass with a temporary cursor.
        EnsureCursor(_cells);
        Array.Copy(cs, _cursor, _cells);
        for (int i = 0; i < n; i++) { _order[_cursor[_cellOf[i]]++] = i; }
        if (PBe != null) { Fit(ref _sBe, n * 6); Fit(ref _sCm, n * 9); Fit(ref _sAux, n); }
        Core().Executor.For(_taskCount, new GatherTask { S = this });
        // Swap the permuted copies in.
        if (PBe != null) { (PBe, _sBe) = (_sBe, PBe); (PCm, _sCm) = (_sCm, PCm); (PAux, _sAux) = (_sAux, PAux); }
        (_x, _dx) = (_dx, _x); (_y, _dy) = (_dy, _y); (_z, _dz) = (_dz, _z);
        (_x0, _nvx) = (_nvx, _x0); (_y0, _nvy) = (_nvy, _y0); (_z0, _nvz) = (_nvz, _z0);
        (PVx, _omx) = (_omx, PVx); (PVy, _omy) = (_omy, PVy); (PVz, _omz) = (_omz, PVz);
        (PId, _sId) = (_sId, PId); (PFluid, _sFluid) = (_sFluid, PFluid);
    }

    private float[] _sBe = new float[6], _sCm = new float[9], _sAux = new float[1];

    private int[] _cursor = new int[16];

    private void EnsureCursor(int n) { if (_cursor.Length < n) { _cursor = new int[Math.Max(n, _cursor.Length * 2)]; } }

    // ------------------------------------------------------------------ neighbours

    private struct NeighbourTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task)
        {
            var s = S;
            int from = task * Chunk, to = Math.Min(s.Count, from + Chunk);
            s.FindNeighbours(from, to);
        }
    }

    private void FindNeighbours(int from, int to)
    {
        float h2 = _k.H2;
        int stride = _stride;
        bool d3 = Dim == 3;
        var cs = _cellStart;
        Span<int> seen = stackalloc int[27];
        bool anyMpm = _anyMpm;
        for (int i = from; i < to; i++)
        {
            // Solid matter has no neighbour list (it exchanges momentum through the grid) and the density constraint of the fluids does not see it.
            if (anyMpm && _isMpm[PFluid[i]]) { _nbrCount[i] = 0; continue; }
            float xi = _x[i], yi = _y[i], zi = _z[i];
            int ix = CellCoord(xi, _gMinX, _gnx), iy = CellCoord(yi, _gMinY, _gny), iz = CellCoord(zi, _gMinZ, _gnz);
            int count = 0;
            int bas = i * stride;
            if (!_hashed)
            {
                int x0 = Math.Max(ix - 1, 0), x1 = Math.Min(ix + 1, _gnx - 1);
                int y0 = Math.Max(iy - 1, 0), y1 = Math.Min(iy + 1, _gny - 1);
                int z0 = d3 ? Math.Max(iz - 1, 0) : 0, z1 = d3 ? Math.Min(iz + 1, _gnz - 1) : 0;
                for (int cz = z0; cz <= z1; cz++)
                {
                    for (int cy = y0; cy <= y1; cy++)
                    {
                        int rowBase = _gnx * (cy + _gny * cz);
                        int a = cs[rowBase + x0], b = cs[rowBase + x1 + 1];
                        for (int j = a; j < b; j++)
                        {
                            if (j == i) { continue; }
                            float dx = xi - _x[j], dy = yi - _y[j], dz = zi - _z[j];
                            if (dx * dx + dy * dy + dz * dz < h2 && count < stride && !(anyMpm && _isMpm[PFluid[j]])) { _nbr[bas + count++] = j; }
                        }
                    }
                }
            }
            else
            {
                int ns = 0;
                for (int cz = d3 ? -1 : 0; cz <= (d3 ? 1 : 0); cz++)
                {
                    for (int cy = -1; cy <= 1; cy++)
                    {
                        for (int cx = -1; cx <= 1; cx++)
                        {
                            long jx = (long)ix + cx, jy = (long)iy + cy, jz = (long)iz + cz;
                            if (jx < 0 || jy < 0 || jz < 0) { continue; }
                            int bk = Bucket((int)jx, (int)jy, (int)jz);
                            bool dup = false;
                            for (int q = 0; q < ns; q++) { if (seen[q] == bk) { dup = true; break; } }
                            if (dup) { continue; }
                            seen[ns++] = bk;
                            int a = cs[bk], b = cs[bk + 1];
                            for (int j = a; j < b; j++)
                            {
                                if (j == i) { continue; }
                                float dx = xi - _x[j], dy = yi - _y[j], dz = zi - _z[j];
                                if (dx * dx + dy * dy + dz * dz < h2 && count < stride && !(anyMpm && _isMpm[PFluid[j]])) { _nbr[bas + count++] = j; }
                            }
                        }
                    }
                }
            }

            _nbrCount[i] = count;
        }
    }

    private WorldCore Core() => Host.Core;
}
