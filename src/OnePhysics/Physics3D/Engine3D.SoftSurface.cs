using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>An edge of a soft body broke at the end of a step (see <see cref="SoftBodyDef.TearStrain"/> and <see cref="SoftBodyDef.TearForce"/>).</summary>
[Experimental("OPHYS0002")]
public readonly struct SoftTearEvent
{
    internal SoftTearEvent(SoftBody body, int a, int b, int particleCount)
    {
        Body = body; ParticleA = a; ParticleB = b; ParticleCount = particleCount;
    }

    /// <summary>The body that tore. It stays one body: pieces that come apart are separate parts of one particle set.</summary>
    public SoftBody Body { get; }

    /// <summary>First particle of the broken edge (its number before any split of the step; the smaller of the two).</summary>
    public int ParticleA { get; }

    /// <summary>Second particle of the broken edge (the larger).</summary>
    public int ParticleB { get; }

    /// <summary>The body's particle count after the step's tears. Particles the tear split off are numbered from the old count upwards.</summary>
    public int ParticleCount { get; }
}

internal sealed partial class Engine3D
{

    // Surface contacts (Bridson, Fedkiw, Anderson 2002, robust treatment of collisions for cloth; Harmon et al. 2008 for the pair types; Macklin et al. 2014 for contacts as position constraints): a particle against
    // a surface triangle, and an edge against an edge.
    //
    // Candidates are kept between substeps and steps like a Verlet neighbour list (Verlet 1967; Ericson 2005, ch. 7 on coherence). A rebuild lists every pair of features whose distance is below the sum of the
    // two spheres' radii plus both bodies' budgets; while no surface particle has moved from its listed position by more than its body's budget (less what the next substep can add), no unlisted pair can have come
    // into contact, so the list stays valid. Each substep the listed pairs that can touch within it become the active set, with the side of the feature each started the substep on; only that set is solved.
    // Every list is a function of the positions and the bodies alone, in a fixed order, so a rebuild is the same on any number of threads.

    private SoftCellGrid[] _tgTri = new SoftCellGrid[4], _tgEdge = new SoftCellGrid[4];
    private double[] _tgIcT = new double[4], _tgMxT = new double[4], _tgIcE = new double[4], _tgMxE = new double[4];
    private bool[] _tgNeedTri = new bool[4], _tgNeedEdge = new bool[4];

    // Candidates (valid while _cbValid): global particle number and global triangle number as numbered by the build, the owners' indices in the awake list, the distance when listed.
    private int[] _vtP = new int[4096], _vtT = new int[4096], _vtBA = new int[4096], _vtBT = new int[4096];
    private float[] _vtD0 = new float[4096];
    private int _vtCount;
    private int[] _eeA = new int[4096], _eeB = new int[4096], _eeBA = new int[4096], _eeBB = new int[4096];
    private float[] _eeD0 = new float[4096];
    private int _eeCount;

    // The active set of the substep: candidate numbers and the side each pair started on.
    private int[] _avK = new int[2048], _aeK = new int[2048];
    private float[] _avS = new float[2048], _aeS = new float[2048];
    private int _avCount, _aeCount;
    private long[] _scKey = new long[4096];
    private int[] _scPos = new int[4096], _scTmpI = new int[4096];
    private float[] _scTmpF = new float[4096];

    private SoftBodyData[] _gtBody = new SoftBodyData[64];
    private int[] _gtLocal = new int[64];
    private int[] _gtV = new int[192], _geV = new int[128];
    private double[] _gtBox = new double[384];
    private SoftBodyData[] _geBody = new SoftBodyData[64];
    private int[] _geLocal = new int[64];
    private double[] _geBox = new double[384];
    private readonly List<SoftTearEvent> _softTears = new();

    // What the candidate lists were built for.
    private SoftBodyData?[] _cbBody = new SoftBodyData?[4];
    private int[]?[] _cbTris = new int[]?[4];
    private int[] _cbN = new int[4];
    private long[] _cbKey1 = new long[4], _cbKey2 = new long[4];
    private int _cbCount;
    private ulong _cbLayers;
    private bool _cbValid;
    private double[]?[] _bpos = new double[]?[4];
    private double[] _bBudget = new double[4], _bTr = new double[4];
    // Motion of each body since the lists were built, relative to its own mean motion: mean displacement and its largest deviation from it, mean velocity and its largest deviation.
    private double[] _bMd = new double[12], _bDev = new double[4];
    private float[] _bMv = new float[12], _bVdev = new float[4];
    // With more bodies than this, motion is bounded per body in absolute terms instead of per pair of bodies in relative terms.
    private const int RelativeBodyLimit = 64;
    private double _gScaleBase;
    private bool[] _bPart = new bool[4];
    private double[] _bExtT = new double[4], _bExtE = new double[4];

    private SurfBuf[] _sbuf = new SurfBuf[8];
    private int[] _skB = new int[16], _skKind = new int[16], _skA = new int[16], _skFrom = new int[16], _skTo = new int[16];
    private int _skCount;

    /// <summary>The tears of the last step, in body and edge order.</summary>
    public ReadOnlySpan<SoftTearEvent> SoftTearEvents => System.Runtime.InteropServices.CollectionsMarshal.AsSpan(_softTears);

    private const float InsideTolerance = 0.05f;
    // A feature can travel in a substep at most this many times the substep's travel at its fastest particle's speed at the start (the solver may add some speed; the margin covers it).
    private const double TravelFactor = 1.5;
    private const double MaxCellVisits = 512.0, MaxCellCoordinate = 1e15;
    // How far a particle may wander from its listed position before the lists are rebuilt, in particle radii (on top of the substep's travel).
    // Speed only: any value gives the same result (a test checks it), a larger one rebuilds less often and lists more pairs.
    internal double SkinFactor = 1.0;
    // Items per search task.
    private const int SurfaceChunk = 96;

    private sealed class SurfBuf
    {
        public int[] A = new int[512], B = new int[512], C = new int[512], D = new int[512];
        public float[] D0 = new float[512];
        public int Count;

        public void Add(int a, int b, int c, int d, float d0)
        {
            if (Count == A.Length)
            {
                int n = Count * 2;
                Array.Resize(ref A, n); Array.Resize(ref B, n); Array.Resize(ref C, n); Array.Resize(ref D, n); Array.Resize(ref D0, n);
            }

            A[Count] = a; B[Count] = b; C[Count] = c; D[Count] = d; D0[Count] = d0; Count++;
        }
    }

    private readonly struct SurfGridProgram : IParallelBody
    {
        private readonly Engine3D _e;
        public SurfGridProgram(Engine3D e) { _e = e; }
        public void Run(int task) => _e.BuildTargetGrids(task);
    }

    private readonly struct SurfSearchProgram : IParallelBody
    {
        private readonly Engine3D _e;
        public SurfSearchProgram(Engine3D e) { _e = e; }
        public void Run(int task) => _e.RunSurfaceTask(task);
    }

    // Brings the candidate lists up to date for the substep that starts now (positions are the substep's start positions) and selects the pairs that can touch in it.
    private void PrepareSurfacePairs(float h, float gmag)
    {
        _avCount = 0; _aeCount = 0;
        int parts = 0;
        bool self = false;
        for (int a = 0; a < _pairSoftCount; a++)
        {
            var d = _awakeSoft[a];
            if (!d.Def.SurfaceCollision || d.SurfTris.Length == 0) { continue; }
            parts++;
            self |= d.Def.SelfCollision;
        }

        if (parts == 0 || (parts < 2 && !self)) { _cbValid = false; _vtCount = 0; _eeCount = 0; return; }
        GrowTo(ref _bTr, _pairSoftCount); GrowTo(ref _bBudget, _pairSoftCount); GrowTo(ref _bPart, _pairSoftCount); GrowTo(ref _bpos, _pairSoftCount);
        for (int a = 0; a < _pairSoftCount; a++)
        {
            var d = _awakeSoft[a];
            _bPart[a] = d.Def.SurfaceCollision && d.SurfTris.Length != 0;
            _bTr[a] = a < _awakeSoftCount && _bPart[a] ? TravelFactor * (MaxParticleSpeed(d) * (double)h + 0.5 * gmag * (double)h * h) : 0.0;
        }

        GrowTo(ref _bMd, 3 * _pairSoftCount); GrowTo(ref _bDev, _pairSoftCount); GrowTo(ref _bMv, 3 * _pairSoftCount); GrowTo(ref _bVdev, _pairSoftCount);
        bool rebuild = !_cbValid || !SameBodies();
        if (!rebuild)
        {
            for (int a = 0; a < _pairSoftCount; a++) { ComputeMotion(a, true, parts <= RelativeBodyLimit); }
            rebuild = !MotionWithinBudget(h, gmag, parts <= RelativeBodyLimit);
        }

        if (rebuild)
        {
            RebuildCandidates();
            for (int a = 0; a < _pairSoftCount; a++) { ComputeMotion(a, false, parts <= RelativeBodyLimit); }
        }

        _selH = h; _gScaleBase = gmag;
        SelectActive();
    }

    private ulong LayerSignature()
    {
        ulong s = 14695981039346656037UL;
        for (int i = 0; i < 32; i++) { s = (s ^ LayerMatrix[i]) * 1099511628211UL; }
        return s;
    }

    private static long Key1(SoftBodyDef d) => ((long)d.CategoryBits << 32) | d.MaskBits;

    private static long Key2(SoftBodyDef d) => ((long)d.GroupIndex << 8) | ((long)d.Layer << 3) | (d.SelfCollision ? 1L : 0L) | (d.SurfaceCollision ? 2L : 0L);

    private bool SameBodies()
    {
        if (_cbCount != _pairSoftCount || _cbLayers != LayerSignature()) { return false; }
        for (int a = 0; a < _pairSoftCount; a++)
        {
            var d = _awakeSoft[a];
            if (!ReferenceEquals(_cbBody[a], d) || _cbN[a] != d.N || !ReferenceEquals(_cbTris[a], d.SurfTris) || _cbKey1[a] != Key1(d.Def) || _cbKey2[a] != Key2(d.Def)) { return false; }
            if (_bPart[a] && _bpos[a] == null) { return false; }
        }

        return true;
    }

    // The motion of a body's surface particles since they were listed (when withDisp) and the spread of their velocities. Two features of one body cannot close on each other faster than the spread of their motion, however fast
    // the body as a whole travels: a falling cloth keeps its lists. With relative false the mean is taken as zero, so the bounds are absolute (many bodies).
    private void ComputeMotion(int a, bool withDisp, bool relative)
    {
        int k3 = 3 * a;
        _bMd[k3] = _bMd[k3 + 1] = _bMd[k3 + 2] = 0.0; _bMv[k3] = _bMv[k3 + 1] = _bMv[k3 + 2] = 0f; _bDev[a] = 0.0; _bVdev[a] = 0f;
        if (!_bPart[a] || a >= _awakeSoftCount) { return; }
        var d = _awakeSoft[a];
        var sp = d.SurfParticles; var x = d.X; var vel = d.Vel;
        int n = sp.Length;
        if (n == 0) { return; }
        double mx = 0.0, my = 0.0, mz = 0.0;
        float vx = 0f, vy = 0f, vz = 0f;
        if (relative)
        {
            double sx = 0.0, sy = 0.0, sz = 0.0, tx = 0.0, ty = 0.0, tz = 0.0;
            var bp = _bpos[a];
            for (int i = 0; i < n; i++)
            {
                int k = 3 * sp[i];
                if (withDisp) { sx += d.Ox + x[k] - bp![3 * i]; sy += d.Oy + x[k + 1] - bp[3 * i + 1]; sz += d.Oz + x[k + 2] - bp[3 * i + 2]; }
                tx += vel[k]; ty += vel[k + 1]; tz += vel[k + 2];
            }

            double inv = 1.0 / n;
            mx = sx * inv; my = sy * inv; mz = sz * inv;
            vx = (float)(tx * inv); vy = (float)(ty * inv); vz = (float)(tz * inv);
        }

        double dev2 = 0.0; float vdev2 = 0f;
        {
            var bp = _bpos[a];
            for (int i = 0; i < n; i++)
            {
                int k = 3 * sp[i];
                if (withDisp)
                {
                    double dx = d.Ox + x[k] - bp![3 * i] - mx, dy = d.Oy + x[k + 1] - bp[3 * i + 1] - my, dz = d.Oz + x[k + 2] - bp[3 * i + 2] - mz;
                    double e2 = dx * dx + dy * dy + dz * dz;
                    if (e2 > dev2) { dev2 = e2; }
                }

                float ux = vel[k] - vx, uy = vel[k + 1] - vy, uz = vel[k + 2] - vz;
                float u2 = ux * ux + uy * uy + uz * uz;
                if (u2 > vdev2) { vdev2 = u2; }
            }
        }

        _bMd[k3] = mx; _bMd[k3 + 1] = my; _bMd[k3 + 2] = mz;
        _bMv[k3] = vx; _bMv[k3 + 1] = vy; _bMv[k3 + 2] = vz;
        _bDev[a] = Math.Sqrt(dev2); _bVdev[a] = float.Sqrt(vdev2);
    }

    // How far two bodies' features can have moved towards each other since the lists were built.
    private double RelDisp(int a, int b)
    {
        if (a == b) { return 2.0 * _bDev[a]; }
        double dx = _bMd[3 * a] - _bMd[3 * b], dy = _bMd[3 * a + 1] - _bMd[3 * b + 1], dz = _bMd[3 * a + 2] - _bMd[3 * b + 2];
        return Math.Sqrt(dx * dx + dy * dy + dz * dz) + _bDev[a] + _bDev[b];
    }

    // How far they can close on each other in a substep of length h: the difference of their mean velocities plus the spread of each, with the difference of the fall under gravity.
    private double RelTrav(int a, int b, double h)
    {
        double dv;
        if (a == b) { dv = 0.0; }
        else
        {
            double dx = (double)_bMv[3 * a] - _bMv[3 * b], dy = (double)_bMv[3 * a + 1] - _bMv[3 * b + 1], dz = (double)_bMv[3 * a + 2] - _bMv[3 * b + 2];
            dv = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        double gs = a == b ? 0.0 : Math.Abs((double)_awakeSoft[a].Def.GravityScale - _awakeSoft[b].Def.GravityScale) * _gScaleBase;
        return TravelFactor * (h * (dv + _bVdev[a] + _bVdev[b]) + 0.5 * gs * h * h);
    }

    // No unlisted pair can have come into contact while this holds for every pair of bodies (or, with many bodies, for every body against its own budget).
    private bool MotionWithinBudget(double h, float gmag, bool relative)
    {
        _gScaleBase = gmag;
        int n = _pairSoftCount;
        for (int a = 0; a < n; a++)
        {
            if (!_bPart[a]) { continue; }
            if (!(RelDisp(a, a) + RelTrav(a, a, h) < 2.0 * _bBudget[a])) { return false; }
            if (!relative) { continue; }
            for (int b = a + 1; b < n; b++)
            {
                if (!_bPart[b]) { continue; }
                if (!(RelDisp(a, b) + RelTrav(a, b, h) < _bBudget[a] + _bBudget[b])) { return false; }
            }
        }

        return true;
    }

    private void RebuildCandidates()
    {
        _vtCount = 0; _eeCount = 0;
        GrowTo(ref _cbBody, _pairSoftCount); GrowTo(ref _cbTris, _pairSoftCount); GrowTo(ref _cbN, _pairSoftCount); GrowTo(ref _cbKey1, _pairSoftCount); GrowTo(ref _cbKey2, _pairSoftCount);
        _cbCount = _pairSoftCount; _cbLayers = LayerSignature(); _cbValid = true;
        for (int a = 0; a < _pairSoftCount; a++)
        {
            var d = _awakeSoft[a];
            d.SurfTriBase = -1;
            _cbBody[a] = d; _cbTris[a] = d.SurfTris; _cbN[a] = d.N; _cbKey1[a] = Key1(d.Def); _cbKey2[a] = Key2(d.Def);
            if (!_bPart[a]) { _bpos[a] = null; _bBudget[a] = 0.0; continue; }
            // The budget: how far the body's particles may wander from the listed positions, and what the substep adds on top.
            _bBudget[a] = SkinFactor * d.Radius + _bTr[a];
            var sp = d.SurfParticles;
            var bp = _bpos[a];
            if (bp == null || bp.Length < 3 * sp.Length) { bp = _bpos[a] = new double[3 * sp.Length]; }
            for (int i = 0; i < sp.Length; i++)
            {
                int k = 3 * sp[i];
                bp[3 * i] = d.Ox + d.X[k]; bp[3 * i + 1] = d.Oy + d.X[k + 1]; bp[3 * i + 2] = d.Oz + d.X[k + 2];
            }
        }

        MarkSurfaceBodies();
        int nt = 0, ne = 0;
        for (int a = 0; a < _pairSoftCount; a++)
        {
            var d = _awakeSoft[a];
            if (!d.SurfaceActive) { continue; }
            d.SurfTriBase = nt; d.SurfEdgeBase = ne;
            nt += d.SurfTris.Length / 3; ne += d.SurfEdgeA.Length;
        }

        if (nt >= SoftCellGrid.MaxItems || ne >= SoftCellGrid.MaxItems) { _cbValid = false; return; }
        GrowTo(ref _gtV, 3 * nt); GrowTo(ref _geV, 2 * ne); GrowTo(ref _gtBody, nt); GrowTo(ref _gtLocal, nt); GrowTo(ref _gtBox, 6 * nt);
        GrowTo(ref _geBody, ne); GrowTo(ref _geLocal, ne); GrowTo(ref _geBox, 6 * ne);
        GrowTo(ref _tgTri, _pairSoftCount); GrowTo(ref _tgEdge, _pairSoftCount); GrowTo(ref _tgIcT, _pairSoftCount); GrowTo(ref _tgMxT, _pairSoftCount); GrowTo(ref _tgIcE, _pairSoftCount); GrowTo(ref _tgMxE, _pairSoftCount);
        GrowTo(ref _tgNeedTri, _pairSoftCount); GrowTo(ref _tgNeedEdge, _pairSoftCount);
        // A feature's box is grown by its body's radius and budget: two features can come within the sum of their radii while both stay inside their budgets only if their grown boxes overlap.
        GrowTo(ref _bExtT, _pairSoftCount); GrowTo(ref _bExtE, _pairSoftCount);
        var extT = _bExtT; var extE = _bExtE;
        for (int a = 0; a < _pairSoftCount; a++)
        {
            var d = _awakeSoft[a];
            if (d.SurfTriBase < 0) { continue; }
            double grow = d.Radius + _bBudget[a], eT = 0.0, eE = 0.0;
            for (int i = 0; i < d.SurfTris.Length / 3; i++)
            {
                int g = d.SurfTriBase + i, o = 6 * g;
                double ext = BoxOf(d, d.SurfTris[3 * i], d.SurfTris[3 * i + 1], d.SurfTris[3 * i + 2], grow, _gtBox, o);
                _gtBody[g] = d; _gtLocal[g] = i;
                _gtV[3 * g] = d.SurfTris[3 * i]; _gtV[3 * g + 1] = d.SurfTris[3 * i + 1]; _gtV[3 * g + 2] = d.SurfTris[3 * i + 2];
                if (ext > eT) { eT = ext; }
            }

            for (int i = 0; i < d.SurfEdgeA.Length; i++)
            {
                int g = d.SurfEdgeBase + i, o = 6 * g;
                double ext = BoxOf(d, d.SurfEdgeA[i], d.SurfEdgeB[i], d.SurfEdgeB[i], grow, _geBox, o);
                _geBody[g] = d; _geLocal[g] = i;
                _geV[2 * g] = d.SurfEdgeA[i]; _geV[2 * g + 1] = d.SurfEdgeB[i];
                if (ext > eE) { eE = ext; }
            }

            extT[a] = eT; extE[a] = eE;
        }

        // Each body with surface neighbours is searched as a target in its own grid, whose cells fit its own features (a global grid would take the cell size of the biggest triangle of the biggest body). The others bring
        // only what lies inside the target's bounds: their particles against its triangles, and, for the lower-numbered ones, their edges against its edges, so each body pair is looked at once. A body with
        // self-collision also searches itself.
        int targets = 0;
        for (int b = 0; b < _pairSoftCount; b++)
        {
            var db = _awakeSoft[b];
            _tgNeedTri[b] = false; _tgNeedEdge[b] = false;
            if (db.SurfTriBase < 0) { continue; }
            bool needTri = db.Def.SelfCollision, needEdge = db.Def.SelfCollision;
            for (int a = 0; a < _pairSoftCount; a++)
            {
                if (a == b || !BodiesMeet(a, b)) { continue; }
                if (a < _awakeSoftCount) { needTri = true; }
                if (a < b) { needEdge = true; }
            }

            if (!needTri && !needEdge) { continue; }
            _tgNeedTri[b] = needTri; _tgNeedEdge[b] = needEdge;
            double cst = 2.0 * Math.Max(extT[b], 1e-3), cse = 2.0 * Math.Max(extE[b], 1e-3);
            // Cells are twice the largest box, so a box that meets a query box has its minimum corner in at most 2 cells per axis of the range searched.
            _tgIcT[b] = 1.0 / cst; _tgMxT[b] = cst * 0.5; _tgIcE[b] = 1.0 / cse; _tgMxE[b] = cse * 0.5;
            _tgTri[b] ??= new SoftCellGrid(); _tgEdge[b] ??= new SoftCellGrid();
            targets++;
        }

        if (targets == 0) { return; }
        // The grids of the targets are built in tasks (one per target), then the searches run in tasks of a fixed number of items each, and the hits are joined in task order.
        Core.Executor.For(_pairSoftCount, new SurfGridProgram(this));
        BuildSearchTasks();
        for (int t = 0; t < _skCount; t++) { _sbuf[t] ??= new SurfBuf(); _sbuf[t]!.Count = 0; }
        Core.Executor.For(_skCount, new SurfSearchProgram(this));
        for (int t = 0; t < _skCount; t++)
        {
            var buf = _sbuf[t]!;
            if (_skKind[t] == 0)
            {
                GrowTo(ref _vtP, _vtCount + buf.Count); GrowTo(ref _vtT, _vtCount + buf.Count); GrowTo(ref _vtBA, _vtCount + buf.Count); GrowTo(ref _vtBT, _vtCount + buf.Count); GrowTo(ref _vtD0, _vtCount + buf.Count);
                for (int i = 0; i < buf.Count; i++)
                {
                    _vtP[_vtCount] = buf.A[i]; _vtT[_vtCount] = buf.B[i]; _vtBA[_vtCount] = buf.C[i]; _vtBT[_vtCount] = buf.D[i]; _vtD0[_vtCount] = buf.D0[i]; _vtCount++;
                }
            }
            else
            {
                GrowTo(ref _eeA, _eeCount + buf.Count); GrowTo(ref _eeB, _eeCount + buf.Count); GrowTo(ref _eeBA, _eeCount + buf.Count); GrowTo(ref _eeBB, _eeCount + buf.Count); GrowTo(ref _eeD0, _eeCount + buf.Count);
                for (int i = 0; i < buf.Count; i++)
                {
                    _eeA[_eeCount] = buf.A[i]; _eeB[_eeCount] = buf.B[i]; _eeBA[_eeCount] = buf.C[i]; _eeBB[_eeCount] = buf.D[i]; _eeD0[_eeCount] = buf.D0[i]; _eeCount++;
                }
            }
        }

        SortCandidates();
    }

    // The candidates are kept in the order of their feature numbers, so the active pairs a substep selects from them come out in that order whatever order the search happened to find them in
    // (which depends on when the lists were last built).
    private void SortCandidates()
    {
        int n = Math.Max(_vtCount, _eeCount);
        GrowTo(ref _scKey, n); GrowTo(ref _scPos, n); GrowTo(ref _scTmpI, n); GrowTo(ref _scTmpF, n);
        for (int i = 0; i < _vtCount; i++) { _scKey[i] = ((long)_vtP[i] << 32) | (long)_vtT[i]; _scPos[i] = i; }
        Array.Sort(_scKey, _scPos, 0, _vtCount);
        Permute(_vtP, _vtCount); Permute(_vtT, _vtCount); Permute(_vtBA, _vtCount); Permute(_vtBT, _vtCount); PermuteF(_vtD0, _vtCount);
        for (int i = 0; i < _eeCount; i++) { _scKey[i] = ((long)_eeA[i] << 32) | (long)_eeB[i]; _scPos[i] = i; }
        Array.Sort(_scKey, _scPos, 0, _eeCount);
        Permute(_eeA, _eeCount); Permute(_eeB, _eeCount); Permute(_eeBA, _eeCount); Permute(_eeBB, _eeCount); PermuteF(_eeD0, _eeCount);
    }

    private void Permute(int[] a, int n)
    {
        for (int i = 0; i < n; i++) { _scTmpI[i] = a[_scPos[i]]; }
        Array.Copy(_scTmpI, a, n);
    }

    private void PermuteF(float[] a, int n)
    {
        for (int i = 0; i < n; i++) { _scTmpF[i] = a[_scPos[i]]; }
        Array.Copy(_scTmpF, a, n);
    }

    private void AddSkTask(int b, int kind, int a, int from, int to)
    {
        GrowTo(ref _skB, _skCount + 1); GrowTo(ref _skKind, _skCount + 1); GrowTo(ref _skA, _skCount + 1); GrowTo(ref _skFrom, _skCount + 1); GrowTo(ref _skTo, _skCount + 1);
        _skB[_skCount] = b; _skKind[_skCount] = kind; _skA[_skCount] = a; _skFrom[_skCount] = from; _skTo[_skCount] = to; _skCount++;
        if (_sbuf.Length < _skCount) { Array.Resize(ref _sbuf, 2 * _sbuf.Length); }
    }

    // The search tasks in the order the hits must be joined: per target body, the particles of each source body against its triangles (source bodies ascending), then the edges of each source body up to the target
    // against its edges. Each source is cut into chunks of SurfaceChunk items.
    private void BuildSearchTasks()
    {
        _skCount = 0;
        for (int b = 0; b < _pairSoftCount; b++)
        {
            var db = _awakeSoft[b];
            if (db.SurfTriBase < 0) { continue; }
            if (_tgNeedTri[b])
            {
                for (int a = 0; a < _awakeSoftCount; a++)
                {
                    var dp = _awakeSoft[a];
                    if (dp.SurfTriBase < 0) { continue; }
                    if (a == b) { if (!db.Def.SelfCollision) { continue; } }
                    else if (!BodiesMeet(a, b)) { continue; }
                    int n = dp.SurfParticles.Length;
                    for (int f = 0; f < n; f += SurfaceChunk) { AddSkTask(b, 0, a, f, Math.Min(n, f + SurfaceChunk)); }
                }
            }

            if (_tgNeedEdge[b])
            {
                for (int a = 0; a <= b; a++)
                {
                    var da = _awakeSoft[a];
                    if (da.SurfTriBase < 0) { continue; }
                    if (a == b) { if (!db.Def.SelfCollision) { continue; } }
                    else if (!BodiesMeet(a, b)) { continue; }
                    int n = da.SurfEdgeA.Length;
                    for (int f = 0; f < n; f += SurfaceChunk) { AddSkTask(b, 1, a, f, Math.Min(n, f + SurfaceChunk)); }
                }
            }
        }
    }

    private void BuildTargetGrids(int b)
    {
        if (b >= _pairSoftCount) { return; }
        var db = _awakeSoft[b];
        if (_tgNeedTri[b])
        {
            var grid = _tgTri[b]!;
            int nTb = db.SurfTris.Length / 3;
            double icT = _tgIcT[b];
            grid.Grow(nTb);
            var tk = grid.Keys;
            for (int i = 0; i < nTb; i++)
            {
                int o = 6 * (db.SurfTriBase + i);
                tk[i] = (SoftCellGrid.Hash((long)Math.Floor(_gtBox[o] * icT), (long)Math.Floor(_gtBox[o + 1] * icT), (long)Math.Floor(_gtBox[o + 2] * icT)) << 24) | (long)i;
            }

            grid.Build(nTb);
        }

        if (_tgNeedEdge[b])
        {
            var grid = _tgEdge[b]!;
            int nEb = db.SurfEdgeA.Length;
            double icE = _tgIcE[b];
            grid.Grow(nEb);
            var ek = grid.Keys;
            for (int i = 0; i < nEb; i++)
            {
                int o = 6 * (db.SurfEdgeBase + i);
                ek[i] = (SoftCellGrid.Hash((long)Math.Floor(_geBox[o] * icE), (long)Math.Floor(_geBox[o + 1] * icE), (long)Math.Floor(_geBox[o + 2] * icE)) << 24) | (long)i;
            }

            grid.Build(nEb);
        }
    }

    private void RunSurfaceTask(int t)
    {
        var buf = _sbuf[t]!;
        if (_skKind[t] == 0) { SearchTriangles(_skB[t], _skA[t], _skFrom[t], _skTo[t], buf); }
        else { SearchEdges(_skB[t], _skA[t], _skFrom[t], _skTo[t], buf); }
    }

    private static bool SurfaceBodiesMeet(SoftBodyData da, SoftBodyData db)
    {
        var u = da.SurfBounds; var v = db.SurfBounds;
        return !(u[0] > v[3] || u[3] < v[0] || u[1] > v[4] || u[4] < v[1] || u[2] > v[5] || u[5] < v[2]);
    }

    private bool BodiesMeet(int a, int b)
    {
        var da = _awakeSoft[a]; var db = _awakeSoft[b];
        if (da.SurfTriBase < 0 || db.SurfTriBase < 0 || (a >= _awakeSoftCount && b >= _awakeSoftCount)) { return false; }
        return SurfaceBodiesMeet(da, db);
    }

    // Particle against triangle: the particle, grown like a feature of its body, looks at every cell that can hold the minimum corner of a box that meets its own.
    private void SearchTriangles(int b, int a, int from, int to, SurfBuf buf)
    {
        Span<long> seen = stackalloc long[64];
        var db = _awakeSoft[b];
        var dp = _awakeSoft[a];
        var bb = db.SurfBounds;
        var grid = _tgTri[b]!;
        var tk = grid.Keys;
        int nTb = db.SurfTris.Length / 3;
        double icT = _tgIcT[b], mxT = _tgMxT[b];
        double ep = dp.Radius + _bBudget[a];
        for (int pi = from; pi < to; pi++)
        {
            int p = dp.SurfParticles[pi];
            double px = dp.Ox + dp.X[3 * p], py = dp.Oy + dp.X[3 * p + 1], pz = dp.Oz + dp.X[3 * p + 2];
            if (a != b && (px + ep < bb[0] || px - ep > bb[3] || py + ep < bb[1] || py - ep > bb[4] || pz + ep < bb[2] || pz - ep > bb[5])) { continue; }
            // A particle that can travel very far, or sits very far out (a diverged body), would visit an enormous range of cells (or overflow the cell numbers); outside a small, sane range every triangle of the target is tested instead.
            double fx0 = (px - ep - mxT) * icT, fx1 = (px + ep) * icT, fy0 = (py - ep - mxT) * icT, fy1 = (py + ep) * icT, fz0 = (pz - ep - mxT) * icT, fz1 = (pz + ep) * icT;
            bool wide = !(Math.Abs(fx0) < MaxCellCoordinate && Math.Abs(fx1) < MaxCellCoordinate && Math.Abs(fy0) < MaxCellCoordinate && Math.Abs(fy1) < MaxCellCoordinate && Math.Abs(fz0) < MaxCellCoordinate && Math.Abs(fz1) < MaxCellCoordinate
                && (fx1 - fx0 + 2.0) * (fy1 - fy0 + 2.0) * (fz1 - fz0 + 2.0) <= MaxCellVisits);
            long x0 = wide ? 0 : (long)Math.Floor(fx0), x1 = wide ? 0 : (long)Math.Floor(fx1);
            long y0 = wide ? 0 : (long)Math.Floor(fy0), y1 = wide ? 0 : (long)Math.Floor(fy1);
            long z0 = wide ? 0 : (long)Math.Floor(fz0), z1 = wide ? 0 : (long)Math.Floor(fz1);
            int nseen = 0;
            if (wide)
            {
                for (int i = 0; i < nTb; i++)
                {
                    int g = db.SurfTriBase + i, o = 6 * g;
                    if (a == b && (p == _gtV[3 * g] || p == _gtV[3 * g + 1] || p == _gtV[3 * g + 2])) { continue; }
                    if (px + ep < _gtBox[o] || px - ep > _gtBox[o + 3] || py + ep < _gtBox[o + 1] || py - ep > _gtBox[o + 4] || pz + ep < _gtBox[o + 2] || pz - ep > _gtBox[o + 5]) { continue; }
                    TryAddVertexTriangle(buf, a, dp, p, px, py, pz, b, db, g);
                }

                continue;
            }

            for (long cz = z0; cz <= z1; cz++)
            {
                for (long cy = y0; cy <= y1; cy++)
                {
                    for (long cx = x0; cx <= x1; cx++)
                    {
                        long hsh = SoftCellGrid.Hash(cx, cy, cz);
                        bool dup = false;
                        for (int u = 0; u < nseen; u++) { if (seen[u] == hsh) { dup = true; break; } }
                        if (dup) { continue; }
                        if (nseen < seen.Length) { seen[nseen++] = hsh; }
                        if (!grid.Find(hsh, out int s0, out int s1)) { continue; }
                        for (int k = s0; k < s1; k++)
                        {
                            int g = db.SurfTriBase + (int)(tk[k] & 0xFFFFFF);
                            int o = 6 * g;
                            if (a == b && (p == _gtV[3 * g] || p == _gtV[3 * g + 1] || p == _gtV[3 * g + 2])) { continue; }
                            if (px + ep < _gtBox[o] || px - ep > _gtBox[o + 3] || py + ep < _gtBox[o + 1] || py - ep > _gtBox[o + 4] || pz + ep < _gtBox[o + 2] || pz - ep > _gtBox[o + 5]) { continue; }
                            TryAddVertexTriangle(buf, a, dp, p, px, py, pz, b, db, g);
                        }
                    }
                }
            }
        }
    }

    // Edge against edge, each pair once: an edge of a lower-numbered body (or of the target itself, against the higher-numbered edges) looks at every cell that can hold the minimum corner of a box that meets its own.
    private void SearchEdges(int b, int a, int from, int to, SurfBuf buf)
    {
        Span<long> seen = stackalloc long[64];
        var db = _awakeSoft[b];
        var da = _awakeSoft[a];
        var bb = db.SurfBounds;
        var grid = _tgEdge[b]!;
        var ek = grid.Keys;
        int nEb = db.SurfEdgeA.Length;
        double icE = _tgIcE[b], mxE = _tgMxE[b];
        for (int ie = from; ie < to; ie++)
        {
            int g = da.SurfEdgeBase + ie, o = 6 * g;
            if (a != b && (_geBox[o] > bb[3] || _geBox[o + 3] < bb[0] || _geBox[o + 1] > bb[4] || _geBox[o + 4] < bb[1] || _geBox[o + 2] > bb[5] || _geBox[o + 5] < bb[2])) { continue; }
            double fx0 = (_geBox[o] - mxE) * icE, fx1 = _geBox[o + 3] * icE, fy0 = (_geBox[o + 1] - mxE) * icE, fy1 = _geBox[o + 4] * icE, fz0 = (_geBox[o + 2] - mxE) * icE, fz1 = _geBox[o + 5] * icE;
            bool wide = !(Math.Abs(fx0) < MaxCellCoordinate && Math.Abs(fx1) < MaxCellCoordinate && Math.Abs(fy0) < MaxCellCoordinate && Math.Abs(fy1) < MaxCellCoordinate && Math.Abs(fz0) < MaxCellCoordinate && Math.Abs(fz1) < MaxCellCoordinate
                && (fx1 - fx0 + 2.0) * (fy1 - fy0 + 2.0) * (fz1 - fz0 + 2.0) <= MaxCellVisits);
            long x0 = wide ? 0 : (long)Math.Floor(fx0), x1 = wide ? 0 : (long)Math.Floor(fx1);
            long y0 = wide ? 0 : (long)Math.Floor(fy0), y1 = wide ? 0 : (long)Math.Floor(fy1);
            long z0 = wide ? 0 : (long)Math.Floor(fz0), z1 = wide ? 0 : (long)Math.Floor(fz1);
            int nseen = 0;
            if (wide)
            {
                for (int i = 0; i < nEb; i++)
                {
                    int h2 = db.SurfEdgeBase + i;
                    if (a == b)
                    {
                        if (h2 <= g) { continue; }
                        if (_geV[2 * g] == _geV[2 * h2] || _geV[2 * g] == _geV[2 * h2 + 1] || _geV[2 * g + 1] == _geV[2 * h2] || _geV[2 * g + 1] == _geV[2 * h2 + 1]) { continue; }
                    }

                    int o2 = 6 * h2;
                    if (_geBox[o2] > _geBox[o + 3] || _geBox[o2 + 3] < _geBox[o] || _geBox[o2 + 1] > _geBox[o + 4] || _geBox[o2 + 4] < _geBox[o + 1] || _geBox[o2 + 2] > _geBox[o + 5] || _geBox[o2 + 5] < _geBox[o + 2]) { continue; }
                    TryAddEdgeEdge(buf, a, b, g, h2);
                }

                continue;
            }

            for (long cz = z0; cz <= z1; cz++)
            {
                for (long cy = y0; cy <= y1; cy++)
                {
                    for (long cx = x0; cx <= x1; cx++)
                    {
                        long hsh = SoftCellGrid.Hash(cx, cy, cz);
                        bool dup = false;
                        for (int u = 0; u < nseen; u++) { if (seen[u] == hsh) { dup = true; break; } }
                        if (dup) { continue; }
                        if (nseen < seen.Length) { seen[nseen++] = hsh; }
                        if (!grid.Find(hsh, out int s0, out int s1)) { continue; }
                        for (int k = s0; k < s1; k++)
                        {
                            int h2 = db.SurfEdgeBase + (int)(ek[k] & 0xFFFFFF);
                            if (a == b)
                            {
                                if (h2 <= g) { continue; }
                                if (_geV[2 * g] == _geV[2 * h2] || _geV[2 * g] == _geV[2 * h2 + 1] || _geV[2 * g + 1] == _geV[2 * h2] || _geV[2 * g + 1] == _geV[2 * h2 + 1]) { continue; }
                            }

                            int o2 = 6 * h2;
                            if (_geBox[o2] > _geBox[o + 3] || _geBox[o2 + 3] < _geBox[o] || _geBox[o2 + 1] > _geBox[o + 4] || _geBox[o2 + 4] < _geBox[o + 1] || _geBox[o2 + 2] > _geBox[o + 5] || _geBox[o2 + 5] < _geBox[o + 2]) { continue; }
                            TryAddEdgeEdge(buf, a, b, g, h2);
                        }
                    }
                }
            }
        }
    }

    // A body takes part in the search when it has self-collision, or when its grown bounds meet another surface body's: an isolated body has nothing to be listed against.
    private void MarkSurfaceBodies()
    {
        for (int a = 0; a < _pairSoftCount; a++)
        {
            var d = _awakeSoft[a];
            d.SurfaceActive = false;
            if (!_bPart[a]) { continue; }
            double x0 = double.MaxValue, y0 = double.MaxValue, z0 = double.MaxValue, x1 = double.MinValue, y1 = double.MinValue, z1 = double.MinValue;
            for (int i = 0; i < d.SurfParticles.Length; i++)
            {
                int k = 3 * d.SurfParticles[i];
                double x = d.Ox + d.X[k], y = d.Oy + d.X[k + 1], z = d.Oz + d.X[k + 2];
                if (x < x0) { x0 = x; } if (x > x1) { x1 = x; }
                if (y < y0) { y0 = y; } if (y > y1) { y1 = y; }
                if (z < z0) { z0 = z; } if (z > z1) { z1 = z; }
            }

            double g = d.Radius + _bBudget[a];
            d.SurfBounds[0] = x0 - g; d.SurfBounds[1] = y0 - g; d.SurfBounds[2] = z0 - g; d.SurfBounds[3] = x1 + g; d.SurfBounds[4] = y1 + g; d.SurfBounds[5] = z1 + g;
        }

        for (int a = 0; a < _pairSoftCount; a++)
        {
            var d = _awakeSoft[a];
            if (!_bPart[a]) { continue; }
            if (d.Def.SelfCollision) { d.SurfaceActive = true; continue; }
            for (int b = 0; b < _pairSoftCount && !d.SurfaceActive; b++)
            {
                if (b == a || (a >= _awakeSoftCount && b >= _awakeSoftCount)) { continue; }
                if (!_bPart[b]) { continue; }
                if (!SurfaceBodiesMeet(d, _awakeSoft[b])) { continue; }
                d.SurfaceActive = true;
            }
        }
    }

    private static float MaxParticleSpeed(SoftBodyData d)
    {
        float m2 = 0f;
        var v = d.Vel;
        for (int i = 0; i < d.N; i++)
        {
            float v2 = v[3 * i] * v[3 * i] + v[3 * i + 1] * v[3 * i + 1] + v[3 * i + 2] * v[3 * i + 2];
            if (v2 > m2) { m2 = v2; }
        }

        return float.Sqrt(m2);
    }

    // The box of a triangle (or an edge, given its end twice) at the start of the listing in world coordinates, grown by `grow`; returns its largest extent.
    private static double BoxOf(SoftBodyData d, int a, int b, int c, double grow, double[] box, int o)
    {
        double x0 = double.MaxValue, y0 = double.MaxValue, z0 = double.MaxValue, x1 = double.MinValue, y1 = double.MinValue, z1 = double.MinValue;
        for (int k = 0; k < 3; k++)
        {
            int p = 3 * (k == 0 ? a : (k == 1 ? b : c));
            double x = d.Ox + d.X[p], y = d.Oy + d.X[p + 1], z = d.Oz + d.X[p + 2];
            if (x < x0) { x0 = x; } if (x > x1) { x1 = x; }
            if (y < y0) { y0 = y; } if (y > y1) { y1 = y; }
            if (z < z0) { z0 = z; } if (z > z1) { z1 = z; }
        }

        box[o] = x0 - grow; box[o + 1] = y0 - grow; box[o + 2] = z0 - grow; box[o + 3] = x1 + grow; box[o + 4] = y1 + grow; box[o + 5] = z1 + grow;
        return Math.Max(x1 - x0, Math.Max(y1 - y0, z1 - z0)) + 2.0 * grow;
    }

    private void TryAddVertexTriangle(SurfBuf buf, int ba, SoftBodyData dp, int p, double px, double py, double pz, int bt, SoftBodyData dt, int g)
    {
        int el = _gtLocal[g];
        int a = dt.SurfTris[3 * el], b = dt.SurfTris[3 * el + 1], c = dt.SurfTris[3 * el + 2];
        float thick = dp.Radius + dt.Radius;
        if (dp == dt)
        {
            if (!dp.Def.SelfCollision || p == a || p == b || p == c) { return; }
            float near2 = 4.41f * thick * thick;
            if (dp.RestDistance2(p, a) < near2 || dp.RestDistance2(p, b) < near2 || dp.RestDistance2(p, c) < near2) { return; }
        }
        else if (!SoftPairAllowed(dp.Def, dt.Def)) { return; }

        // The exact distance when listed against what the two bodies can close before the next rebuild.
        double reach = thick + _bBudget[ba] + _bBudget[bt];
        double ax = dt.Ox + dt.X[3 * a], ay = dt.Oy + dt.X[3 * a + 1], az = dt.Oz + dt.X[3 * a + 2];
        double bx = dt.Ox + dt.X[3 * b], by = dt.Oy + dt.X[3 * b + 1], bz = dt.Oz + dt.X[3 * b + 2];
        double cx = dt.Ox + dt.X[3 * c], cy = dt.Oy + dt.X[3 * c + 1], cz = dt.Oz + dt.X[3 * c + 2];
        double d2 = SoftGeometry.PointTriangleSq(px, py, pz, ax, ay, az, bx, by, bz, cx, cy, cz);
        if (d2 >= reach * reach) { return; }
        buf.Add(dp.Base + p, g, ba, bt, (float)Math.Sqrt(d2));
    }

    private void TryAddEdgeEdge(SurfBuf buf, int ba, int bb, int g1, int g2)
    {
        var d1 = _geBody[g1]; var d2 = _geBody[g2];
        int l1 = _geLocal[g1], l2 = _geLocal[g2];
        int a = d1.SurfEdgeA[l1], b = d1.SurfEdgeB[l1], c = d2.SurfEdgeA[l2], d = d2.SurfEdgeB[l2];
        float thick = d1.Radius + d2.Radius;
        if (d1 == d2)
        {
            if (!d1.Def.SelfCollision || a == c || a == d || b == c || b == d) { return; }
            float near2 = 4.41f * thick * thick;
            if (d1.RestDistance2(a, c) < near2 || d1.RestDistance2(a, d) < near2 || d1.RestDistance2(b, c) < near2 || d1.RestDistance2(b, d) < near2) { return; }
        }
        else if (!SoftPairAllowed(d1.Def, d2.Def)) { return; }

        double reach = thick + _bBudget[ba] + _bBudget[bb];
        double ax = d1.Ox + d1.X[3 * a], ay = d1.Oy + d1.X[3 * a + 1], az = d1.Oz + d1.X[3 * a + 2];
        double bx = d1.Ox + d1.X[3 * b], by = d1.Oy + d1.X[3 * b + 1], bz = d1.Oz + d1.X[3 * b + 2];
        double cx = d2.Ox + d2.X[3 * c], cy = d2.Oy + d2.X[3 * c + 1], cz = d2.Oz + d2.X[3 * c + 2];
        double dx = d2.Ox + d2.X[3 * d], dy = d2.Oy + d2.X[3 * d + 1], dz = d2.Oz + d2.X[3 * d + 2];
        double q2 = SoftGeometry.SegmentSegmentSq(ax, ay, az, bx, by, bz, cx, cy, cz, dx, dy, dz);
        if (q2 >= reach * reach) { return; }
        buf.Add(g1, g2, ba, bb, (float)Math.Sqrt(q2));
    }

    // The pairs that can touch in this substep: the exact distance at its start is below the two spheres' radii plus what the two bodies can move in it. A pair stores the side it starts on (0 when the feature is
    // not inside the triangle or the two edges); the first pass that finds it inside sets it from where it is then.
    private double _selH;

    // Candidates per selection task. Each task writes its own list of the pairs it keeps (in candidate order) and the lists are joined in task order, so the active set is the candidate order whatever the thread count.
    private const int SelectChunk = 256;

    private sealed class SelBuf
    {
        public int[] K = new int[SelectChunk];
        public float[] S = new float[SelectChunk];
        public int Count;
    }

    private SelBuf[] _selBuf = new SelBuf[8];
    private int _selVt, _selTasks;

    private readonly struct SelectProgram : IParallelBody
    {
        private readonly Engine3D _e;
        public SelectProgram(Engine3D e) { _e = e; }
        public void Run(int task) => _e.SelectTask(task);
    }

    private void SelectActive()
    {
        GrowTo(ref _avK, _vtCount); GrowTo(ref _avS, _vtCount); GrowTo(ref _aeK, _eeCount); GrowTo(ref _aeS, _eeCount);
        _selVt = (_vtCount + SelectChunk - 1) / SelectChunk;
        _selTasks = _selVt + (_eeCount + SelectChunk - 1) / SelectChunk;
        if (_selBuf.Length < _selTasks) { Array.Resize(ref _selBuf, Math.Max(_selTasks, 2 * _selBuf.Length)); }
        for (int t = 0; t < _selTasks; t++) { _selBuf[t] ??= new SelBuf(); _selBuf[t]!.Count = 0; }
        if (_selTasks == 0) { return; }
        // Few candidates: not worth waking the pool.
        if (_selTasks == 1) { SelectTask(0); } else { Core.Executor.For(_selTasks, new SelectProgram(this)); }
        for (int t = 0; t < _selTasks; t++)
        {
            var buf = _selBuf[t]!;
            if (t < _selVt) { Array.Copy(buf.K, 0, _avK, _avCount, buf.Count); Array.Copy(buf.S, 0, _avS, _avCount, buf.Count); _avCount += buf.Count; }
            else { Array.Copy(buf.K, 0, _aeK, _aeCount, buf.Count); Array.Copy(buf.S, 0, _aeS, _aeCount, buf.Count); _aeCount += buf.Count; }
        }
    }

    private void SelectTask(int t)
    {
        var buf = _selBuf[t]!;
        int n = 0;
        if (t < _selVt)
        {
            int from = t * SelectChunk, to = Math.Min(_vtCount, from + SelectChunk);
            for (int k = from; k < to; k++)
            {
                if (SelectVertexTriangle(k, out float side)) { buf.K[n] = k; buf.S[n] = side; n++; }
            }
        }
        else
        {
            int from = (t - _selVt) * SelectChunk, to = Math.Min(_eeCount, from + SelectChunk);
            for (int k = from; k < to; k++)
            {
                if (SelectEdgeEdge(k, out float side)) { buf.K[n] = k; buf.S[n] = side; n++; }
            }
        }

        buf.Count = n;
    }

    private bool SelectVertexTriangle(int k, out float side)
    {
        side = 0f;
        int ba = _vtBA[k], bt = _vtBT[k];
        var dp = _awakeSoft[ba]; var dt = _awakeSoft[bt];
        float thick = dp.Radius + dt.Radius;
        double reach = thick + RelTrav(ba, bt, _selH);
        if (_vtD0[k] - RelDisp(ba, bt) - 1e-5 >= reach) { return false; }
        int g = _vtP[k];
        int p = g - dp.Base;
        int el = _gtLocal[_vtT[k]];
        int a = dt.SurfTris[3 * el], b = dt.SurfTris[3 * el + 1], c = dt.SurfTris[3 * el + 2];
        if (dp.InvMass[p] == 0f && dt.InvMass[a] == 0f && dt.InvMass[b] == 0f && dt.InvMass[c] == 0f) { return false; }
        double px = dp.Ox + dp.X[3 * p], py = dp.Oy + dp.X[3 * p + 1], pz = dp.Oz + dp.X[3 * p + 2];
        double ax = dt.Ox + dt.X[3 * a], ay = dt.Oy + dt.X[3 * a + 1], az = dt.Oz + dt.X[3 * a + 2];
        double bx = dt.Ox + dt.X[3 * b], by = dt.Oy + dt.X[3 * b + 1], bz = dt.Oz + dt.X[3 * b + 2];
        double cx = dt.Ox + dt.X[3 * c], cy = dt.Oy + dt.X[3 * c + 1], cz = dt.Oz + dt.X[3 * c + 2];
        if (SoftGeometry.PointTriangleSq(px, py, pz, ax, ay, az, bx, by, bz, cx, cy, cz) >= reach * reach) { return false; }
        bool inside = VertexTriangle(dp, p, dt, a, b, c, out _, out _, out _, out float dist, out _, out _, out _);
        side = inside ? (dist >= 0f ? 1f : -1f) : 0f;
        return true;
    }

    private bool SelectEdgeEdge(int k, out float side)
    {
        side = 0f;
        int ba = _eeBA[k], bb = _eeBB[k];
        var d1 = _awakeSoft[ba]; var d2 = _awakeSoft[bb];
        float thick = d1.Radius + d2.Radius;
        double reach = thick + RelTrav(ba, bb, _selH);
        if (_eeD0[k] - RelDisp(ba, bb) - 1e-5 >= reach) { return false; }
        int g1 = _eeA[k], g2 = _eeB[k];
        int l1 = _geLocal[g1], l2 = _geLocal[g2];
        int a = d1.SurfEdgeA[l1], b = d1.SurfEdgeB[l1], c = d2.SurfEdgeA[l2], d = d2.SurfEdgeB[l2];
        if (d1.InvMass[a] == 0f && d1.InvMass[b] == 0f && d2.InvMass[c] == 0f && d2.InvMass[d] == 0f) { return false; }
        double ax = d1.Ox + d1.X[3 * a], ay = d1.Oy + d1.X[3 * a + 1], az = d1.Oz + d1.X[3 * a + 2];
        double bx = d1.Ox + d1.X[3 * b], by = d1.Oy + d1.X[3 * b + 1], bz = d1.Oz + d1.X[3 * b + 2];
        double cx = d2.Ox + d2.X[3 * c], cy = d2.Oy + d2.X[3 * c + 1], cz = d2.Oz + d2.X[3 * c + 2];
        double dx = d2.Ox + d2.X[3 * d], dy = d2.Oy + d2.X[3 * d + 1], dz = d2.Oz + d2.X[3 * d + 2];
        if (SoftGeometry.SegmentSegmentSq(ax, ay, az, bx, by, bz, cx, cy, cz, dx, dy, dz) >= reach * reach) { return false; }
        bool inside = EdgeEdge(d1, a, b, d2, c, d, out _, out _, out _, out float dist, out _, out _);
        side = inside ? (dist >= 0f ? 1f : -1f) : 0f;
        return true;
    }

    // The particle's projection on the triangle's plane: unit normal (right-handed around the triangle), signed distance of the particle along it, barycentric coordinates of the projection.
    // True when the projection is inside the triangle (within a small tolerance).
    private static bool VertexTriangle(SoftBodyData dp, int p, SoftBodyData dt, int a, int b, int c, out float nx, out float ny, out float nz, out float dist, out float ba, out float bb, out float bc)
    {
        nx = ny = nz = dist = ba = bb = bc = 0f;
        int kp = 3 * p;
        float ax = SoftDelta(dt, dp, 3 * a, kp, 0), ay = SoftDelta(dt, dp, 3 * a, kp, 1), az = SoftDelta(dt, dp, 3 * a, kp, 2);
        float e1x = SoftDelta(dt, dp, 3 * b, kp, 0) - ax, e1y = SoftDelta(dt, dp, 3 * b, kp, 1) - ay, e1z = SoftDelta(dt, dp, 3 * b, kp, 2) - az;
        float e2x = SoftDelta(dt, dp, 3 * c, kp, 0) - ax, e2y = SoftDelta(dt, dp, 3 * c, kp, 1) - ay, e2z = SoftDelta(dt, dp, 3 * c, kp, 2) - az;
        nx = e1y * e2z - e1z * e2y; ny = e1z * e2x - e1x * e2z; nz = e1x * e2y - e1y * e2x;
        float nl2 = nx * nx + ny * ny + nz * nz;
        if (!(nl2 > 1e-24f)) { return false; }
        float inl = 1f / float.Sqrt(nl2);
        nx *= inl; ny *= inl; nz *= inl;
        float wx = -ax, wy = -ay, wz = -az;
        dist = nx * wx + ny * wy + nz * wz;
        float d00 = e1x * e1x + e1y * e1y + e1z * e1z, d01 = e1x * e2x + e1y * e2y + e1z * e2z, d11 = e2x * e2x + e2y * e2y + e2z * e2z;
        float d20 = wx * e1x + wy * e1y + wz * e1z, d21 = wx * e2x + wy * e2y + wz * e2z;
        float den = d00 * d11 - d01 * d01;
        if (!(den > 0f)) { return false; }
        float v = (d11 * d20 - d01 * d21) / den, w2 = (d00 * d21 - d01 * d20) / den;
        ba = 1f - v - w2; bb = v; bc = w2;
        return ba >= -InsideTolerance && bb >= -InsideTolerance && bc >= -InsideTolerance;
    }

    // The closest points of two edges (a,b) and (c,d), when both lie inside the edges: unit direction n of d1 x d2, signed distance from the second point to the first along it, parameters s and t.
    private static bool EdgeEdge(SoftBodyData d1, int a, int b, SoftBodyData d2, int c, int d, out float nx, out float ny, out float nz, out float dist, out float s, out float t)
    {
        nx = ny = nz = dist = s = t = 0f;
        int ka = 3 * a;
        float e1x = SoftDelta(d1, d1, 3 * b, ka, 0), e1y = SoftDelta(d1, d1, 3 * b, ka, 1), e1z = SoftDelta(d1, d1, 3 * b, ka, 2);
        float cx = SoftDelta(d2, d1, 3 * c, ka, 0), cy = SoftDelta(d2, d1, 3 * c, ka, 1), cz = SoftDelta(d2, d1, 3 * c, ka, 2);
        float e2x = SoftDelta(d2, d1, 3 * d, ka, 0) - cx, e2y = SoftDelta(d2, d1, 3 * d, ka, 1) - cy, e2z = SoftDelta(d2, d1, 3 * d, ka, 2) - cz;
        float rx = -cx, ry = -cy, rz = -cz;
        float aa = e1x * e1x + e1y * e1y + e1z * e1z, ee = e2x * e2x + e2y * e2y + e2z * e2z;
        float ff = e2x * rx + e2y * ry + e2z * rz, c1 = e1x * rx + e1y * ry + e1z * rz, bb = e1x * e2x + e1y * e2y + e1z * e2z;
        float den = aa * ee - bb * bb;
        if (!(aa > 0f) || !(ee > 0f) || !(den > 1e-9f * aa * ee)) { return false; }
        s = (bb * ff - c1 * ee) / den;
        if (!(s > 0f && s < 1f)) { return false; }
        t = (bb * s + ff) / ee;
        if (!(t > 0f && t < 1f)) { return false; }
        nx = e1y * e2z - e1z * e2y; ny = e1z * e2x - e1x * e2z; nz = e1x * e2y - e1y * e2x;
        float nl = float.Sqrt(nx * nx + ny * ny + nz * nz);
        if (!(nl > 0f)) { return false; }
        float inl = 1f / nl;
        nx *= inl; ny *= inl; nz *= inl;
        float qx = rx + s * e1x - t * e2x, qy = ry + s * e1y - t * e2y, qz = rz + s * e1z - t * e2z;
        dist = nx * qx + ny * qy + nz * qz;
        return true;
    }


    private void SolveSurfacePairs()
    {
        for (int k = 0; k < _avCount; k++) { SolveVertexTriangle(_avK[k], k); }
        for (int k = 0; k < _aeCount; k++) { SolveEdgeEdge(_aeK[k], k); }
    }

    private void SolveVertexTriangle(int k, int act)
    {
        int g = _vtP[k];
        var dp = _awakeSoft[_vtBA[k]];
        int p = g - dp.Base;
        int gt = _vtT[k];
        var dt = _awakeSoft[_vtBT[k]];
        int el = _gtLocal[gt];
        int a = dt.SurfTris[3 * el], b = dt.SurfTris[3 * el + 1], c = dt.SurfTris[3 * el + 2];
        if (!VertexTriangle(dp, p, dt, a, b, c, out float nx, out float ny, out float nz, out float dist, out float ba, out float bb, out float bc)) { return; }
        float s = _avS[act];
        if (s == 0f) { s = dist >= 0f ? 1f : -1f; _avS[act] = s; }
        float thick = dp.Radius + dt.Radius;
        float cv = s * dist - thick;
        if (!(cv < 0f)) { return; }
        // A particle that is already well behind the triangle is brought back by at most the contact thickness per pass, so a wrong side never launches it.
        if (cv < -thick) { cv = -thick; }
        float wp = dp.InvMass[p], wa = dt.InvMass[a], wb = dt.InvMass[b], wc = dt.InvMass[c];
        float w = wp + ba * ba * wa + bb * bb * wb + bc * bc * wc;
        if (!(w > 0f)) { return; }
        float lam = -cv / w;
        float sx = s * nx, sy = s * ny, sz = s * nz;
        int kp = 3 * p, ka = 3 * a, kb = 3 * b, kc = 3 * c;
        float f = wp * lam;
        dp.X[kp] += sx * f; dp.X[kp + 1] += sy * f; dp.X[kp + 2] += sz * f;
        f = ba * wa * lam; dt.X[ka] -= sx * f; dt.X[ka + 1] -= sy * f; dt.X[ka + 2] -= sz * f;
        f = bb * wb * lam; dt.X[kb] -= sx * f; dt.X[kb + 1] -= sy * f; dt.X[kb + 2] -= sz * f;
        f = bc * wc * lam; dt.X[kc] -= sx * f; dt.X[kc + 1] -= sy * f; dt.X[kc + 2] -= sz * f;
        float mu = float.Sqrt(dp.Def.Friction * dt.Def.Friction);
        if (!(mu > 0f)) { return; }
        // Coulomb friction as a position correction: the part of the particle's displacement relative to the point of the triangle under it that lies in the triangle's plane is removed, up to mu times the normal correction.
        float ux = (dp.X[kp] - dp.XP[kp]) - (ba * (dt.X[ka] - dt.XP[ka]) + bb * (dt.X[kb] - dt.XP[kb]) + bc * (dt.X[kc] - dt.XP[kc]));
        float uy = (dp.X[kp + 1] - dp.XP[kp + 1]) - (ba * (dt.X[ka + 1] - dt.XP[ka + 1]) + bb * (dt.X[kb + 1] - dt.XP[kb + 1]) + bc * (dt.X[kc + 1] - dt.XP[kc + 1]));
        float uz = (dp.X[kp + 2] - dp.XP[kp + 2]) - (ba * (dt.X[ka + 2] - dt.XP[ka + 2]) + bb * (dt.X[kb + 2] - dt.XP[kb + 2]) + bc * (dt.X[kc + 2] - dt.XP[kc + 2]));
        float un = ux * nx + uy * ny + uz * nz;
        float tx = ux - nx * un, ty = uy - ny * un, tz = uz - nz * un;
        float len = float.Sqrt(tx * tx + ty * ty + tz * tz);
        if (!(len > 1e-12f)) { return; }
        float il = 1f / len;
        tx *= il; ty *= il; tz *= il;
        float lt = len / w;
        float cap = mu * lam;
        if (lt > cap) { lt = cap; }
        f = wp * lt; dp.X[kp] -= tx * f; dp.X[kp + 1] -= ty * f; dp.X[kp + 2] -= tz * f;
        f = ba * wa * lt; dt.X[ka] += tx * f; dt.X[ka + 1] += ty * f; dt.X[ka + 2] += tz * f;
        f = bb * wb * lt; dt.X[kb] += tx * f; dt.X[kb + 1] += ty * f; dt.X[kb + 2] += tz * f;
        f = bc * wc * lt; dt.X[kc] += tx * f; dt.X[kc + 1] += ty * f; dt.X[kc + 2] += tz * f;
    }

    private void SolveEdgeEdge(int k, int act)
    {
        int g1 = _eeA[k], g2 = _eeB[k];
        var d1 = _awakeSoft[_eeBA[k]]; var d2 = _awakeSoft[_eeBB[k]];
        int l1 = _geLocal[g1], l2 = _geLocal[g2];
        int a = d1.SurfEdgeA[l1], b = d1.SurfEdgeB[l1], c = d2.SurfEdgeA[l2], d = d2.SurfEdgeB[l2];
        if (!EdgeEdge(d1, a, b, d2, c, d, out float nx, out float ny, out float nz, out float dist, out float s, out float t)) { return; }
        float sg = _aeS[act];
        if (sg == 0f) { sg = dist >= 0f ? 1f : -1f; _aeS[act] = sg; }
        float thick = d1.Radius + d2.Radius;
        float cv = sg * dist - thick;
        if (!(cv < 0f)) { return; }
        if (cv < -thick) { cv = -thick; }
        float wa = d1.InvMass[a], wb = d1.InvMass[b], wc = d2.InvMass[c], wd = d2.InvMass[d];
        float ca = 1f - s, cb = s, cc = 1f - t, cd = t;
        float w = ca * ca * wa + cb * cb * wb + cc * cc * wc + cd * cd * wd;
        if (!(w > 0f)) { return; }
        float lam = -cv / w;
        float sx = sg * nx, sy = sg * ny, sz = sg * nz;
        int ka = 3 * a, kb = 3 * b, kc = 3 * c, kd = 3 * d;
        float f = ca * wa * lam; d1.X[ka] += sx * f; d1.X[ka + 1] += sy * f; d1.X[ka + 2] += sz * f;
        f = cb * wb * lam; d1.X[kb] += sx * f; d1.X[kb + 1] += sy * f; d1.X[kb + 2] += sz * f;
        f = cc * wc * lam; d2.X[kc] -= sx * f; d2.X[kc + 1] -= sy * f; d2.X[kc + 2] -= sz * f;
        f = cd * wd * lam; d2.X[kd] -= sx * f; d2.X[kd + 1] -= sy * f; d2.X[kd + 2] -= sz * f;
        float mu = float.Sqrt(d1.Def.Friction * d2.Def.Friction);
        if (!(mu > 0f)) { return; }
        float ux = (ca * (d1.X[ka] - d1.XP[ka]) + cb * (d1.X[kb] - d1.XP[kb])) - (cc * (d2.X[kc] - d2.XP[kc]) + cd * (d2.X[kd] - d2.XP[kd]));
        float uy = (ca * (d1.X[ka + 1] - d1.XP[ka + 1]) + cb * (d1.X[kb + 1] - d1.XP[kb + 1])) - (cc * (d2.X[kc + 1] - d2.XP[kc + 1]) + cd * (d2.X[kd + 1] - d2.XP[kd + 1]));
        float uz = (ca * (d1.X[ka + 2] - d1.XP[ka + 2]) + cb * (d1.X[kb + 2] - d1.XP[kb + 2])) - (cc * (d2.X[kc + 2] - d2.XP[kc + 2]) + cd * (d2.X[kd + 2] - d2.XP[kd + 2]));
        float un = ux * nx + uy * ny + uz * nz;
        float tx = ux - nx * un, ty = uy - ny * un, tz = uz - nz * un;
        float len = float.Sqrt(tx * tx + ty * ty + tz * tz);
        if (!(len > 1e-12f)) { return; }
        float il = 1f / len;
        tx *= il; ty *= il; tz *= il;
        float lt = len / w;
        float cap = mu * lam;
        if (lt > cap) { lt = cap; }
        f = ca * wa * lt; d1.X[ka] -= tx * f; d1.X[ka + 1] -= ty * f; d1.X[ka + 2] -= tz * f;
        f = cb * wb * lt; d1.X[kb] -= tx * f; d1.X[kb + 1] -= ty * f; d1.X[kb + 2] -= tz * f;
        f = cc * wc * lt; d2.X[kc] += tx * f; d2.X[kc + 1] += ty * f; d2.X[kc + 2] += tz * f;
        f = cd * wd * lt; d2.X[kd] += tx * f; d2.X[kd + 1] += ty * f; d2.X[kd + 2] += tz * f;
    }

    // ------------------------------------------------------------------ the net speed that decides rest

    // The motion of the step as a whole: the substeps' last velocity can swing back and forth around a position that barely moves, so rest is judged from the net displacement of the step,
    // smoothed over about ten steps. The mass-weighted mean square speed of the free particles.
    private const float SmoothK = 0.1f;
    private const float MaxNetSpeed2 = 1e30f;

    private static void NetSpeed(SoftBodyData d, float invDt)
    {
        double ke = 0.0, mass = 0.0;
        for (int i = 0; i < d.N; i++)
        {
            if (d.InvMass[i] == 0f) { continue; }
            float dx = (d.X[3 * i] - d.X0[3 * i]) * invDt, dy = (d.X[3 * i + 1] - d.X0[3 * i + 1]) * invDt, dz = (d.X[3 * i + 2] - d.X0[3 * i + 2]) * invDt;
            double mi = 1.0 / d.InvMass[i];
            ke += mi * (dx * dx + dy * dy + dz * dz); mass += mi;
        }

        float now = mass > 0.0 ? (float)(ke / mass) : 0f;
        // A diverged body (a particle at infinity or not a number) reads as moving as fast as the format allows, so the value stays finite and a snapshot of it restores.
        if (!(now <= MaxNetSpeed2)) { now = MaxNetSpeed2; }
        d.StepSpeed2 += (now - d.StepSpeed2) * SmoothK;
    }

    // ------------------------------------------------------------------ plasticity and tearing at the end of a step

    private long[]?[] _tearKeys = new long[]?[4];

    private void DetectSoftTears(float h)
    {
        GrowTo(ref _tearKeys, _awakeSoftCount);
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            var d = _awakeSoft[a];
            _tearKeys[a] = d.Def.PlasticYield > 0f || d.Def.TearStrain > 0f || d.Def.TearForce > 0f ? PlasticAndTears(d, h) : null;
        }
    }

    private static long[]? PlasticAndTears(SoftBodyData d, float h)
    {
        var keys = d.DetectTears(h);
        d.ApplyPlasticity();
        return keys.Length == 0 ? null : keys;
    }

    private void ApplySoftTears()
    {
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            var keys = _tearKeys[a];
            if (keys == null) { continue; }
            _tearKeys[a] = null;
            var d = _awakeSoft[a];
            // A tear can add at most one particle per element corner; stop tearing before the particle numbering runs out.
            if ((long)_softParticles + d.Tets.Length + d.Tris.Length > MaxSoftParticles) { continue; }
            int added = d.Tear(keys);
            int slot = Array.IndexOf(SoftData, d);
            var handle = new SoftBody(World!, slot, SoftHandles.GenerationOf(slot));
            foreach (long key in keys) { _softTears.Add(new SoftTearEvent(handle, (int)(key >> 20), (int)(key & 0xFFFFF), d.N)); }
            _softParticles += added;
            d.SleepTime = 0f;
            d.HadMovingContact = true;
        }
    }
}
