using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    // The solve of one substep: SoftIterations passes, each a sequence of stages. A stage runs the tasks of one solver phase of every body (the colours of the edges, then of the bends, then of the
    // tetrahedra, then pressure) in parallel; a body that is small runs all its phases as one task in the first stage. Then a stage for the contacts with shapes and one for the particle pairs,
    // each a single task in a fixed order. The split into tasks comes from the bodies' constraint graphs alone, so the result is the same on any number of threads.

    // The three sizes below only decide how the work is cut into tasks; the result is the same for any value (a test and the hash matrix check that), so a benchmark may change them.

    /// <summary>Bodies with at least this many particles are split across tasks by colour; smaller ones are solved by one task, because below this size the waits between colours cost more than the shared work saves.</summary>
    internal static int SoftBigBody = 1024;

    /// <summary>Constraints per task within a colour (a multiple of the lane width).</summary>
    internal static int SoftTaskSize = 64;

    private int[] _stBody = new int[64], _stPhase = new int[64], _stFirst = new int[64], _stEnd = new int[64];
    private int[] _stStart = new int[32];
    private int _stStages, _stMost;

    // Lists the tasks of every constraint stage of a pass. The table depends on the awake bodies only.
    private void BuildSoftTasks()
    {
        int stages = 1;
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            var d = _awakeSoft[a];
            if (d.N >= SoftBigBody && d.Phases > stages) { stages = d.Phases; }
        }

        GrowTo(ref _stStart, stages + 1);
        int n = 0, most = 1;
        for (int s = 0; s < stages; s++)
        {
            _stStart[s] = n;
            for (int a = 0; a < _awakeSoftCount; a++)
            {
                var d = _awakeSoft[a];
                if (d.N < SoftBigBody)
                {
                    if (s == 0) { AddSoftTask(ref n, a, -1, 0, 0); }
                    continue;
                }

                if (s >= d.Phases) { continue; }
                d.Phase(s, out _, out int first, out int end, out bool serial);
                if (serial) { AddSoftTask(ref n, a, s, first, end); continue; }
                for (int f = first; f < end; f += SoftTaskSize) { AddSoftTask(ref n, a, s, f, Math.Min(end, f + SoftTaskSize)); }
            }

            if (n - _stStart[s] > most) { most = n - _stStart[s]; }
        }

        _stStart[stages] = n;
        _stStages = stages;
        _stMost = most;
    }

    private void AddSoftTask(ref int n, int body, int phase, int first, int end)
    {
        GrowTo(ref _stBody, n + 1); GrowTo(ref _stPhase, n + 1); GrowTo(ref _stFirst, n + 1); GrowTo(ref _stEnd, n + 1);
        _stBody[n] = body; _stPhase[n] = phase; _stFirst[n] = first; _stEnd[n] = end;
        n++;
    }

    private void RunSoftTask(int t, float h)
    {
        var d = _awakeSoft[_stBody[t]];
        int ph = _stPhase[t];
        if (ph < 0)
        {
            for (int p = 0; p < d.Phases; p++)
            {
                d.Phase(p, out int kind, out int first, out int end, out bool serial);
                SolveRange(d, kind, first, end, h, !serial);
            }

            return;
        }

        d.Phase(ph, out int k, out _, out _, out bool ser);
        SolveRange(d, k, _stFirst[t], _stEnd[t], h, !ser);
    }

    private readonly struct SoftSolveProgram : IStagedBody
    {
        private readonly Engine3D _e;
        private readonly float _h, _invH;
        private readonly int _perPass;
        private readonly float _tSkin;

        public SoftSolveProgram(Engine3D e, float h, float invH, float tSkin)
        {
            _e = e; _h = h; _invH = invH; _tSkin = tSkin; _perPass = e._stStages + 3;
        }

        public int TaskCount(int stage)
        {
            int k = stage % _perPass;
            if (k == _e._stStages + 2) { return _e._skinCount; }
            if (k >= _e._stStages) { return 1; }
            return _e._stStart[k + 1] - _e._stStart[k];
        }

        public void Run(int stage, int task)
        {
            int k = stage % _perPass;
            if (k < _e._stStages) { _e.RunSoftTask(_e._stStart[k] + task, _h); }
            else if (k == _e._stStages) { _e.SolveContacts(_h, _invH); }
            else if (k == _e._stStages + 1) { _e.SolvePairs(); }
            else { SolveSkin(_e._awakeSoft[_e._skinBody[task]], _tSkin); }
        }

        // Implemented here so that the single-thread path does not box the struct to reach the interface's default.
        public void RunRange(int stage, int first, int count)
        {
            for (int i = 0; i < count; i++) { Run(stage, first + i); }
        }
    }

    private int[] _skinBody = new int[4];
    private int _skinCount;

    private void SolveSubstep(float h, float invH, int iterations, float tSkin)
    {
        if (iterations <= 0) { return; }
        BuildSoftTasks();
        // The skin limits come last in every pass, after the contacts and pairs, one task per skinned body.
        _skinCount = 0;
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            if (!_awakeSoft[a].IsSkinned) { continue; }
            GrowTo(ref _skinBody, _skinCount + 1);
            _skinBody[_skinCount++] = a;
        }

#if STAGE_TIMERS
        StageProbe.Mark(StageProbe.SoftConstraints);
#endif
        Core.Executor.ForStages(iterations * (_stStages + 3), new SoftSolveProgram(this, h, invH, tSkin), _stMost);
    }

    // ------------------------------------------------------------------ contact detection and pair search in tasks

    /// <summary>Particles per detection task.</summary>
    internal static int DetectChunk = 128;

    private SoftTaskBuf[] _tb = new SoftTaskBuf[8];
    private int[] _dtBody = new int[16], _dtFrom = new int[16], _dtTo = new int[16];
    private int _dtCount;

    private void EnsureTaskBufs(int n)
    {
        if (_tb.Length < n) { Array.Resize(ref _tb, Math.Max(n, 2 * _tb.Length)); }
        for (int i = 0; i < n; i++) { _tb[i] ??= new SoftTaskBuf(); }
    }

    // The particles of every awake body cut into chunks, in body and particle order; the chunk bounds depend on the particle counts alone.
    private void BuildDetectTasks()
    {
        int n = 0;
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            int count = _awakeSoft[a].N;
            for (int f = 0; f < count; f += DetectChunk)
            {
                GrowTo(ref _dtBody, n + 1); GrowTo(ref _dtFrom, n + 1); GrowTo(ref _dtTo, n + 1);
                _dtBody[n] = a; _dtFrom[n] = f; _dtTo[n] = Math.Min(count, f + DetectChunk); n++;
            }
        }

        _dtCount = n;
        EnsureTaskBufs(n);
    }

    private readonly struct DetectProgram : IParallelBody
    {
        private readonly Engine3D _e;
        public DetectProgram(Engine3D e) { _e = e; }
        public void Run(int task) => _e.DetectTask(task);
    }

    private readonly struct PairProgram : IParallelBody
    {
        private readonly Engine3D _e;
        public PairProgram(Engine3D e) { _e = e; }
        public void Run(int task) => _e.PairTask(task);
    }

    private void DetectTask(int t)
    {
        var buf = _tb[t];
        buf.Count = 0; buf.Moving = false;
        DetectRange(_awakeSoft[_dtBody[t]], _dtFrom[t], _dtTo[t], buf);
    }

    private void PairTask(int t)
    {
        int g0 = t * PairChunk;
        FindPairs(g0, Math.Min(_pgTotal, g0 + PairChunk), _tb[t]);
    }

    // Finds the contacts of every particle against the candidate shapes, in tasks, and joins them in particle order, so the list is the one a single pass would make. The rigid bodies that take
    // a reaction are registered here, in that order, after the tasks are done.
    private void DetectAll()
    {
        for (int a = 0; a < _awakeSoftCount; a++) { _awakeSoft[a].HadMovingContact = false; }
        Core.Executor.For(_dtCount, new DetectProgram(this));
        _scCount = 0;
        for (int t = 0; t < _dtCount; t++)
        {
            var tb = _tb[t];
            if (tb.Moving) { _awakeSoft[_dtBody[t]].HadMovingContact = true; }
            GrowTo(ref _sc, _scCount + tb.Count);
            for (int i = 0; i < tb.Count; i++)
            {
                _sc[_scCount++] = tb.C[i];
                if (tb.C[i].Coupled) { TouchSoftBody(tb.C[i].Rigid); }
            }
        }
    }
}

/// <summary>Output of one detection or pair-search task: contacts (at most four per particle) or pairs, joined in task order afterwards.</summary>
internal sealed class SoftTaskBuf
{
    public SoftContact[] C = new SoftContact[64];
    public int Count;
    public bool Moving;
    public long[] Pairs = new long[64];
    public int PairCount;
}
