using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    // The solve of one substep: SoftIterations passes, each a sequence of stages.
    //   stage 0          the interior constraints of every block of every body, one task per block (no wait inside a block);
    //   boundary stages  the constraints that cross blocks, one colour per stage (or one task per body), then the pressure constraint;
    //   contacts         the contacts with shapes, in tasks of whole particles; contacts that move a rigid body are one serial task, in detection order;
    //   pairs            the particle pairs, one task in the order listed;
    //   skin             the skin limits, one task per skinned body.
    // The split into tasks comes from the bodies' blocks and constraint graphs alone, so the result is the same on any number of threads.

    // The sizes below only decide how the work is cut into tasks; the result is the same for any value (a test and the hash matrix check that), so a benchmark may change them.
    // (The size of a block is different: it decides the constraint order, so it is fixed when a body is created; see SoftBlockSize.)

    /// <summary>Contacts per task (whole particles).</summary>
    internal static int SoftContactChunk = 16;

    /// <summary>Experiment: tasks every stage reports so that the executor keeps its workers.</summary>
    internal static int SoftPadTasks = 8;

    private int[] _stBody = new int[64], _stRegion = new int[64];
    private int[] _stStart = new int[4];
    private int _stStages, _stMost;
    private int[] _ctIdx = new int[64], _ctStart = new int[16];
    private int _ctTasks;

    // Lists the tasks of the constraint stages of a pass: stage 0 has a task for every block of every body that has interior constraints, stage 1 one task per body for its boundary constraints and its pressure
    // constraint. The table depends on the awake bodies only.
    private void BuildSoftTasks()
    {
        GrowTo(ref _stStart, 3);
        int n = 0;
        _stStart[0] = 0;
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            var d = _awakeSoft[a];
            for (int b = 0; b < d.BlockCount; b++) { if (d.StepStart[b + 1] > d.StepStart[b]) { AddSoftTask(ref n, a, b); } }
        }

        _stStart[1] = n;
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            var d = _awakeSoft[a];
            if (d.BoundarySteps > 0 || d.HasPressure) { AddSoftTask(ref n, a, d.BlockCount); }
        }

        _stStart[2] = n;
        _stStages = 2;
        _stMost = Math.Max(_stStart[1], _stStart[2] - _stStart[1]);
    }

    private void AddSoftTask(ref int n, int body, int region)
    {
        GrowTo(ref _stBody, n + 1); GrowTo(ref _stRegion, n + 1);
        _stBody[n] = body; _stRegion[n] = region;
        n++;
    }

    // Solves the steps of one region of a body: for each kind, the groups of four in lanes, then the singles; the boundary set ends with the pressure constraint.
    private void RunSoftTask(int t, float h)
    {
        var d = _awakeSoft[_stBody[t]];
        int r = _stRegion[t];
        for (int s = d.StepStart[r]; s < d.StepStart[r + 1]; s++)
        {
            int kind = d.StepKind[s];
            SolveRange(d, kind, d.StepFirst[s], d.StepQuadEnd[s], h, true);
            SolveRange(d, kind, d.StepQuadEnd[s], d.StepEnd[s], h, false);
        }

        if (r == d.BlockCount && d.HasPressure) { SolveRange(d, 3, 0, 1, h, false); }
    }

    // The contacts of the substep cut into tasks. A particle's contacts stay together and in order. Particles with a contact that moves a rigid body (shared state) form the first task, in detection order;
    // the other particles are independent of each other and are cut into chunks of about SoftContactChunk contacts.
    private void BuildContactTasks()
    {
        GrowTo(ref _ctIdx, _scCount + 1);
        int n = 0, tasks = 0;
        GrowTo(ref _ctStart, 2);
        _ctStart[0] = 0;
        for (int i = 0; i < _scCount;)
        {
            int j = i + 1;
            bool coupled = _sc[i].Coupled;
            while (j < _scCount && _sc[j].Particle == _sc[i].Particle && _sc[j].Body == _sc[i].Body) { coupled |= _sc[j].Coupled; j++; }
            if (coupled) { for (int k = i; k < j; k++) { _ctIdx[n++] = k; } }
            i = j;
        }

        if (n > 0) { tasks = 1; _ctStart[1] = n; }
        int chunkStart = n;
        for (int i = 0; i < _scCount;)
        {
            int j = i + 1;
            bool coupled = _sc[i].Coupled;
            while (j < _scCount && _sc[j].Particle == _sc[i].Particle && _sc[j].Body == _sc[i].Body) { coupled |= _sc[j].Coupled; j++; }
            if (!coupled)
            {
                for (int k = i; k < j; k++) { _ctIdx[n++] = k; }
                if (n - chunkStart >= SoftContactChunk)
                {
                    GrowTo(ref _ctStart, tasks + 2);
                    _ctStart[++tasks] = n; chunkStart = n;
                }
            }

            i = j;
        }

        if (n > chunkStart)
        {
            GrowTo(ref _ctStart, tasks + 2);
            _ctStart[++tasks] = n;
        }

        _ctTasks = tasks;
        if (tasks > _stMost) { _stMost = tasks; }
    }

    private readonly struct SoftSolveProgram : IStagedBody
    {
        private readonly Engine3D _e;
        private readonly float _h, _invH;
        private readonly int _perPass;
        private readonly float _tSkin;
        private readonly int _pad;

        public SoftSolveProgram(Engine3D e, float h, float invH, float tSkin)
        {
            _e = e; _h = h; _invH = invH; _tSkin = tSkin; _perPass = e._stStages + 3; _pad = SoftPadTasks;
        }

        private int Real(int k)
        {
            if (k < _e._stStages) { return _e._stStart[k + 1] - _e._stStart[k]; }
            if (k == _e._stStages) { return _e._ctTasks; }
            if (k == _e._stStages + 1) { return _e._pairCount > 0 ? 1 : 0; }
            return _e._skinCount;
        }

        public int TaskCount(int stage)
        {
            int r = Real(stage % _perPass);
            return r > 0 && r < _pad ? _pad : r;
        }

        public void Run(int stage, int task)
        {
            int k = stage % _perPass;
            if (task >= Real(k)) { return; }
            if (k < _e._stStages) { _e.RunSoftTask(_e._stStart[k] + task, _h); }
            else if (k == _e._stStages) { _e.SolveContactTask(task, _h, _invH); }
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
        BuildContactTasks();
        // The skin limits come last in every pass, after the contacts and pairs, one task per skinned body.
        _skinCount = 0;
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            if (!_awakeSoft[a].IsSkinned) { continue; }
            GrowTo(ref _skinBody, _skinCount + 1);
            _skinBody[_skinCount++] = a;
        }

        if (_skinCount > _stMost) { _stMost = _skinCount; }
#if STAGE_TIMERS
        StageProbe.Mark(StageProbe.SoftConstraints);
#endif
        Core.Executor.ForStages(iterations * (_stStages + 3), new SoftSolveProgram(this, h, invH, tSkin), _stMost);
    }

    // ------------------------------------------------------------------ contact detection and pair search in tasks

    /// <summary>Particles per detection task.</summary>
    internal static int DetectChunk = 32;

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
