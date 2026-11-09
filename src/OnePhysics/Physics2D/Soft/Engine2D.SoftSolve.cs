using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    // The solve of one substep: SoftIterations passes, each a sequence of stages.
    //   stage 0          the interior constraints of every block of every body, one task per block (no wait inside a block);
    //   stage 1          the boundary constraints, the pressure constraint and the shape-matching clusters of every body, one task per body;
    //   contacts         the contacts with shapes, in tasks of whole particles; contacts that move a rigid body are one serial task, in detection order;
    //   pairs            the particle pairs, one task in the order listed;
    //   targets          the distance limits of targeted particles, one task per body that has targets.
    // The split into tasks comes from the bodies' blocks alone, so the result is the same on any number of threads.

    /// <summary>The executor takes as many workers as there is one task each in the average stage. A pass has a stage of blocks, then stages of one task (boundary, pairs) or none, so the average undercounts the blocks;
    /// a stage with tasks reports at least this many (the extra tasks do nothing), which only keeps more workers for the block stage. Speed only; the result does not depend on it.</summary>
    internal static int SoftPadTasks = 8;

    /// <summary>Contacts per task (whole particles).</summary>
    internal static int SoftContactChunk = 16;

    private int[] _stBody = new int[64], _stRegion = new int[64];
    private int[] _stStart = new int[4];
    private int _stStages, _stMost;
    private int[] _ctIdx = new int[64], _ctStart = new int[16];
    private int _ctTasks;

    private void BuildSoftTasks()
    {
        GrowTo(ref _stStart, 3);
        int n = 0;
        _stStart[0] = 0;
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            var d = _awakeSoft[a];
            for (int b = 0; b < d.BlockCount; b++) { if (d.BlockHasWork(b)) { AddSoftTask(ref n, a, b); } }
        }

        _stStart[1] = n;
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            var d = _awakeSoft[a];
            if (d.HasBoundaryWork) { AddSoftTask(ref n, a, d.BlockCount); }
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

    // Solves the constraints of one region of a body: edges, bends, triangles; the boundary set goes on with the pressure constraint and the shape-matching clusters.
    private void RunSoftTask(int t, float h, bool reverse)
    {
        var d = _awakeSoft[_stBody[t]];
        int r = _stRegion[t];
        if (d.UsesNetwork)
        {
            SolveEdges(d, d.EdgeStart[r], d.EdgeStart[r + 1], h, reverse);
            SolveBends(d, d.BendStart[r], d.BendStart[r + 1], h, reverse);
            SolveAreas(d, d.TriStart[r], d.TriStart[r + 1], h, reverse);
            if (r == d.BlockCount && d.HasPressure) { SolvePressure(d, h); }
        }

        if (r == d.BlockCount && d.UsesMatching) { SolveClusters(d, h); }
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
        private readonly Engine2D _e;
        private readonly float _h, _invH;
        private readonly int _perPass;
        private readonly float _tTarget;
        private readonly int _pad;

        public SoftSolveProgram(Engine2D e, float h, float invH, float tTarget)
        {
            _e = e; _h = h; _invH = invH; _tTarget = tTarget; _perPass = e._stStages + 3; _pad = SoftPadTasks;
        }

        public int TaskCount(int stage)
        {
            int r = Real(stage % _perPass);
            return r > 0 && r < _pad ? _pad : r;
        }

        private int Real(int k)
        {
            if (k < _e._stStages) { return _e._stStart[k + 1] - _e._stStart[k]; }
            if (k == _e._stStages) { return _e._ctTasks; }
            if (k == _e._stStages + 1) { return _e._pairCount > 0 || _e._seCount > 0 ? 1 : 0; }
            return _e._targetCount;
        }

        public void Run(int stage, int task)
        {
            int k = stage % _perPass;
            if (task >= Real(k)) { return; }
            if (k < _e._stStages) { _e.RunSoftTask(_e._stStart[k] + task, _h, ((stage / _perPass + _e._softSubstep) & 1) != 0); }
            else if (k == _e._stStages) { _e.SolveContactTask(task, _h, _invH); }
            else if (k == _e._stStages + 1) { _e.SolvePairs(); }
            else { LimitTargets(_e._awakeSoft[_e._targetBody[task]]); }
        }

        // Implemented here so that the single-thread path does not box the struct to reach the interface's default.
        public void RunRange(int stage, int first, int count)
        {
            for (int i = 0; i < count; i++) { Run(stage, first + i); }
        }
    }

    private int _softSubstep;
    private int[] _targetBody = new int[4];
    private int _targetCount;

    private void SolveSubstep(float h, float invH, int iterations)
    {
        if (iterations <= 0) { return; }
        BuildSoftTasks();
        BuildContactTasks();
        // The target limits come last in every pass, after the contacts and pairs, one task per body that has targets.
        _targetCount = 0;
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            if (!_awakeSoft[a].HasTargets) { continue; }
            GrowTo(ref _targetBody, _targetCount + 1);
            _targetBody[_targetCount++] = a;
        }

        if (_targetCount > _stMost) { _stMost = _targetCount; }
        Core.Profile.Mark(StepStage.SoftConstraints);
        Core.Executor.ForStages(iterations * (_stStages + 3), new SoftSolveProgram(this, h, invH, 1f), _stMost);
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
        private readonly Engine2D _e;
        public DetectProgram(Engine2D e) { _e = e; }
        public void Run(int task) => _e.DetectTask(task);
    }

    private readonly struct PairProgram : IParallelBody
    {
        private readonly Engine2D _e;
        public PairProgram(Engine2D e) { _e = e; }
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

    // ------------------------------------------------------------------ prediction and finishing

    private static void PredictSoft(SoftBodyData d, float h, Vector2F g)
    {
        float gs = d.Def.GravityScale;
        float damp = 1f / (1f + d.Def.LinearDamping * h);
        float gx = g.X * gs * h, gy = g.Y * gs * h;
        var x = d.X; var xp = d.XP; var vel = d.Vel; var inv = d.InvMass;
        for (int i = 0; i < d.N; i++)
        {
            int k = 2 * i;
            xp[k] = x[k]; xp[k + 1] = x[k + 1];
            if (inv[i] == 0f) { continue; }
            float vx = (vel[k] + gx) * damp, vy = (vel[k + 1] + gy) * damp;
            vel[k] = vx; vel[k + 1] = vy;
            x[k] += vx * h; x[k + 1] += vy * h;
        }
    }

    private void FinishSubstep(SoftBodyData d, float invH)
    {
        var x = d.X; var xp = d.XP; var vel = d.Vel; var inv = d.InvMass;
        float vmax = Opt.MaxLinearSpeed;
        float vmax2 = vmax * vmax;
        for (int i = 0; i < d.N; i++)
        {
            int k = 2 * i;
            if (inv[i] == 0f) { vel[k] = 0f; vel[k + 1] = 0f; continue; }
            float vx = (x[k] - xp[k]) * invH, vy = (x[k + 1] - xp[k + 1]) * invH;
            float s2 = vx * vx + vy * vy;
            if (s2 > vmax2)
            {
                float f = vmax / float.Sqrt(s2);
                vx *= f; vy *= f;
            }

            vel[k] = vx; vel[k + 1] = vy;
        }
    }

    // ------------------------------------------------------------------ kinematic targets

    // Called once at the start of the step's first substep prediction by PredictTargets: the targets of the substep are interpolated between the previous step's pose and the current one; a particle whose
    // maximum distance is 0 is put on its target, which is also what makes it kinematic for the constraints.
    private static void PredictTargets(SoftBodyData d, float t)
    {
        var now = d.TargetNow;
        for (int i = 0; i < d.N; i++)
        {
            if (d.TargetOn[i] == 0) { continue; }
            int k = 2 * i;
            double tx = d.TargetPrev[k] + (d.TargetPos[k] - d.TargetPrev[k]) * t, ty = d.TargetPrev[k + 1] + (d.TargetPos[k + 1] - d.TargetPrev[k + 1]) * t;
            now[k] = (float)(tx - d.Ox); now[k + 1] = (float)(ty - d.Oy);
            if (d.TargetMax[i] == 0f && d.Pinned[i] == 0) { d.X[k] = now[k]; d.X[k + 1] = now[k + 1]; }
        }
    }

    private static void EndTargets(SoftBodyData d) => Array.Copy(d.TargetPos, d.TargetPrev, d.TargetPos.Length);

    // The distance limit of the targets (the long-range attachments of Kim, Chentanez and Müller 2012; Müller et al. 2007): a free particle stays within its maximum distance of its target.
    // The limit is a hard projection, last in every pass, so it beats contacts.
    private static void LimitTargets(SoftBodyData d)
    {
        var x = d.X; var now = d.TargetNow;
        for (int i = 0; i < d.N; i++)
        {
            if (d.TargetOn[i] == 0 || d.InvMass[i] == 0f) { continue; }
            int k = 2 * i;
            float dx = x[k] - now[k], dy = x[k + 1] - now[k + 1];
            float lim = d.TargetMax[i];
            float len2 = dx * dx + dy * dy;
            if (!(len2 > lim * lim)) { continue; }
            float len = float.Sqrt(len2);
            float f = lim / len;
            x[k] = now[k] + dx * f; x[k + 1] = now[k + 1] + dy * f;
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
    public long[] SurfPairs = new long[64];
    public float[] SurfSide = new float[64];
    public int SurfCount;
}
