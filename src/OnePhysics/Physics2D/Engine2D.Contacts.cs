using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>The step stages of the 2D engine. A struct so the core's generic step specializes on it; all state lives in the engine object.</summary>
internal readonly struct Stages2D : IWorldStages
{
    private readonly Engine2D _e;

    public Stages2D(Engine2D e) => _e = e;

    public void UpdateContacts(WorldCore world) => _e.UpdateContacts(world);

    public void BeginSolve(WorldCore world, int batches, in StepContext context) => _e.BeginSolve(batches, in context);

    public void PrepareBatch(WorldCore world, ReadOnlySpan<int> constraints, int batch, in StepContext context) => _e.PrepareBatch(constraints, batch, in context);

    public void Prepare(WorldCore world, ReadOnlySpan<int> constraints, in StepContext context) => _e.PrepareJoints(constraints, in context);

    public void IntegrateVelocities(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context) => _e.IntegrateVelocities(bodies, in context);

    public void SolveBatch(WorldCore world, ReadOnlySpan<int> constraints, SolvePhase phase, in StepContext context) => _e.SolveBatch(constraints, phase, in context);

    public void IntegratePositions(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context) => _e.IntegratePositions(bodies, in context);

    public void StoreResults(WorldCore world, ReadOnlySpan<int> constraints, int eventSlot, in StepContext context)
    {
        // Impulses are updated in place in each constraint's payload, so there is nothing to copy back; joints report their reaction and may break, and contacts raise hit events.
        _e.StoreContactResults(constraints); // first: it also copies the wide joint impulses back into the payloads the joint results read
        _e.StoreJointResults(constraints, in context);
        _e.EmitHits(world, constraints, eventSlot);
    }

    public float BodyMotion(WorldCore world, int body) => _e.BodyMotion(body);

    public bool BodyMotionIsConcurrent => true;

    public void FinalizeBodies(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context) => _e.FinalizeBodies(bodies, in context);

    public void AfterFinalize(WorldCore world, in StepContext context) => _e.RunBullets();
}

/// <summary>The pose of a body at the start of the contact stage and what the pair loops need to know about it, in one record so a pair reads two records instead of a dozen scattered arrays.</summary>
internal struct BodyHot
{
    public const int Active = 1, AsleepDynamic = 2;
    public double X, Y;
    public float C, S, Lcx, Lcy, Extent;
    public int Flags;
}

/// <summary>The pose of B in A's frame when a pair last had no points, and how far apart (beyond the speculative distance) the shapes were proved to be then.</summary>
internal struct SkipFrame
{
    public float X, Y, C, S, Clear;

    /// <summary>Farthest point of B's shape from B's centre of mass, so the bound holds for any kind of body (a kinematic or static body has no stored extent).</summary>
    public float Reach;
}

/// <summary>One entry of the pair table: the two shapes (A below B), their contact constraint while they touch (or -1), the owning bodies (-1 until first needed) and the narrowphase hints.</summary>
internal struct Pair2D
{
    public int A, B, Cons, BodyA, BodyB;
    public bool Live, Sensor;
    public int MixEpoch; // the material epoch whose mixed coefficients the constraint's payload holds
    public SkipFrame Skip;
}

internal sealed partial class Engine2D
{
    // ------------------------------------------------------------------ pair filter for the broadphase

    private readonly struct PairFilter : IPairFilter
    {
        private readonly Engine2D _e;

        public PairFilter(Engine2D e) => _e = e;

        public bool IsSensor(int shape) => _e.IsSensor(shape);

        public bool ShouldCollide(int shapeA, int shapeB)
        {
            // A pair that already has a table entry is not new, so it is dropped here instead of being sorted with the new ones and dropped after.
            // The table is only read while pairs are searched; it changes in the serial stage that follows.
            if (_e.PairMap.ContainsKey(shapeA < shapeB ? PairKey(shapeA, shapeB) : PairKey(shapeB, shapeA))) { return false; }
            var owner = _e.Core.Shapes.Owner;
            return owner[shapeA] != owner[shapeB] && _e.CanCollide(shapeA, shapeB) && !_e.JointsPreventCollision(owner[shapeA], owner[shapeB]);
        }
    }

    private struct PairResult
    {
        public byte Status; // 0 untouched, 1 manifold computed, 2 boxes no longer overlap
        public bool Recycled; // the manifold is the previous one moved with the bodies (see Recycle), not a new narrowphase result
        public int Count;
        public float Nx, Ny;
        public float RAx0, RAy0, RBx0, RBy0, Sep0;
        public float RAx1, RAy1, RBx1, RBy1, Sep1;
        public uint Id0, Id1;
    }

    private int[] _updateList = new int[64];
    private PairResult[] _results = new PairResult[64];
    private int _collideBlocks;

    // ------------------------------------------------------------------ contact stage

    public void UpdateContacts(WorldCore world)
    {
        var bodies = world.Bodies;
        int nb = bodies.HighWater;
        EnsureBodies(nb);
        if (Hot.Length < nb) { Hot = new BodyHot[Math.Max(nb, Hot.Length * 2)]; }
        ReconcileJoints();
        SizeJointStorage();
        // 0. A changed filter re-evaluates the pair table before anything else (see Refilter).
        Refilter();

        // 1. Broadphase boxes of every body that moved last step. The displacement stretches the box ahead of the motion. The boxes and the test against the stored boxes run in parallel;
        //    the tree itself is updated serially, in body order.
        var awake = world.AwakeBodies;
        EnsureBoxStorage();
        _stamp++;
        int boxTasks = (awake.Length + 63) / 64;
        if (_moverLists.Length < boxTasks) { Array.Resize(ref _moverLists, Math.Max(boxTasks, _moverLists.Length * 2)); Array.Resize(ref _moverCounts, _moverLists.Length); }
        world.Executor.For(boxTasks, new BoxTask(this, world.AwakeArray, awake.Length));
        // The shapes whose boxes left their stored fat box, in the order of the awake list (task by task, each task in list order), are moved in the tree here, serially.
        for (int t = 0; t < boxTasks; t++)
        {
            int[] movers = _moverLists[t];
            for (int i = 0, n = _moverCounts[t]; i < n; i++)
            {
                int s = movers[i];
                int b = world.Shapes.Owner[s];
                ref ShapeBox sb = ref _sbox[s];
                Bp.MoveProxy(Proxy[s], new Aabb2D(sb.TMinX, sb.TMinY, sb.TMaxX, sb.TMaxY), Dp[2 * b], Dp[2 * b + 1]);
                // The proxy's stored box changed: keep the copy the pair loop reads in step with it.
                ref readonly Aabb2D fat = ref Bp.FatBoundsOfLive(Proxy[s]);
                sb.FMinX = fat.MinX; sb.FMinY = fat.MinY; sb.FMaxX = fat.MaxX; sb.FMaxY = fat.MaxY;
            }
        }
        // 2. Start-of-step pose and zero accumulated motion for every body (sleeping bodies may wake during the stage). Bodies are independent, so this runs in blocks.
        world.Executor.For((nb + 1023) / 1024, new StartPoseTask(this, nb));
#if STAGE_TIMERS
        StageProbe.Mark(StageProbe.Pairs);
#endif
        // 3. New pairs from the broadphase, in its sorted order.
        var found = Bp.UpdatePairs(new PairFilter(this), world.Executor);
        bool anyFresh = false;
        for (int i = 0; i < found.Length; i++)
        {
            int a = found[i].UserA, b = found[i].UserB;
            if (a > b) { (a, b) = (b, a); }
            // The pair filter already dropped every pair with a table entry, and the broadphase reports each pair of shapes once, so none of these is in the table yet.
            if (!PairAllowed(a, b)) { continue; }
            int np = AllocatePair(a, b);
            if (_fresh.Length < PairHigh) { Array.Resize(ref _fresh, Math.Max(PairHigh, _fresh.Length * 2)); }
            // A pair with a sensor is evaluated once when found even if both bodies are asleep, so a sleeping body inside a sensor still gets its enter event.
            _fresh[np] = IsSensor(a) || IsSensor(b);
            anyFresh |= _fresh[np];
        }
#if STAGE_TIMERS
        StageProbe.Mark(StageProbe.Narrowphase);
#endif
        // 4. Pairs that need a fresh manifold: at least one body awake (or a kinematic body in motion).
        int need = (PairHigh + 1023) / 1024 * 1024;
        if (_updateList.Length < need) { _updateList = new int[Math.Max(need, _updateList.Length * 2)]; _results = new PairResult[_updateList.Length]; _actItems = new int[_updateList.Length]; }
        if (_actCounts.Length < need / 64) { _actCounts = new int[Math.Max(need / 64, _actCounts.Length * 2)]; _recycledCounts = new int[_actCounts.Length]; }
        var owner = world.Shapes.Owner;
        // 5. Manifolds, in parallel, in blocks of 64 pair slots: a block picks its pairs that need updating and computes them in the same pass, writing its own part of the lists and results
        // (entry i of the sparse list is slot block * 64 + k for the block's k-th pair to update), so a pair's records are read once. Walking the blocks in order walks the pairs in pair order.
        int chunks = (PairHigh + 63) / 64;
        if (_blockCounts.Length < chunks) { _blockCounts = new int[Math.Max(chunks, _blockCounts.Length * 2)]; }
        _collideBlocks = chunks;
        world.Executor.For(chunks, new CollideTask(this, anyFresh));
        RecycledCount = 0;
        for (int t = 0; t < chunks; t++) { RecycledCount += _recycledCounts[t]; }
        if (anyFresh) { Array.Clear(_fresh, 0, Math.Min(PairHigh, _fresh.Length)); }
#if STAGE_TIMERS
        StageProbe.Mark(StageProbe.Apply);
#endif
        // 6. Apply serially in ascending pair order: this fixes the order constraints are added, which fixes their colors. Writing the manifolds into existing constraints happens afterwards, in parallel.
        var cs = world.Constraints;
        // Only the pairs the collide stage listed need the serial stage (a constraint to add or remove, a pair to drop, a sensor, a sleeping body to wake); the rest are final already.
        for (int t = 0; t < chunks; t++)
        for (int k = 0, an = _actCounts[t]; k < an; k++)
        {
            int i = _actItems[t * 64 + k];
            int p = _updateList[i];
            ref PairResult res = ref _results[i];
            if (res.Status == 2)
            {
                EndPairConstraint(p);
                FreePair(p);
                continue;
            }

            if (res.Count == 0)
            {
                EndPairConstraint(p);
                continue;
            }

            int sa = Pairs[p].A, sb = Pairs[p].B;
            int c = Pairs[p].Cons;
            if (IsSensor(sa) || IsSensor(sb))
            {
                // A sensor overlap exists while the shapes overlap (separation at most 0) and gives enter and exit events. It is a mark on the pair, not a constraint, so a sensor never takes
                // a constraint slot or a color: adding, moving or removing one cannot change how contacts are ordered or solved.
                bool overlap = res.Sep0 <= 0f || (res.Count > 1 && res.Sep1 <= 0f);
                if (overlap && c == -1)
                {
                    Pairs[p].Cons = SensorOverlap;
                    world.EmitSensorEvent(true, sa, sb);
                }
                else if (!overlap && c == SensorOverlap) { EndPairConstraint(p); }

                continue;
            }

            if (c < 0)
            {
                var desc = new ConstraintDesc
                {
                    Kind = ConstraintKind.Contact,
                    BodyA = world.HandleOfBody(owner[sa]),
                    BodyB = world.HandleOfBody(owner[sb]),
                    ShapeA = world.HandleOfShape(sa),
                    ShapeB = world.HandleOfShape(sb),
                    Flags = ConstraintBits.ReportPersist,
                };
                Pairs[p].Cons = world.AddConstraint(desc);
            }
            else
            {
                // A sleeping body that a moving kinematic body touches must wake.
                int ba = owner[sa], bb = owner[sb];
                if (bodies.IsDynamic(ba) && !bodies.IsAwake(ba)) { world.WakeIsland(bodies.Island[ba]); }
                if (bodies.IsDynamic(bb) && !bodies.IsAwake(bb)) { world.WakeIsland(bodies.Island[bb]); }
            }

            if (res.Status != 4) { res.Status = 3; } // 3: a manifold to write into the pair's constraint; 4: a recycled one already written
        }
        if (Opt.MaterialMixer != null) { new WriteTask(this).RunAll(); } // a user mixer runs serially, in pair order
        else { world.Executor.For(_collideBlocks, new WriteTask(this)); }
    }

    private Shape WrapShape(int slot) => new(World!, Core.HandleOfShape(slot));

    private static bool IsActive(BodyStore bodies, int b)
    {
        if (bodies.IsDynamic(b)) { return bodies.IsAwake(b); }
        if (bodies.TypeOf(b) != BodyType.Kinematic) { return false; }
        var lin = bodies.LinearVelocityOf(b);
        return lin[0] != 0f || lin[1] != 0f || bodies.AngularVelocityOf(b)[0] != 0f;
    }

    private readonly struct CollideTask : IParallelBody
    {
        private readonly Engine2D _e; private readonly bool _anyFresh;

        public CollideTask(Engine2D e, bool anyFresh) { _e = e; _anyFresh = anyFresh; }

        public void Run(int task)
        {
            var e = _e;
            int lo = task * 64, hi = Math.Min(lo + 64, e.PairHigh);
            ReadOnlySpan<BodyHot> hot = e.Hot;
            int acts = 0, recycled = 0, listed = 0;
            for (int p = lo; p < hi; p++)
            {
                ref Pair2D pr = ref e.Pairs[p];
                if (!pr.Live) { continue; }
                e.Ready(ref pr);
                int flags = hot[pr.BodyA].Flags | hot[pr.BodyB].Flags;
                // A pair needs a fresh manifold when a body is awake (or a kinematic body is moving), or when it was just found and has a sensor.
                if ((flags & BodyHot.Active) == 0 && !(_anyFresh && e._fresh[p])) { continue; }
                int i = lo + listed++;
                e._updateList[i] = p;
                bool act;
                if (e.Collide(i, ref pr, hot))
                {
                    // Recycled in place: the manifold is already in the constraint. Only a sleeping body that must wake needs the serial stage.
                    recycled++;
                    act = (flags & BodyHot.AsleepDynamic) != 0;
                    // What the serial stage and the write stage read: a manifold already written (an entry left from an earlier step must not be taken for a new one).
                    ref PairResult done = ref e._results[i];
                    done.Status = 4; done.Count = 1;
                }
                else
                {
                    ref PairResult res = ref e._results[i];
                    int c = pr.Cons;
                    if (res.Status == 2) { act = true; }
                    else if (res.Count == 0) { act = c != -1; }
                    else
                    {
                        // A sleeping body in a touching pair must wake; the serial stage looks again, since an earlier pair may have woken it already.
                        act = c < 0 || pr.Sensor || (flags & BodyHot.AsleepDynamic) != 0;
                        if (!pr.Sensor)
                        {
                            if (res.Recycled) { recycled++; }
                            if (res.Status == 1) { res.Status = 3; } // a fresh manifold to write into the pair's constraint (the serial stage sets the same for the pairs it handles)
                        }
                    }
                }

                if (act) { e._actItems[task * 64 + acts++] = i; }
            }

            e._blockCounts[task] = listed;
            e._actCounts[task] = acts;
            e._recycledCounts[task] = recycled;
        }
    }

    public BodyHot[] Hot = new BodyHot[8];

    /// <summary>The material coefficients a pair's payload holds are valid while this equals the pair's epoch; a friction or restitution setter moves it on.</summary>
    public int MaterialEpoch;

    private void Ready(ref Pair2D pr)
    {
        if (pr.BodyA >= 0) { return; }
        var owner = Core.Shapes.Owner;
        pr.BodyA = owner[pr.A]; pr.BodyB = owner[pr.B];
        pr.Sensor = IsSensor(pr.A) || IsSensor(pr.B);
    }

    private int[] _actItems = new int[64];
    private int[] _actCounts = new int[1];
    private int[] _recycledCounts = new int[1];
    private int[][] _moverLists = [];
    private int[] _moverCounts = [];
    private int[] _blockCounts = new int[4];

    private readonly struct StartPoseTask : IParallelBody
    {
        private readonly Engine2D _e; private readonly int _count;

        public StartPoseTask(Engine2D e, int count) { _e = e; _count = count; }

        public void Run(int task)
        {
            var e = _e;
            var bodies = e.Core.Bodies;
            var pos = bodies.Position;
            var rot = bodies.Rotation;
            int lo = task * 1024, hi = Math.Min(lo + 1024, _count);
            for (int b = lo; b < hi; b++)
            {
                if (!bodies.IsLive(b)) { continue; }
                e.StartPos[2 * b] = pos[2 * b]; e.StartPos[2 * b + 1] = pos[2 * b + 1];
                e.StartRot[2 * b] = rot[2 * b]; e.StartRot[2 * b + 1] = rot[2 * b + 1];
                e.Dp[2 * b] = 0f; e.Dp[2 * b + 1] = 0f;
                ref BodyHot h = ref e.Hot[b];
                h.X = pos[2 * b]; h.Y = pos[2 * b + 1]; h.C = rot[2 * b]; h.S = rot[2 * b + 1];
                h.Lcx = e.LocalCenter[2 * b]; h.Lcy = e.LocalCenter[2 * b + 1]; h.Extent = e.MaxExtent[b];
                h.Flags = (IsActive(bodies, b) ? BodyHot.Active : 0) | (bodies.IsDynamic(b) && !bodies.IsAwake(b) ? BodyHot.AsleepDynamic : 0);
            }
        }
    }

    private readonly struct BoxTask : IParallelBody
    {
        private readonly Engine2D _e; private readonly int[] _list; private readonly int _count;

        public BoxTask(Engine2D e, int[] list, int count) { _e = e; _list = list; _count = count; }

        public void Run(int task)
        {
            var e = _e;
            var bodies = e.Core.Bodies;
            int lo = task * 64, hi = Math.Min(lo + 64, _count);
            int[] movers = e._moverLists[task] ?? (e._moverLists[task] = new int[64]);
            int moved = 0;
            for (int i = lo; i < hi; i++)
            {
                int b = _list[i];
                if (!bodies.IsLive(b)) { continue; }
                for (int s = e.ShapeHead[b]; s >= 0; s = e.NextShape[s])
                {
                    var box = e.TightBox(b, in e.Geom[s]);
                    ref ShapeBox sb = ref e._sbox[s];
                    sb.TMinX = box.MinX; sb.TMinY = box.MinY; sb.TMaxX = box.MaxX; sb.TMaxY = box.MaxY;
                    e._boxStamp[s] = e._stamp;
                    ref readonly Aabb2D fat = ref e.Bp.FatBoundsOfLive(e.Proxy[s]); // every shape has a live proxy
                    sb.FMinX = fat.MinX; sb.FMinY = fat.MinY; sb.FMaxX = fat.MaxX; sb.FMaxY = fat.MaxY;
                    bool stays = Aabb2D.Contains(in fat, box);
                    if (!stays)
                    {
                        if (moved == movers.Length) { Array.Resize(ref movers, moved * 2); e._moverLists[task] = movers; }
                        movers[moved++] = s;
                    }
                }
            }

            e._moverCounts[task] = moved;
        }
    }

    private readonly struct WriteTask : IParallelBody
    {
        private readonly Engine2D _e;

        public WriteTask(Engine2D e) => _e = e;

        public void RunAll() { for (int t = 0; t < _e._collideBlocks; t++) { Run(t); } }

        public void Run(int task)
        {
            var e = _e;
            var cs = e.Core.Constraints;
            int lo = task * 64, hi = lo + e._blockCounts[task];
            for (int i = lo; i < hi; i++)
            {
                ref PairResult res = ref e._results[i];
                if (res.Status != 3) { continue; }
                int p = e._updateList[i];
                if (res.Recycled) { e.WriteRecycled(cs, e.Pairs[p].Cons, e.Pairs[p].A, e.Pairs[p].B, in res); }
                else { e.WriteManifold(cs, e.Pairs[p].Cons, e.Pairs[p].A, e.Pairs[p].B, in res); }
            }
        }
    }

    private int[] _boxStamp = new int[8];
    private int _stamp;

    // Per shape, for the shapes of awake bodies this step: the tight box (what the pair loop's gap test uses) and a copy of the proxy's stored box, side by side so the pair loop reads one record
    // instead of walking from the proxy index through the tree to a node. Valid where _boxStamp equals the current stamp.
    private struct ShapeBox
    {
        public double TMinX, TMinY, TMaxX, TMaxY, FMinX, FMinY, FMaxX, FMaxY;
    }

    private ShapeBox[] _sbox = new ShapeBox[8];

    private void EnsureBoxStorage()
    {
        int n = Core.Shapes.HighWater;
        if (_boxStamp.Length < n) { int cap = Math.Max(n, _boxStamp.Length * 2); _sbox = new ShapeBox[cap]; _boxStamp = new int[cap]; }
    }

    // Returns true when the pair's manifold was recycled in place (written straight into its constraint); otherwise the result is in _results[i].
    private bool Collide(int i, ref Pair2D pr, ReadOnlySpan<BodyHot> hot)
    {
        int sa = pr.A, sb = pr.B;
        ref readonly BodyHot hA = ref hot[pr.BodyA];
        ref readonly BodyHot hB = ref hot[pr.BodyB];
        int existing = pr.Cons;
        bool plain = !pr.Sensor && Opt.MaterialMixer == null;
        // A touching pair that has barely moved reuses its manifold. This is tried before the bounds tests: a manifold within the speculative distance
        // means the boxes overlap, so they would not have stopped it, and it saves loading the proxies of the pairs that are still touching.
        if (RecycleManifolds && existing >= 0 && plain && TryRecycleInPlace(existing, ref pr, in hA, in hB)) { return true; }

        ref PairResult res = ref _results[i];
        res = default;
        // Shapes of awake bodies have this step's tight box and a copy of their stored box in one record; any other shape (asleep, static) reads its stored box from the tree.
        const double Gap = Constants2D.SpeculativeDistance + 0.005;
        double ax0, ay0, ax1, ay1, bx0, by0, bx1, by1;
        double fax0, fay0, fax1, fay1, fbx0, fby0, fbx1, fby1;
        if (_boxStamp[sa] == _stamp)
        {
            ref readonly ShapeBox qa = ref _sbox[sa];
            ax0 = qa.TMinX; ay0 = qa.TMinY; ax1 = qa.TMaxX; ay1 = qa.TMaxY;
            fax0 = qa.FMinX; fay0 = qa.FMinY; fax1 = qa.FMaxX; fay1 = qa.FMaxY;
        }
        else
        {
            ref readonly Aabb2D fatA = ref Bp.FatBoundsOfLive(Proxy[sa]);
            fax0 = ax0 = fatA.MinX; fay0 = ay0 = fatA.MinY; fax1 = ax1 = fatA.MaxX; fay1 = ay1 = fatA.MaxY;
        }

        if (_boxStamp[sb] == _stamp)
        {
            ref readonly ShapeBox qb = ref _sbox[sb];
            bx0 = qb.TMinX; by0 = qb.TMinY; bx1 = qb.TMaxX; by1 = qb.TMaxY;
            fbx0 = qb.FMinX; fby0 = qb.FMinY; fbx1 = qb.FMaxX; fby1 = qb.FMaxY;
        }
        else
        {
            ref readonly Aabb2D fatB = ref Bp.FatBoundsOfLive(Proxy[sb]);
            fbx0 = bx0 = fatB.MinX; fby0 = by0 = fatB.MinY; fbx1 = bx1 = fatB.MaxX; fby1 = by1 = fatB.MaxY;
        }

        // The stored boxes (the same test the broadphase uses) no longer overlap: the pair ends.
        if (!(fax0 <= fbx1 && fbx0 <= fax1 && fay0 <= fby1 && fby0 <= fay1)) { res.Status = 2; return false; }
        res.Status = 1;
        // Shapes whose bounds (this step's tight box for a body that moved, the stored box for one that did not) are farther apart than the speculative distance cannot touch: no manifold,
        // without running the narrowphase. The extra margin keeps the shortcut clear of rounding at the threshold.
        if (ax0 - Gap > bx1 || bx0 - Gap > ax1 || ay0 - Gap > by1 || by0 - Gap > ay1) { return false; }

        var qA = new Rot2(hA.C, hA.S); var qB = new Rot2(hB.C, hB.S);
        var lcA = new Vec2(hA.Lcx, hA.Lcy); var lcB = new Vec2(hB.Lcx, hB.Lcy);
        double pAx = hA.X, pAy = hA.Y, pBx = hB.X, pBy = hB.Y;
        // A pair without a manifold whose last narrowphase proved the shapes apart by more than the speculative distance stays empty until they have moved that far toward each other.
        ref SkipFrame skip = ref pr.Skip;
        if (existing < 0 && skip.Clear > 0f && StillApart(in skip, qA, qB, (float)(pBx - pAx), (float)(pBy - pAy))) { return false; }
        if (RecycleManifolds && existing >= 0 && !plain && TryRecycle(existing, sa, sb, hB.Extent, qA, qB, (float)(pBx - pAx), (float)(pBy - pAy), ref res))
        {
            // A recycled manifold only rewrites the pair's own payload, and nothing in the serial stage can remove it, so it is written here, in parallel, instead of going through the result list.
            // A user mixer runs serially in pair order, and a pair with a sensor shape never writes a contact payload.
            if (Opt.MaterialMixer == null && !pr.Sensor) { WriteRecycled(Core.Constraints, existing, sa, sb, in res); res.Status = 4; }
            return false;
        }

        // Subtract the double positions first; only the (small) difference of the body origins becomes float.
        Vec2 wA = qA.Rotate(lcA), wB = qB.Rotate(lcB);
        double dx = (pBx - (double)wB.X) - (pAx - (double)wA.X);
        double dy = (pBy - (double)wB.Y) - (pAy - (double)wA.Y);
        var xf = Transform2.InvMul(new Transform2(default, qA), new Transform2(new Vec2((float)dx, (float)dy), qB));
        var m = Collide2D.Collide(in Geom[sa], in Geom[sb], xf, Constants2D.SpeculativeDistance, out float clearance);
        res.Count = m.PointCount;
        if (m.PointCount == 0)
        {
            if (existing < 0 && clearance > 0f)
            {
                float cA = qA.C, sA = qA.S, cB = qB.C, sB = qB.S;
                float fdx = (float)(pBx - pAx), fdy = (float)(pBy - pAy);
                skip.X = cA * fdx + sA * fdy; skip.Y = cA * fdy - sA * fdx;
                skip.C = cA * cB + sA * sB; skip.S = cA * sB - sA * cB;
                skip.Clear = clearance;
                skip.Reach = ReachOf(in Geom[sb], hB.Lcx, hB.Lcy);
            }
            else { skip.Clear = 0f; }

            return false;
        }

        skip.Clear = 0f;

        Vec2 n = qA.Rotate(m.Normal);
        res.Nx = n.X; res.Ny = n.Y;
        // Contact arms from each centre of mass; B's arm differs from A's by the centre-of-mass offset.
        Vec2 comDiff = new((float)(pAx - pBx), (float)(pAy - pBy));
        for (int k = 0; k < m.PointCount; k++)
        {
            var mp = k == 0 ? m.Point0 : m.Point1;
            Vec2 ra = qA.Rotate(mp.Point - lcA);
            Vec2 rb = ra + comDiff;
            if (k == 0) { res.RAx0 = ra.X; res.RAy0 = ra.Y; res.RBx0 = rb.X; res.RBy0 = rb.Y; res.Sep0 = mp.Separation; res.Id0 = mp.Id; }
            else { res.RAx1 = ra.X; res.RAy1 = ra.Y; res.RBx1 = rb.X; res.RBy1 = rb.Y; res.Sep1 = mp.Separation; res.Id1 = mp.Id; }
        }

        // A pair that already has its constraint writes the fresh manifold right here, while the payload is still in cache from the recycle test: the serial stage can only wake bodies for it,
        // never remove or move it. Pairs without a constraint (and runs with a user mixer, and sensors) write after the serial stage.
        if (existing >= 0 && plain)
        {
            WriteManifold(Core.Constraints, existing, sa, sb, in res, pr.MixEpoch == MaterialEpoch);
            pr.MixEpoch = MaterialEpoch;
            res.Status = 4;
        }

        return false;
    }

    // The farthest point of a shape from the body's centre of mass (given in the body frame), radius included.
    private static float ReachOf(in Shape2D g, float lcx, float lcy)
    {
        var cv = g.Vertices;
        float reach = 0f;
        for (int i = 0; i < cv.Length; i++)
        {
            float ex = cv[i].X - lcx, ey = cv[i].Y - lcy;
            float e = float.Sqrt(ex * ex + ey * ey);
            if (e > reach) { reach = e; }
        }

        return reach + g.Radius;
    }

    // Whether a pair recorded as apart still is. Every point of B's shape moves, relative to A, by at most the move of B's centre of mass plus the chord of the turn times the point's distance from that centre
    // (the turn's cosine is the product of the two stored and current rotations, so no angle is needed). The slack covers float rounding of the relative pose, which grows with its size.
    private static bool StillApart(in SkipFrame f, Rot2 qA, Rot2 qB, float dx, float dy)
    {
        float cA = qA.C, sA = qA.S;
        float ex = cA * dx + sA * dy - f.X, ey = cA * dy - sA * dx - f.Y;
        float dc = (cA * qB.C + sA * qB.S) * f.C + (cA * qB.S - sA * qB.C) * f.S;
        float turn = 2f * (1f - dc);
        float radius = f.Reach;
        float slack = 0.001f + 1e-6f * (float.Abs(dx) + float.Abs(dy) + radius);
        float used = float.Sqrt(ex * ex + ey * ey) + float.Sqrt(turn > 0f ? turn : 0f) * radius + slack;
        return used <= f.Clear; // false for NaN
    }

    // keepMix: the payload still holds the pair's current mixed coefficients (nothing has changed a material since it was written), so they are kept instead of mixed again.
    private void WriteManifold(ConstraintStore cs, int c, int sa, int sb, in PairResult res, bool keepMix = false)
    {
        var pl = cs.Payload(c);
        var ids = cs.FeatureIdsOf(c);
        // Offsets below are all under Header + 2 * PointStride (+ the trailing floats), which the payload layout of the planar world always holds.
        const int Needed = ManifoldLayout.Header + 2 * ManifoldLayout.PointStride;
        if (pl.Length < Needed || ids.Length < 2) { throw new InvalidOperationException("the 2D payload layout is smaller than a two-point manifold"); }
        int oldCount = cs.PointCount[c];
        int count = res.Count;

        // Warm-start impulses of the points that keep their feature id (at most two points on either side).
        float oldN0 = 0f, oldN1 = 0f, oldT0 = 0f, oldT1 = 0f;
        if (oldCount > 0)
        {
            uint id0 = res.Id0, id1 = res.Id1;
            uint o0 = ids[0], o1 = oldCount > 1 ? ids[1] : 0xFFFFFFFFu;
            // Feature ids are never 0xFFFFFFFF (two bytes), so the sentinel cannot match.
            if (o0 == id0) { oldN0 = Mem.At(pl, ManifoldLayout.NormalImpulse(0)); oldT0 = Mem.At(pl, ManifoldLayout.Tangent1Impulse(0)); }
            else if (o1 == id0) { oldN0 = Mem.At(pl, ManifoldLayout.NormalImpulse(1)); oldT0 = Mem.At(pl, ManifoldLayout.Tangent1Impulse(1)); }
            if (count > 1)
            {
                if (o0 == id1) { oldN1 = Mem.At(pl, ManifoldLayout.NormalImpulse(0)); oldT1 = Mem.At(pl, ManifoldLayout.Tangent1Impulse(0)); }
                else if (o1 == id1) { oldN1 = Mem.At(pl, ManifoldLayout.NormalImpulse(1)); oldT1 = Mem.At(pl, ManifoldLayout.Tangent1Impulse(1)); }
            }
        }

        float rolling = oldCount > 0 ? Mem.At(pl, ManifoldLayout.RollingImpulse) : 0f;
        float mixFriction, mixRestitution, mixRolling;
        if (keepMix) { mixFriction = Mem.At(pl, ManifoldLayout.Friction); mixRestitution = Mem.At(pl, ManifoldLayout.Restitution); mixRolling = Mem.At(pl, ManifoldLayout.RollingResistance); }
        else { var mixed = Mix(sa, sb); mixFriction = mixed.Friction; mixRestitution = mixed.Restitution; mixRolling = mixed.RollingResistance; }
        pl.Clear();
        Mem.At(pl, ManifoldLayout.Normal) = res.Nx; Mem.At(pl, ManifoldLayout.Normal + 1) = res.Ny;
        Mem.At(pl, ManifoldLayout.Friction) = mixFriction;
        Mem.At(pl, ManifoldLayout.Restitution) = mixRestitution;
        Mem.At(pl, ManifoldLayout.RollingResistance) = mixRolling;
        Mem.At(pl, ManifoldLayout.RollingImpulse) = rolling;
        for (int k = 0; k < count; k++)
        {
            float rax = k == 0 ? res.RAx0 : res.RAx1, ray = k == 0 ? res.RAy0 : res.RAy1, rbx = k == 0 ? res.RBx0 : res.RBx1, rby = k == 0 ? res.RBy0 : res.RBy1;
            Mem.At(pl, ManifoldLayout.AnchorA(k)) = rax; Mem.At(pl, ManifoldLayout.AnchorA(k) + 1) = ray;
            Mem.At(pl, ManifoldLayout.AnchorB(k)) = rbx; Mem.At(pl, ManifoldLayout.AnchorB(k) + 1) = rby;
            Mem.At(pl, ManifoldLayout.Separation(k)) = k == 0 ? res.Sep0 : res.Sep1;
            Mem.At(pl, ManifoldLayout.NormalImpulse(k)) = k == 0 ? oldN0 : oldN1;
            Mem.At(pl, ManifoldLayout.Tangent1Impulse(k)) = k == 0 ? oldT0 : oldT1;
            ids[k] = k == 0 ? res.Id0 : res.Id1;
        }

        for (int k = count; k < ids.Length; k++) { ids[k] = 0; }
        cs.PointCount[c] = (byte)count;
        StoreRecycleFrame(pl, sa, sb, in res);
    }

    // ------------------------------------------------------------------ manifold recycling

    // Catto's contact recycling (Box2D 3.2 release notes: a touching contact keeps its manifold until the shapes have moved about 5 cm or turned about 11 degrees relative to each other).
    // A manifold is a set of points fixed to the two bodies, so while the bodies move rigidly the points move with them and the separation changes by the motion of the two points along the normal.
    // The narrowphase is skipped for such a pair; the arms, the normal and the separations are rebuilt from the body-fixed copies kept in the payload.
    // Extent-scaled bound on how far any point of B has moved relative to A since the manifold was made (metres), and how deep a recycled point may have sunk.
    // Box2D 3.2 allows 5 cm and about 11 degrees; mixed-terrain stability failed at that, because turning about a contact point sinks other vertices by extent * sin(angle).
    /// <summary>Whether a touching pair that has barely moved reuses its manifold instead of running the narrowphase. On by default; off gives every pair a fresh manifold each step (the result differs slightly, so hashes differ).</summary>
    public bool RecycleManifolds = true;

    /// <summary>How many manifolds the last step reused.</summary>
    public int RecycledCount;

    private const float RecycleTol = 0.01f;
    private const float RoundContactMinRadius = 0.02f;
    private const float RecycleDepth = 0.05f;

    // Payload slots the 2D contact does not use (third components, the second tangent, the trailing floats). Offsets in floats.
    private static class Rc
    {
        public const int NormalX = 2, NormalY = 7;             // normal in the frame of the body that owns it
        public const int Owner = 32;                           // 0 none, 1 shape A, 2 shape B
        public const int RelX = 8, RelY = 11, RelC = 14, RelS = 18; // pose of B's centre of mass and rotation in A's frame when the manifold was made
    }

    // The five floats of point k's body-fixed record: arm A x, y, arm B x, y, separation.
    private static int PointSlot(int k, int f)
    {
        // point 0: 20, 33, 34, 35, 36   point 1: 37, 38, 39, 23, 26
        return k == 0 ? (f == 0 ? 20 : 32 + f) : (f < 3 ? 37 + f : f == 3 ? 23 : 26);
    }

    private void StoreRecycleFrame(Span<float> pl, int sa, int sb, in PairResult res)
    {
        var owner = Core.Shapes.Owner;
        int ba = owner[sa], bb = owner[sb];
        var bodies = Core.Bodies;
        var pos = bodies.Position; var rot = bodies.Rotation;
        float cA = rot[2 * ba], sA = rot[2 * ba + 1], cB = rot[2 * bb], sB = rot[2 * bb + 1];
        float dx = (float)(pos[2 * bb] - pos[2 * ba]), dy = (float)(pos[2 * bb + 1] - pos[2 * ba + 1]);
        // The normal belongs to the body whose face made it (the reference face): it is that face's normal and turns with that body. A manifold whose normal is no face normal
        // (a vertex against a vertex, a disc) has no such body and is never recycled.
        int frame = NormalOwner(sa, sb, cA, sA, cB, sB, res.Nx, res.Ny);
        float radiusA = Geom[sa]._radius, radiusB = Geom[sb]._radius;
        // A single point whose normal is no face normal, between round shapes (discs, capsule ends, rounded corners), is the line between the two cores' closest points: it is
        // stored as those two points, fixed to their bodies, and the normal and separation are rebuilt from them (a disc rolling over a disc keeps its contact without a narrowphase).
        // Shapes with no radius are left out: the line between two bare corners that touch has no direction to rebuild.
        if (frame == 0 && res.Count == 1 && radiusA + radiusB >= RoundContactMinRadius) { frame = 3; }
        Mem.At(pl, Rc.Owner) = frame;
        if (frame == 3)
        {
            float hA = 0.5f * res.Sep0 + radiusA, hB = 0.5f * res.Sep0 + radiusB;
            float coreAx = res.RAx0 - hA * res.Nx, coreAy = res.RAy0 - hA * res.Ny;
            float coreBx = res.RBx0 + hB * res.Nx, coreBy = res.RBy0 + hB * res.Ny;
            Mem.At(pl, Rc.NormalX) = 0f; Mem.At(pl, Rc.NormalY) = 0f;
            Mem.At(pl, PointSlot(0, 0)) = cA * coreAx + sA * coreAy; Mem.At(pl, PointSlot(0, 1)) = cA * coreAy - sA * coreAx;
            Mem.At(pl, PointSlot(0, 2)) = cB * coreBx + sB * coreBy; Mem.At(pl, PointSlot(0, 3)) = cB * coreBy - sB * coreBx;
            Mem.At(pl, PointSlot(0, 4)) = res.Sep0;
            Mem.At(pl, Rc.RelX) = cA * dx + sA * dy; Mem.At(pl, Rc.RelY) = cA * dy - sA * dx;
            Mem.At(pl, Rc.RelC) = cA * cB + sA * sB; Mem.At(pl, Rc.RelS) = cA * sB - sA * cB;
            return;
        }

        if (frame == 2) { Mem.At(pl, Rc.NormalX) = cB * res.Nx + sB * res.Ny; Mem.At(pl, Rc.NormalY) = cB * res.Ny - sB * res.Nx; }
        else { Mem.At(pl, Rc.NormalX) = cA * res.Nx + sA * res.Ny; Mem.At(pl, Rc.NormalY) = cA * res.Ny - sA * res.Nx; }
        Mem.At(pl, Rc.RelX) = cA * dx + sA * dy; Mem.At(pl, Rc.RelY) = cA * dy - sA * dx;
        Mem.At(pl, Rc.RelC) = cA * cB + sA * sB; Mem.At(pl, Rc.RelS) = cA * sB - sA * cB;
        for (int k = 0; k < res.Count; k++)
        {
            float rax = k == 0 ? res.RAx0 : res.RAx1, ray = k == 0 ? res.RAy0 : res.RAy1, rbx = k == 0 ? res.RBx0 : res.RBx1, rby = k == 0 ? res.RBy0 : res.RBy1;
            Mem.At(pl, PointSlot(k, 0)) = cA * rax + sA * ray; Mem.At(pl, PointSlot(k, 1)) = cA * ray - sA * rax;
            Mem.At(pl, PointSlot(k, 2)) = cB * rbx + sB * rby; Mem.At(pl, PointSlot(k, 3)) = cB * rby - sB * rbx;
            Mem.At(pl, PointSlot(k, 4)) = k == 0 ? res.Sep0 : res.Sep1;
        }
    }

    private int NormalOwner(int sa, int sb, float cA, float sA, float cB, float sB, float nx, float ny)
    {
        const float Parallel = 0.99999f;
        ref readonly Shape2D ga = ref Geom[sa];
        if (ga._count >= 2)
        {
            for (int i = 0; i < ga._count; i++)
            {
                Vec2 f = ga._n[i];
                if ((cA * f.X - sA * f.Y) * nx + (sA * f.X + cA * f.Y) * ny > Parallel) { return 1; }
            }
        }

        ref readonly Shape2D gb = ref Geom[sb];
        if (gb._count >= 2)
        {
            for (int i = 0; i < gb._count; i++)
            {
                Vec2 f = gb._n[i];
                if ((cB * f.X - sB * f.Y) * nx + (sB * f.X + cB * f.Y) * ny < -Parallel) { return 2; }
            }
        }

        return 0;
    }

    private bool TryRecycle(int c, int sa, int sb, float extentB, Rot2 qA, Rot2 qB, float dx, float dy, ref PairResult res)
    {
        var cs = Core.Constraints;
        int count = cs.PointCount[c];
        if (count == 0 || cs.KindOf(c) != ConstraintKind.Contact) { return false; }
        var pl = cs.Payload(c);
        // Read as a float and compared: snapshot bytes may hold anything here, and only 1 and 2 mean a stored frame (old snapshots hold 0).
        float ownerF = Mem.At(pl, Rc.Owner);
        int owner = ownerF == 1f ? 1 : ownerF == 2f ? 2 : 0; // a round contact (3) is recycled only in place
        if (owner == 0) { return false; }
        float cA = qA.C, sA = qA.S, cB = qB.C, sB = qB.S;
        // How far B has moved and turned in A's frame since the manifold was made.
        float relX = cA * dx + sA * dy, relY = cA * dy - sA * dx;
        float ex = relX - Mem.At(pl, Rc.RelX), ey = relY - Mem.At(pl, Rc.RelY);
        float e2 = ex * ex + ey * ey;
        float rc = cA * cB + sA * sB, rs = cA * sB - sA * cB;
        float c0 = Mem.At(pl, Rc.RelC), s0 = Mem.At(pl, Rc.RelS);
        float dc = rc * c0 + rs * s0, ds = rs * c0 - rc * s0; // rotation from the made pose to now
        if (!(dc > 0f)) { return false; }
        float lim = RecycleTol - float.Sqrt(e2); // what is left of the budget for the turn
        if (!(lim >= 0f && (ds < 0f ? -ds : ds) * extentB <= lim)) { return false; }

        float nlx = Mem.At(pl, Rc.NormalX), nly = Mem.At(pl, Rc.NormalY);
        float nx, ny;
        if (owner == 2) { nx = cB * nlx - sB * nly; ny = sB * nlx + cB * nly; }
        else { nx = cA * nlx - sA * nly; ny = sA * nlx + cA * nly; }
        for (int k = 0; k < count; k++)
        {
            float ax = Mem.At(pl, PointSlot(k, 0)), ay = Mem.At(pl, PointSlot(k, 1)), bx = Mem.At(pl, PointSlot(k, 2)), by = Mem.At(pl, PointSlot(k, 3));
            float rax = cA * ax - sA * ay, ray = sA * ax + cA * ay;
            float rbx = cB * bx - sB * by, rby = sB * bx + cB * by;
            // Separation now: the one when made plus the motion of B's point relative to A's point along the normal.
            float sep = Mem.At(pl, PointSlot(k, 4)) + (((dx + rbx) - rax) * nx + ((dy + rby) - ray) * ny);
            if (!(sep <= Constants2D.SpeculativeDistance && sep >= -RecycleDepth)) { return false; } // false for NaN too
            if (k == 0) { res.RAx0 = rax; res.RAy0 = ray; res.RBx0 = rbx; res.RBy0 = rby; res.Sep0 = sep; }
            else { res.RAx1 = rax; res.RAy1 = ray; res.RBx1 = rbx; res.RBy1 = rby; res.Sep1 = sep; }
        }

        res.Nx = nx; res.Ny = ny; res.Count = count; res.Status = 1; res.Recycled = true;
        return true;
    }

    // The recycle test and the write of TryRecycle and WriteRecycled for a plain contact (no sensor, no user mixer), straight from the body records into the constraint's payload.
    // Same arithmetic in the same order, so the result is the same bits; nothing is written unless every point passes.
    private bool TryRecycleInPlace(int c, ref Pair2D pr, in BodyHot hA, in BodyHot hB)
    {
        var cs = Core.Constraints;
        int count = cs.PointCount[c];
        if (count == 0 || count > 2 || cs.KindOf(c) != ConstraintKind.Contact) { return false; }
        var pl = cs.Payload(c);
        float ownerF = Mem.At(pl, Rc.Owner);
        int owner = ownerF == 1f ? 1 : ownerF == 2f ? 2 : ownerF == 3f ? 3 : 0;
        if (owner == 0 || (owner == 3 && count != 1)) { return false; }
        float cA = hA.C, sA = hA.S, cB = hB.C, sB = hB.S;
        float dx = (float)(hB.X - hA.X), dy = (float)(hB.Y - hA.Y);
        float relX = cA * dx + sA * dy, relY = cA * dy - sA * dx;
        float ex = relX - Mem.At(pl, Rc.RelX), ey = relY - Mem.At(pl, Rc.RelY);
        float e2 = ex * ex + ey * ey;
        float rc = cA * cB + sA * sB, rs = cA * sB - sA * cB;
        float c0 = Mem.At(pl, Rc.RelC), s0 = Mem.At(pl, Rc.RelS);
        float dc = rc * c0 + rs * s0, ds = rs * c0 - rc * s0;
        if (!(dc > 0f)) { return false; }
        float lim = RecycleTol - float.Sqrt(e2);
        if (!(lim >= 0f && (ds < 0f ? -ds : ds) * hB.Extent <= lim)) { return false; }

        if (owner == 3) { return RecycleRound(pl, ref pr, in hA, in hB, dx, dy); }

        float nlx = Mem.At(pl, Rc.NormalX), nly = Mem.At(pl, Rc.NormalY);
        float nx, ny;
        if (owner == 2) { nx = cB * nlx - sB * nly; ny = sB * nlx + cB * nly; }
        else { nx = cA * nlx - sA * nly; ny = sA * nlx + cA * nly; }
        float rax0 = 0f, ray0 = 0f, rbx0 = 0f, rby0 = 0f, sep0 = 0f, rax1 = 0f, ray1 = 0f, rbx1 = 0f, rby1 = 0f, sep1 = 0f;
        for (int k = 0; k < count; k++)
        {
            float ax = Mem.At(pl, PointSlot(k, 0)), ay = Mem.At(pl, PointSlot(k, 1)), bx = Mem.At(pl, PointSlot(k, 2)), by = Mem.At(pl, PointSlot(k, 3));
            float rax = cA * ax - sA * ay, ray = sA * ax + cA * ay;
            float rbx = cB * bx - sB * by, rby = sB * bx + cB * by;
            float sep = Mem.At(pl, PointSlot(k, 4)) + (((dx + rbx) - rax) * nx + ((dy + rby) - ray) * ny);
            if (!(sep <= Constants2D.SpeculativeDistance && sep >= -RecycleDepth)) { return false; }
            if (k == 0) { rax0 = rax; ray0 = ray; rbx0 = rbx; rby0 = rby; sep0 = sep; }
            else { rax1 = rax; ray1 = ray; rbx1 = rbx; rby1 = rby; sep1 = sep; }
        }

        // A material can change between steps (a friction or restitution setter moves the epoch), and a fresh manifold would pick that up, so a recycled one does too.
        if (pr.MixEpoch != MaterialEpoch)
        {
            var mixed = Mix(pr.A, pr.B);
            Mem.At(pl, ManifoldLayout.Friction) = mixed.Friction;
            Mem.At(pl, ManifoldLayout.Restitution) = mixed.Restitution;
            Mem.At(pl, ManifoldLayout.RollingResistance) = mixed.RollingResistance;
            pr.MixEpoch = MaterialEpoch;
        }

        Mem.At(pl, ManifoldLayout.Normal) = nx; Mem.At(pl, ManifoldLayout.Normal + 1) = ny;
        Mem.At(pl, ManifoldLayout.AnchorA(0)) = rax0; Mem.At(pl, ManifoldLayout.AnchorA(0) + 1) = ray0;
        Mem.At(pl, ManifoldLayout.AnchorB(0)) = rbx0; Mem.At(pl, ManifoldLayout.AnchorB(0) + 1) = rby0;
        Mem.At(pl, ManifoldLayout.Separation(0)) = sep0;
        if (count > 1)
        {
            Mem.At(pl, ManifoldLayout.AnchorA(1)) = rax1; Mem.At(pl, ManifoldLayout.AnchorA(1) + 1) = ray1;
            Mem.At(pl, ManifoldLayout.AnchorB(1)) = rbx1; Mem.At(pl, ManifoldLayout.AnchorB(1) + 1) = rby1;
            Mem.At(pl, ManifoldLayout.Separation(1)) = sep1;
        }

        return true;
    }

    // The recycle of a round contact: the stored closest points of the two cores, carried by their bodies, give the new normal and separation exactly.
    private bool RecycleRound(Span<float> pl, ref Pair2D pr, in BodyHot hA, in BodyHot hB, float dx, float dy)
    {
        float cA = hA.C, sA = hA.S, cB = hB.C, sB = hB.S;
        float ax = Mem.At(pl, PointSlot(0, 0)), ay = Mem.At(pl, PointSlot(0, 1)), bx = Mem.At(pl, PointSlot(0, 2)), by = Mem.At(pl, PointSlot(0, 3));
        float coreAx = cA * ax - sA * ay, coreAy = sA * ax + cA * ay;   // closest point on A's core, from A's centre of mass
        float coreBx = cB * bx - sB * by, coreBy = sB * bx + cB * by;   // closest point on B's core, from B's centre of mass
        float ddx = (dx + coreBx) - coreAx, ddy = (dy + coreBy) - coreAy;
        float radiusA = Geom[pr.A]._radius, radiusB = Geom[pr.B]._radius, radii = radiusA + radiusB;
        float len = float.Sqrt(ddx * ddx + ddy * ddy);
        if (!(len >= 0.5f * radii)) { return false; } // the cores are nearly on top of each other: the direction between them is not reliable, make a fresh manifold (false for NaN too)
        float inv = 1f / len;
        float nx = ddx * inv, ny = ddy * inv;
        float sep = len - radii;
        if (!(sep <= Constants2D.SpeculativeDistance && sep >= -RecycleDepth)) { return false; }
        float hAr = 0.5f * sep + radiusA, hBr = 0.5f * sep + radiusB;
        float rax = coreAx + hAr * nx, ray = coreAy + hAr * ny;
        float rbx = coreBx - hBr * nx, rby = coreBy - hBr * ny;
        if (pr.MixEpoch != MaterialEpoch)
        {
            var mixed = Mix(pr.A, pr.B);
            Mem.At(pl, ManifoldLayout.Friction) = mixed.Friction;
            Mem.At(pl, ManifoldLayout.Restitution) = mixed.Restitution;
            Mem.At(pl, ManifoldLayout.RollingResistance) = mixed.RollingResistance;
            pr.MixEpoch = MaterialEpoch;
        }

        Mem.At(pl, ManifoldLayout.Normal) = nx; Mem.At(pl, ManifoldLayout.Normal + 1) = ny;
        Mem.At(pl, ManifoldLayout.AnchorA(0)) = rax; Mem.At(pl, ManifoldLayout.AnchorA(0) + 1) = ray;
        Mem.At(pl, ManifoldLayout.AnchorB(0)) = rbx; Mem.At(pl, ManifoldLayout.AnchorB(0) + 1) = rby;
        Mem.At(pl, ManifoldLayout.Separation(0)) = sep;
        return true;
    }

    private void WriteRecycled(ConstraintStore cs, int c, int sa, int sb, in PairResult res)
    {
        var pl = cs.Payload(c);
        // A material can change between steps (a body's friction setter), and a fresh manifold would pick that up, so a recycled one does too.
        var mixed = Mix(sa, sb);
        Mem.At(pl, ManifoldLayout.Friction) = mixed.Friction;
        Mem.At(pl, ManifoldLayout.Restitution) = mixed.Restitution;
        Mem.At(pl, ManifoldLayout.RollingResistance) = mixed.RollingResistance;
        Mem.At(pl, ManifoldLayout.Normal) = res.Nx; Mem.At(pl, ManifoldLayout.Normal + 1) = res.Ny;
        for (int k = 0; k < res.Count; k++)
        {
            Mem.At(pl, ManifoldLayout.AnchorA(k)) = k == 0 ? res.RAx0 : res.RAx1; Mem.At(pl, ManifoldLayout.AnchorA(k) + 1) = k == 0 ? res.RAy0 : res.RAy1;
            Mem.At(pl, ManifoldLayout.AnchorB(k)) = k == 0 ? res.RBx0 : res.RBx1; Mem.At(pl, ManifoldLayout.AnchorB(k) + 1) = k == 0 ? res.RBy0 : res.RBy1;
            Mem.At(pl, ManifoldLayout.Separation(k)) = k == 0 ? res.Sep0 : res.Sep1;
        }
    }

    private Material2D Mix(int sa, int sb)
    {
        var mixer = Opt.MaterialMixer;
        if (mixer != null)
        {
            return mixer(new Material2D(Friction[sa], Restitution[sa], Rolling[sa]), new Material2D(Friction[sb], Restitution[sb], Rolling[sb]));
        }

        // Friction: geometric mean (a slippery surface stays slippery); restitution and rolling resistance: the larger.
        float f = float.Sqrt(Friction[sa] * Friction[sb]);
        float r = Restitution[sa] > Restitution[sb] ? Restitution[sa] : Restitution[sb];
        float rr = Rolling[sa] > Rolling[sb] ? Rolling[sa] : Rolling[sb];
        return new Material2D(f, r, rr);
    }
}
