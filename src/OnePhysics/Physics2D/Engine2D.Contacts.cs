using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Broadphase;
using RecursiveAI.OnePhysics.Collision2D;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>The step stages of the 2D engine. A struct so the core's generic step specializes on it; all state lives in the engine object.</summary>
internal readonly struct Stages2D : IWorldStages
{
    private readonly Engine2D _e;

    public Stages2D(Engine2D e) => _e = e;

    public void UpdateContacts(WorldCore world) => _e.UpdateContacts(world);

    public void BeginSolve(WorldCore world, int batches, in StepContext context) => _e.BeginSolve(batches);

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

    public void FinalizeBodies(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context) => _e.FinalizeBodies(bodies, in context);

    public void AfterFinalize(WorldCore world, in StepContext context) => _e.RunBullets();
}

internal sealed partial class Engine2D
{
    // ------------------------------------------------------------------ pair filter for the broadphase

    private readonly struct PairFilter : IPairFilter
    {
        private readonly Engine2D _e;

        public PairFilter(Engine2D e) => _e = e;

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
    private int _updateCount;

    // ------------------------------------------------------------------ contact stage

    public void UpdateContacts(WorldCore world)
    {
        var bodies = world.Bodies;
        int nb = bodies.HighWater;
        EnsureBodies(nb);
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
                Bp.MoveProxy(Proxy[s], new Aabb2D(_boxes[4 * s], _boxes[4 * s + 1], _boxes[4 * s + 2], _boxes[4 * s + 3]), Dp[2 * b], Dp[2 * b + 1]);
            }
        }
        // 2. Start-of-step pose and zero accumulated motion for every body (sleeping bodies may wake during the stage). Bodies are independent, so this runs in blocks.
        world.Executor.For((nb + 1023) / 1024, new StartPoseTask(this, nb));
        // 3. New pairs from the broadphase, in its sorted order.
        var found = Bp.UpdatePairs(new PairFilter(this), world.Executor);
        bool anyFresh = false;
        for (int i = 0; i < found.Length; i++)
        {
            int a = found[i].UserA, b = found[i].UserB;
            if (a > b) { (a, b) = (b, a); }
            if (PairMap.ContainsKey(PairKey(a, b))) { continue; }
            if (!PairAllowed(a, b)) { continue; }
            int np = AllocatePair(a, b);
            if (_fresh.Length < PairHigh) { Array.Resize(ref _fresh, Math.Max(PairHigh, _fresh.Length * 2)); }
            // A pair with a sensor is evaluated once when found even if both bodies are asleep, so a sleeping body inside a sensor still gets its enter event.
            _fresh[np] = IsSensor(a) || IsSensor(b);
            anyFresh |= _fresh[np];
        }
        // 4. Pairs that need a fresh manifold: at least one body awake (or a kinematic body in motion).
        int need = (PairHigh + 1023) / 1024 * 1024;
        if (_updateList.Length < need) { _updateList = new int[Math.Max(need, _updateList.Length * 2)]; _results = new PairResult[_updateList.Length]; _actItems = new int[_updateList.Length]; }
        if (_actCounts.Length < need / 64) { _actCounts = new int[Math.Max(need / 64, _actCounts.Length * 2)]; _recycledCounts = new int[_actCounts.Length]; }
        var owner = world.Shapes.Owner;
        int listBlocks = (PairHigh + 1023) / 1024;
        if (_blockCounts.Length < listBlocks) { _blockCounts = new int[Math.Max(listBlocks, _blockCounts.Length * 2)]; }
        world.Executor.For(listBlocks, new UpdateListTask(this, world, anyFresh));
        // Each block wrote its pairs at the start of its own 1024 slots, in pair order; closing the gaps in block order gives the list a single serial loop would have built.
        _updateCount = 0;
        RecycledCount = 0;
        for (int k = 0; k < listBlocks; k++)
        {
            int n = _blockCounts[k];
            if (n > 0 && _updateCount != k * 1024) { Array.Copy(_updateList, k * 1024, _updateList, _updateCount, n); }
            _updateCount += n;
        }

        if (anyFresh) { Array.Clear(_fresh, 0, Math.Min(PairHigh, _fresh.Length)); }
        // 5. Manifolds, in parallel; each task writes only its own results.
        int chunks = (_updateCount + 63) / 64;
        world.Executor.For(chunks, new CollideTask(this));
        for (int t = 0; t < chunks; t++) { RecycledCount += _recycledCounts[t]; }
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
                if (PairCons[p] >= 0) { world.RemoveConstraint(PairCons[p]); }
                FreePair(p);
                continue;
            }

            if (res.Count == 0)
            {
                if (PairCons[p] >= 0) { world.RemoveConstraint(PairCons[p]); PairCons[p] = -1; }
                continue;
            }

            int sa = PairA[p], sb = PairB[p];
            int c = PairCons[p];
            if (IsSensor(sa) || IsSensor(sb))
            {
                // A sensor overlap is a constraint that is never solved: it exists while the shapes overlap (separation at most 0) and gives enter and exit events.
                bool overlap = res.Sep0 <= 0f || (res.Count > 1 && res.Sep1 <= 0f);
                if (overlap && c < 0)
                {
                    PairCons[p] = world.AddConstraint(new ConstraintDesc { Kind = ConstraintKind.Sensor, BodyA = world.HandleOfBody(owner[sa]), BodyB = world.HandleOfBody(owner[sb]), ShapeA = world.HandleOfShape(sa), ShapeB = world.HandleOfShape(sb) });
                }
                else if (!overlap && c >= 0)
                {
                    world.RemoveConstraint(c);
                    PairCons[p] = -1;
                }

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
                PairCons[p] = world.AddConstraint(desc);
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
        else { world.Executor.For((_updateCount + 63) / 64, new WriteTask(this)); }
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
        private readonly Engine2D _e;

        public CollideTask(Engine2D e) => _e = e;

        public void Run(int task)
        {
            var e = _e;
            int lo = task * 64, hi = Math.Min(lo + 64, e._updateCount);
            ReadOnlySpan<double> pos = e.Core.Bodies.Position;
            ReadOnlySpan<float> rot = e.Core.Bodies.Rotation;
            ReadOnlySpan<int> owner = e.Core.Shapes.Owner;
            var bodies = e.Core.Bodies;
            int acts = 0, recycled = 0;
            for (int i = lo; i < hi; i++)
            {
                e.Collide(i, pos, rot, owner);
                ref PairResult res = ref e._results[i];
                int p = e._updateList[i];
                int c = e.PairCons[p];
                bool act;
                if (res.Status == 2) { act = true; }
                else if (res.Count == 0) { act = c >= 0; }
                else
                {
                    int sa = e.PairA[p], sb = e.PairB[p];
                    int ba = owner[sa], bb = owner[sb];
                    // A sleeping body in a touching pair must wake; the serial stage looks again, since an earlier pair may have woken it already.
                    bool sensor = e.IsSensor(sa) || e.IsSensor(sb);
                    act = c < 0 || sensor || (bodies.IsDynamic(ba) && !bodies.IsAwake(ba)) || (bodies.IsDynamic(bb) && !bodies.IsAwake(bb));
                    if (!sensor)
                    {
                        if (res.Recycled) { recycled++; }
                        if (res.Status == 1) { res.Status = 3; } // a fresh manifold to write into the pair's constraint (the serial stage sets the same for the pairs it handles)
                    }
                }

                if (act) { e._actItems[task * 64 + acts++] = i; }
            }

            e._actCounts[task] = acts;
            e._recycledCounts[task] = recycled;
        }
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
            }
        }
    }

    private readonly struct UpdateListTask : IParallelBody
    {
        private readonly Engine2D _e; private readonly WorldCore _w; private readonly bool _anyFresh;

        public UpdateListTask(Engine2D e, WorldCore w, bool anyFresh) { _e = e; _w = w; _anyFresh = anyFresh; }

        public void Run(int block)
        {
            var e = _e;
            var bodies = _w.Bodies;
            var owner = _w.Shapes.Owner;
            int lo = block * 1024, hi = Math.Min(lo + 1024, e.PairHigh);
            int n = 0;
            for (int p = lo; p < hi; p++)
            {
                if (!e.PairLive[p]) { continue; }
                if (IsActive(bodies, owner[e.PairA[p]]) || IsActive(bodies, owner[e.PairB[p]]) || (_anyFresh && e._fresh[p])) { e._updateList[lo + n++] = p; }
            }

            e._blockCounts[block] = n;
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
                    e._boxes[4 * s] = box.MinX; e._boxes[4 * s + 1] = box.MinY; e._boxes[4 * s + 2] = box.MaxX; e._boxes[4 * s + 3] = box.MaxY;
                    e._boxStamp[s] = e._stamp;
                    bool stays = Aabb2D.Contains(e.Bp.GetFatBounds(e.Proxy[s]), box);
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

        public void RunAll() { for (int t = 0; t < (_e._updateCount + 63) / 64; t++) { Run(t); } }

        public void Run(int task)
        {
            var e = _e;
            var cs = e.Core.Constraints;
            int lo = task * 64, hi = Math.Min(lo + 64, e._updateCount);
            for (int i = lo; i < hi; i++)
            {
                ref PairResult res = ref e._results[i];
                if (res.Status != 3) { continue; }
                int p = e._updateList[i];
                if (res.Recycled) { e.WriteRecycled(cs, e.PairCons[p], e.PairA[p], e.PairB[p], in res); }
                else { e.WriteManifold(cs, e.PairCons[p], e.PairA[p], e.PairB[p], in res); }
            }
        }
    }

    private int[] _boxStamp = new int[8];
    private int _stamp;
    private double[] _boxes = new double[32];

    private void EnsureBoxStorage()
    {
        int n = Core.Shapes.HighWater;
        if (_boxStamp.Length < n) { int cap = Math.Max(n, _boxStamp.Length * 2); _boxes = new double[cap * 4]; _boxStamp = new int[cap]; }
    }

    private void Collide(int i, ReadOnlySpan<double> pos, ReadOnlySpan<float> rot, ReadOnlySpan<int> owner)
    {
        int p = _updateList[i];
        ref PairResult res = ref _results[i];
        res = default;
        int sa = PairA[p], sb = PairB[p];
        ref readonly Aabb2D fatA = ref Bp.FatBoundsOfLive(Proxy[sa]);
        ref readonly Aabb2D fatB = ref Bp.FatBoundsOfLive(Proxy[sb]);
        if (!Aabb2D.Overlaps(in fatA, in fatB)) { res.Status = 2; return; }
        res.Status = 1;

        // Shapes whose bounds (this step's tight box for a body that moved, the stored box for one that did not) are farther apart than the speculative distance cannot touch: no manifold,
        // without running the narrowphase. The extra margin keeps the shortcut clear of rounding at the threshold.
        const double Gap = Constants2D.SpeculativeDistance + 0.005;
        double ax0, ay0, ax1, ay1, bx0, by0, bx1, by1;
        if (_boxStamp[sa] == _stamp) { ax0 = _boxes[4 * sa]; ay0 = _boxes[4 * sa + 1]; ax1 = _boxes[4 * sa + 2]; ay1 = _boxes[4 * sa + 3]; }
        else { ax0 = fatA.MinX; ay0 = fatA.MinY; ax1 = fatA.MaxX; ay1 = fatA.MaxY; }
        if (_boxStamp[sb] == _stamp) { bx0 = _boxes[4 * sb]; by0 = _boxes[4 * sb + 1]; bx1 = _boxes[4 * sb + 2]; by1 = _boxes[4 * sb + 3]; }
        else { bx0 = fatB.MinX; by0 = fatB.MinY; bx1 = fatB.MaxX; by1 = fatB.MaxY; }
        if (ax0 - Gap > bx1 || bx0 - Gap > ax1 || ay0 - Gap > by1 || by0 - Gap > ay1) { return; }

        int ba = owner[sa], bb = owner[sb];
        var qA = new Rot2(rot[2 * ba], rot[2 * ba + 1]); var qB = new Rot2(rot[2 * bb], rot[2 * bb + 1]);
        var lcA = new Vec2(LocalCenter[2 * ba], LocalCenter[2 * ba + 1]); var lcB = new Vec2(LocalCenter[2 * bb], LocalCenter[2 * bb + 1]);
        double pAx = pos[2 * ba], pAy = pos[2 * ba + 1], pBx = pos[2 * bb], pBy = pos[2 * bb + 1];
        int existing = PairCons[p];
        if (RecycleManifolds && existing >= 0 && TryRecycle(existing, sa, sb, bb, qA, qB, (float)(pBx - pAx), (float)(pBy - pAy), ref res))
        {
            // A recycled manifold only rewrites the pair's own payload, and nothing in the serial stage can remove it, so it is written here, in parallel, instead of going through the result list.
            // A user mixer runs serially in pair order, and a pair with a sensor shape never writes a contact payload.
            if (Opt.MaterialMixer == null && !IsSensor(sa) && !IsSensor(sb)) { WriteRecycled(Core.Constraints, existing, sa, sb, in res); res.Status = 4; }
            return;
        }

        // Subtract the double positions first; only the (small) difference of the body origins becomes float.
        Vec2 wA = qA.Rotate(lcA), wB = qB.Rotate(lcB);
        double dx = (pBx - (double)wB.X) - (pAx - (double)wA.X);
        double dy = (pBy - (double)wB.Y) - (pAy - (double)wA.Y);
        var xf = Transform2.InvMul(new Transform2(default, qA), new Transform2(new Vec2((float)dx, (float)dy), qB));
        var m = Collide2D.Collide(in Geom[sa], in Geom[sb], xf, Constants2D.SpeculativeDistance);
        res.Count = m.PointCount;
        if (m.PointCount == 0) { return; }

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
    }

    private void WriteManifold(ConstraintStore cs, int c, int sa, int sb, in PairResult res)
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
        pl.Clear();
        var mixed = Mix(sa, sb);
        Mem.At(pl, ManifoldLayout.Normal) = res.Nx; Mem.At(pl, ManifoldLayout.Normal + 1) = res.Ny;
        Mem.At(pl, ManifoldLayout.Friction) = mixed.Friction;
        Mem.At(pl, ManifoldLayout.Restitution) = mixed.Restitution;
        Mem.At(pl, ManifoldLayout.RollingResistance) = mixed.RollingResistance;
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

    private const float RecycleTol = 0.003f;
    private const float RecycleDepth = 0.02f;

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
        Mem.At(pl, Rc.Owner) = frame;
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

    private bool TryRecycle(int c, int sa, int sb, int ba, Rot2 qA, Rot2 qB, float dx, float dy, ref PairResult res)
    {
        var cs = Core.Constraints;
        int count = cs.PointCount[c];
        if (count == 0 || cs.KindOf(c) != ConstraintKind.Contact) { return false; }
        var pl = cs.Payload(c);
        // Read as a float and compared: snapshot bytes may hold anything here, and only 1 and 2 mean a stored frame (old snapshots hold 0).
        float ownerF = Mem.At(pl, Rc.Owner);
        int owner = ownerF == 1f ? 1 : ownerF == 2f ? 2 : 0;
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
        if (!(lim >= 0f && (ds < 0f ? -ds : ds) * MaxExtent[ba] <= lim)) { return false; }

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
