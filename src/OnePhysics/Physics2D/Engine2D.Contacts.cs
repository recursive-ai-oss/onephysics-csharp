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
            var owner = _e.Core.Shapes.Owner;
            return owner[shapeA] != owner[shapeB] && _e.CanCollide(shapeA, shapeB) && !_e.JointsPreventCollision(owner[shapeA], owner[shapeB]);
        }
    }

    private struct PairResult
    {
        public byte Status; // 0 untouched, 1 manifold computed, 2 boxes no longer overlap
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
        world.Executor.For((awake.Length + 63) / 64, new BoxTask(this, world.AwakeArray, awake.Length));
        foreach (int b in awake)
        {
            if (!bodies.IsLive(b)) { continue; }
            for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
            {
                if (_needMove[s] == 0) { continue; }
                Bp.MoveProxy(Proxy[s], new Aabb2D(_boxes[4 * s], _boxes[4 * s + 1], _boxes[4 * s + 2], _boxes[4 * s + 3]), Dp[2 * b], Dp[2 * b + 1]);
            }
        }

        // 2. Start-of-step pose and zero accumulated motion for every body (sleeping bodies may wake during the stage).
        var pos = bodies.Position;
        var rot = bodies.Rotation;
        for (int b = 0; b < nb; b++)
        {
            if (!bodies.IsLive(b)) { continue; }
            StartPos[2 * b] = pos[2 * b]; StartPos[2 * b + 1] = pos[2 * b + 1];
            StartRot[2 * b] = rot[2 * b]; StartRot[2 * b + 1] = rot[2 * b + 1];
            Dp[2 * b] = 0f; Dp[2 * b + 1] = 0f;
        }

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
        if (_updateList.Length < PairHigh) { _updateList = new int[Math.Max(PairHigh, _updateList.Length * 2)]; _results = new PairResult[_updateList.Length]; }
        _updateCount = 0;
        var owner = world.Shapes.Owner;
        for (int p = 0; p < PairHigh; p++)
        {
            if (!PairLive[p]) { continue; }
            if (IsActive(bodies, owner[PairA[p]]) || IsActive(bodies, owner[PairB[p]]) || (anyFresh && _fresh[p])) { _updateList[_updateCount++] = p; }
        }

        if (anyFresh) { Array.Clear(_fresh, 0, Math.Min(PairHigh, _fresh.Length)); }

        // 5. Manifolds, in parallel; each task writes only its own results.
        int chunks = (_updateCount + 63) / 64;
        world.Executor.For(chunks, new CollideTask(this));

        // 6. Apply serially in ascending pair order: this fixes the order constraints are added, which fixes their colors. Writing the manifolds into existing constraints happens afterwards, in parallel.
        var cs = world.Constraints;
        for (int i = 0; i < _updateCount; i++)
        {
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

            res.Status = 3; // a manifold to write into the pair's constraint
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
            for (int i = lo; i < hi; i++) { e.Collide(i, pos, rot, owner); }
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
            for (int i = lo; i < hi; i++)
            {
                int b = _list[i];
                if (!bodies.IsLive(b)) { continue; }
                for (int s = e.ShapeHead[b]; s >= 0; s = e.NextShape[s])
                {
                    var box = e.TightBox(b, in e.Geom[s]);
                    e._boxes[4 * s] = box.MinX; e._boxes[4 * s + 1] = box.MinY; e._boxes[4 * s + 2] = box.MaxX; e._boxes[4 * s + 3] = box.MaxY;
                    e._boxStamp[s] = e._stamp;
                    e._needMove[s] = (byte)(Aabb2D.Contains(e.Bp.GetFatBounds(e.Proxy[s]), box) ? 0 : 1);
                }
            }
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
                e.WriteManifold(cs, e.PairCons[p], e.PairA[p], e.PairB[p], in res);
            }
        }
    }

    private byte[] _needMove = new byte[8];
    private int[] _boxStamp = new int[8];
    private int _stamp;
    private double[] _boxes = new double[32];

    private void EnsureBoxStorage()
    {
        int n = Core.Shapes.HighWater;
        if (_needMove.Length < n) { _needMove = new byte[Math.Max(n, _needMove.Length * 2)]; _boxes = new double[_needMove.Length * 4]; _boxStamp = new int[_needMove.Length]; }
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
