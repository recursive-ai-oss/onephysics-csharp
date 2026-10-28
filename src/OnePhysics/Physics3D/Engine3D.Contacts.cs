using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Broadphase;
using RecursiveAI.OnePhysics.Collision3D;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>The step stages of the 3D engine. A struct so the core's generic step specializes on it; all state lives in the engine object.</summary>
internal readonly struct Stages3D : IWorldStages
{
    private readonly Engine3D _e;

    public Stages3D(Engine3D e) => _e = e;

    public void UpdateContacts(WorldCore world) => _e.UpdateContacts(world);

    public void Prepare(WorldCore world, ReadOnlySpan<int> constraints, in StepContext context) {
        // Contacts are prepared per batch (PrepareBatch); nothing else needs chunk-wise preparation yet.
    }

    public void IntegrateVelocities(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context) => _e.IntegrateVelocities(bodies, in context);

    public void SolveBatch(WorldCore world, ReadOnlySpan<int> constraints, SolvePhase phase, in StepContext context) => _e.SolveBatch(constraints, phase, in context);

    public void IntegratePositions(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context) => _e.IntegratePositions(bodies, in context);

    public void StoreResults(WorldCore world, ReadOnlySpan<int> constraints, int eventSlot, in StepContext context)
    {
        // Contact impulses go from the wide groups back to the payloads; joints work out their reaction and may ask to break.
        _e.StoreResults(constraints);
        _e.StoreJointResults(constraints, in context);
        _e.EmitHits(world, constraints, eventSlot);
    }

    public void BeginSolve(WorldCore world, int batches, in StepContext context) => _e.BeginSolve(batches, in context);

    public void PrepareBatch(WorldCore world, ReadOnlySpan<int> constraints, int batch, in StepContext context) => _e.PrepareBatch(constraints, batch, in context);

    public float BodyMotion(WorldCore world, int body) => _e.BodyMotion(body);

    public void FinalizeBodies(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context) => _e.FinalizeBodies(bodies, in context);

    // Spelled out because calling a default interface member on a struct boxes it: that was 48 bytes allocated by every step.
    public void AfterFinalize(WorldCore world, in StepContext context) { }

    public bool BodyMotionIsConcurrent => false;
}

[InlineArray(28)]
internal struct PointBuffer
{
    private float _e;
}

[InlineArray(12)]
internal struct ImpulseBuffer
{
    private float _e;
}

[InlineArray(4)]
internal struct IdBuffer
{
    private uint _e;
}

/// <summary>One contact (one sub-shape pair) produced by the narrowphase: the normal in world axes and, per point, the arms from both centres of mass, the separation and the feature id.</summary>
internal struct ContactOut
{
    public int SubA, SubB, Count;
    /// <summary>The constraint that carries this contact once the pair has been applied, and the pair's shape slots; the payload is written afterwards, in parallel.</summary>
    public int Cons, ShapeA, ShapeB;
    /// <summary>What the polyhedron test wants to keep for the pair's next step (see <see cref="EdgeHint3D"/>), written to the tail of the payload.</summary>
    public EdgeHint3D Hint;
    /// <summary>The manifold is the pair's previous one moved with the bodies (see TryRecycle), not a new narrowphase result.</summary>
    public bool Recycled;
    public float Nx, Ny, Nz;
    public PointBuffer P; // per point: rA (3), rB (3), separation
    public IdBuffer Id;
}

/// <summary>Working storage of one chunk of narrowphase work. Results never depend on which thread runs a chunk or on what an instance did before.</summary>
internal sealed class NarrowScratch
{
    // The generator and its buffers are large (hundreds of kilobytes) and hold nothing between calls, so one set per thread serves every chunk that thread runs; a set per chunk cost 300 MB
    // on a scene with thousands of pairs. Only the results (Out) belong to the chunk.
    private sealed class Heavy
    {
        public readonly ManifoldGenerator3D Gen = new();
        public readonly List<ChildContact3D> Children = new();
        public readonly List<SoupTriangle3D> Tris = new();
        public readonly List<int> Ids = new();
        public readonly TriangleShapeBuffer3D TriBuf = new();
    }

    [ThreadStatic]
    private static Heavy? t_heavy;

    private static Heavy H => t_heavy ??= new Heavy();

    public static ManifoldGenerator3D Gen => H.Gen;
    public static List<ChildContact3D> Children => H.Children;
    public static List<SoupTriangle3D> Tris => H.Tris;
    public static List<int> Ids => H.Ids;
    public static TriangleShapeBuffer3D TriBuf => H.TriBuf;
    public ContactOut[] Out = new ContactOut[128];
    public int OutCount;

    public ref ContactOut Next()
    {
        if (OutCount == Out.Length) { Array.Resize(ref Out, Out.Length * 2); }
        return ref Out[OutCount++];
    }
}

internal sealed partial class Engine3D
{
    // ------------------------------------------------------------------ pair filter for the broadphase

    private readonly struct PairFilter : IPairFilter
    {
        private readonly Engine3D _e;

        public PairFilter(Engine3D e) => _e = e;

        public bool IsSensor(int shape) => _e.IsSensor(shape);

        public bool ShouldCollide(int shapeA, int shapeB)
        {
            var owner = _e.Core.Shapes.Owner;
            return owner[shapeA] != owner[shapeB] && _e.CanCollide(shapeA, shapeB) && !_e.JointsPreventCollision(owner[shapeA], owner[shapeB]);
        }
    }

    private byte[] _active = new byte[8]; // per body: awake and dynamic, or a kinematic body in motion (see IsActive); valid for the contact stage that filled it
    private int[] _boxStamp = new int[8];
    private int _stamp;
    private double[] _boxes = new double[48];
    private int[] _updateList = new int[64];
    private byte[] _resStatus = new byte[64]; // 0 untouched, 1 manifolds computed, 2 boxes no longer overlap
    private int[] _resFirst = new int[64];
    private int[] _resCount = new int[64];
    private int _updateCount;
    private NarrowScratch[] _scratch = Array.Empty<NarrowScratch>();

    // ------------------------------------------------------------------ contact stage

    public void UpdateContacts(WorldCore world)
    {
        var bodies = world.Bodies;
        int nb = bodies.HighWater;
        EnsureBodies(nb);
        ReconcileJoints();

        // 0. A changed filter re-evaluates the pair table before anything else (see Refilter).
        Refilter();

        // The tight box of every shape of a moving body is kept for this step: a pair whose shapes are farther apart than the speculative distance has no contact points, and two box comparisons say so
        // without loading either shape.
        int shapeHigh = world.Shapes.HighWater;
        if (_boxStamp.Length < shapeHigh) { int cap = Math.Max(shapeHigh, _boxStamp.Length * 2); _boxes = new double[cap * 6]; _boxStamp = new int[cap]; }
        _stamp++;
        // 1. Broadphase boxes of every body that moved last step. The displacement stretches the box ahead of the motion.
        // The tight boxes are pure functions of body state and are computed in parallel over fixed chunks of the awake list; the tree is then updated serially in list order.
        var awakeList = world.AwakeBodies;
        int boxChunks = (awakeList.Length + BoxChunk - 1) / BoxChunk;
        world.Executor.For(boxChunks, new TightBoxTask(this, boxChunks));
        foreach (int b in awakeList)
        {
            if (!bodies.IsLive(b)) { continue; }
            for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
            {
                int o = 6 * s;
                var tight = new Aabb3D(_boxes[o], _boxes[o + 1], _boxes[o + 2], _boxes[o + 3], _boxes[o + 4], _boxes[o + 5]);
                Bp.MoveProxy(Proxy[s], tight, Dp[3 * b], Dp[3 * b + 1], Dp[3 * b + 2]);
            }
        }

        // 2. Start-of-step pose, zero accumulated motion and world-axes inverse inertia for every body (sleeping bodies may wake during the stage).
        var rot = bodies.Rotation;
        var invI = bodies.InverseInertia;
        var invMass = bodies.InverseMass;
        if (_sleepers.Length < nb) { _sleepers = new int[Math.Max(nb, _sleepers.Length * 2)]; }
        _sleeperCount = 0;
        if (_active.Length < nb) { _active = new byte[Math.Max(nb, _active.Length * 2)]; }
        if (_isSleeper.Length < _active.Length) { _isSleeper = new byte[_active.Length]; }
        // Each body's start pose, zeroed motion and packed row depend on that body alone, so fixed chunks of bodies run in parallel; the sleepers are then listed in body order.
        _initCount = nb;
        world.Executor.For((nb + BodyInitChunk - 1) / BodyInitChunk, new BodyInitTask(this));
        for (int b = 0; b < nb; b++)
        {
            if (_isSleeper[b] != 0) { _sleepers[_sleeperCount++] = b; }
        }

#if STAGE_TIMERS
        StageProbe.Mark(StageProbe.Pairs);
#endif
        // 3. New pairs from the broadphase, in its sorted order.
        var found = Bp.UpdatePairs(new PairFilter(this), world.Executor);
        bool anyFresh = false;
        // A burst of new pairs (a stack waking, a pile landing) would otherwise rehash the table again and again.
        if (found.Length > 64) { PairMap.EnsureCapacity(PairMap.Count + found.Length); }
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

        // Most pairs end up with one constraint; making room for them (and half as many again) at once saves the repeated doubling when thousands of pairs appear in one step.
        world.Constraints.Reserve(PairHigh + (PairHigh >> 1));
        if (_skip.Length < PairHigh) { Array.Resize(ref _skip, Math.Max(PairHigh, _skip.Length * 2)); }
#if STAGE_TIMERS
        StageProbe.Mark(StageProbe.Narrowphase);
#endif
        // 4. Pairs that need fresh manifolds: at least one body awake (or a kinematic body in motion). Soup-against-soup pairs never collide.
        if (_updateList.Length < PairHigh)
        {
            int n = Math.Max(PairHigh, _updateList.Length * 2);
            _updateList = new int[n]; _resStatus = new byte[n]; _resFirst = new int[n]; _resCount = new int[n];
        }

        var owner = world.Shapes.Owner;
        // With no awake dynamic body and no kinematic body, only a freshly found sensor pair can need work.
        // Fixed chunks of the pair table each list their own pairs in place, then the chunks are packed in order, so the list is the same whatever the thread count.
        _updateCount = 0;
        if (world.AwakeBodies.Length > 0 || anyFresh)
        {
            int pchunks = (PairHigh + PairListChunk - 1) / PairListChunk;
            if (_pairChunkCount.Length < pchunks) { _pairChunkCount = new int[Math.Max(pchunks, _pairChunkCount.Length * 2)]; }
            world.Executor.For(pchunks, new PairListTask(this, anyFresh));
            for (int ck = 0; ck < pchunks; ck++)
            {
                int n = _pairChunkCount[ck];
                if (_updateCount != ck * PairListChunk) { Array.Copy(_updateList, ck * PairListChunk, _updateList, _updateCount, n); }
                _updateCount += n;
            }
        }

        if (anyFresh) { Array.Clear(_fresh, 0, Math.Min(PairHigh, _fresh.Length)); }

        // 5. Manifolds, in parallel over fixed chunks; each chunk writes only its own results.
        int chunks = (_updateCount + 63) / 64;
        if (_scratch.Length < chunks)
        {
            int old = _scratch.Length;
            Array.Resize(ref _scratch, Math.Max(chunks, old * 2));
            for (int i = old; i < _scratch.Length; i++) { _scratch[i] = new NarrowScratch(); }
        }

        if (_applyCount.Length < chunks) { _applyCount = new int[Math.Max(chunks, _applyCount.Length * 2)]; }
        if (_applyIdx.Length < chunks * 64) { _applyIdx = new int[Math.Max(chunks * 64, _applyIdx.Length * 2)]; }
        for (int i = 0; i < chunks; i++) { _scratch[i].OutCount = 0; }
        world.Executor.For(chunks, new CollideTask(this));

        // The serial pass only adds, matches and removes constraints (their order fixes the colors); writing the manifolds into the payloads is independent per constraint and runs afterwards in parallel.
        // A user material mixer is called in pair order on this thread instead.
        _deferWrite = Opt.MaterialMixer == null && _updateCount >= 64;
#if STAGE_TIMERS
        StageProbe.Mark(StageProbe.Apply);
#endif
        // 6. Apply serially in ascending pair order: this fixes the order constraints are added, which fixes their colors.
        var cs = world.Constraints;
        // Only the pairs the narrowphase tasks listed need the serial pass (a box pair that is still apart and has no constraint has nothing to add, match or remove); they are visited in ascending pair order.
        for (int ck = 0; ck < chunks; ck++)
        {
            int firstSlot = ck * 64, listed = _applyCount[ck];
            for (int j = 0; j < listed; j++)
            {
                int i = _applyIdx[firstSlot + j];
                int p = _updateList[i];
                if (_resStatus[i] == 2)
                {
                    EndSensorOverlap(p);
                    for (int c = PairCons[p]; c >= 0;)
                    {
                        int nx = ConsNext[c];
                        world.RemoveConstraint(c);
                        c = nx;
                    }

                    FreePair(p);
                    continue;
                }

                if (IsSensor(PairA[p]) || IsSensor(PairB[p])) { ApplySensorPair(world, p, _scratch[ck], _resFirst[i], _resCount[i], owner); continue; }
                ApplyPair(world, p, _scratch[ck], _resFirst[i], _resCount[i], owner);
            }
        }

        if (_deferWrite) { world.Executor.For(chunks, new WriteTask(this, cs)); }
        // Bodies that were asleep when the stage began and have been woken by it start the step like any other.
        for (int i = 0; i < _sleeperCount; i++)
        {
            int b = _sleepers[i];
            if (!bodies.IsAwake(b)) { continue; }
            RefreshWorldInertia(b, invMass, invI, rot);
            PackBody(b);
        }

        int ch = cs.HighWater;
        Grow(ref ConsNext, ch);
        SizeJointStorage();
    }

    private void ApplyPair(WorldCore world, int p, NarrowScratch sc, int first, int count, ReadOnlySpan<int> owner)
    {
        int sa = PairA[p], sb = PairB[p];
        var bodies = world.Bodies;
        int ba = owner[sa], bb = owner[sb];
        int old = PairCons[p], head = -1, tail = -1, j = 0;
        while (old >= 0 || j < count)
        {
            var cs = world.Constraints;
            bool haveOld = old >= 0, haveNew = j < count;
            int cmp;
            if (haveOld && haveNew)
            {
                ref ContactOut n = ref sc.Out[first + j];
                int oa = cs.SubShapeA[old], ob = cs.SubShapeB[old];
                cmp = oa != n.SubA ? (oa < n.SubA ? -1 : 1) : ob != n.SubB ? (ob < n.SubB ? -1 : 1) : 0;
            }
            else { cmp = haveOld ? -1 : 1; }

            if (cmp < 0)
            {
                int next = ConsNext[old];
                world.RemoveConstraint(old);
                old = next;
                continue;
            }

            int c;
            if (cmp > 0)
            {
                ref ContactOut n = ref sc.Out[first + j];
                var desc = new ConstraintDesc
                {
                    Kind = ConstraintKind.Contact,
                    BodyA = world.HandleOfBody(ba),
                    BodyB = world.HandleOfBody(bb),
                    ShapeA = world.HandleOfShape(sa),
                    ShapeB = world.HandleOfShape(sb),
                    SubShapeA = n.SubA,
                    SubShapeB = n.SubB,
                    Flags = ConstraintBits.ReportPersist,
                };
                c = world.AddConstraint(desc);
                Grow(ref ConsNext, c + 1);
            }
            else
            {
                c = old;
                old = ConsNext[c];
                // A sleeping body that a moving kinematic body touches must wake.
                if (bodies.IsDynamic(ba) && !bodies.IsAwake(ba)) { world.WakeIsland(bodies.Island[ba]); }
                if (bodies.IsDynamic(bb) && !bodies.IsAwake(bb)) { world.WakeIsland(bodies.Island[bb]); }
            }

            if (_deferWrite) { ref ContactOut o = ref sc.Out[first + j]; o.Cons = c; o.ShapeA = sa; o.ShapeB = sb; }
            else { WriteManifold(world.Constraints, c, sa, sb, in sc.Out[first + j]); }
            j++;
            ConsNext[c] = -1;
            if (tail >= 0) { ConsNext[tail] = c; } else { head = c; }
            tail = c;
        }

        PairCons[p] = head;
    }

    private Shape WrapShape(int slot) => new(World!, Core.HandleOfShape(slot));

    private static bool IsActive(BodyStore bodies, int b)
    {
        if (bodies.IsDynamic(b)) { return bodies.IsAwake(b); }
        if (bodies.TypeOf(b) != BodyType.Kinematic) { return false; }
        var lin = bodies.LinearVelocityOf(b);
        var ang = bodies.AngularVelocityOf(b);
        return lin[0] != 0f || lin[1] != 0f || lin[2] != 0f || ang[0] != 0f || ang[1] != 0f || ang[2] != 0f;
    }

    private bool _deferWrite;
    private int[] _sleepers = new int[64];
    private int _sleeperCount;

    private const int BodyInitChunk = 128;
    private byte[] _isSleeper = new byte[8];
    private int _initCount;

    private readonly struct BodyInitTask : IParallelBody
    {
        private readonly Engine3D _e;

        public BodyInitTask(Engine3D e) => _e = e;

        public void Run(int task)
        {
            var e = _e;
            var bodies = e.Core.Bodies;
            var pos = bodies.Position;
            var rot = bodies.Rotation;
            var invI = bodies.InverseInertia;
            var invMass = bodies.InverseMass;
            int lo = task * BodyInitChunk, hi = Math.Min(lo + BodyInitChunk, e._initCount);
            for (int b = lo; b < hi; b++)
            {
                if (!bodies.IsLive(b)) { e._active[b] = 0; e._isSleeper[b] = 0; continue; }
                e._active[b] = IsActive(bodies, b) ? (byte)1 : (byte)0;
                e.StartPos[3 * b] = pos[3 * b]; e.StartPos[3 * b + 1] = pos[3 * b + 1]; e.StartPos[3 * b + 2] = pos[3 * b + 2];
                for (int k = 0; k < 4; k++) { e.StartRot[4 * b + k] = rot[4 * b + k]; }
                e.Dp[3 * b] = 0f; e.Dp[3 * b + 1] = 0f; e.Dp[3 * b + 2] = 0f;
                e.Dq[4 * b] = 0f; e.Dq[4 * b + 1] = 0f; e.Dq[4 * b + 2] = 0f; e.Dq[4 * b + 3] = 1f;
                // A sleeping body's rotation has not changed since its inertia was last refreshed, so refreshing again would give the same bits; if the stage wakes it, it is refreshed below.
                if (bodies.IsDynamic(b) && !bodies.IsAwake(b)) { e._isSleeper[b] = 1; continue; }
                e._isSleeper[b] = 0;
                e.RefreshWorldInertia(b, invMass, invI, rot);
                e.PackBody(b);
            }
        }
    }

    private const int BoxChunk = 32;

    private readonly struct TightBoxTask : IParallelBody
    {
        private readonly Engine3D _e;

        public TightBoxTask(Engine3D e, int chunks) { _e = e; }

        public void Run(int task)
        {
            var e = _e;
            var list = e.Core.AwakeBodies;
            var bodies = e.Core.Bodies;
            int lo = task * BoxChunk, hi = Math.Min(lo + BoxChunk, list.Length);
            for (int i = lo; i < hi; i++)
            {
                int b = list[i];
                if (!bodies.IsLive(b)) { continue; }
                for (int s = e.ShapeHead[b]; s >= 0; s = e.NextShape[s])
                {
                    var tight = e.TightBox(b, s);
                    int o = 6 * s;
                    e._boxes[o] = tight.MinX; e._boxes[o + 1] = tight.MinY; e._boxes[o + 2] = tight.MinZ; e._boxes[o + 3] = tight.MaxX; e._boxes[o + 4] = tight.MaxY; e._boxes[o + 5] = tight.MaxZ;
                    e._boxStamp[s] = e._stamp;
                }
            }
        }
    }

    private int[] _applyIdx = new int[64];
    private int[] _applyCount = new int[1];

    private const int PairListChunk = 1024;
    private int[] _pairChunkCount = new int[8];

    private readonly struct PairListTask : IParallelBody
    {
        private readonly Engine3D _e;
        private readonly bool _anyFresh;

        public PairListTask(Engine3D e, bool anyFresh) { _e = e; _anyFresh = anyFresh; }

        public void Run(int task)
        {
            var e = _e;
            int lo = task * PairListChunk, hi = Math.Min(lo + PairListChunk, e.PairHigh), n = 0;
            var owner = e.Core.Shapes.Owner;
            var active = e._active;
            var fresh = e._fresh;
            for (int p = lo; p < hi; p++)
            {
                if (!e.PairLive[p]) { continue; }
                if (active[owner[e.PairA[p]]] != 0 || active[owner[e.PairB[p]]] != 0 || (_anyFresh && p < fresh.Length && fresh[p])) { e._updateList[lo + n++] = p; }
            }

            e._pairChunkCount[task] = n;
        }
    }

    private readonly struct WriteTask : IParallelBody
    {
        private readonly Engine3D _e;
        private readonly ConstraintStore _cs;

        public WriteTask(Engine3D e, ConstraintStore cs) { _e = e; _cs = cs; }

        public void Run(int task)
        {
            int lo = task * 64, hi = Math.Min(lo + 64, _e._updateCount);
            var sc = _e._scratch[task];
            for (int i = lo; i < hi; i++)
            {
                if (_e._resStatus[i] != 1) { continue; }
                int p = _e._updateList[i];
                if (_e.IsSensor(_e.PairA[p]) || _e.IsSensor(_e.PairB[p])) { continue; }
                int first = _e._resFirst[i];
                for (int j = 0; j < _e._resCount[i]; j++)
                {
                    ref ContactOut o = ref sc.Out[first + j];
                    _e.WriteManifold(_cs, o.Cons, o.ShapeA, o.ShapeB, in o);
                }
            }
        }
    }

    private readonly struct CollideTask : IParallelBody
    {
        private readonly Engine3D _e;

        public CollideTask(Engine3D e) => _e = e;

        public void Run(int task)
        {
            int lo = task * 64, hi = Math.Min(lo + 64, _e._updateCount);
            var sc = _e._scratch[task];
            var e = _e;
            int listed = 0;
            for (int i = lo; i < hi; i++)
            {
                e.Collide(i, sc);
                // Worth the serial pass: the boxes no longer overlap (the pair goes), or the pair has points or a constraint to match.
                if (e._resStatus[i] == 2 || e._resCount[i] != 0 || e.PairCons[e._updateList[i]] != -1) { e._applyIdx[lo + listed++] = i; }
            }

            e._applyCount[task] = listed;
        }
    }

    // ------------------------------------------------------------------ narrowphase

    /// <summary>The placement of a pair in world terms: A's shape orientation, the arm from A's centre of mass to A's shape origin, and the centre-of-mass offset between the bodies.</summary>
    private struct PairFrame
    {
        public QuaternionF Rwa;
        public Vector3F ArmA;
        public Vector3F ComDiff;
    }

    private void Collide(int i, NarrowScratch sc)
    {
        int p = _updateList[i];
        _resStatus[i] = 0; _resCount[i] = 0; _resFirst[i] = sc.OutCount;
        int sa = PairA[p], sb = PairB[p];
        ref readonly Aabb3D fa = ref Bp.FatBoundsOfLive(Proxy[sa]);
        ref readonly Aabb3D fb = ref Bp.FatBoundsOfLive(Proxy[sb]);
        if (!Aabb3D.Overlaps(fa, fb)) { _resStatus[i] = 2; return; }
        _resStatus[i] = 1;
        // Tight boxes (this step's for a shape that moved, the stored fat box for one that did not); the extra margin keeps the shortcut clear of rounding at the threshold.
        {
            const double Gap = 0.005;
            double gap = Opt.SpeculativeDistance + Gap;
            double a0, a1, a2, a3, a4, a5, b0, b1, b2, b3, b4, b5;
            if (_boxStamp[sa] == _stamp) { int o = 6 * sa; a0 = _boxes[o]; a1 = _boxes[o + 1]; a2 = _boxes[o + 2]; a3 = _boxes[o + 3]; a4 = _boxes[o + 4]; a5 = _boxes[o + 5]; }
            else { a0 = fa.MinX; a1 = fa.MinY; a2 = fa.MinZ; a3 = fa.MaxX; a4 = fa.MaxY; a5 = fa.MaxZ; }
            if (_boxStamp[sb] == _stamp) { int o = 6 * sb; b0 = _boxes[o]; b1 = _boxes[o + 1]; b2 = _boxes[o + 2]; b3 = _boxes[o + 3]; b4 = _boxes[o + 4]; b5 = _boxes[o + 5]; }
            else { b0 = fb.MinX; b1 = fb.MinY; b2 = fb.MinZ; b3 = fb.MaxX; b4 = fb.MaxY; b5 = fb.MaxZ; }
            if (a0 - gap > b3 || b0 - gap > a3 || a1 - gap > b4 || b1 - gap > a4 || a2 - gap > b5 || b2 - gap > a5) { return; }
        }

        var ga = Geoms[GeomOf[sa]]!;
        var gb = Geoms[GeomOf[sb]]!;
        if (ga.Soup != null && gb.Soup != null) { return; }
        var owner = Core.Shapes.Owner;
        int ba = owner[sa], bb = owner[sb];
        var bodies = Core.Bodies;
        var pA = bodies.PositionOf(ba); var pB = bodies.PositionOf(bb);
        var qA = RotationOfBody(ba); var qB = RotationOfBody(bb);
        // A pair without a constraint whose last query proved the shapes apart stays empty until their relative motion has used up the margin by which they were apart.
        int existingCons = PairCons[p];
        if (existingCons < 0 && _skip[p].Clear > 0f && StillApart(in _skip[p], qA, qB, (float)(pB[0] - pA[0]), (float)(pB[1] - pA[1]), (float)(pB[2] - pA[2]), MaxExtent[bb])) { return; }
        // A touching convex pair that has barely moved reuses its manifold; this is tried before the shape frames are built, which the reuse does not need.
        if (RecycleManifolds && existingCons >= 0 && ga.Soup == null && gb.Soup == null && ga.IsConvex && gb.IsConvex
            && TryRecycle(sc, existingCons, ba, bb, qA, qB, new Vector3F((float)(pB[0] - pA[0]), (float)(pB[1] - pA[1]), (float)(pB[2] - pA[2]))))
        {
            _resCount[i] = 1;
            return;
        }

        var la = Local[sa]; var lb = Local[sb];
        var lcA = LocalCenterOf(ba); var lcB = LocalCenterOf(bb);

        // Shape frames in world axes. Positions are subtracted in double first; only the (small) difference becomes float.
        Vector3F armA = qA.Rotate(la.Translation - lcA), armB = qB.Rotate(lb.Translation - lcB);
        QuaternionF rwa = qA * la.Rotation, rwb = qB * lb.Rotation;
        double dx = (pB[0] + (double)armB.X) - (pA[0] + (double)armA.X);
        double dy = (pB[1] + (double)armB.Y) - (pA[1] + (double)armA.Y);
        double dz = (pB[2] + (double)armB.Z) - (pA[2] + (double)armA.Z);
        var inv = rwa.Conjugate();
        var bInA = new RigidTransformF(inv * rwb, inv.Rotate(new Vector3F((float)dx, (float)dy, (float)dz)));
        var f = new PairFrame { Rwa = rwa, ArmA = armA, ComDiff = new Vector3F((float)(pA[0] - pB[0]), (float)(pA[1] - pB[1]), (float)(pA[2] - pB[2])) };
        float spec = Opt.SpeculativeDistance;

        if (ga.Soup != null || gb.Soup != null)
        {
            bool soupIsA = ga.Soup != null;
            var other = soupIsA ? gb : ga;
            // Temporal coherence for a convex shape on a triangle soup: while it has hardly moved relative to the soup, every triangle contact of the pair keeps its manifold (the pair's triangle set can only
            // change by a triangle coming within speculative range, which the next full query, at most a few millimetres of motion later, finds).
            if (RecycleManifolds && other.IsConvex && PairCons[p] >= 0 && TryRecycleSoup(sc, p, ba, bb, qA, qB, new Vector3F((float)(pB[0] - pA[0]), (float)(pB[1] - pA[1]), (float)(pB[2] - pA[2])), soupIsA ? 1 : 2))
            {
                _resCount[i] = sc.OutCount - _resFirst[i];
                return;
            }

            if (soupIsA) { SoupAgainst(sc, in f, ga.Soup!, true, gb, in lb, in bInA, spec); }
            else { SoupAgainst(sc, in f, gb.Soup!, false, ga, in la, in bInA, spec); }
        }
        else if (ga.IsConvex && gb.IsConvex)
        {
            // Temporal coherence: the hint from the pair's contact of the last step, if it has one, goes in; what the test learned this time goes out with the new manifold.
            var gen = NarrowScratch.Gen;
            int oldCons = PairCons[p];
            var cstore = Core.Constraints;
            gen.HintIn = oldCons >= 0 && cstore.PointCount[oldCons] > 0 ? new EdgeHint3D(cstore.Payload(oldCons).Slice(ManifoldLayout.HintOffset, 7)) : default;
            var m = gen.Collide(ga.Convex, gb.Convex, bInA, spec);
            if (m.PointCount == 0 && oldCons < 0 && gen.ClearanceOut > 0f)
            {
                // The pose of B's body in A's body at this query, and how far apart the shapes were.
                var invA = qA.Conjugate();
                var d = invA.Rotate(new Vector3F((float)(pB[0] - pA[0]), (float)(pB[1] - pA[1]), (float)(pB[2] - pA[2])));
                var qr = invA * qB;
                float sg = qr.W < 0f ? -1f : 1f;
                _skip[p] = new SkipFrame3 { X = d.X, Y = d.Y, Z = d.Z, Qx = sg * qr.X, Qy = sg * qr.Y, Qz = sg * qr.Z, Qw = sg * qr.W, Clear = gen.ClearanceOut };
            }
            else { _skip[p].Clear = 0f; }

            int before = sc.OutCount;
            Emit(sc, in f, 0, 0, in m, default, false, default, false, default);
            if (sc.OutCount > before) { sc.Out[before].Hint = gen.HintOut; }
        }
        else
        {
            NarrowScratch.Children.Clear();
            if (ga.Compound != null && gb.IsConvex) { NarrowScratch.Gen.CollideCompoundConvex(ga.Compound, gb.Convex, bInA, NarrowScratch.Children, spec); }
            else if (ga.IsConvex && gb.Compound != null) { NarrowScratch.Gen.CollideConvexCompound(ga.Convex, gb.Compound, bInA, NarrowScratch.Children, spec); }
            else { NarrowScratch.Gen.CollideCompoundCompound(ga.Compound!, gb.Compound!, bInA, NarrowScratch.Children, spec); }
            foreach (var cc in NarrowScratch.Children)
            {
                var m = cc.Manifold;
                Emit(sc, in f, cc.SubShapeA, cc.SubShapeB, in m, default, false, default, false, default);
            }
        }

        _resCount[i] = sc.OutCount - _resFirst[i];
    }

    /// <summary>The pose of B's body in A's body when a pair without a constraint last had no contact points, and how far apart the shapes were then (beyond the speculative distance).</summary>
    private struct SkipFrame3
    {
        public float X, Y, Z, Qx, Qy, Qz, Qw, Clear;
    }

    private SkipFrame3[] _skip = new SkipFrame3[8];

    // Whether a pair recorded as apart still is. Every point of B's shape moves, relative to A, by at most the change of B's centre of mass in A's frame plus the chord of the relative turn times the point's distance
    // from that centre (chord = 2 sin(angle / 2) = twice the vector part of the relative rotation). The slack covers float rounding of the relative pose, which grows with its size.
    private static bool StillApart(in SkipFrame3 f, QuaternionF qA, QuaternionF qB, float dx, float dy, float dz, float extent)
    {
        var invA = qA.Conjugate();
        var d = invA.Rotate(new Vector3F(dx, dy, dz));
        float ex = d.X - f.X, ey = d.Y - f.Y, ez = d.Z - f.Z;
        var qr = invA * qB;
        float sg = qr.W < 0f ? -1f : 1f;
        float x = sg * qr.X, y = sg * qr.Y, z = sg * qr.Z, w = sg * qr.W;
        // Vector part of qr * conj(q0), with q0 = (Qx, Qy, Qz, Qw) the stored relative rotation (Qw >= 0).
        float rx = w * -f.Qx + f.Qw * x + (y * -f.Qz - z * -f.Qy);
        float ry = w * -f.Qy + f.Qw * y + (z * -f.Qx - x * -f.Qz);
        float rz = w * -f.Qz + f.Qw * z + (x * -f.Qy - y * -f.Qx);
        float turn = 2f * float.Sqrt(rx * rx + ry * ry + rz * rz);
        float slack = 0.001f + 1e-6f * (float.Abs(dx) + float.Abs(dy) + float.Abs(dz) + extent);
        float used = float.Sqrt(ex * ex + ey * ey + ez * ez) + turn * extent + slack;
        return used <= f.Clear; // false for NaN
    }

    /// <summary>Contacts of a triangle soup (mesh or height field) with a convex shape or a compound. <paramref name="soupIsA"/> says which side of the pair the soup is.</summary>
    private static void SoupAgainst(NarrowScratch sc, in PairFrame f, TriangleSoup3D soup, bool soupIsA, Geom3D other, in RigidTransformF otherLocal, in RigidTransformF bInA, float spec)
    {
        // The other shape's frame in the soup's frame.
        RigidTransformF otherInSoup = soupIsA ? bInA : bInA.Inverse();
        RigidTransformF soupFromA = soupIsA ? RigidTransformF.Identity : otherInSoup; // points in A's frame into the soup's frame (soup is B: A in B)
        int first = sc.OutCount;
        if (other.IsConvex)
        {
            SoupVsConvex(sc, in f, soup, soupIsA, other.Convex, default, false, otherInSoup, in soupFromA, spec, -1);
            return;
        }

        var comp = other.Compound!;
        for (int j = 0; j < comp.SlotCount; j++)
        {
            if (!comp.IsLive(j)) { continue; }
            var child = comp.GetChild(j);
            SoupVsConvex(sc, in f, soup, soupIsA, child.Shape, child.Local, true, otherInSoup * child.Local, in soupFromA, spec, j);
        }

        // Keys must be ascending for the pair's constraint chain: (sub-shape A, sub-shape B). With the soup as A the loops above ran child first.
        for (int a = first + 1; a < sc.OutCount; a++)
        {
            var t = sc.Out[a];
            int k = a - 1;
            while (k >= first && (sc.Out[k].SubA > t.SubA || (sc.Out[k].SubA == t.SubA && sc.Out[k].SubB > t.SubB))) { sc.Out[k + 1] = sc.Out[k]; k--; }
            sc.Out[k + 1] = t;
        }
    }

    private static void SoupVsConvex(NarrowScratch sc, in PairFrame f, TriangleSoup3D soup, bool soupIsA, in ConvexShape3D shape, in RigidTransformF shapeLocal, bool hasChild,
        in RigidTransformF shapeInSoup, in RigidTransformF soupFromA, float spec, int child)
    {
        NarrowScratch.Tris.Clear();
        soup.GetCandidates(shape, shapeInSoup, spec, NarrowScratch.Tris, NarrowScratch.Ids);
        // For a child of a compound the manifold comes back in the child's frame; mapToA moves it into the compound's frame (only when the compound is shape A).
        for (int t = 0; t < NarrowScratch.Tris.Count; t++)
        {
            var tri = NarrowScratch.Tris[t];
            var tshape = NarrowScratch.TriBuf.Set(tri.V0, tri.V1, tri.V2);
            Manifold3D m;
            int subA, subB;
            bool map = hasChild && !soupIsA;
            if (soupIsA)
            {
                m = NarrowScratch.Gen.Collide(tshape, shape, shapeInSoup, spec);
                subA = tri.Id; subB = child < 0 ? 0 : child;
            }
            else
            {
                m = NarrowScratch.Gen.Collide(shape, tshape, shapeInSoup.Inverse(), spec);
                subA = child < 0 ? 0 : child; subB = tri.Id;
            }

            if (m.PointCount == 0) { continue; }
            Emit(sc, in f, subA, subB, in m, shapeLocal, map, soupFromA, true, tri, soupIsA);
        }
    }

    private static void Emit(NarrowScratch sc, in PairFrame f, int subA, int subB, in Manifold3D m, in RigidTransformF mapToA, bool map, in RigidTransformF soupFromA, bool filter, in SoupTriangle3D tri,
        bool soupIsA = false)
    {
        if (m.PointCount == 0) { return; }
        Vector3F n = map ? mapToA.Rotation.Rotate(m.Normal) : m.Normal;
        int count = m.PointCount;
        if (filter)
        {
            // Internal-edge rule (Rouwé's write-ups on internal edges, Gregorius on contact generation): a point whose closest triangle feature is an inactive edge, or a vertex whose
            // two edges in this triangle are both inactive, lies on a seam between flat neighbours. If the manifold's normal also leans away from the face normal, the neighbour's own
            // face contact is the right one, and this one would kick a body sliding over the seam: the manifold is dropped. Contacts on the back of a triangle are not filtered.
            Vector3F triN = tri.Normal;
            Vector3F towards = soupIsA ? n : soupFromA.Rotation.Rotate(-n); // from the triangle towards the other shape, in the soup's frame
            float d = Vector3F.Dot(towards, triN);
            if (d >= -0.1f && d < GhostCosine)
            {
                for (int k = 0; k < count; k++)
                {
                    Vector3F pt = map ? mapToA.TransformPoint(m[k].Point) : m[k].Point;
                    if (IsGhostRegion(soupIsA ? pt : soupFromA.TransformPoint(pt), in tri)) { return; }
                }
            }
        }

        ref ContactOut o = ref sc.Next();
        o.SubA = subA; o.SubB = subB; o.Hint = default; o.Recycled = false;
        Vector3F nw = f.Rwa.Rotate(n);
        o.Nx = nw.X; o.Ny = nw.Y; o.Nz = nw.Z;
        for (int k = 0; k < count; k++)
        {
            var mp = m[k];
            Vector3F pt = map ? mapToA.TransformPoint(mp.Point) : mp.Point;
            Vector3F ra = f.ArmA + f.Rwa.Rotate(pt);
            Vector3F rb = ra + f.ComDiff;
            int q = k * 7;
            o.P[q] = ra.X; o.P[q + 1] = ra.Y; o.P[q + 2] = ra.Z; o.P[q + 3] = rb.X; o.P[q + 4] = rb.Y; o.P[q + 5] = rb.Z; o.P[q + 6] = mp.Separation;
            o.Id[k] = mp.Id;
        }

        o.Count = count;
    }

    // A manifold whose normal agrees with the triangle's own normal to within this cosine (under 0.3 degrees) needs no treatment.
    private const float GhostCosine = 0.99999f;

    private static bool IsGhostRegion(Vector3F pointInSoup, in SoupTriangle3D tri)
    {
        int region = ClosestRegion(pointInSoup, tri.V0, tri.V1, tri.V2);
        if (region == 0)
        {
            // A point on a seam is a hair inside the face as often as a hair outside it, and which one float rounding picks depends on the exact pose (a spinning sphere crosses the same seam at
            // different phases). Within 5 mm of an inactive edge (the contact point sits midway between the surfaces, so a penetrating shape puts it off the edge by about half its depth) it counts as being on that edge.
            return NearInactiveEdge(pointInSoup, in tri, 0, tri.V0, tri.V1) || NearInactiveEdge(pointInSoup, in tri, 1, tri.V1, tri.V2) || NearInactiveEdge(pointInSoup, in tri, 2, tri.V2, tri.V0);
        }

        if (region <= 3) { return !tri.IsEdgeActive(region - 1); }
        int v = region - 4;
        return !tri.IsEdgeActive((v + 2) % 3) && !tri.IsEdgeActive(v);
    }

    private const float SeamTolerance = 5e-3f;

    private static bool NearInactiveEdge(Vector3F p, in SoupTriangle3D tri, int edge, Vector3F a, Vector3F b)
    {
        if (tri.IsEdgeActive(edge)) { return false; }
        // Distance from the point to the segment (Ericson 5.1.2).
        Vector3F ab = b - a, ap = p - a;
        float len2 = Vector3F.Dot(ab, ab);
        float t = len2 > 0f ? Vector3F.Dot(ap, ab) / len2 : 0f;
        t = t < 0f ? 0f : t > 1f ? 1f : t;
        Vector3F d = ap - ab * t;
        return Vector3F.Dot(d, d) <= SeamTolerance * SeamTolerance;
    }

    /// <summary>Which feature of a triangle is nearest to a point (Ericson, Real-Time Collision Detection, 5.1.5): 0 face, 1..3 edge 0..2, 4..6 vertex 0..2.</summary>
    internal static int ClosestRegion(Vector3F p, Vector3F a, Vector3F b, Vector3F c)
    {
        Vector3F ab = b - a, ac = c - a, ap = p - a;
        float d1 = Vector3F.Dot(ab, ap), d2 = Vector3F.Dot(ac, ap);
        if (d1 <= 0f && d2 <= 0f) { return 4; }
        Vector3F bp = p - b;
        float d3 = Vector3F.Dot(ab, bp), d4 = Vector3F.Dot(ac, bp);
        if (d3 >= 0f && d4 <= d3) { return 5; }
        float vc = d1 * d4 - d3 * d2;
        if (vc <= 0f && d1 >= 0f && d3 <= 0f) { return 1; }
        Vector3F cp = p - c;
        float d5 = Vector3F.Dot(ab, cp), d6 = Vector3F.Dot(ac, cp);
        if (d6 >= 0f && d5 <= d6) { return 6; }
        float vb = d5 * d2 - d1 * d6;
        if (vb <= 0f && d2 >= 0f && d6 <= 0f) { return 3; }
        float va = d3 * d6 - d5 * d4;
        if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f) { return 2; }
        return 0;
    }

    // ------------------------------------------------------------------ payload

    private void WriteManifold(ConstraintStore cs, int c, int sa, int sb, in ContactOut res)
    {
        if (res.Recycled) { WriteRecycled(cs, c, sa, sb, in res); return; }
        var pl = cs.Payload(c);
        var ids = cs.FeatureIdsOf(c);
        int oldCount = cs.PointCount[c];
        // Warm starting: impulses of the old point with the same feature id carry over.
        var old = default(ImpulseBuffer);
        for (int k = 0; k < res.Count; k++)
        {
            uint id = res.Id[k];
            for (int j = 0; j < oldCount; j++)
            {
                if (ids[j] != id) { continue; }
                old[k] = pl[ManifoldLayout.NormalImpulse(j)]; old[4 + k] = pl[ManifoldLayout.Tangent1Impulse(j)]; old[8 + k] = pl[ManifoldLayout.Tangent2Impulse(j)];
                break;
            }
        }

        float r0 = 0f, r1 = 0f, r2 = 0f;
        if (oldCount > 0) { r0 = pl[ManifoldLayout.RollingImpulse]; r1 = pl[ManifoldLayout.RollingImpulse + 1]; r2 = pl[ManifoldLayout.RollingImpulse + 2]; }
        pl.Clear();
        var mixed = Mix(sa, sb);
        pl[ManifoldLayout.Normal] = res.Nx; pl[ManifoldLayout.Normal + 1] = res.Ny; pl[ManifoldLayout.Normal + 2] = res.Nz;
        pl[ManifoldLayout.Friction] = mixed.Friction;
        pl[ManifoldLayout.Restitution] = mixed.Restitution;
        pl[ManifoldLayout.RollingResistance] = mixed.RollingResistance;
        pl[ManifoldLayout.RollingImpulse] = r0; pl[ManifoldLayout.RollingImpulse + 1] = r1; pl[ManifoldLayout.RollingImpulse + 2] = r2;
        for (int k = 0; k < res.Count; k++)
        {
            int q = 7 * k;
            int a = ManifoldLayout.AnchorA(k), b = ManifoldLayout.AnchorB(k);
            pl[a] = res.P[q]; pl[a + 1] = res.P[q + 1]; pl[a + 2] = res.P[q + 2];
            pl[b] = res.P[q + 3]; pl[b + 1] = res.P[q + 4]; pl[b + 2] = res.P[q + 5];
            pl[ManifoldLayout.Separation(k)] = res.P[q + 6];
            pl[ManifoldLayout.NormalImpulse(k)] = old[k];
            pl[ManifoldLayout.Tangent1Impulse(k)] = old[4 + k];
            pl[ManifoldLayout.Tangent2Impulse(k)] = old[8 + k];
            ids[k] = res.Id[k];
        }

        res.Hint.CopyTo(pl.Slice(ManifoldLayout.HintOffset, 7));
        for (int k = res.Count; k < ids.Length; k++) { ids[k] = 0; }
        cs.PointCount[c] = (byte)res.Count;
        StoreRecycleFrame(pl, sa, sb, in res);
    }

    // ------------------------------------------------------------------ manifold recycling

    // Catto's contact reuse (Box2D 3.2 release notes: a touching contact keeps its manifold until the shapes have moved about 5 cm or turned about 11 degrees relative to each other), as 2D does it.
    // A manifold is a set of points fixed to the two bodies, so while the bodies move rigidly the points move with them and the separation changes by the motion of the two points along the normal.
    // The narrowphase is skipped for such a pair; the arms, the normal and the separations are rebuilt from the body-fixed copies kept in the payload.
    /// <summary>Whether a touching pair of convex shapes that has barely moved reuses its manifold instead of running the narrowphase. On by default; off gives every pair a fresh manifold each step (the result differs slightly, so hashes differ).</summary>
    public bool RecycleManifolds = true;

    // Bound on how far any point of B has moved relative to A since the manifold was made (metres, with the turn scaled by B's extent), and how deep a reused point may have sunk.
    private const float RecycleTol = 0.0035f;
    private const float RecycleDepth = 0.02f;
    // A normal counts as a face normal when it is parallel to one to within about a quarter of a degree.
    private const float FaceParallel = 0.99999f;

    private static class Rc
    {
        public const int Base = ManifoldLayout.RecycleOffset;
        public const int Owner = Base;             // 0 none, 1 shape A's body, 2 shape B's body
        public const int Normal = Base + 1;        // the normal in the owner body's frame
        public const int Rel = Base + 4;           // B's centre of mass in A's frame (3), then the rotation of B in A (x, y, z, w)
        public const int Points = Base + 11;       // per point: arm A in A's frame (3), arm B in B's frame (3), the separation
    }

    private static bool IsFaceNormal(in ConvexShape3D shape, Vector3F n)
    {
        switch (shape.Kind)
        {
            case ConvexShapeKind3D.Box:
                return Math.Abs(n.X) > FaceParallel || Math.Abs(n.Y) > FaceParallel || Math.Abs(n.Z) > FaceParallel;
            case ConvexShapeKind3D.Cylinder:
                return Math.Abs(n.Y) > FaceParallel;
            case ConvexShapeKind3D.Hull:
                var hull = shape.Hull;
                if (hull == null) { return false; }
                foreach (var face in hull.Faces) { if (Vector3F.Dot(face.Normal, n) > FaceParallel) { return true; } }
                return false;
            default:
                return false;
        }
    }

    private void StoreRecycleFrame(Span<float> pl, int sa, int sb, in ContactOut res)
    {
        var ga = Geoms[GeomOf[sa]]!;
        var gb = Geoms[GeomOf[sb]]!;
        // 0: two convex shapes, 1 or 2: the soup is shape A or B and the other shape is convex.
        int soup = ga.Soup != null && gb.IsConvex ? 1 : gb.Soup != null && ga.IsConvex ? 2 : 0;
        if (soup == 0 && !(ga.IsConvex && gb.IsConvex)) { return; }
        var owner = Core.Shapes.Owner;
        int ba = owner[sa], bb = owner[sb];
        var bodies = Core.Bodies;
        var pos = bodies.Position;
        QuaternionF qA = RotationOfBody(ba), qB = RotationOfBody(bb);
        var n = new Vector3F(res.Nx, res.Ny, res.Nz);
        // The normal belongs to the body whose face made it, the reference face: it is that face's normal and turns with that body. A normal that is no face normal (edge against edge, a ball, a capsule)
        // follows shape A; it is then right to within the relative motion the reuse allows, divided by the shapes' size.
        int frame = soup == 2 ? 2 : 1;
        if (soup == 0 && !IsFaceNormal(ga.Convex, (qA * Local[sa].Rotation).Conjugate().Rotate(n)) && IsFaceNormal(gb.Convex, (qB * Local[sb].Rotation).Conjugate().Rotate(-n))) { frame = 2; }
        pl[Rc.Owner] = frame;
        var nl = (frame == 1 ? qA : qB).Conjugate().Rotate(n);
        pl[Rc.Normal] = nl.X; pl[Rc.Normal + 1] = nl.Y; pl[Rc.Normal + 2] = nl.Z;
        var invA = qA.Conjugate();
        var dW = new Vector3F((float)(pos[3 * bb] - pos[3 * ba]), (float)(pos[3 * bb + 1] - pos[3 * ba + 1]), (float)(pos[3 * bb + 2] - pos[3 * ba + 2]));
        // The relative pose is kept as the other body's in the frame of the reference body: A, or the soup's body when B is the soup (the soup's own points are fixed in its frame, so only the other body's move).
        var invR = soup == 2 ? qB.Conjugate() : invA;
        var d = invR.Rotate(soup == 2 ? -dW : dW);
        pl[Rc.Rel] = d.X; pl[Rc.Rel + 1] = d.Y; pl[Rc.Rel + 2] = d.Z;
        var qr = invR * (soup == 2 ? qA : qB);
        pl[Rc.Rel + 3] = qr.X; pl[Rc.Rel + 4] = qr.Y; pl[Rc.Rel + 5] = qr.Z; pl[Rc.Rel + 6] = qr.W;
        var invB = qB.Conjugate();
        for (int k = 0; k < res.Count; k++)
        {
            int q = k * 7, o = Rc.Points + 7 * k;
            var la = invA.Rotate(new Vector3F(res.P[q], res.P[q + 1], res.P[q + 2]));
            var lb = invB.Rotate(new Vector3F(res.P[q + 3], res.P[q + 4], res.P[q + 5]));
            pl[o] = la.X; pl[o + 1] = la.Y; pl[o + 2] = la.Z; pl[o + 3] = lb.X; pl[o + 4] = lb.Y; pl[o + 5] = lb.Z; pl[o + 6] = res.P[q + 6];
        }
    }

    /// <summary>Builds the pair's manifold from the one kept in the constraint's payload when B has moved and turned relative to A by less than the bound; false when the narrowphase has to run.</summary>
    private bool TryRecycle(NarrowScratch sc, int c, int ba, int bb, QuaternionF qA, QuaternionF qB, Vector3F dW, int soup = 0)
    {
        var cs = Core.Constraints;
        int count = cs.PointCount[c];
        if (count == 0 || cs.KindOf(c) != ConstraintKind.Contact) { return false; }
        var pl = cs.Payload(c);
        // Compared as floats: snapshot bytes may hold anything here, and only 1 and 2 mean a stored frame.
        float ownerF = pl[Rc.Owner];
        int owner = ownerF == 1f ? 1 : ownerF == 2f ? 2 : 0;
        if (owner == 0) { return false; }
        // The reference body is A, or the soup's body when B is the soup; the other body is the mover (see StoreRecycleFrame).
        var invA = qA.Conjugate();
        var invR = soup == 2 ? qB.Conjugate() : invA;
        int mover = soup == 2 ? ba : bb;
        // How far the mover has moved and turned in the reference body's frame since the manifold was made.
        var d = invR.Rotate(soup == 2 ? -dW : dW);
        float ex = d.X - pl[Rc.Rel], ey = d.Y - pl[Rc.Rel + 1], ez = d.Z - pl[Rc.Rel + 2];
        var q0 = new QuaternionF(pl[Rc.Rel + 3], pl[Rc.Rel + 4], pl[Rc.Rel + 5], pl[Rc.Rel + 6]);
        var dq = (invR * (soup == 2 ? qA : qB)) * q0.Conjugate();
        float lim = RecycleTol - float.Sqrt(ex * ex + ey * ey + ez * ez); // what is left of the budget for the turn
        float turn = 2f * float.Sqrt(dq.X * dq.X + dq.Y * dq.Y + dq.Z * dq.Z);
        if (!(lim >= 0f && turn * MaxExtent[mover] <= lim)) { return false; } // false for NaN too
        var nl = new Vector3F(pl[Rc.Normal], pl[Rc.Normal + 1], pl[Rc.Normal + 2]);
        Vector3F n = (owner == 2 ? qB : qA).Rotate(nl);
        ref ContactOut o = ref sc.Next();
        o.SubA = soup != 0 ? cs.SubShapeA[c] : 0; o.SubB = soup != 0 ? cs.SubShapeB[c] : 0; o.Hint = default; o.Recycled = true;
        for (int k = 0; k < count; k++)
        {
            int b = Rc.Points + 7 * k;
            Vector3F ra = qA.Rotate(new Vector3F(pl[b], pl[b + 1], pl[b + 2]));
            Vector3F rb = qB.Rotate(new Vector3F(pl[b + 3], pl[b + 4], pl[b + 5]));
            // Separation now: the one when made plus the motion of B's point relative to A's point along the normal.
            float sep = pl[b + 6] + Vector3F.Dot(dW + rb - ra, n);
            if (!(sep <= Opt.SpeculativeDistance && sep >= -RecycleDepth)) { sc.OutCount--; return false; }
            int q = k * 7;
            o.P[q] = ra.X; o.P[q + 1] = ra.Y; o.P[q + 2] = ra.Z; o.P[q + 3] = rb.X; o.P[q + 4] = rb.Y; o.P[q + 5] = rb.Z; o.P[q + 6] = sep;
        }

        // The normal is already in world axes; Emit's rotation by the shape frame is not needed.
        o.Nx = n.X; o.Ny = n.Y; o.Nz = n.Z;
        o.Count = count;
        return true;
    }

    /// <summary>Reuses every triangle contact of a convex-on-soup pair, or none: false (with nothing emitted) as soon as one of them has moved too far.</summary>
    private bool TryRecycleSoup(NarrowScratch sc, int p, int ba, int bb, QuaternionF qA, QuaternionF qB, Vector3F dW, int soup)
    {
        int start = sc.OutCount;
        for (int c = PairCons[p]; c >= 0; c = ConsNext[c])
        {
            if (!TryRecycle(sc, c, ba, bb, qA, qB, dW, soup)) { sc.OutCount = start; return false; }
        }

        return sc.OutCount > start;
    }

    private void WriteRecycled(ConstraintStore cs, int c, int sa, int sb, in ContactOut res)
    {
        var pl = cs.Payload(c);
        // A material can change between steps (a body's friction setter), and a fresh manifold would pick that up, so a reused one does too.
        var mixed = Mix(sa, sb);
        pl[ManifoldLayout.Normal] = res.Nx; pl[ManifoldLayout.Normal + 1] = res.Ny; pl[ManifoldLayout.Normal + 2] = res.Nz;
        pl[ManifoldLayout.Friction] = mixed.Friction;
        pl[ManifoldLayout.Restitution] = mixed.Restitution;
        pl[ManifoldLayout.RollingResistance] = mixed.RollingResistance;
        for (int k = 0; k < res.Count; k++)
        {
            int q = 7 * k;
            int a = ManifoldLayout.AnchorA(k), b = ManifoldLayout.AnchorB(k);
            pl[a] = res.P[q]; pl[a + 1] = res.P[q + 1]; pl[a + 2] = res.P[q + 2];
            pl[b] = res.P[q + 3]; pl[b + 1] = res.P[q + 4]; pl[b + 2] = res.P[q + 5];
            pl[ManifoldLayout.Separation(k)] = res.P[q + 6];
        }
    }

    private Material3D Mix(int sa, int sb)
    {
        var mixer = Opt.MaterialMixer;
        if (mixer != null)
        {
            return mixer(new Material3D(Friction[sa], Restitution[sa], Rolling[sa]), new Material3D(Friction[sb], Restitution[sb], Rolling[sb]));
        }

        // Friction: geometric mean (a slippery surface stays slippery); restitution and rolling resistance: the larger.
        float fr = float.Sqrt(Friction[sa] * Friction[sb]);
        float re = Restitution[sa] > Restitution[sb] ? Restitution[sa] : Restitution[sb];
        float rr = Rolling[sa] > Rolling[sb] ? Rolling[sa] : Rolling[sb];
        return new Material3D(fr, re, rr);
    }
}
