using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>The step stages of the 3D engine. A struct so the core's generic step specializes on it; all state lives in the engine object.</summary>
internal readonly struct Stages3D : IWorldStages
{
    private readonly Engine3D _e;

    public Stages3D(Engine3D e) => _e = e;

    public void UpdateContacts(WorldCore world) { _e.UpdateContacts(world); _e.ApplyFieldContacts(); _e.StepArticulations(); _e.PackLateWakes(); }

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

    public bool JointNeedsFullSchedule(WorldCore world, int constraint) => _e.JointNeedsFullSchedule(constraint);

    public void PrepareBatch(WorldCore world, ReadOnlySpan<int> constraints, int batch, in StepContext context) => _e.PrepareBatch(constraints, batch, in context);

    public float BodyMotion(WorldCore world, int body) => _e.BodyMotion(body);

    public float[]? MotionExtent(WorldCore world) => _e.MaxExtent;

    public bool TouchesMovingKinematic(WorldCore world, int body) => _e.TouchesMovingKinematic(body);

    public void FinalizeBodies(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context) => _e.FinalizeBodies(bodies, in context);

    // Spelled out because calling a default interface member on a struct boxes it: that was 48 bytes allocated by every step.
    public void AfterFinalize(WorldCore world, in StepContext context) { _e.EndArticulations(); }

    public bool BodyMotionIsConcurrent => true;

    // Spelled out for the same reason as AfterFinalize.
    public bool NeedsRestitutionPass(WorldCore world, in StepContext context) => _e.NeedsRestitutionPass(context.Schedule);
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
    public int OriginalSubA, OriginalSubB;
    public bool FlatPatch;
    public float FlatPlane;
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
        public readonly Engine3D.SdfProbeBuffer Sdf = new();
        public readonly Engine3D.BoxJob[] BoxJobs = new Engine3D.BoxJob[64];
        public readonly ManifoldGenerator3D.BoxAxes[] BoxRes = new ManifoldGenerator3D.BoxAxes[64];
    }

    [ThreadStatic]
    private static Heavy? t_heavy;

    private static Heavy H => t_heavy ??= new Heavy();

    public static ManifoldGenerator3D Gen => H.Gen;
    public static List<ChildContact3D> Children => H.Children;
    public static List<SoupTriangle3D> Tris => H.Tris;
    public static List<int> Ids => H.Ids;
    /// <summary>Hands a chunk the thread's box queue and its axis results (a chunk has at most 64 pairs, so 64 entries always suffice). One set per thread, like Gen: a chunk fills and empties it
    /// inside its own task, and a set per chunk was a cold 16 KB on every chunk at one thread.</summary>
    public static void TakeBoxQueue(NarrowScratch sc) { var h = H; sc.BoxJobs = h.BoxJobs; sc.BoxRes = h.BoxRes; }
    public static TriangleShapeBuffer3D TriBuf => H.TriBuf;
    internal static Engine3D.SdfProbeBuffer Sdf => H.Sdf;
    public ContactOut[] Out = new ContactOut[128];
    public int OutCount;
    public Vector3F[] FlatLocalPoints = Array.Empty<Vector3F>();
    public int Recycled, RecycledPoints, FreshQueries;
    /// <summary>The field of the pair being collided belongs to a moving body (set per pair).</summary>
    public bool SdfMoves;
    /// <summary>Pairs of this chunk that passed the tight-box test, and those among them that ran the narrowphase (not skipped as apart, not reusing a manifold); read by the work counters.</summary>
    public int Reached, Fresh;
    /// <summary>The chunk's survivors of the box phase (pairs whose fat and tight boxes overlap), as positions in the update list in ascending order, with each pair's speculative distance;
    /// <see cref="SurvivorCount"/> of them are valid. Phase 2 of <c>CollideTask</c> runs the pose-level work on exactly these, so a later pass can take them in batches (e.g. eight at a time).</summary>
    public readonly int[] Survivor = new int[64];
    public readonly float[] SurvivorSpec = new float[64];
    public int SurvivorCount;
    /// <summary>The chunk's fresh box–box pairs, queued in phase 2 (Engine3D.QueueBoxPair) and run by Engine3D.RunBoxJobs; BoxRes holds their axis tests.</summary>
    public Engine3D.BoxJob[] BoxJobs = null!;
    public ManifoldGenerator3D.BoxAxes[] BoxRes = null!;
    public int BoxCount;

    public ref ContactOut Next()
    {
        if (OutCount == Out.Length) { Array.Resize(ref Out, Out.Length * 2); }
        return ref Out[OutCount++];
    }
}

internal sealed partial class Engine3D
{
    internal ContactModificationPipeline ContactModifications => Core.ContactModifications;

    // ------------------------------------------------------------------ pair filter for the broadphase

    private readonly struct PairFilter : IPairFilter
    {
        private readonly Engine3D _e;

        public PairFilter(Engine3D e) => _e = e;

        public bool IsSensor(int shape) => _e.IsSensor(shape);

        public bool ShouldCollide(int shapeA, int shapeB)
        {
            // A pair that already has a table entry is not new, so it is dropped here instead of being sorted with the new ones and dropped after.
            // The table is only read while pairs are searched; it changes in the serial stage that follows.
            if (_e.PairMap.ContainsKey(shapeA < shapeB ? PairKey(shapeA, shapeB) : PairKey(shapeB, shapeA))) { return false; }
            var owner = _e.Core.Shapes.Owner;
            return owner[shapeA] != owner[shapeB] && _e.CanCollide(shapeA, shapeB) && !_e.JointsPreventCollision(owner[shapeA], owner[shapeB]) && !_e.ArticulationBlocks(owner[shapeA], owner[shapeB]);
        }
    }

    /// <summary>Work done by the last contact stage: pairs the broadphase reported as new, pairs revisited (an awake body), pairs that passed the tight-box test, and those that ran the narrowphase.</summary>
    internal int StatNewPairs, StatUpdated, StatReached, StatFresh;

    private byte[] _active = new byte[8]; // per body: awake and dynamic, or a kinematic body in motion (see IsActive); valid for the contact stage that filled it
    private int[] _boxStamp = new int[8];
    private byte[] _boxInside = new byte[8]; // per shape, set with the tight box: 1 when the box lies inside the proxy's stored box, so the tree update has nothing to do
    // The move prepared by TightBoxTask for a shape it listed: _moveState 1 = _moveBox is the new fat box (Broadphase.CommitMove), 2 = a check failed and _moveBox is the tight box for MoveProxy to reject.
    private byte[] _moveState = new byte[8];
    private Aabb3D[] _moveBox = new Aabb3D[8];
    private int[][] _moveLists = Array.Empty<int[]>(); // per box chunk: the shapes whose proxy grows, in awake-list and shape order
    private int[] _moveCount = Array.Empty<int>();
    private int _stamp;
    private int _skipMarginBits;
    private double[] _boxes = new double[48 + 2]; // six doubles a shape, plus two of padding so BoxPhase256 can load a box's upper half (offset 3) as four doubles
    private int[] _updateList = new int[64];
    private byte[] _resStatus = new byte[64]; // 0 untouched, 1 manifolds computed, 2 boxes no longer overlap, 3 a reused manifold that replaced the pair's one constraint in place, payload already written
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
        // A clearance is measured beyond the speculative margin used by that query.
        int marginBits = BitConverter.SingleToInt32Bits(Opt.SpeculativeDistance);
        if (_skipMarginBits != marginBits) { Array.Clear(_skip); _skipMarginBits = marginBits; }

        // 0. A changed filter re-evaluates the pair table before anything else (see Refilter).
        Refilter();

        // The tight box of every shape of a moving body is kept for this step: a pair whose shapes are farther apart than the speculative distance has no contact points, and two box comparisons say so
        // without loading either shape.
        int shapeHigh = world.Shapes.HighWater;
        if (_boxStamp.Length < shapeHigh) { int cap = Math.Max(shapeHigh, _boxStamp.Length * 2); _boxes = new double[cap * 6 + 2]; _boxStamp = new int[cap]; _boxInside = new byte[cap]; _moveState = new byte[cap]; _moveBox = new Aabb3D[cap]; }
        _stamp++;
        // 1. Broadphase boxes of every body that moved last step. The displacement stretches the box ahead of the motion.
        // The tight boxes, and for a shape that left its stored box the stretched box, MoveProxy's checks and the new fat box, are pure functions of body state and the shape's own leaf: they are computed
        // in parallel over fixed chunks of the awake list. The tree is then changed serially in list order.
        var awakeList = world.AwakeBodies;
        int boxChunks = (awakeList.Length + BoxChunk - 1) / BoxChunk;
        if (_moveLists.Length < boxChunks)
        {
            int old = _moveLists.Length;
            Array.Resize(ref _moveLists, Math.Max(boxChunks, old * 2));
            _moveCount = new int[_moveLists.Length];
            for (int i = old; i < _moveLists.Length; i++) { _moveLists[i] = new int[BoxChunk]; }
        }

        world.Executor.For(boxChunks, new TightBoxTask(this, boxChunks));
        Core.Profile.Mark(StepStage.Move);
#if STAGE_TIMERS
        StageProbe.Mark(StageProbe.Move);
#endif
        // The tasks did everything of the move but the tree change: each chunk lists, in awake-list and shape order, the shapes whose proxy must grow, with the new fat box. Walking the chunks in order
        // makes the same EnlargeProxy / MarkMoved calls in the same order as one serial pass. A shape whose checks failed is handed to MoveProxy here, which throws the exception it always did.
        var owner0 = world.Shapes.Owner;
        for (int ck = 0; ck < boxChunks; ck++)
        {
            int listed = _moveCount[ck];
            if (listed == 0) { continue; }
            int[] list = _moveLists[ck];
            for (int j = 0; j < listed; j++)
            {
                int s = list[j];
                if (_moveState[s] == 1) { Bp.CommitMove(Proxy[s], _moveBox[s]); continue; }
                int b = owner0[s];
                Bp.MoveProxy(Proxy[s], _moveBox[s], WorldCore.Grid(Dp[3 * b]), WorldCore.Grid(Dp[3 * b + 1]), WorldCore.Grid(Dp[3 * b + 2]));
            }
        }

        // 2. Start-of-step pose, zero accumulated motion and world-axes inverse inertia for every body (sleeping bodies may wake during the stage).
        Core.Profile.Mark(StepStage.Init);
#if STAGE_TIMERS
        StageProbe.Mark(StageProbe.Init);
#endif
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

        Core.Profile.Mark(StepStage.Pairs);
#if STAGE_TIMERS
        StageProbe.Mark(StageProbe.Pairs);
#endif
        // 3. New pairs from the broadphase, in its sorted order.
        var found = Bp.UpdatePairs(new PairFilter(this), world.Executor);
        world.Counters.BroadphasePairs = found.Length;
        bool anyFresh = false;
        bool userFilters = Opt.PairFilter != null || ContactFilter != null;
        for (int i = 0; i < found.Length; i++)
        {
            int a = found[i].UserA, b = found[i].UserB;
            if (a > b) { (a, b) = (b, a); }
            // The pair filter already dropped every pair with a table entry, and the broadphase reports each pair of shapes once, so none of these is in the table yet.
            // It also already applied the pure part of PairAllowed (different bodies, CanCollide, joints, articulations; all symmetric, and nothing they read changes between the search and here), so only the
            // user callbacks are left, and they are called here serially in the sorted pair order as before.
            if (userFilters && !PairCallbacksAllow(a, b)) { continue; }
            int np = AllocatePair(a, b);
            if (_fresh.Length < PairHigh) { Array.Resize(ref _fresh, Math.Max(PairHigh, _fresh.Length * 2)); }
            // A pair with a sensor is evaluated once when found even if both bodies are asleep, so a sleeping body inside a sensor still gets its enter event.
            _fresh[np] = IsSensor(a) || IsSensor(b);
            anyFresh |= _fresh[np];
        }

        // The broadphase never pairs a kinematic body with a static one, but the links of an articulation are kinematic bodies that must touch the ground: their pairs are found here.
        anyFresh |= AddLinkPairs();

        // Most pairs end up with one constraint; making room for them (and half as many again) at once saves the repeated doubling when thousands of pairs appear in one step.
        world.Constraints.Reserve(PairHigh + (PairHigh >> 1));
        if (_skip.Length < PairHigh) { Array.Resize(ref _skip, Math.Max(PairHigh, _skip.Length * 2)); }
        Core.Profile.Mark(StepStage.Narrowphase);
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

        // Work counters of the step, for benchmark reports: pairs found by the broadphase this step, and the sums of the per-chunk counts below (set after the narrowphase).
        StatNewPairs = found.Length;
        if (_applyCount.Length < chunks) { _applyCount = new int[Math.Max(chunks, _applyCount.Length * 2)]; _writeMask = new ulong[_applyCount.Length]; }
        if (_applyIdx.Length < chunks * 64) { _applyIdx = new int[Math.Max(chunks * 64, _applyIdx.Length * 2)]; }
        for (int i = 0; i < chunks; i++) { _scratch[i].OutCount = 0; _scratch[i].Reached = 0; _scratch[i].Fresh = 0; _scratch[i].Recycled = 0; }
        StatUpdated = _updateCount;
        StatReached = 0; StatFresh = 0;
        // The serial pass only adds, matches and removes constraints (their order fixes the colors); writing the manifolds into the payloads is independent per constraint and runs afterwards in parallel.
        // A user material mixer is called in pair order on this thread instead.
        _deferWrite = Opt.MaterialMixer == null && _updateCount >= 64;
        world.Executor.For(chunks, new CollideTask(this));
        for (int i = 0; i < chunks; i++) { StatReached += _scratch[i].Reached; StatFresh += _scratch[i].Fresh; }
        world.Counters.UpdatedPairs = _updateCount;
        for (int i = 0; i < chunks; i++)
        {
            world.Counters.NarrowphasePairs += _scratch[i].FreshQueries;
            world.Counters.RecycledManifolds += _scratch[i].Recycled;
            world.Counters.RecycledContactPoints += _scratch[i].RecycledPoints;
        }

        Core.Profile.Mark(StepStage.Apply);
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
        if (world.ContactMotionEnabled) world.ContactMotion.Capture(world);
        if (ContactModifications.Refresh) { ContactModifications.Run(world); }
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
        ConfigureJointBreaking();
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

    // Whether the pair's new contacts are exactly its present constraints (same sub-shape keys in the same order, at least one) and nothing needs waking. If so the constraints are recorded for the payload write
    // and the pair skips the serial pass. Reads only; it runs inside the parallel narrowphase, where nothing adds, removes or wakes.
    private bool MatchesInPlace(int i, NarrowScratch sc)
    {
        if (_resStatus[i] != 1) { return false; }
        int count = _resCount[i];
        int p = _updateList[i];
        int c = PairCons[p];
        if (count == 0 || c < 0) { return false; }
        int sa = PairA[p], sb = PairB[p];
        if (IsSensor(sa) || IsSensor(sb)) { return false; }
        var bodies = Core.Bodies;
        var owner = Core.Shapes.Owner;
        int ba = owner[sa], bb = owner[sb];
        if ((bodies.IsDynamic(ba) && !bodies.IsAwake(ba)) || (bodies.IsDynamic(bb) && !bodies.IsAwake(bb))) { return false; }
        var cs = Core.Constraints;
        int first = _resFirst[i];
        for (int j = 0; j < count; j++, c = ConsNext[c])
        {
            if (c < 0 || cs.SubShapeA[c] != sc.Out[first + j].SubA || cs.SubShapeB[c] != sc.Out[first + j].SubB) { return false; }
        }

        if (c >= 0) { return false; }
        c = PairCons[p];
        for (int j = 0; j < count; j++, c = ConsNext[c])
        {
            ref ContactOut o = ref sc.Out[first + j];
            o.Cons = c; o.ShapeA = sa; o.ShapeB = sb;
        }

        return true;
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

    /// <summary>
    /// Field contacts and the articulation step run after the contact stage and may wake islands. A body they woke was asleep when the stage packed the bodies, so its velocity row is copied here, before
    /// the solve reads it (see <see cref="Pk"/>). The bodies the stage itself woke were packed at its end; copying those again changes nothing.
    /// </summary>
    internal void PackLateWakes()
    {
        if ((_fields == null || _fields.Count == 0) && (_articulations == null || _articulations.Count == 0)) { return; }
        var bodies = Core.Bodies;
        for (int i = 0; i < _sleeperCount; i++)
        {
            int b = _sleepers[i];
            if (bodies.IsAwake(b)) { PackVelocity(b); }
        }
    }

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
                if (!bodies.IsLive(b)) { e._active[b] = 0; e._isSleeper[b] = 0; e.ReachLinear[b] = 0f; e.ReachTurn[b] = 0f; e.ReachSum[b] = 0f; continue; }
                e.SetReach(b);
                e._active[b] = IsActive(bodies, b) || e.IsLinkBody(b) ? (byte)1 : (byte)0;
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
            int[] list0 = e._moveLists[task];
            int listed = 0;
            Span<double> disp = stackalloc double[3];
            for (int i = lo; i < hi; i++)
            {
                int b = list[i];
                if (!bodies.IsLive(b)) { continue; }
                for (int s = e.ShapeHead[b]; s >= 0; s = e.NextShape[s])
                {
                    e.TightBoxCore(b, s, out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
                    int o = 6 * s;
                    var boxes = e._boxes;
                    boxes[o] = x0; boxes[o + 1] = y0; boxes[o + 2] = z0; boxes[o + 3] = x1; boxes[o + 4] = y1; boxes[o + 5] = z1;
                    e._boxStamp[s] = e._stamp;
                    // Most moving shapes stay inside their stored box; the test reads the tree, which nothing changes until the serial pass. A box with a NaN never counts as inside, and a displacement that is not
                    // finite is left to the tree update, which rejects it.
                    int b3 = 3 * b;
                    var dp = e.Dp;
                    byte inside = 0;
                    if (float.IsFinite(dp[b3]) && float.IsFinite(dp[b3 + 1]) && float.IsFinite(dp[b3 + 2]))
                    {
                        ref readonly var fat = ref e.Bp.FatBoundsOfLive(e.Proxy[s]);
                        // A body that can travel farther than the speculative distance in one step needs room in its stored box for that travel. Otherwise a body that has used up its slack since the box was
                        // made can cross the last gap to a surface inside one step with no pair made for it yet, and meets the surface with no contact. Slow bodies keep the plain test.
                        if (e.FastTravel(b, out double tx, out double ty, out double tz))
                        {
                            // Room for the travel on the side it moves towards.
                            if (fat.MinX <= x0 - Math.Max(0.0, -tx) && fat.MinY <= y0 - Math.Max(0.0, -ty) && fat.MinZ <= z0 - Math.Max(0.0, -tz)
                                && z1 + Math.Max(0.0, tz) <= fat.MaxZ && y1 + Math.Max(0.0, ty) <= fat.MaxY && x1 + Math.Max(0.0, tx) <= fat.MaxX) { inside = 1; }
                        }
                        else if (fat.MinX <= x0 && fat.MinY <= y0 && fat.MinZ <= z0 && z1 <= fat.MaxZ && y1 <= fat.MaxY && x1 <= fat.MaxX) { inside = 1; }
                    }

                    e._boxInside[s] = inside;
                    if (inside != 0) { continue; }
                    // Not inside: the broadphase move of the serial pass, up to the tree change. The box is grown by the travel exactly as before, then Broadphase.PrepareMove does MoveProxy's checks,
                    // its containment test and its fat box without touching the tree (each shape reads only its own leaf, which no task changes).
                    var tight = new Aabb3D(x0, y0, z0, x1, y1, z1);
                    if (e.FastTravel(b, out double mx, out double my, out double mz))
                    {
                        // The stored box is made around the box grown by the body's travel on the side it moves towards, so the test above holds for the whole of the next step. It never shrinks the box and
                        // stays far enough inside the range the broadphase accepts for the margin the broadphase adds.
                        double lim = BoxLimits.MaxCoordinate - e.Opt.BroadphaseMargin - 1.0;
                        tight = new Aabb3D(Math.Min(tight.MinX, Math.Max(-lim, tight.MinX - Math.Max(0.0, -mx))), Math.Min(tight.MinY, Math.Max(-lim, tight.MinY - Math.Max(0.0, -my))), Math.Min(tight.MinZ, Math.Max(-lim, tight.MinZ - Math.Max(0.0, -mz))),
                            Math.Max(tight.MaxX, Math.Min(lim, tight.MaxX + Math.Max(0.0, mx))), Math.Max(tight.MaxY, Math.Min(lim, tight.MaxY + Math.Max(0.0, my))), Math.Max(tight.MaxZ, Math.Min(lim, tight.MaxZ + Math.Max(0.0, mz))));
                    }

                    disp[0] = WorldCore.Grid(dp[b3]); disp[1] = WorldCore.Grid(dp[b3 + 1]); disp[2] = WorldCore.Grid(dp[b3 + 2]);
                    int state = e.Bp.PrepareMove(e.Proxy[s], tight, disp, out Aabb3D grown);
                    if (state == 0) { continue; }
                    e._moveState[s] = (byte)state;
                    e._moveBox[s] = state == 1 ? grown : tight;
                    if (listed == list0.Length) { Array.Resize(ref list0, listed * 2); e._moveLists[task] = list0; }
                    list0[listed++] = s;
                }
            }

            e._moveCount[task] = listed;
        }
    }

    private int[] _applyIdx = new int[64];
    private int[] _applyCount = new int[1];
    // Per chunk, the update-list positions (bit k = lo + k) whose results WriteTask writes: survivors with results that CollideSurvivor did not write itself (status 1, count > 0).
    private ulong[] _writeMask = new ulong[1];

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
                if (e.ContactModifications.Refresh || active[owner[e.PairA[p]]] != 0 || active[owner[e.PairB[p]]] != 0 || (_anyFresh && p < fresh.Length && fresh[p])) { e._updateList[lo + n++] = p; }
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
            int lo = task * 64;
            var sc = _e._scratch[task];
            // Only the chunk's pairs in the write mask (status 1 with results, ascending); a reuse that matched in place (status 3) was written by CollideSurvivor already, and the
            // pairs without results have nothing to write. Same set and order as walking the chunk and skipping status != 1 and count 0.
            for (ulong w = _e._writeMask[task]; w != 0; w &= w - 1)
            {
                int i = lo + BitOperations.TrailingZeroCount(w);
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

    // The chunk runs in three phases. Phase 1 (BoxPhase) is a call-free loop over the chunk's pairs: fat-box test, speculative distance, tight-box test; it writes the result arrays of the
    // pairs that leave there and lists the survivors in the scratch. Phase 2 runs the pose-level work (StillApart, recycle, CollideFresh) on the survivors only. Phase 3 writes the chunk's apply list in
    // ascending order from a 64-bit mask (bit k = update-list position lo + k), so the serial pass sees the same list as when every pair ran start to finish in turn. Results are identical: an early-out
    // pair touches no shared state but its own result slots, and the survivors run in their original order.
    private readonly struct CollideTask : IParallelBody
    {
        private readonly Engine3D _e;

        public CollideTask(Engine3D e) => _e = e;

        public void Run(int task)
        {
            var e = _e;
            int lo = task * 64, hi = Math.Min(lo + 64, e._updateCount);
            var sc = e._scratch[task];
            ulong listed = e.KernelWidth == 8 ? BoxPhase256(e, lo, hi, sc) : BoxPhase(e, lo, hi, sc);
            int recycled = 0, recycledPoints = 0, n = sc.SurvivorCount;
            // A survivor that needs a fresh box-box query is queued by CollideSurvivor (sc.BoxJobs) and run after the loop by RunBoxJobs, whose axis tests take the queue eight pairs at a time.
            // A pair's contacts are a contiguous run of sc.Out found through _resFirst/_resCount only (nothing reads sc.Out in order), so running the queued pairs later changes no result.
            // A pair that is not queued is accounted for at once, right after its CollideSurvivor call. That matters for a reuse written in place (status 3): it handed its scratch slot back
            // (OutCount--), and Account must read the slot's Recycled and Count before the next survivor (or a sphere-apart box pair finishing at once in QueueBoxPair) writes there. Queued pairs
            // write sc.Out only in RunBoxJobs, after the loop, when no reuse can hand back a slot any more; FinishBoxJob sets their _resFirst then, so it never points at a handed-back slot.
            sc.BoxCount = 0;
            ulong write = 0;
            if (n != 0) { NarrowScratch.TakeBoxQueue(sc); }
            for (int k = 0; k < n; k++)
            {
                int i = sc.Survivor[k];
                if (e.CollideSurvivor(i, sc, sc.SurvivorSpec[k])) { continue; } // queued: accounted for below, after RunBoxJobs
                Account(e, sc, i, lo, ref listed, ref write, ref recycled, ref recycledPoints);
            }

            if (sc.BoxCount != 0)
            {
                e.RunBoxJobs(sc);
                for (int k = 0; k < sc.BoxCount; k++) { Account(e, sc, sc.BoxJobs[k].I, lo, ref listed, ref write, ref recycled, ref recycledPoints); }
            }

            int m = 0;
            int[] apply = e._applyIdx;
            while (listed != 0)
            {
                apply[lo + m++] = lo + BitOperations.TrailingZeroCount(listed);
                listed &= listed - 1;
            }

            sc.Recycled = recycled; sc.RecycledPoints = recycledPoints; sc.FreshQueries = sc.Fresh;
            e._applyCount[task] = m;
            e._writeMask[task] = write;
        }

        // A pair's share of the chunk's counts, its bit in the apply mask and its bit in the write mask (the bits' positions fix the order of the serial pass and of WriteTask, so pairs may be
        // accounted for in any order).
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void Account(Engine3D e, NarrowScratch sc, int i, int lo, ref ulong listed, ref ulong write, ref int recycled, ref int recycledPoints)
        {
            int count = e._resCount[i];
            if (count != 0 && sc.Out[e._resFirst[i]].Recycled)
            {
                recycled += count;
                for (int c = 0; c < count; c++) { recycledPoints += sc.Out[e._resFirst[i] + c].Count; }
                // A convex reuse that CollideSurvivor found to match in place (status 3) is already written: nothing for the serial pass, nothing for WriteTask. A reuse is never queued, so this
                // runs right after its CollideSurvivor call, while its handed-back slot is intact. Any other reuse (a convex one that does not match in place, a triangle-soup pair's set of
                // reused manifolds) is judged below like a fresh result.
                if (e._resStatus[i] == 3) { return; }
            }

            if (count != 0) { write |= 1UL << (i - lo); }
            // Worth the serial pass: the pair has points or a constraint to match (a survivor's boxes overlap, so it is never a pair that goes).
            // A pair whose new contacts are exactly its constraints, one for one, changes nothing the serial pass owns (no constraint is added or removed, so no color moves); it only needs its payloads written.
            if ((count != 0 || e.PairCons[e._updateList[i]] != -1) && !(e._deferWrite && e.MatchesInPlace(i, sc))) { listed |= 1UL << (i - lo); }
        }
    }

    // ------------------------------------------------------------------ narrowphase

    /// <summary>The placement of a pair in world terms: A's shape orientation, the arm from A's centre of mass to A's shape origin, and the centre-of-mass offset between the bodies.</summary>
    internal struct PairFrame
    {
        public QuaternionF Rwa;
        public Vector3F ArmA;
        public Vector3F ComDiff;
        // Set when the pair is searched beyond the fixed speculative distance: the bodies' velocities, the fixed distance and the step, for the per-point closing test in Emit.
        public bool Approach;
        public Vector3F VA, WA, VB, WB;
        public float Spec, Dt;
    }

    /// <summary>Fills <see cref="ReachLinear"/> and <see cref="ReachTurn"/> of a body from its velocities and the step (nothing for a static body, no swing for a kinematic one).</summary>
    private void SetReach(int b)
    {
        var bodies = Core.Bodies;
        var type = bodies.TypeOf(b);
        float dt = Core.ContactStepDt, linear = 0f, turn = 0f;
        if (type != BodyType.Static)
        {
            var v = bodies.LinearVelocityOf(b);
            linear = float.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) * dt;
            if (type == BodyType.Dynamic)
            {
                var w = bodies.AngularVelocityOf(b);
                turn = float.Sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) * MaxExtent[b] * dt;
            }
        }

        ReachLinear[b] = linear; ReachTurn[b] = turn; ReachSum[b] = linear + turn;
    }

    /// <summary>
    /// Whether a body's centre can travel more than four times the speculative distance in the coming step; then <paramref name="tx"/>, <paramref name="ty"/>, <paramref name="tz"/> are that travel (velocity
    /// times the step). A NaN anywhere gives false: such a body is rejected elsewhere. The turn of a body is not part of this: a turning body's box changes every step anyway, and its swing into a
    /// surface is continuous collision's.
    /// </summary>
    private bool FastTravel(int b, out double tx, out double ty, out double tz)
    {
        double dt = Core.ContactStepDt;
        var bodies = Core.Bodies;
        tx = ty = tz = 0.0;
        if (bodies.TypeOf(b) == BodyType.Static) { return false; }
        var v = bodies.LinearVelocityOf(b);
        tx = v[0] * dt; ty = v[1] * dt; tz = v[2] * dt;
        return Math.Sqrt((tx * tx) + (ty * ty) + (tz * tz)) > FastFactor * Opt.SpeculativeDistance;
    }

    /// <summary>
    /// The distance up to which the narrowphase looks for contact points between two bodies. The fixed speculative distance covers slow approaches; a pair whose surfaces can close by more than twice
    /// that in one step (a fast fall, or the far end of a long shape swinging in) is searched out to the closing distance, so a point that will meet the other surface during the step already has a
    /// contact when the step starts, instead of being found after the shapes overlap (Catto, GDC 2013, continuous collision: a speculative contact has to reach as far as the motion of the step).
    /// Which of the points found are kept is decided by each point's own closing speed (see <see cref="Emit"/>). Pairs that move slowly keep the fixed distance, so a settled world is untouched.
    /// </summary>
    /// BoxPhase carries this method's fast path inline (and calls FastPairSpeculativeDistance for the rest); this method is the reference form.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private float PairSpeculativeDistance(int ba, int bb)
    {
        float spec = Opt.SpeculativeDistance;
        // The two bodies' own reaches bound the closing distance, so a pair of slow bodies is settled without looking at the velocities.
        if (!(ReachSum[ba] + ReachSum[bb] > (float)FastFactor * spec)) { return spec; }
        return FastPairSpeculativeDistance(ba, bb, spec);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private float FastPairSpeculativeDistance(int ba, int bb, float spec)
    {
        var bodies = Core.Bodies;
        if (bodies.TypeOf(ba) == BodyType.Dynamic && bodies.TypeOf(bb) == BodyType.Dynamic) { return spec; }
        var va = bodies.LinearVelocityOf(ba); var vb = bodies.LinearVelocityOf(bb);
        float dx = vb[0] - va[0], dy = vb[1] - va[1], dz = vb[2] - va[2];
        // The swing of a turning body counts against a static or kinematic partner only; between two dynamic bodies the relative linear speed decides, so a tumbling pile is not searched farther.
        float turn = (bodies.TypeOf(bb) != BodyType.Dynamic ? ReachTurn[ba] : 0f) + (bodies.TypeOf(ba) != BodyType.Dynamic ? ReachTurn[bb] : 0f);
        float sweep = (float.Sqrt(dx * dx + dy * dy + dz * dz) * Core.ContactStepDt) + turn;
        if (!(sweep > (float)FastFactor * spec)) { return spec; }
        return sweep < MaxPairSpeculation ? sweep : MaxPairSpeculation;
    }

    private const float MaxPairSpeculation = 0.25f;
    // A body counts as fast when it can travel or swing more than this many times the speculative distance in a step (about 4.8 m/s at 60 Hz with the default distance).
    private const double FastFactor = 4.0;

    // Phase 1 of a narrowphase chunk (a pair's collide step up to the tight-box test, for the pairs lo..hi-1 of the update list). No calls on the common path (the rare fast pair calls
    // FastPairSpeculativeDistance), arrays hoisted once and read without bounds checks (every index comes from the update list and the pair table, checked when the pairs were made), so the loads of
    // consecutive pairs overlap. Writes each pair's status/count/first; the survivors go to sc.Survivor/SurvivorSpec. Returns the apply-list mask of the pairs that left here: a pair whose fat boxes
    // no longer overlap (status 2, it goes) and a tight-box exit that still has a constraint (or sensor overlap) to remove.
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static ulong BoxPhase(Engine3D e, int lo, int hi, NarrowScratch sc)
    {
        // Base references of the arrays, taken once (a byref spilled to the stack reloads without the null check an array reference needs).
        ref int upd = ref Mem.Ref(e._updateList);
        ref int pairA = ref Mem.Ref(e.PairA), pairB = ref Mem.Ref(e.PairB);
        ref int proxy = ref Mem.Ref(e.Proxy), pairCons = ref Mem.Ref(e.PairCons);
        ref int boxStamp = ref Mem.Ref(e._boxStamp);
        ref int resCount = ref Mem.Ref(e._resCount), resFirst = ref Mem.Ref(e._resFirst);
        ref byte resStatus = ref Mem.Ref(e._resStatus);
        ref int surv = ref Mem.Ref(sc.Survivor);
        ref float survSpec = ref Mem.Ref(sc.SurvivorSpec);
        ref float reachSum = ref Mem.Ref(e.ReachSum);
        ref double boxes = ref Mem.Ref(e._boxes);
        ref int owner = ref Mem.Ref(e.Core.Shapes.Owner);
        var bp = e.Bp;
        var trees = new TreeNodes();
        trees[0] = bp.TreeAt(0).NodeArray; trees[1] = bp.TreeAt(1).NodeArray; trees[2] = bp.TreeAt(2).NodeArray;
        int stamp = e._stamp, n = 0;
        float spec = e.Opt.SpeculativeDistance, fast = (float)FastFactor * spec;
        ulong listed = 0;
        for (int i = lo; i < hi; i++)
        {
            int p = Mem.Add(ref upd, i);
            int sa = Mem.Add(ref pairA, p), sb = Mem.Add(ref pairB, p);
            int ka = Mem.Add(ref proxy, sa), kb = Mem.Add(ref proxy, sb);
            // fa, fb: the fat boxes, six doubles MinX MinY MinZ MaxX MaxY MaxZ (Aabb3D's field order, and the order of _boxes).
            ref double fa = ref Mem.As<Aabb3D, double>(ref Mem.At(Mem.Add(ref trees[0], ka & 3), ka >> 2).Box);
            ref double fb = ref Mem.As<Aabb3D, double>(ref Mem.At(Mem.Add(ref trees[0], kb & 3), kb >> 2).Box);
            // Every comparison is evaluated (non-short-circuit & and |): they are pure, and one flag per test replaces a chain of up to six branches whose exit varies from pair to pair.
            bool fatOk = (fa <= Mem.Add(ref fb, 3)) & (fb <= Mem.Add(ref fa, 3)) & (Mem.Add(ref fa, 1) <= Mem.Add(ref fb, 4)) & (Mem.Add(ref fb, 1) <= Mem.Add(ref fa, 4))
                & (Mem.Add(ref fa, 2) <= Mem.Add(ref fb, 5)) & (Mem.Add(ref fb, 2) <= Mem.Add(ref fa, 5));
            int ba = Mem.Add(ref owner, sa), bb = Mem.Add(ref owner, sb);
            // PairSpeculativeDistance, its fast path inline: the two bodies' own reaches bound the closing distance.
            float pairSpec = spec;
            if (fatOk && Mem.Add(ref reachSum, ba) + Mem.Add(ref reachSum, bb) > fast) { pairSpec = e.FastPairSpeculativeDistance(ba, bb, spec); }
            // Tight boxes (this step's for a shape that moved, the stored fat box for one that did not); the extra margin keeps the shortcut clear of rounding at the threshold.
            const double Gap = 0.005;
            double gap = pairSpec + Gap;
            ref double a = ref fa, b = ref fb;
            if (Mem.Add(ref boxStamp, sa) == stamp) { a = ref Mem.Add(ref boxes, 6 * sa); }
            if (Mem.Add(ref boxStamp, sb) == stamp) { b = ref Mem.Add(ref boxes, 6 * sb); }
            bool apart = (a - gap > Mem.Add(ref b, 3)) | (b - gap > Mem.Add(ref a, 3))
                | (Mem.Add(ref a, 1) - gap > Mem.Add(ref b, 4)) | (Mem.Add(ref b, 1) - gap > Mem.Add(ref a, 4))
                | (Mem.Add(ref a, 2) - gap > Mem.Add(ref b, 5)) | (Mem.Add(ref b, 2) - gap > Mem.Add(ref a, 5));
            // Status 2 when the fat boxes no longer overlap (the pair goes), else 1; count 0 (a survivor's count and first are set in phase 2; first is never read for a count of 0).
            Mem.Add(ref resStatus, i) = (byte)(fatOk ? 1 : 2);
            Mem.Add(ref resCount, i) = 0; Mem.Add(ref resFirst, i) = 0;
            // The apply list takes a pair that goes, and a tight-box exit with a constraint (or sensor overlap) to remove; survivors are decided in phase 2.
            bool list = !fatOk | (apart & Mem.Add(ref pairCons, p) != -1);
            listed |= (ulong)(list ? 1 : 0) << (i - lo);
            // Survivors: written unconditionally, kept by advancing n.
            Mem.Add(ref surv, n) = i; Mem.Add(ref survSpec, n) = pairSpec;
            n += fatOk & !apart ? 1 : 0;
        }

        sc.SurvivorCount = n;
        return listed;
    }

    // BoxPhase with both box tests on four lanes (x, y, z and one lane that is masked off). A box's six doubles load as two overlapping quads: Min = (MinX MinY MinZ MaxX) at offset 0,
    // Max = (MaxX MaxY MaxZ next) at offset 3, which stays inside a tree node (the Box is followed by the node's ints) and inside _boxes (two doubles of padding). The lane compares are the scalar
    // ones (ordered, so a NaN compares false either way) on the same doubles, so the decisions are identical to BoxPhase's.
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static ulong BoxPhase256(Engine3D e, int lo, int hi, NarrowScratch sc)
    {
        ref int upd = ref Mem.Ref(e._updateList);
        ref int pairA = ref Mem.Ref(e.PairA), pairB = ref Mem.Ref(e.PairB);
        ref int proxy = ref Mem.Ref(e.Proxy), pairCons = ref Mem.Ref(e.PairCons);
        ref int boxStamp = ref Mem.Ref(e._boxStamp);
        ref int resCount = ref Mem.Ref(e._resCount);
        ref byte resStatus = ref Mem.Ref(e._resStatus);
        ref int surv = ref Mem.Ref(sc.Survivor);
        ref float survSpec = ref Mem.Ref(sc.SurvivorSpec);
        ref float reachSum = ref Mem.Ref(e.ReachSum);
        ref double boxes = ref Mem.Ref(e._boxes);
        ref int owner = ref Mem.Ref(e.Core.Shapes.Owner);
        var bp = e.Bp;
        var trees = new TreeNodes();
        trees[0] = bp.TreeAt(0).NodeArray; trees[1] = bp.TreeAt(1).NodeArray; trees[2] = bp.TreeAt(2).NodeArray;
        int stamp = e._stamp, n = 0;
        float spec = e.Opt.SpeculativeDistance, fast = (float)FastFactor * spec;
        const double Gap = 0.005;
        var gap = Vector256.Create(spec + Gap);
        ulong listed = 0;
        for (int i = lo; i < hi; i++)
        {
            int p = Mem.Add(ref upd, i);
            int sa = Mem.Add(ref pairA, p), sb = Mem.Add(ref pairB, p);
            int ka = Mem.Add(ref proxy, sa), kb = Mem.Add(ref proxy, sb);
            float pairSpec = spec;
            ref double fa = ref Mem.As<Aabb3D, double>(ref Mem.At(Mem.Add(ref trees[0], ka & 3), ka >> 2).Box);
            ref double fb = ref Mem.As<Aabb3D, double>(ref Mem.At(Mem.Add(ref trees[0], kb & 3), kb >> 2).Box);
            // 1 when the fat boxes overlap on x, y and z (bits 0-2 all set), else 0.
            int fatOk = (int)((Vector256.ExtractMostSignificantBits(Vector256.LessThanOrEqual(Vector256.LoadUnsafe(ref fa), Vector256.LoadUnsafe(ref fb, 3))
                & Vector256.LessThanOrEqual(Vector256.LoadUnsafe(ref fb), Vector256.LoadUnsafe(ref fa, 3))) & 7) + 1) >> 3;
            int ba = Mem.Add(ref owner, sa), bb = Mem.Add(ref owner, sb);
            // PairSpeculativeDistance, its fast path inline: a pair of slow bodies is searched to the fixed distance. The rare call comes before any vector is loaded for the tight test, so no
            // vector is live across it (a live one would be spilled on every pair).
            var g = gap;
            if (fatOk != 0 && Mem.Add(ref reachSum, ba) + Mem.Add(ref reachSum, bb) > fast) { pairSpec = e.FastPairSpeculativeDistance(ba, bb, spec); g = Vector256.Create(pairSpec + Gap); }
            // Tight boxes (this step's for a shape that moved, the stored fat box for one that did not); the extra margin keeps the shortcut clear of rounding at the threshold.
            ref double a = ref fa, b = ref fb;
            if (Mem.Add(ref boxStamp, sa) == stamp) { a = ref Mem.Add(ref boxes, 6 * sa); }
            if (Mem.Add(ref boxStamp, sb) == stamp) { b = ref Mem.Add(ref boxes, 6 * sb); }
            // 1 when the tight boxes are apart by more than the gap on some axis (any of bits 0-2), else 0.
            int apart = (int)((Vector256.ExtractMostSignificantBits(Vector256.GreaterThan(Vector256.LoadUnsafe(ref a) - g, Vector256.LoadUnsafe(ref b, 3))
                | Vector256.GreaterThan(Vector256.LoadUnsafe(ref b) - g, Vector256.LoadUnsafe(ref a, 3))) & 7) + 7) >> 3;
            // Status 2 when the fat boxes no longer overlap (the pair goes), else 1; count 0. A survivor's first slot is set in phase 2; an early-out's is never read (its count is 0).
            Mem.Add(ref resStatus, i) = (byte)(2 - fatOk);
            Mem.Add(ref resCount, i) = 0;
            // The apply list takes a pair that goes, and a tight-box exit with a constraint (or sensor overlap) to remove; survivors are decided in phase 2.
            int hasCons = Mem.Add(ref pairCons, p) != -1 ? 1 : 0;
            listed |= (ulong)((fatOk ^ 1) | (fatOk & apart & hasCons)) << (i - lo);
            // Survivors: written unconditionally, kept by advancing n.
            Mem.Add(ref surv, n) = i; Mem.Add(ref survSpec, n) = pairSpec;
            n += fatOk & (apart ^ 1);
        }

        sc.SurvivorCount = n;
        return listed;
    }

    // The three trees' node arrays of the box phase, indexed by a proxy key's low two bits (slot 3 unused).
    [InlineArray(4)]
    private struct TreeNodes { private DynamicTree<Aabb3D>.Node[] _e; }

    // Phase 2 of a narrowphase chunk, for a pair that survived BoxPhase (its boxes overlap; status 1, count 0 already written): a pair's collide step from the pose loads on.
    // The index and pose loads are unchecked (Mem.At): the indices are the ones BoxPhase used, and a live body's pose lanes lie inside the store's arrays (3 and 4 lanes a body in 3D).
    private bool CollideSurvivor(int i, NarrowScratch sc, float pairSpec)
    {
        int p = Mem.At(_updateList, i);
        Mem.At(_resFirst, i) = sc.OutCount;
        int sa = Mem.At(PairA, p), sb = Mem.At(PairB, p);
        ref int owner = ref Mem.Ref(Core.Shapes.Owner);
        int ba = Mem.Add(ref owner, sa), bb = Mem.Add(ref owner, sb);
        var bodies = Core.Bodies;
        double[] pos = bodies.PositionArray;
        float[] rot = bodies.RotationArray;
        ReadOnlySpan<double> pA = Mem.ReadOnly(ref Mem.At(pos, 3 * ba), 3), pB = Mem.ReadOnly(ref Mem.At(pos, 3 * bb), 3);
        ref float ra = ref Mem.At(rot, 4 * ba), rb = ref Mem.At(rot, 4 * bb);
        var qA = new QuaternionF(ra, Mem.Add(ref ra, 1), Mem.Add(ref ra, 2), Mem.Add(ref ra, 3));
        var qB = new QuaternionF(rb, Mem.Add(ref rb, 1), Mem.Add(ref rb, 2), Mem.Add(ref rb, 3));
        // A pair without a constraint whose last query proved the shapes apart stays empty until their relative motion has used up the margin by which they were apart. Such a record only exists for a pair
        // that was searched, so the shapes are neither empty nor two triangle soups, and the check comes before the shapes are looked at.
        int existingCons = Mem.At(PairCons, p);
        if (existingCons < 0)
        {
            ref SkipFrame3 skip = ref Mem.At(_skip, p);
            if (skip.Clear > 0f && StillApart(in skip, qA, qB, (float)(pB[0] - pA[0]), (float)(pB[1] - pA[1]), (float)(pB[2] - pA[2]), skip.Spec - pairSpec)) { sc.Reached++; return false; }
        }

        var ga = Geoms[GeomOf[sa]]!;
        var gb = Geoms[GeomOf[sb]]!;
        if (ga.Convex.Kind == ConvexShapeKind3D.Empty || gb.Convex.Kind == ConvexShapeKind3D.Empty) { return false; }
        if (ga.Soup != null && gb.Soup != null && !(ga.Soup is SdfSoup3D && gb.Soup is SdfSoup3D)) { return false; }
        sc.Reached++;
        // A touching convex pair that has barely moved reuses its manifold; this is tried before the shape frames are built, which the reuse does not need.
        if (!ContactModifications.Refresh && RecycleManifolds && existingCons >= 0 && ga.Soup == null && gb.Soup == null && ga.IsConvex && gb.IsConvex
            && !HalfSpace3D.HasPlane(ga.Convex, gb.Convex)
            && TryRecycle(sc, existingCons, ba, bb, qA, qB, new Vector3F((float)(pB[0] - pA[0]), (float)(pB[1] - pA[1]), (float)(pB[2] - pA[2])), pairSpec))
        {
            _resCount[i] = 1;
            // MatchesInPlace decided here, from what is already loaded: the reuse is one contact on sub-shapes (0, 0) and replaces the pair's constraint in place when that is the pair's only
            // constraint, with the same sub-shape keys, neither shape is a sensor and no dynamic body of the pair sleeps (exactly MatchesInPlace's tests for a single result). Status 3 records it.
            // Such a pair is also written here, straight into its constraint's payload (WriteRecycled, while the payload is still in cache from TryRecycle), instead of by WriteTask after
            // the serial pass; its scratch slot is handed back (OutCount--), so the reuses of a chunk all go through one hot slot. This is safe because between here and WriteTask nothing reads or
            // moves this constraint's payload: the other pairs' narrowphase reads only their own constraints, the serial pass does not visit the pair (not listed) and only adds, removes and wakes
            // other pairs' constraints (slots are stable: removal clears only the removed slot), and the write reads only the shapes' materials, which no stage changes. CollideTask still reads the
            // slot's Recycled and Count right after this call (Account; a reuse is never queued), before the next survivor can reuse it; _resFirst / _resCount of a status-3 pair are not read after that.
            if (_deferWrite && Mem.At(ConsNext, existingCons) < 0 && !IsSensor(sa) && !IsSensor(sb)
                && !(bodies.IsDynamic(ba) && !bodies.IsAwake(ba)) && !(bodies.IsDynamic(bb) && !bodies.IsAwake(bb)))
            {
                var cs = Core.Constraints;
                if (cs.SubShapeA[existingCons] == 0 && cs.SubShapeB[existingCons] == 0)
                {
                    WriteRecycled(cs, existingCons, sa, sb, in sc.Out[sc.OutCount - 1]);
                    sc.OutCount--;
                    Mem.At(_resStatus, i) = 3;
                }
            }

            return false;
        }

        // Two boxes go to the chunk's box queue (RunBoxJobs); every other fresh pair runs now.
        if (ga.Soup == null && gb.Soup == null && ga.IsConvex && gb.IsConvex && ga.Convex.Kind == ConvexShapeKind3D.Box && gb.Convex.Kind == ConvexShapeKind3D.Box) { return QueueBoxPair(i, sc, p, sa, sb, ga, gb, ba, bb, pA, pB, qA, qB, pairSpec); }
        CollideFresh(i, sc, p, sa, sb, ga, gb, ba, bb, pA, pB, qA, qB, pairSpec);
        return false;
    }

    // A fresh box–box pair, queued with what the scalar CollideFresh needs after its pose block.
    internal struct BoxJob
    {
        public int I, P, Sa, Sb, Ba, Bb;
        public float Spec;
        public QuaternionF QA, QB;
        public Geom3D Ga, Gb;
        public QuaternionF Rwa;
        public Vector3F ArmA, ComDiff;
        public RigidTransformF BInA;
        public EdgeHint3D Hint;
        public int Need;
        public float Limit;
    }

    // CollideFresh's pose block for a box–box pair; the rest of the query runs in RunBoxJobs once the chunk's axis tests are done.
    [MethodImpl(MethodImplOptions.NoInlining)]
    private bool QueueBoxPair(int i, NarrowScratch sc, int p, int sa, int sb, Geom3D ga, Geom3D gb, int ba, int bb, ReadOnlySpan<double> pA, ReadOnlySpan<double> pB, QuaternionF qA, QuaternionF qB, float spec)
    {
        sc.Fresh++;
        var la = Local[sa]; var lb = Local[sb];
        var lcA = LocalCenterOf(ba); var lcB = LocalCenterOf(bb);
        Vector3F armA = la.Translation.X == lcA.X && la.Translation.Y == lcA.Y && la.Translation.Z == lcA.Z ? default : qA.RotateInline(la.Translation - lcA);
        Vector3F armB = lb.Translation.X == lcB.X && lb.Translation.Y == lcB.Y && lb.Translation.Z == lcB.Z ? default : qB.RotateInline(lb.Translation - lcB);
        QuaternionF rwa = qA.Times(la.Rotation), rwb = qB.Times(lb.Rotation);
        double dx = (pB[0] - pA[0]) + ((double)armB.X - (double)armA.X);
        double dy = (pB[1] - pA[1]) + ((double)armB.Y - (double)armA.Y);
        double dz = (pB[2] - pA[2]) + ((double)armB.Z - (double)armA.Z);
        var inv = rwa.Conjugated();
        ref BoxJob j = ref sc.BoxJobs[sc.BoxCount++];
        j.I = i; j.P = p; j.Sa = sa; j.Sb = sb; j.Ba = ba; j.Bb = bb; j.Spec = spec; j.QA = qA; j.QB = qB; j.Ga = ga; j.Gb = gb;
        j.BInA = new RigidTransformF(inv.Times(rwb), inv.RotateInline(new Vector3F((float)dx, (float)dy, (float)dz)));
        j.Rwa = rwa; j.ArmA = armA; j.ComDiff = new Vector3F((float)(pA[0] - pB[0]), (float)(pA[1] - pB[1]), (float)(pA[2] - pB[2]));
        // Temporal coherence, as in CollideFresh: the hint from the pair's contact of the last step (constraints do not change during the narrowphase).
        int oldCons = PairCons[p];
        var cstore = Core.Constraints;
        j.Hint = oldCons >= 0 && cstore.PointCount[oldCons] > 0 ? new EdgeHint3D(cstore.Payload(oldCons).Slice(ManifoldLayout.HintOffset, 7)) : default;
        j.Need = ManifoldGenerator3D.BoxAxesNeed(ga.Convex, gb.Convex, j.BInA, spec, j.Hint, out j.Limit);
        // A pair whose bounding spheres are apart reads no axes (Collide returns before PolyPoly): it finishes now and leaves the queue, so it is not written to and read from it again.
        if (j.Need == 0) { sc.BoxCount--; FinishBoxJob(sc, ref j, NarrowScratch.Gen); return false; }
        return true;
    }

    // The chunk's queued box–box pairs. First the axis tests of every pair (faces of both boxes and the nine edge axes; ManifoldGenerator3D.BoxAxes), then per pair in queue order the rest of
    // CollideFresh with the generator taking the precomputed axes (Gen.Pre) instead of running BoxFaces/BoxEdges. The edge axes are computed for every pair; the generator still applies its edge hint
    // and ignores them when the hint skips the edge search, exactly as when it skipped computing them.
    private void RunBoxJobs(NarrowScratch sc)
    {
        int n = sc.BoxCount;
        BoxJob[] jobs = sc.BoxJobs;
        ManifoldGenerator3D.BoxAxes[] res = sc.BoxRes;
        ManifoldGenerator3D.BoxAxesBatch(jobs, res, n, KernelWidth == 8);
        var gen = NarrowScratch.Gen;
        for (int k = 0; k < n; k++)
        {
            gen.Pre = res[k]; gen.HavePre = true;
            FinishBoxJob(sc, ref jobs[k], gen);
        }
    }

    // The rest of CollideFresh for a queued box pair whose pose is in the job (with gen.Pre/HavePre set when its axes were computed ahead).
    private void FinishBoxJob(NarrowScratch sc, ref BoxJob j, ManifoldGenerator3D gen)
    {
        {
            int i = j.I, p = j.P;
            _resFirst[i] = sc.OutCount;
            var f = new PairFrame { Rwa = j.Rwa, ArmA = j.ArmA, ComDiff = j.ComDiff };
            float spec = j.Spec;
            if (spec > Opt.SpeculativeDistance)
            {
                var bodies = Core.Bodies;
                int ba = j.Ba, bb = j.Bb;
                var va = bodies.LinearVelocityOf(ba); var wa = bodies.AngularVelocityOf(ba); var vb = bodies.LinearVelocityOf(bb); var wb = bodies.AngularVelocityOf(bb);
                f.Approach = true; f.Spec = Opt.SpeculativeDistance; f.Dt = Core.ContactStepDt;
                f.VA = new Vector3F(va[0], va[1], va[2]); f.VB = new Vector3F(vb[0], vb[1], vb[2]);
                f.WA = bodies.TypeOf(ba) == BodyType.Dynamic ? new Vector3F(wa[0], wa[1], wa[2]) : default; f.WB = bodies.TypeOf(bb) == BodyType.Dynamic ? new Vector3F(wb[0], wb[1], wb[2]) : default;
            }

            gen.HintIn = j.Hint;
            var m = gen.Collide(j.Ga.Convex, j.Gb.Convex, j.BInA, spec);
            gen.HavePre = false;
            if (m.PointCount == 0 && gen.ClearanceOut > 0f)
            {
                var pos = Core.Bodies.PositionArray;
                int ba = j.Ba, bb = j.Bb;
                var invA = j.QA.Conjugated();
                var d = invA.RotateInline(new Vector3F((float)(pos[3 * bb] - pos[3 * ba]), (float)(pos[3 * bb + 1] - pos[3 * ba + 1]), (float)(pos[3 * bb + 2] - pos[3 * ba + 2])));
                var qr = invA.Times(j.QB);
                float sg = qr.W < 0f ? -1f : 1f;
                var axis = Local[j.Sa].Rotation.RotateInline(gen.ClearanceAxisOut);
                _skip[p] = new SkipFrame3 { Nx = axis.X, Ny = axis.Y, Nz = axis.Z, X = d.X, Y = d.Y, Z = d.Z, Qx = sg * qr.X, Qy = sg * qr.Y, Qz = sg * qr.Z, Qw = sg * qr.W, Clear = gen.ClearanceOut, Spec = spec, Reach = ReachOfShape(bb, j.Sb) };
            }
            else { _skip[p].Clear = 0f; }

            int before = sc.OutCount;
            Emit(sc, in f, 0, 0, in m, in NoTransform, false, in NoTransform, false, in NoTriangle);
            if (sc.OutCount > before) { sc.Out[before].Hint = gen.HintOut; }
            _resCount[i] = sc.OutCount - _resFirst[i];
        }
    }

    // The pairs that need a narrowphase run are a minority (most are skipped, found apart by their boxes or reuse their manifold), so the work for them is out of line: the early exits above

    // The pairs that need a narrowphase run are a minority (most are skipped, found apart by their boxes or reuse their manifold), so the work for them is out of line: the early exits above
    // then run in a frame that holds none of the shape frames and manifolds this part needs.
    [MethodImpl(MethodImplOptions.NoInlining)]
    private void CollideFresh(int i, NarrowScratch sc, int p, int sa, int sb, Geom3D ga, Geom3D gb, int ba, int bb, ReadOnlySpan<double> pA, ReadOnlySpan<double> pB, QuaternionF qA, QuaternionF qB, float spec)
    {
        sc.Fresh++;
        var la = Local[sa]; var lb = Local[sb];
        var lcA = LocalCenterOf(ba); var lcB = LocalCenterOf(bb);

        // Shape frames in world axes. Positions are subtracted in double first; only the (small) difference becomes float.
        // A shape placed at its body's centre of mass has no arm; turning a zero vector gives a zero vector exactly, so the common single-shape body skips the two rotations.
        Vector3F armA = la.Translation.X == lcA.X && la.Translation.Y == lcA.Y && la.Translation.Z == lcA.Z ? default : qA.RotateInline(la.Translation - lcA);
        Vector3F armB = lb.Translation.X == lcB.X && lb.Translation.Y == lcB.Y && lb.Translation.Z == lcB.Z ? default : qB.RotateInline(lb.Translation - lcB);
        QuaternionF rwa = qA.Times(la.Rotation), rwb = qB.Times(lb.Rotation);
        double dx = (pB[0] - pA[0]) + ((double)armB.X - (double)armA.X);
        double dy = (pB[1] - pA[1]) + ((double)armB.Y - (double)armA.Y);
        double dz = (pB[2] - pA[2]) + ((double)armB.Z - (double)armA.Z);
        var inv = rwa.Conjugated();
        var bInA = new RigidTransformF(inv.Times(rwb), inv.RotateInline(new Vector3F((float)dx, (float)dy, (float)dz)));
        var f = new PairFrame { Rwa = rwa, ArmA = armA, ComDiff = new Vector3F((float)(pA[0] - pB[0]), (float)(pA[1] - pB[1]), (float)(pA[2] - pB[2])) };
        if (spec > Opt.SpeculativeDistance)
        {
            // Points farther than the fixed distance are kept only as far as the surfaces at that point close in during the step (see Emit).
            var bodies = Core.Bodies;
            var va = bodies.LinearVelocityOf(ba); var wa = bodies.AngularVelocityOf(ba); var vb = bodies.LinearVelocityOf(bb); var wb = bodies.AngularVelocityOf(bb);
            f.Approach = true; f.Spec = Opt.SpeculativeDistance; f.Dt = Core.ContactStepDt;
            f.VA = new Vector3F(va[0], va[1], va[2]); f.VB = new Vector3F(vb[0], vb[1], vb[2]);
            f.WA = bodies.TypeOf(ba) == BodyType.Dynamic ? new Vector3F(wa[0], wa[1], wa[2]) : default; f.WB = bodies.TypeOf(bb) == BodyType.Dynamic ? new Vector3F(wb[0], wb[1], wb[2]) : default;
        }

        if (ga.Soup is SdfSoup3D fieldA && gb.Soup is SdfSoup3D fieldB)
        {
            SdfVsSdf(sc, in f, fieldA, fieldB, in bInA, spec);
        }
        else if (ga.Soup != null || gb.Soup != null)
        {
            FreshSoup(i, sc, p, sa, sb, ga, gb, ba, bb, pA, pB, qA, qB, in f, in la, in lb, in bInA, spec);
        }
        else if (ga.IsConvex && gb.IsConvex)
        {
            // Temporal coherence: the hint from the pair's contact of the last step, if it has one, goes in; what the test learned this time goes out with the new manifold.
            var gen = NarrowScratch.Gen;
            int oldCons = PairCons[p];
            var cstore = Core.Constraints;
            gen.HintIn = oldCons >= 0 && cstore.PointCount[oldCons] > 0 ? new EdgeHint3D(cstore.Payload(oldCons).Slice(ManifoldLayout.HintOffset, 7)) : default;
            var m = gen.Collide(ga.Convex, gb.Convex, bInA, spec);
            if (m.PointCount == 0 && gen.ClearanceOut > 0f)
            {
                // The pose of B's body in A's body at this query, and how far apart the shapes were.
                var invA = qA.Conjugated();
                var d = invA.RotateInline(new Vector3F((float)(pB[0] - pA[0]), (float)(pB[1] - pA[1]), (float)(pB[2] - pA[2])));
                var qr = invA.Times(qB);
                float sg = qr.W < 0f ? -1f : 1f;
                var axis = la.Rotation.RotateInline(gen.ClearanceAxisOut);
                _skip[p] = new SkipFrame3 { Nx = axis.X, Ny = axis.Y, Nz = axis.Z, X = d.X, Y = d.Y, Z = d.Z, Qx = sg * qr.X, Qy = sg * qr.Y, Qz = sg * qr.Z, Qw = sg * qr.W, Clear = gen.ClearanceOut, Spec = spec, Reach = ReachOfShape(bb, sb) };
            }
            else { _skip[p].Clear = 0f; }

            int before = sc.OutCount;
            Emit(sc, in f, 0, 0, in m, in NoTransform, false, in NoTransform, false, in NoTriangle);
            if (sc.OutCount > before) { sc.Out[before].Hint = gen.HintOut; }
        }
        else
        {
            FreshCompound(sc, ga, gb, in f, in bInA, spec);
        }

        _resCount[i] = sc.OutCount - _resFirst[i];
    }

    // The branches below are rarely taken, so they live out of line: CollideFresh keeps a small frame (it used to hold the locals of every branch, 1.4 KB, and cleared 800 bytes of them on each call).
    [MethodImpl(MethodImplOptions.NoInlining)]
    private void FreshSoup(int i, NarrowScratch sc, int p, int sa, int sb, Geom3D ga, Geom3D gb, int ba, int bb, ReadOnlySpan<double> pA, ReadOnlySpan<double> pB, QuaternionF qA, QuaternionF qB,
        in PairFrame f, in RigidTransformF la, in RigidTransformF lb, in RigidTransformF bInA, float spec)
    {
        bool soupIsA = ga.Soup != null;
        var other = soupIsA ? gb : ga;
        // Temporal coherence for a convex shape on a triangle soup: while it has hardly moved relative to the soup, every triangle contact of the pair keeps its manifold (the pair's triangle set can only
        // change by a triangle coming within speculative range, which the next full query, at most a few millimetres of motion later, finds).
        if (!ContactModifications.Refresh && RecycleManifolds && other.IsConvex && PairCons[p] >= 0 && TryRecycleSoup(sc, p, ba, bb, qA, qB, new Vector3F((float)(pB[0] - pA[0]), (float)(pB[1] - pA[1]), (float)(pB[2] - pA[2])), soupIsA ? 1 : 2, spec))
        {
            // The pair reused its manifolds, so it did not run the narrowphase after all.
            sc.Fresh--;
            _resCount[i] = sc.OutCount - _resFirst[i];
            return;
        }

        sc.SdfMoves = Core.Bodies.TypeOf(soupIsA ? ba : bb) != BodyType.Static;
        if (soupIsA) { SoupAgainst(sc, in f, ga.Soup!, true, gb, in lb, in bInA, spec); }
        else { SoupAgainst(sc, in f, gb.Soup!, false, ga, in la, in bInA, spec); }
        if (Core.ContactMaterials.IsPresent)
        {
            for (int entry = _resFirst[i]; entry < sc.OutCount; entry++)
            {
                ref var output = ref sc.Out[entry];
                if (!output.FlatPatch) continue;
                if (soupIsA) output.SubA = Core.ContactMaterials.CanonicalId(sa, output.OriginalSubA, output.SubA);
                else output.SubB = Core.ContactMaterials.CanonicalId(sb, output.OriginalSubB, output.SubB);
            }
        }
        MergeFlatPatches(sc, _resFirst[i], soupIsA, (soupIsA ? qB : qA).Conjugated());
        _resCount[i] = sc.OutCount - _resFirst[i];
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void FreshCompound(NarrowScratch sc, Geom3D ga, Geom3D gb, in PairFrame f, in RigidTransformF bInA, float spec)
    {
            NarrowScratch.Children.Clear();
            if (ga.Compound != null && gb.IsConvex) { NarrowScratch.Gen.CollideCompoundConvex(ga.Compound, gb.Convex, bInA, NarrowScratch.Children, spec); }
            else if (ga.IsConvex && gb.Compound != null) { NarrowScratch.Gen.CollideConvexCompound(ga.Convex, gb.Compound, bInA, NarrowScratch.Children, spec); }
            else { NarrowScratch.Gen.CollideCompoundCompound(ga.Compound!, gb.Compound!, bInA, NarrowScratch.Children, spec); }
            foreach (var cc in NarrowScratch.Children)
            {
                var m = cc.Manifold;
                Emit(sc, in f, cc.SubShapeA, cc.SubShapeB, in m, in NoTransform, false, in NoTransform, false, in NoTriangle);
            }
    }


    /// <summary>The pose of B's body in A's body when a pair without a constraint last had no contact points, and how far apart the shapes were then (beyond the speculative distance).</summary>
    internal struct SkipFrame3
    {
        public float X, Y, Z, Qx, Qy, Qz, Qw, Clear;
        /// <summary>The search distance of the query that found the pair apart; <see cref="Clear"/> is the gap beyond it.</summary>
        public float Spec;
        public float Nx, Ny, Nz; // optional separating direction in A's body frame

        /// <summary>Farthest point of B's shape from B's centre of mass; a kinematic or static body has no stored extent, so the bound comes from the shape.</summary>
        public float Reach;
    }

    private SkipFrame3[] _skip = new SkipFrame3[8];

    // Placeholders for the arguments Emit ignores outside a triangle-soup contact, so a call does not build and clear a fresh copy of each.
    private static readonly RigidTransformF NoTransform = Zeroed<RigidTransformF>();
    private static readonly SoupTriangle3D NoTriangle = Zeroed<SoupTriangle3D>();
    private static T Zeroed<T>() where T : struct => default;

    // The farthest point of a convex shape from its body's centre of mass, from the shape's bounds in the body frame.
    private float ReachOfShape(int b, int s)
    {
        ShapeBounds3D.Compute(Geoms[GeomOf[s]]!.Convex, Local[s], out var mn, out var mx);
        float cx = LocalCenter[3 * b], cy = LocalCenter[3 * b + 1], cz = LocalCenter[3 * b + 2];
        float dx = Math.Max(Math.Abs(mn.X - cx), Math.Abs(mx.X - cx)), dy = Math.Max(Math.Abs(mn.Y - cy), Math.Abs(mx.Y - cy)), dz = Math.Max(Math.Abs(mn.Z - cz), Math.Abs(mx.Z - cz));
        return float.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    // Whether a pair recorded as apart still is. Every point of B's shape moves, relative to A, by at most the change of B's centre of mass in A's frame plus the chord of the relative turn times the point's distance
    // from that centre (chord = 2 sin(angle / 2) = twice the vector part of the relative rotation). The slack covers float rounding of the relative pose, which grows with its size.
    internal static bool StillApart(in SkipFrame3 f, QuaternionF qA, QuaternionF qB, float dx, float dy, float dz, float searchChange = 0f)
    {
        var invA = qA.Conjugated();
        var d = invA.RotateInline(new Vector3F(dx, dy, dz));
        float ex = d.X - f.X, ey = d.Y - f.Y, ez = d.Z - f.Z;
        var qr = invA.Times(qB);
        float sg = qr.W < 0f ? -1f : 1f;
        float x = sg * qr.X, y = sg * qr.Y, z = sg * qr.Z, w = sg * qr.W;
        // Vector part of qr * conj(q0), with q0 = (Qx, Qy, Qz, Qw) the stored relative rotation (Qw >= 0).
        float rx = w * -f.Qx + f.Qw * x + (y * -f.Qz - z * -f.Qy);
        float ry = w * -f.Qy + f.Qw * y + (z * -f.Qx - x * -f.Qz);
        float rz = w * -f.Qz + f.Qw * z + (x * -f.Qy - y * -f.Qx);
        float turn = 2f * float.Sqrt(rx * rx + ry * ry + rz * rz);
        float slack = 0.001f + 1e-6f * (float.Abs(dx) + float.Abs(dy) + float.Abs(dz) + f.Reach);
        float axis2 = f.Nx * f.Nx + f.Ny * f.Ny + f.Nz * f.Nz;
        float used;
        if (axis2 > 0.99f && axis2 < 1.01f)
        {
            // Catto 2013, continuous collision: project motion onto a separating plane.
            // Only translation towards the supporting plane closes its gap; tangential motion
            // cannot. Turning B can close it by at most the chord times B's reach and |axis|.
            float closing = -(ex * f.Nx + ey * f.Ny + ez * f.Nz);
            used = closing + turn * f.Reach * float.Sqrt(axis2) + slack;
        }
        else { used = float.Sqrt(ex * ex + ey * ey + ez * ez) + turn * f.Reach + slack; }
        // The gap now needed is the one the pair is searched to now, so a longer search leaves less room (searchChange is the old distance minus the new one).
        return used <= f.Clear + searchChange; // false for NaN
    }

    /// <summary>Contacts of a triangle soup (mesh or height field) with a convex shape or a compound. <paramref name="soupIsA"/> says which side of the pair the soup is.</summary>
    private static void SoupAgainst(NarrowScratch sc, in PairFrame f, TriangleSoup3D soup, bool soupIsA, Geom3D other, in RigidTransformF otherLocal, in RigidTransformF bInA, float spec)
    {
        // The other shape's frame in the soup's frame.
        RigidTransformF otherInSoup = soupIsA ? bInA : bInA.Inverted();
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
        // A signed distance field collides as a field, not as its triangles.
        if (soup is SdfSoup3D field) { SdfVsConvex(sc, in f, field, soupIsA, in shape, in shapeLocal, hasChild, in shapeInSoup, in soupFromA, spec, child); return; }
        // The per-thread buffers are looked up once, not once per triangle.
        var tris = NarrowScratch.Tris;
        var triBuf = NarrowScratch.TriBuf;
        var gen = NarrowScratch.Gen;
        tris.Clear();
        soup.CollectCandidates(shape, shapeInSoup, spec, tris, NarrowScratch.Ids);
        // The shape in the soup's frame is the same for every triangle; with the soup as B the manifold is computed in the shape's frame, which needs the inverse.
        var soupInShape = soupIsA ? default : shapeInSoup.Inverted();
        // Every triangle is tested against the same shape in the same pose, so the shape's polyhedron is built once for the pair, not once per triangle.
        gen.BeginPin(soupIsA ? 2 : 1);
        // For a child of a compound the manifold comes back in the child's frame; mapToA moves it into the compound's frame (only when the compound is shape A).
        for (int t = 0; t < tris.Count; t++)
        {
            var tri = tris[t];
            var tshape = triBuf.Set(tri.V0, tri.V1, tri.V2);
            Manifold3D m;
            int subA, subB;
            bool map = hasChild && !soupIsA;
            if (soupIsA)
            {
                m = gen.Collide(tshape, shape, shapeInSoup, spec);
                subA = tri.Id; subB = child < 0 ? 0 : child;
            }
            else
            {
                m = gen.Collide(shape, tshape, soupInShape, spec);
                subA = child < 0 ? 0 : child; subB = tri.Id;
            }

            if (m.PointCount == 0) { continue; }
            Emit(sc, in f, subA, subB, in m, shapeLocal, map, soupFromA, true, tri, soupIsA, triBuf.Normal);
        }

        gen.EndPin();
    }

    private static void Emit(NarrowScratch sc, in PairFrame f, int subA, int subB, in Manifold3D m, in RigidTransformF mapToA, bool map, in RigidTransformF soupFromA, bool filter, in SoupTriangle3D tri,
        bool soupIsA = false, Vector3F triNormal = default)
    {
        if (m.PointCount == 0) { return; }
        Vector3F n = map ? mapToA.Rotation.RotateInline(m.Normal) : m.Normal;
        int count = m.PointCount;
        if (filter)
        {
            // Internal-edge rule (Rouwé's write-ups on internal edges, Gregorius on contact generation): a point whose closest triangle feature is an inactive edge, or a vertex whose
            // two edges in this triangle are both inactive, lies on a seam between flat neighbours. If the manifold's normal also leans away from the face normal, the neighbour's own
            // face contact is the right one, and this one would kick a body sliding over the seam: the manifold is dropped. Contacts on the back of a triangle are not filtered.
            Vector3F triN = triNormal;
            Vector3F towards = soupIsA ? n : soupFromA.Rotation.RotateInline(-n); // from the triangle towards the other shape, in the soup's frame
            float d = Vector3F.Dot(towards, triN);
            if (d >= -0.1f && d < FlatPatchCosine)
            {
                for (int k = 0; k < count; k++)
                {
                    Vector3F pt = map ? mapToA.TransformPoint(m[k].Point) : m[k].Point;
                    if (IsGhostRegion(soupIsA ? pt : soupFromA.TransformPoint(pt), in tri)) { return; }
                }
            }
        }

        ref ContactOut o = ref sc.Next();
        o.OriginalSubA = subA; o.OriginalSubB = subB; o.SubA = subA; o.SubB = subB; o.Hint = default; o.Recycled = false; o.FlatPatch = false; o.FlatPlane = 0;
        if (filter && tri.SharedPatch)
        {
            Vector3F towards = soupIsA ? n : soupFromA.Rotation.RotateInline(-n);
            if (Vector3F.Dot(towards, triNormal) >= FlatPatchCosine)
            {
                o.FlatPatch = true; o.FlatPlane = Vector3F.Dot(triNormal, tri.V0);
                if (soupIsA) o.SubA = tri.CoplanarPatchId; else o.SubB = tri.CoplanarPatchId;
                n = soupIsA ? triNormal : -soupFromA.Rotation.Conjugated().RotateInline(triNormal);
            }
        }
        Vector3F nw = f.Rwa.RotateInline(n);
        o.Nx = nw.X; o.Ny = nw.Y; o.Nz = nw.Z;
        int written = 0;
        for (int k = 0; k < count; k++)
        {
            var mp = m[k];
            Vector3F pt = map ? mapToA.TransformPoint(mp.Point) : mp.Point;
            Vector3F ra = f.ArmA + f.Rwa.RotateInline(pt);
            Vector3F rb = ra + f.ComDiff;
            if (f.Approach && mp.Separation > f.Spec)
            {
                // The pair was searched beyond the fixed distance. A point beyond it stays only if the surfaces there close in by more than the gap during the step: the speed at which B's point
                // approaches A's along the contact normal, from both bodies' linear and angular velocities.
                Vector3F vrel = f.VB + Vector3F.Cross(f.WB, rb) - f.VA - Vector3F.Cross(f.WA, ra);
                float closing = -(vrel.X * nw.X + vrel.Y * nw.Y + vrel.Z * nw.Z);
                if (!(mp.Separation <= f.Spec + (closing > 0f ? closing * f.Dt : 0f))) { continue; }
            }

            int q = written * 7;
            o.P[q] = ra.X; o.P[q + 1] = ra.Y; o.P[q + 2] = ra.Z; o.P[q + 3] = rb.X; o.P[q + 4] = rb.Y; o.P[q + 5] = rb.Z; o.P[q + 6] = mp.Separation;
            o.Id[written] = mp.Id;
            written++;
        }

        if (written == 0) { sc.OutCount--; return; }
        o.Count = written;
    }

    // A manifold whose normal agrees with the triangle's own normal to within this cosine (under 0.3 degrees) needs no treatment.
    private const float GhostCosine = 0.99999f;

    // A manifold on a flat mesh whose normal is within this cosine (about 1.8 degrees) of the triangle's normal is that triangle's face contact. A hull lying a degree off flat gets a normal from its
    // own faces or edges, a hair off the floor's, and every one of its contacts near a seam used to be dropped as a seam contact, leaving the shape no contact at all over a whole cell.
    private const float FlatPatchCosine = 0.9995f;

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
        int matchedMask = 0;
        var old = default(ImpulseBuffer);
        for (int k = 0; k < res.Count; k++)
        {
            uint id = res.Id[k];
            for (int j = 0; j < oldCount; j++)
            {
                if (ids[j] != id) { continue; }
                matchedMask |= 1 << k;
                old[k] = Mem.At(pl, ManifoldLayout.NormalImpulse(j)); old[4 + k] = Mem.At(pl, ManifoldLayout.Tangent1Impulse(j)); old[8 + k] = Mem.At(pl, ManifoldLayout.Tangent2Impulse(j));
                break;
            }
        }

        Core.InspectContactBirth(c, res.Count, matchedMask);
        float r0 = 0f, r1 = 0f, r2 = 0f;
        if (oldCount > 0) { r0 = Mem.At(pl, ManifoldLayout.RollingImpulse); r1 = Mem.At(pl, ManifoldLayout.RollingImpulse + 1); r2 = Mem.At(pl, ManifoldLayout.RollingImpulse + 2); }
        pl.Clear();
        var mixed = Mix(sa, sb, cs.SubShapeA[c], cs.SubShapeB[c]);
        Mem.At(pl, ManifoldLayout.Normal) = res.Nx; Mem.At(pl, ManifoldLayout.Normal + 1) = res.Ny; Mem.At(pl, ManifoldLayout.Normal + 2) = res.Nz;
        Mem.At(pl, ManifoldLayout.Friction) = mixed.Friction;
        Mem.At(pl, ManifoldLayout.Restitution) = mixed.Restitution;
        Mem.At(pl, ManifoldLayout.RollingResistance) = mixed.RollingResistance;
        Mem.At(pl, ManifoldLayout.RollingImpulse) = r0; Mem.At(pl, ManifoldLayout.RollingImpulse + 1) = r1; Mem.At(pl, ManifoldLayout.RollingImpulse + 2) = r2;
        for (int k = 0; k < res.Count; k++)
        {
            int q = 7 * k;
            int a = ManifoldLayout.AnchorA(k), b = ManifoldLayout.AnchorB(k);
            Mem.At(pl, a) = res.P[q]; Mem.At(pl, a + 1) = res.P[q + 1]; Mem.At(pl, a + 2) = res.P[q + 2];
            Mem.At(pl, b) = res.P[q + 3]; Mem.At(pl, b + 1) = res.P[q + 4]; Mem.At(pl, b + 2) = res.P[q + 5];
            Mem.At(pl, ManifoldLayout.Separation(k)) = res.P[q + 6];
            Mem.At(pl, ManifoldLayout.NormalImpulse(k)) = old[k];
            Mem.At(pl, ManifoldLayout.Tangent1Impulse(k)) = old[4 + k];
            Mem.At(pl, ManifoldLayout.Tangent2Impulse(k)) = old[8 + k];
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
            case ConvexShapeKind3D.TaperedCylinder:
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
        if (soup == 0 && !IsFaceNormal(ga.Convex, qA.Times(Local[sa].Rotation).Conjugated().RotateInline(n)) && IsFaceNormal(gb.Convex, qB.Times(Local[sb].Rotation).Conjugated().RotateInline(-n))) { frame = 2; }
        Mem.At(pl, Rc.Owner) = frame;
        var nl = (frame == 1 ? qA : qB).Conjugated().RotateInline(n);
        Mem.At(pl, Rc.Normal) = nl.X; Mem.At(pl, Rc.Normal + 1) = nl.Y; Mem.At(pl, Rc.Normal + 2) = nl.Z;
        var invA = qA.Conjugated();
        var dW = new Vector3F((float)(pos[3 * bb] - pos[3 * ba]), (float)(pos[3 * bb + 1] - pos[3 * ba + 1]), (float)(pos[3 * bb + 2] - pos[3 * ba + 2]));
        // The relative pose is kept as the other body's in the frame of the reference body: A, or the soup's body when B is the soup (the soup's own points are fixed in its frame, so only the other body's move).
        var invR = soup == 2 ? qB.Conjugated() : invA;
        var d = invR.RotateInline(soup == 2 ? -dW : dW);
        Mem.At(pl, Rc.Rel) = d.X; Mem.At(pl, Rc.Rel + 1) = d.Y; Mem.At(pl, Rc.Rel + 2) = d.Z;
        var qr = invR.Times(soup == 2 ? qA : qB);
        Mem.At(pl, Rc.Rel + 3) = qr.X; Mem.At(pl, Rc.Rel + 4) = qr.Y; Mem.At(pl, Rc.Rel + 5) = qr.Z; Mem.At(pl, Rc.Rel + 6) = qr.W;
        var invB = qB.Conjugated();
        for (int k = 0; k < res.Count; k++)
        {
            int q = k * 7, o = Rc.Points + 7 * k;
            var la = invA.RotateInline(new Vector3F(res.P[q], res.P[q + 1], res.P[q + 2]));
            var lb = invB.RotateInline(new Vector3F(res.P[q + 3], res.P[q + 4], res.P[q + 5]));
            Mem.At(pl, o) = la.X; Mem.At(pl, o + 1) = la.Y; Mem.At(pl, o + 2) = la.Z; Mem.At(pl, o + 3) = lb.X; Mem.At(pl, o + 4) = lb.Y; Mem.At(pl, o + 5) = lb.Z; Mem.At(pl, o + 6) = res.P[q + 6];
        }
    }

    /// <summary>Builds the pair's manifold from the one kept in the constraint's payload when B has moved and turned relative to A by less than the bound; false when the narrowphase has to run.</summary>
    private bool TryRecycle(NarrowScratch sc, int c, int ba, int bb, QuaternionF qA, QuaternionF qB, Vector3F dW, float spec, int soup = 0)
    {
        var cs = Core.Constraints;
        int count = cs.PointCount[c];
        if (count == 0 || cs.KindOf(c) != ConstraintKind.Contact) { return false; }
        var pl = cs.Payload(c);
        // Compared as floats: snapshot bytes may hold anything here, and only 1 and 2 mean a stored frame.
        float ownerF = Mem.At(pl, Rc.Owner);
        int owner = ownerF == 1f ? 1 : ownerF == 2f ? 2 : 0;
        if (owner == 0) { return false; }
        // The reference body is A, or the soup's body when B is the soup; the other body is the mover (see StoreRecycleFrame).
        var invA = qA.Conjugated();
        var invR = soup == 2 ? qB.Conjugated() : invA;
        int mover = soup == 2 ? ba : bb;
        // How far the mover has moved and turned in the reference body's frame since the manifold was made.
        var d = invR.RotateInline(soup == 2 ? -dW : dW);
        float ex = d.X - Mem.At(pl, Rc.Rel), ey = d.Y - Mem.At(pl, Rc.Rel + 1), ez = d.Z - Mem.At(pl, Rc.Rel + 2);
        var q0 = new QuaternionF(Mem.At(pl, Rc.Rel + 3), Mem.At(pl, Rc.Rel + 4), Mem.At(pl, Rc.Rel + 5), Mem.At(pl, Rc.Rel + 6));
        float lim = RecycleTol - float.Sqrt(ex * ex + ey * ey + ez * ez); // what is left of the budget for the turn
        if (!(lim >= 0f)) { return false; } // the translation alone has used the budget (false for NaN too); the turn need not be measured
        var dq = invR.Times(soup == 2 ? qA : qB).Times(q0.Conjugated());
        float turn = 2f * float.Sqrt(dq.X * dq.X + dq.Y * dq.Y + dq.Z * dq.Z);
        if (!(turn * MaxExtent[mover] <= lim)) { return false; } // false for NaN too
        var nl = new Vector3F(Mem.At(pl, Rc.Normal), Mem.At(pl, Rc.Normal + 1), Mem.At(pl, Rc.Normal + 2));
        Vector3F n = (owner == 2 ? qB : qA).RotateInline(nl);
        ref ContactOut o = ref sc.Next();
        o.SubA = soup != 0 ? cs.SubShapeA[c] : 0; o.SubB = soup != 0 ? cs.SubShapeB[c] : 0; o.Hint = default; o.Recycled = true;
        for (int k = 0; k < count; k++)
        {
            int b = Rc.Points + 7 * k;
            Vector3F ra = qA.RotateInline(new Vector3F(Mem.At(pl, b), Mem.At(pl, b + 1), Mem.At(pl, b + 2)));
            Vector3F rb = qB.RotateInline(new Vector3F(Mem.At(pl, b + 3), Mem.At(pl, b + 4), Mem.At(pl, b + 5)));
            // Separation now: the one when made plus the motion of B's point relative to A's point along the normal.
            float sep = Mem.At(pl, b + 6) + Vector3F.Dot(dW + rb - ra, n);
            if (!(sep <= spec && sep >= -RecycleDepth)) { sc.OutCount--; return false; }
            int q = k * 7;
            o.P[q] = ra.X; o.P[q + 1] = ra.Y; o.P[q + 2] = ra.Z; o.P[q + 3] = rb.X; o.P[q + 4] = rb.Y; o.P[q + 5] = rb.Z; o.P[q + 6] = sep;
        }

        // The normal is already in world axes; Emit's rotation by the shape frame is not needed.
        o.Nx = n.X; o.Ny = n.Y; o.Nz = n.Z;
        o.Count = count;
        return true;
    }

    /// <summary>Reuses every triangle contact of a convex-on-soup pair, or none: false (with nothing emitted) as soon as one of them has moved too far.</summary>
    private bool TryRecycleSoup(NarrowScratch sc, int p, int ba, int bb, QuaternionF qA, QuaternionF qB, Vector3F dW, int soup, float spec)
    {
        int start = sc.OutCount;
        for (int c = PairCons[p]; c >= 0; c = ConsNext[c])
        {
            if (!TryRecycle(sc, c, ba, bb, qA, qB, dW, spec, soup)) { sc.OutCount = start; return false; }
        }

        return sc.OutCount > start;
    }

    private void WriteRecycled(ConstraintStore cs, int c, int sa, int sb, in ContactOut res)
    {
        var pl = cs.Payload(c);
        // A material can change between steps (a body's friction setter), and a fresh manifold would pick that up, so a reused one does too.
        var mixed = Mix(sa, sb, cs.SubShapeA[c], cs.SubShapeB[c]);
        Mem.At(pl, ManifoldLayout.Normal) = res.Nx; Mem.At(pl, ManifoldLayout.Normal + 1) = res.Ny; Mem.At(pl, ManifoldLayout.Normal + 2) = res.Nz;
        Mem.At(pl, ManifoldLayout.Friction) = mixed.Friction;
        Mem.At(pl, ManifoldLayout.Restitution) = mixed.Restitution;
        Mem.At(pl, ManifoldLayout.RollingResistance) = mixed.RollingResistance;
        for (int k = 0; k < res.Count; k++)
        {
            int q = 7 * k;
            int a = ManifoldLayout.AnchorA(k), b = ManifoldLayout.AnchorB(k);
            Mem.At(pl, a) = res.P[q]; Mem.At(pl, a + 1) = res.P[q + 1]; Mem.At(pl, a + 2) = res.P[q + 2];
            Mem.At(pl, b) = res.P[q + 3]; Mem.At(pl, b + 1) = res.P[q + 4]; Mem.At(pl, b + 2) = res.P[q + 5];
            Mem.At(pl, ManifoldLayout.Separation(k)) = res.P[q + 6];
        }
    }

    // Keep the ordinary contact path scalar: body-frame surface configuration is resolved separately.
    private ContactMaterial Mix(int sa, int sb, int subA = 0, int subB = 0)
    {
        float fa = Friction[sa], fb = Friction[sb], ra = Restitution[sa], rb = Restitution[sb], rra = Rolling[sa], rrb = Rolling[sb];
        if (Core.ContactMaterials.IsPresent)
        {
            var a = Core.ContactMaterials.Resolve(sa, subA, new ContactMaterial(fa, ra, rra));
            var b = Core.ContactMaterials.Resolve(sb, subB, new ContactMaterial(fb, rb, rrb));
            fa = a.Friction; fb = b.Friction; ra = a.Restitution; rb = b.Restitution; rra = a.RollingResistance; rrb = b.RollingResistance;
        }

        var mixer = Opt.MaterialMixer;
        if (mixer != null)
        {
            var mixed = mixer(new Material3D(fa, ra, rra), new Material3D(fb, rb, rrb));
            return new ContactMaterial(mixed.Friction, mixed.Restitution, mixed.RollingResistance);
        }

        // Friction: geometric mean (a slippery surface stays slippery); restitution and rolling resistance: the larger.
        float fr = float.Sqrt(fa * fb);
        float re = ra > rb ? ra : rb;
        float rr = rra > rrb ? rra : rrb;
        return new ContactMaterial(fr, re, rr);
    }
}
