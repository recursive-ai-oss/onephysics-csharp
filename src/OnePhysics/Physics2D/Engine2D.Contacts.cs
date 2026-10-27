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

    public void Prepare(WorldCore world, ReadOnlySpan<int> constraints, in StepContext context) => _e.Prepare(constraints, in context);

    public void IntegrateVelocities(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context) => _e.IntegrateVelocities(bodies, in context);

    public void SolveBatch(WorldCore world, ReadOnlySpan<int> constraints, SolvePhase phase, in StepContext context) => _e.SolveBatch(constraints, phase, in context);

    public void IntegratePositions(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context) => _e.IntegratePositions(bodies, in context);

    public void StoreResults(WorldCore world, ReadOnlySpan<int> constraints, int eventSlot, in StepContext context)
    {
        // Impulses are updated in place in each constraint's payload, so there is nothing to copy back; only hit events are raised.
        _e.EmitHits(world, constraints, eventSlot);
    }

    public float BodyMotion(WorldCore world, int body) => _e.BodyMotion(body);

    public void FinalizeBodies(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context) => _e.FinalizeBodies(bodies, in context);
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
            return owner[shapeA] != owner[shapeB] && _e.CanCollide(shapeA, shapeB);
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

        // 0. A changed filter re-evaluates the pair table before anything else (see Refilter).
        Refilter();

        // 1. Broadphase boxes of every body that moved last step. The displacement stretches the box ahead of the motion.
        foreach (int b in world.AwakeBodies)
        {
            if (!bodies.IsLive(b)) { continue; }
            for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
            {
                Bp.MoveProxy(Proxy[s], TightBox(b, in Geom[s]), Dp[2 * b], Dp[2 * b + 1]);
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
            Dq[2 * b] = 1f; Dq[2 * b + 1] = 0f;
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

        // 6. Apply serially in ascending pair order: this fixes the order constraints are added, which fixes their colors.
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
                c = world.AddConstraint(desc);
                PairCons[p] = c;
            }
            else
            {
                // A sleeping body that a moving kinematic body touches must wake.
                int ba = owner[sa], bb = owner[sb];
                if (bodies.IsDynamic(ba) && !bodies.IsAwake(ba)) { world.WakeIsland(bodies.Island[ba]); }
                if (bodies.IsDynamic(bb) && !bodies.IsAwake(bb)) { world.WakeIsland(bodies.Island[bb]); }
            }

            WriteManifold(world.Constraints, c, sa, sb, in res);
        }

        int ch = world.Constraints.HighWater;
        if (Scratch.Length < ch * ContactKernel2D.Stride) { Scratch = new float[Math.Max(ch, Scratch.Length / ContactKernel2D.Stride * 2) * ContactKernel2D.Stride]; }
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
            int lo = task * 64, hi = Math.Min(lo + 64, _e._updateCount);
            for (int i = lo; i < hi; i++) { _e.Collide(i); }
        }
    }

    private void Collide(int i)
    {
        int p = _updateList[i];
        ref PairResult res = ref _results[i];
        res = default;
        int sa = PairA[p], sb = PairB[p];
        if (!Aabb2D.Overlaps(Bp.GetFatBounds(Proxy[sa]), Bp.GetFatBounds(Proxy[sb]))) { res.Status = 2; return; }
        res.Status = 1;
        var owner = Core.Shapes.Owner;
        int ba = owner[sa], bb = owner[sb];
        var bodies = Core.Bodies;
        var pA = bodies.PositionOf(ba); var pB = bodies.PositionOf(bb);
        var rA = bodies.RotationOf(ba); var rB = bodies.RotationOf(bb);
        var qA = new Rot2(rA[0], rA[1]); var qB = new Rot2(rB[0], rB[1]);
        var lcA = new Vec2(LocalCenter[2 * ba], LocalCenter[2 * ba + 1]); var lcB = new Vec2(LocalCenter[2 * bb], LocalCenter[2 * bb + 1]);

        // Subtract the double positions first; only the (small) difference of the body origins becomes float.
        Vec2 wA = qA.Rotate(lcA), wB = qB.Rotate(lcB);
        double dx = (pB[0] - (double)wB.X) - (pA[0] - (double)wA.X);
        double dy = (pB[1] - (double)wB.Y) - (pA[1] - (double)wA.Y);
        var xf = Transform2.InvMul(new Transform2(default, qA), new Transform2(new Vec2((float)dx, (float)dy), qB));
        var m = Collide2D.Collide(in Geom[sa], in Geom[sb], xf, Constants2D.SpeculativeDistance);
        res.Count = m.PointCount;
        if (m.PointCount == 0) { return; }

        Vec2 n = qA.Rotate(m.Normal);
        res.Nx = n.X; res.Ny = n.Y;
        // Contact arms from each centre of mass; B's arm differs from A's by the centre-of-mass offset.
        Vec2 comDiff = new((float)(pA[0] - pB[0]), (float)(pA[1] - pB[1]));
        for (int k = 0; k < m.PointCount; k++)
        {
            var mp = m[k];
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
        int oldCount = cs.PointCount[c];
        float oldN0 = 0f, oldN1 = 0f, oldT0 = 0f, oldT1 = 0f;
        for (int k = 0; k < res.Count; k++)
        {
            uint id = k == 0 ? res.Id0 : res.Id1;
            for (int j = 0; j < oldCount; j++)
            {
                if (ids[j] != id) { continue; }
                if (k == 0) { oldN0 = pl[ManifoldLayout.NormalImpulse(j)]; oldT0 = pl[ManifoldLayout.Tangent1Impulse(j)]; }
                else { oldN1 = pl[ManifoldLayout.NormalImpulse(j)]; oldT1 = pl[ManifoldLayout.Tangent1Impulse(j)]; }
                break;
            }
        }

        float rolling = oldCount > 0 ? pl[ManifoldLayout.RollingImpulse] : 0f;
        pl.Clear();
        var mixed = Mix(sa, sb);
        pl[ManifoldLayout.Normal] = res.Nx; pl[ManifoldLayout.Normal + 1] = res.Ny;
        pl[ManifoldLayout.Friction] = mixed.Friction;
        pl[ManifoldLayout.Restitution] = mixed.Restitution;
        pl[ManifoldLayout.RollingResistance] = mixed.RollingResistance;
        pl[ManifoldLayout.RollingImpulse] = rolling;
        for (int k = 0; k < res.Count; k++)
        {
            float rax = k == 0 ? res.RAx0 : res.RAx1, ray = k == 0 ? res.RAy0 : res.RAy1, rbx = k == 0 ? res.RBx0 : res.RBx1, rby = k == 0 ? res.RBy0 : res.RBy1;
            pl[ManifoldLayout.AnchorA(k)] = rax; pl[ManifoldLayout.AnchorA(k) + 1] = ray;
            pl[ManifoldLayout.AnchorB(k)] = rbx; pl[ManifoldLayout.AnchorB(k) + 1] = rby;
            pl[ManifoldLayout.Separation(k)] = k == 0 ? res.Sep0 : res.Sep1;
            pl[ManifoldLayout.NormalImpulse(k)] = k == 0 ? oldN0 : oldN1;
            pl[ManifoldLayout.Tangent1Impulse(k)] = k == 0 ? oldT0 : oldT1;
            ids[k] = k == 0 ? res.Id0 : res.Id1;
        }

        for (int k = res.Count; k < ids.Length; k++) { ids[k] = 0; }
        cs.PointCount[c] = (byte)res.Count;
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
