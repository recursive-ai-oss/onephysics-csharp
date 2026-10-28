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

    public void Prepare(WorldCore world, ReadOnlySpan<int> constraints, in StepContext context) => _e.Prepare(constraints, in context);

    public void IntegrateVelocities(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context) => _e.IntegrateVelocities(bodies, in context);

    public void SolveBatch(WorldCore world, ReadOnlySpan<int> constraints, SolvePhase phase, in StepContext context) => _e.SolveBatch(constraints, phase, in context);

    public void IntegratePositions(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context) => _e.IntegratePositions(bodies, in context);

    public void StoreResults(WorldCore world, ReadOnlySpan<int> constraints, int eventSlot, in StepContext context)
    {
        // Contact impulses are updated in place in each constraint's payload, so there is nothing to copy back; joints work out their reaction and may ask to break.
        _e.StoreJointResults(constraints, in context);
    }

    public float BodyMotion(WorldCore world, int body) => _e.BodyMotion(body);

    public void FinalizeBodies(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context) => _e.FinalizeBodies(bodies, in context);
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
    public float Nx, Ny, Nz;
    public PointBuffer P; // per point: rA (3), rB (3), separation
    public IdBuffer Id;
}

/// <summary>Working storage of one chunk of narrowphase work. Results never depend on which thread runs a chunk or on what an instance did before.</summary>
internal sealed class NarrowScratch
{
    public readonly ManifoldGenerator3D Gen = new();
    public readonly List<ChildContact3D> Children = new();
    public readonly List<SoupTriangle3D> Tris = new();
    public readonly List<int> Ids = new();
    public readonly TriangleShapeBuffer3D TriBuf = new();
    public ContactOut[] Out = new ContactOut[16];
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

        public bool ShouldCollide(int shapeA, int shapeB)
        {
            var owner = _e.Core.Shapes.Owner;
            return owner[shapeA] != owner[shapeB] && _e.CanCollide(shapeA, shapeB) && !_e.JointsPreventCollision(owner[shapeA], owner[shapeB]);
        }
    }

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

        // 1. Broadphase boxes of every body that moved last step. The displacement stretches the box ahead of the motion.
        foreach (int b in world.AwakeBodies)
        {
            if (!bodies.IsLive(b)) { continue; }
            for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
            {
                Bp.MoveProxy(Proxy[s], TightBox(b, s), Dp[3 * b], Dp[3 * b + 1], Dp[3 * b + 2]);
            }
        }

        // 2. Start-of-step pose, zero accumulated motion and world-axes inverse inertia for every body (sleeping bodies may wake during the stage).
        var pos = bodies.Position;
        var rot = bodies.Rotation;
        var invI = bodies.InverseInertia;
        var invMass = bodies.InverseMass;
        for (int b = 0; b < nb; b++)
        {
            if (!bodies.IsLive(b)) { continue; }
            StartPos[3 * b] = pos[3 * b]; StartPos[3 * b + 1] = pos[3 * b + 1]; StartPos[3 * b + 2] = pos[3 * b + 2];
            for (int k = 0; k < 4; k++) { StartRot[4 * b + k] = rot[4 * b + k]; }
            Dp[3 * b] = 0f; Dp[3 * b + 1] = 0f; Dp[3 * b + 2] = 0f;
            Dq[4 * b] = 0f; Dq[4 * b + 1] = 0f; Dq[4 * b + 2] = 0f; Dq[4 * b + 3] = 1f;
            RefreshWorldInertia(b, invMass, invI, rot);
        }

        // 3. New pairs from the broadphase, in its sorted order.
        var found = Bp.UpdatePairs(new PairFilter(this), world.Executor);
        var pairFilter = Opt.PairFilter;
        for (int i = 0; i < found.Length; i++)
        {
            int a = found[i].UserA, b = found[i].UserB;
            if (a > b) { (a, b) = (b, a); }
            if (PairMap.ContainsKey(PairKey(a, b))) { continue; }
            if (pairFilter != null && !pairFilter(WrapShape(a), WrapShape(b))) { continue; }
            AllocatePair(a, b);
        }

        // 4. Pairs that need fresh manifolds: at least one body awake (or a kinematic body in motion). Soup-against-soup pairs never collide.
        if (_updateList.Length < PairHigh)
        {
            int n = Math.Max(PairHigh, _updateList.Length * 2);
            _updateList = new int[n]; _resStatus = new byte[n]; _resFirst = new int[n]; _resCount = new int[n];
        }

        _updateCount = 0;
        var owner = world.Shapes.Owner;
        for (int p = 0; p < PairHigh; p++)
        {
            if (!PairLive[p]) { continue; }
            if (IsActive(bodies, owner[PairA[p]]) || IsActive(bodies, owner[PairB[p]])) { _updateList[_updateCount++] = p; }
        }

        // 5. Manifolds, in parallel over fixed chunks; each chunk writes only its own results.
        int chunks = (_updateCount + 63) / 64;
        if (_scratch.Length < chunks)
        {
            int old = _scratch.Length;
            Array.Resize(ref _scratch, Math.Max(chunks, old * 2));
            for (int i = old; i < _scratch.Length; i++) { _scratch[i] = new NarrowScratch(); }
        }

        for (int i = 0; i < chunks; i++) { _scratch[i].OutCount = 0; }
        world.Executor.For(chunks, new CollideTask(this));

        // 6. Apply serially in ascending pair order: this fixes the order constraints are added, which fixes their colors.
        var cs = world.Constraints;
        for (int i = 0; i < _updateCount; i++)
        {
            int p = _updateList[i];
            if (_resStatus[i] == 2)
            {
                for (int c = PairCons[p]; c >= 0;)
                {
                    int nx = ConsNext[c];
                    world.RemoveConstraint(c);
                    c = nx;
                }

                FreePair(p);
                continue;
            }

            ApplyPair(world, p, _scratch[i >> 6], _resFirst[i], _resCount[i], owner);
        }

        int ch = cs.HighWater;
        Grow(ref ConsNext, ch);
        if (Scratch.Length < ch * Stride3D.Constraint) { Scratch = new float[Math.Max(ch, Scratch.Length / Stride3D.Constraint * 2) * Stride3D.Constraint]; }
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

            WriteManifold(world.Constraints, c, sa, sb, in sc.Out[first + j]);
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

    private readonly struct CollideTask : IParallelBody
    {
        private readonly Engine3D _e;

        public CollideTask(Engine3D e) => _e = e;

        public void Run(int task)
        {
            int lo = task * 64, hi = Math.Min(lo + 64, _e._updateCount);
            var sc = _e._scratch[task];
            for (int i = lo; i < hi; i++) { _e.Collide(i, sc); }
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
        if (!Aabb3D.Overlaps(Bp.GetFatBounds(Proxy[sa]), Bp.GetFatBounds(Proxy[sb]))) { _resStatus[i] = 2; return; }
        _resStatus[i] = 1;
        var ga = Geoms[GeomOf[sa]]!;
        var gb = Geoms[GeomOf[sb]]!;
        if (ga.Soup != null && gb.Soup != null) { return; }
        var owner = Core.Shapes.Owner;
        int ba = owner[sa], bb = owner[sb];
        var bodies = Core.Bodies;
        var pA = bodies.PositionOf(ba); var pB = bodies.PositionOf(bb);
        var qA = RotationOfBody(ba); var qB = RotationOfBody(bb);
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

        if (ga.Soup != null) { SoupAgainst(sc, in f, ga.Soup, true, gb, in lb, in bInA, spec); }
        else if (gb.Soup != null) { SoupAgainst(sc, in f, gb.Soup, false, ga, in la, in bInA, spec); }
        else if (ga.IsConvex && gb.IsConvex)
        {
            var m = sc.Gen.Collide(ga.Convex, gb.Convex, bInA, spec);
            Emit(sc, in f, 0, 0, in m, default, false, default, false, default);
        }
        else
        {
            sc.Children.Clear();
            if (ga.Compound != null && gb.IsConvex) { sc.Gen.CollideCompoundConvex(ga.Compound, gb.Convex, bInA, sc.Children, spec); }
            else if (ga.IsConvex && gb.Compound != null) { sc.Gen.CollideConvexCompound(ga.Convex, gb.Compound, bInA, sc.Children, spec); }
            else { sc.Gen.CollideCompoundCompound(ga.Compound!, gb.Compound!, bInA, sc.Children, spec); }
            foreach (var cc in sc.Children)
            {
                var m = cc.Manifold;
                Emit(sc, in f, cc.SubShapeA, cc.SubShapeB, in m, default, false, default, false, default);
            }
        }

        _resCount[i] = sc.OutCount - _resFirst[i];
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
        sc.Tris.Clear();
        soup.GetCandidates(shape, shapeInSoup, spec, sc.Tris, sc.Ids);
        // For a child of a compound the manifold comes back in the child's frame; mapToA moves it into the compound's frame (only when the compound is shape A).
        for (int t = 0; t < sc.Tris.Count; t++)
        {
            var tri = sc.Tris[t];
            var tshape = sc.TriBuf.Set(tri.V0, tri.V1, tri.V2);
            Manifold3D m;
            int subA, subB;
            bool map = hasChild && !soupIsA;
            if (soupIsA)
            {
                m = sc.Gen.Collide(tshape, shape, shapeInSoup, spec);
                subA = tri.Id; subB = child < 0 ? 0 : child;
            }
            else
            {
                m = sc.Gen.Collide(shape, tshape, shapeInSoup.Inverse(), spec);
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
        o.SubA = subA; o.SubB = subB;
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
        if (region == 0) { return false; }
        if (region <= 3) { return !tri.IsEdgeActive(region - 1); }
        int v = region - 4;
        return !tri.IsEdgeActive((v + 2) % 3) && !tri.IsEdgeActive(v);
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

        for (int k = res.Count; k < ids.Length; k++) { ids[k] = 0; }
        cs.PointCount[c] = (byte)res.Count;
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
