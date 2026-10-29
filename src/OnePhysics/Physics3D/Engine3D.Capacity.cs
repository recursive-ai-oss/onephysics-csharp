using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    /// <summary>
    /// Sizes the pair table, the per-step lists, the solver rows and the broadphase scratch for up to the given counts, and prepares the calling thread's narrowphase scratch, so steps within them allocate nothing.
    /// Never changes a result. Counts below what the world holds now are raised to it.
    /// </summary>
    public void Reserve(int pairs, int contacts, int bodies, int shapes)
    {
        bodies = Math.Max(bodies, Core.Bodies.HighWater);
        shapes = Math.Max(shapes, Core.Shapes.HighWater);
        pairs = Math.Max(pairs, PairHigh);
        EnsureBodies(bodies);
        EnsureShapes(shapes);
        Grow(ref PairA, pairs); Grow(ref PairB, pairs); Grow(ref PairCons, pairs); Grow(ref PairLive, pairs); Grow(ref PairFree, pairs); Grow(ref SensorFree, pairs);
        PairMap.Reserve(pairs);
        // A restore reads into staging arrays and swaps them in, so they need the same room as the live ones.
        Fit(ref _sPa, pairs); Fit(ref _sPb, pairs); Fit(ref _sPc, pairs); Fit(ref _sPl, pairs); Fit(ref _sFree, pairs); Fit(ref _sChainLen, pairs);
        Fit(ref _sConsNext, contacts); Fit(ref _sInChain, contacts);
        _sChains.EnsureCapacity(contacts);
        if (_fresh.Length < pairs) { Array.Resize(ref _fresh, pairs); }
        if (_skip.Length < pairs) { Array.Resize(ref _skip, pairs); }
        if (_updateList.Length < pairs) { _updateList = new int[pairs]; _resStatus = new byte[pairs]; _resFirst = new int[pairs]; _resCount = new int[pairs]; }
        int pchunks = (pairs + PairListChunk - 1) / PairListChunk;
        if (_pairChunkCount.Length < pchunks) { _pairChunkCount = new int[pchunks]; }
        int chunks = (pairs + 63) / 64;
        if (_applyCount.Length < chunks) { _applyCount = new int[chunks]; }
        if (_applyIdx.Length < chunks * 64) { _applyIdx = new int[chunks * 64]; }
        if (_scratch.Length < chunks)
        {
            int old = _scratch.Length;
            Array.Resize(ref _scratch, chunks);
            for (int i = old; i < _scratch.Length; i++) { _scratch[i] = new NarrowScratch(); }
        }

        if (_boxStamp.Length < shapes) { _boxes = new double[shapes * 6]; _boxStamp = new int[shapes]; _boxInside = new byte[shapes]; }
        if (_active.Length < bodies) { _active = new byte[bodies]; _isSleeper = new byte[bodies]; }
        if (_sleepers.Length < bodies) { _sleepers = new int[bodies]; }

        int batches = contacts / ConstraintColoring.Lanes + ConstraintColoring.ColorCount + 2;
        if (Wide.Length < batches * W3D.GroupFloats) { Wide = new float[batches * W3D.GroupFloats]; WideIdx = new int[batches * W3D.IdxPerGroup]; }
        if (SlotOf.Length < contacts) { SlotOf = new int[contacts]; }
        if (BatchInfo.Length < 4 * contacts) { BatchInfo = new int[4 * contacts]; }
        if (_sortTmp.Length < contacts) { _sortTmp = new int[contacts]; }
        if (ConsNext.Length < contacts) { Array.Resize(ref ConsNext, contacts); }
        EnsureJointSlots(contacts);
        if (JScratch.Length < contacts * JRow3.Stride) { JScratch = new float[contacts * JRow3.Stride]; }
        Bp.Reserve(shapes, pairs);
        PrepareThread();
    }

    /// <summary>Creates the calling thread's narrowphase scratch (it is made on first use otherwise).</summary>
    internal static void PrepareThread()
    {
        var gen = NarrowScratch.Gen;
        NarrowScratch.Children.EnsureCapacity(64);
        NarrowScratch.Tris.EnsureCapacity(64);
        NarrowScratch.Ids.EnsureCapacity(256);
        _ = CastIds();
        QueryScratch.Warm(64); // batch queries run on the pool's threads too
        TriangleSoup3D.PrepareThread();

        // A few throwaway pairs, shallow and deep, so the generator's lazily grown buffers exist on this thread before it takes a pair from a step (which pairs a worker gets depends on timing).
        var box = ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f));
        var ball = ConvexShape3D.Sphere(0.5f);
        var capsule = ConvexShape3D.Capsule(0.5f, 0.25f);
        var cylinder = ConvexShape3D.Cylinder(0.5f, 0.5f);
        ReadOnlySpan<float> offsets = [0.9f, 0.4f, 0.1f];
        foreach (float x in offsets)
        {
            var at = new RigidTransformF(QuaternionF.Identity, new Vector3F(x, 0.1f, 0f));
            gen.Collide(box, box, at);
            gen.Collide(box, ball, at);
            gen.Collide(ball, box, at);
            gen.Collide(capsule, box, at);
            gen.Collide(box, capsule, at);
            gen.Collide(capsule, capsule, at);
            gen.Collide(cylinder, box, at);
            gen.Collide(cylinder, cylinder, at);
        }
    }
}
