using System;
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

        if (_boxStamp.Length < shapes) { _boxes = new double[shapes * 6]; _boxStamp = new int[shapes]; }
        if (_active.Length < bodies) { _active = new byte[bodies]; _isSleeper = new byte[bodies]; }
        if (_sleepers.Length < bodies) { _sleepers = new int[bodies]; }

        int batches = contacts / ConstraintColoring.Lanes + ConstraintColoring.ColorCount + 2;
        if (Wide.Length < batches * W3D.GroupFloats) { Wide = new float[batches * W3D.GroupFloats]; WideIdx = new int[batches * W3D.IdxPerGroup]; }
        if (SlotOf.Length < contacts) { SlotOf = new int[contacts]; }
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
        _ = NarrowScratch.Gen;
    }
}
