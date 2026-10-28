using System;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    /// <summary>
    /// Sizes the pair table, the per-step lists, the solver rows and the broadphase scratch for up to the given counts, so steps within them allocate nothing. Never changes a result.
    /// Counts below what the world holds now are raised to it.
    /// </summary>
    public void Reserve(int pairs, int contacts, int bodies, int shapes)
    {
        bodies = Math.Max(bodies, Core.Bodies.HighWater);
        shapes = Math.Max(shapes, Core.Shapes.HighWater);
        pairs = Math.Max(pairs, PairHigh);
        EnsureBodies(bodies);
        EnsureShapes(shapes);
        if (Hot.Length < bodies) { Hot = new BodyHot[bodies]; }
        EnsureBoxStorage();
        Grow(ref Pairs, pairs); Grow(ref PairFree, pairs); Grow(ref SensorFree, pairs);
        PairMap.Reserve(pairs);
        // A restore reads into staging arrays and swaps them in, so they need the same room as the live ones.
        Fit(ref _sPa, pairs); Fit(ref _sPb, pairs); Fit(ref _sPc, pairs); Fit(ref _sPl, pairs); Fit(ref _sFree, pairs);
        if (_fresh.Length < pairs) { Array.Resize(ref _fresh, pairs); }
        int need = (pairs + 1023) / 1024 * 1024;
        if (_updateList.Length < need) { _updateList = new int[need]; _results = new PairResult[need]; _actItems = new int[need]; }
        if (_actCounts.Length < need / 64) { _actCounts = new int[need / 64]; _recycledCounts = new int[need / 64]; }
        int chunks = (pairs + 63) / 64;
        if (_blockCounts.Length < chunks) { _blockCounts = new int[chunks]; }
        int boxTasks = (bodies + 63) / 64;
        if (_moverLists.Length < boxTasks) { Array.Resize(ref _moverLists, boxTasks); Array.Resize(ref _moverCounts, boxTasks); }
        for (int t = 0; t < boxTasks; t++) { _moverLists[t] ??= new int[64]; }

        // Solver rows: one group per batch of lanes, plus the part-filled batch every color can end with.
        int batches = contacts / W2D.Lanes + ConstraintColoring.ColorCount + 2;
        if (BodyState.Length < (bodies + 1) * BS2D.Stride) { BodyState = new float[(bodies + 1) * BS2D.Stride]; }
        if (Wide.Length < batches * W2D.GroupFloats)
        {
            Wide = new float[batches * W2D.GroupFloats]; WideIdx = new int[batches * W2D.IdxPerGroup]; WideCons = new int[batches * W2D.Lanes];
            GroupRestitution = new byte[batches]; GroupRolling = new byte[batches]; GroupPoints = new byte[batches];
        }

        if (SlotOf.Length < contacts) { SlotOf = new int[contacts]; JSlotOf = new int[contacts]; BatchSimple = new byte[contacts]; }
        if (_partOne.Length < contacts) { _partOne = new int[contacts]; _partTwo = new int[contacts]; }
        int blocks = (bodies + BeginBlock - 1) / BeginBlock;
        if (_beginMoving.Length < blocks) { _beginMoving = new int[blocks]; }
        EnsureJointSlots(contacts);
        if (JScratch.Length < contacts * JRow.Stride) { JScratch = new float[contacts * JRow.Stride]; }
        Bp.Reserve(shapes, pairs);
    }
}
