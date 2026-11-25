using System;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class HandleTable
{
    /// <summary>Makes this table an independent copy of <paramref name="s"/>: the slots below the high-water mark and the free-slot heap (the scratch array is not state).</summary>
    internal void CopyFrom(HandleTable s, bool trim = false)
    {
        int cap = trim ? Math.Max(16, s._highWater) : s._generation.Length;
        ForkArrays.Prefix(ref _generation, s._generation, s._highWater, cap);
        ForkArrays.Prefix(ref _live, s._live, s._highWater, cap);
        ForkArrays.Prefix(ref _heap, s._heap, s._heapCount, trim ? cap : s._heap.Length);
        _heapCount = s._heapCount;
        _highWater = s._highWater;
        _liveCount = s._liveCount;
    }
}
