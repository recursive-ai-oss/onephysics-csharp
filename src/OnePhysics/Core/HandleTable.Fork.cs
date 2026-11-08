using System;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class HandleTable
{
    internal void CopyFrom(HandleTable s)
    {
        _generation = (int[])s._generation.Clone();
        _live = (bool[])s._live.Clone();
        _heap = (int[])s._heap.Clone();
        _heapCount = s._heapCount;
        _highWater = s._highWater;
        _liveCount = s._liveCount;
        _seenScratch = (bool[])s._seenScratch.Clone();
    }
}
