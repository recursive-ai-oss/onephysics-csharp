using System;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class IslandSet
{
    internal void CopyFrom(IslandSet s)
    {
        _ids.CopyFrom(s._ids);
        ForkArrays.RowsInto(ref _bodies, s._bodies);
        ForkArrays.Into(ref _count, s._count);
        ForkArrays.Into(ref _asleep, s._asleep);
        ForkArrays.Into(ref _removed, s._removed);
        ForkArrays.Into(ref _scratchVisited, s._scratchVisited);
        ForkArrays.Into(ref _snapshot, s._snapshot);
        ForkArrays.Into(ref _countsScratch, s._countsScratch);
    }
}
