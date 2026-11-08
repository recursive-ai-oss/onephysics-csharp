using System;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class IslandSet
{
    internal void CopyFrom(IslandSet s)
    {
        _ids.CopyFrom(s._ids);
        _bodies = ForkArrays.CopyRows(s._bodies);
        _count = (int[])s._count.Clone();
        _asleep = (bool[])s._asleep.Clone();
        _removed = (int[])s._removed.Clone();
        _scratchVisited = (bool[])s._scratchVisited.Clone();
        _snapshot = (int[])s._snapshot.Clone();
        _countsScratch = (int[])s._countsScratch.Clone();
    }
}
