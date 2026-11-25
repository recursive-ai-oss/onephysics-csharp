using System;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class ConstraintColoring
{
    internal void CopyForFork(ConstraintColoring s)
    {
        ForkArrays.Into(ref _mask, s._mask);
        ForkArrays.Into(ref _list, s._list);
        s._start.CopyTo(_start, 0);
        s._cursor.CopyTo(_cursor, 0);
        ForkArrays.Into(ref _hist, s._hist);
        ForkArrays.Into(ref _bucketOf, s._bucketOf);
    }
}
