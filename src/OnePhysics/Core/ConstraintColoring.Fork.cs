using System;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class ConstraintColoring
{
    internal void CopyForFork(ConstraintColoring s)
    {
        _mask = (ulong[])s._mask.Clone();
        _list = (int[])s._list.Clone();
        s._start.CopyTo(_start, 0);
        s._cursor.CopyTo(_cursor, 0);
        _hist = (int[])s._hist.Clone();
        _bucketOf = (byte[])s._bucketOf.Clone();
    }
}
