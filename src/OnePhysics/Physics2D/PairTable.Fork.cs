using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class PairTable
{
    internal void CopyFrom(PairTable s)
    {
        _entries = (Entry[])s._entries.Clone();
        _count = s._count;
        _shift = s._shift;
    }
}
