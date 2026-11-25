using System;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class ShapeStore
{
    /// <summary>Makes this store an independent copy of <paramref name="s"/>: only the slots below its high-water mark are copied when the capacities agree (a store of another capacity gets an array of the source's length, copied up to the mark), so unused capacity costs nothing.</summary>
    internal void CopyFrom(ShapeStore s)
    {
        ForkArrays.Prefix(ref _owner, s._owner, s.HighWater);
        Handles.CopyFrom(s.Handles);
    }
}
