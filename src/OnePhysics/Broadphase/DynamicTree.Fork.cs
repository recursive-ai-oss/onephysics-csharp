using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.LowLevel;

public partial class DynamicTree<TBox> where TBox : struct, IBox<TBox>
{
    internal void CopyFrom(DynamicTree<TBox> s)
    {
        // Only the slots below the high-water mark hold nodes (free ones are chained below it); the array doubles when a node needs the next slot, so the copy needs no spare capacity.
        _nodes = new Node[Math.Max(s._used, 16)];
        Array.Copy(s._nodes, _nodes, s._used);
        _used = s._used;
        _free = s._free;
        _root = s._root;
        _leafCount = s._leafCount;
        _enlarged = s._enlarged;
        // The rebuild scratch (item, region and walk lists, centroids, stacks, bins) is sized by the rebuild that needs it, as in a new tree.
    }
}
