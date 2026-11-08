using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.LowLevel;

public partial class DynamicTree<TBox> where TBox : struct, IBox<TBox>
{
    internal void CopyFrom(DynamicTree<TBox> s)
    {
        _nodes = (Node[])s._nodes.Clone();
        _used = s._used;
        _free = s._free;
        _root = s._root;
        _leafCount = s._leafCount;
        _enlarged = s._enlarged;
        _rbItems = (int[])s._rbItems.Clone();
        _rbRegion = (int[])s._rbRegion.Clone();
        _rbWalk = (int[])s._rbWalk.Clone();
        _rbCentroids = (double[])s._rbCentroids.Clone();
        _rbStack = (BuildItem[])s._rbStack.Clone();
        s._rbBinCount.CopyTo(_rbBinCount, 0);
        s._rbBinBox.CopyTo(_rbBinBox, 0);
        s._rbRightArea.CopyTo(_rbRightArea, 0);
        s._rbRightCount.CopyTo(_rbRightCount, 0);
        _rbUnits = (BuildItem[])s._rbUnits.Clone();
        _rbUnitStacks = (BuildItem[])s._rbUnitStacks.Clone();
        _rbTop = (int[])s._rbTop.Clone();
        _valSeen = (bool[])s._valSeen.Clone();
        _valOrder = (int[])s._valOrder.Clone();
    }
}
