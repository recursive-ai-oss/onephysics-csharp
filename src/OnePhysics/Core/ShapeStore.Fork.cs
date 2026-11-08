using System;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class ShapeStore
{
    internal void CopyFrom(ShapeStore s)
    {
        _owner = (int[])s._owner.Clone();
        Handles.CopyFrom(s.Handles);
    }
}
