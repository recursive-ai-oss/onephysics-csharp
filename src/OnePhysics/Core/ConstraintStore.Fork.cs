using System;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class ConstraintStore
{
    internal void CopyFrom(ConstraintStore s)
    {
        _kind = (byte[])s._kind.Clone();
        _bodyA = (int[])s._bodyA.Clone();
        _bodyB = (int[])s._bodyB.Clone();
        _shapeA = (int[])s._shapeA.Clone();
        _shapeAGen = (int[])s._shapeAGen.Clone();
        _shapeB = (int[])s._shapeB.Clone();
        _shapeBGen = (int[])s._shapeBGen.Clone();
        _subA = (int[])s._subA.Clone();
        _subB = (int[])s._subB.Clone();
        _flags = (byte[])s._flags.Clone();
        _color = (byte[])s._color.Clone();
        _points = (byte[])s._points.Clone();
        _feature = (uint[])s._feature.Clone();
        _breakValue = (float[])s._breakValue.Clone();
        _payload = (float[])s._payload.Clone();
        _edgeNext = (int[])s._edgeNext.Clone();
        _edgePrev = (int[])s._edgePrev.Clone();
        Handles.CopyFrom(s.Handles);
    }
}
