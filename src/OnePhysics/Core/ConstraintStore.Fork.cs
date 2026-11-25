using System;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class ConstraintStore
{
    /// <summary>Makes this store an independent copy of <paramref name="s"/>: only the slots below its high-water mark are copied when the capacities agree (a store of another capacity gets an array of the source's length, copied up to the mark), so unused capacity costs nothing.</summary>
    internal void CopyFrom(ConstraintStore s, bool trim = false)
    {
        int n = s.HighWater, pts = _layout.MaxManifoldPoints, pay = _layout.PayloadFloats;
        // A fork has no use for the source's spare capacity (the store grows by doubling when it is needed), which is most of a big pile's payload array; a mark keeps it, so that copying back allocates nothing.
        int cap = trim ? Math.Max(16, n) : s.Capacity;
        ForkArrays.Prefix(ref _kind, s._kind, n, cap); ForkArrays.Prefix(ref _bodyA, s._bodyA, n, cap); ForkArrays.Prefix(ref _bodyB, s._bodyB, n, cap);
        ForkArrays.Prefix(ref _shapeA, s._shapeA, n, cap); ForkArrays.Prefix(ref _shapeAGen, s._shapeAGen, n, cap); ForkArrays.Prefix(ref _shapeB, s._shapeB, n, cap); ForkArrays.Prefix(ref _shapeBGen, s._shapeBGen, n, cap);
        ForkArrays.Prefix(ref _subA, s._subA, n, cap); ForkArrays.Prefix(ref _subB, s._subB, n, cap); ForkArrays.Prefix(ref _flags, s._flags, n, cap); ForkArrays.Prefix(ref _color, s._color, n, cap); ForkArrays.Prefix(ref _points, s._points, n, cap);
        ForkArrays.Prefix(ref _feature, s._feature, n * pts, cap * pts); ForkArrays.Prefix(ref _breakValue, s._breakValue, n, cap);
        ForkArrays.Prefix(ref _payload, s._payload, n * pay, cap * pay);
        ForkArrays.Prefix(ref _edgeNext, s._edgeNext, 2 * n, 2 * cap); ForkArrays.Prefix(ref _edgePrev, s._edgePrev, 2 * n, 2 * cap);
        Handles.CopyFrom(s.Handles, trim);
        JointRemovals = s.JointRemovals; LifetimeJoints = s.LifetimeJoints; BreaksRequested = s.BreaksRequested;
    }
}
