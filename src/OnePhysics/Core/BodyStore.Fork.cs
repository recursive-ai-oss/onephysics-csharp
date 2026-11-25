using System;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class BodyStore
{
    /// <summary>Makes this store an independent copy of <paramref name="s"/>: only the slots below its high-water mark are copied when the capacities agree (a store of another capacity gets an array of the source's length, copied up to the mark), so unused capacity costs nothing.</summary>
    internal void CopyFrom(BodyStore s)
    {
        int n = s.HighWater;
        ForkArrays.Prefix(ref _position, s._position, n * _layout.PositionLanes);
        ForkArrays.Prefix(ref _rotation, s._rotation, n * _layout.RotationLanes);
        ForkArrays.Prefix(ref _linear, s._linear, n * _layout.LinearLanes);
        ForkArrays.Prefix(ref _angular, s._angular, n * _layout.AngularLanes);
        ForkArrays.Prefix(ref _invMass, s._invMass, n);
        ForkArrays.Prefix(ref _invInertia, s._invInertia, n * _layout.InertiaLanes);
        ForkArrays.Prefix(ref _sleepTime, s._sleepTime, n);
        ForkArrays.Prefix(ref _type, s._type, n);
        ForkArrays.Prefix(ref _flags, s._flags, n);
        ForkArrays.Prefix(ref _island, s._island, n);
        ForkArrays.Prefix(ref _edgeHead, s._edgeHead, n);
        Handles.CopyFrom(s.Handles);
    }
}
