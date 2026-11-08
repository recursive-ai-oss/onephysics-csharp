using System;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class BodyStore
{
    internal void CopyFrom(BodyStore s)
    {
        _position = (double[])s._position.Clone();
        _rotation = (float[])s._rotation.Clone();
        _linear = (float[])s._linear.Clone();
        _angular = (float[])s._angular.Clone();
        _invMass = (float[])s._invMass.Clone();
        _invInertia = (float[])s._invInertia.Clone();
        _sleepTime = (float[])s._sleepTime.Clone();
        _type = (byte[])s._type.Clone();
        _flags = (byte[])s._flags.Clone();
        _island = (int[])s._island.Clone();
        _edgeHead = (int[])s._edgeHead.Clone();
        Handles.CopyFrom(s.Handles);
    }
}
