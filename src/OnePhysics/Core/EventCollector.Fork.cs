using System;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class EventCollector
{
    internal void CopyFrom(EventCollector s)
    {
        _slots = ForkArrays.CopyRows(s._slots);
        _counts = (int[])s._counts.Clone();
        _delivered = (PhysicsEvent[])s._delivered.Clone();
        _deliveredCount = s._deliveredCount;
    }
}
