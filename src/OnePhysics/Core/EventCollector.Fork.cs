using System;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class EventCollector
{
    /// <summary>
    /// Makes this collector hold the same events as <paramref name="s"/>: the delivered events and the pending ones of every slot, each copied up to its count.
    /// Rows keep whatever capacity they have (a row only has to be long enough), so copying into a collector that already has the storage allocates nothing.
    /// </summary>
    internal void CopyFrom(EventCollector s)
    {
        if (_slots.Length != s._slots.Length)
        {
            int old = _slots.Length;
            Array.Resize(ref _slots, s._slots.Length);
            Array.Resize(ref _counts, s._slots.Length);
            for (int i = old; i < _slots.Length; i++) { _slots[i] = new PhysicsEvent[16]; }
        }

        for (int i = 0; i < _slots.Length; i++)
        {
            int n = s._counts[i];
            if (_slots[i].Length < n) { _slots[i] = new PhysicsEvent[Math.Max(n, 16)]; }
            Array.Copy(s._slots[i], _slots[i], n);
            _counts[i] = n;
        }

        if (_delivered.Length < s._deliveredCount) { _delivered = new PhysicsEvent[Math.Max(s._deliveredCount, 16)]; }
        Array.Copy(s._delivered, _delivered, s._deliveredCount);
        _deliveredCount = s._deliveredCount;
    }
}
