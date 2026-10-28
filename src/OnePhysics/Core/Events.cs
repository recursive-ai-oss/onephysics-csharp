using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>
/// Collects events during a step and delivers them after it in a fixed order. Parallel code writes to its own slot (the chunk or task index), so no locks are needed;
/// the delivery sorts everything with <see cref="PhysicsEvent.Compare"/>, which makes the result independent of slots, threads and timing.
/// Events raised between steps (destroying a body, creating a joint) wait in the collector and are delivered at the end of the next step.
/// </summary>
[Experimental("OPHYS0001")]
public sealed class EventCollector
{
    private PhysicsEvent[][] _slots = new PhysicsEvent[1][];
    private int[] _counts = new int[1];
    private PhysicsEvent[] _delivered = Array.Empty<PhysicsEvent>();
    private int _deliveredCount;
    private readonly WorldOptions _options;

    internal EventCollector(WorldOptions options)
    {
        _options = options;
        _slots[0] = new PhysicsEvent[16];
    }

    /// <summary>The events delivered by the last step, in delivery order. Valid until the next step.</summary>
    public ReadOnlySpan<PhysicsEvent> Delivered => _delivered.AsSpan(0, _deliveredCount);

    /// <summary>Number of events collected and not yet delivered.</summary>
    public int PendingCount
    {
        get
        {
            int n = 0;
            for (int i = 0; i < _counts.Length; i++) { n += _counts[i]; }
            return n;
        }
    }

    /// <summary>Gives slot 0 room for <paramref name="events"/> events and every other slot (up to <paramref name="slots"/>) room for <paramref name="perSlot"/>, so a step that raises that many allocates nothing.</summary>
    internal void Reserve(int events, int slots, int perSlot)
    {
        EnsureSlots(slots);
        if (_slots[0].Length < events) { Array.Resize(ref _slots[0], events); }
        for (int i = 1; i < _slots.Length; i++) { if (_slots[i].Length < perSlot) { Array.Resize(ref _slots[i], perSlot); } }
        if (_delivered.Length < events) { Array.Resize(ref _delivered, events); }
    }

    /// <summary>Makes sure slots <c>0..count-1</c> exist. Call from serial code before a parallel phase that emits events.</summary>
    public void EnsureSlots(int count)
    {
        if (count <= _slots.Length) { return; }
        int old = _slots.Length;
        Array.Resize(ref _slots, count);
        Array.Resize(ref _counts, count);
        for (int i = old; i < count; i++) { _slots[i] = new PhysicsEvent[16]; }
    }

    internal bool Wants(EventKind kind) => (_options.Events & (EventMask)(1 << (int)kind)) != 0;

    /// <summary>Records an event in a slot if its kind is enabled. Distinct concurrent callers must use distinct slots.</summary>
    public void Emit(int slot, in PhysicsEvent e)
    {
        if (!Wants(e.Kind)) { return; }
        ref int n = ref _counts[slot];
        if (n == _slots[slot].Length) { Array.Resize(ref _slots[slot], n * 2); }
        _slots[slot][n++] = e;
    }

    /// <summary>Exchanges the collected and delivered events (and their storage) with another collector of the same world; used to commit a restore without copying.</summary>
    internal void SwapWith(EventCollector other)
    {
        (_slots, other._slots) = (other._slots, _slots);
        (_counts, other._counts) = (other._counts, _counts);
        (_delivered, other._delivered) = (other._delivered, _delivered);
        (_deliveredCount, other._deliveredCount) = (other._deliveredCount, _deliveredCount);
    }

    internal void ClearDelivered() => _deliveredCount = 0;

    /// <summary>Moves everything collected into the delivered list, sorted.</summary>
    internal void Deliver()
    {
        int total = PendingCount;
        if (total > _delivered.Length) { _delivered = new PhysicsEvent[Math.Max(total, _delivered.Length * 2)]; }
        int k = 0;
        for (int s = 0; s < _slots.Length; s++)
        {
            Array.Copy(_slots[s], 0, _delivered, k, _counts[s]);
            k += _counts[s];
            _counts[s] = 0;
        }

        _deliveredCount = k;
        Sorting.Sort(_delivered.AsSpan(0, k), new EventComparer());
    }

    internal void Write(StateWriter w)
    {
        w.WriteInt32(_deliveredCount);
        for (int i = 0; i < _deliveredCount; i++) { _delivered[i].Write(w); }
        w.WriteInt32(PendingCount);
        for (int s = 0; s < _slots.Length; s++)
        {
            for (int i = 0; i < _counts[s]; i++) { _slots[s][i].Write(w); }
        }
    }

    /// <summary>Replaces this collector's events with the saved ones, reusing its storage when it is big enough. Only a collector nobody uses (a spare) may be read into.</summary>
    internal void Read(StateReader r)
    {
        int d = r.ReadCount(PhysicsEvent.SnapshotBytes, "delivered events");
        if (_delivered.Length < Math.Max(d, 16)) { _delivered = new PhysicsEvent[Math.Max(d, 16)]; }
        for (int i = 0; i < d; i++) { _delivered[i] = PhysicsEvent.Read(r); }
        _deliveredCount = 0;
        int p = r.ReadCount(PhysicsEvent.SnapshotBytes, "pending events");
        if (_slots[0].Length < Math.Max(p, 16)) { _slots[0] = new PhysicsEvent[Math.Max(p, 16)]; }
        Array.Clear(_counts);
        for (int i = 0; i < p; i++) { _slots[0][i] = PhysicsEvent.Read(r); }
        _counts[0] = p;
        _deliveredCount = d;
    }

    private readonly struct EventComparer : IComparer<PhysicsEvent>
    {
        public int Compare(PhysicsEvent x, PhysicsEvent y) => PhysicsEvent.Compare(in x, in y);
    }
}
