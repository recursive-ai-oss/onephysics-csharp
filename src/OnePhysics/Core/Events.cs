using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePhysics.Core;

/// <summary>The kinds of event a step can produce, in the order they sort in a delivery.</summary>
public enum EventKind : byte
{
    /// <summary>A contact constraint was created.</summary>
    ContactBegin = 0,

    /// <summary>A contact is still there at the end of a step (only for contacts flagged <see cref="ConstraintBits.ReportPersist"/>).</summary>
    ContactPersist = 1,

    /// <summary>A contact constraint was removed.</summary>
    ContactEnd = 2,

    /// <summary>A sensor overlap began.</summary>
    SensorEnter = 3,

    /// <summary>A sensor overlap ended.</summary>
    SensorExit = 4,

    /// <summary>A joint broke and was removed. <see cref="PhysicsEvent.Value"/> is the break value.</summary>
    JointBreak = 5,

    /// <summary>A body fell asleep.</summary>
    Sleep = 6,

    /// <summary>A body woke up.</summary>
    Wake = 7,

    /// <summary>A contact's approach speed at the start of the step was above the world's hit threshold; <see cref="PhysicsEvent.Value"/> is the approach speed. Opt-in through <see cref="EventMask.ContactHit"/>.</summary>
    ContactHit = 8,
}

/// <summary>An event. Contact and sensor events carry both bodies and shapes and the sub-shape ids; joint events carry the joint and its bodies; sleep and wake carry body A only.</summary>
public readonly struct PhysicsEvent
{
    /// <summary>What happened.</summary>
    public EventKind Kind { get; }

    /// <summary>First body.</summary>
    public BodyHandle BodyA { get; }

    /// <summary>Second body (null for sleep and wake).</summary>
    public BodyHandle BodyB { get; }

    /// <summary>Shape of A (null for joint, sleep and wake events).</summary>
    public ShapeHandle ShapeA { get; }

    /// <summary>Shape of B.</summary>
    public ShapeHandle ShapeB { get; }

    /// <summary>Sub-shape of A.</summary>
    public int SubShapeA { get; }

    /// <summary>Sub-shape of B.</summary>
    public int SubShapeB { get; }

    /// <summary>The joint for <see cref="EventKind.JointBreak"/>; null otherwise.</summary>
    public JointHandle Joint { get; }

    /// <summary>Kind-specific number (break value for joint breaks); zero otherwise.</summary>
    public float Value { get; }

    /// <summary>Creates an event from all its parts.</summary>
    public PhysicsEvent(EventKind kind, BodyHandle a, BodyHandle b, ShapeHandle sa, ShapeHandle sb, int subA, int subB, JointHandle joint, float value)
    {
        Kind = kind; BodyA = a; BodyB = b; ShapeA = sa; ShapeB = sb; SubShapeA = subA; SubShapeB = subB; Joint = joint; Value = value;
    }

    /// <summary>
    /// The delivery order: kind, then body A, body B, shape A, shape B, sub-shape A, sub-shape B, joint (each by slot index, then generation), then the value's bit pattern.
    /// Total on every field, so events that compare equal are identical and the order does not depend on how they were collected.
    /// </summary>
    public static int Compare(in PhysicsEvent x, in PhysicsEvent y)
    {
        int c = ((int)x.Kind).CompareTo((int)y.Kind);
        if (c != 0) { return c; }
        c = x.BodyA.Index.CompareTo(y.BodyA.Index); if (c != 0) { return c; }
        c = x.BodyB.Index.CompareTo(y.BodyB.Index); if (c != 0) { return c; }
        c = x.ShapeA.Index.CompareTo(y.ShapeA.Index); if (c != 0) { return c; }
        c = x.ShapeB.Index.CompareTo(y.ShapeB.Index); if (c != 0) { return c; }
        c = x.SubShapeA.CompareTo(y.SubShapeA); if (c != 0) { return c; }
        c = x.SubShapeB.CompareTo(y.SubShapeB); if (c != 0) { return c; }
        c = x.Joint.Index.CompareTo(y.Joint.Index); if (c != 0) { return c; }
        c = x.BodyA.Generation.CompareTo(y.BodyA.Generation); if (c != 0) { return c; }
        c = x.BodyB.Generation.CompareTo(y.BodyB.Generation); if (c != 0) { return c; }
        c = x.ShapeA.Generation.CompareTo(y.ShapeA.Generation); if (c != 0) { return c; }
        c = x.ShapeB.Generation.CompareTo(y.ShapeB.Generation); if (c != 0) { return c; }
        c = x.Joint.Generation.CompareTo(y.Joint.Generation); if (c != 0) { return c; }
        return BitConverter.SingleToUInt32Bits(x.Value).CompareTo(BitConverter.SingleToUInt32Bits(y.Value));
    }

    internal void Write(StateWriter w)
    {
        w.WriteByte((byte)Kind);
        w.WriteInt32(BodyA.World); w.WriteInt32(BodyA.Index); w.WriteInt32(BodyA.Generation);
        w.WriteInt32(BodyB.World); w.WriteInt32(BodyB.Index); w.WriteInt32(BodyB.Generation);
        w.WriteInt32(ShapeA.World); w.WriteInt32(ShapeA.Index); w.WriteInt32(ShapeA.Generation);
        w.WriteInt32(ShapeB.World); w.WriteInt32(ShapeB.Index); w.WriteInt32(ShapeB.Generation);
        w.WriteInt32(SubShapeA); w.WriteInt32(SubShapeB);
        w.WriteInt32(Joint.World); w.WriteInt32(Joint.Index); w.WriteInt32(Joint.Generation);
        w.WriteSingle(Value);
    }

    internal const int SnapshotBytes = 1 + 4 * 17 + 4;

    internal static PhysicsEvent Read(StateReader r)
    {
        byte k = r.ReadByte();
        if (k > (byte)EventKind.ContactHit) { throw r.Error("invalid event kind"); }
        var a = new BodyHandle(r.ReadInt32(), r.ReadInt32(), r.ReadInt32());
        var b = new BodyHandle(r.ReadInt32(), r.ReadInt32(), r.ReadInt32());
        var sa = new ShapeHandle(r.ReadInt32(), r.ReadInt32(), r.ReadInt32());
        var sb = new ShapeHandle(r.ReadInt32(), r.ReadInt32(), r.ReadInt32());
        int subA = r.ReadInt32(), subB = r.ReadInt32();
        var j = new JointHandle(r.ReadInt32(), r.ReadInt32(), r.ReadInt32());
        float v = r.ReadSingle();
        return new PhysicsEvent((EventKind)k, a, b, sa, sb, subA, subB, j, v);
    }
}

/// <summary>
/// Collects events during a step and delivers them after it in a fixed order. Parallel code writes to its own slot (the chunk or task index), so no locks are needed;
/// the delivery sorts everything with <see cref="PhysicsEvent.Compare"/>, which makes the result independent of slots, threads and timing.
/// Events raised between steps (destroying a body, creating a joint) wait in the collector and are delivered at the end of the next step.
/// </summary>
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
