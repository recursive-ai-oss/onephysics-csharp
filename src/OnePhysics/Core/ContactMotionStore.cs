using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

internal sealed class ContactMotionStore : ISnapshotSection
{
    private readonly List<Entry> _records = new();
    private readonly record struct Key(int A, int Ag, int B, int Bg, int SubA, int SubB) : IComparable<Key>
    {
        public int CompareTo(Key b)
        {
            int c = A.CompareTo(b.A); if (c == 0) c = Ag.CompareTo(b.Ag); if (c == 0) c = B.CompareTo(b.B); if (c == 0) c = Bg.CompareTo(b.Bg); if (c == 0) c = SubA.CompareTo(b.SubA); if (c == 0) c = SubB.CompareTo(b.SubB); return c;
        }
    }
    private record struct Entry(Key Key, ContactMotion Motion, bool Alive);
    public int SectionId => 0x43444154;
    public bool IsOptional => true;
    public bool IsPresent => Enabled;
    internal bool Enabled { get; set; }
    private int Find(Key key)
    {
        int lo = 0, hi = _records.Count;
        while (lo < hi) { int m = lo + (hi - lo) / 2; if (_records[m].Key.CompareTo(key) < 0) lo = m + 1; else hi = m; }
        return lo;
    }
    private static Key KeyOf(WorldCore w, int c)
    {
        var cs = w.Constraints; int a = cs.ShapeA[c], b = cs.ShapeB[c];
        return new(a, w.Shapes.Handles.GenerationOf(a), b, w.Shapes.Handles.GenerationOf(b), cs.SubShapeA[c], cs.SubShapeB[c]);
    }
    internal ContactMotion ForEvent(in PhysicsEvent e)
    {
        if (!Enabled) return default;
        var key = new Key(e.ShapeA.Index, e.ShapeA.Generation, e.ShapeB.Index, e.ShapeB.Generation, e.SubShapeA, e.SubShapeB);
        int i = Find(key); return i < _records.Count && _records[i].Key == key ? _records[i].Motion : default;
    }
    internal ContactMotion ForConstraint(WorldCore w, int c)
    {
        if (!Enabled) return default;
        var key = KeyOf(w, c); int i = Find(key); return i < _records.Count && _records[i].Key == key ? _records[i].Motion : default;
    }
    internal void Capture(WorldCore w)
    {
        // At most one live generation and one ended generation per reserved contact slot survive a step.
        _records.EnsureCapacity(2 * w.Constraints.SlotCapacity);
        _records.RemoveAll(static r => !r.Alive);
        for (int i = 0; i < _records.Count; i++) _records[i] = _records[i] with { Alive = false };
        var cs = w.Constraints;
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Contact || cs.PointCount[c] == 0) continue;
            var key = KeyOf(w, c); int i = Find(key);
            if (i < _records.Count && _records[i].Key == key) { _records[i] = _records[i] with { Alive = true }; continue; }
            var pl = cs.Payload(c); var n = Normal(w, pl); float approach = 0;
            for (int p = 0; p < cs.PointCount[c]; p++)
            {
                var relative = SurfaceContacts.PointVelocity(w, cs.BodyB[c], pl, ManifoldLayout.AnchorB(p)) - SurfaceContacts.PointVelocity(w, cs.BodyA[c], pl, ManifoldLayout.AnchorA(p));
                approach = Math.Max(approach, -Vector3F.Dot(relative, n));
            }
            _records.Insert(i, new Entry(key, new ContactMotion(approach, 0, 0), true));
        }
    }
    internal void Update(WorldCore w)
    {
        var cs = w.Constraints;
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Contact || cs.PointCount[c] == 0) continue;
            var key = KeyOf(w, c); int index = Find(key); if (index == _records.Count || _records[index].Key != key) continue;
            var pl = cs.Payload(c); var n = Normal(w, pl); float sliding = 0, rolling = 0;
            int a = cs.BodyA[c], b = cs.BodyB[c];
            var angularA = Angular(w, a); var angularB = Angular(w, b); var dw = angularB - angularA;
            var rotation = w.Layout.PositionLanes == 3 ? Vector3F.Cross(dw, n) : dw;
            float rollRate = float.Sqrt(Vector3F.Dot(rotation, rotation));
            var surface = SurfaceContacts.Rotate(w, a, w.Surfaces.Get(cs.ShapeA[c]).Velocity) - SurfaceContacts.Rotate(w, b, w.Surfaces.Get(cs.ShapeB[c]).Velocity);
            for (int p = 0; p < cs.PointCount[c]; p++)
            {
                int ra = ManifoldLayout.AnchorA(p), rb = ManifoldLayout.AnchorB(p);
                var dv = SurfaceContacts.PointVelocity(w, b, pl, rb) - SurfaceContacts.PointVelocity(w, a, pl, ra) - surface;
                dv -= n * Vector3F.Dot(dv, n); sliding = Math.Max(sliding, float.Sqrt(Vector3F.Dot(dv, dv)));
                float armA = Length(w, pl, ra), armB = Length(w, pl, rb);
                float arm = w.Bodies.IsDynamic(a) ? (w.Bodies.IsDynamic(b) ? Math.Min(armA, armB) : armA) : armB; rolling = Math.Max(rolling, rollRate * arm);
            }
            var previous = _records[index]; _records[index] = previous with { Motion = new ContactMotion(previous.Motion.ImpactSpeed, sliding, rolling) };
        }
    }
    private static Vector3F Normal(WorldCore w, ReadOnlySpan<float> p) => new(p[0], p[1], w.Layout.PositionLanes == 3 ? p[2] : 0);
    private static Vector3F Angular(WorldCore w, int b) { var a = w.Bodies.AngularVelocityOf(b); return w.Layout.PositionLanes == 3 ? new(a[0], a[1], a[2]) : new(0, 0, a[0]); }
    private static float Length(WorldCore w, ReadOnlySpan<float> p, int a) => float.Sqrt(p[a] * p[a] + p[a + 1] * p[a + 1] + (w.Layout.PositionLanes == 3 ? p[a + 2] * p[a + 2] : 0));
    internal long GrowthBound(int slots) => Math.Max(0L, 2L * slots - _records.Count) * 37L;
    public void Write(StateWriter w)
    {
        w.WriteInt32(1); w.WriteInt32(_records.Count);
        foreach (var r in _records)
        {
            w.WriteInt32(r.Key.A); w.WriteInt32(r.Key.Ag); w.WriteInt32(r.Key.B); w.WriteInt32(r.Key.Bg); w.WriteInt32(r.Key.SubA); w.WriteInt32(r.Key.SubB);
            w.WriteSingle(r.Motion.ImpactSpeed); w.WriteSingle(r.Motion.SlidingSpeed); w.WriteSingle(r.Motion.RollingSpeed); w.WriteBool(r.Alive);
        }
    }
    public void Read(StateReader r)
    {
        if (r.ReadInt32() != 1) throw r.Error("Unsupported contact motion version.");
        int count = r.ReadCount(37, "contact motion records"); var records = new List<Entry>(count); Key previous = default;
        for (int i = 0; i < count; i++)
        {
            var key = new Key(r.ReadInt32(), r.ReadInt32(), r.ReadInt32(), r.ReadInt32(), r.ReadInt32(), r.ReadInt32());
            float impact = r.ReadSingle(), sliding = r.ReadSingle(), rolling = r.ReadSingle(); bool alive = r.ReadBool();
            if (key.A < 0 || key.B < 0 || key.Ag < 0 || key.Bg < 0 || (i > 0 && previous.CompareTo(key) >= 0) || !(impact >= 0 && float.IsFinite(impact) && sliding >= 0 && float.IsFinite(sliding) && rolling >= 0 && float.IsFinite(rolling))) throw r.Error("Invalid or unordered contact motion record.");
            previous = key; records.Add(new Entry(key, new ContactMotion(impact, sliding, rolling), alive));
        }
        _records.Clear(); _records.AddRange(records); Enabled = true;
    }
    public void Reset() { Enabled = false; _records.Clear(); }
    internal void CopyFrom(ContactMotionStore source) { Enabled = source.Enabled; _records.Clear(); _records.AddRange(source._records); }
    public ISnapshotSection Fork(WorldCore destination) { var copy = new ContactMotionStore(); copy.CopyFrom(this); return copy; }
}
