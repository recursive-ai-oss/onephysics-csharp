using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

internal sealed class SurfaceStore : ISnapshotSection
{
    private readonly int _dimension;
    private readonly List<Entry> _entries = new();
    private readonly record struct Entry(int Shape, int Generation, SurfaceProperties Value);
    internal SurfaceStore(int dimension) => _dimension = dimension;
    public int SectionId => 0x53555246;
    public bool IsOptional => true;
    public bool IsPresent => _entries.Count != 0;
    private int Find(int shape)
    {
        int lo = 0, hi = _entries.Count;
        while (lo < hi) { int m = lo + (hi - lo) / 2; if (_entries[m].Shape < shape) lo = m + 1; else hi = m; }
        return lo;
    }
    internal SurfaceProperties Get(int shape)
    {
        int i = Find(shape); return i < _entries.Count && _entries[i].Shape == shape ? _entries[i].Value : default;
    }
    internal void Remove(int shape) { int i = Find(shape); if (i < _entries.Count && _entries[i].Shape == shape) _entries.RemoveAt(i); }
    internal void Set(ShapeHandle shape, SurfaceProperties value)
    {
        value.Validate(_dimension);
        int i = Find(shape.Index);
        if (value.IsDefault) { Remove(shape.Index); return; }
        var entry = new Entry(shape.Index, shape.Generation, value);
        if (i < _entries.Count && _entries[i].Shape == shape.Index) _entries[i] = entry;
        else _entries.Insert(i, entry);
    }
    public void Write(StateWriter w)
    {
        w.WriteInt32(1); w.WriteInt32(_entries.Count);
        foreach (var e in _entries)
        {
            w.WriteInt32(e.Shape); w.WriteInt32(e.Generation);
            WriteVector(w, e.Value.Velocity); WriteVector(w, e.Value.OneWayDirection); WriteVector(w, e.Value.FrictionAxis);
            w.WriteSingle(e.Value.LongitudinalFriction); w.WriteSingle(e.Value.TransverseFriction); w.WriteSingle(e.Value.SpinningFriction);
        }
    }
    private static void WriteVector(StateWriter w, Vector3F v) { w.WriteSingle(v.X); w.WriteSingle(v.Y); w.WriteSingle(v.Z); }
    private static Vector3F ReadVector(StateReader r) => new(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
    public void Read(StateReader r)
    {
        if (r.ReadInt32() != 1) throw r.Error("Unsupported surface section version.");
        int count = r.ReadCount(56, "surface records");
        var entries = new List<Entry>(count);
        int previous = -1;
        for (int i = 0; i < count; i++)
        {
            int shape = r.ReadInt32(), generation = r.ReadInt32();
            if (shape <= previous || generation < 0) throw r.Error("Surface records must be strictly ordered live shape keys.");
            previous = shape;
            var value = new SurfaceProperties { Velocity = ReadVector(r), OneWayDirection = ReadVector(r), FrictionAxis = ReadVector(r), LongitudinalFriction = r.ReadSingle(), TransverseFriction = r.ReadSingle(), SpinningFriction = r.ReadSingle() };
            try { value.Validate(_dimension); } catch (ArgumentException) { throw r.Error("Invalid surface property."); }
            if (value.IsDefault) throw r.Error("Default surface records must be omitted.");
            entries.Add(new Entry(shape, generation, value));
        }
        _entries.Clear(); _entries.AddRange(entries);
    }
    internal void ValidateReferences(ShapeStore shapes)
    {
        foreach (var entry in _entries)
            if (!shapes.Handles.IsAlive(entry.Shape, entry.Generation)) throw new SnapshotFormatException("Surface refers to a stale shape.", 0);
    }
    public void Reset() => _entries.Clear();
    internal void CopyFrom(SurfaceStore source) { _entries.Clear(); _entries.AddRange(source._entries); }
    public ISnapshotSection Fork(WorldCore destination) { var copy = new SurfaceStore(_dimension); copy.CopyFrom(this); return copy; }
}
