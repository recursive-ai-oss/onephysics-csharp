using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePhysics.LowLevel;

internal sealed class ContactMaterialStore : ISnapshotSection
{
    private sealed class Entry(int shape, int generation, ContactMaterial[] materials, int[] indices)
    {
        internal readonly int Shape = shape, Generation = generation;
        internal readonly ContactMaterial[] Materials = materials;
        internal readonly int[] Indices = indices;
        internal int[]? Canonical;
    }
    private readonly List<Entry> _entries = new();
    internal Func<int, int>? SubshapeCount;
    internal Func<int, int, int>? PlaneId;
    public int SectionId => 0x4D415453;
    public bool IsOptional => true;
    public bool IsPresent => _entries.Count != 0;
    private int Find(int shape)
    {
        int lo = 0, hi = _entries.Count;
        while (lo < hi) { int m = lo + (hi - lo) / 2; if (_entries[m].Shape < shape) lo = m + 1; else hi = m; }
        return lo;
    }
    private Entry? Get(int shape) { int i = Find(shape); return i < _entries.Count && _entries[i].Shape == shape ? _entries[i] : null; }
    /// <summary>After a voxel edit: new solid cells take a material index (or keep theirs when negative) and the merged-plane table is rebuilt on next use.</summary>
    internal void EditSubshapes(int shape, ReadOnlySpan<int> cells, int material)
    {
        var e = Get(shape); if (e == null) return;
        e.Canonical = null;
        if (material < 0) return;
        foreach (int cell in cells) if ((uint)(12 * cell + 11) < (uint)e.Indices.Length) e.Indices.AsSpan(12 * cell, 12).Fill(material);
    }
    internal int MaterialCount(int shape) => Get(shape)?.Materials.Length ?? 0;
    internal void Remove(int shape) { int i = Find(shape); if (i < _entries.Count && _entries[i].Shape == shape) _entries.RemoveAt(i); }
    internal void Set(ShapeHandle shape, ReadOnlySpan<ContactMaterial> materials, ReadOnlySpan<int> indices)
    {
        if (materials.Length == 0 && indices.Length == 0) { Remove(shape.Index); return; }
        int expected = SubshapeCount?.Invoke(shape.Index) ?? 1;
        if (indices.Length != expected) throw new ArgumentException("Material index count must equal triangle/segment count " + expected + ".", nameof(indices));
        for (int i = 0; i < indices.Length; i++) if ((uint)indices[i] >= (uint)materials.Length) throw new ArgumentOutOfRangeException(nameof(indices), indices[i], "Material index at element " + i + " is outside the table.");
        int position = Find(shape.Index); var e = new Entry(shape.Index, shape.Generation, materials.ToArray(), indices.ToArray());
        if (position < _entries.Count && _entries[position].Shape == shape.Index) _entries[position] = e; else _entries.Insert(position, e);
    }
    internal ContactMaterial Resolve(int shape, int subshape, ContactMaterial ordinary)
    {
        var e = Get(shape); return e != null && (uint)subshape < (uint)e.Indices.Length ? e.Materials[e.Indices[subshape]] : ordinary;
    }
    internal int CanonicalId(int shape, int subshape, int ordinary)
    {
        var e = Get(shape); return e?.Canonical != null && (uint)subshape < (uint)e.Canonical.Length ? e.Canonical[subshape] : ordinary;
    }
    internal int MaterialIndex(int shape, int subshape) { var e = Get(shape); return e != null && (uint)subshape < (uint)e.Indices.Length ? e.Indices[subshape] : -1; }
    internal void Prepare()
    {
        foreach (var e in _entries)
        {
            if (e.Canonical != null) continue;
            var canonical = new int[e.Indices.Length]; var groups = new SortedDictionary<(int Plane, int Material), int>();
            for (int i = 0; i < canonical.Length; i++)
            {
                var key = (PlaneId?.Invoke(e.Shape, i) ?? i, e.Indices[i]);
                if (!groups.TryGetValue(key, out int first)) { first = i; groups.Add(key, first); }
                canonical[i] = first;
            }
            e.Canonical = canonical;
        }
    }
    public void Write(StateWriter w)
    {
        w.WriteInt32(1); w.WriteInt32(_entries.Count);
        foreach (var e in _entries)
        {
            w.WriteInt32(e.Shape); w.WriteInt32(e.Generation); w.WriteInt32(e.Materials.Length);
            foreach (var m in e.Materials) { w.WriteSingle(m.Friction); w.WriteSingle(m.Restitution); w.WriteSingle(m.RollingResistance); }
            w.WriteInt32(e.Indices.Length); foreach (int index in e.Indices) w.WriteInt32(index);
        }
    }
    public void Read(StateReader r)
    {
        if (r.ReadInt32() != 1) throw r.Error("Unsupported indexed material version.");
        int count = r.ReadCount(16, "indexed material shapes"); var entries = new List<Entry>(count); int previous = -1;
        for (int i = 0; i < count; i++)
        {
            int shape = r.ReadInt32(), generation = r.ReadInt32(); if (shape <= previous || generation < 0) throw r.Error("Indexed material shapes must be ordered valid keys."); previous = shape;
            int n = r.ReadCount(12, "material coefficients"); if (n == 0) throw r.Error("Material table must not be empty.");
            var materials = new ContactMaterial[n];
            for (int m = 0; m < n; m++)
            {
                float friction = r.ReadSingle(), restitution = r.ReadSingle(), rolling = r.ReadSingle();
                try { materials[m] = new ContactMaterial(friction, restitution, rolling); } catch (ArgumentException) { throw r.Error("Invalid indexed material coefficient."); }
            }
            n = r.ReadCount(4, "material indices"); if (n == 0) throw r.Error("Material indices must not be empty."); var indices = new int[n];
            for (int k = 0; k < n; k++) { indices[k] = r.ReadInt32(); if ((uint)indices[k] >= (uint)materials.Length) throw r.Error("Material index is outside its table."); }
            entries.Add(new Entry(shape, generation, materials, indices));
        }
        _entries.Clear(); _entries.AddRange(entries);
    }
    internal void ValidateReferences(ShapeStore shapes)
    {
        foreach (var e in _entries)
            if (!shapes.Handles.IsAlive(e.Shape, e.Generation) || (SubshapeCount != null && e.Indices.Length != SubshapeCount(e.Shape))) throw new SnapshotFormatException("Indexed materials refer to a stale shape or disagree with triangle/segment count.", 0);
    }
    public void Reset() => _entries.Clear();
    internal void CopyFrom(ContactMaterialStore source)
    {
        _entries.Clear(); foreach (var e in source._entries) _entries.Add(new Entry(e.Shape, e.Generation, (ContactMaterial[])e.Materials.Clone(), (int[])e.Indices.Clone()));
    }
    public ISnapshotSection Fork(WorldCore destination) { var copy = new ContactMaterialStore(); copy.CopyFrom(this); return copy; }
}
