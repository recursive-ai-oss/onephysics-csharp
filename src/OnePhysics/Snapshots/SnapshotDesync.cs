using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>One named primitive or byte block in snapshot v1. Offsets refer to the inspected bytes (full snapshot or region archive). Tooling only; not a live view.</summary>
public sealed class SnapshotField
{
    private readonly byte[] _encoded;
    internal SnapshotField(SnapshotLocation location, byte[] bytes, int occurrence = 0)
    {
        Occurrence = occurrence;
        Section = location.Section; Entity = location.Entity; Index = location.Index; Generation = location.Generation;
        Name = location.Field; Offset = location.Offset; ByteCount = location.Size; ValueType = location.Type;
        Value = SnapshotDesync.Value(bytes, location);
        _encoded = bytes.AsSpan(location.Offset, location.Size).ToArray();
    }
    /// <summary>Logical section, e.g. bodies, constraints, engine3d or softbody3d.</summary>
    public string Section { get; }
    /// <summary>Entity kind, e.g. body, joint, contact, character, vehicle, ragdoll or softbody; world for configuration.</summary>
    public string Entity { get; }
    /// <summary>Stable entity slot, or -1 for configuration/framing.</summary>
    public int Index { get; }
    /// <summary>Handle generation, or zero where there is no generational handle.</summary>
    public int Generation { get; }
    /// <summary>Field name; vector lanes and array elements are explicit.</summary>
    public string Name { get; }
    /// <summary>Zero-based occurrence of this field within its entity, e.g. repeated wheel/vertex rows, in canonical write order.</summary>
    public int Occurrence { get; }
    /// <summary>Zero-based byte offset in the inspected full snapshot or region archive.</summary>
    public int Offset { get; }
    /// <summary>Encoded byte count.</summary>
    public int ByteCount { get; }
    /// <summary>Primitive type: byte, bool, int32, uint32, int64, float32, float64 or bytes.</summary>
    public string ValueType { get; }
    /// <summary>Invariant readable value, including raw bits for floats. Long byte blocks show their length and a prefix.</summary>
    public string Value { get; }
    /// <summary>Returns a copy of the encoded little-endian field bytes for raw tooling.</summary>
    public byte[] GetValueBytes() => (byte[])_encoded.Clone();
}

/// <summary>A named differing field. A missing entity/field is represented by a null side, not a fabricated value.</summary>
public sealed class SnapshotDifference
{
    internal SnapshotDifference(SnapshotField? a, SnapshotField? b) { A = a; B = b; }
    /// <summary>Field in the first snapshot, or null when absent.</summary>
    public SnapshotField? A { get; }
    /// <summary>Field in the second snapshot, or null when absent.</summary>
    public SnapshotField? B { get; }
    /// <summary>Logical section of the differing field.</summary>
    public string Section => (A ?? B!).Section;
    /// <summary>Entity kind.</summary>
    public string Entity => (A ?? B!).Entity;
    /// <summary>Stable slot or -1 for configuration.</summary>
    public int Index => (A ?? B!).Index;
    /// <summary>Generation of the first existing side.</summary>
    public int Generation => (A ?? B!).Generation;
    /// <summary>Named field.</summary>
    public string Field => (A ?? B!).Name;
    /// <summary>Occurrence of the named field within this entity.</summary>
    public int Occurrence => (A ?? B!).Occurrence;
    /// <summary>Invariant readable line, with both values and the entity's slot/generation.</summary>
    public override string ToString() => Section + ": " + Entity + (Index >= 0 ? " " + Index.ToString(CultureInfo.InvariantCulture) + " (gen " + Generation.ToString(CultureInfo.InvariantCulture) + ")" : "") + " " + Field + (Occurrence > 0 ? " #" + Occurrence.ToString(CultureInfo.InvariantCulture) : "") + ": a=" + (A?.Value ?? "<missing>") + " b=" + (B?.Value ?? "<missing>");
}

/// <summary>Tooling report ordered by section, entity kind, numeric slot/generation, field and occurrence; fields only in the second follow.</summary>
public sealed class SnapshotDiffReport
{
    internal SnapshotDiffReport(List<SnapshotDifference> differences) => Differences = differences.AsReadOnly();
    /// <summary>All differences in stable order. No reflection or callbacks are required to enumerate them.</summary>
    public IReadOnlyList<SnapshotDifference> Differences { get; }
    /// <summary>Number of named differing fields, not number of differing bytes.</summary>
    public int Count => Differences.Count;
    /// <summary>First differing field, or null for equality.</summary>
    public SnapshotDifference? First => Count == 0 ? null : Differences[0];
    /// <summary>Readable summary and at most the first 16 fields. Enumerate Differences for the complete report.</summary>
    public override string ToString()
    {
        if (Count == 0) { return "Snapshots are identical (0 differing fields)."; }
        var s = new StringBuilder(Count.ToString(CultureInfo.InvariantCulture) + " differing fields:\n");
        for (int i = 0; i < Math.Min(Count, 16); i++) { s.Append(Differences[i]).Append('\n'); }
        if (Count > 16) { s.Append("… enumerate Differences for the remaining fields.\n"); }
        return s.ToString();
    }
}

/// <summary>Named snapshot inspection and desync comparison. Uses the real snapshot readers for validation and writers for field locations, not a duplicate binary parser.</summary>
public static class SnapshotDesync
{
    /// <summary>Validates a full snapshot v1 and returns its named fields in write order. Allocates tooling state, including a temporary world.</summary>
    /// <exception cref="SnapshotFormatException">Invalid snapshot, version, dimension or section.</exception>
    public static IReadOnlyList<SnapshotField> Inspect(ReadOnlySpan<byte> snapshot)
    {
        var d = Describe(snapshot);
        return Materialize(d);
    }

    /// <summary>Compares two full snapshots v1, including events and every registered section, by named field. Both inputs are validated before comparison. Unknown sections are rejected, not ignored.</summary>
    /// <exception cref="SnapshotFormatException">Either input is invalid or not a supported full snapshot.</exception>
    public static SnapshotDiffReport Compare(ReadOnlySpan<byte> a, ReadOnlySpan<byte> b) => Compare(Describe(a), Describe(b));

    // Read only the core and section envelope first, using its existing validator. The broadphase margin is a construction option, not a mutable restore option.
    private sealed class HeaderProbe : ISnapshotSection
    {
        internal HeaderProbe(int id, bool optional = false) { SectionId = id; IsOptional = optional; }
        public int SectionId { get; }
        public bool IsOptional { get; }
        internal double Margin;
        public void Write(StateWriter writer) { }
        public void Read(StateReader reader)
        {
            if (SectionId is 0x42503244 or 0x42503344 && reader.Remaining >= 12) { reader.ReadUInt32(); Margin = reader.ReadDouble(); }
            reader.Skip(reader.Remaining);
        }
    }

    /// <summary>Inspects a raw core, including custom registered sections. Does not modify the core. Custom writers may use StateWriter.DescribeField for semantic labels; otherwise no bytes are omitted and opaque ranges are reported.</summary>
    /// <exception cref="ArgumentNullException">The core is null.</exception>
    public static IReadOnlyList<SnapshotField> Inspect(WorldCore core)
    { ArgumentNullException.ThrowIfNull(core); return Materialize(core.DescribeSnapshot()); }

    /// <summary>Compares raw cores with their registered schemas, without restoring them or invoking callbacks. Unknown custom payload ranges are compared too.</summary>
    /// <exception cref="ArgumentNullException">Either core is null.</exception>
    public static SnapshotDiffReport Compare(WorldCore a, WorldCore b)
    { ArgumentNullException.ThrowIfNull(a); ArgumentNullException.ThrowIfNull(b); return Compare(a.DescribeSnapshot(), b.DescribeSnapshot()); }

    internal static SnapshotDescription Describe(ReadOnlySpan<byte> bytes)
    {
        if (bytes.Length < 12 || BinaryPrimitives.ReadUInt32LittleEndian(bytes) != 0x4E53504F || BinaryPrimitives.ReadUInt32LittleEndian(bytes.Slice(4)) != 1)
        { throw new SnapshotFormatException("expected full snapshot v1 for desync inspection", 0); }
        int lanes = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(8));
        if (lanes != 2 && lanes != 3) { throw new SnapshotFormatException("unsupported position lane count for desync inspection", 8); }
        WorldCore schema = lanes == 2
            ? new Physics2D.PhysicsWorld2D(new Physics2D.WorldOptions2D { WorldId = 93 }).Core
            : new Physics3D.PhysicsWorld3D(new Physics3D.WorldOptions3D { WorldId = 94 }).Core;
        var core = new WorldCore(schema.Options);
        HeaderProbe? bp = null;
        foreach (var section in schema.SnapshotSections)
        {
            // The contact-edit marker is a built-in core section, already registered by the constructor.
            if (section.SectionId == core.ContactModifications.SectionId || section.SectionId == core.Surfaces.SectionId || section.SectionId == core.ContactMotion.SectionId || section.SectionId == core.ContactMaterials.SectionId) { continue; }
            if (section is BodySleepStore or BodyControlStore) continue;
            var probe = new HeaderProbe(section.SectionId, section.IsOptional);
            core.AddSection(probe);
            if (section.SectionId is 0x42503244 or 0x42503344) { bp = probe; }
        }
        if (bp == null) { throw new SnapshotFormatException("world schema has no broadphase section", 0); }
        core.Restore(bytes);
        if (!double.IsFinite(bp.Margin) || bp.Margin < 0 || (lanes == 2 && bp.Margin > 100) || !double.IsFinite(WorldCore.Grid(bp.Margin)))
        { throw new SnapshotFormatException("invalid broadphase margin in snapshot", 0); }
        if (lanes == 2)
        {
            var w = new Physics2D.PhysicsWorld2D(new Physics2D.WorldOptions2D { WorldId = 93, BroadphaseMargin = bp.Margin });
            w.Restore(bytes); return w.Core.DescribeSnapshot();
        }
        if (lanes == 3)
        {
            var w = new Physics3D.PhysicsWorld3D(new Physics3D.WorldOptions3D { WorldId = 94, BroadphaseMargin = bp.Margin });
            w.Restore(bytes); return w.Core.DescribeSnapshot();
        }
        throw new SnapshotFormatException("unsupported position lane count for desync inspection", 8);
    }

    internal static IReadOnlyList<SnapshotField> Materialize(SnapshotDescription d)
    {
        var list = new List<SnapshotField>(d.Fields.Count);
        var occurrences = new Dictionary<string, int>(StringComparer.Ordinal);
        foreach (var f in d.Fields)
        {
            occurrences.TryGetValue(f.Key, out int n); occurrences[f.Key] = n + 1;
            list.Add(new SnapshotField(f, d.Bytes, n));
        }
        return list.AsReadOnly();
    }

    internal static SnapshotDiffReport Compare(SnapshotDescription a, SnapshotDescription b)
    {
        var am = Keys(a.Fields); var bm = Keys(b.Fields);
        var differences = new List<SnapshotDifference>();
        foreach (var pair in am)
        {
            var x = pair.Value;
            if (!bm.TryGetValue(pair.Key, out var y)) { differences.Add(new SnapshotDifference(new SnapshotField(x, a.Bytes, Occurrence(pair.Key)), null)); }
            else if (x.Type != y.Type || !a.Bytes.AsSpan(x.Offset, x.Size).SequenceEqual(b.Bytes.AsSpan(y.Offset, y.Size)))
            { differences.Add(new SnapshotDifference(new SnapshotField(x, a.Bytes, Occurrence(pair.Key)), new SnapshotField(y, b.Bytes, Occurrence(pair.Key)))); }
        }
        foreach (var pair in bm)
        {
            if (!am.ContainsKey(pair.Key)) { differences.Add(new SnapshotDifference(null, new SnapshotField(pair.Value, b.Bytes, Occurrence(pair.Key)))); }
        }
        return new SnapshotDiffReport(differences);
    }

    private static int Occurrence(string key) => int.Parse(key.AsSpan(key.Length - 10), NumberStyles.None, CultureInfo.InvariantCulture);

    // Sorted keys give a specified order independent of runtime hash tables. The occurrence suffix disambiguates repeated record fields such as wheel parameters.
    internal static SortedDictionary<string, SnapshotLocation> Keys(List<SnapshotLocation> fields)
    {
        var map = new SortedDictionary<string, SnapshotLocation>(StringComparer.Ordinal);
        var occurrences = new Dictionary<string, int>(StringComparer.Ordinal);
        foreach (var f in fields)
        {
            string key = f.Key;
            occurrences.TryGetValue(key, out int n); occurrences[key] = n + 1;
            map.Add(key + "/" + n.ToString("D10", CultureInfo.InvariantCulture), f);
        }
        return map;
    }

    internal static string Value(byte[] bytes, SnapshotLocation f)
    {
        var b = bytes.AsSpan(f.Offset, f.Size);
        return f.Type switch
        {
            "float32" => BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(b)).ToString("R", CultureInfo.InvariantCulture) + " [0x" + BinaryPrimitives.ReadUInt32LittleEndian(b).ToString("x8", CultureInfo.InvariantCulture) + "]",
            "float64" => BitConverter.Int64BitsToDouble(BinaryPrimitives.ReadInt64LittleEndian(b)).ToString("R", CultureInfo.InvariantCulture) + " [0x" + BinaryPrimitives.ReadUInt64LittleEndian(b).ToString("x16", CultureInfo.InvariantCulture) + "]",
            "int32" => BinaryPrimitives.ReadInt32LittleEndian(b).ToString(CultureInfo.InvariantCulture),
            "uint32" => BinaryPrimitives.ReadUInt32LittleEndian(b).ToString(CultureInfo.InvariantCulture),
            "int64" => BinaryPrimitives.ReadInt64LittleEndian(b).ToString(CultureInfo.InvariantCulture),
            "bool" => b[0] == 0 ? "false" : "true",
            "byte" => b[0].ToString(CultureInfo.InvariantCulture),
            _ => Convert.ToHexString(b.Slice(0, Math.Min(b.Length, 32))) + (b.Length > 32 ? "… (" + b.Length.ToString(CultureInfo.InvariantCulture) + " bytes)" : ""),
        };
    }
}
