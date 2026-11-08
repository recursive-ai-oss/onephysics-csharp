using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.Text;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>A self-contained, versioned archive of a spatial region's selected entities and complete constraint closure. It is not a full snapshot v1. Bounds use inclusive body-origin/particle-point membership; see docs/spec/snapshots.md.</summary>
public sealed class RegionSnapshot
{
    /// <summary>Default parser field-count limit (1,000,000), independent of the input-byte limit. Hosts may override it explicitly for larger archives.</summary>
    public const int DefaultMaximumFields = 1000000;
    private const uint Magic = 0x5241504F; // OPAR
    private readonly SnapshotDescription _description;
    internal readonly double[] Minimum, Maximum;
    internal RegionSnapshot(SnapshotDescription description, int dimension, double[] min, double[] max)
    { _description = description; Dimension = dimension; Minimum = min; Maximum = max; }
    /// <summary>Coordinate dimension, 2 or 3.</summary>
    public int Dimension { get; }
    /// <summary>Inclusive minimum coordinates in metres, 2 or 3 lanes, relative to the saved origin.</summary>
    public ReadOnlySpan<double> MinimumCoordinates => Minimum;
    /// <summary>Inclusive maximum coordinates in metres, 2 or 3 lanes, relative to the saved origin.</summary>
    public ReadOnlySpan<double> MaximumCoordinates => Maximum;
    /// <summary>Named fields in canonical archive order. Returns an independent read-only list.</summary>
    public IReadOnlyList<SnapshotField> Fields
    {
        get => SnapshotDesync.Materialize(_description);
    }
    /// <summary>Returns an independent copy of the canonical little-endian region archive bytes.</summary>
    public byte[] ToArray() => (byte[])_description.Bytes.Clone();
    /// <summary>64-bit hash over the canonical archive bytes, using the world's byte-folding hash function. Includes bounds, tick, origin and simulation options.</summary>
    public ulong ComputeHash() { var writer = StateWriter.ForHash(); writer.WriteBytes(_description.Bytes); return writer.Hash; }
    /// <summary>Compares named region fields. Bounds/dimension differences are included.</summary>
    /// <exception cref="ArgumentNullException">The other archive is null.</exception>
    public SnapshotDiffReport FindDifferences(RegionSnapshot other)
    { ArgumentNullException.ThrowIfNull(other); return SnapshotDesync.Compare(_description, other._description); }

    /// <summary>Reads an archive, enforcing size, UTF-8, field count/ranges, numeric metadata, version and primitive widths before exposing it. Does not mutate a world. Defaults: input limit 256 MiB and field limit 1,000,000. Hosts may explicitly raise either.</summary>
    /// <exception cref="SnapshotFormatException">Malformed archive, field or bound.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A byte or field limit is negative.</exception>
    public static RegionSnapshot Read(ReadOnlySpan<byte> bytes, int maximumBytes = SnapshotEncoding.DefaultMaximumDecodedBytes, int maximumFields = DefaultMaximumFields)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(maximumBytes);
        ArgumentOutOfRangeException.ThrowIfNegative(maximumFields);
        if (bytes.Length > maximumBytes) { throw new SnapshotFormatException("region archive exceeds maximumBytes", 0); }
        var data = bytes.ToArray(); var r = new StateReader(data);
        if (r.ReadUInt32() != Magic || r.ReadUInt32() != 1) { throw r.Error("unknown region archive magic/version"); }
        int dimension = r.ReadInt32();
        if (dimension != 2 && dimension != 3) { throw r.Error("region dimension must be 2 or 3"); }
        var min = new double[dimension]; var max = new double[dimension];
        var fields = new List<SnapshotLocation> { new("region", "world", -1, 0, "dimension", -1, 8, 4, "int32", LiteralName: true) };
        for (int i = 0; i < dimension; i++) { int at = r.Position; min[i] = r.ReadDouble(); fields.Add(Bound("minimum", i, at)); }
        for (int i = 0; i < dimension; i++) { int at = r.Position; max[i] = r.ReadDouble(); fields.Add(Bound("maximum", i, at)); }
        try { ValidateBounds(min, max); } catch (ArgumentOutOfRangeException) { throw r.Error("invalid region coordinate bounds (finite, ordered, within ±2097152 required)"); }
        int count = r.ReadCount(29, "region fields");
        if (count > maximumFields) { throw r.Error("region field count exceeds maximumFields"); }
        var utf8 = new UTF8Encoding(false, true);
        for (int i = 0; i < count; i++)
        {
            string section = ReadString(r, utf8); string entity = ReadString(r, utf8);
            int index = r.ReadInt32(), generation = r.ReadInt32();
            if (index < -1 || generation < 0) { throw r.Error("invalid region entity slot/generation"); }
            string field = ReadString(r, utf8); string type = ReadString(r, utf8);
            int size = r.ReadCount(1, "region field bytes");
            int expected = type switch { "bool" or "byte" => 1, "int32" or "uint32" or "float32" => 4, "int64" or "float64" => 8, "bytes" => size, _ => -1 };
            if (size != expected) { throw r.Error("region field type/width is invalid"); }
            int at = r.Position;
            r.Skip(size);
            fields.Add(new SnapshotLocation(section, entity, index, generation, field, -1, at, size, type, LiteralName: true));
        }
        int previousOffset = -1;
        var records = fields.GetRange(1 + 2 * dimension, fields.Count - 1 - 2 * dimension);
        foreach (var pair in SnapshotDesync.Keys(records))
        {
            if (pair.Value.Offset <= previousOffset) { throw r.Error("region fields are not in canonical order"); }
            previousOffset = pair.Value.Offset;
        }
        if (r.Remaining != 0) { throw r.Error("trailing bytes in region archive"); }
        return new RegionSnapshot(new SnapshotDescription { Bytes = data, Fields = fields }, dimension, min, max);
    }

    internal SnapshotDescription Description => _description;
    internal static RegionSnapshot Create(SnapshotDescription source, IEnumerable<SnapshotLocation> selection, double[] min, double[] max)
    {
        var writer = StateWriter.ToBuffer();
        writer.WriteUInt32(Magic); writer.WriteUInt32(1); writer.WriteInt32(min.Length);
        var fields = new List<SnapshotLocation> { new("region", "world", -1, 0, "dimension", -1, 8, 4, "int32", LiteralName: true) };
        for (int i = 0; i < min.Length; i++) { fields.Add(Bound("minimum", i, writer.Length)); writer.WriteDouble(min[i]); }
        for (int i = 0; i < max.Length; i++) { fields.Add(Bound("maximum", i, writer.Length)); writer.WriteDouble(max[i]); }
        var sorted = SnapshotDesync.Keys(new List<SnapshotLocation>(selection));
        writer.WriteInt32(sorted.Count);
        foreach (var pair in sorted)
        {
            var f = pair.Value;
            WriteString(writer, f.Section); WriteString(writer, f.Entity);
            writer.WriteInt32(f.Index); writer.WriteInt32(f.Generation);
            WriteString(writer, f.Field); WriteString(writer, f.Type); writer.WriteInt32(f.Size);
            fields.Add(f with { Expression = f.Field, Element = -1, Offset = writer.Length, LiteralName = true });
            writer.WriteBytes(source.Bytes.AsSpan(f.Offset, f.Size));
        }
        return new RegionSnapshot(new SnapshotDescription { Bytes = writer.ToArray(), Fields = fields }, min.Length, (double[])min.Clone(), (double[])max.Clone());
    }

    private static SnapshotLocation Bound(string field, int lane, int offset) => new("region", "world", -1, 0, field + "." + "xyz"[lane], -1, offset, 8, "float64");
    private static void WriteString(StateWriter writer, string s) { var b = Encoding.UTF8.GetBytes(s); writer.WriteInt32(b.Length); writer.WriteBytes(b); }
    private static string ReadString(StateReader reader, Encoding utf8)
    {
        int count = reader.ReadCount(1, "region UTF-8 bytes");
        if (count == 0 || count > 4096) { throw reader.Error("region field identifier length must be 1..4096"); }
        var b = new byte[count]; reader.ReadBytes(b);
        try { return utf8.GetString(b); } catch (DecoderFallbackException) { throw reader.Error("invalid UTF-8 in region field identifier"); }
    }
    internal static void ValidateBounds(ReadOnlySpan<double> minimum, ReadOnlySpan<double> maximum)
    {
        for (int i = 0; i < minimum.Length; i++)
        {
            string lane = i.ToString(System.Globalization.CultureInfo.InvariantCulture);
            if (!double.IsFinite(minimum[i]) || Math.Abs(minimum[i]) > 2097152)
            { throw new ArgumentOutOfRangeException(nameof(minimum), minimum[i], "Region minimum lane " + lane + " must be finite and within ±2097152."); }
            if (!double.IsFinite(maximum[i]) || Math.Abs(maximum[i]) > 2097152)
            { throw new ArgumentOutOfRangeException(nameof(maximum), maximum[i], "Region maximum lane " + lane + " must be finite and within ±2097152."); }
            if (minimum[i] > maximum[i])
            { throw new ArgumentOutOfRangeException(nameof(minimum), minimum[i], "Region minimum lane " + lane + " must not exceed maximum " + maximum[i].ToString("R", System.Globalization.CultureInfo.InvariantCulture) + "."); }
        }
    }
}
