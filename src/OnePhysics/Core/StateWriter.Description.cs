using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class StateWriter
{
    private static readonly UTF8Encoding FieldUtf8 = new(false, true);
    private List<SnapshotLocation>? _description;
    private WorldCore? _descriptionWorld;
    private string _descriptionSection = "header", _descriptionEntity = "world", _descriptionTable = "";
    private int _descriptionIndex = -1, _descriptionGeneration, _descriptionTree, _descriptionBodyA = -1, _descriptionBodyB = -1;

    internal void Describe(WorldCore world) { _description = new(); _descriptionWorld = world; }
    internal void DescribeSection(string name)
    {
        if (_description == null) { return; }
        _descriptionSection = name; _descriptionTable = "";
        DescribeEntity("world", -1);
    }
    internal void DescribeEntity(string kind, int index, int generation = 0)
    {
        if (_description == null) { return; }
        _descriptionEntity = kind; _descriptionIndex = index; _descriptionGeneration = generation; _descriptionBodyA = -1; _descriptionBodyB = -1;
    }
    internal void DescribeTable(string kind)
    {
        if (_description == null) { return; }
        _descriptionTable = kind; DescribeEntity("world", -1);
    }
    internal void DescribeSlot(int index, int generation)
    {
        if (_description == null) { return; }
        DescribeEntity(_descriptionTable, index, generation);
    }

    internal void DescribeEvent(int index, int bodyA, int bodyB, bool pending)
    {
        DescribeEntity(pending ? "pending event" : "event", index);
        _descriptionBodyA = bodyA; _descriptionBodyB = bodyB;
    }
    internal void DescribeTree(int index) { _descriptionTree = index; DescribeEntity("tree", index); }
    internal void DescribeNode(int index) => DescribeEntity("tree node", (index << 2) | _descriptionTree);

    /// <summary>Labels an already-written field for desync tooling in a custom section. Does not change snapshot/hash bytes. Offset is from this writer's start; length must lie entirely in the written buffer. Type is bytes by default, or byte/bool/int32/uint32/int64/float32/float64 at its matching width.</summary>
    /// <exception cref="ArgumentException">The name/type/width is invalid.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The range is outside the bytes already written.</exception>
    public void DescribeField(string name, int offset, int length, string type = "bytes")
    {
        ArgumentException.ThrowIfNullOrEmpty(name);
        if (FieldUtf8.GetByteCount(name) > 4096) { throw new ArgumentOutOfRangeException(nameof(name), "Field names must be valid UTF-8 and contain at most 4096 encoded bytes."); }
        int expected = type switch { "byte" or "bool" => 1, "int32" or "uint32" or "float32" => 4, "int64" or "float64" => 8, "bytes" => length, _ => -1 };
        if (expected != length) { throw new ArgumentException("Field type and byte width do not match.", nameof(type)); }
        if (_hashing || _counting) { return; }
        if (offset < 0 || length < 0 || offset > _length || length > _length - offset) { throw new ArgumentOutOfRangeException(nameof(offset), "Field range must lie in written bytes."); }
        DescribeValue(length, type, name, offset: offset, literal: true);
    }

    internal SnapshotDescription Description()
    {
        // Legacy/custom writers can use the original primitive API. Never ignore their bytes: uncovered ranges get an explicit opaque framing/payload field.
        var fields = _description!;
        fields.Sort(static (a, b) => a.Offset.CompareTo(b.Offset));
        var gaps = new List<SnapshotLocation>();
        int end = 0; string section = "header";
        foreach (var f in fields)
        {
            if (f.Offset > end) { gaps.Add(new SnapshotLocation(section, "world", -1, 0, "unlabelled framing/payload", -1, end, f.Offset - end, "bytes")); }
            end = Math.Max(end, f.Offset + f.Size); section = f.Section;
        }
        if (end < _length) { gaps.Add(new SnapshotLocation(section, "world", -1, 0, "unlabelled framing/payload", -1, end, _length - end, "bytes")); }
        fields.AddRange(gaps);
        fields.Sort(static (a, b) => a.Offset.CompareTo(b.Offset));
        return new SnapshotDescription { Bytes = ToArray(), Fields = fields };
    }

    private void DescribeValue(int size, string type, string field, int element = -1, int offset = -1, bool literal = false)
    {
        if (_description == null) { return; }
        _description.Add(new SnapshotLocation(_descriptionSection, _descriptionEntity, _descriptionIndex, _descriptionGeneration, field, element, offset < 0 ? _length : offset, size, type, _descriptionBodyA, _descriptionBodyB, literal));
    }

    private void DescribeArray(int count, int size, string type, string field)
    {
        if (_description == null) { return; }
        int stride = 0;
        string kind = _descriptionEntity;
        var world = _descriptionWorld!;
        string column = field.Split('.')[0];
        if (_descriptionSection == "bodies")
        {
            kind = "body";
            stride = column switch
            {
                "Position" => world.Layout.PositionLanes, "Rotation" => world.Layout.RotationLanes,
                "LinearVelocity" => world.Layout.LinearLanes, "AngularVelocity" => world.Layout.AngularLanes,
                "InverseInertia" => world.Layout.InertiaLanes,
                "InverseMass" or "SleepTime" or "Type" or "Flags" or "Island" => 1, _ => 0,
            };
        }
        else if (_descriptionSection == "constraints")
        {
            kind = "constraint";
            stride = column switch { "PayloadAll" => world.Layout.PayloadFloats, "FeatureIds" => world.Layout.MaxManifoldPoints, _ => 1 };
        }
        else if (_descriptionSection == "shapes" && column == "_owner") { kind = "shape"; stride = 1; }
        else if (_descriptionSection.StartsWith("engine", StringComparison.Ordinal))
        {
            int dim = world.Layout.PositionLanes;
            stride = column switch
            {
                "LocalCenter" or "Force" or "Dp" => dim, "Torque" => world.Layout.AngularLanes,
                "Inertia" => dim == 3 ? 6 : 1,
                "Mass" or "MaxExtent" or "MinExtent" or "LinearDamping" or "AngularDamping" or "GravityScale" or "BodyFlags" or "ShapeHead" or "ShapeTail" => 1,
                _ => 0,
            };
            if (stride > 0) { kind = "body"; }
        }
        if (column == "_generation" && _descriptionTable.Length > 0) { kind = _descriptionTable; stride = 1; }
        if (column == "_heap") { kind = "allocator"; stride = 0; }
        for (int i = 0; i < count; i++)
        {
            int index = stride > 0 ? i / stride : _descriptionIndex;
            int lane = stride > 0 ? i % stride : i;
            string entryKind = kind;
            int gen = _descriptionGeneration;
            if (stride > 0)
            {
                HandleTable? handles = kind switch { "body" => world.Bodies.Handles, "shape" => world.Shapes.Handles, "constraint" => world.Constraints.Handles, _ => null };
                if (handles != null && index < handles.HighWater) { gen = handles.GenerationOf(index); }
                if (kind == "constraint" && index < world.Constraints.HighWater && world.Constraints.IsLive(index))
                { entryKind = world.Constraints.KindOf(index) == ConstraintKind.Joint ? "joint" : "contact"; }
            }
            _description.Add(new SnapshotLocation(_descriptionSection, entryKind, index, gen, field, lane, _length + i * size, size, type, _descriptionBodyA, _descriptionBodyB));
        }
    }

    internal void NamedByte(byte v, [CallerArgumentExpression(nameof(v))] string field = "") { DescribeValue(1, "byte", field); WriteByte(v); }
    internal void NamedBool(bool v, [CallerArgumentExpression(nameof(v))] string field = "") { DescribeValue(1, "bool", field); WriteBool(v); }
    internal void NamedInt32(int v, [CallerArgumentExpression(nameof(v))] string field = "") { DescribeValue(4, "int32", field); WriteInt32(v); }
    internal void NamedUInt32(uint v, [CallerArgumentExpression(nameof(v))] string field = "") { DescribeValue(4, "uint32", field); WriteUInt32(v); }
    internal void NamedInt64(long v, [CallerArgumentExpression(nameof(v))] string field = "") { DescribeValue(8, "int64", field); WriteInt64(v); }
    internal void NamedSingle(float v, [CallerArgumentExpression(nameof(v))] string field = "") { DescribeValue(4, "float32", field); WriteSingle(v); }
    internal void NamedDouble(double v, [CallerArgumentExpression(nameof(v))] string field = "") { DescribeValue(8, "float64", field); WriteDouble(v); }
    internal void NamedSingles(ReadOnlySpan<float> v, [CallerArgumentExpression(nameof(v))] string field = "") { DescribeArray(v.Length, 4, "float32", field); WriteSingles(v); }
    internal void NamedDoubles(ReadOnlySpan<double> v, [CallerArgumentExpression(nameof(v))] string field = "") { DescribeArray(v.Length, 8, "float64", field); WriteDoubles(v); }
    internal void NamedInt32s(ReadOnlySpan<int> v, [CallerArgumentExpression(nameof(v))] string field = "") { DescribeArray(v.Length, 4, "int32", field); WriteInt32s(v); }
    internal void NamedUInt32s(ReadOnlySpan<uint> v, [CallerArgumentExpression(nameof(v))] string field = "") { DescribeArray(v.Length, 4, "uint32", field); WriteUInt32s(v); }
    internal void NamedBytes(ReadOnlySpan<byte> v, [CallerArgumentExpression(nameof(v))] string field = "")
    {
        if (_description != null)
        {
            if (_descriptionSection is "bodies" or "constraints" or "shapes" || field.StartsWith("BodyFlags", StringComparison.Ordinal)) { DescribeArray(v.Length, 1, "byte", field); }
            else { DescribeValue(v.Length, "bytes", field); }
        }
        WriteBytes(v);
    }
}
