using System;
using System.Globalization;

namespace RecursiveAI.OnePhysics.Core;

/// <summary>What a constraint is.</summary>
public enum ConstraintKind : byte
{
    /// <summary>Not a constraint (a dead slot).</summary>
    None = 0,

    /// <summary>A contact manifold between two shapes; solved, colored, links islands, produces contact events.</summary>
    Contact = 1,

    /// <summary>An overlap of a sensor shape; never solved, colored or linked, produces sensor events.</summary>
    Sensor = 2,

    /// <summary>A joint with up to six rows; solved, colored, links islands, may break.</summary>
    Joint = 3,
}

/// <summary>Per-constraint flag bits.</summary>
[Flags]
public enum ConstraintBits : byte
{
    /// <summary>No flags.</summary>
    None = 0,

    /// <summary>Emit <see cref="EventKind.ContactPersist"/> for this contact at the end of every step it is awake.</summary>
    ReportPersist = 1,

    /// <summary>Set by a stage during the step to ask the core to break this joint after the step's solve; the value goes in <see cref="ConstraintStore.BreakValue"/>.</summary>
    BreakRequested = 2,
}

/// <summary>Everything needed to create a constraint. Orientation matters: body A and shape A are the first of the pair as the engine defines it (the contact normal points from A to B).</summary>
public struct ConstraintDesc
{
    /// <summary>Contact, sensor or joint.</summary>
    public ConstraintKind Kind { get; set; }

    /// <summary>First body (live; may be static or kinematic).</summary>
    public BodyHandle BodyA { get; set; }

    /// <summary>Second body (live, different from A; may be static or kinematic).</summary>
    public BodyHandle BodyB { get; set; }

    /// <summary>Shape of body A, or the null handle for joints.</summary>
    public ShapeHandle ShapeA { get; set; }

    /// <summary>Shape of body B, or the null handle for joints.</summary>
    public ShapeHandle ShapeB { get; set; }

    /// <summary>Sub-shape of A (compound child or mesh triangle), 0 when the shape has none. Part of the contact key.</summary>
    public int SubShapeA { get; set; }

    /// <summary>Sub-shape of B.</summary>
    public int SubShapeB { get; set; }

    /// <summary>Flag bits.</summary>
    public ConstraintBits Flags { get; set; }
}

/// <summary>
/// Struct-of-arrays storage of all constraints (contacts, sensor overlaps and joints) in one index space, which is what the scheduler colors.
/// A contact's identity for events and for cross-frame matching is (shape A, sub-shape A, shape B, sub-shape B) plus per-point feature ids, so compound and mesh children are first-class.
/// Per-constraint persistent data (manifold geometry and warm-start impulses, or joint rows) lives in a fixed-stride float payload.
/// </summary>
public sealed class ConstraintStore
{
    /// <summary>Color value of a constraint that is not in the coloring: sensors and constraints without a dynamic body.</summary>
    public const byte Uncolored = 254;

    /// <summary>Color value of a constraint that did not fit in <see cref="ConstraintColoring.ColorCount"/> colors; solved serially.</summary>
    public const byte Overflow = 255;

    private readonly DimensionLayout _layout;
    private byte[] _kind = Array.Empty<byte>();
    private int[] _bodyA = Array.Empty<int>();
    private int[] _bodyB = Array.Empty<int>();
    private int[] _shapeA = Array.Empty<int>();
    private int[] _shapeAGen = Array.Empty<int>();
    private int[] _shapeB = Array.Empty<int>();
    private int[] _shapeBGen = Array.Empty<int>();
    private int[] _subA = Array.Empty<int>();
    private int[] _subB = Array.Empty<int>();
    private byte[] _flags = Array.Empty<byte>();
    private byte[] _color = Array.Empty<byte>();
    private byte[] _points = Array.Empty<byte>();
    private uint[] _feature = Array.Empty<uint>();
    private float[] _breakValue = Array.Empty<float>();
    private float[] _payload = Array.Empty<float>();
    private int[] _edgeNext = Array.Empty<int>();
    private int[] _edgePrev = Array.Empty<int>();

    internal ConstraintStore(DimensionLayout layout, int capacity = 64)
    {
        _layout = layout;
        Handles = new HandleTable();
        Allocate(capacity);
    }

    /// <summary>The slot allocator for constraints; a <see cref="JointHandle"/> carries a slot's generation.</summary>
    public HandleTable Handles { get; internal set; }

    /// <summary>The layout this store was built for.</summary>
    public DimensionLayout Layout => _layout;

    /// <summary>One more than the highest constraint slot ever used.</summary>
    public int HighWater => Handles.HighWater;

    /// <summary>Number of live constraints.</summary>
    public int Count => Handles.Count;

    /// <summary>Floats of payload per constraint.</summary>
    public int PayloadFloats => _layout.PayloadFloats;

    /// <summary>The <see cref="ConstraintKind"/> of each slot, as bytes.</summary>
    public ReadOnlySpan<byte> Kind => _kind.AsSpan(0, HighWater);

    /// <summary>First body slot of each constraint.</summary>
    public ReadOnlySpan<int> BodyA => _bodyA.AsSpan(0, HighWater);

    /// <summary>Second body slot of each constraint.</summary>
    public ReadOnlySpan<int> BodyB => _bodyB.AsSpan(0, HighWater);

    /// <summary>Shape slot of A (-1 for none).</summary>
    public ReadOnlySpan<int> ShapeA => _shapeA.AsSpan(0, HighWater);

    /// <summary>Shape slot of B (-1 for none).</summary>
    public ReadOnlySpan<int> ShapeB => _shapeB.AsSpan(0, HighWater);

    /// <summary>Sub-shape id of A.</summary>
    public ReadOnlySpan<int> SubShapeA => _subA.AsSpan(0, HighWater);

    /// <summary>Sub-shape id of B.</summary>
    public ReadOnlySpan<int> SubShapeB => _subB.AsSpan(0, HighWater);

    /// <summary>The <see cref="ConstraintBits"/> of each constraint, writable so stages can request a break.</summary>
    public Span<byte> Flags => _flags.AsSpan(0, HighWater);

    /// <summary>Color of each constraint: 0..<see cref="ConstraintColoring.ColorCount"/>-1, <see cref="Overflow"/> or <see cref="Uncolored"/>.</summary>
    public ReadOnlySpan<byte> Color => _color.AsSpan(0, HighWater);

    internal Span<byte> ColorWritable => _color.AsSpan(0, HighWater);

    /// <summary>Number of manifold points in use per contact (zero for others).</summary>
    public Span<byte> PointCount => _points.AsSpan(0, HighWater);

    /// <summary>Per-point feature ids of contacts, <see cref="DimensionLayout.MaxManifoldPoints"/> per constraint; matches points across frames so warm-start impulses carry over.</summary>
    public Span<uint> FeatureIds => _feature.AsSpan(0, HighWater * _layout.MaxManifoldPoints);

    /// <summary>The value that triggered a break request, reported in the joint-break event.</summary>
    public Span<float> BreakValue => _breakValue.AsSpan(0, HighWater);

    /// <summary>The persistent float payload of one constraint.</summary>
    public Span<float> Payload(int constraint) => _payload.AsSpan(constraint * _layout.PayloadFloats, _layout.PayloadFloats);

    /// <summary>The persistent payload of all constraints, <see cref="PayloadFloats"/> floats each.</summary>
    public Span<float> PayloadAll => _payload.AsSpan(0, HighWater * _layout.PayloadFloats);

    /// <summary>Feature ids of one contact.</summary>
    public Span<uint> FeatureIdsOf(int constraint) => _feature.AsSpan(constraint * _layout.MaxManifoldPoints, _layout.MaxManifoldPoints);

    /// <summary>True when the slot holds a live constraint.</summary>
    public bool IsLive(int constraint) => Handles.IsLive(constraint);

    /// <summary>Kind of a live constraint.</summary>
    public ConstraintKind KindOf(int constraint) => (ConstraintKind)_kind[constraint];

    internal int ShapeGenerationA(int c) => _shapeAGen[c];

    internal int ShapeGenerationB(int c) => _shapeBGen[c];

    private int Capacity => _kind.Length;

    private void Allocate(int cap)
    {
        _kind = new byte[cap]; _bodyA = new int[cap]; _bodyB = new int[cap];
        _shapeA = new int[cap]; _shapeAGen = new int[cap]; _shapeB = new int[cap]; _shapeBGen = new int[cap];
        _subA = new int[cap]; _subB = new int[cap]; _flags = new byte[cap]; _color = new byte[cap]; _points = new byte[cap];
        _feature = new uint[cap * _layout.MaxManifoldPoints]; _breakValue = new float[cap];
        _payload = new float[cap * _layout.PayloadFloats];
        _edgeNext = new int[cap * 2]; _edgePrev = new int[cap * 2];
    }

    /// <summary>Makes room for at least <paramref name="count"/> constraints at once, so a step that is about to add many does not grow every array by doubling several times.</summary>
    internal void Reserve(int count)
    {
        // Grows by at least half again: growing to exactly the requested size copied every array (the payloads are tens of megabytes in a big pile) each time the pair count rose by one.
        if (count > Capacity) { Resize(Math.Max(count, Capacity + (Capacity >> 1))); }
        Handles.Reserve(Capacity);
    }

    private void Resize(int cap)
    {
        Array.Resize(ref _kind, cap); Array.Resize(ref _bodyA, cap); Array.Resize(ref _bodyB, cap);
        Array.Resize(ref _shapeA, cap); Array.Resize(ref _shapeAGen, cap); Array.Resize(ref _shapeB, cap); Array.Resize(ref _shapeBGen, cap);
        Array.Resize(ref _subA, cap); Array.Resize(ref _subB, cap); Array.Resize(ref _flags, cap); Array.Resize(ref _color, cap); Array.Resize(ref _points, cap);
        Array.Resize(ref _feature, cap * _layout.MaxManifoldPoints); Array.Resize(ref _breakValue, cap);
        Array.Resize(ref _payload, cap * _layout.PayloadFloats);
        Array.Resize(ref _edgeNext, cap * 2); Array.Resize(ref _edgePrev, cap * 2);
    }

    internal int AllocateSlot()
    {
        int c = Handles.Allocate();
        if (c >= Capacity) { Resize(Capacity * 2); }

        return c;
    }

    internal void Fill(int c, in ConstraintDesc d, int bodyA, int bodyB, int shapeA, int shapeAGen, int shapeB, int shapeBGen)
    {
        _kind[c] = (byte)d.Kind; _bodyA[c] = bodyA; _bodyB[c] = bodyB;
        _shapeA[c] = shapeA; _shapeAGen[c] = shapeAGen; _shapeB[c] = shapeB; _shapeBGen[c] = shapeBGen;
        _subA[c] = d.SubShapeA; _subB[c] = d.SubShapeB; _flags[c] = (byte)d.Flags; _color[c] = Uncolored; _points[c] = 0; _breakValue[c] = 0;
        _edgePrev[2 * c] = -2; _edgePrev[2 * c + 1] = -2; _edgeNext[2 * c] = -1; _edgeNext[2 * c + 1] = -1;
    }

    internal void ClearSlot(int c)
    {
        _kind[c] = 0; _bodyA[c] = 0; _bodyB[c] = 0; _shapeA[c] = 0; _shapeAGen[c] = 0; _shapeB[c] = 0; _shapeBGen[c] = 0;
        _subA[c] = 0; _subB[c] = 0; _flags[c] = 0; _color[c] = 0; _points[c] = 0; _breakValue[c] = 0;
        FeatureIdsOf(c).Clear(); Payload(c).Clear();
        _edgePrev[2 * c] = -2; _edgePrev[2 * c + 1] = -2; _edgeNext[2 * c] = -1; _edgeNext[2 * c + 1] = -1;
    }

    // ---- adjacency: per dynamic body, a doubly linked list of the edges (constraint side) touching it. Edge e = 2 * constraint + side. ----

    /// <summary>First edge of a body's list, or -1. Constraint of an edge is <c>e >> 1</c>; side 0 is body A, side 1 body B. Only non-sensor constraints list on dynamic bodies.</summary>
    /// <summary>Next edge in the same body's list, or -1.</summary>
    public int NextEdge(int edge) => _edgeNext[edge];

    internal void LinkEdge(BodyStore bodies, int c, int side)
    {
        int e = 2 * c + side;
        int body = side == 0 ? _bodyA[c] : _bodyB[c];
        var head = bodies.EdgeHead;
        int first = head[body];
        _edgePrev[e] = -1;
        _edgeNext[e] = first;
        if (first >= 0) { _edgePrev[first] = e; }
        head[body] = e;
    }

    internal void UnlinkEdge(BodyStore bodies, int c, int side)
    {
        int e = 2 * c + side;
        if (_edgePrev[e] == -2) { return; }
        int body = side == 0 ? _bodyA[c] : _bodyB[c];
        int p = _edgePrev[e], n = _edgeNext[e];
        if (p >= 0) { _edgeNext[p] = n; } else { bodies.EdgeHead[body] = n; }
        if (n >= 0) { _edgePrev[n] = p; }
        _edgePrev[e] = -2; _edgeNext[e] = -1;
    }

    // ---- snapshot ----

    internal void Write(StateWriter w)
    {
        Handles.Write(w);
        int n = HighWater;
        w.WriteBytes(_kind.AsSpan(0, n));
        w.WriteInt32s(BodyA); w.WriteInt32s(BodyB);
        w.WriteInt32s(_shapeA.AsSpan(0, n)); w.WriteInt32s(_shapeAGen.AsSpan(0, n)); w.WriteInt32s(_shapeB.AsSpan(0, n)); w.WriteInt32s(_shapeBGen.AsSpan(0, n));
        w.WriteInt32s(SubShapeA); w.WriteInt32s(SubShapeB);
        w.WriteBytes(Flags); w.WriteBytes(Color); w.WriteBytes(PointCount);
        w.WriteUInt32s(FeatureIds);
        w.WriteSingles(BreakValue);
        w.WriteSingles(PayloadAll);
        w.WriteInt32(n);
    }

    private static int BytesPerSlot(DimensionLayout l) => 1 + 8 * 4 + 3 + 4 * l.MaxManifoldPoints + 4 + 4 * l.PayloadFloats;

    internal static ConstraintStore Read(StateReader r, DimensionLayout layout)
    {
        var s = new ConstraintStore(layout, 1);
        s.ReadInto(r);
        return s;
    }

    /// <summary>Fills this store from a snapshot, reusing its arrays when they are big enough. Only a store nobody uses (a spare) may be read into: its contents are undefined after a failure.</summary>
    internal void ReadInto(StateReader r)
    {
        var layout = _layout;
        var handles = Handles;
        handles.ReadInto(r, "constraint");
        int n = handles.HighWater;
        if ((long)n * BytesPerSlot(layout) > r.Remaining) { throw r.Error("constraint columns are longer than the remaining bytes"); }
        if (Capacity < Math.Max(64, n)) { Resize(Math.Max(64, n)); }
        var s = this;
        r.ReadBytes(s._kind.AsSpan(0, n));
        r.ReadInt32s(s._bodyA.AsSpan(0, n)); r.ReadInt32s(s._bodyB.AsSpan(0, n));
        r.ReadInt32s(s._shapeA.AsSpan(0, n)); r.ReadInt32s(s._shapeAGen.AsSpan(0, n)); r.ReadInt32s(s._shapeB.AsSpan(0, n)); r.ReadInt32s(s._shapeBGen.AsSpan(0, n));
        r.ReadInt32s(s._subA.AsSpan(0, n)); r.ReadInt32s(s._subB.AsSpan(0, n));
        r.ReadBytes(s._flags.AsSpan(0, n)); r.ReadBytes(s._color.AsSpan(0, n)); r.ReadBytes(s._points.AsSpan(0, n));
        r.ReadUInt32s(s.FeatureIds);
        r.ReadSingles(s.BreakValue, true, "joint break values");
        r.ReadSingles(s.PayloadAll, true, "constraint payloads");
        if (r.ReadInt32() != n) { throw r.Error("constraint column framing mismatch"); }
        for (int c = 0; c < n; c++)
        {
            s._edgePrev[2 * c] = -2; s._edgePrev[2 * c + 1] = -2; s._edgeNext[2 * c] = -1; s._edgeNext[2 * c + 1] = -1;
            if (!handles.IsLive(c))
            {
                bool dirty = s._kind[c] != 0 || s._bodyA[c] != 0 || s._bodyB[c] != 0 || s._shapeA[c] != 0 || s._shapeAGen[c] != 0 || s._shapeB[c] != 0 || s._shapeBGen[c] != 0
                    || s._subA[c] != 0 || s._subB[c] != 0 || s._flags[c] != 0 || s._color[c] != 0 || s._points[c] != 0 || s._breakValue[c] != 0;
                foreach (uint f in s.FeatureIdsOf(c)) { dirty |= f != 0; }
                foreach (float f in s.Payload(c)) { dirty |= BitConverter.SingleToUInt32Bits(f) != 0; }
                if (dirty) { throw r.Error("dead constraint slot " + c.ToString(CultureInfo.InvariantCulture) + " is not zeroed"); }
                continue;
            }

            if (s._kind[c] < (byte)ConstraintKind.Contact || s._kind[c] > (byte)ConstraintKind.Joint) { throw r.Error("invalid constraint kind"); }
            if ((s._flags[c] & ~(byte)(ConstraintBits.ReportPersist | ConstraintBits.BreakRequested)) != 0) { throw r.Error("invalid constraint flags"); }
            if (s._points[c] > layout.MaxManifoldPoints || (s._kind[c] != (byte)ConstraintKind.Contact && s._points[c] != 0)) { throw r.Error("invalid manifold point count"); }
            if (s._bodyA[c] == s._bodyB[c]) { throw r.Error("constraint joins a body to itself"); }
        }
    }
}

/// <summary>Shape slots: ownership and lifetime only. Geometry belongs to the engines, which keep their own arrays indexed by shape slot and save them through <see cref="ISnapshotSection"/>.</summary>
public sealed class ShapeStore
{
    private int[] _owner = new int[64];

    internal ShapeStore()
    {
        Handles = new HandleTable();
    }

    /// <summary>The slot allocator for shapes.</summary>
    public HandleTable Handles { get; internal set; }

    /// <summary>One more than the highest shape slot ever used.</summary>
    public int HighWater => Handles.HighWater;

    /// <summary>Owning body slot of each shape, -1 for dead slots.</summary>
    public ReadOnlySpan<int> Owner => _owner.AsSpan(0, HighWater);

    internal int AllocateSlot(int owner)
    {
        int s = Handles.Allocate();
        if (s >= _owner.Length) { Array.Resize(ref _owner, _owner.Length * 2); }
        _owner[s] = owner;
        return s;
    }

    internal void FreeSlot(int s)
    {
        Handles.Free(s);
        _owner[s] = -1;
    }

    internal void Write(StateWriter w)
    {
        Handles.Write(w);
        w.WriteInt32s(Owner);
    }

    internal static ShapeStore Read(StateReader r)
    {
        var s = new ShapeStore();
        s.ReadInto(r);
        return s;
    }

    /// <summary>Fills this store from a snapshot, reusing its array when it is big enough. Only a store nobody uses (a spare) may be read into.</summary>
    internal void ReadInto(StateReader r)
    {
        Handles.ReadInto(r, "shape");
        int n = Handles.HighWater;
        if ((long)n * 4 > r.Remaining) { throw r.Error("shape columns are longer than the remaining bytes"); }
        if (_owner.Length < Math.Max(64, n)) { _owner = new int[Math.Max(64, n)]; }
        r.ReadInt32s(_owner.AsSpan(0, n));
    }
}
