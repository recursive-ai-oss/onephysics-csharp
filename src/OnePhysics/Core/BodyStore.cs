using System;

namespace RecursiveAI.OnePhysics.Core;

/// <summary>How a body takes part in the simulation.</summary>
public enum BodyType : byte
{
    /// <summary>Never moves, has no inverse mass, joins no island and no color.</summary>
    Static = 0,

    /// <summary>Moves under user control, has no inverse mass, never sleeps, joins no island and no color.</summary>
    Kinematic = 1,

    /// <summary>Moves under forces and constraints; belongs to an island.</summary>
    Dynamic = 2,
}

/// <summary>Per-body flag bits.</summary>
[Flags]
public enum BodyBits : byte
{
    /// <summary>No flags.</summary>
    None = 0,

    /// <summary>The body is awake. Static and kinematic bodies are always awake.</summary>
    Awake = 1,

    /// <summary>The body wants continuous collision against other moving bodies (used by the engines).</summary>
    Bullet = 2,

    /// <summary>The body never lets its island fall asleep.</summary>
    NoSleep = 4,
}

/// <summary>
/// Struct-of-arrays body storage shared by both engines. Positions are double (relative to the world origin); rotations, velocities and inverse mass and inertia are float32.
/// Every per-body array is indexed by body slot; vector-valued arrays hold the lanes of one body contiguously (stride from <see cref="DimensionLayout"/>).
/// Slots are valid below <see cref="HighWater"/>; a dead slot is zeroed. The spans returned here are views of the current arrays and are invalidated by anything that creates a body.
/// </summary>
/// <remarks>
/// Soft-body particles are meant to live here too: a particle is a Dynamic body that uses only the position, linear velocity and inverse mass lanes, so it joins islands, colors and sleep exactly like a rigid body;
/// its edge, bend and volume constraints are constraints of their own kinds referencing particle slots.
/// </remarks>
public sealed class BodyStore
{
    private readonly DimensionLayout _layout;
    private double[] _position = Array.Empty<double>();
    private float[] _rotation = Array.Empty<float>();
    private float[] _linear = Array.Empty<float>();
    private float[] _angular = Array.Empty<float>();
    private float[] _invMass = Array.Empty<float>();
    private float[] _invInertia = Array.Empty<float>();
    private float[] _sleepTime = Array.Empty<float>();
    private byte[] _type = Array.Empty<byte>();
    private byte[] _flags = Array.Empty<byte>();
    private int[] _island = Array.Empty<int>();
    private int[] _edgeHead = Array.Empty<int>();

    internal BodyStore(DimensionLayout layout, int capacity = 64)
    {
        _layout = layout;
        Handles = new HandleTable();
        Allocate(capacity);
    }

    /// <summary>The slot allocator for bodies.</summary>
    public HandleTable Handles { get; internal set; }

    /// <summary>The layout this store was built for.</summary>
    public DimensionLayout Layout => _layout;

    /// <summary>One more than the highest body slot ever used.</summary>
    public int HighWater => Handles.HighWater;

    /// <summary>Number of live bodies.</summary>
    public int Count => Handles.Count;

    /// <summary>Positions relative to the world origin: <see cref="DimensionLayout.PositionLanes"/> doubles per body.</summary>
    public Span<double> Position => _position.AsSpan(0, HighWater * _layout.PositionLanes);

    /// <summary>Rotations: <see cref="DimensionLayout.RotationLanes"/> floats per body.</summary>
    public Span<float> Rotation => _rotation.AsSpan(0, HighWater * _layout.RotationLanes);

    /// <summary>Linear velocities: <see cref="DimensionLayout.LinearLanes"/> floats per body.</summary>
    public Span<float> LinearVelocity => _linear.AsSpan(0, HighWater * _layout.LinearLanes);

    /// <summary>Angular velocities: <see cref="DimensionLayout.AngularLanes"/> floats per body.</summary>
    public Span<float> AngularVelocity => _angular.AsSpan(0, HighWater * _layout.AngularLanes);

    // Spare floats after the last velocity slot, so a kernel may load a whole eight-float window starting at any body's lanes (it uses the first three and ignores the rest).
    private const int VelocityPadding = 8;

    /// <summary>The linear velocity array including its padding (see <c>VelocityPadding</c>): body b's lanes start at b * LinearLanes and eight floats can be read from there. Read-only for kernels that gather rows.</summary>
    internal ReadOnlySpan<float> LinearVelocityPadded => _linear;

    /// <summary>The angular velocity array including its padding; see <see cref="LinearVelocityPadded"/>.</summary>
    internal ReadOnlySpan<float> AngularVelocityPadded => _angular;

    /// <summary>Inverse mass per body; zero for static and kinematic bodies.</summary>
    public Span<float> InverseMass => _invMass.AsSpan(0, HighWater);

    /// <summary>Inverse inertia in the body frame: <see cref="DimensionLayout.InertiaLanes"/> floats per body; zero for static and kinematic bodies.</summary>
    public Span<float> InverseInertia => _invInertia.AsSpan(0, HighWater * _layout.InertiaLanes);

    /// <summary>Seconds each body has been at rest (maintained by the step).</summary>
    public Span<float> SleepTime => _sleepTime.AsSpan(0, HighWater);

    /// <summary>The <see cref="BodyType"/> of each body, as bytes.</summary>
    public Span<byte> Type => _type.AsSpan(0, HighWater);

    /// <summary>The <see cref="BodyBits"/> of each body, as bytes.</summary>
    public Span<byte> Flags => _flags.AsSpan(0, HighWater);

    /// <summary>Island id of each dynamic body, -1 for others and for dead slots.</summary>
    public ReadOnlySpan<int> Island => _island.AsSpan(0, HighWater);

    internal Span<int> IslandWritable => _island.AsSpan(0, HighWater);

    internal Span<int> EdgeHead => _edgeHead.AsSpan(0, HighWater);

    /// <summary>First edge of a body's constraint list, or -1 (see <see cref="ConstraintStore.NextEdge"/>). The constraint of an edge is <c>edge >> 1</c>; side 0 is body A, side 1 body B. Dynamic bodies only; sensors are listed, static and kinematic bodies are not.</summary>
    public int FirstEdge(int body) => _edgeHead[body];

    /// <summary>True when the slot holds a live body.</summary>
    public bool IsLive(int body) => Handles.IsLive(body);

    /// <summary>The body's type; the slot must be live.</summary>
    public BodyType TypeOf(int body) => (BodyType)_type[body];

    /// <summary>True when the body is awake.</summary>
    public bool IsAwake(int body) => (_flags[body] & (byte)BodyBits.Awake) != 0;

    /// <summary>True when the body is live and dynamic.</summary>
    public bool IsDynamic(int body) => Handles.IsLive(body) && _type[body] == (byte)BodyType.Dynamic;

    /// <summary>Position lanes of one body.</summary>
    public Span<double> PositionOf(int body) => _position.AsSpan(body * _layout.PositionLanes, _layout.PositionLanes);

    /// <summary>Rotation lanes of one body.</summary>
    public Span<float> RotationOf(int body) => _rotation.AsSpan(body * _layout.RotationLanes, _layout.RotationLanes);

    /// <summary>Linear velocity lanes of one body.</summary>
    public Span<float> LinearVelocityOf(int body) => _linear.AsSpan(body * _layout.LinearLanes, _layout.LinearLanes);

    /// <summary>Angular velocity lanes of one body.</summary>
    public Span<float> AngularVelocityOf(int body) => _angular.AsSpan(body * _layout.AngularLanes, _layout.AngularLanes);

    /// <summary>Inverse inertia lanes of one body.</summary>
    public Span<float> InverseInertiaOf(int body) => _invInertia.AsSpan(body * _layout.InertiaLanes, _layout.InertiaLanes);

    private void Allocate(int capacity)
    {
        _position = new double[capacity * _layout.PositionLanes];
        _rotation = new float[capacity * _layout.RotationLanes];
        _linear = new float[capacity * _layout.LinearLanes + VelocityPadding];
        _angular = new float[capacity * _layout.AngularLanes + VelocityPadding];
        _invMass = new float[capacity];
        _invInertia = new float[capacity * _layout.InertiaLanes];
        _sleepTime = new float[capacity];
        _type = new byte[capacity];
        _flags = new byte[capacity];
        _island = new int[capacity];
        _edgeHead = new int[capacity];
    }

    private int Capacity => _type.Length;

    internal int AllocateSlot()
    {
        int b = Handles.Allocate();
        if (b >= Capacity)
        {
            int cap = Capacity * 2;
            Array.Resize(ref _position, cap * _layout.PositionLanes);
            Array.Resize(ref _rotation, cap * _layout.RotationLanes);
            Array.Resize(ref _linear, cap * _layout.LinearLanes + VelocityPadding);
            Array.Resize(ref _angular, cap * _layout.AngularLanes + VelocityPadding);
            Array.Resize(ref _invMass, cap);
            Array.Resize(ref _invInertia, cap * _layout.InertiaLanes);
            Array.Resize(ref _sleepTime, cap);
            Array.Resize(ref _type, cap);
            Array.Resize(ref _flags, cap);
            Array.Resize(ref _island, cap);
            Array.Resize(ref _edgeHead, cap);
        }

        return b;
    }

    internal void ClearSlot(int b)
    {
        PositionOf(b).Clear(); RotationOf(b).Clear(); LinearVelocityOf(b).Clear(); AngularVelocityOf(b).Clear(); InverseInertiaOf(b).Clear();
        _invMass[b] = 0; _sleepTime[b] = 0; _type[b] = 0; _flags[b] = 0; _island[b] = -1; _edgeHead[b] = -1;
    }

    internal void Write(StateWriter w)
    {
        Handles.Write(w);
        int n = HighWater;
        w.WriteDoubles(Position);
        w.WriteSingles(Rotation);
        w.WriteSingles(LinearVelocity);
        w.WriteSingles(AngularVelocity);
        w.WriteSingles(InverseMass);
        w.WriteSingles(InverseInertia);
        w.WriteSingles(SleepTime);
        w.WriteBytes(Type);
        w.WriteBytes(Flags);
        w.WriteInt32s(Island);
        w.WriteInt32(n); // trailing count: cheap framing check
    }

    /// <summary>Bytes the per-body columns take for one slot in the snapshot.</summary>
    private static int BytesPerSlot(DimensionLayout l) => 8 * l.PositionLanes + 4 * (l.RotationLanes + l.LinearLanes + l.AngularLanes + l.InertiaLanes + 2) + 2 + 4 + 5;

    internal static BodyStore Read(StateReader r, DimensionLayout layout)
    {
        var s = new BodyStore(layout, 1);
        s.ReadInto(r);
        return s;
    }

    /// <summary>Fills this store from a snapshot, reusing its arrays when they are big enough. Only a store nobody uses (a spare) may be read into: its contents are undefined after a failure.</summary>
    internal void ReadInto(StateReader r)
    {
        var layout = _layout;
        var handles = Handles;
        handles.ReadInto(r, "body");
        int n = handles.HighWater;
        if ((long)n * BytesPerSlot(layout) > r.Remaining) { throw r.Error("body columns are longer than the remaining bytes"); }
        if (Capacity < Math.Max(64, n)) { Allocate(Math.Max(64, n)); }
        var s = this;
        r.ReadDoubles(s.Position, true, "body positions");
        r.ReadSingles(s.Rotation, true, "body rotations");
        r.ReadSingles(s.LinearVelocity, true, "body linear velocities");
        r.ReadSingles(s.AngularVelocity, true, "body angular velocities");
        r.ReadSingles(s.InverseMass, true, "body inverse masses");
        r.ReadSingles(s.InverseInertia, true, "body inverse inertias");
        r.ReadSingles(s.SleepTime, true, "body sleep times");
        r.ReadBytes(s.Type);
        r.ReadBytes(s.Flags);
        r.ReadInt32s(s.IslandWritable);
        if (r.ReadInt32() != n) { throw r.Error("body column framing mismatch"); }
        for (int b = 0; b < n; b++)
        {
            s._edgeHead[b] = -1;
            if (!handles.IsLive(b))
            {
                if (s.IsSlotNonZero(b)) { throw r.Error("dead body slot " + b.ToString(System.Globalization.CultureInfo.InvariantCulture) + " is not zeroed"); }
                continue;
            }

            byte t = s._type[b];
            if (t > (byte)BodyType.Dynamic) { throw r.Error("invalid body type"); }
            if ((s._flags[b] & ~(byte)(BodyBits.Awake | BodyBits.Bullet | BodyBits.NoSleep)) != 0) { throw r.Error("invalid body flags"); }
            if (s._invMass[b] < 0f || s._sleepTime[b] < 0f) { throw r.Error("negative inverse mass or sleep time"); }
            if (t == (byte)BodyType.Dynamic)
            {
                if (!(s._invMass[b] > 0f)) { throw r.Error("dynamic body without positive inverse mass"); }
            }
            else
            {
                if (s._invMass[b] != 0f || s._island[b] != -1 || (s._flags[b] & (byte)BodyBits.Awake) == 0) { throw r.Error("static or kinematic body with mass, island or sleeping"); }
            }
        }
    }

    private bool IsSlotNonZero(int b)
    {
        foreach (double d in PositionOf(b)) { if (BitConverter.DoubleToUInt64Bits(d) != 0) { return true; } }
        foreach (float f in RotationOf(b)) { if (BitConverter.SingleToUInt32Bits(f) != 0) { return true; } }
        foreach (float f in LinearVelocityOf(b)) { if (BitConverter.SingleToUInt32Bits(f) != 0) { return true; } }
        foreach (float f in AngularVelocityOf(b)) { if (BitConverter.SingleToUInt32Bits(f) != 0) { return true; } }
        foreach (float f in InverseInertiaOf(b)) { if (BitConverter.SingleToUInt32Bits(f) != 0) { return true; } }
        return BitConverter.SingleToUInt32Bits(_invMass[b]) != 0 || BitConverter.SingleToUInt32Bits(_sleepTime[b]) != 0 || _type[b] != 0 || _flags[b] != 0 || _island[b] != -1;
    }
}
