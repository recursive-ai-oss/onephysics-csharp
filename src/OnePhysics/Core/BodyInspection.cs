using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>Live read-only body view. Read between steps; getters validate the body generation. Does not allocate.</summary>
public readonly struct BodyInspection
{
    private readonly WorldCore _world;
    private readonly BodyHandle _body;
    internal BodyInspection(WorldCore world, BodyHandle body) { _world = world; _body = body; }
    private int Slot => _world == null ? throw new InvalidHandleException("The default inspection is not attached to a body.") : _world.Resolve(_body);

    /// <summary>Stable island slot, or -1 for a body that does not belong to a dynamic island.</summary>
    public int Island { get { int b = Slot; return _world.Bodies.Island[b]; } }
    /// <summary>True for an awake dynamic or kinematic body; false for a static or sleeping body.</summary>
    public bool IsAwake { get { int b = Slot; return _world.Bodies.IsAwake(b); } }
    /// <summary>Accumulated resting time in seconds; reset on wake.</summary>
    public float SleepTime { get { int b = Slot; return _world.Bodies.SleepTime[b]; } }
    /// <summary>Gravity impulse before damping, kg m/s. World axes; 2D uses xy for linear and z for angular.</summary>
    public RecursiveAI.OneMath.Vector3D GravityImpulse { get { int b = Slot; return _world.BodyReadback.Vector(b, 0); } }
    /// <summary>Integrated applied-force impulse plus public direct impulses since the preceding step, kg m/s; excludes solver and service-internal impulses. World axes; 2D uses xy for linear and z for angular.</summary>
    public RecursiveAI.OneMath.Vector3D ExternalImpulse { get { int b = Slot; return _world.BodyReadback.Vector(b, 3); } }
    /// <summary>Integrated applied torque plus public direct angular impulses, kg m squared/s. World axes; 2D uses xy for linear and z for angular.</summary>
    public RecursiveAI.OneMath.Vector3D ExternalAngularImpulse { get { int b = Slot; return _world.BodyReadback.Vector(b, 6); } }
    /// <summary>Linear momentum removed by damping, kg m/s, before speed clamps. World axes; 2D uses xy for linear and z for angular.</summary>
    public RecursiveAI.OneMath.Vector3D DampingImpulse { get { int b = Slot; return _world.BodyReadback.Vector(b, 9); } }
    /// <summary>Angular momentum removed by damping about the centre of mass, kg m squared/s, before speed clamps. World axes; 2D uses xy for linear and z for angular.</summary>
    public RecursiveAI.OneMath.Vector3D DampingAngularImpulse { get { int b = Slot; return _world.BodyReadback.Vector(b, 12); } }
    /// <summary>Bit mask of colours in Constraints: bits 0..23 are colours, bit 24 is serial overflow. Uncoloured constraints contribute no bit. No allocation.</summary>
    public uint ConstraintColorMask
    {
        get { uint result = 0; foreach (var c in Constraints) { if (c.Color < 24) { result |= 1u << c.Color; } else if (c.Color == 255) { result |= 1u << 24; } } return result; }
    }
    /// <summary>Contact and joint constraints touching the body, in ascending slot order, including sleeping contacts; excludes sensors.</summary>
    public BodyConstraintEnumerable Constraints { get { _ = Slot; return new(_world, _body); } }
    /// <summary>Number of contact and joint constraints. Linear in the world's constraint slot count; no allocation.</summary>
    public int ConstraintCount
    {
        get { int n = 0; foreach (var c in Constraints) { n++; } return n; }
    }
}

/// <summary>Allocation-free, read-only enumeration of a body's contacts and joints, ascending by constraint slot. Invalidated by world mutation.</summary>
public readonly struct BodyConstraintEnumerable
{
    private readonly WorldCore _world;
    private readonly BodyHandle _body;
    internal BodyConstraintEnumerable(WorldCore world, BodyHandle body) { _world = world; _body = body; }
    /// <summary>Creates the enumerator. Validates the body handle.</summary>
    public BodyConstraintEnumerator GetEnumerator() => new(_world, _body);
}

/// <summary>Constraint enumerator. Read between steps; invalidated by world mutation.</summary>
public struct BodyConstraintEnumerator
{
    private readonly WorldCore _world;
    private readonly BodyHandle _body;
    private int _slot;
    internal BodyConstraintEnumerator(WorldCore world, BodyHandle body)
    {
        if (world == null) { throw new InvalidHandleException("The default enumeration is not attached to a body."); }
        _world = world; _body = body; _slot = -1; world.Resolve(body);
    }
    /// <summary>Current constraint's read-only view. Only valid after MoveNext returns true.</summary>
    public ConstraintInspection Current
    {
        get
        {
            if (_world == null || (uint)_slot >= (uint)_world.Constraints.HighWater || !_world.Constraints.IsLive(_slot))
            { throw new InvalidOperationException("The enumerator is not positioned on a live constraint."); }
            return new(_world, _slot);
        }
    }
    /// <summary>Advances to the next contact or joint. Throws if the body has been destroyed.</summary>
    public bool MoveNext()
    {
        if (_world == null) { throw new InvalidHandleException("The default enumerator is not attached to a body."); }
        int b = _world.Resolve(_body);
        var cs = _world.Constraints;
        while (++_slot < cs.HighWater)
        {
            if (!cs.IsLive(_slot) || cs.KindOf(_slot) == ConstraintKind.Sensor) { continue; }
            if (cs.BodyA[_slot] == b || cs.BodyB[_slot] == b) { return true; }
        }
        return false;
    }
}

/// <summary>A read-only constraint view over solver storage. Valid until the world is mutated. Contact impulses are the final substep's warm-start impulses, not whole-step integrals.</summary>
public readonly ref struct ConstraintInspection
{
    private readonly WorldCore _world;
    private readonly int _slot;
    internal ConstraintInspection(WorldCore world, int slot) { _world = world; _slot = slot; }
    private WorldCore World => _world ?? throw new InvalidOperationException("The default inspection is not attached to a constraint.");
    /// <summary>Stable constraint slot (shared index space for contacts and joints).</summary>
    public int Slot => _slot;
    /// <summary>True for a contact manifold; false for a joint.</summary>
    public bool IsContact => World.Constraints.KindOf(_slot) == ConstraintKind.Contact;
    /// <summary>First body handle. A contact's normal points from A to B.</summary>
    public BodyHandle BodyA => World.HandleOfBody(World.Constraints.BodyA[_slot]);
    /// <summary>Second body handle.</summary>
    public BodyHandle BodyB => World.HandleOfBody(World.Constraints.BodyB[_slot]);
    /// <summary>Solver colour 0..23, 255 for serial overflow, 254 for uncoloured.</summary>
    public byte Color => World.Constraints.Color[_slot];
    /// <summary>Persistent solver payload, read-only. Layout is dimension and constraint kind dependent.</summary>
    public ReadOnlySpan<float> Payload => World.Constraints.Payload(_slot);
    /// <summary>Number of points in a contact, zero for a joint.</summary>
    public int PointCount => IsContact ? World.Constraints.PointCount[_slot] : 0;
    /// <summary>World-space normal components (two in 2D, three in 3D). Empty for joints.</summary>
    public ReadOnlySpan<float> Normal => IsContact ? Payload.Slice(ManifoldLayout.Normal, World.Layout.LinearLanes) : ReadOnlySpan<float>.Empty;
    /// <summary>Reads one contact point. Throws ArgumentOutOfRangeException unless index is in [0, PointCount).</summary>
    public ContactPointInspection Point(int index)
    {
        if ((uint)index >= (uint)PointCount) { throw new ArgumentOutOfRangeException(nameof(index), index, "Point index must be less than PointCount."); }
        return new ContactPointInspection(Payload, index, World.Layout.LinearLanes == 3);
    }
}

/// <summary>One contact point's final stored solver values. Read between steps. Impulses have units kg·m/s and belong to the last substep, not the whole step.</summary>
public readonly ref struct ContactPointInspection
{
    private readonly ReadOnlySpan<float> _payload;
    private readonly int _index;
    private readonly bool _spatial;
    internal ContactPointInspection(ReadOnlySpan<float> payload, int index, bool spatial) { _payload = payload; _index = index; _spatial = spatial; }
    private float Value(int offset) => _payload.IsEmpty ? throw new InvalidOperationException("The default inspection is not attached to a contact point.") : _payload[offset];
    /// <summary>Stored signed separation in metres at manifold update; negative is penetration. Not recomputed at the final pose.</summary>
    public float Separation => Value(ManifoldLayout.Separation(_index));
    /// <summary>Accumulated normal impulse in the final substep, nonnegative.</summary>
    public float NormalImpulse => Value(ManifoldLayout.NormalImpulse(_index));
    /// <summary>Signed first tangent impulse in the solver's tangent basis.</summary>
    public float Tangent1Impulse => Value(ManifoldLayout.Tangent1Impulse(_index));
    /// <summary>Signed second tangent impulse; zero in 2D.</summary>
    public float Tangent2Impulse => _spatial ? Value(ManifoldLayout.Tangent2Impulse(_index)) : 0f;
}
