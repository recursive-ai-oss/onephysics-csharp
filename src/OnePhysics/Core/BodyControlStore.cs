using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

internal struct BodyControlState
{
    internal BodyControlSettings Settings;
    internal byte Overrides;
    internal float Mass;
    internal Vector3F Center, Principal;
    internal QuaternionF InertiaRotation;
    internal Vector3D TargetPosition;
    internal QuaternionF TargetRotation;
    internal readonly bool Present => Overrides != 0 || !Settings.IsDefault;
}

internal sealed class BodyControlStore : ISnapshotSection
{
    private readonly WorldCore _world;
    internal BodyControlState[] Items = Array.Empty<BodyControlState>();
    private BodyControlState[] _staging = Array.Empty<BodyControlState>();
    private int _count;
    internal float DefaultAngularSpeed = float.MaxValue;
    private int _lockCount;
    internal bool HasLocks => _lockCount != 0;
    internal bool Active => _count != 0 || DefaultAngularSpeed != float.MaxValue;
    internal BodyControlStore(WorldCore world) => _world = world;
    public int SectionId => 0x4243544C;
    public bool IsOptional => true;
    public bool IsPresent => Active;
    internal RigidbodyConstraints ConstraintsOf(int b) => b < Items.Length ? Items[b].Settings.Constraints : 0;
    internal BodyControlState Get(int b) => b < Items.Length ? Items[b] : default;
    internal void Set(int b, BodyControlState value)
    {
        if (b >= Items.Length) { if (!value.Present) return; Array.Resize(ref Items, Math.Max(b + 1, Math.Max(16, Items.Length * 2))); }
        if (Items[b].Present) _count--; if (Items[b].Settings.Constraints != 0) _lockCount--;
        Items[b] = value; if (value.Present) _count++; if (value.Settings.Constraints != 0) _lockCount++;
    }
    public void Reset() { Array.Clear(Items); _count = 0; _lockCount = 0; DefaultAngularSpeed = float.MaxValue; }
    internal void CopyFrom(BodyControlStore s) { Items = (BodyControlState[])s.Items.Clone(); _count = s._count; _lockCount = s._lockCount; DefaultAngularSpeed = s.DefaultAngularSpeed; }
    public ISnapshotSection Fork(WorldCore destination) { var c = new BodyControlStore(destination); c.CopyFrom(this); return c; }
    public void Write(StateWriter w)
    {
        w.NamedInt32(1, "version"); w.NamedSingle(DefaultAngularSpeed); w.NamedInt32(_count, "count");
        for (int b = 0; b < Items.Length; b++)
        {
            var s = Items[b]; if (!s.Present) continue;
            w.DescribeEntity("body", b, _world.Bodies.Handles.GenerationOf(b));
            w.NamedInt32(b, "slot"); w.NamedInt32((int)s.Settings.Constraints, "constraints");
            w.NamedSingle(s.Settings.MaxLinearSpeed); w.NamedSingle(s.Settings.MaxAngularSpeed); w.NamedSingle(s.Settings.MaxDepenetrationSpeed);
            w.NamedByte(s.Overrides); w.NamedSingle(s.Mass);
            w.NamedSingle(s.Center.X); w.NamedSingle(s.Center.Y); w.NamedSingle(s.Center.Z);
            w.NamedSingle(s.Principal.X); w.NamedSingle(s.Principal.Y); w.NamedSingle(s.Principal.Z);
            w.NamedSingle(s.InertiaRotation.X); w.NamedSingle(s.InertiaRotation.Y); w.NamedSingle(s.InertiaRotation.Z); w.NamedSingle(s.InertiaRotation.W);
            w.NamedDouble(s.TargetPosition.X); w.NamedDouble(s.TargetPosition.Y); w.NamedDouble(s.TargetPosition.Z);
            w.NamedSingle(s.TargetRotation.X); w.NamedSingle(s.TargetRotation.Y); w.NamedSingle(s.TargetRotation.Z); w.NamedSingle(s.TargetRotation.W);
        }
    }
    public void Read(StateReader r) => Read(r, _world.Bodies);
    internal void Read(StateReader r, BodyStore bodies)
    {
        if (r.ReadInt32() != 1) throw r.Error("unsupported body control version");
        float angular = r.ReadSingle(); if (!float.IsFinite(angular) || angular < 0) throw r.Error("invalid default angular speed");
        int count = r.ReadCount(1, "body controls"); if (count > bodies.Count) throw r.Error("too many body controls");
        if (_staging.Length < bodies.HighWater) Array.Resize(ref _staging, bodies.HighWater); Array.Clear(_staging);
        int last = -1; int locks = 0;
        for (int i = 0; i < count; i++)
        {
            int b = r.ReadInt32(); if (b <= last || b >= bodies.HighWater || !bodies.IsLive(b)) throw r.Error("invalid body control slot"); last = b;
            var constraints = (RigidbodyConstraints)r.ReadInt32(); float linear = r.ReadSingle(), spin = r.ReadSingle(), push = r.ReadSingle();
            BodyControlSettings settings;
            try { settings = new(constraints, linear, spin, push); ValidateAxes(settings, bodies.Layout.PositionLanes); } catch (ArgumentOutOfRangeException) { throw r.Error("invalid body control settings"); }
            var s = new BodyControlState { Settings = settings, Overrides = r.ReadByte(), Mass = r.ReadSingle(), Center = new(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()), Principal = new(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()), InertiaRotation = new(r.ReadSingle(), r.ReadSingle(), r.ReadSingle(), r.ReadSingle()), TargetPosition = new(r.ReadDouble(), r.ReadDouble(), r.ReadDouble()), TargetRotation = new(r.ReadSingle(), r.ReadSingle(), r.ReadSingle(), r.ReadSingle()) };
            if ((s.Overrides & ~31) != 0 || !s.Present) throw r.Error("invalid body control override flags");
            if (!float.IsFinite(s.Mass) || !Finite(s.Center) || !Finite(s.Principal) || !Finite(s.InertiaRotation) || !Finite(s.TargetRotation) || !double.IsFinite(s.TargetPosition.X) || !double.IsFinite(s.TargetPosition.Y) || !double.IsFinite(s.TargetPosition.Z)) throw r.Error("nonfinite body controls");
            if ((s.Overrides & 7) != 0 && !bodies.IsDynamic(b)) throw r.Error("mass overrides require a dynamic body");
            if ((s.Overrides & 1) != 0 && !ValidMass(s.Mass)) throw r.Error("invalid mass override");
            if ((s.Overrides & 2) != 0 && (Math.Abs(s.Center.X) > 2097152 || Math.Abs(s.Center.Y) > 2097152 || Math.Abs(s.Center.Z) > 2097152)) throw r.Error("invalid center override");
            if ((s.Overrides & 4) != 0 && (!ValidMass(s.Principal.X) || (bodies.Layout.PositionLanes == 3 && (!ValidMass(s.Principal.Y) || !ValidMass(s.Principal.Z) || !Unit(s.InertiaRotation))))) throw r.Error("invalid inertia override");
            if ((s.Overrides & 24) != 0 && bodies.TypeOf(b) != BodyType.Kinematic) throw r.Error("kinematic drive on a non-kinematic body");
            if ((s.Overrides & 8) != 0 && (bodies.TypeOf(b) != BodyType.Kinematic || !Unit(s.TargetRotation))) throw r.Error("invalid kinematic target");
            if (bodies.Layout.PositionLanes == 2 && (s.Center.Z != 0 || s.Principal.Y != 0 || s.Principal.Z != 0 || ((s.Overrides & 8) != 0 && (s.TargetPosition.Z != 0 || s.TargetRotation.X != 0 || s.TargetRotation.Y != 0)))) throw r.Error("nonplanar body controls");
            _staging[b] = s; if (constraints != 0) locks++;
        }
        (Items, _staging) = (_staging, Items); _count = count; _lockCount = locks; DefaultAngularSpeed = angular;
    }
    internal static void ValidateAxes(BodyControlSettings s, int dim) { if (dim == 2 && ((((int)s.Constraints & 4) != 0 && ((int)s.Constraints & 7) != 7) || (((int)s.Constraints & 24) != 0 && ((int)s.Constraints & 56) != 56))) throw new ArgumentOutOfRangeException(nameof(s), "Only X/Y position and Z rotation locks apply in 2D."); }
    internal static bool ValidMass(float x) => x >= 1e-18f && x <= 1e30f;
    private static bool Finite(Vector3F v) => float.IsFinite(v.X) && float.IsFinite(v.Y) && float.IsFinite(v.Z);
    private static bool Finite(QuaternionF q) => float.IsFinite(q.X) && float.IsFinite(q.Y) && float.IsFinite(q.Z) && float.IsFinite(q.W);
    private static bool Unit(QuaternionF q) => Math.Abs((double)q.X*q.X + (double)q.Y*q.Y + (double)q.Z*q.Z + (double)q.W*q.W - 1) < 1e-5;
}
