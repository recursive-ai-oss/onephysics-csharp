using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed class MachineData2D
{
    public Vector2D FixedA, FixedB;
    public Vector2F[] Points = [];
    public bool Loop, Align;
    public JointAxisSettings X, Y, Angle;
    public MachineData2D Copy() => new() { FixedA = FixedA, FixedB = FixedB, Points = (Vector2F[])Points.Clone(), Loop = Loop, Align = Align, X = X, Y = Y, Angle = Angle };
    public static void ValidateAxis(JointAxisSettings a, bool angular)
    {
        if (a.Mode > JointAxisMode.Free || !float.IsFinite(a.Lower) || !float.IsFinite(a.Upper) || a.Lower > a.Upper || (angular && (a.Lower < -3.1415927f || a.Upper > 3.1415927f)))
        { throw new InvalidJointException("Configurable axis requires a valid mode and ordered finite limits (angular limits within -pi..pi)."); }
        if (!float.IsFinite(a.MotorSpeed) || !float.IsFinite(a.MaxMotor) || a.MaxMotor < 0f || !float.IsFinite(a.SpringHertz) || a.SpringHertz < 0f || !float.IsFinite(a.SpringDampingRatio) || a.SpringDampingRatio < 0f)
        { throw new InvalidJointException("Configurable motor speed must be finite; motor cap, spring frequency and damping must be finite and nonnegative."); }
    }
    public static void ValidatePoint(Vector2D p)
    {
        if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || Math.Abs(p.X) > 2097152.0 || Math.Abs(p.Y) > 2097152.0)
        { throw new ArgumentOutOfRangeException(nameof(p), p, "Joint point must be finite and within +-2^21."); }
    }
    public void Validate(JointType2D type)
    {
        ValidatePoint(FixedA); ValidatePoint(FixedB);
        ValidateAxis(X, false); ValidateAxis(Y, false); ValidateAxis(Angle, true);
        if (type == JointType2D.Path)
        {
            if (Points.Length < 2 || Points.Length > 256) { throw new InvalidJointException("Path requires 2..256 points."); }
            bool distinct = false;
            for (int i = 0; i < Points.Length; i++) { ValidatePoint(new(Points[i].X, Points[i].Y)); if (i > 0 && (Points[i] - Points[0]).LengthSquared() > 1e-12f) { distinct = true; } }
            if (!distinct) { throw new InvalidJointException("Path points must not all coincide."); }
        }
        if (type == JointType2D.Configurable) { ValidateAxis(X, false); ValidateAxis(Y, false); ValidateAxis(Angle, true); }
    }
}

internal sealed partial class Engine2D
{
    internal MachineData2D?[] Machines = new MachineData2D?[16];
    private MachineSection? _machineSection;
    internal ISnapshotSection MachineSnapshotSection => _machineSection ??= new MachineSection(this);
    internal void SetMachine(int c, MachineData2D data)
    {
        if (Machines.Length <= c) { Array.Resize(ref Machines, Math.Max(c + 1, Machines.Length * 2)); }
        Machines[c] = data;
    }
    internal void CheckMachineShift(ReadOnlySpan<double> offset)
    {
        var cs = Core.Constraints;
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Joint || Joints[c].Type != (byte)JointType2D.Pulley) { continue; }
            var m = Machines[c]!;
            MachineData2D.ValidatePoint(new(m.FixedA.X - offset[0], m.FixedA.Y - offset[1]));
            MachineData2D.ValidatePoint(new(m.FixedB.X - offset[0], m.FixedB.Y - offset[1]));
        }
    }
    internal void ShiftMachineJoints(ReadOnlySpan<double> offset)
    {
        var cs = Core.Constraints;
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Joint || Joints[c].Type != (byte)JointType2D.Pulley) { continue; }
            var m = Machines[c]!; m.FixedA -= new Vector2D(offset[0], offset[1]); m.FixedB -= new Vector2D(offset[0], offset[1]);
        }
    }
    private static bool NeedsMachine(byte type) => type == (byte)JointType2D.Pulley || type == (byte)JointType2D.Path || type == (byte)JointType2D.Configurable;
    private sealed class MachineSection(Engine2D e) : ISnapshotSection
    {
        public int SectionId => 0x4A324458;
        public bool IsOptional => true;
        public bool IsPresent
        {
            get { var cs = e.Core.Constraints; for (int c = 0; c < cs.HighWater; c++) { if (cs.IsLive(c) && cs.KindOf(c) == ConstraintKind.Joint && NeedsMachine(e.Joints[c].Type)) { return true; } } return false; }
        }
        public void Reset() { Array.Clear(e.Machines); }
        public ISnapshotSection Fork(WorldCore destination) => ((Engine2D)destination.GetSnapshotSection(Id)).MachineSnapshotSection;
        public void Write(StateWriter w)
        {
            w.DescribeEntity("world", -1);
            w.NamedUInt32(1);
            int count = 0; var cs = e.Core.Constraints;
            for (int c = 0; c < cs.HighWater; c++) { if (cs.IsLive(c) && cs.KindOf(c) == ConstraintKind.Joint && NeedsMachine(e.Joints[c].Type)) { count++; } }
            w.NamedInt32(count);
            for (int c = 0; c < cs.HighWater; c++)
            {
                if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Joint || !NeedsMachine(e.Joints[c].Type)) { continue; }
                var d = e.Machines[c]!;
                w.DescribeEntity("joint", c, cs.Handles.GenerationOf(c));
                w.NamedInt32(c); w.NamedByte(e.Joints[c].Type);
                w.NamedDouble(d.FixedA.X); w.NamedDouble(d.FixedA.Y); w.NamedDouble(d.FixedB.X); w.NamedDouble(d.FixedB.Y);
                w.NamedBool(d.Loop); w.NamedBool(d.Align);
                WriteAxis(w, d.X); WriteAxis(w, d.Y); WriteAxis(w, d.Angle);
                w.NamedInt32(d.Points.Length);
                foreach (var p in d.Points) { w.NamedSingle(p.X); w.NamedSingle(p.Y); }
            }
        }
        public void Read(StateReader r)
        {
            if (r.ReadUInt32() != 1) { throw r.Error("unknown planar machine section version"); }
            int count = r.ReadCount(1, "machine joints");
            var data = new MachineData2D?[Math.Max(16, e.Joints.Length)];
            int last = -1;
            for (int i = 0; i < count; i++)
            {
                int c = r.ReadInt32(); byte type = r.ReadByte();
                if (c <= last || c >= e.Joints.Length || !NeedsMachine(type) || e.Joints[c].Type != type) { throw r.Error("machine joint slot/type mismatch"); }
                last = c;
                var d = new MachineData2D { FixedA = new(r.ReadDouble(), r.ReadDouble()), FixedB = new(r.ReadDouble(), r.ReadDouble()), Loop = r.ReadBool(), Align = r.ReadBool(), X = ReadAxis(r), Y = ReadAxis(r), Angle = ReadAxis(r) };
                int n = r.ReadCount(8, "path points");
                if (n > 256) { throw r.Error("too many path points"); }
                d.Points = new Vector2F[n]; for (int k = 0; k < n; k++) { d.Points[k] = new(r.ReadSingle(), r.ReadSingle()); }
                try { d.Validate((JointType2D)type); }
                catch (Exception ex) when (ex is ArgumentException or PhysicsException) { throw r.Error("invalid planar machine joint: " + ex.Message); }
                data[c] = d;
            }
            e.Machines = data;
        }
        private static void WriteAxis(StateWriter w, JointAxisSettings a)
        {
            w.NamedByte((byte)a.Mode); w.NamedSingle(a.Lower); w.NamedSingle(a.Upper); w.NamedBool(a.EnableMotor); w.NamedSingle(a.MotorSpeed); w.NamedSingle(a.MaxMotor);
            w.NamedBool(a.EnableSpring); w.NamedSingle(a.SpringHertz); w.NamedSingle(a.SpringDampingRatio);
        }
        private static JointAxisSettings ReadAxis(StateReader r) => new() { Mode = (JointAxisMode)r.ReadByte(), Lower = r.ReadSingle(), Upper = r.ReadSingle(), EnableMotor = r.ReadBool(), MotorSpeed = r.ReadSingle(), MaxMotor = r.ReadSingle(), EnableSpring = r.ReadBool(), SpringHertz = r.ReadSingle(), SpringDampingRatio = r.ReadSingle() };
    }
}
