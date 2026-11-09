using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>The saved and live state of one ragdoll: the definition (copied at creation), where each part's body and joint are, and what the pose drive aims at.</summary>
internal sealed class RagdollState
{
    public RagdollDef Def = new();
    public int[] Body = Array.Empty<int>(), BodyGen = Array.Empty<int>();
    public int[] Joint = Array.Empty<int>(), JointGen = Array.Empty<int>();   // constraint slot and generation of the part's joint; -1 for the root
    public float[] Target = Array.Empty<float>();                            // wanted joint angle per part (the pose entry); entry 0 is unused
    public bool Powered;
    public float Strength;

    // Scratch for forward kinematics (not saved), so posing allocates nothing.
    public float[] DeltaAngle = Array.Empty<float>(), PosX = Array.Empty<float>();
    public double[] TargetX = Array.Empty<double>(), TargetY = Array.Empty<double>();
    public float[] TargetAngle = Array.Empty<float>();

    public int Parts => Def.Parts.Length;

    public void Allocate()
    {
        int n = Def.Parts.Length;
        Body = new int[n]; BodyGen = new int[n]; Joint = new int[n]; JointGen = new int[n]; Target = new float[n];
        Array.Fill(Joint, -1);
        Derive();
    }

    public void Derive()
    {
        int n = Def.Parts.Length;
        DeltaAngle = new float[n]; PosX = new float[n]; TargetX = new double[n]; TargetY = new double[n]; TargetAngle = new float[n];
    }

    public RagdollState CloneForFork()
    {
        var c = (RagdollState)MemberwiseClone();
        c.Body = (int[])Body.Clone(); c.BodyGen = (int[])BodyGen.Clone(); c.Joint = (int[])Joint.Clone(); c.JointGen = (int[])JointGen.Clone(); c.Target = (float[])Target.Clone();
        c.DeltaAngle = (float[])DeltaAngle.Clone(); c.PosX = (float[])PosX.Clone(); c.TargetX = (double[])TargetX.Clone(); c.TargetY = (double[])TargetY.Clone(); c.TargetAngle = (float[])TargetAngle.Clone();
        c.Def = Def.Clone();
        return c;
    }
}

internal sealed partial class Engine2D
{
    /// <summary>Slots of the ragdolls (handles with generations, lowest free slot first).</summary>
    public HandleTable RagHandles = new();

    public RagdollState?[] Ragdolls = new RagdollState?[4];

    private int[] _ragOfBody = Array.Empty<int>();

    public bool HasRagdolls => RagHandles.Count > 0;

    /// <summary>Reserves a ragdoll slot; the caller fills <see cref="Ragdolls"/> and calls <see cref="RegisterRagdoll"/> or gives the slot back with <see cref="CancelRagdoll"/>.</summary>
    public int ReserveRagdoll()
    {
        int r = RagHandles.Allocate();
        if (r >= Ragdolls.Length) { Array.Resize(ref Ragdolls, Math.Max(r + 1, Ragdolls.Length * 2)); }
        return r;
    }

    public void CancelRagdoll(int r)
    {
        Ragdolls[r] = null;
        RagHandles.Free(r);
    }

    public void RegisterRagdoll(int r, RagdollState s)
    {
        Ragdolls[r] = s;
        IndexRagdollBodies(r, s);
    }

    private void IndexRagdollBodies(int r, RagdollState s)
    {
        for (int i = 0; i < s.Parts; i++)
        {
            int body = s.Body[i];
            if (body >= _ragOfBody.Length)
            {
                int old = _ragOfBody.Length;
                Array.Resize(ref _ragOfBody, Math.Max(body + 1, old * 2));
                for (int k = old; k < _ragOfBody.Length; k++) { _ragOfBody[k] = -1; }
            }

            _ragOfBody[body] = r;
        }
    }

    /// <summary>The ragdoll slot whose part is the given body slot, or -1.</summary>
    public int RagdollOfBody(int body) => body < _ragOfBody.Length ? _ragOfBody[body] : -1;

    /// <summary>Forgets a ragdoll without touching its bodies and joints.</summary>
    public void ForgetRagdoll(int r)
    {
        var s = Ragdolls[r]!;
        for (int i = 0; i < s.Parts; i++)
        {
            if (s.Body[i] < _ragOfBody.Length && _ragOfBody[s.Body[i]] == r) { _ragOfBody[s.Body[i]] = -1; }
        }

        CancelRagdoll(r);
    }

    /// <summary>Called when a body is destroyed: a ragdoll it belongs to stops being one (its other bodies and joints stay as plain bodies and joints).</summary>
    public void DropRagdollOfBody(int body)
    {
        int r = RagdollOfBody(body);
        if (r >= 0) { ForgetRagdoll(r); }
    }

    /// <summary>True while the part's joint exists and is still the revolute joint the ragdoll made.</summary>
    public bool RagdollJointAlive(RagdollState s, int part)
    {
        int c = s.Joint[part];
        if (c < 0) { return false; }
        var cs = Core.Constraints;
        if (!cs.Handles.IsAlive(c, s.JointGen[part]) || cs.KindOf(c) != ConstraintKind.Joint) { return false; }
        BindJoints();
        return Joints[c].Type == (byte)JointType2D.Revolute;
    }

    /// <summary>The joint angle of a part (child angle minus parent angle minus the bind pose's), radians within -π..π.</summary>
    public float RagdollJointAngle(int c)
    {
        var cs = Core.Constraints;
        var bodies = Core.Bodies;
        var ra = bodies.RotationOf(cs.BodyA[c]); var rb = bodies.RotationOf(cs.BodyB[c]);
        float relC = rb[0] * ra[0] + rb[1] * ra[1], relS = rb[1] * ra[0] - rb[0] * ra[1];
        float refA = Joints[c].RefAngle;
        float cr = Math32.Cos(refA), sr = Math32.Sin(refA);
        return Math32.Atan2(relS * cr - relC * sr, relC * cr + relS * sr);
    }

    private static float WrapPi(float a)
    {
        const float Pi = 3.1415927f;
        if (a > Pi) { a -= TwoPi; }
        else if (a < -Pi) { a += TwoPi; }
        return a;
    }

    /// <summary>The distance, radians, between a part's current joint angle and the pose entry <paramref name="pose"/>.</summary>
    public float RagdollJointError(RagdollState s, int part, float pose)
    {
        BindJoints();
        return Math.Abs(WrapPi(pose - RagdollJointAngle(s.Joint[part])));
    }

    /// <summary>
    /// Pose drive, run once per step before the solver, in ragdoll slot order: each driven joint's motor is aimed at the target angle with a speed proportional to the error (response rate <see cref="RagdollPartDef.DriveHertz"/>)
    /// and limited to the part's torque. The motor is a velocity constraint with a torque cap, so it is stable at any gain and any substep count (Catto, "Soft Constraints", GDC 2011, for the same argument about soft limits).
    /// </summary>
    public void StepRagdolls()
    {
        if (RagHandles.Count == 0) { return; }
        BindJoints();
        var bodies = Core.Bodies;
        var cs = Core.Constraints;
        for (int r = 0; r < RagHandles.HighWater; r++)
        {
            if (!RagHandles.IsLive(r)) { continue; }
            var s = Ragdolls[r]!;
            if (!s.Powered) { continue; }
            for (int i = 1; i < s.Parts; i++)
            {
                if (!RagdollJointAlive(s, i)) { continue; }
                int c = s.Joint[i];
                if (!bodies.IsAwake(cs.BodyA[c]) && !bodies.IsAwake(cs.BodyB[c])) { continue; }
                var p = s.Def.Parts[i];
                float err = WrapPi(s.Target[i] - RagdollJointAngle(c));
                float speed = TwoPi * p.DriveHertz * err;
                if (speed > p.MaxDriveSpeed) { speed = p.MaxDriveSpeed; } else if (speed < -p.MaxDriveSpeed) { speed = -p.MaxDriveSpeed; }
                ref var d = ref Joints[c];
                d.Flags |= JointData2D.FlagMotor;
                d.MotorSpeed = speed;
                d.MaxMotor = p.MaxTorque * s.Strength;
            }
        }
    }

    // ------------------------------------------------------------------ snapshot section

    /// <summary>Id of the 2D ragdoll snapshot section ("RAG2").</summary>
    public const int RagdollSectionId = 0x32474152;

    private RagdollSection? _ragdollSection;

    internal ISnapshotSection RagdollSnapshotSection => _ragdollSection ??= new RagdollSection(this);

    internal void WriteRagdolls(StateWriter w)
    {
        w.NamedInt32(1);   // section version
        w.DescribeTable("ragdoll");
        RagHandles.Write(w);
        for (int r = 0; r < RagHandles.HighWater; r++)
        {
            w.DescribeEntity("ragdoll", r, RagHandles.GenerationOf(r));
            if (!RagHandles.IsLive(r)) { continue; }
            var s = Ragdolls[r]!;
            var d = s.Def;
            w.NamedInt32(d.Parts.Length);
            w.NamedByte((byte)d.SelfCollision); w.NamedSingle(d.AngularDamping); w.NamedUInt32(d.CategoryBits); w.NamedUInt32(d.MaskBits);
            for (int i = 0; i < d.Parts.Length; i++)
            {
                var p = d.Parts[i];
                w.NamedInt32(p.Parent); w.NamedByte((byte)p.Shape);
                w.NamedSingle(p.ShapeHalfSize.X); w.NamedSingle(p.ShapeHalfSize.Y); w.NamedSingle(p.Position.X); w.NamedSingle(p.Position.Y); w.NamedSingle(p.Angle);
                w.NamedSingle(p.Mass); w.NamedSingle(p.Friction);
                w.NamedSingle(p.JointPosition.X); w.NamedSingle(p.JointPosition.Y);
                w.NamedSingle(p.LowerAngle); w.NamedSingle(p.UpperAngle);
                w.NamedSingle(p.DriveHertz); w.NamedSingle(p.MaxDriveSpeed); w.NamedSingle(p.MaxTorque); w.NamedSingle(p.BreakForce); w.NamedSingle(p.BreakTorque);
                w.NamedInt32(s.Body[i]); w.NamedInt32(s.BodyGen[i]); w.NamedInt32(s.Joint[i]); w.NamedInt32(s.JointGen[i]); w.NamedSingle(s.Target[i]);
            }

            w.NamedBool(s.Powered); w.NamedSingle(s.Strength);
        }
    }

    private sealed class RagdollBlock
    {
        public HandleTable Handles = new();
        public RagdollState?[] States = Array.Empty<RagdollState?>();
    }

    /// <summary>Bytes of one part's record in a snapshot (the minimum a count may claim).</summary>
    private const int RagdollPartBytes = 4 + 1 + 5 * 4 + 2 * 4 + 2 * 4 + 2 * 4 + 5 * 4 + 5 * 4;

    private static RagdollBlock ReadRagdolls(StateReader r, int bodyCount)
    {
        int version = r.ReadInt32();
        if (version != 1) { throw r.Error("unsupported 2D ragdoll section version " + version.ToString(CultureInfo.InvariantCulture)); }
        var b = new RagdollBlock { Handles = HandleTable.Read(r, "ragdoll") };
        if (b.Handles.Count == 0) { throw r.Error("a ragdoll section holding no ragdolls"); }
        b.States = new RagdollState?[Math.Max(b.Handles.HighWater, 1)];
        var used = new bool[Math.Max(bodyCount, 1)];
        for (int k = 0; k < b.Handles.HighWater; k++)
        {
            if (!b.Handles.IsLive(k)) { continue; }
            int n = r.ReadCount(RagdollPartBytes, "ragdoll parts");
            if (n < 1 || n > RagdollDef.MaxParts) { throw r.Error("a ragdoll with " + n.ToString(CultureInfo.InvariantCulture) + " parts"); }
            var s = new RagdollState();
            var d = new RagdollDef { Parts = new RagdollPartDef[n] };
            byte self = r.ReadByte();
            if (self > (byte)RagdollSelfCollision.All) { throw r.Error("invalid ragdoll self-collision mode"); }
            d.SelfCollision = (RagdollSelfCollision)self; d.AngularDamping = r.ReadSingle(); d.CategoryBits = r.ReadUInt32(); d.MaskBits = r.ReadUInt32();
            s.Def = d;
            s.Body = new int[n]; s.BodyGen = new int[n]; s.Joint = new int[n]; s.JointGen = new int[n]; s.Target = new float[n];
            for (int i = 0; i < n; i++)
            {
                var p = new RagdollPartDef { Parent = r.ReadInt32() };
                byte shape = r.ReadByte();
                if (shape > (byte)RagdollShapeKind.Box) { throw r.Error("invalid ragdoll shape kind"); }
                p.Shape = (RagdollShapeKind)shape;
                p.ShapeHalfSize = new Vector2F(r.ReadSingle(), r.ReadSingle()); p.Position = new Vector2F(r.ReadSingle(), r.ReadSingle()); p.Angle = r.ReadSingle();
                p.Mass = r.ReadSingle(); p.Friction = r.ReadSingle();
                p.JointPosition = new Vector2F(r.ReadSingle(), r.ReadSingle());
                p.LowerAngle = r.ReadSingle(); p.UpperAngle = r.ReadSingle();
                p.DriveHertz = r.ReadSingle(); p.MaxDriveSpeed = r.ReadSingle(); p.MaxTorque = r.ReadSingle(); p.BreakForce = r.ReadSingle(); p.BreakTorque = r.ReadSingle();
                d.Parts[i] = p;
                s.Body[i] = r.ReadInt32(); s.BodyGen[i] = r.ReadInt32(); s.Joint[i] = r.ReadInt32(); s.JointGen[i] = r.ReadInt32(); s.Target[i] = r.ReadSingle();
                if (s.Body[i] < 0 || s.Body[i] >= bodyCount || s.BodyGen[i] < 1 || used[s.Body[i]]) { throw r.Error("invalid ragdoll part body"); }
                used[s.Body[i]] = true;
                if (p.Parent < 0 ? s.Joint[i] != -1 : (s.Joint[i] < 0 || s.JointGen[i] < 1)) { throw r.Error("invalid ragdoll part joint"); }
                if (!float.IsFinite(s.Target[i]) || Math.Abs(s.Target[i]) > 1e6f) { throw r.Error("invalid ragdoll target angle"); }
            }

            try { d.Validate(); }
            catch (Exception e) when (e is ArgumentException or PhysicsException) { throw r.Error("invalid ragdoll definition: " + e.Message); }
            s.Powered = r.ReadBool(); s.Strength = r.ReadSingle();
            if (!(s.Strength >= 0f && s.Strength <= 1f) || (s.Powered && !(s.Strength > 0f))) { throw r.Error("invalid ragdoll drive strength"); }
            s.Derive();
            b.States[k] = s;
        }

        return b;
    }

    private void CommitRagdolls(RagdollBlock? b)
    {
        Array.Fill(_ragOfBody, -1);
        if (b == null)
        {
            RagHandles.Reset();
            Array.Clear(Ragdolls);
            return;
        }

        RagHandles = b.Handles;
        Ragdolls = b.States;
        for (int r = 0; r < b.Handles.HighWater; r++)
        {
            if (b.Handles.IsLive(r)) { IndexRagdollBodies(r, b.States[r]!); }
        }
    }

    /// <summary>After a restore: every part must sit on a live dynamic body of the generation it names.</summary>
    internal bool RagdollsConsistent()
    {
        var bodies = Core.Bodies;
        for (int r = 0; r < RagHandles.HighWater; r++)
        {
            if (!RagHandles.IsLive(r)) { continue; }
            var s = Ragdolls[r]!;
            for (int i = 0; i < s.Parts; i++)
            {
                int b = s.Body[i];
                if (!bodies.Handles.IsAlive(b, s.BodyGen[i]) || bodies.TypeOf(b) != BodyType.Dynamic || b >= _ragOfBody.Length || _ragOfBody[b] != r) { return false; }
                // A part's joint, while it lives, must be a revolute joint: that is what the pose drive writes to.
                if (s.Joint[i] >= 0)
                {
                    var cs = Core.Constraints;
                    int c = s.Joint[i];
                    if (cs.Handles.IsAlive(c, s.JointGen[i]) && cs.KindOf(c) == ConstraintKind.Joint)
                    {
                        BindJoints();
                        if (Joints[c].Type != (byte)JointType2D.Revolute) { return false; }
                    }
                }
            }
        }

        return true;
    }

    private sealed class RagdollSection(Engine2D e) : ISnapshotSection
    {
        public int SectionId => RagdollSectionId;
        public bool IsOptional => true;
        public bool IsPresent => e.RagHandles.Count > 0;

        public void Reset()
        {
            if (e.RagHandles.HighWater == 0) { return; }
            e.CommitRagdolls(null);
        }

        public void Write(StateWriter w) => e.WriteRagdolls(w);

        public void Read(StateReader r) => e.CommitRagdolls(ReadRagdolls(r, e.RestoredBodyCount));

        public ISnapshotSection Fork(WorldCore destination)
        {
            var dst = (Engine2D)destination.GetSnapshotSection(Engine2D.Id);
            dst.RagHandles = new HandleTable();
            dst.RagHandles.CopyFrom(e.RagHandles);
            dst.Ragdolls = new RagdollState?[e.Ragdolls.Length];
            for (int i = 0; i < dst.Ragdolls.Length; i++) { dst.Ragdolls[i] = e.Ragdolls[i]?.CloneForFork(); }
            dst._ragOfBody = (int[])e._ragOfBody.Clone();
            return dst.RagdollSnapshotSection;
        }
    }
}
