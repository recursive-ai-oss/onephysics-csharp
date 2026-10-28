using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>The saved and live state of one ragdoll: the definition (copied at creation), where each part's body and joint are, and how hard the pose drive works.</summary>
internal sealed class RagdollState
{
    public RagdollDef Def = new();
    public int[] Body = Array.Empty<int>(), BodyGen = Array.Empty<int>();
    public int[] Joint = Array.Empty<int>(), JointGen = Array.Empty<int>();   // constraint slot and generation of the part's joint; -1 for the root
    public bool Powered;
    public float Strength;

    // Derived (not saved): the bind pose normalised, and scratch for forward kinematics so posing allocates nothing.
    public QuaternionF[] BindRot = Array.Empty<QuaternionF>(), JointRot = Array.Empty<QuaternionF>();
    public QuaternionF[] TargetRot = Array.Empty<QuaternionF>();
    public Vector3D[] TargetPos = Array.Empty<Vector3D>();

    public int Parts => Def.Parts.Length;

    public void Allocate()
    {
        int n = Def.Parts.Length;
        Body = new int[n]; BodyGen = new int[n]; Joint = new int[n]; JointGen = new int[n];
        Array.Fill(Joint, -1);
        Derive();
    }

    /// <summary>Rebuilds what follows from the definition: normalised bind rotations and the scratch arrays.</summary>
    public void Derive()
    {
        int n = Def.Parts.Length;
        BindRot = new QuaternionF[n]; JointRot = new QuaternionF[n]; TargetRot = new QuaternionF[n]; TargetPos = new Vector3D[n];
        for (int i = 0; i < n; i++)
        {
            BindRot[i] = Engine3D.Normalize(Def.Parts[i].Rotation, "Rotation");
            JointRot[i] = Engine3D.Normalize(Def.Parts[i].JointRotation, "JointRotation");
        }
    }
}

internal sealed partial class Engine3D
{
    private const uint BlockRagdolls = 16;

    /// <summary>Slots of the ragdolls (handles with generations, lowest free slot first).</summary>
    public HandleTable RagHandles = new();

    public RagdollState?[] Ragdolls = new RagdollState?[4];

    private int[] _ragOfBody = Array.Empty<int>();

    private bool HasRagdolls => RagHandles.Count > 0;

    /// <summary>Reserves a ragdoll slot; the caller fills <see cref="Ragdolls"/> and calls <see cref="IndexRagdollBodies"/> or gives the slot back with <see cref="CancelRagdoll"/>.</summary>
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

    /// <summary>True while the part's joint exists and is still the swing-twist joint the ragdoll made.</summary>
    public bool RagdollJointAlive(RagdollState s, int part)
    {
        int c = s.Joint[part];
        if (c < 0) { return false; }
        var cs = Core.Constraints;
        return cs.Handles.IsAlive(c, s.JointGen[part]) && cs.KindOf(c) == ConstraintKind.Joint;
    }

    /// <summary>The angle, radians, between a part's current relative joint rotation and the pose entry <paramref name="pose"/> (a normalised rotation in the bind pose's axes).</summary>
    public float RagdollJointError(RagdollState s, int part, QuaternionF pose)
    {
        var cs = Core.Constraints;
        int c = s.Joint[part];
        BindJoints();
        ref JointData3D jd = ref Joints[c];
        var fa = RotationOfBody(cs.BodyA[c]) * jd.QA;
        var fb = RotationOfBody(cs.BodyB[c]) * jd.QB;
        var rel = fa.Conjugated() * fb;
        var r = s.JointRot[part];
        var tgt = r.Conjugated() * pose * r;
        var qe = rel * tgt.Conjugated();
        float sn = float.Sqrt(qe.X * qe.X + qe.Y * qe.Y + qe.Z * qe.Z);
        float w = qe.W < 0f ? -qe.W : qe.W;
        return (float)(2.0 * Math64.Atan2(sn, w));
    }

    // ------------------------------------------------------------------ snapshot block

    private void WriteRagdolls(StateWriter w)
    {
        RagHandles.Write(w);
        for (int r = 0; r < RagHandles.HighWater; r++)
        {
            if (!RagHandles.IsLive(r)) { continue; }
            var s = Ragdolls[r]!;
            var d = s.Def;
            w.WriteInt32(d.Parts.Length);
            w.WriteByte((byte)d.SelfCollision); w.WriteSingle(d.AngularDamping); w.WriteUInt32(d.CategoryBits); w.WriteUInt32(d.MaskBits);
            for (int i = 0; i < d.Parts.Length; i++)
            {
                var p = d.Parts[i];
                w.WriteInt32(p.Parent); w.WriteByte((byte)p.Shape);
                WriteVec(w, p.ShapeSize); WriteVec(w, p.Position); WriteQuat(w, p.Rotation);
                w.WriteSingle(p.Mass); w.WriteSingle(p.Friction);
                WriteVec(w, p.JointPosition); WriteQuat(w, p.JointRotation);
                w.WriteSingle(p.LowerTwist); w.WriteSingle(p.UpperTwist); w.WriteSingle(p.SwingYHalfAngle); w.WriteSingle(p.SwingZHalfAngle);
                w.WriteSingle(p.DriveStiffness); w.WriteSingle(p.DriveDamping); w.WriteSingle(p.MaxTorque);
                w.WriteInt32(s.Body[i]); w.WriteInt32(s.BodyGen[i]); w.WriteInt32(s.Joint[i]); w.WriteInt32(s.JointGen[i]);
            }

            w.WriteBool(s.Powered); w.WriteSingle(s.Strength);
        }
    }

    private static void WriteQuat(StateWriter w, QuaternionF q) { w.WriteSingle(q.X); w.WriteSingle(q.Y); w.WriteSingle(q.Z); w.WriteSingle(q.W); }

    private static QuaternionF ReadQuat(StateReader r) => new(r.ReadSingle(), r.ReadSingle(), r.ReadSingle(), r.ReadSingle());

    private sealed class RagdollBlock
    {
        public HandleTable Handles = new();
        public RagdollState?[] States = Array.Empty<RagdollState?>();
    }

    /// <summary>Bytes of one part's record in a snapshot (the minimum a count may claim).</summary>
    private const int RagdollPartBytes = 4 + 1 + 12 + 12 + 16 + 4 + 4 + 12 + 16 + 4 * 4 + 3 * 4 + 4 * 4;

    private static RagdollBlock ReadRagdolls(StateReader r, int bodyCount)
    {
        var b = new RagdollBlock { Handles = HandleTable.Read(r, "ragdoll") };
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
            s.Body = new int[n]; s.BodyGen = new int[n]; s.Joint = new int[n]; s.JointGen = new int[n];
            for (int i = 0; i < n; i++)
            {
                var p = new RagdollPartDef { Parent = r.ReadInt32() };
                byte shape = r.ReadByte();
                if (shape > (byte)RagdollShapeKind.Box) { throw r.Error("invalid ragdoll shape kind"); }
                p.Shape = (RagdollShapeKind)shape;
                p.ShapeSize = ReadVec(r); p.Position = ReadVec(r); p.Rotation = ReadQuat(r);
                p.Mass = r.ReadSingle(); p.Friction = r.ReadSingle();
                p.JointPosition = ReadVec(r); p.JointRotation = ReadQuat(r);
                p.LowerTwist = r.ReadSingle(); p.UpperTwist = r.ReadSingle(); p.SwingYHalfAngle = r.ReadSingle(); p.SwingZHalfAngle = r.ReadSingle();
                p.DriveStiffness = r.ReadSingle(); p.DriveDamping = r.ReadSingle(); p.MaxTorque = r.ReadSingle();
                d.Parts[i] = p;
                s.Body[i] = r.ReadInt32(); s.BodyGen[i] = r.ReadInt32(); s.Joint[i] = r.ReadInt32(); s.JointGen[i] = r.ReadInt32();
                if (s.Body[i] < 0 || s.Body[i] >= bodyCount || s.BodyGen[i] < 1 || used[s.Body[i]]) { throw r.Error("invalid ragdoll part body"); }
                used[s.Body[i]] = true;
                if (p.Parent < 0 ? s.Joint[i] != -1 : (s.Joint[i] < 0 || s.JointGen[i] < 1)) { throw r.Error("invalid ragdoll part joint"); }
            }

            try { d.Validate(); }
            catch (Exception e) when (e is ArgumentException or PhysicsException) { throw r.Error("invalid ragdoll definition: " + e.Message); }
            s.Powered = r.ReadBool(); s.Strength = r.ReadSingle();
            if (!(s.Strength >= 0f && s.Strength <= 1f) || (s.Powered && !(s.Strength > 0f))) { throw r.Error("invalid ragdoll drive strength"); }
            // A stored rotation can be finite yet overflow when squared; Derive rejects it.
            try { s.Derive(); }
            catch (ArgumentException e) { throw r.Error("invalid ragdoll definition: " + e.Message); }
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
    private bool RagdollsConsistent()
    {
        var bodies = Core.Bodies;
        for (int r = 0; r < RagHandles.HighWater; r++)
        {
            if (!RagHandles.IsLive(r)) { continue; }
            var s = Ragdolls![r]!;
            for (int i = 0; i < s.Parts; i++)
            {
                int b = s.Body[i];
                if (!bodies.Handles.IsAlive(b, s.BodyGen[i]) || bodies.TypeOf(b) != BodyType.Dynamic || _ragOfBody[b] != r) { return false; }
                // A part's joint, while it lives, must be a swing-twist joint: that is what the pose drive writes to.
                if (s.Joint[i] >= 0 && RagdollJointAlive(s, i))
                {
                    BindJoints();
                    if (Joints[s.Joint[i]].Type != (int)JointType3D.SwingTwist) { return false; }
                }
            }
        }

        return true;
    }
}
