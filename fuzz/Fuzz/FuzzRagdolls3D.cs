using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;

// Fuzz targets for 3D ragdolls.
// "world3d-ragdolls" turns bytes into a program: hostile ragdoll definitions (parent indices that point forward, at themselves or nowhere, several roots, NaN and huge numbers), poses, drive strengths, stepping,
// teleporting and keyframing, destroying parts and whole figures, stale handles and snapshot round trips. Only argument errors and typed physics errors may come out.
// "world3d-ragdoll-snapshot" flips bytes in the snapshot of a world with powered ragdolls: rejected with the world unchanged, or accepted and re-serialized to the same bytes.
internal static class FuzzRagdolls3D
{
    private static int s_ragdolls, s_refused, s_steps, s_accepted, s_rejected;
    private static byte[]? s_base;

    static FuzzRagdolls3D()
    {
        AppDomain.CurrentDomain.ProcessExit += (_, _) => Console.WriteLine("ragdoll targets: ragdolls " + s_ragdolls + ", refused " + s_refused + ", steps " + s_steps + "; snapshots accepted " + s_accepted + ", rejected " + s_rejected);
    }

    private sealed class Reader
    {
        private readonly byte[] _d;
        private int _i;

        public Reader(byte[] d) { _d = d; }

        public bool More => _i < _d.Length;

        public byte Byte() => _i < _d.Length ? _d[_i++] : (byte)0;

        public float Float()
        {
            byte b = Byte();
            switch (b)
            {
                case 250: return float.NaN;
                case 251: return float.PositiveInfinity;
                case 252: return float.NegativeInfinity;
                case 253: return 3e6f;
                case 254: return 1e-9f;
                case 255: return -0f;
            }

            int v = (short)(b | (Byte() << 8));
            return v / 256f;
        }

        public Vector3F V3F() => new(Float(), Float(), Float());

        public QuaternionF Quat() => new(Float(), Float(), Float(), Float());

        public bool Bool() => (Byte() & 1) != 0;
    }

    private static bool Allowed(Exception e) => e is ArgumentException || e is PhysicsException;

    private static void StepTolerant(PhysicsWorld3D w)
    {
        try { w.Step(1f / 60f); s_steps++; }
        catch (BodyOutOfBoundsException e) { e.Body.Destroy(); }
    }

    private static PhysicsWorld3D NewWorld()
    {
        var o = new WorldOptions3D { WorldId = 9, Substeps = 4 };
        Ragdoll.ApplyRecommendedOptions(o);
        return new PhysicsWorld3D(o);
    }

    private static RagdollPartDef Part(Reader r, RagdollPartDef good)
    {
        // Mostly sane, with fields replaced by hostile values on a byte's say-so.
        var p = good;
        byte m = r.Byte();
        if ((m & 1) != 0) { p.Parent = r.Byte() % 20 - 3; }
        if ((m & 2) != 0) { p.Shape = (RagdollShapeKind)(r.Byte() % 5); p.ShapeHalfSize = r.V3F(); }
        if ((m & 4) != 0) { p.Position = r.V3F(); }
        if ((m & 8) != 0) { p.Rotation = r.Quat(); }
        if ((m & 16) != 0) { p.Mass = r.Float(); p.Friction = r.Float(); }
        if ((m & 32) != 0) { p.JointPosition = r.V3F(); p.JointRotation = r.Quat(); }
        if ((m & 64) != 0) { p.LowerTwist = r.Float(); p.UpperTwist = r.Float(); p.SwingYHalfAngle = r.Float(); p.SwingZHalfAngle = r.Float(); }
        if ((m & 128) != 0) { p.DriveStiffness = r.Float(); p.DriveDamping = r.Float(); p.MaxTorque = r.Float(); }
        return p;
    }

    private static RagdollDef Def(Reader r)
    {
        var def = r.Byte() % 3 == 0 ? RagdollDef.CreateHumanoid(0.5f + r.Byte() / 100f, 10f + r.Byte()) : RagdollDef.Humanoid;
        int n = r.Byte() % 6 == 0 ? r.Byte() % 70 : def.Parts.Length;
        if (n != def.Parts.Length)
        {
            var parts = new RagdollPartDef[n];
            for (int i = 0; i < n; i++) { parts[i] = i < def.Parts.Length ? def.Parts[i] : new RagdollPartDef { Parent = i == 0 ? -1 : r.Byte() % i }; }
            def.Parts = parts;
        }

        int edits = r.Byte() % 4;
        for (int e = 0; e < edits && def.Parts.Length > 0; e++)
        {
            int i = r.Byte() % def.Parts.Length;
            def.Parts[i] = Part(r, def.Parts[i]);
        }

        if (r.Byte() % 8 == 0) { def.SelfCollision = (RagdollSelfCollision)(r.Byte() % 4); }
        if (r.Byte() % 8 == 0) { def.AngularDamping = r.Float(); }
        if (r.Byte() % 16 == 0 && def.Parts.Length > 0) { def.Parts[r.Byte() % def.Parts.Length] = null!; }
        return def;
    }

    private static QuaternionF[] Pose(Reader r, int n)
    {
        var p = new QuaternionF[r.Byte() % 8 == 0 ? r.Byte() % 20 : n];
        for (int i = 0; i < p.Length; i++) { p[i] = r.Byte() % 5 == 0 ? r.Quat() : new QuaternionF(r.Float() % 1f, 0f, 0f, 1f); }
        return p;
    }

    private static Ragdoll Pick(List<Ragdoll> rs, Reader r)
    {
        byte b = r.Byte();
        if (rs.Count == 0 || b >= 240) { return default; }
        return rs[b % rs.Count];
    }

    public static void Run(byte[] data)
    {
        var r = new Reader(data);
        var w = NewWorld();
        var floor = w.AddBox(new Vector3F(80f, 1f, 80f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var rs = new List<Ragdoll>();
        int ops = 4 + r.Byte() % 48;
        for (int n = 0; n < ops && r.More; n++)
        {
            try { Op(w, rs, r, floor); }
            catch (Exception e) when (Allowed(e)) { s_refused++; }
        }

        for (int i = 0; i < 4; i++) { StepTolerant(w); }
        byte[] snap = w.Snapshot();
        var w2 = NewWorld();
        try { w2.Restore(snap); }
        catch (SnapshotFormatException e) when (e.Message.Contains("out of range", StringComparison.Ordinal) || e.Message.Contains("non-finite", StringComparison.Ordinal)) { return; }
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world hash differs"); }
        if (!w2.Snapshot().AsSpan().SequenceEqual(snap)) { throw new InvalidOperationException("restored world re-serializes differently"); }
        if (w2.GetRagdolls().Length != w.GetRagdolls().Length) { throw new InvalidOperationException("restored world has a different ragdoll count"); }
        StepTolerant(w); StepTolerant(w2);
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world diverged after a step"); }
    }

    private static void Op(PhysicsWorld3D w, List<Ragdoll> rs, Reader r, Body floor)
    {
        switch (r.Byte() % 11)
        {
            case 0:
            case 1:
                {
                    var def = Def(r);
                    var q = r.Bool() ? default : r.Quat();
                    var pose = r.Byte() % 4 == 0 ? Pose(r, def.Parts?.Length ?? 0) : default;
                    rs.Add(w.AddRagdoll(def, new Vector3D(r.Float() % 20f, 1.0 + (r.Byte() % 4), r.Float() % 20f), q, pose));
                    s_ragdolls++;
                    break;
                }
            case 2:
            case 3: { var g = Pick(rs, r); g.DriveToPose(Pose(r, g.IsValid ? g.PartCount : 0), r.Float() % 2f); break; }
            case 4: { int n = 1 + r.Byte() % 6; for (int i = 0; i < n; i++) { StepTolerant(w); } break; }
            case 5: { var g = Pick(rs, r); g.SetPose(Pose(r, g.IsValid ? g.PartCount : 0), new Vector3D(r.Float(), r.Float(), r.Float()), r.Bool() ? default : r.Quat()); break; }
            case 6: { var g = Pick(rs, r); g.MoveToPose(Pose(r, g.IsValid ? g.PartCount : 0), new Vector3D(r.Float(), r.Float(), r.Float()), default, r.Float()); break; }
            case 7: { var g = Pick(rs, r); if (r.Bool()) { g.Remove(); } else if (g.IsValid) { g.GetBody(r.Byte() % g.PartCount).Destroy(); } break; }
            case 8:
                {
                    var g = Pick(rs, r);
                    if (g.IsValid) { int n = g.PartCount; _ = g.GetJoint(r.Byte() % (n + 1)); _ = g.GetBody(r.Byte() % (n + 1)); _ = g.IsPowered; _ = g.Strength; _ = g.GetPoseError(Pose(r, n)); g.StopDriving(); }
                    else { _ = g.PartCount; }
                    break;
                }
            case 9: { var g = Pick(rs, r); if (g.IsValid) { var j = g.GetJoint(1 + r.Byte() % Math.Max(1, g.PartCount - 1)); if (j.IsValid) { j.SetPoseDrive(r.Quat(), r.Float(), r.Float(), r.Float()); } } break; }
            default:
                {
                    byte[] snap = w.Snapshot();
                    var w2 = NewWorld();
                    try { w2.Restore(snap); }
                    catch (SnapshotFormatException e) when (e.Message.Contains("out of range", StringComparison.Ordinal) || e.Message.Contains("non-finite", StringComparison.Ordinal)) { break; }
                    if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world hash differs"); }
                    break;
                }
        }
    }

    private static byte[] Base()
    {
        if (s_base != null) { return s_base; }
        var w = NewWorld();
        w.AddBox(new Vector3F(80f, 1f, 80f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var pose = new QuaternionF[13];
        Array.Fill(pose, QuaternionF.Identity);
        pose[3] = new QuaternionF(0f, 0f, 0.3f, 0.95f);
        for (int i = 0; i < 3; i++)
        {
            var g = w.AddRagdoll(i == 1 ? RagdollDef.CreateHumanoid(1.3f, 40f) : RagdollDef.Humanoid, new Vector3D(-3 + 3 * i, 0.5 * i, 0));
            if (i != 2) { g.DriveToPose(pose, i == 0 ? 1f : 0.4f); }
        }

        for (int s = 0; s < 40; s++) { w.Step(1f / 60f); }
        return s_base = w.Snapshot();
    }

    public static void SnapshotTarget(byte[] data)
    {
        var bytes = (byte[])Base().Clone();
        int mutations = data.Length > 0 ? 1 + data[0] % 5 : 0;
        for (int i = 1, m = 0; i + 2 < data.Length && m < mutations; i += 3, m++)
        {
            int at = ((data[i] << 8) | data[i + 1]) % bytes.Length;
            bytes[at] ^= data[i + 2];
        }

        if (data.Length > 0 && (data[0] & 0x0F) == 0x0F) { Array.Resize(ref bytes, (data[^1] * bytes.Length) / 256); }

        var w = NewWorld();
        byte[] empty = w.Snapshot();
        try { w.Restore(bytes); }
        catch (SnapshotFormatException)
        {
            s_rejected++;
            if (!w.Snapshot().AsSpan().SequenceEqual(empty)) { throw new InvalidOperationException("a rejected snapshot changed the world"); }
            return;
        }

        s_accepted++;
        if (!w.Snapshot().AsSpan().SequenceEqual(bytes)) { throw new InvalidOperationException("accepted snapshot does not re-serialize to the same bytes"); }
        var pose = new QuaternionF[13];
        Array.Fill(pose, QuaternionF.Identity);
        foreach (var g in w.GetRagdolls()) { if (g.PartCount == 13) { _ = g.GetPoseError(pose); g.DriveToPose(pose, 0.5f); } _ = g.IsPowered; }
        StepTolerant(w);
        StepTolerant(w);
    }
}
