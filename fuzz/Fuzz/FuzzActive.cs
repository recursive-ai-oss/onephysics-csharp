#pragma warning disable OPHYS0003
using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// Fuzz targets for active ragdolls (balance, walking, animation blending), both dimensions.
// "world2d-active" and "world3d-active" turn bytes into a program: hostile ActiveRagdollDef values (NaN, infinities, huge and negative gains, bone indices out of range or equal), modes, commands, animation poses, blend
// weights, hit reactions, stepping, destroying ragdolls and parts, stale handles and snapshot round trips. Only argument errors and typed physics errors may come out.
// "world2d-active-snapshot" and "world3d-active-snapshot" flip bytes in the snapshot of a world with walking and blended figures (the ACT2 and ACT3 sections): rejected with the world unchanged, or accepted and
// re-serialized to the same bytes.
internal static class FuzzActive
{
    private static int s_ops, s_refused, s_accepted, s_rejected;
    private static byte[]? s_base2, s_base3;

    static FuzzActive()
    {
        AppDomain.CurrentDomain.ProcessExit += (_, _) => Console.WriteLine("active ragdoll targets: ops " + s_ops + ", refused " + s_refused + "; snapshots accepted " + s_accepted + ", rejected " + s_rejected);
    }

    private sealed class Reader
    {
        private readonly byte[] _d; private int _i;
        public Reader(byte[] d) { _d = d; }
        public bool More => _i < _d.Length;
        public byte Byte() => _i < _d.Length ? _d[_i++] : (byte)0;
        public float Float()
        {
            byte b = Byte();
            switch (b) { case 250: return float.NaN; case 251: return float.PositiveInfinity; case 252: return float.NegativeInfinity; case 253: return 3e6f; case 254: return 1e-9f; case 255: return -0f; }
            return (short)(b | (Byte() << 8)) / 256f;
        }
    }

    private static bool Allowed(Exception e) => e is ArgumentException || e is PhysicsException || e is InvalidOperationException;

    private static ActiveRagdollDef Def(Reader r)
    {
        var d = ActiveRagdollDef.Humanoid;
        byte m = r.Byte(), m2 = r.Byte();
        if ((m & 1) != 0) d.PdHertz = r.Float();
        if ((m & 2) != 0) d.LegPdHertz = r.Float();
        if ((m & 4) != 0) d.LoadPdHertz = r.Float();
        if ((m & 8) != 0) d.TargetSlew = r.Float();
        if ((m & 16) != 0) d.SwingTorqueScale = r.Float();
        if ((m & 32) != 0) d.PdDampingRatio = r.Float();
        if ((m & 64) != 0) d.TorqueScale = r.Float();
        if ((m & 128) != 0) d.RootMode = (AnimationRootMode)r.Byte();
        if ((m2 & 1) != 0) d.HitRecoveryTime = r.Float();
        if ((m2 & 2) != 0) d.HitRadius = r.Float();
        if ((m2 & 4) != 0) d.Balance.CaptureGain = r.Float();
        if ((m2 & 8) != 0) d.Balance.StepTrigger = r.Float();
        if ((m2 & 16) != 0) d.Walker.StepTime = r.Float();
        if ((m2 & 32) != 0) d.Walker.HipHeight = r.Float();
        if ((m2 & 64) != 0) d.Walker.PlacementGain = r.Float();
        if ((m2 & 128) != 0) { d.Bones.Pelvis = r.Byte() % 16 - 1; d.Bones.LeftThigh = r.Byte() % 16 - 1; d.Bones.RightFoot = r.Byte() % 16 - 1; }
        if (r.Byte() % 16 == 0) d.Balance = null!;
        return d;
    }

    private static void Guard(Action a, ref int refused)
    {
        try { a(); s_ops++; }
        catch (Exception e) when (Allowed(e)) { refused++; }
    }

    public static void Run2D(byte[] data)
    {
        var o = new P2.WorldOptions2D { WorldId = 21 }; P2.Ragdoll.ApplyRecommendedOptions(o);
        var w = new P2.PhysicsWorld2D(o);
        w.AddBox(new Vector2F(80f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        var rs = new List<P2.Ragdoll>();
        var r = new Reader(data);
        int refused = 0;
        for (int n = 0; r.More && n < 64; n++)
        {
            var g = rs.Count == 0 ? default : rs[r.Byte() % rs.Count];
            Guard(() =>
            {
                switch (r.Byte() % 12)
                {
                    case 0: rs.Add(w.AddRagdoll(P2.RagdollDef.Humanoid, new Vector2D(3 * rs.Count, 0))); break;
                    case 1: g.EnableActive(Def(r)); break;
                    case 2: g.Stand(); break;
                    case 3: g.Walk(r.Float()); break;
                    case 4: g.StopController(); break;
                    case 5: { var pose = new float[r.Byte() % 16]; for (int i = 0; i < pose.Length; i++) pose[i] = r.Float(); g.SetAnimationPose(pose, new Vector2D(r.Float(), r.Float()), r.Float()); break; }
                    case 6: g.SetBlendWeight(r.Byte() % 16 - 1, r.Float()); break;
                    case 7: g.ApplyHitReaction(r.Byte() % 16 - 1, r.Float()); break;
                    case 8: g.Hit(r.Byte() % 16 - 1, new Vector2F(r.Float(), r.Float()), r.Float()); break;
                    case 9: { int k = 1 + r.Byte() % 8; for (int i = 0; i < k; i++) w.StepChecked(1f / 60f); break; }
                    case 10: if (r.Byte() % 3 == 0) g.Remove(); else g.DisableActive(); break;
                    default: { var s = w.Snapshot(); var w2 = new P2.PhysicsWorld2D(o); w2.Restore(s); if (w2.ComputeHash() != w.ComputeHash()) throw new InvalidOperationException("restored world hash differs"); break; }
                }
            }, ref refused);
        }

        s_refused += refused;
    }

    public static void Run3D(byte[] data)
    {
        var o = new P3.WorldOptions3D { WorldId = 22 }; P3.Ragdoll.ApplyRecommendedOptions(o);
        var w = new P3.PhysicsWorld3D(o);
        w.AddBox(new Vector3F(80f, 1f, 80f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var rs = new List<P3.Ragdoll>();
        var r = new Reader(data);
        int refused = 0;
        for (int n = 0; r.More && n < 64; n++)
        {
            var g = rs.Count == 0 ? default : rs[r.Byte() % rs.Count];
            Guard(() =>
            {
                switch (r.Byte() % 12)
                {
                    case 0: rs.Add(w.AddRagdoll(P3.RagdollDef.Humanoid, new Vector3D(3 * rs.Count, 0, 0))); break;
                    case 1: g.EnableActive(Def(r)); break;
                    case 2: g.Stand(r.Float()); break;
                    case 3: g.Walk(new Vector2F(r.Float(), r.Float()), r.Float()); break;
                    case 4: g.StopController(); break;
                    case 5: { var pose = new QuaternionF[r.Byte() % 16]; for (int i = 0; i < pose.Length; i++) pose[i] = new QuaternionF(r.Float(), r.Float(), r.Float(), r.Float()); g.SetAnimationPose(pose, new Vector3D(r.Float(), r.Float(), r.Float()), new QuaternionF(r.Float(), r.Float(), r.Float(), r.Float())); break; }
                    case 6: g.SetBlendWeight(r.Byte() % 16 - 1, r.Float()); break;
                    case 7: g.ApplyHitReaction(r.Byte() % 16 - 1, r.Float()); break;
                    case 8: g.Hit(r.Byte() % 16 - 1, new Vector3F(r.Float(), r.Float(), r.Float()), r.Float()); break;
                    case 9: { int k = 1 + r.Byte() % 8; for (int i = 0; i < k; i++) { try { w.StepChecked(1f / 60f); } catch (P3.BodyOutOfBoundsException e) { e.Body.Destroy(); } } break; }
                    case 10: if (r.Byte() % 3 == 0) g.Remove(); else g.DisableActive(); break;
                    default: { var s = w.Snapshot(); var w2 = new P3.PhysicsWorld3D(o); w2.Restore(s); if (w2.ComputeHash() != w.ComputeHash()) throw new InvalidOperationException("restored world hash differs"); break; }
                }
            }, ref refused);
        }

        s_refused += refused;
    }

    private static byte[] Base2D()
    {
        if (s_base2 != null) return s_base2;
        var o = new P2.WorldOptions2D { WorldId = 21 }; P2.Ragdoll.ApplyRecommendedOptions(o);
        var w = new P2.PhysicsWorld2D(o);
        w.AddBox(new Vector2F(80f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        var a = w.AddRagdoll(P2.RagdollDef.Humanoid, new Vector2D(0, 0)); a.EnableActive(); a.Stand();
        var b = w.AddRagdoll(P2.RagdollDef.Humanoid, new Vector2D(5, 0)); b.EnableActive(); b.Stand();
        var pose = new float[13]; pose[3] = 0.4f; b.SetAnimationPose(pose, new Vector2D(5, 0)); b.SetBlendWeight(3, 0.3f);
        for (int s = 0; s < 80; s++) w.StepChecked(1f / 60f);
        a.Walk(1f); b.Hit(4, new Vector2F(10f, 0f));
        for (int s = 0; s < 60; s++) w.StepChecked(1f / 60f);
        return s_base2 = w.Snapshot();
    }

    private static byte[] Base3D()
    {
        if (s_base3 != null) return s_base3;
        var o = new P3.WorldOptions3D { WorldId = 22 }; P3.Ragdoll.ApplyRecommendedOptions(o);
        var w = new P3.PhysicsWorld3D(o);
        w.AddBox(new Vector3F(80f, 1f, 80f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var a = w.AddRagdoll(P3.RagdollDef.Humanoid, new Vector3D(0, 0, 0)); a.EnableActive(); a.Stand();
        var b = w.AddRagdoll(P3.RagdollDef.Humanoid, new Vector3D(5, 0, 0)); b.EnableActive(); b.Stand();
        var pose = new QuaternionF[13]; Array.Fill(pose, QuaternionF.Identity); pose[3] = new QuaternionF(0.2f, 0f, 0f, 0.98f); b.SetAnimationPose(pose, new Vector3D(5, 0, 0)); b.SetBlendWeight(3, 0.3f);
        for (int s = 0; s < 80; s++) w.StepChecked(1f / 60f);
        a.Walk(new Vector2F(0f, 0.8f), 0f); b.Hit(4, new Vector3F(10f, 0f, 0f));
        for (int s = 0; s < 40; s++) { try { w.StepChecked(1f / 60f); } catch (P3.BodyOutOfBoundsException) { break; } }
        return s_base3 = w.Snapshot();
    }

    private static void Snap(byte[] data, byte[] baseBytes, Func<object> newWorld, Action<object, byte[]> restore, Func<object, byte[]> snap, Action<object> after)
    {
        var bytes = (byte[])baseBytes.Clone();
        int mutations = data.Length > 0 ? 1 + data[0] % 5 : 0;
        for (int i = 1, m = 0; i + 2 < data.Length && m < mutations; i += 3, m++) { int at = ((data[i] << 8) | data[i + 1]) % bytes.Length; bytes[at] ^= data[i + 2]; }
        if (data.Length > 0 && (data[0] & 0x0F) == 0x0F) { Array.Resize(ref bytes, (data[^1] * bytes.Length) / 256); }
        var w = newWorld();
        byte[] empty = snap(w);
        try { restore(w, bytes); }
        catch (SnapshotFormatException)
        {
            s_rejected++;
            if (!snap(w).AsSpan().SequenceEqual(empty)) throw new InvalidOperationException("a rejected snapshot changed the world");
            return;
        }

        s_accepted++;
        if (!snap(w).AsSpan().SequenceEqual(bytes)) throw new InvalidOperationException("accepted snapshot does not re-serialize to the same bytes");
        after(w);
    }

    public static void Snapshot2D(byte[] data) => Snap(data, Base2D(), () => { var o = new P2.WorldOptions2D { WorldId = 21 }; P2.Ragdoll.ApplyRecommendedOptions(o); return new P2.PhysicsWorld2D(o); },
        (w, b) => ((P2.PhysicsWorld2D)w).Restore(b), w => ((P2.PhysicsWorld2D)w).Snapshot(),
        w => { var pw = (P2.PhysicsWorld2D)w; try { pw.StepChecked(1f / 60f); pw.StepChecked(1f / 60f); } catch (P2.BodyOutOfBoundsException e) { e.Body.Destroy(); } });

    public static void Snapshot3D(byte[] data) => Snap(data, Base3D(), () => { var o = new P3.WorldOptions3D { WorldId = 22 }; P3.Ragdoll.ApplyRecommendedOptions(o); return new P3.PhysicsWorld3D(o); },
        (w, b) => ((P3.PhysicsWorld3D)w).Restore(b), w => ((P3.PhysicsWorld3D)w).Snapshot(),
        w => { var pw = (P3.PhysicsWorld3D)w; try { pw.StepChecked(1f / 60f); pw.StepChecked(1f / 60f); } catch (P3.BodyOutOfBoundsException e) { e.Body.Destroy(); } });
}
