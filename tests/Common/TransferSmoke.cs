using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// Body-set transfer in both worlds: a stack and a ragdoll leave one world as bytes and arrive in another, which then steps like the first; equal at one and three threads.
internal static class TransferSmoke
{
    internal static bool Run(out ulong digest)
    {
        ulong a = Planar(1), b = Planar(3), c = Spatial(1), d = Spatial(3);
        digest = a ^ (c * 31);
        return a == b && c == d && a != 0 && c != 0;
    }

    private static ulong Planar(int threads)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 9, ThreadCount = threads, SleepEnabled = false });
        w.AddBox(new Vector2F(40, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 4; i++) w.AddBox(new Vector2F(1, 1), at: new Vector2D(0.02 * i, 0.5 + 1.001 * i));
        w.AddRagdoll(P2.RagdollDef.Humanoid, new Vector2D(6, 1.2), 1.5f);
        for (int i = 0; i < 30; i++) w.Step(1f / 60);
        var all = new System.Collections.Generic.List<P2.Body>();
        foreach (var body in w.Bodies) { if (body.Type == BodyType.Dynamic) all.Add(body); }
        var packet = BodySetPacket.Read(w.ExtractBodies(all.ToArray(), new BodySetOptions { Move = false }).ToArray());
        var copy = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 9, ThreadCount = threads, SleepEnabled = false });
        copy.AddBox(new Vector2F(40, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var map = copy.InsertBodies(packet);
        for (int i = 0; i < 30; i++) { w.Step(1f / 60); copy.Step(1f / 60); }
        bool same = true;
        for (int i = 0; i < map.NewBodies.Length; i++) { same &= w.GetBody(new BodyHandle(w.Id, map.OldBodies[i].Index, map.OldBodies[i].Generation)).Position == copy.GetBody(map.NewBodies[i]).Position; }
        return same && map.NewRagdolls.Length == 1 ? copy.ComputeHash() : 0;
    }

    private static ulong Spatial(int threads)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 9, ThreadCount = threads, SleepEnabled = false });
        w.AddBox(new Vector3F(40, 1, 40), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 4; i++) w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0.02 * i, 0.5 + 1.001 * i, 0));
        w.AddRagdoll(P3.RagdollDef.Humanoid, new Vector3D(6, 1.2, 0));
        for (int i = 0; i < 30; i++) w.Step(1f / 60);
        var all = new System.Collections.Generic.List<P3.Body>();
        foreach (var body in w.Bodies) { if (body.Type == BodyType.Dynamic) all.Add(body); }
        var packet = BodySetPacket.Read(w.ExtractBodies(all.ToArray(), new BodySetOptions { Move = false }).ToArray());
        var copy = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 9, ThreadCount = threads, SleepEnabled = false });
        copy.AddBox(new Vector3F(40, 1, 40), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var map = copy.InsertBodies(packet);
        for (int i = 0; i < 30; i++) { w.Step(1f / 60); copy.Step(1f / 60); }
        bool same = true;
        for (int i = 0; i < map.NewBodies.Length; i++) { same &= w.GetBody(new BodyHandle(w.Id, map.OldBodies[i].Index, map.OldBodies[i].Generation)).Position == copy.GetBody(map.NewBodies[i]).Position; }
        return same && map.NewRagdolls.Length == 1 ? copy.ComputeHash() : 0;
    }
}
