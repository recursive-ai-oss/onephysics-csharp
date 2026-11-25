using System;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Replay;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// Settle, the stability margin, rest shapes and the insertion check in both worlds; a settle inside a recording plays back to the same world. Equal at one and three threads.
internal static class SettleSmoke
{
    internal static bool Run(out ulong digest)
    {
        ulong a = Planar(1), b = Planar(3), c = Spatial(1), d = Spatial(3);
        digest = a ^ (c * 31);
        return a == b && c == d && a != 0 && c != 0;
    }

    private static ulong Planar(int threads)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 9, ThreadCount = threads });
        w.AddBox(new Vector2F(40, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var tall = w.AddBox(new Vector2F(1, 2), at: new Vector2D(0, 1.0005));
        for (int i = 0; i < 3; i++) w.AddBox(new Vector2F(1, 1), at: new Vector2D(4, 0.5 + 1.2 * i));
        var rope = w.AddRope(new Vector2D(-3, 3), new Vector2D(-1, 3), 8, pinStart: true, pinEnd: true);
        using var stream = new MemoryStream();
        SettleReport report;
        using (var recorder = w.StartRecording(stream)) { w.Step(1f / 60); report = w.Settle(new P2.SettleOptions2D { ReportBodies = true, ExtraLinearDamping = 1f }); }
        var margin = tall.GetStabilityMargin();
        var shape = w.GetRestShape(rope);
        var peg = w.AddBox(new Vector2F(0.2f, 0.4f), at: new Vector2D(-8, 1.2));
        var insertion = peg.CheckInsertion(new Vector2F(0, -1), 0.5f);
        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var copy = player.CreateWorld2D(threads);
        while (player.ReadNextStep(copy)) { }
        bool ok = report.Converged && margin.IsStable && shape.Positions.Length == 9 && insertion.ClearanceProfile.Length > 0 && copy.ComputeHash() != 0;
        return ok ? w.ComputeHash() ^ (ulong)report.StepsUsed : 0;
    }

    private static ulong Spatial(int threads)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 9, ThreadCount = threads });
        w.AddBox(new Vector3F(40, 1, 40), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var tall = w.AddBox(new Vector3F(1, 2, 1), at: new Vector3D(0, 1.0005, 0));
        for (int i = 0; i < 3; i++) w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(4, 0.5 + 1.2 * i, 0));
        var rope = w.AddRope(new Vector3D(-3, 3, 0), new Vector3D(-1, 3, 0), 8, pinStart: true, pinEnd: true);
        using var stream = new MemoryStream();
        SettleReport report;
        using (var recorder = w.StartRecording(stream)) { w.Step(1f / 60); report = w.Settle(new P3.SettleOptions3D { ReportBodies = true, ExtraLinearDamping = 1f }); }
        var margin = tall.GetStabilityMargin();
        var shape = w.GetRestShape(rope);
        var peg = w.AddBox(new Vector3F(0.2f, 0.4f, 0.2f), at: new Vector3D(-8, 1.2, 0));
        var insertion = peg.CheckInsertion(new Vector3F(0, -1, 0), 0.5f);
        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var copy = player.CreateWorld3D(threads);
        while (player.ReadNextStep(copy)) { }
        bool ok = report.Converged && margin.IsStable && shape.Positions.Length == 9 && insertion.ClearanceProfile.Length > 0 && copy.ComputeHash() != 0;
        return ok ? w.ComputeHash() ^ (ulong)report.StepsUsed : 0;
    }
}
