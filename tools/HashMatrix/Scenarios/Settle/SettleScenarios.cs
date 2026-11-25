using System;
using System.IO;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Replay;
using RecursiveAI.OnePhysics.Rods;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

// Settle, stability margin, rest shapes and the insertion check: the world hash after every slice of a settle, and every number the reports carry.
internal static class SettleScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("settle/pile-3d", Pile3);
        Scenarios.Register("settle/pile-2d", Pile2);
        Scenarios.Register("settle/string-3d", String3);
        Scenarios.Register("settle/string-2d", String2);
    }

    private static void Write(ScenarioContext ctx, SettleReport r)
    {
        ctx.Write(r.Converged ? 1 : 0); ctx.Write(r.StepsUsed); ctx.Write(r.BodiesJudged);
        ctx.Write(r.WorstSpeed); ctx.Write(r.WorstAngularSpeed); ctx.Write(r.WorstForce); ctx.Write(r.WorstDeformableSpeed);
        ctx.Write(r.UnsupportedCount); ctx.Write(r.OutOfBoundsCount);
        ctx.Write(r.WorstSpeedBody.Index); ctx.Write(r.WorstForceBody.Index);
        if (r.Bodies != null) foreach (var b in r.Bodies) { ctx.Write(b.Body.Index); ctx.Write(b.Speed); ctx.Write(b.AngularSpeed); ctx.Write(b.Force); ctx.Write(b.IsSupported ? 1 : 0); }
    }

    private static void Pile3(ScenarioContext ctx)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 801, ThreadCount = ctx.Threads });
        w.AddBox(new(40, 1, 40), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var tall = w.AddBox(new(1, 2, 1), new Vector3D(6, 1.0005, 0), material: new P3.Material3D(2f, 0f));
        for (int i = 0; i < 12; i++) w.AddBox(new(0.5f, 0.5f, 0.5f), new Vector3D(i % 3 * 0.6, 0.3 + 0.7 * (i / 3), (i % 2) * 0.1));
        w.AddSphere(0.4f, new Vector3D(-3, 2, 0));
        // Slices of a settle that cannot converge: the hash after each is the trajectory.
        for (int k = 0; k < 30; k++)
        {
            var r = w.Settle(new P3.SettleOptions3D { MaxSteps = 10, StableSteps = 100000, ExtraLinearDamping = 0.25f });
            Write(ctx, r); ctx.Write((long)w.ComputeHash()); ctx.EndStep();
        }
        var done = w.Settle(new P3.SettleOptions3D { ReportBodies = true, ExtraLinearDamping = 0.5f, Bounds = (new Vector3D(-20, -1, -20), new Vector3D(20, 20, 20)) });
        Write(ctx, done); ctx.Write((long)w.ComputeHash()); ctx.EndStep();
        var m = tall.GetStabilityMargin();
        ctx.Write(m.HasSupport ? 1 : 0); ctx.Write(m.IsStable ? 1 : 0); ctx.Write(m.Margin); ctx.Write(m.MinTipAngle); ctx.Write(m.MinTippingForce); ctx.Write(m.SlidingForce);
        for (int i = 0; i < m.TipAngles.Length; i++) { ctx.Write(m.TipAngles[i]); ctx.Write(m.TippingForces[i]); }
        foreach (var p in m.SupportPolygon) { ctx.Write(p.X); ctx.Write(p.Y); ctx.Write(p.Z); }
        ctx.EndStep();
        // Insertion of a peg into a pocket made of three static boxes.
        var pocket = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 802, ThreadCount = ctx.Threads });
        pocket.AddBox(new(2, 1, 2), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        pocket.AddBox(new(0.2f, 0.5f, 0.62f), new Vector3D(0.21, 0.25, 0), type: BodyType.Static);
        pocket.AddBox(new(0.2f, 0.5f, 0.62f), new Vector3D(-0.21, 0.25, 0), type: BodyType.Static);
        pocket.AddBox(new(0.22f, 0.5f, 0.2f), new Vector3D(0, 0.25, 0.21), type: BodyType.Static);
        pocket.AddBox(new(0.22f, 0.5f, 0.2f), new Vector3D(0, 0.25, -0.21), type: BodyType.Static);
        var peg = pocket.AddBox(new(0.2f, 0.4f, 0.2f), new Vector3D(0.003, 0.8, 0));
        foreach (float d in new[] { 0.6f, 0.7f })
        {
            var ins = peg.CheckInsertion(new Vector3F(0, -1, 0), d);
            ctx.Write(ins.Seats ? 1 : 0); ctx.Write(ins.IsBlocked ? 1 : 0); ctx.Write(ins.FirstContactDistance); ctx.Write(ins.MinClearance); ctx.Write(ins.MinClearanceDistance); ctx.Write(ins.TravelDistance);
            foreach (var c in ins.ClearanceProfile) ctx.Write(c);
        }
        ctx.EndStep();
        // A recording that contains a settle plays back to the same world.
        var source = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 803, ThreadCount = ctx.Threads });
        source.AddBox(new(40, 1, 40), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 5; i++) source.AddBox(new(1, 1, 1), new Vector3D(0.1 * i, 0.5 + 1.3 * i, 0));
        using var stream = new MemoryStream();
        using (var recorder = source.StartRecording(stream))
        {
            for (int i = 0; i < 4; i++) source.Step(1f / 60);
            Write(ctx, source.Settle(new P3.SettleOptions3D { ExtraLinearDamping = 0.5f }));
            source.Step(1f / 60);
        }
        stream.Position = 0;
        var player = new ReplayPlayer(stream, "settle-3d");
        var copy = player.CreateWorld3D(ctx.Threads);
        while (player.ReadNextStep(copy)) { ctx.Write(copy.StepCount); ctx.Write((long)copy.ComputeHash()); }
        if (copy.ComputeHash() != source.ComputeHash()) throw new InvalidOperationException("A recorded settle did not replay to the same world.");
        ctx.EndStep();
    }

    private static void Pile2(ScenarioContext ctx)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 811, ThreadCount = ctx.Threads });
        w.AddBox(new(40, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        var tall = w.AddBox(new(1, 2), new Vector2D(6, 1.0005), material: new P2.Material2D(2f, 0f));
        for (int i = 0; i < 10; i++) w.AddBox(new(0.5f, 0.5f), new Vector2D(i % 3 * 0.6, 0.3 + 0.7 * (i / 3)));
        w.AddCircle(0.4f, new Vector2D(-3, 2));
        for (int k = 0; k < 30; k++)
        {
            var r = w.Settle(new P2.SettleOptions2D { MaxSteps = 10, StableSteps = 100000, ExtraLinearDamping = 0.25f });
            Write(ctx, r); ctx.Write((long)w.ComputeHash()); ctx.EndStep();
        }
        var done = w.Settle(new P2.SettleOptions2D { ReportBodies = true, ExtraLinearDamping = 0.5f, Bounds = (new Vector2D(-20, -1), new Vector2D(20, 20)) });
        Write(ctx, done); ctx.Write((long)w.ComputeHash()); ctx.EndStep();
        var m = tall.GetStabilityMargin();
        ctx.Write(m.HasSupport ? 1 : 0); ctx.Write(m.IsStable ? 1 : 0); ctx.Write(m.Margin); ctx.Write(m.MinTipAngle); ctx.Write(m.MinTippingForce); ctx.Write(m.SlidingForce);
        for (int i = 0; i < m.TipAngles.Length; i++) { ctx.Write(m.TipAngles[i]); ctx.Write(m.TippingForces[i]); }
        foreach (var p in m.SupportPolygon) { ctx.Write(p.X); ctx.Write(p.Y); }
        ctx.EndStep();
        var slot = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 812, ThreadCount = ctx.Threads });
        slot.AddBox(new(2, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        slot.AddBox(new(0.2f, 0.5f), new Vector2D(0.21, 0.25), type: BodyType.Static);
        slot.AddBox(new(0.2f, 0.5f), new Vector2D(-0.21, 0.25), type: BodyType.Static);
        var peg = slot.AddBox(new(0.2f, 0.4f), new Vector2D(0.003, 0.8));
        foreach (float d in new[] { 0.6f, 0.7f })
        {
            var ins = peg.CheckInsertion(new Vector2F(0, -1), d);
            ctx.Write(ins.Seats ? 1 : 0); ctx.Write(ins.IsBlocked ? 1 : 0); ctx.Write(ins.FirstContactDistance); ctx.Write(ins.MinClearance); ctx.Write(ins.MinClearanceDistance); ctx.Write(ins.TravelDistance);
            foreach (var c in ins.ClearanceProfile) ctx.Write(c);
        }
        ctx.EndStep();
        var source = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 813, ThreadCount = ctx.Threads });
        source.AddBox(new(40, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 5; i++) source.AddBox(new(1, 1), new Vector2D(0.1 * i, 0.5 + 1.3 * i));
        using var stream = new MemoryStream();
        using (var recorder = source.StartRecording(stream))
        {
            for (int i = 0; i < 4; i++) source.Step(1f / 60);
            Write(ctx, source.Settle(new P2.SettleOptions2D { ExtraLinearDamping = 0.5f }));
            source.Step(1f / 60);
        }
        stream.Position = 0;
        var player = new ReplayPlayer(stream, "settle-2d");
        var copy = player.CreateWorld2D(ctx.Threads);
        while (player.ReadNextStep(copy)) { ctx.Write(copy.StepCount); ctx.Write((long)copy.ComputeHash()); }
        if (copy.ComputeHash() != source.ComputeHash()) throw new InvalidOperationException("A recorded settle did not replay to the same world.");
        ctx.EndStep();
    }

    private static RodMaterial StringMaterial => new(1e9f, 4e8f, 0.001f, 1000f);

    private static void String3(ScenarioContext ctx)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 821, ThreadCount = ctx.Threads });
        int n = 32; var pts = new Vector3D[n + 1];
        for (int i = 0; i <= n; i++) { double x = -2 + 4.0 * i / n; pts[i] = new Vector3D(x, -0.8 * (1 - x * x / 4), 0); }
        var rod = w.AddRod(new P3.RodDef { Points = pts, Material = StringMaterial, PinnedParticles = [0, n], Damping = 1f });
        var cloth = w.AddCloth(2f, 2f, 6, new Vector3D(0, 3, 4), pinCorners: true);
        for (int k = 0; k < 20; k++)
        {
            Write(ctx, w.Settle(new P3.SettleOptions3D { MaxSteps = 20, StableSteps = 100000, ExtraLinearDamping = 1f }));
            ctx.Write((long)w.ComputeHash()); ctx.EndStep();
        }
        foreach (var shape in new[] { w.GetRestShape(rod), w.GetRestShape(cloth) })
        {
            foreach (var p in shape.Positions) { ctx.Write(p.X); ctx.Write(p.Y); ctx.Write(p.Z); }
            ctx.Write(shape.Sag); ctx.Write(shape.ArcLength); ctx.Write(shape.MaxSpeed); ctx.Write(shape.Triangles.Length); ctx.Write(shape.Frames.Length);
            foreach (var f in shape.Frames) { ctx.Write(f.X); ctx.Write(f.Y); ctx.Write(f.Z); ctx.Write(f.W); }
            ctx.EndStep();
        }
    }

    private static void String2(ScenarioContext ctx)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 822, ThreadCount = ctx.Threads });
        int n = 32; var pts = new Vector2D[n + 1];
        for (int i = 0; i <= n; i++) { double x = -2 + 4.0 * i / n; pts[i] = new Vector2D(x, -0.8 * (1 - x * x / 4)); }
        var rod = w.AddRod(new P2.RodDef { Points = pts, Material = StringMaterial, PinnedParticles = [0, n], Damping = 1f });
        for (int k = 0; k < 20; k++)
        {
            Write(ctx, w.Settle(new P2.SettleOptions2D { MaxSteps = 20, StableSteps = 100000, ExtraLinearDamping = 1f }));
            ctx.Write((long)w.ComputeHash()); ctx.EndStep();
        }
        var shape = w.GetRestShape(rod);
        foreach (var p in shape.Positions) { ctx.Write(p.X); ctx.Write(p.Y); }
        ctx.Write(shape.Sag); ctx.Write(shape.ArcLength); ctx.Write(shape.MaxSpeed);
        foreach (var f in shape.Frames) ctx.Write(f);
        ctx.EndStep();
    }
}
