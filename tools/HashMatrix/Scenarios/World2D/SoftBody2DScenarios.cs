using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class SoftBody2DScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        foreach (string name in SoftBodies2DScene.Names)
        {
            string scene = name;
            Scenarios.Register("world2d/" + scene, ctx => SoftBodies2DScene.Run(scene, ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }));
        }

        Scenarios.Register("world2d/softfeatures", RunFeatures);
        Scenarios.Register("world2d/softtear", RunTear);
    }

    // Every kind of body and every host hook once: a quadratic shape-matched blob, a skin with pressure, a pinned jelly, a driven pose, a limited target, a rigid body falling on them,
    // then a snapshot and restore, a fork, an origin shift and a changed substep count, with the world hash after every step and the snapshot bytes at the hooks.
    private static void RunFeatures(ScenarioContext ctx)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = ctx.Threads, WorldId = 92 });
        w.AddBox(new Vector2F(40f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        w.AddSoftBody(new SoftBodyDef { Mesh = SoftBodyMesh.Disc(0.5f, 3, 18), Model = SoftBodyModel.Both, MatchQuadratic = true, MatchBlend = 0.4f, MatchCompliance = 1e-4f, EdgeCompliance = 1e-3f, Mass = 3f, Position = new Vector2D(-3, 2) });
        w.AddBalloon(0.5f, new Vector2D(0, 2), 20);
        var hung = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(1f, 1f), 3, 3), new Vector2D(3, 4));
        hung.PinnedParticles = new[] { 12, 15 };
        hung.Mass = 5f;
        w.AddSoftBody(hung);
        var driven = w.AddSoftBox(new Vector2F(0.8f, 0.8f), new Vector2D(6, 1), SoftMaterial.Rubber, 3);
        var limited = w.AddSoftBox(new Vector2F(0.8f, 0.8f), new Vector2D(8, 1), SoftMaterial.Jelly, 3);
        w.AddBox(new Vector2F(0.6f, 0.6f), new Vector2D(0.1, 5), density: 5f);
        for (int i = 0; i < 150; i++)
        {
            if (i == 10) { driven.SetTargetPose(new Vector2D(6, 3), 0.3f, 0f); limited.SetTargetPose(new Vector2D(8, 1), 0f, 0.15f); }
            if (i == 30) { driven.SetTargetPose(new Vector2D(7, 3), -0.3f, 0f); limited.AddVelocity(new Vector2F(4f, 6f)); }
            if (i == 50) { var s = w.Snapshot(); w = w.Fork(); w.Restore(s); ctx.Write(w.Snapshot()); driven = w.GetSoftBody(3); limited = w.GetSoftBody(4); }
            if (i == 70) { w.Options.SoftSubsteps = 6; w.Options.SoftIterations = 2; }
            if (i == 90) { w.ShiftOrigin(new Vector2D(16, -8)); }
            if (i == 110) { driven.ClearTarget(0); w.GetSoftBody(2).SetPinned(12, false); }
            w.Step(1f / 60);
            ctx.Write((long)w.ComputeHash());
            ctx.EndStep();
        }

        ctx.Write(w.Snapshot());
    }

    // Tearing, plasticity and surface contact: a strip pulled apart by driven particles until it tears, a plastic jelly box dropped on a second box (surface contact between bodies), then a snapshot and restore, a fork and an origin shift,
    // with the world hash after every step and the snapshot bytes at the hooks.
    private static void RunTear(ScenarioContext ctx)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = ctx.Threads, WorldId = 93 });
        w.AddBox(new Vector2F(40f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var strip = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(2f, 0.2f), 10, 1), new Vector2D(0, 3));
        strip.EdgeCompliance = 1e-4f; strip.AreaCompliance = 1e-3f; strip.Mass = 1f; strip.TearStrain = 0.15f; strip.GravityScale = 0f;
        var s = w.AddSoftBody(strip);
        var lower = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(1.2f, 0.6f), 4, 2), new Vector2D(6, 0.3));
        lower.Mass = 6f; lower.PlasticYield = 0.05f;
        w.AddSoftBody(lower);
        var upper = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(0.9f, 0.6f), 3, 2), new Vector2D(6.15, 1.5));
        upper.Mass = 4f; upper.PlasticYield = 0.05f; upper.EdgeCompliance = 3e-4f;
        w.AddSoftBody(upper);
        for (int i = 0; i < 200; i++)
        {
            if (i == 90) { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); ctx.Write(w.Snapshot()); s = w.GetSoftBody(0); }
            if (i == 130) { w.ShiftOrigin(new Vector2D(8, -4)); }
            double sep = 0.004 * i;
            for (int j = 0; j <= 1; j++)
            {
                s.SetTarget(j * 11, new Vector2D(-1 - sep / 2 + (i >= 130 ? -8 : 0), 3 - 0.1 + 0.2 * j + (i >= 130 ? 4 : 0)), 0);
                s.SetTarget(j * 11 + 10, new Vector2D(1 + sep / 2 + (i >= 130 ? -8 : 0), 3 - 0.1 + 0.2 * j + (i >= 130 ? 4 : 0)), 0);
            }

            w.Step(1f / 60);
            ctx.Write((long)w.ComputeHash());
            ctx.EndStep();
        }

        ctx.Write(w.Snapshot());
    }
}
