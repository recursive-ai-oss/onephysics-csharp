using System.Runtime.CompilerServices;
using RecursiveAI.OnePhysics;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class BodyControlsScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("world2d/body-sleep-and-interpolation", Run2);
        Scenarios.Register("world3d/body-sleep-and-interpolation", Run3);
    }
    private static void Run2(ScenarioContext ctx)
    {
        var w = new P2.PhysicsWorld2D(new() { WorldId = 871, ThreadCount = ctx.Threads });
        w.AddBox(new(40, 1), at: new(0, -0.5), type: BodyType.Static);
        var body = w.AddBox(new(1, 1), at: new(0, 2)); body.SleepSettings = new(0.1f, 0.2f, 0.25f);
        var f = new P2.FixedStepAccumulator(w, 1f / 60, 4);
        for (int i = 0; i < 300; i++)
        {
            if (i == 100) body.NeverSleep = true;
            if (i == 150) { body.NeverSleep = false; body.SleepSettings = new(0.05f, 0.1f, 0.1f); }
            ctx.Write(f.Advance(i % 3 == 0 ? 0.025 : 0.0125));
            var p = f.GetInterpolatedPose(body);
            ctx.Write((long)w.ComputeHash()); ctx.Write(p.Position.X); ctx.Write(p.Position.Y); ctx.Write(p.Rotation.C); ctx.Write(p.Rotation.S); ctx.Write(f.InterpolationAlpha);
            ctx.EndStep();
        }
    }
    private static void Run3(ScenarioContext ctx)
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = 872, ThreadCount = ctx.Threads });
        w.AddBox(new(40, 1, 40), at: new(0, -0.5, 0), type: BodyType.Static);
        var body = w.AddBox(new(1, 1, 1), at: new(0, 2, 0)); body.SleepSettings = new(0.1f, 0.2f, 0.25f);
        var f = new P3.FixedStepAccumulator(w, 1f / 60, 4);
        for (int i = 0; i < 300; i++)
        {
            if (i == 100) body.NeverSleep = true;
            if (i == 150) { body.NeverSleep = false; body.SleepSettings = new(0.05f, 0.1f, 0.1f); }
            ctx.Write(f.Advance(i % 3 == 0 ? 0.025 : 0.0125));
            var p = f.GetInterpolatedPose(body);
            ctx.Write((long)w.ComputeHash()); ctx.Write(p.Position.X); ctx.Write(p.Position.Y); ctx.Write(p.Position.Z);
            ctx.Write(p.Rotation.X); ctx.Write(p.Rotation.Y); ctx.Write(p.Rotation.Z); ctx.Write(p.Rotation.W); ctx.Write(f.InterpolationAlpha);
            ctx.EndStep();
        }
    }
}
