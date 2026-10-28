using System.Runtime.CompilerServices;
using Bar;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// The 2D world on the shared scene definitions, hashed after every step: the 20-row pyramid for all 600 steps (it falls asleep early) and the 4000-body mixed pile for its first 300 steps
/// (long enough to land, pile up and start settling), and a mixed scene that also snapshots, restores and shifts the origin halfway.
/// The number of threads comes from the runner and must not change a bit.
/// </summary>
internal static class World2DScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("world2d/pyramid-20", ctx => RunScene(ctx, "pyramid-20", 600));
        Scenarios.Register("world2d/large-pile", ctx => RunScene(ctx, "large-pile", 300));
        Scenarios.Register("world2d/restore-and-shift", RunRestoreAndShift);
    }

    private static void RunScene(ScenarioContext ctx, string name, int steps)
    {
        var scene = Gen.All().First(s => s.Name == name);
        var (world, _, _) = SceneBuilder2D.Build(scene, ctx.Threads);
        float dt = (float)scene.Dt;
        for (int i = 0; i < steps; i++)
        {
            world.Step(dt);
            ctx.Write((long)world.ComputeHash());
            ctx.EndStep();
        }
    }

    private static void RunRestoreAndShift(ScenarioContext ctx)
    {
        var scene = Gen.All().First(s => s.Name == "tumbler");
        var (world, _, _) = SceneBuilder2D.Build(scene, ctx.Threads);
        float dt = (float)scene.Dt;
        byte[]? snap = null;
        for (int i = 0; i < 240; i++)
        {
            if (i == 100) { snap = world.Snapshot(); ctx.Write(snap); }
            if (i == 140) { world.Restore(snap!); }
            if (i == 180) { world.ShiftOrigin(new Vector2D(32.0, -8.0)); }
            world.Step(dt);
            ctx.Write((long)world.ComputeHash());
            ctx.EndStep();
        }
    }
}
