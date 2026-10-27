using System.Runtime.CompilerServices;
using Bar;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// The 3D world on the shared scene definitions, hashed after every step: the 10-layer box pyramid for 300 steps (it falls asleep early), the mixed pile for its first 100 steps,
/// the mesh terrain for 100 steps (landing on triangles), and a pile that is snapshotted, restored and shifted halfway.
/// The number of threads comes from the runner and must not change a bit.
/// </summary>
internal static class World3DScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("world3d/pyramid-10", ctx => RunScene(ctx, "pyramid-3d-10", 300));
        Scenarios.Register("world3d/pile", ctx => RunScene(ctx, "large-pile-3d", 100));
        Scenarios.Register("world3d/terrain", ctx => RunScene(ctx, "mesh-terrain-3d", 100));
        Scenarios.Register("world3d/restore-and-shift", RunRestoreAndShift);
    }

    private static void RunScene(ScenarioContext ctx, string name, int steps)
    {
        var scene = Gen.All().First(s => s.Name == name);
        var (world, _, _) = SceneBuilder3D.Build(scene, ctx.Threads);
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
        var scene = Gen.All().First(s => s.Name == "large-pile-3d");
        var (world, _, _) = SceneBuilder3D.Build(scene, ctx.Threads);
        float dt = (float)scene.Dt;
        byte[]? snap = null;
        for (int i = 0; i < 90; i++)
        {
            if (i == 30) { snap = world.Snapshot(); ctx.Write(snap); }
            if (i == 45) { world.Restore(snap!); }
            if (i == 60) { world.ShiftOrigin(new Vector3D(32.0, -8.0, 16.0)); }
            world.Step(dt);
            ctx.Write((long)world.ComputeHash());
            ctx.EndStep();
        }
    }
}
