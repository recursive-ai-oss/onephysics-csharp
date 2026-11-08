using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class ReplayScenarios
{
    [ModuleInitializer]
    internal static void Register()
    {
        Scenarios.Register("replay/checkpoint-2d", Planar);
        Scenarios.Register("replay/checkpoint-3d", Spatial);
    }

    private static void Planar(ScenarioContext context)
    {
        using var input = typeof(ReplayScenarios).Assembly.GetManifestResourceStream("replay.planar")
            ?? throw new InvalidOperationException("Missing planar replay fixture.");
        var player = new ReplayPlayer(input, "planar.oppr");
        var world = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = context.Threads });
        while (player.ReadNextStep(world))
        {
            context.Write(world.StepCount);
            context.Write(unchecked((long)world.ComputeHash()));
            context.EndStep();
        }
    }

    private static void Spatial(ScenarioContext context)
    {
        using var input = typeof(ReplayScenarios).Assembly.GetManifestResourceStream("replay.spatial")
            ?? throw new InvalidOperationException("Missing spatial replay fixture.");
        var player = new ReplayPlayer(input, "spatial.oppr");
        var world = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = context.Threads });
        while (player.ReadNextStep(world))
        {
            context.Write(world.StepCount);
            context.Write(unchecked((long)world.ComputeHash()));
            context.EndStep();
        }
    }
}
