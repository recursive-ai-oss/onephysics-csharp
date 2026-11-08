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
        Scenarios.Register("replay/recorded-2d", RecordedPlanar);
        Scenarios.Register("replay/recorded-3d", RecordedSpatial);
        Scenarios.Register("scene/state-2d", ScenePlanar);
        Scenarios.Register("scene/state-3d", SceneSpatial);
    }

    // Explicit ids keep these scenarios independent of the auto-id sequence used by other scenarios.
    private static void Planar(ScenarioContext context)
    {
        using var input = typeof(ReplayScenarios).Assembly.GetManifestResourceStream("replay.planar")
            ?? throw new InvalidOperationException("Missing planar replay fixture.");
        var player = new ReplayPlayer(input, "planar.oppr");
        var world = new PhysicsWorld2D(new WorldOptions2D { WorldId = 24, ThreadCount = context.Threads });
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
        var world = new PhysicsWorld3D(new WorldOptions3D { WorldId = 24, ThreadCount = context.Threads });
        while (player.ReadNextStep(world))
        {
            context.Write(world.StepCount);
            context.Write(unchecked((long)world.ComputeHash()));
            context.EndStep();
        }
    }
    private static PhysicsWorld2D PlayPlanar(ScenarioContext context, bool emit)
    {
        using var input=typeof(ReplayScenarios).Assembly.GetManifestResourceStream("replay.recorded-planar") ?? throw new InvalidOperationException("Missing recorded planar fixture.");
        var player=new ReplayPlayer(input,"recorded-planar.oppr");var world=player.CreateWorld2D(context.Threads);
        while(player.ReadNextStep(world))if(emit){context.Write(world.StepCount);context.Write(unchecked((long)world.ComputeHash()));context.EndStep();}
        return world;
    }
    private static PhysicsWorld3D PlaySpatial(ScenarioContext context, bool emit)
    {
        using var input=typeof(ReplayScenarios).Assembly.GetManifestResourceStream("replay.recorded-spatial") ?? throw new InvalidOperationException("Missing recorded spatial fixture.");
        var player=new ReplayPlayer(input,"recorded-spatial.oppr");var world=player.CreateWorld3D(context.Threads);
        while(player.ReadNextStep(world))if(emit){context.Write(world.StepCount);context.Write(unchecked((long)world.ComputeHash()));context.EndStep();}
        return world;
    }
    private static void RecordedPlanar(ScenarioContext context)=>PlayPlanar(context,true);
    private static void RecordedSpatial(ScenarioContext context)=>PlaySpatial(context,true);
    private static void ScenePlanar(ScenarioContext context)
    {
        var source=PlayPlanar(context,false);string text=source.SaveScene();
        var copy=PhysicsWorld2D.LoadScene(new System.IO.StringReader(text),threadCount:context.Threads);
        if(text!=copy.SaveScene())throw new InvalidOperationException("Noncanonical planar scene round trip.");
        for(int i=0;i<8;i++){source.Step(1f/60);copy.Step(1f/60);if(source.ComputeHash()!=copy.ComputeHash())throw new InvalidOperationException("Scene lost planar state.");context.Write(unchecked((long)copy.ComputeHash()));context.EndStep();}
    }
    private static void SceneSpatial(ScenarioContext context)
    {
        var source=PlaySpatial(context,false);string text=source.SaveScene();
        var copy=PhysicsWorld3D.LoadScene(new System.IO.StringReader(text),threadCount:context.Threads);
        if(text!=copy.SaveScene())throw new InvalidOperationException("Noncanonical spatial scene round trip.");
        for(int i=0;i<8;i++){source.Step(1f/60);copy.Step(1f/60);if(source.ComputeHash()!=copy.ComputeHash())throw new InvalidOperationException("Scene lost spatial state.");context.Write(unchecked((long)copy.ComputeHash()));context.EndStep();}
    }
}
