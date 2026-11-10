using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class WorldScaleScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        // Streamed terrain tiles with a moving window (3D with a car and walkers, 2D with walkers), a level-of-detail world of box walls with a walking focus, and a slow-motion bubble beside normal time; each snapshotted and restored half-way.
        Scenarios.Register("worldscale/streaming-3d", ctx => WorldScaleScenes.RunStreaming3D(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }, 240));
        Scenarios.Register("worldscale/streaming-2d", ctx => WorldScaleScenes.RunStreaming2D(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }, 300));
        Scenarios.Register("worldscale/lod-3d", ctx => WorldScaleScenes.RunLod3D(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }, 300));
        Scenarios.Register("worldscale/timescale-2d", ctx => WorldScaleScenes.RunTimeScale2D(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }, 400));
    }
}
