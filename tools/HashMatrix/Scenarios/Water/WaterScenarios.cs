using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class WaterScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        // The wave tanks (a boat, a barge, crates and spheres, pulses of water), snapshotted and restored half-way.
        Scenarios.Register("world3d/water", ctx => WaterScenes.Run3D(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }));
        Scenarios.Register("world2d/water", ctx => WaterScenes.Run2D(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }));
    }
}
