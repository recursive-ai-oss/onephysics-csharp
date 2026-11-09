using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class SwimScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        // Swimming characters in buoyancy volumes, shallow water and particle fluid, with a pond of floating deformables, a wave-making paddle, deformable queries and contact events;
        // snapshotted and restored into a fresh world half-way.
        Scenarios.Register("world3d/swim", ctx => SwimScenes.Run3D(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }));
        Scenarios.Register("world2d/swim", ctx => SwimScenes.Run2D(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }));
    }
}
