using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class Strip2DScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("world2d/strip-2d", ctx => Strip2DScene.Run(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }, 200));
    }
}
