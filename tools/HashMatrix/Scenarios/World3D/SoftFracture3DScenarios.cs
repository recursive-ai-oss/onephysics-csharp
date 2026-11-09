using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class SoftFracture3DScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        foreach (string name in SoftFracture3DScene.Names)
        {
            string scene = name;
            Scenarios.Register("world3d/" + scene, ctx => SoftFracture3DScene.Run(scene, ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }, 200));
        }
    }
}
