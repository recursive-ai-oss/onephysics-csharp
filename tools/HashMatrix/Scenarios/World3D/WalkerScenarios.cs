using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;

namespace RecursiveAI.OnePhysics.HashMatrix;

// The 3D walkers: sixteen figures on the 1.5 m path with the 90 degree turn, a shove on one of them, and a snapshot taken and restored into a fork half-way through a stride. Every per-step world hash is compared.
internal static class WalkerScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("world3d/walkers", ctx =>
        {
            var sc = Walkers3DScene.Create(Walkers3DScene.Terrain.Path, 0, ctx.Threads, 336);
            for (int step = 0; step < 300; step++)
            {
                Walkers3DScene.Command(sc, step);
                if (step == 200) { sc.Rags[3].GetBody(0).ApplyImpulse(new Vector3F(20f, 0f, 0f)); }
                if (step == 233) { var snap = sc.World.Snapshot(); var fork = sc.World.Fork(); fork.Restore(snap); ctx.Write(snap); sc.Rebind(fork); }
                sc.World.Step(Walkers3DScene.Dt);
                ctx.Write((long)sc.World.ComputeHash());
                ctx.EndStep();
            }
        });
    }
}
