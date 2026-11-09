using System.Runtime.CompilerServices;
using Bar;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class MachineScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("world2d/machines", ctx =>
        {
            var w = GenMachines.Build(2).CreateWorld2D(ctx.Threads);
            byte[]? snapshot = null;
            for (int i = 0; i < 600; i++)
            {
                if (i == 100) snapshot = w.Snapshot();
                if (i == 300) w.Restore(snapshot!);
                w.Step(1f / 60f); ctx.Write((long)w.ComputeHash()); ctx.EndStep();
            }
        });
        Scenarios.Register("world3d/machines", ctx =>
        {
            var w = GenMachines.Build(3).CreateWorld3D(ctx.Threads);
            byte[]? snapshot = null;
            for (int i = 0; i < 600; i++)
            {
                if (i == 100) snapshot = w.Snapshot();
                if (i == 300) w.Restore(snapshot!);
                w.Step(1f / 60f); ctx.Write((long)w.ComputeHash()); ctx.EndStep();
            }
        });
    }
}
