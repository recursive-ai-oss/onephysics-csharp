using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class ArticulationScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        // The articulation scenes with their scripts, snapshotted and restored into a fork half-way.
        foreach (var (name, steps) in new[] { ("articulation-arm-3d", 360), ("quadruped-3d", 300) })
        {
            string n = name; int total = steps;
            Scenarios.Register("world3d/" + n, ctx =>
            {
                var s = ArticulationScenes3D.Build(n, ctx.Threads);
                var w = s.World;
                for (int i = 0; i < total; i++)
                {
                    if (i == 150) { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); ctx.Write(snap); s.Rebind(w); }
                    s.Script(i);
                    w.Step(ArticulationScenes3D.Dt);
                    ctx.Write((long)w.ComputeHash()); ctx.EndStep();
                }
            });
        }

        Scenarios.Register("world2d/articulation-chain-2d", ctx =>
        {
            var s = ArticulationScenes2D.Build("articulation-chain-2d", ctx.Threads);
            var w = s.World;
            for (int i = 0; i < 300; i++)
            {
                if (i == 150) { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); ctx.Write(snap); s.Rebind(w); }
                s.Script(i);
                w.Step(ArticulationScenes2D.Dt);
                ctx.Write((long)w.ComputeHash()); ctx.EndStep();
            }
        });
    }
}
