using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class RailBikeScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        // The rail and bicycle stability scenes with their scripts, snapshotted and restored into a fork half-way (a bicycle that falls is destroyed by the script, so the run covers removal too).
        foreach (var (name, steps) in new[] { ("rail-3d", 240), ("bicycles-3d", 300) })
        {
            string n = name; int total = steps;
            Scenarios.Register("world3d/" + n, ctx =>
            {
                var s = RailBikeScenes3D.Build(n, ctx.Threads);
                var w = s.World;
                for (int i = 0; i < total; i++)
                {
                    if (i == 120) { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); ctx.Write(snap); s.Rebind(w); }
                    s.Script(i);
                    w.Step(RailBikeScenes3D.Dt);
                    ctx.Write((long)w.ComputeHash()); ctx.EndStep();
                }
            });
        }

        foreach (var (name, steps) in new[] { ("rail-2d", 240), ("bicycles-2d", 240) })
        {
            string n = name; int total = steps;
            Scenarios.Register("world2d/" + n, ctx =>
            {
                var s = RailBikeScenes2D.Build(n, ctx.Threads);
                var w = s.World;
                for (int i = 0; i < total; i++)
                {
                    if (i == 120) { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); ctx.Write(snap); s.Rebind(w); }
                    s.Script(i);
                    w.Step(RailBikeScenes2D.Dt);
                    ctx.Write((long)w.ComputeHash()); ctx.EndStep();
                }
            });
        }
    }
}
