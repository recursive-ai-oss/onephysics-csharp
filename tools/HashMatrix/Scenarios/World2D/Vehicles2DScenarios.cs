using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class Vehicles2DScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("world2d/vehicles", ctx => Vehicles2DScene.Run(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }));

        // The tracked and motorcycle stability scenes (a shortened run with their scripts), snapshotted and restored half-way.
        foreach (var name in new[] { "tracked-2d", "motorcycle-2d" })
        {
            string n = name;
            Scenarios.Register("world2d/" + n, ctx =>
            {
                var s = Vehicles2DScene.Build(n, ctx.Threads);
                var w = s.World;
                for (int i = 0; i < 160; i++)
                {
                    if (i == 80) { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); ctx.Write(snap); s.Rebind(w); }
                    s.Script(i);
                    w.Step(Vehicles2DScene.Dt);
                    ctx.Write((long)w.ComputeHash()); ctx.EndStep();
                }
            });
        }

        Scenarios.Register("world2d/ragdolls", ctx =>
        {
            var s = Vehicles2DScene.RagdollPileScene(ctx.Threads);
            var w = s.World;
            for (int i = 0; i < 200; i++)
            {
                if (i == 100) { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); ctx.Write(snap); }
                if (i == 140) { w.GetRagdolls()[3].Remove(); }
                w.Step(Vehicles2DScene.Dt);
                ctx.Write((long)w.ComputeHash()); ctx.EndStep();
            }
        });
    }
}
