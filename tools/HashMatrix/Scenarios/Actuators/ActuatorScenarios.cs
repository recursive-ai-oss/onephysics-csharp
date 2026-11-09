using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class ActuatorScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        // The composite scenes (boat, hovercraft, glider in a gust, thruster rig), snapshotted and restored half-way.
        Scenarios.Register("world3d/actuators", ctx => Actuators3DScene.Run(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }));
        Scenarios.Register("world2d/actuators", ctx => Actuators2DScene.Run(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }));

        // The stability scenes (a shortened run with their scripts), snapshotted and restored half-way.
        foreach (var name in Actuators3DScene.Names)
        {
            string n = name;
            Scenarios.Register("world3d/" + n, ctx =>
            {
                var s = Actuators3DScene.Build(n, ctx.Threads);
                var w = s.World;
                for (int i = 0; i < 160; i++)
                {
                    if (i == 80) { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); ctx.Write(snap); s.Rebind(w); }
                    s.Script(i);
                    w.Step(Actuators3DScene.Dt);
                    ctx.Write((long)w.ComputeHash()); ctx.EndStep();
                }
            });
        }

        foreach (var name in Actuators2DScene.Names)
        {
            string n = name;
            Scenarios.Register("world2d/" + n, ctx =>
            {
                var s = Actuators2DScene.Build(n, ctx.Threads);
                var w = s.World;
                for (int i = 0; i < 160; i++)
                {
                    if (i == 80) { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); ctx.Write(snap); s.Rebind(w); }
                    s.Script(i);
                    w.Step(Actuators2DScene.Dt);
                    ctx.Write((long)w.ComputeHash()); ctx.EndStep();
                }
            });
        }
    }
}
