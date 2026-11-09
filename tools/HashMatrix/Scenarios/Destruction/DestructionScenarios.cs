using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Tests.Common;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class DestructionScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("world3d/destruction", ctx =>
        {
            var (w, target) = DestructionScenes.Spatial(ctx.Threads);
            for (int i = 0; i < 240; i++)
            {
                if (i == 30 && target.IsValid) w.Slice(target, new Vector3D(-8, 14 - 1.226, 2), new Vector3F(1, 0.3f, 0.2f));
                if (i == 60) { var s = w.Snapshot(); w = w.Fork(); w.Restore(s); ctx.Write(w.Snapshot()); }
                if (i == 120) w.ShiftOrigin(new Vector3D(64, 0, -32));
                w.Step(1f / 60); ctx.Write((long)w.ComputeHash()); ctx.EndStep();
            }
        });
        Scenarios.Register("world2d/destruction", ctx =>
        {
            var (w, target) = DestructionScenes.Planar(ctx.Threads);
            for (int i = 0; i < 240; i++)
            {
                if (i == 30 && target.IsValid) w.Slice(target, new Vector2D(-8, 14 - 1.226), new Vector2F(1, 0.3f));
                if (i == 60) { var s = w.Snapshot(); w = w.Fork(); w.Restore(s); ctx.Write(w.Snapshot()); }
                if (i == 120) w.ShiftOrigin(new Vector2D(64, 0));
                w.Step(1f / 60); ctx.Write((long)w.ComputeHash()); ctx.EndStep();
            }
        });
    }
}
