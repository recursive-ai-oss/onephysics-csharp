using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Fluids;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

// The particle-fluid scenes at coarse spacing: the world hash after every step, a fork and restore part-way (with the snapshot bytes), an origin shift, and the events and a ray and an overlap query of each step.
internal static class FluidScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Register2D("world2d/fluids-dam", "dam-break-2d", 0.06f);
        Register2D("world2d/fluids-rope", "fluid-rope-2d", 0.08f);
        Register3D("world3d/fluids-dam", "dam-break-3d", 0.1f);
        Register3D("world3d/fluids-boxes", "fluid-boxes-3d", 0.1f);
        Register3D("world3d/fluids-cloth", "fluid-cloth-3d", 0.1f);
        Register3D("world3d/fluids-sand", "sand-pile-3d", FluidScenes.CoarseSpacing("sand-pile-3d"));
        Register2D("world2d/fluids-hopper", "hopper-2d", FluidScenes.CoarseSpacing("hopper-2d"));
        Register3D("world3d/fluids-honey", "honey-coil-3d", FluidScenes.CoarseSpacing("honey-coil-3d"));
        Register3D("world3d/fluids-snow", "snow-impact-3d", FluidScenes.CoarseSpacing("snow-impact-3d"));
        Register2D("world2d/fluids-clay", "clay-press-2d", FluidScenes.CoarseSpacing("clay-press-2d"));
    }

    private static void Register2D(string row, string scene, float spacing)
    {
        Scenarios.Register(row, ctx =>
        {
            var w = FluidScenes.Build2D(scene, ctx.Threads, spacing);
            var results = new FluidParticleInfo[16];
            for (int i = 0; i < 120; i++)
            {
                if (i == 40) { var s = w.Snapshot(); w = w.Fork(); w.Restore(s); ctx.Write(s); }
                if (i == 80) { w.ShiftOrigin(new Vector2D(16, -8)); }
                FluidScenes.BeforeStep2D(scene, w, i, i >= 80 ? new Vector2D(16, -8) : default);
                w.Step(FluidScenes.Dt);
                ctx.Write((long)w.ComputeHash());
                foreach (var e in w.FluidEvents) { ctx.Write((int)e.Kind); ctx.Write(e.Body.Index); ctx.Write(e.Fluid); ctx.Write(e.SubmergedFraction); }
                int n = w.OverlapFluidSphere(new Vector2D(0.5 + (i >= 80 ? -16 : 0), 0.2 + (i >= 80 ? 8 : 0)), 0.3f, results, out int total);
                ctx.Write(total); for (int k = 0; k < n; k++) { ctx.Write(results[k].Id); }
                ctx.Write(w.RaycastFluid(new Vector2D(0.5 + (i >= 80 ? -16 : 0), 3 + (i >= 80 ? 8 : 0)), new Vector2F(0, -1), out var hit) ? hit.Distance : -1f);
                ctx.EndStep();
            }
        });
    }

    private static void Register3D(string row, string scene, float spacing)
    {
        Scenarios.Register(row, ctx =>
        {
            var w = FluidScenes.Build3D(scene, ctx.Threads, spacing);
            var results = new FluidParticleInfo[16];
            for (int i = 0; i < 120; i++)
            {
                if (i == 40) { var s = w.Snapshot(); w = w.Fork(); w.Restore(s); ctx.Write(s); }
                if (i == 80) { w.ShiftOrigin(new Vector3D(16, -8, 2)); }
                w.Step(FluidScenes.Dt);
                ctx.Write((long)w.ComputeHash());
                foreach (var e in w.FluidEvents) { ctx.Write((int)e.Kind); ctx.Write(e.Body.Index); ctx.Write(e.Fluid); ctx.Write(e.SubmergedFraction); }
                double sx = i >= 80 ? 16 : 0, sy = i >= 80 ? -8 : 0, sz = i >= 80 ? 2 : 0;
                int n = w.OverlapFluidSphere(new Vector3D(0.2 - sx, 0.2 - sy, 0.2 - sz), 0.3f, results, out int total);
                ctx.Write(total); for (int k = 0; k < n; k++) { ctx.Write(results[k].Id); }
                ctx.Write(w.RaycastFluid(new Vector3D(0.2 - sx, 3 - sy, 0.2 - sz), new Vector3F(0, -1, 0), out var hit) ? hit.Distance : -1f);
                ctx.EndStep();
            }
        });
    }
}
