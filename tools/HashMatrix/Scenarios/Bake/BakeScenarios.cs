using System.Runtime.CompilerServices;
using RecursiveAI.OnePhysics.Bake;
using RecursiveAI.OnePhysics.Tests.Common;

namespace RecursiveAI.OnePhysics.HashMatrix;

// The bytes of a bake cache: rigid bodies, a soft body, cloth, a rope, a particle fluid and a fracturing crate in one world, per dimension,
// at stride 1 (every track) and at stride 3 with coarser quanta. The cache is a function of the world's history, so these rows must be equal on every column.
internal static class BakeScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("bake/world3d", ctx => Run(ctx, spatial: true));
        Scenarios.Register("bake/world2d", ctx => Run(ctx, spatial: false));
    }

    private static void Run(ScenarioContext ctx, bool spatial)
    {
        var a = new BakeOptions { SceneName = "matrix", LibraryVersion = "matrix", ReplayDigest = new byte[32] };
        var b = new BakeOptions { SceneName = "matrix", LibraryVersion = "matrix", FrameStride = 3, PositionExponent = -12, VertexExponent = -10, RotationBits = 10, BlockFrames = 8, Tracks = BakeTracks.Rigid | BakeTracks.Soft | BakeTracks.Events };
        var c = new BakeOptions { SceneName = "matrix", LibraryVersion = "matrix", PositionExponent = -40, RotationBits = 0, Tracks = BakeTracks.Rigid | BakeTracks.Fracture };
        var options = new[] { a, b, c };
        var files = new byte[3][];
        for (int k = 0; k < 3; k++)
        {
            if (spatial)
            {
                var w = BakeScenes.Spatial(ctx.Threads);
                var r = w.BeginBake(options[k]);
                Steps(ctx, w.Step, w.ComputeHash, 150, k == 0);
                files[k] = r.Finish();
            }
            else
            {
                var w = BakeScenes.Planar(ctx.Threads);
                var r = w.BeginBake(options[k]);
                Steps(ctx, w.Step, w.ComputeHash, 150, k == 0);
                files[k] = r.Finish();
            }
        }

        foreach (var f in files) { ctx.Write(f.Length); ctx.Write(f); ctx.EndStep(); }
    }

    private static void Steps(ScenarioContext ctx, System.Action<float> step, System.Func<ulong> hash, int n, bool hashes = true)
    {
        for (int i = 0; i < n; i++) { step(BakeScenes.Dt); if (hashes) { ctx.Write((long)hash()); ctx.EndStep(); } }
    }
}
