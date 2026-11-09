using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class TerrainEditScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        // Bodies on a signed distance field terrain, a voxel terrain dug under bodies and characters, and a bitmask terrain carved by projectiles: a shortened run each, snapshotted and restored half-way.
        Scenarios.Register("world3d/sdf-pile-3d", ctx => TerrainEditScenes.Run("sdf-pile-3d", ctx.Threads, true, 160, h => { ctx.Write((long)h); ctx.EndStep(); }));
        Scenarios.Register("world3d/voxel-dig-3d", ctx => TerrainEditScenes.Run("voxel-dig-3d", ctx.Threads, true, 160, h => { ctx.Write((long)h); ctx.EndStep(); }));
        Scenarios.Register("world2d/carve-2d", ctx => TerrainEditScenes.Run("carve-2d", ctx.Threads, true, 160, h => { ctx.Write((long)h); ctx.EndStep(); }));
    }
}
