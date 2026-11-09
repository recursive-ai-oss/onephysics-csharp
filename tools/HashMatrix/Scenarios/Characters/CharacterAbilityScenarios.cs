using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class CharacterAbilityScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        // The ability scenes (soft bodies, ropes, ledges and ladders, vehicles and decks), a shortened run each, snapshotted and restored half-way.
        foreach (var name in CharacterAbilityScenes3D.Names)
        {
            string n = name;
            Scenarios.Register("world3d/" + n, ctx => CharacterAbilityScenes3D.Run(n, ctx.Threads, true, 160, h => { ctx.Write((long)h); ctx.EndStep(); }));
        }

        foreach (var name in CharacterAbilityScenes2D.Names)
        {
            string n = name;
            Scenarios.Register("world2d/" + n, ctx => CharacterAbilityScenes2D.Run(n, ctx.Threads, true, 160, h => { ctx.Write((long)h); ctx.EndStep(); }));
        }
    }
}
