using System.Runtime.CompilerServices;
using System.Text;
using RecursiveAI.OnePhysics.Interchange;
using RecursiveAI.OnePhysics.Interchange.Tests;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>What the importers produce, simulated: the world hash after every step of a world built from each sample file. The same arm through URDF and through MJCF gives one column.</summary>
internal static class InterchangeScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("interchange/urdf-arm", ctx => InterchangeSamples.RunArm(UrdfImporter.Import(InterchangeSamples.Read("arm6.urdf")).Definition, 300, (_, h) => { ctx.Write((long)h); ctx.EndStep(); }));
        Scenarios.Register("interchange/mjcf-arm", ctx => InterchangeSamples.RunArm(MjcfImporter.Import(InterchangeSamples.Read("arm6.xml")).Definition, 300, (_, h) => { ctx.Write((long)h); ctx.EndStep(); }));
        Scenarios.Register("interchange/gltf-stack", ctx => InterchangeSamples.RunScene3D(GltfPhysicsImporter.Import(Encoding.UTF8.GetBytes(InterchangeSamples.Twenty())).Scene, 300, (_, h) => { ctx.Write((long)h); ctx.EndStep(); }));
        Scenarios.Register("interchange/tiled-level", ctx => InterchangeSamples.RunMap(TiledImporter.Import(InterchangeSamples.Read("level.tmj")), 300, (_, h) => { ctx.Write((long)h); ctx.EndStep(); }));
        Scenarios.Register("interchange/ldtk-level", ctx => InterchangeSamples.RunMap(LdtkImporter.Import(InterchangeSamples.Read("level.ldtk")), 300, (_, h) => { ctx.Write((long)h); ctx.EndStep(); }));
    }
}
