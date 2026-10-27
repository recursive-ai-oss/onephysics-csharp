using System.Runtime.CompilerServices;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Core.Reference;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// The determinism checks that already exist as tests, registered so the matrix runs them on every platform: the core's reference particle scene (world hash after every step),
/// the snapshot smoke scene, and the collision and broadphase hashes (one value each).
/// </summary>
internal static class CoreScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        // 300 particles for 120 steps; the final hash is the regression anchor 11996105962752397070.
        Scenarios.Register("core/golden", ctx =>
        {
            var scene = ParticleScene.CreatePile(300, 1234, threadCount: ctx.Threads, worldId: 1);
            for (int i = 0; i < 120; i++)
            {
                scene.Step(1f / 60f);
                ctx.Write((long)scene.World.ComputeHash());
                ctx.EndStep();
            }
        });

        // The smoke scene of the AOT and WASM apps: 200 particles, snapshot after 60 steps, restore into a fresh world, 60 more steps on both; final hash 9dbdc6bae21b4f5d.
        Scenarios.Register("core/smoke", ctx =>
        {
            const float dt = 1f / 60f;
            var a = ParticleScene.CreatePile(200, 42, threadCount: ctx.Threads, worldId: 1);
            for (int i = 0; i < 60; i++) { a.Step(dt); ctx.Write((long)a.World.ComputeHash()); ctx.EndStep(); }
            byte[] snap = a.World.Snapshot();
            var c = new ParticleScene(new WorldOptions { WorldId = 1 });
            c.World.Restore(snap);
            for (int i = 0; i < 60; i++)
            {
                a.Step(dt); c.Step(dt);
                ctx.Write((long)a.World.ComputeHash());
                ctx.Write((long)c.World.ComputeHash());
                ctx.EndStep();
            }
        });

        Scenarios.Register("collision/hash2d", ctx => { ctx.Write((long)RecursiveAI.OnePhysics.Collision2D.Reference.CollisionHash2D.Compute()); ctx.EndStep(); });
        Scenarios.Register("collision/hash3d", ctx => { ctx.Write((long)Collision3DHash.Compute()); ctx.EndStep(); });
        Scenarios.Register("broadphase/hash", ctx => { ctx.Write((long)BroadphaseHash.Compute()); ctx.EndStep(); });
    }
}
