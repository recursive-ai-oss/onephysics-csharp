using System;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Core.Reference;

// Shared by the NativeAOT and browser-WASM smoke apps: steps a small deterministic scene on one and on several threads,
// snapshots and restores it, and checks that every path gives the same world hash.
internal static class SmokeProgram
{
    private static int Main()
    {
        const float dt = 1f / 60f;
        var a = ParticleScene.CreatePile(200, 42, threadCount: 1, worldId: 1);
        var b = ParticleScene.CreatePile(200, 42, threadCount: 4, worldId: 1);
        for (int i = 0; i < 60; i++) { a.Step(dt); b.Step(dt); }
        byte[] snap = a.World.Snapshot();
        var c = new ParticleScene(new WorldOptions { WorldId = 1 });
        c.World.Restore(snap);
        for (int i = 0; i < 60; i++) { a.Step(dt); b.Step(dt); c.Step(dt); }
        ulong h = a.World.ComputeHash();
        bool ok = h == b.World.ComputeHash() && h == c.World.ComputeHash();
        // The 2D collision scenario must give the same hash on every platform (see docs/collision2d.md).
        ulong ch = RecursiveAI.OnePhysics.Collision2D.Reference.CollisionHash2D.Compute();
        ok &= ch == 0x31E47F1BCE1B321FUL;
        Console.WriteLine("collision2d " + ch.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        Console.WriteLine(ok ? "OK " + h.ToString("x16", System.Globalization.CultureInfo.InvariantCulture) : "FAIL");
        return ok ? 0 : 1;
    }
}
