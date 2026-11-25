using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// Worlds authored in another length unit: a 2D pile in pixels (32 to the metre) and a 3D pile with joints, a character and a mesh-free ground in millimetres. Both snapshot the world part way, restore it into a fresh
/// world built from the same creation options and carry on, and the scenario fails if the two disagree at any step. Each step's world hash goes into the manifest, so the unit section (written only for a unit other than
/// one metre) is part of what every platform and column must agree on.
/// </summary>
internal static class LengthUnitScenarios
{
    private const float Dt = 1f / 60f;

    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("world2d/length-unit-pixels", Run2D);
        Scenarios.Register("world3d/length-unit-millimetres", Run3D);
    }

    private static ulong Next(ref ulong s)
    {
        ulong z = s += 0x9E3779B97F4A7C15UL;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        return z ^ (z >> 31);
    }

    private static double Range(ref ulong s, double lo, double hi) => lo + (hi - lo) * ((Next(ref s) >> 40) * (1.0 / 16777216.0));

    private static P2.PhysicsWorld2D NewWorld2D(int threads) => new(new P2.WorldOptions2D { ThreadCount = threads, WorldId = 71, LengthUnit = LengthUnit.Pixels(32) });

    private static void Run2D(ScenarioContext ctx)
    {
        const double K = 32.0;
        var w = NewWorld2D(ctx.Threads);
        w.AddBox(new((float)(60 * K), (float)(1 * K)), at: new(0, -0.5 * K), type: BodyType.Static);
        w.AddBox(new((float)(1 * K), (float)(20 * K)), at: new(-30 * K, 9.5 * K), type: BodyType.Static);
        w.AddBox(new((float)(1 * K), (float)(20 * K)), at: new(30 * K, 9.5 * K), type: BodyType.Static);
        ulong s = 71717;
        for (int i = 0; i < 80; i++)
        {
            double x = Range(ref s, -20, 20) * K, y = (1 + (i / 10) * 1.3) * K;
            switch (i % 3)
            {
                case 0: w.AddBox(new((float)(Range(ref s, 0.5, 1.2) * K), (float)(Range(ref s, 0.5, 1.2) * K)), at: new(x, y)); break;
                case 1: w.AddCircle((float)(Range(ref s, 0.3, 0.7) * K), at: new(x, y)); break;
                default: w.AddCapsule((float)(Range(ref s, 0.3, 0.6) * K), (float)(0.25 * K), at: new(x, y)); break;
            }
        }

        var ch = w.AddCharacter(new(-12 * K, 1.0 * K));
        byte[]? snapshot = null;
        P2.PhysicsWorld2D? copy = null;
        for (int i = 0; i < 180; i++)
        {
            ch.Move(new((float)(0.05 * K), (float)(-0.1 * K)), Dt);
            w.Step(Dt);
            ulong h = w.ComputeHash();
            ctx.Write((long)h);
            if (copy != null)
            {
                foreach (var c in copy.Characters) { c.Move(new((float)(0.05 * K), (float)(-0.1 * K)), Dt); }
                copy.Step(Dt);
                if (copy.ComputeHash() != h) { throw new InvalidOperationException("restored 2D world diverged at step " + i); }
            }

            ctx.EndStep();
            if (i == 60)
            {
                snapshot = w.Snapshot();
                copy = NewWorld2D(ctx.Threads);
                copy.Restore(snapshot);
                if (copy.ComputeHash() != h) { throw new InvalidOperationException("restored 2D world differs from the original"); }
            }
        }

        // The exact-state scene text carries the unit too.
        var loaded = P2.PhysicsWorld2D.LoadScene(new System.IO.StringReader(w.SaveScene()));
        if (loaded.ComputeHash() != w.ComputeHash() || !loaded.Options.LengthUnit.Equals(LengthUnit.Pixels(32))) { throw new InvalidOperationException("scene text lost the length unit"); }
        ctx.Write((long)loaded.ComputeHash());
        ctx.EndStep();
    }

    private static P3.PhysicsWorld3D NewWorld3D(int threads) => new(new P3.WorldOptions3D { ThreadCount = threads, WorldId = 72, LengthUnit = LengthUnit.Millimetres });

    private static void Run3D(ScenarioContext ctx)
    {
        const double K = 1000.0;
        var w = NewWorld3D(ctx.Threads);
        w.AddBox(new((float)(40 * K), (float)(1 * K), (float)(40 * K)), at: new(0, -0.5 * K, 0), type: BodyType.Static);
        ulong s = 72727;
        for (int i = 0; i < 60; i++)
        {
            double x = Range(ref s, -8, 8) * K, y = (0.7 + (i / 8) * 1.1) * K, z = Range(ref s, -8, 8) * K;
            switch (i % 3)
            {
                case 0: w.AddBox(new((float)(Range(ref s, 0.4, 1.0) * K), (float)(Range(ref s, 0.4, 1.0) * K), (float)(Range(ref s, 0.4, 1.0) * K)), at: new(x, y, z)); break;
                case 1: w.AddSphere((float)(Range(ref s, 0.25, 0.5) * K), at: new(x, y, z)); break;
                default: w.AddCapsule((float)(0.4 * K), (float)(0.2 * K), at: new(x, y, z)); break;
            }
        }

        var anchor = w.AddBox(new((float)(0.5 * K), (float)(0.5 * K), (float)(0.5 * K)), at: new(12 * K, 6 * K, 0), type: BodyType.Static);
        var bob = w.AddSphere((float)(0.4 * K), at: new(12 * K, 4.5 * K, 0));
        w.AddHinge(anchor, bob, new(12 * K, 6 * K, 0), new Vector3F(0, 0, 1));
        var ch = w.AddCharacter(new(-14 * K, 1.0 * K, 0));
        P3.PhysicsWorld3D? copy = null;
        for (int i = 0; i < 150; i++)
        {
            ch.Move(new((float)(0.05 * K), (float)(-0.1 * K), 0), Dt);
            w.Step(Dt);
            ulong h = w.ComputeHash();
            ctx.Write((long)h);
            if (copy != null)
            {
                foreach (var c in copy.Characters) { c.Move(new((float)(0.05 * K), (float)(-0.1 * K), 0), Dt); }
                copy.Step(Dt);
                if (copy.ComputeHash() != h) { throw new InvalidOperationException("restored 3D world diverged at step " + i); }
            }

            ctx.EndStep();
            if (i == 50)
            {
                copy = NewWorld3D(ctx.Threads);
                copy.Restore(w.Snapshot());
                if (copy.ComputeHash() != h) { throw new InvalidOperationException("restored 3D world differs from the original"); }
            }
        }

        var loaded = P3.PhysicsWorld3D.LoadScene(new System.IO.StringReader(w.SaveScene()));
        if (loaded.ComputeHash() != w.ComputeHash() || !loaded.Options.LengthUnit.Equals(LengthUnit.Millimetres)) { throw new InvalidOperationException("scene text lost the length unit"); }
        ctx.Write((long)loaded.ComputeHash());
        ctx.EndStep();
    }
}
