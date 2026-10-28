using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Core;

/// <summary>Runs alone: the counter is process-wide, so any other test allocating at the same time would be counted.</summary>
[CollectionDefinition("Allocation", DisableParallelization = true)]
public sealed class AllocationGroup { }

/// <summary>After <c>Warmup()</c>, a pile with a character, a sensor and a ray each step allocates nothing, from the first step, at one and several threads.</summary>
[Collection("Allocation")]
public class WarmAllocationTests
{
    private const float Dt = 1f / 60f;

    // The runtime's own tiered compilation can allocate a few kilobytes on any thread in the first moments of a process, whatever the library does. An allocation the library makes
    // repeats in every run, so the scenario passes when one of three fresh worlds is clean.
    private static void Settled(Func<string?> run)
    {
        string? last = null;
        for (int attempt = 0; attempt < 3; attempt++)
        {
            last = run();
            if (last == null) { return; }
        }

        Assert.Fail(last);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(8)]
    public void World2DAllocatesNothingAfterWarmup(int threads) => Settled(() => Run2D(threads));

    private static string? Run2D(int threads)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = threads, WorldId = 3 });
        w.AddBox(new Vector2F(40, 1), at: new Vector2D(0, -1), type: BodyType.Static);
        for (int i = 0; i < 40; i++)
        {
            int row = i / 8, col = i % 8;
            w.AddBox(new Vector2F(1, 1), at: new Vector2D((col * 1.05) - 4 + ((row % 2) * 0.5), 2.5 + (row * 1.02)));
        }

        var player = w.AddCharacter(at: new Vector2D(8, 1));
        var zone = w.CreateBody(BodyType.Static, new Vector2D(-2, 1));
        w.CreateBox(zone, 3, 2, def: new Physics2D.ShapeDef { IsSensor = true });
        w.Warmup();

        var hits = new QueryHit2D[16];
        var scratch = new byte[1 << 22];
        long total = 0;
        for (int i = 1; i <= 400; i++)
        {
            long a0 = GC.GetTotalAllocatedBytes(true), t0 = GC.GetAllocatedBytesForCurrentThread();
            player.Move(new Vector2F(-0.04f, -0.1f), Dt);
            w.Step(Dt);
            w.Raycast(new Vector2D(0, 10), new Vector2F(0, -1), out _);
            w.OverlapCircleAll(new Vector2D(0, 1), 3f, hits);
            w.ComputeHash();
            if (i % 50 == 0)
            {
                if (!w.TrySnapshot(scratch, out int n)) { return "the snapshot did not fit"; }
                    w.Restore(scratch.AsSpan(0, n));
                }

            total += GC.GetTotalAllocatedBytes(true) - a0;
            long own = GC.GetAllocatedBytesForCurrentThread() - t0;
            if (own != 0) { return $"step {i}: the calling thread allocated {own} bytes"; }
        }

        // The calling thread must allocate nothing. The process-wide count also sees the worker threads, and a stray runtime allocation of a few bytes now and then (a timer, the test host);
        // a worker that allocated per step would add up to far more than the allowance.
        return total <= 512 ? null : $"the process allocated {total} bytes over 400 steps";
    }

    [Theory]
    [InlineData(1)]
    [InlineData(8)]
    public void World3DAllocatesNothingAfterWarmup(int threads) => Settled(() => Run3D(threads));

    private static string? Run3D(int threads)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 3 });
        w.AddBox(new Vector3F(40, 1, 40), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 40; i++)
        {
            int row = i / 8, col = i % 8;
            w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D((col * 1.05) - 4 + ((row % 2) * 0.5), 2.5 + (row * 1.02), 0.1 * (i % 3)));
        }

        var player = w.AddCharacter(new Vector3D(8, 1, 0));
        var zone = w.CreateBody(BodyType.Static, new Vector3D(-2, 1, 0));
        w.CreateBox(zone, new Vector3F(3, 2, 3), 0f, new Physics3D.ShapeDef { IsSensor = true, Density = 0f });
        w.Warmup();

        var hits = new QueryHit3D[16];
        var scratch = new byte[1 << 22];
        long total = 0;
        // 400 steps cover the pile landing and settling, the phase in which deep contacts use the penetration solver.
        for (int i = 1; i <= 400; i++)
        {
            long a0 = GC.GetTotalAllocatedBytes(true), t0 = GC.GetAllocatedBytesForCurrentThread();
            player.Move(new Vector3F(-0.04f, -0.1f, 0), Dt);
            w.Step(Dt);
            w.Raycast(new Vector3D(0, 10, 0), new Vector3F(0, -1, 0), out _);
            w.OverlapSphereAll(new Vector3D(0, 1, 0), 3f, hits);
            w.ComputeHash();
            if (i % 50 == 0)
            {
                if (!w.TrySnapshot(scratch, out int n)) { return "the snapshot did not fit"; }
                    w.Restore(scratch.AsSpan(0, n));
                }

            total += GC.GetTotalAllocatedBytes(true) - a0;
            long own = GC.GetAllocatedBytesForCurrentThread() - t0;
            if (own != 0) { return $"step {i}: the calling thread allocated {own} bytes"; }
        }

        // The calling thread must allocate nothing. The process-wide count also sees the worker threads, and a stray runtime allocation of a few bytes now and then (a timer, the test host);
        // a worker that allocated per step would add up to far more than the allowance.
        return total <= 512 ? null : $"the process allocated {total} bytes over 400 steps";
    }
}
