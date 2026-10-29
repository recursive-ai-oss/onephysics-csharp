using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
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
    private const int Steps = 5000;

    // The test project turns tiered compilation off (see the csproj). With it on, the runtime compiles an on-stack-replacement version of a method with a hot loop on the calling thread the first
    // time the loop runs long, and a background thread installs optimized code; both allocate inside the runtime, from a few hundred bytes to a few kilobytes, whatever the library does.
    // Measured on randomized scenes: with quick JIT for loops on, about one world in ten saw such an event on the calling thread and none with it off; at one thread the background
    // installs alone gave a few hundred bytes process-wide in about a third of the runs. With tiering off, anything counted here is the library's.
    // What the library allocates repeats exactly in every fresh world built the same way (same bodies, same steps), so the scenario passes when one of three fresh worlds is clean; an allocation that
    // shows up once and not in the others (a runtime event, measured on a host running other work, with the process pinned to three cores) is not the library's.
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
        w.CreateBox(zone, new Vector2F(6, 4), def: new Physics2D.ShapeDef { IsSensor = true });
        w.Warmup();

        var hits = new QueryHit2D[16];
        var scratch = new byte[1 << 22];
        long total = 0, firstBytes = 0; int first = -1;
        for (int i = 1; i <= Steps; i++)
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

            long step = GC.GetTotalAllocatedBytes(true) - a0;
            if (step != 0 && first < 0) { first = i; firstBytes = step; }
            total += step;
            long own = GC.GetAllocatedBytesForCurrentThread() - t0;
            if (own != 0) { return $"step {i}: the calling thread allocated {own} bytes"; }
        }

        // The calling thread must allocate nothing. The process-wide count also sees the worker threads, and a stray runtime allocation of a few bytes now and then (a timer, the test host);
        // a worker that allocated per step would add up to far more than the allowance.
        return total <= 512 ? null : $"the process allocated {total} bytes over {Steps} steps (first: {firstBytes} at step {first})";
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
        w.CreateBox(zone, new Vector3F(6, 4, 6), 0f, new Physics3D.ShapeDef { IsSensor = true, Density = 0f });
        w.Warmup();

        var hits = new QueryHit3D[16];
        var scratch = new byte[1 << 22];
        var query = new PhysicsQuery3D(w);
        var rays = new RayInput3D[130];
        var spheres = new SphereInput3D[130];
        for (int k = 0; k < rays.Length; k++)
        {
            rays[k] = new RayInput3D(new Vector3D((k % 13) - 6, 10, (k / 13) - 5), new Vector3F(0, -1, 0), 30f);
            spheres[k] = new SphereInput3D(new Vector3D((k % 13) - 6, 6, (k / 13) - 5), 0.4f, new Vector3F(0, -1, 0), 20f);
        }

        var rayHits = new QueryHit3D[rays.Length];
        var sphereHits = new QueryHit3D[spheres.Length];
        var counts = new int[spheres.Length];
        long total = 0, firstBytes = 0; int first = -1;
        // The first steps cover the pile landing and settling, the phase in which deep contacts use the penetration solver; the rest are a settled pile asleep and awake.
        for (int i = 1; i <= Steps; i++)
        {
            long a0 = GC.GetTotalAllocatedBytes(true), t0 = GC.GetAllocatedBytesForCurrentThread();
            player.Move(new Vector3F(-0.04f, -0.1f, 0), Dt);
            w.Step(Dt);
            w.Raycast(new Vector3D(0, 10, 0), new Vector3F(0, -1, 0), out _);
            w.OverlapSphereAll(new Vector3D(0, 1, 0), 3f, hits);
            w.SphereCast(new Vector3D(0, 10, 0), 0.3f, new Vector3F(0.1f, -1, 0), out _);
            w.BoxCast(new Vector3D(1, 10, 0), new Vector3F(0.5f, 0.5f, 0.5f), QuaternionF.Identity, new Vector3F(0, -1, 0), out _);
            w.OverlapBoxAll(new Vector3D(0, 1, 0), new Vector3F(2, 2, 2), QuaternionF.Identity, hits);
            w.OverlapCapsuleAll(new Vector3D(0, 1, 0), 1f, 0.5f, QuaternionF.Identity, hits);
            query.RaycastBatch(rays, rayHits);
            query.SphereCastBatch(spheres, sphereHits);
            query.OverlapSphereCountBatch(spheres, counts);
            w.ComputeHash();
            if (i % 50 == 0)
            {
                if (!w.TrySnapshot(scratch, out int n)) { return "the snapshot did not fit"; }
                    w.Restore(scratch.AsSpan(0, n));
                }

            long step = GC.GetTotalAllocatedBytes(true) - a0;
            if (step != 0 && first < 0) { first = i; firstBytes = step; }
            total += step;
            long own = GC.GetAllocatedBytesForCurrentThread() - t0;
            if (own != 0) { return $"step {i}: the calling thread allocated {own} bytes"; }
        }

        // The calling thread must allocate nothing. The process-wide count also sees the worker threads, and a stray runtime allocation of a few bytes now and then (a timer, the test host);
        // a worker that allocated per step would add up to far more than the allowance.
        return total <= 512 ? null : $"the process allocated {total} bytes over {Steps} steps (first: {firstBytes} at step {first})";
    }
}
