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

    // What is counted: the calling thread's own allocations per step (exact, per thread), and the pool threads' own allocations, read from the threads themselves at fixed points
    // (never a process-wide counter, which also sees the test host's timers and other threads that have nothing to do with the library). A window in which the runtime compiled a
    // method on that thread is not counted (see AllocMeter): the first run of a path nothing exercised before compiles it, and compiling allocates whatever the library does.
    private const int WorkerSlots = 64;

    // The first world a process runs may meet code and types for the first time at a step the priming did not reach (a hash on a worker count nothing used yet, a batch query in a world of one
    // thread); the runtime then loads or binds them on the calling thread and allocates, which is not the library holding memory. A first world, run to the end and not judged, takes those;
    // the second is judged and must allocate nothing.
    private static void Verdict(Func<string?> scenario)
    {
        _ = scenario();
        string? problem = scenario();
        Assert.True(problem == null, problem);
    }

    private static string? CheckWorkers(long[] bytes0, long[] compiled0, long[] bytes1, long[] compiled1, int step)
    {
        for (int k = 0; k < WorkerSlots; k++)
        {
            if (bytes1[k] != bytes0[k] && compiled1[k] == compiled0[k]) { return $"step {step}: pool thread {k} allocated {bytes1[k] - bytes0[k]} bytes"; }
        }

        return null;
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(8)]
    [InlineData(32)]
    public void World2DAllocatesNothingAfterWarmup(int threads) => Verdict(() => Run2D(threads));

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
        var wb0 = new long[WorkerSlots]; var wc0 = new long[WorkerSlots]; var wb1 = new long[WorkerSlots]; var wc1 = new long[WorkerSlots];
        WorkerPool.SampleWorkers(wb0, wc0);
        for (int i = 1; i <= Steps; i++)
        {
            var window = AllocMeter.Start();
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

            long own = window.Bytes();
            if (own != 0) { return $"step {i}: the calling thread allocated {own} bytes"; }

            if (i % 100 == 0)
            {
                WorkerPool.SampleWorkers(wb1, wc1);
                var bad = CheckWorkers(wb0, wc0, wb1, wc1, i);
                if (bad != null) { return bad; }
                (wb0, wb1) = (wb1, wb0);
                (wc0, wc1) = (wc1, wc0);
            }
        }

        return null;
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(8)]
    [InlineData(32)]
    public void World3DAllocatesNothingAfterWarmup(int threads) => Verdict(() => Run3D(threads));

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
        var wb0 = new long[WorkerSlots]; var wc0 = new long[WorkerSlots]; var wb1 = new long[WorkerSlots]; var wc1 = new long[WorkerSlots];
        WorkerPool.SampleWorkers(wb0, wc0);
        // The first steps cover the pile landing and settling, the phase in which deep contacts use the penetration solver; the rest are a settled pile asleep and awake.
        for (int i = 1; i <= Steps; i++)
        {
            var window = AllocMeter.Start();
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

            long own = window.Bytes();
            if (own != 0) { return $"step {i}: the calling thread allocated {own} bytes"; }

            if (i % 100 == 0)
            {
                WorkerPool.SampleWorkers(wb1, wc1);
                var bad = CheckWorkers(wb0, wc0, wb1, wc1, i);
                if (bad != null) { return bad; }
                (wb0, wb1) = (wb1, wb0);
                (wc0, wc1) = (wc1, wc0);
            }
        }

        return null;
    }
}
