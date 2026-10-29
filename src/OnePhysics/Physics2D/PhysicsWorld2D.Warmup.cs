using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    // Bit 0: primed with one thread (serial paths), bit 1: primed with several (worker paths). The two run different code.
    private static int s_primedMask;
    private static readonly object s_primeLock = new();

    // The first use of a code path in a process allocates a little in the runtime itself (type loading and generic instantiation), whatever the world holds. Warmup runs the paths the
    // steady state uses once, on a throwaway world, so that a host's first step, query and character move allocate nothing.
    private static void PrimeProcess(int threads)
    {
        lock (s_primeLock)
        {
            int bit = threads > 1 ? 2 : 1;
            if ((s_primedMask & bit) != 0) { return; }
            // Every call goes through the worker pool while priming, so the pool state the host's steps need exists before the first of them (see TaskExecutor).
            TaskExecutor.SetPrimeParallel(threads > 1);
            try
            {
                var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 1, ThreadCount = threads });
                w.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -1), type: BodyType.Static);
                for (int i = 0; i < 60; i++) { w.AddBox(new Vector2F(1, 1), at: new Vector2D(-5 + 1.05 * (i % 10), 0.8 + 1.3 * (i / 10))); }
                var ball = w.AddCircle(0.5f, at: new Vector2D(2, 3));
                var zone = w.CreateBody(BodyType.Static, new Vector2D(0, 1));
                w.CreateBox(zone, new Vector2F(6, 4), def: new ShapeDef { IsSensor = true });
                int sink = 0;
                zone.OnTriggerEnter += _ => sink++;
                ball.OnCollisionEnter += _ => sink++;
                var player = w.AddCharacter(at: new Vector2D(6, 1));
                var buffer = new QueryHit2D[8];
                // Long enough for the pile to land, settle and fall asleep, so the impact, resting and sleeping paths have run.
                for (int i = 0; i < 150; i++)
                {
                    player.Move(new Vector2F(-0.05f, -0.1f), 1f / 60);
                    w.Step(1f / 60);
                    w.Raycast(new Vector2D(0, 10), new Vector2F(0, -1), out _);
                    w.RaycastAll(new Vector2D(0, 10), new Vector2F(0, -1), buffer);
                    w.OverlapCircleAll(new Vector2D(0, 1), 2f, buffer);
                    w.ComputeHash();
                }

                // The batch queries run on the pool too; each has its own body type.
                var query = new PhysicsQuery2D(w);
                var rays = new RayInput2D[130];
                var circles = new CircleInput2D[130];
                for (int k = 0; k < rays.Length; k++)
                {
                    rays[k] = new RayInput2D(new Vector2D((k % 13) - 6, 10), new Vector2F(0, -1), 30f);
                    circles[k] = new CircleInput2D(new Vector2D((k % 13) - 6, 6), 0.4f, new Vector2F(0, -1), 20f);
                }

                var rayHits = new QueryHit2D[rays.Length];
                var circleHits = new QueryHit2D[circles.Length];
                var counts = new int[circles.Length];
                query.RaycastBatch(rays, rayHits);
                query.CircleCastBatch(circles, circleHits);
                query.OverlapCircleCountBatch(circles, counts);
                w.WarmupCore(false);
                GC.KeepAlive(buffer);
                if (sink >= 0) { s_primedMask |= bit; }
            }
            finally
            {
                TaskExecutor.SetPrimeParallel(false);
            }
        }
    }
}
