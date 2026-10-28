using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    private static bool s_primed;
    private static int s_primedThreads;
    private static readonly object s_primeLock = new();

    // The first use of a code path in a process allocates a little in the runtime itself (type loading and generic instantiation), whatever the world holds. Warmup runs the paths the
    // steady state uses once, on a throwaway world, so that a host's first step, query and character move allocate nothing.
    private static void PrimeProcess(int threads)
    {
        lock (s_primeLock)
        {
            if (s_primed && s_primedThreads >= threads) { return; }
            var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 1, ThreadCount = threads });
            w.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -1), type: BodyType.Static);
            for (int i = 0; i < 60; i++) { w.AddBox(new Vector2F(1, 1), at: new Vector2D(-5 + 1.05 * (i % 10), 0.5 + 1.02 * (i / 10))); }
            var ball = w.AddCircle(0.5f, at: new Vector2D(2, 3));
            var zone = w.CreateBody(BodyType.Static, new Vector2D(0, 1));
            w.CreateBox(zone, 3, 2, def: new ShapeDef { IsSensor = true });
            int sink = 0;
            zone.OnTriggerEnter += _ => sink++;
            ball.OnCollisionEnter += _ => sink++;
            var player = w.AddCharacter(at: new Vector2D(6, 1));
            var buffer = new QueryHit2D[8];
            for (int i = 0; i < 40; i++)
            {
                player.Move(new Vector2F(-0.05f, -0.1f), 1f / 60);
                w.Step(1f / 60);
                w.Raycast(new Vector2D(0, 10), new Vector2F(0, -1), out _);
                w.RaycastAll(new Vector2D(0, 10), new Vector2F(0, -1), buffer);
                w.OverlapCircleAll(new Vector2D(0, 1), 2f, buffer);
                w.ComputeHash();
            }

            w.WarmupCore(false);
            GC.KeepAlive(buffer);
            s_primed = sink >= 0;
            s_primedThreads = threads;
        }
    }
}
