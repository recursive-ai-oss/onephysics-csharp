using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
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
                var w = new PhysicsWorld3D(new WorldOptions3D { WorldId = 1, ThreadCount = threads });
                w.AddBox(new Vector3F(20, 1, 20), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
                for (int i = 0; i < 60; i++) { w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(-5 + 1.05 * (i % 10), 0.8 + 1.3 * (i / 10), 0)); }
                var ball = w.AddSphere(0.5f, at: new Vector3D(2, 3, 0));
                var zone = w.CreateBody(BodyType.Static, new Vector3D(0, 1, 0));
                w.CreateBox(zone, new Vector3F(6, 4, 6), 0f, new ShapeDef { IsSensor = true, Density = 0f });
                int sink = 0;
                zone.OnTriggerEnter += _ => sink++;
                ball.OnCollisionEnter += _ => sink++;
                var player = w.AddCharacter(new Vector3D(6, 1, 0));
                var buffer = new QueryHit3D[8];
                // Long enough for the pile to land, settle and fall asleep, so the impact, resting and sleeping paths have run; each query kind a host calls runs every step.
                for (int i = 0; i < 150; i++)
                {
                    player.Move(new Vector3F(-0.05f, -0.1f, 0f), 1f / 60);
                    w.Step(1f / 60);
                    w.Raycast(new Vector3D(0, 10, 0), new Vector3F(0, -1, 0), out _);
                    w.RaycastAll(new Vector3D(0, 10, 0), new Vector3F(0, -1, 0), buffer);
                    w.OverlapSphereAll(new Vector3D(0, 1, 0), 2f, buffer);
                    w.SphereCast(new Vector3D(0, 10, 0), 0.3f, new Vector3F(0.1f, -1, 0), out _);
                    w.BoxCast(new Vector3D(1, 10, 0), new Vector3F(0.5f, 0.5f, 0.5f), QuaternionF.Identity, new Vector3F(0, -1, 0), out _);
                    w.OverlapBoxAll(new Vector3D(0, 1, 0), new Vector3F(2, 2, 2), QuaternionF.Identity, buffer);
                    w.OverlapCapsuleAll(new Vector3D(0, 1, 0), 1f, 0.5f, QuaternionF.Identity, buffer);
                    w.ComputeHash();
                }

                // The batch queries run on the pool too; each has its own body type.
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
                query.RaycastBatch(rays, rayHits);
                query.SphereCastBatch(spheres, sphereHits);
                query.OverlapSphereCountBatch(spheres, counts);
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
