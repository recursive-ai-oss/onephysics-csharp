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
            var w = new PhysicsWorld3D(new WorldOptions3D { WorldId = 1, ThreadCount = threads });
            w.AddBox(new Vector3F(20, 1, 20), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
            for (int i = 0; i < 60; i++) { w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(-5 + 1.05 * (i % 10), 0.5 + 1.02 * (i / 10), 0)); }
            var ball = w.AddSphere(0.5f, at: new Vector3D(2, 3, 0));
            var zone = w.CreateBody(BodyType.Static, new Vector3D(0, 1, 0));
            w.CreateBox(zone, new Vector3F(3, 2, 3), 0f, new ShapeDef { IsSensor = true, Density = 0f });
            int sink = 0;
            zone.OnTriggerEnter += _ => sink++;
            ball.OnCollisionEnter += _ => sink++;
            var player = w.AddCharacter(new Vector3D(6, 1, 0));
            var buffer = new QueryHit3D[8];
            for (int i = 0; i < 40; i++)
            {
                player.Move(new Vector3F(-0.05f, -0.1f, 0f), 1f / 60);
                w.Step(1f / 60);
                w.Raycast(new Vector3D(0, 10, 0), new Vector3F(0, -1, 0), out _);
                w.RaycastAll(new Vector3D(0, 10, 0), new Vector3F(0, -1, 0), buffer);
                w.OverlapSphereAll(new Vector3D(0, 1, 0), 2f, buffer);
                w.ComputeHash();
            }

            w.WarmupCore(false);
            GC.KeepAlive(buffer);
            if (sink >= 0) { s_primedMask |= bit; }
        }
    }
}
