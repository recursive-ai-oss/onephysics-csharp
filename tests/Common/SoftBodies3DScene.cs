using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;

// A 3D scene with soft bodies: a cloth over a sphere, a jelly cube and a rubber ball on a floor, a rigid box falling on the cube, and a pinned cloth. 150 steps; at step 70 the world is snapshotted and
// restored into a fresh world. Shared by the hash-matrix runner and both smoke apps (the trim and ahead-of-time checks see the soft-body engine through it).
internal static class SoftBodies3DScene
{
    private static PhysicsWorld3D Build(int threads)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 53 });
        w.AddBox(new Vector3F(60f, 1f, 60f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        w.AddSphere(0.5f, new Vector3D(0, 0.5, 0), type: BodyType.Static);
        w.AddCloth(1.6f, 1.6f, 10, new Vector3D(0, 1.4, 0), pinCorners: false);
        w.AddSoftBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(2, 0.8, 0), SoftMaterial.Jelly, 3);
        w.AddSoftBall(0.3f, new Vector3D(-2, 0.8, 0), SoftMaterial.Rubber, 3);
        w.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), new Vector3D(2, 2.2, 0.05), density: 30f);
        w.AddCloth(1f, 1f, 6, new Vector3D(0, 2.5, 3), pinCorners: true);
        return w;
    }

    internal static ulong Run(int threads, bool restoreHalfway, Action<ulong>? perStep = null)
    {
        var w = Build(threads);
        ulong hash = 14695981039346656037UL;
        for (int step = 0; step < 150; step++)
        {
            if (step == 70 && restoreHalfway)
            {
                var snap = w.Snapshot();
                w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 53 });
                w.Restore(snap);
            }

            w.Step(1f / 60f);
            hash = (hash ^ w.ComputeHash()) * 1099511628211UL;
            perStep?.Invoke(hash);
        }

        return hash;
    }
}
