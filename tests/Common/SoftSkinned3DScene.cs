using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;

// A 3D scene with a skinned soft body: a sleeve (an open tube) on a two-joint arm that swings, with a static ball in the way and a rigid box dropped on it. The free part is held within 6 cm of the skinned
// surface and kept out of a back-stop; its cuff is kinematic. 150 steps; at step 70 the world is snapshotted and restored into a fresh world. Angles come from a triangle wave (exact arithmetic) and rotations from
// onemath, so the scene is the same on every platform. Shared by the hash-matrix runner and both smoke apps.
internal static class SoftSkinned3DScene
{
    private const int Rings = 11, Around = 12;
    private static readonly Vector3F[] Bind = { new Vector3F(0f, 0f, 0f), new Vector3F(0f, 0.5f, 0f) };

    private static float Tri(int step, int period)
    {
        int m = step % period;
        float x = (float)m / period;
        return x < 0.5f ? 4f * x - 1f : 3f - 4f * x;
    }

    private static void Pose(int step, SkinJointPose[] joints)
    {
        var qLow = QuaternionF.CreateFromAxisAngle(new Vector3F(0f, 0f, 1f), 0.5f * Tri(step, 90));
        var qHigh = qLow * QuaternionF.CreateFromAxisAngle(new Vector3F(1f, 0f, 0f), 0.9f * Tri(step + 20, 70));
        var o = qLow.Rotate(Bind[1] - Bind[0]);
        var at = new Vector3D(0, 0.4, 0);
        joints[0] = new SkinJointPose(at, qLow);
        joints[1] = new SkinJointPose(new Vector3D(at.X + o.X, at.Y + o.Y, at.Z + o.Z), qHigh);
    }

    private static PhysicsWorld3D Build(int threads, out SoftBody sleeve)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 59 });
        w.AddBox(new Vector3F(20f, 1f, 20f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        w.AddSphere(0.2f, new Vector3D(0.3, 0.9, 0), type: BodyType.Static);
        int n = Rings * Around;
        var rest = new Vector3F[n]; var nrm = new Vector3F[n]; var joints = new int[4 * n]; var weights = new float[4 * n];
        var max = new float[n]; var bd = new float[n]; var br = new float[n];
        var tris = new List<int>();
        for (int r = 0; r < Rings; r++)
        {
            for (int a = 0; a < Around; a++)
            {
                int i = r * Around + a;
                var radial = QuaternionF.CreateFromAxisAngle(new Vector3F(0f, 1f, 0f), 6.2831855f * a / Around).Rotate(new Vector3F(1f, 0f, 0f));
                rest[i] = new Vector3F(0.1f * radial.X, r / (float)(Rings - 1), 0.1f * radial.Z);
                nrm[i] = radial;
                float t = Math.Clamp((r - (Rings - 1) / 2f + 1f) / 2f, 0f, 1f);
                joints[4 * i] = 0; joints[4 * i + 1] = 1; weights[4 * i] = 1f - t; weights[4 * i + 1] = t;
                max[i] = r == 0 ? 0f : 0.06f;
                if (r > 0) { bd[i] = 0.05f; br[i] = 0.05f; }
                if (r + 1 < Rings)
                {
                    int b = (a + 1) % Around;
                    tris.AddRange(new[] { r * Around + a, r * Around + b, (r + 1) * Around + a, r * Around + b, (r + 1) * Around + b, (r + 1) * Around + a });
                }
            }
        }

        var mesh = SoftBodyMesh.FromTriangles(rest, tris.ToArray());
        var skin = SoftSkin.Create(mesh, new[] { -1, 0 }, Bind, joints, weights, max, bd, br, nrm);
        sleeve = w.AddSoftBody(new SoftBodyDef { Mesh = mesh, Skin = skin, Position = new Vector3D(0, 0.4, 0), Mass = 0.5f, ParticleRadius = 0.01f, BendCompliance = 1e-3f, AllowSleep = false });
        w.AddBox(new Vector3F(0.15f, 0.15f, 0.15f), new Vector3D(-0.05, 1.3, 0.02), density: 30f);
        return w;
    }

    internal static ulong Run(int threads, bool restoreHalfway, Action<ulong>? perStep = null)
    {
        var w = Build(threads, out var sleeve);
        var joints = new SkinJointPose[2];
        ulong hash = 14695981039346656037UL;
        for (int step = 0; step < 150; step++)
        {
            if (step == 70 && restoreHalfway)
            {
                var snap = w.Snapshot();
                w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 59 });
                w.Restore(snap);
                sleeve = w.GetSoftBody(0);
            }

            Pose(step, joints);
            sleeve.SetSkeletonTransforms(joints);
            w.Step(1f / 60f);
            hash = (hash ^ w.ComputeHash()) * 1099511628211UL;
            perStep?.Invoke(hash);
        }

        return hash;
    }
}
