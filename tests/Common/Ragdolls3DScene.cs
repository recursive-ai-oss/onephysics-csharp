using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;

// A 3D scene with ragdolls: a humanoid driven at full strength through a looping arm and leg pose, a smaller one at half strength, a limp one, and one carried by keyframes, with boxes thrown at them.
// Shared by the hash-matrix runner and both smoke apps. The pose errors enter the per-step hash beside the world hash, so any platform-dependent value shows up.
internal static class Ragdolls3DScene
{
    private static QuaternionF About(Vector3F axis, float angle)
    {
        double h = 0.5 * angle;
        float s = (float)Math64.Sin(h), c = (float)Math64.Cos(h);
        return new QuaternionF(axis.X * s, axis.Y * s, axis.Z * s, c);
    }

    /// <summary>The looping pose at a step: a triangle wave (exact arithmetic) swings the left arm sideways and bends an elbow and a knee, the other limbs stay in the bind pose.</summary>
    private static void Pose(QuaternionF[] pose, int step, float amount)
    {
        Array.Fill(pose, QuaternionF.Identity);
        int phase = step % 120;
        float t = (phase < 60 ? phase : 120 - phase) / 60f;   // 0..1..0
        pose[3] = About(new Vector3F(0, 0, 1), 1.0f * t * amount);
        pose[4] = About(new Vector3F(1, 0, 0), -0.8f * t * amount);
        pose[8] = About(new Vector3F(1, 0, 0), 0.3f * t * amount);
    }

    /// <summary>Runs 240 steps; <paramref name="perStep"/> receives the running hash after each. At step 110 the world is snapshotted and restored into a fresh one when <paramref name="restoreHalfway"/> is set, which must not change a bit.</summary>
    internal static ulong Run(int threads, bool restoreHalfway, Action<ulong>? perStep = null)
    {
        var opts = new WorldOptions3D { ThreadCount = threads, WorldId = 52 };
        Ragdoll.ApplyRecommendedOptions(opts);
        var w = new PhysicsWorld3D(opts);
        w.AddBox(new Vector3F(200f, 1f, 200f), new Vector3D(0, -0.5, 0), type: BodyType.Static, material: new Material3D(0.8f, 0f, 0f));
        var rags = new Ragdoll[4];
        rags[0] = w.AddRagdoll(RagdollDef.Humanoid, new Vector3D(0, 0.001, 0));
        rags[1] = w.AddRagdoll(RagdollDef.CreateHumanoid(1.2f, 30f), new Vector3D(3, 0.001, 0), About(new Vector3F(0, 1, 0), 0.6f));
        rags[2] = w.AddRagdoll(RagdollDef.Humanoid, new Vector3D(6, 2.0, 0), About(new Vector3F(1, 0, 0), 0.4f));
        var free = RagdollDef.Humanoid;
        free.SelfCollision = RagdollSelfCollision.None;
        rags[3] = w.AddRagdoll(free, new Vector3D(-3, 1.2, 0));
        var pose = new QuaternionF[13];

        ulong hash = 0xCBF29CE484222325UL;
        void Mix(ulong x) { hash = (hash ^ x) * 0x100000001B3UL; hash ^= hash >> 29; }
        const float dt = 1f / 60f;
        for (int step = 0; step < 240; step++)
        {
            if (step == 110 && restoreHalfway)
            {
                var snap = w.Snapshot();
                opts = new WorldOptions3D { ThreadCount = threads, WorldId = 52 };
                Ragdoll.ApplyRecommendedOptions(opts);
                w = new PhysicsWorld3D(opts);
                w.Restore(snap);
                rags = w.GetRagdolls();
            }

            if (step % 20 == 0) { Pose(pose, step, 1f); rags[0].DriveToPose(pose, 1f); Pose(pose, step, 0.6f); rags[1].DriveToPose(pose, 0.5f); }
            if (step == 30 || step == 90)
            {
                var b = w.AddBox(new Vector3F(0.25f, 0.25f, 0.25f), at: new Vector3D(step == 30 ? 0 : 3, 1.2, -2.5), density: 150f);
                b.LinearVelocity = new Vector3F(0, 0.5f, 7f);
            }

            if (step == 60) { Pose(pose, 0, 1f); rags[3].DriveToPose(pose, 0.3f); }
            Pose(pose, step, 1f);
            rags[2].MoveToPose(pose, new Vector3D(6, 1.0, 0), default, dt);
            if (step == 150) { rags[3].SetPose(pose, new Vector3D(-3, 1.0, 0)); }

            w.Step(dt);
            Mix(w.ComputeHash());
            Pose(pose, step, 1f);
            for (int i = 0; i < rags.Length; i++) { Mix(BitConverter.SingleToUInt32Bits(rags[i].GetPoseError(pose))); }
            perStep?.Invoke(hash);
        }

        return hash;
    }
}
