#pragma warning disable OPHYS0002, OPHYS0003
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;

namespace RecursiveAI.OnePhysics.HashMatrix;

// Active ragdolls in both dimensions: a standing figure pushed twice and hit, a figure whose bones are blended with a moving animation, a snapshot taken and restored half-way.
internal static class ActiveRagdollScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("world2d/active-ragdolls", ctx =>
        {
            var o = new Physics2D.WorldOptions2D { ThreadCount = ctx.Threads, WorldId = 318 };
            Physics2D.Ragdoll.ApplyRecommendedOptions(o);
            var w = new Physics2D.PhysicsWorld2D(o);
            w.AddBox(new Vector2F(400f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
            var stander = w.AddRagdoll(Physics2D.RagdollDef.Humanoid, new Vector2D(0, 0));
            stander.EnableActive(); stander.Stand();
            var dancer = w.AddRagdoll(Physics2D.RagdollDef.Humanoid, new Vector2D(3, 0));
            dancer.EnableActive();
            for (int i = 1; i < dancer.PartCount; i++) { dancer.SetBlendWeight(i, i % 3 == 0 ? 0f : 0.4f); }
            for (int step = 0; step < 240; step++)
            {
                if (step == 100) { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); ctx.Write(snap); stander = w.GetRagdolls()[0]; dancer = w.GetRagdolls()[1]; }
                if (step == 60) { stander.GetBody(1).ApplyImpulse(new Vector2F(25f, 0f)); }
                if (step == 130) { stander.GetBody(1).ApplyImpulse(new Vector2F(-45f, 0f)); stander.ApplyHitReaction(1, 0.8f); }
                var pose = new float[dancer.PartCount];
                for (int i = 1; i < pose.Length; i++) { pose[i] = 0.04f * (float)Math64.Sin(0.07 * step + i); }
                dancer.SetAnimationPose(pose, new Vector2D(3 + 0.002 * step, 0), 0f);
                w.Step(1f / 60f);
                ctx.Write((long)w.ComputeHash()); ctx.EndStep();
            }
        });

        Scenarios.Register("world3d/active-ragdolls", ctx =>
        {
            var o = new Physics3D.WorldOptions3D { ThreadCount = ctx.Threads, WorldId = 319 };
            Physics3D.Ragdoll.ApplyRecommendedOptions(o);
            var w = new Physics3D.PhysicsWorld3D(o);
            w.AddBox(new Vector3F(400f, 1f, 400f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
            var stander = w.AddRagdoll(Physics3D.RagdollDef.Humanoid, new Vector3D(0, 0, 0));
            stander.EnableActive(); stander.Stand();
            var dancer = w.AddRagdoll(Physics3D.RagdollDef.Humanoid, new Vector3D(3, 0, 0));
            dancer.EnableActive();
            for (int i = 1; i < dancer.PartCount; i++) { dancer.SetBlendWeight(i, i % 3 == 0 ? 0f : 0.4f); }
            for (int step = 0; step < 240; step++)
            {
                if (step == 100) { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); ctx.Write(snap); stander = w.GetRagdolls()[0]; dancer = w.GetRagdolls()[1]; }
                if (step == 60) { stander.GetBody(1).ApplyImpulse(new Vector3F(0f, 0f, 20f)); }
                if (step == 130) { stander.GetBody(1).ApplyImpulse(new Vector3F(0f, 0f, -30f)); stander.ApplyHitReaction(1, 0.8f); }
                var pose = new QuaternionF[dancer.PartCount];
                pose[0] = QuaternionF.Identity;
                for (int i = 1; i < pose.Length; i++) { double a = 0.06 * Math64.Sin(0.07 * step + i); pose[i] = new QuaternionF((float)Math64.Sin(a * 0.5), 0f, 0f, (float)Math64.Cos(a * 0.5)); }
                dancer.SetAnimationPose(pose, new Vector3D(3 + 0.002 * step, 0, 0));
                w.Step(1f / 60f);
                ctx.Write((long)w.ComputeHash()); ctx.EndStep();
            }
        });
    }
}
