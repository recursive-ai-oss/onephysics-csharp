#pragma warning disable OPHYS0003
using System;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

public class ActiveRagdoll2DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D NewWorld(int threads = 1, int id = 118)
    {
        var o = new WorldOptions2D { ThreadCount = threads, WorldId = id };
        Ragdoll.ApplyRecommendedOptions(o);
        return new PhysicsWorld2D(o);
    }

    private static void Floor(PhysicsWorld2D w) => w.AddBox(new Vector2F(400f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);

    private static void Run(PhysicsWorld2D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    private static (double X, double Y, float A)[] Poses(Ragdoll r) => Enumerable.Range(0, r.PartCount).Select(i => { var b = r.GetBody(i); var p = b.Position; return (p.X, p.Y, b.Angle); }).ToArray();

    private static float[] WavingPose(int n, int step)
    {
        var pose = new float[n];
        for (int i = 1; i < n; i++) { pose[i] = 0.04f * (float)Math64.Sin(0.05 * step + i); }   // inside every joint's limits
        return pose;
    }

    [Fact]
    public void StableDriveStaysBoundedAtAGainWhereExplicitPdDiverges()
    {
        // An explicit proportional-derivative step on one hinge of inertia 0.5 at 200 Hz and 60 Hz stepping: the textbook loop is unstable once (2 pi f dt)^2 is large.
        double kp = 0.5 * Math.Pow(2 * Math.PI * 200, 2), kd = 2 * 0.5 * 2 * Math.PI * 200, th = 0.1, om = 0;
        for (int i = 0; i < 200; i++) { double tau = kp * (0 - th) - kd * om; om += Dt * tau / 0.5; th += Dt * om; }
        Assert.True(!double.IsFinite(th) || Math.Abs(th) > 1e6, "the explicit controller should have diverged");

        var w = NewWorld();
        w.Options.Gravity = new Vector2F(0f, 0f);
        var r = w.AddRagdoll(RagdollDef.Humanoid, new Vector2D(0, 0));
        var def = ActiveRagdollDef.Humanoid; def.PdHertz = 100f; def.LegPdHertz = 100f;
        r.EnableActive(def);
        var pose = WavingPose(r.PartCount, 0);
        r.SetAnimationPose(pose);
        for (int i = 1; i < r.PartCount; i++) { r.SetBlendWeight(i, 0.01f); }
        Run(w, 180);
        Assert.True(r.GetPoseError(pose) < 0.12f, "the drive should stay bounded around the pose at 100 Hz: error " + r.GetPoseError(pose));
        for (int i = 0; i < r.PartCount; i++) { var v = r.GetBody(i).AngularVelocity; Assert.True(float.IsFinite(v) && Math.Abs(v) < 50f); }
    }

    [Fact]
    public void WeightZeroFollowsTheAnimationBitForBit()
    {
        var w = NewWorld();
        Floor(w);
        var r = w.AddRagdoll(RagdollDef.Humanoid, new Vector2D(0, 0));
        r.EnableActive();
        var reference = NewWorld(id: 119);
        var rr = reference.AddRagdoll(RagdollDef.Humanoid, new Vector2D(0, 0));
        for (int step = 0; step < 90; step++)
        {
            var pose = WavingPose(r.PartCount, step);
            var at = new Vector2D(0.01 * step, 0.2); float angle = 0.002f * step;
            r.SetAnimationPose(pose, at, angle);
            w.Step(Dt);
            rr.SetPose(pose, at, angle);
            Assert.Equal(Poses(rr), Poses(r));
        }
    }

    [Fact]
    public void WeightOneIsBitForBitAPlainRagdoll()
    {
        var a = NewWorld(); var b = NewWorld(id: 120);
        Floor(a); Floor(b);
        var ra = a.AddRagdoll(RagdollDef.Humanoid, new Vector2D(0, 1.0), 0.3f);
        var rb = b.AddRagdoll(RagdollDef.Humanoid, new Vector2D(0, 1.0), 0.3f);
        ra.EnableActive();
        var w1 = Enumerable.Repeat(1f, ra.PartCount).ToArray();
        ra.SetBlendWeights(w1);
        ra.SetAnimationPose(WavingPose(ra.PartCount, 3));
        for (int i = 0; i < 150; i++) { a.Step(Dt); b.Step(Dt); }
        Assert.Equal(Poses(rb), Poses(ra));
    }

    [Fact]
    public void HitReactionRecoversWithItsTimeConstant()
    {
        var w = NewWorld();
        Floor(w);
        var r = w.AddRagdoll(RagdollDef.Humanoid, new Vector2D(0, 0));
        var def = ActiveRagdollDef.Humanoid; def.HitRecoveryTime = 0.4f;
        r.EnableActive(def);
        r.SetBlendWeight(3, 0.2f);
        r.ApplyHitReaction(3, 1f);
        float w0 = r.GetEffectiveBlendWeight(3);
        Assert.True(w0 > 0.99f);
        Run(w, 24);   // 0.4 s: one time constant
        float expected = 0.2f + 0.8f * (float)Math.Exp(-1.0);
        Assert.InRange(r.GetEffectiveBlendWeight(3), expected - 0.02f, expected + 0.02f);
        Run(w, 120);
        Assert.InRange(r.GetEffectiveBlendWeight(3), 0.2f, 0.215f);
    }

    [Theory]
    [InlineData(20f, false)]
    [InlineData(50f, true)]
    public void StandingFigureRecoversFromAPush(float impulse, bool mayStep)
    {
        var w = NewWorld();
        Floor(w);
        var r = w.AddRagdoll(RagdollDef.Humanoid, new Vector2D(0, 0));
        r.EnableActive();
        r.Stand();
        Run(w, 90);
        r.GetBody(1).ApplyImpulse(new Vector2F(impulse, 0f));
        Run(w, 360);
        Assert.True(r.GetBody(0).Position.Y > 0.85, "the pelvis should still be at standing height");
        Assert.True(r.GetBody(2).Position.Y > 1.4, "the head should be up");
        if (!mayStep) { Assert.Equal(0, r.GetActiveInfo().StepCount); }
    }

    [Fact]
    public void SnapshotRestoreAndForkReplayTheSameHashes()
    {
        var w = NewWorld();
        Floor(w);
        var r = w.AddRagdoll(RagdollDef.Humanoid, new Vector2D(0, 0));
        r.EnableActive();
        r.Stand();
        r.SetAnimationPose(WavingPose(r.PartCount, 4));
        r.SetBlendWeight(4, 0.3f);
        Run(w, 40);
        r.GetBody(1).ApplyImpulse(new Vector2F(45f, 0f));
        r.ApplyHitReaction(1, 0.7f);
        Run(w, 20);
        var snap = w.Snapshot();
        var fork = w.Fork();
        var hashes = new ulong[60];
        for (int i = 0; i < 60; i++) { w.Step(Dt); hashes[i] = w.ComputeHash(); }
        var bytes = w.Snapshot();
        var forkHashes = new ulong[60];
        for (int i = 0; i < 60; i++) { fork.Step(Dt); forkHashes[i] = fork.ComputeHash(); }
        Assert.Equal(hashes, forkHashes);
        w.Restore(snap);
        for (int i = 0; i < 60; i++) { w.Step(Dt); Assert.Equal(hashes[i], w.ComputeHash()); }
        Assert.Equal(bytes, w.Snapshot());
        var fresh = NewWorld();
        fresh.Restore(snap);
        for (int i = 0; i < 60; i++) { fresh.Step(Dt); Assert.Equal(hashes[i], fresh.ComputeHash()); }
    }

    [Fact]
    public void OneThreadAndEightThreadsAgree()
    {
        ulong Run1(int threads)
        {
            var w = NewWorld(threads, id: 130 + threads);
            Floor(w);
            var r = w.AddRagdoll(RagdollDef.Humanoid, new Vector2D(0, 0));
            r.EnableActive(); r.Walk(0.8f);
            for (int i = 0; i < 200; i++) { w.Step(Dt); }
            return w.ComputeHash();
        }

        Assert.Equal(Run1(1), Run1(8));
    }

    [Fact]
    public void WorldsWithoutActiveRagdollsKeepTheirBytes()
    {
        var a = NewWorld(); var b = NewWorld(id: 121);
        a.AddRagdoll(RagdollDef.Humanoid, new Vector2D(0, 1)); b.AddRagdoll(RagdollDef.Humanoid, new Vector2D(0, 1));
        var rb = b.GetRagdolls()[0];
        rb.EnableActive(); rb.DisableActive();
        Assert.Equal(a.ComputeHash(), b.ComputeHash());
        Assert.Equal(a.Snapshot(), b.Snapshot());
    }

    [Fact]
    public void BadInputIsRefusedWithNamedErrors()
    {
        var w = NewWorld();
        var r = w.AddRagdoll(RagdollDef.Humanoid, new Vector2D(0, 1));
        Assert.Throws<InvalidOperationException>(() => r.Stand());
        Assert.Throws<ArgumentOutOfRangeException>(() => r.EnableActive(new ActiveRagdollDef { Bones = new BipedBones { LeftThigh = 99 } }));
        var bad = ActiveRagdollDef.Humanoid; bad.Walker.StepTime = float.NaN;
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() => r.EnableActive(bad));
        Assert.Contains("StepTime", ex.Message);
        Assert.False(r.IsActive);
        r.EnableActive();
        Assert.Throws<ArgumentOutOfRangeException>(() => r.SetBlendWeight(2, 1.5f));
        Assert.Throws<ArgumentException>(() => r.SetAnimationPose(new float[3]));
        Assert.Throws<ArgumentOutOfRangeException>(() => r.Walk(float.NaN));
        Assert.Throws<InvalidOperationException>(() => r.DriveToPose(new float[r.PartCount]));
        r.Remove();
        Assert.Throws<StaleHandleException>(() => r.IsActive);
    }
}
