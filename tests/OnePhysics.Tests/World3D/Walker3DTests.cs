using System;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Replay;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

/// <summary>The 3D walker: walking, turning, pushes, snapshots, replays and thread counts, on the happy path (a ragdoll preset and <c>Walk</c>) and on the detailed one.</summary>
public class Walker3DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld3D NewWorld(int threads = 1, int id = 330)
    {
        var o = new WorldOptions3D { ThreadCount = threads, WorldId = id };
        Ragdoll.ApplyRecommendedOptions(o);
        var w = new PhysicsWorld3D(o);
        w.AddBox(new Vector3F(400f, 1f, 400f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        return w;
    }

    private static Ragdoll NewWalker(PhysicsWorld3D w, double x = 0)
    {
        var def = RagdollDef.Humanoid; def.CategoryBits = 2u; def.MaskBits = 1u;
        var r = w.AddRagdoll(def, new Vector3D(x, 0, 0));
        r.EnableActive();
        r.Stand();
        return r;
    }

    private static void Run(PhysicsWorld3D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    private static double Yaw(Ragdoll r) { var f = r.GetBody(0).Rotation.Rotate(new Vector3F(0f, 0f, 1f)); return Math.Atan2(f.X, f.Z); }

    [Fact]
    public void ARagdollPresetWalksAtTheCommandedSpeedWithoutFalling()
    {
        var w = NewWorld();
        var r = NewWalker(w);
        Run(w, 60);
        r.Walk(0f, 0.8f);
        Run(w, 300);
        double z0 = r.GetBody(0).Position.Z;
        Run(w, 900);   // 15 s
        var p = r.GetBody(0).Position;
        Assert.True(p.Y > 0.7, "the walker is still on its feet");
        double speed = (p.Z - z0) / 15.0;
        Assert.InRange(speed, 0.6, 1.0);
        Assert.True(Math.Abs(p.X) < 1.5, "no large sideways drift");
        Assert.Equal(ActiveMode.Walk, r.Mode);
        Assert.True(r.GetActiveInfo().StepCount > 20);
    }

    [Fact]
    public void ADetailedWalkCommandAndAQuarterTurnAreFollowed()
    {
        var w = NewWorld();
        var r = NewWalker(w);
        Run(w, 60);
        r.Walk(new Vector2F(0f, 0.8f), 0f);
        Run(w, 300);
        float quarter = (float)(Math.PI / 2);
        r.Walk(new Vector2F(0.8f, 0f), quarter);   // a right angle at once
        Run(w, 900);
        var p = r.GetBody(0).Position;
        Assert.True(p.Y > 0.7, "the walker did not fall in the turn");
        Assert.InRange(Yaw(r), quarter - 0.3, quarter + 0.3);
        Assert.True(p.X > 6.0, "it carried on along the new heading");
    }

    [Theory]
    [InlineData(20f, 0f, 0)]
    [InlineData(-20f, 0f, 5)]
    [InlineData(20f, 0f, 11)]
    [InlineData(-20f, 0f, 16)]
    [InlineData(0f, 30f, 2)]
    [InlineData(0f, -30f, 7)]
    [InlineData(0f, 30f, 13)]
    [InlineData(0f, -30f, 18)]
    public void AWalkerRecoversFromAShoveAtAnyPhase(float impulseX, float impulseZ, int delaySteps)
    {
        var w = NewWorld();
        var r = NewWalker(w);
        Run(w, 60);
        r.Walk(0f, 0.8f);
        Run(w, 400 + delaySteps);
        r.GetBody(0).ApplyImpulse(new Vector3F(impulseX, 0f, impulseZ));
        Run(w, 500);
        Assert.True(r.GetBody(0).Position.Y > 0.7, "the walker recovered");
    }

    [Fact]
    public void AFigureToldToStopWhileWalkingEndsStanding()
    {
        var w = NewWorld();
        var r = NewWalker(w);
        Run(w, 60);
        r.Walk(0f, 0.8f);
        Run(w, 367);
        r.Walk(0f, 0f);
        Run(w, 300);
        var info = r.GetActiveInfo();
        Assert.True(r.GetBody(0).Position.Y > 0.7, "it is standing");
        Assert.True(Math.Abs(r.GetBody(0).LinearVelocity.Z) < 0.3f);
        Assert.True(info.GroundedFeet >= 1);
    }

    [Fact]
    public void OneThreadAndEightThreadsWalkTheSameWay()
    {
        ulong Run1(int threads)
        {
            var w = NewWorld(threads, 331);
            var r = NewWalker(w);
            Run(w, 60);
            r.Walk(0f, 0.8f);
            Run(w, 250);
            r.Walk(1.2f, 0.8f);
            Run(w, 250);
            r.GetBody(0).ApplyImpulse(new Vector3F(15f, 0f, 0f));
            Run(w, 200);
            return w.ComputeHash();
        }

        Assert.Equal(Run1(1), Run1(8));
    }

    [Fact]
    public void ASnapshotTakenMidStrideForkAndRestoreContinueBitForBit()
    {
        var w = NewWorld(1, 332);
        var r = NewWalker(w);
        Run(w, 60);
        r.Walk(0.3f, 0.8f);
        Run(w, 233);   // some way into a stride
        var snap = w.Snapshot();
        var hashes = new ulong[200];
        for (int i = 0; i < hashes.Length; i++) { w.Step(Dt); hashes[i] = w.ComputeHash(); }
        var bytes = w.Snapshot();

        var fork = NewWorld(1, 332);
        fork.Restore(snap);
        for (int i = 0; i < hashes.Length; i++) { fork.Step(Dt); Assert.Equal(hashes[i], fork.ComputeHash()); }
        Assert.Equal(bytes, fork.Snapshot());

        w.Restore(snap);
        for (int i = 0; i < hashes.Length; i++) { w.Step(Dt); Assert.Equal(hashes[i], w.ComputeHash()); }
        Assert.Equal(bytes, w.Snapshot());

        var copy = w.Fork();
        copy.Restore(snap);
        for (int i = 0; i < 50; i++) { copy.Step(Dt); Assert.Equal(hashes[i], copy.ComputeHash()); }
    }

    [Fact]
    public void ARecordedWalkPlaysBackWithEveryHashEqual()
    {
        var w = NewWorld(1, 333);
        var r = NewWalker(w);
        using var stream = new MemoryStream();
        using (var recording = w.StartRecording(stream))
        {
            for (int i = 0; i < 360; i++)
            {
                if (i == 20) { r.Walk(0f, 0.8f); }
                if (i == 200) { r.Walk(0.9f, 0.7f); }
                if (i == 300) { r.GetBody(0).ApplyImpulse(new Vector3F(0f, 0f, 20f)); }
                w.Step(Dt);
            }
        }

        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var replay = new PhysicsWorld3D(new WorldOptions3D { WorldId = 333 });
        int steps = 0;
        while (player.ReadNextStep(replay)) { steps++; }
        Assert.Equal(360, steps);
        Assert.Equal(w.ComputeHash(), replay.ComputeHash());
    }

    [Fact]
    public void WalkRejectsValuesItCannotUse()
    {
        var w = NewWorld();
        var r = NewWalker(w);
        Assert.Throws<ArgumentOutOfRangeException>(() => r.Walk(float.NaN, 1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => r.Walk(0f, -1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => r.Walk(0f, float.PositiveInfinity));
        Assert.Throws<ArgumentOutOfRangeException>(() => r.Walk(new Vector2F(float.NaN, 0f), 0f));
        r.Walk(0f, 0.8f);
        Assert.Equal(ActiveMode.Walk, r.Mode);
    }

    [Fact]
    public void TheDefaultActiveSettingsOfA3dFigureAreTheTunedWalkingOnes()
    {
        var d = ActiveRagdollDef.Humanoid3D;
        Assert.Equal(0.15f, d.Walker.StepWidth);
        Assert.Equal(2f, d.Walker.UpperBodyHertz);
        Assert.Equal(7f, d.LoadPdHertz);
        var plain = ActiveRagdollDef.Humanoid;
        Assert.Equal(0.2f, plain.Walker.StepWidth);   // the shared preset (and the planar walker) is unchanged
        var w = NewWorld();
        var r = NewWalker(w);
        r.EnableActive(plain);   // a detailed definition replaces the defaults
        r.Walk(0f, 0.5f);
        Run(w, 120);
        Assert.Equal(ActiveMode.Walk, r.Mode);
    }
}
