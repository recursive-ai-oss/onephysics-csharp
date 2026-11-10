using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

public class Ragdoll2DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D NewWorld(int threads = 1, int id = 94)
    {
        var o = new WorldOptions2D { ThreadCount = threads, WorldId = id };
        Ragdoll.ApplyRecommendedOptions(o);
        return new PhysicsWorld2D(o);
    }

    private static Body Floor(PhysicsWorld2D w) => w.AddBox(new Vector2F(200f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static, material: new Material2D(0.8f, 0f, 0f));

    private static void Run(PhysicsWorld2D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    private static float Wrap(float a)
    {
        const float Pi = 3.1415927f;
        while (a > Pi) { a -= 2f * Pi; }
        while (a < -Pi) { a += 2f * Pi; }
        return a;
    }

    /// <summary>The joint angle of a part as the pose convention defines it: child angle minus parent angle, minus the same in the bind pose.</summary>
    private static float JointAngle(Ragdoll r, RagdollDef def, int part)
    {
        var p = def.Parts[part];
        float bind = p.Angle - def.Parts[p.Parent].Angle;
        return Wrap(r.GetBody(part).Angle - r.GetBody(p.Parent).Angle - bind);
    }

    [Fact]
    public void JointLimitsHoldUnderADrop()
    {
        var w = NewWorld();
        Floor(w);
        var def = RagdollDef.Humanoid;
        var r = w.AddRagdoll(def, new Vector2D(0, 3), 0.7f);
        float worst = 0f, worstAtRest = 0f;
        for (int i = 0; i < 360; i++)
        {
            w.Step(Dt);
            for (int p = 1; p < def.Parts.Length; p++)
            {
                float a = JointAngle(r, def, p);
                float over = Math.Max(def.Parts[p].LowerAngle - a, a - def.Parts[p].UpperAngle);
                worst = Math.Max(worst, over);
                if (i >= 300) { worstAtRest = Math.Max(worstAtRest, over); }
            }
        }

        // The impact at about 7 m/s pushes a joint a few degrees past its limit for a few steps; at rest the limits hold.
        Assert.True(worst < 0.2f, $"worst limit violation {worst} rad");
        Assert.True(worstAtRest < 0.02f, $"limit violation at rest {worstAtRest} rad");
        // The figure came to rest on the floor.
        Assert.True(r.Root.Position.Y < 1.2);
    }

    [Fact]
    public void PoseDriveReachesATargetPose()
    {
        var w = NewWorld();
        Floor(w);
        var def = RagdollDef.Humanoid;
        var r = w.AddRagdoll(def, new Vector2D(0, 0.02));
        var pose = new float[def.Parts.Length];
        pose[3] = 1.0f; pose[4] = 0.8f; pose[5] = 0.5f; pose[2] = 0.2f;   // raise the arms, tilt the head
        r.DriveToPose(pose);
        Assert.True(r.IsPowered);
        Run(w, 240);
        float err = r.GetPoseError(pose);
        Assert.True(err < 0.1f, $"pose error {err} rad");
        Assert.Equal(1.0f, JointAngle(r, def, 3), 0.1f);
    }

    [Fact]
    public void LimpFigureIsNotDriven()
    {
        var w = NewWorld();
        var def = RagdollDef.Humanoid;
        var r = w.AddRagdoll(def, new Vector2D(0, 5));
        Assert.False(r.IsPowered);
        r.DriveToPose(new float[def.Parts.Length], 1f);
        r.StopDriving();
        Assert.False(r.IsPowered);
    }

    [Fact]
    public void ABrokenJointComesOff()
    {
        var w = NewWorld();
        Floor(w);
        var def = RagdollDef.Humanoid;
        def.Parts[4].BreakTorque = 5f;   // a weak left forearm
        var r = w.AddRagdoll(def, new Vector2D(0, 4));
        Assert.True(r.IsAttached(4));
        Run(w, 240);
        Assert.False(r.IsAttached(4));
        Assert.True(r.IsAttached(3));
        Assert.True(r.IsValid);
    }

    [Fact]
    public void SetPoseTeleportsAndMoveToPoseArrives()
    {
        var w = NewWorld();
        var def = RagdollDef.Humanoid;
        var r = w.AddRagdoll(def, new Vector2D(0, 10));
        var pose = new float[def.Parts.Length];
        pose[3] = 0.9f; pose[0] = 0.3f;
        r.SetPose(pose, new Vector2D(2, 10), 0f);
        Assert.Equal(0.9f, JointAngle(r, def, 3), 0.01f);
        w.Options.Gravity = default;
        pose[3] = 0.2f;
        r.MoveToPose(pose, new Vector2D(2, 10), 0f, Dt);
        w.Step(Dt);
        Assert.Equal(0.2f, JointAngle(r, def, 3), 0.06f);   // one step carries a 0.7 rad move to within a tenth of it
        for (int i = 0; i < 6; i++) { r.MoveToPose(pose, new Vector2D(2, 10), 0f, Dt); w.Step(Dt); }
        Assert.Equal(0.2f, JointAngle(r, def, 3), 0.01f);
    }

    [Fact]
    public void StaleHandlesAndHostileDefinitionsAreRefused()
    {
        var w = NewWorld();
        var r = w.AddRagdoll(RagdollDef.Humanoid, new Vector2D(0, 2));
        int bodies = w.BodyCount;
        r.Remove();
        Assert.False(r.IsValid);
        Assert.ThrowsAny<Exception>(() => r.StopDriving());
        Assert.Equal(bodies - 13, w.BodyCount);

        var d = RagdollDef.Humanoid; d.Parts[3].Mass = -1f;
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddRagdoll(d));
        d = RagdollDef.Humanoid; d.Parts[3].Parent = 9;
        Assert.ThrowsAny<Exception>(() => w.AddRagdoll(d));
        d = RagdollDef.Humanoid; d.Parts[3].LowerAngle = 2f; d.Parts[3].UpperAngle = 1f;
        Assert.ThrowsAny<Exception>(() => w.AddRagdoll(d));
        d = RagdollDef.Humanoid; d.Parts[5].ShapeHalfSize = new Vector2F(float.NaN, 1f);
        Assert.ThrowsAny<ArgumentException>(() => w.AddRagdoll(d));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddRagdoll(RagdollDef.Humanoid, new Vector2D(1e9, 0)));
        Assert.Throws<ArgumentException>(() => w.AddRagdoll(RagdollDef.Humanoid, default, 0f, new float[3]));
        Assert.Equal(0, w.BodyCount);
        Assert.Empty(w.GetRagdolls());
    }

    private static PhysicsWorld2D Pile(int threads)
    {
        var w = NewWorld(threads);
        Floor(w);
        var def = RagdollDef.Humanoid;
        var pose = new float[def.Parts.Length];
        for (int i = 0; i < 12; i++)
        {
            var r = w.AddRagdoll(def, new Vector2D((i % 3) * 0.5 - 0.5, 1 + i * 1.3), 0.1f * i);
            if (i % 3 == 0) { r.DriveToPose(pose, 0.5f); }
        }

        Run(w, 240);
        return w;
    }

    [Fact]
    public void OneAndEightThreadsAgree() => Assert.Equal(Pile(1).ComputeHash(), Pile(8).ComputeHash());

    [Fact]
    public void SnapshotRestoreAndForkStepIdentically()
    {
        var w = Pile(1);
        var snap = w.Snapshot();
        var fork = w.Fork();
        var hashes = new List<ulong>();
        for (int i = 0; i < 90; i++) { w.Step(Dt); hashes.Add(w.ComputeHash()); }
        var end = w.Snapshot();
        for (int i = 0; i < 90; i++) { fork.Step(Dt); Assert.Equal(hashes[i], fork.ComputeHash()); }

        w.Restore(snap);
        for (int i = 0; i < 90; i++) { w.Step(Dt); Assert.Equal(hashes[i], w.ComputeHash()); }
        Assert.True(end.AsSpan().SequenceEqual(w.Snapshot()));

        var fresh = NewWorld();
        fresh.Restore(snap);
        var rs = fresh.GetRagdolls();
        Assert.Equal(12, rs.Length);
        Assert.True(rs[0].IsPowered);
        Assert.False(rs[1].IsPowered);
        for (int i = 0; i < 90; i++) { fresh.Step(Dt); Assert.Equal(hashes[i], fresh.ComputeHash()); }
    }
}
