using System;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Replay;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;


namespace RecursiveAI.OnePhysics.Tests.Articulations;

/// <summary>Host hooks for articulations: origin shift, regions and replay.</summary>
public sealed class ArticulationHostTests
{
    private const float Dt = 1f / 60f;

    private static P3.PhysicsWorld3D World3D(int id)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = id });
        w.AddBox(new Vector3F(200f, 1f, 200f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        w.AddArticulation(P3.ArticulationDef.Quadruped(), new Vector3D(0, 0.625, 0));
        var arm = w.AddArticulation(P3.ArticulationDef.RobotArm(0.9f, 7, 8f), new Vector3D(3, 0, 0));
        arm.SetJointTarget(2, 0.5f);
        w.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), at: new Vector3D(0.625, 0.375, 0.125), density: 200f);
        return w;
    }

    private static P2.PhysicsWorld2D World2D(int id)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = id });
        w.AddBox(new Vector2F(200f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        var arm = w.AddArticulation(P2.ArticulationDef.Arm(0.9f, 3, 4f), new Vector2D(3, 0));
        arm.SetJointTarget(1, 0.6f);
        w.AddArticulation(P2.ArticulationDef.Chain(8, 0.25f, 1f, 5f), new Vector2D(0, 3));
        return w;
    }

    // The articulation works relative to its root and on the 2^-32 grid, so a world shifted by whole grid steps steps bit for bit like the unshifted one, translated (inputs on the grid, as any Q31.32 value is).
    [Fact]
    public void ShiftingTheOriginStepsLikeTheUnshiftedWorld3D()
    {
        var a = World3D(901); var b = World3D(902);
        var shift = new Vector3D(64, 0, -32);
        for (int i = 0; i < 300; i++) { a.Step(Dt); b.Step(Dt); }
        a.ShiftOrigin(shift);
        for (int i = 0; i < 300; i++) { a.Step(Dt); b.Step(Dt); }
        var qa = a.GetArticulations(); var qb = b.GetArticulations();
        for (int k = 0; k < qa.Length; k++)
        {
            for (int l = 0; l < qa[k].LinkCount; l++)
            {
                var pa = qa[k].GetLink(l).Position; var pb = qb[k].GetLink(l).Position;
                Assert.Equal(pb.X - shift.X, pa.X); Assert.Equal(pb.Y - shift.Y, pa.Y); Assert.Equal(pb.Z - shift.Z, pa.Z);
                Assert.Equal(qb[k].GetLink(l).Rotation, qa[k].GetLink(l).Rotation);
            }

            var ja = new double[qa[k].VelocityCount]; var jb = new double[qb[k].VelocityCount];
            qa[k].GetVelocities(ja); qb[k].GetVelocities(jb);
            for (int j = 0; j < ja.Length; j++) { Assert.Equal(jb[j], ja[j]); }
        }
    }

    [Fact]
    public void ShiftingTheOriginStepsLikeTheUnshiftedWorld2D()
    {
        var a = World2D(903); var b = World2D(904);
        var shift = new Vector2D(64, -32);
        for (int i = 0; i < 300; i++) { a.Step(Dt); b.Step(Dt); }
        a.ShiftOrigin(shift);
        for (int i = 0; i < 300; i++) { a.Step(Dt); b.Step(Dt); }
        var qa = a.GetArticulations(); var qb = b.GetArticulations();
        for (int k = 0; k < qa.Length; k++)
        {
            for (int l = 0; l < qa[k].LinkCount; l++)
            {
                var pa = qa[k].GetLink(l).Position; var pb = qb[k].GetLink(l).Position;
                Assert.Equal(pb.X - shift.X, pa.X); Assert.Equal(pb.Y - shift.Y, pa.Y);
            }
        }
    }

    [Fact]
    public void ARegionHoldingOneLinkHoldsTheWholeArticulation3D()
    {
        var w = World3D(905);
        for (int i = 0; i < 30; i++) { w.Step(Dt); }
        var quad = w.GetArticulations()[0]; var arm = w.GetArticulations()[1];
        // A box around the quadruped's torso only: the legs reach out of it, the arm is 3 m away.
        var min = new Vector3D(-0.1, 0.3, -0.1); var max = new Vector3D(0.1, 0.9, 0.1);
        var region = w.CaptureRegion(min, max); var hash = region.ComputeHash();
        for (int l = 0; l < quad.LinkCount; l++) { Assert.Contains(region.Fields, f => f.Entity == "body" && f.Index == quad.GetLink(l).Handle.Index); }
        for (int l = 0; l < arm.LinkCount; l++) { Assert.DoesNotContain(region.Fields, f => f.Entity == "body" && f.Index == arm.GetLink(l).Handle.Index); }
        var encoded = region.ToArray();
        Assert.Equal(encoded, RegionSnapshot.Read(encoded).ToArray());
        arm.SetJointVelocity(2, 1f);
        Assert.Equal(hash, w.ComputeRegionHash(min, max));
        quad.SetJointVelocity(2, 1f);
        Assert.NotEqual(hash, w.ComputeRegionHash(min, max));
        var armBefore = new double[arm.VelocityCount]; arm.GetVelocities(armBefore);
        w.RestoreRegion(encoded);
        Assert.Equal(hash, w.ComputeRegionHash(min, max));
        var armAfter = new double[arm.VelocityCount]; arm.GetVelocities(armAfter);
        Assert.Equal(armBefore, armAfter);
        // The restored articulation steps on like the original would.
        for (int i = 0; i < 30; i++) { w.Step(Dt); }
    }

    [Fact]
    public void ARegionHoldingOneLinkHoldsTheWholeArticulation2D()
    {
        var w = World2D(906);
        for (int i = 0; i < 30; i++) { w.Step(Dt); }
        var chain = w.GetArticulations()[1]; var arm = w.GetArticulations()[0];
        var min = new Vector2D(-0.1, 2.5); var max = new Vector2D(0.1, 3.5);
        var region = w.CaptureRegion(min, max); var hash = region.ComputeHash();
        for (int l = 0; l < chain.LinkCount; l++) { Assert.Contains(region.Fields, f => f.Entity == "body" && f.Index == chain.GetLink(l).Handle.Index); }
        for (int l = 0; l < arm.LinkCount; l++) { Assert.DoesNotContain(region.Fields, f => f.Entity == "body" && f.Index == arm.GetLink(l).Handle.Index); }
        var encoded = region.ToArray();
        arm.SetJointVelocity(1, 1f);
        Assert.Equal(hash, w.ComputeRegionHash(min, max));
        chain.SetJointVelocity(1, 2f);
        Assert.NotEqual(hash, w.ComputeRegionHash(min, max));
        w.RestoreRegion(encoded);
        Assert.Equal(hash, w.ComputeRegionHash(min, max));
    }

    [Fact]
    public void ARecordedSessionReplaysWithEveryHashEqual3D()
    {
        var w = World3D(907);
        var hashes = new ulong[90];
        using var stream = new MemoryStream();
        using (var recorder = w.StartRecording(stream))
        {
            var arm = w.GetArticulations()[1]; var quad = w.GetArticulations()[0];
            for (int i = 0; i < hashes.Length; i++)
            {
                if (i == 10) { arm.SetTaskTarget(arm.LinkCount - 1, new Vector3F(0, 0.1286f, 0), new Vector3D(3, 0.5, 0.4)); }
                if (i == 20) { quad.ApplyForce(0, new Vector3F(30, 0, 0), quad.GetLink(0).Position); }
                if (i == 30) { arm.SetJointTarget(1, 0.7f); }
                if (i == 40) { arm.ClearTaskTarget(); }
                if (i == 50) { quad.SetDrive(2, new ArticulationDrive { Stiffness = 40f, Damping = 2f, MaxForce = 30f, TargetPosition = 0.2f }); }
                w.Step(Dt);
                hashes[i] = w.ComputeHash();
            }
        }

        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var copy = player.CreateWorld3D();
        int n = 0;
        while (player.ReadNextStep(copy)) { Assert.Equal(hashes[n++], copy.ComputeHash()); }
        Assert.Equal(hashes.Length, n);
        Assert.Equal(w.Snapshot(), copy.Snapshot());
    }

    [Fact]
    public void ARecordedSessionReplaysWithEveryHashEqual2D()
    {
        var w = World2D(908);
        var hashes = new ulong[90];
        using var stream = new MemoryStream();
        using (var recorder = w.StartRecording(stream))
        {
            var arm = w.GetArticulations()[0];
            for (int i = 0; i < hashes.Length; i++)
            {
                if (i == 10) { arm.SetTaskTarget(arm.LinkCount - 1, new Vector2F(0, 0.3f), new Vector2D(3.5, 0.6)); }
                if (i == 30) { arm.SetJointTarget(1, 0.2f); }
                if (i == 40) { arm.ClearTaskTarget(); }
                w.Step(Dt);
                hashes[i] = w.ComputeHash();
            }
        }

        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var copy = player.CreateWorld2D();
        int n = 0;
        while (player.ReadNextStep(copy)) { Assert.Equal(hashes[n++], copy.ComputeHash()); }
        Assert.Equal(hashes.Length, n);
        Assert.Equal(w.Snapshot(), copy.Snapshot());
    }
}
