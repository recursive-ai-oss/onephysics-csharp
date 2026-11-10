using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

#pragma warning disable OPHYS0003, OPHYS0002

namespace RecursiveAI.OnePhysics.Tests.Articulations;

public sealed class ArticulationWorldTests2D
{
    private static void Trace(string line)
    {
        string? path = Environment.GetEnvironmentVariable("ARTI_TRACE");
        if (path != null) { System.IO.File.AppendAllText(path, line + "\n"); }
    }

    [Fact]
    public void GravityCompensationHoldsAPlanarArm()
    {
        var world = new PhysicsWorld2D();
        var arm = world.AddArticulation(ArticulationDef.Arm(0.9f, 3), new Vector2D(0, 0.5));
        arm.SetJointPosition(1, 0.8f); arm.SetJointTarget(1, 0.8f);
        arm.SetJointPosition(2, -1.1f); arm.SetJointTarget(2, -1.1f);
        arm.SetJointPosition(3, 0.5f); arm.SetJointTarget(3, 0.5f);
        var q0 = new double[arm.PositionCount]; arm.GetPositions(q0);
        for (int i = 0; i < 600; i++) { world.Step(1f / 60f); }
        var q1 = new double[arm.PositionCount]; arm.GetPositions(q1);
        double worst = 0;
        for (int i = 0; i < q0.Length; i++) { worst = Math.Max(worst, Math.Abs(q1[i] - q0[i])); }
        Assert.True(worst < 1e-4, "drift " + worst);
    }

    [Fact]
    public void ChainOfSixtyFourLinksWithAThousandToOneEndMassStaysConnectedAndStable()
    {
        var world = new PhysicsWorld2D();
        var chain = world.AddArticulation(ArticulationDef.Chain(64, 0.25f, 1f, 1000f), new Vector2D(0, 30));
        double worstSeparation = 0, worstSpeed = 0;
        for (int i = 0; i < 600; i++)
        {
            world.Step(1f / 60f);
            for (int l = 1; l < chain.LinkCount; l++)
            {
                var parent = chain.GetLink(l - 1); var child = chain.GetLink(l);
                var o = l == 1 ? new Vector2F(0, 0) : new Vector2F(0.25f, 0);
                var r = parent.Rotation;
                var expect = new Vector2D(parent.Position.X + r.C * o.X - r.S * o.Y, parent.Position.Y + r.S * o.X + r.C * o.Y);
                worstSeparation = Math.Max(worstSeparation, Math.Sqrt((child.Position.X - expect.X) * (child.Position.X - expect.X) + (child.Position.Y - expect.Y) * (child.Position.Y - expect.Y)));
                worstSpeed = Math.Max(worstSpeed, child.LinearVelocity.Length());
            }
        }

        Trace("2d chain: separation " + worstSeparation + " speed " + worstSpeed);
        Assert.True(worstSeparation < 1e-4, "separation " + worstSeparation);
        Assert.True(worstSpeed < 60.0, "speed " + worstSpeed);
    }

    [Fact]
    public void ChainSwingsIntoAFloorAndABoxWithoutSinkingOrExploding()
    {
        var world = new PhysicsWorld2D();
        world.AddBox(new Vector2F(20f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        var box = world.AddBox(new Vector2F(0.3f, 0.3f), new Vector2D(2.5, 0.15));
        var chain = world.AddArticulation(ArticulationDef.Chain(12, 0.25f, 1f, 5f), new Vector2D(0, 2.2));
        double worstSpeed = 0, lowest = 1e9; int touching = 0;
        for (int i = 0; i < 600; i++)
        {
            world.Step(1f / 60f);
            for (int l = 1; l < chain.LinkCount; l++) { var b = chain.GetLink(l); lowest = Math.Min(lowest, b.Position.Y); worstSpeed = Math.Max(worstSpeed, b.LinearVelocity.Length()); }
            if (chain.ContactPointCount > 0) { touching++; }
        }

        Trace("2d chain on floor: lowest link origin " + lowest + " speed " + worstSpeed + " box " + box.Position + " steps with contacts " + touching);
        Assert.True(touching > 20, "the chain never touched anything");
        Assert.True(lowest > -0.02, "a link went through the floor: " + lowest);
        Assert.True(worstSpeed < 40.0, "speed " + worstSpeed);
        Assert.True(box.Position.X > 2.0 && box.Position.X < 4.0 && box.Position.Y > 0.1, "box at " + box.Position);
    }

    [Fact]
    public void SnapshotRestoreAndForkReplayTheSameHashes()
    {
        PhysicsWorld2D Make()
        {
            var w = new PhysicsWorld2D();
            w.AddBox(new Vector2F(20f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
            w.AddBox(new Vector2F(0.3f, 0.3f), new Vector2D(1.5, 0.15));
            w.AddArticulation(ArticulationDef.Chain(10, 0.25f, 1f, 3f), new Vector2D(0, 2));
            return w;
        }

        var a = Make();
        for (int i = 0; i < 60; i++) { a.Step(1f / 60f); }
        var bytes = a.Snapshot();
        var fork = a.Fork();
        var b = new PhysicsWorld2D();
        b.Restore(bytes);
        Assert.Equal(a.ComputeHash(), b.ComputeHash());
        for (int i = 0; i < 60; i++)
        {
            a.Step(1f / 60f); b.Step(1f / 60f); fork.Step(1f / 60f);
            Assert.Equal(a.ComputeHash(), b.ComputeHash());
            Assert.Equal(a.ComputeHash(), fork.ComputeHash());
        }

        var diff = a.FindDifferences(b);
        Assert.True(diff.Count == 0, diff.ToString());
        Assert.Equal(a.Snapshot(), b.Snapshot());
    }
}
