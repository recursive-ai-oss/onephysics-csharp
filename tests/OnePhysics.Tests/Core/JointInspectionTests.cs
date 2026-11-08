using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Core;

public class JointInspectionTests
{
    private const float Dt = 1f / 60f;

    [Theory]
    [InlineData(1, 1)]
    [InlineData(8, 8)]
    public void HangingMassHasWholeStepWeightInBothDimensions(int threads, int width)
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { ThreadCount = threads, SleepEnabled = false });
        a.Engine.KernelWidth = width;
        var pivot = a.AddCircle(0.05f, default, BodyType.Static);
        var bob = a.AddBox(new Vector2F(0.5f, 0.5f), new Vector2D(0, -1));
        var hinge = a.AddHinge(pivot, bob, default);
        var b = new P3.PhysicsWorld3D(new P3.WorldOptions3D { ThreadCount = threads, SleepEnabled = false });
        b.Engine.KernelWidth = width;
        var top = b.AddSphere(0.05f, default, BodyType.Static);
        var mass = b.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(0, -1, 0));
        var joint = b.AddPoint(top, mass, default);
        for (int i = 0; i < 240; i++) { a.Step(Dt); b.Step(Dt); }
        var x = hinge.Inspect(); var y = joint.Inspect();
        Assert.InRange(x.ForceB.Y / (bob.Mass * 9.81), 0.99, 1.01);
        Assert.InRange(y.ForceB.Y / (mass.Mass * 9.81), 0.99, 1.01);
        Assert.Equal(-x.ForceA.Y, x.ForceB.Y);
        Assert.Equal(-y.ForceA.Y, y.ForceB.Y);
        Assert.InRange(Math.Abs(x.TorqueB), 0, 1e-6);
        Assert.InRange(Math.Abs(y.TorqueB.Z), 0, 1e-6);
        Assert.True(x.Color < 24 && y.Color < 24, $"colours {x.Color}/{y.Color}");
        Assert.True(x.Island >= 0 && y.Island >= 0, $"islands {x.Island}/{y.Island}");
        Assert.False(x.IsBroken || y.IsBroken || x.IsMotorActive || y.IsMotorActive);
        Assert.True((x.Rows[0].Flags & JointRowBits.Enabled) != 0, "2D rows missing");
        Assert.True((y.Rows[2].Flags & JointRowBits.Enabled) != 0, "3D rows missing");
    }

    [Fact]
    public void SymmetricTwoJointTrussCarriesHalfTheWeightPerRod()
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { SleepEnabled = false });
        var l = a.AddCircle(0.05f, new Vector2D(-1, 1), BodyType.Static);
        var r = a.AddCircle(0.05f, new Vector2D(1, 1), BodyType.Static);
        var bob = a.AddBox(new Vector2F(0.2f, 0.2f), default);
        var left = a.AddDistance(l, bob, l.Position, bob.Position);
        var right = a.AddDistance(r, bob, r.Position, bob.Position);
        var b = new P3.PhysicsWorld3D(new P3.WorldOptions3D { SleepEnabled = false });
        var l3 = b.AddSphere(0.05f, new Vector3D(-1, 1, 0), BodyType.Static);
        var r3 = b.AddSphere(0.05f, new Vector3D(1, 1, 0), BodyType.Static);
        var bob3 = b.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), default);
        var left3 = b.AddDistance(l3, bob3, l3.Position, bob3.Position);
        var right3 = b.AddDistance(r3, bob3, r3.Position, bob3.Position);
        for (int i = 0; i < 360; i++) { a.Step(Dt); b.Step(Dt); }
        double half = bob.Mass * 9.81 / 2, half3 = bob3.Mass * 9.81 / 2;
        Assert.InRange(left.Inspect().ForceB.Y / half, 0.98, 1.02);
        Assert.InRange(right.Inspect().ForceB.Y / half, 0.98, 1.02);
        Assert.InRange(left.Inspect().ForceB.X / -half, 0.98, 1.02);
        Assert.InRange(right.Inspect().ForceB.X / half, 0.98, 1.02);
        Assert.InRange(left3.Inspect().ForceB.Y / half3, 0.98, 1.02);
        Assert.InRange(right3.Inspect().ForceB.Y / half3, 0.98, 1.02);
        Assert.InRange(left3.Inspect().ForceB.X / -half3, 0.98, 1.02);
        Assert.InRange(right3.Inspect().ForceB.X / half3, 0.98, 1.02);
    }

    [Fact]
    public void ConstrainedTorqueReadbackBalancesAnAppliedCouple()
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { Gravity = default, SleepEnabled = false });
        var anchor = a.AddCircle(0.1f, new Vector2D(0, 2), BodyType.Static);
        var body = a.AddBox(new Vector2F(1, 1), default);
        var weld = a.AddFixed(anchor, body, body.Position);
        var b = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = default, SleepEnabled = false });
        var top = b.AddSphere(0.1f, new Vector3D(0, 2, 0), BodyType.Static);
        var mass = b.AddBox(new Vector3F(1, 1, 1), default);
        var gear = b.AddGear(top, mass, new Vector3F(0, 0, 1), new Vector3F(0, 0, 1));
        for (int i = 0; i < 120; i++) { body.ApplyTorque(3f); mass.ApplyTorque(new Vector3F(0, 0, 3)); a.Step(Dt); b.Step(Dt); }
        Assert.InRange(weld.Inspect().TorqueB, -3.03, -2.97);
        Assert.InRange(gear.Inspect().TorqueB.Z, -3.03, -2.97);
        Assert.InRange(weld.Inspect().TorqueA, 2.97, 3.03);
        // Gear axes have the same signed Jacobian, so their two torques need not be opposite.
        Assert.InRange(gear.Inspect().TorqueA.Z, -3.03, -2.97);
        Assert.True((gear.Inspect().Rows[0].Flags & JointRowBits.Velocity) != 0);
    }

    [Fact]
    public void HeldViewRetainsBreakReadbackAndRejectsReusedJointGeneration()
    {
        var world = new P3.PhysicsWorld3D(new P3.WorldOptions3D { SleepEnabled = false });
        var a = world.AddSphere(0.1f, new Vector3D(0, 1, 0), BodyType.Static);
        var b = world.AddSphere(0.2f, default);
        var joint = world.AddPoint(a, b, a.Position);
        joint.BreakForce = 0.0001f;
        var view = joint.Inspect();
        world.Step(Dt);
        Assert.False(joint.IsValid);
        Assert.True(view.IsBroken);
        Assert.True(view.ForceB.Y > 0);
        Assert.True(view.Color < 24);
        Assert.True(view.Island >= 0);
        Assert.True(joint.Inspect().IsBroken);
        Assert.True(world.InspectJoint(joint.Joint).IsBroken);
        world.AddPoint(a, b, b.Position);
        Assert.Throws<StaleHandleException>(() => view.ForceB);
    }

    [Fact]
    public void LimitsAndMotorsReportActivityIncludingZeroCapacity()
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { SleepEnabled = false });
        var pivot = a.AddCircle(0.1f, new Vector2D(0, 3), BodyType.Static);
        var body = a.AddBox(new Vector2F(0.2f, 0.2f), default);
        var hinge = a.AddHinge(pivot, body, body.Position);
        hinge.SetLimits(0, 0); hinge.EnableLimit = true; hinge.EnableMotor = true; hinge.MotorSpeed = 1; hinge.MaxMotorTorque = 1;
        var free = a.AddBox(new Vector2F(0.2f, 0.2f), new Vector2D(5, 0));
        var zero = a.AddRelative(pivot, free, maxForce: 0, maxTorque: 0);
        var b = new P3.PhysicsWorld3D(new P3.WorldOptions3D { SleepEnabled = false });
        var top = b.AddSphere(0.1f, new Vector3D(0, 3, 0), BodyType.Static);
        var mass = b.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), default);
        var h3 = b.AddHinge(top, mass, mass.Position, new Vector3F(0, 0, 1));
        h3.SetLimits(0, 0); h3.EnableLimit = true; h3.EnableMotor = true; h3.MotorSpeed = 1; h3.MaxMotorTorque = 1;
        var other = b.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), new Vector3D(5, 0, 0));
        var driven = b.AddCharacterJoint(top, other, other.Position, new Vector3F(1, 0, 0));
        driven.SetPoseDrive(QuaternionF.Identity, 10, 1, 0);
        a.Step(Dt); b.Step(Dt);
        Assert.True(hinge.Inspect().IsLimitActive && hinge.Inspect().IsMotorActive, $"2D limit/motor {hinge.Inspect().IsLimitActive}/{hinge.Inspect().IsMotorActive}");
        Assert.True(h3.Inspect().IsLimitActive && h3.Inspect().IsMotorActive, $"3D limit/motor {h3.Inspect().IsLimitActive}/{h3.Inspect().IsMotorActive}");
        Assert.False(zero.Inspect().IsMotorActive || driven.Inspect().IsMotorActive);
        hinge.EnableMotor = false; h3.EnableMotor = false;
        a.Step(Dt); b.Step(Dt);
        Assert.False(hinge.Inspect().IsMotorActive || h3.Inspect().IsMotorActive);
    }

    [Fact]
    public void EveryJointSceneReportsDeterministicallyAndDoesNotChangeSnapshots()
    {
        var a = World2D.JointTests.BuildJointScene(1, kernelWidth: 1);
        var b = World2D.JointTests.BuildJointScene(8, kernelWidth: 8);
        var c = World3D.Joint3DTests.BuildJointScene(1);
        var d = World3D.Joint3DTests.BuildJointScene(8);
        c.Engine.KernelWidth = 1; d.Engine.KernelWidth = 8;
        c.Options.SleepEnabled = false; d.Options.SleepEnabled = false;
        foreach (var world in new[] { c, d })
        {
            var pinion = world.AddBox(new Vector3F(0.3f, 0.3f, 0.3f), new Vector3D(20, 10, 0));
            var rack = world.AddBox(new Vector3F(0.3f, 0.3f, 0.3f), new Vector3D(22, 10, 0));
            world.AddRackAndPinion(pinion, rack, new Vector3F(0, 0, 1), new Vector3F(1, 0, 0), 2f);
        }
        for (int i = 0; i < 120; i++)
        {
            a.Step(Dt); b.Step(Dt); c.Step(Dt); d.Step(Dt);
            Assert.Equal(a.ComputeHash(), b.ComputeHash()); Assert.Equal(c.ComputeHash(), d.ComputeHash());
            Assert.True(a.Core.JointReadback.Impulses.SequenceEqual(b.Core.JointReadback.Impulses));
            Assert.True(c.Core.JointReadback.Impulses.SequenceEqual(d.Core.JointReadback.Impulses));
        }
        foreach (var world in new[] { a.Core, c.Core })
        {
            for (int j = 0; j < world.Constraints.HighWater; j++)
            {
                if (!world.Constraints.IsLive(j) || world.Constraints.KindOf(j) != ConstraintKind.Joint) { continue; }
                var view = world.JointReadback.Inspect(world.HandleOfJoint(j));
                bool found = false;
                foreach (var row in view.Rows) { if (row.Flags != 0) { found = true; Assert.True(float.IsFinite(row.Error)); } }
                Assert.True(found, $"no rows for {world.Layout.LinearLanes}D joint slot {j}, awake {world.Bodies.IsAwake(world.Constraints.BodyB[j])}");
                Assert.True(double.IsFinite(view.ForceB.X) && double.IsFinite(view.TorqueB.Y));
            }
        }
    }
}
