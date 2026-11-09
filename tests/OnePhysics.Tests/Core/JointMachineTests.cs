using System;
using System.IO;
using RecursiveAI.OnePhysics.Replay;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Core;

public class JointMachineTests
{
    private const float Dt = 1f / 60f;

    [Theory]
    [InlineData(1)]
    [InlineData(8)]
    public void FiltersApplyNoImpulseAndExpireWhileSleeping(int threads)
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { Gravity = default, ThreadCount = threads });
        var b = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = default, ThreadCount = threads });
        var x = a.AddCircle(1f, default); var y = a.AddCircle(1f, default);
        var u = b.AddSphere(1f, default); var v = b.AddSphere(1f, default);
        var j = a.CreateJoint(new P2.FilterJointDef { BodyA = x, BodyB = y, LifetimeSteps = 3 });
        var k = b.CreateJoint(new P3.FilterJointDef { BodyA = u, BodyB = v, LifetimeSteps = 3 });
        int breaks2 = 0, breaks3 = 0;
        j.OnJointBreak += e => { Assert.Equal(0f, e.Value); breaks2++; };
        k.OnJointBreak += e => { Assert.Equal(0f, e.Value); breaks3++; };
        a.Step(Dt); b.Step(Dt);
        Assert.Equal(default, x.LinearVelocity); Assert.Equal(default, u.LinearVelocity);
        Assert.Equal(2, j.LifetimeStepsRemaining); Assert.Equal(2, k.LifetimeStepsRemaining);
        var ca = a.Fork(); var cb = b.Fork();
        var sa = P2.PhysicsWorld2D.LoadScene(new StringReader(a.SaveScene()));
        var sb = P3.PhysicsWorld3D.LoadScene(new StringReader(b.SaveScene()));
        Assert.Equal(a.Snapshot(), ca.Snapshot()); Assert.Equal(b.Snapshot(), cb.Snapshot());
        x.Sleep(); y.Sleep(); u.Sleep(); v.Sleep();
        a.Step(Dt); b.Step(Dt); Assert.True(j.IsValid && k.IsValid);
        a.Step(Dt); b.Step(Dt); Assert.False(j.IsValid || k.IsValid);
        Assert.Equal(1, breaks2); Assert.Equal(1, breaks3);
        Assert.Throws<StaleHandleException>(() => j.LifetimeStepsRemaining);
        ca.Step(Dt); cb.Step(Dt); sa.Step(Dt); sb.Step(Dt);
        Assert.Equal(ca.Snapshot(), sa.Snapshot()); Assert.Equal(cb.Snapshot(), sb.Snapshot());
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BreakingUsesWholeStepWeightWithoutRequiringInspection(bool inspection)
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { DetailedInspection = inspection, SleepEnabled = false });
        var b = new P3.PhysicsWorld3D(new P3.WorldOptions3D { DetailedInspection = inspection, SleepEnabled = false });
        var x = a.AddCircle(0.1f, default, BodyType.Static); var y = a.AddCircle(0.1f, new(0, -1));
        var u = b.AddSphere(0.1f, default, BodyType.Static); var v = b.AddSphere(0.1f, new(0, -1, 0));
        var j = a.AddHinge(x, y, default); var k = b.AddPoint(u, v, default);
        j.BreakForce = y.Mass * 10f; k.BreakForce = v.Mass * 10f;
        for (int i = 0; i < 60; i++) { a.Step(Dt); b.Step(Dt); }
        Assert.True(j.IsValid && k.IsValid);
        j.BreakForce = y.Mass * 9f; k.BreakForce = v.Mass * 9f;
        a.Step(Dt); b.Step(Dt);
        Assert.False(j.IsValid || k.IsValid);
    }

    [Fact]
    public void SpatialMouseCapsVectorForceAndMotorReachesPose()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = default, DetailedInspection = true, SleepEnabled = false });
        var a = w.AddSphere(0.1f, default, BodyType.Static);
        var b = w.AddSphere(0.1f, new(2, 3, 4));
        var mouse = w.AddMouse(a, b, new(5, 6, 7), 2f);
        for (int i = 0; i < 600; i++)
        {
            w.Step(Dt); var f = mouse.Inspect().ForceB;
            Assert.InRange(double.Sqrt(f.X * f.X + f.Y * f.Y + f.Z * f.Z), 0, 2.0001);
            Assert.False(mouse.Inspect().IsMotorActive);
        }
        Assert.InRange((b.Position - new Vector3D(5, 6, 7)).Length(), 0, 0.001);
        mouse.Destroy(); b.LinearVelocity = default; b.Rotation = new QuaternionF(0, 0, 0.3f, 0.9539392f);
        var motor = w.AddMotor(a, b, new(1, 2, 3), QuaternionF.Identity, 100, 100);
        for (int i = 0; i < 600; i++) { w.Step(Dt); }
        Assert.InRange((b.Position - new Vector3D(1, 2, 3)).Length(), 0, 0.001);
        Assert.True(motor.IsValid);
        Assert.InRange(Math.Abs(b.Rotation.X) + Math.Abs(b.Rotation.Y) + Math.Abs(b.Rotation.Z), 0, 0.001);
    }

    [Fact]
    public void SpatialSuspensionSettlesAtStaticDeflection()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = new(-9.81f, 0, 0), SleepEnabled = false });
        var a = w.AddSphere(0.1f, default, BodyType.Static); var b = w.AddSphere(0.1f, default);
        w.CreateJoint(new P3.WheelJointDef { BodyA = a, BodyB = b, EnableSpring = true, SpringHertz = 2f, SpringDampingRatio = 0.7f });
        for (int i = 0; i < 600; i++) { w.Step(Dt); }
        double expected = -9.81 / Math.Pow(2 * Math.PI * 2, 2);
        Assert.InRange(b.Position.X / expected, 0.9, 1.1);
        Assert.InRange(Math.Abs(b.Position.Y) + Math.Abs(b.Position.Z), 0, 0.0001);
    }
    [Fact]
    public void PlanarGearRackPulleyAndConfigurableMatchKinematics()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { Gravity = default, SleepEnabled = false });
        var ground = w.AddCircle(0.05f, new(0, 20), BodyType.Static);
        var a = w.AddCircle(0.2f, new(-4, 0)); var b = w.AddCircle(0.2f, new(-2, 0));
        w.AddHinge(ground, a, a.Position); w.AddHinge(ground, b, b.Position);
        w.AddGear(a, b, 2f); a.AngularVelocity = 3;
        var rack = w.AddBox(new Vector2F(0.2f, 0.2f), new(0, 0));
        w.AddSlider(ground, rack, rack.Position, new(1, 0)); w.AddRackAndPinion(a, rack, new(1, 0), 0.5f);
        var c = w.AddCircle(0.1f, new(5, 0)); var d = w.AddCircle(0.1f, new(7, 0));
        w.AddPulley(c, d, c.Position, d.Position, new(5, 3), new(7, 3)); c.LinearVelocity = new(0, 1);
        var e = w.AddCircle(0.1f, new(10, 0));
        w.CreateJoint(new P2.ConfigurableJointDef { BodyA = ground, BodyB = e, LocalAnchorA = new(10, -20), X = new() { Mode = P2.JointAxisMode.Free, EnableMotor = true, MotorSpeed = 0.5f, MaxMotor = 10 } });
        for (int i = 0; i < 1000; i++)
        {
            w.Step(Dt);
            if (i > 10) { Assert.InRange(Math.Abs(a.AngularVelocity + 2 * b.AngularVelocity), 0, 0.001); }
            Assert.InRange(Math.Abs(rack.LinearVelocity.X - 0.5 * a.AngularVelocity), 0, 0.001);
            double rope = (c.Position - new Vector2D(5, 3)).Length() + (d.Position - new Vector2D(7, 3)).Length();
            Assert.InRange(rope, 5.98, 6.02);
        }
        Assert.InRange(e.Position.X, 18, 19);
        Assert.InRange(Math.Abs(e.Position.Y), 0, 0.001);
        var fork = w.Fork();
        var loaded = P2.PhysicsWorld2D.LoadScene(new StringReader(w.SaveScene()));
        Assert.Equal(w.Snapshot(), fork.Snapshot()); Assert.Equal(w.Snapshot(), loaded.Snapshot());
        for (int i = 0; i < 10; i++) { w.Step(Dt); fork.Step(Dt); loaded.Step(Dt); Assert.Equal(w.ComputeHash(), fork.ComputeHash()); Assert.Equal(w.ComputeHash(), loaded.ComputeHash()); }
    }

    [Fact]
    public void PlanarPathFollowsSplineAndThreadsMatch()
    {
        P2.PhysicsWorld2D Build(int threads)
        {
            var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { Gravity = default, SleepEnabled = false, ThreadCount = threads, WorldId = 21, DetailedInspection = true });
            w.SetLayerCollision(0, 0, false);
            var ground = w.AddCircle(0.1f, default, BodyType.Static);
            for (int i = 0; i < 16; i++)
            {
                var b = w.AddCircle(0.05f, new(0, 0.1 * i));
                w.CreateJoint(new P2.PathJointDef { BodyA = ground, BodyB = b, Points = [new(0, 0), new(2, 0), new(4, 2), new(6, 2)], EnableMotor = true, MotorSpeed = 0.3f, MaxMotorForce = 10 });
            }
            return w;
        }
        var a = Build(1); var b = Build(8);
        for (int i = 0; i < 1000; i++)
        {
            a.Step(Dt); b.Step(Dt); Assert.Equal(a.ComputeHash(), b.ComputeHash());
            if (i > 100)
            {
                var p = a.Core.Bodies.PositionOf(1);
                P2.PathSpline2D.Closest([new(0, 0), new(2, 0), new(4, 2), new(6, 2)], false, new((float)p[0], (float)p[1]), out _, out var point, out _);
                Assert.InRange((point - new Vector2F((float)p[0], (float)p[1])).Length(), 0, 0.001);
            }
        }
        Assert.Equal(JointRowBits.None, a.GetJoint(a.Core.HandleOfJoint(0)).Inspect().Rows[4].Flags);
        var restored = P2.PhysicsWorld2D.LoadScene(new StringReader(a.SaveScene())); Assert.Equal(a.Snapshot(), restored.Snapshot());
    }

    [Fact]
    public void SpatialMachinesForkSceneAndRecordEveryMutation()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = default, WorldId = 32 });
        using var stream = new MemoryStream();
        using (w.StartRecording(stream))
        {
            var g = w.AddSphere(0.05f, default, BodyType.Static);
            var a = w.AddSphere(0.1f, new(2, 0, 0));
            var b = w.AddSphere(0.1f, new(4, 0, 0));
            var c = w.AddSphere(0.1f, new(6, 0, 0));
            var wheel = w.AddWheel(g, a, a.Position, new(1, 0, 0));
            wheel.EnableMotor = true; wheel.MotorSpeed = 0.3f; wheel.MaxMotorTorque = 0.01f;
            var motor = w.AddMotor(g, b, new(4, 1, 0), QuaternionF.Identity, 10, 10);
            var mouse = w.AddMouse(g, c, new(6, 1, 0), 10);
            for (int i = 0; i < 30; i++) w.Step(Dt);
            mouse.Target = new(6, 2, 0); motor.SetOffset(new(4, 2, 0), QuaternionF.Identity);
            for (int i = 0; i < 30; i++) w.Step(Dt);
        }
        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var replay = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 32 });
        while (player.ReadNextStep(replay)) { }
        Assert.Equal(w.Snapshot(), replay.Snapshot());
        var fork = w.Fork(); var scene = P3.PhysicsWorld3D.LoadScene(new StringReader(w.SaveScene()), threadCount: 8);
        byte[] snapshot = w.Snapshot();
        for (int i = 0; i < 60; i++) { w.Step(Dt); fork.Step(Dt); scene.Step(Dt); Assert.Equal(w.ComputeHash(), fork.ComputeHash()); Assert.Equal(w.ComputeHash(), scene.ComputeHash()); }
        byte[] expected = w.Snapshot(); w.Restore(snapshot);
        for (int i = 0; i < 60; i++) w.Step(Dt);
        Assert.Equal(expected, w.Snapshot());
    }

    [Fact]
    public void PlanarMachinesRecordCreationAxisEditsAndLifetime()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { Gravity = default, WorldId = 33 });
        using var stream = new MemoryStream();
        using (w.StartRecording(stream))
        {
            var g = w.AddCircle(0.1f, default, BodyType.Static);
            var a = w.AddCircle(0.1f, new(2, 0)); var b = w.AddCircle(0.1f, new(4, 0));
            var config = w.AddConfigurable(g, a, a.Position);
            config.SetAxisMode(P2.JointAxes.LinearX, P2.JointAxisMode.Free);
            config.SetAxisLimits(P2.JointAxes.LinearX, -1, 1);
            config.SetAxisMotor(P2.JointAxes.LinearX, true, 0.2f, 1);
            config.SetAxisSpring(P2.JointAxes.AngularZ, true, 2, 0.7f);
            var path = w.AddPath(g, b, [new(4, 0), new(5, 0), new(6, 1)]);
            path.EnableMotor = true; path.MotorSpeed = 0.1f; path.MaxMotorForce = 2;
            path.LifetimeStepsRemaining = 25;
            Assert.Throws<ArgumentOutOfRangeException>(() => path.LifetimeStepsRemaining = -1);
            for (int i = 0; i < 40; i++) w.Step(Dt);
        }
        stream.Position = 0; var player = new ReplayPlayer(stream);
        var replay = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 33 });
        while (player.ReadNextStep(replay)) { }
        Assert.Equal(w.Snapshot(), replay.Snapshot());
    }

    [Fact]
    public void RampedLoadsBreakAtTheFirstWholeStepAboveThreshold()
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { Gravity = default, SleepEnabled = false });
        var b = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = default, SleepEnabled = false });
        var g = a.AddCircle(0.1f, default, BodyType.Static); var mass = a.AddCircle(0.1f, default);
        var h = b.AddSphere(0.1f, default, BodyType.Static); var body = b.AddSphere(0.1f, default);
        var j = a.AddHinge(g, mass, default); var k = b.AddPoint(h, body, default);
        j.BreakForce = 10; k.BreakForce = 10;
        for (int step = 1; step <= 3; step++)
        {
            mass.ApplyForce(new(0, 4 * step)); body.ApplyForce(new(0, 4 * step, 0));
            a.Step(Dt); b.Step(Dt);
            Assert.Equal(step < 3, j.IsValid); Assert.Equal(step < 3, k.IsValid);
        }
    }

    [Fact]
    public void AuthoredMachineDataAndBreakSettingsRoundTrip()
    {
        const string text = "scene 1\nname machines\ndim 2\ngravity 0 0\nmaterial 0 0.6 0\nbody static 0 0 0 0 0 0 -\nshape circle 0 0 0.1 0 0\nbody dynamic 1 0 0 0 0 0 -\nshape circle 0 1 0.1 0 0\njoint gear 0 1 0 0 0 0 data 1 2 lifetime 3 break 5 Infinity\n";
        var scene = RecursiveAI.OnePhysics.Scene.Scene.Read(new StringReader(text));
        using var writer = new StringWriter(); scene.Write(writer);
        var copy = RecursiveAI.OnePhysics.Scene.Scene.Read(new StringReader(writer.ToString()));
        Assert.Equal(scene.Joints[0].Data, copy.Joints[0].Data);
        var world = copy.CreateWorld2D();
        for (int i = 0; i < 3; i++) world.Step(Dt);
        Assert.Equal(0, world.JointCount);
    }

    [Fact]
    public void RampedTorqueBreaksAtTheFirstWholeStepAboveThreshold()
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { Gravity = default, SleepEnabled = false });
        var b = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = default, SleepEnabled = false });
        var g = a.AddCircle(0.5f, default, BodyType.Static); var mass = a.AddCircle(0.5f, default);
        var h = b.AddSphere(0.5f, default, BodyType.Static); var body = b.AddSphere(0.5f, default);
        var j = a.AddFixed(g, mass, default); var k = b.AddFixed(h, body, default);
        j.BreakTorque = 10; k.BreakTorque = 10;
        for (int step = 1; step <= 3; step++)
        {
            mass.ApplyTorque(4 * step); body.ApplyTorque(new(0, 0, 4 * step));
            a.Step(Dt); b.Step(Dt);
            Assert.Equal(step < 3, j.IsValid); Assert.Equal(step < 3, k.IsValid);
        }
    }

    [Fact]
    public void OriginShiftRefusesOutOfRangeMachineTargetsAtomically()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = default });
        var a = w.AddSphere(0.1f, default, BodyType.Static); var b = w.AddSphere(0.1f, new(1, 0, 0));
        var mouse = w.AddMouse(a, b, new(2, 0, 0), 10);
        byte[] before = w.Snapshot();
        Assert.Throws<ArgumentOutOfRangeException>(() => w.ShiftOrigin(new(3000000, 0, 0)));
        Assert.Equal(before, w.Snapshot());
        w.ShiftOrigin(new(1, 0, 0));
        Assert.Equal(new Vector3D(1, 0, 0), mouse.Target);
        var restored = P3.PhysicsWorld3D.LoadScene(new StringReader(w.SaveScene()));
        Assert.Equal(w.Snapshot(), restored.Snapshot());
    }

    [Fact]
    public void VeryLargeFiniteMeanForceProducesRestorableBreakEvents()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { Gravity = default, SleepEnabled = false });
        var a = w.AddCircle(0.5f, default, BodyType.Static); var b = w.AddCircle(0.5f, default);
        var joint = w.AddHinge(a, b, default); joint.BreakForce = 1;
        b.LinearVelocity = new(1e10f, 0);
        float got = 0; joint.OnJointBreak += e => got = e.Value;
        w.Step(1e-37f);
        Assert.False(joint.IsValid); Assert.Equal(float.MaxValue, got);
        var copy = w.Fork(); copy.Restore(w.Snapshot()); Assert.Equal(w.Snapshot(), copy.Snapshot());
    }

    [Fact]
    public void RawConstraintPayloadRemainsOpaqueUntilLifetimeIsEnabled()
    {
        var w = new WorldCore(new WorldOptions { Layout = DimensionLayout.Planar, WorldId = 125 });
        var a = w.CreateBody(BodyType.Dynamic, new double[] { 0, 0 }); var b = w.CreateBody(BodyType.Dynamic, new double[] { 1, 0 });
        int c = w.AddConstraint(new ConstraintDesc { Kind = ConstraintKind.Joint, BodyA = a, BodyB = b });
        w.Constraints.Payload(c)[32] = 1.25f; w.Constraints.Payload(c)[33] = -4f;
        Assert.Equal(0, JointLifetime.Get(w.Constraints, c));
        var copy = new WorldCore(new WorldOptions { Layout = DimensionLayout.Planar, WorldId = 125 });
        copy.Restore(w.Snapshot()); Assert.Equal(w.Snapshot(), copy.Snapshot());
        JointLifetime.Set(w.Constraints, c, int.MaxValue);
        copy.Restore(w.Snapshot()); Assert.Equal(int.MaxValue, JointLifetime.Get(copy.Constraints, c));
    }

    [Fact]
    public void TargetShortcutsRejectNaNForceWithoutCreatingAJoint()
    {
        var a = new P2.PhysicsWorld2D(); var b = new P3.PhysicsWorld3D();
        var g = a.AddCircle(0.1f, default, BodyType.Static); var x = a.AddCircle(0.1f, new(1, 0));
        var h = b.AddSphere(0.1f, default, BodyType.Static); var y = b.AddSphere(0.1f, new(1, 0, 0));
        byte[] beforeA = a.Snapshot(), beforeB = b.Snapshot();
        Assert.Throws<ArgumentOutOfRangeException>(() => a.AddTarget(g, x, default, maxForce: float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => b.AddTarget(h, y, default, maxForce: float.NaN));
        Assert.Equal(beforeA, a.Snapshot()); Assert.Equal(beforeB, b.Snapshot());
    }

    [Fact]
    public void ZeroFrequencyDoesNotLockAFreeSuspensionOrConfigurableAxis()
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { Gravity = default });
        var b = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = default });
        var g = a.AddCircle(0.1f, default, BodyType.Static); var x = a.AddCircle(0.1f, default);
        var h = b.AddSphere(0.1f, default, BodyType.Static); var y = b.AddSphere(0.1f, default);
        a.CreateJoint(new P2.ConfigurableJointDef { BodyA = g, BodyB = x, X = new() { Mode = P2.JointAxisMode.Free, EnableSpring = true }, Y = new() { Mode = P2.JointAxisMode.Free }, Angle = new() { Mode = P2.JointAxisMode.Free } });
        b.CreateJoint(new P3.WheelJointDef { BodyA = h, BodyB = y, EnableSpring = true });
        x.LinearVelocity = new(1, 0); y.LinearVelocity = new(1, 0, 0);
        a.Step(Dt); b.Step(Dt);
        Assert.Equal(1f, x.LinearVelocity.X); Assert.Equal(1f, y.LinearVelocity.X);
    }

}
