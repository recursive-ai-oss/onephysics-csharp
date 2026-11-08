using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.LowLevel;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Core;

[Collection("Allocation")]
public class InspectionDetailTests
{
    private const float Dt = 1f / 60;

    [Fact]
    public void GravityDampingAndExternalImpulsesBalanceLinearMomentum()
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { Gravity = new Vector2F(0, -10), SleepEnabled = false });
        var b = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = new Vector3F(0, -10, 0), SleepEnabled = false });
        var body2 = a.CreateBody(new P2.BodyDef { LinearDamping = 1, AngularDamping = 2, LinearVelocity = new Vector2F(2, 3) });
        a.CreateBox(body2, new Vector2F(1, 1));
        var body3 = b.CreateBody(new P3.BodyDef { LinearDamping = 1, AngularDamping = 2, LinearVelocity = new Vector3F(2, 3, 0) });
        b.CreateBox(body3, new Vector3F(1, 1, 1));
        body2.ApplyImpulse(new Vector2F(1, 0)); body2.ApplyForce(new Vector2F(4, 0)); body2.ApplyTorque(2);
        body3.ApplyImpulse(new Vector3F(1, 0, 0)); body3.ApplyForce(new Vector3F(4, 0, 0)); body3.ApplyTorque(new Vector3F(0, 0, 2));
        a.Step(Dt); b.Step(Dt);
        var x = body2.Inspect(); var y = body3.Inspect();
        Assert.Equal(-10 * body2.Mass * (double)Dt, x.GravityImpulse.Y, 7);
        Assert.Equal(-10 * body3.Mass * (double)Dt, y.GravityImpulse.Y, 7);
        Assert.Equal(1 + 4 * (double)Dt, x.ExternalImpulse.X, 7);
        Assert.Equal(1 + 4 * (double)Dt, y.ExternalImpulse.X, 7);
        var ix = x.GravityImpulse + x.ExternalImpulse + x.DampingImpulse;
        var iy = y.GravityImpulse + y.ExternalImpulse + y.DampingImpulse;
        Assert.InRange(Math.Abs((body2.LinearVelocity.X - 2) * body2.Mass - ix.X), 0, 2e-6);
        Assert.InRange(Math.Abs((body2.LinearVelocity.Y - 3) * body2.Mass - ix.Y), 0, 2e-6);
        Assert.InRange(Math.Abs((body3.LinearVelocity.X - 2) * body3.Mass - iy.X), 0, 2e-6);
        Assert.InRange(Math.Abs((body3.LinearVelocity.Y - 3) * body3.Mass - iy.Y), 0, 2e-6);
        Assert.True(x.DampingImpulse.X < 0 && y.DampingImpulse.X < 0);
        Assert.True(x.DampingAngularImpulse < 0 && y.DampingAngularImpulse.Z < 0);
        Assert.Equal(2 * (double)Dt, x.ExternalAngularImpulse, 7);
        Assert.Equal(2 * (double)Dt, y.ExternalAngularImpulse.Z, 7);
        a.Step(Dt); b.Step(Dt);
        Assert.Equal(default, body2.Inspect().ExternalImpulse);
        Assert.Equal(default, body3.Inspect().ExternalImpulse);
    }

    [Fact]
    public void TinyContactAndPendulumCountersHaveKnownAnswers()
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { SleepEnabled = false });
        var floor = a.AddBox(new Vector2F(10, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        a.AddBox(new Vector2F(1, 1), new Vector2D(0, 0.5));
        a.Step(Dt);
        Assert.Equal(1, a.Counters.NewManifolds); Assert.Equal(2, a.Counters.NewContactPoints);
        Assert.Equal(0, a.Counters.PersistedManifolds + a.Counters.RecycledManifolds);
        a.Step(Dt);
        Assert.Equal(0, a.Counters.NewManifolds);
        Assert.Equal(1, a.Counters.PersistedManifolds + a.Counters.RecycledManifolds);
        Assert.True(floor.Inspect().ConstraintColorMask != 0);
        var b = new P2.PhysicsWorld2D(new P2.WorldOptions2D { SleepEnabled = false });
        var top = b.AddCircle(0.1f, default, BodyType.Static);
        var mass = b.AddCircle(0.2f, new Vector2D(0, -1));
        b.AddHinge(top, mass, default);
        b.Step(Dt);
        Assert.Equal(1, b.Counters.Joints); Assert.Equal(0, b.Counters.Manifolds);
        Assert.Equal(1, b.Counters.AwakeBodies); Assert.Equal(1, b.Counters.AwakeIslands);
        Assert.Equal(b.Options.Substeps, b.Counters.WorldWarmStartPasses);
        Assert.Equal(1, b.Counters.WorldRestitutionPasses);
    }

    [Fact]
    public void SoftControllerAndCcdCountersAndProfilingAreAvailable()
    {
        var world = new P3.PhysicsWorld3D(new P3.WorldOptions3D { SleepEnabled = false });
        world.AddCloth(1, 1, 2, new Vector3D(0, 4, 0), pinCorners: true);
        world.AddCharacter(new Vector3D(10, 5, 0), P3.CharacterDef.Humanoid);
        var chassis = world.AddBox(new Vector3F(2, 0.5f, 4), new Vector3D(20, 5, 0));
        world.AddVehicle(chassis, P3.VehicleDef.Car);
        long stamp = 0; world.Profile.TimestampSource = () => ++stamp;
        world.Step(Dt);
        Assert.Equal(9, world.Counters.SoftParticles); Assert.Equal(1, world.Counters.SoftBlocks);
        Assert.Equal(1, world.Counters.Characters); Assert.Equal(1, world.Counters.Vehicles);
        Assert.True(world.Profile.Durations[(int)StepStage.SoftConstraints] > 0);
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { Gravity = default });
        w.AddBox(new Vector2F(0.1f, 20), new Vector2D(10, 0), type: BodyType.Static);
        var bullet = w.AddCircle(0.1f, default); bullet.IsBullet = true; bullet.LinearVelocity = new Vector2F(300, 0);
        bool hit = false;
        for (int i = 0; i < 4; i++) { w.Step(Dt); Assert.True(w.Counters.CcdBodies >= w.Counters.BulletBodies); hit |= w.Counters.CcdHits > 0; }
        Assert.True(hit);
    }

    [Fact]
    public void SpatialInspectionAndOverlaysAllocateNothingInSteadyState()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { SleepEnabled = false });
        var anchor = w.AddSphere(0.1f, new Vector3D(0, 3, 0), BodyType.Static);
        var mass = w.AddBox(new Vector3F(1, 1, 1), new Vector3D(0, 2, 0));
        var joint = w.AddPoint(anchor, mass, anchor.Position);
        var geometry = new DebugGeometry();
        void Read()
        {
            _ = mass.Inspect().GravityImpulse; _ = mass.Inspect().ConstraintColorMask;
            var view = joint.Inspect(); _ = view.ForceB; _ = view.TorqueB; _ = view.Rows[0].Error;
            w.GetDebugGeometry(DebugContent.Inspection, geometry);
        }
        for (int i = 0; i < 80; i++) { w.Step(Dt); Read(); }
        Assert.Equal(2, geometry.Bodies.Length); Assert.Single(geometry.ConstraintInspection.ToArray());
        Assert.Equal(2, geometry.InspectionVectors.Length); Assert.Equal(3, geometry.JointErrors.Length);
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 20; i++) { w.Step(Dt); Read(); }
        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);
        geometry.Clear(); Assert.Empty(geometry.InspectionVectors.ToArray()); Assert.Empty(geometry.JointErrors.ToArray());
    }
}
