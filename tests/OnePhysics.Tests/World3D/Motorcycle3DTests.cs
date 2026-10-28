using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

public class Motorcycle3DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld3D NewWorld() => new(new WorldOptions3D { ThreadCount = 1, WorldId = 94 });

    private static void Floor(PhysicsWorld3D w) =>
        w.AddBox(new Vector3F(1600f, 1f, 1600f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static, material: new Material3D(0.8f, 0f, 0f));

    /// <summary>Throttle that holds a speed (a proportional cut-in; the bike's own engine does the rest).</summary>
    private static float Cruise(Vehicle bike, float target) => Math.Clamp(0.25f * (target - bike.ForwardSpeed) + 0.15f, 0f, 1f);

    private static void RunCruise(PhysicsWorld3D w, Vehicle bike, float speed, float steer, int steps)
    {
        for (int i = 0; i < steps; i++) { bike.SetInput(Cruise(bike, speed), steer); w.Step(Dt); }
    }

    [Fact]
    public void StandsUprightOnItsOwnAtRest()
    {
        var w = NewWorld();
        Floor(w);
        var bike = w.AddMotorcycle(new Vector3D(0, 0, 0));
        Assert.Equal(VehicleKind.Motorcycle, bike.Kind);
        for (int i = 0; i < 300; i++) { w.Step(Dt); }
        Assert.InRange(Math.Abs(bike.LeanAngle), 0f, 0.01f);
        Assert.True(bike.GetWheel(0).IsInContact && bike.GetWheel(1).IsInContact);
        // A shove sideways is caught.
        { var ch = bike.Chassis; ch.AngularVelocity = new Vector3F(0f, 0f, 0.8f); }
        for (int i = 0; i < 300; i++) { w.Step(Dt); }
        Assert.InRange(Math.Abs(bike.LeanAngle), 0f, 0.02f);
    }

    [Fact]
    public void HoldsLeanInASteadyTurnWithoutFalling()
    {
        var w = NewWorld();
        Floor(w);
        var bike = w.AddMotorcycle(new Vector3D(0, 0, 0));
        RunCruise(w, bike, 8f, 0f, 360);
        Assert.InRange(bike.ForwardSpeed, 7f, 9f);
        float sum = 0f, target = 0f, minLean = 9f, maxLean = -9f, yawSum = 0f; int n = 0;
        for (int i = 0; i < 600; i++)
        {
            bike.SetInput(Cruise(bike, 8f), 0.3f);
            w.Step(Dt);
            if (i >= 300)
            {
                sum += bike.LeanAngle; target += bike.TargetLeanAngle; n++;
                minLean = Math.Min(minLean, bike.LeanAngle); maxLean = Math.Max(maxLean, bike.LeanAngle);
                yawSum += bike.Chassis.AngularVelocity.Y;
            }
        }

        float lean = sum / n, want = target / n;
        Assert.InRange(want, 0.3f, 0.7f);                 // a real lean was asked for ...
        Assert.InRange(lean, want - 0.06f, want + 0.06f);  // ... and held
        Assert.InRange(maxLean - minLean, 0f, 0.1f);       // steadily
        // The yaw rate of the single-track model: v tan(delta) / L, turning towards +X (positive about +Y).
        float delta = 0.3f * bike.SteerLimit;
        float idealYaw = bike.ForwardSpeed * MathF.Tan(delta) / 1.4f;
        Assert.InRange(yawSum / n / idealYaw, 0.7f, 1.2f);
        Assert.True(bike.Chassis.Position.Y > 0.3);
    }

    [Fact]
    public void LeansTheOtherWayForTheOtherTurn()
    {
        var w = NewWorld();
        Floor(w);
        var bike = w.AddMotorcycle(new Vector3D(0, 0, 0));
        RunCruise(w, bike, 8f, 0f, 360);
        RunCruise(w, bike, 8f, -0.3f, 300);
        Assert.True(bike.LeanAngle < -0.2f);
    }

    [Fact]
    public void BrakesToAStopUpright()
    {
        var w = NewWorld();
        Floor(w);
        var bike = w.AddMotorcycle(new Vector3D(0, 0, 0));
        RunCruise(w, bike, 10f, 0f, 420);
        for (int i = 0; i < 360; i++) { bike.SetInput(0f, 0f, 0.8f); w.Step(Dt); }
        Assert.InRange(bike.Chassis.LinearVelocity.Length(), 0f, 0.1f);
        for (int i = 0; i < 120; i++) { bike.SetInput(0f, 0f, 0.8f); w.Step(Dt); }
        Assert.InRange(Math.Abs(bike.LeanAngle), 0f, 0.03f);
        var up = bike.Chassis.Rotation.Rotate(new Vector3F(0f, 1f, 0f));
        Assert.True(up.Y > 0.999f);
    }

    [Fact]
    public void SteeringLimitShrinksWithSpeed()
    {
        var w = NewWorld();
        Floor(w);
        var def = MotorcycleDef.Road;
        var bike = w.AddMotorcycle(new Vector3D(0, 0, 0), def);
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        Assert.Equal(def.MaxSteerAngle, bike.SteerLimit, 1e-5f);
        RunCruise(w, bike, 17f, 0f, 600);
        float mid = bike.SteerLimit;
        Assert.InRange(mid, def.HighSpeedSteerAngle + 0.01f, def.MaxSteerAngle - 0.01f);
        RunCruise(w, bike, 33f, 0f, 1200);
        Assert.True(bike.SteerLimit < mid);
    }

    [Fact]
    public void WithoutTheAssistItFallsOver()
    {
        var w = NewWorld();
        Floor(w);
        var def = MotorcycleDef.Road;
        def.MaxLeanTorque = 0f;
        var bike = w.AddMotorcycle(new Vector3D(0, 0, 0), def);
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        { var ch = bike.Chassis; ch.AngularVelocity = new Vector3F(0f, 0f, 1.5f); }
        for (int i = 0; i < 300; i++) { w.Step(Dt); }
        Assert.True(Math.Abs(bike.LeanAngle) > 1.0f);
    }

    [Fact]
    public void SnapshotMidTurnIsByteExact()
    {
        var w = NewWorld();
        Floor(w);
        var bike = w.AddMotorcycle(new Vector3D(0, 0, 0));
        RunCruise(w, bike, 8f, 0f, 240);
        RunCruise(w, bike, 8f, 0.4f, 60);
        var snap = w.Snapshot();
        float lean = bike.LeanAngle;
        RunCruise(w, bike, 8f, 0.4f, 100);
        var hash = w.ComputeHash();
        var bytes = w.Snapshot();
        var fresh = NewWorld();
        fresh.Restore(snap);
        var b2 = fresh.GetVehicles()[0];
        Assert.Equal(lean, b2.LeanAngle);
        RunCruise(fresh, b2, 8f, 0.4f, 100);
        Assert.Equal(hash, fresh.ComputeHash());
        Assert.Equal(bytes, fresh.Snapshot());
    }
}
