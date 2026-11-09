#pragma warning disable OPHYS0003
using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

public class Vehicle2DTests
{
    private const float Dt = 1f / 60f;
    private const float G = 9.81f;

    private static PhysicsWorld2D NewWorld(int threads = 1, int id = 92) => new(new WorldOptions2D { ThreadCount = threads, WorldId = id });

    /// <summary>Static floor with its top at y = 0.</summary>
    private static Body Floor(PhysicsWorld2D w, float friction = 0.6f, double half = 400, double cx = 0) =>
        w.AddBox(new Vector2F((float)(2 * half), 1f), at: new Vector2D(cx, -0.5), type: BodyType.Static, material: new Material2D(friction, 0f, 0f));

    private static void Run(PhysicsWorld2D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    private static Vehicle CarOn(PhysicsWorld2D w, double x, VehicleDef? def = null)
    {
        def ??= VehicleDef.Car;
        return w.AddVehicle(new Vector2D(x, 0), def, new Vector2F(3.2f, 0.8f), 1500f);
    }

    [Fact]
    public void SuspensionAtRestHoldsTheStaticDeflection()
    {
        var w = NewWorld();
        Floor(w);
        var car = CarOn(w, 0);
        Run(w, 360);
        float defl = G / ((2f * MathF.PI * 1.5f) * (2f * MathF.PI * 1.5f));
        float sum = 0f;
        for (int i = 0; i < 2; i++)
        {
            var ws = car.GetWheel(i);
            if (!ws.IsInContact) Assert.Fail($"wheel {i} y {car.Chassis.Position.Y} len {ws.SuspensionLength} c {ws.Center}");
            Assert.Equal(0.5f - defl, ws.SuspensionLength, 0.05f * defl + 0.003f);
            sum += ws.SuspensionForce;
        }

        Assert.Equal(1500f * G, sum, 0.02f * 1500f * G);
        double y0 = car.Chassis.Position.Y;
        Run(w, 60);
        Assert.Equal(y0, car.Chassis.Position.Y, 0.002);
    }

    [Fact]
    public void AStraightLaunchReachesTheSpeedTheDrivetrainPredicts()
    {
        var w = NewWorld();
        Floor(w, half: 4000);
        var def = VehicleDef.Car;
        def.Transmission.Automatic = false;
        var car = CarOn(w, -3000, def);
        car.Gear = 1;
        float predicted = 6500f * 2f * MathF.PI / 60f * 0.34f / (3.5f * 3.4f);
        for (int i = 0; i < 60 * 12; i++)
        {
            car.Drive(1f);
            w.Step(Dt);
            Assert.True(car.ForwardSpeed < predicted * 1.03f, $"step {i}: {car.ForwardSpeed} above {predicted}");
        }

        Assert.InRange(car.ForwardSpeed, predicted * 0.95f, predicted * 1.03f);
        Assert.InRange(car.EngineRpm, 5800f, 6800f);
    }

    [Fact]
    public void TheAutomaticBoxShiftsUpAndGoesFaster()
    {
        var w = NewWorld();
        Floor(w, half: 6000);
        var car = CarOn(w, -5000);
        for (int i = 0; i < 60 * 16; i++) { car.Drive(1f); w.Step(Dt); }
        Assert.True(car.Gear >= 3, $"gear {car.Gear}");
        Assert.True(car.ForwardSpeed > 25f, $"speed {car.ForwardSpeed}");
    }

    [Fact]
    public void BrakingStopsWithinTheTyresFrictionLimit()
    {
        var w = NewWorld();
        Floor(w, half: 2000);
        var def = VehicleDef.Car;
        foreach (var wd in def.Wheels) { wd.MaxBrakeTorque = 8000f; }
        var car = CarOn(w, -500, def);
        Run(w, 120);
        var chassis = car.Chassis;
        chassis.LinearVelocity = new Vector2F(20f, 0f);
        double x0 = car.Chassis.Position.X;
        for (int i = 0; i < 60 * 8; i++) { car.Drive(0f, 0f, 1f); w.Step(Dt); }
        double dist = car.Chassis.Position.X - x0;
        // Every tyre slides at slip -1, where the curve is about 0.939 of its peak; the grip is sqrt(tyre friction * floor friction).
        float mu = MathF.Sqrt(1.5f * 0.6f) * 0.939f;
        double expected = 20.0 * 20.0 / (2.0 * mu * G);
        Assert.InRange(dist, expected * 0.95, expected * 1.05);
        Assert.True(Math.Abs(car.Chassis.LinearVelocity.X) < 0.2f, $"speed {car.Chassis.LinearVelocity.X}");
    }

    [Fact]
    public void StaleVehicleHandleThrowsClearly()
    {
        var w = NewWorld();
        Floor(w);
        var car = CarOn(w, 0);
        Assert.True(car.IsValid);
        car.Chassis.Destroy();
        Assert.False(car.IsValid);
        Assert.ThrowsAny<Exception>(() => car.Drive(1f));
        Assert.Empty(w.GetVehicles());
    }

    [Fact]
    public void HostileDefinitionsAreRefused()
    {
        var w = NewWorld();
        var d = VehicleDef.Car; d.Wheels[0].Radius = float.NaN;
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddVehicle(new Vector2D(0, 0), d, new Vector2F(3, 1), 1000f));
        d = VehicleDef.Car; d.Engine.MaxRpm = 10f;
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddVehicle(new Vector2D(0, 0), d, new Vector2F(3, 1), 1000f));
        d = VehicleDef.Car; d.Differentials = new[] { new DifferentialDef { Left = 5, Right = -1 } };
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddVehicle(new Vector2D(0, 0), d, new Vector2F(3, 1), 1000f));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddVehicle(new Vector2D(0, 0), VehicleDef.Car, new Vector2F(3, 1), -1f));
        Assert.Empty(w.GetVehicles());
        Assert.Equal(0, w.BodyCount);
    }

    private static ulong DriveHash(int threads, int steps, Func<PhysicsWorld2D, Vehicle> make, out PhysicsWorld2D world)
    {
        var w = NewWorld(threads);
        Floor(w, half: 4000);
        var v = make(w);
        for (int i = 0; i < steps; i++) { v.Drive(i < 200 ? 1f : 0.3f, 0f, i > 400 ? 0.5f : 0f); w.Step(Dt); }
        world = w;
        return w.ComputeHash();
    }

    [Fact]
    public void OneAndEightThreadsAgree()
    {
        ulong a = DriveHash(1, 500, w => CarOn(w, 0), out _);
        ulong b = DriveHash(8, 500, w => CarOn(w, 0), out _);
        Assert.Equal(a, b);
    }

    [Fact]
    public void SnapshotRestoreStepsIdentically()
    {
        DriveHash(1, 150, w => CarOn(w, 0), out var w1);
        var snap = w1.Snapshot();
        var hashes = new List<ulong>();
        var car = w1.GetVehicles()[0];
        for (int i = 0; i < 120; i++) { car.Drive(1f); w1.Step(Dt); hashes.Add(w1.ComputeHash()); }
        var bytes = w1.Snapshot();

        w1.Restore(snap);
        car = w1.GetVehicles()[0];
        for (int i = 0; i < 120; i++) { car.Drive(1f); w1.Step(Dt); Assert.Equal(hashes[i], w1.ComputeHash()); }
        Assert.True(bytes.AsSpan().SequenceEqual(w1.Snapshot()));

        var fresh = NewWorld();
        fresh.Restore(snap);
        var c2 = fresh.GetVehicles()[0];
        for (int i = 0; i < 120; i++) { c2.Drive(1f); fresh.Step(Dt); Assert.Equal(hashes[i], fresh.ComputeHash()); }
    }

    [Fact]
    public void ForkContinuesIdentically()
    {
        DriveHash(1, 100, w => CarOn(w, 0), out var w1);
        var f = w1.Fork();
        var a = w1.GetVehicles()[0]; var b = f.GetVehicles()[0];
        for (int i = 0; i < 100; i++) { a.Drive(1f); b.Drive(1f); w1.Step(Dt); f.Step(Dt); }
        Assert.Equal(w1.ComputeHash(), f.ComputeHash());
    }

    [Fact]
    public void WorldWithoutVehiclesKeepsItsSnapshotFree()
    {
        var w = NewWorld();
        Floor(w);
        w.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 3));
        Run(w, 10);
        var bytes = w.Snapshot();
        var w2 = NewWorld();
        Floor(w2);
        var car = CarOn(w2, 100);
        car.Chassis.Destroy();
        Assert.True(w2.GetVehicles().Length == 0);
        Assert.True(bytes.Length > 0);
    }

    [Fact]
    public void TracksClimbAndMotorcycleRuns()
    {
        var w = NewWorld();
        Floor(w, half: 2000);
        var tank = w.AddTank(new Vector2D(0, 0));
        var bike = w.AddMotorcycle(new Vector2D(30, 0));
        for (int i = 0; i < 600; i++) { tank.Drive(1f); bike.Drive(1f); w.Step(Dt); }
        Assert.True(tank.Chassis.Position.X > 10, $"tank {tank.Chassis.Position.X}");
        Assert.True(bike.Chassis.Position.X > 60, $"bike {bike.Chassis.Position.X}");
    }
}
