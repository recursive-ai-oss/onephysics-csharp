#pragma warning disable OPHYS0003
using System;
using System.Collections.Generic;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Replay;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

/// <summary>Vehicles and ragdolls through the host hooks: recording and replay, scene files, desync labels, regions, snapshot bounds, origin shift and the debug geometry.</summary>
public class Vehicles2DHostTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D World(int id = 95)
    {
        var o = new WorldOptions2D { WorldId = id };
        Ragdoll.ApplyRecommendedOptions(o);
        var w = new PhysicsWorld2D(o);
        w.AddBox(new Vector2F(600f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static, material: new Material2D(0.8f, 0f, 0f));
        w.AddCar(new Vector2D(0, 0));
        w.AddTank(new Vector2D(30, 0));
        w.AddMotorcycle(new Vector2D(60, 0));
        w.AddRagdoll(RagdollDef.Humanoid, new Vector2D(90, 0.5));
        return w;
    }

    private static void Drive(PhysicsWorld2D w) { foreach (var v in w.GetVehicles()) { v.Drive(0.6f, 0.1f); } }

    [Fact]
    public void RecordedVehiclesAndRagdollsReplayWithEveryHashEqual()
    {
        var w = World();
        using var stream = new MemoryStream();
        var expected = new List<ulong>();
        using (var recorder = w.StartRecording(stream))
        {
            for (int i = 0; i < 90; i++)
            {
                Drive(w);
                if (i == 20) { w.GetRagdolls()[0].DriveToPose(new float[13], 0.7f); }
                if (i == 40) { w.GetVehicles()[0].Gear = 2; }
                w.Step(Dt);
                expected.Add(w.ComputeHash());
            }
        }

        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var copy = World();
        int step = 0;
        while (player.ReadNextStep(copy)) { Assert.Equal(expected[step++], copy.ComputeHash()); }
        Assert.Equal(90, step);
        Assert.Equal(w.Snapshot(), copy.Snapshot());
    }

    [Fact]
    public void SceneFileKeepsVehiclesAndRagdolls()
    {
        var w = World();
        for (int i = 0; i < 40; i++) { Drive(w); w.Step(Dt); }
        string text = w.SaveScene();
        var loaded = PhysicsWorld2D.LoadScene(new StringReader(text));
        Assert.Equal(w.ComputeHash(), loaded.ComputeHash());
        Assert.Equal(3, loaded.GetVehicles().Length);
        Assert.Single(loaded.GetRagdolls());
        for (int i = 0; i < 40; i++) { Drive(w); Drive(loaded); w.Step(Dt); loaded.Step(Dt); }
        Assert.Equal(w.ComputeHash(), loaded.ComputeHash());
    }

    [Fact]
    public void DesyncFinderNamesTheVehicleField()
    {
        var a = World();
        for (int i = 0; i < 20; i++) { Drive(a); a.Step(Dt); }
        var b = a.Fork();
        Assert.Equal(0, a.FindDifferences(b).Count);
        b.GetVehicles()[0].Drive(1f, 0.9f);
        b.Step(Dt); a.Step(Dt);
        var report = a.FindDifferences(b);
        Assert.True(report.Count > 0);
        Assert.Contains(report.Differences, d => d.Section.Length > 0);
        a.GetVehicles()[0].Drive(0.6f, 0.1f);
        var c = a.Fork();
        c.GetRagdolls()[0].DriveToPose(new float[13], 1f);
        Assert.True(a.FindDifferences(c).Count > 0);
    }

    [Fact]
    public void RegionCoversWholeVehicleAndRagdoll()
    {
        var w = World();
        for (int i = 0; i < 30; i++) { Drive(w); w.Step(Dt); }
        // A box around the car's chassis alone: the car (chassis and its ground body when dynamic) is selected as one unit; the tank is not.
        var car = w.GetVehicles()[0].Chassis.Position;
        var min = new Vector2D(car.X - 1, car.Y - 1); var max = new Vector2D(car.X + 1, car.Y + 1);
        ulong h = w.ComputeRegionHash(min, max);
        var tank = w.GetVehicles()[1];
        tank.Drive(-1f);
        Assert.Equal(h, w.ComputeRegionHash(min, max));
        w.GetVehicles()[0].Drive(-1f, 0.5f);
        Assert.NotEqual(h, w.ComputeRegionHash(min, max));
        // The ragdoll: one part inside the box selects every part.
        var rp = w.GetRagdolls()[0].GetBody(5).Position;
        var rmin = new Vector2D(rp.X - 0.05, rp.Y - 0.05); var rmax = new Vector2D(rp.X + 0.05, rp.Y + 0.05);
        var archive = w.CaptureRegion(rmin, rmax);
        Assert.True(archive.Fields.Count > 13);
        ulong hr = w.ComputeRegionHash(rmin, rmax);
        w.GetRagdolls()[0].DriveToPose(new float[13], 1f);
        w.Step(Dt);
        Assert.NotEqual(hr, w.ComputeRegionHash(rmin, rmax));
    }

    [Fact]
    public void SnapshotBoundCoversVehiclesAndRagdolls()
    {
        var w = World();
        int before = w.GetSnapshotSizeBound();
        for (int i = 0; i < 20; i++) { Drive(w); w.Step(Dt); }
        Assert.True(w.Snapshot().Length <= w.GetSnapshotSizeBound());
        Assert.True(w.GetSnapshotSizeBound() >= before);
        var plain = new PhysicsWorld2D();
        plain.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 3));
        Assert.True(plain.Snapshot().Length <= plain.GetSnapshotSizeBound());
    }

    [Fact]
    public void OriginShiftCarriesTheVehiclesAlong()
    {
        var w = World();
        for (int i = 0; i < 60; i++) { Drive(w); w.Step(Dt); }
        var car = w.GetVehicles()[0];
        double x = car.Chassis.Position.X;
        float speed = car.ForwardSpeed;
        w.ShiftOrigin(new Vector2D(16, 0));
        Assert.Equal(x - 16, car.Chassis.Position.X, 1e-9);
        Assert.True(car.GetWheel(0).IsInContact);
        Assert.InRange(car.GetWheel(0).Center.X - car.Chassis.Position.X, 0.8, 1.8);   // the wheel stays attached to the chassis
        for (int i = 0; i < 60; i++) { Drive(w); w.Step(Dt); }
        Assert.True(car.ForwardSpeed > speed * 0.9f);
        Assert.True(car.GetWheel(0).IsInContact);
    }

    [Fact]
    public void DebugGeometryHasOneRecordPerWheel()
    {
        var w = World();
        for (int i = 0; i < 10; i++) { w.Step(Dt); }
        var g = new RecursiveAI.OnePhysics.Debug.DebugGeometry();
        w.GetDebugGeometry(RecursiveAI.OnePhysics.Debug.DebugContent.Vehicles, g);
        int wheels = 0;
        foreach (var v in w.GetVehicles()) { wheels += v.WheelCount; }
        Assert.Equal(wheels, g.Wheels.Length);
        Assert.Equal(3, w.Counters.Vehicles);
    }

    [Fact]
    public void TrackedVehicleReachesItsCommandedSpeedAndStops()
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 96 });
        w.AddBox(new Vector2F(800f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static, material: new Material2D(0.9f, 0f, 0f));
        var tank = w.AddTank(new Vector2D(-300, 0));
        for (int i = 0; i < 600; i++) { tank.Drive(0.5f); w.Step(Dt); }
        Assert.InRange(tank.TrackSpeed, 0.5f * 12f * 0.9f, 0.5f * 12f * 1.1f);
        for (int i = 0; i < 300; i++) { tank.Drive(0f, 0f, 1f); w.Step(Dt); }
        Assert.True(Math.Abs(tank.Chassis.LinearVelocity.X) < 0.05f);
    }

    [Fact]
    public void MotorcyclePitchInputRotatesItInTheAir()
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 97 });
        var bike = w.AddMotorcycle(new Vector2D(0, 30));   // dropped from 30 m: in the air for a while
        var chassis = bike.Chassis;
        for (int i = 0; i < 40; i++) { bike.Drive(0f, 1f); w.Step(Dt); }
        Assert.True(chassis.AngularVelocity > 0.2f, $"nose-up input should spin the bike counter-clockwise: {chassis.AngularVelocity}");
        var w2 = new PhysicsWorld2D(new WorldOptions2D { WorldId = 97 });
        var bike2 = w2.AddMotorcycle(new Vector2D(0, 30));
        var chassis2 = bike2.Chassis;
        for (int i = 0; i < 40; i++) { bike2.Drive(0f, -1f); w2.Step(Dt); }
        Assert.True(chassis2.AngularVelocity < -0.2f);
    }
}
