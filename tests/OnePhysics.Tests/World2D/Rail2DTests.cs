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

/// <summary>Side-view rail vehicles: grades, adhesion-limited traction and braking, couplers with slack, and the host hooks.</summary>
public class Rail2DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D NewWorld(int id = 141) => new(new WorldOptions2D { WorldId = id });

    [Fact]
    public void TrackProfileFollowsItsPointsWithoutOvershoot()
    {
        var w = NewWorld();
        var def = TrackDef.Straight(300);
        def.Elevation = new[] { new Vector2F(0, 0), new Vector2F(100, 0), new Vector2F(150, 5), new Vector2F(200, 5), new Vector2F(300, 5) };
        var track = w.AddTrack(def);
        Assert.Equal(300.0, track.Length, 1e-6);
        Assert.Equal(0.0, track.SurfaceAt(50, out _).Y, 1e-9);
        Assert.Equal(5.0, track.SurfaceAt(250, out float g).Y, 1e-6);
        Assert.Equal(0f, g, 1e-6f);
        double prev = -1; for (int s = 100; s <= 150; s++) { double y = track.SurfaceAt(s, out _).Y; Assert.True(y >= prev - 1e-9 && y >= 0 && y <= 5.0 + 1e-9); prev = y; }
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddTrack(new TrackDef { Elements = new[] { TrackElement.Arc(100, 200) } }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddTrack(new TrackDef { Elements = new[] { TrackElement.Straight(10), TrackElement.Straight(10) } }));
    }

    [Fact]
    public void LocomotiveClimbsAGradeAtItsAdhesionLimit()
    {
        var w = NewWorld();
        var def = TrackDef.Straight(2000); def.Friction = 0.2f;
        def.Elevation = new[] { new Vector2F(0, 0), new Vector2F(100, 0), new Vector2F(200, 10), new Vector2F(2000, 10) };   // a 10 % grade from 100 m to 200 m, then level
        var track = w.AddTrack(def);
        var loco = w.AddRailVehicle(track, 20, RailDef.Locomotive);
        loco.Drive(throttle: 1f);
        for (int i = 0; i < 60 * 6; i++) { w.Step(Dt); }
        float v1 = loco.Speed;
        for (int i = 0; i < 60 * 2; i++) { w.Step(Dt); }
        double a = (loco.Speed - v1) / 2.0;
        // Full traction is 300 kN (3.8 m/s^2); the wheels slip and the creep curve falls past its peak, so the pull ends between half of mu m g and mu m g (1.96 m/s^2).
        Assert.InRange(a, 0.2 * 9.81 * 0.5, 0.2 * 9.81 * 1.02);
        Assert.True(loco.IsOnTrack);
        var ws = loco.Wheels.GetWheel(0);
        Assert.True(ws.AngularVelocity * 0.46f > 1.1f * loco.Speed, "the driven wheels spin faster than the vehicle moves");
        // Slower, on the grade: the same pull loses g sin(slope).
        for (int i = 0; i < 60 * 40; i++) { w.Step(Dt); }
        Assert.True(loco.Car.Position.X > 150, "it has climbed the grade");
    }

    [Fact]
    public void BrakesStopTheVehicleAndDoNotLiftItOffTheRail()
    {
        var w = NewWorld();
        var track = w.AddTrack(TrackDef.Straight(2000));
        var loco = w.AddRailVehicle(track, 40, RailDef.Locomotive, 15f);
        loco.Drive(brake: 0.5f);
        for (int i = 0; i < 60 * 12; i++) { w.Step(Dt); }
        Assert.True(loco.Speed < 0.3f, $"braking should stop it: {loco.Speed}");
        Assert.True(loco.IsOnTrack);
    }

    private static PhysicsWorld2D Train(int threads, out RailVehicle loco, out RailVehicle[] cars)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 142, ThreadCount = threads });
        var def = TrackDef.Straight(3000);
        def.Elevation = new[] { new Vector2F(0, 0), new Vector2F(400, 0), new Vector2F(500, 2), new Vector2F(3000, 2) };
        var track = w.AddTrack(def);
        loco = w.AddRailVehicle(track, 100, RailDef.Locomotive);
        cars = new RailVehicle[3];
        double s = 100 - 9.25 - 0.6 - 0.02 - 5.25;
        for (int i = 0; i < 3; i++)
        {
            cars[i] = w.AddRailVehicle(track, s, RailDef.Freight);
            s -= 5.25 + 0.6 + 0.02 + 5.25;
        }

        loco.Couple(cars[0], thisFront: false, otherFront: true);
        cars[0].Couple(cars[1], thisFront: false, otherFront: true);
        cars[1].Couple(cars[2], thisFront: false, otherFront: true);
        return w;
    }

    [Fact]
    public void CoupledTrainTakesUpItsSlackWithBoundedForcesAndIsDeterministic()
    {
        var w = Train(1, out var loco, out var cars);
        double peak = 0;
        for (int i = 0; i < 60 * 25; i++)
        {
            loco.Drive(throttle: i < 60 * 15 ? 0.6f : 0f, brake: i >= 60 * 15 ? 0.1f : 0f);
            w.Step(Dt);
            peak = Math.Max(peak, Math.Abs(loco.CouplerForce(false)));
        }

        Assert.True(peak > 1e4, "the draw gear works");
        Assert.True(peak < loco.Definition.CouplerBreakForce, $"the run-in stays inside the coupler rating: {peak}");
        Assert.False(loco.IsCouplerBroken(false));
        Assert.True(Math.Abs(cars[2].Speed - loco.Speed) < 2f, "the train moves as one");
        Assert.True(loco.IsOnTrack && cars[0].IsOnTrack && cars[2].IsOnTrack);
        var w4 = Train(4, out var l4, out _);
        for (int i = 0; i < 60 * 25; i++) { l4.Drive(throttle: i < 60 * 15 ? 0.6f : 0f, brake: i >= 60 * 15 ? 0.1f : 0f); w4.Step(Dt); }
        Assert.Equal(w.ComputeHash(), w4.ComputeHash());
    }

    [Fact]
    public void OverloadedDrawGearBreaks()
    {
        var w = NewWorld();
        var track = w.AddTrack(TrackDef.Straight(2000));
        var weak = RailDef.Locomotive; weak.CouplerBreakForce = 2e4f;
        var loco = w.AddRailVehicle(track, 40, weak);
        var car = w.AddRailVehicle(track, 40 - 9.25 - 0.6 - 0.02 - 5.25, RailDef.Freight);
        loco.Couple(car, thisFront: false, otherFront: true);
        loco.Drive(throttle: 1f);
        for (int i = 0; i < 60 * 8; i++) { w.Step(Dt); }
        Assert.True(loco.IsCouplerBroken(false) && car.IsCouplerBroken(true));
        Assert.True(loco.Speed > car.Speed + 2f);
    }

    [Fact]
    public void StaleHandlesThrowAndDefinitionsAreValidated()
    {
        var w = NewWorld();
        var track = w.AddTrack(TrackDef.Straight(100));
        var veh = w.AddRailVehicle(track, 30, RailDef.Freight);
        Assert.Throws<ArgumentOutOfRangeException>(() => veh.Drive(throttle: float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => veh.Drive(brake: 2f));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddRailVehicle(track, 30, new RailDef { Axles = 1 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddRailVehicle(track, 30, new RailDef { CarMass = float.NaN }));
        var h = veh.RawHandle; var th = track.RawHandle;
        Assert.True(w.IsRailVehicleAlive(h) && w.IsTrackAlive(th));
        veh.Destroy();
        Assert.False(veh.IsValid);
        Assert.Throws<StaleHandleException>(() => _ = veh.Speed);
        Assert.Throws<StaleHandleException>(() => w.GetRailVehicle(h));
        Assert.Throws<InvalidHandleException>(() => _ = default(RailVehicle).Car);
        track.Destroy();
        Assert.Throws<StaleHandleException>(() => w.GetTrack(th));
        Assert.Empty(w.GetRailVehicles());
    }

    private static PhysicsWorld2D Running()
    {
        var w = NewWorld(143);
        var def = TrackDef.Straight(1000);
        def.Elevation = new[] { new Vector2F(0, 0), new Vector2F(100, 0), new Vector2F(150, 4), new Vector2F(1000, 4) };
        var track = w.AddTrack(def);
        var loco = w.AddRailVehicle(track, 40, RailDef.Tram, 8f);
        loco.Drive(throttle: 0.5f);
        var car = w.AddRailVehicle(track, 40 - 6.25 - 0.6 - 0.02 - 5.25, RailDef.Freight, 8f);
        car.Couple(loco, thisFront: true, otherFront: false);
        return w;
    }

    [Fact]
    public void SnapshotForkReplaySceneDesyncRegionShiftAndDebugCoverRail()
    {
        var a = Running();
        for (int i = 0; i < 60; i++) { a.Step(Dt); }
        var snap = a.Snapshot();
        var hashes = new List<ulong>();
        for (int i = 0; i < 60; i++) { a.Step(Dt); hashes.Add(a.ComputeHash()); }
        var b = Running(); b.Restore(snap);
        for (int i = 0; i < 60; i++) { b.Step(Dt); Assert.Equal(hashes[i], b.ComputeHash()); }
        Assert.Equal(a.Snapshot(), b.Snapshot());
        var plain = NewWorld(143);
        plain.AddCircle(0.5f, new Vector2D(0, 3));
        b.Restore(plain.Snapshot());
        Assert.Empty(b.GetRailVehicles()); Assert.Empty(b.GetTracks());

        var f = a.Fork();
        Assert.Equal(a.ComputeHash(), f.ComputeHash());
        Assert.Equal(0, a.FindDifferences(f).Count);
        f.GetRailVehicles()[0].Drive(throttle: 1f);
        f.Step(Dt); a.Step(Dt);
        Assert.True(a.FindDifferences(f).Count > 0);

        var rec = Running();
        using var stream = new MemoryStream();
        var expected = new List<ulong>();
        using (var recorder = rec.StartRecording(stream))
        {
            for (int i = 0; i < 60; i++) { if (i == 20) { rec.GetRailVehicles()[0].Drive(throttle: -0.2f, brake: 0.1f); } rec.Step(Dt); expected.Add(rec.ComputeHash()); }
        }

        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var copy = Running();
        int step = 0;
        while (player.ReadNextStep(copy)) { Assert.Equal(expected[step++], copy.ComputeHash()); }
        Assert.Equal(60, step);
        var loaded = PhysicsWorld2D.LoadScene(new StringReader(rec.SaveScene()));
        Assert.Equal(rec.ComputeHash(), loaded.ComputeHash());
        for (int i = 0; i < 30; i++) { rec.Step(Dt); loaded.Step(Dt); }
        Assert.Equal(rec.ComputeHash(), loaded.ComputeHash());

        var p = rec.GetRailVehicles()[0].Car.Position;
        var min = new Vector2D(p.X - 1, p.Y - 1); var max = new Vector2D(p.X + 1, p.Y + 1);
        ulong h = rec.ComputeRegionHash(min, max);
        rec.GetRailVehicles()[0].Drive(throttle: 1f); rec.Step(Dt);
        Assert.NotEqual(h, rec.ComputeRegionHash(min, max));
        double x = rec.GetRailVehicles()[0].Car.Position.X;
        double trackX = rec.GetTracks()[0].SurfaceAt(0, out _).X;
        rec.ShiftOrigin(new Vector2D(16, 0));
        Assert.Equal(x - 16, rec.GetRailVehicles()[0].Car.Position.X, 1e-9);
        Assert.Equal(trackX - 16, rec.GetTracks()[0].SurfaceAt(0, out _).X, 1e-9);
        for (int i = 0; i < 30; i++) { rec.Step(Dt); }
        Assert.True(rec.GetRailVehicles()[0].IsOnTrack);
        var g = new RecursiveAI.OnePhysics.Debug.DebugGeometry();
        rec.GetDebugGeometry(RecursiveAI.OnePhysics.Debug.DebugContent.Vehicles, g);
        Assert.Equal(1, g.Tracks.Length);
        Assert.Equal(6, g.Wheels.Length);
    }
}
