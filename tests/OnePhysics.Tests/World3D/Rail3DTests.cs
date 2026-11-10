using System;
using System.Collections.Generic;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Replay;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

/// <summary>Rail vehicles in 3D: wheel-rail contact against Klingel's hunting wavelength, curving at the balancing speed, adhesion limits, couplers, switches and the host hooks.</summary>
public class Rail3DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld3D NewWorld(int threads = 1, int id = 131) => new(new WorldOptions3D { WorldId = id, ThreadCount = threads });

    private static double Lateral(Track track, WheelsetState ws)
    {
        var f = track.FrameAt(ws.Chainage);
        var p = ws.Body.Position;
        return (p.X - f.Position.X) * f.Right.X + (p.Y - f.Position.Y) * f.Right.Y + (p.Z - f.Position.Z) * f.Right.Z;
    }

    [Fact]
    public void TrackGeometryIsExactForStraightsArcsAndClothoids()
    {
        var w = NewWorld();
        var track = w.AddTrack(TrackDef.Curve(300, 200, 60, 0.12f, 100));
        Assert.Equal(520.0, track.Length, 1e-4);
        // A quarter of the way through the arc the heading has turned by the transition's half-integral plus arc/radius.
        double headingAtArc = 0.5 * 60 / 300 + 100 / 300.0;
        var f = track.FrameAt(160 + 100);
        Assert.Equal(Math.Sin(headingAtArc), f.Tangent.X, 1e-4);
        Assert.Equal(Math.Cos(headingAtArc), f.Tangent.Z, 1e-4);
        Assert.Equal(1.0 / 300, f.Curvature, 1e-6);
        Assert.Equal(Math.Asin(0.12 / 1.435), f.CantAngle, 1e-4);
        // The arc is circular: the distance from the centre of curvature stays the radius.
        var a = track.FrameAt(200); var b = track.FrameAt(300);
        double pa = Math.Atan2(a.Tangent.X, a.Tangent.Z), pb = Math.Atan2(b.Tangent.X, b.Tangent.Z);
        double cxa = a.Position.X + 300 * Math.Cos(pa), cza = a.Position.Z - 300 * Math.Sin(pa);
        double cxb = b.Position.X + 300 * Math.Cos(pb), czb = b.Position.Z - 300 * Math.Sin(pb);
        Assert.Equal(cxa, cxb, 0.01); Assert.Equal(cza, czb, 0.01);
        // Elevation and cant profiles.
        var up = TrackDef.Straight(200); up.Elevation = new[] { new Vector2F(0, 0), new Vector2F(100, 0), new Vector2F(150, 5), new Vector2F(200, 5) };
        var t2 = w.AddTrack(up);
        Assert.Equal(0.0, t2.FrameAt(50).Position.Y, 1e-9);
        Assert.Equal(5.0, t2.FrameAt(180).Position.Y, 1e-6);
        Assert.InRange(t2.FrameAt(125).Position.Y, 2.4, 2.6);
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddTrack(new TrackDef { Elements = new[] { TrackElement.Straight(float.NaN) } }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddTrack(new TrackDef { Elements = new[] { TrackElement.Arc(10, 3) } }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddTrack(new TrackDef { Start = new Vector3D(0.1, 0, 0) }));
    }

    [Fact]
    public void FreeWheelsetHuntsAtKlingelsWavelength()
    {
        // Kinematic hunting of a conical wheelset: wavelength 2 pi sqrt(r0 b / gamma) (Klingel 1883), r0 the wheel radius, b half the tread span, gamma the conicity.
        var w = NewWorld();
        var track = w.AddTrack(TrackDef.Straight(2000));
        var rd = RailDef.Wheelset;
        float v = 10f;
        var veh = w.AddRailVehicle(track, 10, rd, v);
        var body = veh.Car;
        body.SetPosition(new Vector3D(0.003, body.Position.Y, body.Position.Z));
        var ts = new List<double>(); var ys = new List<double>();
        for (int i = 0; i < 360; i++) { w.Step(Dt); ys.Add(Lateral(track, veh.GetWheelset(0))); ts.Add((i + 1) * Dt); }
        var zc = new List<double>();
        for (int i = 1; i < ys.Count; i++) { if (ys[i - 1] * ys[i] < 0) { zc.Add(ts[i - 1] + Dt * ys[i - 1] / (ys[i - 1] - ys[i])); } }
        Assert.True(zc.Count >= 6, $"the wheelset should oscillate: {zc.Count} zero crossings");
        double period = 2 * (zc[^1] - zc[1]) / (zc.Count - 2);
        double klingel = 2 * Math.PI * Math.Sqrt(rd.WheelRadius * (0.5 * rd.TreadSpan) / rd.Conicity);
        Assert.InRange(period * v, klingel * 0.9, klingel * 1.1);
    }

    private static double LoadTransfer(RailVehicle v)
    {
        double num = 0, den = 0;
        for (int k = 0; k < v.WheelsetCount; k++) { var s = v.GetWheelset(k); num += s.VerticalLoadLeft - s.VerticalLoadRight; den += s.VerticalLoadLeft + s.VerticalLoadRight; }
        return num / den;
    }

    private static double SteadyArcLoadTransfer(double speed, out bool onTrack)
    {
        var w = NewWorld();
        var track = w.AddTrack(TrackDef.Curve(300, 400, 60, 0.12f, 100));
        var veh = w.AddRailVehicle(track, 40, RailDef.Locomotive, (float)speed);
        double sum = 0; int n = 0;
        for (int i = 0; i < 60 * 28; i++)
        {
            w.Step(Dt);
            double s0 = veh.GetWheelset(0).Chainage, s3 = veh.GetWheelset(3).Chainage;
            if (s0 > 220 && s3 < 540) { sum += LoadTransfer(veh); n++; }
        }

        onTrack = veh.IsOnTrack;
        Assert.True(n > 300);
        return sum / n;
    }

    [Fact]
    public void CantedCurveAtTheBalancingSpeedLoadsBothRailsEqually()
    {
        // The balancing speed of a banked curve: v^2 / R = g tan(phi). There the lateral force vanishes and so does the load transfer; faster the outer rail takes more, slower the inner.
        double phi = Math.Asin(0.12 / 1.435);
        double vb = Math.Sqrt(9.81 * 300 * Math.Tan(phi));
        double slow = SteadyArcLoadTransfer(0.7 * vb, out bool onSlow);
        double bal = SteadyArcLoadTransfer(vb, out bool onBal);
        double fast = SteadyArcLoadTransfer(1.3 * vb, out bool onFast);
        Assert.True(onSlow && onBal && onFast);
        Assert.InRange(Math.Abs(bal), 0, 0.03);
        // Quasi-static load transfer (a_lateral / g) (h / b) with the lateral acceleration along the banked track, h = 1.9 m the centre-of-mass height, b = 0.7525 m half the rail spacing.
        double Predicted(double v) => (v * v / 300 * Math.Cos(phi) - 9.81 * Math.Sin(phi)) / 9.81 * 1.9 / 0.7525;
        Assert.InRange(slow, Predicted(0.7 * vb) * 1.35, Predicted(0.7 * vb) * 0.65);
        Assert.InRange(fast, Predicted(1.3 * vb) * 0.65, Predicted(1.3 * vb) * 1.35);
    }

    [Fact]
    public void LocomotiveUnloadsItsInnerWheelsNearTheQuasiStaticOverturningSpeed()
    {
        // On a flat curve of radius R the rigid-body estimate of the speed at which the inner wheels unload is sqrt(g R b / h), b half the rail spacing, h the height of the centre of mass.
        double predicted = Math.Sqrt(9.81 * 150 * 0.7525 / 1.9);
        double firstUnloaded = double.NaN;
        for (double v = 12; v <= 28 && double.IsNaN(firstUnloaded); v += 2)
        {
            var w = NewWorld();
            var track = w.AddTrack(TrackDef.Curve(150, 300, 40, 0f, 100));
            var veh = w.AddRailVehicle(track, 40, RailDef.Locomotive, (float)v);
            double inner = 0; int n = 0; bool derailed = false;
            double staticLoad = 0.5 * 78000 * 9.81 / 4;
            for (int i = 0; i < 60 * 14; i++)
            {
                w.Step(Dt);
                double s0 = veh.GetWheelset(0).Chainage, s3 = veh.GetWheelset(3).Chainage;
                if (s0 > 190 && s3 < 400) { for (int k = 0; k < 4; k++) { inner += veh.GetWheelset(k).VerticalLoadRight; n++; } }
                if (i > 120 && !veh.IsOnTrack) { derailed = true; }
            }

            if (derailed || (n > 0 && inner / n < 0.25 * staticLoad)) { firstUnloaded = v; }
        }

        Assert.False(double.IsNaN(firstUnloaded), "the inner wheels never unloaded");
        Assert.InRange(firstUnloaded, 0.65 * predicted, 1.1 * predicted);
    }

    [Fact]
    public void TractionIsLimitedByAdhesion()
    {
        var w = NewWorld();
        var td = TrackDef.Straight(2000); td.Friction = 0.15f;
        var track = w.AddTrack(td);
        var veh = w.AddRailVehicle(track, 20, RailDef.Locomotive);
        veh.Drive(throttle: 1f);
        for (int i = 0; i < 60 * 4; i++) { w.Step(Dt); }
        float v1 = veh.Speed;
        for (int i = 0; i < 60 * 2; i++) { w.Step(Dt); }
        double a = (veh.Speed - v1) / 2.0;
        // 300 kN of traction would give 3.8 m/s^2; the wheels slip at the friction limit: mu g.
        Assert.InRange(a, 0.15 * 9.81 * 0.9, 0.15 * 9.81 * 1.1);
        var ws = veh.GetWheelset(0);
        Assert.True(ws.SpinSpeed * ws.Radius > 1.2f * veh.Speed, "the driven wheels spin faster than the vehicle moves");
        // With dry rails the same locomotive takes up its full tractive effort.
        var w2 = NewWorld(1, 132);
        var d2 = TrackDef.Straight(2000); d2.Friction = 0.5f;
        var t2 = w2.AddTrack(d2);
        var v2 = w2.AddRailVehicle(t2, 20, RailDef.Locomotive);
        v2.Drive(throttle: 1f);
        for (int i = 0; i < 60 * 3; i++) { w2.Step(Dt); }
        Assert.True(v2.Speed > 1.5f * veh.Speed * 3f / 6f * 2f / 1.5f * 0.9f && v2.Speed > 0.15f * 9.81f * 3f * 1.5f);
    }

    [Fact]
    public void BrakesStopTheVehicleWithinTheAdhesionLimit()
    {
        var w = NewWorld();
        var track = w.AddTrack(TrackDef.Straight(2000));
        var veh = w.AddRailVehicle(track, 40, RailDef.Locomotive, 15f);
        veh.Drive(brake: 0.5f);   // 125 kN on 78 t: 1.6 m/s^2, inside the adhesion limit
        for (int i = 0; i < 60 * 12; i++) { w.Step(Dt); }
        Assert.True(veh.Speed < 0.3f, $"braking should stop it: {veh.Speed}");
        Assert.True(veh.IsOnTrack);
    }

    private static PhysicsWorld3D Train(int threads, out RailVehicle loco, out RailVehicle[] cars)
    {
        var w = NewWorld(threads, 133);
        var track = w.AddTrack(TrackDef.Curve(400, 200, 60, 0.08f, 150));
        loco = w.AddRailVehicle(track, 60, RailDef.Locomotive);
        cars = new RailVehicle[3];
        // Each car's centre sits one coupler reach, the free length and a little slack behind the coupler of the one ahead.
        double s = 60 - 9.25 - 0.6 - 0.02 - 5.25;
        for (int i = 0; i < 3; i++)
        {
            cars[i] = w.AddRailVehicle(track, s, RailDef.Freight);
            s -= 5.25 + 0.6 + 0.02 + 5.25;
        }

        // The couplers: the locomotive's rear end to the first car's front, and so on.
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
        Assert.True(loco.IsOnTrack && cars[0].IsOnTrack && cars[1].IsOnTrack && cars[2].IsOnTrack);
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
    public void SwitchesSendWheelsetsDownTheBranchOnlyWhenSetDiverging()
    {
        double Run(bool diverging, out bool onBranch)
        {
            var w = NewWorld();
            var main = w.AddTrack(TrackDef.Straight(400));
            var branch = new TrackDef { Elements = new[] { TrackElement.Transition(40, 0f, 1f / 300), TrackElement.Arc(100, 300), TrackElement.Straight(200) } };
            var sw = main.AddSwitch(100, branch, diverging);
            var veh = w.AddRailVehicle(main, 40, RailDef.Tram, 12f);
            for (int i = 0; i < 60 * 12; i++) { w.Step(Dt); }
            var ws = veh.GetWheelset(0);
            onBranch = ws.Track == sw.Branch;
            Assert.True(veh.IsOnTrack);
            return veh.Car.Position.X;
        }

        double straight = Run(false, out bool onMain);
        double turned = Run(true, out bool onBranch);
        Assert.False(onMain);
        Assert.True(onBranch);
        Assert.True(Math.Abs(straight) < 0.05);
        Assert.True(turned > 5, $"the diverging route curves away to the right: {turned}");
    }

    [Fact]
    public void TrackCollisionLetsOtherBodiesRestOnTheRailsAndWheelsetsPassThrough()
    {
        var w = NewWorld();
        var track = w.AddTrack(TrackDef.Straight(100));
        var box = w.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), new Vector3D(0, 0.6, 50));
        var sleeper = w.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), new Vector3D(0.75, 0.6, 60));
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.InRange(box.Position.Y, 0.015, 0.05);   // on the ballast between the rails
        Assert.InRange(sleeper.Position.Y, 0.2 - 0.01, 0.2 + 0.05);                 // on the rail head
        var veh = w.AddRailVehicle(track, 20, RailDef.Wheelset, 3f);
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.True(veh.IsOnTrack);
        Assert.InRange(veh.Car.Position.Y, 0.44, 0.47);
    }

    [Fact]
    public void NadalLimitMatchesTheClosedForm()
    {
        float a = 1.2217305f;
        float expected = (MathF.Tan(a) - 0.3f) / (1f + 0.3f * MathF.Tan(a));
        Assert.Equal(expected, PhysicsWorld3D.NadalLimit(a, 0.3f), 3);
        Assert.InRange(PhysicsWorld3D.NadalLimit(a, 0.3f), 1.3f, 1.4f);
    }

    [Fact]
    public void StaleHandlesThrowAndDefinitionsAreValidated()
    {
        var w = NewWorld();
        var track = w.AddTrack(TrackDef.Straight(100));
        var veh = w.AddRailVehicle(track, 30, RailDef.Freight);
        Assert.Throws<ArgumentOutOfRangeException>(() => veh.Drive(throttle: float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => veh.Drive(brake: 2f));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddRailVehicle(track, 30, new RailDef { Bogies = 1 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddRailVehicle(track, 30, new RailDef { CarMass = float.NaN }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddRailVehicle(track, 30, new RailDef { CarMass = 0f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddRailVehicle(track, double.NaN, RailDef.Freight));
        var h = veh.RawHandle; var th = track.RawHandle;
        Assert.True(w.IsRailVehicleAlive(h) && w.IsTrackAlive(th));
        veh.Destroy();
        Assert.False(veh.IsValid);
        Assert.Throws<StaleHandleException>(() => _ = veh.Speed);
        Assert.Throws<StaleHandleException>(() => w.GetRailVehicle(h));
        Assert.Throws<InvalidHandleException>(() => _ = default(RailVehicle).Car);
        track.Destroy();
        Assert.Throws<StaleHandleException>(() => _ = track.Length);
        Assert.Throws<StaleHandleException>(() => w.GetTrack(th));
        Assert.Empty(w.GetRailVehicles());
    }

    private static PhysicsWorld3D Running(int threads = 1)
    {
        var w = NewWorld(threads, 134);
        var main = w.AddTrack(TrackDef.Curve(300, 200, 60, 0.1f, 80));
        main.AddSwitch(40, new TrackDef { Elements = new[] { TrackElement.Arc(80, -400), TrackElement.Straight(100) } }, true);
        var loco = w.AddRailVehicle(main, 30, RailDef.Tram, 8f);
        loco.Drive(throttle: 0.5f);
        var car = w.AddRailVehicle(main, 12, RailDef.Freight, 8f);
        car.Couple(loco, thisFront: true, otherFront: false);
        return w;
    }

    [Fact]
    public void SnapshotRestoreForkReplaySceneAndDesyncFinderCoverRail()
    {
        var a = Running();
        for (int i = 0; i < 60; i++) { a.Step(Dt); }
        var snap = a.Snapshot();
        var hashes = new List<ulong>();
        for (int i = 0; i < 60; i++) { a.Step(Dt); hashes.Add(a.ComputeHash()); }
        var bytes = a.Snapshot();
        var b = Running(); b.Restore(snap);
        for (int i = 0; i < 60; i++) { b.Step(Dt); Assert.Equal(hashes[i], b.ComputeHash()); }
        Assert.Equal(bytes, b.Snapshot());
        // A restore into a world without rail drops it, and a snapshot without the section leaves none.
        var empty = NewWorld(1, 134);
        empty.Restore(snap);
        Assert.Equal(2, empty.GetRailVehicles().Length);
        var plain = NewWorld(1, 134);
        plain.AddSphere(0.5f, new Vector3D(0, 3, 0));
        empty.Restore(plain.Snapshot());
        Assert.Empty(empty.GetRailVehicles());
        Assert.Empty(empty.GetTracks());

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
        var loaded = PhysicsWorld3D.LoadScene(new StringReader(rec.SaveScene()));
        Assert.Equal(rec.ComputeHash(), loaded.ComputeHash());
        for (int i = 0; i < 30; i++) { rec.Step(Dt); loaded.Step(Dt); }
        Assert.Equal(rec.ComputeHash(), loaded.ComputeHash());
    }

    [Fact]
    public void OriginShiftMovesTheTracksExactlyAndRunsStayBitIdentical()
    {
        var a = Running(); var b = Running();
        for (int i = 0; i < 30; i++) { a.Step(Dt); b.Step(Dt); }
        var offset = new Vector3D(0, 0, 4096);
        b.ShiftOrigin(offset);
        for (int i = 0; i < 90; i++) { a.Step(Dt); b.Step(Dt); }
        var va = a.GetRailVehicles(); var vb = b.GetRailVehicles();
        for (int i = 0; i < va.Length; i++)
        {
            Assert.Equal(va[i].Car.Position.Z - 4096, vb[i].Car.Position.Z);
            for (int k = 0; k < va[i].WheelsetCount; k++) { Assert.Equal(va[i].GetWheelset(k).Body.Position.X, vb[i].GetWheelset(k).Body.Position.X); Assert.Equal(va[i].GetWheelset(k).Body.Position.Z - 4096, vb[i].GetWheelset(k).Body.Position.Z); }
        }

        Assert.Equal(a.GetTracks()[0].FrameAt(100).Position.Z - 4096, b.GetTracks()[0].FrameAt(100).Position.Z);
    }

    [Fact]
    public void RegionKeepsAVehicleTogetherAndDebugGeometryShowsTrackAndContacts()
    {
        var w = Running();
        for (int i = 0; i < 40; i++) { w.Step(Dt); }
        var car = w.GetRailVehicles()[0].Car.Position;
        var min = new Vector3D(car.X - 0.5, car.Y - 0.5, car.Z - 0.5); var max = new Vector3D(car.X + 0.5, car.Y + 0.5, car.Z + 0.5);
        ulong h = w.ComputeRegionHash(min, max);
        w.GetRailVehicles()[0].Drive(throttle: -1f);
        w.Step(Dt);
        Assert.NotEqual(h, w.ComputeRegionHash(new Vector3D(min.X - 1, min.Y - 1, min.Z), max));
        var g = new RecursiveAI.OnePhysics.Debug.DebugGeometry();
        w.GetDebugGeometry(RecursiveAI.OnePhysics.Debug.DebugContent.Vehicles, g);
        Assert.Equal(2, g.Tracks.Length);
        Assert.True(g.RailContacts.Length >= 8);
        Assert.True(g.Tracks[0].Centerline.Length > 10);
    }
}
