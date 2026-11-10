using System;
using System.Collections.Generic;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Replay;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

/// <summary>The side-view bicycle: weight transfer under braking (the onset of a stoppie against the static load-transfer prediction), wheelies, and the host hooks.</summary>
public class Bicycle2DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D NewWorld(int id = 121, float friction = 3f)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = id });
        w.AddBox(new Vector2F(1000, 1), at: new Vector2D(0, -0.5), type: BodyType.Static, material: new Material2D(friction, 0f, 0f));
        return w;
    }

    [Fact]
    public void StoppieStartsAtTheStaticLoadTransferDeceleration()
    {
        // The rear wheel unloads when the deceleration reaches g b / h: b the distance from the centre of mass to the front contact point, h its height (the front brake limited by grip, not by the lever).
        var def = BicycleDef.Benchmark;
        def.TyreFriction = 3f; def.MaxFrontBrakeTorque = 2000f; def.TyreHertz = 25f;
        var w = NewWorld();
        var bike = w.AddBicycle(new Vector2D(0, 0), def, 20f);
        var speeds = new List<float>();
        double onset = double.NaN;
        for (int i = 0; i < 600 && double.IsNaN(onset); i++)
        {
            bike.Ride(frontBrake: Math.Min(1f, 0.04f * i * Dt));   // a slow ramp keeps the dynamics quasi-static
            w.Step(Dt);
            speeds.Add(bike.Speed);
            if (i > 10 && bike.Wheels.GetWheel(1).SuspensionForce < 1f)
            {
                double a = 0;
                for (int k = i - 5; k <= i; k++) { a += (speeds[k - 1] - speeds[k]) / Dt; }
                onset = a / 6;
            }
        }

        double predicted = 9.81 * (def.WheelBase - def.CenterX) / def.CenterHeight;
        Assert.False(double.IsNaN(onset), "the rear wheel never lifted");
        Assert.InRange(onset, predicted * 0.95, predicted * 1.05);
    }

    [Fact]
    public void RearBrakeAloneNeverLiftsTheFrontAndPedallingTransfersWeightBack()
    {
        var w = NewWorld(122, 0.9f);
        var bike = w.AddBicycle(new Vector2D(0, 0), BicycleDef.Road, 10f);
        for (int i = 0; i < 120; i++) { bike.Ride(rearBrake: 1f); w.Step(Dt); }
        Assert.False(bike.FrontWheelLifted);
        Assert.True(bike.Speed < 10f);
        var w2 = NewWorld(123, 0.9f);
        var b2 = w2.AddBicycle(new Vector2D(0, 0), BicycleDef.Road, 2f);
        for (int i = 0; i < 60; i++) { b2.Ride(pedal: 1f); w2.Step(Dt); }
        var rear = b2.Wheels.GetWheel(1).SuspensionForce; var front = b2.Wheels.GetWheel(0).SuspensionForce;
        Assert.True(rear > front, $"driving loads the rear wheel more: rear {rear}, front {front}");
        Assert.True(b2.Speed > 2f);
    }

    [Fact]
    public void RiderLeanCanLiftTheFrontWheelAndSteadyRollingStaysLevel()
    {
        var w = NewWorld(124, 0.9f);
        var bike = w.AddBicycle(new Vector2D(0, 0), BicycleDef.Road, 5f);
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.True(Math.Abs(bike.PitchAngle) < 0.01f);
        for (int i = 0; i < 40; i++) { bike.Ride(lean: 1f); w.Step(Dt); }
        Assert.True(bike.PitchAngle > 0.0001f, $"leaning back pitches the nose up: {bike.PitchAngle}");
        var w2 = NewWorld(125, 0.9f);
        var b2 = w2.AddBicycle(new Vector2D(0, 0), BicycleDef.Road, 5f);
        for (int i = 0; i < 160; i++) { b2.Ride(lean: -1f); w2.Step(Dt); }
        Assert.True(b2.PitchAngle < -0.0001f);
    }

    [Fact]
    public void StaleBicycleThrowsAndInputsAreValidated()
    {
        var w = NewWorld();
        var bike = w.AddBicycle(new Vector2D(0, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => bike.Ride(pedal: float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => bike.Ride(rearBrake: -1f));
        var h = bike.RawHandle;
        Assert.True(w.IsBicycleAlive(h));
        bike.Frame.Destroy();
        Assert.False(bike.IsValid);
        Assert.Throws<StaleHandleException>(() => _ = bike.Speed);
        Assert.Throws<StaleHandleException>(() => w.GetBicycle(h));
        Assert.Throws<InvalidHandleException>(() => _ = default(Bicycle).Frame);
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddBicycle(new Vector2D(0, 0), new BicycleDef { Mass = -1f }));
        Assert.Empty(w.GetBicycles());
    }

    private static PhysicsWorld2D Riding(int id = 125)
    {
        var w = NewWorld(id, 0.9f);
        w.AddBicycle(new Vector2D(0, 0), BicycleDef.Road, 6f).Ride(pedal: 0.5f);
        w.AddBicycle(new Vector2D(40, 0), BicycleDef.Benchmark, 8f).Ride(frontBrake: 0.2f, lean: 0.1f);
        return w;
    }

    [Fact]
    public void SnapshotForkReplaySceneDesyncRegionShiftAndDebugCoverTheBicycle()
    {
        var a = Riding();
        for (int i = 0; i < 30; i++) { a.Step(Dt); }
        var snap = a.Snapshot();
        var hashes = new List<ulong>();
        for (int i = 0; i < 40; i++) { a.Step(Dt); hashes.Add(a.ComputeHash()); }
        var b = Riding(); b.Restore(snap);
        for (int i = 0; i < 40; i++) { b.Step(Dt); Assert.Equal(hashes[i], b.ComputeHash()); }
        Assert.Equal(a.Snapshot(), b.Snapshot());

        var fork = a.Fork();
        Assert.Equal(0, a.FindDifferences(fork).Count);
        fork.GetBicycles()[0].Ride(rearBrake: 1f);
        fork.Step(Dt); a.Step(Dt);
        Assert.True(a.FindDifferences(fork).Count > 0);

        var rec = Riding();
        using var stream = new MemoryStream();
        var expected = new List<ulong>();
        using (var recorder = rec.StartRecording(stream))
        {
            for (int i = 0; i < 40; i++) { if (i == 10) { rec.GetBicycles()[1].Ride(rearBrake: 0.6f); } rec.Step(Dt); expected.Add(rec.ComputeHash()); }
        }

        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var copy = Riding();
        int step = 0;
        while (player.ReadNextStep(copy)) { Assert.Equal(expected[step++], copy.ComputeHash()); }
        Assert.Equal(40, step);
        var loaded = PhysicsWorld2D.LoadScene(new StringReader(rec.SaveScene()));
        Assert.Equal(rec.ComputeHash(), loaded.ComputeHash());
        for (int i = 0; i < 20; i++) { rec.Step(Dt); loaded.Step(Dt); }
        Assert.Equal(rec.ComputeHash(), loaded.ComputeHash());

        var p = rec.GetBicycles()[0].Frame.Position;
        ulong h = rec.ComputeRegionHash(new Vector2D(p.X - 1, p.Y - 1), new Vector2D(p.X + 1, p.Y + 1));
        rec.GetBicycles()[0].Ride(pedal: -1f); rec.Step(Dt);
        Assert.NotEqual(h, rec.ComputeRegionHash(new Vector2D(p.X - 1, p.Y - 1), new Vector2D(p.X + 1, p.Y + 1)));
        double x = rec.GetBicycles()[0].Frame.Position.X;
        rec.ShiftOrigin(new Vector2D(16, 0));
        Assert.Equal(x - 16, rec.GetBicycles()[0].Frame.Position.X, 1e-9);
        for (int i = 0; i < 20; i++) { rec.Step(Dt); }
        Assert.True(rec.GetBicycles()[0].Wheels.GetWheel(1).IsInContact);
        var g = new RecursiveAI.OnePhysics.Debug.DebugGeometry();
        rec.GetDebugGeometry(RecursiveAI.OnePhysics.Debug.DebugContent.Vehicles, g);
        Assert.Equal(4, g.Wheels.Length);
    }
}
