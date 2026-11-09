#pragma warning disable OPHYS0002
using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Replay;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

/// <summary>The Whipple-model bicycle against the benchmark of Meijaard, Papadopoulos, Ruina and Schwab (Proc. R. Soc. A 463, 2007), and the bicycle through the host hooks.</summary>
public class Bicycle3DTests
{
    // The paper's benchmark linearised equations M q'' + v C1 q' + (g K0 + v^2 K2) q = 0, q = (roll, steer): the reference for the speeds and rates below.
    private static readonly double[,] M = { { 80.81722, 2.31941332208709 }, { 2.31941332208709, 0.29784188199686 } };
    private static readonly double[,] K0 = { { -80.95, -2.59951685249872 }, { -2.59951685249872, -0.80329488458618 } };
    private static readonly double[,] K2 = { { 0, 76.59734589573222 }, { 0, 2.65431523794604 } };
    private static readonly double[,] C1 = { { 0, 33.86641391492494 }, { -0.85035641456978, 1.68540397397560 } };

    private static Complex[] PaperEigenvalues(double v)
    {
        Complex F(Complex l)
        {
            Complex E(int i, int j) => l * l * M[i, j] + l * v * C1[i, j] + 9.81 * K0[i, j] + v * v * K2[i, j];
            return E(0, 0) * E(1, 1) - E(0, 1) * E(1, 0);
        }

        var co = new Complex[5];
        for (int k = 0; k < 5; k++)
        {
            Complex s = 0;
            for (int j = 0; j < 5; j++) { var z = Complex.FromPolarCoordinates(1, 2 * Math.PI * j / 5); s += F(z) * Complex.Pow(z, -k); }
            co[k] = s / 5;
        }

        var r = new Complex[4];
        for (int i = 0; i < 4; i++) { r[i] = Complex.Pow(new Complex(0.4, 0.9), i); }
        for (int it = 0; it < 500; it++)
        {
            for (int i = 0; i < 4; i++)
            {
                Complex p = 0;
                for (int k = 4; k >= 0; k--) { p = p * r[i] + co[k]; }
                p /= co[4];
                Complex d = 1;
                for (int j = 0; j < 4; j++) { if (j != i) { d *= r[i] - r[j]; } }
                r[i] -= p / d;
            }
        }

        return r;
    }

    private static double PaperMaxRe(double v) { double m = double.MinValue; foreach (var x in PaperEigenvalues(v)) { m = Math.Max(m, x.Real); } return m; }

    private static double PaperCrossing(double lo, double hi, bool rising)
    {
        for (int i = 0; i < 60; i++)
        {
            double mid = 0.5 * (lo + hi);
            bool unstable = PaperMaxRe(mid) > 0;
            if (unstable == rising) { hi = mid; } else { lo = mid; }
        }

        return 0.5 * (lo + hi);
    }

    private static PhysicsWorld3D NewWorld(float rate, int threads = 1, int id = 111)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { WorldId = id, ThreadCount = threads });
        w.AddBox(new Vector3F(400, 1, 400), at: new Vector3D(0, -0.5, 0), type: BodyType.Static, material: new Material3D(1f, 0f, 0f));
        return w;
    }

    // Growth rate (1/s) of the lean amplitude of the uncontrolled benchmark bicycle: the slope of the log of its 0.5 s peaks between 2 s and the end of the run, after a small lean-rate perturbation.
    private static double GrowthRate(float speed, float rate)
    {
        var w = NewWorld(rate);
        var bike = w.AddBicycle(new Vector3D(0, 0, -20), BicycleDef.Benchmark, speed);
        var f = bike.Frame; var k = bike.Fork;
        f.AngularVelocity = new Vector3F(0, 0, -0.05f); k.AngularVelocity = new Vector3F(0, 0, -0.05f);
        float dt = 1f / rate;
        int n = (int)(10 / dt), per = (int)(0.5 / dt);
        var lean = new double[n];
        for (int i = 0; i < n; i++)
        {
            w.Step(dt);
            lean[i] = bike.LeanAngle;
            if (Math.Abs(lean[i]) > 0.4) { n = i; break; }
        }

        var xs = new List<double>(); var ys = new List<double>();
        for (int w2 = 0; (w2 + 1) * per <= n; w2++)
        {
            double peak = 0;
            for (int i = w2 * per; i < (w2 + 1) * per; i++) { peak = Math.Max(peak, Math.Abs(lean[i])); }
            if (w2 >= 4 && peak > 1e-6) { xs.Add((w2 + 0.5) * 0.5); ys.Add(Math.Log(peak)); }
        }

        double sx = 0, sy = 0, sxx = 0, sxy = 0; int c = xs.Count;
        for (int i = 0; i < c; i++) { sx += xs[i]; sy += ys[i]; sxx += xs[i] * xs[i]; sxy += xs[i] * ys[i]; }
        return (c * sxy - sx * sy) / (c * sxx - sx * sx);
    }

    [Fact]
    public void PaperReferenceReproducesTheBenchmarkSpeeds()
    {
        Assert.InRange(PaperCrossing(3, 5, false), 4.29, 4.30);   // weave speed 4.292 m/s
        Assert.InRange(PaperCrossing(5, 8, true), 6.02, 6.03);    // capsize speed 6.024 m/s
    }

    [Fact]
    public void UncontrolledBicycleIsSelfStableBetweenWeaveAndCapsizeSpeeds()
    {
        // Measured weave and capsize speeds (where the growth rate of the lean crosses zero) against the paper's 4.29 and 6.02 m/s, at 120 steps a second.
        double lo = 3.75, hi = 4.75;
        for (int i = 0; i < 6; i++) { double mid = 0.5 * (lo + hi); if (GrowthRate((float)mid, 120f) > 0) { lo = mid; } else { hi = mid; } }
        double weave = 0.5 * (lo + hi);
        lo = 5.5; hi = 7.0;
        for (int i = 0; i < 6; i++) { double mid = 0.5 * (lo + hi); if (GrowthRate((float)mid, 120f) > 0) { hi = mid; } else { lo = mid; } }
        double capsize = 0.5 * (lo + hi);
        Assert.InRange(weave, 4.292 * 0.95, 4.292 * 1.05);
        Assert.InRange(capsize, 6.024 * 0.92, 6.024 * 1.08);
        Assert.True(GrowthRate(3.0f, 120f) > 0.3, "weave grows below the band");
        Assert.True(GrowthRate(7.5f, 120f) > 0.05, "capsize grows above the band");
        // Inside the band the slowest decay at 5 m/s is the paper's capsize eigenvalue (-0.323 per second) within 25 %.
        double rate = GrowthRate(5.0f, 120f);
        Assert.InRange(rate, -0.323 * 1.25, -0.323 * 0.75);
    }

    [Fact]
    public void BicycleKeepsItsSpeedAndStaysUprightInsideTheBand()
    {
        var w = NewWorld(60);
        var bike = w.AddBicycle(new Vector3D(0, 0, -20), BicycleDef.Benchmark, 5f);
        var f = bike.Frame; f.AngularVelocity = new Vector3F(0, 0, -0.05f);
        for (int i = 0; i < 600; i++) { w.Step(1f / 60f); }
        Assert.InRange(bike.Speed, 4.9f, 5.1f);
        Assert.True(Math.Abs(bike.LeanAngle) < 0.02f);
        Assert.False(bike.HasFallen);
        Assert.True(bike.RearWheel.GetWheel(0).IsInContact && bike.FrontWheel.GetWheel(0).IsInContact);
    }

    [Fact]
    public void StandingBicycleFallsOver()
    {
        var w = NewWorld(60);
        var bike = w.AddBicycle(new Vector3D(0, 0, 0), BicycleDef.Benchmark, 0f);
        var f = bike.Frame; f.AngularVelocity = new Vector3F(0, 0, -0.05f);
        for (int i = 0; i < 300; i++) { w.Step(1f / 60f); }
        Assert.True(bike.HasFallen);
    }

    [Fact]
    public void PedalsAndBrakesChangeTheSpeed()
    {
        var w = NewWorld(60);
        var bike = w.AddBicycle(new Vector3D(0, 0, -50), BicycleDef.Road, 6f);
        for (int i = 0; i < 90; i++) { bike.Ride(pedal: 1f); w.Step(1f / 60f); }
        float fast = bike.Speed;
        Assert.True(fast > 6.5f, $"pedalling should speed it up: {fast}");
        for (int i = 0; i < 60; i++) { bike.Ride(rearBrake: 0.8f); w.Step(1f / 60f); }
        Assert.True(bike.Speed < fast - 0.5f);
    }

    [Fact]
    public void StaleBicycleThrowsAndInputsAreValidated()
    {
        var w = NewWorld(60);
        var bike = w.AddBicycle(new Vector3D(0, 0, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => bike.Ride(steer: float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => bike.Ride(frontBrake: 2f));
        var h = bike.RawHandle;
        Assert.True(w.IsBicycleAlive(h));
        bike.Destroy();
        Assert.False(bike.IsValid);
        Assert.Throws<StaleHandleException>(() => _ = bike.Speed);
        Assert.Throws<StaleHandleException>(() => w.GetBicycle(h));
        Assert.Throws<InvalidHandleException>(() => _ = default(Bicycle).Frame);
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddBicycle(new Vector3D(0, 0, 0), new BicycleDef { WheelBase = float.NaN }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddBicycle(new Vector3D(0, 0, 0), new BicycleDef { RearFrame = new BicyclePart { Ixx = 1, Izz = 1, Ixz = 5 } }));
    }

    private static PhysicsWorld3D Riding(int threads = 1)
    {
        var w = NewWorld(60, threads, 112);
        var bike = w.AddBicycle(new Vector3D(0, 0, -20), BicycleDef.Benchmark, 5f);
        var f = bike.Frame; f.AngularVelocity = new Vector3F(0, 0, -0.05f);
        var bike2 = w.AddBicycle(new Vector3D(10, 0, -20), BicycleDef.Road, 6f, QuaternionF.CreateFromAxisSinCos(new Vector3F(0, 1, 0), 0.0499792f, 0.99875f));
        bike2.Ride(pedal: 0.3f, lean: 0.1f);
        return w;
    }

    [Fact]
    public void SnapshotRestoreReproducesTheRunAndHashesMatchAcrossThreadCounts()
    {
        var a = Riding();
        for (int i = 0; i < 40; i++) { a.Step(1f / 60f); }
        var snap = a.Snapshot();
        var hashes = new List<ulong>();
        for (int i = 0; i < 60; i++) { a.Step(1f / 60f); hashes.Add(a.ComputeHash()); }
        var bytes = a.Snapshot();
        var b = Riding();
        b.Restore(snap);
        for (int i = 0; i < 60; i++) { b.Step(1f / 60f); Assert.Equal(hashes[i], b.ComputeHash()); }
        Assert.Equal(bytes, b.Snapshot());
        var t4 = Riding(4);
        for (int i = 0; i < 100; i++) { t4.Step(1f / 60f); }
        Assert.Equal(a.ComputeHash(), t4.ComputeHash());
        // A world without bicycles carries no bicycle section.
        var plain = NewWorld(60);
        plain.AddSphere(0.5f, new Vector3D(0, 3, 0));
        var restored = Riding(); restored.Restore(plain.Snapshot());
        Assert.Empty(restored.GetBicycles());
    }

    [Fact]
    public void ForkRecordedReplaySceneAndDesyncFinderSeeTheBicycle()
    {
        var a = Riding();
        for (int i = 0; i < 20; i++) { a.Step(1f / 60f); }
        var fork = a.Fork();
        Assert.Equal(a.ComputeHash(), fork.ComputeHash());
        Assert.Equal(0, a.FindDifferences(fork).Count);
        fork.GetBicycles()[0].Ride(steer: 0.5f);
        fork.Step(1f / 60f); a.Step(1f / 60f);
        Assert.True(a.FindDifferences(fork).Count > 0);
        Assert.Contains(a.FindDifferences(fork).Differences, d => d.Section.Length > 0);

        var rec = Riding();
        using var stream = new MemoryStream();
        var expected = new List<ulong>();
        using (var recorder = rec.StartRecording(stream))
        {
            for (int i = 0; i < 50; i++) { if (i == 10) { rec.GetBicycles()[0].Ride(pedal: 0.5f, lean: -0.3f); } rec.Step(1f / 60f); expected.Add(rec.ComputeHash()); }
        }

        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var copy = Riding();
        int step = 0;
        while (player.ReadNextStep(copy)) { Assert.Equal(expected[step++], copy.ComputeHash()); }
        Assert.Equal(50, step);

        var loaded = PhysicsWorld3D.LoadScene(new StringReader(rec.SaveScene()));
        Assert.Equal(rec.ComputeHash(), loaded.ComputeHash());
        Assert.Equal(2, loaded.GetBicycles().Length);
        for (int i = 0; i < 20; i++) { rec.Step(1f / 60f); loaded.Step(1f / 60f); }
        Assert.Equal(rec.ComputeHash(), loaded.ComputeHash());
    }

    [Fact]
    public void RegionKeepsBothBodiesOfABicycleTogetherAndShiftCarriesItAlong()
    {
        var w = Riding();
        for (int i = 0; i < 30; i++) { w.Step(1f / 60f); }
        var bike = w.GetBicycles()[0];
        var p = bike.Fork.Position;
        var min = new Vector3D(p.X - 0.2, p.Y - 0.2, p.Z - 0.2); var max = new Vector3D(p.X + 0.2, p.Y + 0.2, p.Z + 0.2);
        ulong h = w.ComputeRegionHash(min, max);
        bike.Ride(steer: 1f); w.Step(1f / 60f); bike.Ride();
        Assert.NotEqual(h, w.ComputeRegionHash(new Vector3D(min.X, min.Y, min.Z + 0.1), new Vector3D(max.X, max.Y, max.Z + 0.1)));
        double z = bike.Frame.Position.Z;
        w.ShiftOrigin(new Vector3D(0, 0, 16));
        Assert.Equal(z - 16, bike.Frame.Position.Z, 1e-9);
        for (int i = 0; i < 30; i++) { w.Step(1f / 60f); }
        Assert.True(bike.RearWheel.GetWheel(0).IsInContact);
    }

    [Fact]
    public void ShapesAddNoMassAndAFallenBicycleLiesOnTheGround()
    {
        const float Dt = 1f / 60f;
        var w = NewWorld(60);
        var bike = w.AddBicycle(new Vector3D(0, 0, -20), BicycleDef.Benchmark, 2.5f);
        var def = BicycleDef.Benchmark;
        Assert.Equal(def.RearFrame.Mass, bike.Frame.Mass, 1e-3);
        Assert.Equal(def.Front.Mass, bike.Fork.Mass, 1e-3);
        var f = bike.Frame; var k = bike.Fork;
        f.AngularVelocity = new Vector3F(0, 0, -0.05f); k.AngularVelocity = new Vector3F(0, 0, -0.05f);
        for (int i = 0; i < 60 * 12; i++) { w.Step(Dt); }
        Assert.True(bike.HasFallen, $"below the stable band it falls: lean {bike.LeanAngle}");
        double lowest = double.MaxValue;
        foreach (var body in new[] { bike.Frame, bike.Fork }) { lowest = Math.Min(lowest, body.Position.Y); }
        Assert.InRange(lowest, -0.05, 1.0);   // lying on the ground, not through it
        float speed = Math.Max(bike.Frame.LinearVelocity.Length(), bike.Fork.LinearVelocity.Length());
        Assert.True(speed < 1.5f, $"it has come to rest: {speed}");
        foreach (var c in w.GetContacts(bike.Frame)) { for (int i = 0; i < c.PointCount; i++) { Assert.True(-c.Separation(i) < 0.05f, "penetration"); } }
    }

    [Fact]
    public void DestroyingEitherBodyOrTheBicycleRemovesBothBodiesAndTheWheels()
    {
        const float Dt = 1f / 60f;
        for (int how = 0; how < 3; how++)
        {
            var w = NewWorld(60);
            int before = w.Core.Bodies.Handles.Count;
            var bike = w.AddBicycle(new Vector3D(0, 0, -20), BicycleDef.Road, 5f);
            Assert.Equal(before + 2, w.Core.Bodies.Handles.Count);
            var frame = bike.Frame; var fork = bike.Fork; var rear = bike.RearWheel; var front = bike.FrontWheel;
            if (how == 0) { frame.Destroy(); } else if (how == 1) { fork.Destroy(); } else { bike.Destroy(); }
            Assert.False(bike.IsValid); Assert.False(frame.IsValid); Assert.False(fork.IsValid);
            Assert.False(rear.IsValid); Assert.False(front.IsValid);
            Assert.Equal(before, w.Core.Bodies.Handles.Count);
            Assert.Empty(w.GetBicycles());
            for (int i = 0; i < 5; i++) { w.Step(Dt); }
        }
    }
}
