using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

public class Vehicle3DTests
{
    private const float Dt = 1f / 60f;
    private const float G = 9.81f;
    private const float RestY = 0.5f + 0.34f - 0.1104f;   // wheel radius + max length - static deflection of the default car

    private static PhysicsWorld3D NewWorld(int threads = 1, int id = 91) => new(new WorldOptions3D { ThreadCount = threads, WorldId = id });

    private static Body Floor(PhysicsWorld3D w, float friction = 0.6f, float half = 400f, double cx = 0) =>
        w.AddBox(new Vector3F(2 * half, 1f, 2 * half), at: new Vector3D(cx, -0.5, 0), type: BodyType.Static, material: new Material3D(friction, 0f, 0f));

    /// <summary>A 1500 kg chassis box with the vehicle on it. The chassis origin is the wheel tops.</summary>
    private static (Body Chassis, Vehicle Car) Car(PhysicsWorld3D w, double x, double y, double z, VehicleDef? def = null, float mass = 1500f)
    {
        var box = new Vector3F(1.8f, 0.5f, 4.2f);
        var body = w.AddBox(box, at: new Vector3D(x, y, z), density: mass / (box.X * box.Y * box.Z));
        return (body, w.AddVehicle(body, def ?? VehicleDef.Car));
    }

    private static void Run(PhysicsWorld3D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    [Fact]
    public void SuspensionAtRestHoldsTheStaticDeflection()
    {
        var w = NewWorld();
        Floor(w);
        var (body, car) = Car(w, 0, 0.9, 0);
        Run(w, 360);
        float defl = G / ((2f * MathF.PI * 1.5f) * (2f * MathF.PI * 1.5f));
        float sum = 0f;
        for (int i = 0; i < 4; i++)
        {
            var ws = car.GetWheel(i);
            Assert.True(ws.IsInContact);
            Assert.Equal(0.5f - defl, ws.SuspensionLength, 0.02f * defl + 0.002f);
            sum += ws.SuspensionForce;
        }

        Assert.Equal(1500f * G, sum, 0.01f * 1500f * G);
        Assert.InRange(body.Position.Y, RestY - 0.01, RestY + 0.01);
    }

    private static double LowestAfterSettling(float damping, out double rest)
    {
        var w = NewWorld();
        Floor(w);
        var def = VehicleDef.Car;
        foreach (var wd in def.Wheels) { wd.DampingRatio = damping; }
        // Start 5 cm above where the wheels carry the car, so it falls onto its equilibrium from above.
        var (body, car) = Car(w, 0, RestY + 0.06, 0, def);
        double min = double.MaxValue;
        for (int i = 0; i < 300; i++)
        {
            w.Step(Dt);
            if (i > 3) { min = Math.Min(min, body.Position.Y); }
        }

        rest = body.Position.Y;
        return min;
    }

    [Fact]
    public void CriticalDampingDoesNotOvershootButLightDampingDoes()
    {
        double min = LowestAfterSettling(1f, out double rest);
        Assert.True(min >= rest - 0.002, $"overshoot {rest - min}");
        double minLight = LowestAfterSettling(0.15f, out double restLight);
        Assert.True(minLight < restLight - 0.01, $"light damping should overshoot, got {restLight - minLight}");
    }

    [Fact]
    public void AStraightLaunchReachesTheSpeedTheDrivetrainPredicts()
    {
        var w = NewWorld();
        Floor(w, half: 2000f);
        var def = VehicleDef.Car;
        def.Transmission.Automatic = false;
        var (body, car) = Car(w, 0, RestY, -500, def);
        car.Gear = 1;
        // Top speed in first gear: the rev limit through gear and final drive to the wheel.
        float predicted = 6500f * 2f * MathF.PI / 60f * 0.34f / (3.5f * 3.4f);
        for (int i = 0; i < 60 * 10; i++)
        {
            car.Drive(1f);
            w.Step(Dt);
            Assert.True(car.ForwardSpeed < predicted * 1.03f, $"step {i}: {car.ForwardSpeed} above {predicted}");
        }

        Assert.InRange(car.ForwardSpeed, predicted * 0.96f, predicted * 1.03f);
        Assert.InRange(car.EngineRpm, 6000f, 6800f);
        Assert.Equal(0f, Math.Abs(car.GetWheel(2).SlipRatio), 0.05f);
    }

    [Fact]
    public void TheAutomaticBoxShiftsUpAndGoesFaster()
    {
        var w = NewWorld();
        Floor(w, half: 4000f);
        var (body, car) = Car(w, 0, RestY, -3000);
        for (int i = 0; i < 60 * 14; i++) { car.Drive(1f); w.Step(Dt); }
        Assert.True(car.Gear >= 3, $"gear {car.Gear}");
        Assert.True(car.ForwardSpeed > 30f, $"speed {car.ForwardSpeed}");
    }

    [Fact]
    public void BrakingStopsWithinTheTyresFrictionLimit()
    {
        var w = NewWorld();
        Floor(w, half: 2000f);
        var def = VehicleDef.Car;
        foreach (var wd in def.Wheels) { wd.MaxBrakeTorque = 8000f; }   // enough to lock every wheel
        var (body, car) = Car(w, 0, RestY, -500, def);
        body.LinearVelocity = new Vector3F(0f, 0f, 20f);
        double z0 = body.Position.Z;
        for (int i = 0; i < 60 * 8; i++)
        {
            car.Drive(0f, 0f, 1f);
            w.Step(Dt);
        }

        double dist = body.Position.Z - z0;
        // All four tyres slide at slip -1: the curve there is about 0.94 of its peak, with sqrt(1.5 * 0.6) friction.
        float mu = MathF.Sqrt(1.5f * 0.6f) * 0.939f;
        double expected = 20.0 * 20.0 / (2.0 * mu * G);
        Assert.InRange(dist, expected * 0.9, expected * 1.1);
        Assert.True(Math.Abs(body.LinearVelocity.Z) < 0.1f);
    }

    [Fact]
    public void ACornerNeverExceedsTheTyrePeakAndGetsCloseToIt()
    {
        var w = NewWorld();
        Floor(w, half: 2000f);
        var (body, car) = Car(w, 0, RestY, 0);
        body.LinearVelocity = new Vector3F(0f, 0f, 14f);
        const int window = 30;
        var ring = new double[window];
        double best = 0;
        for (int i = 0; i < 60 * 10; i++)
        {
            var v0 = body.LinearVelocity;
            float speed = MathF.Sqrt(v0.X * v0.X + v0.Z * v0.Z);
            // Hold the speed with the throttle and wind the steering on slowly, so the corner tightens until the tyres saturate.
            car.Drive(Math.Clamp(0.45f + 0.3f * (14f - speed), 0f, 1f), Math.Min(1f, i / 360f));
            w.Step(Dt);
            // Centripetal acceleration: the part of the planar acceleration across the direction of travel.
            var v = body.LinearVelocity;
            float ax = (v.X - v0.X) / Dt, az = (v.Z - v0.Z) / Dt;
            ring[i % window] = Math.Abs((v.X * az - v.Z * ax) / Math.Max(speed, 1f));
            if (i >= window) { double mean = 0; foreach (double a in ring) { mean += a; } best = Math.Max(best, mean / window); }
        }

        float limit = MathF.Sqrt(1.5f * 0.6f) * G;
        Assert.True(best <= limit * 1.08, $"averaged lateral acceleration {best} above the tyre peak {limit}");
        Assert.True(best >= limit * 0.6, $"averaged lateral acceleration {best} never got near the peak {limit}");
    }

    private static double SplitMuSpeed(float limitedSlip, out float spinDifference)
    {
        var w = NewWorld();
        // Left side (x < 0) icy, right side dry.
        Floor(w, 0.04f, 400f, -400);
        Floor(w, 0.6f, 400f, 400);
        var def = VehicleDef.Car;
        def.Transmission.Automatic = false;
        def.Differentials[0].LimitedSlipRatio = limitedSlip;
        var (body, car) = Car(w, 0, RestY, -300, def);
        car.Gear = 1;
        float diff = 0f;
        for (int i = 0; i < 60 * 4; i++)
        {
            car.Drive(1f);
            w.Step(Dt);
            if (i > 60) { diff += Math.Abs(car.GetWheel(2).AngularVelocity - car.GetWheel(3).AngularVelocity); }
        }

        spinDifference = diff / (60 * 3);
        return car.ForwardSpeed;
    }

    [Fact]
    public void ALimitedSlipDifferentialPullsBetterThanAnOpenOneOnSplitFriction()
    {
        double open = SplitMuSpeed(1f, out float openSpin);
        double lsd = SplitMuSpeed(8f, out float lsdSpin);
        Assert.True(lsd > open * 1.25 + 0.5, $"open {open} m/s, limited slip {lsd} m/s");
        Assert.True(openSpin > lsdSpin, $"wheel speed difference open {openSpin}, limited slip {lsdSpin}");
    }

    private static double MeanRoll(float barStiffness)
    {
        var w = NewWorld();
        Floor(w, half: 2000f);
        var def = VehicleDef.Car;
        foreach (var bar in def.AntiRollBars) { bar.Stiffness = barStiffness; }
        var (body, car) = Car(w, 0, RestY, 0, def);
        body.LinearVelocity = new Vector3F(0f, 0f, 12f);
        double sum = 0; int count = 0;
        for (int i = 0; i < 60 * 5; i++)
        {
            car.Drive(0.2f, 0.12f);
            w.Step(Dt);
            if (i >= 150)
            {
                var axisX = body.Rotation.Rotate(new Vector3F(1f, 0f, 0f));
                sum += Math.Abs(Math.Asin(axisX.Y)); count++;
            }
        }

        return sum / count;
    }

    [Fact]
    public void AntiRollBarsReduceBodyRoll()
    {
        double soft = MeanRoll(0f), stiff = MeanRoll(120000f);
        Assert.True(soft > 0.01, $"the corner should roll the body, got {soft}");
        Assert.True(stiff < soft * 0.85, $"roll without bars {soft}, with {stiff}");
    }

    [Fact]
    public void WheelsDoNotSinkOrSnagOnMeshSeams()
    {
        var w = NewWorld();
        // A flat triangle mesh of 2 m cells, 200 m long: every cell edge is a seam.
        const int n = 101;
        var verts = new Vector3F[n * 5];
        for (int j = 0; j < n; j++) { for (int i = 0; i < 5; i++) { verts[j * 5 + i] = new Vector3F(-4f + 2f * i, 0f, 2f * j - 20f); } }
        var idx = new List<int>();
        for (int j = 0; j + 1 < n; j++)
        {
            for (int i = 0; i < 4; i++)
            {
                int a = j * 5 + i, b = a + 1, c = a + 5, d = c + 1;
                if (((i + j) & 1) == 0) { idx.AddRange(new[] { a, c, b, b, c, d }); } else { idx.AddRange(new[] { a, c, d, a, d, b }); }
            }
        }

        w.AddMesh(verts, idx.ToArray());
        var def = VehicleDef.Car;
        var (body, car) = Car(w, 0, RestY, -15, def);
        float lowest = float.MaxValue;
        for (int i = 0; i < 60 * 6; i++)
        {
            car.Drive(0.6f);
            w.Step(Dt);
            for (int k = 0; k < 4; k++) { lowest = Math.Min(lowest, (float)car.GetWheel(k).Center.Y); }
        }

        Assert.True(lowest > 0.34f - 0.03f, $"a wheel centre dropped to {lowest}");
        Assert.True(car.ForwardSpeed > 8f, $"the car snagged: {car.ForwardSpeed} m/s");
        Assert.True(body.Position.Z > 20, $"z = {body.Position.Z}");
    }

    private static PhysicsWorld3D DriveScene(int threads, int steps, out List<ulong> hashes)
    {
        var w = NewWorld(threads, 92);
        Floor(w);
        // A dynamic slab with a heavy box on it as moving ground, and three cars with different inputs.
        w.AddBox(new Vector3F(6f, 0.4f, 6f), at: new Vector3D(10, 0.2, 10), density: 400f);
        for (int k = 0; k < 3; k++)
        {
            Car(w, -6 * k, RestY, -10 * k - 5, k == 1 ? VehicleDef.Truck : VehicleDef.Car, k == 1 ? 8000f : 1500f);
        }

        var cars = w.GetVehicles();
        hashes = new List<ulong>();
        for (int i = 0; i < steps; i++)
        {
            for (int c = 0; c < cars.Length; c++) { if (cars[c].IsValid) { cars[c].Drive(0.8f, 0.3f * MathF.Sin(0.02f * i + c), c == 0 ? 0f : 0.1f * c * (i % 90 > 60 ? 1 : 0)); } }
            w.Step(Dt);
            hashes.Add(w.ComputeHash());
        }

        return w;
    }

    [Fact]
    public void SnapshotAndRestoreMidDriveIsByteExact()
    {
        var w = NewWorld(1, 93);
        Floor(w);
        var cars = new List<Vehicle>();
        for (int k = 0; k < 3; k++) { cars.Add(Car(w, -6 * k, RestY, -10, k == 1 ? VehicleDef.Kart : VehicleDef.Car, k == 1 ? 150f : 1500f).Car); }
        void Tick(int i) { for (int c = 0; c < cars.Count; c++) { cars[c].Drive(0.9f, 0.4f * MathF.Sin(0.03f * i + c), i % 100 > 80 ? 0.5f : 0f, i % 150 > 140 ? 1f : 0f); } w.Step(Dt); }
        for (int i = 0; i < 150; i++) { Tick(i); }
        var snap = w.Snapshot();
        var first = new List<ulong>();
        for (int i = 150; i < 300; i++) { Tick(i); first.Add(w.ComputeHash()); }
        var end1 = w.Snapshot();

        w.Restore(snap);
        Assert.Equal(snap, w.Snapshot());
        var second = new List<ulong>();
        for (int i = 150; i < 300; i++) { Tick(i); second.Add(w.ComputeHash()); }
        Assert.Equal(first, second);
        Assert.Equal(end1, w.Snapshot());

        // Into a fresh world built the same way.
        var f = NewWorld(1, 93);
        Floor(f);
        for (int k = 0; k < 3; k++) { Car(f, -6 * k, RestY, -10, k == 1 ? VehicleDef.Kart : VehicleDef.Car, k == 1 ? 150f : 1500f); }
        f.Restore(snap);
        Assert.Equal(snap, f.Snapshot());
        var fcars = f.GetVehicles();
        for (int i = 150; i < 300; i++) { for (int c = 0; c < fcars.Length; c++) { fcars[c].Drive(0.9f, 0.4f * MathF.Sin(0.03f * i + c), i % 100 > 80 ? 0.5f : 0f, i % 150 > 140 ? 1f : 0f); } f.Step(Dt); }
        Assert.Equal(end1, f.Snapshot());
    }

    [Fact]
    public void OneThreadAndManyThreadsGiveTheSameWorlds()
    {
        DriveScene(1, 240, out var h1);
        DriveScene(8, 240, out var h8);
        Assert.Equal(h1, h8);
    }

    [Fact]
    public void BadDefinitionsAndInputsAreTypedErrors()
    {
        var w = NewWorld();
        Floor(w);
        var (body, car) = Car(w, 0, RestY, 0);
        Assert.Throws<ArgumentOutOfRangeException>(() => car.Drive(float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => car.Drive(0f, 2f));
        Assert.Throws<ArgumentOutOfRangeException>(() => car.Drive(0f, 0f, -0.1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => car.Drive(0f, 0f, 0f, float.PositiveInfinity));
        Assert.Throws<ArgumentOutOfRangeException>(() => car.GetWheel(4));
        Assert.Throws<PhysicsException>(() => w.AddVehicle(body, VehicleDef.Car));
        var other = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(20, 3, 0));
        var d = VehicleDef.Car; d.Wheels[0].Radius = float.NaN;
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() => w.AddVehicle(other, d));
        Assert.Contains("Wheels[0].Radius", ex.Message);
        d = VehicleDef.Car; d.Differentials[0].Right = 9;
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddVehicle(other, d));
        d = VehicleDef.Car; d.Wheels = Array.Empty<WheelDef>();
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddVehicle(other, d));
        d = VehicleDef.Car; d.Wheels[1].SuspensionMaxLength = d.Wheels[1].SuspensionMinLength;
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddVehicle(other, d));
        var stat = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(30, 3, 0), type: BodyType.Static);
        Assert.Throws<ArgumentException>(() => w.AddVehicle(stat, VehicleDef.Car));
        Assert.True(w.TryGetVehicle(body, out var found) && found == car);
        body.Destroy();
        Assert.False(car.IsValid);
        Assert.Throws<StaleHandleException>(() => car.Drive(1f));
        Assert.Empty(w.GetVehicles());
        foreach (var p in new[] { VehicleDef.Car, VehicleDef.Truck, VehicleDef.Kart }) { p.Validate(); }
    }

    [Fact]
    public void ACarParkedOnADynamicSlabLoadsItAndStaysFinite()
    {
        var w = NewWorld();
        Floor(w);
        var slab = w.AddBox(new Vector3F(8f, 0.5f, 10f), at: new Vector3D(0, 0.25, 0), density: 500f);
        var (body, car) = Car(w, 0, 0.5 + RestY + 0.1, 0);
        Run(w, 300);
        Assert.True(car.GetWheel(0).IsInContact);
        Assert.Equal(slab.RawHandle.Index, car.GetWheel(0).GroundBody.RawHandle.Index);
        Assert.InRange(body.Position.Y, 0.5 + RestY - 0.05, 0.5 + RestY + 0.05);
        Assert.True(double.IsFinite(body.Position.X) && Math.Abs(body.LinearVelocity.Y) < 0.1f);
    }
}
