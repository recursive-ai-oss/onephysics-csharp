using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;

// Fuzz targets for 3D wheeled vehicles.
// "world3d-vehicles" turns bytes into a program: chassis bodies, hostile vehicle definitions (wheel geometry, tyre curves, engine, gearbox, differentials, anti-roll bars), drive inputs, stepping,
// destroying chassis and ground bodies, stale handles and snapshot round trips. Only argument errors and typed physics errors may come out.
// "world3d-vehicle-snapshot" flips bytes in the snapshot of a world with vehicles: rejected with the world unchanged, or accepted and re-serialized to the same bytes.
internal static class FuzzVehicles3D
{
    private static int s_vehicles, s_refused, s_steps, s_accepted, s_rejected;
    private static byte[]? s_base;

    static FuzzVehicles3D()
    {
        AppDomain.CurrentDomain.ProcessExit += (_, _) => Console.WriteLine("vehicle targets: vehicles " + s_vehicles + ", refused " + s_refused + ", steps " + s_steps + "; snapshots accepted " + s_accepted + ", rejected " + s_rejected);
    }

    private sealed class Reader
    {
        private readonly byte[] _d;
        private int _i;

        public Reader(byte[] d) { _d = d; }

        public bool More => _i < _d.Length;

        public byte Byte() => _i < _d.Length ? _d[_i++] : (byte)0;

        public float Float()
        {
            byte b = Byte();
            switch (b)
            {
                case 250: return float.NaN;
                case 251: return float.PositiveInfinity;
                case 252: return float.NegativeInfinity;
                case 253: return 3e6f;
                case 254: return 1e-9f;
                case 255: return -0f;
            }

            int v = (short)(b | (Byte() << 8));
            return v / 256f;
        }

        public Vector3F V3F() => new(Float(), Float(), Float());

        public bool Bool() => (Byte() & 1) != 0;
    }

    private static bool Allowed(Exception e) => e is ArgumentException || e is PhysicsException;

    private static void StepTolerant(PhysicsWorld3D w)
    {
        try { w.Step(1f / 60f); s_steps++; }
        catch (BodyOutOfBoundsException e) { e.Body.Destroy(); }
    }

    private static PhysicsWorld3D NewWorld() => new(new WorldOptions3D { WorldId = 9, Substeps = 4 });

    private static Body[] Scene(PhysicsWorld3D w)
    {
        return
        [
            w.AddBox(new Vector3F(80f, 1f, 80f), new Vector3D(0, -0.5, 0), type: BodyType.Static),
            w.AddMesh([new Vector3F(0, 0, 0), new Vector3F(0, 0, 6), new Vector3F(8, 2, 6), new Vector3F(8, 2, 0)], [0, 1, 2, 0, 2, 3], new Vector3D(-14, 0, -3)),
            w.AddBox(new Vector3F(3f, 0.4f, 3f), new Vector3D(10, 0.2, 4)),
        ];
    }

    private static TyreDef Tyre(Reader r)
    {
        return new TyreDef
        {
            Friction = r.Float(), LongitudinalB = r.Float(), LongitudinalC = r.Float(), LongitudinalE = r.Float(), LateralB = r.Float(), LateralC = r.Float(), LateralE = r.Float(), LowSpeed = r.Float(),
        };
    }

    private static WheelDef Wheel(Reader r, VehicleDef good, int i)
    {
        if (r.Byte() < 160) { var w = good.Wheels[i % good.Wheels.Length].Clone(); if (r.Bool()) { w.Radius = r.Float(); } if (r.Bool()) { w.SpringFrequency = r.Float(); } return w; }
        return new WheelDef
        {
            Position = r.V3F(), SuspensionDirection = r.V3F(), SteerAxis = r.V3F(), AxleDirection = r.V3F(), ForwardDirection = r.V3F(), Radius = r.Float(), Width = r.Float(), SuspensionMinLength = r.Float(), SuspensionMaxLength = r.Float(),
            SpringFrequency = r.Float(), DampingRatio = r.Float(), PreloadLength = r.Float(), SprungMass = r.Float(), Inertia = r.Float(), AngularDamping = r.Float(), MaxSteerAngle = r.Float(), MaxBrakeTorque = r.Float(),
            MaxHandbrakeTorque = r.Float(), Tyre = Tyre(r), CastShape = r.Bool() ? WheelCastShape.Sphere : WheelCastShape.Cylinder,
        };
    }

    private static VehicleDef Def(Reader r)
    {
        VehicleDef good = r.Byte() % 3 == 0 ? VehicleDef.Truck : r.Bool() ? VehicleDef.Car : VehicleDef.Kart;
        if (r.Byte() < 140) { return good; }
        int n = r.Byte() % 7;
        var wheels = new WheelDef[n];
        for (int i = 0; i < n; i++) { wheels[i] = Wheel(r, good, i); }
        var def = new VehicleDef { Wheels = wheels };
        if (r.Bool()) { def.Engine = new EngineDef { MaxTorque = r.Float(), MinRpm = r.Float(), MaxRpm = r.Float(), Inertia = r.Float(), EngineBraking = r.Float(), ClutchStrength = r.Float(), TorqueCurve = r.Bool() ? null : new[] { r.Float(), r.Float(), r.Float() } }; }
        if (r.Bool()) { def.Transmission = new TransmissionDef { Automatic = r.Bool(), ForwardRatios = r.Bool() ? null : new[] { r.Float(), r.Float() }, ReverseRatio = r.Float(), SwitchTime = r.Float(), ShiftUpRpm = r.Float(), ShiftDownRpm = r.Float() }; }
        int d = r.Byte() % 3;
        def.Differentials = new DifferentialDef[d];
        for (int i = 0; i < d; i++) { def.Differentials[i] = new DifferentialDef { Left = r.Byte() % 7 - 1, Right = r.Byte() % 7 - 1, Ratio = r.Float(), LeftRightSplit = r.Float(), LimitedSlipRatio = r.Float(), EngineTorqueRatio = r.Float() }; }
        int a = r.Byte() % 3;
        def.AntiRollBars = new AntiRollBarDef[a];
        for (int i = 0; i < a; i++) { def.AntiRollBars[i] = new AntiRollBarDef { Left = r.Byte() % 7 - 1, Right = r.Byte() % 7 - 1, Stiffness = r.Float() }; }
        return def;
    }

    private static TrackedVehicleDef TrackedDef(Reader r)
    {
        var d = TrackedVehicleDef.Tank;
        if (r.Byte() < 140) { return d; }
        if (r.Bool()) { d.SprocketRadius = r.Float(); }
        if (r.Bool()) { d.MaxTrackSpeed = r.Float(); }
        if (r.Bool()) { d.MaxDriveTorque = r.Float(); }
        if (r.Bool()) { d.MaxBrakeTorque = r.Float(); }
        if (r.Bool()) { d.LateralFriction = r.Float(); }
        if (r.Bool()) { d.Grip = Tyre(r); }
        if (r.Bool()) { d.LeftWheels = Array.Empty<WheelDef>(); }
        else if (r.Bool()) { d.LeftWheels = new[] { Wheel(r, VehicleDef.Car, 0) }; }
        if (r.Bool()) { d.RightWheels = new WheelDef[r.Byte() % 20]; for (int i = 0; i < d.RightWheels.Length; i++) { d.RightWheels[i] = Wheel(r, VehicleDef.Truck, i); } }
        return d;
    }

    private static MotorcycleDef BikeDef(Reader r)
    {
        var d = MotorcycleDef.Road;
        if (r.Byte() < 140) { return d; }
        if (r.Bool()) { d.MaxSteerAngle = r.Float(); }
        if (r.Bool()) { d.HighSpeedSteerAngle = r.Float(); }
        if (r.Bool()) { d.SteerFadeStartSpeed = r.Float(); }
        if (r.Bool()) { d.SteerFadeEndSpeed = r.Float(); }
        if (r.Bool()) { d.MaxLeanAngle = r.Float(); }
        if (r.Bool()) { d.FallAngle = r.Float(); }
        if (r.Bool()) { d.LeanFrequency = r.Float(); }
        if (r.Bool()) { d.LeanDamping = r.Float(); }
        if (r.Bool()) { d.MaxLeanTorque = r.Float(); }
        if (r.Bool()) { d.FinalDriveRatio = r.Float(); }
        if (r.Bool()) { d.Front = Wheel(r, VehicleDef.Car, 0); }
        if (r.Bool()) { d.Rear = Wheel(r, VehicleDef.Car, 2); }
        return d;
    }

    private static Vehicle Pick(List<Vehicle> vs, Reader r)
    {
        byte b = r.Byte();
        if (vs.Count == 0 || b >= 240) { return default; }
        return vs[b % vs.Count];
    }

    public static void Run(byte[] data)
    {
        var r = new Reader(data);
        var w = NewWorld();
        var bodies = new List<Body>(Scene(w));
        var vs = new List<Vehicle>();
        int ops = 4 + r.Byte() % 48;
        for (int n = 0; n < ops && r.More; n++)
        {
            try { Op(w, bodies, vs, r); }
            catch (Exception e) when (Allowed(e)) { s_refused++; }
        }

        for (int i = 0; i < 4; i++) { StepTolerant(w); }
        byte[] snap = w.Snapshot();
        var w2 = NewWorld();
        try { w2.Restore(snap); }
        catch (SnapshotFormatException e) when (e.Message.Contains("out of range", StringComparison.Ordinal) || e.Message.Contains("non-finite", StringComparison.Ordinal)) { return; }
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world hash differs"); }
        if (!w2.Snapshot().AsSpan().SequenceEqual(snap)) { throw new InvalidOperationException("restored world re-serializes differently"); }
        if (w2.GetVehicles().Length != w.GetVehicles().Length) { throw new InvalidOperationException("restored world has a different vehicle count"); }
        StepTolerant(w); StepTolerant(w2);
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world diverged after a step"); }
    }

    private static void Op(PhysicsWorld3D w, List<Body> bodies, List<Vehicle> vs, Reader r)
    {
        switch (r.Byte() % 12)
        {
            case 0:
            case 1:
                {
                    var def = Def(r);
                    var chassis = w.AddBox(new Vector3F(1.8f, 0.6f, 4f), new Vector3D(r.Float() % 20f, 1.2, r.Float() % 20f));
                    bodies.Add(chassis);
                    vs.Add(w.AddVehicle(r.Byte() < 20 ? bodies[r.Byte() % bodies.Count] : chassis, def));
                    s_vehicles++;
                    break;
                }
            case 10:
                {
                    var def = TrackedDef(r);
                    var chassis = w.AddBox(new Vector3F(3.2f, 1f, 6f), new Vector3D(r.Float() % 20f, 1.4, r.Float() % 20f), density: 400f);
                    bodies.Add(chassis);
                    vs.Add(w.AddVehicle(chassis, def));
                    s_vehicles++;
                    break;
                }
            case 11:
                {
                    var def = BikeDef(r);
                    var chassis = w.AddBox(new Vector3F(0.3f, 0.5f, 1.6f), new Vector3D(r.Float() % 20f, 1.0, r.Float() % 20f), density: 1000f);
                    bodies.Add(chassis);
                    vs.Add(w.AddVehicle(chassis, def));
                    s_vehicles++;
                    break;
                }
            case 2:
            case 3:
            case 4: { var v = Pick(vs, r); v.Drive(r.Float(), r.Float(), r.Float(), r.Float()); break; }
            case 5: { int n = 1 + r.Byte() % 6; for (int i = 0; i < n; i++) { StepTolerant(w); } break; }
            case 6: { var v = Pick(vs, r); v.Gear = (int)r.Float(); break; }
            case 7: { var v = Pick(vs, r); if (r.Bool()) { v.Destroy(); } else { var b = bodies[r.Byte() % bodies.Count]; if (b.IsValid && b != bodies[0]) { b.Destroy(); } } break; }
            case 8:
                {
                    var v = Pick(vs, r);
                    if (v.IsValid) { int n = v.WheelCount; _ = v.GetWheel(r.Byte() % (n + 1)); _ = v.EngineRpm; _ = v.ForwardSpeed; _ = v.Gear; _ = v.Chassis.Position; _ = v.Kind; if (v.Kind == VehicleKind.Tracked) { _ = v.GetTrackSpeed(r.Byte() % 2); } if (v.Kind == VehicleKind.Motorcycle) { _ = v.LeanAngle; _ = v.SteerLimit; } v.SetInput(Math.Clamp(r.Float(), -1f, 1f), Math.Clamp(r.Float(), -1f, 1f), Math.Clamp(Math.Abs(r.Float()), 0f, 1f)); }
                    else { _ = v.WheelCount; }
                    break;
                }
            default:
                {
                    byte[] snap = w.Snapshot();
                    var w2 = NewWorld();
                    try { w2.Restore(snap); }
                    catch (SnapshotFormatException e) when (e.Message.Contains("out of range", StringComparison.Ordinal) || e.Message.Contains("non-finite", StringComparison.Ordinal)) { break; }
                    if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world hash differs"); }
                    break;
                }
        }
    }

    private static byte[] Base()
    {
        if (s_base != null) { return s_base; }
        var w = NewWorld();
        Scene(w);
        for (int i = 0; i < 3; i++)
        {
            var chassis = w.AddBox(new Vector3F(1.8f, 0.6f, 4f), new Vector3D(-6 + 6 * i, 1.0, 0));
            var v = w.AddVehicle(chassis, i == 0 ? VehicleDef.Car : i == 1 ? VehicleDef.Truck : VehicleDef.Kart);
            v.Drive(0.7f, 0.2f * i);
        }

        var tank = w.AddTank(new Vector3D(-20, 0, 10)); tank.SetInput(0.6f, 0.3f);
        var bike = w.AddMotorcycle(new Vector3D(20, 0, 10)); bike.SetInput(0.5f, 0.2f);
        for (int s = 0; s < 40; s++) { w.Step(1f / 60f); }
        return s_base = w.Snapshot();
    }

    public static void SnapshotTarget(byte[] data)
    {
        var bytes = (byte[])Base().Clone();
        int mutations = data.Length > 0 ? 1 + data[0] % 5 : 0;
        for (int i = 1, m = 0; i + 2 < data.Length && m < mutations; i += 3, m++)
        {
            int at = ((data[i] << 8) | data[i + 1]) % bytes.Length;
            bytes[at] ^= data[i + 2];
        }

        if (data.Length > 0 && (data[0] & 0x0F) == 0x0F) { Array.Resize(ref bytes, (data[^1] * bytes.Length) / 256); }

        var w = NewWorld();
        byte[] empty = w.Snapshot();
        try { w.Restore(bytes); }
        catch (SnapshotFormatException)
        {
            s_rejected++;
            if (!w.Snapshot().AsSpan().SequenceEqual(empty)) { throw new InvalidOperationException("a rejected snapshot changed the world"); }
            return;
        }

        s_accepted++;
        if (!w.Snapshot().AsSpan().SequenceEqual(bytes)) { throw new InvalidOperationException("accepted snapshot does not re-serialize to the same bytes"); }
        foreach (var v in w.GetVehicles()) { _ = v.EngineRpm; _ = v.GetWheel(0); v.Drive(1f); }
        StepTolerant(w);
        StepTolerant(w);
    }
}
