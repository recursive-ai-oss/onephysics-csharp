#pragma warning disable OPHYS0003
using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;

// Fuzz targets for 2D vehicles and ragdolls.
// "world2d-vehicles" and "world2d-ragdolls" turn bytes into a program: bodies, hostile definitions (wheel geometry, tyre curves, engine, gearbox, differentials, skeletons with bad parents, limits and masses),
// inputs, poses, stepping, destroying chassis, ground and parts, stale handles and snapshot round trips. Only argument errors and typed physics errors may come out.
// "world2d-vehicle-snapshot" and "world2d-ragdoll-snapshot" flip bytes in the snapshot of a world with vehicles or ragdolls: rejected with the world unchanged, or accepted and re-serialized to the same bytes.
internal static class FuzzVehicles2D
{
    private static int s_made, s_refused, s_steps, s_accepted, s_rejected;
    private static byte[]? s_vehicleBase, s_ragdollBase;

    static FuzzVehicles2D()
    {
        AppDomain.CurrentDomain.ProcessExit += (_, _) => Console.WriteLine("2d vehicle and ragdoll targets: made " + s_made + ", refused " + s_refused + ", steps " + s_steps + "; snapshots accepted " + s_accepted + ", rejected " + s_rejected);
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

        public Vector2F V2F() => new(Float(), Float());
        public bool Bool() => (Byte() & 1) != 0;
    }

    private static bool Allowed(Exception e) => e is ArgumentException || e is PhysicsException;

    private static void StepTolerant(PhysicsWorld2D w)
    {
        try { w.StepChecked(1f / 60f); s_steps++; }
        catch (BodyOutOfBoundsException e) { e.Body.Destroy(); }
    }

    private static PhysicsWorld2D NewWorld() => new(new WorldOptions2D { WorldId = 9, Substeps = 4 });

    private static Body[] Scene(PhysicsWorld2D w) =>
    [
        w.AddBox(new Vector2F(160f, 1f), new Vector2D(0, -0.5), type: BodyType.Static),
        w.AddBox(new Vector2F(3f, 0.4f), new Vector2D(10, 0.2)),
    ];

    private static TyreDef Tyre(Reader r) => new() { Friction = r.Float(), LongitudinalB = r.Float(), LongitudinalC = r.Float(), LongitudinalE = r.Float(), LowSpeed = r.Float() };

    private static WheelDef Wheel(Reader r, VehicleDef good, int i)
    {
        if (r.Byte() < 160) { var w = good.Wheels[i % good.Wheels.Length].Clone(); if (r.Bool()) { w.Radius = r.Float(); } if (r.Bool()) { w.SpringFrequency = r.Float(); } return w; }
        return new WheelDef
        {
            Position = r.V2F(), SuspensionDirection = r.V2F(), ForwardDirection = r.V2F(), Radius = r.Float(), SuspensionMinLength = r.Float(), SuspensionMaxLength = r.Float(),
            SpringFrequency = r.Float(), DampingRatio = r.Float(), PreloadLength = r.Float(), SprungMass = r.Float(), Inertia = r.Float(), AngularDamping = r.Float(), MaxBrakeTorque = r.Float(),
            MaxHandbrakeTorque = r.Float(), Tyre = Tyre(r),
        };
    }

    private static EngineDef Engine(Reader r) => new() { MaxTorque = r.Float(), MinRpm = r.Float(), MaxRpm = r.Float(), Inertia = r.Float(), EngineBraking = r.Float(), ClutchStrength = r.Float(), TorqueCurve = r.Bool() ? null : new[] { r.Float(), r.Float(), r.Float() } };

    private static TransmissionDef Gearbox(Reader r) => new() { Automatic = r.Bool(), ForwardRatios = r.Bool() ? null : new[] { r.Float(), r.Float() }, ReverseRatio = r.Float(), SwitchTime = r.Float(), ShiftUpRpm = r.Float(), ShiftDownRpm = r.Float() };

    private static VehicleDef Def(Reader r)
    {
        VehicleDef good = r.Byte() % 3 == 0 ? VehicleDef.Truck : r.Bool() ? VehicleDef.Car : VehicleDef.Kart;
        if (r.Byte() < 140) { return good; }
        int n = r.Byte() % 7;
        var wheels = new WheelDef[n];
        for (int i = 0; i < n; i++) { wheels[i] = Wheel(r, good, i); }
        var def = new VehicleDef { Wheels = wheels, AirPitchTorque = r.Float(), GroundPitchTorque = r.Float() };
        if (r.Bool()) { def.Engine = Engine(r); }
        if (r.Bool()) { def.Transmission = Gearbox(r); }
        int d = r.Byte() % 3;
        def.Differentials = new DifferentialDef[d];
        for (int i = 0; i < d; i++) { def.Differentials[i] = new DifferentialDef { Left = r.Byte() % 7 - 1, Right = r.Byte() % 7 - 1, Ratio = r.Float(), LeftRightSplit = r.Float(), LimitedSlipRatio = r.Float(), EngineTorqueRatio = r.Float() }; }
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
        if (r.Bool()) { d.AirPitchTorque = r.Float(); }
        if (r.Bool()) { d.Grip = Tyre(r); }
        if (r.Bool()) { d.Wheels = Array.Empty<WheelDef>(); }
        else if (r.Bool()) { d.Wheels = new WheelDef[r.Byte() % 20]; for (int i = 0; i < d.Wheels.Length; i++) { d.Wheels[i] = Wheel(r, VehicleDef.Truck, i); } }
        return d;
    }

    private static MotorcycleDef BikeDef(Reader r)
    {
        var d = MotorcycleDef.Road;
        if (r.Byte() < 140) { return d; }
        if (r.Bool()) { d.LeanTorque = r.Float(); }
        if (r.Bool()) { d.FinalDriveRatio = r.Float(); }
        if (r.Bool()) { d.Engine = Engine(r); }
        if (r.Bool()) { d.Transmission = Gearbox(r); }
        if (r.Bool()) { d.Front = Wheel(r, VehicleDef.Car, 0); }
        if (r.Bool()) { d.Rear = Wheel(r, VehicleDef.Car, 1); }
        return d;
    }

    private static T Pick<T>(List<T> l, Reader r) => l.Count == 0 ? default! : l[r.Byte() % l.Count];

    private static void VehicleOp(PhysicsWorld2D w, List<Body> bodies, List<Vehicle> vs, Reader r)
    {
        switch (r.Byte() % 12)
        {
            case 0: bodies.Add(w.AddBox(r.V2F(), new Vector2D(r.Float() % 40f, r.Float() % 10f), r.Float(), density: r.Float())); break;
            case 1: { var b = w.AddBox(new Vector2F(3f, 0.6f), new Vector2D(r.Float() % 30f, 1.0 + r.Byte() % 3)); bodies.Add(b); vs.Add(w.AddVehicle(b, Def(r))); s_made++; break; }
            case 2: vs.Add(w.AddVehicle(new Vector2D(r.Float() % 30f, r.Float() % 3f), Def(r), r.V2F(), r.Float(), r.Float())); s_made++; break;
            case 3: { var b = w.AddBox(new Vector2F(5f, 1f), new Vector2D(r.Float() % 30f, 1.2)); bodies.Add(b); vs.Add(w.AddVehicle(b, TrackedDef(r))); s_made++; break; }
            case 4: { var b = w.AddBox(new Vector2F(1.6f, 0.4f), new Vector2D(r.Float() % 30f, 1.0)); bodies.Add(b); vs.Add(w.AddVehicle(b, BikeDef(r))); s_made++; break; }
            case 5: { var v = Pick(vs, r); if (v.IsValid) { v.Drive(r.Float(), r.Float(), r.Float(), r.Float()); } break; }
            case 6: { var v = Pick(vs, r); if (v.IsValid) { v.Gear = r.Byte() % 9 - 2; } break; }
            case 7: for (int i = 0, n = 1 + r.Byte() % 8; i < n; i++) { StepTolerant(w); } break;
            case 8: { var b = Pick(bodies, r); if (b.IsValid) { b.Destroy(); } break; }
            case 9: { var v = Pick(vs, r); if (v.IsValid) { v.Destroy(); } break; }
            case 10:
                {
                    var v = Pick(vs, r);
                    if (v.IsValid) { _ = v.EngineRpm; _ = v.ForwardSpeed; _ = v.Gear; _ = v.Chassis.Position; _ = v.Kind; _ = v.TrackSpeed; if (v.WheelCount > 0) { _ = v.GetWheel(r.Byte() % v.WheelCount); } }
                    else if (vs.Count > 0) { try { v.Drive(0f); } catch (StaleHandleException) { } catch (InvalidHandleException) { } }
                    break;
                }
            default: w.ShiftOrigin(new Vector2D(r.Byte() % 8, 0)); break;
        }
    }

    private static void RoundTrip(PhysicsWorld2D w, Func<PhysicsWorld2D, int> count)
    {
        for (int i = 0; i < 4; i++) { StepTolerant(w); }
        byte[] snap = w.Snapshot();
        var w2 = NewWorld();
        try { w2.Restore(snap); }
        catch (SnapshotFormatException e) when (e.Message.Contains("out of range", StringComparison.Ordinal) || e.Message.Contains("non-finite", StringComparison.Ordinal)) { return; }
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world hash differs"); }
        if (!w2.Snapshot().AsSpan().SequenceEqual(snap)) { throw new InvalidOperationException("restored world re-serializes differently"); }
        if (count(w2) != count(w)) { throw new InvalidOperationException("restored world has a different count"); }
        StepTolerant(w); StepTolerant(w2);
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world diverged after a step"); }
    }

    public static void RunVehicles(byte[] data)
    {
        var r = new Reader(data);
        var w = NewWorld();
        var bodies = new List<Body>(Scene(w));
        var vs = new List<Vehicle>();
        int ops = 4 + r.Byte() % 48;
        for (int n = 0; n < ops && r.More; n++)
        {
            try { VehicleOp(w, bodies, vs, r); }
            catch (Exception e) when (Allowed(e)) { s_refused++; }
        }

        RoundTrip(w, x => x.GetVehicles().Length);
    }

    // ---- ragdolls ---------------------------------------------------------------------------------

    private static RagdollPartDef Part(Reader r, RagdollPartDef good) => r.Byte() < 170 ? good.Clone() : new RagdollPartDef
    {
        Parent = r.Byte() % 9 - 1, Shape = (RagdollShapeKind)(r.Byte() % 4), ShapeHalfSize = r.V2F(), Position = r.V2F(), Angle = r.Float(), Mass = r.Float(), Friction = r.Float(), JointPosition = r.V2F(),
        LowerAngle = r.Float(), UpperAngle = r.Float(), DriveHertz = r.Float(), MaxDriveSpeed = r.Float(), MaxTorque = r.Float(), BreakForce = r.Float(), BreakTorque = r.Float(),
    };

    private static RagdollDef RagDef(Reader r)
    {
        var good = r.Bool() ? RagdollDef.Humanoid : RagdollDef.CreateHumanoid(0.5f + r.Byte() / 100f, 5f + r.Byte());
        if (r.Byte() < 150) { return good; }
        int n = r.Byte() % 16;
        var parts = new RagdollPartDef[n];
        for (int i = 0; i < n; i++) { parts[i] = Part(r, good.Parts[i % good.Parts.Length]); if (i == 0 && r.Byte() < 200) { parts[i].Parent = -1; } }
        return new RagdollDef { Parts = parts, SelfCollision = (RagdollSelfCollision)(r.Byte() % 3), AngularDamping = r.Float(), CategoryBits = r.Byte(), MaskBits = r.Byte() };
    }

    private static float[] Pose(Reader r, int n)
    {
        var p = new float[r.Byte() < 230 ? n : r.Byte() % 20];
        for (int i = 0; i < p.Length; i++) { p[i] = r.Float() % 7f; }
        return p;
    }

    private static void RagdollOp(PhysicsWorld2D w, List<Body> bodies, List<Ragdoll> rs, Reader r)
    {
        switch (r.Byte() % 10)
        {
            case 0: bodies.Add(w.AddBox(r.V2F(), new Vector2D(r.Float() % 40f, r.Float() % 10f), r.Float(), density: r.Float())); break;
            case 1: { var def = RagDef(r); rs.Add(w.AddRagdoll(def, new Vector2D(r.Float() % 20f, 1.0 + r.Byte() % 4), r.Float(), r.Bool() ? default : Pose(r, def.Parts?.Length ?? 0))); s_made++; break; }
            case 2: { var g = Pick(rs, r); if (g.IsValid) { g.DriveToPose(Pose(r, g.PartCount), r.Float()); } break; }
            case 3: { var g = Pick(rs, r); if (g.IsValid) { g.StopDriving(); } break; }
            case 4: { var g = Pick(rs, r); if (g.IsValid) { g.SetPose(Pose(r, g.PartCount), new Vector2D(r.Float() % 30f, r.Float() % 10f), r.Float()); } break; }
            case 5: { var g = Pick(rs, r); if (g.IsValid) { g.MoveToPose(Pose(r, g.PartCount), new Vector2D(r.Float() % 30f, r.Float() % 10f), r.Float(), r.Float()); } break; }
            case 6: for (int i = 0, n = 1 + r.Byte() % 8; i < n; i++) { StepTolerant(w); } break;
            case 7: { var g = Pick(rs, r); if (g.IsValid) { g.Remove(); } break; }
            case 8: { var g = Pick(rs, r); if (g.IsValid) { _ = g.GetPoseError(Pose(r, g.PartCount)); _ = g.IsAttached(r.Byte() % g.PartCount); _ = g.GetBody(r.Byte() % g.PartCount).Position; _ = g.GetJoint(1 + r.Byte() % Math.Max(1, g.PartCount - 1)); } else if (rs.Count > 0) { try { g.StopDriving(); } catch (StaleHandleException) { } catch (InvalidHandleException) { } } break; }
            default: { var b = Pick(bodies, r); if (b.IsValid) { b.Destroy(); } break; }
        }
    }

    public static void RunRagdolls(byte[] data)
    {
        var r = new Reader(data);
        var w = NewWorld();
        var bodies = new List<Body>(Scene(w));
        var rs = new List<Ragdoll>();
        int ops = 4 + r.Byte() % 40;
        for (int n = 0; n < ops && r.More; n++)
        {
            try { RagdollOp(w, bodies, rs, r); }
            catch (Exception e) when (Allowed(e)) { s_refused++; }
        }

        RoundTrip(w, x => x.GetRagdolls().Length);
    }

    // ---- snapshots --------------------------------------------------------------------------------

    private static byte[] VehicleBase()
    {
        if (s_vehicleBase != null) { return s_vehicleBase; }
        var w = NewWorld();
        Scene(w);
        w.AddCar(new Vector2D(-6, 0)).Drive(0.7f, 0.2f);
        w.AddTruck(new Vector2D(0, 0)).Drive(0.5f);
        w.AddKart(new Vector2D(6, 0)).Drive(1f);
        w.AddTank(new Vector2D(-20, 0)).Drive(0.6f);
        w.AddMotorcycle(new Vector2D(20, 0)).Drive(0.5f, 0.2f);
        for (int s = 0; s < 40; s++) { w.StepChecked(1f / 60f); }
        return s_vehicleBase = w.Snapshot();
    }

    private static byte[] RagdollBase()
    {
        if (s_ragdollBase != null) { return s_ragdollBase; }
        var w = NewWorld();
        Scene(w);
        var def = RagdollDef.Humanoid;
        def.Parts[4].BreakTorque = 5f;
        var a = w.AddRagdoll(def, new Vector2D(-3, 1));
        a.DriveToPose(new float[def.Parts.Length], 0.5f);
        w.AddRagdoll(RagdollDef.CreateHumanoid(1.3f, 40f), new Vector2D(3, 3), 0.4f);
        for (int s = 0; s < 40; s++) { w.StepChecked(1f / 60f); }
        return s_ragdollBase = w.Snapshot();
    }

    private static void SnapshotOf(byte[] data, byte[] snapshot, Action<PhysicsWorld2D> touch)
    {
        var bytes = (byte[])snapshot.Clone();
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
        touch(w);
        StepTolerant(w);
        StepTolerant(w);
    }

    public static void VehicleSnapshotTarget(byte[] data) => SnapshotOf(data, VehicleBase(), w => { foreach (var v in w.GetVehicles()) { _ = v.EngineRpm; if (v.WheelCount > 0) { _ = v.GetWheel(0); } v.Drive(1f); } });

    public static void RagdollSnapshotTarget(byte[] data) => SnapshotOf(data, RagdollBase(), w => { foreach (var g in w.GetRagdolls()) { _ = g.IsPowered; _ = g.GetPoseError(new float[g.PartCount]); } });
}
