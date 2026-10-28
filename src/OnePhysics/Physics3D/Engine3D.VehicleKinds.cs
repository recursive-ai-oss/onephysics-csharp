using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    private const uint BlockVehicleKinds = 32;

    // ------------------------------------------------------------------ tracked vehicles

    /// <summary>
    /// Fills the road wheels' drive torques and returns the brake input. Each track is a rigid belt over its road wheels: the sprocket pushes the belt towards the commanded speed
    /// (left = throttle + steering, right = throttle − steering, scaled together when either exceeds 1) with at most its torque, so the belt force is the belt's inertia over the step capped at <c>MaxDriveTorque / SprocketRadius</c>.
    /// </summary>
    private static float TrackDrive(VehicleState s, float h)
    {
        var t = s.Track!;
        int n = s.Wheels;
        float fwd = s.Forward, st = s.Right;
        float cl = fwd + st, cr = fwd - st;
        float m = Math.Max(1f, Math.Max(Math.Abs(cl), Math.Abs(cr)));
        float keep = 1f - s.Brake;   // the brake overrides the drive
        cl = cl / m * keep; cr = cr / m * keep;
        // A track that is not commanded is held by its brake, as a tank's steering brakes hold it on a slope; a driven track releases.
        s.BrakeLeft = cl == 0f ? 1f : s.Brake;
        s.BrakeRight = cr == 0f ? 1f : s.Brake;
        float fmax = t.MaxDriveTorque / t.SprocketRadius;
        for (int side = 0; side < 2; side++)
        {
            int a = side == 0 ? 0 : s.TrackLeft, z = side == 0 ? s.TrackLeft : n;
            float target = (side == 0 ? cl : cr) * t.MaxTrackSpeed;
            float mt = 0f, vm = 0f;
            for (int i = a; i < z; i++)
            {
                float r = s.Def.Wheels[i].Radius;
                float mi = s.Def.Wheels[i].Inertia / (r * r);   // the wheel's inertia seen along the belt
                mt += mi; vm += mi * s.Omega[i] * r;
            }

            float vk = vm / mt;
            float f = mt * (target - vk) / h;
            if (f > fmax) { f = fmax; } else if (f < -fmax) { f = -fmax; }
            for (int i = a; i < z; i++)
            {
                float r = s.Def.Wheels[i].Radius;
                s.Torque[i] = f * (s.Def.Wheels[i].Inertia / (r * r)) / mt * r;
            }
        }

        return s.Brake;
    }

    /// <summary>The wheel visited at position <paramref name="k"/> of the interleaved order: left 0, right 0, left 1, right 1 ...; the longer track's extra wheels come last.</summary>
    private static int InterleavedTrackWheel(VehicleState s, int k)
    {
        int nl = s.TrackLeft, nr = s.Wheels - nl, common = Math.Min(nl, nr);
        if (k < 2 * common) { return (k & 1) == 0 ? k >> 1 : nl + (k >> 1); }
        return k - common;
    }

    /// <summary>After the tyres have acted: the belt is rigid, so the wheels of a track share one speed (inertia-weighted), limited to the track speed limit.</summary>
    private static void CoupleTracks(VehicleState s)
    {
        var t = s.Track!;
        int n = s.Wheels;
        for (int side = 0; side < 2; side++)
        {
            int a = side == 0 ? 0 : s.TrackLeft, z = side == 0 ? s.TrackLeft : n;
            float mt = 0f, vm = 0f;
            for (int i = a; i < z; i++)
            {
                float r = s.Def.Wheels[i].Radius;
                float mi = s.Def.Wheels[i].Inertia / (r * r);
                mt += mi; vm += mi * s.Omega[i] * r;
            }

            float v = vm / mt;
            if (v > t.MaxTrackSpeed) { v = t.MaxTrackSpeed; } else if (v < -t.MaxTrackSpeed) { v = -t.MaxTrackSpeed; }
            for (int i = a; i < z; i++) { s.Omega[i] = v / s.Def.Wheels[i].Radius; }
        }
    }

    /// <summary>Belt speed of one track as of the last step, m/s; positive moves the hull forward.</summary>
    public static float TrackSpeedOf(VehicleState s, int track)
    {
        int a = track == 0 ? 0 : s.TrackLeft, z = track == 0 ? s.TrackLeft : s.Wheels;
        float mt = 0f, vm = 0f;
        for (int i = a; i < z; i++)
        {
            float r = s.Def.Wheels[i].Radius;
            float mi = s.Def.Wheels[i].Inertia / (r * r);
            mt += mi; vm += mi * s.Omega[i] * r;
        }

        return vm / mt;
    }

    // ------------------------------------------------------------------ motorcycles

    public static float BikeSteerLimitAt(VehicleState s, float speed) => BikeSteerLimit(s, speed);

    /// <summary>Front-wheel angle limit at a speed: the full angle up to the fade start, the high-speed angle from the fade end, linear between.</summary>
    private static float BikeSteerLimit(VehicleState s, float speed)
    {
        var b = s.Bike!;
        float v = Math.Abs(speed);
        float full = s.Def.Wheels[0].MaxSteerAngle;
        if (v <= b.SteerFadeStartSpeed) { return full; }
        if (v >= b.SteerFadeEndSpeed) { return b.HighSpeedSteerAngle; }
        float k = (v - b.SteerFadeStartSpeed) / (b.SteerFadeEndSpeed - b.SteerFadeStartSpeed);
        return full + (b.HighSpeedSteerAngle - full) * k;
    }

    /// <summary>
    /// The balance assist: measures the lean (the angle of the chassis' up axis from vertical, about the heading, positive to the right), picks the lean a steady turn needs at the current speed and steering
    /// (the single-track relation tan φ = v² tan δ / (g L), Cossalter, Motorcycle Dynamics) and applies a spring-damper roll impulse about the heading towards it. The spring is implicit, so any frequency is stable
    /// (Catto 2011, soft constraints).
    /// </summary>
    private void BalanceBike(VehicleState s, float h, QuaternionF q, float speed)
    {
        var b = s.Bike!;
        var bodies = Core.Bodies;
        int body = s.Body;
        var g = Opt.Gravity;
        float gl = (float)Math.Sqrt(g.X * g.X + g.Y * g.Y + g.Z * g.Z);
        if (!(gl > 0f)) { return; }
        var up = g * (-1f / gl);
        var wf = s.Def.Wheels[0];
        var fwdC = q.Rotate(wf.ForwardDirection);
        var fh = fwdC - up * Vector3F.Dot(fwdC, up);
        float l2 = fh.LengthSquared();
        if (!(l2 > 1e-4f)) { return; }
        fh = fh * (1f / (float)Math.Sqrt(l2));
        var rh = Vector3F.Cross(up, fh);
        var upC = q.Rotate(new Vector3F(0f, 1f, 0f));
        float lean = Math32.Atan2(Vector3F.Dot(upC, rh), Vector3F.Dot(upC, up));
        s.Lean = lean;
        float wheelbase = Math.Abs(Vector3F.Dot(wf.Position - s.Def.Wheels[1].Position, wf.ForwardDirection));
        float target = 0f;
        if (speed > 0f && wheelbase > 0f)
        {
            float delta = s.Right * BikeSteerLimit(s, speed);
            float tanD = Math32.Sin(delta) / Math32.Cos(delta);
            target = Math32.Atan(speed * speed * tanD / (gl * wheelbase));
            if (target > b.MaxLeanAngle) { target = b.MaxLeanAngle; } else if (target < -b.MaxLeanAngle) { target = -b.MaxLeanAngle; }
        }

        // The wanted lean moves at a bounded rate (1.5 rad/s), so a sudden change of steering is followed smoothly instead of by a torque-limited snap that overshoots.
        float maxMove = 1.5f * h, move = target - s.LeanTarget;
        if (move > maxMove) { move = maxMove; } else if (move < -maxMove) { move = -maxMove; }
        s.LeanTarget += move;
        target = s.LeanTarget;
        if (Math.Abs(lean) > b.FallAngle || !(b.MaxLeanTorque > 0f)) { s.LeanIntegral = 0f; return; }
        var ang = bodies.AngularVelocityOf(body);
        float rate = -(ang[0] * fh.X + ang[1] * fh.Y + ang[2] * fh.Z);   // lean to the right is a turn about the heading in the negative sense
        var iw = ApplyWorldInertia(body, fh);
        float invI0 = Vector3F.Dot(fh, iw);
        if (!(invI0 > 0f)) { return; }
        // The bike rolls about the line through its two contact patches, not about its centre of mass. The assist therefore turns the whole bike about that line: it changes the angular velocity about the heading and the linear
        // velocity of the centre of mass together, so the patches do not slide. The inertia it faces is the roll inertia about the centre of mass plus m h² (parallel axis theorem), h the height of the centre of mass above the line.
        var lc = LocalCenterOf(body);
        var d0 = s.Def.Wheels[0]; var d1 = s.Def.Wheels[1];
        float hgt = lc.Y - 0.5f * ((d0.Position.Y + d0.SuspensionDirection.Y * s.Length[0]) + (d1.Position.Y + d1.SuspensionDirection.Y * s.Length[1])) + 0.5f * (d0.Radius + d1.Radius);
        if (hgt < 0f) { hgt = 0f; }
        float invI = 1f / (1f / invI0 + hgt * hgt / bodies.InverseMass[body]);
        float wn = TwoPi * b.LeanFrequency;
        float k = wn * wn, c = 2f * b.LeanDamping * wn, ki = 0.5f * wn * k;
        // The integral term takes up what the tyres push with steadily (the lateral force of a turn acts at the contact patch, below the centre of mass); without it the spring settles short of the target.
        float err = target - lean;
        float cap = h * b.MaxLeanTorque * invI;   // largest change of lean rate in one step
        float iLimit = b.MaxLeanTorque * invI / ki;
        float li = s.LeanIntegral + err * h;
        if (li > iLimit) { li = iLimit; } else if (li < -iLimit) { li = -iLimit; }
        s.LeanIntegral = li;
        float dRate = h * (k * err + ki * li - c * rate) / (1f + h * c + h * h * k);
        if (dRate > cap) { dRate = cap; } else if (dRate < -cap) { dRate = -cap; }
        var dOmega = fh * -dRate;   // lean to the right is a turn about the heading in the negative sense
        ang[0] += dOmega.X; ang[1] += dOmega.Y; ang[2] += dOmega.Z;
        var dv = Vector3F.Cross(dOmega, upC * hgt);
        var lin = bodies.LinearVelocityOf(body);
        lin[0] += dv.X; lin[1] += dv.Y; lin[2] += dv.Z;
    }

    // ------------------------------------------------------------------ snapshot block

    private bool HasVehicleKinds()
    {
        for (int v = 0; v < VehHandles.HighWater; v++)
        {
            if (VehHandles.IsLive(v) && Vehicles[v]!.Kind != VehicleKind.Wheeled) { return true; }
        }

        return false;
    }

    private void WriteVehicleKinds(StateWriter w)
    {
        int count = 0;
        for (int v = 0; v < VehHandles.HighWater; v++) { if (VehHandles.IsLive(v) && Vehicles[v]!.Kind != VehicleKind.Wheeled) { count++; } }
        w.WriteInt32(count);
        for (int v = 0; v < VehHandles.HighWater; v++)
        {
            if (!VehHandles.IsLive(v)) { continue; }
            var s = Vehicles[v]!;
            if (s.Kind == VehicleKind.Wheeled) { continue; }
            w.WriteInt32(v); w.WriteByte((byte)s.Kind);
            if (s.Kind == VehicleKind.Tracked)
            {
                var t = s.Track!;
                w.WriteSingle(t.SprocketRadius); w.WriteSingle(t.MaxTrackSpeed); w.WriteSingle(t.MaxDriveTorque); w.WriteSingle(t.MaxBrakeTorque); w.WriteSingle(t.LateralFriction); w.WriteInt32(s.TrackLeft);
            }
            else
            {
                var b = s.Bike!;
                w.WriteSingle(b.HighSpeedSteerAngle); w.WriteSingle(b.SteerFadeStartSpeed); w.WriteSingle(b.SteerFadeEndSpeed); w.WriteSingle(b.MaxLeanAngle); w.WriteSingle(b.FallAngle);
                w.WriteSingle(b.LeanFrequency); w.WriteSingle(b.LeanDamping); w.WriteSingle(b.MaxLeanTorque); w.WriteSingle(s.Lean); w.WriteSingle(s.LeanTarget); w.WriteSingle(s.LeanIntegral);
            }
        }
    }

    /// <summary>Reads the kinds block over the vehicles just read: rebuilds each tracked or motorcycle definition from the wheeled one plus the saved scalars and validates it.</summary>
    private static void ReadVehicleKinds(StateReader r, VehicleBlock b)
    {
        int count = r.ReadCount(5, "vehicle kinds");
        if (count == 0) { throw r.Error("a vehicle kinds block holding no vehicles"); }
        int prev = -1;
        for (int k = 0; k < count; k++)
        {
            int v = r.ReadInt32();
            if (v <= prev || v >= b.Handles.HighWater || !b.Handles.IsLive(v)) { throw r.Error("invalid vehicle kind slot"); }
            prev = v;
            var s = b.States[v]!;
            byte kind = r.ReadByte();
            try
            {
                if (kind == (byte)VehicleKind.Tracked)
                {
                    float sr = r.ReadSingle(), ms = r.ReadSingle(), dt = r.ReadSingle(), bt = r.ReadSingle(), lf = r.ReadSingle();
                    int left = r.ReadInt32();
                    if (left < 1 || left > 16 || s.Wheels - left < 1 || s.Wheels - left > 16) { throw r.Error("invalid track split"); }
                    var t = new TrackedVehicleDef
                    {
                        LeftWheels = s.Def.Wheels[..left], RightWheels = s.Def.Wheels[left..], Grip = s.Def.Wheels[0].Tyre,
                        SprocketRadius = sr, MaxTrackSpeed = ms, MaxDriveTorque = dt, MaxBrakeTorque = bt, LateralFriction = lf,
                    };
                    t.Validate();
                    s.Kind = VehicleKind.Tracked; s.Track = t; s.TrackLeft = left;
                }
                else if (kind == (byte)VehicleKind.Motorcycle)
                {
                    if (s.Wheels != 2 || s.Def.Differentials.Length != 1) { throw r.Error("a motorcycle needs two wheels and one differential"); }
                    var m = new MotorcycleDef
                    {
                        Front = s.Def.Wheels[0], Rear = s.Def.Wheels[1], Engine = s.Def.Engine, Transmission = s.Def.Transmission, FinalDriveRatio = s.Def.Differentials[0].Ratio,
                        MaxSteerAngle = s.Def.Wheels[0].MaxSteerAngle,
                        HighSpeedSteerAngle = r.ReadSingle(), SteerFadeStartSpeed = r.ReadSingle(), SteerFadeEndSpeed = r.ReadSingle(), MaxLeanAngle = r.ReadSingle(), FallAngle = r.ReadSingle(),
                        LeanFrequency = r.ReadSingle(), LeanDamping = r.ReadSingle(), MaxLeanTorque = r.ReadSingle(),
                    };
                    m.Validate();
                    s.Lean = r.ReadSingle(); s.LeanTarget = r.ReadSingle(); s.LeanIntegral = r.ReadSingle();
                    if (!float.IsFinite(s.Lean + s.LeanTarget + s.LeanIntegral)) { throw r.Error("non-finite lean"); }
                    s.Kind = VehicleKind.Motorcycle; s.Bike = m;
                }
                else { throw r.Error("unknown vehicle kind " + kind.ToString(System.Globalization.CultureInfo.InvariantCulture)); }
            }
            catch (ArgumentException e) { throw r.Error("invalid vehicle kind parameters: " + e.Message); }
        }
    }
}
