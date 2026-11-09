using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>The saved and live state of one vehicle: the definition (copied at creation), the driver's inputs, the drivetrain and one record per wheel.</summary>
internal sealed class VehicleState
{
    public int Body, BodyGen;
    public VehicleDef Def = new();

    // Controller kind and its parameters (null for the kinds that do not use them).
    public VehicleKind Kind;
    public TrackedVehicleDef? Track;
    public MotorcycleDef? Bike;
    public float TrackBrake;   // tracked: brake input this step (not saved: set by the drive every step)

    // Inputs of the last Drive call.
    public float Forward, Pitch, Brake, Handbrake;

    // Drivetrain.
    public float EngineW;          // rad/s
    public int Gear;               // -1 reverse, 0 neutral, 1..N
    public float ShiftTimer;       // seconds left with the clutch open

    // Per wheel.
    public float[] Sprung = Array.Empty<float>();
    public float[] Omega = Array.Empty<float>(), Spin = Array.Empty<float>(), Length = Array.Empty<float>();
    public float[] SpringForce = Array.Empty<float>(), SlipRatio = Array.Empty<float>(), Fx = Array.Empty<float>();
    public float[] Nx = Array.Empty<float>(), Ny = Array.Empty<float>();
    public double[] Cx = Array.Empty<double>(), Cy = Array.Empty<double>();
    public byte[] Contact = Array.Empty<byte>();
    public int[] GroundBody = Array.Empty<int>(), GroundGen = Array.Empty<int>();

    // Derived (not saved).
    public float[] Curve = Array.Empty<float>(), Ratios = Array.Empty<float>();
    public Shape2D[] CastShapes = Array.Empty<Shape2D>();
    public int[] GroundShape = Array.Empty<int>();
    public float[] Torque = Array.Empty<float>(), Coef = Array.Empty<float>(), Compression = Array.Empty<float>(), Stop = Array.Empty<float>();

    /// <summary>Torque a rider adds to each wheel for the coming step, N·m, and a per-wheel brake input that replaces the vehicle's (negative: none). Set again every step by their owner, so neither is saved.</summary>
    public float[] ExtraTorque = Array.Empty<float>(), WheelBrake = Array.Empty<float>();

    public int Wheels => Def.Wheels.Length;

    public void Allocate()
    {
        int n = Def.Wheels.Length;
        Sprung = new float[n]; Omega = new float[n]; Spin = new float[n]; Length = new float[n];
        SpringForce = new float[n]; SlipRatio = new float[n]; Fx = new float[n];
        Nx = new float[n]; Ny = new float[n]; Cx = new double[n]; Cy = new double[n];
        Contact = new byte[n]; GroundBody = new int[n]; GroundGen = new int[n];
        Array.Fill(GroundBody, -1);
        Derive();
    }

    /// <summary>Rebuilds what follows from the definition: the torque curve, the gear ratios and the cast shapes.</summary>
    public void Derive()
    {
        int n = Def.Wheels.Length;
        Curve = Def.Engine.Curve; Ratios = Def.Transmission.Ratios;
        CastShapes = new Shape2D[n];
        for (int i = 0; i < n; i++) { CastShapes[i] = Shape2D.CreateCircle(new Vec2(0f, 0f), Def.Wheels[i].Radius); }
        GroundShape = new int[n]; Torque = new float[n]; Coef = new float[n]; Compression = new float[n]; Stop = new float[n];
        ExtraTorque = new float[n]; WheelBrake = new float[n]; Array.Fill(WheelBrake, -1f);
    }

    public VehicleState CloneForFork()
    {
        var c = (VehicleState)MemberwiseClone();
        c.Sprung = (float[])Sprung.Clone(); c.Omega = (float[])Omega.Clone(); c.Spin = (float[])Spin.Clone(); c.Length = (float[])Length.Clone();
        c.SpringForce = (float[])SpringForce.Clone(); c.SlipRatio = (float[])SlipRatio.Clone(); c.Fx = (float[])Fx.Clone();
        c.Nx = (float[])Nx.Clone(); c.Ny = (float[])Ny.Clone(); c.Cx = (double[])Cx.Clone(); c.Cy = (double[])Cy.Clone();
        c.Contact = (byte[])Contact.Clone(); c.GroundBody = (int[])GroundBody.Clone(); c.GroundGen = (int[])GroundGen.Clone();
        c.Curve = (float[])Curve.Clone(); c.Ratios = (float[])Ratios.Clone(); c.CastShapes = (Shape2D[])CastShapes.Clone();
        c.GroundShape = (int[])GroundShape.Clone(); c.Torque = (float[])Torque.Clone(); c.Coef = (float[])Coef.Clone();
        c.Compression = (float[])Compression.Clone(); c.Stop = (float[])Stop.Clone();
        c.ExtraTorque = (float[])ExtraTorque.Clone(); c.WheelBrake = (float[])WheelBrake.Clone();
        c.Def = Def.Clone(); c.Track = Track?.Clone(); c.Bike = Bike?.Clone();
        return c;
    }
}

internal sealed partial class Engine2D
{
    private const float TwoPi = 6.2831855f;

    /// <summary>Slots of the vehicles (handles with generations, lowest free slot first).</summary>
    public HandleTable VehHandles = new();

    public VehicleState?[] Vehicles = new VehicleState?[4];

    private int[] _vehOfBody = Array.Empty<int>();
    private int _castChassis = -1;
    private Func<Shape, bool>? _vehiclePredicate;

    public bool HasVehicles => VehHandles.Count > 0;

    public int CreateVehicle(BodyHandle body, VehicleDef def, TrackedVehicleDef? track = null, MotorcycleDef? bike = null)
    {
        int b = Core.Resolve(body);
        if (Core.Bodies.TypeOf(b) != BodyType.Dynamic) { throw new ArgumentException("A vehicle's chassis must be a dynamic body.", nameof(body)); }
        if (VehicleOfBody(b) >= 0) { throw new PhysicsException("The body already carries a vehicle."); }
        def.Validate();
        var s = new VehicleState { Body = body.Index, BodyGen = body.Generation, Def = def.Clone() };
        if (track != null) { s.Kind = VehicleKind.Tracked; s.Track = track.Clone(); }
        else if (bike != null) { s.Kind = VehicleKind.Motorcycle; s.Bike = bike.Clone(); }
        s.Allocate();
        float mass = Mass[b];
        for (int i = 0; i < s.Wheels; i++)
        {
            var w = s.Def.Wheels[i];
            s.Sprung[i] = w.SprungMass > 0f ? w.SprungMass : mass / s.Wheels;
            s.Length[i] = w.SuspensionMaxLength;
        }

        s.Gear = 1;
        s.EngineW = s.Def.Engine.MinRpm * (TwoPi / 60f);
        int v = VehHandles.Allocate();
        if (v >= Vehicles.Length) { Array.Resize(ref Vehicles, Math.Max(v + 1, Vehicles.Length * 2)); }
        Vehicles[v] = s;
        IndexVehicleBody(b, v);
        return v;
    }

    private void IndexVehicleBody(int body, int vehicle)
    {
        if (body >= _vehOfBody.Length)
        {
            int old = _vehOfBody.Length;
            Array.Resize(ref _vehOfBody, Math.Max(body + 1, old * 2));
            for (int i = old; i < _vehOfBody.Length; i++) { _vehOfBody[i] = -1; }
        }

        _vehOfBody[body] = vehicle;
    }

    /// <summary>The vehicle slot whose chassis is the given body slot, or -1.</summary>
    public int VehicleOfBody(int body) => body < _vehOfBody.Length ? _vehOfBody[body] : -1;

    public void DestroyVehicle(int v)
    {
        var s = Vehicles[v]!;
        if (s.Body < _vehOfBody.Length) { _vehOfBody[s.Body] = -1; }
        Vehicles[v] = null;
        VehHandles.Free(v);
    }

    /// <summary>Called when a body is destroyed: its vehicle, if it is the chassis of one, goes with it.</summary>
    public void DropVehicleOfBody(int body)
    {
        int v = VehicleOfBody(body);
        if (v >= 0) { DestroyVehicle(v); }
    }

    // ------------------------------------------------------------------ the step

    private static float Clamp01(float x) => x < 0f ? 0f : x > 1f ? 1f : x;

    private float EffInv(int body, float rx, float ry, float ux, float uy)
    {
        var bodies = Core.Bodies;
        float m = bodies.InverseMass[body];
        if (!(m > 0f)) { return 0f; }
        float c = rx * uy - ry * ux;
        return m + bodies.InverseInertia[body] * c * c;
    }

    private void ApplyImpulseAtArm(int body, float rx, float ry, float jx, float jy)
    {
        var bodies = Core.Bodies;
        float m = bodies.InverseMass[body];
        if (!(m > 0f)) { return; }
        var v = bodies.LinearVelocityOf(body);
        v[0] += m * jx; v[1] += m * jy;
        bodies.AngularVelocityOf(body)[0] += bodies.InverseInertia[body] * (rx * jy - ry * jx);
    }

    private void PointVelocity(int body, float rx, float ry, out float vx, out float vy)
    {
        var bodies = Core.Bodies;
        var v = bodies.LinearVelocityOf(body);
        float w = bodies.AngularVelocityOf(body)[0];
        vx = v[0] - w * ry; vy = v[1] + w * rx;
    }

    private void WakeBodyIsland(int b)
    {
        var bodies = Core.Bodies;
        if (bodies.IsDynamic(b)) { Core.WakeIsland(bodies.Island[b]); bodies.SleepTime[b] = 0f; }
    }

    /// <summary>
    /// Runs every vehicle once, before the world steps, in slot order: casts the wheels, then applies each wheel's suspension and tyre impulses to the chassis (and to a dynamic ground body) and advances the drivetrain.
    /// The impulses are sequential, in wheel order, from the velocities as they are at that moment, so the result does not depend on the thread count; the solver then integrates and resolves contacts as usual.
    /// </summary>
    public void StepVehicles(PhysicsWorld2D world, float dt)
    {
        if (VehHandles.Count == 0) { return; }
        _vehiclePredicate ??= sh => Core.Shapes.Owner[sh.Handle.Index] != _castChassis;
        for (int v = 0; v < VehHandles.HighWater; v++)
        {
            if (!VehHandles.IsLive(v)) { continue; }
            StepVehicle(world, Vehicles[v]!, dt);
        }

        _castChassis = -1;
    }

    private static float Magic(float b, float c, float e, float x)
    {
        // Pacejka's magic formula, normalised to a peak of about 1: sin(C atan(B x - E (B x - atan(B x)))).
        float bx = b * x;
        float inner = bx - e * (bx - Math32.Atan(bx));
        return Math32.Sin(c * Math32.Atan(inner));
    }

    private void StepVehicle(PhysicsWorld2D world, VehicleState s, float h)
    {
        var bodies = Core.Bodies;
        int b = s.Body;
        if (!bodies.IsAwake(b))
        {
            if (s.Forward == 0f && s.Brake == 0f && s.Handbrake == 0f && s.Pitch == 0f) { return; }
            WakeBodyIsland(b);
        }
        else if (s.Forward != 0f || s.Brake > 0f || s.Handbrake > 0f || s.Pitch != 0f) { bodies.SleepTime[b] = 0f; }

        // The wheel forces below are timed for the world's own substep count, so the chassis' island runs that schedule.
        Core.KeepJointSchedule(b);
        var def = s.Def;
        int n = s.Wheels;
        var rotB = bodies.RotationOf(b);
        float qc = rotB[0], qs = rotB[1];
        float lcx = LocalCenter[2 * b], lcy = LocalCenter[2 * b + 1];
        var pp = bodies.PositionOf(b);
        double px = pp[0], py = pp[1];
        _castChassis = b;
        var filter = new QueryFilter2D { Predicate = _vehiclePredicate };

        // Drivetrain first: wheel torques from the engine, from the speeds as they were at the end of the last step.
        var f0 = def.Wheels[0].ForwardDirection;
        float fwx = qc * f0.X - qs * f0.Y, fwy = qs * f0.X + qc * f0.Y;
        var lin0 = bodies.LinearVelocityOf(b);
        float forwardSpeed = lin0[0] * fwx + lin0[1] * fwy;
        float brakeInput = s.Kind == VehicleKind.Tracked ? TrackDrive(s, h) : DriveTrain(s, h, forwardSpeed);

        // Phase 1: sweep every wheel.
        bool anyContact = false;
        for (int i = 0; i < n; i++)
        {
            var wd = def.Wheels[i];
            float tx = wd.Position.X - lcx, ty = wd.Position.Y - lcy;
            double topX = px + (double)(qc * tx - qs * ty), topY = py + (double)(qs * tx + qc * ty);
            float dx = qc * wd.SuspensionDirection.X - qs * wd.SuspensionDirection.Y, dy = qs * wd.SuspensionDirection.X + qc * wd.SuspensionDirection.Y;
            // A chassis just inside the supported range can carry a wheel top just outside it; the public query rejects such a start, and a step must not fail half way, so that wheel finds no ground.
            double lim = BoxLimits.MaxCoordinate;
            bool inRange = Math.Abs(topX) <= lim && Math.Abs(topY) <= lim;   // NaN fails
            var hh = default(QueryHit2D);
            bool hit = inRange && world.ShapeCast(in s.CastShapes[i], new Vector2D(topX, topY), 0f, new Vector2F(dx, dy), out hh, wd.SuspensionMaxLength + 1e-4f, in filter);
            s.Contact[i] = 0;
            s.Compression[i] = 0f; s.Stop[i] = 0f;
            if (hit)
            {
                float nx = hh.Normal.X, ny = hh.Normal.Y;
                if (!(nx * nx + ny * ny > 0.25f)) { nx = -dx; ny = -dy; }   // started overlapping: no usable normal, push straight up the suspension
                float cosUp = -(nx * dx + ny * dy);
                if (cosUp >= 0.05f)
                {
                    float t = hh.Distance;
                    s.Contact[i] = 1; anyContact = true;
                    s.Length[i] = Math.Min(wd.SuspensionMaxLength, Math.Max(t, wd.SuspensionMinLength));
                    s.Compression[i] = (wd.SuspensionMaxLength - s.Length[i]) + wd.PreloadLength;
                    s.Stop[i] = Math.Max(0f, wd.SuspensionMinLength - t);
                    s.Nx[i] = nx; s.Ny[i] = ny;
                    if (t > 1e-5f) { s.Cx[i] = hh.Point.X; s.Cy[i] = hh.Point.Y; }
                    else { s.Cx[i] = topX + (double)(dx * (t + wd.Radius)); s.Cy[i] = topY + (double)(dy * (t + wd.Radius)); }
                    int gb = hh.Body.Handle.Index;
                    s.GroundBody[i] = gb; s.GroundGen[i] = bodies.Handles.GenerationOf(gb);
                    s.GroundShape[i] = hh.Shape.Handle.Index;
                }
            }

            if (s.Contact[i] == 0)
            {
                s.Length[i] = wd.SuspensionMaxLength;
                s.GroundBody[i] = -1; s.GroundGen[i] = 0;
            }
        }

        // Pitch control: the rider's weight shift on the ground, air control off it. A torque on the chassis alone.
        float pitchTorque = anyContact ? def.GroundPitchTorque : def.AirPitchTorque;
        if (s.Pitch != 0f && pitchTorque > 0f && bodies.InverseInertia[b] > 0f)
        {
            float face = f0.X < 0f ? -1f : 1f;   // a vehicle facing -X lifts its nose by turning the other way
            bodies.AngularVelocityOf(b)[0] += bodies.InverseInertia[b] * (h * pitchTorque * s.Pitch * face);
        }

        // Phase 2: forces.
        int substeps = Math.Max(1, Opt.Substeps);
        float gravityLead = h * (substeps - 1) / (2f * substeps);
        for (int i = 0; i < n; i++)
        {
            var wd = def.Wheels[i];
            float r = wd.Radius;
            float omega = s.Omega[i];
            float inv = 1f / wd.Inertia;
            // Wheel speed after the drive torque and the brakes; the tyre then acts on this speed (a split step that stays stable for any torque).
            float driveTorque = s.Torque[i];
            if (s.ExtraTorque[i] != 0f) { driveTorque += s.ExtraTorque[i]; }
            float ws = omega + h * driveTorque * inv;
            float bi = s.Kind == VehicleKind.Tracked ? s.TrackBrake : brakeInput;
            if (s.WheelBrake[i] >= 0f) { bi = s.WheelBrake[i]; }
            float brakeT = bi * wd.MaxBrakeTorque + s.Handbrake * wd.MaxHandbrakeTorque;
            if (brakeT > 0f)
            {
                float dwb = Math.Min(h * brakeT * inv, Math.Abs(ws));
                ws = ws > 0f ? ws - dwb : ws + dwb;
            }

            ws /= 1f + h * wd.AngularDamping;
            // A brake that has stopped the wheel holds it against the tyre up to its torque: the tyre force then sees the whole chassis, not just the wheel's spin-up.
            bool locked = brakeT > 0f && ws == 0f;
            s.Fx[i] = 0f; s.SlipRatio[i] = 0f;
            if (s.Contact[i] == 0)
            {
                s.SpringForce[i] = 0f;
                ws = SaneSpin(ws);
                s.Omega[i] = ws; s.Spin[i] = WrapAngle(s.Spin[i] + ws * h);
                continue;
            }

            int groundShape = s.GroundShape[i];
            int gb = s.GroundBody[i];
            float cpx = (float)(s.Cx[i] - px), cpy = (float)(s.Cy[i] - py);
            var gp = bodies.PositionOf(gb);
            float gx0 = (float)(s.Cx[i] - gp[0]), gy0 = (float)(s.Cy[i] - gp[1]);
            float nx = s.Nx[i], ny = s.Ny[i];

            // Suspension: a spring (explicit, capped by what restores the wheel in one step) and a damper (implicit in the effective mass), along the contact normal, pushing only.
            // Catto, "Soft Constraints" (GDC 2011) for the implicit damper.
            float invN = EffInv(b, cpx, cpy, nx, ny) + EffInv(gb, gx0, gy0, nx, ny);
            float meffN = invN > 0f ? 1f / invN : 0f;
            PointVelocity(b, cpx, cpy, out float vax, out float vay);
            PointVelocity(gb, gx0, gy0, out float vbx, out float vby);
            float relx = vax - vbx, rely = vay - vby;
            // The velocity at the start of a step carries the sawtooth of gravity applied in substeps; positions follow the mean velocity, so the damper reads that one.
            float gs = GravityScale[b] * gravityLead;
            float closing = -((relx - Opt.Gravity.X * gs) * nx + (rely - Opt.Gravity.Y * gs) * ny);
            float k = s.Sprung[i] * (TwoPi * wd.SpringFrequency) * (TwoPi * wd.SpringFrequency);
            float c = 2f * wd.DampingRatio * s.Sprung[i] * (TwoPi * wd.SpringFrequency);
            float comp = s.Compression[i];
            float spring = h * k * comp;
            if (meffN > 0f) { spring = Math.Min(spring, meffN * comp / h); }
            float damp = h * c * closing / (1f + h * c * invN);
            float pn = spring + damp;
            if (s.Stop[i] > 0f && meffN > 0f) { pn += meffN * Math.Max(0f, closing + 0.2f * s.Stop[i] / h); }
            if (pn < 0f) { pn = 0f; }
            s.SpringForce[i] = pn / h;
            if (pn > 0f)
            {
                ApplyImpulseAtArm(b, cpx, cpy, nx * pn, ny * pn);
                if (gb != b && bodies.InverseMass[gb] > 0f) { ApplyImpulseAtArm(gb, gx0, gy0, -nx * pn, -ny * pn); WakeBodyIsland(gb); }
            }

            // Tyre.
            float load = pn / h;
            float wheelOmegaAfter = ws;
            if (load > 0f)
            {
                var tyre = wd.Tyre;
                var fd = wd.ForwardDirection;
                float fwxw = qc * fd.X - qs * fd.Y, fwyw = qs * fd.X + qc * fd.Y;
                float dn = fwxw * nx + fwyw * ny;
                float fdx = fwxw - nx * dn, fdy = fwyw - ny * dn;
                float fl = fdx * fdx + fdy * fdy;
                if (fl > 1e-8f)
                {
                    float il = 1f / float.Sqrt(fl);
                    fdx *= il; fdy *= il;
                    PointVelocity(b, cpx, cpy, out vax, out vay);
                    PointVelocity(gb, gx0, gy0, out vbx, out vby);
                    float vlong = (vax - vbx) * fdx + (vay - vby) * fdy;
                    float vref = Math.Max(Math.Abs(vlong), tyre.LowSpeed);
                    float vsx = ws * r - vlong;
                    float sx = vsx / vref;
                    s.SlipRatio[i] = sx;
                    float gx = Magic(tyre.LongitudinalB, tyre.LongitudinalC, tyre.LongitudinalE, sx);
                    float mu = float.Sqrt(tyre.Friction * Friction[groundShape]);
                    float fxw = mu * load * gx;
                    // Implicit limit: a force never reverses the slip velocity it acts on within the step.
                    float invF = EffInv(b, cpx, cpy, fdx, fdy) + EffInv(gb, gx0, gy0, fdx, fdy);
                    float maxX = Math.Abs(vsx) / (h * (invF + (locked ? 0f : r * r * inv)));
                    if (Math.Abs(fxw) > maxX) { fxw = fxw < 0f ? -maxX : maxX; }
                    // A braked wheel passes at most what its brake can hold: a tyre force above brake torque over radius would spin the wheel forward. Cars keep the whole friction budget (a stand-in for the chassis' weight on the wheel).
                    if (brakeT > 0f && s.Kind != VehicleKind.Wheeled) { float hold = brakeT / r; if (Math.Abs(fxw) > hold) { fxw = fxw < 0f ? -hold : hold; } }
                    s.Fx[i] = fxw;
                    float jx = fdx * (h * fxw), jy = fdy * (h * fxw);
                    ApplyImpulseAtArm(b, cpx, cpy, jx, jy);
                    if (gb != b && bodies.InverseMass[gb] > 0f) { ApplyImpulseAtArm(gb, gx0, gy0, -jx, -jy); WakeBodyIsland(gb); }
                    wheelOmegaAfter = ws - h * fxw * r * inv;
                    if (locked)
                    {
                        float net = -h * fxw * r, hold = h * brakeT;
                        wheelOmegaAfter = Math.Abs(net) <= hold ? 0f : (net - (net < 0f ? -hold : hold)) * inv;
                    }
                }
            }

            wheelOmegaAfter = SaneSpin(wheelOmegaAfter);
            s.Omega[i] = wheelOmegaAfter;
            s.Spin[i] = WrapAngle(s.Spin[i] + wheelOmegaAfter * h);
        }

        if (s.Kind == VehicleKind.Tracked) { CoupleTracks(s); }
    }

    /// <summary>The fastest a wheel is allowed to turn, rad/s: far beyond anything a drive reaches, and the bound the snapshot reader holds the saved speeds to.</summary>
    private const float MaxWheelSpin = 1e9f;

    /// <summary>Keeps a wheel speed finite and within <see cref="MaxWheelSpin"/> (NaN becomes 0), so the state a step writes is always one the reader accepts. A speed inside the bound passes unchanged.</summary>
    private static float SaneSpin(float w) => w > MaxWheelSpin ? MaxWheelSpin : w < -MaxWheelSpin ? -MaxWheelSpin : float.IsNaN(w) ? 0f : w;

    private static float WrapAngle(float a)
    {
        const float Pi = 3.1415927f;
        if (a > Pi) { a -= TwoPi; }
        else if (a < -Pi) { a += TwoPi; }
        return a;
    }

    // ------------------------------------------------------------------ drivetrain

    /// <summary>Advances engine, clutch and gearbox for one step and fills <see cref="VehicleState.Torque"/>; returns the brake input to apply (the driver's, or more when the drive input opposes the motion).</summary>
    private static float DriveTrain(VehicleState s, float h, float forwardSpeed)
    {
        var def = s.Def;
        var eng = def.Engine;
        var tr = def.Transmission;
        int n = s.Wheels;
        int gears = s.Ratios.Length;
        float brake = s.Brake;
        float fwd = s.Forward;
        float throttle = 0f;
        if (s.ShiftTimer > 0f) { s.ShiftTimer = Math.Max(0f, s.ShiftTimer - h); }

        if (tr.Automatic)
        {
            if (fwd > 0f)
            {
                if (s.Gear <= 0)
                {
                    if (forwardSpeed > -1f) { SetGear(s, 1, tr); } else { brake = Math.Max(brake, fwd); }
                }

                if (s.Gear > 0) { throttle = fwd; }
            }
            else if (fwd < 0f)
            {
                if (s.Gear >= 0)
                {
                    if (forwardSpeed > 1f) { brake = Math.Max(brake, -fwd); }
                    else { SetGear(s, -1, tr); }
                }

                if (s.Gear < 0) { throttle = -fwd; }
            }
            else if (s.Gear == 0) { SetGear(s, 1, tr); }

            if (s.Gear > 0 && s.ShiftTimer <= 0f)
            {
                // The box shifts on the speed the driven wheels imply for the engine in the current gear, not on the engine's own speed: with the clutch open the engine revs freely, and shifting on that would run
                // through every gear one after the other, each shift opening the clutch again.
                float rpm = ImpliedEngineW(s) * (60f / TwoPi);
                if (rpm > tr.ShiftUpRpm && s.Gear < gears && throttle > 0f) { SetGear(s, s.Gear + 1, tr); }
                else if (rpm < tr.ShiftDownRpm && s.Gear > 1) { SetGear(s, s.Gear - 1, tr); }
            }
        }
        else
        {
            if (s.Gear > 0) { throttle = Math.Max(fwd, 0f); if (fwd < 0f) { brake = Math.Max(brake, -fwd); } }
            else if (s.Gear < 0) { throttle = Math.Max(-fwd, 0f); if (fwd > 0f) { brake = Math.Max(brake, fwd); } }
        }

        // Gear ratio, signed.
        float gr = s.Gear > 0 ? s.Ratios[s.Gear - 1] : s.Gear < 0 ? -tr.ReverseRatio : 0f;
        float engage = s.ShiftTimer > 0f ? 0f : 1f;

        // Torque shares of the driven wheels (with limited-slip bias), their gain from the clutch shaft and the shaft speed they imply.
        float totalShare = 0f;
        for (int d = 0; d < def.Differentials.Length; d++) { totalShare += def.Differentials[d].EngineTorqueRatio; }
        for (int i = 0; i < n; i++) { s.Coef[i] = 0f; s.Torque[i] = 0f; }
        float wShaft = 0f, invIn = 0f;
        if (totalShare > 0f && gr != 0f)
        {
            for (int d = 0; d < def.Differentials.Length; d++)
            {
                var df = def.Differentials[d];
                float er = df.EngineTorqueRatio / totalShare;
                if (!(er > 0f)) { continue; }
                float sl, sr;
                if (df.Left >= 0 && df.Right >= 0)
                {
                    sl = df.LeftRightSplit; sr = 1f - sl;
                    if (df.LimitedSlipRatio > 1f)
                    {
                        float wl = Math.Abs(s.Omega[df.Left]), wr = Math.Abs(s.Omega[df.Right]);
                        float bias = Clamp01(Math.Abs(wl - wr) * 2f);
                        float target = df.LimitedSlipRatio / (1f + df.LimitedSlipRatio);
                        if (wl >= wr) { sr += bias * Math.Max(0f, target - sr); sl = 1f - sr; }
                        else { sl += bias * Math.Max(0f, target - sl); sr = 1f - sl; }
                    }
                }
                else if (df.Left >= 0) { sl = 1f; sr = 0f; }
                else { sl = 0f; sr = 1f; }

                float g = gr * df.Ratio * er;
                if (df.Left >= 0) { s.Coef[df.Left] = g * sl; }
                if (df.Right >= 0) { s.Coef[df.Right] = g * sr; }
            }

            for (int i = 0; i < n; i++)
            {
                float ci = s.Coef[i];
                if (ci == 0f) { continue; }
                wShaft += ci * s.Omega[i];
                invIn += ci * ci / def.Wheels[i].Inertia;
            }
        }

        float wMax = eng.MaxRpm * (TwoPi / 60f), wIdle = eng.MinRpm * (TwoPi / 60f);
        float we = s.EngineW;
        // Engine torque: the curve at the current speed, cut smoothly at the limit; engine braking with the throttle closed; an idle controller that holds the idle speed when the clutch lets go.
        float x = we / wMax * (s.Curve.Length - 1);
        int i0 = x >= s.Curve.Length - 1 ? s.Curve.Length - 2 : (int)x;
        float fr = x - i0; if (fr > 1f) { fr = 1f; }
        float curve = s.Curve[i0] + (s.Curve[i0 + 1] - s.Curve[i0]) * fr;
        float limiter = Clamp01((wMax - we) / (0.02f * wMax));
        float te = eng.MaxTorque * curve * throttle * limiter;
        te -= eng.EngineBraking * eng.MaxTorque * Clamp01(we / wMax) * (1f - throttle);
        te += 0.3f * eng.MaxTorque * Clamp01((wIdle - we) / (0.2f * wIdle));

        // Centrifugal-style clutch: it carries nothing at idle speed and everything from 1.5 times idle, so the car neither creeps nor stalls the engine.
        float cap = eng.ClutchStrength * eng.MaxTorque * engage * Clamp01((we - wIdle) / (0.5f * wIdle));
        float tc = 0f;
        if (invIn > 0f && cap > 0f)
        {
            // Torque that would bring engine and shaft to the same speed within the step; the clutch carries at most that, and at most its capacity.
            float teq = (we - wShaft) / (h * (1f / eng.Inertia + invIn));
            tc = teq < -cap ? -cap : teq > cap ? cap : teq;
            for (int i = 0; i < n; i++) { s.Torque[i] = tc * s.Coef[i]; }
        }

        we += h * (te - tc) / eng.Inertia;
        if (!(we >= 0f)) { we = 0f; }   // an engine never turns backwards; this also keeps a NaN out of the saved state
        float top = 1.2f * wMax;
        if (we > top) { we = top; }
        s.EngineW = we;
        return brake;
    }

    /// <summary>The engine speed the driven wheels turn in the current gear if the clutch were locked, rad/s: the torque-weighted mean wheel speed of the differentials through their ratios and the gear.</summary>
    private static float ImpliedEngineW(VehicleState s)
    {
        var def = s.Def;
        float gr = s.Gear > 0 ? s.Ratios[s.Gear - 1] : 0f;
        float sum = 0f, share = 0f;
        for (int d = 0; d < def.Differentials.Length; d++)
        {
            var df = def.Differentials[d];
            float w = df.Left >= 0 && df.Right >= 0 ? 0.5f * (Math.Abs(s.Omega[df.Left]) + Math.Abs(s.Omega[df.Right])) : df.Left >= 0 ? Math.Abs(s.Omega[df.Left]) : Math.Abs(s.Omega[df.Right]);
            sum += df.EngineTorqueRatio * w * df.Ratio * gr; share += df.EngineTorqueRatio;
        }

        return share > 0f ? sum / share : 0f;
    }

    private static void SetGear(VehicleState s, int gear, TransmissionDef tr)
    {
        if (s.Gear == gear) { return; }
        s.Gear = gear;
        s.ShiftTimer = tr.SwitchTime;
    }

    // ------------------------------------------------------------------ tracked vehicles

    /// <summary>
    /// Fills the road wheels' drive torques and returns the brake input. The track is a rigid belt over its road wheels: the sprocket pushes the belt towards the commanded speed with at most its torque,
    /// so the belt force is the belt's inertia over the step capped at <c>MaxDriveTorque / SprocketRadius</c>.
    /// </summary>
    private static float TrackDrive(VehicleState s, float h)
    {
        var t = s.Track!;
        int n = s.Wheels;
        float cmd = s.Forward * (1f - s.Brake);   // the brake overrides the drive
        // A belt that is not commanded is held by its brake, as a tank's parking brake holds it on a slope; a driven belt releases.
        s.TrackBrake = cmd == 0f ? 1f : s.Brake;
        float fmax = t.MaxDriveTorque / t.SprocketRadius;
        float target = cmd * t.MaxTrackSpeed;
        float mt = 0f, vm = 0f;
        for (int i = 0; i < n; i++)
        {
            float r = s.Def.Wheels[i].Radius;
            float mi = s.Def.Wheels[i].Inertia / (r * r);   // the wheel's inertia seen along the belt
            mt += mi; vm += mi * s.Omega[i] * r;
        }

        float vk = vm / mt;
        float f = mt * (target - vk) / h;
        if (f > fmax) { f = fmax; } else if (f < -fmax) { f = -fmax; }
        for (int i = 0; i < n; i++)
        {
            float r = s.Def.Wheels[i].Radius;
            s.Torque[i] = f * (s.Def.Wheels[i].Inertia / (r * r)) / mt * r;
        }

        return s.Brake;
    }

    /// <summary>After the tyres have acted: the belt is rigid, so the wheels share one speed (inertia-weighted), limited to the track speed limit.</summary>
    private static void CoupleTracks(VehicleState s)
    {
        var t = s.Track!;
        float v = TrackSpeedOf(s);
        if (v > t.MaxTrackSpeed) { v = t.MaxTrackSpeed; } else if (v < -t.MaxTrackSpeed) { v = -t.MaxTrackSpeed; }
        for (int i = 0; i < s.Wheels; i++) { s.Omega[i] = v / s.Def.Wheels[i].Radius; }
    }

    /// <summary>Belt speed as of the last step, m/s; positive moves the hull forward.</summary>
    public static float TrackSpeedOf(VehicleState s)
    {
        float mt = 0f, vm = 0f;
        for (int i = 0; i < s.Wheels; i++)
        {
            float r = s.Def.Wheels[i].Radius;
            float mi = s.Def.Wheels[i].Inertia / (r * r);
            mt += mi; vm += mi * s.Omega[i] * r;
        }

        return vm / mt;
    }

    // ------------------------------------------------------------------ origin shift

    /// <summary>After the core shifted the origin: the stored contact points are world positions, so they move with it.</summary>
    public void ShiftVehicles(ReadOnlySpan<double> offset)
    {
        for (int v = 0; v < VehHandles.HighWater; v++)
        {
            if (!VehHandles.IsLive(v)) { continue; }
            var s = Vehicles[v]!;
            for (int i = 0; i < s.Wheels; i++) { s.Cx[i] -= offset[0]; s.Cy[i] -= offset[1]; }
        }
    }

    // ------------------------------------------------------------------ debug

    /// <summary>Emits one entry per wheel of every live vehicle, vehicle slot then wheel index (z = 0, axle along +Z).</summary>
    public void AddDebugVehicles(Debug.DebugGeometry g)
    {
        var bodies = Core.Bodies;
        for (int v = 0; v < VehHandles.HighWater; v++)
        {
            if (!VehHandles.IsLive(v)) { continue; }
            var s = Vehicles[v]!;
            int b = s.Body;
            var rot = bodies.RotationOf(b);
            float qc = rot[0], qs = rot[1];
            float lcx = LocalCenter[2 * b], lcy = LocalCenter[2 * b + 1];
            var pp = bodies.PositionOf(b);
            for (int i = 0; i < s.Wheels; i++)
            {
                var wd = s.Def.Wheels[i];
                float tx = wd.Position.X - lcx, ty = wd.Position.Y - lcy;
                double topX = pp[0] + (double)(qc * tx - qs * ty), topY = pp[1] + (double)(qs * tx + qc * ty);
                float dx = qc * wd.SuspensionDirection.X - qs * wd.SuspensionDirection.Y, dy = qs * wd.SuspensionDirection.X + qc * wd.SuspensionDirection.Y;
                var top = new Vector3D(topX, topY, 0.0);
                var center = new Vector3D(topX + (double)(dx * s.Length[i]), topY + (double)(dy * s.Length[i]), 0.0);
                bool contact = s.Contact[i] != 0;
                g.Add(new Debug.DebugWheel(Core.HandleOfBody(b), i, top, center, wd.Radius, 0f, new Vector3F(0f, 0f, 1f), contact,
                    contact ? new Vector3D(s.Cx[i], s.Cy[i], 0.0) : default, contact ? new Vector3F(s.Nx[i], s.Ny[i], 0f) : default));
            }
        }
    }

    // ------------------------------------------------------------------ snapshot

    private static void WriteVec2F(StateWriter w, Vector2F v) { w.NamedSingle(v.X); w.NamedSingle(v.Y); }

    private static Vector2F ReadVec2F(StateReader r) => new(r.ReadSingle(), r.ReadSingle());

    internal void WriteVehicles(StateWriter w)
    {
        w.NamedInt32(1);   // section version
        w.DescribeTable("vehicle");
        VehHandles.Write(w);
        for (int v = 0; v < VehHandles.HighWater; v++)
        {
            w.DescribeEntity("vehicle", v, VehHandles.GenerationOf(v));
            if (!VehHandles.IsLive(v)) { continue; }
            var s = Vehicles[v]!;
            w.NamedInt32(s.Body); w.NamedInt32(s.BodyGen); w.NamedByte((byte)s.Kind);
            WriteVehicleDef(w, s.Def);
            if (s.Kind == VehicleKind.Tracked)
            {
                var t = s.Track!;
                w.NamedSingle(t.SprocketRadius); w.NamedSingle(t.MaxTrackSpeed); w.NamedSingle(t.MaxDriveTorque); w.NamedSingle(t.MaxBrakeTorque);
            }
            else if (s.Kind == VehicleKind.Motorcycle) { w.NamedSingle(s.Bike!.LeanTorque); }

            w.NamedSingle(s.Forward); w.NamedSingle(s.Pitch); w.NamedSingle(s.Brake); w.NamedSingle(s.Handbrake);
            w.NamedSingle(s.EngineW); w.NamedInt32(s.Gear); w.NamedSingle(s.ShiftTimer);
            int n = s.Wheels;
            for (int i = 0; i < n; i++)
            {
                w.NamedSingle(s.Sprung[i]); w.NamedSingle(s.Omega[i]); w.NamedSingle(s.Spin[i]); w.NamedSingle(s.Length[i]);
                w.NamedSingle(s.SpringForce[i]); w.NamedSingle(s.SlipRatio[i]); w.NamedSingle(s.Fx[i]);
                w.NamedSingle(s.Nx[i]); w.NamedSingle(s.Ny[i]);
                w.NamedDouble(s.Cx[i]); w.NamedDouble(s.Cy[i]);
                w.NamedByte(s.Contact[i]); w.NamedInt32(s.GroundBody[i]); w.NamedInt32(s.GroundGen[i]);
            }
        }
    }

    private static void WriteVehicleDef(StateWriter w, VehicleDef d)
    {
        w.NamedInt32(d.Wheels.Length);
        foreach (var wd in d.Wheels)
        {
            WriteVec2F(w, wd.Position); WriteVec2F(w, wd.SuspensionDirection); WriteVec2F(w, wd.ForwardDirection);
            w.NamedSingle(wd.Radius); w.NamedSingle(wd.SuspensionMinLength); w.NamedSingle(wd.SuspensionMaxLength); w.NamedSingle(wd.SpringFrequency);
            w.NamedSingle(wd.DampingRatio); w.NamedSingle(wd.PreloadLength); w.NamedSingle(wd.SprungMass); w.NamedSingle(wd.Inertia); w.NamedSingle(wd.AngularDamping);
            w.NamedSingle(wd.MaxBrakeTorque); w.NamedSingle(wd.MaxHandbrakeTorque);
            var t = wd.Tyre;
            w.NamedSingle(t.Friction); w.NamedSingle(t.LongitudinalB); w.NamedSingle(t.LongitudinalC); w.NamedSingle(t.LongitudinalE); w.NamedSingle(t.LowSpeed);
        }

        var e = d.Engine;
        w.NamedSingle(e.MaxTorque); w.NamedSingle(e.MinRpm); w.NamedSingle(e.MaxRpm); w.NamedSingle(e.Inertia); w.NamedSingle(e.EngineBraking); w.NamedSingle(e.ClutchStrength);
        var curve = e.Curve;
        w.NamedInt32(curve.Length); w.NamedSingles(curve);
        var tr = d.Transmission;
        w.NamedBool(tr.Automatic); w.NamedSingle(tr.ReverseRatio); w.NamedSingle(tr.SwitchTime); w.NamedSingle(tr.ShiftUpRpm); w.NamedSingle(tr.ShiftDownRpm);
        var ratios = tr.Ratios;
        w.NamedInt32(ratios.Length); w.NamedSingles(ratios);
        w.NamedInt32(d.Differentials.Length);
        foreach (var df in d.Differentials)
        {
            w.NamedInt32(df.Left); w.NamedInt32(df.Right); w.NamedSingle(df.Ratio); w.NamedSingle(df.LeftRightSplit); w.NamedSingle(df.LimitedSlipRatio); w.NamedSingle(df.EngineTorqueRatio);
        }

        w.NamedSingle(d.AirPitchTorque); w.NamedSingle(d.GroundPitchTorque);
    }

    private static VehicleDef ReadVehicleDef(StateReader r)
    {
        var d = new VehicleDef();
        int nw = r.ReadCount(88, "vehicle wheels");
        if (nw > 32) { throw r.Error("a vehicle with more than 32 wheels"); }
        d.Wheels = new WheelDef[nw];
        for (int i = 0; i < nw; i++)
        {
            var wd = new WheelDef
            {
                Position = ReadVec2F(r), SuspensionDirection = ReadVec2F(r), ForwardDirection = ReadVec2F(r),
                Radius = r.ReadSingle(), SuspensionMinLength = r.ReadSingle(), SuspensionMaxLength = r.ReadSingle(), SpringFrequency = r.ReadSingle(),
                DampingRatio = r.ReadSingle(), PreloadLength = r.ReadSingle(), SprungMass = r.ReadSingle(), Inertia = r.ReadSingle(), AngularDamping = r.ReadSingle(),
                MaxBrakeTorque = r.ReadSingle(), MaxHandbrakeTorque = r.ReadSingle(),
            };
            wd.Tyre = new TyreDef { Friction = r.ReadSingle(), LongitudinalB = r.ReadSingle(), LongitudinalC = r.ReadSingle(), LongitudinalE = r.ReadSingle(), LowSpeed = r.ReadSingle() };
            d.Wheels[i] = wd;
        }

        d.Engine = new EngineDef { MaxTorque = r.ReadSingle(), MinRpm = r.ReadSingle(), MaxRpm = r.ReadSingle(), Inertia = r.ReadSingle(), EngineBraking = r.ReadSingle(), ClutchStrength = r.ReadSingle() };
        int nc = r.ReadCount(4, "torque curve samples");
        if (nc > 64) { throw r.Error("a torque curve with more than 64 samples"); }
        var curve = new float[nc];
        r.ReadSingles(curve, true, "torque curve");
        d.Engine.TorqueCurve = curve;
        d.Transmission = new TransmissionDef { Automatic = r.ReadBool(), ReverseRatio = r.ReadSingle(), SwitchTime = r.ReadSingle(), ShiftUpRpm = r.ReadSingle(), ShiftDownRpm = r.ReadSingle() };
        int ng = r.ReadCount(4, "gear ratios");
        if (ng > 12) { throw r.Error("a gearbox with more than 12 gears"); }
        var ratios = new float[ng];
        r.ReadSingles(ratios, true, "gear ratios");
        d.Transmission.ForwardRatios = ratios;
        int nd = r.ReadCount(24, "differentials");
        if (nd > 16) { throw r.Error("more than 16 differentials"); }
        d.Differentials = new DifferentialDef[nd];
        for (int i = 0; i < nd; i++)
        {
            d.Differentials[i] = new DifferentialDef { Left = r.ReadInt32(), Right = r.ReadInt32(), Ratio = r.ReadSingle(), LeftRightSplit = r.ReadSingle(), LimitedSlipRatio = r.ReadSingle(), EngineTorqueRatio = r.ReadSingle() };
        }

        d.AirPitchTorque = r.ReadSingle(); d.GroundPitchTorque = r.ReadSingle();
        try { d.Validate(); }
        catch (ArgumentException e) { throw r.Error("invalid vehicle definition: " + e.Message); }
        return d;
    }

    private sealed class VehicleBlock
    {
        public HandleTable Handles = new();
        public VehicleState?[] States = Array.Empty<VehicleState?>();
    }

    private static VehicleBlock ReadVehicles(StateReader r, int bodyCount)
    {
        int version = r.ReadInt32();
        if (version != 1) { throw r.Error("unsupported 2D vehicle section version " + version.ToString(CultureInfo.InvariantCulture)); }
        var b = new VehicleBlock { Handles = HandleTable.Read(r, "vehicle") };
        if (b.Handles.Count == 0) { throw r.Error("a vehicle section holding no vehicles"); }
        b.States = new VehicleState?[Math.Max(b.Handles.HighWater, 1)];
        for (int v = 0; v < b.Handles.HighWater; v++)
        {
            if (!b.Handles.IsLive(v)) { continue; }
            var s = new VehicleState { Body = r.ReadInt32(), BodyGen = r.ReadInt32() };
            byte kind = r.ReadByte();
            if (s.Body < 0 || s.Body >= bodyCount || s.BodyGen < 1) { throw r.Error("invalid vehicle chassis"); }
            if (kind > (byte)VehicleKind.Motorcycle) { throw r.Error("unknown vehicle kind " + kind.ToString(CultureInfo.InvariantCulture)); }
            s.Kind = (VehicleKind)kind;
            s.Def = ReadVehicleDef(r);
            try
            {
                if (s.Kind == VehicleKind.Tracked)
                {
                    var t = new TrackedVehicleDef
                    {
                        Wheels = s.Def.Wheels, Grip = s.Def.Wheels[0].Tyre, SprocketRadius = r.ReadSingle(), MaxTrackSpeed = r.ReadSingle(), MaxDriveTorque = r.ReadSingle(), MaxBrakeTorque = r.ReadSingle(),
                        AirPitchTorque = s.Def.AirPitchTorque,
                    };
                    t.Validate();
                    s.Track = t.Clone();
                }
                else if (s.Kind == VehicleKind.Motorcycle)
                {
                    if (s.Wheels != 2 || s.Def.Differentials.Length != 1) { throw r.Error("a motorcycle needs two wheels and one differential"); }
                    var m = new MotorcycleDef
                    {
                        Front = s.Def.Wheels[0], Rear = s.Def.Wheels[1], Engine = s.Def.Engine, Transmission = s.Def.Transmission, FinalDriveRatio = s.Def.Differentials[0].Ratio, LeanTorque = r.ReadSingle(),
                    };
                    m.Validate();
                    s.Bike = m.Clone();
                }
            }
            catch (ArgumentException e) { throw r.Error("invalid vehicle kind parameters: " + e.Message); }

            s.Forward = r.ReadSingle(); s.Pitch = r.ReadSingle(); s.Brake = r.ReadSingle(); s.Handbrake = r.ReadSingle();
            s.EngineW = r.ReadSingle(); s.Gear = r.ReadInt32(); s.ShiftTimer = r.ReadSingle();
            if (!(Math.Abs(s.Forward) <= 1f) || !(Math.Abs(s.Pitch) <= 1f) || !(s.Brake >= 0f && s.Brake <= 1f) || !(s.Handbrake >= 0f && s.Handbrake <= 1f)) { throw r.Error("invalid vehicle inputs"); }
            // The drivetrain step keeps the engine between 0 and 1.2 times its rev limit, so a saved speed outside that was not written by a step.
            float engineTop = 1.2f * (s.Def.Engine.MaxRpm * (TwoPi / 60f));
            if (!float.IsFinite(s.EngineW) || s.EngineW < 0f || s.EngineW > engineTop || !float.IsFinite(s.ShiftTimer) || s.ShiftTimer < 0f) { throw r.Error("invalid engine state"); }
            if (s.Gear < -1 || s.Gear > s.Def.Transmission.Ratios.Length) { throw r.Error("invalid gear"); }
            s.Allocate();
            for (int i = 0; i < s.Wheels; i++)
            {
                s.Sprung[i] = r.ReadSingle(); s.Omega[i] = r.ReadSingle(); s.Spin[i] = r.ReadSingle(); s.Length[i] = r.ReadSingle();
                s.SpringForce[i] = r.ReadSingle(); s.SlipRatio[i] = r.ReadSingle(); s.Fx[i] = r.ReadSingle();
                s.Nx[i] = r.ReadSingle(); s.Ny[i] = r.ReadSingle();
                s.Cx[i] = r.ReadDouble(); s.Cy[i] = r.ReadDouble();
                s.Contact[i] = r.ReadByte(); s.GroundBody[i] = r.ReadInt32(); s.GroundGen[i] = r.ReadInt32();
                float sum = s.Sprung[i] + s.Omega[i] + s.Spin[i] + s.Length[i] + s.SpringForce[i] + s.SlipRatio[i] + s.Fx[i] + s.Nx[i] + s.Ny[i];
                if (!float.IsFinite(sum) || !double.IsFinite(s.Cx[i] + s.Cy[i])) { throw r.Error("non-finite wheel state"); }
                if (!(s.Sprung[i] > 0f) || s.Sprung[i] > 1e30f || s.Contact[i] > 1) { throw r.Error("invalid wheel state"); }
                if (!(Math.Abs(s.Omega[i]) <= MaxWheelSpin)) { throw r.Error("invalid wheel state: angular velocity beyond the limit"); }
                if (s.GroundBody[i] < -1 || s.GroundBody[i] >= bodyCount || (s.GroundBody[i] >= 0 && s.GroundGen[i] < 1)) { throw r.Error("invalid wheel ground body"); }
            }

            b.States[v] = s;
        }

        return b;
    }

    private void CommitVehicles(VehicleBlock? b)
    {
        Array.Fill(_vehOfBody, -1);
        if (b == null)
        {
            VehHandles.Reset();
            Array.Clear(Vehicles);
            return;
        }

        VehHandles = b.Handles;
        Vehicles = b.States;
        for (int v = 0; v < b.Handles.HighWater; v++)
        {
            if (b.Handles.IsLive(v)) { IndexVehicleBody(b.States[v]!.Body, v); }
        }
    }

    /// <summary>After a restore: every vehicle must sit on a live dynamic body of the generation it names, no body carries two, and every wheel's ground body is live.</summary>
    internal bool VehiclesConsistent()
    {
        var bodies = Core.Bodies;
        for (int v = 0; v < VehHandles.HighWater; v++)
        {
            if (!VehHandles.IsLive(v)) { continue; }
            var s = Vehicles[v]!;
            int b = s.Body;
            if (!bodies.Handles.IsAlive(b, s.BodyGen) || bodies.TypeOf(b) != BodyType.Dynamic || b >= _vehOfBody.Length || _vehOfBody[b] != v) { return false; }
            for (int i = 0; i < s.Wheels; i++)
            {
                if (s.Contact[i] != 0 && !bodies.Handles.IsAlive(s.GroundBody[i], s.GroundGen[i])) { return false; }
            }
        }

        return true;
    }

    /// <summary>Id of the 2D vehicle snapshot section ("VEH2").</summary>
    public const int VehicleSectionId = 0x32484556;

    private VehicleSection? _vehicleSection;

    internal ISnapshotSection VehicleSnapshotSection => _vehicleSection ??= new VehicleSection(this);

    /// <summary>The optional snapshot section that carries vehicles. It is written only while a vehicle exists, so worlds without vehicles keep their bytes and hashes.</summary>
    private sealed class VehicleSection(Engine2D e) : ISnapshotSection
    {
        public int SectionId => VehicleSectionId;
        public bool IsOptional => true;
        public bool IsPresent => e.VehHandles.Count > 0;

        public void Reset()
        {
            if (e.VehHandles.HighWater == 0) { return; }
            e.CommitVehicles(null);
        }

        public void Write(StateWriter w) => e.WriteVehicles(w);

        public void Read(StateReader r) => e.CommitVehicles(ReadVehicles(r, e.RestoredBodyCount));

        public ISnapshotSection Fork(WorldCore destination)
        {
            var dst = (Engine2D)destination.GetSnapshotSection(Engine2D.Id);
            dst.VehHandles = new HandleTable();
            dst.VehHandles.CopyFrom(e.VehHandles);
            dst.Vehicles = new VehicleState?[e.Vehicles.Length];
            for (int i = 0; i < dst.Vehicles.Length; i++) { dst.Vehicles[i] = e.Vehicles[i]?.CloneForFork(); }
            dst._vehOfBody = (int[])e._vehOfBody.Clone();
            return dst.VehicleSnapshotSection;
        }
    }
}
