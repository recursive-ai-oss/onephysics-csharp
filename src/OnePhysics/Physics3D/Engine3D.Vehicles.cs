using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>The saved and live state of one vehicle: the definition (copied at creation), the driver's inputs, the drivetrain and one record per wheel.</summary>
internal sealed class VehicleState
{
    public int Body, BodyGen;
    public VehicleDef Def = new();

    // Controller kind and its parameters (null for the kind that does not use them). A tracked vehicle's wheels are the left track's followed by the right track's.
    public VehicleKind Kind;
    public TrackedVehicleDef? Track;
    public MotorcycleDef? Bike;
    public int TrackLeft;
    public float BrakeLeft, BrakeRight;   // tracked: brake input per track this step (not saved: set by the drive every step)
    public float Lean, LeanTarget, LeanIntegral;   // motorcycle: measured and wanted lean, radians (positive to the right)

    // Inputs of the last Drive call.
    public float Forward, Right, Brake, Handbrake;

    // Drivetrain.
    public float EngineW;          // rad/s
    public int Gear;               // -1 reverse, 0 neutral, 1..N
    public float ShiftTimer;       // seconds left with the clutch open

    // Per wheel.
    public float[] Sprung = Array.Empty<float>();
    public float[] Omega = Array.Empty<float>(), Spin = Array.Empty<float>(), Steer = Array.Empty<float>(), Length = Array.Empty<float>();
    public float[] SpringForce = Array.Empty<float>(), SlipRatio = Array.Empty<float>(), SlipAngle = Array.Empty<float>(), Fx = Array.Empty<float>(), Fy = Array.Empty<float>();
    public float[] Nx = Array.Empty<float>(), Ny = Array.Empty<float>(), Nz = Array.Empty<float>();
    public double[] Cx = Array.Empty<double>(), Cy = Array.Empty<double>(), Cz = Array.Empty<double>();
    public byte[] Contact = Array.Empty<byte>();
    public int[] GroundBody = Array.Empty<int>(), GroundGen = Array.Empty<int>();

    // Derived (not saved).
    public float[] Curve = Array.Empty<float>(), Ratios = Array.Empty<float>();
    public ConvexShape3D[] CastShapes = Array.Empty<ConvexShape3D>();
    public int[] GroundShape = Array.Empty<int>();
    public float[] Torque = Array.Empty<float>(), Coef = Array.Empty<float>(), Compression = Array.Empty<float>(), Stop = Array.Empty<float>();

    public int Wheels => Def.Wheels.Length;

    public void Allocate()
    {
        int n = Def.Wheels.Length;
        Sprung = new float[n]; Omega = new float[n]; Spin = new float[n]; Steer = new float[n]; Length = new float[n];
        SpringForce = new float[n]; SlipRatio = new float[n]; SlipAngle = new float[n]; Fx = new float[n]; Fy = new float[n];
        Nx = new float[n]; Ny = new float[n]; Nz = new float[n]; Cx = new double[n]; Cy = new double[n]; Cz = new double[n];
        Contact = new byte[n]; GroundBody = new int[n]; GroundGen = new int[n];
        Array.Fill(GroundBody, -1);
        Derive();
    }

    /// <summary>Rebuilds what follows from the definition: the torque curve, the gear ratios and the cast shapes.</summary>
    public void Derive()
    {
        int n = Def.Wheels.Length;
        Curve = Def.Engine.Curve; Ratios = Def.Transmission.Ratios;
        CastShapes = new ConvexShape3D[n];
        for (int i = 0; i < n; i++)
        {
            var w = Def.Wheels[i];
            if (w.CastShape == WheelCastShape.Sphere) { CastShapes[i] = ConvexShape3D.Sphere(w.Radius); }
            else
            {
                // Edge rounding keeps the cast well conditioned and lets the wheel roll over mesh edges; the outer size stays the wheel's.
                float cr = Math.Min(0.15f * w.Radius, 0.5f * w.Width);
                CastShapes[i] = ConvexShape3D.Cylinder(0.5f * w.Width, w.Radius, cr);
            }
        }

        GroundShape = new int[n]; Torque = new float[n]; Coef = new float[n]; Compression = new float[n]; Stop = new float[n];
    }
}

internal sealed partial class Engine3D
{
    private const uint BlockVehicles = 8;
    private const float TwoPi = 6.2831855f;

    /// <summary>Slots of the vehicles (handles with generations, lowest free slot first).</summary>
    public HandleTable VehHandles = new();

    public VehicleState?[] Vehicles = new VehicleState?[4];

    private int[] _vehOfBody = Array.Empty<int>();
    private int _castChassis = -1;
    private Func<Shape, bool>? _vehiclePredicate;

    private bool HasVehicles => VehHandles.Count > 0;

    public int CreateVehicle(BodyHandle body, VehicleDef def, TrackedVehicleDef? track = null, MotorcycleDef? bike = null)
    {
        int b = Core.Resolve(body);
        if (Core.Bodies.TypeOf(b) != BodyType.Dynamic) { throw new ArgumentException("A vehicle's chassis must be a dynamic body.", nameof(body)); }
        if (VehicleOfBody(b) >= 0) { throw new PhysicsException("The body already carries a vehicle."); }
        def.Validate();
        var s = new VehicleState { Body = body.Index, BodyGen = body.Generation, Def = def.Clone() };
        if (track != null) { s.Kind = VehicleKind.Tracked; s.Track = track.Clone(); s.TrackLeft = track.LeftWheels.Length; }
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

    private float EffInv(int body, Vector3F r, Vector3F u)
    {
        var bodies = Core.Bodies;
        float m = bodies.InverseMass[body];
        if (!(m > 0f)) { return 0f; }
        var rxu = Vector3F.Cross(r, u);
        var iw = ApplyWorldInertia(body, rxu);
        return m + Vector3F.Dot(rxu, iw);
    }

    private void ApplyImpulseAtArm(int body, Vector3F r, Vector3F j)
    {
        var bodies = Core.Bodies;
        float m = bodies.InverseMass[body];
        if (!(m > 0f)) { return; }
        var v = bodies.LinearVelocityOf(body);
        v[0] += m * j.X; v[1] += m * j.Y; v[2] += m * j.Z;
        var dw = ApplyWorldInertia(body, Vector3F.Cross(r, j));
        var w = bodies.AngularVelocityOf(body);
        w[0] += dw.X; w[1] += dw.Y; w[2] += dw.Z;
    }

    private Vector3F PointVelocity(int body, Vector3F r)
    {
        var bodies = Core.Bodies;
        var v = bodies.LinearVelocityOf(body);
        var w = bodies.AngularVelocityOf(body);
        return new Vector3F(v[0], v[1], v[2]) + Vector3F.Cross(new Vector3F(w[0], w[1], w[2]), r);
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
    public void StepVehicles(PhysicsWorld3D world, float dt)
    {
        if (VehHandles.Count == 0) { return; }
        if (_vehiclePredicate == null) { _vehiclePredicate = sh => Core.Shapes.Owner[sh.Handle.Index] != _castChassis; }
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

    private void StepVehicle(PhysicsWorld3D world, VehicleState s, float h)
    {
        var bodies = Core.Bodies;
        int b = s.Body;
        if (!bodies.IsAwake(b))
        {
            if (s.Forward == 0f && s.Brake == 0f && s.Handbrake == 0f) { return; }
            WakeBodyIsland(b);
        }
        else if (s.Forward != 0f || s.Brake > 0f || s.Handbrake > 0f) { bodies.SleepTime[b] = 0f; }

        // The wheel forces below are timed for the world's own substep count, so the chassis' island runs that schedule.
        Core.KeepJointSchedule(b);
        RefreshWorldInertia(b);
        var def = s.Def;
        int n = s.Wheels;
        var q = RotationOfBody(b);
        var lc = LocalCenterOf(b);
        var pp = bodies.PositionOf(b);
        double px = pp[0], py = pp[1], pz = pp[2];
        _castChassis = b;
        var filter = new QueryFilter3D { Predicate = _vehiclePredicate };

        // Drivetrain first: wheel torques from the engine, from the speeds as they were at the end of the last step.
        var fwdAxis = q.Rotate(def.Wheels[0].ForwardDirection);
        var lin0 = bodies.LinearVelocityOf(b);
        float forwardSpeed = lin0[0] * fwdAxis.X + lin0[1] * fwdAxis.Y + lin0[2] * fwdAxis.Z;
        float brakeInput = s.Kind == VehicleKind.Tracked ? TrackDrive(s, h) : DriveTrain(s, h, forwardSpeed);

        // The balance assist reads the lean rate before the tyres kick the chassis this step, and acts before they do.
        if (s.Kind == VehicleKind.Motorcycle) { BalanceBike(s, h, q, forwardSpeed); }

        // Phase 1: sweep every wheel.
        for (int i = 0; i < n; i++)
        {
            var wd = def.Wheels[i];
            float steer = s.Right * wd.MaxSteerAngle;
            if (s.Kind == VehicleKind.Motorcycle) { steer = i == 0 ? s.Right * BikeSteerLimit(s, forwardSpeed) : 0f; }
            s.Steer[i] = steer;
            var rTop = q.Rotate(wd.Position - lc);
            var top = new Vector3D(px + rTop.X, py + rTop.Y, pz + rTop.Z);
            var dir = q.Rotate(wd.SuspensionDirection);
            var axle = q.Rotate(wd.AxleDirection);
            if (steer != 0f)
            {
                var sa = q.Rotate(wd.SteerAxis);
                var qs = QuaternionF.CreateFromAxisSinCos(sa, Math32.Sin(0.5f * steer), Math32.Cos(0.5f * steer));
                axle = qs.Rotate(axle);
            }

            var rot = AlignY(axle);
            // A chassis just inside the supported range can carry a wheel top just outside it; the public query rejects such a start, and a step must not fail half way, so that wheel finds no ground.
            double lim = BoxLimits.MaxCoordinate;
            bool inRange = Math.Abs(top.X) <= lim && Math.Abs(top.Y) <= lim && Math.Abs(top.Z) <= lim;   // NaN fails
            var hh = default(QueryHit3D);
            bool hit = inRange && world.ShapeCast(in s.CastShapes[i], top, rot, dir, out hh, wd.SuspensionMaxLength + 1e-4f, in filter);
            s.Contact[i] = 0;
            s.Compression[i] = 0f; s.Stop[i] = 0f;
            if (hit)
            {
                var n3 = hh.Normal;
                if (!(n3.LengthSquared() > 0.25f)) { n3 = dir * -1f; }   // started overlapping: no usable normal, push straight up the suspension
                float cosUp = -(n3.X * dir.X + n3.Y * dir.Y + n3.Z * dir.Z);
                if (cosUp >= 0.05f)
                {
                    float t = hh.Distance;
                    s.Contact[i] = 1;
                    s.Length[i] = Math.Min(wd.SuspensionMaxLength, Math.Max(t, wd.SuspensionMinLength));
                    s.Compression[i] = (wd.SuspensionMaxLength - s.Length[i]) + wd.PreloadLength;
                    s.Stop[i] = Math.Max(0f, wd.SuspensionMinLength - t);
                    s.Nx[i] = n3.X; s.Ny[i] = n3.Y; s.Nz[i] = n3.Z;
                    var cp = t > 1e-5f ? hh.Point : new Vector3D(top.X + (double)(dir.X * (t + wd.Radius)), top.Y + (double)(dir.Y * (t + wd.Radius)), top.Z + (double)(dir.Z * (t + wd.Radius)));
                    s.Cx[i] = cp.X; s.Cy[i] = cp.Y; s.Cz[i] = cp.Z;
                    int gb = hh.Body.Handle.Index;
                    s.GroundBody[i] = gb; s.GroundGen[i] = bodies.Handles.GenerationOf(gb);
                    s.GroundShape[i] = hh.Shape.Handle.Index;
                    RefreshWorldInertia(gb);   // the ground body takes impulses too; its cached world inertia may be stale (and is not saved)
                }
            }

            if (s.Contact[i] == 0)
            {
                s.Length[i] = wd.SuspensionMaxLength;
                s.GroundBody[i] = -1; s.GroundGen[i] = 0;
            }
        }

        // Phase 2: forces.
        int substeps = Math.Max(1, Opt.Substeps);
        float gravityLead = h * (substeps - 1) / (2f * substeps);
        for (int ii = 0; ii < n; ii++)
        {
            // A tracked vehicle visits its wheels left, right, left, right ...: the impulses are applied one after the other, so a side-by-side order would favour one track.
            int i = s.Kind == VehicleKind.Tracked ? InterleavedTrackWheel(s, ii) : ii;
            var wd = def.Wheels[i];
            float r = wd.Radius;
            float omega = s.Omega[i];
            float inv = 1f / wd.Inertia;
            // Wheel speed after the drive torque and the brakes; the tyre then acts on this speed (a split step that stays stable for any torque).
            float ws = omega + h * s.Torque[i] * inv;
            float bi = s.Kind == VehicleKind.Tracked ? (i < s.TrackLeft ? s.BrakeLeft : s.BrakeRight) : brakeInput;
            float brakeT = bi * wd.MaxBrakeTorque + s.Handbrake * wd.MaxHandbrakeTorque;
            if (brakeT > 0f)
            {
                float dwb = Math.Min(h * brakeT * inv, Math.Abs(ws));
                ws = ws > 0f ? ws - dwb : ws + dwb;
            }

            ws /= 1f + h * wd.AngularDamping;
            // A brake that has stopped the wheel holds it against the tyre up to its torque: the tyre force then sees the whole chassis, not just the wheel's spin-up.
            bool locked = brakeT > 0f && ws == 0f;
            s.Fx[i] = 0f; s.Fy[i] = 0f; s.SlipRatio[i] = 0f;
            if (s.Contact[i] == 0)
            {
                s.SpringForce[i] = 0f; s.SlipAngle[i] = 0f;
                ws = SaneSpin(ws);
                s.Omega[i] = ws; s.Spin[i] = WrapAngle(s.Spin[i] + ws * h);
                continue;
            }

            int groundShape = s.GroundShape[i];
            int gb = s.GroundBody[i];
            var rotC = q;
            var cpx = s.Cx[i]; var cpy = s.Cy[i]; var cpz = s.Cz[i];
            var arm = new Vector3F((float)(cpx - px), (float)(cpy - py), (float)(cpz - pz));
            var gp = bodies.PositionOf(gb);
            var armG = new Vector3F((float)(cpx - gp[0]), (float)(cpy - gp[1]), (float)(cpz - gp[2]));
            var nrm = new Vector3F(s.Nx[i], s.Ny[i], s.Nz[i]);

            // Suspension: a spring (explicit, capped by what restores the wheel in one step) and a damper (implicit in the effective mass), along the contact normal, pushing only.
            // Catto, "Soft Constraints" (GDC 2011) for the implicit damper.
            float invN = EffInv(b, arm, nrm) + EffInv(gb, armG, nrm);
            float meffN = invN > 0f ? 1f / invN : 0f;
            var rel = PointVelocity(b, arm) - PointVelocity(gb, armG);
            // The velocity at the start of a step carries the sawtooth of gravity applied in substeps; positions follow the mean velocity, so the damper reads that one.
            float closing = -Vector3F.Dot(rel - Opt.Gravity * (gravityLead * GravityScale[b]), nrm);
            float k = s.Sprung[i] * (TwoPi * wd.SpringFrequency) * (TwoPi * wd.SpringFrequency);
            float c = 2f * wd.DampingRatio * s.Sprung[i] * (TwoPi * wd.SpringFrequency);
            float comp = s.Compression[i];
            float extra = 0f;
            for (int a = 0; a < def.AntiRollBars.Length; a++)
            {
                var bar = def.AntiRollBars[a];
                int other = bar.Left == i ? bar.Right : bar.Right == i ? bar.Left : -1;
                if (other >= 0) { extra += bar.Stiffness * (comp - s.Compression[other]); }
            }

            float spring = h * k * comp;
            if (meffN > 0f) { spring = Math.Min(spring, meffN * comp / h); }
            spring += h * extra;
            float damp = h * c * closing / (1f + h * c * invN);
            float pn = spring + damp;
            if (s.Stop[i] > 0f && meffN > 0f) { pn += meffN * Math.Max(0f, closing + 0.2f * s.Stop[i] / h); }
            if (pn < 0f) { pn = 0f; }
            s.SpringForce[i] = pn / h;
            if (pn > 0f)
            {
                var jn = nrm * pn;
                ApplyImpulseAtArm(b, arm, jn);
                if (gb != b && bodies.InverseMass[gb] > 0f) { ApplyImpulseAtArm(gb, armG, jn * -1f); WakeBodyIsland(gb); }
            }

            // Tyre.
            float load = pn / h;
            float wheelOmegaAfter = ws;
            if (load > 0f)
            {
                var tyre = wd.Tyre;
                var fwdW = ForwardOf(q, wd, s.Steer[i]);
                var fdir = fwdW - nrm * Vector3F.Dot(fwdW, nrm);
                float fl = fdir.LengthSquared();
                if (fl > 1e-8f)
                {
                    fdir = fdir * (1f / (float)Math.Sqrt(fl));
                    var ldir = Vector3F.Cross(nrm, fdir);
                    rel = PointVelocity(b, arm) - PointVelocity(gb, armG);
                    float vlong = Vector3F.Dot(rel, fdir), vlat = Vector3F.Dot(rel, ldir);
                    float vref = Math.Max(Math.Abs(vlong), tyre.LowSpeed);
                    float vsx = ws * r - vlong;
                    float sx = vsx / vref, sy = vlat / vref;
                    s.SlipRatio[i] = sx;
                    s.SlipAngle[i] = Math32.Atan(sy);
                    float gx = Magic(tyre.LongitudinalB, tyre.LongitudinalC, tyre.LongitudinalE, sx);
                    float gy = -Magic(tyre.LateralB, tyre.LateralC, tyre.LateralE, sy);
                    // Friction ellipse: the forces share one friction budget.
                    float u2 = gx * gx + gy * gy;
                    if (u2 > 1f) { float iu = 1f / (float)Math.Sqrt(u2); gx *= iu; gy *= iu; }
                    float mu = (float)Math.Sqrt(tyre.Friction * Friction[groundShape]);
                    float fxw = mu * load * gx, fyw = mu * load * gy;
                    if (s.Kind == VehicleKind.Tracked) { fyw *= s.Track!.LateralFriction; }   // a track slides sideways more easily than it grips along the belt (Wong, Theory of Ground Vehicles)
                    // Implicit limits: a force never reverses the slip velocity it acts on within the step.
                    float invL = EffInv(b, arm, ldir) + EffInv(gb, armG, ldir);
                    float invF = EffInv(b, arm, fdir) + EffInv(gb, armG, fdir);
                    float maxY = invL > 0f ? Math.Abs(vlat) / (h * invL) : 0f;
                    if (Math.Abs(fyw) > maxY) { fyw = fyw < 0f ? -maxY : maxY; }
                    float maxX = Math.Abs(vsx) / (h * (invF + (locked ? 0f : r * r * inv)));
                    if (Math.Abs(fxw) > maxX) { fxw = fxw < 0f ? -maxX : maxX; }
                    // A braked wheel passes at most what its brake can hold: a tyre force above brake torque over radius would spin the wheel forward. Cars keep the whole friction budget (a stand-in for the chassis' weight on the wheel).
                    if (brakeT > 0f && s.Kind != VehicleKind.Wheeled) { float hold = brakeT / r; if (Math.Abs(fxw) > hold) { fxw = fxw < 0f ? -hold : hold; } }
                    s.Fx[i] = fxw; s.Fy[i] = fyw;
                    var jt = fdir * (h * fxw) + ldir * (h * fyw);
                    ApplyImpulseAtArm(b, arm, jt);
                    if (gb != b && bodies.InverseMass[gb] > 0f) { ApplyImpulseAtArm(gb, armG, jt * -1f); WakeBodyIsland(gb); }
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

    private static Vector3F ForwardOf(QuaternionF q, WheelDef wd, float steer)
    {
        var f = q.Rotate(wd.ForwardDirection);
        if (steer == 0f) { return f; }
        var sa = q.Rotate(wd.SteerAxis);
        var qs = QuaternionF.CreateFromAxisSinCos(sa, Math32.Sin(0.5f * steer), Math32.Cos(0.5f * steer));
        return qs.Rotate(f);
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
                float rpm = s.EngineW * (60f / TwoPi);
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

    private static void SetGear(VehicleState s, int gear, TransmissionDef tr)
    {
        if (s.Gear == gear) { return; }
        s.Gear = gear;
        s.ShiftTimer = tr.SwitchTime;
    }

    // ------------------------------------------------------------------ debug

    /// <summary>Emits one entry per wheel of every live vehicle, vehicle slot then wheel index.</summary>
    public void AddDebugVehicles(Debug.DebugGeometry g)
    {
        var bodies = Core.Bodies;
        for (int v = 0; v < VehHandles.HighWater; v++)
        {
            if (!VehHandles.IsLive(v)) { continue; }
            var s = Vehicles[v]!;
            int b = s.Body;
            var q = RotationOfBody(b);
            var lc = LocalCenterOf(b);
            var pp = bodies.PositionOf(b);
            for (int i = 0; i < s.Wheels; i++)
            {
                var wd = s.Def.Wheels[i];
                var rTop = q.Rotate(wd.Position - lc);
                var dir = q.Rotate(wd.SuspensionDirection);
                var top = new Vector3D(pp[0] + rTop.X, pp[1] + rTop.Y, pp[2] + rTop.Z);
                var center = new Vector3D(top.X + (double)(dir.X * s.Length[i]), top.Y + (double)(dir.Y * s.Length[i]), top.Z + (double)(dir.Z * s.Length[i]));
                bool contact = s.Contact[i] != 0;
                g.Add(new Debug.DebugWheel(Core.HandleOfBody(b), i, top, center, wd.Radius, wd.Width, q.Rotate(wd.AxleDirection), contact,
                    contact ? new Vector3D(s.Cx[i], s.Cy[i], s.Cz[i]) : default, contact ? new Vector3F(s.Nx[i], s.Ny[i], s.Nz[i]) : default));
            }
        }
    }

    // ------------------------------------------------------------------ snapshot block

    private void WriteVehicles(StateWriter w)
    {
        VehHandles.Write(w);
        for (int v = 0; v < VehHandles.HighWater; v++)
        {
            if (!VehHandles.IsLive(v)) { continue; }
            var s = Vehicles[v]!;
            w.WriteInt32(s.Body); w.WriteInt32(s.BodyGen);
            WriteVehicleDef(w, s.Def);
            w.WriteSingle(s.Forward); w.WriteSingle(s.Right); w.WriteSingle(s.Brake); w.WriteSingle(s.Handbrake);
            w.WriteSingle(s.EngineW); w.WriteInt32(s.Gear); w.WriteSingle(s.ShiftTimer);
            int n = s.Wheels;
            for (int i = 0; i < n; i++)
            {
                w.WriteSingle(s.Sprung[i]); w.WriteSingle(s.Omega[i]); w.WriteSingle(s.Spin[i]); w.WriteSingle(s.Steer[i]); w.WriteSingle(s.Length[i]);
                w.WriteSingle(s.SpringForce[i]); w.WriteSingle(s.SlipRatio[i]); w.WriteSingle(s.SlipAngle[i]); w.WriteSingle(s.Fx[i]); w.WriteSingle(s.Fy[i]);
                w.WriteSingle(s.Nx[i]); w.WriteSingle(s.Ny[i]); w.WriteSingle(s.Nz[i]);
                w.WriteDouble(s.Cx[i]); w.WriteDouble(s.Cy[i]); w.WriteDouble(s.Cz[i]);
                w.WriteByte(s.Contact[i]); w.WriteInt32(s.GroundBody[i]); w.WriteInt32(s.GroundGen[i]);
            }
        }
    }

    private static void WriteVec(StateWriter w, Vector3F v) { w.WriteSingle(v.X); w.WriteSingle(v.Y); w.WriteSingle(v.Z); }

    private static Vector3F ReadVec(StateReader r) => new(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());

    private static void WriteVehicleDef(StateWriter w, VehicleDef d)
    {
        w.WriteInt32(d.Wheels.Length);
        foreach (var wd in d.Wheels)
        {
            WriteVec(w, wd.Position); WriteVec(w, wd.SuspensionDirection); WriteVec(w, wd.SteerAxis); WriteVec(w, wd.AxleDirection); WriteVec(w, wd.ForwardDirection);
            w.WriteSingle(wd.Radius); w.WriteSingle(wd.Width); w.WriteSingle(wd.SuspensionMinLength); w.WriteSingle(wd.SuspensionMaxLength); w.WriteSingle(wd.SpringFrequency);
            w.WriteSingle(wd.DampingRatio); w.WriteSingle(wd.PreloadLength); w.WriteSingle(wd.SprungMass); w.WriteSingle(wd.Inertia); w.WriteSingle(wd.AngularDamping);
            w.WriteSingle(wd.MaxSteerAngle); w.WriteSingle(wd.MaxBrakeTorque); w.WriteSingle(wd.MaxHandbrakeTorque); w.WriteByte((byte)wd.CastShape);
            var t = wd.Tyre;
            w.WriteSingle(t.Friction); w.WriteSingle(t.LongitudinalB); w.WriteSingle(t.LongitudinalC); w.WriteSingle(t.LongitudinalE);
            w.WriteSingle(t.LateralB); w.WriteSingle(t.LateralC); w.WriteSingle(t.LateralE); w.WriteSingle(t.LowSpeed);
        }

        var e = d.Engine;
        w.WriteSingle(e.MaxTorque); w.WriteSingle(e.MinRpm); w.WriteSingle(e.MaxRpm); w.WriteSingle(e.Inertia); w.WriteSingle(e.EngineBraking); w.WriteSingle(e.ClutchStrength);
        var curve = e.Curve;
        w.WriteInt32(curve.Length); w.WriteSingles(curve);
        var tr = d.Transmission;
        w.WriteBool(tr.Automatic); w.WriteSingle(tr.ReverseRatio); w.WriteSingle(tr.SwitchTime); w.WriteSingle(tr.ShiftUpRpm); w.WriteSingle(tr.ShiftDownRpm);
        var ratios = tr.Ratios;
        w.WriteInt32(ratios.Length); w.WriteSingles(ratios);
        w.WriteInt32(d.Differentials.Length);
        foreach (var df in d.Differentials)
        {
            w.WriteInt32(df.Left); w.WriteInt32(df.Right); w.WriteSingle(df.Ratio); w.WriteSingle(df.LeftRightSplit); w.WriteSingle(df.LimitedSlipRatio); w.WriteSingle(df.EngineTorqueRatio);
        }

        w.WriteInt32(d.AntiRollBars.Length);
        foreach (var ab in d.AntiRollBars) { w.WriteInt32(ab.Left); w.WriteInt32(ab.Right); w.WriteSingle(ab.Stiffness); }
    }

    private static VehicleDef ReadVehicleDef(StateReader r)
    {
        var d = new VehicleDef();
        int nw = r.ReadCount(150, "vehicle wheels");
        if (nw > 32) { throw r.Error("a vehicle with more than 32 wheels"); }
        d.Wheels = new WheelDef[nw];
        for (int i = 0; i < nw; i++)
        {
            var wd = new WheelDef
            {
                Position = ReadVec(r), SuspensionDirection = ReadVec(r), SteerAxis = ReadVec(r), AxleDirection = ReadVec(r), ForwardDirection = ReadVec(r),
                Radius = r.ReadSingle(), Width = r.ReadSingle(), SuspensionMinLength = r.ReadSingle(), SuspensionMaxLength = r.ReadSingle(), SpringFrequency = r.ReadSingle(),
                DampingRatio = r.ReadSingle(), PreloadLength = r.ReadSingle(), SprungMass = r.ReadSingle(), Inertia = r.ReadSingle(), AngularDamping = r.ReadSingle(),
                MaxSteerAngle = r.ReadSingle(), MaxBrakeTorque = r.ReadSingle(), MaxHandbrakeTorque = r.ReadSingle(), CastShape = (WheelCastShape)r.ReadByte(),
            };
            wd.Tyre = new TyreDef
            {
                Friction = r.ReadSingle(), LongitudinalB = r.ReadSingle(), LongitudinalC = r.ReadSingle(), LongitudinalE = r.ReadSingle(),
                LateralB = r.ReadSingle(), LateralC = r.ReadSingle(), LateralE = r.ReadSingle(), LowSpeed = r.ReadSingle(),
            };
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

        int na = r.ReadCount(12, "anti-roll bars");
        if (na > 16) { throw r.Error("more than 16 anti-roll bars"); }
        d.AntiRollBars = new AntiRollBarDef[na];
        for (int i = 0; i < na; i++) { d.AntiRollBars[i] = new AntiRollBarDef { Left = r.ReadInt32(), Right = r.ReadInt32(), Stiffness = r.ReadSingle() }; }
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
        var b = new VehicleBlock { Handles = HandleTable.Read(r, "vehicle") };
        b.States = new VehicleState?[Math.Max(b.Handles.HighWater, 1)];
        for (int v = 0; v < b.Handles.HighWater; v++)
        {
            if (!b.Handles.IsLive(v)) { continue; }
            var s = new VehicleState { Body = r.ReadInt32(), BodyGen = r.ReadInt32() };
            if (s.Body < 0 || s.Body >= bodyCount || s.BodyGen < 1) { throw r.Error("invalid vehicle chassis"); }
            s.Def = ReadVehicleDef(r);
            s.Forward = r.ReadSingle(); s.Right = r.ReadSingle(); s.Brake = r.ReadSingle(); s.Handbrake = r.ReadSingle();
            s.EngineW = r.ReadSingle(); s.Gear = r.ReadInt32(); s.ShiftTimer = r.ReadSingle();
            if (!(Math.Abs(s.Forward) <= 1f) || !(Math.Abs(s.Right) <= 1f) || !(s.Brake >= 0f && s.Brake <= 1f) || !(s.Handbrake >= 0f && s.Handbrake <= 1f)) { throw r.Error("invalid vehicle inputs"); }
            // The drivetrain step keeps the engine between 0 and 1.2 times its rev limit, so a saved speed outside that was not written by a step.
            float engineTop = 1.2f * (s.Def.Engine.MaxRpm * (TwoPi / 60f));
            if (!float.IsFinite(s.EngineW) || s.EngineW < 0f || s.EngineW > engineTop || !float.IsFinite(s.ShiftTimer) || s.ShiftTimer < 0f) { throw r.Error("invalid engine state"); }
            if (s.Gear < -1 || s.Gear > s.Def.Transmission.Ratios.Length) { throw r.Error("invalid gear"); }
            try { s.Allocate(); }
            catch (ArgumentException e) { throw r.Error("invalid wheel dimensions: " + e.Message); }
            for (int i = 0; i < s.Wheels; i++)
            {
                s.Sprung[i] = r.ReadSingle(); s.Omega[i] = r.ReadSingle(); s.Spin[i] = r.ReadSingle(); s.Steer[i] = r.ReadSingle(); s.Length[i] = r.ReadSingle();
                s.SpringForce[i] = r.ReadSingle(); s.SlipRatio[i] = r.ReadSingle(); s.SlipAngle[i] = r.ReadSingle(); s.Fx[i] = r.ReadSingle(); s.Fy[i] = r.ReadSingle();
                s.Nx[i] = r.ReadSingle(); s.Ny[i] = r.ReadSingle(); s.Nz[i] = r.ReadSingle();
                s.Cx[i] = r.ReadDouble(); s.Cy[i] = r.ReadDouble(); s.Cz[i] = r.ReadDouble();
                s.Contact[i] = r.ReadByte(); s.GroundBody[i] = r.ReadInt32(); s.GroundGen[i] = r.ReadInt32();
                float sum = s.Sprung[i] + s.Omega[i] + s.Spin[i] + s.Steer[i] + s.Length[i] + s.SpringForce[i] + s.SlipRatio[i] + s.SlipAngle[i] + s.Fx[i] + s.Fy[i] + s.Nx[i] + s.Ny[i] + s.Nz[i];
                if (!float.IsFinite(sum) || !double.IsFinite(s.Cx[i] + s.Cy[i] + s.Cz[i])) { throw r.Error("non-finite wheel state"); }
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
    private bool VehiclesConsistent()
    {
        var bodies = Core.Bodies;
        for (int v = 0; v < VehHandles.HighWater; v++)
        {
            if (!VehHandles.IsLive(v)) { continue; }
            var s = Vehicles[v]!;
            int b = s.Body;
            if (!bodies.Handles.IsAlive(b, s.BodyGen) || bodies.TypeOf(b) != BodyType.Dynamic || _vehOfBody[b] != v) { return false; }
            for (int i = 0; i < s.Wheels; i++)
            {
                if (s.Contact[i] != 0 && !bodies.Handles.IsAlive(s.GroundBody[i], s.GroundGen[i])) { return false; }
            }
        }

        return true;
    }
}
