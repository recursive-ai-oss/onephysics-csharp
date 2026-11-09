using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// A tyre: how much force the contact patch makes along the rolling direction for a given slip. The curve is Pacejka's "magic formula" (Pacejka, <i>Tire and Vehicle Dynamics</i>):
/// <c>f(s) = sin(C · atan(B·s − E·(B·s − atan(B·s))))</c>, scaled by friction and the load on the wheel. The slip ratio is the wheel's surface speed against the ground speed along the rolling direction.
/// A 2D vehicle has no sideways direction, so there is no lateral curve: only the longitudinal part of the 3D tyre model applies.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public sealed class TyreDef
{
    /// <summary>Peak friction coefficient of the tyre; combined with the ground shape's friction as <c>sqrt(tyre · ground)</c>, the world's rule. Default 1.5 (about 0.95 on the default ground). Within 0 to 1000.</summary>
    public float Friction { get; set; } = 1.5f;

    /// <summary>Stiffness factor B. Default 10; larger gives a sharper peak at a smaller slip ratio. Positive, at most 1000.</summary>
    public float LongitudinalB { get; set; } = 10f;

    /// <summary>Shape factor C. Default 1.65; controls the height of the curve at large slip. Positive, at most 1000.</summary>
    public float LongitudinalC { get; set; } = 1.65f;

    /// <summary>Curvature factor E. Default 0.9; within -1000 to 1.</summary>
    public float LongitudinalE { get; set; } = 0.9f;

    /// <summary>Speed below which slip is normalised by this speed instead of the true one (the slip ratio is undefined at rest), m/s. Default 1. Within 0.001 to 1000.</summary>
    public float LowSpeed { get; set; } = 1f;

    /// <summary>A copy.</summary>
    public TyreDef Clone() => (TyreDef)MemberwiseClone();

    internal void Validate(string where)
    {
        VehicleCheck2D.Within(Friction, 0f, 1000f, where + ".Friction");
        VehicleCheck2D.Positive(LongitudinalB, where + ".LongitudinalB"); VehicleCheck2D.Positive(LongitudinalC, where + ".LongitudinalC");
        VehicleCheck2D.Within(LongitudinalB, 0f, 1000f, where + ".LongitudinalB"); VehicleCheck2D.Within(LongitudinalC, 0f, 1000f, where + ".LongitudinalC"); VehicleCheck2D.Within(LongitudinalE, -1000f, 1f, where + ".LongitudinalE");
        VehicleCheck2D.Within(LowSpeed, 1e-3f, 1000f, where + ".LowSpeed");
    }
}

/// <summary>
/// One wheel: where its suspension is attached to the chassis, how the suspension behaves, how the wheel brakes and which tyre it carries. Positions and directions are in the chassis body's frame
/// (+X forward and +Y up in the presets). The wheel centre sits at <c>Position + SuspensionDirection · length</c> with <c>length</c> between <see cref="SuspensionMinLength"/> and <see cref="SuspensionMaxLength"/>;
/// <see cref="Position"/> is the highest the wheel centre can be. The wheel is not a body: a circle of the wheel's radius sweeps along the suspension for the ground.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public sealed class WheelDef
{
    /// <summary>Top of the suspension: the wheel centre at zero length, in the chassis frame. Default the origin.</summary>
    public Vector2F Position { get; set; }

    /// <summary>Unit direction the wheel extends along, in the chassis frame. Default (0, -1).</summary>
    public Vector2F SuspensionDirection { get; set; } = new(0f, -1f);

    /// <summary>Unit rolling direction, in the chassis frame: positive wheel speed moves the chassis this way. Default (1, 0). The first wheel's direction is the vehicle's forward.</summary>
    public Vector2F ForwardDirection { get; set; } = new(1f, 0f);

    /// <summary>Wheel radius, m. Default 0.35. Positive, at most 100.</summary>
    public float Radius { get; set; } = 0.35f;

    /// <summary>Shortest suspension length (fully compressed, the bump stop), m. Default 0.1. At least 0.</summary>
    public float SuspensionMinLength { get; set; } = 0.1f;

    /// <summary>Longest suspension length (fully extended), m. Default 0.5. Greater than the minimum, at most 100.</summary>
    public float SuspensionMaxLength { get; set; } = 0.5f;

    /// <summary>Spring frequency, Hz, of the sprung mass on this wheel. Default 1.5. Positive, at most 25.</summary>
    public float SpringFrequency { get; set; } = 1.5f;

    /// <summary>Damping ratio of the spring (1 is critical: no overshoot). Default 0.6. At least 0.</summary>
    public float DampingRatio { get; set; } = 0.6f;

    /// <summary>Extra compression the spring force assumes at every length, m: the spring pushes with <c>k · (maxLength − length + preload)</c>, so the car sits higher for the same load. Default 0. At least 0.</summary>
    public float PreloadLength { get; set; }

    /// <summary>Mass this wheel's spring carries, kg. 0 (default) means the chassis mass divided by the number of wheels. Within 0 to 1e30.</summary>
    public float SprungMass { get; set; }

    /// <summary>Moment of inertia of the wheel and what spins with it, kg·m². Default 1. Within 0.001 to 1000000.</summary>
    public float Inertia { get; set; } = 1f;

    /// <summary>Angular damping of the wheel's spin, 1/s. Default 0.2. At least 0.</summary>
    public float AngularDamping { get; set; } = 0.2f;

    /// <summary>Brake torque at full brake input, N·m. Default 3000. At least 0.</summary>
    public float MaxBrakeTorque { get; set; } = 3000f;

    /// <summary>Brake torque at full handbrake input, N·m; 0 for a wheel the handbrake does not hold. Default 0. At least 0.</summary>
    public float MaxHandbrakeTorque { get; set; }

    /// <summary>The tyre. Default the default <see cref="TyreDef"/>.</summary>
    public TyreDef Tyre { get; set; } = new();

    /// <summary>A deep copy.</summary>
    public WheelDef Clone() { var c = (WheelDef)MemberwiseClone(); c.Tyre = Tyre.Clone(); return c; }

    internal void Validate(int index)
    {
        string w = "Wheels[" + index.ToString(CultureInfo.InvariantCulture) + "]";
        VehicleCheck2D.Vec(Position, w + ".Position");
        VehicleCheck2D.UnitVec(SuspensionDirection, w + ".SuspensionDirection"); VehicleCheck2D.UnitVec(ForwardDirection, w + ".ForwardDirection");
        VehicleCheck2D.Positive(Radius, w + ".Radius"); VehicleCheck2D.Within(Radius, 0f, 100f, w + ".Radius");
        VehicleCheck2D.AtLeast0(SuspensionMinLength, w + ".SuspensionMinLength");
        if (!(SuspensionMaxLength > SuspensionMinLength) || !float.IsFinite(SuspensionMaxLength) || SuspensionMaxLength > 100f)
        {
            throw new ArgumentOutOfRangeException(w + ".SuspensionMaxLength", SuspensionMaxLength, w + ".SuspensionMaxLength must be finite, greater than SuspensionMinLength (" + SuspensionMinLength.ToString("R", CultureInfo.InvariantCulture) + ") and at most 100.");
        }

        VehicleCheck2D.Positive(SpringFrequency, w + ".SpringFrequency"); VehicleCheck2D.Within(SpringFrequency, 0f, 25f, w + ".SpringFrequency");
        VehicleCheck2D.AtLeast0(DampingRatio, w + ".DampingRatio"); VehicleCheck2D.AtLeast0(PreloadLength, w + ".PreloadLength"); VehicleCheck2D.Within(SprungMass, 0f, 1e30f, w + ".SprungMass");
        VehicleCheck2D.Within(Inertia, 1e-3f, 1e6f, w + ".Inertia"); VehicleCheck2D.AtLeast0(AngularDamping, w + ".AngularDamping");
        VehicleCheck2D.AtLeast0(MaxBrakeTorque, w + ".MaxBrakeTorque"); VehicleCheck2D.AtLeast0(MaxHandbrakeTorque, w + ".MaxHandbrakeTorque");
        if (Tyre == null) { throw new ArgumentNullException(w + ".Tyre", w + ".Tyre must not be null."); }
        Tyre.Validate(w + ".Tyre");
    }
}

/// <summary>An engine: peak torque, rev range and a torque curve.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public sealed class EngineDef
{
    /// <summary>Torque at the curve's value 1, N·m. Default 400. Positive, at most 10000000.</summary>
    public float MaxTorque { get; set; } = 400f;

    /// <summary>Idle speed, revolutions per minute; the engine holds it when the clutch is open or the throttle is released. Default 1000. Positive.</summary>
    public float MinRpm { get; set; } = 1000f;

    /// <summary>Rev limit, rpm; torque is cut at it. Default 6500. Greater than <see cref="MinRpm"/>, at most 100000.</summary>
    public float MaxRpm { get; set; } = 6500f;

    /// <summary>Moment of inertia of the engine and flywheel, kg·m². Default 0.5. Within 0.001 to 1000000.</summary>
    public float Inertia { get; set; } = 0.5f;

    /// <summary>
    /// Torque against speed: samples of a fraction of <see cref="MaxTorque"/> (0..1) at evenly spaced speeds from 0 to <see cref="MaxRpm"/>, linearly interpolated.
    /// Null (default) is a typical road engine: rising to a peak at 60 % of the range and falling to 70 % at the limit. Two to 64 samples, each within 0..1.
    /// </summary>
    public float[]? TorqueCurve { get; set; }

    /// <summary>Engine braking: torque opposing the engine with the throttle closed, as a fraction of <see cref="MaxTorque"/> at the rev limit, growing linearly with speed. Default 0.15. At least 0.</summary>
    public float EngineBraking { get; set; } = 0.15f;

    /// <summary>The largest torque the clutch carries when fully engaged, as a multiple of <see cref="MaxTorque"/>. Default 2. Positive, at most 1000.</summary>
    public float ClutchStrength { get; set; } = 2f;

    internal static readonly float[] DefaultCurve = { 0.6f, 0.85f, 0.95f, 1f, 0.97f, 0.9f, 0.8f, 0.7f };

    /// <summary>A deep copy.</summary>
    public EngineDef Clone() { var c = (EngineDef)MemberwiseClone(); c.TorqueCurve = (float[]?)TorqueCurve?.Clone(); return c; }

    internal float[] Curve => TorqueCurve ?? DefaultCurve;

    internal void Validate()
    {
        VehicleCheck2D.Positive(MaxTorque, "Engine.MaxTorque"); VehicleCheck2D.Positive(MinRpm, "Engine.MinRpm");
        if (!(MaxRpm > MinRpm) || !float.IsFinite(MaxRpm) || MaxRpm > 100000f) { throw new ArgumentOutOfRangeException("Engine.MaxRpm", MaxRpm, "Engine.MaxRpm must be finite, greater than MinRpm and at most 100000."); }
        VehicleCheck2D.Within(Inertia, 1e-3f, 1e6f, "Engine.Inertia"); VehicleCheck2D.Within(MaxTorque, 0f, 1e7f, "Engine.MaxTorque"); VehicleCheck2D.Within(ClutchStrength, 0f, 1000f, "Engine.ClutchStrength");
        VehicleCheck2D.Positive(ClutchStrength, "Engine.ClutchStrength"); VehicleCheck2D.AtLeast0(EngineBraking, "Engine.EngineBraking");
        var c = Curve;
        if (c == null || c.Length < 2 || c.Length > 64) { throw new ArgumentOutOfRangeException("Engine.TorqueCurve", c?.Length ?? 0, "Engine.TorqueCurve needs 2 to 64 samples."); }
        for (int i = 0; i < c.Length; i++)
        {
            if (!(c[i] >= 0f && c[i] <= 1f)) { throw new ArgumentOutOfRangeException("Engine.TorqueCurve", c[i], "Engine.TorqueCurve[" + i.ToString(CultureInfo.InvariantCulture) + "] must be within 0..1."); }
        }
    }
}

/// <summary>A gearbox, automatic or manual.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public sealed class TransmissionDef
{
    /// <summary>True (default) for an automatic box that shifts by engine speed and picks reverse from the drive input; false for a manual one set with <c>Vehicle.Gear</c>.</summary>
    public bool Automatic { get; set; } = true;

    /// <summary>Forward gear ratios, first gear first (engine turns per turn of the driveshaft). Null (default) is 3.5, 2.2, 1.5, 1.1, 0.85, 0.7. One to 12 entries, each positive and at most 1000.</summary>
    public float[]? ForwardRatios { get; set; }

    /// <summary>Reverse ratio (positive number). Default 3.2. Positive, at most 1000.</summary>
    public float ReverseRatio { get; set; } = 3.2f;

    /// <summary>Seconds the clutch is open during a shift. Default 0.25. At least 0.</summary>
    public float SwitchTime { get; set; } = 0.25f;

    /// <summary>An automatic box shifts up above this speed, rpm. Default 5200. Greater than <see cref="ShiftDownRpm"/>.</summary>
    public float ShiftUpRpm { get; set; } = 5200f;

    /// <summary>An automatic box shifts down below this speed, rpm. Default 2200. Positive.</summary>
    public float ShiftDownRpm { get; set; } = 2200f;

    internal static readonly float[] DefaultRatios = { 3.5f, 2.2f, 1.5f, 1.1f, 0.85f, 0.7f };

    /// <summary>A deep copy.</summary>
    public TransmissionDef Clone() { var c = (TransmissionDef)MemberwiseClone(); c.ForwardRatios = (float[]?)ForwardRatios?.Clone(); return c; }

    internal float[] Ratios => ForwardRatios ?? DefaultRatios;

    internal void Validate()
    {
        var r = Ratios;
        if (r.Length < 1 || r.Length > 12) { throw new ArgumentOutOfRangeException("Transmission.ForwardRatios", r.Length, "Transmission.ForwardRatios needs 1 to 12 entries."); }
        for (int i = 0; i < r.Length; i++) { string n = "Transmission.ForwardRatios[" + i.ToString(CultureInfo.InvariantCulture) + "]"; VehicleCheck2D.Positive(r[i], n); VehicleCheck2D.Within(r[i], 0f, 1000f, n); }
        VehicleCheck2D.Positive(ReverseRatio, "Transmission.ReverseRatio"); VehicleCheck2D.Within(ReverseRatio, 0f, 1000f, "Transmission.ReverseRatio"); VehicleCheck2D.AtLeast0(SwitchTime, "Transmission.SwitchTime");
        VehicleCheck2D.Positive(ShiftDownRpm, "Transmission.ShiftDownRpm");
        if (!(ShiftUpRpm > ShiftDownRpm) || !float.IsFinite(ShiftUpRpm)) { throw new ArgumentOutOfRangeException("Transmission.ShiftUpRpm", ShiftUpRpm, "Transmission.ShiftUpRpm must be finite and greater than ShiftDownRpm."); }
    }
}

/// <summary>
/// A differential sharing the driveshaft's torque between two wheels (or giving it all to one wheel with the other index -1). In a side view an axle has one wheel, so a differential is usually a
/// centre differential between a front and a rear wheel, or a single driven wheel; the two indices are called <see cref="Left"/> and <see cref="Right"/> as in the 3D definition, with no geometric meaning.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public sealed class DifferentialDef
{
    /// <summary>Index of the first wheel in <see cref="VehicleDef.Wheels"/>, or -1 for none.</summary>
    public int Left { get; set; }

    /// <summary>Index of the second wheel, or -1 for none.</summary>
    public int Right { get; set; } = 1;

    /// <summary>Final drive ratio of this differential. Default 3.4. Positive, at most 1000.</summary>
    public float Ratio { get; set; } = 3.4f;

    /// <summary>Share of this differential's torque that goes to <see cref="Left"/> at equal wheel speeds. Default 0.5. Within 0..1.</summary>
    public float LeftRightSplit { get; set; } = 0.5f;

    /// <summary>
    /// Limited slip, as a torque bias ratio: when the wheels turn at different speeds the slower one may receive up to this multiple of the faster one's torque, ramping in over 0.5 rad/s of speed difference.
    /// 1 or less (default) is an open differential: the split never changes. At most 100.
    /// </summary>
    public float LimitedSlipRatio { get; set; } = 1f;

    /// <summary>This differential's share of the engine torque among all differentials (shares are normalised). Default 1. At least 0.</summary>
    public float EngineTorqueRatio { get; set; } = 1f;

    /// <summary>A copy.</summary>
    public DifferentialDef Clone() => (DifferentialDef)MemberwiseClone();

    internal void Validate(int index, int wheels)
    {
        string w = "Differentials[" + index.ToString(CultureInfo.InvariantCulture) + "]";
        if (Left < -1 || Left >= wheels) { throw new ArgumentOutOfRangeException(w + ".Left", Left, w + ".Left must be -1 or a wheel index below " + wheels.ToString(CultureInfo.InvariantCulture) + "."); }
        if (Right < -1 || Right >= wheels) { throw new ArgumentOutOfRangeException(w + ".Right", Right, w + ".Right must be -1 or a wheel index below " + wheels.ToString(CultureInfo.InvariantCulture) + "."); }
        if (Left < 0 && Right < 0) { throw new ArgumentOutOfRangeException(w, w + " drives no wheel."); }
        if (Left == Right) { throw new ArgumentOutOfRangeException(w + ".Right", Right, w + " drives one wheel twice."); }
        VehicleCheck2D.Positive(Ratio, w + ".Ratio"); VehicleCheck2D.Within(Ratio, 0f, 1000f, w + ".Ratio");
        if (!(LeftRightSplit >= 0f && LeftRightSplit <= 1f)) { throw new ArgumentOutOfRangeException(w + ".LeftRightSplit", LeftRightSplit, w + ".LeftRightSplit must be within 0..1."); }
        if (!(LimitedSlipRatio >= 0f && LimitedSlipRatio <= 100f)) { throw new ArgumentOutOfRangeException(w + ".LimitedSlipRatio", LimitedSlipRatio, w + ".LimitedSlipRatio must be within 0..100."); }
        VehicleCheck2D.AtLeast0(EngineTorqueRatio, w + ".EngineTorqueRatio");
    }
}

/// <summary>
/// A wheeled side-view vehicle: wheels with suspension and tyres, an engine, a gearbox and differentials, driven through <see cref="Vehicle.Drive"/>, with air control. Wheels are listed in a fixed order; their
/// indices name them everywhere (the differentials, <see cref="Vehicle.GetWheel"/>). Chassis frame: +X forward and +Y up in the presets. Definitions are classes so the arrays and the presets can be edited freely;
/// <see cref="PhysicsWorld2D.AddVehicle(Body, VehicleDef)"/> copies what it needs.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public sealed class VehicleDef
{
    /// <summary>The wheels, 1 to 32.</summary>
    public WheelDef[] Wheels { get; set; } = Array.Empty<WheelDef>();

    /// <summary>The engine.</summary>
    public EngineDef Engine { get; set; } = new();

    /// <summary>The gearbox.</summary>
    public TransmissionDef Transmission { get; set; } = new();

    /// <summary>The differentials; none means no wheel is driven. At most 16.</summary>
    public DifferentialDef[] Differentials { get; set; } = Array.Empty<DifferentialDef>();

    /// <summary>Torque on the chassis at full pitch input while no wheel touches the ground (air control), N·m. Positive pitch lifts the nose. Default 0. Within 0 to 1e9.</summary>
    public float AirPitchTorque { get; set; }

    /// <summary>Torque on the chassis at full pitch input while a wheel touches the ground (a rider shifting weight, wheelie and stoppie control), N·m. Default 0. Within 0 to 1e9.</summary>
    public float GroundPitchTorque { get; set; }

    /// <summary>A deep copy.</summary>
    public VehicleDef Clone()
    {
        var c = new VehicleDef { Engine = Engine.Clone(), Transmission = Transmission.Clone(), AirPitchTorque = AirPitchTorque, GroundPitchTorque = GroundPitchTorque };
        c.Wheels = new WheelDef[Wheels.Length];
        for (int i = 0; i < Wheels.Length; i++) { c.Wheels[i] = Wheels[i].Clone(); }
        c.Differentials = new DifferentialDef[Differentials.Length];
        for (int i = 0; i < Differentials.Length; i++) { c.Differentials[i] = Differentials[i].Clone(); }
        return c;
    }

    /// <summary>Checks every value; throws an <see cref="ArgumentOutOfRangeException"/> naming the field, the value and the bound.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is out of range.</exception>
    /// <exception cref="ArgumentNullException">A part is null.</exception>
    public void Validate()
    {
        if (Wheels == null) { throw new ArgumentNullException(nameof(Wheels), "Wheels must not be null."); }
        if (Engine == null) { throw new ArgumentNullException(nameof(Engine), "Engine must not be null."); }
        if (Transmission == null) { throw new ArgumentNullException(nameof(Transmission), "Transmission must not be null."); }
        if (Differentials == null) { throw new ArgumentNullException(nameof(Differentials), "Differentials must not be null."); }
        if (Wheels.Length < 1 || Wheels.Length > 32) { throw new ArgumentOutOfRangeException(nameof(Wheels), Wheels.Length, "A vehicle needs 1 to 32 wheels."); }
        if (Differentials.Length > 16) { throw new ArgumentOutOfRangeException(nameof(Differentials), Differentials.Length, "At most 16 differentials."); }
        VehicleCheck2D.Within(AirPitchTorque, 0f, 1e9f, nameof(AirPitchTorque)); VehicleCheck2D.Within(GroundPitchTorque, 0f, 1e9f, nameof(GroundPitchTorque));
        for (int i = 0; i < Wheels.Length; i++)
        {
            if (Wheels[i] == null) { throw new ArgumentNullException(nameof(Wheels), "Wheels[" + i.ToString(CultureInfo.InvariantCulture) + "] is null."); }
            Wheels[i].Validate(i);
        }

        Engine.Validate();
        Transmission.Validate();
        for (int i = 0; i < Differentials.Length; i++)
        {
            if (Differentials[i] == null) { throw new ArgumentNullException(nameof(Differentials), "Differentials[" + i.ToString(CultureInfo.InvariantCulture) + "] is null."); }
            Differentials[i].Validate(i, Wheels.Length);
        }

        // A wheel is driven by at most one differential (its torque would be counted twice otherwise).
        var seen = new bool[Wheels.Length];
        foreach (var d in Differentials)
        {
            for (int k = 0; k < 2; k++)
            {
                int w = k == 0 ? d.Left : d.Right;
                if (w < 0) { continue; }
                if (seen[w]) { throw new ArgumentOutOfRangeException(nameof(Differentials), w, "Wheel " + w.ToString(CultureInfo.InvariantCulture) + " is driven by two differentials."); }
                seen[w] = true;
            }
        }
    }

    private static WheelDef Wheel(float x, float y, float radius, float maxLen, float freq, float damping, float handbrake) => new()
    {
        Position = new Vector2F(x, y), Radius = radius, SuspensionMaxLength = maxLen, SuspensionMinLength = 0.1f * maxLen, SpringFrequency = freq, DampingRatio = damping, MaxHandbrakeTorque = handbrake,
    };

    /// <summary>
    /// A rear-wheel-drive passenger car for a chassis of about 1500 kg with its origin at the wheel tops (wheels 1.3 m ahead and behind): 0.34 m wheels, 1.5 Hz suspension at damping 0.6, 400 N·m engine,
    /// six-speed automatic, handbrake on the rear wheel, 600 N·m of air control and 300 N·m of weight shift on the ground. Wheel 0 is the front wheel, wheel 1 the rear (driven).
    /// </summary>
    public static VehicleDef Car => new()
    {
        Wheels = new[] { Wheel(1.3f, 0f, 0.34f, 0.5f, 1.5f, 0.6f, 0f), Wheel(-1.3f, 0f, 0.34f, 0.5f, 1.5f, 0.6f, 4000f) },
        Differentials = new[] { new DifferentialDef { Left = -1, Right = 1 } },
        AirPitchTorque = 600f, GroundPitchTorque = 300f,
    };

    /// <summary>
    /// A heavy truck for a chassis of about 8000 kg: 0.5 m wheels, softer 1.1 Hz suspension with 0.7 m travel, 1500 N·m diesel-like engine limited to 2800 rpm, six-speed automatic with low gears, all-wheel drive with the torque split evenly
    /// (wheel 0 front, wheel 1 rear). 3000 N·m of air control.
    /// </summary>
    public static VehicleDef Truck
    {
        get
        {
            var d = new VehicleDef
            {
                Wheels = new[] { Wheel(2.2f, 0f, 0.5f, 0.7f, 1.1f, 0.7f, 0f), Wheel(-2.2f, 0f, 0.5f, 0.7f, 1.1f, 0.7f, 0f) },
                Differentials = new[] { new DifferentialDef { Left = 0, Right = 1, Ratio = 3.4f, LeftRightSplit = 0.5f } },
                AirPitchTorque = 3000f, GroundPitchTorque = 1500f,
                Engine = new EngineDef { MaxTorque = 1500f, MinRpm = 600f, MaxRpm = 2800f, Inertia = 2f, TorqueCurve = new[] { 0.7f, 0.9f, 1f, 1f, 0.95f, 0.8f }, EngineBraking = 0.2f },
                Transmission = new TransmissionDef { ForwardRatios = new[] { 6f, 3.8f, 2.5f, 1.7f, 1.2f, 0.9f }, ReverseRatio = 5f, ShiftUpRpm = 2300f, ShiftDownRpm = 1100f, SwitchTime = 0.4f },
            };
            foreach (var w in d.Wheels) { w.Inertia = 4f; w.MaxBrakeTorque = 12000f; }
            return d;
        }
    }

    /// <summary>A go-kart for a chassis of about 150 kg: 0.13 m wheels on short, stiff 3 Hz suspension, 30 N·m engine, one fixed gear, driving the rear wheel (wheel 1; wheel 0 is the front), 40 N·m of air control.</summary>
    public static VehicleDef Kart
    {
        get
        {
            var d = new VehicleDef
            {
                Wheels = new[] { Wheel(0.55f, 0f, 0.13f, 0.12f, 3f, 0.8f, 0f), Wheel(-0.55f, 0f, 0.13f, 0.12f, 3f, 0.8f, 0f) },
                Differentials = new[] { new DifferentialDef { Left = -1, Right = 1, Ratio = 1.5f } },
                AirPitchTorque = 40f, GroundPitchTorque = 20f,
                Engine = new EngineDef { MaxTorque = 30f, MinRpm = 1500f, MaxRpm = 9000f, Inertia = 0.05f, EngineBraking = 0.1f },
                Transmission = new TransmissionDef { ForwardRatios = new[] { 2.5f }, ReverseRatio = 2.5f, ShiftUpRpm = 8900f, ShiftDownRpm = 1200f, SwitchTime = 0.05f },
            };
            foreach (var w in d.Wheels) { w.Inertia = 0.05f; w.MaxBrakeTorque = 150f; w.SuspensionMinLength = 0.01f; }
            return d;
        }
    }
}

/// <summary>Which controller drives a <see cref="Vehicle"/>.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public enum VehicleKind : byte
{
    /// <summary>Engine, gearbox and differentials driving wheels with tyres (<see cref="VehicleDef"/>).</summary>
    Wheeled = 0,

    /// <summary>A belt over sprung road wheels, driven at a commanded speed (<see cref="TrackedVehicleDef"/>).</summary>
    Tracked = 1,

    /// <summary>Two wheels and a rider whose lean is the chassis' pitch (<see cref="MotorcycleDef"/>).</summary>
    Motorcycle = 2,
}

/// <summary>
/// A tracked vehicle in side view: one belt over a set of sprung road wheels and a driven sprocket. Each road wheel sweeps for the ground like a wheel of a <see cref="VehicleDef"/>, with the same spring and damper;
/// the tyre curve is replaced by the track's grip (a Pacejka-type curve for the contact between belt and ground). The belt is rigid: every road wheel turns at the same belt speed, which the sprocket drives
/// towards the commanded speed within its torque limit (Wong, <i>Theory of Ground Vehicles</i>, ch. 5). Inputs (<see cref="Vehicle.SetInput"/>): throttle −1..1 drives the belt; the second input pitches the hull in the air.
/// There is no skid steering in a side view.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public sealed class TrackedVehicleDef
{
    /// <summary>Road wheels of the track, 1 to 32. Their tyre and handbrake values are ignored.</summary>
    public WheelDef[] Wheels { get; set; } = Array.Empty<WheelDef>();

    /// <summary>The track's contact with the ground, shared by all road wheels. Default friction 1.5 (about 1.1 on the default ground).</summary>
    public TyreDef Grip { get; set; } = new() { Friction = 1.5f, LongitudinalB = 12f, LowSpeed = 1f };

    /// <summary>Radius of the driven sprocket, m: converts the torque limits to a force along the belt. Default 0.3. Positive.</summary>
    public float SprocketRadius { get; set; } = 0.3f;

    /// <summary>Fastest belt speed, m/s: the speed commanded at full input and a hard limit on the belt. Default 12. Positive.</summary>
    public float MaxTrackSpeed { get; set; } = 12f;

    /// <summary>Torque the sprocket can deliver, N·m; the belt force is this over <see cref="SprocketRadius"/>. Default 20000. Positive.</summary>
    public float MaxDriveTorque { get; set; } = 20000f;

    /// <summary>Brake torque at the sprocket at full brake input, N·m. Default 120000. At least 0.</summary>
    public float MaxBrakeTorque { get; set; } = 120000f;

    /// <summary>Torque on the hull at full pitch input in the air, N·m. Default 20000. At least 0.</summary>
    public float AirPitchTorque { get; set; } = 20000f;

    /// <summary>Size (length, height) of the hull box <see cref="PhysicsWorld2D.AddTank"/> creates, m. Default 6 × 1.</summary>
    public Vector2F ChassisSize { get; set; } = new(6f, 1f);

    /// <summary>Mass of the hull <see cref="PhysicsWorld2D.AddTank"/> creates, kg. Default 20000. Positive.</summary>
    public float ChassisMass { get; set; } = 20000f;

    /// <summary>A deep copy.</summary>
    public TrackedVehicleDef Clone()
    {
        var c = (TrackedVehicleDef)MemberwiseClone();
        c.Wheels = new WheelDef[Wheels.Length];
        for (int i = 0; i < Wheels.Length; i++) { c.Wheels[i] = Wheels[i].Clone(); }
        c.Grip = Grip.Clone();
        return c;
    }

    internal void Validate()
    {
        if (Wheels == null) { throw new ArgumentNullException(nameof(Wheels), "Wheels must not be null."); }
        if (Wheels.Length < 1 || Wheels.Length > 32) { throw new ArgumentOutOfRangeException(nameof(Wheels), Wheels.Length, "A track needs 1 to 32 road wheels."); }
        if (Grip == null) { throw new ArgumentNullException(nameof(Grip), "Grip must not be null."); }
        Grip.Validate("Grip");
        VehicleCheck2D.Positive(SprocketRadius, nameof(SprocketRadius)); VehicleCheck2D.Within(SprocketRadius, 0f, 1000f, nameof(SprocketRadius));
        VehicleCheck2D.Positive(MaxTrackSpeed, nameof(MaxTrackSpeed)); VehicleCheck2D.Within(MaxTrackSpeed, 0f, 1e4f, nameof(MaxTrackSpeed));
        VehicleCheck2D.Positive(MaxDriveTorque, nameof(MaxDriveTorque)); VehicleCheck2D.Within(MaxDriveTorque, 0f, 1e9f, nameof(MaxDriveTorque));
        VehicleCheck2D.AtLeast0(MaxBrakeTorque, nameof(MaxBrakeTorque)); VehicleCheck2D.Within(MaxBrakeTorque, 0f, 1e9f, nameof(MaxBrakeTorque));
        VehicleCheck2D.Within(AirPitchTorque, 0f, 1e9f, nameof(AirPitchTorque));
        VehicleCheck2D.Positive(ChassisMass, nameof(ChassisMass)); VehicleCheck2D.Vec(ChassisSize, nameof(ChassisSize));
        if (!(ChassisSize.X > 0f && ChassisSize.Y > 0f)) { throw new ArgumentOutOfRangeException(nameof(ChassisSize), ChassisSize, "ChassisSize must be positive."); }
        for (int i = 0; i < Wheels.Length; i++)
        {
            if (Wheels[i] == null) { throw new ArgumentNullException(nameof(Wheels), "Wheels[" + i.ToString(CultureInfo.InvariantCulture) + "] is null."); }
            Wheels[i].Validate(i);
        }
    }

    /// <summary>The wheeled definition the engine runs the road wheels with: the track grip as their tyre, the brake torque spread over the wheels in proportion to their radius.</summary>
    internal VehicleDef ToWheeled()
    {
        Validate();
        var d = new VehicleDef { Wheels = new WheelDef[Wheels.Length], AirPitchTorque = AirPitchTorque };
        for (int i = 0; i < Wheels.Length; i++)
        {
            var w = Wheels[i].Clone();
            w.Tyre = Grip.Clone(); w.MaxHandbrakeTorque = 0f;
            w.MaxBrakeTorque = MaxBrakeTorque * w.Radius / SprocketRadius / Wheels.Length;
            d.Wheels[i] = w;
        }

        return d;
    }

    /// <summary>
    /// A main battle tank for a 20 t hull of 6 × 1 m: five 0.45 m road wheels on 2 Hz torsion-bar-like suspension, a 20 kN·m sprocket, 12 m/s belt. It climbs 30 % grades. Wheel 0 is the frontmost.
    /// </summary>
    public static TrackedVehicleDef Tank
    {
        get
        {
            var d = new TrackedVehicleDef { Wheels = new WheelDef[5] };
            for (int i = 0; i < 5; i++) { d.Wheels[i] = RoadWheel(2.4f - 1.2f * i); }
            return d;
        }
    }

    private static WheelDef RoadWheel(float x) => new()
    {
        Position = new Vector2F(x, -0.2f), Radius = 0.45f, SuspensionMinLength = 0.1f, SuspensionMaxLength = 0.45f, SpringFrequency = 2f, DampingRatio = 0.7f,
        Inertia = 40f, AngularDamping = 0.05f,   // the wheel with its share of the belt, sprocket and drivetrain
    };
}

/// <summary>
/// A motorcycle in side view: a front wheel, a driven rear wheel and a rider. The rider's lean is the chassis' pitch: the second drive input (positive lifts the front) applies <see cref="LeanTorque"/> to the chassis,
/// on the ground and in the air, so a wheelie (driving torque lifts the front wheel through the tyre force below the centre of mass) and a stoppie (front braking pushes the rear up) can be caught or provoked.
/// The wheels sweep for the ground like those of a <see cref="VehicleDef"/>. Chassis frame: +X forward, +Y up.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public sealed class MotorcycleDef
{
    /// <summary>The front wheel (wheel 0). Its tyre and brake torque apply.</summary>
    public WheelDef Front { get; set; } = new();

    /// <summary>The rear wheel (wheel 1, driven).</summary>
    public WheelDef Rear { get; set; } = new();

    /// <summary>The engine.</summary>
    public EngineDef Engine { get; set; } = new();

    /// <summary>The gearbox.</summary>
    public TransmissionDef Transmission { get; set; } = new();

    /// <summary>Final drive ratio between the gearbox and the rear wheel. Default 3. Positive.</summary>
    public float FinalDriveRatio { get; set; } = 3f;

    /// <summary>Chassis torque at full lean input, N·m, on the ground and in the air. Default 800. Within 0 to 1e9.</summary>
    public float LeanTorque { get; set; } = 800f;

    /// <summary>Size (length, height) of the chassis box <see cref="PhysicsWorld2D.AddMotorcycle"/> creates, m. Default 1.6 × 0.4.</summary>
    public Vector2F ChassisSize { get; set; } = new(1.6f, 0.4f);

    /// <summary>Mass of the chassis (with rider) <see cref="PhysicsWorld2D.AddMotorcycle"/> creates, kg. Default 250. Positive.</summary>
    public float ChassisMass { get; set; } = 250f;

    /// <summary>A deep copy.</summary>
    public MotorcycleDef Clone()
    {
        var c = (MotorcycleDef)MemberwiseClone();
        c.Front = Front.Clone(); c.Rear = Rear.Clone(); c.Engine = Engine.Clone(); c.Transmission = Transmission.Clone();
        return c;
    }

    internal void Validate()
    {
        if (Front == null || Rear == null || Engine == null || Transmission == null) { throw new ArgumentNullException(nameof(Front), "Front, Rear, Engine and Transmission must not be null."); }
        VehicleCheck2D.Positive(FinalDriveRatio, nameof(FinalDriveRatio)); VehicleCheck2D.Within(FinalDriveRatio, 0f, 1000f, nameof(FinalDriveRatio));
        VehicleCheck2D.Within(LeanTorque, 0f, 1e9f, nameof(LeanTorque));
        VehicleCheck2D.Positive(ChassisMass, nameof(ChassisMass)); VehicleCheck2D.Vec(ChassisSize, nameof(ChassisSize));
        if (!(ChassisSize.X > 0f && ChassisSize.Y > 0f)) { throw new ArgumentOutOfRangeException(nameof(ChassisSize), ChassisSize, "ChassisSize must be positive."); }
        Front.Validate(0); Rear.Validate(1); Engine.Validate(); Transmission.Validate();
    }

    /// <summary>The wheeled definition the engine runs the two wheels with: front wheel 0, rear wheel 1, one differential driving the rear wheel.</summary>
    internal VehicleDef ToWheeled()
    {
        Validate();
        return new VehicleDef
        {
            Wheels = new[] { Front.Clone(), Rear.Clone() },
            Engine = Engine.Clone(), Transmission = Transmission.Clone(),
            Differentials = new[] { new DifferentialDef { Left = -1, Right = 1, Ratio = FinalDriveRatio } },
            AirPitchTorque = LeanTorque, GroundPitchTorque = LeanTorque,
        };
    }

    /// <summary>
    /// A 250 kg road motorcycle (chassis 1.6 × 0.4 m): 0.32 m wheels 1.4 m apart on 2.5 Hz suspension, 90 N·m engine, five-speed automatic box, 800 N·m of lean.
    /// </summary>
    public static MotorcycleDef Road
    {
        get
        {
            var d = new MotorcycleDef
            {
                Front = Wheel(0.7f), Rear = Wheel(-0.7f),
                Engine = new EngineDef { MaxTorque = 90f, MinRpm = 1200f, MaxRpm = 10000f, Inertia = 0.1f, EngineBraking = 0.1f },
                Transmission = new TransmissionDef { ForwardRatios = new[] { 2.8f, 1.9f, 1.4f, 1.1f, 0.9f }, ReverseRatio = 2.8f, ShiftUpRpm = 8000f, ShiftDownRpm = 3500f, SwitchTime = 0.1f },
            };
            d.Front.MaxBrakeTorque = 500f; d.Rear.MaxBrakeTorque = 400f;
            return d;
        }
    }

    private static WheelDef Wheel(float x) => new()
    {
        Position = new Vector2F(x, 0.1f), Radius = 0.32f, SuspensionMinLength = 0.1f, SuspensionMaxLength = 0.4f, SpringFrequency = 2.5f, DampingRatio = 0.7f,
        Inertia = 0.8f, AngularDamping = 0.05f, Tyre = new TyreDef { Friction = 1.6f },
    };
}

internal static class VehicleCheck2D
{
    public static void Positive(float v, string name)
    {
        if (!(v > 0f) || !float.IsFinite(v)) { throw new ArgumentOutOfRangeException(name, v, name + " must be finite and greater than 0."); }
    }

    public static void AtLeast0(float v, string name)
    {
        if (!(v >= 0f) || !float.IsFinite(v)) { throw new ArgumentOutOfRangeException(name, v, name + " must be finite and at least 0."); }
    }

    /// <summary>Finite and within [lo, hi]. Wide enough for any real vehicle; the limits keep products of a definition's numbers (ratios, inertias, torques) far from float overflow, which would put NaN in the drivetrain.</summary>
    public static void Within(float v, float lo, float hi, string name)
    {
        if (!(v >= lo && v <= hi)) { throw new ArgumentOutOfRangeException(name, v, name + " must be within " + lo.ToString("R", CultureInfo.InvariantCulture) + " to " + hi.ToString("R", CultureInfo.InvariantCulture) + "."); }
    }

    public static void Vec(Vector2F v, string name)
    {
        if (!float.IsFinite(v.X) || !float.IsFinite(v.Y) || Math.Abs(v.X) > 1e4f || Math.Abs(v.Y) > 1e4f)
        {
            throw new ArgumentOutOfRangeException(name, v, name + " must be finite with components within +-10000.");
        }
    }

    public static void UnitVec(Vector2F v, string name)
    {
        float l2 = v.X * v.X + v.Y * v.Y;
        if (!float.IsFinite(l2) || !(l2 > 0.9f && l2 < 1.1f)) { throw new ArgumentOutOfRangeException(name, v, name + " must be a unit vector (length 1 within 5 %)."); }
    }
}
