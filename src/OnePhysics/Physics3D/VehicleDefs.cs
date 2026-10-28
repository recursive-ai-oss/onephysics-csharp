using System;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>The shape a wheel sweeps against the world to find the ground.</summary>
public enum WheelCastShape : byte
{
    /// <summary>A cylinder with rounded edges (the wheel's radius and width): rolls over the edges of meshes and height fields without snagging.</summary>
    Cylinder = 0,

    /// <summary>A sphere of the wheel's radius: cheaper, ignores the width.</summary>
    Sphere = 1,
}

/// <summary>
/// A tyre: how much force the contact patch makes for a given slip. Each direction follows a Pacejka "magic formula" curve
/// (Pacejka, <i>Tire and Vehicle Dynamics</i>): <c>f(s) = sin(C · atan(B·s − E·(B·s − atan(B·s))))</c>, scaled by friction and the load on the wheel. The longitudinal curve is a function of the slip ratio
/// (wheel surface speed against ground speed along the rolling direction), the lateral curve of the tangent of the slip angle. The two forces are limited together by a friction ellipse
/// (Milliken and Milliken, <i>Race Car Vehicle Dynamics</i>, ch. 2): when the normalised forces' length exceeds 1 both are scaled back to it.
/// </summary>
public sealed class TyreDef
{
    /// <summary>Peak friction coefficient of the tyre; combined with the ground shape's friction as <c>sqrt(tyre · ground)</c>, the world's rule. Default 1.5 (about 0.95 on the default ground). At least 0.</summary>
    public float Friction { get; set; } = 1.5f;

    /// <summary>Longitudinal stiffness factor B. Default 10; larger gives a sharper peak at a smaller slip ratio. Positive.</summary>
    public float LongitudinalB { get; set; } = 10f;

    /// <summary>Longitudinal shape factor C. Default 1.65; controls the height of the curve at large slip. Positive.</summary>
    public float LongitudinalC { get; set; } = 1.65f;

    /// <summary>Longitudinal curvature factor E. Default 0.9; at most 1.</summary>
    public float LongitudinalE { get; set; } = 0.9f;

    /// <summary>Lateral stiffness factor B. Default 8. Positive.</summary>
    public float LateralB { get; set; } = 8f;

    /// <summary>Lateral shape factor C. Default 1.5. Positive.</summary>
    public float LateralC { get; set; } = 1.5f;

    /// <summary>Lateral curvature factor E. Default 0.5; at most 1.</summary>
    public float LateralE { get; set; } = 0.5f;

    /// <summary>Speed below which slip is normalised by this speed instead of the true one (the slip ratio and angle are undefined at rest), m/s. Default 1. Positive.</summary>
    public float LowSpeed { get; set; } = 1f;

    /// <summary>A copy.</summary>
    public TyreDef Clone() => (TyreDef)MemberwiseClone();

    internal void Validate(string where)
    {
        VehicleCheck.AtLeast0(Friction, where + ".Friction");
        VehicleCheck.Positive(LongitudinalB, where + ".LongitudinalB"); VehicleCheck.Positive(LongitudinalC, where + ".LongitudinalC"); VehicleCheck.AtMost1(LongitudinalE, where + ".LongitudinalE");
        VehicleCheck.Positive(LateralB, where + ".LateralB"); VehicleCheck.Positive(LateralC, where + ".LateralC"); VehicleCheck.AtMost1(LateralE, where + ".LateralE");
        VehicleCheck.Positive(LowSpeed, where + ".LowSpeed");
    }
}

/// <summary>
/// One wheel: where its suspension is attached to the chassis, how the suspension behaves, how the wheel steers and brakes and which tyre it carries. Positions and directions are in the chassis body's frame.
/// The wheel centre sits at <c>Position + SuspensionDirection · length</c> with <c>length</c> between <see cref="SuspensionMinLength"/> and <see cref="SuspensionMaxLength"/>; <see cref="Position"/> is the highest the wheel centre can be.
/// </summary>
public sealed class WheelDef
{
    /// <summary>Top of the suspension: the wheel centre at zero length, in the chassis frame. Default the origin.</summary>
    public Vector3F Position { get; set; }

    /// <summary>Unit direction the wheel extends along, in the chassis frame. Default (0, -1, 0).</summary>
    public Vector3F SuspensionDirection { get; set; } = new(0f, -1f, 0f);

    /// <summary>Unit axis the wheel steers about, in the chassis frame. Default (0, 1, 0).</summary>
    public Vector3F SteerAxis { get; set; } = new(0f, 1f, 0f);

    /// <summary>Unit axle direction at zero steer, in the chassis frame. Default (1, 0, 0). The wheel's spin angle turns about it; see <see cref="ForwardDirection"/> for which way is forward.</summary>
    public Vector3F AxleDirection { get; set; } = new(1f, 0f, 0f);

    /// <summary>Unit rolling direction at zero steer, in the chassis frame: positive wheel speed moves the chassis this way. Default (0, 0, 1).</summary>
    public Vector3F ForwardDirection { get; set; } = new(0f, 0f, 1f);

    /// <summary>Wheel radius, m. Default 0.35. Positive.</summary>
    public float Radius { get; set; } = 0.35f;

    /// <summary>Wheel width, m. Default 0.25. Positive.</summary>
    public float Width { get; set; } = 0.25f;

    /// <summary>Shortest suspension length (fully compressed, the bump stop), m. Default 0.1. At least 0.</summary>
    public float SuspensionMinLength { get; set; } = 0.1f;

    /// <summary>Longest suspension length (fully extended), m. Default 0.5. Greater than the minimum.</summary>
    public float SuspensionMaxLength { get; set; } = 0.5f;

    /// <summary>Spring frequency, Hz, of the sprung mass on this wheel. Default 1.5. Positive, at most 25.</summary>
    public float SpringFrequency { get; set; } = 1.5f;

    /// <summary>Damping ratio of the spring (1 is critical: no overshoot). Default 0.6. At least 0.</summary>
    public float DampingRatio { get; set; } = 0.6f;

    /// <summary>Extra compression the spring force assumes at every length, m: the spring pushes with <c>k · (maxLength − length + preload)</c>, so the car sits higher for the same load. Default 0. At least 0.</summary>
    public float PreloadLength { get; set; }

    /// <summary>Mass this wheel's spring carries, kg. 0 (default) means the chassis mass divided by the number of wheels. At least 0.</summary>
    public float SprungMass { get; set; }

    /// <summary>Moment of inertia of the wheel and what spins with it, kg·m². Default 1. Positive.</summary>
    public float Inertia { get; set; } = 1f;

    /// <summary>Angular damping of the wheel's spin, 1/s. Default 0.2. At least 0.</summary>
    public float AngularDamping { get; set; } = 0.2f;

    /// <summary>Steering angle at full input, radians; 0 for a wheel that does not steer. Default 0. At most π/2 in size.</summary>
    public float MaxSteerAngle { get; set; }

    /// <summary>Brake torque at full brake input, N·m. Default 3000. At least 0.</summary>
    public float MaxBrakeTorque { get; set; } = 3000f;

    /// <summary>Brake torque at full handbrake input, N·m; 0 for a wheel the handbrake does not hold. Default 0. At least 0.</summary>
    public float MaxHandbrakeTorque { get; set; }

    /// <summary>The tyre. Default the default <see cref="TyreDef"/>.</summary>
    public TyreDef Tyre { get; set; } = new();

    /// <summary>What the wheel sweeps against the ground. Default <see cref="WheelCastShape.Cylinder"/>.</summary>
    public WheelCastShape CastShape { get; set; }

    /// <summary>A deep copy.</summary>
    public WheelDef Clone() { var c = (WheelDef)MemberwiseClone(); c.Tyre = Tyre.Clone(); return c; }

    internal void Validate(int index)
    {
        string w = "Wheels[" + index.ToString(CultureInfo.InvariantCulture) + "]";
        VehicleCheck.Vec(Position, w + ".Position");
        VehicleCheck.UnitVec(SuspensionDirection, w + ".SuspensionDirection"); VehicleCheck.UnitVec(SteerAxis, w + ".SteerAxis");
        VehicleCheck.UnitVec(AxleDirection, w + ".AxleDirection"); VehicleCheck.UnitVec(ForwardDirection, w + ".ForwardDirection");
        VehicleCheck.Positive(Radius, w + ".Radius"); VehicleCheck.Positive(Width, w + ".Width");
        if (Radius > 100f) { throw new ArgumentOutOfRangeException(w + ".Radius", Radius, w + ".Radius must be at most 100."); }
        if (Width > 100f) { throw new ArgumentOutOfRangeException(w + ".Width", Width, w + ".Width must be at most 100."); }
        VehicleCheck.AtLeast0(SuspensionMinLength, w + ".SuspensionMinLength");
        if (!(SuspensionMaxLength > SuspensionMinLength) || !float.IsFinite(SuspensionMaxLength) || SuspensionMaxLength > 100f)
        {
            throw new ArgumentOutOfRangeException(w + ".SuspensionMaxLength", SuspensionMaxLength, w + ".SuspensionMaxLength must be finite, greater than SuspensionMinLength (" + SuspensionMinLength.ToString(CultureInfo.InvariantCulture) + ") and at most 100.");
        }

        VehicleCheck.Positive(SpringFrequency, w + ".SpringFrequency");
        if (SpringFrequency > 25f) { throw new ArgumentOutOfRangeException(w + ".SpringFrequency", SpringFrequency, w + ".SpringFrequency must be at most 25 Hz."); }
        VehicleCheck.AtLeast0(DampingRatio, w + ".DampingRatio"); VehicleCheck.AtLeast0(PreloadLength, w + ".PreloadLength"); VehicleCheck.AtLeast0(SprungMass, w + ".SprungMass");
        VehicleCheck.Positive(Inertia, w + ".Inertia"); VehicleCheck.AtLeast0(AngularDamping, w + ".AngularDamping");
        if (!(Math.Abs(MaxSteerAngle) <= 1.5707964f)) { throw new ArgumentOutOfRangeException(w + ".MaxSteerAngle", MaxSteerAngle, w + ".MaxSteerAngle must be finite and at most pi/2 in size."); }
        VehicleCheck.AtLeast0(MaxBrakeTorque, w + ".MaxBrakeTorque"); VehicleCheck.AtLeast0(MaxHandbrakeTorque, w + ".MaxHandbrakeTorque");
        if (Tyre == null) { throw new ArgumentNullException(w + ".Tyre", w + ".Tyre must not be null."); }
        Tyre.Validate(w + ".Tyre");
        if (CastShape != WheelCastShape.Cylinder && CastShape != WheelCastShape.Sphere) { throw new ArgumentOutOfRangeException(w + ".CastShape", CastShape, w + ".CastShape is not a known shape."); }
    }
}

/// <summary>An engine: peak torque, rev range and a torque curve.</summary>
public sealed class EngineDef
{
    /// <summary>Torque at the curve's value 1, N·m. Default 400. Positive.</summary>
    public float MaxTorque { get; set; } = 400f;

    /// <summary>Idle speed, revolutions per minute; the engine holds it when the clutch is open or the throttle is released. Default 1000. Positive.</summary>
    public float MinRpm { get; set; } = 1000f;

    /// <summary>Rev limit, rpm; torque is cut at it. Default 6500. Greater than <see cref="MinRpm"/>.</summary>
    public float MaxRpm { get; set; } = 6500f;

    /// <summary>Moment of inertia of the engine and flywheel, kg·m². Default 0.5. Positive.</summary>
    public float Inertia { get; set; } = 0.5f;

    /// <summary>
    /// Torque against speed: samples of a fraction of <see cref="MaxTorque"/> (0..1) at evenly spaced speeds from 0 to <see cref="MaxRpm"/>, linearly interpolated.
    /// Null (default) is a typical road engine: rising to a peak at 60 % of the range and falling to 70 % at the limit. At least two samples, each within 0..1.
    /// </summary>
    public float[]? TorqueCurve { get; set; }

    /// <summary>Engine braking: torque opposing the engine with the throttle closed, as a fraction of <see cref="MaxTorque"/> at the rev limit, growing linearly with speed. Default 0.15. At least 0.</summary>
    public float EngineBraking { get; set; } = 0.15f;

    /// <summary>The largest torque the clutch carries when fully engaged, as a multiple of <see cref="MaxTorque"/>. Default 2. Positive.</summary>
    public float ClutchStrength { get; set; } = 2f;

    internal static readonly float[] DefaultCurve = { 0.6f, 0.85f, 0.95f, 1f, 0.97f, 0.9f, 0.8f, 0.7f };

    /// <summary>A deep copy.</summary>
    public EngineDef Clone() { var c = (EngineDef)MemberwiseClone(); c.TorqueCurve = (float[]?)TorqueCurve?.Clone(); return c; }

    internal float[] Curve => TorqueCurve ?? DefaultCurve;

    internal void Validate()
    {
        VehicleCheck.Positive(MaxTorque, "Engine.MaxTorque"); VehicleCheck.Positive(MinRpm, "Engine.MinRpm");
        if (!(MaxRpm > MinRpm) || !float.IsFinite(MaxRpm) || MaxRpm > 100000f) { throw new ArgumentOutOfRangeException("Engine.MaxRpm", MaxRpm, "Engine.MaxRpm must be finite, greater than MinRpm and at most 100000."); }
        VehicleCheck.Positive(Inertia, "Engine.Inertia"); VehicleCheck.AtLeast0(EngineBraking, "Engine.EngineBraking"); VehicleCheck.Positive(ClutchStrength, "Engine.ClutchStrength");
        var c = Curve;
        if (c.Length < 2 || c.Length > 64) { throw new ArgumentOutOfRangeException("Engine.TorqueCurve", c.Length, "Engine.TorqueCurve needs 2 to 64 samples."); }
        for (int i = 0; i < c.Length; i++)
        {
            if (!(c[i] >= 0f && c[i] <= 1f)) { throw new ArgumentOutOfRangeException("Engine.TorqueCurve", c[i], "Engine.TorqueCurve[" + i.ToString(CultureInfo.InvariantCulture) + "] must be within 0..1."); }
        }
    }
}

/// <summary>A gearbox, automatic or manual.</summary>
public sealed class TransmissionDef
{
    /// <summary>True (default) for an automatic box that shifts by engine speed and picks reverse from the drive input; false for a manual one set with <c>Vehicle.Gear</c>.</summary>
    public bool Automatic { get; set; } = true;

    /// <summary>Forward gear ratios, first gear first (engine turns per turn of the driveshaft). Null (default) is 3.5, 2.2, 1.5, 1.1, 0.85, 0.7. One to 12 entries, each positive.</summary>
    public float[]? ForwardRatios { get; set; }

    /// <summary>Reverse ratio (positive number). Default 3.2. Positive.</summary>
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
        for (int i = 0; i < r.Length; i++) { VehicleCheck.Positive(r[i], "Transmission.ForwardRatios[" + i.ToString(CultureInfo.InvariantCulture) + "]"); }
        VehicleCheck.Positive(ReverseRatio, "Transmission.ReverseRatio"); VehicleCheck.AtLeast0(SwitchTime, "Transmission.SwitchTime");
        VehicleCheck.Positive(ShiftDownRpm, "Transmission.ShiftDownRpm");
        if (!(ShiftUpRpm > ShiftDownRpm) || !float.IsFinite(ShiftUpRpm)) { throw new ArgumentOutOfRangeException("Transmission.ShiftUpRpm", ShiftUpRpm, "Transmission.ShiftUpRpm must be finite and greater than ShiftDownRpm."); }
    }
}

/// <summary>A differential sharing the driveshaft's torque between two wheels of one axle (or one wheel with <see cref="Right"/> = -1).</summary>
public sealed class DifferentialDef
{
    /// <summary>Index of the left wheel in <see cref="VehicleDef.Wheels"/>, or -1 for none.</summary>
    public int Left { get; set; }

    /// <summary>Index of the right wheel, or -1 for none.</summary>
    public int Right { get; set; } = 1;

    /// <summary>Final drive ratio of this differential. Default 3.4. Positive.</summary>
    public float Ratio { get; set; } = 3.4f;

    /// <summary>Share of this differential's torque that goes to the left wheel at equal wheel speeds. Default 0.5. Within 0..1.</summary>
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
        VehicleCheck.Positive(Ratio, w + ".Ratio");
        if (!(LeftRightSplit >= 0f && LeftRightSplit <= 1f)) { throw new ArgumentOutOfRangeException(w + ".LeftRightSplit", LeftRightSplit, w + ".LeftRightSplit must be within 0..1."); }
        if (!(LimitedSlipRatio >= 0f && LimitedSlipRatio <= 100f)) { throw new ArgumentOutOfRangeException(w + ".LimitedSlipRatio", LimitedSlipRatio, w + ".LimitedSlipRatio must be within 0..100."); }
        VehicleCheck.AtLeast0(EngineTorqueRatio, w + ".EngineTorqueRatio");
    }
}

/// <summary>An anti-roll bar: a torsion spring between the two wheels of an axle that adds to the spring force of the more compressed wheel and takes it from the other.</summary>
public sealed class AntiRollBarDef
{
    /// <summary>Index of the left wheel.</summary>
    public int Left { get; set; }

    /// <summary>Index of the right wheel.</summary>
    public int Right { get; set; } = 1;

    /// <summary>Force per metre of compression difference between the two wheels, N/m. Default 20000. At least 0.</summary>
    public float Stiffness { get; set; } = 20000f;

    /// <summary>A copy.</summary>
    public AntiRollBarDef Clone() => (AntiRollBarDef)MemberwiseClone();

    internal void Validate(int index, int wheels)
    {
        string w = "AntiRollBars[" + index.ToString(CultureInfo.InvariantCulture) + "]";
        if (Left < 0 || Left >= wheels) { throw new ArgumentOutOfRangeException(w + ".Left", Left, w + ".Left must be a wheel index below " + wheels.ToString(CultureInfo.InvariantCulture) + "."); }
        if (Right < 0 || Right >= wheels) { throw new ArgumentOutOfRangeException(w + ".Right", Right, w + ".Right must be a wheel index below " + wheels.ToString(CultureInfo.InvariantCulture) + "."); }
        if (Left == Right) { throw new ArgumentOutOfRangeException(w + ".Right", Right, w + " joins a wheel to itself."); }
        VehicleCheck.AtLeast0(Stiffness, w + ".Stiffness");
    }
}

/// <summary>
/// A wheeled vehicle: wheels with suspension and tyres, an engine, a gearbox, differentials and anti-roll bars, driven through <see cref="Vehicle.Drive"/>. Wheels are listed in a fixed order; their indices name them everywhere
/// (the differentials, the anti-roll bars, <see cref="Vehicle.GetWheel"/>). Chassis frame: +X right, +Y up, +Z forward in the presets; any other layout is fine as long as the wheels' directions say so.
/// Definitions are classes so the arrays of wheels and the presets can be edited freely; <see cref="PhysicsWorld3D.AddVehicle(Body, VehicleDef)"/> copies what it needs.
/// </summary>
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

    /// <summary>The anti-roll bars. At most 16.</summary>
    public AntiRollBarDef[] AntiRollBars { get; set; } = Array.Empty<AntiRollBarDef>();

    /// <summary>A deep copy.</summary>
    public VehicleDef Clone()
    {
        var c = new VehicleDef { Engine = Engine.Clone(), Transmission = Transmission.Clone() };
        c.Wheels = new WheelDef[Wheels.Length];
        for (int i = 0; i < Wheels.Length; i++) { c.Wheels[i] = Wheels[i].Clone(); }
        c.Differentials = new DifferentialDef[Differentials.Length];
        for (int i = 0; i < Differentials.Length; i++) { c.Differentials[i] = Differentials[i].Clone(); }
        c.AntiRollBars = new AntiRollBarDef[AntiRollBars.Length];
        for (int i = 0; i < AntiRollBars.Length; i++) { c.AntiRollBars[i] = AntiRollBars[i].Clone(); }
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
        if (AntiRollBars == null) { throw new ArgumentNullException(nameof(AntiRollBars), "AntiRollBars must not be null."); }
        if (Wheels.Length < 1 || Wheels.Length > 32) { throw new ArgumentOutOfRangeException(nameof(Wheels), Wheels.Length, "A vehicle needs 1 to 32 wheels."); }
        if (Differentials.Length > 16) { throw new ArgumentOutOfRangeException(nameof(Differentials), Differentials.Length, "At most 16 differentials."); }
        if (AntiRollBars.Length > 16) { throw new ArgumentOutOfRangeException(nameof(AntiRollBars), AntiRollBars.Length, "At most 16 anti-roll bars."); }
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

        for (int i = 0; i < AntiRollBars.Length; i++)
        {
            if (AntiRollBars[i] == null) { throw new ArgumentNullException(nameof(AntiRollBars), "AntiRollBars[" + i.ToString(CultureInfo.InvariantCulture) + "] is null."); }
            AntiRollBars[i].Validate(i, Wheels.Length);
        }

        // A wheel is driven by at most one differential (its torque would be counted twice otherwise).
        var seen = new bool[Wheels.Length];
        foreach (var d in Differentials)
        {
            foreach (int w in new[] { d.Left, d.Right })
            {
                if (w < 0) { continue; }
                if (seen[w]) { throw new ArgumentOutOfRangeException(nameof(Differentials), w, "Wheel " + w.ToString(CultureInfo.InvariantCulture) + " is driven by two differentials."); }
                seen[w] = true;
            }
        }
    }

    private static VehicleDef FourWheeler(float halfTrack, float frontZ, float rearZ, float radius, float width, float topY, float maxLen, float freq, float damping, float steer, bool frontDrive, bool rearDrive, float arb)
    {
        var d = new VehicleDef { Wheels = new WheelDef[4] };
        for (int i = 0; i < 4; i++)
        {
            bool front = i < 2, left = (i & 1) == 0;
            d.Wheels[i] = new WheelDef
            {
                Position = new Vector3F(left ? -halfTrack : halfTrack, topY, front ? frontZ : rearZ),
                Radius = radius, Width = width, SuspensionMaxLength = maxLen, SuspensionMinLength = 0.1f * maxLen, SpringFrequency = freq, DampingRatio = damping,
                MaxSteerAngle = front ? steer : 0f, MaxHandbrakeTorque = front ? 0f : 4000f,
            };
        }

        var diffs = new System.Collections.Generic.List<DifferentialDef>();
        if (frontDrive) { diffs.Add(new DifferentialDef { Left = 0, Right = 1, EngineTorqueRatio = frontDrive && rearDrive ? 0.5f : 1f }); }
        if (rearDrive) { diffs.Add(new DifferentialDef { Left = 2, Right = 3, EngineTorqueRatio = frontDrive && rearDrive ? 0.5f : 1f }); }
        d.Differentials = diffs.ToArray();
        d.AntiRollBars = new[] { new AntiRollBarDef { Left = 0, Right = 1, Stiffness = arb }, new AntiRollBarDef { Left = 2, Right = 3, Stiffness = arb } };
        return d;
    }

    /// <summary>
    /// A rear-wheel-drive passenger car for a chassis of about 1500 kg with its origin at the wheel tops (wheels 0.8 m either side, 1.3 m ahead and behind): 0.34 m wheels, 1.5 Hz suspension at damping 0.6,
    /// 400 N·m engine, six-speed automatic, open differential, handbrake on the rear wheels, front steering to 0.5 rad, anti-roll bars of 20 kN/m.
    /// </summary>
    public static VehicleDef Car => FourWheeler(0.8f, 1.3f, -1.3f, 0.34f, 0.25f, 0f, 0.5f, 1.5f, 0.6f, 0.5f, false, true, 20000f);

    /// <summary>
    /// A heavy truck for a chassis of about 8000 kg: 0.5 m wheels, softer 1.1 Hz suspension with 0.7 m travel, 1500 N·m diesel-like engine limited to 2800 rpm, six-speed automatic with low gears, all-wheel drive with the torque split evenly, steering to 0.4 rad,
    /// stiff anti-roll bars.
    /// </summary>
    public static VehicleDef Truck
    {
        get
        {
            var d = FourWheeler(1.0f, 2.2f, -2.2f, 0.5f, 0.35f, 0f, 0.7f, 1.1f, 0.7f, 0.4f, true, true, 80000f);
            d.Engine = new EngineDef { MaxTorque = 1500f, MinRpm = 600f, MaxRpm = 2800f, Inertia = 2f, TorqueCurve = new[] { 0.7f, 0.9f, 1f, 1f, 0.95f, 0.8f }, EngineBraking = 0.2f };
            d.Transmission = new TransmissionDef { ForwardRatios = new[] { 6f, 3.8f, 2.5f, 1.7f, 1.2f, 0.9f }, ReverseRatio = 5f, ShiftUpRpm = 2300f, ShiftDownRpm = 1100f, SwitchTime = 0.4f };
            foreach (var w in d.Wheels) { w.Inertia = 4f; w.MaxBrakeTorque = 12000f; w.MaxHandbrakeTorque = 0f; }
            return d;
        }
    }

    /// <summary>A go-kart for a chassis of about 150 kg and 1.0 × 0.2 × 1.4 m: 0.13 m wheels on short, stiff 3 Hz suspension, 30 N·m engine, one fixed gear, a locked rear axle (limited slip 100), no anti-roll bars, steering to 0.6 rad.</summary>
    public static VehicleDef Kart
    {
        get
        {
            var d = FourWheeler(0.45f, 0.55f, -0.55f, 0.13f, 0.14f, 0f, 0.12f, 3f, 0.8f, 0.6f, false, true, 0f);
            d.AntiRollBars = Array.Empty<AntiRollBarDef>();
            d.Engine = new EngineDef { MaxTorque = 30f, MinRpm = 1500f, MaxRpm = 9000f, Inertia = 0.05f, EngineBraking = 0.1f };
            d.Transmission = new TransmissionDef { ForwardRatios = new[] { 2.5f }, ReverseRatio = 2.5f, ShiftUpRpm = 8900f, ShiftDownRpm = 1200f, SwitchTime = 0.05f };
            d.Differentials = new[] { new DifferentialDef { Left = 2, Right = 3, Ratio = 1.5f, LimitedSlipRatio = 100f } };
            foreach (var w in d.Wheels) { w.Inertia = 0.05f; w.MaxBrakeTorque = 150f; w.MaxHandbrakeTorque = 0f; w.SuspensionMinLength = 0.01f; }
            return d;
        }
    }
}

internal static class VehicleCheck
{
    public static void Positive(float v, string name)
    {
        if (!(v > 0f) || !float.IsFinite(v)) { throw new ArgumentOutOfRangeException(name, v, name + " must be finite and greater than 0."); }
    }

    public static void AtLeast0(float v, string name)
    {
        if (!(v >= 0f) || !float.IsFinite(v)) { throw new ArgumentOutOfRangeException(name, v, name + " must be finite and at least 0."); }
    }

    public static void AtMost1(float v, string name)
    {
        if (!(v <= 1f)) { throw new ArgumentOutOfRangeException(name, v, name + " must be at most 1."); }
    }

    public static void Vec(Vector3F v, string name)
    {
        if (!float.IsFinite(v.X) || !float.IsFinite(v.Y) || !float.IsFinite(v.Z) || Math.Abs(v.X) > 1e4f || Math.Abs(v.Y) > 1e4f || Math.Abs(v.Z) > 1e4f)
        {
            throw new ArgumentOutOfRangeException(name, v, name + " must be finite with components within +-10000.");
        }
    }

    public static void UnitVec(Vector3F v, string name)
    {
        float l2 = v.X * v.X + v.Y * v.Y + v.Z * v.Z;
        if (!float.IsFinite(l2) || !(l2 > 0.9f && l2 < 1.1f)) { throw new ArgumentOutOfRangeException(name, v, name + " must be a unit vector (length 1 within 5 %)."); }
    }
}
