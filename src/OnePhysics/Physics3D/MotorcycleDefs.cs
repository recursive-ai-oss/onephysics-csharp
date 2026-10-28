using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// A motorcycle: a front wheel that steers, a driven rear wheel, a lean controller and a front-wheel steering limit that shrinks with speed. The wheels sweep for the ground like those of a <see cref="VehicleDef"/>.
/// Chassis frame: +X right, +Y up, +Z forward in the presets.
/// <para>
/// <b>Lean.</b> In a steady turn at speed <c>v</c> with front-wheel angle <c>δ</c> and wheelbase <c>L</c> the bike must lean by <c>atan(v² · tan δ / (g · L))</c> for gravity to balance the centripetal acceleration
/// (the single-track model, Cossalter, <i>Motorcycle Dynamics</i>, ch. 3). The tyres supply the sideways force; the balance assist is a spring-damper torque about the bike's heading that steers the lean angle to that target
/// (and to upright at rest), limited to <see cref="MaxLeanTorque"/>. Past <see cref="FallAngle"/> the assist lets go and the bike falls.
/// </para>
/// </summary>
public sealed class MotorcycleDef
{
    /// <summary>The front wheel (steered). Its <see cref="WheelDef.MaxSteerAngle"/> is replaced by <see cref="MaxSteerAngle"/> and the speed limit; its tyre and brake torque apply.</summary>
    public WheelDef Front { get; set; } = new();

    /// <summary>The rear wheel (driven, steering ignored).</summary>
    public WheelDef Rear { get; set; } = new();

    /// <summary>The engine.</summary>
    public EngineDef Engine { get; set; } = new();

    /// <summary>The gearbox.</summary>
    public TransmissionDef Transmission { get; set; } = new();

    /// <summary>Final drive ratio between the gearbox and the rear wheel. Default 3. Positive.</summary>
    public float FinalDriveRatio { get; set; } = 3f;

    /// <summary>Front-wheel angle at full steering input at or below <see cref="SteerFadeStartSpeed"/>, radians. Default 0.5. Within 0..π/2.</summary>
    public float MaxSteerAngle { get; set; } = 0.5f;

    /// <summary>Front-wheel angle at full input at or above <see cref="SteerFadeEndSpeed"/>, radians; between the two speeds the limit falls linearly. Default 0.06. Within 0..<see cref="MaxSteerAngle"/>.</summary>
    public float HighSpeedSteerAngle { get; set; } = 0.06f;

    /// <summary>Speed up to which the full steering angle is allowed, m/s. Default 5. At least 0.</summary>
    public float SteerFadeStartSpeed { get; set; } = 5f;

    /// <summary>Speed from which only <see cref="HighSpeedSteerAngle"/> is allowed, m/s. Default 30. Greater than the start speed.</summary>
    public float SteerFadeEndSpeed { get; set; } = 30f;

    /// <summary>Largest lean the controller steers towards, radians. Default 0.7 (about 40°). Within 0..<see cref="FallAngle"/>.</summary>
    public float MaxLeanAngle { get; set; } = 0.7f;

    /// <summary>Lean beyond which the balance assist stops and the bike falls over, radians. Default 1.1. At most 1.45.</summary>
    public float FallAngle { get; set; } = 1.1f;

    /// <summary>Natural frequency of the lean spring, Hz. Default 5. Positive, at most 20.</summary>
    public float LeanFrequency { get; set; } = 5f;

    /// <summary>Damping ratio of the lean spring (1 is critical). Default 1. At least 0.</summary>
    public float LeanDamping { get; set; } = 1f;

    /// <summary>Largest torque the balance assist applies about the heading, N·m. Default 1500. At least 0 (0 turns the assist off).</summary>
    public float MaxLeanTorque { get; set; } = 1500f;

    /// <summary>Size of the chassis box <see cref="PhysicsWorld3D.AddMotorcycle"/> creates, m. Default 0.3 × 0.4 × 1.6.</summary>
    public Vector3F ChassisSize { get; set; } = new(0.3f, 0.4f, 1.6f);

    /// <summary>Mass of the chassis <see cref="PhysicsWorld3D.AddMotorcycle"/> creates, kg. Default 250. Positive.</summary>
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
        VehicleCheck.Positive(FinalDriveRatio, nameof(FinalDriveRatio));
        if (!(MaxSteerAngle >= 0f && MaxSteerAngle <= 1.5707964f)) { throw new ArgumentOutOfRangeException(nameof(MaxSteerAngle), MaxSteerAngle, "MaxSteerAngle must be within 0..pi/2."); }
        if (!(HighSpeedSteerAngle >= 0f && HighSpeedSteerAngle <= MaxSteerAngle)) { throw new ArgumentOutOfRangeException(nameof(HighSpeedSteerAngle), HighSpeedSteerAngle, "HighSpeedSteerAngle must be within 0..MaxSteerAngle."); }
        VehicleCheck.AtLeast0(SteerFadeStartSpeed, nameof(SteerFadeStartSpeed));
        if (!(SteerFadeEndSpeed > SteerFadeStartSpeed) || !float.IsFinite(SteerFadeEndSpeed)) { throw new ArgumentOutOfRangeException(nameof(SteerFadeEndSpeed), SteerFadeEndSpeed, "SteerFadeEndSpeed must be finite and greater than SteerFadeStartSpeed."); }
        if (!(FallAngle > 0f && FallAngle <= 1.45f)) { throw new ArgumentOutOfRangeException(nameof(FallAngle), FallAngle, "FallAngle must be within 0..1.45."); }
        if (!(MaxLeanAngle >= 0f && MaxLeanAngle <= FallAngle)) { throw new ArgumentOutOfRangeException(nameof(MaxLeanAngle), MaxLeanAngle, "MaxLeanAngle must be within 0..FallAngle."); }
        if (!(LeanFrequency > 0f && LeanFrequency <= 20f)) { throw new ArgumentOutOfRangeException(nameof(LeanFrequency), LeanFrequency, "LeanFrequency must be within 0..20 Hz."); }
        VehicleCheck.AtLeast0(LeanDamping, nameof(LeanDamping)); VehicleCheck.AtLeast0(MaxLeanTorque, nameof(MaxLeanTorque));
        VehicleCheck.Positive(ChassisMass, nameof(ChassisMass)); VehicleCheck.Vec(ChassisSize, nameof(ChassisSize));
        if (!(ChassisSize.X > 0f && ChassisSize.Y > 0f && ChassisSize.Z > 0f)) { throw new ArgumentOutOfRangeException(nameof(ChassisSize), ChassisSize, "ChassisSize must be positive."); }
    }

    /// <summary>The wheeled definition the engine runs the two wheels with: front wheel 0, rear wheel 1, one differential driving the rear wheel.</summary>
    internal VehicleDef ToWheeled()
    {
        Validate();
        var f = Front.Clone(); var r = Rear.Clone();
        f.MaxSteerAngle = MaxSteerAngle; r.MaxSteerAngle = 0f;
        return new VehicleDef
        {
            Wheels = new[] { f, r },
            Engine = Engine.Clone(), Transmission = Transmission.Clone(),
            Differentials = new[] { new DifferentialDef { Left = -1, Right = 1, Ratio = FinalDriveRatio } },
        };
    }

    /// <summary>
    /// A 250 kg road motorcycle (chassis 0.3 × 0.4 × 1.6 m): 0.32 m wheels 1.4 m apart on 2.5 Hz suspension, 90 N·m engine, five-speed automatic box, steering to 0.5 rad below 5 m/s and 0.06 rad above 30 m/s,
    /// lean to 40°.
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

    private static WheelDef Wheel(float z) => new()
    {
        Position = new Vector3F(0f, 0.1f, z), Radius = 0.32f, Width = 0.16f, SuspensionMinLength = 0.1f, SuspensionMaxLength = 0.4f, SpringFrequency = 2.5f, DampingRatio = 0.7f,
        Inertia = 0.8f, AngularDamping = 0.05f, CastShape = WheelCastShape.Sphere, Tyre = new TyreDef { Friction = 1.6f, LateralB = 10f },
    };
}
