using System;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Which controller drives a <see cref="Vehicle"/>.</summary>
public enum VehicleKind : byte
{
    /// <summary>Engine, gearbox and differentials driving wheels with tyres (<see cref="VehicleDef"/>).</summary>
    Wheeled = 0,

    /// <summary>Two tracks driven independently, steered by skidding (<see cref="TrackedVehicleDef"/>).</summary>
    Tracked = 1,

    /// <summary>Two wheels with a lean controller (<see cref="MotorcycleDef"/>).</summary>
    Motorcycle = 2,
}

/// <summary>
/// A tracked vehicle: two tracks, each a belt over a set of sprung road wheels and a driven sprocket. Each road wheel sweeps for the ground like a wheel of a <see cref="VehicleDef"/>, with the same spring and damper
/// (<see cref="WheelDef"/> describes where and how stiff); the tyre curve is replaced by the track's grip, a Pacejka-type curve (Pacejka, <i>Tire and Vehicle Dynamics</i>) for the contact between belt and ground, whose
/// lateral part is what makes skid steering work (Wong, <i>Theory of Ground Vehicles</i>, ch. 5: the track slides sideways as the hull yaws, and the sliding friction resists the turn).
/// The belt is rigid: every road wheel of a track turns at the same belt speed, which the sprocket drives towards the commanded speed within its torque limit.
/// Inputs (<see cref="Vehicle.SetInput"/>): throttle −1..1 drives both tracks, steering −1..1 speeds the left track up and slows the right one (positive turns right, towards +X); with no throttle the tracks turn in opposite directions
/// and the vehicle pivots in place. Chassis frame as for <see cref="VehicleDef"/>: +X right, +Y up, +Z forward in the presets.
/// </summary>
public sealed class TrackedVehicleDef
{
    /// <summary>Road wheels of the left track (−X side), 1 to 16, front to back or any fixed order. Their <see cref="WheelDef.MaxSteerAngle"/>, tyre and handbrake values are ignored.</summary>
    public WheelDef[] LeftWheels { get; set; } = Array.Empty<WheelDef>();

    /// <summary>Road wheels of the right track (+X side), 1 to 16.</summary>
    public WheelDef[] RightWheels { get; set; } = Array.Empty<WheelDef>();

    /// <summary>The track's contact with the ground, shared by all road wheels: friction and the shape of the force curves. Default friction 1.5 (about 1.1 on the default ground).</summary>
    public TyreDef Grip { get; set; } = new() { Friction = 1.5f, LongitudinalB = 12f, LateralB = 6f, LowSpeed = 1f };

    /// <summary>
    /// Sideways grip of the tracks as a fraction of their grip along the belt. Skid steering only works when it is below 1: the sideways friction is the turning resistance (Wong, <i>Theory of Ground Vehicles</i>, ch. 5 gives 0.5 to 0.7 for
    /// steel tracks on firm ground). Default 0.5. Within 0..1, excluding 0.
    /// </summary>
    public float LateralFriction { get; set; } = 0.5f;

    /// <summary>Radius of the driven sprocket, m: converts the torque limits to a force along the belt. Default 0.3. Positive.</summary>
    public float SprocketRadius { get; set; } = 0.3f;

    /// <summary>Fastest belt speed, m/s: the speed commanded at full input and a hard limit on the belt. Default 12. Positive.</summary>
    public float MaxTrackSpeed { get; set; } = 12f;

    /// <summary>Torque each sprocket can deliver, N·m; the belt force is this over <see cref="SprocketRadius"/>. Default 20000. Positive.</summary>
    public float MaxDriveTorque { get; set; } = 20000f;

    /// <summary>Brake torque at each sprocket at full brake input, N·m. Default 120000. At least 0.</summary>
    public float MaxBrakeTorque { get; set; } = 120000f;

    /// <summary>Size of the hull box <see cref="PhysicsWorld3D.AddTank"/> creates, m. Default 3.2 × 1 × 6.</summary>
    public Vector3F ChassisSize { get; set; } = new(3.2f, 1f, 6f);

    /// <summary>Mass of the hull <see cref="PhysicsWorld3D.AddTank"/> creates, kg. Default 20000. Positive.</summary>
    public float ChassisMass { get; set; } = 20000f;

    /// <summary>A deep copy.</summary>
    public TrackedVehicleDef Clone()
    {
        var c = (TrackedVehicleDef)MemberwiseClone();
        c.LeftWheels = CloneWheels(LeftWheels); c.RightWheels = CloneWheels(RightWheels); c.Grip = Grip.Clone();
        return c;
    }

    private static WheelDef[] CloneWheels(WheelDef[] w)
    {
        var r = new WheelDef[w.Length];
        for (int i = 0; i < w.Length; i++) { r[i] = w[i].Clone(); }
        return r;
    }

    internal void Validate()
    {
        if (LeftWheels == null || RightWheels == null) { throw new ArgumentNullException(nameof(LeftWheels), "The wheel arrays must not be null."); }
        if (LeftWheels.Length < 1 || LeftWheels.Length > 16) { throw new ArgumentOutOfRangeException(nameof(LeftWheels), LeftWheels.Length, "A track needs 1 to 16 road wheels."); }
        if (RightWheels.Length < 1 || RightWheels.Length > 16) { throw new ArgumentOutOfRangeException(nameof(RightWheels), RightWheels.Length, "A track needs 1 to 16 road wheels."); }
        if (Grip == null) { throw new ArgumentNullException(nameof(Grip), "Grip must not be null."); }
        Grip.Validate("Grip");
        VehicleCheck.Positive(SprocketRadius, nameof(SprocketRadius));
        if (!(LateralFriction > 0f && LateralFriction <= 1f)) { throw new ArgumentOutOfRangeException(nameof(LateralFriction), LateralFriction, "LateralFriction must be within 0..1 and not 0."); } VehicleCheck.Positive(MaxTrackSpeed, nameof(MaxTrackSpeed));
        VehicleCheck.Positive(MaxDriveTorque, nameof(MaxDriveTorque)); VehicleCheck.AtLeast0(MaxBrakeTorque, nameof(MaxBrakeTorque));
        VehicleCheck.Positive(ChassisMass, nameof(ChassisMass)); VehicleCheck.Vec(ChassisSize, nameof(ChassisSize));
        if (!(ChassisSize.X > 0f && ChassisSize.Y > 0f && ChassisSize.Z > 0f)) { throw new ArgumentOutOfRangeException(nameof(ChassisSize), ChassisSize, "ChassisSize must be positive."); }
    }

    /// <summary>The wheeled definition the engine runs the road wheels with: left wheels then right wheels, the track grip as their tyre, the brake torque spread over the wheels in proportion to their radius.</summary>
    internal VehicleDef ToWheeled()
    {
        Validate();
        var d = new VehicleDef { Wheels = new WheelDef[LeftWheels.Length + RightWheels.Length] };
        for (int t = 0; t < 2; t++)
        {
            var src = t == 0 ? LeftWheels : RightWheels;
            for (int i = 0; i < src.Length; i++)
            {
                var w = src[i].Clone();
                w.Tyre = Grip.Clone(); w.MaxSteerAngle = 0f; w.MaxHandbrakeTorque = 0f;
                w.MaxBrakeTorque = MaxBrakeTorque * w.Radius / SprocketRadius / src.Length;
                d.Wheels[t == 0 ? i : LeftWheels.Length + i] = w;
            }
        }

        return d;
    }

    /// <summary>
    /// A main battle tank for a 20 t hull of 3.2 × 1 × 6 m: five 0.45 m road wheels a side on 2 Hz torsion-bar-like suspension, 20 kN·m sprockets, 12 m/s belts. It climbs 30 % grades and turns on the spot at more than half the
    /// kinematic rate (the speed difference of the belts over the track gauge).
    /// </summary>
    public static TrackedVehicleDef Tank
    {
        get
        {
            var d = new TrackedVehicleDef { LeftWheels = new WheelDef[5], RightWheels = new WheelDef[5] };
            for (int i = 0; i < 5; i++)
            {
                float z = 2.4f - 1.2f * i;
                d.LeftWheels[i] = RoadWheel(-1.3f, z);
                d.RightWheels[i] = RoadWheel(1.3f, z);
            }

            return d;
        }
    }

    private static WheelDef RoadWheel(float x, float z) => new()
    {
        Position = new Vector3F(x, -0.2f, z), Radius = 0.45f, Width = 0.5f, SuspensionMinLength = 0.1f, SuspensionMaxLength = 0.45f, SpringFrequency = 2f, DampingRatio = 0.7f,
        Inertia = 40f, AngularDamping = 0.05f,   // the wheel with its share of the belt, sprocket and drivetrain
    };
}
