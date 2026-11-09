using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Aero;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// An aerodynamic surface on a shape (set <see cref="ShapeDef.Aero"/> or <see cref="Shape.Aero"/>): lift and drag from the shape's orientation to the relative flow of the air (the world's <see cref="PhysicsWorld3D.AirDensity"/> and the wind of its area fields)
/// or of the water, applied to the body at the surface's centre of pressure. The model is documented on <see cref="AeroCoefficients"/>.
/// </summary>
/// <remarks>
/// A <see cref="AeroSurfaceKind.Lifting"/> surface lives in the plane spanned by <see cref="Forward"/> and <see cref="Up"/>: <see cref="Forward"/> is the direction the surface flies in at zero angle of attack (the leading edge points that way) and
/// <see cref="Up"/> is its lifting side. Flow along the third axis (the span) makes no force. With <see cref="Area"/> 0 the area is the shape's silhouette seen along <see cref="Up"/> (its planform), taken once from the shape's geometry.
/// A <see cref="AeroSurfaceKind.Bluff"/> surface has drag only, on the shape's silhouette seen along the flow each step when <see cref="Area"/> is 0. Meshes and height fields have no silhouette; give them an area. Sleeping bodies feel no aerodynamic forces.
/// </remarks>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public sealed class AeroSurface
{
    /// <summary>Lifting (default) or bluff.</summary>
    public AeroSurfaceKind Kind { get; set; }

    /// <summary>The lift and drag curves. Default <see cref="AeroCoefficients.Wing"/> with aspect ratio 6.</summary>
    public AeroCoefficients Coefficients { get; set; } = AeroCoefficients.Wing();

    /// <summary>Reference area, m², finite, 0 to 10^6; 0 takes it from the shape's silhouette (see the remarks). Default 0.</summary>
    public float Area { get; set; }

    /// <summary>Direction of travel at zero angle of attack, in the shape's frame; finite, not zero, normalised on use. Default (0, 0, 1).</summary>
    public Vector3F Forward { get; set; } = new(0f, 0f, 1f);

    /// <summary>The lifting side, in the shape's frame; need not be perpendicular to <see cref="Forward"/> (the part along it is removed). Default (0, 1, 0).</summary>
    public Vector3F Up { get; set; } = new(0f, 1f, 0f);

    /// <summary>Where the force acts, as an offset from the shape's origin in the shape's frame, metres. For a wing this is the aerodynamic centre, about a quarter of a chord behind the leading edge. Default zero.</summary>
    public Vector3F CenterOfPressure { get; set; }

    /// <summary>Air (default) or water.</summary>
    public AeroMedium Medium { get; set; }

    /// <summary>A lifting surface of the given area (0: from the shape) and aspect ratio, finite wing coefficients.</summary>
    public static AeroSurface Wing(float area = 0f, float aspectRatio = 6f) => new() { Area = area, Coefficients = AeroCoefficients.Wing(aspectRatio) };

    /// <summary>A flat plate of the given area and aspect ratio: lower stall, and a broadside drag following Hoerner's flat-plate data.</summary>
    public static AeroSurface Plate(float area = 0f, float aspectRatio = 2f) => new() { Area = area, Coefficients = AeroCoefficients.Plate(aspectRatio) };

    /// <summary>A bluff body with the given drag coefficient, drag only, on the shape's silhouette along the flow.</summary>
    public static AeroSurface Body(float dragCoefficient = 1f, float area = 0f) => new() { Kind = AeroSurfaceKind.Bluff, Coefficients = AeroCoefficients.Bluff(dragCoefficient), Area = area };

    /// <summary>A copy.</summary>
    public AeroSurface Clone() => (AeroSurface)MemberwiseClone();

    internal SurfaceSpec ToSpec() => new()
    {
        Kind = Kind, Coef = Coefficients, Area = Area, Chord = new Vector3F(-Forward.X, -Forward.Y, -Forward.Z), Normal = Up, Center = CenterOfPressure, Medium = Medium,
    };

    internal static AeroSurface FromSpec(SurfaceSpec s) => new()
    {
        Kind = s.Kind, Coefficients = s.Coef, Area = s.Area, Forward = new Vector3F(-s.Chord.X, -s.Chord.Y, -s.Chord.Z), Up = s.Normal, CenterOfPressure = s.Center, Medium = s.Medium,
    };
}

/// <summary>A thruster (rocket, jet, fan or propeller) on a body: a force along an axis at a point, with a throttle, a response time and an optional gimbal. See <see cref="PhysicsWorld3D.AddThruster"/>.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public sealed class ThrusterDef
{
    /// <summary>Where the thrust acts, in the body's frame (relative to the body origin), metres. Default zero.</summary>
    public Vector3F Position { get; set; }

    /// <summary>The direction the force pushes the body, in the body's frame; finite, not zero, normalised on use. Default (0, 1, 0).</summary>
    public Vector3F Direction { get; set; } = new(0f, 1f, 0f);

    /// <summary>Thrust at full throttle, newtons, 0 to 10^9. Default 100.</summary>
    public float MaxThrust { get; set; } = 100f;

    /// <summary>Time constant of the first-order lag between the throttle and the thrust, seconds, 0 to 60. Default 0.1; 0 follows the throttle at once.</summary>
    public float ResponseTime { get; set; } = 0.1f;

    /// <summary>Largest gimbal deflection, radians, 0 to 1.2. Default 0 (fixed).</summary>
    public float MaxGimbalAngle { get; set; }

    /// <summary>Axis the first gimbal angle turns the thrust about, in the body's frame (unit after normalising); zero (default) picks one perpendicular to <see cref="Direction"/>. The second gimbal angle turns about the axis perpendicular to both.</summary>
    public Vector3F GimbalAxis { get; set; }

    /// <summary>The fraction of <see cref="MaxThrust"/> available in reverse (throttle below 0), 0 to 1. Default 0: no reverse.</summary>
    public float ReverseFraction { get; set; }

    /// <summary>Reaction torque about the thrust axis, newton-metres per newton of thrust (a rotor's drag torque), −100 to 100. Default 0.</summary>
    public float SpinTorque { get; set; }

    /// <summary>For a propeller: the speed along the axis, relative to the medium, at which thrust has run out (thrust falls linearly to it). 0 (default) means constant thrust.</summary>
    public float PitchSpeed { get; set; }

    /// <summary>Air (default: works anywhere) or water (works only under the surface; fades in over <see cref="ImmersionDepth"/>).</summary>
    public AeroMedium Medium { get; set; }

    /// <summary>For a water thruster: the depth over which the thrust fades in as the propeller goes under, metres, 0.001 to 1000. Default 0.3.</summary>
    public float ImmersionDepth { get; set; } = 0.3f;

    /// <summary>A rocket motor: constant thrust of the given size along +Y with a short response and no reverse.</summary>
    public static ThrusterDef Rocket(float maxThrust, Vector3F position = default, Vector3F direction = default) =>
        new() { MaxThrust = maxThrust, Position = position, Direction = direction.X == 0f && direction.Y == 0f && direction.Z == 0f ? new Vector3F(0f, 1f, 0f) : direction, ResponseTime = 0.05f };

    /// <summary>A copy.</summary>
    public ThrusterDef Clone() => (ThrusterDef)MemberwiseClone();

    internal ThrusterSpec ToSpec() => new()
    {
        Position = Position, Direction = Direction, GimbalAxis = GimbalAxis, MaxThrust = MaxThrust, ResponseTime = ResponseTime, MaxGimbal = MaxGimbalAngle, ReverseFraction = ReverseFraction,
        SpinTorque = SpinTorque, PitchSpeed = PitchSpeed, ImmersionDepth = ImmersionDepth, Medium = Medium,
    };

    internal static ThrusterSpec[] ToSpecs(ThrusterDef[]? defs)
    {
        if (defs == null) { return Array.Empty<ThrusterSpec>(); }
        var a = new ThrusterSpec[defs.Length];
        for (int i = 0; i < a.Length; i++) { a[i] = (defs[i] ?? throw new ArgumentNullException(nameof(defs), "Thruster " + i.ToString(CultureInfo.InvariantCulture) + " is null.")).ToSpec(); }
        return a;
    }
}

/// <summary>Ready-made thruster layouts.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public static class ThrusterRig
{
    /// <summary>
    /// Four vertical rotors of a quadcopter on the X and Z axes at <paramref name="armLength"/> from the body origin, each pushing along +Y, response 0.05 s, spinning alternately with a reaction of <paramref name="spinTorquePerThrust"/>.
    /// Throttle the four together for height; add the same amount to opposite pairs for pitch and roll, and to the alternate pair for yaw. Order: +X, −X, +Z, −Z.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The arm length is not within 0.001 to 1000 or the thrust is negative.</exception>
    public static ThrusterDef[] Quadcopter(float armLength, float maxThrustEach, float spinTorquePerThrust = 0.02f)
    {
        if (!(armLength >= 0.001f && armLength <= 1000f)) { throw new ArgumentOutOfRangeException(nameof(armLength), armLength, "The arm length must be within 0.001 to 1000 m."); }
        if (!(maxThrustEach >= 0f && maxThrustEach <= 1e9f)) { throw new ArgumentOutOfRangeException(nameof(maxThrustEach), maxThrustEach, "The thrust must be within 0 to 1e9 N."); }
        ThrusterDef Rotor(Vector3F at, float spin) => new() { Position = at, MaxThrust = maxThrustEach, ResponseTime = 0.05f, SpinTorque = spin * spinTorquePerThrust };
        return
        [
            Rotor(new Vector3F(armLength, 0f, 0f), 1f), Rotor(new Vector3F(-armLength, 0f, 0f), 1f),
            Rotor(new Vector3F(0f, 0f, armLength), -1f), Rotor(new Vector3F(0f, 0f, -armLength), -1f),
        ];
    }
}

/// <summary>A keel, rudder or hydrofoil of a boat: a lifting surface in water in the boat's own axes.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public sealed class BoatFoilDef
{
    /// <summary>Where the force acts, in the body's frame, metres. Default zero.</summary>
    public Vector3F Position { get; set; }

    /// <summary>Area, m², above 0 and at most 10^6. Default 0.1.</summary>
    public float Area { get; set; } = 0.1f;

    /// <summary>Aspect ratio for the lift slope and the induced drag, 0.1 to 100. Default 2.</summary>
    public float AspectRatio { get; set; } = 2f;

    /// <summary>False (default): a vertical fin that resists sideways motion (its lifting side is the boat's lateral axis). True: a horizontal foil that lifts along the boat's up axis. In a 2D world every foil is horizontal.</summary>
    public bool Horizontal { get; set; }

    /// <summary>How far the foil follows the rudder: 1 for a rudder, 0 for a fixed keel; the foil turns by rudder angle × gain about the up axis. −10 to 10. Default 0.</summary>
    public float SteerGain { get; set; }

    /// <summary>A copy.</summary>
    public BoatFoilDef Clone() => (BoatFoilDef)MemberwiseClone();
}

/// <summary>
/// A boat: a hull on a dynamic body that floats on the world's water (<see cref="PhysicsWorld3D.AddWater"/> or a buoyancy effector), with hydrodynamic hull drag, a keel and rudder as lifting surfaces in water, and propellers. See <see cref="PhysicsWorld3D.AddBoat"/>.
/// </summary>
/// <remarks>
/// Hull drag is a quadratic law per hull axis in volumetric coefficients: force = ½ρ·C·V<sup>2/3</sup>·|u|·u for the velocity u of the water relative to the hull along that axis and the volume V of the hull under water; it replaces the fluid's own
/// linear drag and damping for this body (the fluid's buoyancy and angular damping still apply), together with <see cref="HeaveDamping"/>. Every drag is limited so that it cannot do more than stop the motion along that axis within a step.
/// Positions are in the body's frame. The presets assume a hull modelled with its origin at the middle of the waterline, +Z forward and +Y up.
/// </remarks>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public sealed class BoatDef
{
    /// <summary>Which way is forward, in the body's frame. Default (0, 0, 1).</summary>
    public Vector3F Forward { get; set; } = new(0f, 0f, 1f);

    /// <summary>Which way is up, in the body's frame. Default (0, 1, 0).</summary>
    public Vector3F Up { get; set; } = new(0f, 1f, 0f);

    /// <summary>Hull drag coefficient along the boat, 0 to 1000. Default 0.1.</summary>
    public float LongitudinalDrag { get; set; } = 0.1f;

    /// <summary>Hull drag coefficient across the boat (leeway), 0 to 1000. Default 1. Ignored in 2D.</summary>
    public float LateralDrag { get; set; } = 1f;

    /// <summary>Hull drag coefficient along the up axis (heave), 0 to 1000. Default 2.</summary>
    public float VerticalDrag { get; set; } = 2f;

    /// <summary>Linear damping of motion along the up axis (heave), per second: force = damping · ρ · V · u with V the volume under water and u the water's velocity relative to the hull along it; it stands for the energy waves carry away, so a floating hull stops bobbing. 0 to 1000. Default 3.</summary>
    public float HeaveDamping { get; set; } = 3f;

    /// <summary>Largest rudder deflection, radians, 0 to 1.2. Default 0.6.</summary>
    public float MaxRudderAngle { get; set; } = 0.6f;

    /// <summary>How fast the rudder moves, rad/s, 0.01 to 100. Default 1.5.</summary>
    public float RudderRate { get; set; } = 1.5f;

    /// <summary>Keel, rudder and hydrofoils.</summary>
    public BoatFoilDef[] Foils { get; set; } = Array.Empty<BoatFoilDef>();

    /// <summary>Propellers (their medium is always water).</summary>
    public ThrusterDef[] Propellers { get; set; } = Array.Empty<ThrusterDef>();

    /// <summary>A copy (arrays and elements copied).</summary>
    public BoatDef Clone()
    {
        var b = (BoatDef)MemberwiseClone();
        b.Foils = new BoatFoilDef[Foils.Length]; for (int i = 0; i < Foils.Length; i++) { b.Foils[i] = Foils[i]?.Clone()!; }
        b.Propellers = new ThrusterDef[Propellers.Length]; for (int i = 0; i < Propellers.Length; i++) { b.Propellers[i] = Propellers[i]?.Clone()!; }
        return b;
    }

    /// <summary>A small rowing or sailing dinghy about 3 m long and 150 kg: a keel, a rudder and a 400 N outboard; top speed about 5 m/s with a 0.15 m³ hull under water.</summary>
    public static BoatDef Dinghy => Make(length: 3f, thrust: 400f, keelArea: 0.25f, rudderArea: 0.08f, longitudinal: 0.1f, lateral: 1f, vertical: 2f);

    /// <summary>A fast planing boat about 6 m and 800 kg: a small keel, a rudder and 6 kN of thrust; low longitudinal drag.</summary>
    public static BoatDef Speedboat => Make(length: 6f, thrust: 6000f, keelArea: 0.4f, rudderArea: 0.15f, longitudinal: 0.06f, lateral: 1f, vertical: 2f);

    /// <summary>A slow, heavy barge about 20 m: a big flat hull with high drag, a large rudder and 8 kN of thrust.</summary>
    public static BoatDef Barge => Make(length: 20f, thrust: 8000f, keelArea: 1.5f, rudderArea: 1f, longitudinal: 0.2f, lateral: 2f, vertical: 4f);

    private static BoatDef Make(float length, float thrust, float keelArea, float rudderArea, float longitudinal, float lateral, float vertical)
    {
        float stern = -0.5f * length;
        return new BoatDef
        {
            LongitudinalDrag = longitudinal, LateralDrag = lateral, VerticalDrag = vertical,
            Foils =
            [
                new BoatFoilDef { Position = new Vector3F(0f, -0.1f * length, 0f), Area = keelArea, AspectRatio = 1.5f },
                new BoatFoilDef { Position = new Vector3F(0f, -0.05f * length, stern), Area = rudderArea, AspectRatio = 1.5f, SteerGain = 1f },
            ],
            Propellers = [new ThrusterDef { Position = new Vector3F(0f, -0.06f * length, stern), Direction = new Vector3F(0f, 0f, 1f), MaxThrust = thrust, ResponseTime = 0.4f, MaxGimbalAngle = 0.5f, ReverseFraction = 0.5f, Medium = AeroMedium.Water }],
        };
    }

    internal BoatSpec ToSpec()
    {
        var spec = new BoatSpec
        {
            Forward = Forward, Up = Up, DragLongitudinal = LongitudinalDrag, DragLateral = LateralDrag, DragVertical = VerticalDrag, HeaveDamping = HeaveDamping, MaxRudderAngle = MaxRudderAngle, RudderRate = RudderRate,
            Propellers = ThrusterDef.ToSpecs(Propellers),
        };
        var foils = Foils ?? Array.Empty<BoatFoilDef>();
        spec.Foils = new SurfaceSpec[foils.Length]; spec.SteerGain = new float[foils.Length];
        var fwd = V3.Direction(Forward, nameof(Forward), false);
        var up = V3.Direction(Up, nameof(Up), false);
        var right = V3.Cross(up, fwd);
        float rl = V3.Len(right);
        right = rl > 1e-6f ? V3.Mul(right, 1f / rl) : up;
        foreach (var p in spec.Propellers) { p.Medium = AeroMedium.Water; }
        for (int i = 0; i < foils.Length; i++)
        {
            var f = foils[i] ?? throw new ArgumentNullException(nameof(Foils), "Foil " + i.ToString(CultureInfo.InvariantCulture) + " is null.");
            spec.Foils[i] = new SurfaceSpec
            {
                Kind = AeroSurfaceKind.Lifting, Coef = AeroCoefficients.Wing(f.AspectRatio), Area = f.Area, Chord = new Vector3F(-fwd.X, -fwd.Y, -fwd.Z),
                Normal = f.Horizontal ? up : right, Center = f.Position, Medium = AeroMedium.Water,
            };
            spec.SteerGain[i] = f.SteerGain;
        }

        return spec;
    }
}

/// <summary>One lift fan of a hovercraft: a cushion of air under a point of the craft that pushes up against whatever shape or water surface lies along its axis.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public sealed class HoverFanDef
{
    /// <summary>Where the fan is, in the body's frame, metres. Default zero.</summary>
    public Vector3F Position { get; set; }

    /// <summary>The way the fan blows, toward the surface, in the body's frame; finite, not zero, normalised on use. Default (0, −1, 0).</summary>
    public Vector3F Direction { get; set; } = new(0f, -1f, 0f);

    /// <summary>The push at zero gap and full lift, newtons, 0 to 10^9. The push falls with the square of the gap: F = MaxLift · lift · (1 − gap/Range)². Default 1000.</summary>
    public float MaxLift { get; set; } = 1000f;

    /// <summary>The gap beyond which the cushion is gone, metres, 0.001 to 10^4. Default 1.</summary>
    public float Range { get; set; } = 1f;

    /// <summary>Damping of the closing speed, N per m/s at zero gap (scaled by the cushion like the push), 0 to 10^9. Default 0.</summary>
    public float Damping { get; set; }

    /// <summary>A copy.</summary>
    public HoverFanDef Clone() => (HoverFanDef)MemberwiseClone();

    internal HoverFanSpec ToSpec() => new() { Position = Position, Direction = Direction, MaxLift = MaxLift, Range = Range, Damping = Damping };
}

/// <summary>
/// A hovercraft: lift fans over a surface (a ray along each fan's axis against every shape but the craft's own, and the water's surface), propulsion thrusters whose gimbals steer, and a skirt that drags on motion along the surface.
/// See <see cref="PhysicsWorld3D.AddHovercraft"/>.
/// </summary>
/// <remarks>
/// At rest a fan holds its share W of the weight at the gap h = Range · (1 − √(W / (MaxLift · lift))). A dynamic body under a fan takes the opposite force at the point under the fan. The skirt and the angular damping scale with the mean squared cushion over the fans.
/// </remarks>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public sealed class HovercraftDef
{
    /// <summary>Which way is forward, in the body's frame. Default (0, 0, 1).</summary>
    public Vector3F Forward { get; set; } = new(0f, 0f, 1f);

    /// <summary>Which way is up, in the body's frame. Default (0, 1, 0).</summary>
    public Vector3F Up { get; set; } = new(0f, 1f, 0f);

    /// <summary>Lift fans; at least one.</summary>
    public HoverFanDef[] Fans { get; set; } = Array.Empty<HoverFanDef>();

    /// <summary>Propulsion thrusters (any medium; their gimbal steers about the up axis).</summary>
    public ThrusterDef[] Thrusters { get; set; } = Array.Empty<ThrusterDef>();

    /// <summary>Drag of the skirt on motion along the surface, N per m/s with the whole cushion up, 0 to 10^9. Default 0.</summary>
    public float SkirtDrag { get; set; }

    /// <summary>Damping of rotation while there is cushion, N·m per rad/s, 0 to 10^9. Default 0.</summary>
    public float AngularDamping { get; set; }

    /// <summary>A copy.</summary>
    public HovercraftDef Clone()
    {
        var h = (HovercraftDef)MemberwiseClone();
        h.Fans = new HoverFanDef[Fans.Length]; for (int i = 0; i < Fans.Length; i++) { h.Fans[i] = Fans[i]?.Clone()!; }
        h.Thrusters = new ThrusterDef[Thrusters.Length]; for (int i = 0; i < Thrusters.Length; i++) { h.Thrusters[i] = Thrusters[i]?.Clone()!; }
        return h;
    }

    /// <summary>A small craft of 200 kg about 2 m long: four fans that hold it at 0.15 m, two rear thrusters of 300 N with gimbals, a skirt and damping.</summary>
    public static HovercraftDef Small => ForMass(200f);

    /// <summary>The <see cref="Small"/> layout scaled for a body of the given mass: fans that hold it at 0.15 m (lift twice its weight at zero gap, range 0.5 m) and thrusters of 150 N per 100 kg.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The mass is not within 1e-3 to 1e9 kg.</exception>
    public static HovercraftDef ForMass(float massKg)
    {
        if (!(massKg >= 1e-3f && massKg <= 1e9f)) { throw new ArgumentOutOfRangeException(nameof(massKg), massKg, "The mass must be within 1e-3 to 1e9 kg."); }
        float share = massKg * 9.81f / 4f;
        float lift = 2f * share;
        HoverFanDef Fan(float x, float z) => new() { Position = new Vector3F(x, -0.1f, z), MaxLift = lift, Range = 0.5f, Damping = 0.4f * lift };
        ThrusterDef Fwd(float x) => new() { Position = new Vector3F(x, 0.2f, -1f), Direction = new Vector3F(0f, 0f, 1f), MaxThrust = 1.5f * massKg, ResponseTime = 0.3f, MaxGimbalAngle = 0.5f };
        return new HovercraftDef
        {
            Fans = [Fan(0.6f, 0.8f), Fan(-0.6f, 0.8f), Fan(0.6f, -0.8f), Fan(-0.6f, -0.8f)],
            Thrusters = [Fwd(0.5f), Fwd(-0.5f)],
            SkirtDrag = 0.4f * massKg, AngularDamping = 0.3f * massKg,
        };
    }

    internal HoverSpec ToSpec()
    {
        var fans = Fans ?? Array.Empty<HoverFanDef>();
        var spec = new HoverSpec { Forward = Forward, Up = Up, SkirtDrag = SkirtDrag, AngularDamping = AngularDamping, Thrusters = ThrusterDef.ToSpecs(Thrusters), Fans = new HoverFanSpec[fans.Length] };
        for (int i = 0; i < fans.Length; i++) { spec.Fans[i] = (fans[i] ?? throw new ArgumentNullException(nameof(Fans), "Fan " + i.ToString(CultureInfo.InvariantCulture) + " is null.")).ToSpec(); }
        return spec;
    }
}
