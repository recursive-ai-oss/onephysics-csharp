using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Aero;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// An aerodynamic surface on a shape (set <see cref="ShapeDef.Aero"/> or <see cref="Shape.Aero"/>): lift and drag from the shape's orientation to the relative flow of the air (the world's <see cref="PhysicsWorld2D.AirDensity"/> and the wind of its area fields)
/// or of the water, applied to the body at the surface's centre of pressure, all in the plane. Areas are per metre of depth (a width in metres), like every planar mass and force. The model is documented on <see cref="AeroCoefficients"/>.
/// </summary>
/// <remarks>
/// A <see cref="AeroSurfaceKind.Lifting"/> surface has a direction <see cref="Forward"/> (the way it flies at zero angle of attack, the leading edge pointing that way) and a lifting side <see cref="Up"/>. With <see cref="Area"/> 0 the area is the shape's width across
/// <see cref="Up"/> (its chord), taken once from the shape's geometry. A <see cref="AeroSurfaceKind.Bluff"/> surface has drag only, on the shape's width across the flow each step when <see cref="Area"/> is 0. Segments and chains have no width; give them an area.
/// Planar shapes have no frame of their own: directions are in the body's frame and <see cref="CenterOfPressure"/> is measured from the shape's centroid. Sleeping bodies feel no aerodynamic forces.
/// </remarks>
public sealed class AeroSurface
{
    /// <summary>Lifting (default) or bluff.</summary>
    public AeroSurfaceKind Kind { get; set; }

    /// <summary>The lift and drag curves. Default <see cref="AeroCoefficients.Wing"/> with aspect ratio 6.</summary>
    public AeroCoefficients Coefficients { get; set; } = AeroCoefficients.Wing();

    /// <summary>Reference area per metre of depth (a width), m, finite, 0 to 10^6; 0 takes it from the shape. Default 0.</summary>
    public float Area { get; set; }

    /// <summary>Direction of travel at zero angle of attack, in the body's frame; finite, not zero, normalised on use. Default (1, 0).</summary>
    public Vector2F Forward { get; set; } = new(1f, 0f);

    /// <summary>The lifting side, in the body's frame; need not be perpendicular to <see cref="Forward"/> (the part along it is removed). Default (0, 1).</summary>
    public Vector2F Up { get; set; } = new(0f, 1f);

    /// <summary>Where the force acts, as an offset from the shape's centroid in the body's axes, metres. Default zero.</summary>
    public Vector2F CenterOfPressure { get; set; }

    /// <summary>Air (default) or water.</summary>
    public AeroMedium Medium { get; set; }

    /// <summary>A lifting surface of the given area (0: from the shape) and aspect ratio, finite wing coefficients.</summary>
    public static AeroSurface Wing(float area = 0f, float aspectRatio = 6f) => new() { Area = area, Coefficients = AeroCoefficients.Wing(aspectRatio) };

    /// <summary>A flat plate of the given area and aspect ratio: lower stall, and a broadside drag following Hoerner's flat-plate data.</summary>
    public static AeroSurface Plate(float area = 0f, float aspectRatio = 2f) => new() { Area = area, Coefficients = AeroCoefficients.Plate(aspectRatio) };

    /// <summary>A bluff body with the given drag coefficient, drag only, on the shape's width across the flow.</summary>
    public static AeroSurface Body(float dragCoefficient = 1f, float area = 0f) => new() { Kind = AeroSurfaceKind.Bluff, Coefficients = AeroCoefficients.Bluff(dragCoefficient), Area = area };

    /// <summary>A copy.</summary>
    public AeroSurface Clone() => (AeroSurface)MemberwiseClone();

    internal SurfaceSpec ToSpec() => new()
    {
        Kind = Kind, Coef = Coefficients, Area = Area, Chord = new Vector3F(-Forward.X, -Forward.Y, 0f), Normal = new Vector3F(Up.X, Up.Y, 0f), Center = new Vector3F(CenterOfPressure.X, CenterOfPressure.Y, 0f), Medium = Medium,
    };

    internal static AeroSurface FromSpec(SurfaceSpec s) => new()
    {
        Kind = s.Kind, Coefficients = s.Coef, Area = s.Area, Forward = new Vector2F(-s.Chord.X, -s.Chord.Y), Up = new Vector2F(s.Normal.X, s.Normal.Y), CenterOfPressure = new Vector2F(s.Center.X, s.Center.Y), Medium = s.Medium,
    };
}

/// <summary>A thruster (rocket, jet, fan or propeller) on a body: a force along an axis at a point, with a throttle, a response time and an optional gimbal. See <see cref="PhysicsWorld2D.AddThruster"/>. Forces are per metre of depth.</summary>
public sealed class ThrusterDef
{
    /// <summary>Where the thrust acts, in the body's frame (relative to the body origin), metres. Default zero.</summary>
    public Vector2F Position { get; set; }

    /// <summary>The direction the force pushes the body, in the body's frame; finite, not zero, normalised on use. Default (0, 1).</summary>
    public Vector2F Direction { get; set; } = new(0f, 1f);

    /// <summary>Thrust at full throttle, newtons, 0 to 10^9. Default 100.</summary>
    public float MaxThrust { get; set; } = 100f;

    /// <summary>Time constant of the first-order lag between the throttle and the thrust, seconds, 0 to 60. Default 0.1; 0 follows the throttle at once.</summary>
    public float ResponseTime { get; set; } = 0.1f;

    /// <summary>Largest gimbal deflection, radians, 0 to 1.2: the thrust turns in the plane by gimbal command × this. Default 0 (fixed). Positive turns counter-clockwise.</summary>
    public float MaxGimbalAngle { get; set; }

    /// <summary>The fraction of <see cref="MaxThrust"/> available in reverse (throttle below 0), 0 to 1. Default 0: no reverse.</summary>
    public float ReverseFraction { get; set; }

    /// <summary>For a propeller: the speed along the axis, relative to the medium, at which thrust has run out (thrust falls linearly to it). 0 (default) means constant thrust.</summary>
    public float PitchSpeed { get; set; }

    /// <summary>Air (default: works anywhere) or water (works only under the surface; fades in over <see cref="ImmersionDepth"/>).</summary>
    public AeroMedium Medium { get; set; }

    /// <summary>For a water thruster: the depth over which the thrust fades in as the propeller goes under, metres, 0.001 to 1000. Default 0.3.</summary>
    public float ImmersionDepth { get; set; } = 0.3f;

    /// <summary>A rocket motor: constant thrust of the given size along +Y with a short response and no reverse.</summary>
    public static ThrusterDef Rocket(float maxThrust, Vector2F position = default, Vector2F direction = default) =>
        new() { MaxThrust = maxThrust, Position = position, Direction = direction.X == 0f && direction.Y == 0f ? new Vector2F(0f, 1f) : direction, ResponseTime = 0.05f };

    /// <summary>A copy.</summary>
    public ThrusterDef Clone() => (ThrusterDef)MemberwiseClone();

    internal ThrusterSpec ToSpec() => new()
    {
        Position = new Vector3F(Position.X, Position.Y, 0f), Direction = new Vector3F(Direction.X, Direction.Y, 0f), MaxThrust = MaxThrust, ResponseTime = ResponseTime, MaxGimbal = MaxGimbalAngle,
        ReverseFraction = ReverseFraction, PitchSpeed = PitchSpeed, ImmersionDepth = ImmersionDepth, Medium = Medium,
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
public static class ThrusterRig
{
    /// <summary>
    /// A lander: a main engine at the body origin pushing along +Y, and two side thrusters at <paramref name="halfWidth"/> either side of it and <paramref name="height"/> above the origin pushing sideways toward the middle, so that firing the
    /// left one pushes the lander right and turns it clockwise about the origin. Order: main, left, right. Responses 0.05 s; the main engine gimbals by up to 0.1 rad.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">A thrust is negative or a size is not within 0.001 to 1000.</exception>
    public static ThrusterDef[] Lander(float mainThrust, float sideThrust, float halfWidth, float height)
    {
        if (!(mainThrust >= 0f && mainThrust <= 1e9f)) { throw new ArgumentOutOfRangeException(nameof(mainThrust), mainThrust, "The thrust must be within 0 to 1e9 N."); }
        if (!(sideThrust >= 0f && sideThrust <= 1e9f)) { throw new ArgumentOutOfRangeException(nameof(sideThrust), sideThrust, "The thrust must be within 0 to 1e9 N."); }
        if (!(halfWidth >= 0.001f && halfWidth <= 1000f)) { throw new ArgumentOutOfRangeException(nameof(halfWidth), halfWidth, "The half width must be within 0.001 to 1000 m."); }
        if (!(height >= 0.001f && height <= 1000f)) { throw new ArgumentOutOfRangeException(nameof(height), height, "The height must be within 0.001 to 1000 m."); }
        return
        [
            new ThrusterDef { MaxThrust = mainThrust, ResponseTime = 0.05f, MaxGimbalAngle = 0.1f },
            new ThrusterDef { Position = new Vector2F(-halfWidth, height), Direction = new Vector2F(1f, 0f), MaxThrust = sideThrust, ResponseTime = 0.05f },
            new ThrusterDef { Position = new Vector2F(halfWidth, height), Direction = new Vector2F(-1f, 0f), MaxThrust = sideThrust, ResponseTime = 0.05f },
        ];
    }
}

/// <summary>A keel, rudder or hydrofoil of a boat in a side view: a lifting surface in water, lifting along the boat's up axis.</summary>
public sealed class BoatFoilDef
{
    /// <summary>Where the force acts, in the body's frame, metres. Default zero.</summary>
    public Vector2F Position { get; set; }

    /// <summary>Area per metre of depth (the chord), m, above 0 and at most 10^6. Default 0.1.</summary>
    public float Area { get; set; } = 0.1f;

    /// <summary>Aspect ratio for the lift slope and the induced drag, 0.1 to 100. Default 2.</summary>
    public float AspectRatio { get; set; } = 2f;

    /// <summary>How far the foil follows the steering command: 1 for a trim foil that turns with it, 0 for a fixed one; the foil turns by the rudder angle × gain (counter-clockwise for positive). −10 to 10. Default 0.</summary>
    public float SteerGain { get; set; }

    /// <summary>A copy.</summary>
    public BoatFoilDef Clone() => (BoatFoilDef)MemberwiseClone();
}

/// <summary>
/// A boat in a side view: a hull on a dynamic body that floats on the world's water (<see cref="PhysicsWorld2D.AddBuoyancyEffector"/>), with hydrodynamic hull drag along and across its length, hydrofoils, and propellers. See <see cref="PhysicsWorld2D.AddBoat"/>.
/// </summary>
/// <remarks>
/// Hull drag is a quadratic law per hull axis in volumetric coefficients: force = ½ρ·C·√A·|u|·u for the velocity u of the water relative to the hull along that axis and the area A of the hull under water (per metre of depth); it replaces the buoyancy
/// field's own linear drag for this body (the buoyancy and angular damping still apply). Every drag is limited so that it cannot do more than stop the motion along that axis within a step. A side view has no leeway: only the longitudinal and vertical coefficients count.
/// Positions are in the body's frame. The presets assume a hull modelled with its origin at the middle of the waterline, +X forward and +Y up.
/// </remarks>
public sealed class BoatDef
{
    /// <summary>Which way is forward, in the body's frame. Default (1, 0).</summary>
    public Vector2F Forward { get; set; } = new(1f, 0f);

    /// <summary>Which way is up, in the body's frame. Default (0, 1).</summary>
    public Vector2F Up { get; set; } = new(0f, 1f);

    /// <summary>Hull drag coefficient along the boat, 0 to 1000. Default 0.1.</summary>
    public float LongitudinalDrag { get; set; } = 0.1f;

    /// <summary>Hull drag coefficient along the up axis (heave), 0 to 1000. Default 2.</summary>
    public float VerticalDrag { get; set; } = 2f;

    /// <summary>Linear damping of motion along the up axis (heave), per second: force = damping · ρ · A · u, A the area under water. It stands for the energy waves carry away, so a floating hull stops bobbing. 0 to 1000. Default 3.</summary>
    public float HeaveDamping { get; set; } = 3f;

    /// <summary>Largest foil deflection, radians, 0 to 1.2. Default 0.6.</summary>
    public float MaxRudderAngle { get; set; } = 0.6f;

    /// <summary>How fast the foils turn, rad/s, 0.01 to 100. Default 1.5.</summary>
    public float RudderRate { get; set; } = 1.5f;

    /// <summary>Keel, trim foils and hydrofoils.</summary>
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

    /// <summary>A small dinghy about 3 m long: a keel foil, a trim foil at the stern and a 400 N outboard (per metre of depth).</summary>
    public static BoatDef Dinghy => Make(length: 3f, thrust: 400f, keelArea: 0.25f, rudderArea: 0.08f, longitudinal: 0.1f, vertical: 2f);

    /// <summary>A fast boat about 6 m long with 6 kN of thrust and low longitudinal drag.</summary>
    public static BoatDef Speedboat => Make(length: 6f, thrust: 6000f, keelArea: 0.4f, rudderArea: 0.15f, longitudinal: 0.06f, vertical: 2f);

    /// <summary>A slow, heavy barge about 20 m long: a big flat hull with high drag and 8 kN of thrust.</summary>
    public static BoatDef Barge => Make(length: 20f, thrust: 8000f, keelArea: 1.5f, rudderArea: 1f, longitudinal: 0.2f, vertical: 4f);

    private static BoatDef Make(float length, float thrust, float keelArea, float rudderArea, float longitudinal, float vertical)
    {
        float stern = -0.5f * length;
        return new BoatDef
        {
            LongitudinalDrag = longitudinal, VerticalDrag = vertical,
            Foils =
            [
                new BoatFoilDef { Position = new Vector2F(0f, -0.1f * length), Area = keelArea, AspectRatio = 1.5f },
                new BoatFoilDef { Position = new Vector2F(stern, -0.05f * length), Area = rudderArea, AspectRatio = 1.5f, SteerGain = 1f },
            ],
            Propellers = [new ThrusterDef { Position = new Vector2F(stern, -0.06f * length), Direction = new Vector2F(1f, 0f), MaxThrust = thrust, ResponseTime = 0.4f, MaxGimbalAngle = 0.3f, ReverseFraction = 0.5f, Medium = AeroMedium.Water }],
        };
    }

    internal BoatSpec ToSpec()
    {
        var spec = new BoatSpec
        {
            Forward = new Vector3F(Forward.X, Forward.Y, 0f), Up = new Vector3F(Up.X, Up.Y, 0f), DragLongitudinal = LongitudinalDrag, DragLateral = 0f, DragVertical = VerticalDrag, HeaveDamping = HeaveDamping,
            MaxRudderAngle = MaxRudderAngle, RudderRate = RudderRate, Propellers = ThrusterDef.ToSpecs(Propellers),
        };
        var foils = Foils ?? Array.Empty<BoatFoilDef>();
        spec.Foils = new SurfaceSpec[foils.Length]; spec.SteerGain = new float[foils.Length];
        var fwd = V3.Direction(spec.Forward, nameof(Forward), true);
        var up = V3.Direction(spec.Up, nameof(Up), true);
        foreach (var p in spec.Propellers) { p.Medium = AeroMedium.Water; }
        for (int i = 0; i < foils.Length; i++)
        {
            var f = foils[i] ?? throw new ArgumentNullException(nameof(Foils), "Foil " + i.ToString(CultureInfo.InvariantCulture) + " is null.");
            spec.Foils[i] = new SurfaceSpec
            {
                Kind = AeroSurfaceKind.Lifting, Coef = AeroCoefficients.Wing(f.AspectRatio), Area = f.Area, Chord = new Vector3F(-fwd.X, -fwd.Y, 0f),
                Normal = up, Center = new Vector3F(f.Position.X, f.Position.Y, 0f), Medium = AeroMedium.Water,
            };
            spec.SteerGain[i] = f.SteerGain;
        }

        return spec;
    }
}

/// <summary>One lift fan of a hovercraft: a cushion of air under a point of the craft that pushes up against whatever shape or water surface lies along its axis.</summary>
public sealed class HoverFanDef
{
    /// <summary>Where the fan is, in the body's frame, metres. Default zero.</summary>
    public Vector2F Position { get; set; }

    /// <summary>The way the fan blows, toward the surface, in the body's frame; finite, not zero, normalised on use. Default (0, −1).</summary>
    public Vector2F Direction { get; set; } = new(0f, -1f);

    /// <summary>The push at zero gap and full lift, newtons per metre of depth, 0 to 10^9. The push falls with the square of the gap: F = MaxLift · lift · (1 − gap/Range)². Default 1000.</summary>
    public float MaxLift { get; set; } = 1000f;

    /// <summary>The gap beyond which the cushion is gone, metres, 0.001 to 10^4. Default 1.</summary>
    public float Range { get; set; } = 1f;

    /// <summary>Damping of the closing speed, N per m/s at zero gap (scaled by the cushion like the push), 0 to 10^9. Default 0.</summary>
    public float Damping { get; set; }

    /// <summary>A copy.</summary>
    public HoverFanDef Clone() => (HoverFanDef)MemberwiseClone();

    internal HoverFanSpec ToSpec() => new() { Position = new Vector3F(Position.X, Position.Y, 0f), Direction = new Vector3F(Direction.X, Direction.Y, 0f), MaxLift = MaxLift, Range = Range, Damping = Damping };
}

/// <summary>
/// A hovercraft in a side view: lift fans over a surface (a ray along each fan's axis against every shape but the craft's own, and the surface of a buoyancy field), propulsion thrusters whose gimbals steer (tilt) them, and a skirt that drags on motion along the surface.
/// See <see cref="PhysicsWorld2D.AddHovercraft"/>.
/// </summary>
/// <remarks>At rest a fan holds its share W of the weight at the gap h = Range · (1 − √(W / (MaxLift · lift))). A dynamic body under a fan takes the opposite force at the point under the fan.</remarks>
public sealed class HovercraftDef
{
    /// <summary>Which way is forward, in the body's frame. Default (1, 0).</summary>
    public Vector2F Forward { get; set; } = new(1f, 0f);

    /// <summary>Which way is up, in the body's frame. Default (0, 1).</summary>
    public Vector2F Up { get; set; } = new(0f, 1f);

    /// <summary>Lift fans; at least one.</summary>
    public HoverFanDef[] Fans { get; set; } = Array.Empty<HoverFanDef>();

    /// <summary>Propulsion thrusters (any medium; the steering command tilts them).</summary>
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

    /// <summary>A small craft of 200 kg per metre of depth about 2 m long: two fans that hold it at 0.15 m, a rear thruster and a skirt.</summary>
    public static HovercraftDef Small => ForMass(200f);

    /// <summary>The <see cref="Small"/> layout scaled for a body of the given mass: fans that hold it at 0.15 m (lift twice its weight at zero gap, range 0.5 m) and a thruster of 150 N per 100 kg.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The mass is not within 1e-3 to 1e9 kg.</exception>
    public static HovercraftDef ForMass(float massKg)
    {
        if (!(massKg >= 1e-3f && massKg <= 1e9f)) { throw new ArgumentOutOfRangeException(nameof(massKg), massKg, "The mass must be within 1e-3 to 1e9 kg."); }
        float share = massKg * 9.81f / 2f;
        float lift = 2f * share;
        HoverFanDef Fan(float x) => new() { Position = new Vector2F(x, -0.1f), MaxLift = lift, Range = 0.5f, Damping = 0.4f * lift };
        return new HovercraftDef
        {
            Fans = [Fan(0.8f), Fan(-0.8f)],
            Thrusters = [new ThrusterDef { Position = new Vector2F(-1f, 0.2f), Direction = new Vector2F(1f, 0f), MaxThrust = 1.5f * massKg, ResponseTime = 0.3f, MaxGimbalAngle = 0.5f }],
            SkirtDrag = 0.4f * massKg, AngularDamping = 0.3f * massKg,
        };
    }

    internal HoverSpec ToSpec()
    {
        var fans = Fans ?? Array.Empty<HoverFanDef>();
        var spec = new HoverSpec
        {
            Forward = new Vector3F(Forward.X, Forward.Y, 0f), Up = new Vector3F(Up.X, Up.Y, 0f), SkirtDrag = SkirtDrag, AngularDamping = AngularDamping,
            Thrusters = ThrusterDef.ToSpecs(Thrusters), Fans = new HoverFanSpec[fans.Length],
        };
        for (int i = 0; i < fans.Length; i++) { spec.Fans[i] = (fans[i] ?? throw new ArgumentNullException(nameof(Fans), "Fan " + i.ToString(CultureInfo.InvariantCulture) + " is null.")).ToSpec(); }
        return spec;
    }
}
