using System;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Cables;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// One node of a pulley system: an anchor where the rope is tied, a wheel it runs over, or a drum it is wound on. Points and axes are in the body's authoring frame when <see cref="Body"/> is set,
/// otherwise in world coordinates. Metres, kilograms, radians and seconds.
/// </summary>
[Experimental("OPHYS0001")]
public sealed class PulleyNodeDef
{
    /// <summary>Anchor, wheel or drum. Default anchor.</summary>
    public CableNodeKind Kind { get; set; }
    /// <summary>The body that carries the node; the null handle (default) fixes it in the world. Must be alive in the destination world.</summary>
    public BodyHandle Body { get; set; }
    /// <summary>Anchor point or wheel centre, finite within ±2^21 metres.</summary>
    public Vector3D Point { get; set; }
    /// <summary>Wheel axle direction (any nonzero finite vector, normalised on creation); default +Z. The rope turns counter-clockwise about it unless <see cref="Direction"/> says otherwise.</summary>
    public Vector3F Axis { get; set; } = new(0, 0, 1);
    /// <summary>Wheel radius, 0.001 to 1000 metres. For a chain sprocket use the pitch radius.</summary>
    public float Radius { get; set; } = 0.1f;
    /// <summary>Which way the rope turns about the axle. Default counter-clockwise.</summary>
    public WrapDirection Direction { get; set; }
    /// <summary>Coulomb friction between rope and rim, 0 to 100, default 0 (a frictionless sheave). The rope slides when the tension ratio across the wheel reaches e^(friction × wrap angle).</summary>
    public float Friction { get; set; }
    /// <summary>The rope never slips on this wheel (a toothed sprocket or a winch drum); default false.</summary>
    public bool NoSlip { get; set; }
    /// <summary>A rotor owned by the pulley system (default) or the body's own rotation.</summary>
    public WheelSpin Spin { get; set; }
    /// <summary>Rotor inertia about the axle in kg·m², positive; infinity locks the wheel. Default 0.05. Ignored for a wheel that spins with its body.</summary>
    public float Inertia { get; set; } = 0.05f;
    /// <summary>Bearing damping of a rotor, per second, 0 to 1000. Default 0.</summary>
    public float BearingDamping { get; set; }
    /// <summary>The carrier is mounted: a static body, or a body held by a joint (a boom, a chassis, a hinge-mounted wheel). The solver then treats it as immovable and passes the rope's force on to the body, where its joints take it. Applies to anchors too; a body that carries any node marked this way is held for the whole system (it is not predicted to fall). Default false: a free body such as a hanging block moves with the rope.</summary>
    public bool FixedAxle { get; set; }
    /// <summary>Extra full turns of rope around the wheel beyond the geometric wrap, 0 to 64. Default 0.</summary>
    public int ExtraTurns { get; set; }
    /// <summary>Rotor motor target speed relative to its carrier, rad/s. Default 0.</summary>
    public float MotorSpeed { get; set; }
    /// <summary>Largest motor torque in N·m; 0 (default) means no motor.</summary>
    public float MotorTorque { get; set; }
    /// <summary>Holding torque of the brake in N·m; 0 (default) means no brake. The brake holds the rotor still relative to its carrier up to this torque.</summary>
    public float BrakeTorque { get; set; }

    /// <summary>An anchor on a body (<paramref name="local"/> in its authoring frame) or, with the null handle, at a world point.</summary>
    public static PulleyNodeDef Anchor(BodyHandle body, Vector3D local) => new() { Kind = CableNodeKind.Anchor, Body = body, Point = local };
    /// <summary>A wheel on a body or in the world.</summary>
    public static PulleyNodeDef Wheel(BodyHandle body, Vector3D center, float radius, Vector3F? axis = null, WrapDirection direction = WrapDirection.CounterClockwise, float friction = 0)
        => new() { Kind = CableNodeKind.Wheel, Body = body, Point = center, Radius = radius, Axis = axis ?? new(0, 0, 1), Direction = direction, Friction = friction };
    /// <summary>A winch drum: a rotor wheel where the rope ends, never slipping.</summary>
    public static PulleyNodeDef Drum(BodyHandle body, Vector3D center, float radius, Vector3F? axis = null, WrapDirection direction = WrapDirection.CounterClockwise, float inertia = 1)
        => new() { Kind = CableNodeKind.Drum, Body = body, Point = center, Radius = radius, Axis = axis ?? new(0, 0, 1), Direction = direction, Inertia = inertia, NoSlip = true };

    internal CableNodeSpec ToSpec() => new()
    {
        Kind = Kind, Body = Body, Point = Point, Axis = Axis, Radius = Radius, Direction = Direction, Friction = Friction, NoSlip = NoSlip, Spin = Spin, Inertia = Inertia, Bearing = BearingDamping,
        FixedAxle = FixedAxle, ExtraTurns = ExtraTurns, MotorSpeed = MotorSpeed, MotorTorque = MotorTorque, BrakeTorque = BrakeTorque,
    };
}

/// <summary>A rope over wheels, a winch, a belt or a chain: the nodes in path order and the solver options. Creation reads the bodies' current poses, so build the system with the bodies where they should hang.</summary>
[Experimental("OPHYS0001")]
public sealed class PulleySystemDef
{
    /// <summary>Nodes in path order. An open rope runs from an anchor or drum, over wheels, to an anchor or drum; a closed loop holds only wheels. 2 to 256 nodes.</summary>
    public PulleyNodeDef[] Nodes { get; set; } = [];
    /// <summary>True for a belt or chain: a closed loop over its wheels, in order. Default false.</summary>
    public bool Closed { get; set; }
    /// <summary>True for a chain: no wheel slips, and the loop length counts in pitches. Default false.</summary>
    public bool Chain { get; set; }
    /// <summary>Rope length in metres including the arcs wound on wheels and drums; 0 (default) means exactly taut. Longer ropes start slack; a rope shorter than the path throws.</summary>
    public double Length { get; set; }
    /// <summary>Extra rope in metres beyond the taut path, 0 to 1e9, default 0; ignored when <see cref="Length"/> is set.</summary>
    public double Slack { get; set; }
    /// <summary>Solver sweeps per step, 1 to 256, default 32.</summary>
    public int Iterations { get; set; } = 32;
    /// <summary>Tension in newtons above which the system breaks for good, default infinity.</summary>
    public float BreakForce { get; set; } = float.PositiveInfinity;
    /// <summary>Least tension in every span, newtons, 0 to 1e9, default 0: the pull of a belt tensioner. A belt transmits torque only through its slack-side tension.</summary>
    public float Preload { get; set; }
    /// <summary>Chain pitch in metres (information only; the loop length comes from <see cref="Length"/>).</summary>
    public float Pitch { get; set; }

    /// <summary>Validates without creating; throws ArgumentException or ArgumentOutOfRangeException naming the node and field. Body liveness is checked on creation.</summary>
    public void Validate()
    {
        ArgumentNullException.ThrowIfNull(Nodes);
        if (Nodes.Length < 2 || Nodes.Length > 256) throw new ArgumentOutOfRangeException(nameof(Nodes), Nodes.Length, "A pulley system has 2 to 256 nodes.");
        foreach (var n in Nodes) ArgumentNullException.ThrowIfNull(n, nameof(Nodes));
    }

    internal CableNodeSpec[] ToSpecs()
    {
        Validate();
        var specs = new CableNodeSpec[Nodes.Length];
        for (int i = 0; i < specs.Length; i++) specs[i] = Nodes[i].ToSpec();
        return specs;
    }
}

/// <summary>An end of a rope for the one-line constructors: a point on a body, or a world point.</summary>
[Experimental("OPHYS0001")]
public readonly struct RopeEnd
{
    internal RopeEnd(BodyHandle body, Vector3D point) { Body = body; Point = point; }
    internal BodyHandle Body { get; }
    internal Vector3D Point { get; }
    /// <summary>The point <paramref name="local"/> in the body's authoring frame.</summary>
    public static RopeEnd On(Body body, Vector3F local) => new(body.RawHandle, new Vector3D(local.X, local.Y, local.Z));
    /// <summary>A fixed world point.</summary>
    public static RopeEnd At(Vector3D world) => new(default, world);
}

/// <summary>A pulley for the one-line constructors: a frictionless sheave turning freely on its carrier.</summary>
[Experimental("OPHYS0001")]
public readonly struct Pulley
{
    /// <summary>A wheel of <paramref name="radius"/> metres centred at <paramref name="center"/> (in the body's authoring frame, or the world when <paramref name="body"/> is the default).</summary>
    public Pulley(Vector3D center, float radius, Body body = default, bool clockwise = false, float friction = 0, Vector3F? axis = null, bool mounted = false, float inertia = 0.05f)
    { Center = center; Radius = radius; Body = body.RawHandle; Clockwise = clockwise; Friction = friction; Axis = axis ?? new(0, 0, 1); Mounted = mounted; Inertia = inertia; }
    /// <summary>Wheel centre.</summary>
    public Vector3D Center { get; }
    /// <summary>Wheel radius in metres.</summary>
    public float Radius { get; }
    /// <summary>Carrier body; the null handle fixes the wheel in the world.</summary>
    public BodyHandle Body { get; }
    /// <summary>Whether the rope turns clockwise about the axis.</summary>
    public bool Clockwise { get; }
    /// <summary>Rope-to-rim friction coefficient.</summary>
    public float Friction { get; }
    /// <summary>Axle direction.</summary>
    public Vector3F Axis { get; }
    /// <summary>Whether the carrier is held by joints (see <see cref="PulleyNodeDef.FixedAxle"/>).</summary>
    public bool Mounted { get; }
    /// <summary>Rotor inertia in kg·m².</summary>
    public float Inertia { get; }
    internal PulleyNodeDef ToNode() => new() { Kind = CableNodeKind.Wheel, Body = Body, Point = Center, Radius = Radius, Direction = Clockwise ? WrapDirection.Clockwise : WrapDirection.CounterClockwise, Friction = Friction, Axis = Axis, FixedAxle = Mounted || Body.IsNull, Inertia = Inertia };
}

/// <summary>A chain sprocket for <c>AddChainDrive</c>: it turns with its body, which should hang on a hinge.</summary>
[Experimental("OPHYS0001")]
public readonly struct Sprocket
{
    /// <summary>A sprocket of <paramref name="teeth"/> teeth centred at <paramref name="center"/> on <paramref name="body"/>.</summary>
    public Sprocket(Body body, Vector3D center, int teeth, Vector3F? axis = null, bool mounted = true)
    { Body = body.RawHandle; Center = center; Teeth = teeth; Axis = axis ?? new(0, 0, 1); Mounted = mounted; }
    /// <summary>The sprocket's body.</summary>
    public BodyHandle Body { get; }
    /// <summary>Centre in the body's authoring frame.</summary>
    public Vector3D Center { get; }
    /// <summary>Number of teeth, 4 to 1000.</summary>
    public int Teeth { get; }
    /// <summary>Axle direction in the body frame.</summary>
    public Vector3F Axis { get; }
    /// <summary>Whether the body is held by a joint, default true.</summary>
    public bool Mounted { get; }
}
