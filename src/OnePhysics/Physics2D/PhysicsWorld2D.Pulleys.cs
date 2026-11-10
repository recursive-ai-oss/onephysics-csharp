using System;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Cables;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>Number of live pulley systems.</summary>
    public int PulleySystemCount => _engine.Cables.Count;

    /// <summary>Adds a pulley system from a definition (tier 2): ropes over wheels, winches, belts and chains, built from the bodies' current poses. See <see cref="PulleySystemDef"/> for the limits.</summary>
    /// <exception cref="ArgumentException">A node is malformed, a body is not alive in this world, or the geometry has no taut path (a rope end inside a wheel).</exception>
    /// <exception cref="ArgumentOutOfRangeException">A parameter is outside its documented range, or the rope is shorter than its path.</exception>
    public PulleySystem AddPulleySystem(PulleySystemDef definition)
    {
        ArgumentNullException.ThrowIfNull(definition);
        var h = _engine.Cables.Create(definition.ToSpecs(), definition.Closed, definition.Chain, definition.Length, definition.Slack, definition.Iterations, definition.BreakForce, definition.Preload, definition.Pitch);
        return new PulleySystem(_engine.Cables, h.Slot, h.Generation);
    }

    /// <summary>
    /// Adds a rope from <paramref name="from"/> over <paramref name="pulleys"/> in order to <paramref name="to"/>, taut (plus <paramref name="slack"/> metres of extra rope). The rope length is conserved across the wheels;
    /// with friction on a wheel it slips when the tension ratio reaches e^(friction × wrap angle). A block-and-tackle is a rope through several pulleys of two blocks.
    /// </summary>
    public PulleySystem AddRopeOverPulleys(RopeEnd from, ReadOnlySpan<Pulley> pulleys, RopeEnd to, double slack = 0)
    {
        if (!(slack >= 0 && double.IsFinite(slack))) throw new ArgumentOutOfRangeException(nameof(slack), slack, "Slack must be finite and not negative.");
        var nodes = new PulleyNodeDef[pulleys.Length + 2];
        nodes[0] = PulleyNodeDef.Anchor(from.Body, from.Point);
        for (int i = 0; i < pulleys.Length; i++) nodes[i + 1] = pulleys[i].ToNode();
        nodes[^1] = PulleyNodeDef.Anchor(to.Body, to.Point);
        return AddPulleySystem(new PulleySystemDef { Nodes = nodes, Slack = slack });
    }

    /// <summary>
    /// Adds a winch: a drum of <paramref name="drumRadius"/> metres turning on <paramref name="carrier"/> (a mounted body, or the world when default) with the rope wound on it, running to <paramref name="load"/>.
    /// The drum is node 0: drive it with <c>SetMotor(0, speed, maxTorque)</c> and hold it with <c>SetBrake(0, torque)</c>; its spin pays the rope in and out. The rope's free length is <c>FreeLength(0)</c>.
    /// </summary>
    public PulleySystem AddWinch(Body carrier, Vector2D drumCenter, float drumRadius, RopeEnd load, WrapDirection direction = WrapDirection.CounterClockwise, float drumInertia = 1, float maxTorque = 0, float brakeTorque = 0)
    {
        var drum = PulleyNodeDef.Drum(carrier.RawHandle, drumCenter, drumRadius, direction, drumInertia);
        drum.FixedAxle = true; drum.MotorTorque = maxTorque; drum.BrakeTorque = brakeTorque;
        return AddPulleySystem(new PulleySystemDef { Nodes = [drum, PulleyNodeDef.Anchor(load.Body, load.Point)] });
    }

    /// <summary>
    /// Adds a belt: a closed loop over <paramref name="wheels"/> in order, with Coulomb <paramref name="friction"/> between belt and rims. Torque passes from wheel to wheel through the belt's tension difference and
    /// is limited by the capstan relation; <paramref name="preload"/> is the least tension in every span (a belt that is slack on one side transmits nothing without it). The wheels turn with their bodies: give each a body on a hinge.
    /// </summary>
    public PulleySystem AddBelt(ReadOnlySpan<Pulley> wheels, float friction = 0.3f, float preload = 0, double slack = 0)
    {
        var nodes = new PulleyNodeDef[wheels.Length];
        for (int i = 0; i < nodes.Length; i++)
        {
            nodes[i] = wheels[i].ToNode(); nodes[i].Friction = friction;
            if (!wheels[i].Body.IsNull) { nodes[i].Spin = WheelSpin.Body; nodes[i].FixedAxle = true; }
        }
        return AddPulleySystem(new PulleySystemDef { Nodes = nodes, Closed = true, Preload = preload, Slack = slack });
    }

    /// <summary>
    /// Adds a chain drive over <paramref name="sprockets"/>: a closed loop that never slips, whose speed ratio is the tooth ratio exactly (a sprocket's pitch radius is teeth × pitch / 2π).
    /// The chain's length is <paramref name="links"/> × <paramref name="pitch"/> (0 picks the shortest even number of links that reaches around the sprockets); a chain longer than the taut path hangs slack on the unloaded side, and one shorter than the path throws.
    /// </summary>
    public PulleySystem AddChainDrive(ReadOnlySpan<Sprocket> sprockets, float pitch, int links = 0)
    {
        if (!(pitch >= 0.001f && pitch <= 10f)) throw new ArgumentOutOfRangeException(nameof(pitch), pitch, "Pitch must be 0.001 to 10 metres.");
        var nodes = new PulleyNodeDef[sprockets.Length];
        for (int i = 0; i < nodes.Length; i++)
        {
            var s = sprockets[i];
            if (s.Teeth < 4 || s.Teeth > 1000) throw new ArgumentOutOfRangeException(nameof(sprockets), s.Teeth, "A sprocket has 4 to 1000 teeth.");
            nodes[i] = new PulleyNodeDef { Kind = CableNodeKind.Wheel, Body = s.Body, Point = s.Center, Radius = (float)(s.Teeth * (double)pitch / (2 * Math.PI)), NoSlip = true, Spin = WheelSpin.Body, FixedAxle = s.Mounted };
        }
        if (links < 0 || links > 100000) throw new ArgumentOutOfRangeException(nameof(links), links, "Links must be 0 (automatic) to 100000.");
        var def = new PulleySystemDef { Nodes = nodes, Closed = true, Chain = true, Pitch = pitch };
        double taut = _engine.Cables.TautLength(def.ToSpecs(), true, true);
        int count = links > 0 ? links : Math.Max(2, 2 * (int)Math.Ceiling(taut / pitch / 2 - 1e-9));
        def.Length = count * (double)pitch;
        return AddPulleySystem(def);
    }
}
