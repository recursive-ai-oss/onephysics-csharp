using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Debug;

/// <summary>A straight span of a rope, belt or chain between its tangent points, in world coordinates relative to the origin. Plain data for drawing.</summary>
public readonly struct DebugPulleySpan
{
    /// <summary>Creates a span entry.</summary>
    public DebugPulleySpan(int system, int span, Vector3D start, Vector3D end, double tension, bool slack)
    { System = system; Span = span; Start = start; End = end; Tension = tension; Slack = slack; }
    /// <summary>Pulley system slot.</summary>
    public int System { get; }
    /// <summary>Span number along the path.</summary>
    public int Span { get; }
    /// <summary>Where the rope leaves its start node.</summary>
    public Vector3D Start { get; }
    /// <summary>Where the rope reaches its end node.</summary>
    public Vector3D End { get; }
    /// <summary>Tension in newtons in the last step.</summary>
    public double Tension { get; }
    /// <summary>True when the span carries no tension and the rope there is longer than its straight line (draw a sag).</summary>
    public bool Slack { get; }
}

/// <summary>A wheel, drum or sprocket of a pulley system with the arc the rope wraps. Plain data for drawing.</summary>
public readonly struct DebugPulleyWheel
{
    /// <summary>Creates a wheel entry.</summary>
    public DebugPulleyWheel(int system, int node, Vector3D center, Vector3F axis, float radius, Vector3D enter, Vector3D leave, double wrap, float spin)
    { System = system; Node = node; Center = center; Axis = axis; Radius = radius; Enter = enter; Leave = leave; Wrap = wrap; Spin = spin; }
    /// <summary>Pulley system slot.</summary>
    public int System { get; }
    /// <summary>Node index along the path.</summary>
    public int Node { get; }
    /// <summary>Wheel centre.</summary>
    public Vector3D Center { get; }
    /// <summary>Unit axle direction in world coordinates.</summary>
    public Vector3F Axis { get; }
    /// <summary>Radius, metres.</summary>
    public float Radius { get; }
    /// <summary>Where the rope touches the rim on its way in (the centre itself for a rope end).</summary>
    public Vector3D Enter { get; }
    /// <summary>Where the rope leaves the rim.</summary>
    public Vector3D Leave { get; }
    /// <summary>Angle the rope wraps, radians, extra turns included.</summary>
    public double Wrap { get; }
    /// <summary>Rotor spin in rad/s (zero for a wheel that turns with its body).</summary>
    public float Spin { get; }
}

public sealed partial class DebugGeometry
{
    private DebugPulleySpan[] _pulleySpans = new DebugPulleySpan[4];
    private int _nPulleySpans;
    private DebugPulleyWheel[] _pulleyWheels = new DebugPulleyWheel[4];
    private int _nPulleyWheels;
    /// <summary>Rope spans in system and span order, valid until Clear or the next world collection.</summary>
    public ReadOnlySpan<DebugPulleySpan> PulleySpans => _pulleySpans.AsSpan(0, _nPulleySpans);
    /// <summary>Wheels in system and node order, valid until Clear or the next world collection.</summary>
    public ReadOnlySpan<DebugPulleyWheel> PulleyWheels => _pulleyWheels.AsSpan(0, _nPulleyWheels);
    /// <summary>Appends a span entry; grows owned storage when needed.</summary>
    public void Add(in DebugPulleySpan span) => Push(ref _pulleySpans, ref _nPulleySpans, in span);
    /// <summary>Appends a wheel entry; grows owned storage when needed.</summary>
    public void Add(in DebugPulleyWheel wheel) => Push(ref _pulleyWheels, ref _nPulleyWheels, in wheel);
}
