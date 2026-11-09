using System;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Cables;

/// <summary>What a node of a pulley system is.</summary>
[Experimental("OPHYS0001")]
public enum CableNodeKind
{
    /// <summary>A point on a body (or in the world) where the rope ends or is tied.</summary>
    Anchor = 0,
    /// <summary>A wheel the rope passes over: a pulley, sheave, sprocket or capstan. The rope wraps it by the angle the geometry gives, plus any extra turns.</summary>
    Wheel = 1,
    /// <summary>A wheel where the rope ends, wound onto its surface (a winch drum). Only the first or last node of an open rope.</summary>
    Drum = 2,
}

/// <summary>Which way the rope turns around a wheel.</summary>
[Experimental("OPHYS0001")]
public enum WrapDirection
{
    /// <summary>Counter-clockwise seen from the tip of the wheel axis (in 2D, counter-clockwise on screen).</summary>
    CounterClockwise = 0,
    /// <summary>Clockwise seen from the tip of the wheel axis.</summary>
    Clockwise = 1,
}

/// <summary>Whether a wheel spins on its own (a rotor with its own inertia, turning on its carrier body) or with its body.</summary>
[Experimental("OPHYS0001")]
public enum WheelSpin
{
    /// <summary>The wheel is a rotor owned by the pulley system and mounted on the node's body (or the world), turning about the axle with a given inertia. Pulleys and winch drums use this.</summary>
    Rotor = 0,
    /// <summary>The wheel is the node's body: it turns with the body's own rotation about the axle. The body is expected to be mounted on a hinge. Sprockets on wheels and cranks use this.</summary>
    Body = 1,
}

/// <summary>One node of a pulley system as stored: geometry in the body's frame plus the wheel's runtime state.</summary>
internal sealed class CableNode
{
    internal CableNodeKind Kind;
    internal int Body = -1, BodyGeneration;
    internal Vector3D Local;
    internal Vector3F Axis = new(0, 0, 1), Ref1 = new(1, 0, 0), Ref2 = new(0, 1, 0);
    internal float Radius;
    internal int Side = 1;
    internal float Friction;
    internal bool NoSlip;
    internal WheelSpin Mode;
    internal float Inertia;
    internal float Bearing;
    internal bool FixedAxle;
    internal float MotorSpeed, MotorTorque, BrakeTorque;
    internal float Spin;
    internal double RotorAngle, AlphaIn, AlphaOut, Reference;
    internal byte SlipState;
    internal float MotorImpulse, BrakeImpulse;
    internal bool IsWheel => Kind != CableNodeKind.Anchor;
    internal CableNode Copy() => (CableNode)MemberwiseClone();
}

/// <summary>One pulley system as stored: a path of nodes, the rope's material length in each span and the multipliers that make the next step start warm.</summary>
internal sealed class CableData
{
    internal CableNode[] Nodes = [];
    internal bool Closed;
    internal bool Chain;
    internal int Iterations = 32;
    internal float BreakForce = float.PositiveInfinity;
    internal float Preload;
    internal float Pitch;
    internal bool Broken;
    internal float LastDt, AdvanceDt;
    internal double[] Material = [];
    internal double[] Impulse = [];
    internal double[] Stretch = [];
    internal double[] Geometric = [];
    internal double[] Wrap = [];
    internal double[] Free = [];
    internal int Spans => Closed ? Nodes.Length : Nodes.Length - 1;
    internal double TotalMaterial { get { double t = 0; foreach (double m in Material) t += m; return t; } }
    internal CableData Copy()
    {
        var c = (CableData)MemberwiseClone();
        c.Nodes = new CableNode[Nodes.Length];
        for (int i = 0; i < Nodes.Length; i++) c.Nodes[i] = Nodes[i].Copy();
        c.Material = (double[])Material.Clone(); c.Impulse = (double[])Impulse.Clone(); c.Stretch = (double[])Stretch.Clone();
        c.Free = (double[])Free.Clone(); c.Geometric = (double[])Geometric.Clone(); c.Wrap = (double[])Wrap.Clone();
        return c;
    }
}
