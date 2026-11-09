using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Cables;

/// <summary>One node as authored, in the 3D vocabulary both dimensions share (a planar world passes Z zero and the Z axis).</summary>
internal struct CableNodeSpec
{
    internal CableNodeKind Kind;
    internal BodyHandle Body;
    internal Vector3D Point;
    internal Vector3F Axis;
    internal float Radius;
    internal WrapDirection Direction;
    internal float Friction;
    internal bool NoSlip;
    internal WheelSpin Spin;
    internal float Inertia;
    internal float Bearing;
    internal bool FixedAxle;
    internal int ExtraTurns;
    internal float MotorSpeed, MotorTorque, BrakeTorque;
}

internal sealed partial class CableStore
{
    private static void CheckFinite(double v, double bound, string what)
    {
        if (!(Math.Abs(v) <= bound)) throw new ArgumentOutOfRangeException(what, v, $"{what} must be finite within ±{bound}.");
    }

    /// <summary>Validates an authored pulley system, builds it from the bodies' current poses and stores it. Errors name the node and the field.</summary>
    internal (int Slot, int Generation) Create(ReadOnlySpan<CableNodeSpec> specs, bool closed, bool chain, double length, double slack, int iterations, float breakForce, float preload, float pitch)
        => Add(Build(specs, closed, chain, length, slack, iterations, breakForce, preload, pitch));

    /// <summary>The length of a taut rope along the authored path, in metres (arcs included), for choosing a belt or chain length.</summary>
    internal double TautLength(ReadOnlySpan<CableNodeSpec> specs, bool closed, bool chain)
        => Build(specs, closed, chain, 0, 0, 1, float.PositiveInfinity, 0, 0).TotalMaterial;

    private CableData Build(ReadOnlySpan<CableNodeSpec> specs, bool closed, bool chain, double length, double slack, int iterations, float breakForce, float preload, float pitch)
    {
        if (!(slack >= 0 && slack <= 1e9)) throw new ArgumentOutOfRangeException(nameof(slack), slack, "Slack must be finite and not negative.");
        int count = specs.Length;
        if (count < 2 || count > NodeLimit) throw new ArgumentOutOfRangeException(nameof(specs), count, "A pulley system has 2 to 256 nodes.");
        if (closed && count < 2) throw new ArgumentOutOfRangeException(nameof(specs), "A closed loop needs at least two wheels.");
        if (iterations < 1 || iterations > 256) throw new ArgumentOutOfRangeException(nameof(iterations), iterations, "Iterations must be 1 to 256.");
        if (!(breakForce > 0) || float.IsNaN(breakForce)) throw new ArgumentOutOfRangeException(nameof(breakForce), breakForce, "BreakForce must be positive (infinity means unbreakable).");
        if (!(preload >= 0 && preload <= 1e9f)) throw new ArgumentOutOfRangeException(nameof(preload), preload, "Preload must be finite, 0 to 1e9 newtons.");
        if (!(length >= 0 && length <= 1e9)) throw new ArgumentOutOfRangeException(nameof(length), length, "Length must be 0 (taut) or a finite positive length.");
        var core = Host.CableCore; var bodies = core.Bodies;
        var data = new CableData { Closed = closed, Chain = chain, Iterations = iterations, BreakForce = breakForce, Preload = preload, Pitch = pitch, Nodes = new CableNode[count] };
        var turns = new int[count]; int wheels = 0;
        for (int i = 0; i < count; i++)
        {
            ref readonly var s = ref specs[i];
            string at = $"node {i}";
            if (s.Kind < CableNodeKind.Anchor || s.Kind > CableNodeKind.Drum) throw new ArgumentOutOfRangeException(nameof(specs), $"{at}: unknown kind.");
            if (closed ? s.Kind != CableNodeKind.Wheel : (i == 0 || i == count - 1) ? s.Kind == CableNodeKind.Wheel : s.Kind != CableNodeKind.Wheel)
                throw new ArgumentException($"{at}: an open rope runs from an anchor or drum to an anchor or drum over wheels; a closed loop holds only wheels.", nameof(specs));
            var node = new CableNode { Kind = s.Kind };
            if (!s.Body.IsNull)
            {
                if (s.Body.World != core.Id || !bodies.Handles.IsAlive(s.Body.Index, s.Body.Generation)) throw new ArgumentException($"{at}: the body must belong to this world and be alive.", nameof(specs));
                node.Body = s.Body.Index; node.BodyGeneration = s.Body.Generation;
            }
            CheckFinite(s.Point.X, 2097152, $"{at} point X"); CheckFinite(s.Point.Y, 2097152, $"{at} point Y"); CheckFinite(s.Point.Z, 2097152, $"{at} point Z");
            if (Host.Planar && s.Point.Z != 0) throw new ArgumentOutOfRangeException(nameof(specs), $"{at}: Z must be zero in a planar world.");
            node.Local = s.Point;
            if (s.Kind != CableNodeKind.Anchor)
            {
                wheels++;
                if (!(s.Radius >= 0.001f && s.Radius <= 1000f)) throw new ArgumentOutOfRangeException(nameof(specs), s.Radius, $"{at}: radius must be 0.001 to 1000 metres.");
                if (!(s.Friction >= 0 && s.Friction <= 100)) throw new ArgumentOutOfRangeException(nameof(specs), s.Friction, $"{at}: friction must be 0 to 100.");
                if (!(s.Inertia > 0) || float.IsNaN(s.Inertia)) throw new ArgumentOutOfRangeException(nameof(specs), s.Inertia, $"{at}: inertia must be positive (infinity locks the wheel).");
                if (!(s.Bearing >= 0 && s.Bearing <= 1000)) throw new ArgumentOutOfRangeException(nameof(specs), s.Bearing, $"{at}: bearing damping must be 0 to 1000 per second.");
                if (s.ExtraTurns < 0 || s.ExtraTurns > 64) throw new ArgumentOutOfRangeException(nameof(specs), s.ExtraTurns, $"{at}: extra turns must be 0 to 64.");
                if (!(s.MotorTorque >= 0 && s.MotorTorque <= 1e12f) || !(s.BrakeTorque >= 0 && s.BrakeTorque <= 1e12f) || !(Math.Abs(s.MotorSpeed) <= 1e6f)) throw new ArgumentOutOfRangeException(nameof(specs), $"{at}: motor speed within ±1e6 rad/s, motor and brake torque 0 to 1e12.");
                if (s.Kind == CableNodeKind.Drum && s.Spin != WheelSpin.Rotor) throw new ArgumentException($"{at}: a drum is a rotor wheel.", nameof(specs));
                if (s.Spin == WheelSpin.Body && (s.MotorTorque > 0 || s.BrakeTorque > 0)) throw new ArgumentException($"{at}: a wheel that spins with its body takes its torque from the body's own joints; motors and brakes belong to rotor wheels.", nameof(specs));
                if (s.Spin == WheelSpin.Body && s.Body.IsNull) throw new ArgumentException($"{at}: a wheel that spins with its body needs a body.", nameof(specs));
                if (s.Spin != WheelSpin.Rotor && s.Spin != WheelSpin.Body) throw new ArgumentOutOfRangeException(nameof(specs), $"{at}: unknown spin mode.");
                double al = (double)s.Axis.X * s.Axis.X + (double)s.Axis.Y * s.Axis.Y + (double)s.Axis.Z * s.Axis.Z;
                if (!(al > 1e-12 && al < 1e12)) throw new ArgumentOutOfRangeException(nameof(specs), s.Axis, $"{at}: axis must be finite and nonzero.");
                double inv = 1 / Math.Sqrt(al);
                var axis = new Vector3F((float)(s.Axis.X * inv), (float)(s.Axis.Y * inv), (float)(s.Axis.Z * inv));
                if (Host.Planar && (axis.X != 0 || axis.Y != 0)) throw new ArgumentOutOfRangeException(nameof(specs), s.Axis, $"{at}: the axis of a planar wheel is Z.");
                node.Axis = axis; CableGeometry.Basis(axis, out node.Ref1, out node.Ref2);
                node.Radius = s.Radius; node.Side = s.Direction == WrapDirection.Clockwise ? -1 : 1; node.Friction = s.Friction; node.NoSlip = s.NoSlip || chain;
                node.Mode = s.Spin; node.Inertia = s.Inertia; node.Bearing = s.Bearing; node.FixedAxle = s.FixedAxle;
                node.MotorSpeed = s.MotorSpeed; node.MotorTorque = s.MotorTorque; node.BrakeTorque = s.BrakeTorque;
                turns[i] = s.ExtraTurns;
            }
            data.Nodes[i] = node;
        }
        if (closed && wheels < 2) throw new ArgumentException("A closed loop needs at least two wheels.", nameof(specs));
        Initialize(data, length, slack, turns);
        return data;
    }
}
