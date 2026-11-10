using System;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Cables;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// A rope over pulleys, a winch, a belt or a chain: a generation-checked handle. Readbacks describe the last step; a destroyed handle throws InvalidOperationException and the default handle is invalid.
/// Spans are numbered along the path (span i runs from node i to node i+1, the last span of a loop back to node 0).
/// </summary>
public readonly struct PulleySystem
{
    private readonly CableStore? _store;
    private readonly int _generation;
    internal PulleySystem(CableStore store, int index, int generation) { _store = store; Index = index; _generation = generation; }
    /// <summary>Slot index, reused lowest-first after destruction.</summary>
    public int Index { get; }
    /// <summary>True while the slot and generation belong to a live system.</summary>
    public bool IsValid => _store != null && _store.Alive(Index, _generation);
    private CableStore Store => IsValid ? _store! : throw new InvalidOperationException("The pulley system has been destroyed or is uninitialized.");
    private CableData Data => Store.Data[Index]!;
    private CableNode Node(int node) { var d = Data; if ((uint)node >= (uint)d.Nodes.Length) throw new ArgumentOutOfRangeException(nameof(node), node, "Node index outside the pulley system."); return d.Nodes[node]; }
    private CableNode Wheel(int node) { var n = Node(node); if (!n.IsWheel) throw new ArgumentException("The node is an anchor, not a wheel.", nameof(node)); return n; }
    private int Span(int span) { var d = Data; if ((uint)span >= (uint)d.Spans) throw new ArgumentOutOfRangeException(nameof(span), span, "Span index outside the pulley system."); return span; }

    /// <summary>Number of nodes.</summary>
    public int NodeCount => Data.Nodes.Length;
    /// <summary>Number of spans: nodes minus one for a rope, equal to the nodes for a loop.</summary>
    public int SpanCount => Data.Spans;
    /// <summary>True for a belt or chain loop.</summary>
    public bool IsClosed => Data.Closed;
    /// <summary>True once a span's tension passed <see cref="BreakForce"/>, or a node's body was destroyed. A broken system exerts no force and stays broken.</summary>
    public bool IsBroken => Data.Broken;
    /// <summary>Tension in newtons above which the system breaks; positive, infinity by default.</summary>
    public float BreakForce
    {
        get => Data.BreakForce;
        set { var s = Store; if (!(value > 0) || float.IsNaN(value)) throw new ArgumentOutOfRangeException(nameof(value), value, "BreakForce must be positive."); using var m = s.Host.CableMutation("PulleySystem.BreakForce"); Data.BreakForce = value; }
    }
    /// <summary>Least tension per span in newtons, 0 to 1e9 (see <see cref="PulleySystemDef.Preload"/>).</summary>
    public float Preload
    {
        get => Data.Preload;
        set { var s = Store; if (!(value >= 0 && value <= 1e9f)) throw new ArgumentOutOfRangeException(nameof(value), value, "Preload must be 0 to 1e9 newtons."); using var m = s.Host.CableMutation("PulleySystem.Preload"); Data.Preload = value; }
    }
    /// <summary>Total rope material in metres: the sum of the spans' material lengths (arcs included). Conserved by slipping; changed only by <see cref="AdjustLength"/>.</summary>
    public double TotalLength => Data.TotalMaterial;
    /// <summary>Material length of a span: the rope between the two wheels' reference points, metres.</summary>
    public double MaterialLength(int span) => Data.Material[Span(span)];
    /// <summary>The straight length of a span between its tangent points in the last step, metres: the free length of a winch's rope.</summary>
    public double FreeLength(int span) => Data.Free[Span(span)];
    /// <summary>Length of the span as the rope measures it, including the arcs from the reference points: stretch is this minus <see cref="MaterialLength"/>.</summary>
    public double SurfaceLength(int span) => Data.Geometric[Span(span)];
    /// <summary>Positive when the span is longer than its rope (the solver is pulling it back), negative when slack, metres.</summary>
    public double Stretch(int span) => Data.Stretch[Span(span)];
    /// <summary>The system's total stretch: the path length the geometry needs minus the rope there is, metres. Positive means the rope is being pulled longer than it is (the solver pulls it back); negative, the rope has slack. Unlike the per-span values it does not change when rope slides through a wheel.</summary>
    public double TotalStretch { get { double t = 0; foreach (double s in Data.Stretch) t += s; return t; } }
    /// <summary>Tension in newtons in the last step; zero for a slack span.</summary>
    public double Tension(int span) { var d = Data; int k = Span(span); return d.LastDt > 0 ? d.Impulse[k] / d.LastDt : 0; }
    /// <summary>Largest span tension, newtons.</summary>
    public double MaximumTension { get { var d = Data; double t = 0; if (d.LastDt > 0) foreach (double l in d.Impulse) t = Math.Max(t, l / d.LastDt); return t; } }
    /// <summary>Estimated sag of a slack span in metres, from a parabola of the same excess length: sqrt(3 L excess / 8).</summary>
    public double Sag(int span) { var d = Data; int k = Span(span); double excess = d.Material[k] - d.Geometric[k]; return excess > 0 ? Math.Sqrt(3 * d.Free[k] * excess / 8) : 0; }
    /// <summary>The angle the rope wraps the wheel, radians, extra turns included.</summary>
    public double Wrap(int node) { Wheel(node); return Data.Wrap[node]; }
    /// <summary>Rotor angle relative to its carrier, radians, unwrapped; zero for a wheel that spins with its body.</summary>
    public double WheelAngle(int node) => Wheel(node).RotorAngle;
    /// <summary>Rotor spin about its axle, rad/s, absolute (relative to the world); zero for a wheel that spins with its body.</summary>
    public float WheelSpeed(int node) => Wheel(node).Mode == WheelSpin.Rotor ? Node(node).Spin : 0;
    /// <summary>Torque the rope exerts on the wheel about its axle in the last step, N·m, positive in the direction the rope travels around it.</summary>
    public double WheelTorque(int node)
    {
        var d = Data; var n = Wheel(node); if (d.LastDt <= 0) return 0;
        int spans = d.Spans, count = d.Nodes.Length; double torque = 0;
        bool hasOut = d.Closed || node < count - 1, hasIn = d.Closed || node > 0;
        if (hasOut) torque += d.Impulse[node % spans];
        if (hasIn) torque -= d.Impulse[(node + count - 1) % count % spans];
        return n.Side * (double)n.Radius * torque / d.LastDt;
    }
    /// <summary>Torque of a rotor's motor or brake on its carrier in the last step, N·m (positive along the axle).</summary>
    public double DriveTorque(int node) { var d = Data; var n = Wheel(node); return d.LastDt > 0 ? (n.MotorImpulse + n.BrakeImpulse) / (double)d.LastDt : 0; }
    /// <summary>Sets a rotor's motor: target speed relative to the carrier in rad/s (within ±1e6) and the largest torque in N·m (0 switches the motor off).</summary>
    public void SetMotor(int node, float speed, float maxTorque)
    {
        var s = Store; var n = Wheel(node);
        if (n.Mode != WheelSpin.Rotor) throw new InvalidOperationException("Only rotor wheels have motors.");
        if (!(Math.Abs(speed) <= 1e6f) || !(maxTorque >= 0 && maxTorque <= 1e12f)) throw new ArgumentOutOfRangeException(nameof(speed), "Speed within ±1e6 rad/s and torque 0 to 1e12 N·m.");
        using var m = s.Host.CableMutation("PulleySystem.SetMotor"); n.MotorSpeed = speed; n.MotorTorque = maxTorque; if (maxTorque == 0) n.MotorImpulse = 0;
    }
    /// <summary>Sets a rotor's brake: the holding torque in N·m, 0 to 1e12 (0 releases it).</summary>
    public void SetBrake(int node, float torque)
    {
        var s = Store; var n = Wheel(node);
        if (n.Mode != WheelSpin.Rotor) throw new InvalidOperationException("Only rotor wheels have brakes.");
        if (!(torque >= 0 && torque <= 1e12f)) throw new ArgumentOutOfRangeException(nameof(torque), torque, "Brake torque 0 to 1e12 N·m.");
        using var m = s.Host.CableMutation("PulleySystem.SetBrake"); n.BrakeTorque = torque; if (torque == 0) n.BrakeImpulse = 0;
    }
    /// <summary>Pays rope out (positive) or takes it in (negative) at the last span, metres: a winch without a drum. The span's material length must stay positive.</summary>
    public void AdjustLength(double delta)
    {
        var s = Store; var d = Data; int last = d.Spans - 1;
        if (!double.IsFinite(delta) || d.Material[last] + delta <= 0) throw new ArgumentOutOfRangeException(nameof(delta), delta, "The span's rope length must stay positive.");
        using var m = s.Host.CableMutation("PulleySystem.AdjustLength"); d.Material[last] += delta;
    }
    /// <summary>Destroys the system; later access throws InvalidOperationException.</summary>
    public void Destroy() => Store.Remove(Index);
}
