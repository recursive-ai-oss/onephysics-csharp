using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public readonly partial struct Body
{
    /// <summary>World-axis locks and speed limits. Default inherits world limits. Setting wakes and projects locked velocities. Invalid or stale handles throw InvalidHandleException.</summary>
    public BodyControlSettings Controls
    {
        get { var (w, _) = Resolve(); return w.Core.GetBodyControls(_handle); }
        set { using var mutation = _world?.RecordMutation("Body.Controls"); var (w, b) = Resolve(); w.Core.SetBodyControls(_handle, value); w.Engine.RefreshWorldInertia(b); }
    }
    /// <summary>World-axis locks, using RigidbodyConstraints vocabulary. Only X/Y translation and Z angular velocity apply in 2D. Setter preserves speed caps.</summary>
    public RigidbodyConstraints Constraints
    {
        get => Controls.Constraints;
        set { var c = Controls; Controls = new(value, c.MaxLinearSpeed, c.MaxAngularSpeed, c.MaxDepenetrationSpeed); }
    }
    /// <summary>Maximum linear speed in m/s; finite and nonnegative. Default float.MaxValue inherits the world cap.</summary>
    public float MaxLinearSpeed { get => Controls.MaxLinearSpeed; set { var c = Controls; Controls = new(c.Constraints, value, c.MaxAngularSpeed, c.MaxDepenetrationSpeed); } }
    /// <summary>Maximum angular speed in rad/s; finite and nonnegative. Default float.MaxValue inherits world caps.</summary>
    public float MaxAngularSpeed { get => Controls.MaxAngularSpeed; set { var c = Controls; Controls = new(c.Constraints, c.MaxLinearSpeed, value, c.MaxDepenetrationSpeed); } }
    /// <summary>Maximum contact push-out speed in m/s; finite and nonnegative. Default float.MaxValue inherits world PushOutVelocity.</summary>
    public float MaxDepenetrationSpeed { get => Controls.MaxDepenetrationSpeed; set { var c = Controls; Controls = new(c.Constraints, c.MaxLinearSpeed, c.MaxAngularSpeed, value); } }
    /// <summary>Overrides mass in kg, in 1e-18..1e30. Does not rescale inertia. Dynamic bodies only; invalid values throw ArgumentOutOfRangeException.</summary>
    public void SetMass(float mass) { using var mutation = _world?.RecordMutation("Body.SetMass"); var (w, b) = Resolve(); RequireDynamic(w, b); w.Engine.SetMassOverride(b, mass); }
    /// <summary>Resets mass alone to the value computed from current shapes. Dynamic bodies only.</summary>
    public void ResetMass() { using var mutation = _world?.RecordMutation("Body.ResetMass"); var (w, b) = Resolve(); RequireDynamic(w, b); w.Engine.ResetMassOverride(b, 1); }
    /// <summary>Local center of mass in metres. Setting preserves the body origin, recomputes shape inertia about this center unless overridden, and wakes the body. Components must be finite within ±2097152. Dynamic bodies only.</summary>
    public Vector3F CenterOfMass
    {
        get { var (w, b) = Resolve(); return w.Engine.LocalCenterOf(b); }
        set { using var mutation = _world?.RecordMutation("Body.CenterOfMass"); var (w, b) = Resolve(); RequireDynamic(w, b); w.Engine.SetCenterOverride(b, value); }
    }
    /// <summary>Resets local center of mass alone to the shape-derived center, preserving the body origin. Dynamic bodies only.</summary>
    public void ResetCenterOfMass() { using var mutation = _world?.RecordMutation("Body.ResetCenterOfMass"); var (w, b) = Resolve(); RequireDynamic(w, b); w.Engine.ResetMassOverride(b, 2); }
    /// <summary>Overrides inertia about the chosen center of mass, as positive principal diagonal (kg m², each 1e-18..1e30) plus its unit orientation in the body frame. Dynamic bodies only; invalid inputs throw ArgumentOutOfRangeException.</summary>
    public void SetInertia(Vector3F principalInertia, QuaternionF rotation) { using var mutation = _world?.RecordMutation("Body.SetInertia"); var (w, b) = Resolve(); RequireDynamic(w, b); w.Engine.SetInertiaOverride(b, principalInertia, rotation); }
    /// <summary>Resets inertia alone to that computed from shapes about the current center. Dynamic bodies only.</summary>
    public void ResetInertia() { using var mutation = _world?.RecordMutation("Body.ResetInertia"); var (w, b) = Resolve(); RequireDynamic(w, b); w.Engine.ResetMassOverride(b, 4); }
    /// <summary>Queues a kinematic target for the next step. Positions are rounded to the 2^-32 grid; orientation is normalized. Solver-visible velocities carry and push contacts. Last target wins. Static/dynamic bodies throw ArgumentException; invalid/out-of-bound inputs throw ArgumentOutOfRangeException. Dynamic drives are not supported.</summary>
    public void MoveKinematic(Vector3D position, QuaternionF rotation) { using var mutation = _world?.RecordMutation("Body.MoveKinematic"); var (w, b) = Resolve(); w.Engine.SetKinematicTarget(b, position, rotation); }
    /// <summary>Queues translation preserving the latest queued orientation, or current orientation if no target exists. Kinematic bodies only.</summary>
    public void MovePosition(Vector3D position) { using var mutation = _world?.RecordMutation("Body.MovePosition"); var (w, b) = Resolve(); w.Engine.SetKinematicPosition(b, position); }
    /// <summary>Queues orientation preserving the latest queued position, or current position if no target exists. Kinematic bodies only.</summary>
    public void MoveRotation(QuaternionF rotation) { using var mutation = _world?.RecordMutation("Body.MoveRotation"); var (w, b) = Resolve(); w.Engine.SetKinematicRotation(b, rotation); }
}
