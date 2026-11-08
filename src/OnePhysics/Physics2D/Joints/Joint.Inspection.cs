using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>Live read-only joint diagnostics; no allocation. A held view remains readable after breaking, until the next step or slot reuse. Read between steps.</summary>

public readonly struct JointInspection
{
    private readonly JointInspectionView _view;
    internal JointInspection(JointInspectionView view) { _view = view; }
    /// <summary>Whole-step mean force in N applied to body A, in world axes. Includes every warm/solve/relax pass.</summary>
    public Vector2D ForceA => new(_view.ForceA.X, _view.ForceA.Y);
    /// <summary>Whole-step mean force in N applied to body B, in world axes. Includes every warm/solve/relax pass.</summary>
    public Vector2D ForceB => new(_view.ForceB.X, _view.ForceB.Y);
    /// <summary>Whole-step mean COM torque in N m applied to body A, in world axes. Includes every warm/solve/relax pass.</summary>
    public double TorqueA => _view.TorqueA.Z;
    /// <summary>Whole-step mean COM torque in N m applied to body B, in world axes. Includes every warm/solve/relax pass.</summary>
    public double TorqueB => _view.TorqueB.Z;
    /// <summary>Recorded IsBroken from the joint inspection; read between steps.</summary>
    public bool IsBroken => _view.IsBroken;
    /// <summary>Recorded IsLimitActive from the joint inspection; read between steps.</summary>
    public bool IsLimitActive => _view.IsLimitActive;
    /// <summary>Recorded IsMotorActive from the joint inspection; read between steps.</summary>
    public bool IsMotorActive => _view.IsMotorActive;
    /// <summary>Recorded Color from the joint inspection; read between steps.</summary>
    public byte Color => _view.Color;
    /// <summary>Recorded Island from the joint inspection; read between steps.</summary>
    public int Island => _view.Island;

    /// <summary>Last evaluated solver errors per impulse slot (32 entries, unused entries have Flags=None).</summary>
    public ReadOnlySpan<JointRowInspection> Rows => _view.Rows;
}

public readonly partial struct Joint
{
    /// <summary>Reads whole-step force/torque, row errors and solver state. A matching last-step break record can be read after destruction; other stale or default handles throw.</summary>
    public JointInspection Inspect() { if (_world == null) { throw new InvalidHandleException("The default Joint is not attached to a world."); } return new(_world.Core.JointReadback.Inspect(RawHandle)); }
}

public sealed partial class PhysicsWorld2D
{
    /// <summary>Inspects a live joint of this world. Throws for a stale, default or foreign handle.</summary>
    public JointInspection InspectJoint(Joint joint) => new(_core.JointReadback.Inspect(joint.RawHandle));
}
