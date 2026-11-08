using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Live read-only joint diagnostics; no allocation. A held view remains readable after breaking, until the next step or slot reuse. Read between steps.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct JointInspection
{
    private readonly JointInspectionView _view;
    internal JointInspection(JointInspectionView view) { _view = view; }
    /// <summary>Whole-step mean force in N applied to body A, in world axes. Includes every warm/solve/relax pass.</summary>
    public Vector3D ForceA => _view.ForceA;
    /// <summary>Whole-step mean force in N applied to body B, in world axes. Includes every warm/solve/relax pass.</summary>
    public Vector3D ForceB => _view.ForceB;
    /// <summary>Whole-step mean COM torque in N m applied to body A, in world axes. Includes every warm/solve/relax pass.</summary>
    public Vector3D TorqueA => _view.TorqueA;
    /// <summary>Whole-step mean COM torque in N m applied to body B, in world axes. Includes every warm/solve/relax pass.</summary>
    public Vector3D TorqueB => _view.TorqueB;
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

public sealed partial class PhysicsWorld3D
{
    /// <summary>Inspects a live joint of this world. Throws for a stale, default or foreign handle.</summary>
    public JointInspection InspectJoint(Joint joint) => new(_core.JointReadback.Inspect(joint.RawHandle));
}
