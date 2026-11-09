using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Disables collision between two distinct bodies; at least one must be dynamic. No physical rows are solved.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public struct FilterJointDef
{
    /// <summary>First body, belonging to the destination world.</summary>
    public Body BodyA { get; set; }
    /// <summary>Second body, belonging to the destination world.</summary>
    public Body BodyB { get; set; }
    /// <summary>Steps until breaking; zero means unlimited. Negative values throw.</summary>
    public int LifetimeSteps { get; set; }
}

public sealed partial class PhysicsWorld3D
{
    /// <summary>Disables collision between two bodies without affecting their motion. Invalid, foreign or identical bodies throw.</summary>
    public Joint AddFilterJoint(Body a, Body b) => CreateJoint(new FilterJointDef { BodyA = a, BodyB = b });

    /// <summary>Creates a filter joint. Invalid bodies or a negative lifetime throw before changing the world.</summary>
    public Joint CreateJoint(in FilterJointDef def)
    {
        using var mutation = RecordMutation("PhysicsWorld3D.CreateJoint");
        if (def.LifetimeSteps < 0) { throw new System.ArgumentOutOfRangeException(nameof(def), def.LifetimeSteps, "LifetimeSteps must be at least zero."); }
        var joint = Make(def.BodyA, def.BodyB, Base(JointType3D.Filter, 0, default, default, float.PositiveInfinity, float.PositiveInfinity));
        JointLifetime.Set(_core.Constraints, joint.RawHandle.Index, def.LifetimeSteps);
        return joint;
    }
}

public readonly partial struct Joint
{
    /// <summary>Remaining whole steps before breaking, including sleeping steps; zero means unlimited. Setting a negative value throws. Edits are recorded for replay.</summary>
    public int LifetimeStepsRemaining
    {
        get { Rec(out var w, out int c); return JointLifetime.Get(w.Core.Constraints, c); }
        set { using var mutation = _world?.RecordMutation("Joint.LifetimeStepsRemaining"); Rec(out var w, out int c); JointLifetime.Set(w.Core.Constraints, c, value); }
    }
}
