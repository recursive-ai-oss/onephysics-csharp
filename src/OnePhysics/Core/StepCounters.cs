using System;

namespace RecursiveAI.OnePhysics;

/// <summary>Always-on, allocation-free diagnostics of the last step. The owning world reuses this object. Not snapshotted or hashed.</summary>
public sealed class StepCounters
{
    private readonly int[] _colors = new int[24];
    /// <summary>Awake bodies presented to the solver, including kinematic bodies.</summary>
    public int AwakeBodies { get; internal set; }
    /// <summary>Awake islands after sleeping is updated.</summary>
    public int AwakeIslands { get; internal set; }
    /// <summary>Sleeping islands after sleeping is updated.</summary>
    public int AsleepIslands { get; internal set; }
    /// <summary>Candidate pairs returned by the broadphase in this step, not all persistent pairs.</summary>
    public int BroadphasePairs { get; internal set; }
    /// <summary>Pairs selected for collision update, including manifold reuse.</summary>
    public int UpdatedPairs { get; internal set; }
    /// <summary>Manifolds reused in this step.</summary>
    public int RecycledManifolds { get; internal set; }
    /// <summary>Active contact manifolds presented to the solver (sleeping contacts excluded).</summary>
    public int Manifolds { get; internal set; }
    /// <summary>Points in active contact manifolds.</summary>
    public int ContactPoints { get; internal set; }
    /// <summary>Active joints presented to the solver.</summary>
    public int Joints { get; internal set; }
    /// <summary>Active constraints per colour, indexed 0 through 23, before break processing.</summary>
    public ReadOnlySpan<int> ConstraintsPerColor => _colors;
    /// <summary>Active constraints in the serial overflow before break processing.</summary>
    public int SerialOverflow { get; internal set; }
    /// <summary>Substeps executed by the world schedule (not multiplied by island count).</summary>
    public int WorldSubsteps { get; internal set; }
    /// <summary>Velocity solve passes executed by the world schedule.</summary>
    public int WorldSolvePasses { get; internal set; }
    /// <summary>Relaxation passes executed by the world schedule.</summary>
    public int WorldRelaxPasses { get; internal set; }
    /// <summary>Substeps executed by the contact-only schedule; zero when that program was not run.</summary>
    public int ContactSubsteps { get; internal set; }
    /// <summary>Velocity solve passes executed by the contact-only schedule.</summary>
    public int ContactSolvePasses { get; internal set; }
    /// <summary>Relaxation passes executed by the contact-only schedule.</summary>
    public int ContactRelaxPasses { get; internal set; }
    /// <summary>Bodies transitioned to sleep during the core step, independent of event masks.</summary>
    public int SleepTransitions { get; internal set; }
    /// <summary>Bodies woken during the core step, independent of event masks.</summary>
    public int WakeTransitions { get; internal set; }

    internal Span<int> Colors => _colors;
    internal void Reset()
    {
        Array.Clear(_colors);
        AwakeBodies = AwakeIslands = AsleepIslands = BroadphasePairs = UpdatedPairs = RecycledManifolds = 0;
        Manifolds = ContactPoints = Joints = SerialOverflow = 0;
        WorldSubsteps = WorldSolvePasses = WorldRelaxPasses = ContactSubsteps = ContactSolvePasses = ContactRelaxPasses = 0;
        SleepTransitions = WakeTransitions = 0;
    }
}
