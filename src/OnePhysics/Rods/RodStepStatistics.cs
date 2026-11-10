namespace RecursiveAI.OnePhysics.Rods;

/// <summary>Last rod step's deterministic diagnostics. No clocks; values reset each step and are included in snapshots and hashes.</summary>
public readonly struct RodStepStatistics
{
    internal RodStepStatistics(float speed,double stretch,float penetration,int contacts,int stops)
    {MaximumSpeed=speed;MaximumStretch=stretch;MaximumPenetration=penetration;Contacts=contacts;ConservativeStops=stops;}
    /// <summary>Peak particle speed sampled after every rod substep, m/s.</summary>
    public float MaximumSpeed { get; }
    /// <summary>Peak absolute relative segment length error after every substep.</summary>
    public double MaximumStretch { get; }
    /// <summary>Peak remaining contact penetration after each substep's final solve, metres. Reports capsule geometry, not pre-impact overshoot.</summary>
    public float MaximumPenetration { get; }
    /// <summary>Largest number of distinct contact features in a substep.</summary>
    public int Contacts { get; }
    /// <summary>Conservative advancement iteration limits reached; the solver stops conservatively rather than declaring a miss.</summary>
    public int ConservativeStops { get; }
}
