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

    /// <summary>Pairs that actually ran a fresh narrowphase query, excluding early rejection and reuse.</summary>
    public int NarrowphasePairs { get; internal set; }
    /// <summary>Active manifolds not present before contact update.</summary>
    public int NewManifolds { get; internal set; }
    /// <summary>Active old manifolds refreshed rather than recycled.</summary>
    public int PersistedManifolds { get; internal set; }
    /// <summary>Active points whose feature id was not present before contact update.</summary>
    public int NewContactPoints { get; internal set; }
    /// <summary>Active feature-matched points refreshed rather than recycled.</summary>
    public int PersistedContactPoints { get; internal set; }
    /// <summary>Points in successfully recycled manifolds.</summary>
    public int RecycledContactPoints { get; internal set; }
    /// <summary>Warm-start passes executed by the world schedule.</summary>
    public int WorldWarmStartPasses { get; internal set; }
    /// <summary>Restitution passes executed by the world schedule.</summary>
    public int WorldRestitutionPasses { get; internal set; }
    /// <summary>Warm-start passes executed by the contact-only schedule.</summary>
    public int ContactWarmStartPasses { get; internal set; }
    /// <summary>Restitution passes executed by the contact-only schedule.</summary>
    public int ContactRestitutionPasses { get; internal set; }
    /// <summary>Bodies selected for a continuous sweep.</summary>
    public int CcdBodies { get; internal set; }
    /// <summary>Bodies stopped by a continuous sweep, counted once per body.</summary>
    public int CcdHits { get; internal set; }
    /// <summary>Bullet bodies selected for continuous collision.</summary>
    public int BulletBodies { get; internal set; }
    /// <summary>Particles in awake soft bodies selected for the soft solve.</summary>
    public int SoftParticles { get; internal set; }
    /// <summary>Blocks in awake soft bodies selected for the soft solve.</summary>
    public int SoftBlocks { get; internal set; }
    /// <summary>Live character controllers after the step.</summary>
    public int Characters { get; internal set; }
    /// <summary>Live vehicle controllers after the step.</summary>
    public int Vehicles { get; internal set; }
    /// <summary>Bodies replaced by fracture or slicing in the last step (impact, manual and slices since the step began).</summary>
    public int Fractures { get; internal set; }
    /// <summary>Bonds broken by stress in the last step.</summary>
    public int BondBreaks { get; internal set; }
    /// <summary>Fragments the debris policy removed in the last step (budget and lifetime).</summary>
    public int FragmentsCulled { get; internal set; }
    /// <summary>Fragments the debris policy tracks after the step.</summary>
    public int Fragments { get; internal set; }
    /// <summary>Field applications dropped in the last step because their result was not finite or was beyond ±1e12 (for example a gravity callback returning NaN). Dropped applications change nothing.</summary>
    public int FieldApplicationsSkipped { get; internal set; }
    internal Span<int> Colors => _colors;
    internal void Reset()
    {
        Array.Clear(_colors);
        NarrowphasePairs = NewManifolds = PersistedManifolds = NewContactPoints = PersistedContactPoints = RecycledContactPoints = WorldWarmStartPasses = WorldRestitutionPasses = ContactWarmStartPasses = ContactRestitutionPasses = CcdBodies = CcdHits = BulletBodies = SoftParticles = SoftBlocks = Characters = Vehicles = 0;
        AwakeBodies = AwakeIslands = AsleepIslands = BroadphasePairs = UpdatedPairs = RecycledManifolds = 0;
        Manifolds = ContactPoints = Joints = SerialOverflow = 0;
        WorldSubsteps = WorldSolvePasses = WorldRelaxPasses = ContactSubsteps = ContactSolvePasses = ContactRelaxPasses = 0;
        SleepTransitions = WakeTransitions = 0;
    }
}
