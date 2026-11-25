using System;

namespace RecursiveAI.OnePhysics.Presentation;

/// <summary>
/// The budgets of a presentation (follower) world: how many cosmetic bodies and particles it may hold and how much work its cloth, soft bodies and rods may spend.
/// A host sets one on <c>PresentationWorld2D/3D.Quality</c> (assigning a copy) and may change it between steps. Everything is applied at the start of the next step.
/// <para>Culling order when a budget is lowered: rigid bodies lightest first, then oldest first, then lowest slot; fluid particles oldest first. Raising a budget never changes what already exists.</para>
/// </summary>
public sealed class PresentationQuality
{
    /// <summary>The value meaning "no limit" for <see cref="MaxBodies"/> and <see cref="MaxParticles"/>.</summary>
    public const int Unlimited = int.MaxValue;

    /// <summary>The most bodies of type Dynamic (debris, secondary ragdoll parts and anything else the host added to the follower that has mass, except pinned ones) the follower keeps. When there are more, the lightest are destroyed first, then the oldest, until the budget holds. 0 to <see cref="Unlimited"/>. Default 1024.</summary>
    public int MaxBodies { get; set; } = 1024;

    /// <summary>The most fluid particles the follower holds (also the fluid store's own budget, so emitters stop there). When there are more, the oldest particles are removed first. 0 to <see cref="Unlimited"/>. Default 65536.</summary>
    public int MaxParticles { get; set; } = 65536;

    /// <summary>Soft-body and cloth substeps per step, 1 to 64, applied to the follower's world options; 0 leaves the world's own setting. Default 4.</summary>
    public int SoftSubsteps { get; set; } = 4;

    /// <summary>Soft-body and cloth solver iterations per substep, 1 to 32; 0 leaves the world's own setting. Default 3.</summary>
    public int SoftIterations { get; set; } = 3;

    /// <summary>The most substeps any rod, rope or hair strand takes, 1 to 64; a rod defined with fewer keeps its own. 0 = every rod keeps its own count. Default 0.</summary>
    public int RodSubsteps { get; set; }

    /// <summary>The most solver iterations any rod, rope or hair strand takes, 1 to 64; a rod defined with fewer keeps its own. 0 = every rod keeps its own count. Default 0.</summary>
    public int RodIterations { get; set; }

    /// <summary>The full budgets: 1024 bodies, 65536 particles, the world's default soft schedule (4 substeps, 3 iterations) and rods as defined.</summary>
    public static PresentationQuality High => new();

    /// <summary>A middle level: 256 bodies, 8192 particles, soft 3 substeps of 2 iterations, rods capped at 4 substeps of 2 iterations.</summary>
    public static PresentationQuality Medium => new() { MaxBodies = 256, MaxParticles = 8192, SoftSubsteps = 3, SoftIterations = 2, RodSubsteps = 4, RodIterations = 2 };

    /// <summary>A cheap level: 64 bodies, 1024 particles, soft 2 substeps of 1 iteration, rods capped at 2 substeps of 1 iteration.</summary>
    public static PresentationQuality Low => new() { MaxBodies = 64, MaxParticles = 1024, SoftSubsteps = 2, SoftIterations = 1, RodSubsteps = 2, RodIterations = 1 };

    /// <summary>No limits on bodies or particles; the soft and rod schedules as in <see cref="High"/>.</summary>
    public static PresentationQuality Unbounded => new() { MaxBodies = Unlimited, MaxParticles = Unlimited };

    /// <summary>Returns an independent copy.</summary>
    public PresentationQuality Copy() => (PresentationQuality)MemberwiseClone();

    /// <summary>Checks every value.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is outside its range; the message names it.</exception>
    public void Validate()
    {
        if (MaxBodies < 0) throw new ArgumentOutOfRangeException(nameof(MaxBodies), MaxBodies, "MaxBodies must be at least 0.");
        if (MaxParticles < 0 || MaxParticles > 16777216 && MaxParticles != Unlimited) throw new ArgumentOutOfRangeException(nameof(MaxParticles), MaxParticles, "MaxParticles must be 0 to 16777216, or Unlimited.");
        if (SoftSubsteps < 0 || SoftSubsteps > 64) throw new ArgumentOutOfRangeException(nameof(SoftSubsteps), SoftSubsteps, "SoftSubsteps must be 0 to 64.");
        if (SoftIterations < 0 || SoftIterations > 32) throw new ArgumentOutOfRangeException(nameof(SoftIterations), SoftIterations, "SoftIterations must be 0 to 32.");
        if (RodSubsteps < 0 || RodSubsteps > 64) throw new ArgumentOutOfRangeException(nameof(RodSubsteps), RodSubsteps, "RodSubsteps must be 0 to 64.");
        if (RodIterations < 0 || RodIterations > 64) throw new ArgumentOutOfRangeException(nameof(RodIterations), RodIterations, "RodIterations must be 0 to 64.");
    }

    // The fluid store's own budget never exceeds its documented limit.
    internal int FluidBudget => MaxParticles == Unlimited ? 16777216 : MaxParticles;
}

/// <summary>
/// The default response to a fracture in the authoritative world: a burst of small cosmetic shards at the impact point. The authoritative fragments themselves are followed as kinematic bodies;
/// the shards are extra, light and short-lived in effect (they fall under the follower's budget like any other cosmetic body).
/// Read with <c>PresentationWorld2D/3D.Debris</c> and set by assigning a copy.
/// <para>Lengths, speeds and densities are in the follower world's units. Each one left at <c>null</c> takes its metre default scaled by the world's <c>LengthUnit</c> when a burst is made (for a millimetre world a shard is 50 units long), like the world's own defaults.</para>
/// </summary>
public sealed class PresentationDebris
{
    /// <summary>Shards per fracture event, 0 to 256 (0 turns the burst off). Default 12.</summary>
    public int Count { get; set; } = 12;

    /// <summary>Full edge length of a shard in world units, above 0. Default (null): 0.05 m.</summary>
    public float? Size { get; set; }

    /// <summary>Radius around the impact point in which shards start, world units, 0 or more. Default (null): 0.2 m.</summary>
    public float? Radius { get; set; }

    /// <summary>Outward speed in world units per second, 0 or more: a shard flies out at between half and one and a half times this, on top of the mean velocity of the authoritative fragments. Default (null): 3 m/s.</summary>
    public float? Speed { get; set; }

    /// <summary>Density of a shard in kg per cubic world unit (per square unit in 2D), above 0. Default (null): 600 kg/m³ (kg/m² in 2D), whatever the unit.</summary>
    public float? Density { get; set; }

    /// <summary>Seed of the deterministic generator that places and launches shards; the stream is also keyed by the follower's step count and the event's index, so equal events give equal shards on every platform. Default 1.</summary>
    public ulong Seed { get; set; } = 1;

    /// <summary>True (default) also follows the authoritative fragments of a followed body as kinematic bodies, so the cosmetic world sees them.</summary>
    public bool FollowFragments { get; set; } = true;

    /// <summary>Returns an independent copy.</summary>
    public PresentationDebris Copy() => (PresentationDebris)MemberwiseClone();

    /// <summary>Checks every value.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is outside its range; the message names it.</exception>
    public void Validate()
    {
        if (Count < 0 || Count > 256) throw new ArgumentOutOfRangeException(nameof(Count), Count, "Count must be 0 to 256.");
        if (Size is { } z && (!(z > 0f) || !float.IsFinite(z) || z > 1e9f)) throw new ArgumentOutOfRangeException(nameof(Size), Size, "Size must be above 0 and finite.");
        if (Radius is { } r && (!(r >= 0f) || !float.IsFinite(r) || r > 1e9f)) throw new ArgumentOutOfRangeException(nameof(Radius), Radius, "Radius must be 0 or more and finite.");
        if (Speed is { } v && (!(v >= 0f) || !float.IsFinite(v) || v > 1e9f)) throw new ArgumentOutOfRangeException(nameof(Speed), Speed, "Speed must be 0 or more and finite.");
        if (Density is { } d && (!(d > 0f) || !float.IsFinite(d) || d > 1e30f)) throw new ArgumentOutOfRangeException(nameof(Density), Density, "Density must be above 0 and finite.");
    }
}
