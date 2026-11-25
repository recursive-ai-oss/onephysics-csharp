using System;

namespace RecursiveAI.OnePhysics.Bake;

/// <summary>
/// Targets for <c>world.ApplyOfflineQuality</c>: a world that is baked, not played, spends its time on accuracy. Each setting is a floor: it is raised to the
/// target and never lowered, so a world already tuned finer keeps its values. Every one is an ordinary world option, so applying them is recorded by a replay and
/// saved in a snapshot like any other change. The effect of each setting is measured in docs/spec/bake.md.
/// </summary>
public sealed class OfflineQualityOptions
{
    /// <summary>Rigid-body substeps per step (the world default is 4). More substeps shorten the integration interval: less penetration and joint drift, closer to the continuous solution, at proportional cost. Default 8.</summary>
    public int Substeps { get; set; } = 8;

    /// <summary>Velocity solve passes per substep (default world value 1). Default 2.</summary>
    public int VelocityIterations { get; set; } = 2;

    /// <summary>Relax passes per substep (default world value 1). Default 2.</summary>
    public int RelaxIterations { get; set; } = 2;

    /// <summary>Substeps of the contact-only schedule that islands without joints run (3D default 2; in 2D it is off and stays off unless the world turned it on). Default 4.</summary>
    public int ContactSubsteps { get; set; } = 4;

    /// <summary>Soft body, cloth and strip substeps per step (default world value 4). Default 8.</summary>
    public int SoftSubsteps { get; set; } = 8;

    /// <summary>Soft body solver iterations per substep (default world value 3). Default 6.</summary>
    public int SoftIterations { get; set; } = 6;

    /// <summary>Particle fluid substeps per step (default world value 2). Default 4.</summary>
    public int FluidSubsteps { get; set; } = 4;

    /// <summary>Particle fluid density iterations per substep (default world value 4). Default 8.</summary>
    public int FluidIterations { get; set; } = 8;

    /// <summary>Turn sleeping off for the whole world, so nothing freezes mid-motion and a body is never woken late. Default true.</summary>
    public bool DisableSleep { get; set; } = true;

    /// <summary>Turn continuous collision on (it is on by default; this only matters for a world that turned it off). Default true.</summary>
    public bool Continuous { get; set; } = true;

    /// <summary>Turn surface collision on for every soft body that exists when the preset is applied (it is off by default in 3D, which makes bodies collide as particle spheres only). Bodies added later keep their own setting. Default true.</summary>
    public bool SoftSurfaceCollision { get; set; } = true;

    /// <summary>Checks every member; <c>ApplyOfflineQuality</c> calls it.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A count is below 1 or beyond 64.</exception>
    public void Validate()
    {
        Check(Substeps, nameof(Substeps)); Check(VelocityIterations, nameof(VelocityIterations)); Check(RelaxIterations, nameof(RelaxIterations));
        Check(ContactSubsteps, nameof(ContactSubsteps)); Check(SoftSubsteps, nameof(SoftSubsteps)); Check(SoftIterations, nameof(SoftIterations));
        Check(FluidSubsteps, nameof(FluidSubsteps)); Check(FluidIterations, nameof(FluidIterations));
    }

    private static void Check(int v, string name)
    {
        if (v < 1 || v > 64) { throw new ArgumentOutOfRangeException(name, v, name + " must be 1 to 64."); }
    }
}
