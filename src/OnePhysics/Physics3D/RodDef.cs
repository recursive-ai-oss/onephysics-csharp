using System;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Rods;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>A discretized circular Cosserat rod. Positions and material frames are copied on creation. Metres, kilograms and seconds.</summary>
[Experimental("OPHYS0001")]
public sealed class RodDef
{
    /// <summary>Rest centerline, 2 to 4097 points; adjacent distances 0.0001 to 100 metres. Required. Coordinates finite within ±2^21.</summary>
    public Vector3D[] Points { get; set; } = [];
    /// <summary>Material; default flexible rope. Cross-section and mass use the same circular model in both dimensions.</summary>
    public RodMaterial Material { get; set; } = RodMaterial.Rope;
    /// <summary>Initial translational velocity, m/s; finite components within ±10000. Default zero.</summary>
    public Vector3F Velocity { get; set; }
    /// <summary>World-pinned particles; default none. Valid indices only; duplicates are harmless.</summary>
    public int[] PinnedParticles { get; set; } = [];
    /// <summary>Clamped material frames; default none. Indices refer to segments, not particles.</summary>
    public int[] ClampedSegments { get; set; } = [];
    /// <summary>Clamps the start tangent at the first particle using a half-cell bending constraint, default false. Requires the first particle pinned. Unlike ClampedSegments, this preserves the boundary location under refinement.</summary>
    public bool ClampStart { get; set; }
    /// <summary>Substeps per world step, 1 to 64; default 8. Independent of the rigid schedule.</summary>
    public int Substeps { get; set; } = 8;
    /// <summary>Nonlinear direct-solve iterations per substep, 1 to 32; default 4.</summary>
    public int Iterations { get; set; } = 4;
    /// <summary>Velocity damping, inverse seconds; finite 0 to 1000, default 0.1. Implicit multiplier 1/(1+damping*dt).</summary>
    public float Damping { get; set; } = 0.1f;
    /// <summary>Gravity multiplier; finite within ±1000. Default 1.</summary>
    public float GravityScale { get; set; } = 1;
    /// <summary>Validates without modifying the definition; throws ArgumentNullException or ArgumentOutOfRangeException naming the invalid field.</summary>
    public void Validate()
    {
        Material.Validate();
        ArgumentNullException.ThrowIfNull(Points); ArgumentNullException.ThrowIfNull(PinnedParticles); ArgumentNullException.ThrowIfNull(ClampedSegments);
        if (Points.Length < 2 || Points.Length > 4097) throw new ArgumentOutOfRangeException(nameof(Points), "Expected 2 to 4097 points.");
        for (int i = 0; i < Points.Length; i++)
        {
            var p = Points[i];
            if (!(double.IsFinite(p.X) && Math.Abs(p.X) <= 2097152 && double.IsFinite(p.Y) && Math.Abs(p.Y) <= 2097152 && double.IsFinite(p.Z) && Math.Abs(p.Z) <= 2097152))
                throw new ArgumentOutOfRangeException(nameof(Points), p, $"Points[{i}] must be finite within ±2097152 metres.");
            if (i > 0)
            {
                var d = Points[i] - Points[i-1]; double l = Math.Sqrt(d.X*d.X+d.Y*d.Y+d.Z*d.Z);
                if (!(l >= 0.0001 && l <= 100)) throw new ArgumentOutOfRangeException(nameof(Points), l, $"Segment {i-1} length must be 0.0001 to 100 metres.");
            }
        }
        foreach (int i in PinnedParticles) if ((uint)i >= (uint)Points.Length) throw new ArgumentOutOfRangeException(nameof(PinnedParticles), i, "Particle index outside Points.");
        if (ClampStart && Array.IndexOf(PinnedParticles,0) < 0) throw new ArgumentException("ClampStart requires particle zero pinned.",nameof(ClampStart));
        foreach (int i in ClampedSegments) if ((uint)i >= (uint)(Points.Length-1)) throw new ArgumentOutOfRangeException(nameof(ClampedSegments), i, "Segment index outside Points.");
        if (Substeps < 1 || Substeps > 64) throw new ArgumentOutOfRangeException(nameof(Substeps));
        if (Iterations < 1 || Iterations > 32) throw new ArgumentOutOfRangeException(nameof(Iterations));
        RodMaterial.Check(Damping, 0, 1000, nameof(Damping)); RodMaterial.Check(GravityScale, -1000, 1000, nameof(GravityScale));
        RodMaterial.Check(Velocity.X, -10000, 10000, nameof(Velocity)); RodMaterial.Check(Velocity.Y, -10000, 10000, nameof(Velocity)); RodMaterial.Check(Velocity.Z, -10000, 10000, nameof(Velocity)); 
    }
}
