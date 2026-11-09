using System;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Rods;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>Generation-checked rod handle. Destroyed or default handles throw InvalidOperationException on access. The world owns all storage.</summary>
[Experimental("OPHYS0001")]
public readonly struct Rod
{
    private readonly RodStore? _store;
    private readonly int _generation;
    internal Rod(RodStore store, int index, int generation) { _store = store; Index = index; _generation = generation; }
    /// <summary>Stable rod slot, reused lowest-first after destruction.</summary>
    public int Index { get; }
    /// <summary>Whether the rod still exists in the original world.</summary>
    public bool IsValid => _store != null && _store.Alive(Index,_generation);
    private RodStore Store => IsValid ? _store! : throw new InvalidOperationException("Rod has been destroyed or is uninitialized.");
    private RodParticles Data => Store.Data[Index]!;
    /// <summary>Number of centerline particles (segments plus one).</summary>
    public int ParticleCount => Data.PositionData.Length;
    /// <summary>Read a particle position in metres. Throws ArgumentOutOfRangeException for an invalid index.</summary>
    public Vector2D GetPosition(int particle)
    {
        var d = Data; Check(particle,d.PositionData.Length); var p = d.PositionData[particle]; return new(p.X,p.Y);
    }
    /// <summary>Read velocity in m/s. Throws ArgumentOutOfRangeException for an invalid index.</summary>
    public Vector2F GetVelocity(int particle)
    {
        var d = Data; Check(particle,d.PositionData.Length); var p = d.VelocityData[particle]; return new(p.X,p.Y);
    }
    /// <summary>Pin or release a particle at its current position. Releasing restores its material-derived mass. Throws ArgumentOutOfRangeException for an invalid index.</summary>
    public void SetPinned(int particle, bool pinned)
    {
        var store = Store; var data = Data; Check(particle,data.PositionData.Length);
        using var mutation = store.Host.RodMutation("Rod.SetPinned"); data.SetPinned(particle,pinned);
    }
    /// <summary>Drive a pinned particle towards a target over the next step's substeps; later steps hold it there. Coordinates finite within ±2^21 metres, rounded to the world 2^-32 grid. Throws InvalidOperationException if not pinned.</summary>
    public void SetTarget(int particle, Vector2D target)
    {
        var store = Store; var data = Data; Check(particle,data.PositionData.Length);
        if (!data.Pinned[particle]) throw new InvalidOperationException("Only a pinned particle has a kinematic target.");
        if (!(double.IsFinite(target.X) && Math.Abs(target.X) <= 2097152 && double.IsFinite(target.Y) && Math.Abs(target.Y) <= 2097152))
            throw new ArgumentOutOfRangeException(nameof(target),target,"Target must be finite within ±2097152 metres.");
        using var mutation = store.Host.RodMutation("Rod.SetTarget");
        data.PinTarget[particle] = new Vector3D(WorldCore.Grid(target.X),WorldCore.Grid(target.Y),0);
    }
    /// <summary>Set a particle velocity, m/s, finite components within ±10000. A pinned particle's velocity is determined by its target instead.</summary>
    public void SetVelocity(int particle, Vector2F velocity)
    {
        var store = Store; var data = Data; Check(particle,data.PositionData.Length);
        RodMaterial.Check(velocity.X,-10000,10000,nameof(velocity)); RodMaterial.Check(velocity.Y,-10000,10000,nameof(velocity)); 
        using var mutation = store.Host.RodMutation("Rod.SetVelocity");
        data.VelocityData[particle] = new Vector3F(velocity.X,velocity.Y,0);
    }
    /// <summary>Maximum absolute relative length error over all segments, dimensionless.</summary>
    public double MaximumStretch => Data.MaximumStretch;
    /// <summary>Raw particle and segment arrays; experimental, single-writer access bypasses validation and recording.</summary>
    public RodParticles Particles => Data;
    /// <summary>Destroy the rod. Later accesses throw InvalidOperationException.</summary>
    public void Destroy() => Store.Remove(Index);
    private static void Check(int index, int count)
    {
        if ((uint)index >= (uint)count) throw new ArgumentOutOfRangeException(nameof(index),index,"Index outside rod storage.");
    }
}
