using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Debug;

/// <summary>One particle of a particle fluid, for drawing.</summary>
public readonly struct DebugFluidParticle
{
    /// <summary>Creates an entry.</summary>
    public DebugFluidParticle(int id, int fluid, Vector3D position, Vector3F velocity)
    { Id = id; Fluid = fluid; Position = position; Velocity = velocity; }

    /// <summary>Id of the particle.</summary>
    public int Id { get; }

    /// <summary>Slot of its fluid.</summary>
    public int Fluid { get; }

    /// <summary>World position (a planar world: z = 0).</summary>
    public Vector3D Position { get; }

    /// <summary>Velocity, m/s.</summary>
    public Vector3F Velocity { get; }
}

/// <summary>An emitter's nozzle or a drain's volume, for drawing.</summary>
public readonly struct DebugFluidVolume
{
    /// <summary>Creates an entry.</summary>
    public DebugFluidVolume(bool isEmitter, int index, FluidVolumeKind kind, Vector3D centre, Vector3F size, Vector3F direction, bool enabled)
    { IsEmitter = isEmitter; Index = index; Kind = kind; Centre = centre; Size = size; Direction = direction; Enabled = enabled; }

    /// <summary>True for an emitter, false for a drain.</summary>
    public bool IsEmitter { get; }

    /// <summary>Slot of the emitter or drain.</summary>
    public int Index { get; }

    /// <summary>Box or sphere.</summary>
    public FluidVolumeKind Kind { get; }

    /// <summary>Centre.</summary>
    public Vector3D Centre { get; }

    /// <summary>Half extents (box) or radius in X (sphere).</summary>
    public Vector3F Size { get; }

    /// <summary>Flow direction of an emitter (zero for a drain).</summary>
    public Vector3F Direction { get; }

    /// <summary>Whether it works now.</summary>
    public bool Enabled { get; }
}

public sealed partial class DebugGeometry
{
    private DebugFluidParticle[] _fluidParticles = new DebugFluidParticle[16];
    private int _nFluidParticles;
    private DebugFluidVolume[] _fluidVolumes = new DebugFluidVolume[4];
    private int _nFluidVolumes;

    /// <summary>Particles of every particle fluid, in storage order, valid until the next collection or Clear.</summary>
    public ReadOnlySpan<DebugFluidParticle> FluidParticles => _fluidParticles.AsSpan(0, _nFluidParticles);

    /// <summary>Emitters then drains, by slot.</summary>
    public ReadOnlySpan<DebugFluidVolume> FluidVolumes => _fluidVolumes.AsSpan(0, _nFluidVolumes);

    /// <summary>Appends a particle; owned storage grows when necessary.</summary>
    public void Add(in DebugFluidParticle p) => Push(ref _fluidParticles, ref _nFluidParticles, in p);

    /// <summary>Appends a volume; owned storage grows when necessary.</summary>
    public void Add(in DebugFluidVolume v) => Push(ref _fluidVolumes, ref _nFluidVolumes, in v);

    private void ClearFluids() { _nFluidParticles = 0; _nFluidVolumes = 0; }
}
