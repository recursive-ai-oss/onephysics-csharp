using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Fluids;

/// <summary>
/// A particle fluid: a material and its particles. Generation-checked: a destroyed or default handle throws <see cref="InvalidOperationException"/> on access. The world owns all storage; the same type serves 2D and 3D worlds.
/// Not the buoyancy volume of <c>AddFluid</c>: that is a planar water surface that floats bodies analytically; this simulates the water itself with particles. They do not interact.
/// </summary>
public readonly struct ParticleFluid
{
    private readonly FluidStore? _store;
    private readonly int _generation;
    internal ParticleFluid(FluidStore store, int index, int generation) { _store = store; Index = index; _generation = generation; }

    /// <summary>Stable slot, reused lowest-first after destruction.</summary>
    public int Index { get; }

    /// <summary>True while this slot and generation belong to a live fluid.</summary>
    public bool IsValid => _store != null && _store.FluidAlive(Index, _generation);

    private FluidStore Resolve() => IsValid ? _store! : throw new InvalidOperationException("The particle fluid has been destroyed or is uninitialized.");

    /// <summary>The store of the world that made the handle; throws when the handle is stale.</summary>
    internal FluidStore Owner => Resolve();

    /// <summary>A copy of the material. Setting validates first; <see cref="ParticleFluidDef.Spacing"/> and <see cref="ParticleFluidDef.KernelRadiusScale"/> cannot change.</summary>
    public ParticleFluidDef Definition
    {
        get => Resolve().Fluids[Index]!.Def.Clone();
        set { var s = Resolve(); ArgumentNullException.ThrowIfNull(value); s.SetFluidDef(Index, value); }
    }

    /// <summary>Number of live particles of this fluid.</summary>
    public int ParticleCount => Resolve().CountOf(Index);

    /// <summary>Mass of one particle, kg (kg per metre of depth in a planar world).</summary>
    public float ParticleMass => Resolve().Fluids[Index]!.Mass;

    /// <summary>Adds an emitter that feeds this fluid.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The definition is out of range.</exception>
    public FluidEmitter AddEmitter(FluidEmitterDef definition)
    {
        var s = Resolve(); int i = s.AddEmitter(Index, definition);
        return new FluidEmitter(s, i, s.EmitterHandles.GenerationOf(i));
    }

    /// <summary>Adds a drain that removes this fluid's particles.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The definition is out of range.</exception>
    public FluidDrain AddDrain(FluidDrainDef definition)
    {
        var s = Resolve(); int i = s.AddDrain(Index, definition);
        return new FluidDrain(s, i, s.DrainHandles.GenerationOf(i));
    }

    /// <summary>Removes the fluid with its particles, emitters and drains. Later access throws.</summary>
    public void Destroy() { Resolve().DestroyFluid(Index); }
}

/// <summary>A nozzle of a <see cref="ParticleFluid"/>. Generation-checked like the fluid.</summary>
public readonly struct FluidEmitter
{
    private readonly FluidStore? _store;
    private readonly int _generation;
    internal FluidEmitter(FluidStore store, int index, int generation) { _store = store; Index = index; _generation = generation; }

    /// <summary>Stable slot.</summary>
    public int Index { get; }

    /// <summary>True while the emitter exists.</summary>
    public bool IsValid => _store != null && _store.EmitterHandles.IsAlive(Index, _generation);

    private FluidStore Resolve() => IsValid ? _store! : throw new InvalidOperationException("The fluid emitter has been destroyed or is uninitialized.");

    /// <summary>A copy of the definition; setting validates first.</summary>
    public FluidEmitterDef Definition
    {
        get => Resolve().Emitters[Index]!.Def.Clone();
        set { var s = Resolve(); ArgumentNullException.ThrowIfNull(value); s.SetEmitter(Index, value); }
    }

    /// <summary>Particles this emitter has emitted so far.</summary>
    public int Emitted => Resolve().Emitters[Index]!.Emitted;

    /// <summary>Removes the emitter.</summary>
    public void Destroy() { Resolve().DestroyEmitter(Index); }
}

/// <summary>A kill volume of a <see cref="ParticleFluid"/>. Generation-checked like the fluid.</summary>
public readonly struct FluidDrain
{
    private readonly FluidStore? _store;
    private readonly int _generation;
    internal FluidDrain(FluidStore store, int index, int generation) { _store = store; Index = index; _generation = generation; }

    /// <summary>Stable slot.</summary>
    public int Index { get; }

    /// <summary>True while the drain exists.</summary>
    public bool IsValid => _store != null && _store.DrainHandles.IsAlive(Index, _generation);

    private FluidStore Resolve() => IsValid ? _store! : throw new InvalidOperationException("The fluid drain has been destroyed or is uninitialized.");

    /// <summary>A copy of the definition; setting validates first.</summary>
    public FluidDrainDef Definition
    {
        get => Resolve().Drains[Index]!.Def.Clone();
        set { var s = Resolve(); ArgumentNullException.ThrowIfNull(value); s.SetDrain(Index, value); }
    }

    /// <summary>Particles this drain has removed so far.</summary>
    public int Removed => Resolve().Drains[Index]!.Removed;

    /// <summary>Removes the drain.</summary>
    public void Destroy() { Resolve().DestroyDrain(Index); }
}
