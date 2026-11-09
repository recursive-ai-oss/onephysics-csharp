using System;
using System.Runtime.InteropServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fluids;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    private int FluidSlot(ParticleFluid fluid)
    {
        if (!ReferenceEquals(fluid.Owner, _engine.ParticleFluids)) { throw new InvalidOperationException("The particle fluid belongs to another world."); }
        return fluid.Index;
    }

    /// <summary>Number of live particle fluids (see <see cref="AddParticleFluid"/>). Default 0; a world without them pays nothing.</summary>
    public int ParticleFluidCount => _engine.ParticleFluids.FluidHandles.Count;

    /// <summary>Number of fluid particles in the world.</summary>
    public int FluidParticleCount => _engine.ParticleFluids.Count;

    /// <summary>How the particle fluids are stepped and when they sleep (see <see cref="FluidSettings"/>). Setting validates, wakes the fluids and is recorded.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A member is out of range.</exception>
    public FluidSettings FluidSettings
    {
        get => _engine.ParticleFluids.Settings;
        set
        {
            value.Validate();
            using var replayMutation = RecordMutation("PhysicsWorld2D.FluidSettings");
            _engine.ParticleFluids.Settings = value;
            _engine.ParticleFluids.Wake();
        }
    }

    /// <summary>Adds a particle fluid: position-based fluids with SPH kernels, two-way coupled with rigid bodies, soft bodies, cloth and rods. See <see cref="ParticleFluidDef"/> for units and defaults.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A member is out of range, or the spacing differs from the world's other fluids.</exception>
    public ParticleFluid AddParticleFluid(ParticleFluidDef definition)
    {
        var s = _engine.ParticleFluids;
        int i = s.AddFluid(definition);
        return new ParticleFluid(s, i, s.FluidHandles.GenerationOf(i));
    }

    /// <summary>Adds a particle fluid that is water (1000 kg/m³, a little viscosity) with the given particle spacing, metres.</summary>
    public ParticleFluid AddParticleWater(float spacing = 0.05f) => AddParticleFluid(ParticleFluidDef.Water(spacing));

    /// <summary>Raw tier: the live particle fluid in a slot. Throws <see cref="ArgumentOutOfRangeException"/> for a dead or out-of-range slot.</summary>
    public ParticleFluid GetParticleFluid(int index)
    {
        var s = _engine.ParticleFluids;
        if (!s.FluidHandles.IsLive(index)) { throw new ArgumentOutOfRangeException(nameof(index), index, "No live particle fluid in this slot."); }
        return new ParticleFluid(s, index, s.FluidHandles.GenerationOf(index));
    }

    /// <summary>Adds one particle. Position finite within ±2^21; velocity finite within ±10000 m/s. The planar world's particles are discs of one metre depth.</summary>
    /// <exception cref="FluidBudgetException">The budget is used up.</exception>
    public void AddFluidParticle(ParticleFluid fluid, Vector2D position, Vector2F velocity = default) => _engine.ParticleFluids.AddParticle(FluidSlot(fluid), new Vector3D(position.X, position.Y, 0.0), new Vector3F(velocity.X, velocity.Y, 0f));

    /// <summary>Fills a box with particles on the fluid's lattice (x fastest, then y; half a spacing inside the minimum corner). Returns the number added.</summary>
    /// <exception cref="FluidBudgetException">The box holds more than the budget allows.</exception>
    public int FillFluidBox(ParticleFluid fluid, Vector2D minimum, Vector2D maximum, Vector2F velocity = default) => _engine.ParticleFluids.FillBox(FluidSlot(fluid), new Vector3D(minimum.X, minimum.Y, 0.0), new Vector3D(maximum.X, maximum.Y, 0.0), new Vector3F(velocity.X, velocity.Y, 0f));

    /// <summary>Fills a disc with lattice particles whose centres lie inside it. Returns the number added.</summary>
    /// <exception cref="FluidBudgetException">The disc holds more than the budget allows.</exception>
    public int FillFluidSphere(ParticleFluid fluid, Vector2D centre, double radius, Vector2F velocity = default) => _engine.ParticleFluids.FillSphere(FluidSlot(fluid), new Vector3D(centre.X, centre.Y, 0.0), radius, new Vector3F(velocity.X, velocity.Y, 0f));

    /// <summary>Copies particles (storage order, after <paramref name="skip"/>) into <paramref name="results"/>; returns how many. The order changes from step to step (particles are sorted by cell); ids do not.</summary>
    public int GetFluidParticles(Span<FluidParticleInfo> results, int skip = 0)
    {
        var s = _engine.ParticleFluids;
        int n = 0;
        for (int p = Math.Max(skip, 0); p < s.Count && n < results.Length; p++) { results[n++] = s.Info(p); }
        return n;
    }

    /// <summary>Particles whose centres lie within <paramref name="radius"/> of the point, ascending in storage order; <paramref name="total"/> counts all even when <paramref name="results"/> is shorter. <paramref name="fluid"/> restricts to one fluid slot (-1: all).</summary>
    public int OverlapFluidSphere(Vector2D centre, float radius, Span<FluidParticleInfo> results, out int total, int fluid = -1)
    {
        if (!(radius >= 0f) || !float.IsFinite(radius)) { throw new ArgumentOutOfRangeException(nameof(radius), radius, "radius must be finite and not negative."); }
        return _engine.ParticleFluids.OverlapSphere(new Vector3D(centre.X, centre.Y, 0.0), radius, fluid, results, out total);
    }

    /// <summary>Particles inside the inclusive box, in storage order.</summary>
    public int OverlapFluidBox(Vector2D minimum, Vector2D maximum, Span<FluidParticleInfo> results, out int total, int fluid = -1) => _engine.ParticleFluids.OverlapBox(new Vector3D(minimum.X, minimum.Y, 0.0), new Vector3D(maximum.X, maximum.Y, 0.0), fluid, results, out total);

    /// <summary>The first particle a ray meets; particles are spheres of half the spacing. Returns false when none is within <paramref name="maxDistance"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The origin, direction or distance is not finite.</exception>
    public bool RaycastFluid(Vector2D origin, Vector2F direction, out FluidRayHit hit, float maxDistance = 1000f, int fluid = -1)
    {
        if (!(double.IsFinite(origin.X) && double.IsFinite(origin.Y))) { throw new ArgumentOutOfRangeException(nameof(origin), origin, "origin must be finite."); }
        if (!(float.IsFinite(direction.X) && float.IsFinite(direction.Y))) { throw new ArgumentOutOfRangeException(nameof(direction), direction, "direction must be finite."); }
        if (!(maxDistance >= 0f) || !float.IsFinite(maxDistance)) { throw new ArgumentOutOfRangeException(nameof(maxDistance), maxDistance, "maxDistance must be finite and not negative."); }
        return _engine.ParticleFluids.RayCast(new Vector3D(origin.X, origin.Y, 0.0), new Vector3F(direction.X, direction.Y, 0f), maxDistance, fluid, out hit);
    }

    /// <summary>The SPH density of a fluid at a point, kg/m³ (all fluids when <paramref name="fluid"/> is -1). About the fluid's rest density inside it and half that on a flat free surface.</summary>
    public float FluidDensityAt(Vector2D point, int fluid = -1)
    {
        var s = _engine.ParticleFluids;
        s.PrepareQueries();
        return s.DensityAt(point.X, point.Y, 0.0, fluid);
    }

    /// <summary>Bodies that entered or left a fluid in the last step, ordered by body slot then fluid slot. Valid until the next <see cref="Step"/>.</summary>
    public ReadOnlySpan<FluidEvent> FluidEvents => CollectionsMarshal.AsSpan(_engine.ParticleFluids.Events);

    /// <summary>How much of the body the fluid covers after the last step, how much of its surface the particles touch and the average force they put on it. Zero for a body the fluid has not met.</summary>
    /// <exception cref="InvalidHandleException">The body is stale.</exception>
    public FluidBodyState GetFluidState(Body body, ParticleFluid fluid)
    {
        int slot = FluidSlot(fluid);
        var h = body.RawHandle;
        _core.Resolve(h);
        return _engine.ParticleFluids.TryGetResult(h.Index, h.Generation, slot, out var r) ? new FluidBodyState(r.Fraction, r.WetArea, new Vector3F(r.Fx, r.Fy, r.Fz)) : default;
    }
}
