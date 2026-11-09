using System;
using System.Runtime.InteropServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fluids;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    private int FluidSlot(ParticleFluid fluid)
    {
        if (!ReferenceEquals(fluid.Owner, _engine.ParticleFluids)) { throw new InvalidOperationException("The particle fluid belongs to another world."); }
        return fluid.Index;
    }

    /// <summary>Number of live particle fluids (see <see cref="AddParticleFluid"/>). Default 0; a world without them pays nothing.</summary>
    public int ParticleFluidCount => _engine.ParticleFluids.FluidHandles.Count;

    /// <summary>What the last step did (work done, contacts, speed). Zero before the first step.</summary>
    public FluidStepStatistics FluidStatistics { get { var s = _engine.ParticleFluids; return new FluidStepStatistics(s.LastStepped, s.Asleep, s.Count, s.LastCandidates, s.LastDeformables, s.LastContacts, s.LastCoupledContacts, s.LastMaxSpeed); } }

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
            using var replayMutation = RecordMutation("PhysicsWorld3D.FluidSettings");
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

    /// <summary>Adds one particle. Position finite within ±2^21; velocity finite within ±10000 m/s.</summary>
    /// <exception cref="FluidBudgetException">The budget is used up.</exception>
    public void AddFluidParticle(ParticleFluid fluid, Vector3D position, Vector3F velocity = default) => _engine.ParticleFluids.AddParticle(FluidSlot(fluid), position, velocity);

    /// <summary>Fills a box with particles on the fluid's lattice (x fastest, then y, then z; half a spacing inside the minimum corner). Returns the number added.</summary>
    /// <exception cref="FluidBudgetException">The box holds more than the budget allows.</exception>
    public int FillFluidBox(ParticleFluid fluid, Vector3D minimum, Vector3D maximum, Vector3F velocity = default) => _engine.ParticleFluids.FillBox(FluidSlot(fluid), minimum, maximum, velocity);

    /// <summary>Fills a ball with lattice particles whose centres lie inside it. Returns the number added.</summary>
    /// <exception cref="FluidBudgetException">The ball holds more than the budget allows.</exception>
    public int FillFluidSphere(ParticleFluid fluid, Vector3D centre, double radius, Vector3F velocity = default) => _engine.ParticleFluids.FillSphere(FluidSlot(fluid), centre, radius, velocity);

    /// <summary>Copies particles (storage order, after <paramref name="skip"/>) into <paramref name="results"/>; returns how many. The order changes from step to step (particles are sorted by cell); ids do not.</summary>
    public int GetFluidParticles(Span<FluidParticleInfo> results, int skip = 0)
    {
        var s = _engine.ParticleFluids;
        int n = 0;
        for (int p = Math.Max(skip, 0); p < s.Count && n < results.Length; p++) { results[n++] = s.Info(p); }
        return n;
    }

    /// <summary>Particles whose centres lie within <paramref name="radius"/> of the point, ascending in storage order; <paramref name="total"/> counts all even when <paramref name="results"/> is shorter. <paramref name="fluid"/> restricts to one fluid slot (-1: all).</summary>
    public int OverlapFluidSphere(Vector3D centre, float radius, Span<FluidParticleInfo> results, out int total, int fluid = -1)
    {
        if (!(radius >= 0f) || !float.IsFinite(radius)) { throw new ArgumentOutOfRangeException(nameof(radius), radius, "radius must be finite and not negative."); }
        return _engine.ParticleFluids.OverlapSphere(centre, radius, fluid, results, out total);
    }

    /// <summary>Particles inside the inclusive box, in storage order.</summary>
    public int OverlapFluidBox(Vector3D minimum, Vector3D maximum, Span<FluidParticleInfo> results, out int total, int fluid = -1) => _engine.ParticleFluids.OverlapBox(minimum, maximum, fluid, results, out total);

    /// <summary>The first particle a ray meets; particles are spheres of half the spacing. Returns false when none is within <paramref name="maxDistance"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The origin, direction or distance is not finite.</exception>
    public bool RaycastFluid(Vector3D origin, Vector3F direction, out FluidRayHit hit, float maxDistance = 1000f, int fluid = -1)
    {
        if (!(double.IsFinite(origin.X) && double.IsFinite(origin.Y) && double.IsFinite(origin.Z))) { throw new ArgumentOutOfRangeException(nameof(origin), origin, "origin must be finite."); }
        if (!(float.IsFinite(direction.X) && float.IsFinite(direction.Y) && float.IsFinite(direction.Z))) { throw new ArgumentOutOfRangeException(nameof(direction), direction, "direction must be finite."); }
        if (!(maxDistance >= 0f) || !float.IsFinite(maxDistance)) { throw new ArgumentOutOfRangeException(nameof(maxDistance), maxDistance, "maxDistance must be finite and not negative."); }
        return _engine.ParticleFluids.RayCast(origin, direction, maxDistance, fluid, out hit);
    }

    /// <summary>The SPH density of a fluid at a point, kg/m³ (all fluids when <paramref name="fluid"/> is -1). About the fluid's rest density inside it and half that on a flat free surface.</summary>
    public float FluidDensityAt(Vector3D point, int fluid = -1)
    {
        var s = _engine.ParticleFluids;
        s.PrepareQueries();
        return s.DensityAt(point.X, point.Y, point.Z, fluid);
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
