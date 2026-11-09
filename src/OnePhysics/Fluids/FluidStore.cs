using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Fluids;

/// <summary>
/// Soft-body particles and rod segments near the fluid, presented as capsules for one step: a point (both ends the same particle) or a segment between two particles. The host fills the positions and masses; the fluid step accumulates the
/// momentum it gave each end and the host applies it.
/// </summary>
internal sealed class FluidDeformables
{
    public int Count;
    public double[] Ax = new double[16], Ay = new double[16], Az = new double[16], Bx = new double[16], By = new double[16], Bz = new double[16];
    public float[] Radius = new float[16], InvA = new float[16], InvB = new float[16];
    public float[] Vax = new float[16], Vay = new float[16], Vaz = new float[16], Vbx = new float[16], Vby = new float[16], Vbz = new float[16];
    /// <summary>What the host needs to find the particles again: a kind (0 soft, 1 rod) in the high byte and the owner's slot, and the particle indices of the two ends.</summary>
    public int[] Owner = new int[16], PA = new int[16], PB = new int[16];
    // Accumulated by the fluid step.
    public float[] JAx = new float[16], JAy = new float[16], JAz = new float[16], JBx = new float[16], JBy = new float[16], JBz = new float[16];
    /// <summary>Fluid mass in contact with each end, summed over the substeps (divide by the substep count): the share of the fluid a deformable's particle is given as extra inertia when it receives the fluid's momentum.</summary>
    public float[] MA = new float[16], MB = new float[16];
    public int Substeps = 1;

    /// <summary>The velocity change end A gets from its accumulated momentum: the momentum over its own mass plus the fluid mass in contact with it.</summary>
    public float DvScaleA(int e) => 1f / (1f / InvA[e] + MA[e] / Substeps);

    /// <summary>The same for end B.</summary>
    public float DvScaleB(int e) => 1f / (1f / InvB[e] + MB[e] / Substeps);

    public void Clear() { Count = 0; }

    private void Grow()
    {
        int c = Count * 2;
        Array.Resize(ref Ax, c); Array.Resize(ref Ay, c); Array.Resize(ref Az, c); Array.Resize(ref Bx, c); Array.Resize(ref By, c); Array.Resize(ref Bz, c);
        Array.Resize(ref Radius, c); Array.Resize(ref InvA, c); Array.Resize(ref InvB, c);
        Array.Resize(ref Vax, c); Array.Resize(ref Vay, c); Array.Resize(ref Vaz, c); Array.Resize(ref Vbx, c); Array.Resize(ref Vby, c); Array.Resize(ref Vbz, c);
        Array.Resize(ref Owner, c); Array.Resize(ref PA, c); Array.Resize(ref PB, c);
        Array.Resize(ref MA, c); Array.Resize(ref MB, c);
        Array.Resize(ref JAx, c); Array.Resize(ref JAy, c); Array.Resize(ref JAz, c); Array.Resize(ref JBx, c); Array.Resize(ref JBy, c); Array.Resize(ref JBz, c);
    }

    /// <summary>Adds a capsule from end A to end B (the same particle for a point).</summary>
    public void Add(double ax, double ay, double az, double bx, double by, double bz, float radius, float invA, float invB,
        float vax, float vay, float vaz, float vbx, float vby, float vbz, int owner, int pa, int pb)
    {
        int n = Count;
        if (n == Ax.Length) { Grow(); }
        Ax[n] = ax; Ay[n] = ay; Az[n] = az; Bx[n] = bx; By[n] = by; Bz[n] = bz;
        Radius[n] = radius; InvA[n] = invA; InvB[n] = invB;
        Vax[n] = vax; Vay[n] = vay; Vaz[n] = vaz; Vbx[n] = vbx; Vby[n] = vby; Vbz[n] = vbz;
        Owner[n] = owner; PA[n] = pa; PB[n] = pb;
        JAx[n] = 0f; JAy[n] = 0f; JAz[n] = 0f; JBx[n] = 0f; JBy[n] = 0f; JBz[n] = 0f; MA[n] = 0f; MB[n] = 0f;
        Count = n + 1;
    }
}

/// <summary>A fluid as the world keeps it: its material and what is derived from it.</summary>
internal sealed class FluidState
{
    public ParticleFluidDef Def = new();
    public float Mass;       // kg per particle
    public float N0;         // SPH density of a particle in a rest lattice, kg/m³ (m³ in 2D: kg/m²)
    public int Count;        // live particles, derived
}

internal sealed class EmitterState
{
    public int Fluid, FluidGen;
    public FluidEmitterDef Def = new();
    public float Phase;      // distance the flow has advanced since the last layer, metres
    public int Emitted;
}

internal sealed class DrainState
{
    public int Fluid = -1, FluidGen;   // -1: every fluid
    public FluidDrainDef Def = new();
    public int Removed;
}

/// <summary>A body that was wet at the end of the last step, for enter and exit events.</summary>
internal struct WetRecord
{
    public int Body, BodyGen, Fluid;
    public bool Wet;
}

/// <summary>
/// Particle fluids, one store shared by both engines (a planar world is embedded with z = 0): position-based fluids with the poly6 and spiky kernels, a uniform grid searched in sorted cell order, and two-way contact with rigid bodies,
/// soft bodies and rods. It runs once per world step, after the rigid step, serially in stage order with each stage parallel over particles in index order and combined in index order, so results do not depend on the thread count.
/// </summary>
internal sealed partial class FluidStore : ISnapshotSection
{
    /// <summary>Section id: "PFLD" read as a little-endian integer.</summary>
    public const int Id = 0x444C4650;

    public const int MaxFluids = 64;

    internal readonly IFluidHost Host;
    internal readonly int Dim;
    internal FluidSettings Settings = FluidSettings.Default;

    public HandleTable FluidHandles = new(), EmitterHandles = new(), DrainHandles = new();
    public FluidState?[] Fluids = new FluidState?[4];
    public EmitterState?[] Emitters = new EmitterState?[4];
    public DrainState?[] Drains = new DrainState?[4];

    /// <summary>Spacing and kernel radius of every fluid in the world (set by the first fluid; 0 while there is none).</summary>
    internal float Spacing, KernelScale;

    // Particles, in the order of the last step's final sort (cell order, ties by earlier order).
    internal int Count;
    internal int[] PId = new int[64];
    internal byte[] PFluid = new byte[64];
    internal double[] PX = new double[64], PY = new double[64], PZ = new double[64];
    internal float[] PVx = new float[64], PVy = new float[64], PVz = new float[64];
    internal int NextId;

    internal bool Asleep;
    internal float QuietTime;
    internal long StepCount;

    internal WetRecord[] Wet = new WetRecord[8];
    internal int WetCount;

    public FluidStore(IFluidHost host)
    {
        Host = host;
        Dim = host.Planar ? 2 : 3;
    }

    public bool HasWork => FluidHandles.Count > 0;

    // ------------------------------------------------------------------ fluids

    internal bool FluidAlive(int i, int gen) => FluidHandles.IsAlive(i, gen);

    internal static void Grow<T>(ref T[] a, int n)
    {
        if (a.Length >= n) { return; }
        int c = Math.Max(n, Math.Max(8, a.Length * 2));
        Array.Resize(ref a, c);
    }

    internal int AddFluid(ParticleFluidDef def)
    {
        ArgumentNullException.ThrowIfNull(def);
        def.Validate();
        if (FluidHandles.Count >= MaxFluids) { throw new InvalidOperationException("A world holds at most " + MaxFluids.ToString(CultureInfo.InvariantCulture) + " particle fluids."); }
        if (FluidHandles.Count > 0)
        {
            if (def.Spacing != Spacing) { throw new ArgumentOutOfRangeException(nameof(def), def.Spacing, "ParticleFluidDef.Spacing is " + def.Spacing.ToString("R", CultureInfo.InvariantCulture) + " but this world's fluids use " + Spacing.ToString("R", CultureInfo.InvariantCulture) + "; every fluid of a world shares one spacing."); }
            if (def.KernelRadiusScale != KernelScale) { throw new ArgumentOutOfRangeException(nameof(def), def.KernelRadiusScale, "ParticleFluidDef.KernelRadiusScale differs from the world's other fluids."); }
        }

        using var mutation = Host.FluidMutation("World.AddParticleFluid");
        if (FluidHandles.Count == 0) { Spacing = def.Spacing; KernelScale = def.KernelRadiusScale; }
        int i = FluidHandles.Allocate();
        Grow(ref Fluids, i + 1);
        var s = new FluidState { Def = def.Clone() };
        Derive(s);
        Fluids[i] = s;
        return i;
    }

    /// <summary>Mass of a particle and the SPH density of a rest lattice, from the definition.</summary>
    internal void Derive(FluidState s)
    {
        var k = new FluidKernels(Dim, Spacing * KernelScale);
        s.Mass = s.Def.RestDensity * (Dim == 3 ? Spacing * Spacing * Spacing : Spacing * Spacing);
        s.N0 = (float)(s.Mass * k.LatticeSum(Spacing));
    }

    internal void SetFluidDef(int i, ParticleFluidDef def)
    {
        def.Validate();
        var f = Fluids[i]!;
        if (def.Spacing != Spacing || def.KernelRadiusScale != KernelScale) { throw new ArgumentOutOfRangeException(nameof(def), "ParticleFluidDef.Spacing and KernelRadiusScale cannot change after the fluid is created."); }
        using var mutation = Host.FluidMutation("ParticleFluid.Definition");
        f.Def = def.Clone();
        Derive(f);
        Wake();
    }

    internal void DestroyFluid(int i)
    {
        using var mutation = Host.FluidMutation("ParticleFluid.Destroy");
        // Remove its particles (order of the rest is kept), emitters, drains and wet records.
        int w = 0;
        for (int p = 0; p < Count; p++)
        {
            if (PFluid[p] == i) { continue; }
            if (w != p) { MoveParticle(p, w); }
            w++;
        }

        Count = w;
        for (int e = 0; e < EmitterHandles.HighWater; e++) { if (EmitterHandles.IsLive(e) && Emitters[e]!.Fluid == i) { Emitters[e] = null; EmitterHandles.Free(e); } }
        for (int d = 0; d < DrainHandles.HighWater; d++) { if (DrainHandles.IsLive(d) && Drains[d]!.Fluid == i) { Drains[d] = null; DrainHandles.Free(d); } }
        int ww = 0;
        for (int k = 0; k < WetCount; k++) { if (Wet[k].Fluid != i) { Wet[ww++] = Wet[k]; } }
        WetCount = ww;
        Fluids[i] = null;
        FluidHandles.Free(i);
        if (FluidHandles.Count == 0) { Spacing = 0f; KernelScale = 0f; Count = 0; }
        Wake();
    }

    private void MoveParticle(int from, int to)
    {
        _qValid = false;
        PId[to] = PId[from]; PFluid[to] = PFluid[from];
        PX[to] = PX[from]; PY[to] = PY[from]; PZ[to] = PZ[from];
        PVx[to] = PVx[from]; PVy[to] = PVy[from]; PVz[to] = PVz[from];
    }

    internal int CountOf(int fluid)
    {
        int n = 0;
        for (int p = 0; p < Count; p++) { if (PFluid[p] == fluid) { n++; } }
        return n;
    }

    internal void Wake() { Asleep = false; QuietTime = 0f; }

    // ------------------------------------------------------------------ particles

    private void EnsureCapacity(int n)
    {
        if (PId.Length >= n) { return; }
        int c = Math.Max(n, PId.Length * 2);
        Array.Resize(ref PId, c); Array.Resize(ref PFluid, c);
        Array.Resize(ref PX, c); Array.Resize(ref PY, c); Array.Resize(ref PZ, c);
        Array.Resize(ref PVx, c); Array.Resize(ref PVy, c); Array.Resize(ref PVz, c);
    }

    private void CheckBudget(int add)
    {
        if ((long)Count + add > Settings.MaxParticles) { throw new FluidBudgetException("Adding " + add.ToString(CultureInfo.InvariantCulture) + " particles to " + Count.ToString(CultureInfo.InvariantCulture) + " would pass the budget of " + Settings.MaxParticles.ToString(CultureInfo.InvariantCulture) + " (FluidSettings.MaxParticles)."); }
    }

    private static void CheckPosition(Vector3D p, string name)
    {
        FluidEmitterDef.Check(p.X, name, 2097152.0); FluidEmitterDef.Check(p.Y, name, 2097152.0); FluidEmitterDef.Check(p.Z, name, 2097152.0);
    }

    private static void CheckVelocity(Vector3F v, string name)
    {
        if (!(float.IsFinite(v.X) && float.IsFinite(v.Y) && float.IsFinite(v.Z) && Math.Abs(v.X) <= 10000f && Math.Abs(v.Y) <= 10000f && Math.Abs(v.Z) <= 10000f)) { throw new ArgumentOutOfRangeException(name, v, name + " must be finite within ±10000 m/s."); }
    }

    private void Append(int fluid, double x, double y, double z, float vx, float vy, float vz)
    {
        EnsureCapacity(Count + 1);
        if (NextId == int.MaxValue) { throw new InvalidOperationException("Particle ids are exhausted."); }
        _qValid = false;
        int p = Count++;
        PId[p] = NextId++; PFluid[p] = (byte)fluid;
        PX[p] = x; PY[p] = y; PZ[p] = Dim == 3 ? z : 0.0;
        PVx[p] = vx; PVy[p] = vy; PVz[p] = Dim == 3 ? vz : 0f;
    }

    internal void AddParticle(int fluid, Vector3D at, Vector3F velocity)
    {
        CheckPosition(at, nameof(at)); CheckVelocity(velocity, nameof(velocity));
        CheckBudget(1);
        using var mutation = Host.FluidMutation("ParticleFluid.AddParticle");
        Append(fluid, at.X, at.Y, at.Z, velocity.X, velocity.Y, velocity.Z);
        Wake();
    }

    /// <summary>Fills the box with particles on the lattice of the spacing, starting half a spacing inside its minimum corner, x fastest then y then z. Returns the count.</summary>
    internal int FillBox(int fluid, Vector3D min, Vector3D max, Vector3F velocity)
    {
        CheckPosition(min, nameof(min)); CheckPosition(max, nameof(max)); CheckVelocity(velocity, nameof(velocity));
        double s = Spacing;
        int nx = Lattice(min.X, max.X, s), ny = Lattice(min.Y, max.Y, s), nz = Dim == 3 ? Lattice(min.Z, max.Z, s) : 1;
        long total = (long)nx * ny * nz;
        if (total > int.MaxValue) { throw new FluidBudgetException("The box holds more particles than the budget allows."); }
        CheckBudget((int)total);
        using var mutation = Host.FluidMutation("ParticleFluid.FillBox");
        EnsureCapacity(Count + (int)total);
        for (int k = 0; k < nz; k++)
        {
            for (int j = 0; j < ny; j++)
            {
                for (int i = 0; i < nx; i++)
                {
                    Append(fluid, min.X + (i + 0.5) * s, min.Y + (j + 0.5) * s, Dim == 3 ? min.Z + (k + 0.5) * s : 0.0, velocity.X, velocity.Y, velocity.Z);
                }
            }
        }

        if (total > 0) { Wake(); }
        return (int)total;
    }

    private static int Lattice(double lo, double hi, double s)
    {
        if (!(hi > lo)) { return 0; }
        double n = Math.Floor((hi - lo) / s + 1e-3);
        return n > 1e8 ? 100000000 : (int)n;
    }

    /// <summary>Fills a sphere (disc) with lattice particles whose centres lie inside it. Returns the count.</summary>
    internal int FillSphere(int fluid, Vector3D centre, double radius, Vector3F velocity)
    {
        CheckPosition(centre, nameof(centre)); CheckVelocity(velocity, nameof(velocity));
        if (!(radius > 0.0 && radius <= 2097152.0)) { throw new ArgumentOutOfRangeException(nameof(radius), radius, "radius must be positive and finite."); }
        double s = Spacing;
        int n = (int)Math.Min(Math.Floor(radius / s) + 1, 100000);
        long cells = Dim == 3 ? (long)(2 * n + 1) * (2 * n + 1) * (2 * n + 1) : (long)(2 * n + 1) * (2 * n + 1);
        if (cells > 200_000_000L) { throw new FluidBudgetException("The sphere is too large for the particle spacing."); }
        // Count first so the budget is checked before anything is added.
        int total = 0;
        for (int pass = 0; pass < 2; pass++)
        {
            if (pass == 1)
            {
                CheckBudget(total);
                EnsureCapacity(Count + total);
            }

            for (int k = Dim == 3 ? -n : 0; k <= (Dim == 3 ? n : 0); k++)
            {
                for (int j = -n; j <= n; j++)
                {
                    for (int i = -n; i <= n; i++)
                    {
                        double dx = i * s, dy = j * s, dz = Dim == 3 ? k * s : 0.0;
                        if (dx * dx + dy * dy + dz * dz > radius * radius) { continue; }
                        if (pass == 0) { total++; }
                        else { Append(fluid, centre.X + dx, centre.Y + dy, Dim == 3 ? centre.Z + dz : 0.0, velocity.X, velocity.Y, velocity.Z); }
                    }
                }
            }
        }

        if (total > 0) { using var mutation = Host.FluidMutation("ParticleFluid.FillSphere"); Wake(); }
        return total;
    }

    // ------------------------------------------------------------------ emitters and drains

    internal int AddEmitter(int fluid, FluidEmitterDef def)
    {
        ArgumentNullException.ThrowIfNull(def);
        def.Validate();
        using var mutation = Host.FluidMutation("ParticleFluid.AddEmitter");
        int i = EmitterHandles.Allocate();
        Grow(ref Emitters, i + 1);
        Emitters[i] = new EmitterState { Fluid = fluid, FluidGen = FluidHandles.GenerationOf(fluid), Def = def.Clone() };
        Wake();
        return i;
    }

    internal int AddDrain(int fluid, FluidDrainDef def)
    {
        ArgumentNullException.ThrowIfNull(def);
        def.Validate();
        using var mutation = Host.FluidMutation("ParticleFluid.AddDrain");
        int i = DrainHandles.Allocate();
        Grow(ref Drains, i + 1);
        Drains[i] = new DrainState { Fluid = fluid, FluidGen = fluid >= 0 ? FluidHandles.GenerationOf(fluid) : 0, Def = def.Clone() };
        Wake();
        return i;
    }

    internal void SetEmitter(int i, FluidEmitterDef def)
    {
        def.Validate();
        using var mutation = Host.FluidMutation("FluidEmitter.Definition");
        Emitters[i]!.Def = def.Clone();
        Wake();
    }

    internal void SetDrain(int i, FluidDrainDef def)
    {
        def.Validate();
        using var mutation = Host.FluidMutation("FluidDrain.Definition");
        Drains[i]!.Def = def.Clone();
        Wake();
    }

    internal void DestroyEmitter(int i) { using var mutation = Host.FluidMutation("FluidEmitter.Destroy"); Emitters[i] = null; EmitterHandles.Free(i); }

    internal void DestroyDrain(int i) { using var mutation = Host.FluidMutation("FluidDrain.Destroy"); Drains[i] = null; DrainHandles.Free(i); }

    /// <summary>Layers emitted this step, in emitter slot order. Each emitter advances its flow by speed × dt and emits one layer for every full spacing, at the offset the flow has already travelled.</summary>
    private void EmitAll(float dt)
    {
        for (int e = 0; e < EmitterHandles.HighWater; e++)
        {
            if (!EmitterHandles.IsLive(e)) { continue; }
            var em = Emitters[e]!;
            var d = em.Def;
            if (!d.Enabled || !(d.Speed > 0f)) { continue; }
            if (!FluidHandles.IsAlive(em.Fluid, em.FluidGen)) { continue; }
            if (d.MaxEmitted > 0 && em.Emitted >= d.MaxEmitted) { continue; }
            float s = Spacing;
            em.Phase += d.Speed * dt;
            while (em.Phase >= s)
            {
                em.Phase -= s;
                if (!EmitLayer(em, em.Phase)) { em.Phase = Math.Min(em.Phase, s); break; }
            }
        }
    }

    /// <summary>One layer of lattice points across the nozzle, <paramref name="travelled"/> metres downstream of the nozzle plane. False when the budget or the emitter's limit stops it.</summary>
    private bool EmitLayer(EmitterState em, float travelled)
    {
        var d = em.Def;
        double dx = d.Direction.X, dy = d.Direction.Y, dz = Dim == 3 ? d.Direction.Z : 0.0;
        double len = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        if (!(len > 0.0)) { return false; }
        dx /= len; dy /= len; dz /= len;
        // Two unit vectors across the flow.
        double ux, uy, uz, vx, vy, vz;
        if (Dim == 2) { ux = -dy; uy = dx; uz = 0.0; vx = 0.0; vy = 0.0; vz = 0.0; }
        else
        {
            // Cross with the axis least aligned with the direction.
            double ax = Math.Abs(dx), ay = Math.Abs(dy), az = Math.Abs(dz);
            double hx = 0, hy = 0, hz = 0;
            if (ax <= ay && ax <= az) { hx = 1; } else if (ay <= az) { hy = 1; } else { hz = 1; }
            ux = dy * hz - dz * hy; uy = dz * hx - dx * hz; uz = dx * hy - dy * hx;
            double ul = Math.Sqrt(ux * ux + uy * uy + uz * uz);
            ux /= ul; uy /= ul; uz /= ul;
            vx = dy * uz - dz * uy; vy = dz * ux - dx * uz; vz = dx * uy - dy * ux;
        }

        double s = Spacing;
        bool disc = d.Kind == FluidVolumeKind.Sphere;
        double ru = disc ? d.Size.X : d.Size.X, rv = disc ? d.Size.X : d.Size.Y;
        int nu = (int)Math.Floor(ru / s), nv = Dim == 3 ? (int)Math.Floor(rv / s) : 0;
        float speed = d.Speed;
        for (int j = -nv; j <= nv; j++)
        {
            for (int i = -nu; i <= nu; i++)
            {
                double a = i * s, b = j * s;
                if (disc && a * a + b * b > ru * ru) { continue; }
                if (Count >= Settings.MaxParticles) { return false; }
                if (d.MaxEmitted > 0 && em.Emitted >= d.MaxEmitted) { return false; }
                double px = d.Position.X + ux * a + vx * b + dx * travelled;
                double py = d.Position.Y + uy * a + vy * b + dy * travelled;
                double pz = d.Position.Z + uz * a + vz * b + dz * travelled;
                Append(em.Fluid, px, py, pz, (float)(dx * speed), (float)(dy * speed), (float)(dz * speed));
                em.Emitted++;
            }
        }

        return true;
    }

    private void DrainAll()
    {
        bool any = false;
        for (int d = 0; d < DrainHandles.HighWater; d++) { if (DrainHandles.IsLive(d) && Drains[d]!.Def.Enabled) { any = true; break; } }
        if (!any || Count == 0) { return; }
        int w = 0;
        for (int p = 0; p < Count; p++)
        {
            bool kill = false;
            for (int d = 0; d < DrainHandles.HighWater && !kill; d++)
            {
                if (!DrainHandles.IsLive(d)) { continue; }
                var ds = Drains[d]!;
                if (!ds.Def.Enabled) { continue; }
                if (ds.Fluid >= 0 && ds.Fluid != PFluid[p]) { continue; }
                if (Inside(ds.Def, PX[p], PY[p], PZ[p])) { ds.Removed++; kill = true; }
            }

            if (kill) { continue; }
            if (w != p) { MoveParticle(p, w); }
            w++;
        }

        Count = w;
    }

    private bool Inside(FluidDrainDef d, double x, double y, double z)
    {
        double dx = x - d.Position.X, dy = y - d.Position.Y, dz = Dim == 3 ? z - d.Position.Z : 0.0;
        if (d.Kind == FluidVolumeKind.Box) { return Math.Abs(dx) <= d.Size.X && Math.Abs(dy) <= d.Size.Y && (Dim == 2 || Math.Abs(dz) <= d.Size.Z); }
        return dx * dx + dy * dy + dz * dz <= (double)d.Size.X * d.Size.X;
    }
}
