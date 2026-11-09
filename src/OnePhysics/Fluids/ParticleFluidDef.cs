using System;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics;

/// <summary>What kind of region a <see cref="FluidEmitterDef"/> or <see cref="FluidDrainDef"/> covers.</summary>
public enum FluidVolumeKind : byte
{
    /// <summary>An axis-aligned box given by its centre and half extents (a planar world uses x and y).</summary>
    Box = 0,

    /// <summary>A sphere (a disc in a planar world) given by its centre and radius.</summary>
    Sphere = 1,
}

/// <summary>
/// Material of a particle fluid: position-based fluids (Macklin and Müller 2013) with the SPH poly6 and spiky kernels (Müller, Charypar, Gross 2003). One definition describes one fluid; a world may hold several
/// (water and oil, say) as long as they share a particle spacing. Units are SI; in a planar world a "volume" is an area of one metre depth, so densities keep their numbers.
/// Defaults are water at 0.05 m spacing. Validation throws <see cref="ArgumentOutOfRangeException"/> naming the member.
/// </summary>
public sealed class ParticleFluidDef
{
    /// <summary>Distance between neighbouring particles at rest, metres. Every fluid of one world has the same spacing (the first fluid sets it). Range 0.001 to 10. Default 0.05.</summary>
    public float Spacing { get; set; } = 0.05f;

    /// <summary>Rest mass density, kg/m³ (kg/m² in a planar world). One particle has mass density × spacing³ (spacing² in 2D). Range 0.01 to 1e5. Default 1000.</summary>
    public float RestDensity { get; set; } = 1000f;

    /// <summary>Smoothing radius as a multiple of the spacing. Larger is smoother and slower. Range 1.2 to 4. Default 2.</summary>
    public float KernelRadiusScale { get; set; } = 2f;

    /// <summary>XSPH viscosity coefficient (0 = none, around 0.01 for water, up to 1 for honey). Range 0 to 1. Default 0.01.</summary>
    public float Viscosity { get; set; } = 0.01f;

    /// <summary>Vorticity confinement strength, m/s² per unit of curl (0 = off). Puts back the swirl that damping removes. Range 0 to 100. Default 0.</summary>
    public float Vorticity { get; set; }

    /// <summary>Artificial pressure strength against clumping at the free surface (the tensile-instability correction); it also gives a little surface tension. Range 0 to 10. Default 0.05.</summary>
    public float TensileCorrection { get; set; } = 0.05f;

    /// <summary>Constraint-force mixing: how soft the density constraint is, as a fraction of the squared gradient norm of the constraint in a rest lattice (so the same value means the same softness in 2D, 3D and at any spacing). Larger is softer and more stable. Range 1e-6 to 100. Default 0.03.</summary>
    public float Relaxation { get; set; } = 0.03f;

    /// <summary>Friction coefficient against shapes (combined with the shape's friction by the geometric mean). Range 0 to 100. Default 0.1.</summary>
    public float Friction { get; set; } = 0.1f;

    /// <summary>Speed limit of a particle, m/s. Range 0.1 to 1000. Default 50.</summary>
    public float MaxSpeed { get; set; } = 50f;

    /// <summary>Scales gravity for this fluid. Default 1.</summary>
    public float GravityScale { get; set; } = 1f;

    /// <summary>Layer (0 to 31) looked up in the world's layer matrix against each shape's layer. Default 0.</summary>
    public int Layer { get; set; }

    /// <summary>Collision category bits of the fluid. Default 1.</summary>
    public uint CategoryBits { get; set; } = 1u;

    /// <summary>Which categories of shape the fluid collides with. Default all.</summary>
    public uint MaskBits { get; set; } = uint.MaxValue;

    /// <summary>Group index: equal non-zero groups collide when positive and never when negative. Default 0.</summary>
    public int GroupIndex { get; set; }

    /// <summary>Whether particles take part in contact with soft bodies and cloth. Default true.</summary>
    public bool CollideWithSoftBodies { get; set; } = true;

    /// <summary>Whether particles take part in contact with rods, ropes and hair. Default true.</summary>
    public bool CollideWithRods { get; set; } = true;

    /// <summary>Water: 1000 kg/m³, a little viscosity.</summary>
    public static ParticleFluidDef Water(float spacing = 0.05f) => new() { Spacing = spacing };

    /// <summary>Oil: 800 kg/m³, more viscous than water, a little cohesive.</summary>
    public static ParticleFluidDef Oil(float spacing = 0.05f) => new() { Spacing = spacing, RestDensity = 800f, Viscosity = 0.08f, TensileCorrection = 0.1f };

    /// <summary>Honey: 1400 kg/m³ and very viscous.</summary>
    public static ParticleFluidDef Honey(float spacing = 0.05f) => new() { Spacing = spacing, RestDensity = 1400f, Viscosity = 0.6f, TensileCorrection = 0.1f, Friction = 0.5f };

    /// <summary>An independent copy.</summary>
    public ParticleFluidDef Clone() => (ParticleFluidDef)MemberwiseClone();

    private static void Range(float v, string name, float lo, float hi)
    {
        if (!(v >= lo && v <= hi)) { throw new ArgumentOutOfRangeException(name, v, "ParticleFluidDef." + name + " is " + v.ToString("R", CultureInfo.InvariantCulture) + "; expected " + lo.ToString("R", CultureInfo.InvariantCulture) + " to " + hi.ToString("R", CultureInfo.InvariantCulture) + "."); }
    }

    /// <summary>Checks every member. Throws <see cref="ArgumentOutOfRangeException"/> naming the first one out of range.</summary>
    public void Validate()
    {
        Range(Spacing, nameof(Spacing), 0.001f, 10f);
        Range(RestDensity, nameof(RestDensity), 0.01f, 1e5f);
        Range(KernelRadiusScale, nameof(KernelRadiusScale), 1.2f, 4f);
        Range(Viscosity, nameof(Viscosity), 0f, 1f);
        Range(Vorticity, nameof(Vorticity), 0f, 100f);
        Range(TensileCorrection, nameof(TensileCorrection), 0f, 10f);
        Range(Relaxation, nameof(Relaxation), 1e-6f, 100f);
        Range(Friction, nameof(Friction), 0f, 100f);
        Range(MaxSpeed, nameof(MaxSpeed), 0.1f, 1000f);
        Range(GravityScale, nameof(GravityScale), -100f, 100f);
        if ((uint)Layer > 31u) { throw new ArgumentOutOfRangeException(nameof(Layer), Layer, "ParticleFluidDef.Layer is " + Layer.ToString(CultureInfo.InvariantCulture) + "; expected 0 to 31."); }
    }
}

/// <summary>World-wide settings of the particle fluids: how they are stepped, when they sleep and how many particles may exist. Read and written through the world's FluidSettings property; saved in the snapshot.</summary>
public struct FluidSettings
{
    /// <summary>The defaults: 2 substeps, 4 density iterations, 262144 particles, sleep after 1 s below 0.02 m/s.</summary>
    public static FluidSettings Default => new() { Substeps = 2, Iterations = 4, MaxParticles = 262144, SleepSpeed = 0.02f, TimeToSleep = 1f, WakeBodySpeed = 0.05f };

    /// <summary>Substeps per world step (1 to 16).</summary>
    public int Substeps { get; set; }

    /// <summary>Density-constraint iterations per substep (1 to 64).</summary>
    public int Iterations { get; set; }

    /// <summary>Budget: the most particles that may exist. Emitters stop at it; adding particles beyond it throws <see cref="FluidBudgetException"/>. 0 to 16777216.</summary>
    public int MaxParticles { get; set; }

    /// <summary>A fluid whose fastest particle stays below this speed (m/s) for <see cref="TimeToSleep"/> goes to sleep: it is not stepped until something touches it. 0 disables sleeping.</summary>
    public float SleepSpeed { get; set; }

    /// <summary>Seconds below <see cref="SleepSpeed"/> before the fluid sleeps.</summary>
    public float TimeToSleep { get; set; }

    /// <summary>A sleeping rigid body that the fluid pushes wakes only when one step changes its velocity by more than this (m/s); smaller pushes are dropped.</summary>
    public float WakeBodySpeed { get; set; }

    internal readonly void Validate()
    {
        if (Substeps < 1 || Substeps > 16) { throw new ArgumentOutOfRangeException(nameof(Substeps), Substeps, "FluidSettings.Substeps must be 1 to 16."); }
        if (Iterations < 1 || Iterations > 64) { throw new ArgumentOutOfRangeException(nameof(Iterations), Iterations, "FluidSettings.Iterations must be 1 to 64."); }
        if (MaxParticles < 0 || MaxParticles > 16777216) { throw new ArgumentOutOfRangeException(nameof(MaxParticles), MaxParticles, "FluidSettings.MaxParticles must be 0 to 16777216."); }
        if (!(SleepSpeed >= 0f && SleepSpeed <= 100f)) { throw new ArgumentOutOfRangeException(nameof(SleepSpeed), SleepSpeed, "FluidSettings.SleepSpeed must be 0 to 100."); }
        if (!(TimeToSleep >= 0f && TimeToSleep <= 1000f)) { throw new ArgumentOutOfRangeException(nameof(TimeToSleep), TimeToSleep, "FluidSettings.TimeToSleep must be 0 to 1000."); }
        if (!(WakeBodySpeed >= 0f && WakeBodySpeed <= 100f)) { throw new ArgumentOutOfRangeException(nameof(WakeBodySpeed), WakeBodySpeed, "FluidSettings.WakeBodySpeed must be 0 to 100."); }
    }
}

/// <summary>
/// A nozzle: emits layers of particles on the fluid's lattice through a disc or box facing <see cref="Direction"/>, one layer each time the flow has advanced one particle spacing, so the stream has the fluid's rest density.
/// The flow rate is speed × nozzle area ÷ spacing^dim particles per second. In a planar world the nozzle is a segment across the direction in the plane (z components are ignored).
/// </summary>
public sealed class FluidEmitterDef
{
    /// <summary>Nozzle centre, world coordinates. Finite within ±2^21.</summary>
    public Vector3D Position { get; set; }

    /// <summary>Direction of the flow (any non-zero length; normalised). Default +x.</summary>
    public Vector3F Direction { get; set; } = new(1f, 0f, 0f);

    /// <summary>Shape of the nozzle's cross-section: a box (half extents <see cref="Size"/> across the flow) or a disc/sphere (radius <see cref="Size"/>.X).</summary>
    public FluidVolumeKind Kind { get; set; } = FluidVolumeKind.Sphere;

    /// <summary>Half extents (box) or radius in X (disc), metres, positive.</summary>
    public Vector3F Size { get; set; } = new(0.1f, 0.1f, 0.1f);

    /// <summary>Exit speed, m/s. 0 stops the nozzle. Range 0 to 1000.</summary>
    public float Speed { get; set; } = 3f;

    /// <summary>Whether the nozzle emits now.</summary>
    public bool Enabled { get; set; } = true;

    /// <summary>The most particles this nozzle may emit over its life (0 = unlimited).</summary>
    public int MaxEmitted { get; set; }

    /// <summary>An independent copy.</summary>
    public FluidEmitterDef Clone() => (FluidEmitterDef)MemberwiseClone();

    internal void Validate()
    {
        Check(Position.X, nameof(Position), 2097152.0); Check(Position.Y, nameof(Position), 2097152.0); Check(Position.Z, nameof(Position), 2097152.0);
        float len2 = Direction.X * Direction.X + Direction.Y * Direction.Y + Direction.Z * Direction.Z;
        if (!float.IsFinite(len2) || !(len2 > 1e-12f)) { throw new ArgumentOutOfRangeException(nameof(Direction), Direction, "FluidEmitterDef.Direction must be finite and non-zero."); }
        if (!(Size.X > 0f && Size.X <= 1000f && Size.Y > 0f && Size.Y <= 1000f && Size.Z > 0f && Size.Z <= 1000f)) { throw new ArgumentOutOfRangeException(nameof(Size), Size, "FluidEmitterDef.Size must be positive and at most 1000."); }
        if (!(Speed >= 0f && Speed <= 1000f)) { throw new ArgumentOutOfRangeException(nameof(Speed), Speed, "FluidEmitterDef.Speed must be 0 to 1000."); }
        if (MaxEmitted < 0) { throw new ArgumentOutOfRangeException(nameof(MaxEmitted), MaxEmitted, "FluidEmitterDef.MaxEmitted must not be negative."); }
    }

    internal static void Check(double v, string name, double bound)
    {
        if (!double.IsFinite(v) || Math.Abs(v) > bound) { throw new ArgumentOutOfRangeException(name, v, name + " component " + v.ToString("R", CultureInfo.InvariantCulture) + " is not finite within ±" + bound.ToString("R", CultureInfo.InvariantCulture) + "."); }
    }
}

/// <summary>A kill volume: particles of its fluid (or any fluid) inside it are removed at the end of every step.</summary>
public sealed class FluidDrainDef
{
    /// <summary>Centre.</summary>
    public Vector3D Position { get; set; }

    /// <summary>Shape of the volume.</summary>
    public FluidVolumeKind Kind { get; set; } = FluidVolumeKind.Box;

    /// <summary>Half extents (box) or radius in X (sphere), metres, positive.</summary>
    public Vector3F Size { get; set; } = new(0.5f, 0.5f, 0.5f);

    /// <summary>Whether the drain removes now.</summary>
    public bool Enabled { get; set; } = true;

    /// <summary>An independent copy.</summary>
    public FluidDrainDef Clone() => (FluidDrainDef)MemberwiseClone();

    internal void Validate()
    {
        FluidEmitterDef.Check(Position.X, nameof(Position), 2097152.0); FluidEmitterDef.Check(Position.Y, nameof(Position), 2097152.0); FluidEmitterDef.Check(Position.Z, nameof(Position), 2097152.0);
        if (!(Size.X > 0f && Size.X <= 1e6f && Size.Y > 0f && Size.Y <= 1e6f && Size.Z > 0f && Size.Z <= 1e6f)) { throw new ArgumentOutOfRangeException(nameof(Size), Size, "FluidDrainDef.Size must be positive and at most 1e6."); }
    }
}

/// <summary>Adding particles would pass <see cref="FluidSettings.MaxParticles"/>.</summary>
public sealed class FluidBudgetException : InvalidOperationException
{
    /// <summary>Creates the exception.</summary>
    public FluidBudgetException(string message) : base(message) { }
}

/// <summary>What happened between a body and a particle fluid.</summary>
public enum FluidEventKind : byte
{
    /// <summary>A body that was dry is now wet: its submerged fraction rose above the enter threshold.</summary>
    Enter = 0,

    /// <summary>A body that was wet is dry again.</summary>
    Exit = 1,
}

/// <summary>A body entering or leaving a fluid, from <c>FluidEvents</c> after a step. Ordered by body slot, then fluid slot.</summary>
public readonly struct FluidEvent
{
    /// <summary>Creates an event.</summary>
    public FluidEvent(FluidEventKind kind, BodyHandle body, int fluid, float submergedFraction)
    {
        Kind = kind; Body = body; Fluid = fluid; SubmergedFraction = submergedFraction;
    }

    /// <summary>Enter or exit.</summary>
    public FluidEventKind Kind { get; }

    /// <summary>The body.</summary>
    public BodyHandle Body { get; }

    /// <summary>Slot of the fluid.</summary>
    public int Fluid { get; }

    /// <summary>The submerged fraction of the body's volume (area in 2D) when the event fired, 0 to 1.</summary>
    public float SubmergedFraction { get; }
}

/// <summary>How much of a body a fluid covers, and the force it puts on it.</summary>
public readonly struct FluidBodyState
{
    /// <summary>Creates the value.</summary>
    public FluidBodyState(float submergedFraction, float wettedArea, Vector3F force)
    {
        SubmergedFraction = submergedFraction; WettedArea = wettedArea; Force = force;
    }

    /// <summary>Fraction of the body's volume (area in 2D) inside the fluid, 0 to 1.</summary>
    public float SubmergedFraction { get; }

    /// <summary>Surface of the body in contact with fluid particles, m² (m in 2D): an estimate from the particles touching it.</summary>
    public float WettedArea { get; }

    /// <summary>The average force of the fluid on the body during the last step, newtons (a planar world: z is zero).</summary>
    public Vector3F Force { get; }
}

/// <summary>What the last fluid step did, for inspection and tests. Not simulation state.</summary>
public readonly struct FluidStepStatistics
{
    /// <summary>Creates the value.</summary>
    public FluidStepStatistics(bool stepped, bool asleep, int particles, int shapes, int deformables, int contacts, int coupledContacts, float maxSpeed)
    {
        Stepped = stepped; Asleep = asleep; Particles = particles; Shapes = shapes; Deformables = deformables; Contacts = contacts; CoupledContacts = coupledContacts; MaxSpeed = maxSpeed;
    }

    /// <summary>False when the fluids were asleep or absent and the step did no work.</summary>
    public bool Stepped { get; }

    /// <summary>Whether the fluids are asleep now.</summary>
    public bool Asleep { get; }

    /// <summary>Particles in the world.</summary>
    public int Particles { get; }

    /// <summary>Shapes near enough to the fluid to be tested against it.</summary>
    public int Shapes { get; }

    /// <summary>Soft-body particles and rod segments near the fluid.</summary>
    public int Deformables { get; }

    /// <summary>Particle contacts in the last substep.</summary>
    public int Contacts { get; }

    /// <summary>Of those, contacts that gave a reaction to a rigid body, soft body or rod.</summary>
    public int CoupledContacts { get; }

    /// <summary>Fastest particle after the step, m/s.</summary>
    public float MaxSpeed { get; }
}
