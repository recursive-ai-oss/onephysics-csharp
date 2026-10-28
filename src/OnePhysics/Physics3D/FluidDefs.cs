using System;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// The description of a body of fluid for a 3D world (middle tier): a flat surface, a density, a flow and drag coefficients. Pass it to <see cref="PhysicsWorld3D.AddFluid"/>;
/// the world copies it. Fluids exist in 3D worlds only.
/// </summary>
/// <remarks>
/// The fluid fills the half-space below its surface. Each awake dynamic body gets an upward force through the centroid of its submerged volume (Archimedes: density × volume × gravity,
/// scaled by <see cref="Body.BuoyancyFactor"/>) and quadratic drag against the fluid's flow. Spheres, boxes and hulls are cut exactly; cylinders, capsules and tapered capsules are cut as 24-sided polyhedra whose
/// volume is within about 1 % of the true one; compounds sum their children; triangle meshes, height fields and sensors do not float and are skipped.
/// </remarks>
public sealed class FluidDef
{
    /// <summary>Largest coordinate accepted for the surface and the region, 2^22 (the same bound as body positions).</summary>
    public const double MaxCoordinate = 4194304.0;

    /// <summary>Unit vector pointing out of the fluid, away from its depths. Default (0, 1, 0). Any finite non-zero vector is accepted and normalised.</summary>
    public Vector3F SurfaceNormal { get; set; } = new(0f, 1f, 0f);

    /// <summary>Where the surface is: the plane is <c>dot(SurfaceNormal, x) = SurfaceOffset</c> for world positions x, metres. For a level pool, the surface height. Default 0.</summary>
    public double SurfaceOffset { get; set; }

    /// <summary>Density of the fluid, kg/m³; finite, above 0 and at most 10^7. Default 1000 (fresh water).</summary>
    public float Density { get; set; } = 1000f;

    /// <summary>Velocity of the fluid in world axes, m/s; finite, each component within ±10^4. Default zero (still water).</summary>
    public Vector3F Flow { get; set; }

    /// <summary>Linear drag coefficient (dimensionless, the usual C<sub>d</sub>): the force is ½ · density · C<sub>d</sub> · A · |v| · v against the velocity relative to the flow, where A is the submerged area the body
    /// presents to that relative velocity. Finite, 0 to 1000. Default 0.5.</summary>
    public float LinearDrag { get; set; } = 0.5f;

    /// <summary>Angular drag coefficient: the torque is ½ · density · C · V<sup>5/3</sup> · |ω| · ω against the body's rotation (the fluid does not rotate), V being the submerged volume. Finite, 0 to 1000. Default 0.5.</summary>
    public float AngularDrag { get; set; } = 0.5f;

    /// <summary>Linear damping of the motion relative to the fluid, per second: the force is damping · density · V · (v − flow) and the torque damping · density · V<sup>5/3</sup> · ω, V being the submerged volume. It stands for viscosity and for the energy waves carry away, which the quadratic drag alone is too weak at small speeds to do, so floating bodies stop bobbing. Finite, 0 to 1000. Default 0; <see cref="PhysicsWorld3D.AddWater(double, float, Vector3F, float)"/> uses 1.</summary>
    public float ViscousDamping { get; set; }

    /// <summary>When true the fluid acts only on bodies whose bounds touch the box <see cref="RegionMin"/> to <see cref="RegionMax"/> (a pool); the surface plane still decides what is submerged. Default false.</summary>
    public bool LimitToRegion { get; set; }

    /// <summary>Lower corner of the region, world coordinates. Only used with <see cref="LimitToRegion"/>.</summary>
    public Vector3D RegionMin { get; set; }

    /// <summary>Upper corner of the region, world coordinates. Only used with <see cref="LimitToRegion"/>.</summary>
    public Vector3D RegionMax { get; set; }

    /// <summary>A copy of this definition.</summary>
    public FluidDef Clone() => (FluidDef)MemberwiseClone();

    /// <summary>Checks every field.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A field is NaN, infinite or outside its range; the message names the field and the value.</exception>
    public void Validate()
    {
        var n = SurfaceNormal;
        if (!float.IsFinite(n.X) || !float.IsFinite(n.Y) || !float.IsFinite(n.Z)) { throw Bad(nameof(SurfaceNormal), n, "must be finite"); }
        double len2 = (double)n.X * n.X + (double)n.Y * n.Y + (double)n.Z * n.Z;
        if (!(len2 > 1e-24) || len2 > 1e24) { throw Bad(nameof(SurfaceNormal), n, "must have a length between 1e-12 and 1e12"); }
        if (!double.IsFinite(SurfaceOffset) || Math.Abs(SurfaceOffset) > MaxCoordinate) { throw Bad(nameof(SurfaceOffset), SurfaceOffset, "must be finite and within ±" + MaxCoordinate.ToString("R", CultureInfo.InvariantCulture)); }
        if (!(Density > 0f && Density <= 1e7f)) { throw Bad(nameof(Density), Density, "must be above 0 and at most 10^7 kg/m³"); }
        var f = Flow;
        if (!(Math.Abs(f.X) <= 1e4f && Math.Abs(f.Y) <= 1e4f && Math.Abs(f.Z) <= 1e4f)) { throw Bad(nameof(Flow), f, "must be finite with each component within ±10^4 m/s"); }
        if (!(LinearDrag >= 0f && LinearDrag <= 1000f)) { throw Bad(nameof(LinearDrag), LinearDrag, "must be within 0 to 1000"); }
        if (!(AngularDrag >= 0f && AngularDrag <= 1000f)) { throw Bad(nameof(AngularDrag), AngularDrag, "must be within 0 to 1000"); }
        if (!(ViscousDamping >= 0f && ViscousDamping <= 1000f)) { throw Bad(nameof(ViscousDamping), ViscousDamping, "must be within 0 to 1000"); }
        if (LimitToRegion)
        {
            CheckCorner(nameof(RegionMin), RegionMin);
            CheckCorner(nameof(RegionMax), RegionMax);
            if (RegionMin.X > RegionMax.X || RegionMin.Y > RegionMax.Y || RegionMin.Z > RegionMax.Z) { throw Bad("Region", RegionMin, "has a minimum corner above the maximum corner " + RegionMax); }
        }
    }

    private static void CheckCorner(string name, Vector3D v)
    {
        if (!double.IsFinite(v.X) || !double.IsFinite(v.Y) || !double.IsFinite(v.Z) || Math.Abs(v.X) > MaxCoordinate || Math.Abs(v.Y) > MaxCoordinate || Math.Abs(v.Z) > MaxCoordinate)
        {
            throw Bad(name, v, "must be finite and within ±" + MaxCoordinate.ToString("R", CultureInfo.InvariantCulture));
        }
    }

    private static ArgumentOutOfRangeException Bad(string name, object value, string rule) =>
        new(name, value, "FluidDef." + name + " is " + Convert.ToString(value, CultureInfo.InvariantCulture) + " but " + rule + ".");
}

/// <summary>
/// A fluid in a 3D world: a small struct handle (world, slot, generation) made by <see cref="PhysicsWorld3D.AddWater(double, float, Vector3F, float)"/> or <see cref="PhysicsWorld3D.AddFluid"/>.
/// It allocates nothing and throws <see cref="StaleHandleException"/> once removed. Properties change the fluid from the next step and wake sleeping bodies.
/// </summary>
public readonly struct Fluid : IEquatable<Fluid>
{
    private readonly PhysicsWorld3D? _world;
    private readonly int _index;
    private readonly int _generation;

    internal Fluid(PhysicsWorld3D world, int index, int generation)
    {
        _world = world;
        _index = index;
        _generation = generation;
    }

    private FluidDef Resolve()
    {
        if (_world == null) { throw new InvalidHandleException("The default Fluid is not attached to a world."); }
        var e = _world.Engine;
        if (!e.FluidHandles.IsAlive(_index, _generation))
        {
            throw new StaleHandleException("Fluid(world " + _world.Id.ToString(CultureInfo.InvariantCulture) + ", #" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ") is stale: the fluid was removed.");
        }

        return e.Fluids[_index]!;
    }

    /// <summary>True while the fluid exists.</summary>
    public bool IsValid => _world != null && _world.Engine.FluidHandles.IsAlive(_index, _generation);

    /// <summary>A copy of the fluid's definition.</summary>
    /// <exception cref="StaleHandleException">The fluid was removed.</exception>
    public FluidDef Definition => Resolve().Clone();

    /// <summary>Replaces the definition (validated first; the fluid is unchanged if it throws) and wakes sleeping bodies.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A field is out of range.</exception>
    /// <exception cref="StaleHandleException">The fluid was removed.</exception>
    public void SetDefinition(FluidDef def)
    {
        ArgumentNullException.ThrowIfNull(def);
        Resolve();
        var prepared = Engine3D.Prepare(def);
        _world!.Engine.Fluids[_index] = prepared;
        _world.Engine.WakeAllDynamic();
    }

    /// <summary>The height of a level surface, metres (the surface offset). Setting it moves the surface and wakes sleeping bodies.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Setting a value that is not finite or beyond ±2^22.</exception>
    public double SurfaceOffset
    {
        get => Resolve().SurfaceOffset;
        set { var d = Definition; d.SurfaceOffset = value; SetDefinition(d); }
    }

    /// <summary>The fluid's velocity, m/s. Setting it wakes sleeping bodies.</summary>
    public Vector3F Flow
    {
        get => Resolve().Flow;
        set { var d = Definition; d.Flow = value; SetDefinition(d); }
    }

    /// <summary>Removes the fluid and wakes sleeping bodies.</summary>
    /// <exception cref="StaleHandleException">The fluid was already removed.</exception>
    public void Remove()
    {
        Resolve();
        var e = _world!.Engine;
        e.Fluids[_index] = null;
        e.FluidHandles.Free(_index);
        e.WakeAllDynamic();
    }

    /// <inheritdoc/>
    public bool Equals(Fluid other) => ReferenceEquals(_world, other._world) && _index == other._index && _generation == other._generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Fluid f && Equals(f);

    /// <inheritdoc/>
    public override int GetHashCode() => (_index * 397) ^ _generation;

    /// <summary>Equality of handles.</summary>
    public static bool operator ==(Fluid a, Fluid b) => a.Equals(b);

    /// <summary>Inequality of handles.</summary>
    public static bool operator !=(Fluid a, Fluid b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _world == null ? "Fluid(default)" : "Fluid(#" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ")";
}

/// <summary>What a fluid surface does to one shape: the part of the volume below the surface, where its centre is, and the area it presents to a flow.</summary>
public readonly struct SubmergedVolume3D
{
    internal SubmergedVolume3D(float volume, Vector3F centroid, float area)
    {
        Volume = volume;
        Centroid = centroid;
        ProjectedArea = area;
    }

    /// <summary>Volume below the surface, m³; 0 when the shape is clear of the fluid.</summary>
    public float Volume { get; }

    /// <summary>Centroid of the submerged volume in the frame the query was given in (the shape pose's frame).</summary>
    public Vector3F Centroid { get; }

    /// <summary>Area of the submerged part's silhouette seen along the flow direction asked for, m²; 0 if no direction was given.</summary>
    public float ProjectedArea { get; }
}
