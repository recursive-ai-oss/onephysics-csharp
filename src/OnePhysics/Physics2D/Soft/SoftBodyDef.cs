using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>The presets of the happy path: starting values for <see cref="SoftBodyDef"/> that behave like the named material at metre scale.</summary>
public enum SoftMaterial
{
    /// <summary>A stiff elastic area or a pressurised skin that springs back fast.</summary>
    Rubber = 1,

    /// <summary>A wobbly area.</summary>
    Jelly = 2,
}

/// <summary>Which constraints hold a 2D soft body together.</summary>
public enum SoftBodyModel
{
    /// <summary>A network of constraints: a distance constraint on every edge, an area constraint on every triangle, bending along an outline, and pressure on a closed outline (mass-spring and finite-element-like bodies).</summary>
    Network = 0,

    /// <summary>Shape matching (Müller et al. 2005): each cluster of particles is pulled towards the rigid, or quadratically deformed, copy of its rest shape that best fits it. No edges are needed; the mesh only names the particles and their rest positions.</summary>
    ShapeMatching = 1,

    /// <summary>The network and shape matching together: the network gives local stiffness, the clusters give the body its memory of the rest shape.</summary>
    Both = 2,
}

/// <summary>
/// The description of a soft body for a 2D world (middle tier). Pass it to <see cref="PhysicsWorld2D.AddSoftBody(SoftBodyDef)"/>; the world copies it. The presets <see cref="Rubber"/>, <see cref="Jelly"/>, <see cref="Balloon"/>
/// and <see cref="Blob"/> give sensible starting values for a metre-scale body.
/// </summary>
/// <remarks>
/// A soft body is a cloud of particles (one per mesh vertex) held together by extended position-based dynamics constraints (Müller et al. 2007; Macklin, Müller, Chentanez 2016; Macklin et al. 2019):
/// <list type="bullet">
/// <item><description>an <b>edge</b> constraint on every edge keeps its length (stretch);</description></item>
/// <item><description>an <b>area</b> constraint on every triangle keeps its area (shear and compression); an <see cref="InternalPressure"/> constraint on a closed outline keeps the enclosed area;</description></item>
/// <item><description>a <b>bend</b> constraint keeps the distance between the neighbours of every outline vertex of a mesh without triangles (a skin), which resists folding;</description></item>
/// <item><description><b>shape matching</b> (see <see cref="Model"/>) pulls clusters of particles towards their best-fitting rest shape.</description></item>
/// </list>
/// Each has a compliance (0 is as stiff as the solver can make it) and a damping time in seconds. Particles are discs of <see cref="ParticleRadius"/> that collide with every
/// shape in the world, with each other (bodies with compatible filters) and, if <see cref="SelfCollision"/> is on, with the rest of their own body.
/// Rigid bodies feel the contacts too: the body is pushed with the same impulse the particle is pushed with, so momentum is conserved.
/// </remarks>
public sealed class SoftBodyDef
{
    /// <summary>Largest compliance accepted.</summary>
    public const float MaxCompliance = 1e6f;

    /// <summary>Largest coordinate accepted for the position, 2^21 metres.</summary>
    public const double MaxCoordinate = 2097152.0;

    /// <summary>The rest shape. Required.</summary>
    public SoftBodyMesh? Mesh { get; set; }

    /// <summary>Where the mesh origin goes, world coordinates, within ±2^21. Default the origin.</summary>
    public Vector2D Position { get; set; }

    /// <summary>Orientation of the mesh, radians counter-clockwise. Finite. Default 0.</summary>
    public float Angle { get; set; }

    /// <summary>Uniform scale applied to the mesh, finite and above 0. Default 1. Rest lengths and areas are those of the scaled mesh.</summary>
    public float Scale { get; set; } = 1f;

    /// <summary>Initial velocity of the body's centre, m/s. Default zero.</summary>
    public Vector2F LinearVelocity { get; set; }

    /// <summary>Initial angular velocity about the mesh origin, radians per second. Default zero.</summary>
    public float AngularVelocity { get; set; }

    /// <summary>Total mass in kilograms, shared among the particles in proportion to the area around each. 0 (default) means 1000 kg per square metre of rest area (a unit-thickness slab of water-like material);
    /// for a mesh without triangles (a skin) it means 10 kg per square metre of enclosed area, carried by the outline. Otherwise finite, above 0 and at most 10^9.</summary>
    public float Mass { get; set; }

    /// <summary>Radius of the disc each particle collides with, metres. 0 (default) uses a quarter of the mean edge length. Otherwise finite, above 0, at most 100.</summary>
    public float ParticleRadius { get; set; }

    /// <summary>Which constraints hold the body together. Default <see cref="SoftBodyModel.Network"/>.</summary>
    public SoftBodyModel Model { get; set; }

    /// <summary>Compliance of the edge (stretch) constraints, m/N. 0 is stiffest. Default 0 (inextensible).</summary>
    public float EdgeCompliance { get; set; }

    /// <summary>Damping time of the edge constraints, seconds: the constraint's own relative velocity decays with this time constant. Finite, 0 to 1. Default 0.</summary>
    public float EdgeDamping { get; set; }

    /// <summary>Compliance of the bend constraints, m/N. Default 1e-3. Bend constraints exist only for a mesh without triangles (see <see cref="SoftBodyMesh.FromOutline"/>).</summary>
    public float BendCompliance { get; set; } = 1e-3f;

    /// <summary>Damping time of the bend constraints, seconds. Default 0.</summary>
    public float BendDamping { get; set; }

    /// <summary>Compliance of the area constraints (per triangle, or of the pressure constraint); the constraint is on the area in square metres. Default 0.</summary>
    public float AreaCompliance { get; set; }

    /// <summary>Damping time of the area constraints, seconds. Default 0.</summary>
    public float AreaDamping { get; set; }

    /// <summary>For a mesh with a closed outline: the enclosed area the body tries to hold, as a multiple of its rest area (1 keeps it, 1.5 inflates it). 0 (default) turns the pressure constraint off. Needs <see cref="SoftBodyMesh.IsClosed"/>.</summary>
    public float InternalPressure { get; set; }

    /// <summary>Compliance of shape matching, in seconds squared: the clusters pull the particles towards their goal like a mass-proportional spring of angular frequency 1/√compliance. 0 (default) snaps to the goal every pass. Finite, 0 to 10^6.</summary>
    public float MatchCompliance { get; set; }

    /// <summary>True to let clusters deform linearly (stretch, shear) and quadratically (bend, twist) before the goal pulls back, instead of matching a rigidly rotated rest shape (Müller et al. 2005, linear and quadratic deformations). Default false.</summary>
    public bool MatchQuadratic { get; set; }

    /// <summary>With <see cref="MatchQuadratic"/>: how much of the fitted deformation the goal keeps, 0 (the rigid goal; stiffest) to 1 (the deformed fit itself; no restoring force). Default 0.5.</summary>
    public float MatchBlend { get; set; } = 0.5f;

    /// <summary>Particles per cluster for shape matching; clusters are compact cells of the rest shape grown by a margin so that neighbours overlap. 0 (default) puts a body of up to 64 particles in one cluster and cuts a larger one into clusters of about 32 (at most 256 clusters).</summary>
    public int ClusterSize { get; set; }

    /// <summary>Coulomb friction coefficient against shapes; combined with the shape's by geometric mean. Finite, 0 to 100. Default 0.4.</summary>
    public float Friction { get; set; } = 0.4f;

    /// <summary>Velocity damping, 1/s: each substep multiplies velocities by 1 / (1 + damping · dt). Finite, 0 to 1000. Default 0.1.</summary>
    public float LinearDamping { get; set; } = 0.1f;

    /// <summary>Multiplier on gravity. Finite. Default 1.</summary>
    public float GravityScale { get; set; } = 1f;

    /// <summary>Whether particles of this body collide with each other (except neighbours joined by an edge). Costs more. Default false.</summary>
    public bool SelfCollision { get; set; }

    /// <summary>Whether the body may fall asleep when every particle has been slow for <see cref="WorldOptions2D.TimeToSleep"/>. Default true.</summary>
    public bool AllowSleep { get; set; } = true;

    /// <summary>Whether the boundary of the body (the outline segments, or the edges that belong to one triangle) is a surface that particles of other bodies cannot cross, and, with <see cref="SelfCollision"/>, particles of this body. Without it only the particle discs collide, and a body is porous between its boundary particles. Costs a search each step while two awake bodies with surface collision exist or a body has <see cref="SelfCollision"/>. Default true.</summary>
    public bool SurfaceCollision { get; set; } = true;

    /// <summary>Tearing by strain: an edge whose length exceeds its rest length by this fraction at the end of a step breaks (see <see cref="SoftBody.BrokenEdgeCount"/>). The strain is measured after the solve, so edges with zero <see cref="EdgeCompliance"/> never reach it; use <see cref="TearForce"/> for those. 0 (default) turns it off. Finite, 0 to 100.</summary>
    public float TearStrain { get; set; }

    /// <summary>Tearing by force: an edge breaks when the constraint force of the last substep of a step, the XPBD multiplier over the squared substep, exceeds this many newtons. 0 (default) turns it off. Finite, 0 to 10^12.</summary>
    public float TearForce { get; set; }

    /// <summary>Plastic yield strain: past this strain (edge length against rest length, triangle area against rest area, or shape-matching offsets against rest offsets) the rest shape follows the deformation (Müller et al. 2004; O'Brien et al. 2002). 0 (default) turns plasticity off. Finite, 0 to 10.</summary>
    public float PlasticYield { get; set; }

    /// <summary>Plastic creep: the fraction of the strain beyond the yield strain that the rest shape takes up each step. 0 to 1, default 0.5.</summary>
    public float PlasticCreep { get; set; } = 0.5f;

    /// <summary>Largest plastic strain: the rest length (area, offset) never leaves 1 ± this fraction of its original value. 0 to 10, default 0.5.</summary>
    public float PlasticMax { get; set; } = 0.5f;


    /// <summary>Fabric behaviour of a strip or membrane: direction-dependent stretch, strain limit and wind (see <see cref="StripParams"/>). Needs a triangle mesh. Default none.</summary>
    public StripParams? Strip { get; set; }

    /// <summary>Indices of particles held where they are (infinite mass) until released; see <see cref="SoftBody.SetPinned"/>. Default none. Indices must be valid and are de-duplicated.</summary>
    public IReadOnlyList<int>? PinnedParticles { get; set; }

    /// <summary>Collision category bits of the body (what it is). Default 1.</summary>
    public uint CategoryBits { get; set; } = 1u;

    /// <summary>Collision mask bits (what it collides with). Default all bits.</summary>
    public uint MaskBits { get; set; } = uint.MaxValue;

    /// <summary>Collision group: bodies and shapes with the same non-zero group always collide when it is positive and never when it is negative. Default 0.</summary>
    public int GroupIndex { get; set; }

    /// <summary>Layer (0 to 31) looked up in the world's layer matrix. Default 0.</summary>
    public int Layer { get; set; }

    /// <summary>A copy of this definition (the mesh is shared; it is immutable).</summary>
    public SoftBodyDef Clone()
    {
        var c = (SoftBodyDef)MemberwiseClone();
        c.PinnedParticles = PinnedParticles == null ? null : new List<int>(PinnedParticles);
        c.Strip = Strip?.Clone();
        return c;
    }

    /// <summary>Rubber: a stiff elastic area that springs back quickly.</summary>
    public static SoftBodyDef Rubber(SoftBodyMesh mesh, Vector2D at = default) => new()
    {
        Mesh = mesh, Position = at, EdgeCompliance = 2e-5f, EdgeDamping = 0.002f, AreaCompliance = 1e-7f, AreaDamping = 0.002f, BendCompliance = 1e-4f, Friction = 0.8f, LinearDamping = 0.1f,
    };

    /// <summary>Jelly: a wobbly area.</summary>
    public static SoftBodyDef Jelly(SoftBodyMesh mesh, Vector2D at = default) => new()
    {
        Mesh = mesh, Position = at, EdgeCompliance = 3e-4f, EdgeDamping = 0.003f, AreaCompliance = 2e-6f, AreaDamping = 0.003f, BendCompliance = 1e-3f, Friction = 0.6f, LinearDamping = 0.2f,
    };

    /// <summary>A balloon: a closed skin (a mesh with a closed outline) that holds its enclosed area against squeezing, like a pressurised ball. Without a <see cref="Mass"/> it weighs 10 kg per square metre of enclosed area.</summary>
    public static SoftBodyDef Balloon(SoftBodyMesh mesh, Vector2D at = default) => new()
    {
        Mesh = mesh, Position = at, EdgeCompliance = 2e-5f, EdgeDamping = 0.002f, AreaCompliance = 1e-7f, AreaDamping = 0.002f, BendCompliance = 1e-4f, InternalPressure = 1f, Friction = 0.8f, LinearDamping = 0.1f,
    };

    /// <summary>A blob: shape matching with linear deformation only, stiff but able to roll and squash a little. Use <see cref="MatchQuadratic"/> for bendy bodies.</summary>
    public static SoftBodyDef Blob(SoftBodyMesh mesh, Vector2D at = default) => new()
    {
        Mesh = mesh, Position = at, Model = SoftBodyModel.ShapeMatching, MatchCompliance = 1e-3f, Friction = 0.6f, LinearDamping = 0.2f,
    };

    /// <summary>Checks every field.</summary>
    /// <exception cref="SoftBodyException">A field is NaN, infinite or outside its range; the message names the field and the value.</exception>
    public void Validate()
    {
        if (Mesh == null) { throw new SoftBodyException(SoftBodyError.InvalidDefinition, "SoftBodyDef.Mesh is null; give the body a mesh.", nameof(Mesh)); }
        var p = Position;
        if (!double.IsFinite(p.X) || !double.IsFinite(p.Y))
        {
            throw new SoftBodyException(SoftBodyError.NonFinite, "SoftBodyDef.Position is " + p + "; coordinates must be finite.", nameof(Position));
        }

        if (Math.Abs(p.X) > MaxCoordinate || Math.Abs(p.Y) > MaxCoordinate)
        {
            throw new SoftBodyException(SoftBodyError.OutOfRange, "SoftBodyDef.Position is " + p + "; coordinates must be within ±" + MaxCoordinate.ToString("R", CultureInfo.InvariantCulture) + ".", nameof(Position));
        }

        if (!float.IsFinite(Angle)) { throw new SoftBodyException(SoftBodyError.NonFinite, "SoftBodyDef.Angle is " + Angle.ToString("R", CultureInfo.InvariantCulture) + "; it must be finite.", nameof(Angle)); }
        Range(Scale, nameof(Scale), 1e-6f, 1e6f);
        var v = LinearVelocity;
        if (!float.IsFinite(v.X) || !float.IsFinite(v.Y) || Math.Abs(v.X) > 1e4f || Math.Abs(v.Y) > 1e4f)
        {
            throw new SoftBodyException(SoftBodyError.OutOfRange, "SoftBodyDef.LinearVelocity is " + v + "; components must be finite and within ±10000.", nameof(LinearVelocity));
        }

        Range(AngularVelocity, nameof(AngularVelocity), -1e4f, 1e4f);
        if (Mass != 0f) { Range(Mass, nameof(Mass), float.Epsilon, 1e9f); }
        if (ParticleRadius != 0f) { Range(ParticleRadius, nameof(ParticleRadius), float.Epsilon, 100f); }
        if (Model != SoftBodyModel.Network && Model != SoftBodyModel.ShapeMatching && Model != SoftBodyModel.Both)
        {
            throw new SoftBodyException(SoftBodyError.OutOfRange, "SoftBodyDef.Model is " + ((int)Model).ToString(CultureInfo.InvariantCulture) + "; expected a SoftBodyModel value.", nameof(Model));
        }

        Range(EdgeCompliance, nameof(EdgeCompliance), 0f, MaxCompliance);
        Range(BendCompliance, nameof(BendCompliance), 0f, MaxCompliance);
        Range(AreaCompliance, nameof(AreaCompliance), 0f, MaxCompliance);
        Range(MatchCompliance, nameof(MatchCompliance), 0f, MaxCompliance);
        Range(MatchBlend, nameof(MatchBlend), 0f, 1f);
        Range(EdgeDamping, nameof(EdgeDamping), 0f, 1f);
        Range(BendDamping, nameof(BendDamping), 0f, 1f);
        Range(AreaDamping, nameof(AreaDamping), 0f, 1f);
        Range(InternalPressure, nameof(InternalPressure), 0f, 100f);
        Range(TearStrain, nameof(TearStrain), 0f, 100f);
        Range(TearForce, nameof(TearForce), 0f, 1e12f);
        Range(PlasticYield, nameof(PlasticYield), 0f, 10f);
        Range(PlasticCreep, nameof(PlasticCreep), 0f, 1f);
        Range(PlasticMax, nameof(PlasticMax), 0f, 10f);
        Range(Friction, nameof(Friction), 0f, 100f);
        Range(LinearDamping, nameof(LinearDamping), 0f, 1000f);
        Range(GravityScale, nameof(GravityScale), -1e6f, 1e6f);
        if (ClusterSize < 0 || ClusterSize > 1 << 20) { throw new SoftBodyException(SoftBodyError.OutOfRange, "SoftBodyDef.ClusterSize is " + ClusterSize.ToString(CultureInfo.InvariantCulture) + "; expected 0 to 1048576.", nameof(ClusterSize)); }
        if ((uint)Layer > 31u) { throw new SoftBodyException(SoftBodyError.OutOfRange, "SoftBodyDef.Layer is " + Layer.ToString(CultureInfo.InvariantCulture) + "; expected 0 to 31.", nameof(Layer)); }
        if (InternalPressure != 0f && !Mesh.IsClosed)
        {
            throw new SoftBodyException(SoftBodyError.NotClosed, "SoftBodyDef.InternalPressure is " + InternalPressure.ToString("R", CultureInfo.InvariantCulture) + " but the mesh has no single closed outline.", nameof(InternalPressure));
        }

        if (Strip != null)
        {
            if (Mesh.TriangleCount == 0) { throw new SoftBodyException(SoftBodyError.InvalidDefinition, "SoftBodyDef.Strip is set on a mesh without triangles; a strip needs a triangle mesh.", nameof(Strip)); }
            Strip.Validate();
        }

        if (PinnedParticles != null)
        {
            for (int i = 0; i < PinnedParticles.Count; i++)
            {
                int k = PinnedParticles[i];
                if ((uint)k >= (uint)Mesh.VertexCount) { throw new SoftBodyException(SoftBodyError.BadIndex, "SoftBodyDef.PinnedParticles[" + i.ToString(CultureInfo.InvariantCulture) + "] is " + k.ToString(CultureInfo.InvariantCulture) + "; particle indices must be in 0 to " + (Mesh.VertexCount - 1).ToString(CultureInfo.InvariantCulture) + ".", nameof(PinnedParticles)); }
            }
        }
    }

    private static void Range(float v, string name, float lo, float hi)
    {
        if (!float.IsFinite(v)) { throw new SoftBodyException(SoftBodyError.NonFinite, "SoftBodyDef." + name + " is " + v.ToString("R", CultureInfo.InvariantCulture) + "; it must be finite.", name); }
        if (v < lo || v > hi) { throw new SoftBodyException(SoftBodyError.OutOfRange, "SoftBodyDef." + name + " is " + v.ToString("R", CultureInfo.InvariantCulture) + "; expected " + lo.ToString("R", CultureInfo.InvariantCulture) + " to " + hi.ToString("R", CultureInfo.InvariantCulture) + ".", name); }
    }
}
