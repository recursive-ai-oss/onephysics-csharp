using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// The description of a soft body for a 3D world (middle tier). Pass it to <see cref="PhysicsWorld3D.AddSoftBody(SoftBodyDef)"/>; the world copies it. The presets <see cref="Cloth"/>, <see cref="Rubber"/> and <see cref="Jelly"/>
/// give sensible starting values for a metre-scale body.
/// </summary>
/// <remarks>
/// A soft body is a cloud of particles (one per mesh vertex) held together by extended position-based dynamics constraints (Müller et al. 2007; Macklin, Müller, Chentanez 2016; Macklin et al. 2019):
/// <list type="bullet">
/// <item><description>an <b>edge</b> constraint on every edge of the mesh keeps its length (stretch);</description></item>
/// <item><description>a <b>bend</b> constraint on every pair of triangles that share an edge keeps the distance between their far corners, which resists folding;</description></item>
/// <item><description>a <b>volume</b> constraint on every tetrahedron keeps its volume, and an <see cref="InternalPressure"/> constraint on a closed surface keeps the enclosed volume.</description></item>
/// </list>
/// Each has a compliance in metres per newton (0 is as stiff as the solver can make it) and a damping time in seconds. Particles are spheres of <see cref="ParticleRadius"/> that collide with every
/// shape in the world, with each other (bodies with compatible filters) and, if <see cref="SelfCollision"/> is on, with the rest of their own body.
/// Rigid bodies feel the contacts too: the body is pushed with the same impulse the particle is pushed with, so momentum is conserved.
/// </remarks>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public sealed class SoftBodyDef
{
    /// <summary>Largest compliance accepted, m/N.</summary>
    public const float MaxCompliance = 1e6f;

    /// <summary>Largest coordinate accepted for the position, 2^21 metres.</summary>
    public const double MaxCoordinate = 2097152.0;

    /// <summary>The rest shape. Required.</summary>
    public SoftBodyMesh? Mesh { get; set; }

    /// <summary>Where the mesh origin goes, world coordinates, within ±2^21. Default the origin.</summary>
    public Vector3D Position { get; set; }

    /// <summary>Orientation of the mesh; any finite non-zero quaternion (normalised on creation). Default identity.</summary>
    public QuaternionF Rotation { get; set; } = QuaternionF.Identity;

    /// <summary>Uniform scale applied to the mesh, finite and above 0. Default 1. Rest lengths and volumes are those of the scaled mesh.</summary>
    public float Scale { get; set; } = 1f;

    /// <summary>Initial velocity of every particle, m/s. Default zero.</summary>
    public Vector3F LinearVelocity { get; set; }

    /// <summary>Total mass in kilograms, shared among the particles in proportion to the area (triangles) or volume (tetrahedra) around each. 0 (default) chooses by kind: 0.3 kg/m² for a cloth,
    /// 1000 kg/m³ for a volume or closed surface. Otherwise finite, above 0 and at most 10^9.</summary>
    public float Mass { get; set; }

    /// <summary>Radius of the sphere each particle collides with, metres: for a cloth, half its thickness. 0 (default) uses a quarter of the mean edge length. Otherwise finite, above 0, at most 100.</summary>
    public float ParticleRadius { get; set; }

    /// <summary>Compliance of the edge (stretch) constraints, m/N. 0 is stiffest. Default 0 (inextensible).</summary>
    public float EdgeCompliance { get; set; }

    /// <summary>Damping time of the edge constraints, seconds: the constraint's own relative velocity decays with this time constant. Finite, 0 to 1. Default 0.</summary>
    public float EdgeDamping { get; set; }

    /// <summary>Compliance of the bend constraints, m/N. Default 1e-3. Only triangles that share an edge are bent against each other; a tetrahedral mesh has none.</summary>
    public float BendCompliance { get; set; } = 1e-3f;

    /// <summary>Damping time of the bend constraints, seconds. Default 0.</summary>
    public float BendDamping { get; set; }

    /// <summary>Compliance of the volume constraints (per tetrahedron, or of the pressure constraint), m³/N-ish: the constraint is on the volume in cubic metres. Default 0.</summary>
    public float VolumeCompliance { get; set; }

    /// <summary>Damping time of the volume constraints, seconds. Default 0.</summary>
    public float VolumeDamping { get; set; }

    /// <summary>For a closed triangle mesh: the enclosed volume the body tries to hold, as a multiple of its rest volume (1 keeps it, 1.5 inflates it). 0 (default) turns the pressure constraint off. Needs <see cref="SoftBodyMesh.IsClosed"/>.</summary>
    public float InternalPressure { get; set; }

    /// <summary>Coulomb friction coefficient against shapes; combined with the shape's by geometric mean. Finite, 0 to 100. Default 0.4.</summary>
    public float Friction { get; set; } = 0.4f;

    /// <summary>Velocity damping, 1/s: each substep multiplies velocities by 1 / (1 + damping · dt). Finite, 0 to 1000. Default 0.1.</summary>
    public float LinearDamping { get; set; } = 0.1f;

    /// <summary>Multiplier on gravity. Finite. Default 1.</summary>
    public float GravityScale { get; set; } = 1f;

    /// <summary>Whether particles of this body collide with each other (except neighbours joined by an edge). Costs more. Default false.</summary>
    public bool SelfCollision { get; set; }

    /// <summary>Whether the body may fall asleep when every particle has been slow for <see cref="WorldOptions3D.TimeToSleep"/>. Default true.</summary>
    public bool AllowSleep { get; set; } = true;

    /// <summary>Indices of particles held where they are (infinite mass) until released; see <see cref="SoftBody.SetPinned"/>. Default none. Indices must be valid and are de-duplicated.</summary>
    public IReadOnlyList<int>? PinnedParticles { get; set; }

    /// <summary>Skinning to a skeleton: per-particle joint weights, maximum distance and back-stop (see <see cref="SoftSkin"/>). Default none. The skin must have been made for this mesh (same particle count). A skinned body starts in its bind pose at
    /// <see cref="Position"/> and <see cref="Rotation"/>; drive it with <see cref="SoftBody.SetSkeletonPose"/> or <see cref="SoftBody.SetSkeletonTransforms"/>. Particles with a maximum distance of 0 are kinematic.</summary>
    public SoftSkin? Skin { get; set; }

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
        return c;
    }

    /// <summary>A cloth: thin, inextensible, bends easily (mass 0.3 kg/m², 1 cm-scale particles, friction 0.5, self-collision on).</summary>
    public static SoftBodyDef Cloth(SoftBodyMesh mesh, Vector3D at = default) => new()
    {
        Mesh = mesh, Position = at, EdgeCompliance = 0f, EdgeDamping = 0.03f, BendCompliance = 2e-3f, BendDamping = 0.03f, Friction = 0.5f, LinearDamping = 0.2f, SelfCollision = true,
    };

    /// <summary>Rubber: a stiff elastic volume that springs back quickly.</summary>
    public static SoftBodyDef Rubber(SoftBodyMesh mesh, Vector3D at = default) => new()
    {
        Mesh = mesh, Position = at, EdgeCompliance = 2e-5f, EdgeDamping = 0.002f, VolumeCompliance = 1e-7f, VolumeDamping = 0.002f, BendCompliance = 1e-4f, Friction = 0.8f, LinearDamping = 0.1f,
    };

    /// <summary>Jelly: a wobbly volume.</summary>
    public static SoftBodyDef Jelly(SoftBodyMesh mesh, Vector3D at = default) => new()
    {
        Mesh = mesh, Position = at, EdgeCompliance = 3e-4f, EdgeDamping = 0.003f, VolumeCompliance = 2e-6f, VolumeDamping = 0.003f, BendCompliance = 1e-3f, Friction = 0.6f, LinearDamping = 0.2f,
    };

    /// <summary>Checks every field.</summary>
    /// <exception cref="SoftBodyException">A field is NaN, infinite or outside its range; the message names the field and the value.</exception>
    public void Validate()
    {
        if (Mesh == null) { throw new SoftBodyException(SoftBodyError.InvalidDefinition, "SoftBodyDef.Mesh is null; give the body a mesh.", nameof(Mesh)); }
        var p = Position;
        if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !double.IsFinite(p.Z))
        {
            throw new SoftBodyException(SoftBodyError.NonFinite, "SoftBodyDef.Position is " + p + "; coordinates must be finite.", nameof(Position));
        }

        if (Math.Abs(p.X) > MaxCoordinate || Math.Abs(p.Y) > MaxCoordinate || Math.Abs(p.Z) > MaxCoordinate)
        {
            throw new SoftBodyException(SoftBodyError.OutOfRange, "SoftBodyDef.Position is " + p + "; coordinates must be finite and within ±" + MaxCoordinate.ToString("R", CultureInfo.InvariantCulture) + ".", nameof(Position));
        }

        var q = Rotation;
        float qn = q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W;
        if (!float.IsFinite(qn) || !(qn > 1e-12f)) { throw new SoftBodyException(SoftBodyError.InvalidDefinition, "SoftBodyDef.Rotation is not a usable quaternion (zero or not finite).", nameof(Rotation)); }
        Range(Scale, nameof(Scale), 1e-6f, 1e6f);
        var v = LinearVelocity;
        if (!float.IsFinite(v.X) || !float.IsFinite(v.Y) || !float.IsFinite(v.Z) || Math.Abs(v.X) > 1e4f || Math.Abs(v.Y) > 1e4f || Math.Abs(v.Z) > 1e4f)
        {
            throw new SoftBodyException(SoftBodyError.OutOfRange, "SoftBodyDef.LinearVelocity is " + v + "; components must be finite and within ±10000.", nameof(LinearVelocity));
        }

        if (Mass != 0f) { Range(Mass, nameof(Mass), float.Epsilon, 1e9f); }
        if (ParticleRadius != 0f) { Range(ParticleRadius, nameof(ParticleRadius), float.Epsilon, 100f); }
        Range(EdgeCompliance, nameof(EdgeCompliance), 0f, MaxCompliance);
        Range(BendCompliance, nameof(BendCompliance), 0f, MaxCompliance);
        Range(VolumeCompliance, nameof(VolumeCompliance), 0f, MaxCompliance);
        Range(EdgeDamping, nameof(EdgeDamping), 0f, 1f);
        Range(BendDamping, nameof(BendDamping), 0f, 1f);
        Range(VolumeDamping, nameof(VolumeDamping), 0f, 1f);
        Range(InternalPressure, nameof(InternalPressure), 0f, 100f);
        Range(Friction, nameof(Friction), 0f, 100f);
        Range(LinearDamping, nameof(LinearDamping), 0f, 1000f);
        Range(GravityScale, nameof(GravityScale), -1e6f, 1e6f);
        if ((uint)Layer > 31u) { throw new SoftBodyException(SoftBodyError.OutOfRange, "SoftBodyDef.Layer is " + Layer.ToString(CultureInfo.InvariantCulture) + "; expected 0 to 31.", nameof(Layer)); }
        if (InternalPressure != 0f && !Mesh.IsClosed)
        {
            throw new SoftBodyException(SoftBodyError.NotClosed, "SoftBodyDef.InternalPressure is " + InternalPressure.ToString("R", CultureInfo.InvariantCulture) + " but the mesh is not a closed, consistently oriented triangle surface.", nameof(InternalPressure));
        }

        if (Skin != null && Skin.ParticleCount != Mesh.VertexCount)
        {
            throw new SoftBodyException(SoftBodyError.BadCount, "SoftBodyDef.Skin was made for " + Skin.ParticleCount.ToString(CultureInfo.InvariantCulture) + " particles; the mesh has " + Mesh.VertexCount.ToString(CultureInfo.InvariantCulture) + ".", nameof(Skin));
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
