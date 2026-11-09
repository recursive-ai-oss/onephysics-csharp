using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// The physical constants of a fabric. The presets are starting values for a metre-scale piece of cloth (the stretch compliances are the inverse of the membrane stiffness in newtons per metre of a square weave:
/// one edge of a grid with spacing h is a spring of stiffness about E·t).
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public sealed class ClothMaterial
{
    /// <summary>Mass per square metre, kg/m². Finite, above 0.</summary>
    public float ArealDensity { get; set; } = 0.15f;

    /// <summary>Stretch compliance along the warp threads, m/N.</summary>
    public float WarpCompliance { get; set; } = 3e-3f;

    /// <summary>Stretch compliance along the weft threads, m/N.</summary>
    public float WeftCompliance { get; set; } = 5e-3f;

    /// <summary>Compliance of the shear (diagonal) edges, m/N.</summary>
    public float ShearCompliance { get; set; } = 5e-2f;

    /// <summary>Bend compliance of the isometric (or dihedral) bend constraints.</summary>
    public float BendCompliance { get; set; } = 2e-3f;

    /// <summary>Strain limit (see <see cref="ClothParams.StrainLimit"/>); 0 for none.</summary>
    public float StrainLimit { get; set; } = 0.05f;

    /// <summary>Coulomb friction against shapes and itself.</summary>
    public float Friction { get; set; } = 0.5f;

    /// <summary>Broadside drag coefficient (see <see cref="ClothParams.Drag"/>).</summary>
    public float Drag { get; set; } = 1.2f;

    /// <summary>Lift slope in 1/rad (see <see cref="ClothParams.LiftSlope"/>).</summary>
    public float LiftSlope { get; set; } = 2f;

    /// <summary>Damping time of the stretch and bend constraints, seconds.</summary>
    public float Damping { get; set; } = 0.02f;

    /// <summary>Velocity damping, 1/s.</summary>
    public float LinearDamping { get; set; } = 0.05f;

    /// <summary>A light plain weave: 0.15 kg/m², stretches a little more across the weft, loose shear.</summary>
    public static ClothMaterial Cotton => new();

    /// <summary>A heavy twill: 0.4 kg/m², stiff in both directions, stiff in bending.</summary>
    public static ClothMaterial Denim => new() { ArealDensity = 0.4f, WarpCompliance = 6e-4f, WeftCompliance = 8e-4f, ShearCompliance = 1e-2f, BendCompliance = 5e-4f, StrainLimit = 0.03f, Friction = 0.6f, Drag = 1.3f };

    /// <summary>A light, slippery fabric: 0.06 kg/m², limp, low friction, catches the wind.</summary>
    public static ClothMaterial Silk => new() { ArealDensity = 0.06f, WarpCompliance = 8e-3f, WeftCompliance = 1e-2f, ShearCompliance = 8e-2f, BendCompliance = 8e-3f, StrainLimit = 0.06f, Friction = 0.25f, Drag = 1.1f, LiftSlope = 2.5f };

    /// <summary>A thick hide: 0.9 kg/m², nearly inextensible, very stiff in bending.</summary>
    public static ClothMaterial Leather => new() { ArealDensity = 0.9f, WarpCompliance = 2e-4f, WeftCompliance = 2e-4f, ShearCompliance = 1e-3f, BendCompliance = 1e-4f, StrainLimit = 0.02f, Friction = 0.7f, Drag = 1.4f, Damping = 0.05f };

    /// <summary>A rubber sheet: 1.5 kg/m², stretches a lot and snaps back, no strain limit before 30 %.</summary>
    public static ClothMaterial RubberSheet => new() { ArealDensity = 1.5f, WarpCompliance = 1.2e-3f, WeftCompliance = 1.2e-3f, ShearCompliance = 2.4e-3f, BendCompliance = 1e-3f, StrainLimit = 0.3f, Friction = 0.9f, Drag = 1.4f, Damping = 0.01f };

    /// <summary>A copy of this material.</summary>
    public ClothMaterial Clone() => (ClothMaterial)MemberwiseClone();
}

/// <summary>
/// A rectangular piece of fabric (middle tier): a grid of <see cref="SegmentsX"/> × <see cref="SegmentsZ"/> cells in the x–z plane, warp along x and weft along z, made of a <see cref="Material"/>. Pass it to
/// <see cref="PhysicsWorld3D.AddCloth(ClothDef)"/>; <see cref="ToSoftBodyDef"/> gives the soft-body definition it makes, for anything the properties here do not reach.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public sealed class ClothDef
{
    /// <summary>Size along x (warp), metres. Default 1.</summary>
    public float Width { get; set; } = 1f;

    /// <summary>Size along z (weft), metres. Default 1.</summary>
    public float Depth { get; set; } = 1f;

    /// <summary>Cells along x, 1 to 1024. Default 16.</summary>
    public int SegmentsX { get; set; } = 16;

    /// <summary>Cells along z, 1 to 1024. Default 16.</summary>
    public int SegmentsZ { get; set; } = 16;

    /// <summary>Centre of the sheet, world coordinates.</summary>
    public Vector3D Position { get; set; }

    /// <summary>Orientation of the sheet; default identity (flat, facing +y).</summary>
    public QuaternionF Rotation { get; set; } = QuaternionF.Identity;

    /// <summary>The fabric. Default <see cref="ClothMaterial.Cotton"/>.</summary>
    public ClothMaterial Material { get; set; } = ClothMaterial.Cotton;

    /// <summary>Total mass in kilograms; 0 (default) takes the material's mass per area.</summary>
    public float Mass { get; set; }

    /// <summary>Particle radius (half the thickness) for collision; 0 (default) takes a quarter of the cell size.</summary>
    public float ParticleRadius { get; set; }

    /// <summary>Rest curvature along x, 1/m: the rest shape is bent into a cylinder of radius 1/curvature about an axis parallel to z, curling towards +y (negative curls the other way). The sheet starts flat unless
    /// <see cref="StartCurved"/>. Needs the dihedral bend, which this switches on. Default 0.</summary>
    public float RestCurvatureX { get; set; }

    /// <summary>Whether the sheet starts in its curved rest shape. Default false (it starts flat and curls).</summary>
    public bool StartCurved { get; set; }

    /// <summary>Particles held in place (infinite mass). See <see cref="PinEdge"/> and <see cref="PinCorners"/> for the usual ones.</summary>
    public IReadOnlyList<int>? PinnedParticles { get; set; }

    /// <summary>Whether to tether every particle to the pinned ones (long range attachments, <see cref="TetherStrain"/>). Default false.</summary>
    public bool Tethers { get; set; }

    /// <summary>Tether slack as a strain. Default 0.02.</summary>
    public float TetherStrain { get; set; } = 0.02f;

    /// <summary>Whether the sheet collides with itself. Default true.</summary>
    public bool SelfCollision { get; set; } = true;

    /// <summary>Whether the sheet's surface stops other bodies' particles and edges (see <see cref="SoftBodyDef.SurfaceCollision"/>). Default true.</summary>
    public bool SurfaceCollision { get; set; } = true;

    /// <summary>Tearing by strain (see <see cref="SoftBodyDef.TearStrain"/>); 0 is off.</summary>
    public float TearStrain { get; set; }

    /// <summary>Tearing by force (see <see cref="SoftBodyDef.TearForce"/>); 0 is off.</summary>
    public float TearForce { get; set; }

    /// <summary>Plastic yield strain (see <see cref="SoftBodyDef.PlasticYield"/>); 0 is off.</summary>
    public float PlasticYield { get; set; }

    /// <summary>Initial velocity of every particle.</summary>
    public Vector3F LinearVelocity { get; set; }

    /// <summary>Multiplier on gravity. Default 1.</summary>
    public float GravityScale { get; set; } = 1f;

    /// <summary>Collision category bits. Default 1.</summary>
    public uint CategoryBits { get; set; } = 1u;

    /// <summary>Collision mask bits. Default all.</summary>
    public uint MaskBits { get; set; } = uint.MaxValue;

    /// <summary>Collision group. Default 0.</summary>
    public int GroupIndex { get; set; }

    /// <summary>Layer (0 to 31). Default 0.</summary>
    public int Layer { get; set; }

    /// <summary>Whether the sheet may fall asleep. Default true.</summary>
    public bool AllowSleep { get; set; } = true;

    /// <summary>The four corner particles of a sheet of <paramref name="segmentsX"/> × <paramref name="segmentsZ"/> cells.</summary>
    public static int[] PinCorners(int segmentsX, int segmentsZ)
    {
        int nx = segmentsX + 1;
        return new[] { 0, segmentsX, segmentsZ * nx, segmentsZ * nx + segmentsX };
    }

    /// <summary>The particles along one edge of the sheet: the edge at z = -Depth/2 (<paramref name="edge"/> 0), x = +Width/2 (1), z = +Depth/2 (2) or x = -Width/2 (3).</summary>
    /// <exception cref="ArgumentOutOfRangeException">The edge is not 0 to 3.</exception>
    public static int[] PinEdge(int segmentsX, int segmentsZ, int edge)
    {
        int nx = segmentsX + 1;
        var r = new List<int>();
        switch (edge)
        {
            case 0: for (int i = 0; i <= segmentsX; i++) { r.Add(i); } break;
            case 1: for (int j = 0; j <= segmentsZ; j++) { r.Add(j * nx + segmentsX); } break;
            case 2: for (int i = 0; i <= segmentsX; i++) { r.Add(segmentsZ * nx + i); } break;
            case 3: for (int j = 0; j <= segmentsZ; j++) { r.Add(j * nx); } break;
            default: throw new ArgumentOutOfRangeException(nameof(edge), edge, "edge must be 0 to 3.");
        }

        return r.ToArray();
    }

    /// <summary>The soft-body definition this cloth makes.</summary>
    /// <exception cref="SoftBodyException">A value is out of range.</exception>
    public SoftBodyDef ToSoftBodyDef()
    {
        var m = Material ?? throw new SoftBodyException(SoftBodyError.InvalidDefinition, "ClothDef.Material is null.", nameof(Material));
        if (!float.IsFinite(RestCurvatureX) || Math.Abs(RestCurvatureX) > 1000f) { throw new SoftBodyException(SoftBodyError.OutOfRange, "ClothDef.RestCurvatureX is " + RestCurvatureX.ToString("R", CultureInfo.InvariantCulture) + "; expected a finite value within ±1000.", nameof(RestCurvatureX)); }
        if (!(m.ArealDensity > 0f) || !float.IsFinite(m.ArealDensity)) { throw new SoftBodyException(SoftBodyError.OutOfRange, "ClothMaterial.ArealDensity is " + m.ArealDensity.ToString("R", CultureInfo.InvariantCulture) + "; it must be finite and above 0.", nameof(m.ArealDensity)); }
        var flat = SoftBodyMesh.Cloth(Width, Depth, SegmentsX, SegmentsZ);
        SoftBodyMesh mesh = flat;
        IReadOnlyList<Vector3F>? start = null;
        float k = RestCurvatureX;
        if (k != 0f)
        {
            var v = new Vector3F[flat.VertexCount];
            for (int i = 0; i < v.Length; i++)
            {
                var p = flat.Vertices[i];
                // An arc of the same length: the thread along x keeps its length, so the rest edge lengths are those of the flat sheet.
                var (s, c) = Math32.SinCos(k * p.X);
                v[i] = new Vector3F(s / k, (1f - c) / k, p.Z);
            }

            mesh = SoftBodyMesh.FromTriangles(v, flat.Triangles.ToArray());
            if (!StartCurved) { start = flat.Vertices.ToArray(); }
        }

        var def = SoftBodyDef.Cloth(mesh, Position);
        def.Rotation = Rotation;
        def.Mass = Mass != 0f ? Mass : m.ArealDensity * Width * Depth;
        def.ParticleRadius = ParticleRadius;
        def.EdgeCompliance = m.WarpCompliance;
        def.EdgeDamping = m.Damping; def.BendDamping = m.Damping;
        def.BendCompliance = m.BendCompliance;
        def.Friction = m.Friction;
        def.LinearDamping = m.LinearDamping;
        def.GravityScale = GravityScale;
        def.SelfCollision = SelfCollision; def.SurfaceCollision = SurfaceCollision;
        def.TearStrain = TearStrain; def.TearForce = TearForce; def.PlasticYield = PlasticYield;
        def.LinearVelocity = LinearVelocity;
        def.CategoryBits = CategoryBits; def.MaskBits = MaskBits; def.GroupIndex = GroupIndex; def.Layer = Layer; def.AllowSleep = AllowSleep;
        def.PinnedParticles = PinnedParticles == null ? null : new List<int>(PinnedParticles);
        def.StartVertices = start;
        def.Fabric = new ClothParams
        {
            WarpCompliance = m.WarpCompliance, WeftCompliance = m.WeftCompliance, ShearCompliance = m.ShearCompliance,
            StrainLimit = m.StrainLimit, DihedralBend = k != 0f, Drag = m.Drag, LiftSlope = m.LiftSlope,
            TetherAnchors = Tethers && PinnedParticles != null ? new List<int>(PinnedParticles) : null, TetherStrain = TetherStrain,
        };
        return def;
    }
}
