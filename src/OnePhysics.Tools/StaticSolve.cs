using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>What was wrong with an input to <see cref="StaticSolver"/>.</summary>
public enum StaticSolveError
{
    /// <summary>The mesh has no vertices or no tetrahedra, or the problem has no loads at all where some are needed.</summary>
    Empty,

    /// <summary>A coordinate, material value, load or option is NaN or infinite.</summary>
    NonFinite,

    /// <summary>A value is outside its supported range (a non-positive modulus, a Poisson ratio of 0.5 or more, a size beyond the limits).</summary>
    OutOfRange,

    /// <summary>A tetrahedron, support or load names a vertex that does not exist, or the tetrahedra array is not a multiple of four.</summary>
    BadIndex,

    /// <summary>A tetrahedron has no volume (repeated vertices, flat or sliver).</summary>
    DegenerateTetrahedron,

    /// <summary>A tetrahedron has negative volume: vertices 0, 1, 2 do not run counter-clockwise seen from vertex 3.</summary>
    InvertedTetrahedron,

    /// <summary>A vertex belongs to no tetrahedron, so it has no stiffness.</summary>
    UnusedVertex,

    /// <summary>The problem has no supports.</summary>
    NoSupports,

    /// <summary>The supports leave a rigid-body motion of the mesh (or of one of its connected parts) free, so the displacement is not determined.</summary>
    Mechanism,

    /// <summary>A support is listed twice, or names no axis.</summary>
    BadSupport,

    /// <summary>A face load names three vertices that are not a boundary face of the mesh.</summary>
    BadFace,

    /// <summary>An option has an invalid value.</summary>
    InvalidOptions,
}

/// <summary>Thrown by <see cref="StaticSolver"/> when an input cannot be solved. The message names the input and what was expected; <see cref="ArgumentException.ParamName"/> is the offending parameter.</summary>
public sealed class StaticSolveException : ArgumentException
{
    /// <summary>Creates the exception.</summary>
    /// <param name="error">The kind of failure.</param>
    /// <param name="message">What was wrong and where.</param>
    /// <param name="paramName">The offending parameter.</param>
    public StaticSolveException(StaticSolveError error, string message, string? paramName = null)
        : base(message, paramName)
    {
        Error = error;
    }

    /// <summary>The kind of failure.</summary>
    public StaticSolveError Error { get; }
}

/// <summary>An isotropic linear-elastic material, in SI units, with an optional strength knock-down across printed layers.</summary>
public readonly struct ElasticMaterial
{
    /// <summary>Young's modulus, Pa. Positive and finite.</summary>
    public double YoungsModulus { get; init; }

    /// <summary>Poisson's ratio, above -1 and below 0.5 (at 0.5 the material is incompressible, which the displacement formulation cannot represent).</summary>
    public double PoissonRatio { get; init; }

    /// <summary>Density, kg/m³, used by gravity loads. 0 or more.</summary>
    public double Density { get; init; }

    /// <summary>Yield or ultimate strength, Pa, that the von Mises stress is compared with. 0 means not given: safety factors are then NaN.</summary>
    public double Strength { get; init; }

    /// <summary>The direction across the layers (the build direction of a printed part), in the mesh's coordinates; it is normalised. A zero vector (the default) means the material has no weak direction.</summary>
    public Vector3F WeakAxis { get; init; }

    /// <summary>The strength across the layers as a fraction of <see cref="Strength"/>, in (0, 1]; 0 (the default) means 1. Needs <see cref="WeakAxis"/> and <see cref="Strength"/> when below 1. The tension normal to the layers and the shear on them are checked against this reduced strength as well as the von Mises stress against <see cref="Strength"/>.</summary>
    public double WeakAxisStrengthRatio { get; init; }

    /// <summary>Creates a material without a weak axis.</summary>
    public static ElasticMaterial Create(double youngsModulus, double poissonRatio, double density = 0, double strength = 0)
        => new() { YoungsModulus = youngsModulus, PoissonRatio = poissonRatio, Density = density, Strength = strength };
}

/// <summary>Which components of a vertex's displacement a support holds.</summary>
[Flags]
public enum SupportAxes
{
    /// <summary>No component (not a valid support).</summary>
    None = 0,

    /// <summary>The x component.</summary>
    X = 1,

    /// <summary>The y component.</summary>
    Y = 2,

    /// <summary>The z component.</summary>
    Z = 4,

    /// <summary>All three components: a fixed vertex.</summary>
    All = 7,
}

/// <summary>Holds some components of a vertex's displacement at a given value (0 by default): a fixed point, a roller (one or two axes) or a prescribed displacement.</summary>
public readonly struct StaticSupport
{
    /// <summary>Creates a support.</summary>
    /// <param name="vertex">Index into the mesh's vertices.</param>
    /// <param name="axes">The components held.</param>
    /// <param name="displacement">The value they are held at, m (components not in <paramref name="axes"/> are ignored); zero by default.</param>
    public StaticSupport(int vertex, SupportAxes axes = SupportAxes.All, Vector3D displacement = default)
    {
        Vertex = vertex; Axes = axes; Displacement = displacement;
    }

    /// <summary>Index of the vertex.</summary>
    public int Vertex { get; }

    /// <summary>The components held.</summary>
    public SupportAxes Axes { get; }

    /// <summary>The value the held components take, m.</summary>
    public Vector3D Displacement { get; }
}

/// <summary>A force at a vertex.</summary>
public readonly struct PointLoad
{
    /// <summary>Creates a point load.</summary>
    /// <param name="vertex">Index into the mesh's vertices.</param>
    /// <param name="force">Force, N, in the mesh's axes.</param>
    public PointLoad(int vertex, Vector3D force) { Vertex = vertex; Force = force; }

    /// <summary>Index of the vertex.</summary>
    public int Vertex { get; }

    /// <summary>The force, N.</summary>
    public Vector3D Force { get; }
}

/// <summary>A uniform load on one boundary triangle of the mesh: a pressure (positive pushes into the body) and/or a traction vector (force per area, Pa).</summary>
public readonly struct FaceLoad
{
    /// <summary>Creates a face load.</summary>
    /// <param name="v0">First vertex of the face, an index into the mesh's vertices.</param>
    /// <param name="v1">Second vertex.</param>
    /// <param name="v2">Third vertex. The three, in any order, must be a face of exactly one tetrahedron.</param>
    /// <param name="pressure">Pressure, Pa; positive pushes into the body, negative pulls (tension).</param>
    /// <param name="traction">Added traction, Pa, in the mesh's axes (shear loads); zero by default.</param>
    public FaceLoad(int v0, int v1, int v2, double pressure, Vector3D traction = default)
    {
        V0 = v0; V1 = v1; V2 = v2; Pressure = pressure; Traction = traction;
    }

    /// <summary>First vertex.</summary>
    public int V0 { get; }

    /// <summary>Second vertex.</summary>
    public int V1 { get; }

    /// <summary>Third vertex.</summary>
    public int V2 { get; }

    /// <summary>Pressure, Pa; positive pushes into the body.</summary>
    public double Pressure { get; }

    /// <summary>Traction added to the pressure, Pa.</summary>
    public Vector3D Traction { get; }
}

/// <summary>The material, supports and loads of a static solve.</summary>
public sealed class StaticProblem
{
    /// <summary>The material of every element.</summary>
    public ElasticMaterial Material { get; set; }

    /// <summary>Gravitational acceleration, m/s², applied to the mass the density gives; zero by default (no self weight).</summary>
    public Vector3D Gravity { get; set; }

    /// <summary>The supports. At least one is needed, and together they must hold every rigid-body motion of the mesh.</summary>
    public List<StaticSupport> Supports { get; } = new();

    /// <summary>Forces at vertices.</summary>
    public List<PointLoad> PointLoads { get; } = new();

    /// <summary>Pressures and tractions on boundary faces.</summary>
    public List<FaceLoad> FaceLoads { get; } = new();
}

/// <summary>The element family of the solve.</summary>
public enum StaticElementOrder
{
    /// <summary>Ten-node tetrahedra (quadratic displacement, linear strain), with the extra nodes at the edge midpoints. Bending is accurate on a coarse mesh. The default.</summary>
    Quadratic = 0,

    /// <summary>Four-node tetrahedra (constant strain). Cheap, but they lock in bending: the deflection of a beam comes out well below the true value unless the mesh is much finer.</summary>
    Linear = 1,
}

/// <summary>Options of <see cref="StaticSolver.Solve(ReadOnlySpan{Vector3F}, ReadOnlySpan{int}, StaticProblem, StaticSolveOptions)"/>. The default (all zero) takes the documented defaults.</summary>
public readonly struct StaticSolveOptions
{
    /// <summary>Element family; quadratic by default.</summary>
    public StaticElementOrder ElementOrder { get; init; }

    /// <summary>Metres per mesh unit; 0 means 1. Positions are scaled by it, and every output is in SI (metres, Pa, N), except locations, which stay in mesh units.</summary>
    public double MetresPerUnit { get; init; }

    /// <summary>The mesh's length unit, as a world states it (<see cref="RecursiveAI.OnePhysics.LengthUnit"/>): a world in millimetres passes <c>LengthUnit.Millimetres</c>. Used when <see cref="MetresPerUnit"/> is 0; a nonzero <see cref="MetresPerUnit"/> wins. Default one metre.</summary>
    public RecursiveAI.OnePhysics.LengthUnit LengthUnit { get; init; }

    /// <summary>Stop when the residual norm is below this fraction of the load norm; 0 means 1e-8. Between 1e-15 and 1e-2. Very stiff, long models have a rounding floor above 1e-10 (the measured residual cannot get below it), where the solve stops with <see cref="StaticSolution.Converged"/> false.</summary>
    public double Tolerance { get; init; }

    /// <summary>Most conjugate-gradient iterations; 0 means 20000. When it is reached the result has <see cref="StaticSolution.Converged"/> false.</summary>
    public int MaxIterations { get; init; }

    /// <summary>Threads for assembly, products and element results; 0 or 1 is single-threaded, at most 256. The result is bit-identical for every value: every reduction runs over fixed blocks in index order.</summary>
    public int MaxThreads { get; init; }
}

/// <summary>A symmetric 3×3 tensor (a strain or a stress). Shear components are tensor components (strain: half the engineering shear).</summary>
public readonly struct SymmetricTensor
{
    /// <summary>Creates a tensor.</summary>
    public SymmetricTensor(double xx, double yy, double zz, double xy, double yz, double zx) { XX = xx; YY = yy; ZZ = zz; XY = xy; YZ = yz; ZX = zx; }

    /// <summary>The xx component.</summary>
    public double XX { get; }

    /// <summary>The yy component.</summary>
    public double YY { get; }

    /// <summary>The zz component.</summary>
    public double ZZ { get; }

    /// <summary>The xy component.</summary>
    public double XY { get; }

    /// <summary>The yz component.</summary>
    public double YZ { get; }

    /// <summary>The zx component.</summary>
    public double ZX { get; }

    /// <summary>The von Mises equivalent, sqrt(½((xx−yy)² + (yy−zz)² + (zz−xx)²) + 3(xy² + yz² + zx²)); meaningful for a stress.</summary>
    public double VonMises
    {
        get
        {
            double a = XX - YY, b = YY - ZZ, c = ZZ - XX;
            return Math.Sqrt(0.5 * (a * a + b * b + c * c) + 3.0 * (XY * XY + YZ * YZ + ZX * ZX));
        }
    }
}

/// <summary>The result of <see cref="StaticSolver"/>. Arrays are indexed like the mesh's vertices and tetrahedra (tetrahedron <c>k</c> is the four indices from <c>4k</c>).</summary>
public sealed class StaticSolution
{
    internal StaticSolution() { }

    /// <summary>Displacement of each vertex, m.</summary>
    public Vector3D[] Displacements { get; internal set; } = Array.Empty<Vector3D>();

    /// <summary>Strain at the centroid of each tetrahedron.</summary>
    public SymmetricTensor[] Strains { get; internal set; } = Array.Empty<SymmetricTensor>();

    /// <summary>Stress at the centroid of each tetrahedron, Pa.</summary>
    public SymmetricTensor[] Stresses { get; internal set; } = Array.Empty<SymmetricTensor>();

    /// <summary>Von Mises stress at each tetrahedron's centroid, Pa.</summary>
    public double[] VonMises { get; internal set; } = Array.Empty<double>();

    /// <summary>The largest von Mises stress over each tetrahedron's centroid and four corners, Pa. Equal to <see cref="VonMises"/> for linear elements. Corners next to supports and point loads show the singular stress those idealisations have, and it grows as the mesh is refined.</summary>
    public double[] PeakVonMises { get; internal set; } = Array.Empty<double>();

    /// <summary>Safety factor of each tetrahedron: strength over the largest utilisation among its centroid and four corners (von Mises against <see cref="ElasticMaterial.Strength"/>, and, with a weak axis, interlayer tension and shear against the reduced strength). <see cref="double.PositiveInfinity"/> for an unstressed element; NaN everywhere when the material has no strength.</summary>
    public double[] SafetyFactors { get; internal set; } = Array.Empty<double>();

    /// <summary>Von Mises stress at each vertex: the average over the tetrahedra around it of the stress at that corner, weighted by volume, Pa.</summary>
    public double[] VertexVonMises { get; internal set; } = Array.Empty<double>();

    /// <summary>The tetrahedron with the lowest safety factor, or with the highest <see cref="PeakVonMises"/> when the material has no strength (lowest index on ties).</summary>
    public int WorstElement { get; internal set; }

    /// <summary>The centroid of <see cref="WorstElement"/>, in mesh units.</summary>
    public Vector3D WorstLocation { get; internal set; }

    /// <summary>The lowest safety factor (NaN without strength).</summary>
    public double MinSafetyFactor { get; internal set; }

    /// <summary>The highest <see cref="PeakVonMises"/>, Pa.</summary>
    public double MaxVonMises { get; internal set; }

    /// <summary>The largest displacement magnitude over the vertices, m.</summary>
    public double MaxDisplacement { get; internal set; }

    /// <summary>The sum of the support reactions, N; it balances the applied loads to the solver's tolerance.</summary>
    public Vector3D TotalReaction { get; internal set; }

    /// <summary>The volume of the mesh, m³.</summary>
    public double Volume { get; internal set; }

    /// <summary>The mass the density gives the mesh, kg.</summary>
    public double Mass { get; internal set; }

    /// <summary>The residual norm of the returned displacement relative to the norm of the load, measured as b − A x (not the iteration's estimate).</summary>
    public double Residual { get; internal set; }

    /// <summary>Conjugate-gradient iterations used.</summary>
    public int Iterations { get; internal set; }

    /// <summary>True when the measured <see cref="Residual"/> is within the tolerance.</summary>
    public bool Converged { get; internal set; }

    /// <summary>Degrees of freedom of the system (three per node).</summary>
    public int DegreesOfFreedom { get; internal set; }
}
