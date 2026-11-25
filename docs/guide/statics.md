# Checking a part: deflection, stress and safety factor

`RecursiveAI.OnePhysics.Tools` can answer "will this hold?" for a physical part: how far it bends under its own weight and a load, where it is stressed most, and by what factor the material has to spare. It is a linear-elastic static solve on the tetrahedra `Tetrahedralizer` makes (or any tetrahedral mesh), in SI units, the same bits on every platform and thread count.

<!-- example: Guide/Statics.cs -->
```csharp
using RecursiveAI.OnePhysics.Tools;

// A bracket given as a closed triangle mesh in millimetres: fill it with tetrahedra, hold its mounting face, hang 50 N from the tip.
var tets = Tetrahedralizer.Generate(vertices, indices, new TetrahedralizeOptions { TargetEdgeLength = 2f });
var problem = new StaticProblem
{
    Material = new ElasticMaterial
    {
        YoungsModulus = 2.3e9, PoissonRatio = 0.36, Density = 1250, Strength = 45e6,
        WeakAxis = new Vector3F(0, 0, 1), WeakAxisStrengthRatio = 0.5,   // printed upright: half the strength across the layers
    },
    Gravity = new Vector3D(0, 0, -9.81),
};
int tip = 0;
for (int i = 0; i < tets.Vertices.Length; i++)
{
    if (tets.Vertices[i].X < 1.5f) { problem.Supports.Add(new StaticSupport(i)); }            // everything within 1.5 mm of the wall is fixed
    if (tets.Vertices[i].X > tets.Vertices[tip].X) { tip = i; }
}
problem.PointLoads.Add(new PointLoad(tip, new Vector3D(0, 0, -50)));

var solution = StaticSolver.Solve(tets, problem, new StaticSolveOptions { MetresPerUnit = 0.001, MaxThreads = 4 });
Console.WriteLine($"tip moves {solution.MaxDisplacement * 1000:F2} mm; weakest element {solution.WorstElement} at {solution.WorstLocation} has safety factor {solution.MinSafetyFactor:F2}");
```

Everything is in SI: Pa, kg/m³, N, m/s², displacements in metres. The mesh may be in other units: `MetresPerUnit` says what one is (0.001 for millimetres). `WorstLocation` comes back in mesh units.

**Supports** hold some or all components of a vertex's displacement (`StaticSupport(vertex, SupportAxes.Z)` is a roller) at zero or at a value you give. They must hold every rigid motion of the part, or the solve refuses with `StaticSolveError.Mechanism`. **Loads** are gravity, forces at vertices, and pressures or tractions on boundary triangles (`FaceLoad`; a positive pressure pushes in).

**Reading the result.** `Displacements` per vertex; per tetrahedron `Strains`, `Stresses`, `VonMises` (at the centroid), `PeakVonMises` and `SafetyFactors` (the worst of the centroid and the four corners); `VertexVonMises` for colouring a surface. The safety factor is the strength over the von Mises stress; with a `WeakAxis` it also checks tension across the layers and shear between them against the reduced strength. `Converged` and `Residual` say how well the equations were solved; `TotalReaction` is the force the supports carry and balances the loads.

**How good it is.** The default is ten-node tetrahedra, which bend correctly on a coarse mesh: a beam ten sections long is within 1 % of beam theory with four cells across its section. Four-node tetrahedra (`ElementOrder = StaticElementOrder.Linear`) are faster but stiff in bending and need about ten times the cells across. `Tetrahedralizer` meshes have a ragged boundary, so for numbers you intend to trust use cells a tenth of the thinnest section or finer and compare `Volume` with `MeshVolume`. Stress at a support or under a point load is a singularity: it rises as the mesh is refined, so judge a part by the stress away from them. The method, the preconditioner, the accuracy table and the limits are in [the specification](https://github.com/recursive-ai-oss/onephysics-csharp/blob/v1.1.0/docs/spec/tools.md).

Bad input (a flat or inverted tetrahedron, no supports, a Poisson ratio of 0.5, NaN) throws `StaticSolveException`, whose `Error` says which and whose message names the value.
