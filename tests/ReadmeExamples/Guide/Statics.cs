using System;
using RecursiveAI.OneMath;                       // Vector3F, Vector3D
using RecursiveAI.OnePhysics.Tools;              // Tetrahedralizer, StaticSolver

namespace GuideExamples;

/// <summary>The code block of docs/guide/statics.md (compiled, not run: it needs a bracket mesh).</summary>
public static class Statics
{
    public static StaticSolution Check(Vector3F[] vertices, int[] indices)
    {
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
        return solution;
    }
}
