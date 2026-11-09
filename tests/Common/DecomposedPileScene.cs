using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Tools;
using RecursiveAI.OnePhysics.Tools.Tests;

namespace RecursiveAI.OnePhysics.Tools.Tests;

/// <summary>
/// A pile of concave bodies (L-blocks, U-blocks, rings, mugs) dropped onto a floor, every one a compound of the convex hulls that <see cref="ConvexDecomposer"/> makes from its mesh. Shared by the tests, the stability runner and the hash matrix.
/// </summary>
public static class DecomposedPileScene
{
    private static ulong Next(ref ulong s)
    {
        ulong z = s += 0x9E3779B97F4A7C15UL;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        return z ^ (z >> 31);
    }

    private static float Unit(ref ulong s) => (Next(ref s) >> 40) * (1f / 16777216f);

    /// <summary>The decomposed shapes the pile is made of, with the mesh volume of each.</summary>
    public static (CompoundPart[] Parts, double Volume, string Name)[] Shapes(int threads = 1)
    {
        var meshes = new (TestMesh Mesh, string Name)[] { (TestMeshes.LBlock(), "L"), (TestMeshes.UBlock(), "U"), (TestMeshes.Torus(1f, 0.35f, 24, 10), "ring"), (TestMeshes.Mug(), "mug") };
        var result = new (CompoundPart[], double, string)[meshes.Length];
        for (int i = 0; i < meshes.Length; i++)
        {
            var m = meshes[i].Mesh;
            var d = ConvexDecomposer.Decompose(m.V, m.I, new ConvexDecompositionOptions { MaxHulls = 24, MaxConcavity = 0.03f, Resolution = 24, MaxThreads = threads });
            var clean = MeshCleaner.Clean(m.V, m.I);
            result[i] = (d.ToCompoundParts(), MeshMass.ComputeDouble(clean.Vertices, clean.Indices).Mass, meshes[i].Name);
        }

        return result;
    }

    /// <summary>Builds the world: a floor and <paramref name="count"/> bodies dropped from a staggered grid with seeded orientations.</summary>
    public static PhysicsWorld3D Build(out Body[] bodies, int count = 24, int threads = 1)
    {
        bodies = new Body[count];
        var world = new PhysicsWorld3D(new WorldOptions3D { WorldId = 77 });
        world.AddBox(new Vector3F(30, 1, 30), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var shapes = Shapes(threads);
        ulong seed = 12345;
        for (int i = 0; i < count; i++)
        {
            var (parts, _, _) = shapes[i % shapes.Length];
            double x = (i % 4) * 3.0 - 4.5, z = ((i / 4) % 3) * 3.0 - 3.0, y = 2.0 + 2.5 * (i / 12);
            var axis = new Vector3F(Unit(ref seed) - 0.5f, Unit(ref seed) + 0.1f, Unit(ref seed) - 0.5f).Normalized();
            var q = QuaternionF.CreateFromAxisAngle(axis, Unit(ref seed) * 6f);
            bodies[i] = world.AddCompound(parts, new Vector3D(x, y, z), q);
        }

        return world;
    }
}
