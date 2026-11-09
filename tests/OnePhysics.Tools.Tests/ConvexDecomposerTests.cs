using System;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tools.Tests;

public class ConvexDecomposerTests
{
    private static double MeshVolume(TestMesh m)
    {
        var c = MeshCleaner.Clean(m.V, m.I);
        return MeshMass.ComputeDouble(c.Vertices, c.Indices).Mass;
    }

    private static bool InsideHull(ConvexHull3D h, double x, double y, double z, double slack = 1e-6)
    {
        foreach (var f in h.Faces)
        {
            if (f.Normal.X * x + f.Normal.Y * y + f.Normal.Z * z > f.Offset + slack) { return false; }
        }

        return true;
    }

    private static void AssertConvex(ConvexHull3D h)
    {
        foreach (var f in h.Faces)
        {
            foreach (var v in h.Vertices) { Assert.True(Vector3F.Dot(f.Normal, v) <= f.Offset + 1e-4f); }
        }

        Assert.True(h.MassProperties.Mass > 0);
    }

    [Fact]
    public void BoxIsOneHull()
    {
        var m = TestMeshes.Box(1, 2, 3, 0.5f, -1, 4);
        var d = ConvexDecomposer.Decompose(m.V, m.I);
        Assert.Single(d.Hulls);
        Assert.True(d.ThresholdMet);
        Assert.Equal(8, d.Hulls[0].Vertices.Length);
        Assert.InRange(d.TotalHullVolume / 6.0, 0.99, 1.01);
    }

    [Fact]
    public void ABlockOfAnyOrientationIsFewHulls()
    {
        // A rotated box has a staircase of voxels; the mesh surface is still flat, so it is still one hull.
        var m = TestMeshes.Box(1, 2, 3);
        var rot = QuaternionF.CreateFromAxisAngle(new Vector3F(1, 1, 0).Normalized(), 0.6f);
        for (int i = 0; i < m.Vertices.Count; i++) { m.Vertices[i] = rot.Rotate(m.Vertices[i]); }
        var d = ConvexDecomposer.Decompose(m.V, m.I);
        Assert.Single(d.Hulls);
        Assert.InRange(d.TotalHullVolume / 6.0, 0.97, 1.03);
    }


    [Fact]
    public void LAndUBlocksAreTheObviousBoxes()
    {
        var l = TestMeshes.LBlock();
        var dl = ConvexDecomposer.Decompose(l.V, l.I);
        Assert.Equal(2, dl.Hulls.Count);
        Assert.True(dl.ThresholdMet);
        Assert.InRange(dl.TotalHullVolume / 3.0, 0.99, 1.02);

        var u = TestMeshes.UBlock();
        var du = ConvexDecomposer.Decompose(u.V, u.I);
        Assert.Equal(3, du.Hulls.Count);
        Assert.True(du.ThresholdMet);
        Assert.InRange(du.TotalHullVolume / 5.0, 0.99, 1.02);
    }

    [Fact]
    public void TorusIsCoveredByConvexHullsWithinTheThreshold()
    {
        const float R = 1f, r = 0.35f;
        var m = TestMeshes.Torus(R, r, 48, 16);
        var opt = new ConvexDecompositionOptions { MaxHulls = 40, MaxConcavity = 0.01f };
        var d = ConvexDecomposer.Decompose(m.V, m.I, opt);
        Assert.True(d.ThresholdMet, "max concavity " + d.MaxConcavity + " with " + d.Hulls.Count + " hulls");
        Assert.True(d.MaxConcavity <= 0.01f);
        Assert.InRange(d.Hulls.Count, 8, 40);
        foreach (var h in d.Hulls) { AssertConvex(h); Assert.True(h.Vertices.Length <= 64); }

        // Union against the analytic torus on a sample grid: the hulls cover nearly all of it and add little.
        double torusVolume = 2 * Math.PI * Math.PI * R * r * r;
        int inTorus = 0, covered = 0, inUnion = 0;
        const int n = 60;
        double lo = -1.4, step = 2.8 / n;
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                for (int k = 0; k < n; k++)
                {
                    double x = lo + (i + 0.5) * step, y = -0.5 + (j + 0.5) / n, z = lo + (k + 0.5) * step;
                    double q = Math.Sqrt(x * x + z * z) - R;
                    bool torus = q * q + y * y <= r * r;
                    bool union = d.Hulls.Any(h => InsideHull(h, x, y, z));
                    if (torus) { inTorus++; }
                    if (torus && union) { covered++; }
                    if (union) { inUnion++; }
                }
            }
        }

        double coverage = (double)covered / inTorus;
        double unionVolume = inUnion * step * step / n;
        Assert.True(coverage > 0.9, "coverage " + coverage);
        Assert.InRange(unionVolume / torusVolume, 0.9, 1.45);
        Assert.InRange(d.TotalHullVolume / MeshVolume(m), 0.8, 1.6);
    }

    [Fact]
    public void MugGetsAHandleAndACavity()
    {
        var m = TestMeshes.Mug();
        var d = ConvexDecomposer.Decompose(m.V, m.I, new ConvexDecompositionOptions { MaxHulls = 32, MaxConcavity = 0.05f });
        Assert.True(d.ThresholdMet, "max concavity " + d.MaxConcavity + " with " + d.Hulls.Count + " hulls");
        foreach (var h in d.Hulls) { AssertConvex(h); }
        // The cavity is open: a point in the middle of the cup, above the floor, lies in no hull.
        Assert.DoesNotContain(d.Hulls, h => InsideHull(h, 0, 1.2, 0));
        // A point in the wall and one in the handle are covered.
        Assert.Contains(d.Hulls, h => InsideHull(h, 0.92, 1.0, 0));
        Assert.True(d.Hulls.Count > 6);
    }

    [Fact]
    public void ResultIsIdenticalAtEveryThreadCountAndRepeats()
    {
        var m = TestMeshes.Torus(1, 0.35f, 32, 12);
        var a = ConvexDecomposer.Decompose(m.V, m.I, new ConvexDecompositionOptions { MaxHulls = 12 });
        var b = ConvexDecomposer.Decompose(m.V, m.I, new ConvexDecompositionOptions { MaxHulls = 12, MaxThreads = 8 });
        var c = ConvexDecomposer.Decompose(m.V, m.I, new ConvexDecompositionOptions { MaxHulls = 12 });
        Assert.Equal(a.Hulls.Count, b.Hulls.Count);
        for (int i = 0; i < a.Hulls.Count; i++)
        {
            Assert.Equal(Bits(a.Hulls[i]), Bits(b.Hulls[i]));
            Assert.Equal(Bits(a.Hulls[i]), Bits(c.Hulls[i]));
        }
    }

    private static int[] Bits(ConvexHull3D h)
    {
        var v = h.Vertices;
        var r = new int[v.Length * 3];
        for (int i = 0; i < v.Length; i++)
        {
            r[3 * i] = BitConverter.SingleToInt32Bits(v[i].X);
            r[3 * i + 1] = BitConverter.SingleToInt32Bits(v[i].Y);
            r[3 * i + 2] = BitConverter.SingleToInt32Bits(v[i].Z);
        }

        return r;
    }

    [Fact]
    public void HullsAreLimitedInVertices()
    {
        var m = TestMeshes.Sphere(1, 24, 32);
        var d = ConvexDecomposer.Decompose(m.V, m.I, new ConvexDecompositionOptions { MaxVerticesPerHull = 20 });
        Assert.Single(d.Hulls);
        Assert.True(d.Hulls[0].Vertices.Length <= 20);
    }

    [Fact]
    public void MaxHullsIsRespectedAndReported()
    {
        var m = TestMeshes.Torus(1, 0.35f, 32, 12);
        var d = ConvexDecomposer.Decompose(m.V, m.I, new ConvexDecompositionOptions { MaxHulls = 3 });
        Assert.True(d.Hulls.Count <= 3);
        Assert.False(d.ThresholdMet);
        Assert.True(d.MaxConcavity > 0.01f);
    }

    [Fact]
    public void WindingAndOpenMeshesDoNotMatter()
    {
        var m = TestMeshes.LBlock();
        var idx = m.I;
        for (int t = 0; t < idx.Length; t += 6) { (idx[t + 1], idx[t + 2]) = (idx[t + 2], idx[t + 1]); }
        Assert.Equal(2, ConvexDecomposer.Decompose(m.V, idx).Hulls.Count);
        var open = m.I.Take(m.I.Length - 6).ToArray(); // a hole in the mesh: the decomposition still works
        Assert.True(ConvexDecomposer.Decompose(m.V, open).Hulls.Count >= 1);
    }

    [Fact]
    public void ScalesFromTinyToHugeWork()
    {
        foreach (float s in new[] { 1e-4f, 1f, 1e5f })
        {
            var m = TestMeshes.LBlock();
            for (int i = 0; i < m.Vertices.Count; i++) { m.Vertices[i] = m.Vertices[i] * s; }
            var d = ConvexDecomposer.Decompose(m.V, m.I);
            Assert.Equal(2, d.Hulls.Count);
        }
    }

    [Fact]
    public void HostileInputsGetTypedErrors()
    {
        var box = TestMeshes.Box(1, 1, 1);
        Assert.Equal(GeometryToolError.Empty, Assert.Throws<GeometryToolException>(() => ConvexDecomposer.Decompose(ReadOnlySpan<Vector3F>.Empty, ReadOnlySpan<int>.Empty)).Error);
        Assert.Equal(GeometryToolError.Empty, Assert.Throws<GeometryToolException>(() => ConvexDecomposer.Decompose(box.V, ReadOnlySpan<int>.Empty)).Error);
        Assert.Equal(GeometryToolError.BadIndex, Assert.Throws<GeometryToolException>(() => ConvexDecomposer.Decompose(box.V, new[] { 0, 1, 99 })).Error);
        var nan = box.V; nan[3] = new Vector3F(float.NaN, 0, 0);
        Assert.Equal(GeometryToolError.NonFinite, Assert.Throws<GeometryToolException>(() => ConvexDecomposer.Decompose(nan, box.I)).Error);
        var big = box.V; big[3] = new Vector3F(5e9f, 0, 0);
        Assert.Equal(GeometryToolError.OutOfRange, Assert.Throws<GeometryToolException>(() => ConvexDecomposer.Decompose(big, box.I)).Error);
        // A flat quad has no volume; so does a mesh of degenerate triangles.
        var flat = new[] { new Vector3F(0, 0, 0), new Vector3F(1, 0, 0), new Vector3F(1, 1, 0), new Vector3F(0, 1, 0) };
        Assert.Equal(GeometryToolError.Degenerate, Assert.Throws<GeometryToolException>(() => ConvexDecomposer.Decompose(flat, new[] { 0, 1, 2, 0, 2, 3 })).Error);
        var line = new[] { new Vector3F(0, 0, 0), new Vector3F(1, 0, 0), new Vector3F(2, 0, 0) };
        Assert.Equal(GeometryToolError.Degenerate, Assert.Throws<GeometryToolException>(() => ConvexDecomposer.Decompose(line, new[] { 0, 1, 2 })).Error);
        foreach (var o in new[]
        {
            new ConvexDecompositionOptions { MaxHulls = -1 }, new ConvexDecompositionOptions { MaxHulls = 5000 }, new ConvexDecompositionOptions { MaxConcavity = float.NaN },
            new ConvexDecompositionOptions { MaxConcavity = -1 }, new ConvexDecompositionOptions { MaxVerticesPerHull = 3 }, new ConvexDecompositionOptions { Resolution = 4 },
            new ConvexDecompositionOptions { Resolution = 1000 }, new ConvexDecompositionOptions { MaxThreads = 100000 },
        })
        {
            Assert.Equal(GeometryToolError.InvalidOptions, Assert.Throws<GeometryToolException>(() => ConvexDecomposer.Decompose(box.V, box.I, o)).Error);
        }
    }

    [Fact]
    public void DecomposedShapeSimulates()
    {
        var m = TestMeshes.LBlock();
        var d = ConvexDecomposer.Decompose(m.V, m.I);
        var world = new PhysicsWorld3D();
        world.AddBox(new Vector3F(20, 1, 20), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var body = world.AddCompound(d.ToCompoundParts(), new Vector3D(0, 3, 0));
        for (int i = 0; i < 240; i++) { world.Step(1f / 60f); }
        // The block's origin is on its underside, so resting on the ground puts the body at y = 0.
        Assert.True(body.Position.Y < 0.05 && body.Position.Y > -0.05, "y = " + body.Position.Y);
        Assert.True(body.LinearVelocity.Length() < 0.05f);
        // The same hulls as a single shape spec attach too.
        var b2 = world.CreateBody(BodyType.Dynamic, new Vector3D(5, 3, 0));
        world.CreateShape(b2, d.ToShapeSpec());
        world.Step(1f / 60f);
    }
}
