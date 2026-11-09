using System;
using RecursiveAI.OneMath;
using Xunit;

namespace RecursiveAI.OnePhysics.Tools.Tests;

public class HullSimplifierTests
{
    private static ConvexHull3D SphereHull(int n, float r = 1f)
    {
        // Fibonacci sphere points: deterministic and well spread.
        var pts = new Vector3F[n];
        double golden = Math.PI * (3 - Math.Sqrt(5));
        for (int i = 0; i < n; i++)
        {
            double y = 1 - 2.0 * (i + 0.5) / n, rr = Math.Sqrt(1 - y * y), th = golden * i;
            pts[i] = new Vector3F((float)(r * rr * Math.Cos(th)), (float)(r * y), (float)(r * rr * Math.Sin(th)));
        }

        return ConvexHull3D.Build(pts);
    }

    private static double BruteForceError(ConvexHull3D original, ConvexHull3D simplified) => HullSimplifier.MaxDistanceOutside(simplified, original.Vertices);

    [Fact]
    public void VertexTargetIsReachedAndErrorIsMeasuredHonestly()
    {
        var h = SphereHull(200);
        Assert.Equal(200, h.Vertices.Length);
        var s = HullSimplifier.Simplify(h, new HullSimplifyOptions { MaxVertices = 40 });
        Assert.True(s.TargetsMet);
        Assert.True(s.Hull.Vertices.Length <= 40);
        Assert.Equal(h.Vertices.Length - s.Hull.Vertices.Length, s.RemovedVertices);
        Assert.Equal(BruteForceError(h, s.Hull), s.Error, 5);
        // An independent lower bound: no original vertex may stand further out than Error from any face plane of the result.
        double plane = 0;
        foreach (var o in h.Vertices) { foreach (var f in s.Hull.Faces) { plane = Math.Max(plane, Vector3F.Dot(f.Normal, o) - f.Offset); } }
        Assert.True(plane <= s.Error + 1e-5);
        // Dropping 160 of 200 points from a unit sphere keeps the shape within a few percent.
        Assert.True(s.Error < 0.1f);
        // Every simplified vertex is an original vertex.
        foreach (var v in s.Hull.Vertices)
        {
            bool found = false;
            foreach (var o in h.Vertices) { if (o == v) { found = true; break; } }
            Assert.True(found);
        }
    }

    [Fact]
    public void ErrorBoundIsHonoured()
    {
        var h = SphereHull(150);
        foreach (float bound in new[] { 0.005f, 0.05f, 0.1f })
        {
            var s = HullSimplifier.Simplify(h, new HullSimplifyOptions { MaxError = bound });
            Assert.True(s.Error <= bound * 1.0001f, "error " + s.Error + " exceeds " + bound);
            Assert.True(BruteForceError(h, s.Hull) <= bound * 1.0001f);
            if (bound >= 0.1f) { Assert.True(s.Hull.Vertices.Length < 150); }
        }

        // A looser bound never keeps more vertices.
        var tight = HullSimplifier.Simplify(h, new HullSimplifyOptions { MaxError = 0.005f });
        var loose = HullSimplifier.Simplify(h, new HullSimplifyOptions { MaxError = 0.1f });
        Assert.True(loose.Hull.Vertices.Length <= tight.Hull.Vertices.Length);
    }

    [Fact]
    public void FaceTargetAndDeterminism()
    {
        var h = SphereHull(120);
        var a = HullSimplifier.Simplify(h, new HullSimplifyOptions { MaxFaces = 30 });
        var b = HullSimplifier.Simplify(h, new HullSimplifyOptions { MaxFaces = 30 });
        Assert.True(a.Hull.Faces.Length <= 30);
        Assert.Equal(a.Hull.Vertices.ToArray(), b.Hull.Vertices.ToArray());
    }

    [Fact]
    public void ABoxCannotBeReducedBelowItsCornersWithoutError()
    {
        var box = ConvexHull3D.Build(TestMeshes.Box(1, 2, 3).V);
        var s = HullSimplifier.Simplify(box, new HullSimplifyOptions { MaxError = 0.001f });
        Assert.Equal(8, s.Hull.Vertices.Length);
        Assert.Equal(0f, s.Error);
        var t = HullSimplifier.Simplify(box, new HullSimplifyOptions { MaxVertices = 4 });
        Assert.Equal(4, t.Hull.Vertices.Length);
        Assert.True(t.Error > 0.5f);
    }

    [Fact]
    public void InvalidOptionsGetTypedErrors()
    {
        var h = SphereHull(30);
        Assert.Equal(GeometryToolError.InvalidOptions, Assert.Throws<GeometryToolException>(() => HullSimplifier.Simplify(h, default)).Error);
        Assert.Equal(GeometryToolError.InvalidOptions, Assert.Throws<GeometryToolException>(() => HullSimplifier.Simplify(h, new HullSimplifyOptions { MaxVertices = 3 })).Error);
        Assert.Equal(GeometryToolError.InvalidOptions, Assert.Throws<GeometryToolException>(() => HullSimplifier.Simplify(h, new HullSimplifyOptions { MaxError = float.NaN })).Error);
        Assert.Throws<ArgumentNullException>(() => HullSimplifier.Simplify(null!, new HullSimplifyOptions { MaxVertices = 8 }));
    }
}
