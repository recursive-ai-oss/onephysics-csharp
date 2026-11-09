using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OneMath;
using Xunit;

namespace RecursiveAI.OnePhysics.Tools.Tests;

public class TriangleMesherTests
{
    private static Vector2F[] Circle(float r, int n, bool ccw = true, float cx = 0, float cy = 0)
    {
        var p = new Vector2F[n];
        for (int i = 0; i < n; i++) { double a = 2 * Math.PI * i / n * (ccw ? 1 : -1); p[i] = new Vector2F(cx + (float)(r * Math64.Cos(a)), cy + (float)(r * Math64.Sin(a))); }
        return p;
    }

    public static IEnumerable<object[]> Cases()
    {
        yield return new object[] { "square" };
        yield return new object[] { "annulus" };
        yield return new object[] { "star" };
        yield return new object[] { "two holes" };
        yield return new object[] { "L" };
    }

    private static (Vector2F[] Outline, Vector2F[][]? Holes) Make(string name)
    {
        switch (name)
        {
            case "square": return (new[] { new Vector2F(0, 0), new Vector2F(4, 0), new Vector2F(4, 3), new Vector2F(0, 3) }, null);
            case "annulus": return (Circle(5, 40), new[] { Circle(2, 20, false) });
            case "star":
            {
                var star = new Vector2F[10];
                for (int i = 0; i < 10; i++) { double a = Math.PI * i / 5; double r = (i & 1) == 0 ? 2.0 : 1.2; star[i] = new Vector2F((float)(r * Math64.Cos(a)), (float)(r * Math64.Sin(a))); }
                return (star, null);
            }

            case "two holes": return (new[] { new Vector2F(-5, -5), new Vector2F(5, -5), new Vector2F(5, 5), new Vector2F(-5, 5) }, new[] { Circle(1.5f, 12, false), Circle(1f, 8, false, 3, -2) });
            default: return (new[] { new Vector2F(0, 0), new Vector2F(4, 0), new Vector2F(4, 1), new Vector2F(1, 1), new Vector2F(1, 4), new Vector2F(0, 4) }, null);
        }
    }

    [Theory]
    [MemberData(nameof(Cases))]
    public void MeshesHaveGoodAnglesAndCoverThePolygonExactly(string name)
    {
        var (outline, holes) = Make(name);
        var r = TriangleMesher.Generate(outline, holes);
        Assert.True(r.QualityMet);
        Assert.True(r.MinAngleDegrees >= 24.5, "min angle " + r.MinAngleDegrees);
        Assert.InRange(r.Area / r.PolygonArea, 1 - 1e-5, 1 + 1e-5);

        // every triangle counter-clockwise, independently measured angles
        double minAngle = 180;
        for (int t = 0; t < r.Triangles.Length / 3; t++)
        {
            var p = new[] { r.Vertices[r.Triangles[3 * t]], r.Vertices[r.Triangles[3 * t + 1]], r.Vertices[r.Triangles[3 * t + 2]] };
            double twice = ((double)p[1].X - p[0].X) * ((double)p[2].Y - p[0].Y) - ((double)p[1].Y - p[0].Y) * ((double)p[2].X - p[0].X);
            Assert.True(twice > 0, "triangle " + t + " is not counter-clockwise");
            for (int k = 0; k < 3; k++)
            {
                var a = p[k]; var b = p[(k + 1) % 3]; var c = p[(k + 2) % 3];
                double ux = b.X - a.X, uy = b.Y - a.Y, vx = c.X - a.X, vy = c.Y - a.Y;
                minAngle = Math.Min(minAngle, Math.Atan2(Math.Abs(ux * vy - uy * vx), ux * vx + uy * vy) * 180 / Math.PI);
            }
        }

        Assert.InRange(minAngle, r.MinAngleDegrees - 1e-3, r.MinAngleDegrees + 1e-3);

        // polygon points come first, in the order given
        int n = outline.Length;
        for (int i = 0; i < n; i++) { Assert.Equal(outline[i], r.Vertices[i]); }
        if (holes is not null) { int o = n; foreach (var h in holes) { for (int i = 0; i < h.Length; i++) { Assert.Equal(h[i], r.Vertices[o + i]); } o += h.Length; } }
        Assert.Equal(n + (holes?.Sum(h => h.Length) ?? 0), r.InputVertexCount);

        // every boundary edge of the mesh lies along a polygon edge: edges used once must be collinear with and inside a polygon edge
        var count = new Dictionary<long, int>();
        for (int t = 0; t < r.Triangles.Length / 3; t++) { for (int k = 0; k < 3; k++) { int a = r.Triangles[3 * t + k], b = r.Triangles[3 * t + (k + 1) % 3]; long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a; count[key] = count.GetValueOrDefault(key) + 1; } }
        var loops = new List<Vector2F[]> { outline };
        if (holes is not null) { loops.AddRange(holes); }
        foreach (var (key, c) in count)
        {
            Assert.True(c <= 2);
            if (c != 1) { continue; }
            var a = r.Vertices[(int)(key >> 32)]; var b = r.Vertices[(int)(key & 0xFFFFFFFF)];
            bool onBoundary = false;
            foreach (var loop in loops)
            {
                for (int i = 0; i < loop.Length && !onBoundary; i++)
                {
                    var p = loop[i]; var q = loop[(i + 1) % loop.Length];
                    double len = Math.Sqrt(((double)q.X - p.X) * ((double)q.X - p.X) + ((double)q.Y - p.Y) * ((double)q.Y - p.Y));
                    double da = Math.Abs(((double)q.X - p.X) * ((double)p.Y - a.Y) - ((double)p.X - a.X) * ((double)q.Y - p.Y)) / len;
                    double db = Math.Abs(((double)q.X - p.X) * ((double)p.Y - b.Y) - ((double)p.X - b.X) * ((double)q.Y - p.Y)) / len;
                    double ta = (((double)a.X - p.X) * ((double)q.X - p.X) + ((double)a.Y - p.Y) * ((double)q.Y - p.Y)) / (len * len), tb = (((double)b.X - p.X) * ((double)q.X - p.X) + ((double)b.Y - p.Y) * ((double)q.Y - p.Y)) / (len * len);
                    if (da < 1e-5 && db < 1e-5 && ta > -1e-5 && ta < 1 + 1e-5 && tb > -1e-5 && tb < 1 + 1e-5) { onBoundary = true; }
                }
            }

            Assert.True(onBoundary, "mesh boundary edge " + a + " - " + b + " is not on the polygon");
        }
    }

    [Fact]
    public void SizeAndAngleOptionsAreFollowed()
    {
        var sq = new[] { new Vector2F(0, 0), new Vector2F(4, 0), new Vector2F(4, 3), new Vector2F(0, 3) };
        var coarse = TriangleMesher.Generate(sq, null, new TriangleMeshOptions { TargetEdgeLength = 1f });
        var fine = TriangleMesher.Generate(sq, null, new TriangleMeshOptions { TargetEdgeLength = 0.5f });
        Assert.InRange((double)fine.Triangles.Length / coarse.Triangles.Length, 3.0, 5.5);
        double maxArea = 0;
        for (int t = 0; t < fine.Triangles.Length / 3; t++)
        {
            var a = fine.Vertices[fine.Triangles[3 * t]]; var b = fine.Vertices[fine.Triangles[3 * t + 1]]; var c = fine.Vertices[fine.Triangles[3 * t + 2]];
            maxArea = Math.Max(maxArea, 0.5 * (((double)b.X - a.X) * ((double)c.Y - a.Y) - ((double)b.Y - a.Y) * ((double)c.X - a.X)));
        }

        Assert.True(maxArea <= 0.4330127 * 0.25 * 1.0001, "largest triangle " + maxArea);
        var sharp = TriangleMesher.Generate(sq, null, new TriangleMeshOptions { TargetEdgeLength = 0.5f, MinAngleDegrees = 30f });
        Assert.True(sharp.MinAngleDegrees >= 29.5, "min angle " + sharp.MinAngleDegrees);
    }

    [Fact]
    public void ANarrowCornerDoesNotStopTheRefinement()
    {
        // A wedge with a 15 degree corner: the angle at that corner cannot be improved, the rest of the mesh still can, and the run ends.
        var wedge = new[] { new Vector2F(0, 0), new Vector2F(4, 0), new Vector2F(4 * (float)Math64.Cos(0.2618), 4 * (float)Math64.Sin(0.2618)) };
        var r = TriangleMesher.Generate(wedge, null, new TriangleMeshOptions { TargetEdgeLength = 0.5f });
        Assert.InRange(r.Area / r.PolygonArea, 1 - 1e-5, 1 + 1e-5);
        Assert.True(r.Triangles.Length / 3 < 20000);
    }

    [Fact]
    public void SameBitsOnEveryRun()
    {
        var (o, h) = Make("two holes");
        var a = TriangleMesher.Generate(o, h);
        var b = TriangleMesher.Generate(o, h);
        Assert.Equal(a.Vertices, b.Vertices);
        Assert.Equal(a.Triangles, b.Triangles);
    }

    [Fact]
    public void HostileInputsFailWithTypedErrors()
    {
        var sq = new[] { new Vector2F(0, 0), new Vector2F(4, 0), new Vector2F(4, 3), new Vector2F(0, 3) };
        GeometryToolError Err(Vector2F[] o, Vector2F[][]? h = null, TriangleMeshOptions opt = default)
        {
            var ex = Assert.Throws<GeometryToolException>(() => TriangleMesher.Generate(o, h, opt));
            Assert.False(string.IsNullOrEmpty(ex.Message));
            return ex.Error;
        }

        Assert.Equal(GeometryToolError.Empty, Err(Array.Empty<Vector2F>()));
        Assert.Equal(GeometryToolError.Empty, Err(new[] { new Vector2F(0, 0), new Vector2F(1, 0) }));
        Assert.Equal(GeometryToolError.NonFinite, Err(new[] { new Vector2F(0, 0), new Vector2F(float.NaN, 0), new Vector2F(1, 1) }));
        Assert.Equal(GeometryToolError.OutOfRange, Err(new[] { new Vector2F(0, 0), new Vector2F(1e9f, 0), new Vector2F(1, 1) }));
        Assert.Equal(GeometryToolError.Degenerate, Err(new[] { new Vector2F(0, 0), new Vector2F(1, 0), new Vector2F(2, 0) }));
        Assert.Equal(GeometryToolError.SelfIntersecting, Err(new[] { new Vector2F(0, 0), new Vector2F(3, 2), new Vector2F(3, 0), new Vector2F(0, 1.5f) }));
        Assert.Equal(GeometryToolError.SelfIntersecting, Err(sq, new[] { Circle(1, 8, false, 10, 10) }));
        Assert.Equal(GeometryToolError.SelfIntersecting, Err(sq, new[] { Circle(1, 8, false, 1, 1.5f), Circle(1, 8, false, 1.5f, 1.5f) }));
        Assert.Equal(GeometryToolError.Empty, Err(sq, new Vector2F[][] { null! }));
        Assert.Equal(GeometryToolError.InvalidOptions, Err(sq, null, new TriangleMeshOptions { MinAngleDegrees = 40f }));
        Assert.Equal(GeometryToolError.InvalidOptions, Err(sq, null, new TriangleMeshOptions { MinAngleDegrees = -1f }));
        Assert.Equal(GeometryToolError.InvalidOptions, Err(sq, null, new TriangleMeshOptions { TargetEdgeLength = float.NaN }));
        Assert.Equal(GeometryToolError.InvalidOptions, Err(sq, null, new TriangleMeshOptions { MaxVertices = 2 }));
        Assert.Equal(GeometryToolError.InvalidOptions, Err(sq, null, new TriangleMeshOptions { MaxVertices = 3 }));   // fewer than the polygon's own points
        // A cap that stops refinement is reported, not thrown.
        var capped = TriangleMesher.Generate(sq, null, new TriangleMeshOptions { TargetEdgeLength = 0.01f, MaxVertices = 200 });
        Assert.False(capped.QualityMet);
        Assert.True(capped.Vertices.Length <= 200);
    }

    [Fact]
    public void SoftBodiesMadeFromTheOutputStayStable()
    {
        var m = TriDropScene2D.Run();
        Assert.True(m.Passes, m.ToString());
    }
}
