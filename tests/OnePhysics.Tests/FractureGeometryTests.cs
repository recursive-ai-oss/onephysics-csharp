using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Destruction;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class FractureGeometryTests
{
    private static Piece Cube(double h = 1) => Piece.Box(new(-h, -h, -h), new(h, h, h));

    [Fact]
    public void ClippingACubeGivesTheAnalyticVolume()
    {
        var cube = Cube();
        var half = cube.Clip(new Vector3D(1, 0, 0), 0.5)!;
        half.Measure(out double v, out var c);
        Assert.Equal(2 * 2 * 1.5, v, 12);
        Assert.Equal(-0.25, c.X, 12);
        // An oblique cut through the centre halves it.
        var n = new Vector3D(1, 2, 3); n /= n.Length();
        var cut = cube.Clip(n, 0)!;
        cut.Measure(out double v2, out _);
        Assert.Equal(4.0, v2, 10);
        Assert.Same(cube, cube.Clip(new Vector3D(1, 0, 0), 5));
        Assert.Null(cube.Clip(new Vector3D(1, 0, 0), -5));
        Assert.Null(cube.Clip(new Vector3D(1, 0, 0), -1)); // touching only
    }

    [Theory]
    [InlineData(1, 8)]
    [InlineData(7, 24)]
    [InlineData(42, 64)]
    public void VoronoiCellsTileTheShapeExactlyAndAreConvex(ulong seed, int count)
    {
        var pts = new List<Vector3F>();
        var rng = new SeedRng(seed * 31 + 7);
        for (int i = 0; i < 40; i++) pts.Add(new Vector3F((float)(rng.NextDouble() * 4 - 2), (float)(rng.NextDouble() * 2 - 1), (float)(rng.NextDouble() * 3 - 1.5)));
        var piece = Piece.FromPoints3D(pts.ToArray())!;
        piece.Measure(out double total, out var centroid);
        var seeds = Fracturer.MakeSeeds([piece], count, FracturePattern.Uniform, seed, default, 0, 1, default);
        var cells = Fracturer.Cells(piece, seeds);
        double sum = 0; var csum = default(Vector3D);
        foreach (var cell in cells)
        {
            if (cell == null) continue;
            cell.Measure(out double v, out var c);
            sum += v; csum += c * v;
            // Convex: every vertex is inside or on every face plane.
            foreach (var f in cell.Faces)
            {
                var nrm = Piece.FaceNormal(f);
                foreach (var vert in cell.Vertices()) Assert.True(Vector3D.Dot(nrm, vert - f[0]) < 1e-9);
            }
        }
        Assert.Equal(total, sum, 9);
        var cc = csum / sum;
        Assert.True((cc - centroid).Length() < 1e-9);
        // No overlap: random sample points fall into exactly one cell (boundaries aside).
        var s = new SeedRng(99);
        piece.Bounds(out var lo, out var hi);
        int inside = 0;
        for (int i = 0; i < 400; i++)
        {
            var p = new Vector3D(lo.X + (hi.X - lo.X) * s.NextDouble(), lo.Y + (hi.Y - lo.Y) * s.NextDouble(), lo.Z + (hi.Z - lo.Z) * s.NextDouble());
            if (!piece.Contains(p, 0)) continue;
            inside++;
            int hits = 0;
            foreach (var cell in cells) if (cell != null && cell.Contains(p, 1e-9)) hits++;
            Assert.True(hits >= 1 && hits <= 2, "point in " + hits + " cells");
            int strict = 0;
            foreach (var cell in cells) if (cell != null && cell.Contains(p, -1e-9)) strict++;
            Assert.True(strict <= 1);
        }
        Assert.True(inside > 50);
    }

    [Fact]
    public void PlanarCellsTileAPolygon()
    {
        var piece = Piece.FromPoints2D([new(-2, -1, 0), new(2, -1, 0), new(3, 0.5f, 0), new(0, 2, 0), new(-2.5f, 1, 0)])!;
        piece.Measure(out double area, out _);
        var seeds = Fracturer.MakeSeeds([piece], 20, FracturePattern.Uniform, 5, default, 0, 1, default);
        var cells = Fracturer.Cells(piece, seeds);
        double sum = 0;
        foreach (var c in cells) if (c != null) { c.Measure(out double a, out _); sum += a; }
        Assert.Equal(area, sum, 10);
    }

    [Fact]
    public void ClusteredSeedsCrowdTheImpact()
    {
        var piece = Cube(2);
        var impact = new Vector3D(1.5, 1.5, 0);
        var seeds = Fracturer.MakeSeeds([piece], 60, FracturePattern.Clustered, 3, impact, 4, 3, default);
        var uniform = Fracturer.MakeSeeds([piece], 60, FracturePattern.Uniform, 3, impact, 4, 3, default);
        double Mean(Vector3D[] s) { double t = 0; foreach (var p in s) t += (p - impact).Length(); return t / s.Length; }
        Assert.True(Mean(seeds) < 0.6 * Mean(uniform));
        // Same seed, same seeds.
        var again = Fracturer.MakeSeeds([piece], 60, FracturePattern.Clustered, 3, impact, 4, 3, default);
        Assert.Equal(seeds, again);
    }

    [Fact]
    public void SharedFacesBetweenAdjacentCellsHaveTheCutArea()
    {
        var cube = Cube();
        var a = cube.Clip(new Vector3D(1, 0, 0), 0.25)!;
        var b = cube.Clip(new Vector3D(-1, 0, 0), -0.25)!;
        var face = Fracturer.Shared(a, b)!.Value;
        Assert.Equal(4.0, face.Measure, 9);
        Assert.Equal(1.0, face.Normal.X, 9);
        Assert.Equal(0.25, face.Centroid.X, 9);
        var far = cube.Clip(new Vector3D(-1, 0, 0), 0.5)!;
        Assert.Null(Fracturer.Shared(cube.Clip(new Vector3D(1, 0, 0), -0.5)!, cube.Clip(new Vector3D(-1, 0, 0), -0.75)!));
        Assert.NotNull(far);
    }
}
