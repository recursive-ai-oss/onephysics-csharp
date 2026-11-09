using System;
using RecursiveAI.OneMath;
using Xunit;

namespace RecursiveAI.OnePhysics.Tools.Tests;

public class SpriteTests
{
    private static double Hyp(double a, double b) => Math.Sqrt(a * a + b * b);

    private static byte[] Mask(int w, int h, Func<double, double, bool> solid)
    {
        var m = new byte[w * h];
        for (int y = 0; y < h; y++) { for (int x = 0; x < w; x++) { m[y * w + x] = solid(x + 0.5, y + 0.5) ? (byte)255 : (byte)0; } }
        return m;
    }

    private static double Area(Vector2F[] p)
    {
        double a = 0;
        for (int i = 0; i < p.Length; i++) { var q = p[(i + 1) % p.Length]; a += (double)p[i].X * q.Y - (double)q.X * p[i].Y; }
        return 0.5 * a;
    }

    private static double DistToPolygon(Vector2F[] p, double x, double y)
    {
        double best = double.MaxValue;
        for (int i = 0; i < p.Length; i++)
        {
            var a = p[i]; var b = p[(i + 1) % p.Length];
            double dx = b.X - a.X, dy = b.Y - a.Y, l2 = dx * dx + dy * dy;
            double t = l2 == 0 ? 0 : Math.Clamp(((x - a.X) * dx + (y - a.Y) * dy) / l2, 0, 1);
            best = Math.Min(best, Hyp(a.X + t * dx - x, a.Y + t * dy - y));
        }

        return best;
    }

    [Fact]
    public void CircleIsOneIslandWithinTolerance()
    {
        const int n = 64;
        var m = Mask(n, n, (x, y) => (x - 32) * (x - 32) + (y - 32) * (y - 32) <= 20 * 20);
        var isl = SpriteOutliner.Trace(m, n, n, new SpriteOutlineOptions { Tolerance = 1f });
        Assert.Single(isl);
        Assert.Empty(isl[0].Holes);
        Assert.True(Area(isl[0].Outer) > 0);
        // Every outline point lies on the circle within the pixel tolerance plus half a pixel of rasterisation.
        foreach (var v in isl[0].Outer) { Assert.InRange(Hyp(v.X - 32, (n - v.Y) - 32), 20 - 1.5, 20 + 1.5); }
        Assert.InRange(Area(isl[0].Outer) / (Math.PI * 400), 0.95, 1.05);
        // The simplified outline is far smaller than the raw one.
        var raw = SpriteOutliner.Trace(m, n, n, new SpriteOutlineOptions { Tolerance = 0f });
        Assert.True(isl[0].Outer.Length < raw[0].Outer.Length / 2);
        // Every raw point is within the tolerance of the simplified outline.
        foreach (var v in raw[0].Outer) { Assert.True(DistToPolygon(isl[0].Outer, v.X, v.Y) <= 1.0001); }
    }

    [Fact]
    public void RingHasAHoleAndDecomposes()
    {
        const int n = 80;
        var m = Mask(n, n, (x, y) => { double r = Hyp(x - 40, y - 40); return r <= 30 && r >= 14; });
        var isl = SpriteOutliner.Trace(m, n, n);
        Assert.Single(isl);
        Assert.Single(isl[0].Holes);
        Assert.True(Area(isl[0].Outer) > 0);
        Assert.True(Area(isl[0].Holes[0]) < 0);
        double expected = Math.PI * (900 - 196);
        var pieces = isl[0].ToConvexPieces();
        double sum = 0;
        foreach (var p in pieces) { sum += Area(p); Assert.True(Area(p) > 0); }
        Assert.InRange(sum / expected, 0.95, 1.05);
        Assert.Equal(Math.Abs(Area(isl[0].Outer)) - Math.Abs(Area(isl[0].Holes[0])), sum, 5);
    }

    [Fact]
    public void TwoIslandsAndAnIslandInsideAHole()
    {
        const int n = 90;
        var m = Mask(n, n, (x, y) =>
            (x > 3 && x < 25 && y > 3 && y < 25) || (x > 50 && x < 80 && y > 50 && y < 80 && !(x > 58 && x < 72 && y > 58 && y < 72)) || (x > 63 && x < 67 && y > 63 && y < 67));
        var isl = SpriteOutliner.Trace(m, n, n);
        Assert.Equal(3, isl.Length);
        int holes = 0;
        foreach (var i in isl) { holes += i.Holes.Length; }
        Assert.Equal(1, holes);
        var all = SpriteOutliner.ToConvexPieces(m, n, n);
        Assert.True(all.Length >= 3);
    }

    [Fact]
    public void NoisyBlobStillDecomposes()
    {
        const int n = 96;
        var rnd = new Random(5);
        var noise = new double[n * n];
        for (int i = 0; i < noise.Length; i++) { noise[i] = rnd.NextDouble(); }
        var m = Mask(n, n, (x, y) =>
        {
            double r = Hyp(x - 48, y - 48), a = Math.Atan2(y - 48, x - 48);
            double edge = 28 + 6 * Math.Sin(5 * a) + 5 * noise[(int)y * n + (int)x];
            return r < edge;
        });
        foreach (float tol in new[] { 0f, 0.5f, 1f, 2f })
        {
            var pieces = SpriteOutliner.ToConvexPieces(m, n, n, new SpriteOutlineOptions { Tolerance = tol });
            Assert.NotEmpty(pieces);
            foreach (var p in pieces) { Assert.True(Area(p) > 0); Assert.True(p.Length <= 8); }
        }
    }

    [Fact]
    public void PlacementOptionsAndEmptyMasks()
    {
        var m = Mask(10, 10, (x, y) => x > 2 && x < 6 && y > 2 && y < 5);
        var up = SpriteOutliner.Trace(m, 10, 10, new SpriteOutlineOptions { PixelSize = 0.5f, Origin = new Vector2F(1, 2) });
        float minY = float.MaxValue, maxY = float.MinValue;
        foreach (var v in up[0].Outer) { minY = Math.Min(minY, v.Y); maxY = Math.Max(maxY, v.Y); }
        // Rows 3..4 from the top of a 10-row image sit at y = 5..7 pixels from the bottom (half-pixel sampling shifts the edge by up to a pixel).
        Assert.InRange(minY, 2 + 0.5 * 4.4, 2 + 0.5 * 5.6);
        Assert.Empty(SpriteOutliner.Trace(new byte[16], 4, 4));
        var full = SpriteOutliner.Trace(new byte[] { 255, 255, 255, 255 }, 2, 2);
        Assert.Single(full);
        Assert.Equal(1, PolygonDecomposerCount(full[0]));
    }

    private static int PolygonDecomposerCount(SpriteIsland i) => i.ToConvexPieces().Length;

    [Fact]
    public void HostileInputsGetTypedErrors()
    {
        GeometryToolError E(Action a) => Assert.Throws<GeometryToolException>(a).Error;
        Assert.Equal(GeometryToolError.OutOfRange, E(() => SpriteOutliner.Trace(new byte[4], 0, 4)));
        Assert.Equal(GeometryToolError.OutOfRange, E(() => SpriteOutliner.Trace(new byte[4], 100000, 1)));
        Assert.Equal(GeometryToolError.BadIndex, E(() => SpriteOutliner.Trace(new byte[5], 2, 2)));
        Assert.Equal(GeometryToolError.InvalidOptions, E(() => SpriteOutliner.Trace(new byte[4], 2, 2, new SpriteOutlineOptions { AlphaThreshold = 0 })));
        Assert.Equal(GeometryToolError.InvalidOptions, E(() => SpriteOutliner.Trace(new byte[4], 2, 2, new SpriteOutlineOptions { Tolerance = float.NaN })));
        Assert.Equal(GeometryToolError.InvalidOptions, E(() => SpriteOutliner.Trace(new byte[4], 2, 2, new SpriteOutlineOptions { PixelSize = 0 })));
        Assert.Equal(GeometryToolError.InvalidOptions, E(() => SpriteOutliner.Trace(new byte[4], 2, 2, new SpriteOutlineOptions { Origin = new Vector2F(float.PositiveInfinity, 0) })));
    }
}
