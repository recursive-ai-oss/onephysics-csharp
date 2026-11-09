using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using Xunit;

namespace RecursiveAI.OnePhysics.Tools.Tests;

public class PolygonTests
{
    private static double Area(Vector2F[] p)
    {
        double a = 0;
        for (int i = 0; i < p.Length; i++) { var q = p[(i + 1) % p.Length]; a += (double)p[i].X * q.Y - (double)q.X * p[i].Y; }
        return 0.5 * a;
    }

    private static bool Convex(Vector2F[] p)
    {
        for (int i = 0; i < p.Length; i++)
        {
            var a = p[i]; var b = p[(i + 1) % p.Length]; var c = p[(i + 2) % p.Length];
            if ((double)(b.X - a.X) * (c.Y - b.Y) - (double)(b.Y - a.Y) * (c.X - b.X) <= 0) { return false; }
        }

        return true;
    }

    private static bool Inside(Vector2F[] poly, double x, double y)
    {
        bool inside = false;
        for (int i = 0, j = poly.Length - 1; i < poly.Length; j = i++)
        {
            if ((poly[i].Y > y) != (poly[j].Y > y) && x < (double)(poly[j].X - poly[i].X) * (y - poly[i].Y) / (poly[j].Y - poly[i].Y) + poly[i].X) { inside = !inside; }
        }

        return inside;
    }

    private static void CheckTiling(Vector2F[] outline, Vector2F[][]? holes, Vector2F[][] pieces, int maxVerts)
    {
        double expected = Math.Abs(Area(outline));
        if (holes != null) { foreach (var h in holes) { expected -= Math.Abs(Area(h)); } }
        double sum = 0;
        foreach (var p in pieces)
        {
            Assert.True(Convex(p), "piece not strictly convex counter-clockwise");
            Assert.True(p.Length <= maxVerts);
            sum += Area(p);
        }

        Assert.Equal(expected, sum, 6);
        // No overlap and full coverage: sample points are in exactly one piece when they are in the polygon, none otherwise.
        var rnd = new Random(3);
        double x0 = double.MaxValue, x1 = double.MinValue, y0 = double.MaxValue, y1 = double.MinValue;
        foreach (var v in outline) { x0 = Math.Min(x0, v.X); x1 = Math.Max(x1, v.X); y0 = Math.Min(y0, v.Y); y1 = Math.Max(y1, v.Y); }
        for (int s = 0; s < 4000; s++)
        {
            double x = x0 + rnd.NextDouble() * (x1 - x0), y = y0 + rnd.NextDouble() * (y1 - y0);
            bool want = Inside(outline, x, y);
            if (want && holes != null) { foreach (var h in holes) { if (Inside(h, x, y)) { want = false; } } }
            int count = 0;
            foreach (var p in pieces) { if (Inside(p, x, y)) { count++; } }
            Assert.Equal(want ? 1 : 0, count);
        }
    }

    private static Vector2F[] Star(int n, float rOut, float rIn)
    {
        var p = new Vector2F[2 * n];
        for (int i = 0; i < 2 * n; i++) { double a = Math.PI * i / n; double r = (i & 1) == 0 ? rOut : rIn; p[i] = new Vector2F((float)(r * Math.Cos(a)), (float)(r * Math.Sin(a))); }
        return p;
    }

    private static Vector2F[] Rect(float x0, float y0, float x1, float y1) => new[] { new Vector2F(x0, y0), new Vector2F(x1, y0), new Vector2F(x1, y1), new Vector2F(x0, y1) };

    [Fact]
    public void ConvexPolygonStaysOne()
    {
        var sq = Rect(0, 0, 2, 3);
        var r = PolygonDecomposer.Decompose(sq);
        Assert.Single(r);
        CheckTiling(sq, null, r, 8);
    }

    [Fact]
    public void StarAndLShape()
    {
        var star = Star(7, 2f, 0.8f);
        foreach (int max in new[] { 3, 4, 8 })
        {
            var r = PolygonDecomposer.Decompose(star, new PolygonDecompositionOptions { MaxVerticesPerPiece = max });
            CheckTiling(star, null, r, max);
        }

        var l = new[] { new Vector2F(0, 0), new Vector2F(2, 0), new Vector2F(2, 1), new Vector2F(1, 1), new Vector2F(1, 2), new Vector2F(0, 2) };
        var rl = PolygonDecomposer.Decompose(l);
        Assert.Equal(2, rl.Length);
        CheckTiling(l, null, rl, 8);
        // Clockwise input gives the same area.
        var rev = (Vector2F[])l.Clone(); Array.Reverse(rev);
        CheckTiling(rev, null, PolygonDecomposer.Decompose(rev), 8);
    }

    [Fact]
    public void HolesAreRespected()
    {
        var outer = Rect(0, 0, 10, 10);
        var holes = new[] { Rect(2, 2, 4, 4), Rect(6, 5, 8, 9), Star(5, 1.2f, 0.5f) };
        for (int i = 0; i < holes[2].Length; i++) { holes[2][i] = holes[2][i] + new Vector2F(5, 2.5f); }
        // hole 2 must not hit hole 1: star spans x 3.8-6.2, y 1.3-3.7 vs rect (6,5)-(8,9): disjoint; vs rect (2,2)-(4,4): overlaps -> move it
        for (int i = 0; i < holes[2].Length; i++) { holes[2][i] = holes[2][i] + new Vector2F(1, -0.5f); }
        var r = PolygonDecomposer.Decompose(outer, holes);
        CheckTiling(outer, holes, r, 8);
        var again = PolygonDecomposer.Decompose(outer, holes);
        Assert.Equal(r.Length, again.Length);
        for (int i = 0; i < r.Length; i++) { Assert.Equal(r[i], again[i]); }
    }

    [Fact]
    public void RingWithManyPieces()
    {
        var outer = new Vector2F[40]; var hole = new Vector2F[40];
        for (int i = 0; i < 40; i++) { double a = 2 * Math.PI * i / 40; outer[i] = new Vector2F((float)(5 * Math.Cos(a)), (float)(5 * Math.Sin(a))); hole[i] = new Vector2F((float)(3 * Math.Cos(a)), (float)(3 * Math.Sin(a))); }
        var r = PolygonDecomposer.Decompose(outer, new[] { hole });
        CheckTiling(outer, new[] { hole }, r, 8);
    }

    [Fact]
    public void HostileInputsGetTypedErrors()
    {
        GeometryToolError E(Action a) => Assert.Throws<GeometryToolException>(a).Error;
        Assert.Equal(GeometryToolError.Empty, E(() => PolygonDecomposer.Decompose(new Vector2F[2])));
        Assert.Equal(GeometryToolError.NonFinite, E(() => PolygonDecomposer.Decompose(new[] { new Vector2F(0, 0), new Vector2F(float.NaN, 0), new Vector2F(1, 1) })));
        Assert.Equal(GeometryToolError.OutOfRange, E(() => PolygonDecomposer.Decompose(new[] { new Vector2F(0, 0), new Vector2F(1e9f, 0), new Vector2F(1, 1) })));
        Assert.Equal(GeometryToolError.Degenerate, E(() => PolygonDecomposer.Decompose(new[] { new Vector2F(0, 0), new Vector2F(1, 1), new Vector2F(2, 2) })));
        Assert.Equal(GeometryToolError.Degenerate, E(() => PolygonDecomposer.Decompose(new[] { new Vector2F(1, 1), new Vector2F(1, 1), new Vector2F(1, 1) })));
        // Bow tie.
        Assert.Equal(GeometryToolError.SelfIntersecting, E(() => PolygonDecomposer.Decompose(new[] { new Vector2F(0, 0), new Vector2F(3, 2), new Vector2F(3, 0), new Vector2F(0, 1) })));
        // Hole outside, hole touching the outline, hole in a hole, overlapping holes.
        var outer = Rect(0, 0, 10, 10);
        Assert.Equal(GeometryToolError.SelfIntersecting, E(() => PolygonDecomposer.Decompose(outer, new[] { Rect(20, 20, 21, 21) })));
        Assert.Equal(GeometryToolError.SelfIntersecting, E(() => PolygonDecomposer.Decompose(outer, new[] { Rect(0, 2, 2, 4) })));
        Assert.Equal(GeometryToolError.SelfIntersecting, E(() => PolygonDecomposer.Decompose(outer, new[] { Rect(2, 2, 6, 6), Rect(3, 3, 4, 4) })));
        Assert.Equal(GeometryToolError.SelfIntersecting, E(() => PolygonDecomposer.Decompose(outer, new[] { Rect(2, 2, 6, 6), Rect(5, 5, 8, 8) })));
        Assert.Equal(GeometryToolError.InvalidOptions, E(() => PolygonDecomposer.Decompose(outer, null, new PolygonDecompositionOptions { MaxVerticesPerPiece = 2 })));
        Assert.Equal(GeometryToolError.Empty, E(() => PolygonDecomposer.Decompose(outer, new Vector2F[][] { null! })));
        // Repeated closing point and duplicate points are tolerated; a spike (zero-width) is not a valid polygon.
        var dup = new[] { new Vector2F(0, 0), new Vector2F(0, 0), new Vector2F(1, 0), new Vector2F(1, 1), new Vector2F(0, 1), new Vector2F(0, 0) };
        Assert.Single(PolygonDecomposer.Decompose(dup));
        // Tiny and huge scales.
        foreach (float s in new[] { 1e-6f, 1e5f })
        {
            var sq = Rect(0, 0, s, 2 * s);
            Assert.Single(PolygonDecomposer.Decompose(sq));
        }
    }

    [Fact]
    public void RandomSimplePolygonsTile()
    {
        var rnd = new Random(11);
        for (int t = 0; t < 40; t++)
        {
            int n = 5 + rnd.Next(25);
            // Star-shaped polygons about the origin are simple by construction.
            var p = new Vector2F[n];
            for (int i = 0; i < n; i++) { double a = 2 * Math.PI * i / n; double r = 1 + 4 * rnd.NextDouble(); p[i] = new Vector2F((float)(r * Math.Cos(a)), (float)(r * Math.Sin(a))); }
            CheckTiling(p, null, PolygonDecomposer.Decompose(p), 8);
            CheckTiling(p, null, PolygonDecomposer.Decompose(p, new PolygonDecompositionOptions { MaxVerticesPerPiece = 3 }), 3);
        }
    }
}
