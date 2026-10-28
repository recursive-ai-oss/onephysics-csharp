using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision2D;

public class ShapeTests
{
    private static void Near(double expected, double actual, double rel, string what = "")
    {
        double tol = rel * Math.Max(1.0, Math.Abs(expected));
        Assert.True(Math.Abs(expected - actual) <= tol, what + " expected " + expected + " got " + actual);
    }

    [Fact]
    public void CircleMass()
    {
        var s = Shape2D.CreateCircle(new Vec2(1, 2), 0.5f);
        MassData2D m = s.ComputeMass();
        Near(Math.PI * 0.25, m.Area, 1e-12);
        Near(0.5 * Math.PI * 0.25 * 0.25, m.InertiaAtCentroid, 1e-12);
        Assert.Equal(1.0, m.CentroidX);
        Near(m.InertiaAtCentroid + m.Area * 5.0, m.InertiaAbout(0, 0), 1e-12);
    }

    [Fact]
    public void BoxMass()
    {
        var s = Shape2D.CreateBox(1.5f, 0.5f);
        MassData2D m = s.ComputeMass();
        Near(3.0, m.Area, 1e-12);
        Near(3.0 * (9.0 + 1.0) / 12.0, m.InertiaAtCentroid, 1e-12);
        Near(0, m.CentroidX, 1e-12);
    }

    [Fact]
    public void TriangleMass()
    {
        // Right triangle with legs 3 and 4: area 6, centroid (1, 4/3), inertia about the centroid is area*(a^2+b^2)/18 = 6*25/18.
        var s = Shape2D.CreatePolygon(new[] { new Vec2(0, 0), new Vec2(3, 0), new Vec2(0, 4) });
        MassData2D m = s.ComputeMass();
        Near(6, m.Area, 1e-6);
        Near(1.0, m.CentroidX, 1e-6);
        Near(4.0 / 3.0, m.CentroidY, 1e-6);
        Near(6.0 * 25.0 / 18.0, m.InertiaAtCentroid, 1e-5);
    }

    [Fact]
    public void CapsuleMassMatchesClosedForm()
    {
        float r = 0.3f, len = 2f;
        var s = Shape2D.CreateCapsule(new Vec2(-1, 0), new Vec2(1, 0), r);
        MassData2D m = s.ComputeMass();
        double rr = r, L = len;
        double rect = 2 * rr * L, disc = Math.PI * rr * rr;
        Near(rect + disc, m.Area, 1e-6);
        // Rectangle about the centre, two half discs (each: own centroid inertia + parallel axis).
        double iRect = rect * (L * L + 4 * rr * rr) / 12.0;
        double hd = disc / 2, ownC = (Math.PI / 4 - 8 / (9 * Math.PI)) * Math.Pow(rr, 4);
        double off = L / 2 + 4 * rr / (3 * Math.PI);
        double expected = iRect + 2 * (ownC + hd * off * off);
        Near(expected, m.InertiaAtCentroid, 1e-6);
    }

    [Fact]
    public void RoundedBoxMassMatchesIntegration()
    {
        float hx = 0.7f, hy = 0.4f, r = 0.25f;
        var s = Shape2D.CreateBox(hx, hy, new Vec2(0.2f, -0.1f), Rot2.Identity, r);
        MassData2D m = s.ComputeMass();
        // Area is exact: rectangle, four strips, one disc.
        double area = 4.0 * hx * hy + 2.0 * r * (2.0 * hx + 2.0 * hy) + Math.PI * r * r;
        Near(area, m.Area, 1e-6);
        // Inertia by integration over a fine grid of the rounded box about its centre.
        int n = 1600;
        double x0 = -hx - r, x1 = hx + r, y0 = -hy - r, y1 = hy + r;
        double dx = (x1 - x0) / n, dy = (y1 - y0) / n, sum = 0;
        for (int i = 0; i < n; i++)
        {
            double x = x0 + (i + 0.5) * dx;
            for (int j = 0; j < n; j++)
            {
                double y = y0 + (j + 0.5) * dy;
                double ex = Math.Max(Math.Abs(x) - hx, 0), ey = Math.Max(Math.Abs(y) - hy, 0);
                if (ex * ex + ey * ey <= r * r) { sum += (x * x + y * y) * dx * dy; }
            }
        }

        Near(sum, m.InertiaAtCentroid, 2e-3);
        Near(0.2, m.CentroidX, 1e-6);
    }

    [Fact]
    public void MassFarFromTheOriginKeepsPrecision()
    {
        // A 1 x 1 box 100000 away: the centroid inertia must still be 1/6 to float precision of the construction; the shift is exact in double.
        var s = Shape2D.CreateBox(0.5f, 0.5f, new Vec2(100000f, -100000f), Rot2.Identity);
        MassData2D m = s.ComputeMass();
        Near(1.0, m.Area, 1e-5);
        Near(1.0 / 6.0, m.InertiaAtCentroid, 1e-4);
        Near(1.0 / 6.0 + 2e10, m.InertiaAbout(0, 0), 1e-6);
    }

    [Fact]
    public void RandomPolygonsMatchBruteForceIntegration()
    {
        var r = new TRng(11);
        for (int k = 0; k < 30; k++)
        {
            Shape2D s = TU.RandomPolygon(ref r);
            MassData2D m = s.ComputeMass();
            // Triangle fan about the centroid computed in double from scratch, independent of the implementation's reference vertex.
            var v = s.Vertices;
            double a = 0, cx = 0, cy = 0;
            for (int i = 0; i < v.Length; i++)
            {
                var p = v[i];
                var q = v[(i + 1) % v.Length];
                double c = (double)p.X * q.Y - (double)p.Y * q.X;
                a += c / 2;
                cx += ((double)p.X + q.X) * c / 6;
                cy += ((double)p.Y + q.Y) * c / 6;
            }

            cx /= a; cy /= a;
            double j2 = 0;
            for (int i = 0; i < v.Length; i++)
            {
                double px = v[i].X - cx, py = v[i].Y - cy, qx = v[(i + 1) % v.Length].X - cx, qy = v[(i + 1) % v.Length].Y - cy;
                double c = px * qy - py * qx;
                j2 += c / 12.0 * (px * px + px * qx + qx * qx + py * py + py * qy + qy * qy);
            }

            Near(a, m.Area, 1e-9);
            Near(j2, m.InertiaAtCentroid, 1e-8);
            Near(cx, m.CentroidX, 1e-9);
        }
    }

    [Fact]
    public void HostilePolygonInputsAreRejectedWithTypedErrors()
    {
        void Expect(ShapeError reason, Vec2[] pts, float radius = 0f)
        {
            var e = Assert.Throws<InvalidShapeException>(() => Shape2D.CreatePolygon(pts, radius));
            Assert.Equal(reason, e.Error);
            Assert.False(string.IsNullOrEmpty(e.Message));
        }

        Expect(ShapeError.TooFewPoints, Array.Empty<Vec2>());
        Expect(ShapeError.TooFewPoints, new[] { new Vec2(0, 0), new Vec2(1, 0) });
        Expect(ShapeError.Degenerate, new[] { new Vec2(0, 0), new Vec2(1, 1), new Vec2(2, 2), new Vec2(3, 3) });
        Expect(ShapeError.Degenerate, new[] { new Vec2(1, 1), new Vec2(1, 1), new Vec2(1, 1) });
        Expect(ShapeError.Degenerate, new[] { new Vec2(0, 0), new Vec2(1e-9f, 0), new Vec2(0, 1e-9f) });
        Expect(ShapeError.NotFinite, new[] { new Vec2(0, 0), new Vec2(float.NaN, 0), new Vec2(0, 1) });
        Expect(ShapeError.NotFinite, new[] { new Vec2(0, 0), new Vec2(float.PositiveInfinity, 0), new Vec2(0, 1) });
        Expect(ShapeError.OutOfRange, new[] { new Vec2(0, 0), new Vec2(2e6f, 0), new Vec2(0, 1) });
        Expect(ShapeError.OutOfRange, new[] { new Vec2(0, 0), new Vec2(1, 0), new Vec2(0, 1) }, -1f);
        Expect(ShapeError.NotFinite, new[] { new Vec2(0, 0), new Vec2(1, 0), new Vec2(0, 1) }, float.NaN);

        var many = new Vec2[9];
        for (int i = 0; i < 9; i++)
        {
            // Nine points on a circle of radius 1 (rational parametrisation): all on the hull.
            float t = -0.9f + 0.225f * i;
            many[i] = new Vec2((1 - t * t) / (1 + t * t), 2 * t / (1 + t * t));
        }

        Expect(ShapeError.TooManyVertices, many);
        Expect(ShapeError.TooManyPoints, new Vec2[70000]);
        Assert.Equal(ShapeError.NotFinite, Assert.Throws<InvalidShapeException>(() => Shape2D.CreateCircle(new Vec2(0, 0), float.NaN)).Error);
        Assert.Equal(ShapeError.OutOfRange, Assert.Throws<InvalidShapeException>(() => Shape2D.CreateCircle(new Vec2(0, 0), 0f)).Error);
        Assert.Equal(ShapeError.Degenerate, Assert.Throws<InvalidShapeException>(() => Shape2D.CreateCapsule(new Vec2(0, 0), new Vec2(0, 0), 1f)).Error);
        Assert.Equal(ShapeError.TooFewPoints, Assert.Throws<InvalidShapeException>(() => new Chain2D(new[] { new Vec2(0, 0) })).Error);
        Assert.Equal(ShapeError.Degenerate, Assert.Throws<InvalidShapeException>(() => new Chain2D(new[] { new Vec2(0, 0), new Vec2(0, 0), new Vec2(1, 0) })).Error);
    }

    [Fact]
    public void DuplicatesAndInteriorPointsAreDropped()
    {
        var s = Shape2D.CreatePolygon(new[]
        {
            new Vec2(0, 0), new Vec2(1, 0), new Vec2(1, 0), new Vec2(2, 0), new Vec2(2, 2), new Vec2(1, 1), new Vec2(0, 2), new Vec2(0, 1), new Vec2(0.5f, 0.5f),
        });
        Assert.Equal(4, s.Count);
        Assert.Equal(4.0, s.ComputeMass().Area, 6);
    }

    [Fact]
    public void RandomHostileHullsAreConvexAndCounterClockwise()
    {
        var r = new TRng(5);
        int ok = 0;
        for (int k = 0; k < 20000; k++)
        {
            int n = r.Int(14);
            var pts = new Vec2[n];
            float scale = r.Int(4) switch { 0 => 1e-4f, 1 => 1f, 2 => 1e3f, _ => 1e5f };
            for (int i = 0; i < n; i++)
            {
                pts[i] = r.Int(10) == 0 && i > 0 ? pts[r.Int(i)] : r.Int(8) == 0 ? new Vec2(Snap(r.Range(-1, 1)) * scale, Snap(r.Range(-1, 1)) * scale) : r.Point(scale);
            }

            Shape2D s;
            try { s = Shape2D.CreatePolygon(pts); }
            catch (InvalidShapeException) { continue; }
            ok++;
            var v = s.Vertices;
            Assert.InRange(v.Length, 3, 8);
            for (int i = 0; i < v.Length; i++)
            {
                Vec2 a = v[i], b = v[(i + 1) % v.Length], c = v[(i + 2) % v.Length];
                Assert.True(Vec2.Cross(b - a, c - b) > 0, "strictly convex, counter-clockwise");
                Vec2 nrm = s.Normals[i];
                Assert.True(Math.Abs(nrm.Length - 1) < 1e-5);
                Assert.True(Math.Abs(Vec2.Dot(nrm, b - a)) <= 1e-4f * (b - a).Length + 1e-12f);
            }

            // Every input point lies inside the hull (within the welding tolerance).
            foreach (Vec2 p in pts)
            {
                for (int i = 0; i < v.Length; i++)
                {
                    Assert.True(Vec2.Dot(s.Normals[i], p - v[i]) <= 1e-4f * scale);
                }
            }
        }

        Assert.True(ok > 5000);
    }

    private static float Snap(float x) => MathF.Round(x * 4) / 4;

    [Fact]
    public void AabbsContainTheShape()
    {
        var r = new TRng(3);
        for (int k = 0; k < 300; k++)
        {
            Shape2D s = TU.RandomShape(ref r);
            Transform2 xf = TU.Pose(ref r, 3f);
            Aabb2 bb = s.ComputeAabb(xf);
            for (int i = 0; i < 50; i++)
            {
                // Points on the rounded core's surface in random directions.
                Vec2 c = xf.Apply(s.Vertices[r.Int(s.Count)]);
                Vec2 d = new Vec2(r.Range(-1, 1), r.Range(-1, 1)).Normalized();
                Vec2 p = c + s.Radius * d;
                Assert.True(p.X >= bb.Lower.X - 1e-5f && p.X <= bb.Upper.X + 1e-5f && p.Y >= bb.Lower.Y - 1e-5f && p.Y <= bb.Upper.Y + 1e-5f);
            }
        }
    }

    [Fact]
    public void ChainSegmentsCarryTheirNeighbours()
    {
        var c = new Chain2D(new[] { new Vec2(0, 0), new Vec2(1, 0), new Vec2(2, 1), new Vec2(3, 1) });
        Assert.Equal(3, c.SegmentCount);
        Shape2D s0 = c.GetSegment(0), s1 = c.GetSegment(1), s2 = c.GetSegment(2);
        Assert.False(s0.HasGhost1);
        Assert.True(s0.HasGhost2);
        Assert.Equal(new Vec2(2, 1).X, s0.Ghost2.X);
        Assert.True(s1.HasGhost1 && s1.HasGhost2);
        Assert.False(s2.HasGhost2);
        var loop = new Chain2D(new[] { new Vec2(0, 0), new Vec2(1, 0), new Vec2(1, 1) }, loop: true);
        Assert.Equal(3, loop.SegmentCount);
        Assert.True(loop.GetSegment(0).HasGhost1 && loop.GetSegment(2).HasGhost2);
        Assert.Throws<ArgumentOutOfRangeException>(() => c.GetSegment(3));
    }
}
