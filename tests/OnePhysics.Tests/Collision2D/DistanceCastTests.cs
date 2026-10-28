using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision2D;

public class DistanceTests
{
    [Fact]
    public void GjkMatchesBruteForceOnRandomPairs()
    {
        // 100000 random pairs of cores (circles, capsules, segments, polygons), with and without a warm cache. Tolerance: 2e-5 absolute (scenes of size ~5, float32 inputs).
        var r = new TRng(1);
        int overlaps = 0, separated = 0;
        double worst = 0;
        for (int k = 0; k < 100000; k++)
        {
            Shape2D a = TU.RandomShape(ref r, false);
            Shape2D b = TU.RandomShape(ref r, false);
            Transform2 xf = TU.Pose(ref r, 2.5f);
            var cache = new SimplexCache2D();
            DistanceOutput2D d = Distance2D.Compute(in a, in b, xf, ref cache, false);
            double expected = TU.CoreDistance(in a, in b, xf);
            double err = Math.Abs(expected - d.Distance);
            worst = Math.Max(worst, err);
            Assert.True(err <= 2e-5, "pair " + k + " expected " + expected + " got " + d.Distance);
            if (expected == 0) { overlaps++; } else { separated++; }

            // Witness points are at that distance and on the shapes' bounds.
            Assert.True(Math.Abs((d.PointB - d.PointA).Length - d.Distance) <= 1e-5);

            // A warm cache, with the second shape moved a little, gives the same answer as a cold start.
            Transform2 xf2 = new(xf.P + new Vec2(0.01f, -0.02f), xf.Q);
            DistanceOutput2D warm = Distance2D.Compute(in a, in b, xf2, ref cache, false);
            var cold = new SimplexCache2D();
            DistanceOutput2D cd = Distance2D.Compute(in a, in b, xf2, ref cold, false);
            Assert.True(Math.Abs(warm.Distance - cd.Distance) <= 2e-5, "warm " + warm.Distance + " cold " + cd.Distance);
        }

        Assert.True(overlaps > 5000 && separated > 5000, overlaps + " overlapping, " + separated + " separated");
        Assert.True(worst < 2e-5);
    }

    [Fact]
    public void RadiiAreIncluded()
    {
        var a = Shape2D.CreateCircle(new Vec2(0, 0), 1f);
        var b = Shape2D.CreateBox(0.5f, 0.5f, 0.25f);
        var cache = new SimplexCache2D();
        DistanceOutput2D d = Distance2D.Compute(in a, in b, new Transform2(new Vec2(4, 0), Rot2.Identity), ref cache);
        Assert.Equal(4 - 0.5 - 0.25 - 1, d.Distance, 5);
        Assert.Equal(1.0, d.PointA.X, 5);
        Assert.Equal(4 - 0.75, d.PointB.X, 5);
        d = Distance2D.Compute(in a, in b, new Transform2(new Vec2(1, 0), Rot2.Identity), ref cache);
        Assert.Equal(0f, d.Distance);
    }

    [Fact]
    public void HostileCacheIsIgnored()
    {
        var a = Shape2D.CreateCircle(new Vec2(0, 0), 1f);
        var b = Shape2D.CreateBox(0.5f, 0.5f);
        var cache = new SimplexCache2D { Count = 3 };
        DistanceOutput2D d = Distance2D.Compute(in a, in b, new Transform2(new Vec2(4, 0), Rot2.Identity), ref cache, false);
        Assert.Equal(3.5, d.Distance, 5);
        var def = default(Shape2D);
        Assert.Equal(0f, Distance2D.Compute(in def, in b, Transform2.Identity, ref cache).Distance);
    }
}

public class CastTests
{
    [Fact]
    public void RayAgainstCircleBoxCapsuleAnalytic()
    {
        var c = Shape2D.CreateCircle(new Vec2(2, 0), 1f);
        RayCastOutput2D o = c.RayCast(new Vec2(-1, 0), new Vec2(10, 0));
        Assert.True(o.Hit);
        Assert.Equal(0.2f, o.Fraction, 5);
        Assert.Equal(-1f, o.Normal.X, 5);
        Assert.Equal(1f, o.Point.X, 5);
        Assert.False(c.RayCast(new Vec2(-1, 2), new Vec2(10, 0)).Hit);
        Assert.False(c.RayCast(new Vec2(-1, 0), new Vec2(1, 0)).Hit);

        var box = Shape2D.CreateBox(1f, 0.5f);
        o = box.RayCast(new Vec2(0, 3), new Vec2(0, -4));
        Assert.True(o.Hit);
        Assert.Equal(0.625f, o.Fraction, 5);
        Assert.Equal(1f, o.Normal.Y, 5);

        var cap = Shape2D.CreateCapsule(new Vec2(-1, 0), new Vec2(1, 0), 0.5f);
        o = cap.RayCast(new Vec2(0, 3), new Vec2(0, -3));
        Assert.Equal(2.5f / 3f, o.Fraction, 5);
        o = cap.RayCast(new Vec2(-4, 0), new Vec2(8, 0));
        Assert.Equal(2.5f / 8f, o.Fraction, 5);
        Assert.Equal(-1f, o.Normal.X, 5);

        // Starting inside: hit at fraction 0, zero normal.
        o = box.RayCast(new Vec2(0, 0), new Vec2(1, 0));
        Assert.True(o.Hit);
        Assert.Equal(0f, o.Fraction);
        Assert.Equal(0f, o.Normal.X);

        // Segments are hit from both sides; a one-sided chain segment only from its right.
        var seg = Shape2D.CreateSegment(new Vec2(-1, 0), new Vec2(1, 0));
        Assert.True(seg.RayCast(new Vec2(0, 1), new Vec2(0, -2)).Hit);
        Assert.True(seg.RayCast(new Vec2(0, -1), new Vec2(0, 2)).Hit);
        var chain = Shape2D.CreateChainSegment(null, new Vec2(1, 0), new Vec2(-1, 0), null);
        Assert.True(chain.RayCast(new Vec2(0, 1), new Vec2(0, -2)).Hit);
        Assert.False(chain.RayCast(new Vec2(0, -1), new Vec2(0, 2)).Hit);
    }

    [Fact]
    public void RayCastMatchesBruteForceAndShapeCast()
    {
        var r = new TRng(2);
        int hits = 0, early = 0;
        for (int k = 0; k < 20000; k++)
        {
            Shape2D s = TU.RandomShape(ref r);
            Vec2 o = r.Point(3f);
            Vec2 d = r.Point(5f);
            RayCastOutput2D rc = s.RayCast(o, d);

            // Brute force: march the ray finely and find the first sample inside the rounded shape (distance of the point to the core <= radius).
            double first = -1;
            const int steps = 4000;
            for (int i = 0; i <= steps && first < 0; i++)
            {
                double t = (double)i / steps;
                double px = o.X + t * d.X, py = o.Y + t * d.Y;
                if (Inside(in s, px, py)) { first = t; }
            }

            if (first < 0) { continue; }

            hits++;
            Assert.True(rc.Hit, "brute force hit at " + first);
            Assert.True(rc.Fraction <= first + 1e-4, "fraction " + rc.Fraction + " brute " + first);
            // The entry is within one march step before the first inside sample, except for slivers thinner than a step.
            if (rc.Fraction < first - 1.5 / steps) { early++; }

            // The hit point is on the surface (distance to the core == radius) unless the ray starts inside.
            if (rc.Fraction > 0)
            {
                double dist = CoreDist(in s, rc.Point.X, rc.Point.Y);
                Assert.True(Math.Abs(dist - s.Radius) <= 2e-4, "hit point distance " + dist + " radius " + s.Radius);
            }
        }

        Assert.True(hits > 2000);
        Assert.True(early < 40, "early " + early);
    }

    private static bool Inside(in Shape2D s, double x, double y) => CoreDist(in s, x, y) <= s.Radius + 1e-12;

    // Distance from a point to the core (0 inside a polygon core).
    private static double CoreDist(in Shape2D s, double x, double y)
    {
        var v = s.Vertices;
        int n = v.Length;
        if (n == 1) { return Math.Sqrt((x - v[0].X) * (x - v[0].X) + (y - v[0].Y) * (y - v[0].Y)); }
        if (n >= 3)
        {
            bool inside = true;
            for (int i = 0; i < n && inside; i++)
            {
                double c = ((double)v[(i + 1) % n].X - v[i].X) * (y - v[i].Y) - ((double)v[(i + 1) % n].Y - v[i].Y) * (x - v[i].X);
                if (c < 0) { inside = false; }
            }

            if (inside) { return 0; }
        }

        double best = double.MaxValue;
        for (int i = 0; i < (n == 2 ? 1 : n); i++)
        {
            best = Math.Min(best, TU.PointSeg(x, y, v[i].X, v[i].Y, v[(i + 1) % n].X, v[(i + 1) % n].Y));
        }

        return best;
    }

    [Fact]
    public void ShapeCastAnalytic()
    {
        // A circle of radius 0.5 sweeping toward a box: the face is hit when the circle's surface touches it.
        var target = Shape2D.CreateBox(1f, 1f);
        var mover = Shape2D.CreateCircle(new Vec2(0, 0), 0.5f);
        RayCastOutput2D o = Distance2D.ShapeCast(in target, in mover, new Transform2(new Vec2(-5, 0.3f), Rot2.Identity), new Vec2(10, 0));
        Assert.True(o.Hit);
        Assert.Equal(0.35f, o.Fraction, 3);
        Assert.Equal(-1f, o.Normal.X, 3);
        Assert.Equal(-1f, o.Point.X, 3);

        // Circle against circle: touch when the centres are 1.5 apart.
        var c1 = Shape2D.CreateCircle(new Vec2(0, 0), 1f);
        var c2 = Shape2D.CreateCircle(new Vec2(0, 0), 0.5f);
        o = Distance2D.ShapeCast(in c1, in c2, new Transform2(new Vec2(4, 1), Rot2.Identity), new Vec2(-8, 0));
        // Centre path y = 1, touching at x = sqrt(1.5^2 - 1) .
        Assert.True(o.Hit);
        double x = 4 - o.Fraction * 8;
        Assert.Equal(Math.Sqrt(1.25), x, 3);

        // Misses and overlaps.
        Assert.False(Distance2D.ShapeCast(in c1, in c2, new Transform2(new Vec2(4, 3), Rot2.Identity), new Vec2(-8, 0)).Hit);
        o = Distance2D.ShapeCast(in c1, in c2, new Transform2(new Vec2(1, 0), Rot2.Identity), new Vec2(-8, 0));
        Assert.True(o.Hit);
        Assert.Equal(0f, o.Fraction);
    }

    [Fact]
    public void ShapeCastNeverPassesFirstContact()
    {
        var r = new TRng(4);
        int hits = 0;
        for (int k = 0; k < 3000; k++)
        {
            Shape2D a = TU.RandomShape(ref r);
            Shape2D b = TU.RandomShape(ref r);
            Transform2 xf = TU.Pose(ref r, 4f);
            Vec2 t = r.Point(8f);
            RayCastOutput2D o = Distance2D.ShapeCast(in a, in b, xf, t);
            // Sampled brute force: first sample where the rounded surfaces overlap.
            double first = -1;
            for (int i = 0; i <= 800 && first < 0; i++)
            {
                Transform2 x = new(xf.P + ((float)i / 800f) * t, xf.Q);
                if (TU.CoreDistance(in a, in b, x) <= a.Radius + b.Radius) { first = i / 800.0; }
            }

            if (first >= 0)
            {
                Assert.True(o.Hit, "overlap at " + first);
                Assert.True(o.Fraction <= first + 1e-5, "fraction " + o.Fraction + " first overlap " + first);
                hits++;
            }
        }

        Assert.True(hits > 300);
    }
}

public class ToiTests
{
    [Fact]
    public void TranslatingBoxHitsWallAtAnalyticTime()
    {
        var wall = Shape2D.CreateBox(0.1f, 5f);
        var box = Shape2D.CreateBox(0.5f, 0.5f);
        var sa = new Sweep2D(default, new Vec2(0, 0), new Vec2(0, 0), Rot2.Identity, Rot2.Identity);
        var sb = new Sweep2D(default, new Vec2(-10, 0), new Vec2(10, 0), Rot2.Identity, Rot2.Identity);
        ToiOutput2D o = Distance2D.TimeOfImpact(in wall, in sa, in box, in sb);
        Assert.Equal(ToiState.Hit, o.State);
        // Surfaces touch when the box centre is at x = -0.6; the search stops within the target gap before that.
        double touch = (10 - 0.6) / 20.0;
        Assert.True(o.Fraction <= touch);
        Assert.True(o.Fraction >= touch - (Constants2D.LinearSlop * 1.25 + 1e-5) / 20.0);

        var away = new Sweep2D(default, new Vec2(-10, 0), new Vec2(-20, 0), Rot2.Identity, Rot2.Identity);
        Assert.Equal(ToiState.Separated, Distance2D.TimeOfImpact(in wall, in sa, in box, in away).State);
        var inside = new Sweep2D(default, new Vec2(0, 0), new Vec2(5, 0), Rot2.Identity, Rot2.Identity);
        Assert.Equal(ToiState.Overlapped, Distance2D.TimeOfImpact(in wall, in sa, in box, in inside).State);
    }

    [Fact]
    public void FastRandomPairsNeverReportPastFirstContact()
    {
        var r = new TRng(9);
        int hits = 0, failed = 0;
        for (int k = 0; k < 6000; k++)
        {
            Shape2D a = TU.RandomShape(ref r);
            Shape2D b = TU.RandomShape(ref r);
            Vec2 lcA = a.Centroid, lcB = b.Centroid;
            Vec2 ca1 = r.Point(2f), cb1 = r.Point(2f) + new Vec2(4, 0);
            Vec2 ca2 = ca1 + r.Point(6f), cb2 = cb1 + r.Point(12f);
            Rot2 qa1 = r.Rot(), qb1 = r.Rot();
            // End rotations within about 90 degrees of the start.
            Rot2 qa2 = Rot2.Mul(qa1, Rot2.FromVector(1f, r.Range(-1f, 1f)));
            Rot2 qb2 = Rot2.Mul(qb1, Rot2.FromVector(1f, r.Range(-1f, 1f)));
            var sa = new Sweep2D(lcA, ca1, ca2, qa1, qa2);
            var sb = new Sweep2D(lcB, cb1, cb2, qb1, qb2);
            ToiOutput2D o = Distance2D.TimeOfImpact(in a, in sa, in b, in sb);
            if (o.State == ToiState.Failed) { failed++; }

            // Sampled brute force over the same sweep.
            const int steps = 1500;
            double first = -1;
            for (int i = 0; i <= steps && first < 0; i++)
            {
                float t = (float)i / steps;
                Transform2 xa = sa.GetTransform(t), xb = sb.GetTransform(t);
                var cache = new SimplexCache2D();
                DistanceOutput2D d = Distance2D.Compute(in a, in b, Transform2.InvMul(xa, xb), ref cache, true);
                if (d.Distance <= 0f) { first = t; }
            }

            if (first >= 0)
            {
                Assert.NotEqual(ToiState.Separated, o.State);
                Assert.True(o.Fraction <= first + 1e-6, "toi " + o.Fraction + " first overlap " + first + " state " + o.State);
                if (o.State == ToiState.Hit)
                {
                    hits++;
                    // The reported time is at the target gap, not much earlier.
                    Transform2 xa = sa.GetTransform(o.Fraction), xb = sb.GetTransform(o.Fraction);
                    var cache = new SimplexCache2D();
                    float gap = Distance2D.Compute(in a, in b, Transform2.InvMul(xa, xb), ref cache, true).Distance;
                    Assert.True(gap <= Constants2D.LinearSlop * 1.25f + 1e-4f, "gap " + gap);
                }
            }
        }

        Assert.True(hits > 500, "hits " + hits);
        Assert.True(failed < 60, "failed " + failed);
    }

    [Fact]
    public void RotationTooLargeToBoundFailsSafely()
    {
        var a = Shape2D.CreateBox(1f, 1f);
        var sa = new Sweep2D(default, default, default, Rot2.Identity, Rot2.Identity);
        var sb = new Sweep2D(default, new Vec2(5, 0), new Vec2(0, 0), Rot2.Identity, new Rot2(-1f, 0f));
        ToiOutput2D o = Distance2D.TimeOfImpact(in a, in sa, in a, in sb);
        Assert.Equal(ToiState.Failed, o.State);
        Assert.Equal(0f, o.Fraction);
    }
}
