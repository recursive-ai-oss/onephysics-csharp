using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

public class NarrowphaseTests
{
    private static readonly Epa3D Epa = new();

    private static RigidTransformF At(float x, float y, float z) => new(QuaternionF.Identity, new Vector3F(x, y, z));

    private static QuaternionF AxisAngle(Vector3F axis, double angle)
    {
        axis = axis.Normalized();
        return QuaternionF.CreateFromAxisSinCos(axis, (float)Math.Sin(angle / 2), (float)Math.Cos(angle / 2));
    }

    private static void Near(double expected, double actual, double tol, string what = "")
    {
        Assert.True(Math.Abs(expected - actual) <= tol, $"{what} expected {expected}, got {actual} (tolerance {tol})");
    }

    private static void NearV(Vector3F expected, Vector3F actual, double tol, string what = "")
    {
        Near(expected.X, actual.X, tol, what + ".X");
        Near(expected.Y, actual.Y, tol, what + ".Y");
        Near(expected.Z, actual.Z, tol, what + ".Z");
    }

    [Fact]
    public void SpheresApartTouchingAndOverlapping()
    {
        ConvexShape3D a = ConvexShape3D.Sphere(1f), b = ConvexShape3D.Sphere(0.5f);
        ConvexDistance3D d = Gjk3D.Distance(a, b, At(3, 0, 0));
        Near(1.5, d.Distance, 1e-6);
        NearV(Vector3F.UnitX, d.Normal, 1e-6);
        NearV(new Vector3F(1, 0, 0), d.PointA, 1e-6);
        NearV(new Vector3F(2.5f, 0, 0), d.PointB, 1e-6);

        ConvexContact3D c = Epa.Collide(a, b, At(1.2f, 0, 0));
        Near(-0.3, c.Separation, 1e-6);
        Assert.True(c.Colliding);
        NearV(Vector3F.UnitX, c.Normal, 1e-6);
        NearV(new Vector3F(1, 0, 0), c.PointA, 1e-6);
        NearV(new Vector3F(0.7f, 0, 0), c.PointB, 1e-6);

        // Concentric: the direction is arbitrary but the depth is the sum of the radii.
        c = Epa.Collide(a, b, At(0, 0, 0));
        Near(-1.5, c.Separation, 1e-6);
        Near(1.0, c.Normal.Length(), 1e-6);

        c = Epa.Collide(a, b, At(0, 0.6f, 0.8f));
        Near(-0.5, c.Separation, 1e-6);
        NearV(new Vector3F(0, 0.6f, 0.8f), c.Normal, 1e-6);
    }

    [Fact]
    public void BoxesFaceEdgeAndVertexContacts()
    {
        ConvexShape3D box = ConvexShape3D.Box(new Vector3F(1, 1, 1));
        // Face to face.
        Near(0.5, Gjk3D.Distance(box, box, At(2.5f, 0, 0)).Distance, 1e-6);
        ConvexContact3D c = Epa.Collide(box, box, At(1.9f, 0.2f, -0.1f));
        Near(-0.1, c.Separation, 1e-5);
        NearV(Vector3F.UnitX, c.Normal, 1e-5);
        c = Epa.Collide(box, box, At(0.1f, 1.7f, 0.2f));
        Near(-0.3, c.Separation, 1e-5);
        NearV(Vector3F.UnitY, c.Normal, 1e-5);

        // Vertex of a rotated box against a face: the corner (-1,-1,-1)/sqrt3 direction of B points at A.
        double s3 = Math.Sqrt(3);
        var toward = new Vector3F(-1, -1, -1);
        Vector3F axis = Vector3F.Cross(toward.Normalized(), new Vector3F(-1, 0, 0));
        double angle = Math.Atan2(axis.Length(), Vector3F.Dot(toward.Normalized(), new Vector3F(-1, 0, 0)));
        var rot = new RigidTransformF(AxisAngle(axis, angle), new Vector3F((float)(1 + s3 + 0.5), 0, 0));
        Near(0.5, Gjk3D.Distance(box, box, rot).Distance, 2e-6, "vertex-face distance");
        NearV(Vector3F.UnitX, Gjk3D.Distance(box, box, rot).Normal, 2e-6);
        var rot2 = new RigidTransformF(rot.Rotation, new Vector3F((float)(1 + s3 - 0.25), 0, 0));
        c = Epa.Collide(box, box, rot2);
        Near(-0.25, c.Separation, 2e-5, "vertex-face depth");
        NearV(Vector3F.UnitX, c.Normal, 2e-4);

        // Edge to edge: A turned 45 degrees about Z (edge along Z pointing at +x), B turned 45 degrees about Y (edge along Y pointing at -x).
        double r2 = Math.Sqrt(2);
        var ra = new RigidTransformF(AxisAngle(Vector3F.UnitZ, Math.PI / 4), default);
        ConvexShape3D rotatedBox = box;
        // Express B in A's frame: B's world = (R_y(45 deg), (d,0,0)); A's world = (R_z(45 deg), 0).
        QuaternionF qa = ra.Rotation, qb = AxisAngle(Vector3F.UnitY, Math.PI / 4);
        QuaternionF rel = qa.Conjugated() * qb;
        float dist = (float)(2 * r2 + 0.4);
        var bInA = new RigidTransformF(rel, qa.Conjugated().Rotate(new Vector3F(dist, 0, 0)));
        ConvexDistance3D ed = Gjk3D.Distance(rotatedBox, box, bInA);
        Near(0.4, ed.Distance, 5e-6, "edge-edge distance");
        // Normal A->B in world is +x.
        NearV(Vector3F.UnitX, qa.Rotate(ed.Normal), 5e-6, "edge normal");
        dist = (float)(2 * r2 - 0.3);
        bInA = new RigidTransformF(rel, qa.Conjugated().Rotate(new Vector3F(dist, 0, 0)));
        c = Epa.Collide(rotatedBox, box, bInA);
        Near(-0.3, c.Separation, 5e-5, "edge-edge depth");
        NearV(Vector3F.UnitX, qa.Rotate(c.Normal), 5e-4, "edge-edge penetration normal");
    }

    [Fact]
    public void RoundedBoxAddsItsRadiusOnlyOnce()
    {
        ConvexShape3D sharp = ConvexShape3D.Box(new Vector3F(1, 1, 1)), round = ConvexShape3D.Box(new Vector3F(1, 1, 1), 0.25f);
        Near(0.5, Gjk3D.Distance(round, sharp, At(2.5f, 0, 0)).Distance, 1e-6);
        Near(0.5 + (0.25 * (Math.Sqrt(2) - 1) * 0), Gjk3D.Distance(round, round, At(2.5f, 0, 0)).Distance, 1e-6);
        // Along a diagonal the rounded corner is closer to the centre than the sharp one.
        double sharpDiag = Gjk3D.Distance(sharp, sharp, At(3, 3, 3)).Distance;
        double roundDiag = Gjk3D.Distance(round, round, At(3, 3, 3)).Distance;
        Assert.True(roundDiag > sharpDiag + 0.1);
        // The rounded corner is a sphere of radius 0.25 centred at (0.75,0.75,0.75).
        double expected = (Math.Sqrt(27) - (2 * Math.Sqrt(3) * 0.75)) - 0.5;
        Near(expected, roundDiag, 2e-5);
    }

    [Fact]
    public void CapsulePairs()
    {
        ConvexShape3D a = ConvexShape3D.Capsule(1f, 0.5f), b = ConvexShape3D.Capsule(0.8f, 0.25f);
        // Parallel side by side.
        Near(0.75 - 0.75 + 1.0, Gjk3D.Distance(a, b, At(1.75f, 0.3f, 0)).Distance, 1e-6);
        ConvexContact3D c = Epa.Collide(a, b, At(0.6f, 0.3f, 0));
        Near(-0.15, c.Separation, 1e-5);
        NearV(Vector3F.UnitX, c.Normal, 1e-5);
        // End to end.
        Near(5.0 - 1.8 - 0.75, Gjk3D.Distance(a, b, At(0, 5, 0)).Distance, 1e-6);
        // Perpendicular, skew: B lies along X, 1.5 above A's axis, crossing over it.
        var turn = new RigidTransformF(AxisAngle(Vector3F.UnitZ, Math.PI / 2), new Vector3F(0.2f, 0.1f, 1.5f));
        ConvexDistance3D d = Gjk3D.Distance(a, b, turn);
        Near(1.5 - 0.75, d.Distance, 2e-6);
        NearV(Vector3F.UnitZ, d.Normal, 2e-6);
        // Axes crossing: the cores intersect, depth is the radii sum and the direction is the common perpendicular.
        turn = new RigidTransformF(AxisAngle(Vector3F.UnitZ, Math.PI / 2), new Vector3F(0.2f, 0.1f, 0f));
        c = Epa.Collide(a, b, turn);
        Near(-0.75, c.Separation, 1e-5);
        Near(1.0, Math.Abs(c.Normal.Z), 1e-5);
        // Slightly off the crossing: the direction follows the offset.
        turn = new RigidTransformF(AxisAngle(Vector3F.UnitZ, Math.PI / 2), new Vector3F(0.2f, 0.1f, -0.1f));
        c = Epa.Collide(a, b, turn);
        Near(-0.65, c.Separation, 1e-5);
        NearV(new Vector3F(0, 0, -1), c.Normal, 1e-4);
    }

    [Fact]
    public void CylinderCases()
    {
        ConvexShape3D cyl = ConvexShape3D.Cylinder(1f, 1f);
        Near(1.0, Gjk3D.Distance(cyl, cyl, At(3, 0.2f, 0)).Distance, 1e-6, "side by side");
        Near(1.0, Gjk3D.Distance(cyl, cyl, At(0.3f, 3, 0.1f)).Distance, 1e-6, "cap to cap");
        // Rim to rim: B tilted 90 degrees about Z touches A's top rim with its own side; a sphere gives a simpler rim case.
        ConvexShape3D ball = ConvexShape3D.Sphere(0.5f);
        double d = Gjk3D.Distance(cyl, ball, At(2, 2, 0)).Distance; // nearest feature: the top rim point (1,1,0)
        Near(Math.Sqrt(2) - 0.5, d, 2e-6, "rim");
        ConvexContact3D c = Epa.Collide(cyl, ball, At(0.9f, 1.2f, 0));
        // Ball centre above the cap, inside the radius: penetration through the cap.
        Near(-(0.5 - 0.2), c.Separation, 2e-5);
        NearV(Vector3F.UnitY, c.Normal, 5e-3);
        c = Epa.Collide(cyl, cyl, At(1.5f, 0, 0));
        Near(-0.5, c.Separation, 2e-5);
        NearV(Vector3F.UnitX, c.Normal, 5e-3);
    }

    [Fact]
    public void TaperedCapsuleAndHullSupport()
    {
        ConvexShape3D tc = ConvexShape3D.TaperedCapsule(1f, 0.25f, 0.75f);
        // The farthest point along +X lies on the tangent line, not on either sphere's equator.
        Vector3F s = tc.Support(Vector3F.UnitX);
        double sinA = (0.75 - 0.25) / 2.0;
        // Tangent normal tilts towards +y by asin(sinA)... for a wider bottom, the outward normal tilts up.
        double cosA = Math.Sqrt(1 - (sinA * sinA));
        // Direction (cosA, sinA, 0) is the tangent-line normal; support in it is on the sphere with radius that maximises c.n + r.
        Vector3F sn = tc.Support(new Vector3F((float)cosA, (float)sinA, 0));
        Near(-1 + (0.75 * sinA), sn.Y, 1e-5);
        Near(0.75 * cosA, sn.X, 1e-5);
        Assert.True(s.X > 0.7f);
        Near(1.25, tc.Support(Vector3F.UnitY).Y, 1e-6);
        Near(-1.75, tc.Support(-Vector3F.UnitY).Y, 1e-6);
        Assert.Throws<ArgumentOutOfRangeException>(() => ConvexShape3D.TaperedCapsule(0.5f, 0.2f, 1.5f));
        Near(2.0 - 1.25 - 0.3, Gjk3D.Distance(tc, ConvexShape3D.Sphere(0.3f), At(0, 2, 0)).Distance, 1e-6);
        Near(3.0 - 1.75 - 0.3, Gjk3D.Distance(tc, ConvexShape3D.Sphere(0.3f), At(0, -3, 0)).Distance, 1e-6);
    }

    [Fact]
    public void InvalidShapesAreRejected()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => ConvexShape3D.Sphere(0f));
        Assert.Throws<ArgumentOutOfRangeException>(() => ConvexShape3D.Sphere(float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => ConvexShape3D.Sphere(float.PositiveInfinity));
        Assert.Throws<ArgumentOutOfRangeException>(() => ConvexShape3D.Box(new Vector3F(1, -1, 1)));
        Assert.Throws<ArgumentOutOfRangeException>(() => ConvexShape3D.Box(new Vector3F(1, 1, 1), 2f));
        Assert.Throws<ArgumentOutOfRangeException>(() => ConvexShape3D.Capsule(1f, -1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => ConvexShape3D.Cylinder(1f, 1f, float.NaN));
        Assert.Throws<ArgumentNullException>(() => ConvexShape3D.FromHull(null!));
        Assert.Throws<ArgumentOutOfRangeException>(() => ConvexShape3D.Sphere(1e9f));
    }

    private static (ConvexHull3D hull, ConvexShape3D shape) RandomHull(TestRng rng)
    {
        var pts = new Vector3F[6 + rng.Next(25)];
        for (int i = 0; i < pts.Length; i++)
        {
            pts[i] = rng.InBall(1f);
        }

        ConvexHull3D hull = ConvexHull3D.Build(pts);
        return (hull, ConvexShape3D.FromHull(hull));
    }

    [Fact]
    public void GjkMatchesBruteForceAndEpaMatchesSatOnRandomHulls()
    {
        var rng = new TestRng(2024);
        int separated = 0, overlapping = 0, skipped = 0;
        double worstDist = 0, worstDepth = 0, worstNormal = 1;
        for (int i = 0; i < 100_000; i++)
        {
            (ConvexHull3D ha, ConvexShape3D sa) = RandomHull(rng);
            (ConvexHull3D hb, ConvexShape3D sb) = RandomHull(rng);
            var xf = new RigidTransformF(rng.Rotation(), rng.InBall(rng.Range(0.2f, 2.8f)));
            Brute.Poly pa = Brute.FromHull(ha, RigidTransformF.Identity), pb = Brute.FromHull(hb, xf);
            var (sep, axis, second) = Brute.Sat(pa, pb);
            ConvexDistance3D d = Gjk3D.Distance(sa, sb, xf);
            if (sep > 2e-4)
            {
                separated++;
                double brute = Brute.Distance(pa, pb);
                Assert.False(d.Overlapping, $"pair {i}: reported overlap, brute distance {brute}");
                worstDist = Math.Max(worstDist, Math.Abs(brute - d.Distance));
                Assert.True(Math.Abs(brute - d.Distance) <= 3e-5, $"pair {i}: distance {d.Distance} vs brute {brute}");
                // Witness points: on their own shapes' surfaces (inside the hull planes) and exactly Distance apart.
                Near(d.Distance, (d.PointB - d.PointA).Length(), 3e-5, $"pair {i} witness gap");
                Near(1.0, d.Normal.Length(), 1e-5);
            }
            else if (sep < -2e-4)
            {
                overlapping++;
                Assert.True(d.Overlapping, $"pair {i}: missed overlap, SAT {sep}");
                ConvexContact3D c = Epa.Collide(sa, sb, xf);
                double depth = -c.Separation;
                worstDepth = Math.Max(worstDepth, Math.Abs(depth + sep));
                Assert.True(Math.Abs(depth + sep) <= 2e-4, $"pair {i}: depth {depth} vs SAT {-sep}");
                if (second - sep > 2e-2)
                {
                    double dot = (c.Normal.X * axis.X) + (c.Normal.Y * axis.Y) + (c.Normal.Z * axis.Z);
                    worstNormal = Math.Min(worstNormal, dot);
                    Assert.True(dot > 0.999, $"pair {i}: normal {c.Normal} vs SAT axis {axis} (dot {dot})");
                }
            }
            else
            {
                skipped++;
            }
        }

        Assert.True(separated > 20_000 && overlapping > 20_000, $"{separated} separated, {overlapping} overlapping, {skipped} skipped");
        Console.WriteLine($"gjk worst distance error {worstDist:E2}, epa worst depth error {worstDepth:E2}, worst normal dot {worstNormal:F6}; {separated} apart, {overlapping} overlapping, {skipped} skipped");
    }

    [Fact]
    public void MixedShapePairsAgreeWithEachOtherUnderSwap()
    {
        // Swapping A and B must mirror the answer: same separation, opposite normal.
        var rng = new TestRng(99);
        var hulls = Collision3DHash.RandomHulls(rng, 8);
        for (int i = 0; i < 20_000; i++)
        {
            ConvexShape3D a = Collision3DHash.RandomShape(rng, hulls), b = Collision3DHash.RandomShape(rng, hulls);
            var xf = new RigidTransformF(rng.Rotation(), rng.InBall(rng.Range(0.2f, 3f)));
            ConvexContact3D ab = Epa.Collide(a, b, xf);
            ConvexContact3D ba = Epa.Collide(b, a, xf.Inverted());
            Assert.True(float.IsFinite(ab.Separation) && float.IsFinite(ba.Separation));
            bool polyhedral = a.Kind is ConvexShapeKind3D.Box or ConvexShapeKind3D.Hull && b.Kind is ConvexShapeKind3D.Box or ConvexShapeKind3D.Hull;
            // Polytope expansion is exact for polyhedra; for smooth shapes it is limited by the vertex budget (about 0.5% worst case).
            Near(ab.Separation, ba.Separation, polyhedral ? 3e-4 : 8e-3, $"swap {i} ({a.Kind}, {b.Kind})");
            if (polyhedral && Math.Abs(ab.Separation) > 1e-3)
            {
                double dot = Vector3F.Dot(ab.Normal, xf.Rotation.Rotate(-ba.Normal));
                Assert.True(dot > 0.99 || Math.Abs(ab.Separation) < 0.02, $"swap {i} ({a.Kind}, {b.Kind}): normals {ab.Normal} vs {xf.Rotation.Rotate(-ba.Normal)} sep {ab.Separation}");
            }
        }
    }

    [Fact]
    public void RayCastsMatchAnalyticAnswers()
    {
        // Sphere.
        ConvexShape3D ball = ConvexShape3D.Sphere(1f);
        Assert.True(Gjk3D.RayCast(ball, At(5, 0, 0), new Vector3F(0, 0, 0), Vector3F.UnitX, 10f, out ConvexCastHit3D h));
        Near(4.0, h.Fraction, 1e-5);
        NearV(-Vector3F.UnitX, h.Normal, 1e-5);
        NearV(new Vector3F(4, 0, 0), h.PointB, 1e-5);
        Assert.False(Gjk3D.RayCast(ball, At(5, 0, 0), new Vector3F(0, 0, 0), Vector3F.UnitX, 3.9f, out _));
        Assert.False(Gjk3D.RayCast(ball, At(5, 1.1f, 0), default, Vector3F.UnitX, 10f, out _));
        Assert.False(Gjk3D.RayCast(ball, At(5, 0, 0), default, -Vector3F.UnitX, 10f, out _));
        // Starting inside hits at zero.
        Assert.True(Gjk3D.RayCast(ball, At(0, 0, 0), default, Vector3F.UnitX, 10f, out h));
        Assert.Equal(0f, h.Fraction);
        Assert.True(h.StartsOverlapping);
        // Box, offset ray at an angle.
        ConvexShape3D box = ConvexShape3D.Box(new Vector3F(1, 2, 3));
        Assert.True(Gjk3D.RayCast(box, At(0, 0, 10), new Vector3F(0.5f, -0.25f, 0), new Vector3F(0, 0, 1), 20f, out h));
        Near(7.0, h.Fraction, 1e-5);
        NearV(-Vector3F.UnitZ, h.Normal, 1e-5);
        // Capsule side: ray along z at x=0.3 hits the cylinder wall at z = -sqrt(r^2 - x^2).
        ConvexShape3D cap = ConvexShape3D.Capsule(1f, 0.5f);
        Assert.True(Gjk3D.RayCast(cap, At(0, 0, 0), new Vector3F(0.3f, 0.2f, -5), Vector3F.UnitZ, 10f, out h));
        double wall = Math.Sqrt(0.25 - 0.09);
        Near(5 - wall, h.Fraction, 2e-5);
        NearV(new Vector3F(0.3f, 0, (float)-wall).Normalized(), h.Normal, 5e-4);
        // Capsule cap: ray down the axis hits the top at y = 1.5.
        Assert.True(Gjk3D.RayCast(cap, At(0, 0, 0), new Vector3F(0, 4, 0), -Vector3F.UnitY, 10f, out h));
        Near(2.5, h.Fraction, 2e-5);
        // Cylinder.
        ConvexShape3D cyl = ConvexShape3D.Cylinder(1f, 2f);
        Assert.True(Gjk3D.RayCast(cyl, At(0, 0, 0), new Vector3F(-5, 0.5f, 0.5f), Vector3F.UnitX, 10f, out h));
        Near(5 - Math.Sqrt(4 - 0.25), h.Fraction, 3e-5);
        Assert.True(Gjk3D.RayCast(cyl, At(0, 0, 0), new Vector3F(1, 5, 1), -Vector3F.UnitY, 10f, out h));
        Near(4.0, h.Fraction, 3e-5);
        NearV(Vector3F.UnitY, h.Normal, 1e-5);
        Assert.Throws<ArgumentOutOfRangeException>(() => Gjk3D.RayCast(ball, At(5, 0, 0), default, Vector3F.UnitX, 0f, out _));
    }

    [Fact]
    public void RayCastsMatchBruteForceOnRandomHulls()
    {
        var rng = new TestRng(555);
        int hits = 0;
        for (int i = 0; i < 20_000; i++)
        {
            (ConvexHull3D hull, ConvexShape3D shape) = RandomHull(rng);
            var xf = new RigidTransformF(rng.Rotation(), rng.InBall(1f));
            Vector3F origin = rng.OnSphere() * rng.Range(1.5f, 4f);
            Vector3F dir = (-origin + rng.InBall(1.2f)).Normalized();
            Brute.Poly poly = Brute.FromHull(hull, xf);
            var expected = Brute.Ray(poly, origin, dir, 8.0);
            bool hit = Gjk3D.RayCast(shape, xf, origin, dir, 8f, out ConvexCastHit3D r);
            if (expected is { } e)
            {
                // Grazing rays are ambiguous; require clear hits.
                double margin = 1e-3;
                var shrunk = Brute.Ray(poly, origin, dir, e.t - margin);
                if (e.t > 0)
                {
                    hits++;
                    Assert.True(hit, $"ray {i}: missed a hull hit at {e.t}");
                    Near(e.t, r.Fraction, 2e-4, $"ray {i}");
                    double dot = (r.Normal.X * e.normal.X) + (r.Normal.Y * e.normal.Y) + (r.Normal.Z * e.normal.Z);
                    Assert.True(dot > 0.98, $"ray {i} normal dot {dot}");
                    _ = shrunk;
                }
            }
            else
            {
                // Misses: only fail when the ray is clearly away from the hull (distance from the hull larger than 1e-3).
                if (hit)
                {
                    Vector3F p = origin + (dir * r.Fraction);
                    Brute.Poly self = poly;
                    double worst = -1e30;
                    for (int f = 0; f < self.Normals.Length; f++)
                    {
                        worst = Math.Max(worst, Brute.V.Dot(self.Normals[f], p) - self.Offsets[f]);
                    }

                    Assert.True(worst < 1e-3, $"ray {i}: reported a hit the brute force does not see (hit point {worst} outside)");
                }
            }
        }

        Assert.True(hits > 4000, $"only {hits} hits");
    }

    [Fact]
    public void ShapeCastsMatchAnalyticAnswers()
    {
        // Sphere towards sphere.
        ConvexShape3D a = ConvexShape3D.Sphere(1f), b = ConvexShape3D.Sphere(0.5f);
        Assert.True(Gjk3D.ShapeCast(a, b, At(10, 0, 0), new Vector3F(20, 0, 0), out ConvexCastHit3D h));
        Near((10 - 1.5) / 20.0, h.Fraction, 1e-5);
        NearV(Vector3F.UnitX, h.Normal, 1e-5);
        NearV(new Vector3F(8.5f + 1, 0, 0), h.PointA, 1e-4);
        NearV(new Vector3F(9.5f, 0, 0), h.PointB, 1e-4);
        Assert.False(Gjk3D.ShapeCast(a, b, At(10, 0, 0), new Vector3F(8, 0, 0), out _));
        Assert.False(Gjk3D.ShapeCast(a, b, At(10, 1.6f, 0), new Vector3F(20, 0, 0), out _));
        // Grazing: passes at distance just under the radii sum.
        Assert.True(Gjk3D.ShapeCast(a, b, At(10, 1.45f, 0), new Vector3F(20, 0, 0), out h));
        double x = 10 - Math.Sqrt((1.5 * 1.5) - (1.45 * 1.45));
        Near(x / 20.0, h.Fraction, 2e-5);
        // Box against box along a diagonal-free axis: faces meet after 0.75.
        ConvexShape3D box = ConvexShape3D.Box(new Vector3F(1, 1, 1));
        Assert.True(Gjk3D.ShapeCast(box, box, At(0.3f, 4, 0.2f), new Vector3F(0, 4, 0), out h));
        Near(0.5, h.Fraction, 1e-5);
        NearV(Vector3F.UnitY, h.Normal, 1e-5);
        // Already overlapping.
        Assert.True(Gjk3D.ShapeCast(box, box, At(0.5f, 0.5f, 0), new Vector3F(0, 4, 0), out h));
        Assert.Equal(0f, h.Fraction);
        Assert.True(h.StartsOverlapping);
        Assert.Throws<ArgumentOutOfRangeException>(() => Gjk3D.ShapeCast(box, box, At(3, 0, 0), new Vector3F(float.NaN, 0, 0), out _));
        // Capsule falling onto a box (rounded shapes: the radius is added back).
        ConvexShape3D cap = ConvexShape3D.Capsule(0.5f, 0.25f);
        Assert.True(Gjk3D.ShapeCast(cap, box, At(0.2f, -5, 0.1f), new Vector3F(0, -10, 0), out h));
        Near((5.0 - 1 - 0.75) / 10.0, h.Fraction, 1e-5);
    }

    [Fact]
    public void ShapeCastsMatchDirectDistanceOnRandomShapes()
    {
        // After sweeping A to the hit fraction, the shapes must touch (distance ~ 0); just before it they must be apart.
        var rng = new TestRng(808);
        var hulls = Collision3DHash.RandomHulls(rng, 8);
        int hits = 0;
        for (int i = 0; i < 20_000; i++)
        {
            ConvexShape3D a = Collision3DHash.RandomShape(rng, hulls), b = Collision3DHash.RandomShape(rng, hulls);
            var xf = new RigidTransformF(rng.Rotation(), rng.InBall(rng.Range(2.5f, 4f)));
            Vector3F motion = rng.InBall(5f);
            if (!Gjk3D.ShapeCast(a, b, xf, motion, out ConvexCastHit3D h))
            {
                // A miss: the swept path must stay clear at sampled times.
                for (int k = 1; k <= 8; k++)
                {
                    var moved = new RigidTransformF(xf.Rotation, xf.Translation - (motion * (k / 8f)));
                    ConvexDistance3D dd = Gjk3D.Distance(a, b, moved);
                    Assert.False(dd.Overlapping && dd.Distance == 0f && Epa.Collide(a, b, moved).Separation < -1e-3f, $"cast {i} missed but shapes overlap at t={k / 8f}");
                }

                continue;
            }

            hits++;
            if (h.StartsOverlapping)
            {
                continue;
            }

            var at = new RigidTransformF(xf.Rotation, xf.Translation - (motion * h.Fraction));
            ConvexContact3D c = Epa.Collide(a, b, at);
            Assert.True(Math.Abs(c.Separation) <= 6e-4, $"cast {i}: separation at the hit is {c.Separation}");
            if (h.Fraction > 1e-3f)
            {
                var before = new RigidTransformF(xf.Rotation, xf.Translation - (motion * (h.Fraction - 1e-3f)));
                Assert.True(Epa.Collide(a, b, before).Separation > -2e-4f, $"cast {i}: already overlapping before the hit");
            }
        }

        Assert.True(hits > 2000, $"only {hits} hits");
    }

    [Fact]
    public void ResultsAreBitIdenticalAcrossThreadsAndRuns()
    {
        ulong first = Collision3DHash.Compute();
        Assert.Equal(first, Collision3DHash.Compute());
        var results = new ulong[4];
        var threads = new System.Threading.Thread[4];
        for (int t = 0; t < threads.Length; t++)
        {
            int ti = t;
            threads[t] = new System.Threading.Thread(() => results[ti] = Collision3DHash.Compute());
            threads[t].Start();
        }

        foreach (var th in threads)
        {
            th.Join();
        }

        Assert.All(results, r => Assert.Equal(first, r));
        Assert.Equal(Collision3DHash.Expected, first);
    }

    [Fact]
    public void MassPropertiesMatchAnalyticShapes()
    {
        var rng = new TestRng(3);
        _ = rng;
        var box = ConvexShape3D.Box(new Vector3F(1, 2, 3)).GetMassProperties();
        Near(48, box.Mass, 1e-4);
        Near(48 * (4 + 9) / 3.0, box.Inertia.M11, 1e-3);
        var sph = ConvexShape3D.Sphere(2f).GetMassProperties();
        Near(4.0 / 3 * Math.PI * 8, sph.Mass, 1e-3);
        Near(0.4 * sph.Mass * 4, sph.Inertia.M22, 1e-3);
        var cyl = ConvexShape3D.Cylinder(1.5f, 0.5f).GetMassProperties();
        double mc = Math.PI * 0.25 * 3;
        Near(mc, cyl.Mass, 1e-4);
        Near(mc * 0.25 / 2, cyl.Inertia.M22, 1e-4);
        Near(mc * ((3 * 0.25) + (4 * 2.25)) / 12, cyl.Inertia.M11, 1e-4);
        // Capsule: cylinder plus two hemispheres.
        double r = 0.5, hh = 1.0;
        var cap = ConvexShape3D.Capsule((float)hh, (float)r).GetMassProperties();
        double vc = Math.PI * r * r * 2 * hh, vs = 4.0 / 3 * Math.PI * r * r * r;
        Near(vc + vs, cap.Mass, 1e-4);
        Near((vc * r * r / 2) + (vs * 0.4 * r * r), cap.Inertia.M22, 1e-4);
        Near((vc * ((r * r / 4) + (hh * hh / 3))) + (vs * ((0.4 * r * r) + (hh * hh) + (0.75 * hh * r))), cap.Inertia.M11, 1e-4);
        Near(cap.Inertia.M11, cap.Inertia.M33, 1e-6);
        // Tapered capsule with equal radii is a capsule; a cone-like one is checked against numeric integration of its profile.
        var same = ConvexShape3D.TaperedCapsule(1f, 0.5f, 0.5f).GetMassProperties();
        Near(cap.Mass, same.Mass, 1e-4);
        Near(cap.Inertia.M11, same.Inertia.M11, 1e-4);
        double h = 1.0, rt = 0.2, rb = 0.7;
        var tap = ConvexShape3D.TaperedCapsule((float)h, (float)rt, (float)rb).GetMassProperties();
        double sn = (rb - rt) / (2 * h), cs = Math.Sqrt(1 - (sn * sn));
        double y1 = -h + (rb * sn), y2 = h + (rt * sn);
        double Rho(double y)
        {
            if (y < y1)
            {
                return Math.Sqrt(Math.Max(0, (rb * rb) - ((y + h) * (y + h))));
            }

            if (y > y2)
            {
                return Math.Sqrt(Math.Max(0, (rt * rt) - ((y - h) * (y - h))));
            }

            return (rb * cs) + (((rt * cs) - (rb * cs)) * (y - y1) / (y2 - y1));
        }

        int n = 400_000;
        double lo = -h - rb, hi = h + rt, dy = (hi - lo) / n, v = 0, m1 = 0;
        for (int i = 0; i < n; i++)
        {
            double y = lo + ((i + 0.5) * dy), rho = Rho(y);
            v += Math.PI * rho * rho * dy;
            m1 += Math.PI * rho * rho * y * dy;
        }

        double yc = m1 / v, ixx = 0, iyy = 0;
        for (int i = 0; i < n; i++)
        {
            double y = lo + ((i + 0.5) * dy), rho = Rho(y);
            double dm = Math.PI * rho * rho * dy;
            ixx += dm * ((rho * rho / 4) + ((y - yc) * (y - yc)));
            iyy += dm * rho * rho / 2;
        }

        Near(v, tap.Mass, 1e-4);
        Near(yc, tap.Center.Y, 1e-4);
        Near(ixx, tap.Inertia.M11, 1e-4);
        Near(iyy, tap.Inertia.M22, 1e-4);
    }

    [Fact]
    public void HullMassPropertiesMatchBoxAndTetrahedron()
    {
        var pts = new Vector3F[8];
        for (int i = 0; i < 8; i++)
        {
            pts[i] = new Vector3F(((i & 1) * 2 - 1) * 1f, (((i >> 1) & 1) * 2 - 1) * 2f, (((i >> 2) & 1) * 2 - 1) * 3f);
        }

        var hull = ConvexShape3D.FromHull(ConvexHull3D.Build(pts)).GetMassProperties();
        var box = ConvexShape3D.Box(new Vector3F(1, 2, 3)).GetMassProperties();
        Near(box.Mass, hull.Mass, 1e-3);
        Near(box.Inertia.M11, hull.Inertia.M11, 2e-3);
        Near(box.Inertia.M22, hull.Inertia.M22, 2e-3);
        Near(box.Inertia.M33, hull.Inertia.M33, 2e-3);
        Near(0, hull.Inertia.M12, 1e-4);
        // A translated copy has the centre of mass moved and the same inertia about it.
        var moved = new Vector3F[8];
        for (int i = 0; i < 8; i++)
        {
            moved[i] = pts[i] + new Vector3F(5, -2, 1);
        }

        var hm = ConvexHull3D.Build(moved).MassProperties;
        NearV(new Vector3F(5, -2, 1), hm.Center, 1e-4);
        Near(hull.Inertia.M11, hm.Inertia.M11, 5e-3);
    }
}
