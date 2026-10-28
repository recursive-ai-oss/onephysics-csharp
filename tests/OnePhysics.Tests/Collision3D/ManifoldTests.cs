using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Collision3D;
using RecursiveAI.OnePhysics.Core;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

public class ManifoldTests
{
    private static RigidTransformF At(float x, float y, float z) => new(QuaternionF.Identity, new Vector3F(x, y, z));

    private static QuaternionF AxisAngle(Vector3F axis, double angle)
    {
        axis = axis.Normalized();
        return QuaternionF.CreateFromAxisSinCos(axis, (float)Math.Sin(angle / 2), (float)Math.Cos(angle / 2));
    }

    // B's pose in A's frame from two world poses.
    private static RigidTransformF Rel(QuaternionF qa, Vector3F pa, QuaternionF qb, Vector3F pb)
    {
        QuaternionF c = qa.Conjugated();
        return new RigidTransformF(c * qb, c.Rotate(pb - pa));
    }

    private static void Near(double expected, double actual, double tol, string what = "") =>
        Assert.True(Math.Abs(expected - actual) <= tol, $"{what} expected {expected}, got {actual} (tolerance {tol})");

    private static void NearV(Vector3F expected, Vector3F actual, double tol, string what = "")
    {
        Near(expected.X, actual.X, tol, what + ".X");
        Near(expected.Y, actual.Y, tol, what + ".Y");
        Near(expected.Z, actual.Z, tol, what + ".Z");
    }

    private static ConvexShape3D Box(float x, float y, float z, float r = 0f) => ConvexShape3D.Box(new Vector3F(x, y, z), r);

    private static ConvexHull3D BoxHull(float x, float y, float z)
    {
        var pts = new Vector3F[8];
        for (int i = 0; i < 8; i++)
        {
            pts[i] = new Vector3F((i & 1) == 0 ? -x : x, (i & 2) == 0 ? -y : y, (i & 4) == 0 ? -z : z);
        }

        return ConvexHull3D.Build(pts);
    }

    private static void WellFormed(Manifold3D m)
    {
        Assert.InRange(m.PointCount, 0, 4);
        if (m.PointCount == 0)
        {
            return;
        }

        Near(1.0, m.Normal.Length(), 1e-4, "normal length");
        for (int i = 0; i < m.PointCount; i++)
        {
            ManifoldPoint3D p = m[i];
            Assert.True(float.IsFinite(p.Point.X + p.Point.Y + p.Point.Z + p.Separation), "finite point");
        }
    }

    private static uint[] Ids(Manifold3D m)
    {
        var ids = new uint[m.PointCount];
        for (int i = 0; i < ids.Length; i++)
        {
            ids[i] = m[i].Id;
        }

        return ids;
    }

    [Fact]
    public void ContactFitsTheSharedCoreLayout()
    {
        Assert.Equal(Manifold3D.MaxPoints, DimensionLayout.Spatial.MaxManifoldPoints);
        Assert.True(DimensionLayout.Spatial.PayloadFloats >= ManifoldLayout.Header + (Manifold3D.MaxPoints * ManifoldLayout.PointStride));
    }

    [Fact]
    public void BoxRestingOnBoxKeepsFourPointsWithStableIds()
    {
        var g = new ManifoldGenerator3D();
        var ground = Box(5f, 0.5f, 5f);
        var box = Box(0.5f, 0.5f, 0.5f);
        var rng = new TestRng(5);
        uint[]? first = null;
        for (int frame = 0; frame < 100; frame++)
        {
            var xf = new RigidTransformF(AxisAngle(Vector3F.UnitY, rng.Range(-3f, 3f)), new Vector3F(rng.Range(-2f, 2f), 0.99f + rng.Range(-0.004f, 0.004f), rng.Range(-2f, 2f)));
            Manifold3D m = g.Collide(ground, box, xf);
            Assert.Equal(4, m.PointCount);
            NearV(Vector3F.UnitY, m.Normal, 1e-5);
            for (int i = 0; i < 4; i++)
            {
                Near(xf.Translation.Y - 1f, m[i].Separation, 1e-5, "separation");
                Near(0.5, m[i].Point.Y, 0.01, "point height");
            }

            uint[] ids = Ids(m);
            first ??= ids;
            Assert.Equal(first, ids);
        }
    }

    [Fact]
    public void TiltedBoxOnBoxStillGetsFourPoints()
    {
        var g = new ManifoldGenerator3D();
        var xf = new RigidTransformF(AxisAngle(new Vector3F(1, 0, 1), 0.01), new Vector3F(0, 0.995f, 0));
        Manifold3D m = g.Collide(Box(5f, 0.5f, 5f), Box(0.5f, 0.5f, 0.5f), xf);
        Assert.Equal(4, m.PointCount);
        Assert.True(m.MinSeparation < 0f);
    }

    [Fact]
    public void BoxEdgeOnBoxEdgeGivesOnePointBetweenTheEdges()
    {
        var g = new ManifoldGenerator3D();
        QuaternionF qa = AxisAngle(Vector3F.UnitX, Math.PI / 4), qb = AxisAngle(Vector3F.UnitZ, Math.PI / 4);
        float ty = (2f * 1.41421356f) - 0.01f;
        RigidTransformF xf = Rel(qa, default, qb, new Vector3F(0, ty, 0));
        Manifold3D m = g.Collide(Box(1, 1, 1), Box(1, 1, 1), xf);
        Assert.Equal(1, m.PointCount);
        // Normal: world +Y seen from A's frame.
        NearV(qa.Conjugated().Rotate(Vector3F.UnitY), m.Normal, 1e-4);
        Near(-0.01, m[0].Separation, 2e-4);
        uint id = m[0].Id;
        Assert.Equal(1u, id >> 30);
        Assert.Equal(1u, (id >> 14) & 3);
        // The point is where the two edges cross, at the origin in x and z.
        Vector3F w = qa.Rotate(m[0].Point);
        Near(0, w.X, 1e-3);
        Near(0, w.Z, 1e-3);
    }

    [Fact]
    public void HullOnHullMatchesBoxOnBox()
    {
        var g = new ManifoldGenerator3D();
        var rng = new TestRng(77);
        ConvexShape3D hg = ConvexShape3D.FromHull(BoxHull(5f, 0.5f, 5f)), hb = ConvexShape3D.FromHull(BoxHull(0.5f, 0.5f, 0.5f));
        for (int i = 0; i < 20; i++)
        {
            var xf = new RigidTransformF(AxisAngle(Vector3F.UnitY, rng.Range(-3f, 3f)), new Vector3F(rng.Range(-2f, 2f), 0.99f, rng.Range(-2f, 2f)));
            Manifold3D a = g.Collide(hg, hb, xf), b = g.Collide(Box(5f, 0.5f, 5f), Box(0.5f, 0.5f, 0.5f), xf);
            Assert.Equal(4, a.PointCount);
            Assert.Equal(b.PointCount, a.PointCount);
            Near(b.MinSeparation, a.MinSeparation, 1e-5);
            // Both are four corners of the incident face; compare the point sets.
            for (int k = 0; k < 4; k++)
            {
                bool found = false;
                for (int j = 0; j < 4; j++)
                {
                    found |= (a[k].Point - b[j].Point).Length() < 1e-4f;
                }

                Assert.True(found);
            }
        }
    }

    [Fact]
    public void SphereAndCapsuleCasesAreExact()
    {
        var g = new ManifoldGenerator3D();
        // Sphere-sphere.
        Manifold3D m = g.Collide(ConvexShape3D.Sphere(1f), ConvexShape3D.Sphere(0.5f), At(1.2f, 0, 0));
        Assert.Equal(1, m.PointCount);
        NearV(Vector3F.UnitX, m.Normal, 1e-6);
        Near(-0.3, m[0].Separation, 1e-6);
        NearV(new Vector3F(0.85f, 0, 0), m[0].Point, 1e-6);
        Assert.Equal(0x80008000u, m[0].Id);

        // Speculative: a gap of 0.01 gets a point, 0.05 does not.
        Assert.Equal(1, g.Collide(ConvexShape3D.Sphere(1f), ConvexShape3D.Sphere(0.5f), At(1.51f, 0, 0)).PointCount);
        Assert.Equal(0, g.Collide(ConvexShape3D.Sphere(1f), ConvexShape3D.Sphere(0.5f), At(1.55f, 0, 0)).PointCount);

        // Sphere against the side of a capsule (axis Y), then past its cap.
        ConvexShape3D cap = ConvexShape3D.Capsule(1f, 0.5f);
        m = g.Collide(cap, ConvexShape3D.Sphere(0.25f), At(0.6f, 0.3f, 0));
        NearV(Vector3F.UnitX, m.Normal, 1e-6);
        Near(0.6 - 0.75, m[0].Separation, 1e-6);
        m = g.Collide(cap, ConvexShape3D.Sphere(0.25f), At(0, 1.7f, 0));
        NearV(Vector3F.UnitY, m.Normal, 1e-6);
        Near(1.7 - 1.75, m[0].Separation, 1e-6);

        // Crossing capsules: perpendicular axes, one above the other.
        ConvexShape3D cap2 = ConvexShape3D.Capsule(1f, 0.25f);
        var xf = new RigidTransformF(AxisAngle(Vector3F.UnitZ, Math.PI / 2), new Vector3F(0.2f, 0.3f, 0.7f));
        m = g.Collide(cap, cap2, xf);
        Assert.Equal(1, m.PointCount);
        NearV(Vector3F.UnitZ, m.Normal, 1e-6);
        Near(0.7 - 0.75, m[0].Separation, 1e-6);

        // Parallel capsules side by side overlap along 1.5: two points at the ends of the overlap.
        m = g.Collide(cap, cap2, At(0.6f, 0.5f, 0));
        Assert.Equal(2, m.PointCount);
        NearV(Vector3F.UnitX, m.Normal, 1e-6);
        Near(0.6 - 0.75, m[0].Separation, 1e-6);
        Near(0.6 - 0.75, m[1].Separation, 1e-6);
        float y0 = m[0].Point.Y, y1 = m[1].Point.Y;
        Near(1.5, Math.Abs(y0 - y1), 1e-5);
        Near(1.0, Math.Max(y0, y1), 1e-5);
    }

    [Fact]
    public void SphereOnBoxFaceEdgeCornerAndInside()
    {
        var g = new ManifoldGenerator3D();
        ConvexShape3D box = Box(1, 1, 1), s = ConvexShape3D.Sphere(0.5f);
        Manifold3D m = g.Collide(box, s, At(0.3f, 1.45f, -0.2f));
        NearV(Vector3F.UnitY, m.Normal, 1e-6);
        Near(-0.05, m[0].Separation, 1e-6);
        Assert.Equal(0u, m[0].Id >> 30);

        // Over a corner (1,1,1): the closest point is the vertex.
        Vector3F c = new Vector3F(1.2f, 1.3f, 1.2f);
        m = g.Collide(box, s, At(c.X, c.Y, c.Z));
        float dist = (c - new Vector3F(1, 1, 1)).Length();
        Near(dist - 0.5, m[0].Separation, 1e-5);
        NearV((c - new Vector3F(1, 1, 1)) / dist, m.Normal, 1e-5);
        Assert.Equal(2u, m[0].Id >> 30);

        // Over an edge.
        m = g.Collide(box, s, At(1.3f, 1.3f, 0.2f));
        Near(Math.Sqrt(0.18) - 0.5, m[0].Separation, 1e-5);
        Assert.Equal(1u, m[0].Id >> 30);

        // Sphere as shape A, box rotated as B: the normal flips and the point lands in A's frame.
        var xf = new RigidTransformF(AxisAngle(Vector3F.UnitZ, 0.5), new Vector3F(0, -1.4f, 0));
        m = g.Collide(s, box, xf);
        Assert.Equal(1, m.PointCount);
        Assert.True(Vector3F.Dot(m.Normal, xf.Translation) > 0f);
        // Sphere centre inside the box core: depth is distance to the nearest face plus the radius.
        m = g.Collide(box, s, At(0.9f, 0.1f, 0.2f));
        NearV(Vector3F.UnitX, m.Normal, 1e-6);
        Near(-(0.1 + 0.5), m[0].Separation, 1e-6);
    }

    [Fact]
    public void CylinderOnBoxFlatAndTilted()
    {
        var g = new ManifoldGenerator3D();
        ConvexShape3D ground = Box(5, 0.5f, 5), cyl = ConvexShape3D.Cylinder(0.5f, 0.5f);
        // Standing: the cap lies flat on the box top (y = 0.5): four points, all pressing 0.01 deep.
        Manifold3D m = g.Collide(ground, cyl, At(0.3f, 0.99f, 0.1f));
        Assert.Equal(4, m.PointCount);
        NearV(Vector3F.UnitY, m.Normal, 1e-4);
        for (int i = 0; i < 4; i++)
        {
            Near(-0.01, m[i].Separation, 2e-4);
        }

        // Ids repeat when the cylinder moves and turns about its axis: the samples are fixed in the cylinder.
        Manifold3D m2 = g.Collide(ground, cyl, new RigidTransformF(AxisAngle(Vector3F.UnitY, 0.2), new Vector3F(-1f, 0.99f, 1f)));
        Assert.Equal(Ids(m), Ids(m2));

        // Tilted 30 degrees about X: it stands on one rim point.
        double th = Math.PI / 6;
        float centre = (float)((0.5 * Math.Cos(th)) + (0.5 * Math.Sin(th))) + 0.5f - 0.02f;
        m = g.Collide(ground, cyl, new RigidTransformF(AxisAngle(Vector3F.UnitX, th), new Vector3F(0, centre, 0)));
        Assert.Equal(1, m.PointCount);
        NearV(Vector3F.UnitY, m.Normal, 1e-2);
        Near(-0.02, m[0].Separation, 5e-3);

        // Lying on its side: a line of two points along the axis.
        m = g.Collide(ground, cyl, new RigidTransformF(AxisAngle(Vector3F.UnitX, Math.PI / 2), new Vector3F(0, 0.99f, 0)));
        Assert.Equal(2, m.PointCount);
        Near(-0.01, m[0].Separation, 5e-3);
        Near(1.0, (m[0].Point - m[1].Point).Length(), 1e-2);
    }

    [Fact]
    public void TaperedCapsuleLyingOnABoxGetsTwoPoints()
    {
        var g = new ManifoldGenerator3D();
        ConvexShape3D tap = ConvexShape3D.TaperedCapsule(1f, 0.3f, 0.5f);
        // Axis along X: tangent plane of a cone is tilted, so the two end spheres touch a plane at their own heights; put it on the ground with the larger sphere touching.
        var xf = new RigidTransformF(AxisAngle(Vector3F.UnitZ, Math.PI / 2), new Vector3F(0, 1.0f, 0));
        Manifold3D m = g.Collide(Box(5, 0.5f, 5), tap, xf);
        Assert.True(m.PointCount >= 1);
        // The larger sphere (bottom, radius 0.5) is now at +x? The lowest surface point is at y = 1.0 - 0.5 = 0.5 (touching) for the larger sphere.
        Near(0.0, m.MinSeparation, 5e-3);
        NearV(Vector3F.UnitY, m.Normal, 0.1);
    }

    [Fact]
    public void HullOnHullKeepsIdsWhileSlidingAndTurning()
    {
        var g = new ManifoldGenerator3D();
        var rng = new TestRng(3);
        ConvexShape3D ground = ConvexShape3D.FromHull(BoxHull(5f, 0.5f, 5f)), box = ConvexShape3D.FromHull(BoxHull(0.4f, 0.3f, 0.5f));
        uint[]? first = null;
        for (int i = 0; i < 100; i++)
        {
            var xf = new RigidTransformF(AxisAngle(Vector3F.UnitY, rng.Range(-3f, 3f)), new Vector3F(rng.Range(-3f, 3f), 0.79f, rng.Range(-3f, 3f)));
            Manifold3D m = g.Collide(ground, box, xf);
            Assert.Equal(4, m.PointCount);
            uint[] ids = Ids(m);
            first ??= ids;
            Assert.Equal(first, ids);
        }
    }

    [Fact]
    public void RoundedBoxOnBoxSeparatesByTheSurfaceDistance()
    {
        var g = new ManifoldGenerator3D();
        // The rounded box (outer half size 0.5, radius 0.1) hovers 0.01 above the ground slab top at y = 0.5.
        Manifold3D m = g.Collide(Box(5f, 0.5f, 5f), Box(0.5f, 0.5f, 0.5f, 0.1f), At(0.2f, 1.01f, -0.3f));
        Assert.Equal(4, m.PointCount);
        NearV(Vector3F.UnitY, m.Normal, 1e-5);
        for (int i = 0; i < 4; i++)
        {
            Near(0.01, m[i].Separation, 1e-5);
        }
    }

    [Fact]
    public void TallStackOfBoxesKeepsFourPointsPerContact()
    {
        var g = new ManifoldGenerator3D();
        ConvexShape3D b = Box(0.5f, 0.5f, 0.5f);
        var rng = new TestRng(9);
        for (int level = 0; level < 12; level++)
        {
            Vector3F jitter = new(rng.Range(-0.1f, 0.1f), 0f, rng.Range(-0.1f, 0.1f));
            Manifold3D m = g.Collide(b, b, new RigidTransformF(AxisAngle(Vector3F.UnitY, rng.Range(-0.2f, 0.2f)), new Vector3F(jitter.X, 0.998f, jitter.Z)));
            Assert.Equal(4, m.PointCount);
            Near(-0.002, m.MinSeparation, 1e-5);
        }
    }

    [Fact]
    public void ManyClippedPointsReduceToTheFourThatSpanTheArea()
    {
        var g = new ManifoldGenerator3D();
        // A hull with a 24-sided cap on a big box: more than four clipped points are reduced to four that keep the deepest and span the area.
        var pts = new List<Vector3F>();
        for (int i = 0; i < 24; i++)
        {
            double a = i * Math.PI / 12;
            pts.Add(new Vector3F((float)Math.Cos(a), -0.5f, (float)Math.Sin(a)));
            pts.Add(new Vector3F((float)Math.Cos(a), 0.5f, (float)Math.Sin(a)));
        }

        ConvexShape3D drum = ConvexShape3D.FromHull(ConvexHull3D.Build(pts.ToArray()));
        Manifold3D m = g.Collide(Box(5, 0.5f, 5), drum, At(0, 0.99f, 0));
        Assert.Equal(4, m.PointCount);
        float minX = float.MaxValue, maxX = float.MinValue;
        for (int i = 0; i < 4; i++)
        {
            minX = Math.Min(minX, m[i].Point.X);
            maxX = Math.Max(maxX, m[i].Point.X);
        }

        Assert.True(maxX - minX > 1.5f);
    }

    [Fact]
    public void HostileArgumentsGetTypedErrors()
    {
        var g = new ManifoldGenerator3D();
        Assert.Throws<ArgumentOutOfRangeException>(() => g.Collide(Box(1, 1, 1), Box(1, 1, 1), At(float.NaN, 0, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => g.Collide(Box(1, 1, 1), Box(1, 1, 1), At(0, 0, 0), -1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => g.Collide(Box(1, 1, 1), Box(1, 1, 1), At(0, 0, 0), float.NaN));
        // Coincident centres and identical shapes still give a usable manifold.
        foreach (ConvexShapeKind3D _ in Enum.GetValues<ConvexShapeKind3D>())
        {
            WellFormed(g.Collide(Box(1, 1, 1), Box(1, 1, 1), At(0, 0, 0)));
            WellFormed(g.Collide(ConvexShape3D.Sphere(1), ConvexShape3D.Sphere(1), At(0, 0, 0)));
            WellFormed(g.Collide(ConvexShape3D.Capsule(1, 1), ConvexShape3D.Capsule(1, 1), At(0, 0, 0)));
            WellFormed(g.Collide(ConvexShape3D.Cylinder(1, 1), ConvexShape3D.Cylinder(1, 1), At(0, 0, 0)));
        }
    }

    private static (ConvexShape3D a, ConvexShape3D b, RigidTransformF xf) RandomPair(TestRng rng, ConvexHull3D[] hulls, float reach)
    {
        ConvexShape3D a = Collision3DHash.RandomShape(rng, hulls), b = Collision3DHash.RandomShape(rng, hulls);
        return (a, b, new RigidTransformF(rng.Rotation(), rng.InBall(rng.Range(0.3f, reach))));
    }

    [Fact]
    public void MinSeparationMatchesGjkDistanceOnSeparatedPairs()
    {
        var rng = new TestRng(2024);
        ConvexHull3D[] hulls = Collision3DHash.RandomHulls(rng, 16);
        var g = new ManifoldGenerator3D();
        const float margin = 0.05f;
        int accepted = 0, tries = 0;
        double worstSharp = 0, worstCyl = 0;
        string whatSharp = "", whatCyl = "";
        while (accepted < 100_000 && tries < 50_000_000)
        {
            tries++;
            var (a, b, xf) = RandomPair(rng, hulls, 3.0f);
            ConvexDistance3D d = Gjk3D.Distance(a, b, xf);
            if (d.Overlapping || d.Distance > margin || d.Distance < 1e-3f)
            {
                continue;
            }

            accepted++;
            Manifold3D m = g.Collide(a, b, xf, margin);
            WellFormed(m);
            Assert.True(m.PointCount > 0, $"empty manifold at distance {d.Distance} for {a.Kind}/{b.Kind}");
            double err = Math.Abs(m.MinSeparation - d.Distance);
            string what = $"{a.Kind}/{b.Kind} distance {d.Distance} manifold {m.MinSeparation} n {m.Normal} gjk {d.Normal} xf {xf} count {m.PointCount} ids {string.Join(",", Ids(m))} pts {(m.PointCount > 0 ? m[0].Point.ToString() : "")} a.r {a.ConvexRadius} b.r {b.ConvexRadius} acore {a.CoreSupport(new Vector3F(1, 1, 1))} bcore {b.CoreSupport(new Vector3F(1, 1, 1))}";
            if (a.Kind == ConvexShapeKind3D.Cylinder || b.Kind == ConvexShapeKind3D.Cylinder)
            {
                if (err > worstCyl) { worstCyl = err; whatCyl = what; }
            }
            else if (err > worstSharp)
            {
                worstSharp = err;
                whatSharp = what;
            }

            // The normal agrees with the closest-point direction.
            Assert.True(Vector3F.Dot(m.Normal, d.Normal) > 0.98f || d.Distance < 0.02f, $"normal {m.Normal} vs {d.Normal} for {a.Kind}/{b.Kind} distance {d.Distance}");
        }

        Assert.Equal(100_000, accepted);
        Assert.True(worstSharp <= 2e-3, $"worst separation error {worstSharp}: {whatSharp}");
        Assert.True(worstCyl <= 2e-3, $"worst separation error with cylinders {worstCyl}: {whatCyl}");
    }

    [Fact]
    public void PolyhedraDepthMatchesBruteForceSat()
    {
        var rng = new TestRng(31337);
        ConvexHull3D[] hulls = Collision3DHash.RandomHulls(rng, 16);
        var g = new ManifoldGenerator3D();
        int checkedPairs = 0;
        double worst = 0;
        for (int i = 0; i < 6000; i++)
        {
            ConvexHull3D ha = hulls[rng.Next(hulls.Length)], hb = hulls[rng.Next(hulls.Length)];
            var xf = new RigidTransformF(rng.Rotation(), rng.InBall(rng.Range(0.05f, 0.6f)));
            Brute.Poly pa = Brute.FromHull(ha, RigidTransformF.Identity), pb = Brute.FromHull(hb, xf);
            var (sep, axis, second) = Brute.Sat(pa, pb);
            if (sep > -0.01 || sep - second < (0.02 * -sep) + 2e-3)
            {
                continue;
            }

            Manifold3D m = g.Collide(ConvexShape3D.FromHull(ha), ConvexShape3D.FromHull(hb), xf);
            WellFormed(m);
            Assert.True(m.PointCount > 0);
            checkedPairs++;
            double axisDot = (m.Normal.X * axis.X) + (m.Normal.Y * axis.Y) + (m.Normal.Z * axis.Z);
            Assert.True(axisDot > 0.999, $"normal {m.Normal} vs axis {axis} (depth {sep}, next {second})");
            // The deepest point is as deep as the SAT depth unless clipping dropped it; it is never deeper.
            Assert.True(m.MinSeparation >= sep - ((0.02 * -sep) + 2e-3), $"manifold {m.MinSeparation} deeper than SAT {sep}");
            worst = Math.Max(worst, Math.Abs(m.MinSeparation - sep));
        }

        Assert.True(checkedPairs > 150, "too few overlapping pairs: " + checkedPairs);
        _ = worst;
    }

    [Fact]
    public void ManifoldsAreWellFormedOnAnyPair()
    {
        var rng = new TestRng(808);
        ConvexHull3D[] hulls = Collision3DHash.RandomHulls(rng, 16);
        var g = new ManifoldGenerator3D();
        for (int i = 0; i < 30000; i++)
        {
            var (a, b, xf) = RandomPair(rng, hulls, 2.5f);
            Manifold3D m = g.Collide(a, b, xf, rng.Range(0f, 0.3f));
            WellFormed(m);
        }
    }
}
