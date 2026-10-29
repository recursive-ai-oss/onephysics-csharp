using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

/// <summary>The dedicated box-against-box separating axis test against the general polyhedron route (the same boxes given as hulls) and against known configurations.</summary>
public class BoxBoxTests
{
    private static ConvexShape3D BoxAsHull(Vector3F h, float radius)
    {
        var pts = new Vector3F[8];
        h -= new Vector3F(radius, radius, radius); // the box's half extents include its rounding
        for (int i = 0; i < 8; i++) { pts[i] = new Vector3F((i & 1) == 0 ? -h.X : h.X, (i & 2) == 0 ? -h.Y : h.Y, (i & 4) == 0 ? -h.Z : h.Z); }
        return ConvexShape3D.FromHull(ConvexHull3D.Build(pts), radius);
    }

    [Fact]
    public void AgreesWithTheGeneralRouteOnRandomPoses()
    {
        var rng = new TestRng(2024);
        var gen = new ManifoldGenerator3D();
        var ref3 = new ManifoldGenerator3D();
        int touching = 0, differing = 0, exact = 0, total = 40000;
        for (int i = 0; i < total; i++)
        {
            var ha = new Vector3F(rng.Range(0.2f, 1.2f), rng.Range(0.2f, 1.2f), rng.Range(0.2f, 1.2f));
            var hb = new Vector3F(rng.Range(0.2f, 1.2f), rng.Range(0.2f, 1.2f), rng.Range(0.2f, 1.2f));
            float ra = rng.Next(3) == 0 ? rng.Range(0f, 0.15f) : 0f, rb = rng.Next(3) == 0 ? rng.Range(0f, 0.15f) : 0f;
            var q = rng.Next(4) == 0 ? QuaternionF.Identity : rng.Rotation();
            var xf = new RigidTransformF(q, rng.InBall(rng.Range(0.3f, 2.8f)));
            Manifold3D m = gen.Collide(ConvexShape3D.Box(ha, ra), ConvexShape3D.Box(hb, rb), xf, 0.02f);
            Manifold3D e = ref3.Collide(BoxAsHull(ha, ra), BoxAsHull(hb, rb), xf, 0.02f);
            if (m.PointCount > 0 || e.PointCount > 0) { touching++; }
            if ((m.PointCount == 0) != (e.PointCount == 0))
            {
                // A pair this close to the speculative distance may fall either side of it.
                differing++;
                float s = m.PointCount > 0 ? m.MinSeparation : e.MinSeparation;
                Assert.True(s > 0.02f - 2e-3f, $"pair {i}: box route {m.PointCount} points, hull route {e.PointCount}, separation {s}; EPA {new Epa3D().Collide(ConvexShape3D.Box(ha, ra), ConvexShape3D.Box(hb, rb), xf).Separation}; ha {ha} hb {hb} ra {ra} rb {rb} xf {xf}");
                continue;
            }

            if (m.PointCount == 0) { continue; }
            Assert.True(Vector3F.Dot(m.Normal, e.Normal) > 0.995f, $"pair {i}: normals {m.Normal} and {e.Normal}");
            if (Math.Abs(m.MinSeparation - e.MinSeparation) >= 3e-3f)
            {
                ConvexContact3D tr = new Epa3D().Collide(ConvexShape3D.Box(ha, ra), ConvexShape3D.Box(hb, rb), xf);
                if (Math.Abs(m.MinSeparation - tr.Separation) < 3e-3f) { exact++; continue; }
                Assert.Fail($"pair {i}: depth {m.MinSeparation} n {m.Normal} against {e.MinSeparation} n {e.Normal}; EPA {tr.Separation} n {tr.Normal}; ha {ha} hb {hb} ra {ra} rb {rb} xf {xf}");
            }
        }

        Assert.True(touching > 3000, $"{touching} touching pairs");
        Assert.True(differing < total / 200, $"{differing} pairs differ in touching");
    }

    [Fact]
    public void SeparatedBoxesNearEachOtherHaveTheGjkDistanceWhateverFeatureIsNearest()
    {
        // Pairs a few millimetres apart: edge against edge, vertex against face and the cases between. The closed-form edge pair must give GJK's distance wherever it is used.
        var rng = new TestRng(77);
        var gen = new ManifoldGenerator3D();
        int compared = 0;
        for (int i = 0; i < 60000 && compared < 3000; i++)
        {
            var ha = new Vector3F(rng.Range(0.2f, 1.2f), rng.Range(0.2f, 1.2f), rng.Range(0.2f, 1.2f));
            var hb = new Vector3F(rng.Range(0.2f, 1.2f), rng.Range(0.2f, 1.2f), rng.Range(0.2f, 1.2f));
            float ra = rng.Next(3) == 0 ? rng.Range(0f, 0.15f) : 0f;
            ConvexShape3D a = ConvexShape3D.Box(ha, ra), b = ConvexShape3D.Box(hb, 0f);
            var q = rng.Rotation();
            // Slide B along a random direction until it is just apart, found by bisection on GJK's distance.
            Vector3F dir = rng.InBall(1f);
            if (!(dir.LengthSquared() > 1e-4f)) { continue; }
            dir = dir.Normalized();
            float lo = 0f, hi = 4f;
            if (Gjk3D.Distance(a, b, new RigidTransformF(q, dir * hi)).Overlapping) { continue; }
            for (int k = 0; k < 40; k++)
            {
                float mid = 0.5f * (lo + hi);
                if (Gjk3D.Distance(a, b, new RigidTransformF(q, dir * mid)).Overlapping) { lo = mid; } else { hi = mid; }
            }

            var xf = new RigidTransformF(q, dir * (hi + rng.Range(0.002f, 0.015f)));
            ConvexDistance3D d = Gjk3D.Distance(a, b, xf);
            if (d.Overlapping || d.Distance > 0.019f || d.Distance < 0.001f) { continue; }
            Manifold3D m = gen.Collide(a, b, xf, 0.02f);
            Assert.True(m.PointCount > 0, $"pose {i}: no points at distance {d.Distance}");
            compared++;
            Assert.True(Math.Abs(m.MinSeparation - d.Distance) < 5e-4f, $"pose {i}: separation {m.MinSeparation} against GJK {d.Distance}");
        }

        Assert.True(compared > 500, $"{compared} pairs compared");
    }

    [Fact]
    public void StackedBoxesMakeAFourPointFaceManifold()
    {
        var gen = new ManifoldGenerator3D();
        // B sits on A with 0.005 between the faces: four points, normal +y, separation 0.005.
        Manifold3D m = gen.Collide(ConvexShape3D.Box(new Vector3F(1f, 0.5f, 1f)), ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f)), new RigidTransformF(QuaternionF.Identity, new Vector3F(0.2f, 1.005f, -0.1f)), 0.02f);
        Assert.Equal(4, m.PointCount);
        Assert.True(m.Normal.Y > 0.9999f);
        Assert.InRange(m.MinSeparation, 0.0049f, 0.0051f);
        // Pressed in by 0.01: separation -0.01.
        m = gen.Collide(ConvexShape3D.Box(new Vector3F(1f, 0.5f, 1f)), ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f)), new RigidTransformF(QuaternionF.Identity, new Vector3F(0f, 0.99f, 0f)), 0.02f);
        Assert.Equal(4, m.PointCount);
        Assert.InRange(m.MinSeparation, -0.0101f, -0.0099f);
        // Hair apart: just beyond the speculative distance gives nothing.
        m = gen.Collide(ConvexShape3D.Box(new Vector3F(1f, 0.5f, 1f)), ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f)), new RigidTransformF(QuaternionF.Identity, new Vector3F(0f, 1.0201f, 0f)), 0.02f);
        Assert.Equal(0, m.PointCount);
    }

    [Fact]
    public void FaceContactIsTheSameWhicheverBoxOwnsTheReferenceFace()
    {
        var gen = new ManifoldGenerator3D();
        var big = ConvexShape3D.Box(new Vector3F(1f, 0.5f, 1f));
        var small = ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f));
        // small on big (A is the reference), then big under small (B is the reference): the same four points seen from the other side.
        Manifold3D m1 = gen.Collide(big, small, new RigidTransformF(QuaternionF.Identity, new Vector3F(0.2f, 1.005f, -0.1f)), 0.02f);
        Manifold3D m2 = gen.Collide(small, big, new RigidTransformF(QuaternionF.Identity, new Vector3F(-0.2f, -1.005f, 0.1f)), 0.02f);
        Assert.Equal(4, m1.PointCount);
        Assert.Equal(4, m2.PointCount);
        Assert.True(m1.Normal.Y > 0.9999f);
        Assert.True(m2.Normal.Y < -0.9999f);
        Assert.InRange(m2.MinSeparation, 0.0049f, 0.0051f);
        // The points are in the small box (A) frame and sit at its four lower corners.
        for (int i = 0; i < m2.PointCount; i++)
        {
            Assert.InRange(Math.Abs(m2[i].Point.X), 0.49f, 0.51f);
            Assert.InRange(Math.Abs(m2[i].Point.Z), 0.49f, 0.51f);
        }
    }

    [Fact]
    public void CrossedEdgesMakeOnePoint()
    {
        var gen = new ManifoldGenerator3D();
        // Two long thin boxes crossing at right angles, B above A by 0.01 between the edges' surfaces: edge against edge.
        var a = ConvexShape3D.Box(new Vector3F(2f, 0.1f, 0.1f));
        var b = ConvexShape3D.Box(new Vector3F(2f, 0.1f, 0.1f));
        var q = QuaternionF.CreateFromAxisAngle(new Vector3F(0f, 1f, 0f), 1.5707964f);
        Manifold3D m = gen.Collide(a, b, new RigidTransformF(q, new Vector3F(0f, 0.21f, 0f)), 0.02f);
        Assert.True(m.PointCount >= 1);
        Assert.True(m.Normal.Y > 0.999f);
        Assert.InRange(m.MinSeparation, 0.0099f, 0.0101f);
        // Turned 45 degrees about the stacking axis and tilted: the same answer through the general route.
        var tilt = QuaternionF.CreateFromAxisAngle(new Vector3F(1f, 0f, 0f), 0.7f) * QuaternionF.CreateFromAxisAngle(new Vector3F(0f, 1f, 0f), 0.6f);
        var xf = new RigidTransformF(tilt, new Vector3F(0.3f, 0.35f, -0.2f));
        Manifold3D m1 = gen.Collide(a, b, xf, 0.02f);
        Manifold3D m2 = new ManifoldGenerator3D().Collide(BoxAsHull(new Vector3F(2f, 0.1f, 0.1f), 0f), BoxAsHull(new Vector3F(2f, 0.1f, 0.1f), 0f), xf, 0.02f);
        Assert.Equal(m2.PointCount > 0, m1.PointCount > 0);
        if (m1.PointCount > 0) { Assert.True(Vector3F.Dot(m1.Normal, m2.Normal) > 0.995f); Assert.True(Math.Abs(m1.MinSeparation - m2.MinSeparation) < 3e-3f); }
    }
}
