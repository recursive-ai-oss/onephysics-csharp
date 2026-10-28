using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

/// <summary>The closest-point capsule-against-box route, against hand-worked answers and against the general GJK/EPA route on random poses.</summary>
public class CapsuleBoxTests
{
    private const float Margin = 0.02f;
    private static readonly QuaternionF AxisAlongX = new(0f, 0f, 0.70710678f, 0.70710678f);

    private static ConvexShape3D Box() => ConvexShape3D.Box(new Vector3F(1f, 1f, 1f));

    private static ConvexShape3D Capsule(float halfHeight, float radius) => ConvexShape3D.Capsule(halfHeight, radius);

    // Box is A at the origin; the capsule is B.
    private static Manifold3D Collide(ConvexShape3D cap, QuaternionF q, Vector3F at, bool general = false)
    {
        var gen = new ManifoldGenerator3D { GeneralCapsuleBox = general };
        return gen.Collide(Box(), cap, new RigidTransformF(q, at), Margin);
    }

    [Fact]
    public void LyingParallelToAFaceGivesBothEnds()
    {
        var m = Collide(Capsule(0.5f, 0.3f), AxisAlongX, new Vector3F(0f, 1.29f, 0f));
        Assert.Equal(2, m.PointCount);
        Assert.True(m.Normal.Y > 0.9999f);
        Assert.InRange(m[0].Separation, -0.0101f, -0.0099f);
        Assert.InRange(m[1].Separation, -0.0101f, -0.0099f);
        Assert.True(Math.Abs(m[0].Point.X - m[1].Point.X) > 0.9f);
    }

    [Fact]
    public void ParallelToAnEdgeLeansOnTheEdgeDiagonal()
    {
        // Core segment along x at (y, z) = (1.2, 1.2): its distance to the top-front edge is 0.2 * sqrt(2).
        var m = Collide(Capsule(0.5f, 0.3f), AxisAlongX, new Vector3F(0f, 1.2f, 1.2f));
        Assert.Equal(2, m.PointCount);
        Assert.InRange(m.Normal.Y, 0.7070f, 0.7072f);
        Assert.InRange(m.Normal.Z, 0.7070f, 0.7072f);
        Assert.InRange(m[0].Separation, 0.2828427f - 0.3f - 1e-4f, 0.2828427f - 0.3f + 1e-4f);
    }

    [Fact]
    public void EndCapOnAFaceIsOnePointStraightDown()
    {
        var m = Collide(Capsule(0.5f, 0.3f), QuaternionF.Identity, new Vector3F(0.2f, 1f + 0.5f + 0.3f - 0.01f, -0.3f));
        Assert.Equal(1, m.PointCount);
        Assert.True(m.Normal.Y > 0.9999f);
        Assert.InRange(m[0].Separation, -0.0101f, -0.0099f);
    }

    [Fact]
    public void SkewToAnEdgeUsesTheEdgeAndGivesTheDiagonalNormal()
    {
        // The capsule axis runs (1, -1, 0) / sqrt 2 past the box edge along z at x = y = 1; its middle is the nearest point, 0.3 * sqrt 2 from the edge.
        var q = new QuaternionF(0f, 0f, -0.92387953f, 0.38268343f);
        var m = Collide(Capsule(0.5f, 0.45f), q, new Vector3F(1.3f, 1.3f, 0f));
        Assert.Equal(1, m.PointCount);
        Assert.InRange(m.Normal.X, 0.7070f, 0.7072f);
        Assert.InRange(m.Normal.Y, 0.7070f, 0.7072f);
        Assert.InRange(m[0].Separation, 0.4242641f - 0.45f - 1e-4f, 0.4242641f - 0.45f + 1e-4f);
    }

    [Fact]
    public void DeepPairsTakeTheGeneralRouteAndAgreeWithItExactly()
    {
        // An end of the core segment inside the core box.
        var cap = Capsule(0.5f, 0.3f);
        var fast = Collide(cap, QuaternionF.Identity, new Vector3F(0.1f, 1.1f, 0f));
        var slow = Collide(cap, QuaternionF.Identity, new Vector3F(0.1f, 1.1f, 0f), true);
        Assert.Equal(slow.PointCount, fast.PointCount);
        Assert.Equal(slow.Normal, fast.Normal);
        for (int i = 0; i < fast.PointCount; i++) { Assert.Equal(slow[i].Separation, fast[i].Separation); Assert.Equal(slow[i].Point, fast[i].Point); }
    }

    [Fact]
    public void SpeculativeBoundaryIsDecidedByTheGap()
    {
        var cap = Capsule(0.5f, 0.3f);
        Assert.Equal(0, Collide(cap, QuaternionF.Identity, new Vector3F(0f, 1.8f + Margin + 1e-3f, 0f)).PointCount);
        var m = Collide(cap, QuaternionF.Identity, new Vector3F(0f, 1.8f + Margin - 1e-3f, 0f));
        Assert.Equal(1, m.PointCount);
        Assert.InRange(m[0].Separation, Margin - 1.1e-3f, Margin);
    }

    [Fact]
    public void TouchingIsOnePointAtZero()
    {
        var m = Collide(Capsule(0.5f, 0.3f), QuaternionF.Identity, new Vector3F(0f, 1.8f, 0f));
        Assert.Equal(1, m.PointCount);
        Assert.InRange(m[0].Separation, -1e-4f, 1e-4f);
    }

    [Fact]
    public void RandomPosesAgreeWithGjkEpa()
    {
        var rng = new TestRng(2024);
        var fast = new ManifoldGenerator3D();
        var slow = new ManifoldGenerator3D { GeneralCapsuleBox = true };
        int touching = 0, compared = 0;
        for (int i = 0; i < 30000; i++)
        {
            ConvexShape3D cap = Capsule(rng.Range(0.1f, 1f), rng.Range(0.1f, 0.5f));
            ConvexShape3D box = ConvexShape3D.Box(new Vector3F(rng.Range(0.3f, 1.2f), rng.Range(0.3f, 1.2f), rng.Range(0.3f, 1.2f)), rng.Next(2) == 0 ? 0f : rng.Range(0f, 0.2f));
            bool capIsA = rng.Next(2) == 0;
            ConvexShape3D a = capIsA ? cap : box, b = capIsA ? box : cap;
            var xf = new RigidTransformF(rng.Rotation(), rng.InBall(rng.Range(0.3f, 2.2f)));
            Manifold3D mf = fast.Collide(a, b, xf, Margin), ms = slow.Collide(a, b, xf, Margin);
            ConvexDistance3D d = Gjk3D.Distance(a, b, xf);
            float gap = d.Overlapping ? -1f : d.Distance;
            // Near the speculative boundary the two routes may round to different sides.
            if (!d.Overlapping && Math.Abs(gap - Margin) < 2e-3f) { continue; }
            compared++;
            Assert.True(mf.PointCount == 0 == (ms.PointCount == 0), $"pose {i}: {mf.PointCount} points against {ms.PointCount}");
            if (mf.PointCount == 0) { continue; }
            touching++;
            Assert.True(Vector3F.Dot(mf.Normal, ms.Normal) > 0.999f, $"pose {i}: normals {mf.Normal} and {ms.Normal}");
            float minF = float.MaxValue, minS = float.MaxValue;
            for (int k = 0; k < mf.PointCount; k++) { minF = Math.Min(minF, mf[k].Separation); }
            for (int k = 0; k < ms.PointCount; k++) { minS = Math.Min(minS, ms[k].Separation); }
            Assert.True(Math.Abs(minF - minS) < 2e-3f, $"pose {i}: deepest separation {minF} against {minS}");
        }

        Assert.True(touching > 1500, $"{touching} touching of {compared}");
    }
}
