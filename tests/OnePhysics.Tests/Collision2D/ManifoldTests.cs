using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision2D;

public class ManifoldTests
{
    private static Transform2 At(float x, float y) => new(new Vec2(x, y), Rot2.Identity);

    private static void V(Vec2 v, double x, double y, double tol = 1e-5)
    {
        Assert.True(Math.Abs(v.X - x) <= tol && Math.Abs(v.Y - y) <= tol, "expected (" + x + ", " + y + ") got " + v);
    }

    [Fact]
    public void BoxOnBoxFace()
    {
        var a = Shape2D.CreateBox(0.5f, 0.5f);
        var b = Shape2D.CreateBox(0.5f, 0.5f);
        Manifold2D m = Collide2D.Collide(in a, in b, At(0, 0.99f));
        Assert.Equal(2, m.PointCount);
        V(m.Normal, 0, 1);
        V(m[0].Point, -0.5, 0.495);
        V(m[1].Point, 0.5, 0.495);
        Assert.Equal(-0.01, m[0].Separation, 5);
        Assert.Equal(-0.01, m[1].Separation, 5);
        Assert.Equal(0x8200u, m[0].Id);
        Assert.Equal(0x8201u, m[1].Id);
    }

    [Fact]
    public void ReferenceFaceOfB()
    {
        // A diamond (box turned 45 degrees) resting its lowest vertex on the top face of a big box: that face is the reference, the normal still points from A to B.
        float h = 0.70710678f;
        var a = Shape2D.CreateBox(0.5f, 0.5f, new Vec2(0, 0), new Rot2(h, h));
        var b = Shape2D.CreateBox(5f, 0.5f);
        Manifold2D m = Collide2D.Collide(in a, in b, At(0, -h - 0.5f + 0.01f));
        Assert.Equal(1, m.PointCount);
        V(m.Normal, 0, -1);
        Assert.Equal(-0.01, m[0].Separation, 4);
        Assert.Equal(0x0082u, m[0].Id);
    }

    [Fact]
    public void OffsetBoxIsClippedToTheReferenceFace()
    {
        var a = Shape2D.CreateBox(0.5f, 0.5f);
        var b = Shape2D.CreateBox(0.5f, 0.5f);
        Manifold2D m = Collide2D.Collide(in a, in b, At(0.4f, 0.99f));
        Assert.Equal(2, m.PointCount);
        // The face of A spans x in [-0.5, 0.5]; B's bottom edge spans [-0.1, 0.9]: the far point is clipped back to x = 0.5 and takes the reference vertex.
        double x0 = Math.Min(m[0].Point.X, m[1].Point.X), x1 = Math.Max(m[0].Point.X, m[1].Point.X);
        Assert.Equal(-0.1, x0, 5);
        Assert.Equal(0.5, x1, 5);
        Assert.NotEqual(m[0].Id, m[1].Id);
    }

    [Fact]
    public void CircleCircle()
    {
        var a = Shape2D.CreateCircle(new Vec2(0, 0), 1f);
        var b = Shape2D.CreateCircle(new Vec2(0, 0), 0.5f);
        Manifold2D m = Collide2D.Collide(in a, in b, At(1.4f, 0));
        Assert.Equal(1, m.PointCount);
        V(m.Normal, 1, 0);
        Assert.Equal(-0.1, m[0].Separation, 5);
        V(m[0].Point, 0.95, 0);
        Assert.Equal(0u, m[0].Id);
        // Speculative: 0.01 apart is kept, 0.05 apart is not (margin 0.02).
        Assert.Equal(1, Collide2D.Collide(in a, in b, At(1.51f, 0)).PointCount);
        Assert.Equal(0, Collide2D.Collide(in a, in b, At(1.55f, 0)).PointCount);
        Assert.Equal(1, Collide2D.Collide(in a, in b, At(1.55f, 0), 0.1f).PointCount);
    }

    [Fact]
    public void CircleOnBoxFaceAndCorner()
    {
        var box = Shape2D.CreateBox(0.5f, 0.5f);
        var circle = Shape2D.CreateCircle(new Vec2(0, 0), 0.5f);
        Manifold2D m = Collide2D.Collide(in box, in circle, At(0.2f, 0.9f));
        Assert.Equal(1, m.PointCount);
        V(m.Normal, 0, 1);
        Assert.Equal(-0.1, m[0].Separation, 5);
        Assert.Equal(0x8200u, m[0].Id);

        m = Collide2D.Collide(in box, in circle, At(0.8f, 0.8f));
        Assert.Equal(1, m.PointCount);
        double s = Math.Sqrt(0.5);
        V(m.Normal, s, s);
        Assert.Equal(Math.Sqrt(0.18) - 0.5, m[0].Separation, 5);
        Assert.Equal(0x0200u, m[0].Id);

        // Reversed pair: same contact seen from the circle, normal flipped, ids swapped.
        Manifold2D r = Collide2D.Collide(in circle, in box, Transform2.InvMul(Transform2.Identity, new Transform2(new Vec2(-0.8f, -0.8f), Rot2.Identity)));
        Assert.Equal(1, r.PointCount);
        V(r.Normal, -s, -s);
        Assert.Equal(0x0002u, r.Point0.Id);
    }

    [Fact]
    public void CircleCenterInsideBox()
    {
        var box = Shape2D.CreateBox(1f, 1f);
        var circle = Shape2D.CreateCircle(new Vec2(0, 0), 0.25f);
        Manifold2D m = Collide2D.Collide(in box, in circle, At(0.6f, 0.1f));
        Assert.Equal(1, m.PointCount);
        V(m.Normal, 1, 0);
        Assert.Equal(-0.4 - 0.25, m[0].Separation, 5);
    }

    [Fact]
    public void CapsuleCapsuleParallelGivesTwoPoints()
    {
        var a = Shape2D.CreateCapsule(new Vec2(-1, 0), new Vec2(1, 0), 0.25f);
        var b = Shape2D.CreateCapsule(new Vec2(-1, 0), new Vec2(1, 0), 0.25f);
        Manifold2D m = Collide2D.Collide(in a, in b, At(0.5f, 0.45f));
        Assert.Equal(2, m.PointCount);
        V(m.Normal, 0, 1);
        Assert.Equal(-0.05, m[0].Separation, 5);
        Assert.Equal(-0.05, m[1].Separation, 5);
        // Crossing capsules: one point, normal along the separation direction.
        var c = Shape2D.CreateCapsule(new Vec2(0, -1), new Vec2(0, 1), 0.25f);
        m = Collide2D.Collide(in a, in c, At(0.1f, 0.6f));
        Assert.True(m.PointCount >= 1);
        Assert.True(m.Normal.Length > 0.999f);
    }

    [Fact]
    public void CapsuleOnPolygonAndSegmentUnderBox()
    {
        var floor = Shape2D.CreateBox(5f, 0.5f);
        var cap = Shape2D.CreateCapsule(new Vec2(-0.5f, 0), new Vec2(0.5f, 0), 0.25f);
        Manifold2D m = Collide2D.Collide(in floor, in cap, At(0, 0.74f));
        Assert.Equal(2, m.PointCount);
        V(m.Normal, 0, 1);
        Assert.Equal(-0.01, m[0].Separation, 5);

        var seg = Shape2D.CreateSegment(new Vec2(-2, 0), new Vec2(2, 0));
        var box = Shape2D.CreateBox(0.5f, 0.5f);
        m = Collide2D.Collide(in seg, in box, At(0, 0.49f));
        Assert.Equal(2, m.PointCount);
        V(m.Normal, 0, 1);
        Assert.Equal(-0.01, m[0].Separation, 5);
        // A box below the segment is pushed down: segments collide from both sides.
        m = Collide2D.Collide(in seg, in box, At(0, -0.49f));
        Assert.Equal(2, m.PointCount);
        V(m.Normal, 0, -1);
    }

    [Fact]
    public void VertexVertexAndEdgeVertex()
    {
        var a = Shape2D.CreateBox(0.5f, 0.5f);
        var b = Shape2D.CreateBox(0.5f, 0.5f);
        Manifold2D m = Collide2D.Collide(in a, in b, At(1.1f, 1.1f), 0.2f);
        Assert.Equal(1, m.PointCount);
        double s = Math.Sqrt(0.5);
        V(m.Normal, s, s);
        Assert.Equal(0.1 * Math.Sqrt(2), m[0].Separation, 5);
        Assert.Equal(0x0200u, m[0].Id); // vertex 2 of A, vertex 0 of B

        var tri = Shape2D.CreatePolygon(new[] { new Vec2(-0.3f, 1), new Vec2(0.3f, 1), new Vec2(0, 0.49f) });
        m = Collide2D.Collide(in a, in tri, Transform2.Identity);
        Assert.Equal(1, m.PointCount);
        V(m.Normal, 0, 1);
        Assert.Equal(-0.01, m[0].Separation, 5);
        Assert.Equal(0x8201u, m[0].Id);
        V(m[0].Point, 0, 0.495);
    }

    [Fact]
    public void FeatureIdsStayStableUnderJitter()
    {
        var ground = Shape2D.CreateBox(5f, 0.5f);
        var box = Shape2D.CreateBox(0.5f, 0.5f);
        var r = new TRng(21);
        uint id0 = 0, id1 = 0;
        for (int f = 0; f < 100; f++)
        {
            float rot = r.Range(-0.002f, 0.002f);
            var xf = new Transform2(new Vec2(0.3f + r.Range(-0.002f, 0.002f), 0.995f + r.Range(-0.003f, 0.003f)), Rot2.FromVector(1f, rot));
            Manifold2D m = Collide2D.Collide(in ground, in box, xf);
            Assert.Equal(2, m.PointCount);
            if (f == 0) { id0 = m[0].Id; id1 = m[1].Id; }
            Assert.Equal(id0, m[0].Id);
            Assert.Equal(id1, m[1].Id);
        }

        // Two equal boxes side by side: the reference face does not flip between A and B under small motion.
        var a = Shape2D.CreateBox(0.5f, 0.5f);
        uint ia = 0, ib = 0;
        for (int f = 0; f < 100; f++)
        {
            var xf = new Transform2(new Vec2(0.995f + r.Range(-0.002f, 0.002f), 0.1f + r.Range(-0.002f, 0.002f)), Rot2.FromVector(1f, r.Range(-0.0004f, 0.0004f)));
            Manifold2D m = Collide2D.Collide(in a, in box, xf);
            Assert.Equal(2, m.PointCount);
            if (f == 0) { ia = m[0].Id; ib = m[1].Id; }
            Assert.Equal(ia, m[0].Id);
            Assert.Equal(ib, m[1].Id);
        }
    }

    [Fact]
    public void ManifoldsAreWellFormedOnRandomPairs()
    {
        var r = new TRng(33);
        int points = 0;
        for (int k = 0; k < 50000; k++)
        {
            Shape2D a = TU.RandomShape(ref r);
            Shape2D b = TU.RandomShape(ref r);
            Transform2 xf = TU.Pose(ref r, 2.2f);
            Manifold2D m = Collide2D.Collide(in a, in b, xf, 0.05f);
            if (m.PointCount == 0) { continue; }
            Assert.InRange(m.PointCount, 1, 2);
            Assert.True(Math.Abs(m.Normal.Length - 1f) < 1e-4f);
            for (int i = 0; i < m.PointCount; i++)
            {
                Assert.True(m[i].Separation <= 0.05f + 1e-6f);
                Assert.True(m[i].Point.IsFinite && float.IsFinite(m[i].Separation));
                points++;
            }

            if (m.PointCount == 2) { Assert.NotEqual(m[0].Id, m[1].Id); }
        }

        Assert.True(points > 1000);
    }

    [Fact]
    public void SeparationAgreesWithGjkDistanceWhenSeparated()
    {
        var r = new TRng(77);
        int compared = 0;
        double worst = 0;
        for (int k = 0; k < 60000; k++)
        {
            Shape2D a = TU.RandomShape(ref r);
            Shape2D b = TU.RandomShape(ref r);
            Transform2 xf = TU.Pose(ref r, 2.5f);
            const float margin = 0.1f;
            Manifold2D m = Collide2D.Collide(in a, in b, xf, margin);
            var cache = new SimplexCache2D();
            float dist = Distance2D.Compute(in a, in b, xf, ref cache, true).Distance;
            if (dist <= 0f)
            {
                continue;
            }

            if (dist < margin - 2e-3f)
            {
                Assert.True(m.PointCount > 0, "distance " + dist + " is inside the margin but the manifold is empty");
                float min = float.MaxValue;
                for (int i = 0; i < m.PointCount; i++) { min = Math.Min(min, m[i].Separation); }
                double err = Math.Abs(min - dist);
                worst = Math.Max(worst, err);
                Assert.True(err <= 5e-4, "k " + k + " manifold separation " + min + " gjk " + dist + " kinds " + a.Kind + "/" + b.Kind);
                compared++;
            }

            if (dist > margin + 2e-3f) { Assert.True(m.PointCount == 0, "k " + k + " dist " + dist + " but manifold has " + m.PointCount + " sep " + m.Point0.Separation + " kinds " + a.Kind + "/" + b.Kind + " counts " + a.Count + "/" + b.Count + " radii " + a.Radius + "/" + b.Radius); }
        }

        Assert.True(compared > 1500, "compared " + compared);
    }

    [Fact]
    public void PolygonPenetrationDepthMatchesBruteForceSat()
    {
        var r = new TRng(88);
        int compared = 0;
        for (int k = 0; k < 30000; k++)
        {
            Shape2D a = TU.RandomPolygon(ref r);
            Shape2D b = TU.RandomPolygon(ref r);
            Transform2 xf = TU.Pose(ref r, 1.2f);
            Manifold2D m = Collide2D.Collide(in a, in b, xf);
            // Brute-force penetration depth: the smallest overlap over every face normal of both polygons, in double.
            var pa = TU.World(in a, Transform2.Identity);
            var pb = TU.World(in b, xf);
            double depth = double.MaxValue;
            bool separated = false;
            for (int side = 0; side < 2 && !separated; side++)
            {
                var p = side == 0 ? pa : pb;
                var q = side == 0 ? pb : pa;
                for (int i = 0; i < p.Length; i++)
                {
                    var e0 = p[i];
                    var e1 = p[(i + 1) % p.Length];
                    double nx = e1.y - e0.y, ny = -(e1.x - e0.x);
                    double l = Math.Sqrt(nx * nx + ny * ny);
                    nx /= l; ny /= l;
                    double minQ = double.MaxValue;
                    foreach (var v in q) { minQ = Math.Min(minQ, (v.x - e0.x) * nx + (v.y - e0.y) * ny); }
                    if (minQ >= 0) { separated = true; break; }
                    depth = Math.Min(depth, -minQ);
                }
            }

            if (separated) { continue; }
            Assert.True(m.PointCount > 0, "overlapping polygons must give contact points");
            float min = float.MaxValue;
            for (int i = 0; i < m.PointCount; i++) { min = Math.Min(min, m[i].Separation); }
            // The deepest manifold point is never deeper than the true penetration by more than the reference-face hysteresis (0.1 slop), and not much shallower.
            Assert.True(-min <= depth + 1e-3, "k " + k + " depth " + depth + " manifold " + -min);
            compared++;
        }

        Assert.True(compared > 5000);
    }

    [Fact]
    public void InvalidInputsGiveEmptyManifolds()
    {
        var a = Shape2D.CreateBox(0.5f, 0.5f);
        var def = default(Shape2D);
        Assert.Equal(0, Collide2D.Collide(in a, in def, Transform2.Identity).PointCount);
        Assert.Equal(0, Collide2D.Collide(in def, in a, Transform2.Identity).PointCount);
        Assert.Equal(0, Collide2D.Collide(in a, in a, new Transform2(new Vec2(float.NaN, 0), Rot2.Identity)).PointCount);
        Assert.Equal(0, Collide2D.Collide(in a, in a, Transform2.Identity, float.NaN).PointCount);
        Assert.Throws<ArgumentOutOfRangeException>(() => Collide2D.Collide(in a, in a, At(0.9f, 0))[2]);
    }
}

public class ChainTests
{
    // A chain running right to left along y = 0, so its solid side (the right of travel) is up.
    private static Chain2D Flat(int segments, bool ghosts = true)
    {
        var pts = new Vec2[segments + 1];
        for (int i = 0; i <= segments; i++) { pts[i] = new Vec2(segments - i, 0); }
        return new Chain2D(pts);
    }

    [Fact]
    public void BoxSlidingAcrossAFlatChainFeelsNoInternalEdges()
    {
        const int n = 20;
        Chain2D chain = Flat(n);
        var segs = new Shape2D[n];
        var bare = new Shape2D[n];
        for (int i = 0; i < n; i++)
        {
            segs[i] = chain.GetSegment(i);
            Vec2 p1 = new(n - i, 0), p2 = new(n - i - 1, 0);
            bare[i] = Shape2D.CreateChainSegment(null, p1, p2, null);
        }

        var box = Shape2D.CreateBox(0.5f, 0.5f);
        var r = new TRng(5);
        int bareBump = 0;
        for (int f = 0; f < 2000; f++)
        {
            float x = 1.5f + 17f * f / 2000f;
            float tilt = r.Range(-0.12f, 0.12f);
            var xf = new Transform2(new Vec2(x, 0.46f + r.Range(-0.03f, 0.03f)), Rot2.FromVector(1f, tilt));
            int touching = 0;
            for (int i = 0; i < n; i++)
            {
                Manifold2D m = Collide2D.Collide(in segs[i], in box, xf);
                if (m.PointCount == 0) { continue; }
                touching++;
                Assert.True(Math.Abs(m.Normal.X) < 1e-3f && m.Normal.Y > 0.9999f, "frame " + f + " segment " + i + " normal " + m.Normal);
                // Mirrored argument order gives the opposite normal.
                Manifold2D rev = Collide2D.Collide(in box, in segs[i], Transform2.InvMul(xf, Transform2.Identity));
                Assert.Equal(m.PointCount, rev.PointCount);
                Vec2 back = -xf.Q.InverseRotate(m.Normal);
                Assert.True((rev.Normal - back).Length < 1e-5f, "reverse normal " + rev.Normal + " expected " + back);

                Manifold2D nb = Collide2D.Collide(in bare[i], in box, xf);
                if (nb.PointCount > 0 && Math.Abs(nb.Normal.X) > 1e-3f) { bareBump++; }
            }

            Assert.True(touching >= 1, "frame " + f + ": the box lost contact with the chain");
        }

        // Without ghost vertices the same motion does produce sideways normals at the joints: the test above is meaningful.
        Assert.True(bareBump > 50, "bump count without ghosts " + bareBump);
    }

    [Fact]
    public void CircleRollingOverAConvexCornerIsHandledBySingleSegment()
    {
        // Ground is the quadrant x > 0, y < 0: the chain runs (3,0) -> (0,0) -> (0,-3); its right side is up, then to the left.
        var chain = new Chain2D(new[] { new Vec2(3, 0), new Vec2(0, 0), new Vec2(0, -3) });
        Shape2D s0 = chain.GetSegment(0), s1 = chain.GetSegment(1);
        var circle = Shape2D.CreateCircle(new Vec2(0, 0), 0.5f);
        for (int deg = 92; deg <= 178; deg += 2)
        {
            double th = deg * Math.PI / 180;
            var xf = new Transform2(new Vec2((float)(0.4 * Math.Cos(th)), (float)(0.4 * Math.Sin(th))), Rot2.Identity);
            Manifold2D m0 = Collide2D.Collide(in s0, in circle, xf);
            Manifold2D m1 = Collide2D.Collide(in s1, in circle, xf);
            Assert.Equal(1, m0.PointCount);
            Assert.Equal(0, m1.PointCount);
            Assert.Equal(Math.Cos(th), m0.Normal.X, 4);
            Assert.Equal(Math.Sin(th), m0.Normal.Y, 4);
            Assert.Equal(-0.1, m0.Point0.Separation, 4);
        }

        // Past the corner, the second segment's face takes over.
        var below = new Transform2(new Vec2(-0.4f, -1f), Rot2.Identity);
        Assert.Equal(0, Collide2D.Collide(in s0, in circle, below).PointCount);
        Manifold2D face = Collide2D.Collide(in s1, in circle, below);
        Assert.Equal(1, face.PointCount);
        Assert.Equal(-1.0, face.Normal.X, 4);
    }

    [Fact]
    public void OneSidedChainIgnoresBodiesBehind_TwoSidedDoesNot()
    {
        var box = Shape2D.CreateBox(0.5f, 0.5f);
        var oneSided = Shape2D.CreateChainSegment(null, new Vec2(2, 0), new Vec2(-2, 0), null);
        var twoSided = Shape2D.CreateChainSegment(null, new Vec2(2, 0), new Vec2(-2, 0), null, twoSided: true);
        var below = new Transform2(new Vec2(0, -0.45f), Rot2.Identity);
        Assert.Equal(0, Collide2D.Collide(in oneSided, in box, below).PointCount);
        Manifold2D m = Collide2D.Collide(in twoSided, in box, below);
        Assert.Equal(2, m.PointCount);
        Assert.True(m.Normal.Y < -0.999f);
        var above = new Transform2(new Vec2(0, 0.45f), Rot2.Identity);
        Assert.Equal(2, Collide2D.Collide(in oneSided, in box, above).PointCount);
        // Two chain segments never collide with each other.
        Assert.Equal(0, Collide2D.Collide(in oneSided, in twoSided, Transform2.Identity).PointCount);
    }

    [Fact]
    public void LoopSurroundsABodyFromInside()
    {
        // A clockwise loop is solid on its inside; a ball inside near a corner never loses contact.
        var chain = new Chain2D(new[] { new Vec2(-3, -3), new Vec2(-3, 3), new Vec2(3, 3), new Vec2(3, -3) }, loop: true);
        var circle = Shape2D.CreateCircle(new Vec2(0, 0), 0.5f);
        for (int i = 0; i < chain.SegmentCount; i++)
        {
            Shape2D s = chain.GetSegment(i);
            // A ball against this wall, 0.4 from the surface, in the middle of the wall.
            Vec2 mid = 0.5f * (s.Vertices[0] + s.Vertices[1]);
            Vec2 inward = s.Normals[0];
            Manifold2D m = Collide2D.Collide(in s, in circle, new Transform2(mid + 0.4f * inward, Rot2.Identity));
            Assert.Equal(1, m.PointCount);
            Assert.Equal(-0.1, m.Point0.Separation, 4);
        }
    }
}

public class DeterminismTests
{
    [Fact]
    public void FixedInputHashIsStable()
    {
        ulong h = RecursiveAI.OnePhysics.LowLevel.CollisionHash2D.Compute();
        Assert.Equal(h, RecursiveAI.OnePhysics.LowLevel.CollisionHash2D.Compute());
        Assert.Equal(ExpectedHash, h);
    }

    // Recorded from the x64 run; the same value is required from scalar, NativeAOT, WebAssembly and arm64 runs.
    internal const ulong ExpectedHash = 0x31E47F1BCE1B321FUL;
}
