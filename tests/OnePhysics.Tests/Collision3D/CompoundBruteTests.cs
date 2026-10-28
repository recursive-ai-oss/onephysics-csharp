using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

/// <summary>Compound queries, child-pair enumeration and edits against brute force over every child.</summary>
public class CompoundBruteTests
{
    private const int Iterations = 100_000;

    private static readonly ConvexHull3D[] Hulls = Collision3DHash.RandomHulls(new TestRng(1234), 10);

    private static ConvexShape3D RandomShape(TestRng rng)
    {
        ConvexShape3D s = Collision3DHash.RandomShape(rng, Hulls);
        if (rng.Next(4) != 0)
        {
            return s;
        }

        float a = rng.Range(0.6f, 1.8f), b = rng.Range(0.6f, 1.8f);
        Vector3F scale = ScaledShape3D.GetScaleSupport(s) switch
        {
            ScaleSupport3D.PerAxis => new Vector3F(a, b, rng.Range(0.6f, 1.8f)),
            ScaleSupport3D.AxisSymmetricY => new Vector3F(a, b, a),
            _ => new Vector3F(a, a, a),
        };
        return ScaledShape3D.Create(s, scale);
    }

    private static CompoundChild3D RandomChild(TestRng rng, float spread) =>
        new(RandomShape(rng), new RigidTransformF(rng.Rotation(), rng.InBall(spread)), rng.Next(5));

    private static CompoundChild3D[] RandomChildren(TestRng rng, int n, float spread)
    {
        var kids = new CompoundChild3D[n];
        for (int i = 0; i < n; i++)
        {
            kids[i] = RandomChild(rng, spread);
        }

        return kids;
    }

    private static Compound3D[] Pool(TestRng rng, int count, int maxChildren, float spread)
    {
        var pool = new Compound3D[count];
        for (int i = 0; i < count; i++)
        {
            pool[i] = Compound3D.Create(RandomChildren(rng, 1 + rng.Next(maxChildren), spread));
        }

        return pool;
    }

    private static Vector3F Center(CompoundShape3D c, int child) => c.GetChild(child).Local.Translation;

    private static bool BruteRay(CompoundShape3D c, Vector3F o, Vector3F d, float maxF, out ConvexCastHit3D best, out int bestChild)
    {
        best = default;
        bestChild = -1;
        for (int i = 0; i < c.SlotCount; i++)
        {
            if (!c.IsLive(i))
            {
                continue;
            }

            PlacedShape3D p = c.GetChild(i);
            if (Gjk3D.RayCast(p.Shape, p.Local, o, d, maxF, out ConvexCastHit3D h) && (bestChild < 0 || h.Fraction < best.Fraction))
            {
                best = h;
                bestChild = i;
            }
        }

        return bestChild >= 0;
    }

    private static void CheckRay(CompoundShape3D c, TestRng rng, ref int hits)
    {
        Vector3F o = rng.InBall(8f);
        int live;
        do
        {
            live = rng.Next(c.SlotCount);
        }
        while (!c.IsLive(live));
        Vector3F d = rng.Next(5) == 0 ? rng.OnSphere() * rng.Range(0.5f, 10f) : (Center(c, live) + rng.InBall(1f)) - o;
        float maxF = rng.Next(3) == 0 ? 1f : rng.Range(0.5f, 3f);
        bool got = c.RayCast(o, d, maxF, out CompoundRayHit3D hit);
        bool want = BruteRay(c, o, d, maxF, out ConvexCastHit3D best, out int child);
        Assert.Equal(want, got);
        bool any = c.RayCastAny(o, d, maxF, out CompoundRayHit3D anyHit);
        Assert.Equal(want, any);
        if (!want)
        {
            return;
        }

        hits++;
        Assert.Equal(child, hit.Child);
        Assert.Equal(best.Fraction, hit.Fraction);
        Assert.Equal(best.PointB, hit.Point);
        Assert.Equal(best.Normal, hit.Normal);
        Assert.Equal(c.GetMaterialId(child), hit.MaterialId);
        Assert.Equal(best.StartsOverlapping, hit.StartsInside);
        // The any-hit result is a real hit of its child, not nearer than the closest one.
        PlacedShape3D ap = c.GetChild(anyHit.Child);
        Assert.True(Gjk3D.RayCast(ap.Shape, ap.Local, o, d, maxF, out ConvexCastHit3D ah));
        Assert.Equal(ah.Fraction, anyHit.Fraction);
        Assert.True(anyHit.Fraction >= hit.Fraction);
    }

    [Fact]
    public void RayCastsEqualBruteForce()
    {
        var rng = new TestRng(100);
        Compound3D[] pool = Pool(rng, 300, 24, 4f);
        int hits = 0;
        for (int i = 0; i < Iterations; i++)
        {
            CheckRay(pool[rng.Next(pool.Length)], rng, ref hits);
        }

        Assert.True(hits > Iterations / 4, "hits " + hits);
    }

    private static void CheckCast(CompoundShape3D c, TestRng rng, ref int hits)
    {
        ConvexShape3D shape;
        do
        {
            shape = RandomShape(rng);
        }
        while (shape.Kind == ConvexShapeKind3D.Sphere && shape.ConvexRadius == 0f);
        var pose = new RigidTransformF(rng.Rotation(), rng.InBall(7f));
        Vector3F motion = rng.Next(4) == 0 ? rng.InBall(3f) : (Center(c, SomeLive(c, rng)) - pose.Translation) * rng.Range(0.8f, 1.5f);
        bool got = c.ShapeCast(shape, pose, motion, out CompoundCastHit3D hit);

        RigidTransformF inverse = pose.Inverted();
        Vector3F inShape = inverse.Rotation.Rotate(motion);
        bool want = false;
        ConvexCastHit3D best = default;
        int child = -1;
        for (int i = 0; i < c.SlotCount; i++)
        {
            if (!c.IsLive(i))
            {
                continue;
            }

            PlacedShape3D p = c.GetChild(i);
            if (Gjk3D.ShapeCast(shape, p.Shape, inverse * p.Local, inShape, out ConvexCastHit3D h) && (child < 0 || h.Fraction < best.Fraction))
            {
                best = h;
                child = i;
                want = true;
            }
        }

        Assert.Equal(want, got);
        if (want)
        {
            hits++;
            Assert.Equal(child, hit.Child);
            Assert.Equal(best.Fraction, hit.Fraction);
            Assert.Equal(best.StartsOverlapping, hit.StartsOverlapping);
            Assert.Equal(c.GetMaterialId(child), hit.MaterialId);
            Assert.Equal(pose.TransformVector(best.Normal), hit.Normal);
        }
    }

    private static int SomeLive(CompoundShape3D c, TestRng rng)
    {
        int live;
        do
        {
            live = rng.Next(c.SlotCount);
        }
        while (!c.IsLive(live));
        return live;
    }

    [Fact]
    public void ShapeCastsEqualBruteForce()
    {
        var rng = new TestRng(200);
        Compound3D[] pool = Pool(rng, 300, 24, 4f);
        int hits = 0;
        for (int i = 0; i < Iterations; i++)
        {
            CheckCast(pool[rng.Next(pool.Length)], rng, ref hits);
        }

        Assert.True(hits > Iterations / 4, "hits " + hits);
    }

    private static List<int> BruteOverlap(CompoundShape3D c, double x0, double y0, double z0, double x1, double y1, double z1)
    {
        var list = new List<int>();
        for (int i = 0; i < c.SlotCount; i++)
        {
            if (!c.IsLive(i))
            {
                continue;
            }

            c.GetChildBounds(i, out Vector3F lo, out Vector3F hi);
            if (lo.X <= x1 && x0 <= hi.X && lo.Y <= y1 && y0 <= hi.Y && lo.Z <= z1 && z0 <= hi.Z)
            {
                list.Add(i);
            }
        }

        return list;
    }

    [Fact]
    public void BoundsOverlapEqualsBruteForce()
    {
        var rng = new TestRng(300);
        Compound3D[] pool = Pool(rng, 300, 24, 4f);
        var got = new List<int>();
        int nonEmpty = 0;
        for (int i = 0; i < Iterations; i++)
        {
            Compound3D c = pool[rng.Next(pool.Length)];
            Vector3F center = rng.InBall(6f), half = new(rng.Range(0f, 2f), rng.Range(0f, 2f), rng.Range(0f, 2f));
            Vector3F lo = center - half, hi = center + half;
            c.OverlapBounds(lo, hi, got);
            List<int> want = BruteOverlap(c, lo.X, lo.Y, lo.Z, hi.X, hi.Y, hi.Z);
            Assert.Equal(want, got);
            nonEmpty += want.Count > 0 ? 1 : 0;
        }

        Assert.True(nonEmpty > Iterations / 10);
    }

    [Fact]
    public void ChildrenTouchingAShapeAreNeverMissedByTheBounds()
    {
        var rng = new TestRng(310);
        Compound3D[] pool = Pool(rng, 100, 16, 3f);
        var ids = new List<int>();
        int touching = 0;
        for (int i = 0; i < 30_000; i++)
        {
            Compound3D c = pool[rng.Next(pool.Length)];
            ConvexShape3D probe;
            do
            {
                probe = RandomShape(rng);
            }
            while (probe.Kind == ConvexShapeKind3D.Sphere && probe.ConvexRadius == 0f);
            var pose = new RigidTransformF(rng.Rotation(), rng.InBall(4f));
            float margin = rng.Next(2) == 0 ? 0f : 0.05f;
            CompoundPairs3D.FindChildren(c, probe, pose, margin, ids);
            for (int k = 0; k < c.SlotCount; k++)
            {
                PlacedShape3D p = c.GetChild(k);
                ConvexDistance3D d = Gjk3D.Distance(probe, p.Shape, pose.Inverted() * p.Local);
                if (d.Overlapping || d.Distance <= margin)
                {
                    touching++;
                    Assert.Contains(k, ids);
                }
            }
        }

        Assert.True(touching > 3000, "touching " + touching);
    }

    [Fact]
    public void ChildPairEnumerationEqualsAllPairs()
    {
        var rng = new TestRng(400);
        Compound3D[] pool = Pool(rng, 200, 7, 3f);
        var pairs = new List<SubShapePair3D>();
        var ids = new List<int>();
        int found = 0;
        for (int it = 0; it < Iterations; it++)
        {
            Compound3D a = pool[rng.Next(pool.Length)], b = pool[rng.Next(pool.Length)];
            var bInA = new RigidTransformF(rng.Rotation(), rng.InBall(5f));
            float margin = rng.Next(3) switch { 0 => 0f, 1 => 0.02f, _ => 0.5f };
            CompoundPairs3D.FindPairs(a, b, bInA, margin, pairs);

            var want = new List<SubShapePair3D>();
            for (int i = 0; i < a.SlotCount; i++)
            {
                a.GetChildBounds(i, out Vector3F lo, out Vector3F hi);
                for (int j = 0; j < b.SlotCount; j++)
                {
                    PlacedShape3D cb = b.GetChild(j);
                    CompoundPairs3D.GetQueryBox(cb.Shape, bInA * cb.Local, margin, out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
                    if (lo.X <= x1 && x0 <= hi.X && lo.Y <= y1 && y0 <= hi.Y && lo.Z <= z1 && z0 <= hi.Z)
                    {
                        want.Add(new SubShapePair3D(i, j));
                    }
                }
            }

            Assert.Equal(want, pairs);
            found += want.Count;

            // The convex-against-compound form, with b's first child as the convex shape.
            PlacedShape3D probe = b.GetChild(0);
            CompoundPairs3D.FindChildren(a, probe.Shape, bInA * probe.Local, margin, ids);
            var wantIds = new List<int>();
            foreach (SubShapePair3D p in want)
            {
                if (p.B == 0)
                {
                    wantIds.Add(p.A);
                }
            }

            Assert.Equal(wantIds, ids);
        }

        Assert.True(found > Iterations, "pairs found " + found);
    }

    [Fact]
    public void ChildPairManifoldsEqualAllPairsAndCoverEveryContact()
    {
        var rng = new TestRng(500);
        Compound3D[] pool = Pool(rng, 100, 6, 2.5f);
        var g = new ManifoldGenerator3D();
        var reference = new ManifoldGenerator3D();
        var results = new List<ChildContact3D>();
        int contacts = 0;
        for (int it = 0; it < 20_000; it++)
        {
            Compound3D a = pool[rng.Next(pool.Length)], b = pool[rng.Next(pool.Length)];
            var bInA = new RigidTransformF(rng.Rotation(), rng.InBall(3f));
            float spec = ManifoldGenerator3D.DefaultSpeculativeDistance;
            g.CollideCompoundCompound(a, b, bInA, results, spec);
            int k = 0;
            for (int i = 0; i < a.SlotCount; i++)
            {
                for (int j = 0; j < b.SlotCount; j++)
                {
                    Manifold3D m = reference.Collide(a.GetChild(i), b.GetChild(j), bInA, spec);
                    if (m.PointCount == 0)
                    {
                        continue;
                    }

                    contacts++;
                    Assert.True(k < results.Count, $"iteration {it}: pair ({i}, {j}) has a manifold but was not enumerated");
                    ChildContact3D c = results[k++];
                    Assert.Equal((i, j), (c.SubShapeA, c.SubShapeB));
                    Assert.Equal(m.PointCount, c.Manifold.PointCount);
                    Assert.Equal(m.Normal, c.Manifold.Normal);
                    for (int p = 0; p < m.PointCount; p++)
                    {
                        Assert.Equal(m[p].Point, c.Manifold[p].Point);
                        Assert.Equal(m[p].Separation, c.Manifold[p].Separation);
                        Assert.Equal(m[p].Id, c.Manifold[p].Id);
                    }
                }
            }

            Assert.Equal(k, results.Count);
        }

        Assert.True(contacts > 2000, "contacts " + contacts);
    }

    // ---- mutable against static ------------------------------------------------------------------------------------------------------------------

    [Fact]
    public void MutableCompoundAfterRandomEditsEqualsARebuiltStaticOne()
    {
        var rng = new TestRng(600);
        int queries = 0, rays = 0, casts = 0;
        var gotIds = new List<int>();
        var wantIds = new List<int>();
        for (int seq = 0; seq < 2000; seq++)
        {
            var m = new MutableCompound3D(RandomChildren(rng, 3 + rng.Next(8), 4f));
            int edits = 20 + rng.Next(20);
            for (int e = 0; e < edits; e++)
            {
                int op = rng.Next(10);
                var live = new List<int>();
                for (int i = 0; i < m.SlotCount; i++)
                {
                    if (m.IsLive(i))
                    {
                        live.Add(i);
                    }
                }

                if (op < 4 || live.Count == 0)
                {
                    m.Add(RandomChild(rng, 4f));
                }
                else if (op < 7)
                {
                    m.Remove(m.GetHandle(live[rng.Next(live.Count)]));
                }
                else
                {
                    m.Move(m.GetHandle(live[rng.Next(live.Count)]), new RigidTransformF(rng.Rotation(), rng.InBall(4f)));
                }

                if (e % 8 == 0)
                {
                    m.Validate();
                }
            }

            m.Validate();
            if (rng.Next(4) == 0)
            {
                m.Optimize();
                m.Validate();
            }

            if (m.Count == 0)
            {
                Assert.False(m.RayCast(default, Vector3F.UnitX, 5f, out _));
                continue;
            }

            var order = new List<int>();
            var kids = new List<CompoundChild3D>();
            for (int i = 0; i < m.SlotCount; i++)
            {
                if (m.IsLive(i))
                {
                    order.Add(i);
                    PlacedShape3D p = m.GetChild(i);
                    kids.Add(new CompoundChild3D(p, m.GetMaterialId(i)));
                }
            }

            Compound3D fresh = Compound3D.Create(kids.ToArray());
            Assert.Equal(fresh.GetMassProperties(), m.GetMassProperties());
            Assert.Equal(fresh.TryGetBounds(out Vector3F flo, out Vector3F fhi), m.TryGetBounds(out Vector3F mlo, out Vector3F mhi));
            Assert.Equal((flo, fhi), (mlo, mhi));

            for (int q = 0; q < 50; q++)
            {
                queries++;
                Vector3F o = rng.InBall(8f);
                Vector3F d = (Center(fresh, rng.Next(fresh.Count)) + rng.InBall(1f)) - o;
                float maxF = rng.Range(0.8f, 2.5f);
                bool a = fresh.RayCast(o, d, maxF, out CompoundRayHit3D fh);
                bool b = m.RayCast(o, d, maxF, out CompoundRayHit3D mh);
                Assert.Equal(a, b);
                if (a)
                {
                    rays++;
                    Assert.Equal(order[fh.Child], mh.Child);
                    Assert.Equal(fh.Fraction, mh.Fraction);
                    Assert.Equal(fh.Point, mh.Point);
                    Assert.Equal(fh.Normal, mh.Normal);
                    Assert.Equal(fh.MaterialId, mh.MaterialId);
                }

                Assert.Equal(fresh.RayCastAny(o, d, maxF, out _), m.RayCastAny(o, d, maxF, out _));

                Vector3F center = rng.InBall(6f), half = new(rng.Range(0f, 2f), rng.Range(0f, 2f), rng.Range(0f, 2f));
                fresh.OverlapBounds(center - half, center + half, wantIds);
                m.OverlapBounds(center - half, center + half, gotIds);
                Assert.Equal(wantIds.Count, gotIds.Count);
                for (int k = 0; k < gotIds.Count; k++)
                {
                    Assert.Equal(order[wantIds[k]], gotIds[k]);
                }

                ConvexShape3D shape;
                do
                {
                    shape = RandomShape(rng);
                }
                while (shape.Kind == ConvexShapeKind3D.Sphere && shape.ConvexRadius == 0f);
                var pose = new RigidTransformF(rng.Rotation(), rng.InBall(6f));
                Vector3F motion = (Center(fresh, rng.Next(fresh.Count)) - pose.Translation) * rng.Range(0.8f, 1.4f);
                bool ca = fresh.ShapeCast(shape, pose, motion, out CompoundCastHit3D fc);
                bool cb = m.ShapeCast(shape, pose, motion, out CompoundCastHit3D mc);
                Assert.Equal(ca, cb);
                if (ca)
                {
                    casts++;
                    Assert.Equal(order[fc.Child], mc.Child);
                    Assert.Equal(fc.Fraction, mc.Fraction);
                    Assert.Equal(fc.Normal, mc.Normal);
                }

                CompoundPairs3D.FindChildren(fresh, shape, pose, 0.02f, wantIds);
                CompoundPairs3D.FindChildren(m, shape, pose, 0.02f, gotIds);
                Assert.Equal(wantIds.Count, gotIds.Count);
                for (int k = 0; k < gotIds.Count; k++)
                {
                    Assert.Equal(order[wantIds[k]], gotIds[k]);
                }
            }
        }

        Assert.True(queries >= 90_000);
        Assert.True(rays > queries / 5 && casts > queries / 5, $"rays {rays}, casts {casts}");
    }
}
