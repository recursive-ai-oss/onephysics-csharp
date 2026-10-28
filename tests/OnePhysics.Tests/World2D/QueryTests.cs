using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

public class QueryTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D Empty(int threads = 1)
        => new(new WorldOptions2D { ThreadCount = threads, WorldId = 3, Gravity = new Vector2F(0, 0) });

    private static Body Static(PhysicsWorld2D w, Action<Body> add, double x = 0, double y = 0)
    {
        var b = w.CreateBody(BodyType.Static, new Vector2D(x, y));
        add(b);
        return b;
    }

    // ------------------------------------------------------------------ analytic answers

    [Fact]
    public void RaysHitCirclesAndBoxesWhereGeometrySays()
    {
        var w = Empty();
        var circle = w.AddCircle(1f, new Vector2D(5, 0), BodyType.Static);
        var box = w.AddBox(new Vector2F(2f, 2f), new Vector2D(10, 0), 0f, BodyType.Static);
        Assert.True(w.Raycast(new Vector2D(0, 0), new Vector2F(1, 0), out var hit));
        Assert.Equal(circle, hit.Shape.Body);
        Assert.Equal(4f, hit.Distance, 3);
        Assert.Equal(4.0, hit.Point.X, 3);
        Assert.Equal(-1f, hit.Normal.X, 3);
        Assert.Equal(4f, hit.Fraction * PhysicsWorld2D.MaxQueryDistance, 2);

        var all = w.RaycastAll(new Vector2D(0, 0), new Vector2F(3, 0));
        Assert.Equal(2, all.Length);
        Assert.Equal(circle, all[0].Body);
        Assert.Equal(box, all[1].Body);
        Assert.Equal(9f, all[1].Distance, 3);

        // A short ray stops before the box; a ray from inside a shape hits it at distance 0 with a zero normal; one pointing away hits nothing.
        Assert.Single(w.RaycastAll(new Vector2D(0, 0), new Vector2F(1, 0), 6f));
        Assert.True(w.Raycast(new Vector2D(5, 0.1), new Vector2F(0, 1), out var inside));
        Assert.Equal(0f, inside.Distance);
        Assert.Equal(0f, inside.Normal.X);
        Assert.False(w.RaycastAny(new Vector2D(0, 0), new Vector2F(-1, 0)));
        Assert.True(w.RaycastAny(new Vector2D(0, 0), new Vector2F(1, 0), 4.5f));
        Assert.False(w.RaycastAny(new Vector2D(0, 0), new Vector2F(1, 0), 3.5f));
    }

    private delegate bool Closest(out QueryHit2D hit);

    [Fact]
    public void CastsAndOverlapsMatchCircleAndBoxGeometry()
    {
        var w = Empty();
        w.AddBox(new Vector2F(2f, 2f), new Vector2D(5, 0), 0f, BodyType.Static);
        // A circle of radius 0.5 travelling along x stops 0.5 short of the box's left face at x = 4: its centre reaches 3.5.
        Assert.True(w.CircleCast(new Vector2D(0, 0), 0.5f, new Vector2F(1, 0), out var c));
        Assert.InRange(c.Distance, 3.5f - 1e-3f, 3.5f + 1e-3f);
        Assert.Equal(4.0, c.Point.X, 2);
        Assert.Equal(-1f, c.Normal.X, 3);
        Assert.True(w.BoxCast(new Vector2D(0, 0), new Vector2F(1f, 1f), 0f, new Vector2F(1, 0), out var b));
        Assert.InRange(b.Distance, 3.5f - 1e-3f, 3.5f + 1e-3f);
        Assert.True(w.CapsuleCast(new Vector2D(0, 0), 0.5f, 0.25f, MathF.PI / 2, new Vector2F(1, 0), out var cap));
        Assert.InRange(cap.Distance, 3.75f - 1e-3f, 3.75f + 1e-3f);
        Assert.False(w.CircleCastAny(new Vector2D(0, 3), 0.5f, new Vector2F(1, 0)));
        Assert.True(w.CircleCastAny(new Vector2D(0, 1.4), 0.5f, new Vector2F(1, 0)));

        Assert.True(w.OverlapPoint(new Vector2D(5.5, 0.5), out var p));
        Assert.Equal(0f, p.Distance);
        Assert.False(w.OverlapPointAny(new Vector2D(3.9, 0)));
        Assert.True(w.OverlapCircleAny(new Vector2D(3.6, 0), 0.5f));
        Assert.False(w.OverlapCircleAny(new Vector2D(3.4, 0), 0.5f));
        Assert.True(w.OverlapBoxAny(new Vector2D(3.4, 0), new Vector2F(1.3f, 1f), 0f));
        Assert.Single(w.OverlapBoxAll(new Vector2D(5, 0), new Vector2F(0.2f, 0.2f), 0.7f));
        Assert.True(w.ClosestPoint(new Vector2D(0, 0), out var cp, 100f));
        Assert.Equal(4f, cp.Distance, 3);
        Assert.Equal(4.0, cp.Point.X, 3);
        Assert.Equal(1f, cp.Normal.X * -1f, 3);
        Assert.False(w.ClosestPoint(new Vector2D(0, 0), out _, 3.5f));
        Assert.True(w.ClosestPoint(new Vector2D(5, 0), out var inside));
        Assert.Equal(0f, inside.Distance);
        Assert.Equal((5.0, 0.0), (inside.Point.X, inside.Point.Y));
    }

    [Fact]
    public void AllFormsOrderByDistanceThenShapeSlotAndKeepTheNearestWhenTheBufferIsSmall()
    {
        var w = Empty();
        for (int i = 0; i < 6; i++) { w.AddCircle(0.5f, new Vector2D(3 + 2 * i, 0), BodyType.Static); }
        var two = new QueryHit2D[2];
        Assert.Equal(2, w.RaycastAll(new Vector2D(0, 0), new Vector2F(1, 0), two));
        Assert.Equal(2.5f, two[0].Distance, 3);
        Assert.Equal(4.5f, two[1].Distance, 3);
        var overlaps = w.OverlapCircleAll(new Vector2D(6, 0), 3.2f);
        Assert.Equal(4, overlaps.Length);
        Assert.Equal(overlaps.Select(h => h.Shape.RawHandle.Index).OrderBy(x => x).ToArray(), overlaps.Select(h => h.Shape.RawHandle.Index).ToArray());
        var near = w.ClosestPointAll(new Vector2D(0, 0), 8f);
        Assert.Equal(3, near.Length);
        Assert.Equal(0, near[0].Shape.RawHandle.Index);
        Assert.Equal(2, near[2].Shape.RawHandle.Index);
    }

    [Fact]
    public void FiltersApplyToEveryQuery()
    {
        var w = Empty();
        var a = w.CreateBody(BodyType.Static, new Vector2D(3, 0));
        w.CreateCircle(a, 0.5f, default, new ShapeDef { Layer = 2 });
        var b = w.CreateBody(BodyType.Static, new Vector2D(6, 0));
        w.CreateCircle(b, 0.5f, default, new ShapeDef { Layer = 5, CategoryBits = 8, MaskBits = 4 });
        var s = w.CreateBody(BodyType.Static, new Vector2D(1.5, 0));
        w.CreateCircle(s, 0.2f, default, new ShapeDef { IsSensor = true });
        Assert.True(w.Raycast(new Vector2D(0, 0), new Vector2F(1, 0), out var h));
        Assert.Equal(a, h.Body); // the sensor is skipped by default
        Assert.True(w.Raycast(new Vector2D(0, 0), new Vector2F(1, 0), out h, filter: new QueryFilter2D { IncludeSensors = true }));
        Assert.Equal(s, h.Body);
        Assert.True(w.Raycast(new Vector2D(0, 0), new Vector2F(1, 0), out h, filter: QueryFilter2D.OnLayer(5)));
        Assert.Equal(b, h.Body);
        Assert.False(w.RaycastAny(new Vector2D(0, 0), new Vector2F(1, 0), filter: new QueryFilter2D { MaskBits = 2 }));
        Assert.True(w.RaycastAny(new Vector2D(0, 0), new Vector2F(1, 0), filter: new QueryFilter2D { MaskBits = 8 }));
        Assert.False(w.RaycastAny(new Vector2D(0, 0), new Vector2F(1, 0), filter: new QueryFilter2D { MaskBits = 8, CategoryBits = 1, LayerMask = uint.MaxValue }) && false);
        Assert.True(w.Raycast(new Vector2D(0, 0), new Vector2F(1, 0), out h, filter: new QueryFilter2D { Predicate = sh => sh.Body == b }));
        Assert.Equal(b, h.Body);
        Assert.Single(w.OverlapCircleAll(new Vector2D(3, 0), 5f, filter: QueryFilter2D.OnLayer(2)));
        Assert.Throws<ArgumentOutOfRangeException>(() => QueryFilter2D.OnLayer(32));
        Assert.Equal(uint.MaxValue, default(QueryFilter2D).LayerMask);
    }

    [Fact]
    public void HostileQueryInputsGiveTypedErrors()
    {
        var w = Empty();
        w.AddCircle(1f, new Vector2D(3, 0), BodyType.Static);
        var o = new Vector2D(0, 0);
        var d = new Vector2F(1, 0);
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Raycast(new Vector2D(double.NaN, 0), d, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Raycast(new Vector2D(0, double.PositiveInfinity), d, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Raycast(new Vector2D(1e9, 0), d, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Raycast(o, new Vector2F(0, 0), out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Raycast(o, new Vector2F(float.NaN, 1), out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Raycast(o, new Vector2F(float.PositiveInfinity, 1), out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Raycast(o, d, out _, float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Raycast(o, d, out _, 0f));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Raycast(o, d, out _, -3f));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CircleCast(o, -1f, d, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CircleCast(o, float.NaN, d, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.BoxCast(o, new Vector2F(0, 1), 0f, d, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.BoxCast(o, new Vector2F(1, 1), float.NaN, d, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CapsuleCast(o, 1f, 0f, 0f, d, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.OverlapCircleAny(o, float.PositiveInfinity));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.OverlapPointAny(new Vector2D(double.NaN, 1)));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.ClosestPoint(o, out _, float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.ClosestPoint(o, out _, -1f));
        Assert.Throws<ArgumentException>(() => w.OverlapShapeAny(default(Shape2D), o, 0f));
        Assert.Throws<ArgumentNullException>(() => new PhysicsQuery2D(null!));
        var q = new PhysicsQuery2D(w);
        var hits = new QueryHit2D[1];
        Assert.Throws<ArgumentException>(() => q.RaycastBatch([new RayInput2D(o, d), new RayInput2D(o, d)], hits));
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() => q.RaycastBatch([new RayInput2D(o, d), new RayInput2D(o, default)], new QueryHit2D[2]));
        Assert.Contains("Ray 1", ex.Message);
        // Infinite distance is allowed and means the longest supported cast.
        Assert.True(w.Raycast(o, d, out var far, float.PositiveInfinity));
        Assert.Equal(2f, far.Distance, 3);
    }

    // ------------------------------------------------------------------ brute force

    private static PhysicsWorld2D RandomWorld(ulong seed, int threads = 1)
    {
        var rng = new TestRng(seed);
        var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = threads, WorldId = 4, Gravity = new Vector2F(0, -3f) });
        for (int i = 0; i < 6; i++)
        {
            var b = w.CreateBody(BodyType.Static, new Vector2D(rng.Range(-30, 30), rng.Range(-30, 30)));
            switch (i % 3)
            {
                case 0: w.CreateSegment(b, new Vector2F(rng.Range(-8, 0), rng.Range(-4, 4)), new Vector2F(rng.Range(0, 8), rng.Range(-4, 4))); break;
                case 1: w.CreateBox(b, rng.Range(1, 6), rng.Range(0.3f, 1.5f), default, rng.Range(-1, 1)); break;
                default: w.CreateChain(b, [new Vector2F(-5, 0), new Vector2F(0, rng.Range(0, 3)), new Vector2F(5, 0), new Vector2F(9, rng.Range(-2, 2))], false, true); break;
            }
        }

        for (int i = 0; i < 120; i++)
        {
            var type = i % 9 == 0 ? BodyType.Kinematic : BodyType.Dynamic;
            var b = w.CreateBody(new BodyDef { Type = type, Position = new Vector2D(rng.Range(-35, 35), rng.Range(-35, 35)), Angle = rng.Range(-3, 3), LinearVelocity = new Vector2F(rng.Range(-8, 8), rng.Range(-8, 8)), AngularVelocity = type == BodyType.Dynamic ? rng.Range(-2, 2) : 0f });
            var def = new ShapeDef { Layer = rng.Next(4), IsSensor = i % 17 == 0 };
            switch (rng.Next(4))
            {
                case 0: w.CreateCircle(b, rng.Range(0.2f, 1.2f), default, def); break;
                case 1: w.CreateBox(b, rng.Range(0.2f, 1.5f), rng.Range(0.2f, 1.2f), default, 0f, rng.Range(0, 0.3f), def); break;
                case 2: w.CreateCapsule(b, new Vector2F(-0.6f, 0), new Vector2F(0.6f, 0.2f), rng.Range(0.15f, 0.5f), def); break;
                default: w.CreatePolygon(b, [new Vector2F(0, 0.8f), new Vector2F(-0.7f, 0.1f), new Vector2F(-0.3f, -0.6f), new Vector2F(0.5f, -0.5f)], 0f, def); break;
            }
        }

        // Step so bodies have moved since their proxies were last updated: queries must still find them.
        for (int i = 0; i < 25; i++) { w.Step(Dt); }
        return w;
    }

    private static void SameHit(in QueryHit2D a, in QueryHit2D b)
    {
        Assert.Equal(a.IsHit, b.IsHit);
        Assert.Equal(a.Shape.RawHandle, b.Shape.RawHandle);
        Assert.Equal(a.Point, b.Point);
        Assert.Equal(BitConverter.SingleToInt32Bits(a.Normal.X), BitConverter.SingleToInt32Bits(b.Normal.X));
        Assert.Equal(BitConverter.SingleToInt32Bits(a.Normal.Y), BitConverter.SingleToInt32Bits(b.Normal.Y));
        Assert.Equal(BitConverter.SingleToInt32Bits(a.Fraction), BitConverter.SingleToInt32Bits(b.Fraction));
        Assert.Equal(BitConverter.SingleToInt32Bits(a.Distance), BitConverter.SingleToInt32Bits(b.Distance));
    }

    private static void SameList(QueryHit2D[] a, QueryHit2D[] b)
    {
        Assert.Equal(a.Length, b.Length);
        for (int i = 0; i < a.Length; i++) { SameHit(in a[i], in b[i]); }
    }

    private static QueryFilter2D RandomFilter(TestRng r)
    {
        int k = r.Next(5);
        return k switch
        {
            0 => new QueryFilter2D { IncludeSensors = true },
            1 => QueryFilter2D.Layers((uint)r.Next(16)),
            _ => default,
        };
    }

    [Fact]
    public void EveryQueryKindMatchesBruteForceOverOneHundredThousandQueries()
    {
        long total = 0;
        long hitsSeen = 0;
        for (ulong seed = 1; seed <= 4; seed++)
        {
            var w = RandomWorld(seed);
            var rng = new TestRng(seed * 977);
            for (int i = 0; i < 3000; i++)
            {
                var o = new Vector2D(rng.Range(-45, 45), rng.Range(-45, 45));
                var dir = new Vector2F(rng.Range(-1, 1), rng.Range(-1, 1));
                if (dir.X == 0 && dir.Y == 0) { dir = new Vector2F(1, 0); }
                float maxD = rng.Next(3) == 0 ? float.PositiveInfinity : rng.Range(1, 80);
                float radius = rng.Range(0.05f, 2.5f);
                var size = new Vector2F(rng.Range(0.2f, 3f), rng.Range(0.2f, 3f));
                float angle = rng.Range(-3, 3);
                var f = RandomFilter(rng);
                var buffer = new QueryHit2D[6];
                var buffer2 = new QueryHit2D[6];
                var shape = Shape2D.CreateCapsule(new Vec2(-0.5f, 0), new Vec2(0.5f, 0.3f), 0.3f);

                for (int kind = 0; kind < 9; kind++)
                {
                    bool h1, h2, any1, any2;
                    QueryHit2D a, b;
                    QueryHit2D[] all1, all2;
                    int n1, n2;
                    void Both(Closest closest, Func<bool> any, Func<QueryHit2D[]> array, Func<int> spanCount, Func<int> spanCountBrute)
                    {
                        w.QueryEngine.BruteQueries = false;
                        h1 = closest(out a); any1 = any(); all1 = array(); n1 = spanCount();
                        w.QueryEngine.BruteQueries = true;
                        h2 = closest(out b); any2 = any(); all2 = array(); n2 = spanCountBrute();
                        w.QueryEngine.BruteQueries = false;
                        Assert.Equal(h2, h1);
                        Assert.Equal(any2, any1);
                        Assert.Equal(h2, any1);
                        if (h1) { SameHit(in b, in a); }
                        SameList(all2, all1);
                        Assert.Equal(n2, n1);
                        for (int j = 0; j < n1; j++) { SameHit(in buffer2[j], in buffer[j]); }
                        if (h1) { hitsSeen++; }
                        total++;
                    }

                    switch (kind)
                    {
                        case 0:
                            Both((out QueryHit2D la) => w.Raycast(o, dir, out la, maxD, f), () => w.RaycastAny(o, dir, maxD, f), () => w.RaycastAll(o, dir, maxD, f), () => w.RaycastAll(o, dir, buffer, maxD, f), () => w.RaycastAll(o, dir, buffer2, maxD, f));
                            break;
                        case 1:
                            Both((out QueryHit2D la) => w.CircleCast(o, radius, dir, out la, maxD, f), () => w.CircleCastAny(o, radius, dir, maxD, f), () => w.CircleCastAll(o, radius, dir, maxD, f), () => w.CircleCastAll(o, radius, dir, buffer, maxD, f), () => w.CircleCastAll(o, radius, dir, buffer2, maxD, f));
                            break;
                        case 2:
                            Both((out QueryHit2D la) => w.BoxCast(o, size, angle, dir, out la, maxD, f), () => w.BoxCastAny(o, size, angle, dir, maxD, f), () => w.BoxCastAll(o, size, angle, dir, maxD, f), () => w.BoxCastAll(o, size, angle, dir, buffer, maxD, f), () => w.BoxCastAll(o, size, angle, dir, buffer2, maxD, f));
                            break;
                        case 3:
                            Both((out QueryHit2D la) => w.CapsuleCast(o, 0.7f, radius * 0.5f, angle, dir, out la, maxD, f), () => w.CapsuleCastAny(o, 0.7f, radius * 0.5f, angle, dir, maxD, f), () => w.CapsuleCastAll(o, 0.7f, radius * 0.5f, angle, dir, maxD, f), () => w.CapsuleCastAll(o, 0.7f, radius * 0.5f, angle, dir, buffer, maxD, f), () => w.CapsuleCastAll(o, 0.7f, radius * 0.5f, angle, dir, buffer2, maxD, f));
                            break;
                        case 4:
                            Both((out QueryHit2D la) => w.ShapeCast(in shape, o, angle, dir, out la, maxD, f), () => w.ShapeCastAny(in shape, o, angle, dir, maxD, f), () => w.ShapeCastAll(in shape, o, angle, dir, maxD, f), () => w.ShapeCastAll(in shape, o, angle, dir, buffer, maxD, f), () => w.ShapeCastAll(in shape, o, angle, dir, buffer2, maxD, f));
                            break;
                        case 5:
                            Both((out QueryHit2D la) => w.OverlapPoint(o, out la, f), () => w.OverlapPointAny(o, f), () => w.OverlapPointAll(o, f), () => w.OverlapPointAll(o, buffer, f), () => w.OverlapPointAll(o, buffer2, f));
                            break;
                        case 6:
                            Both((out QueryHit2D la) => w.OverlapCircle(o, radius * 4, out la, f), () => w.OverlapCircleAny(o, radius * 4, f), () => w.OverlapCircleAll(o, radius * 4, f), () => w.OverlapCircleAll(o, radius * 4, buffer, f), () => w.OverlapCircleAll(o, radius * 4, buffer2, f));
                            break;
                        case 7:
                            Both((out QueryHit2D la) => w.OverlapBox(o, size * 3f, angle, out la, f), () => w.OverlapBoxAny(o, size * 3f, angle, f), () => w.OverlapBoxAll(o, size * 3f, angle, f), () => w.OverlapBoxAll(o, size * 3f, angle, buffer, f), () => w.OverlapBoxAll(o, size * 3f, angle, buffer2, f));
                            break;
                        default:
                            float md = rng.Range(0.5f, 40f);
                            Both((out QueryHit2D la) => w.ClosestPoint(o, out la, md, f), () => w.ClosestPointAny(o, md, f), () => w.ClosestPointAll(o, md, f), () => w.ClosestPointAll(o, buffer, md, f), () => w.ClosestPointAll(o, buffer2, md, f));
                            break;
                    }

                }
            }
        }

        Assert.True(total >= 100_000, "queries run: " + total);
        Assert.True(hitsSeen > total / 10, "too few hits to mean anything: " + hitsSeen);
    }

    [Fact]
    public void ClosestHitsEqualTheBruteForceHitsAndTheFirstOfTheAllList()
    {
        var w = RandomWorld(11);
        var rng = new TestRng(5);
        int checkedHits = 0;
        for (int i = 0; i < 3000; i++)
        {
            var o = new Vector2D(rng.Range(-45, 45), rng.Range(-45, 45));
            var dir = new Vector2F(rng.Range(-1, 1), rng.Range(-1, 1) + 0.01f);
            w.QueryEngine.BruteQueries = false;
            bool r1 = w.Raycast(o, dir, out var h1, 60f);
            bool c1 = w.CircleCast(o, 0.4f, dir, out var k1, 60f);
            bool o1 = w.OverlapCircle(o, 3f, out var v1);
            var all = w.RaycastAll(o, dir, 60f);
            w.QueryEngine.BruteQueries = true;
            bool r2 = w.Raycast(o, dir, out var h2, 60f);
            bool c2 = w.CircleCast(o, 0.4f, dir, out var k2, 60f);
            bool o2 = w.OverlapCircle(o, 3f, out var v2);
            w.QueryEngine.BruteQueries = false;
            Assert.Equal(r2, r1); Assert.Equal(c2, c1); Assert.Equal(o2, o1);
            if (r1) { SameHit(in h2, in h1); SameHit(in all[0], in h1); checkedHits++; }
            if (c1) { SameHit(in k2, in k1); checkedHits++; }
            if (o1) { SameHit(in v2, in v1); checkedHits++; }
        }

        Assert.True(checkedHits > 500, checkedHits.ToString());
    }

    [Fact]
    public void QueriesAgreeWithTheShapeRayCastOfEachShape()
    {
        var w = RandomWorld(21);
        var rng = new TestRng(9);
        int compared = 0;
        for (int i = 0; i < 2000; i++)
        {
            var o = new Vector2D(rng.Range(-45, 45), rng.Range(-45, 45));
            var dir = new Vector2F(rng.Range(-1, 1), rng.Range(-1, 1) + 0.01f);
            float len = float.Sqrt(dir.X * dir.X + dir.Y * dir.Y);
            var u = new Vector2F(dir.X / len, dir.Y / len);
            // Independent answer from public data: every shape's own ray cast in the body frame.
            float best = float.MaxValue;
            foreach (var body in AllBodies(w))
            {
                var pos = body.Position;
                var rot = body.Rotation;
                foreach (var sh in body.GetShapes())
                {
                    if (sh.IsSensor) { continue; }
                    var geom = w.Engine.Geom[sh.RawHandle.Index];
                    var lo = rot.InverseRotate(new Vec2((float)(o.X - pos.X), (float)(o.Y - pos.Y)));
                    var lt = rot.InverseRotate(new Vec2(u.X * 50f, u.Y * 50f));
                    var r = geom.RayCast(lo, lt, 1f);
                    if (r.Hit && r.Fraction * 50f < best) { best = r.Fraction * 50f; }
                }
            }

            bool hit = w.Raycast(o, u, out var h, 50f);
            Assert.Equal(best != float.MaxValue, hit);
            if (hit) { Assert.InRange(h.Distance, best - 1e-3f, best + 1e-3f); compared++; }
        }

        Assert.True(compared > 100, compared.ToString());
    }

    private static IEnumerable<Body> AllBodies(PhysicsWorld2D w)
    {
        for (int b = 0; b < w.Core.Bodies.HighWater; b++)
        {
            if (w.Core.Bodies.IsLive(b)) { yield return w.GetBody(w.Core.HandleOfBody(b)); }
        }
    }

    [Fact]
    public void QueriesDoNotChangeTheSimulation()
    {
        var a = RandomWorld(31);
        var b = RandomWorld(31);
        var rng = new TestRng(3);
        for (int step = 0; step < 40; step++)
        {
            for (int i = 0; i < 50; i++)
            {
                a.Raycast(new Vector2D(rng.Range(-40, 40), rng.Range(-40, 40)), new Vector2F(1, 0.3f), out _);
                a.OverlapCircleAny(new Vector2D(rng.Range(-40, 40), rng.Range(-40, 40)), 2f);
            }

            a.Step(Dt); b.Step(Dt);
            Assert.Equal(b.ComputeHash(), a.ComputeHash());
        }
    }

    [Fact]
    public void SteadyStateQueriesWithCallerBuffersAllocateNothing()
    {
        var w = RandomWorld(41);
        var buffer = new QueryHit2D[8];
        var shape = Shape2D.CreateBox(0.5f, 0.3f);
        var f = QueryFilter2D.Layers(0b1011);
        void Round()
        {
            for (int i = 0; i < 100; i++)
            {
                var o = new Vector2D(-30 + i * 0.6, 12 - i * 0.2);
                var d = new Vector2F(1, 0.2f);
                w.Raycast(o, d, out _, 100f, f);
                w.RaycastAll(o, d, buffer, 100f, f);
                w.CircleCast(o, 0.3f, d, out _, 100f, f);
                w.ShapeCastAll(in shape, o, 0.4f, d, buffer, 100f, f);
                w.OverlapCircleAll(o, 3f, buffer, f);
                w.OverlapShapeAny(in shape, o, 0.2f, f);
                w.ClosestPoint(o, out _, 50f, f);
                w.ClosestPointAll(o, buffer, 20f, f);
            }
        }

        Round();
        Round();
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 5; i++) { Round(); }
        long after = GC.GetAllocatedBytesForCurrentThread();
        Assert.Equal(0, after - before);
        // The lazy per-step state is rebuilt after a step without allocating either.
        w.Step(Dt);
        Round();
        before = GC.GetAllocatedBytesForCurrentThread();
        Round();
        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);
    }

    [Fact]
    public void BatchQueriesEqualSingleQueriesAndDoNotDependOnThreads()
    {
        var results = new List<QueryHit2D[]>();
        var counts = new List<int[]>();
        foreach (int threads in new[] { 1, 4, 8 })
        {
            var w = RandomWorld(51, threads);
            var rng = new TestRng(8);
            var rays = new RayInput2D[500];
            var circles = new CircleInput2D[500];
            for (int i = 0; i < rays.Length; i++)
            {
                var o = new Vector2D(rng.Range(-40, 40), rng.Range(-40, 40));
                var d = new Vector2F(rng.Range(-1, 1), rng.Range(-1, 1) + 0.01f);
                rays[i] = new RayInput2D(o, d, rng.Range(5, 90));
                circles[i] = new CircleInput2D(o, rng.Range(0.1f, 2f), d, rng.Range(5, 40));
            }

            var q = new PhysicsQuery2D(w);
            var hits = new QueryHit2D[rays.Length];
            int n = q.RaycastBatch(rays, hits);
            Assert.True(n > 20);
            for (int i = 0; i < rays.Length; i++)
            {
                bool single = w.Raycast(rays[i].Origin, rays[i].Direction, out var h, rays[i].MaxDistance);
                Assert.Equal(single, hits[i].IsHit);
                if (single) { SameHit(in h, in hits[i]); }
            }

            var chits = new QueryHit2D[circles.Length];
            q.CircleCastBatch(circles, chits);
            var cnt = new int[circles.Length];
            q.OverlapCircleCountBatch(circles, cnt);
            for (int i = 0; i < circles.Length; i++) { Assert.Equal(w.OverlapCircleAll(circles[i].Center, circles[i].Radius).Length, cnt[i]); }
            results.Add(hits.Concat(chits).ToArray());
            counts.Add(cnt);
        }

        for (int t = 1; t < 3; t++)
        {
            SameList(results[0], results[t]);
            Assert.Equal(counts[0], counts[t]);
        }
    }

    [Fact]
    public void TheQueryObjectUsesItsFilter()
    {
        var w = Empty();
        var a = w.CreateBody(BodyType.Static, new Vector2D(3, 0));
        w.CreateCircle(a, 0.5f, default, new ShapeDef { Layer = 2 });
        var q = new PhysicsQuery2D(w) { Filter = QueryFilter2D.OnLayer(1) };
        Assert.False(q.Raycast(new Vector2D(0, 0), new Vector2F(1, 0), out _));
        q.Filter = QueryFilter2D.OnLayer(2);
        Assert.True(q.Raycast(new Vector2D(0, 0), new Vector2F(1, 0), out var h));
        Assert.Equal(a, h.Body);
        Assert.True(q.ClosestPoint(new Vector2D(0, 0), out _));
        var buffer = new QueryHit2D[4];
        Assert.Equal(1, q.OverlapShapeAll(Shape2D.CreateCircle(default, 4f), new Vector2D(0, 0), 0f, buffer));
    }
}
