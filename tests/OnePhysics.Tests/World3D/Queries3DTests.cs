using System;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Tests.Collision3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

public class Queries3DTests
{
    private const float Dt = 1f / 60f;

    // Convex shapes of every kind, a compound, a triangle mesh and a height field scattered in a 60 m box.
    private static PhysicsWorld3D MixedWorld(int seed = 3)
    {
        var w = World3DTests.NewWorld(1, 13, o => o.Gravity = default);
        var rng = new TestRng((ulong)seed);
        for (int i = 0; i < 60; i++)
        {
            var at = new Vector3D(rng.Range(-30, 30), rng.Range(0, 20), rng.Range(-30, 30));
            var rot = rng.Rotation();
            var type = i % 3 == 0 ? BodyType.Static : BodyType.Dynamic;
            switch (i % 6)
            {
                case 0: w.AddBox(new Vector3F(rng.Range(0.5f, 3), rng.Range(0.5f, 3), rng.Range(0.5f, 3)), at, rot, type); break;
                case 1: w.AddSphere(rng.Range(0.3f, 1.5f), at, type); break;
                case 2: w.AddCapsule(rng.Range(0.3f, 1.5f), rng.Range(0.2f, 0.8f), at, rot, type); break;
                case 3: w.AddCylinder(rng.Range(0.3f, 1.5f), rng.Range(0.3f, 1f), at, rot, type); break;
                case 4:
                {
                    var pts = new Vector3F[12];
                    for (int k = 0; k < pts.Length; k++) { pts[k] = rng.InBall(1.2f); }
                    w.AddConvexHull(pts, at, rot, type);
                    break;
                }
                default:
                    w.AddCompound([new CompoundPart(ShapeSpec3D.Box(new Vector3F(1.6f, 0.6f, 0.6f)), default), new CompoundPart(ShapeSpec3D.Sphere(0.5f), new Vector3F(1f, 0.5f, 0f))], at, rot, type);
                    break;
            }
        }

        // A bumpy mesh patch and a height field under everything.
        var verts = new Vector3F[11 * 11];
        for (int j = 0; j < 11; j++) { for (int i = 0; i < 11; i++) { verts[j * 11 + i] = new Vector3F(i * 2f, 0.4f * ((i * 7 + j * 3) % 5), j * 2f); } }
        var idx = new System.Collections.Generic.List<int>();
        for (int j = 0; j < 10; j++)
        {
            for (int i = 0; i < 10; i++)
            {
                int a = j * 11 + i;
                idx.AddRange([a, a + 11, a + 1, a + 1, a + 11, a + 12]);
            }
        }

        w.AddMesh(verts, idx.ToArray(), new Vector3D(-10, 6, -10));
        const int N = 17;
        var h = new float[N * N];
        for (int j = 0; j < N; j++) { for (int i = 0; i < N; i++) { h[j * N + i] = 0.3f * ((i * 5 + j * 2) % 4); } }
        w.AddHeightField(h, N, N, new Vector3F(2f, 1f, 2f), new Vector3D(-16, -2, -16));
        w.Step(Dt);
        return w;
    }

    private static void SameHit(QueryHit3D a, QueryHit3D b, string what)
    {
        Assert.True(a.Shape.Equals(b.Shape), what + " shape");
        Assert.Equal(a.Fraction, b.Fraction);
        Assert.Equal(a.Distance, b.Distance);
        Assert.Equal(a.Point, b.Point);
        Assert.Equal(a.Normal, b.Normal);
        Assert.Equal(a.SubShapeId, b.SubShapeId);
    }

    private static void SameAll(QueryHit3D[] a, QueryHit3D[] b, string what)
    {
        Assert.Equal(a.Length, b.Length);
        for (int i = 0; i < a.Length; i++) { SameHit(a[i], b[i], what); }
    }

    private const int Count = 100_000;

    [Fact]
    public void RaysMatchBruteForce()
    {
        var w = MixedWorld();
        var rng = new TestRng(1);
        int hits = 0;
        for (int i = 0; i < Count; i++)
        {
            var o = new Vector3D(rng.Range(-40, 40), rng.Range(-5, 25), rng.Range(-40, 40));
            var d = rng.OnSphere();
            float max = i % 4 == 0 ? float.PositiveInfinity : rng.Range(1, 40);
            w.Brute = false;
            bool h1 = w.Raycast(o, d, out var a, max);
            var all1 = i % 50 == 0 ? w.RaycastAll(o, d, max) : [];
            w.Brute = true;
            bool h2 = w.Raycast(o, d, out var b, max);
            Assert.Equal(h2, h1);
            if (h1) { hits++; SameHit(a, b, "ray"); }
            if (i % 50 == 0) { SameAll(all1, w.RaycastAll(o, d, max), "rayall"); }
        }

        w.Brute = false;
        Assert.True(hits > Count / 50, "too few rays hit anything: " + hits);
    }

    [Fact]
    public void ShapeCastsMatchBruteForce()
    {
        var w = MixedWorld();
        var rng = new TestRng(2);
        int hits = 0;
        for (int i = 0; i < Count; i++)
        {
            var o = new Vector3D(rng.Range(-40, 40), rng.Range(-5, 25), rng.Range(-40, 40));
            var d = rng.OnSphere();
            float max = i % 5 == 0 ? float.PositiveInfinity : rng.Range(1, 40);
            var rot = rng.Rotation();
            float p0 = rng.Range(0.2f, 2), p1 = rng.Range(0.2f, 2), p2 = rng.Range(0.2f, 2);
            int kind = i % 3;
            bool Cast(bool brute, out QueryHit3D hit)
            {
                w.Brute = brute;
                return kind switch
                {
                    0 => w.SphereCast(o, p0 * 0.5f, d, out hit, max),
                    1 => w.BoxCast(o, new Vector3F(p0, p1, p2), rot, d, out hit, max),
                    _ => w.CapsuleCast(o, p0, p1 * 0.3f, rot, d, out hit, max),
                };
            }

            bool h1 = Cast(false, out var a);
            bool h2 = Cast(true, out var b);
            Assert.Equal(h2, h1);
            if (h1) { hits++; SameHit(a, b, "cast"); }
        }

        w.Brute = false;
        Assert.True(hits > Count / 10, "too few casts hit anything: " + hits);
    }

    [Fact]
    public void OverlapsAndClosestPointMatchBruteForce()
    {
        var w = MixedWorld();
        var rng = new TestRng(3);
        int hits = 0;
        var buf1 = new QueryHit3D[64];
        var buf2 = new QueryHit3D[64];
        for (int i = 0; i < Count; i++)
        {
            var o = new Vector3D(rng.Range(-40, 40), rng.Range(-5, 25), rng.Range(-40, 40));
            var rot = rng.Rotation();
            float p0 = rng.Range(0.2f, 4), p1 = rng.Range(0.2f, 4), p2 = rng.Range(0.2f, 4);
            int kind = i % 4;
            (bool, int) Run(bool brute, QueryHit3D[] buf, out QueryHit3D hit)
            {
                w.Brute = brute;
                switch (kind)
                {
                    case 0:
                    {
                        bool r = w.OverlapSphere(o, p0, out hit);
                        return (r, w.OverlapSphereAll(o, p0, buf));
                    }
                    case 1:
                    {
                        bool r = w.OverlapBox(o, new Vector3F(p0, p1, p2), rot, out hit);
                        return (r, w.OverlapBoxAll(o, new Vector3F(p0, p1, p2), rot, buf));
                    }
                    case 2:
                    {
                        bool r = w.OverlapCapsule(o, p0, p1 * 0.3f, rot, out hit);
                        return (r, w.OverlapCapsuleAll(o, p0, p1 * 0.3f, rot, buf));
                    }
                    default:
                    {
                        bool r = w.ClosestPoint(o, out hit, 6f);
                        return (r, w.ClosestPointAll(o, buf, 6f));
                    }
                }
            }

            var (r1, n1) = Run(false, buf1, out var a);
            var (r2, n2) = Run(true, buf2, out var b);
            Assert.Equal(r2, r1);
            Assert.Equal(n2, n1);
            if (r1) { hits++; SameHit(a, b, "overlap"); }
            for (int k = 0; k < n1; k++) { SameHit(buf1[k], buf2[k], "overlap all"); }
        }

        w.Brute = false;
        Assert.True(hits > Count / 10, "too few queries hit anything: " + hits);
    }

    // The box rejections and the closed-form sphere overlaps skip the exact test only when its answer is clear; with them off every candidate goes to the exact test, and nothing may differ.
    [Fact]
    public void QuickRejectsChangeNoResult()
    {
        var w = MixedWorld();
        var rng = new TestRng(5);
        int hits = 0, touching = 0;
        var buf1 = new QueryHit3D[64];
        var buf2 = new QueryHit3D[64];
        for (int i = 0; i < 40_000; i++)
        {
            var o = new Vector3D(rng.Range(-40, 40), rng.Range(-5, 25), rng.Range(-40, 40));
            var d = rng.OnSphere();
            var rot = rng.Rotation();
            float p0 = rng.Range(0.2f, 4), p1 = rng.Range(0.2f, 3), p2 = rng.Range(0.2f, 3);
            float max = rng.Range(1, 40);
            // Sphere radii that graze the nearest shape: its distance plus or minus a few thousandths.
            float radius = p0;
            if (i % 3 == 0 && w.ClosestPoint(o, out var near, 12f))
            {
                float[] deltas = [-2e-3f, -1e-3f, -1e-4f, -1e-5f, 0f, 1e-5f, 1e-4f, 1e-3f, 2e-3f];
                radius = Math.Max(0f, near.Distance + deltas[(i / 3) % deltas.Length]);
                touching++;
            }

            (int, bool, QueryHit3D, bool, QueryHit3D, bool, QueryHit3D, int) Run(bool quick)
            {
                w.QuickRejects = quick;
                var buf = quick ? buf1 : buf2;
                bool ray = w.Raycast(o, d, out var hr, max);
                bool sph = w.OverlapSphere(o, radius, out var hs);
                int n = w.OverlapSphereAll(o, radius, buf);
                bool cast = i % 2 == 0 ? w.SphereCast(o, p1 * 0.4f, d, out var hc, max) : w.BoxCast(o, new Vector3F(p0, p1, p2), rot, d, out hc, max);
                return (n, ray, hr, sph, hs, cast, hc, n);
            }

            var a = Run(true);
            var b = Run(false);
            w.QuickRejects = true;
            Assert.Equal(b.Item2, a.Item2);
            if (a.Item2) { hits++; SameHit(a.Item3, b.Item3, "ray"); }
            Assert.Equal(b.Item4, a.Item4);
            if (a.Item4) { SameHit(a.Item5, b.Item5, "overlap"); }
            Assert.Equal(b.Item1, a.Item1);
            for (int k = 0; k < a.Item1; k++) { SameHit(buf1[k], buf2[k], "overlap all"); }
            Assert.Equal(b.Item6, a.Item6);
            if (a.Item6) { SameHit(a.Item7, b.Item7, "cast"); }
        }

        Assert.True(hits > 300 && touching > 1000, hits + " rays hit, " + touching + " grazing queries");
    }

    [Fact]
    public void KnownAnswers()
    {
        var w = World3DTests.NewWorld(1, 5, o => o.Gravity = default);
        var box = w.AddBox(new Vector3F(2, 2, 2), new Vector3D(0, 0, 0), type: BodyType.Static);
        var ball = w.AddSphere(1f, new Vector3D(10, 0, 0), type: BodyType.Static);
        w.Step(Dt);
        Assert.True(w.Raycast(new Vector3D(-5, 0.3, 0.2), new Vector3F(1, 0, 0), out var h));
        Assert.Equal(4f, h.Distance, 3);
        Assert.Equal(-1.0, h.Point.X, 3);
        Assert.Equal(-1f, h.Normal.X, 3);
        Assert.True(h.Body.Equals(box));
        // Sphere radius 0.5 swept along +x from x=-5 stops with its surface on the box face: centre at x=-1.5.
        Assert.True(w.SphereCast(new Vector3D(-5, 0, 0), 0.5f, new Vector3F(1, 0, 0), out h));
        Assert.Equal(3.5f, h.Distance, 3);
        Assert.Equal(-1.0, h.Point.X, 3);
        Assert.Equal(-1f, h.Normal.X, 3);
        // A sweep from the box to the ball hits the ball first when started between them.
        Assert.True(w.SphereCast(new Vector3D(5, 0, 0), 0.5f, new Vector3F(1, 0, 0), out h));
        Assert.True(h.Body.Equals(ball));
        Assert.Equal(3.5f, h.Distance, 3);
        Assert.Equal(9.0, h.Point.X, 3);
        Assert.True(w.OverlapSphereAny(new Vector3D(1.4, 0, 0), 0.5f));
        Assert.False(w.OverlapSphereAny(new Vector3D(1.6, 0, 0), 0.5f));
        Assert.True(w.OverlapPoint(new Vector3D(0.5, 0.5, 0.5), out h));
        Assert.True(h.Body.Equals(box));
        Assert.True(w.ClosestPoint(new Vector3D(4, 0.2, 0.3), out h));
        Assert.Equal(1.0, h.Point.X, 3);
        Assert.Equal(0.2, h.Point.Y, 3);
        Assert.Equal(3f, h.Distance, 3);
        Assert.False(w.Raycast(new Vector3D(-5, 5, 0), new Vector3F(1, 0, 0), out _));
        Assert.False(w.Raycast(new Vector3D(-5, 0, 0), new Vector3F(1, 0, 0), out _, 2f));
    }

    [Fact]
    public void QueriesDoNotAllocateInSteadyState()
    {
        var w = MixedWorld();
        var buf = new QueryHit3D[32];
        var names = new[] { "ray", "rayAll", "sphereCast", "overlapSphereAll", "overlapBoxAny", "closest" };
        var bytes = new long[names.Length];
        void Round(bool measure)
        {
            var rng = new TestRng(9);
            for (int i = 0; i < 200; i++)
            {
                var o = new Vector3D(rng.Range(-30, 30), rng.Range(0, 20), rng.Range(-30, 30));
                var d = rng.OnSphere();
                for (int k = 0; k < names.Length; k++)
                {
                    long before = GC.GetAllocatedBytesForCurrentThread();
                    switch (k)
                    {
                        case 0: w.Raycast(o, d, out _, 30f); break;
                        case 1: w.RaycastAll(o, d, buf, 30f); break;
                        case 2: w.SphereCast(o, 0.5f, d, out _, 20f); break;
                        case 3: w.OverlapSphereAll(o, 2f, buf); break;
                        case 4: w.OverlapBoxAny(o, new Vector3F(1, 1, 1), QuaternionF.Identity); break;
                        default: w.ClosestPoint(o, out _, 10f); break;
                    }

                    if (measure) { bytes[k] += GC.GetAllocatedBytesForCurrentThread() - before; }
                }
            }
        }

        Round(false);
        Round(false);
        Round(true);
        string report = string.Join(", ", names.Select((n, k) => n + "=" + bytes[k]));
        Assert.True(bytes.All(b => b == 0), "allocated bytes per query kind: " + report);
    }
}
