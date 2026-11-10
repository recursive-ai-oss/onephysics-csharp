using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;
using Rod3Def = RecursiveAI.OnePhysics.Physics3D.RodDef;
using Rod2Def = RecursiveAI.OnePhysics.Physics2D.RodDef;
using SoftMesh3 = RecursiveAI.OnePhysics.Physics3D.SoftBodyMesh;
using SoftDef3 = RecursiveAI.OnePhysics.Physics3D.SoftBodyDef;

namespace RecursiveAI.OnePhysics.Tests.Water;

/// <summary>Ray, shape cast, overlap and closest-point queries against soft bodies, cloth and rods, with known answers and a brute-force check.</summary>
public class DeformableQueryTests
{
    private static PhysicsWorld3D World3(int id = 361, int threads = 1) => new(new WorldOptions3D { ThreadCount = threads, WorldId = id });

    private static Rod3Def Line3(Vector3D from, Vector3D to, int segments, float radius)
    {
        var pts = new Vector3D[segments + 1];
        for (int i = 0; i <= segments; i++) { pts[i] = from + (to - from) * ((double)i / segments); }
        return new Rod3Def { Points = pts, Material = new RecursiveAI.OnePhysics.Rods.RodMaterial(1e7f, 4e6f, radius, 400f) };
    }

    private static Vector3D Blend(Physics3D.SoftBody body, in DeformableHit3D h)
    {
        var a = body.GetPosition(h.ParticleA); var b = body.GetPosition(h.ParticleB); var c = body.GetPosition(h.ParticleC);
        return a * h.Barycentric.X + b * h.Barycentric.Y + c * h.Barycentric.Z;
    }

    [Fact]
    public void ARayAtARodHitsTheSegmentWithItsWeights3D()
    {
        var w = World3();
        var rod = w.AddRod(Line3(new Vector3D(0, 1, 0), new Vector3D(2, 1, 0), 4, 0.02f));
        Assert.True(w.RaycastDeformables(new Vector3D(0.75, 3, 0), new Vector3F(0, -1, 0), out var hit));
        Assert.Equal(DeformableKind.Rod, hit.Kind);
        Assert.Equal(rod.Index, hit.Slot);
        Assert.Equal(DeformableElement.Segment, hit.Element);
        Assert.Equal(1, hit.Index);
        Assert.Equal(1, hit.ParticleA); Assert.Equal(2, hit.ParticleB);
        Assert.InRange(hit.Distance, 1.979f, 1.981f);
        Assert.InRange(hit.Point.Y, 1.0199, 1.0201);
        Assert.InRange(hit.Normal.Y, 0.999f, 1.001f);
        Assert.InRange(hit.Barycentric.X, 0.49f, 0.51f); Assert.InRange(hit.Barycentric.Y, 0.49f, 0.51f);
        // A ray beside it misses; one that starts inside hits at 0.
        Assert.False(w.RaycastDeformables(new Vector3D(0.75, 3, 0.5), new Vector3F(0, -1, 0), out _));
        Assert.True(w.RaycastDeformables(new Vector3D(0.75, 1, 0), new Vector3F(0, -1, 0), out var inside));
        Assert.Equal(0f, inside.Distance);
    }

    [Fact]
    public void ARayAtClothHitsATriangleAndItsWeightsGiveThePoint()
    {
        var w = World3();
        var cloth = w.AddCloth(2f, 2f, 8, new Vector3D(0, 2, 0));
        Assert.True(w.RaycastDeformables(new Vector3D(0.13, 4, -0.21), new Vector3F(0, -1, 0), out var hit));
        Assert.Equal(DeformableKind.Cloth, hit.Kind);
        Assert.Equal(DeformableElement.Triangle, hit.Element);
        Assert.InRange(hit.Distance, 1.999f, 2.001f);
        Assert.InRange(hit.Point.X, 0.129, 0.131); Assert.InRange(hit.Point.Z, -0.211, -0.209);
        var rebuilt = Blend(cloth, in hit);
        Assert.InRange((rebuilt - hit.Point).Length(), 0.0, 1e-5);
        Assert.InRange(hit.Barycentric.X + hit.Barycentric.Y + hit.Barycentric.Z, 0.9999f, 1.0001f);
        Assert.InRange(hit.Barycentric.X, -1e-4f, 1.0001f); Assert.InRange(hit.Barycentric.Y, -1e-4f, 1.0001f); Assert.InRange(hit.Barycentric.Z, -1e-4f, 1.0001f);
    }

    [Fact]
    public void ARayAtASoftBallEntersItsSurface()
    {
        var w = World3();
        var ball = w.AddSoftBall(0.5f, new Vector3D(0, 1, 0), segments: 5);
        Assert.True(w.RaycastDeformables(new Vector3D(0.05, 3, 0.02), new Vector3F(0, -1, 0), out var hit));
        Assert.Equal(DeformableKind.SoftBody, hit.Kind);
        Assert.InRange((Blend(ball, in hit) - hit.Point).Length(), 0.0, 1e-5);
        Assert.InRange(hit.Point.Y, 1.3, 1.52);
        Assert.True(hit.Normal.Y > 0.8f);
        // All hits from above: the top and the bottom of the ball, nearest first.
        var all = new DeformableHit3D[8];
        int n = w.RaycastDeformablesAll(new Vector3D(0.05, 3, 0.02), new Vector3F(0, -1, 0), all);
        Assert.True(n >= 2);
        for (int i = 1; i < n; i++) { Assert.True(all[i].Distance >= all[i - 1].Distance); }
        Assert.Equal(hit.Index, all[0].Index);
        Assert.True(w.RaycastDeformablesAny(new Vector3D(0.05, 3, 0.02), new Vector3F(0, -1, 0)));
    }

    [Fact]
    public void ClosestPointAndSphereCastAgainstClothHaveTheKnownDistances()
    {
        var w = World3();
        var cloth = w.AddCloth(2f, 2f, 8, new Vector3D(0, 2, 0));
        Assert.True(w.ClosestDeformable(new Vector3D(0.3, 2.5, 0.1), out var c));
        Assert.InRange(c.Distance, 0.499f, 0.501f);
        Assert.InRange(c.Point.X, 0.299, 0.301); Assert.InRange(c.Point.Y, 1.999, 2.001); Assert.InRange(c.Point.Z, 0.099, 0.101);
        Assert.True(c.Normal.Y > 0.99f);
        Assert.False(w.ClosestDeformable(new Vector3D(0.3, 5, 0.1), out _, maxDistance: 1f));
        Assert.True(w.SphereCastDeformables(new Vector3D(0.3, 4, 0.1), 0.1f, new Vector3F(0, -1, 0), out var s));
        Assert.InRange(s.Distance, 1.899f, 1.901f);
        Assert.InRange(s.Fraction, 0f, 1f);
        Assert.InRange(s.Point.Y, 1.999, 2.001);
        Assert.False(w.SphereCastDeformables(new Vector3D(5, 4, 0.1), 0.1f, new Vector3F(0, -1, 0), out _, 10f));
        _ = cloth;
    }

    private static double PointTriangle(Vector3D p, Vector3D a, Vector3D b, Vector3D c)
    {
        // Ericson 5.1.5, closest point on a triangle.
        var ab = b - a; var ac = c - a; var ap = p - a;
        double d1 = Vector3D.Dot(ab, ap), d2 = Vector3D.Dot(ac, ap);
        if (d1 <= 0 && d2 <= 0) { return (p - a).Length(); }
        var bp = p - b; double d3 = Vector3D.Dot(ab, bp), d4 = Vector3D.Dot(ac, bp);
        if (d3 >= 0 && d4 <= d3) { return (p - b).Length(); }
        double vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0) { return (p - (a + ab * (d1 / (d1 - d3)))).Length(); }
        var cp = p - c; double d5 = Vector3D.Dot(ab, cp), d6 = Vector3D.Dot(ac, cp);
        if (d6 >= 0 && d5 <= d6) { return (p - c).Length(); }
        double vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0) { return (p - (a + ac * (d2 / (d2 - d6)))).Length(); }
        double va = d3 * d6 - d5 * d4;
        if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) { return (p - (b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6))))).Length(); }
        double denom = 1.0 / (va + vb + vc);
        return (p - (a + ab * (vb * denom) + ac * (vc * denom))).Length();
    }

    [Fact]
    public void OverlapsAgreeWithABruteForceOverEveryTriangle()
    {
        var w = World3();
        var ball = w.AddSoftBall(0.6f, new Vector3D(0.2, 1, -0.1), segments: 5);
        var tris = ball.Mesh.Triangles;
        var rng = new Random(7);
        var buffer = new DeformableHit3D[2048];
        int checkedSpheres = 0;
        for (int k = 0; k < 40; k++)
        {
            var c = new Vector3D(rng.NextDouble() * 1.6 - 0.8, 1 + rng.NextDouble() * 1.6 - 0.8, rng.NextDouble() * 1.6 - 0.8);
            float r = 0.05f + (float)rng.NextDouble() * 0.3f;
            int expected = 0;
            for (int t = 0; t < tris.Length; t += 3)
            {
                if (PointTriangle(c, ball.GetPosition(tris[t]), ball.GetPosition(tris[t + 1]), ball.GetPosition(tris[t + 2])) <= r) { expected++; }
            }

            int n = w.OverlapSphereDeformablesAll(c, r, buffer);
            // The engine's answer may differ from a float-free brute force only for triangles within a hair of the sphere.
            int near = 0;
            for (int t = 0; t < tris.Length; t += 3)
            {
                double d = PointTriangle(c, ball.GetPosition(tris[t]), ball.GetPosition(tris[t + 1]), ball.GetPosition(tris[t + 2]));
                if (Math.Abs(d - r) < 1e-4) { near++; }
            }

            Assert.InRange(n, expected - near, expected + near);
            if (expected > 0) { checkedSpheres++; }
        }

        Assert.True(checkedSpheres > 5);
    }

    [Fact]
    public void FiltersChooseKindsAndLayers()
    {
        var w = World3();
        w.AddRod(Line3(new Vector3D(-2, 1, 0), new Vector3D(2, 1, 0), 8, 0.02f));
        w.AddCloth(2f, 2f, 4, new Vector3D(0, 2, 0));
        var down = new Vector3F(0, -1, 0);
        Assert.True(w.RaycastDeformables(new Vector3D(0.1, 5, 0.013), down, out var any));
        Assert.Equal(DeformableKind.Cloth, any.Kind);
        Assert.True(w.RaycastDeformables(new Vector3D(0.1, 5, 0.013), down, out var rod, filter: DeformableFilter3D.Of(DeformableKinds.Rods)));
        Assert.Equal(DeformableKind.Rod, rod.Kind);
        Assert.False(w.RaycastDeformables(new Vector3D(0.1, 5, 0.013), down, out _, filter: DeformableFilter3D.Of(DeformableKinds.SoftBodies)));
        Assert.False(w.RaycastDeformables(new Vector3D(0.1, 5, 0.013), down, out _, filter: new DeformableFilter3D { LayerMask = 1u << 5 }));
        var all = new DeformableHit3D[4];
        int n = w.RaycastDeformablesAll(new Vector3D(0.1, 5, 0.013), down, all);
        Assert.Equal(2, n);
        Assert.Equal(DeformableKind.Cloth, all[0].Kind); Assert.Equal(DeformableKind.Rod, all[1].Kind);
    }

    [Fact]
    public void BatchesEqualTheCallLoopAtAnyThreadCountAndAllocateNothingWarm()
    {
        var rays = new RayInput3D[300];
        var rng = new Random(3);
        for (int i = 0; i < rays.Length; i++) { rays[i] = new RayInput3D(new Vector3D(rng.NextDouble() * 3 - 1.5, 4, rng.NextDouble() * 3 - 1.5), new Vector3F(0.05f * (float)rng.NextDouble(), -1, 0f)); }
        DeformableHit3D[][] results = new DeformableHit3D[2][];
        int index = 0;
        foreach (int threads in new[] { 1, 4 })
        {
            var w = World3(362, threads);
            w.AddCloth(3f, 3f, 12, new Vector3D(0, 2, 0));
            w.AddSoftBall(0.5f, new Vector3D(0, 3, 0), segments: 4);
            w.AddRod(Line3(new Vector3D(-2, 1, 0), new Vector3D(2, 1, 0), 16, 0.03f));
            var q = new PhysicsQuery3D(w);
            var hits = new DeformableHit3D[rays.Length];
            int count = q.RaycastDeformablesBatch(rays, hits);
            Assert.True(count > 100);
            for (int i = 0; i < rays.Length; i++)
            {
                bool single = w.RaycastDeformables(rays[i].Origin, rays[i].Direction, out var h);
                Assert.Equal(single, hits[i].IsHit);
                if (single) { Assert.Equal(h.Point.X, hits[i].Point.X); Assert.Equal(h.Point.Y, hits[i].Point.Y); Assert.Equal(h.Distance, hits[i].Distance); Assert.Equal(h.Index, hits[i].Index); Assert.Equal(h.Slot, hits[i].Slot); }
            }

            results[index++] = hits;
            // Warm: no allocation per call on this thread (the smallest of a few windows after a collection, as in the other allocation tests).
            var buf = new DeformableHit3D[16];
            var sphere = ConvexShape3D.Sphere(0.2f);
            void Round()
            {
                for (int k = 0; k < 50; k++)
                {
                    w.RaycastDeformables(rays[k].Origin, rays[k].Direction, out _);
                    w.RaycastDeformablesAll(rays[k].Origin, rays[k].Direction, buf);
                    w.ClosestDeformable(new Vector3D(0, 2.3, 0), out _);
                    w.OverlapDeformablesAll(sphere, new Vector3D(0, 2.1, 0), default, buf);
                    w.ShapeCastDeformables(sphere, new Vector3D(0.3, 4, 0), default, new Vector3F(0, -1, 0), out _);
                }
            }

            Round(); Round();
            long smallest = long.MaxValue;
            for (int attempt = 0; attempt < 4 && smallest != 0; attempt++)
            {
                Core.AllocMeter.Settle();
                smallest = Math.Min(smallest, Core.AllocMeter.Of(Round));
            }

            Assert.Equal(0L, smallest);
        }

        for (int i = 0; i < rays.Length; i++)
        {
            Assert.Equal(results[0][i].IsHit, results[1][i].IsHit);
            Assert.Equal(results[0][i].Point.X, results[1][i].Point.X);
            Assert.Equal(results[0][i].Distance, results[1][i].Distance);
        }
    }

    [Fact]
    public void QueriesNeverChangeTheHash()
    {
        var w = World3();
        w.AddCloth(2f, 2f, 6, new Vector3D(0, 2, 0));
        w.Step(1f / 60f);
        ulong before = w.ComputeHash();
        var buf = new DeformableHit3D[8];
        w.RaycastDeformablesAll(new Vector3D(0, 5, 0), new Vector3F(0, -1, 0), buf);
        w.OverlapSphereDeformablesAll(new Vector3D(0, 2, 0), 0.5f, buf);
        Assert.Equal(before, w.ComputeHash());
    }

    [Fact]
    public void BadInputsAreRejectedWithNamedErrors()
    {
        var w = World3();
        w.AddCloth(2f, 2f, 4, new Vector3D(0, 2, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.RaycastDeformables(new Vector3D(double.NaN, 0, 0), new Vector3F(0, -1, 0), out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.RaycastDeformables(new Vector3D(0, 5, 0), default, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.ClosestDeformable(new Vector3D(0, 5, 0), out _, -1f));
        Assert.Throws<ArgumentException>(() => w.OverlapDeformablesAll(ConvexShape3D.Plane(new Vector3F(0, 1, 0)), default, default, new DeformableHit3D[2]));
        var q = new PhysicsQuery3D(w);
        Assert.Throws<ArgumentOutOfRangeException>(() => q.RaycastDeformablesBatch(new[] { new RayInput3D(new Vector3D(0, 5, 0), default) }, new DeformableHit3D[1]));
        Assert.Throws<ArgumentException>(() => q.RaycastDeformablesBatch(new[] { new RayInput3D(new Vector3D(0, 5, 0), new Vector3F(0, -1, 0)) }, Array.Empty<DeformableHit3D>()));
    }

    // ------------------------------------------------------------------ 2D

    private static PhysicsWorld2D World2(int id = 363) => new(new WorldOptions2D { ThreadCount = 1, WorldId = id });

    [Fact]
    public void RodsAndSoftBodiesAreHitAndMeasuredIn2D()
    {
        var w = World2();
        var pts = new Vector2D[5];
        for (int i = 0; i < 5; i++) { pts[i] = new Vector2D(i * 0.5, 1); }
        var rod = w.AddRod(new Rod2Def { Points = pts, Material = new RecursiveAI.OnePhysics.Rods.RodMaterial(1e7f, 4e6f, 0.02f, 400f) });
        Assert.True(w.RaycastDeformables(new Vector2D(0.75, 3), new Vector2F(0, -1), out var hit));
        Assert.Equal(DeformableKind.Rod, hit.Kind); Assert.Equal(rod.Index, hit.Slot); Assert.Equal(1, hit.Index);
        Assert.InRange(hit.Distance, 1.979f, 1.981f); Assert.InRange(hit.Normal.Y, 0.99f, 1.01f);
        Assert.InRange(hit.Barycentric.X, 0.49f, 0.51f);
        Assert.True(w.ClosestDeformable(new Vector2D(1.0, 1.6), out var c));
        Assert.InRange(c.Distance, 0.579f, 0.581f);
        Assert.True(w.CircleCastDeformables(new Vector2D(1.2, 4), 0.1f, new Vector2F(0, -1), out var cast));
        Assert.InRange(cast.Distance, 2.879f, 2.881f);
        var buf = new DeformableHit2D[16];
        // A circle of radius 0.3 over the rod at x = 1 touches the two segments that meet there.
        Assert.Equal(2, w.OverlapCircleDeformablesAll(new Vector2D(1.0, 1.2), 0.3f, buf));
        Assert.Equal(0, w.OverlapCircleDeformablesAll(new Vector2D(1.0, 1.6), 0.3f, buf));
        Assert.False(w.RaycastDeformables(new Vector2D(0.75, 3), new Vector2F(0, -1), out _, filter: DeformableFilter2D.Of(DeformableKinds.SoftBodies)));
    }
}

/// <summary>Contact begin and end events of soft bodies, cloth and rods against rigid bodies.</summary>
public class DeformableContactEventTests
{
    private const float Dt = 1f / 60f;

    [Fact]
    public void ASoftBallDroppedOnAFloorBeginsAndLaterEndsItsContactAndEventsAreOptIn()
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = 1, WorldId = 371 });
        var floor = w.AddBox(new Vector3F(20f, 1f, 20f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var ball = w.AddSoftBall(0.4f, new Vector3D(0, 1.0, 0), segments: 4);
        var seen = new List<DeformableContactEvent>();
        w.DeformableContactChanged += seen.Add;
        // Off by default: the ball lands and nothing is reported.
        for (int i = 0; i < 90; i++) { w.Step(Dt); }
        Assert.Empty(seen);
        Assert.Equal(0, w.DeformableContactEvents.Length);
        w.TrackDeformableContacts = true;
        var polled = new List<DeformableContactEvent>();
        ball.AddVelocity(new Vector3F(0f, 8f, 0f));
        for (int i = 0; i < 400; i++) { w.Step(Dt); polled.AddRange(w.DeformableContactEvents.ToArray()); }
        // The ball leaves the floor, comes back and begins touching it again; the events alternate for the one pair and both delivery forms agree.
        Assert.Equal(seen.Count, polled.Count);
        Assert.Contains(seen, e => e.Kind == DeformableContactEventKind.Ended && e.Body == floor.RawHandle && e.Slot == ball.Slot && e.Deformable == DeformableKind.SoftBody);
        Assert.Contains(seen, e => e.Kind == DeformableContactEventKind.Began && e.Body == floor.RawHandle);
        for (int i = 1; i < seen.Count; i++) { Assert.NotEqual(seen[i - 1].Kind, seen[i].Kind); }
    }

    [Fact]
    public void ARopeLandingOnABoxBeginsAContactWithIt2D()
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = 1, WorldId = 372 });
        var box = w.AddBox(new Vector2F(4f, 1f), new Vector2D(0, 0), type: BodyType.Static);
        var pts = new Vector2D[9];
        for (int i = 0; i < pts.Length; i++) { pts[i] = new Vector2D(-1 + 0.25 * i, 1.5); }
        var rod = w.AddRod(new RecursiveAI.OnePhysics.Physics2D.RodDef { Points = pts, Material = new RecursiveAI.OnePhysics.Rods.RodMaterial(1e7f, 4e6f, 0.02f, 800f) });
        w.TrackDeformableContacts = true;
        var seen = new List<DeformableContactEvent>();
        for (int i = 0; i < 240; i++) { w.Step(Dt); seen.AddRange(w.DeformableContactEvents.ToArray()); }
        Assert.Single(seen);
        Assert.Equal(DeformableContactEventKind.Began, seen[0].Kind);
        Assert.Equal(DeformableKind.Rod, seen[0].Deformable);
        Assert.Equal(rod.Index, seen[0].Slot);
        Assert.Equal(box.RawHandle, seen[0].Body);
    }
}
