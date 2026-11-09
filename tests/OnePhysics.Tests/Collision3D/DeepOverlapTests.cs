using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

/// <summary>A deep overlap of polyhedra must show its depth: the manifold's deepest point lies as deep as the separating axis test says the shapes overlap.</summary>
public class DeepOverlapTests
{
    // A Voronoi-cell-like shard that sank 0.4 m into a wide floor box: its most downward-facing face lay wholly above the floor's top plane, so the clipped face contact had only a grazing point
    // (separation 0.00015) and the manifold said the shapes were touching.
    [Fact]
    public void ShardSunkIntoAFloorReportsItsDepth()
    {
        Vector3F[] v = [new(-0.4f, -0.33541682f, -0.4f), new(-0.4f, 0.27962822f, 0.25309962f), new(-0.4f, 0.4f, -0.4f), new(-0.4f, 0.4f, 0.33835882f), new(-0.35175997f, 0.4f, 0.32535082f),
            new(-0.34493467f, 0.38787726f, 0.31492382f), new(-0.2980615f, -0.24280511f, -0.4f), new(-0.08743452f, 0.20214702f, -0.13071288f), new(-0.07405485f, 0.4f, -0.4f), new(-0.043347318f, 0.4f, -0.2511907f)];
        var hull = ConvexShape3D.FromHull(ConvexHull3D.Build(v));
        var floor = ConvexShape3D.Box(new Vector3F(40, 0.5f, 40));
        var bInA = new RigidTransformF(new QuaternionF(0.107940584f, -0.22759598f, 0.135829f, 0.958175f), new Vector3F(0.6130078f, 0.3898255f, 4.6793437f));
        foreach (bool swapped in new[] { false, true })
        {
            var gen = new ManifoldGenerator3D();
            var m = swapped ? gen.Collide(hull, floor, bInA.Inverted(), 0.02f) : gen.Collide(floor, hull, bInA, 0.02f);
            float deepest = 0f;
            for (int k = 0; k < m.PointCount; k++) { deepest = Math.Min(deepest, m[k].Separation); }
            Assert.True(deepest < -0.39f, $"swapped={swapped}: deepest point {deepest}, the overlap is 0.40");
        }
    }

    [Fact]
    public void RandomDeepOverlapsReportTheDepthOfTheirAxis()
    {
        var rng = new TestRng(0x5EED);
        int checkedPairs = 0;
        for (int trial = 0; trial < 4000; trial++)
        {
            var pts = new Vector3F[8 + rng.Next(8)];
            for (int i = 0; i < pts.Length; i++) { pts[i] = new Vector3F(rng.Range(-0.5f, 0.5f), rng.Range(-0.08f, 0.08f), rng.Range(-0.5f, 0.5f)); } // a flat shard
            ConvexHull3D hullB;
            try { hullB = ConvexHull3D.Build(pts); } catch (HullBuildException) { continue; }
            var a = trial % 2 == 0 ? ConvexShape3D.Box(new Vector3F(2, 0.5f, 2)) : ConvexShape3D.FromHull(ConvexHull3D.Build([new(-2f, -0.5f, -2f), new(2f, -0.5f, -2f), new(-2f, 0.5f, -2f), new(2f, 0.5f, -2f), new(-2f, -0.5f, 2f), new(2f, -0.5f, 2f), new(-2f, 0.5f, 2f), new(2f, 0.5f, 2f)]));
            var axis = rng.OnSphere();
            var q = QuaternionF.CreateFromAxisAngle(axis, rng.Range(0f, 6.28f));
            var t = new Vector3F(rng.Range(-1f, 1f), rng.Range(0.1f, 0.6f), rng.Range(-1f, 1f));
            var bInA = new RigidTransformF(q, t);
            var polyA = Brute.FromHull(a.Kind == ConvexShapeKind3D.Hull ? a.Hull! : ConvexHull3D.Build([new(-2f, -0.5f, -2f), new(2f, -0.5f, -2f), new(-2f, 0.5f, -2f), new(2f, 0.5f, -2f), new(-2f, -0.5f, 2f), new(2f, -0.5f, 2f), new(-2f, 0.5f, 2f), new(2f, 0.5f, 2f)]), RigidTransformF.Identity);
            var polyB = Brute.FromHull(hullB, bInA);
            var (sep, _, _) = Brute.Sat(polyA, polyB);
            // Deep overlaps. The manifold may pick an axis a little deeper than the best one (a coherence bias towards its reference face), so its deepest point is compared with a tolerance.
            if (!(sep < -0.06)) { continue; }
            var m = new ManifoldGenerator3D().Collide(a, ConvexShape3D.FromHull(hullB), bInA, 0.02f);
            Assert.True(m.PointCount > 0, $"trial {trial}: no points for an overlap of {sep}");
            float deepest = 0f;
            for (int k = 0; k < m.PointCount; k++) { deepest = Math.Min(deepest, m[k].Separation); }
            Assert.True(Math.Abs(deepest - sep) <= 0.01 + 0.03 * Math.Abs(sep), $"trial {trial}: deepest point {deepest}, separating axis test says {sep}");
            checkedPairs++;
        }

        Assert.True(checkedPairs > 200, $"only {checkedPairs} pairs checked");
    }
}
