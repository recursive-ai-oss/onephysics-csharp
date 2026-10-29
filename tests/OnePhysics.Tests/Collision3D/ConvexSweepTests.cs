using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

/// <summary>The closed-form ray and sphere-sweep leaf tests against the general GJK route on random shapes and segments.</summary>
public class ConvexSweepTests
{
    private static List<ConvexShape3D> Shapes(TestRng rng)
    {
        var list = new List<ConvexShape3D>
        {
            ConvexShape3D.Sphere(0.4f),
            ConvexShape3D.Capsule(0.3f, 0.25f),
            ConvexShape3D.Capsule(1.5f, 0.1f),
            ConvexShape3D.Box(new Vector3F(0.5f, 0.3f, 0.7f)),
            ConvexShape3D.Box(new Vector3F(0.5f, 0.3f, 0.7f), 0.1f),
            ConvexShape3D.Cylinder(0.5f, 0.3f),
        };
        for (int h = 0; h < 4; h++)
        {
            var pts = new Vector3F[6 + h * 3];
            for (int i = 0; i < pts.Length; i++) { pts[i] = rng.InBall(0.8f); }
            list.Add(ConvexShape3D.FromHull(ConvexHull3D.Build(pts), h == 3 ? 0.05f : 0f));
        }

        return list;
    }

    [Fact]
    public void RaysAndSphereSweepsAgreeWithTheGeneralRoute()
    {
        var rng = new TestRng(0x5EE9);
        int decided = 0, total = 0, hits = 0;
        foreach (var shape in Shapes(rng))
        {
            for (int i = 0; i < 4000; i++)
            {
                float mover = (i & 1) == 0 ? 0f : rng.Range(0.05f, 0.5f);
                Vector3F o = rng.InBall(3f);
                Vector3F d = (i % 3 == 0) ? rng.OnSphere() * rng.Range(0.5f, 6f) : (rng.InBall(0.7f) - o) * rng.Range(0.8f, 1.6f); // two thirds aimed near the shape
                total++;
                var outcome = ConvexSweep3D.Test(shape, mover, o, d, out float f, out Vector3F n);
                if (outcome == SweepOutcome.Unsure) { continue; }
                decided++;
                bool gHit; float gf; Vector3F gn;
                if (mover == 0f)
                {
                    gHit = Gjk3D.RayCast(shape, RigidTransformF.Identity, o, d, 1f, out var h);
                    gf = h.Fraction; gn = h.Normal;
                }
                else
                {
                    var rel = new RigidTransformF(QuaternionF.Identity, o);
                    gHit = Gjk3D.ShapeCast(shape, ConvexShape3D.Sphere(mover), rel, -d, out var h);
                    gf = h.Fraction; gn = h.Normal;
                    if (gHit && h.StartsOverlapping) { Assert.Fail($"closed form answered a start inside: kind {shape.Kind} i {i}"); }
                }

                if (outcome == SweepOutcome.Miss)
                {
                    Assert.False(gHit, $"closed form missed but the general route hit at {gf}: kind {shape.Kind} radius {mover} i {i}");
                    continue;
                }

                hits++;
                Assert.True(gHit, $"closed form hit at {f} but the general route missed: kind {shape.Kind} radius {mover} i {i}");
                float len = d.Length();
                Assert.True(System.Math.Abs(f - gf) * len < 2e-3f, $"fraction {f} against {gf}: kind {shape.Kind} radius {mover} i {i}");
                Assert.True(n.X * gn.X + n.Y * gn.Y + n.Z * gn.Z > 0.99f, $"normal {n} against {gn}: kind {shape.Kind} radius {mover} i {i}");
            }
        }

        Assert.True(decided > total / 4, $"only {decided} of {total} decided");
        Assert.True(hits > 3000, $"only {hits} hits");
    }

    [Fact]
    public void StartsInsideAreLeftToTheGeneralRoute()
    {
        var shape = ConvexShape3D.Capsule(0.3f, 0.25f);
        Assert.Equal(SweepOutcome.Unsure, ConvexSweep3D.Test(shape, 0f, new Vector3F(0.1f, 0.1f, 0f), new Vector3F(1f, 0f, 0f), out _, out _));
        Assert.Equal(SweepOutcome.Unsure, ConvexSweep3D.Test(ConvexShape3D.Box(new Vector3F(1f, 1f, 1f)), 0f, default, new Vector3F(1f, 0f, 0f), out _, out _));
    }
}
