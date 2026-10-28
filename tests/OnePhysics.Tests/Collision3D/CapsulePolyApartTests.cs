using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

/// <summary>The capsule-against-polyhedron early exit against GJK distance over random poses: it only fires for pairs that really are apart, and its clearance never overstates the gap.</summary>
public class CapsulePolyApartTests
{
    [Fact]
    public void ApartPairsAreApartAndClearanceIsALowerBound()
    {
        var rng = new TestRng(77);
        var hulls = Collision3DHash.RandomHulls(rng, 8);
        var gen = new ManifoldGenerator3D();
        int exits = 0, apart = 0;
        for (int i = 0; i < 20000; i++)
        {
            ConvexShape3D cap = ConvexShape3D.Capsule(rng.Range(0.1f, 1f), rng.Range(0.1f, 0.6f));
            ConvexShape3D poly = rng.Next(2) == 0
                ? ConvexShape3D.Box(new Vector3F(rng.Range(0.3f, 1.2f), rng.Range(0.3f, 1.2f), rng.Range(0.3f, 1.2f)), rng.Next(2) == 0 ? 0f : rng.Range(0f, 0.25f))
                : ConvexShape3D.FromHull(hulls[rng.Next(hulls.Length)], rng.Next(3) == 0 ? rng.Range(0f, 0.2f) : 0f);
            bool capIsA = rng.Next(2) == 0;
            ConvexShape3D a = capIsA ? cap : poly, b = capIsA ? poly : cap;
            var xf = new RigidTransformF(rng.Rotation(), rng.InBall(rng.Range(0.2f, 3.5f)));
            ConvexDistance3D d = Gjk3D.Distance(a, b, xf);
            float gap = d.Overlapping ? -1f : d.Distance;
            Manifold3D m = gen.Collide(a, b, xf, 0.02f);
            if (gap > 0.02f + 1e-3f)
            {
                apart++;
                Assert.Equal(0, m.PointCount);
            }

            if (m.PointCount == 0 && gen.ClearanceOut > 0f)
            {
                exits++;
                Assert.True(gap > 0.02f - 1e-3f, $"pair {i}: reported clear by {gen.ClearanceOut} but the gap is {gap}");
                Assert.True(gen.ClearanceOut <= gap - 0.02f + 1e-3f, $"pair {i}: clearance {gen.ClearanceOut} exceeds the gap {gap} beyond the speculative distance");
            }
        }

        Assert.True(exits > 1000 && apart > exits / 4, $"{exits} exits, {apart} apart pairs");
    }
}
