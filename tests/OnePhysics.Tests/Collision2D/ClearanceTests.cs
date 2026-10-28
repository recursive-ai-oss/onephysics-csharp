using System;
using RecursiveAI.OnePhysics.Collision2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision2D;

public class ClearanceTests
{
    private static Shape2D Random(ref TRng r)
    {
        switch (r.Int(4))
        {
            case 0: return Shape2D.CreateCircle(r.Point(0.5f), r.Range(0.1f, 1f));
            case 1: return Shape2D.CreateCapsule(new Vec2(-r.Range(0.1f, 1f), 0f), new Vec2(r.Range(0.1f, 1f), r.Range(-0.3f, 0.3f)), r.Range(0.1f, 0.5f));
            default: return TU.RandomPolygon(ref r, r.Int(2) == 0 ? 0f : r.Range(0.02f, 0.2f));
        }
    }

    private static float Reach(in Shape2D s)
    {
        float m = 0f;
        foreach (Vec2 v in s.Vertices) { m = Math.Max(m, v.Length); }
        return m + s.Radius;
    }

    // An empty manifold with a clearance stays empty for every rigid motion of B that moves none of its points by more than the clearance:
    // the translation plus the chord of the turn times the farthest point's distance from B's origin.
    [Fact]
    public void EmptyManifoldStaysEmptyWithinItsClearance()
    {
        var r = new TRng(77);
        int proven = 0;
        for (int trial = 0; trial < 20000; trial++)
        {
            Shape2D a = Random(ref r), b = Random(ref r);
            var xf = new Transform2(r.Point(4f), r.Rot());
            Manifold2D m = Collide2D.Collide(in a, in b, xf, Constants2D.SpeculativeDistance, out float clearance);
            if (m.PointCount != 0) { Assert.Equal(0f, clearance); continue; }
            if (!(clearance > 0f)) { continue; }
            proven++;
            float reach = Reach(in b);
            for (int k = 0; k < 8; k++)
            {
                Rot2 turn = r.Int(2) == 0 ? Rot2.Identity : Rot2.FromVector(1f, r.Range(-0.5f, 0.5f));
                float chord = MathF.Sqrt(2f * (1f - turn.C));
                float budget = clearance * 0.98f - 1e-4f - chord * reach;
                if (budget <= 0f) { continue; }
                Vec2 dir = r.Point(1f);
                float len = dir.Length;
                if (len < 1e-3f) { continue; }
                Vec2 t = dir * (r.Range(0f, 1f) * budget / len);
                var moved = new Transform2(xf.P + t, Rot2.Mul(xf.Q, turn));
                Manifold2D m2 = Collide2D.Collide(in a, in b, moved, Constants2D.SpeculativeDistance, out _);
                Assert.True(m2.PointCount == 0, "trial " + trial + " clearance " + clearance + " budget " + budget);
            }
        }

        Assert.True(proven > 2000, "only " + proven + " pairs had a clearance");
    }
}
