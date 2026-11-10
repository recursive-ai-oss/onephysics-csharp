using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Fuzz;

// Behaviours behind fuzz findings that deserve a statement of their own beside the replayed inputs in testdata/fuzz.
public sealed class Robustness2DFindingsTests
{
    // A slider anchored 200 m from two light bodies asks the solver for enormous impulses. The speed limit applied before the solve is not enough: runaway speeds must also be cut before bodies move,
    // so the world stays finite however a joint is posed.
    [Fact]
    public void AFarAnchoredJointKeepsEveryBodyFinite()
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 9, Substeps = 4 });
        var a = w.AddCircle(0.5f, new Vector2D(1, 6));
        var b = w.AddCircle(0.5f, new Vector2D(0, 8), BodyType.Static);
        w.AddSlider(a, b, new Vector2D(-124.9375, 157.3125), new Vector2F(4.015625f, 68.69922f));
        for (int i = 0; i < 20; i++)
        {
            w.Step(1f / 60f);
            var p = a.Position; var v = a.LinearVelocity;
            Assert.True(double.IsFinite(p.X) && double.IsFinite(p.Y) && float.IsFinite(v.X) && float.IsFinite(v.Y));
            Assert.True(v.X * v.X + v.Y * v.Y <= 6400.5f * 6400.5f);
        }

        Assert.NotEmpty(w.Snapshot());
    }
}
