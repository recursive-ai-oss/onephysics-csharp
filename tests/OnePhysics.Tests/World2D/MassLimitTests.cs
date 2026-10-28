using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

public class MassLimitTests
{
    [Fact]
    public void TinyBodyAloneIsRefused()
    {
        var w = new PhysicsWorld2D();
        var body = w.CreateBody(BodyType.Dynamic, default);
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateCircle(body, 0.5f, default, new ShapeDef { Density = 1e-30f }));
        Assert.Contains("inertia", ex.Message, StringComparison.Ordinal);
        // The refused shape is gone and the body steps as an empty dynamic body.
        w.Step(1f / 60f);
    }

    [Fact]
    public void LimitsAreInclusiveAtTheDocumentedBounds()
    {
        var w = new PhysicsWorld2D();
        var body = w.CreateBody(BodyType.Dynamic, default);
        // Circle of radius 1: mass pi d, inertia pi d / 2, so density 1e-19 falls below the 1e-18 floor and 1e-17 clears it.
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateCircle(body, 1f, default, new ShapeDef { Density = 1e-19f }));
        w.CreateCircle(body, 1f, default, new ShapeDef { Density = 1e-17f });
        Assert.True(body.Mass > 0f);
    }
}
