using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

public class MassLimitTests3D
{
    [Fact]
    public void TinyDensityIsRefusedWithTypedError()
    {
        var w = new PhysicsWorld3D();
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() => w.AddSphere(1e-3f, density: 1e-30f));
        Assert.Contains("inertia", ex.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void ReasonableBodiesPassAndStep()
    {
        var w = new PhysicsWorld3D();
        var b = w.AddBox(new Vector3F(1, 1, 1), new Vector3D(0, 5, 0));
        w.Step(1f / 60f);
        Assert.True(b.Mass > 0f);
    }

    [Fact]
    public void NonFiniteVelocityIsRefusedBeforeTheStep()
    {
        var w = new PhysicsWorld3D();
        var b = w.AddBox(new Vector3F(1, 1, 1), new Vector3D(0, 5, 0));
        w.Core.Bodies.LinearVelocityOf(0)[0] = float.NaN;
        var ex = Assert.Throws<BodyOutOfBoundsException>(() => w.Step(1f / 60f));
        Assert.Contains("non-finite position, velocity or rotation", ex.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void FrictionAboveTheLimitIsRefused()
    {
        var w = new PhysicsWorld3D();
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddBox(new Vector3F(1, 1, 1), material: new Material3D(2e15f)));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Material3D(0.5f, 0f, float.PositiveInfinity));
        var ok = w.AddBox(new Vector3F(1, 1, 1), material: new Material3D(1e15f));
        Assert.True(ok.IsValid);
    }

    [Fact]
    public void ShiftOriginRefusesBeforeChangingAnythingWhenABodyWouldLeaveTheRange()
    {
        var w = new PhysicsWorld3D();
        w.AddBox(new Vector3F(1, 1, 1), new Vector3D(2000000, 0, 0));
        long before = (long)w.ComputeHash();
        Assert.Throws<BodyOutOfBoundsException>(() => w.ShiftOrigin(new Vector3D(-4000000, 0, 0)));
        Assert.Equal(before, (long)w.ComputeHash());
        w.Step(1f / 60f);
    }
}
