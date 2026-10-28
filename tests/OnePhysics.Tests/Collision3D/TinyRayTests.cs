using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Collision3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

public class TinyRayTests
{
    [Fact]
    public void DenormalLengthRayDoesNotHitAShapeMetresAway()
    {
        var box = ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f), 0.001f);
        var at = new RigidTransformF(QuaternionF.Identity, new Vector3F(-4f, 0f, 0f));
        Assert.False(Gjk3D.RayCast(box, at, new Vector3F(-5.9375f, -8f, 1f), new Vector3F(-8f, -8f, -8f), 3.18e-43f, out _));
    }

    [Fact]
    public void TinyRayStartingInsideStillHitsAtZero()
    {
        var box = ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f), 0.001f);
        var at = new RigidTransformF(QuaternionF.Identity, new Vector3F(-4f, 0f, 0f));
        Assert.True(Gjk3D.RayCast(box, at, new Vector3F(-4f, 0f, 0f), new Vector3F(1f, 0f, 0f), 1e-30f, out var hit));
        Assert.Equal(0f, hit.Fraction);
    }
}

public class AspectRatioTests
{
    [Fact]
    public void ShapesThinnerThanTheDocumentedRatioAreRefused()
    {
        Assert.Throws<System.ArgumentOutOfRangeException>(() => ConvexShape3D.Box(new Vector3F(524288f, 1f, 0.0078125f)));
        Assert.Throws<System.ArgumentOutOfRangeException>(() => ConvexShape3D.Cylinder(1.5e6f, 0.2f));
        Assert.Throws<System.ArgumentOutOfRangeException>(() => ConvexShape3D.Capsule(1e6f, 0.5f));
        Assert.Throws<System.ArgumentOutOfRangeException>(() => ConvexShape3D.TaperedCapsule(1e6f, 0.5f, 0.6f));
    }

    [Fact]
    public void TheRatioLimitItselfIsAccepted()
    {
        _ = ConvexShape3D.Box(new Vector3F(1048576f, 1f, 1f));
        _ = ConvexShape3D.Box(new Vector3F(1000f, 1f, 0.01f));
        _ = ConvexShape3D.Capsule(1000f, 0.01f);
    }
}
