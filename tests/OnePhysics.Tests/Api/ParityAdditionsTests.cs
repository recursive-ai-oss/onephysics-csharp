using System;
using RecursiveAI.OneMath;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Api;

/// <summary>Members added so the two worlds match: each behaves as its counterpart in the other dimension.</summary>
public class ParityAdditionsTests
{
    [Fact]
    public void AngularImpulseChangesAngularVelocityByImpulseOverInertia2D()
    {
        var world = new P2.PhysicsWorld2D(new P2.WorldOptions2D { Gravity = default });
        var b = world.AddCircle(0.5f, at: default);
        b.ApplyAngularImpulse(2f);
        Assert.Equal(2f / b.Inertia, b.AngularVelocity, 5);
        Assert.Throws<ArgumentOutOfRangeException>(() => b.ApplyAngularImpulse(float.NaN));
        var wall = world.AddBox(new Vector2F(1, 1), at: new Vector2D(5, 0), type: BodyType.Static);
        Assert.Throws<BodyTypeException>(() => wall.ApplyAngularImpulse(1f));
    }

    [Fact]
    public void CapsuleOverlapFindsAShapeAndMissesAnEmptyRegion2D()
    {
        var world = new P2.PhysicsWorld2D();
        world.AddBox(new Vector2F(2, 2), at: default, type: BodyType.Static);
        Assert.True(world.OverlapCapsule(new Vector2D(1.2, 0), 0.5f, 0.3f, 0f, out _));
        Assert.True(world.OverlapCapsuleAny(new Vector2D(1.2, 0), 0.5f, 0.3f, 0f));
        Assert.Single(world.OverlapCapsuleAll(new Vector2D(1.2, 0), 0.5f, 0.3f, 0f));
        Assert.False(world.OverlapCapsuleAny(new Vector2D(10, 0), 0.5f, 0.3f, 0f));
        var buffer = new P2.QueryHit2D[4];
        Assert.Equal(1, world.OverlapCapsuleAll(new Vector2D(1.2, 0), 0.5f, 0.3f, 0f, buffer));
    }

    [Fact]
    public void BulletFlagRoundTripsAndStaticBodiesRefuseIt3D()
    {
        var world = new P3.PhysicsWorld3D();
        var b = world.AddSphere(0.5f, at: new Vector3D(0, 5, 0));
        Assert.False(b.IsBullet);
        b.IsBullet = true;
        Assert.True(b.IsBullet);
        b.IsBullet = false;
        Assert.False(b.IsBullet);
        var floor = world.AddBox(new Vector3F(10, 1, 10), at: default, type: BodyType.Static);
        Assert.Throws<BodyTypeException>(() => floor.IsBullet = true);
    }

    [Fact]
    public void PlanarOptionsValidateLikeSpatialOnes()
    {
        new P2.WorldOptions2D().Validate();
        Assert.Throws<ArgumentOutOfRangeException>(() => new P2.WorldOptions2D { SoftSubsteps = 0 }.Validate());
    }
}
