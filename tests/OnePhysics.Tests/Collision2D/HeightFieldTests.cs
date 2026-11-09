using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision2D;

public sealed class HeightFieldTests
{
    [Fact]
    public void GhostVerticesStopAtHoles()
    {
        var field = new HeightField2D([0,1,2,3,4], holes: [false,false,true,false]);
        Assert.True(field.GetSegment(0).HasGhost1);
        Assert.False(field.GetSegment(0).HasGhost2);
        Assert.False(field.GetSegment(1).HasGhost1);
        Assert.True(field.GetSegment(1).HasGhost2);
        Assert.False(field.GetSegment(3).HasGhost1);
        Assert.False(field.GetSegment(3).HasGhost2);
        Assert.Throws<ArgumentException>(() => field.GetSegment(2));
        Assert.Throws<ArgumentOutOfRangeException>(() => field.GetSegment(-1));
    }

    [Fact]
    public void FlatColumnJoinsDoNotKickSlidingBox()
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { Gravity = new Vector2F(0,-10) });
        w.AddHeightField(new float[65], material: new Material2D(friction: 0));
        var box = w.AddBox(new Vector2F(0.8f,1), at: new Vector2D(1,0.5), material: new Material2D(friction: 0));
        box.LinearVelocity = new Vector2F(3,0);
        // Check after the initial contact has formed: the speculative skin may correct the initial pose.
        for(int i=0;i<60;i++) w.Step(1f/60);
        for(int i=0;i<1000;i++)
        {
            w.Step(1f/60);
            Assert.InRange(box.LinearVelocity.Y, -1e-4f, 1e-4f);
        }
        Assert.True(box.Position.X > 50);
    }

    [Fact]
    public void HostileInputsAndStaticRestriction()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new HeightField2D([]));
        Assert.Throws<ArgumentOutOfRangeException>(() => new HeightField2D([0,0], 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new HeightField2D([0,0], float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => new HeightField2D([0,0], holes: [false,true]));
        Assert.Throws<InvalidShapeException>(() => new HeightField2D([0,float.NaN]));
        var w = new PhysicsWorld2D();
        Assert.Throws<StaticOnlyShapeException>(() => w.CreateHeightField(w.AddBox(new Vector2F(1,1)), new HeightField2D([0,0])));
    }

    [Fact]
    public void SnapshotsAndForksRetainHolesAndGhosts()
    {
        var a = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = 1 });
        var b = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = 8 });
        foreach (var w in new[] {a,b})
        {
            w.AddHeightField([0,0,0,0,0], holes: [false,false,true,false]);
            w.AddCircle(0.3f, at: new Vector2D(1,2));
        }
        var bytes = a.Snapshot();
        a.Restore(bytes); Assert.Equal(bytes,a.Snapshot());
        var fork = a.Fork();
        for(int i=0;i<60;i++)
        {
            a.Step(1f/60); b.Step(1f/60); fork.Step(1f/60);
            Assert.Equal(a.ComputeHash(),b.ComputeHash()); Assert.Equal(a.ComputeHash(),fork.ComputeHash());
        }
    }
}
