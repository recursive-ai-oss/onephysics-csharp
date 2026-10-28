using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

public class ContactRecycleTests
{
    private const float Dt = 1f / 60f;

    private static float SlideSpeedAfter(bool dropFriction)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = 1, WorldId = 3 });
        w.AddBox(new Vector2F(100f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static, material: new Material2D(1f));
        var box = w.AddBox(new Vector2F(1f, 1f), at: new Vector2D(0, 0.5), material: new Material2D(1f));
        box.LinearVelocity = new Vector2F(5f, 0f);
        for (int i = 0; i < 8; i++) { w.Step(Dt); }
        if (dropFriction) { box.GetShapes()[0].Friction = 0f; }
        for (int i = 0; i < 12; i++) { w.Step(Dt); }
        return box.LinearVelocity.X;
    }

    [Fact]
    public void AFrictionChangeReachesAContactThatKeepsItsManifold()
    {
        // With friction 1 against friction 1 the box loses 10 m/s every second; with the box's friction set to 0 the mixed friction (geometric mean) is 0 and it keeps the speed it had.
        float held = SlideSpeedAfter(dropFriction: true);
        float braked = SlideSpeedAfter(dropFriction: false);
        Assert.True(held > 3.6f, $"slid at {held} after friction was set to 0");
        Assert.True(braked < 2.5f, $"slid at {braked} with friction 1");
    }

    [Fact]
    public void ARestingPileOfDiscsKeepsRoundContactsAndStaysInsideItsBox()
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = 1, WorldId = 4 });
        w.AddBox(new Vector2F(20f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        w.AddBox(new Vector2F(1f, 20f), at: new Vector2D(-6, 10), type: BodyType.Static);
        w.AddBox(new Vector2F(1f, 20f), at: new Vector2D(6, 10), type: BodyType.Static);
        var discs = new Body[60];
        for (int i = 0; i < discs.Length; i++) { discs[i] = w.AddCircle(0.3f, new Vector2D(-4 + (i % 10) * 0.85, 1 + (i / 10) * 0.7)); }
        int recycled = 0;
        for (int i = 0; i < 400; i++) { w.Step(Dt); if (i > 60) { recycled += w.Engine.RecycledCount; } }

        var cs = w.Engine.Core.Constraints;
        int round = 0;
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (cs.IsLive(c) && cs.KindOf(c) == ConstraintKind.Contact && cs.PointCount[c] == 1 && cs.Payload(c)[32] == 3f) { round++; }
        }

        Assert.True(round > 10, $"only {round} round contacts");
        Assert.True(recycled > 0);
        foreach (var d in discs)
        {
            Assert.InRange(d.Position.X, -5.8, 5.8);
            Assert.InRange(d.Position.Y, 0.2, 12);
        }
    }

    [Fact]
    public void RecycledAndFreshManifoldsGiveTheSameRestingColumnOfBoxes()
    {
        // Recycling is an approximation of a fresh manifold, so a settled stack must end up where it does with recycling off.
        double Top(bool recycle)
        {
            var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = 1, WorldId = 5 });
            w.Engine.RecycleManifolds = recycle;
            w.AddBox(new Vector2F(20f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
            Body top = default;
            for (int i = 0; i < 8; i++) { top = w.AddBox(new Vector2F(1f, 1f), at: new Vector2D(0, 0.5 + i * 1.01)); }
            for (int i = 0; i < 300; i++) { w.Step(Dt); }
            return top.Position.Y;
        }

        Assert.InRange(Math.Abs(Top(true) - Top(false)), 0.0, 0.03);
    }
}
