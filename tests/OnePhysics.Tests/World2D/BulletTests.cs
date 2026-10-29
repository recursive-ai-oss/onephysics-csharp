using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

public class BulletTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D NoGravity(int threads = 1, int id = 41)
        => new(new WorldOptions2D { Gravity = default, ThreadCount = threads, WorldId = id });

    private static Body Bullet(PhysicsWorld2D w, double x, double y, float vx, float vy, bool bullet)
    {
        var def = new BodyDef { Type = BodyType.Dynamic, Position = new Vector2D(x, y), LinearVelocity = new Vector2F(vx, vy), IsBullet = bullet };
        var b = w.CreateBody(in def);
        w.CreateCircle(b, 0.1f);
        return b;
    }

    private static Body ThinBox(PhysicsWorld2D w, double x, double y, float vx, float vy, float hx = 0.05f, float hy = 2f)
    {
        var def = new BodyDef { Type = BodyType.Dynamic, Position = new Vector2D(x, y), LinearVelocity = new Vector2F(vx, vy) };
        var b = w.CreateBody(in def);
        w.CreateBox(b, new Vector2F(2f * hx, 2f * hy));
        return b;
    }

    [Fact]
    public void ABulletStopsAtTheAnalyticTimeOfImpactAgainstAThinMovingBox()
    {
        var w = NoGravity();
        var bullet = Bullet(w, 0, 0, 100f, 0f, true);
        var box = ThinBox(w, 20, 0, -50f, 0f);
        // Closing speed 150 m/s; the surfaces are 20 - 0.05 - 0.1 = 19.85 apart. Continuous collision stops the bullet where the gap is the target gap (0.005):
        // after tau = (19.85 - 0.005) / 150 s, i.e. at x = 100 tau, inside step 8 (tau = 0.1323 s lies between steps 7 and 8 of 1/60 s).
        double expected = 100.0 * (19.85 - 0.005) / 150.0;
        for (int i = 0; i < 7; i++) { w.Step(Dt); }
        Assert.InRange(bullet.WorldCenterOfMass.X, 11.6, 11.7); // 7 steps of 1.667 m, untouched
        w.Step(Dt);
        Assert.InRange(bullet.WorldCenterOfMass.X, expected - 0.003, expected + 0.003);
        Assert.Equal(100f, bullet.LinearVelocity.X, 3);
        Assert.True(bullet.WorldCenterOfMass.X < box.WorldCenterOfMass.X - 0.05, "the bullet is still on its side of the box");
    }

    [Fact]
    public void WithoutTheFlagTheSameBodyTunnelsThroughTheBox()
    {
        var w = NoGravity();
        var plain = Bullet(w, 0, 0, 100f, 0f, false);
        var box = ThinBox(w, 20, 0, -50f, 0f);
        for (int i = 0; i < 8; i++) { w.Step(Dt); }
        Assert.True(plain.WorldCenterOfMass.X > box.WorldCenterOfMass.X, "no continuous collision against a moving body without the bullet flag");
    }

    [Fact]
    public void ABulletHitsABoxMovingAcrossItsPathAndStaysOnTheNearSide()
    {
        double Run(bool flag, out double plateY)
        {
            var w = NoGravity();
            // The bullet flies along +x at 2 m per step; a plate 1 m tall falls 2 m per step across its path during step 5: it is clear above the path after step 4 and clear below it after step 5,
            // and the two cross at x = 9, so only a sweep sees the hit.
            var bullet = Bullet(w, 0, 0, 120f, 0f, flag);
            var plate = ThinBox(w, 9.15, 9.0, 0f, -120f, 0.05f, 0.5f);
            for (int i = 0; i < 5; i++) { w.Step(Dt); }
            plateY = plate.WorldCenterOfMass.Y;
            return bullet.WorldCenterOfMass.X;
        }

        Assert.True(Run(true, out double y) < 9.0, "the bullet is stopped before the plate's line");
        Assert.InRange(y, -1.01, -0.99);
        Assert.Equal(10.0, Run(false, out _), 2);
    }

    [Fact]
    public void TheResultDoesNotDependOnTheThreadCount()
    {
        ulong Run(int threads)
        {
            var w = NoGravity(threads, 43);
            for (int i = 0; i < 150; i++)
            {
                Bullet(w, -40 + (i % 7) * 0.2, -30 + i * 0.4, 90f + (i % 5) * 7f, (i % 3) - 1f, i % 2 == 0);
            }

            for (int i = 0; i < 30; i++) { ThinBox(w, -20 + i * 1.5, 0, 0f, (i % 2 == 0 ? 1 : -1) * 40f, 0.05f, 3f); }
            ulong h = 0;
            for (int s = 0; s < 40; s++) { w.Step(Dt); h = h * 31 + w.ComputeHash(); }
            return h;
        }

        ulong one = Run(1);
        Assert.Equal(one, Run(4));
        Assert.Equal(one, Run(8));
    }

    [Fact]
    public void EqualTimesOfImpactGoToTheLowerShapeSlot()
    {
        // Two identical plates at the same distance: the bullet is stopped at the same time by both whichever order they were created in; the pose is the same either way.
        double Run(bool swap)
        {
            var w = NoGravity();
            var bullet = Bullet(w, 0, 0, 100f, 0f, true);
            if (swap) { ThinBox(w, 5, 0.0, 0f, 0f); ThinBox(w, 5, 0.0, 0f, 0f, 0.05f, 2f); }
            else { ThinBox(w, 5, 0.0, 0f, 0f, 0.05f, 2f); ThinBox(w, 5, 0.0, 0f, 0f); }
            for (int i = 0; i < 6; i++) { w.Step(Dt); }
            return bullet.WorldCenterOfMass.X;
        }

        Assert.Equal(Run(false), Run(true));
    }

    [Fact]
    public void TheBulletFlagCanBeChangedAndFailsOnStaticAndStaleBodies()
    {
        var w = NoGravity();
        var b = Bullet(w, 0, 0, 1f, 0f, false);
        Assert.False(b.IsBullet);
        b.IsBullet = true;
        Assert.True(b.IsBullet);
        var wall = w.AddBox(new Vector2F(1, 1), type: BodyType.Static);
        Assert.Throws<BodyTypeException>(() => wall.IsBullet = true);
        var snap = w.Snapshot();
        var w2 = NoGravity(1, 41);
        w2.Restore(snap);
        Assert.True(w2.GetBody(b.RawHandle).IsBullet);
        b.Destroy();
        Assert.Throws<StaleHandleException>(() => _ = b.IsBullet);
    }
}
