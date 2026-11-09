using System;
using System.Linq;
using RecursiveAI.OneMath;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class Sdf2DTests
{
    [Fact]
    public void PolygonFieldIsExactAndContourClosesAroundIt()
    {
        Vec2[] poly = [new(0, 0), new(4, 0), new(4, 1), new(2, 3), new(0, 1)];
        var f = SdfGrid2D.FromPolygon(poly, 0.1f, 3);
        Assert.InRange(f.Value(new(2, 0.5f)), -0.55f, -0.45f); Assert.InRange(f.Value(new(5, 0.5f)), 0.95f, 1.05f); Assert.InRange(f.Value(new(2, -0.4f)), 0.38f, 0.42f);
        var loops = f.ExtractContours(0.02f); Assert.Single(loops);
        var loop = loops[0]; Assert.InRange(loop.Length, 5, 60);
        // Counter-clockwise (positive area) with every point on the zero set within half a cell.
        double area = 0; for (int i = 0; i < loop.Length; i++) { var a = loop[i]; var b = loop[(i + 1) % loop.Length]; area += a.X * b.Y - b.X * a.Y; }
        Assert.True(area > 0);
        foreach (var p in loop) Assert.True(Math.Abs(f.Value(p)) < 0.05f, "contour point " + p + " has field " + f.Value(p));
        Assert.True(Math.Abs(area / 2 - 8.0) < 0.3, "polygon area 8, contour area " + area / 2);
    }

    [Fact]
    public void ContoursOfSeveralBlobsAndHolesAreSeparateLoops()
    {
        var f = SdfGrid2D.FromFunction(new(-6, -4), new(6, 4), 0.1f, SdfGrid2D.Difference(SdfGrid2D.Union(SdfGrid2D.Circle(new(-3, 0), 1.5f), SdfGrid2D.Box(new(3, 0), new(1.5f, 1.5f))), SdfGrid2D.Circle(new(3, 0), 0.6f)));
        Assert.Equal(3, f.ExtractContours(0.02f).Count); // two outlines and the hole
        var solid = SdfGrid2D.FromFunction(new(0, 0), new(2, 2), 0.1f, SdfGrid2D.Circle(new(1, 1), 5f)); // fills the whole grid
        Assert.Single(solid.ExtractContours());
        Assert.Empty(SdfGrid2D.FromFunction(new(0, 0), new(2, 2), 0.1f, SdfGrid2D.Circle(new(1, 1), -1f)).ExtractContours());
    }

    [Fact]
    public void SphereTraceHitsTheContourWithinATolerance()
    {
        var f = SdfGrid2D.FromFunction(new(-4, -4), new(4, 4), 0.1f, SdfGrid2D.Circle(default, 2f));
        Assert.True(f.Trace(new(0, 5), new(0, -1), 20, out float t, out var n, out bool inside)); Assert.False(inside);
        Assert.InRange(t, 2.99f, 3.01f); Assert.InRange(n.Y, 0.99f, 1.01f);
        Assert.False(f.Trace(new(3.5f, 5), new(0, -1), 20, out _, out _, out _) && false);
        Assert.False(f.Trace(new(3.9f, 3.9f), new(1, 1), 20, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => f.Trace(new(0, 5), default, 1, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => f.Value(new(float.NaN, 0)));
    }

    [Fact]
    public void BodiesRestOnAndSlideAcrossFieldTerrain()
    {
        var w = new P2.PhysicsWorld2D(new() { SleepEnabled = false });
        // A bowl-shaped terrain: a slab with a circular dip.
        var field = SdfGrid2D.FromFunction(new(-12, -4), new(12, 3), 0.1f, SdfGrid2D.Difference(SdfGrid2D.Box(new(0, -2), new(11, 2)), SdfGrid2D.Circle(new(0, 3), 4f)));
        var terrain = w.AddSdf(field, material: new P2.Material2D(0.6f));
        var ball = w.AddCircle(0.4f, new(-2, 2)); var box = w.AddBox(new(0.8f, 0.8f), new(6, 1.5));
        for (int i = 0; i < 600; i++) w.Step(1f / 60);
        // The ball swings in the circular dip (it stays between the rims, never leaves through the contour); the box rests on the flat.
        Assert.InRange(ball.Position.X, -2.6, 2.6); Assert.InRange(ball.Position.Y, -0.75, 0.1);
        Assert.InRange(box.Position.Y, 0.35, 0.5);
        Assert.True(w.Raycast(new(8, 5), new(0, -1), out var hit, 10)); Assert.InRange(hit.Distance, 3.9, 5.1);
    }

    [Fact]
    public void FieldTerrainSnapshotsAndHashesLikeAnyOtherWorld()
    {
        var w = new P2.PhysicsWorld2D(); var field = SdfGrid2D.FromPolygon([new(0, 0), new(8, 0), new(8, 1), new(0, 1)], 0.1f);
        w.AddSdf(field); w.AddBox(new(0.5f, 0.5f), new(3, 3));
        for (int i = 0; i < 100; i++) w.Step(1f / 60);
        var b = new P2.PhysicsWorld2D(); b.Restore(w.Snapshot()); Assert.Equal(w.Snapshot(), b.Snapshot());
        for (int i = 0; i < 30; i++) { w.Step(1f / 60); b.Step(1f / 60); Assert.Equal(w.ComputeHash(), b.ComputeHash()); }
        Assert.Throws<ArgumentOutOfRangeException>(() => SdfGrid2D.FromPolygon([new(0, 0), new(1, 0)], 0.1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => SdfGrid2D.FromPolygon([new(0, 0), new(float.NaN, 0), new(1, 1)], 0.1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => new SdfGrid2D(1, 4, new float[4], 1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => field.ExtractContours(-1));
        Assert.Throws<ArgumentOutOfRangeException>(() => SdfGrid2D.FromFunction(new(0, 0), new(1e6f, 1e6f), 0.1f, _ => 1f));
    }
}
