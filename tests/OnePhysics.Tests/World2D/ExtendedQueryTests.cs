using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

public class ExtendedQueryTests
{
    private static PhysicsWorld2D World(int threads = 1) => new(new WorldOptions2D { ThreadCount = threads, WorldId = 73, Gravity = default });

    [Fact]
    public void SelectedQueriesIgnoreNearerWorldShapesAndRejectStaleHandles()
    {
        var w = World();
        var near = w.AddCircle(1f, new Vector2D(5, 0), BodyType.Static);
        var far = w.AddCircle(1f, new Vector2D(10, 0), BodyType.Static);
        var shapes = far.GetShapes();
        Assert.True(w.Raycast(new Vector2D(0, 0), new Vector2F(1, 0), out var hit));
        Assert.Equal(near, hit.Body);
        Assert.True(far.Raycast(new Vector2D(0, 0), new Vector2F(1, 0), out hit));
        Assert.Equal(9f, hit.Distance, 3);
        Assert.True(shapes[0].ClosestPoint(new Vector2D(0, 0), out hit));
        Assert.Equal(9f, hit.Distance, 3);
        var mover = Shape2D.CreateCircle(default, 1f);
        Assert.True(far.ShapeCast(mover, new Vector2D(0, 0), 0f, new Vector2F(1, 0), out hit, 20f));
        Assert.Equal(8f, hit.Distance, 2);
        Assert.True(shapes[0].OverlapShape(mover, new Vector2D(10, 0), 0f, out hit));
        Assert.False(shapes[0].Raycast(new Vector2D(0, 0), new Vector2F(1, 0), out _, 20f, QueryFilter2D.Layers(0)));
        far.Destroy();
        Assert.Throws<StaleHandleException>(() => far.Raycast(new Vector2D(0, 0), new Vector2F(1, 0), out _));
        Assert.Throws<StaleHandleException>(() => shapes[0].ClosestPoint(new Vector2D(0, 0), out _));
    }

    [Fact]
    public void BroadphaseUsesInclusiveBoxesAndStableTruncation()
    {
        var w = World();
        var a = w.AddCircle(1f, new Vector2D(5, 0), BodyType.Static);
        w.AddCircle(1f, new Vector2D(5, 0), BodyType.Static);
        var results = new Shape[1];
        Assert.Equal(1, w.QueryAabb(new Vector2D(3, -2), new Vector2D(7, 2), results));
        Assert.Equal(a, results[0].Body);
        Assert.Equal(0, w.QueryAabb(new Vector2D(0, 0), new Vector2D(0, 0), results));
        Assert.Equal(0, w.QueryAabb(new Vector2D(3, -2), new Vector2D(7, 2), results, QueryFilter2D.Layers(0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.QueryAabb(new Vector2D(5, 0), new Vector2D(0, 0), results));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.QueryAabb(new Vector2D(double.NaN, 0), new Vector2D(0, 0), results));
    }

    [Fact]
    public void MixedBatchMatchesScalarAtOneAndEightThreads()
    {
        var one = World();
        var eight = World(8);
        one.AddCircle(1f, new Vector2D(5, 0), BodyType.Static);
        eight.AddCircle(1f, new Vector2D(5, 0), BodyType.Static);
        var shape = Shape2D.CreateCircle(default, 1f);
        var inputs = new QueryInput2D[513];
        for (int i = 0; i < inputs.Length; i++)
        {
            inputs[i] = (i % 4) switch
            {
                0 => QueryInput2D.Ray(new Vector2D(0, 0), new Vector2F(1, 0), 20f),
                1 => QueryInput2D.Cast(shape, new Vector2D(0, 0), 0f, new Vector2F(1, 0), 20f),
                2 => QueryInput2D.Overlap(shape, new Vector2D(5, 0), 0f),
                _ => QueryInput2D.Closest(new Vector2D(0, 0), 20f)
            };
        }
        var a = new QueryHit2D[inputs.Length];
        var b = new QueryHit2D[inputs.Length];
        Assert.Equal(inputs.Length, one.QueryBatch(inputs, a));
        Assert.Equal(inputs.Length, eight.QueryBatch(inputs, b));
        for (int i = 0; i < inputs.Length; i++)
        {
            Assert.Equal(a[i].Distance, b[i].Distance);
            Assert.Equal(a[i].Point, b[i].Point);
            Assert.Equal(a[i].Normal, b[i].Normal);
            Assert.Equal(a[i].Shape.RawHandle.Index, b[i].Shape.RawHandle.Index);
            QueryHit2D expected;
            switch (i % 4)
            {
                case 0: Assert.True(one.Raycast(new Vector2D(0, 0), new Vector2F(1, 0), out expected, 20f)); break;
                case 1: Assert.True(one.ShapeCast(shape, new Vector2D(0, 0), 0f, new Vector2F(1, 0), out expected, 20f)); break;
                case 2: Assert.True(one.OverlapShape(shape, new Vector2D(5, 0), 0f, out expected)); break;
                default: Assert.True(one.ClosestPoint(new Vector2D(0, 0), out expected, 20f)); break;
            }
            Assert.Equal(expected.Distance, a[i].Distance);
            Assert.Equal(expected.Point, a[i].Point);
        }
        var counts = new int[inputs.Length];
        var all = new QueryHit2D[inputs.Length * 2];
        eight.QueryAllBatch(inputs, all, counts, 2);
        var any = new bool[inputs.Length];
        eight.QueryAnyBatch(inputs, any);
        for (int i = 0; i < inputs.Length; i++) { Assert.Equal(1, counts[i]); Assert.True(any[i]); Assert.True(all[2 * i].IsHit); Assert.False(all[2 * i + 1].IsHit); }
        eight.QueryAllBatch(inputs, Array.Empty<QueryHit2D>(), counts, 0);
        Assert.All(counts, count => Assert.Equal(0, count));
    }

    [Fact]
    public void BatchValidatesBeforeWritingAndDoesNotChangeWorldHash()
    {
        var w = World();
        w.AddCircle(1f, new Vector2D(5, 0), BodyType.Static);
        ulong hash = w.ComputeHash();
        var inputs = new[] { QueryInput2D.Ray(new Vector2D(0, 0), new Vector2F(1, 0)), default };
        var hits = new QueryHit2D[2];
        Assert.Throws<ArgumentException>(() => w.QueryBatch(inputs, hits));
        Assert.False(hits[0].IsHit);
        Assert.Throws<ArgumentException>(() => w.QueryBatch(inputs, Array.Empty<QueryHit2D>()));
        Assert.Throws<ArgumentOutOfRangeException>(() => QueryInput2D.Ray(new Vector2D(0, 0), new Vector2F(1, 0), -1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => QueryInput2D.Ray(new Vector2D(0, 0), default));
        w.QueryBatch(inputs.AsSpan(0, 1), hits);
        Assert.Equal(hash, w.ComputeHash());
    }

    [Fact]
    public void PenetrationDepthAndNormalSeparateSpheres()
    {
        var w = World();
        w.AddCircle(1f, default, BodyType.Static);
        var shape = Shape2D.CreateCircle(default, 1f);
        Assert.True(w.Penetration(shape, new Vector2D(1.5, 0), 0f, out var hit));
        Assert.Equal(0.5f, hit.Distance, 3);
        Assert.Equal(1f, hit.Normal.X, 3);
        Assert.False(w.Penetration(shape, new Vector2D(2.1, 0), 0f, out _));
        var all = new QueryHit2D[2];
        Assert.Equal(1, w.PenetrationAll(shape, new Vector2D(1.5, 0), 0f, all));
        Assert.Equal(hit.Distance, all[0].Distance);
        var batch = new[] { QueryInput2D.Penetration(shape, new Vector2D(1.5, 0), 0f) };
        Assert.Equal(1, w.QueryBatch(batch, all));
        Assert.Equal(hit.Distance, all[0].Distance);
    }

    [Fact]
    public void WholeBodySweepExcludesSelfAndLeavesWorldUnchanged()
    {
        var w = World();
        var source = w.AddCircle(1f, default);
        var target = w.AddCircle(1f, new Vector2D(5, 0), BodyType.Static);
        ulong hash = w.ComputeHash();
        Assert.True(source.Sweep(new Vector2F(1, 0), out var hit, 10f));
        Assert.Equal(target, hit.Body);
        Assert.Equal(3f, hit.Distance, 3);
        Assert.Equal(hash, w.ComputeHash());
        Assert.False(source.Sweep(new Vector2F(-1, 0), out _, 10f));
        Assert.Throws<ArgumentOutOfRangeException>(() => source.Sweep(default, out _));
    }

    [Fact]
    public void MixedBatchBuffersAreAllocationFreeAfterWarmup()
    {
        var w = World();
        w.AddCircle(1f, new Vector2D(5, 0), BodyType.Static);
        var queries = new PhysicsQuery2D(w);
        var inputs = new[] { QueryInput2D.Ray(default, new Vector2F(1, 0), 10f) };
        var hits = new QueryHit2D[1];
        for (int i = 0; i < 20; i++) { queries.QueryBatch(inputs, hits); }
        Core.AllocMeter.AssertAllocatesNothing(() => { for (int i = 0; i < 100; i++) { queries.QueryBatch(inputs, hits); } }, "mixed query batches");
    }

    [Fact]
    public void ConvexPairPenetrationHasCorrectDepthAndClearMissOutputs()
    {
        var shape = Shape2D.CreateCircle(default, 1f);
        Assert.True(PhysicsWorld2D.ComputePenetration(shape, default, 0f, shape, new Vector2D(1.5, 0), 0f, out var direction, out float depth, out _));
        Assert.Equal(0.5f, depth, 3); Assert.Equal(1f, direction.X, 3);
        Assert.False(PhysicsWorld2D.ComputePenetration(shape, default, 0f, shape, new Vector2D(3, 0), 0f, out direction, out depth, out var point));
        Assert.Equal(default, direction); Assert.Equal(0f, depth); Assert.Equal(default, point);
    }

    [Fact]
    public void BodySweepRejectsForeignWorldEvenWhenIdsAndSlotsMatch()
    {
        var a = World(); var b = World();
        a.AddCircle(1f, default); var foreign = b.AddCircle(1f, default);
        Assert.Throws<InvalidHandleException>(() => a.SweepBody(foreign, new Vector2F(1, 0), out _));
    }

    [Fact]
    public void CallbackReentryRejectsBatchAndLeavesCallerOutputUnchanged()
    {
        var world = World(); world.AddCircle(1f, new Vector2D(5, 0), BodyType.Static);
        var query = new PhysicsQuery2D(world);
        var inputs = new[] { QueryInput2D.Ray(default, new Vector2F(1, 0), 10f) };
        var hits = new QueryHit2D[1];
        query.Filter = new QueryFilter2D { Predicate = _ => { query.QueryBatch(inputs, hits); return true; } };
        Assert.Throws<InvalidOperationException>(() => query.QueryBatch(inputs, hits));
        Assert.False(hits[0].IsHit);
        query.Filter = default;
        Assert.Equal(1, query.QueryBatch(inputs, hits));
    }

    [Fact]
    public void BodySweepIncludesOffCentreShapesAndJointExclusion()
    {
        var w = World(); var source = w.AddCircle(1f, default);
        w.CreateCircle(source, 0.5f, new Vector2F(2, 0));
        var target = w.AddCircle(1f, new Vector2D(5, 0), BodyType.Static);
        Assert.True(source.Sweep(new Vector2F(1, 0), out var hit, 10f));
        Assert.Equal(1.5f, hit.Distance, 2); Assert.Equal(target, hit.Body);
        w.AddDistance(source, target, default, new Vector2D(5, 0));
        Assert.False(source.Sweep(new Vector2F(1, 0), out _, 10f, true));
    }
}
