using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

public class ExtendedQueryTests
{
    private static PhysicsWorld3D World(int threads = 1) => new(new WorldOptions3D { ThreadCount = threads, WorldId = 73, Gravity = default });

    [Fact]
    public void SelectedQueriesIgnoreNearerWorldShapesAndRejectStaleHandles()
    {
        var w = World();
        var near = w.AddSphere(1f, new Vector3D(5, 0, 0), BodyType.Static);
        var far = w.AddSphere(1f, new Vector3D(10, 0, 0), BodyType.Static);
        var shapes = far.GetShapes();
        Assert.True(w.Raycast(new Vector3D(0, 0, 0), new Vector3F(1, 0, 0), out var hit));
        Assert.Equal(near, hit.Body);
        Assert.True(far.Raycast(new Vector3D(0, 0, 0), new Vector3F(1, 0, 0), out hit));
        Assert.Equal(9f, hit.Distance, 3);
        Assert.True(shapes[0].ClosestPoint(new Vector3D(0, 0, 0), out hit));
        Assert.Equal(9f, hit.Distance, 3);
        var mover = ConvexShape3D.Sphere(1f);
        Assert.True(far.ShapeCast(mover, new Vector3D(0, 0, 0), default, new Vector3F(1, 0, 0), out hit, 20f));
        Assert.Equal(8f, hit.Distance, 2);
        Assert.True(shapes[0].OverlapShape(mover, new Vector3D(10, 0, 0), default, out hit));
        Assert.False(shapes[0].Raycast(new Vector3D(0, 0, 0), new Vector3F(1, 0, 0), out _, 20f, QueryFilter3D.Layers(0)));
        far.Destroy();
        Assert.Throws<StaleHandleException>(() => far.Raycast(new Vector3D(0, 0, 0), new Vector3F(1, 0, 0), out _));
        Assert.Throws<StaleHandleException>(() => shapes[0].ClosestPoint(new Vector3D(0, 0, 0), out _));
    }

    [Fact]
    public void BroadphaseUsesInclusiveBoxesAndStableTruncation()
    {
        var w = World();
        var a = w.AddSphere(1f, new Vector3D(5, 0, 0), BodyType.Static);
        w.AddSphere(1f, new Vector3D(5, 0, 0), BodyType.Static);
        var results = new Shape[1];
        Assert.Equal(1, w.QueryAabb(new Vector3D(3, -2, -2), new Vector3D(7, 2, 2), results));
        Assert.Equal(a, results[0].Body);
        Assert.Equal(0, w.QueryAabb(new Vector3D(0, 0, 0), new Vector3D(0, 0, 0), results));
        Assert.Equal(0, w.QueryAabb(new Vector3D(3, -2, -2), new Vector3D(7, 2, 2), results, QueryFilter3D.Layers(0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.QueryAabb(new Vector3D(5, 0, 0), new Vector3D(0, 0, 0), results));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.QueryAabb(new Vector3D(double.NaN, 0, 0), new Vector3D(0, 0, 0), results));
    }

    [Fact]
    public void MixedBatchMatchesScalarAtOneAndEightThreads()
    {
        var one = World();
        var eight = World(8);
        one.AddSphere(1f, new Vector3D(5, 0, 0), BodyType.Static);
        eight.AddSphere(1f, new Vector3D(5, 0, 0), BodyType.Static);
        var shape = ConvexShape3D.Sphere(1f);
        var inputs = new QueryInput3D[513];
        for (int i = 0; i < inputs.Length; i++)
        {
            inputs[i] = (i % 4) switch
            {
                0 => QueryInput3D.Ray(new Vector3D(0, 0, 0), new Vector3F(1, 0, 0), 20f),
                1 => QueryInput3D.Cast(shape, new Vector3D(0, 0, 0), default, new Vector3F(1, 0, 0), 20f),
                2 => QueryInput3D.Overlap(shape, new Vector3D(5, 0, 0), default),
                _ => QueryInput3D.Closest(new Vector3D(0, 0, 0), 20f)
            };
        }
        var a = new QueryHit3D[inputs.Length];
        var b = new QueryHit3D[inputs.Length];
        Assert.Equal(inputs.Length, one.QueryBatch(inputs, a));
        Assert.Equal(inputs.Length, eight.QueryBatch(inputs, b));
        for (int i = 0; i < inputs.Length; i++)
        {
            Assert.Equal(a[i].Distance, b[i].Distance);
            Assert.Equal(a[i].Point, b[i].Point);
            Assert.Equal(a[i].Normal, b[i].Normal);
            Assert.Equal(a[i].Shape.RawHandle.Index, b[i].Shape.RawHandle.Index);
            QueryHit3D expected;
            switch (i % 4)
            {
                case 0: Assert.True(one.Raycast(new Vector3D(0, 0, 0), new Vector3F(1, 0, 0), out expected, 20f)); break;
                case 1: Assert.True(one.ShapeCast(shape, new Vector3D(0, 0, 0), default, new Vector3F(1, 0, 0), out expected, 20f)); break;
                case 2: Assert.True(one.OverlapShape(shape, new Vector3D(5, 0, 0), default, out expected)); break;
                default: Assert.True(one.ClosestPoint(new Vector3D(0, 0, 0), out expected, 20f)); break;
            }
            Assert.Equal(expected.Distance, a[i].Distance);
            Assert.Equal(expected.Point, a[i].Point);
        }
        var counts = new int[inputs.Length];
        var all = new QueryHit3D[inputs.Length * 2];
        eight.QueryAllBatch(inputs, all, counts, 2);
        var any = new bool[inputs.Length];
        eight.QueryAnyBatch(inputs, any);
        for (int i = 0; i < inputs.Length; i++) { Assert.Equal(1, counts[i]); Assert.True(any[i]); Assert.True(all[2 * i].IsHit); Assert.False(all[2 * i + 1].IsHit); }
        eight.QueryAllBatch(inputs, Array.Empty<QueryHit3D>(), counts, 0);
        Assert.All(counts, count => Assert.Equal(0, count));
    }

    [Fact]
    public void BatchValidatesBeforeWritingAndDoesNotChangeWorldHash()
    {
        var w = World();
        w.AddSphere(1f, new Vector3D(5, 0, 0), BodyType.Static);
        ulong hash = w.ComputeHash();
        var inputs = new[] { QueryInput3D.Ray(new Vector3D(0, 0, 0), new Vector3F(1, 0, 0)), default };
        var hits = new QueryHit3D[2];
        Assert.Throws<ArgumentException>(() => w.QueryBatch(inputs, hits));
        Assert.False(hits[0].IsHit);
        Assert.Throws<ArgumentException>(() => w.QueryBatch(inputs, Array.Empty<QueryHit3D>()));
        Assert.Throws<ArgumentOutOfRangeException>(() => QueryInput3D.Ray(new Vector3D(0, 0, 0), new Vector3F(1, 0, 0), -1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => QueryInput3D.Ray(new Vector3D(0, 0, 0), default));
        w.QueryBatch(inputs.AsSpan(0, 1), hits);
        Assert.Equal(hash, w.ComputeHash());
    }

    [Fact]
    public void PenetrationDepthAndNormalSeparateSpheres()
    {
        var w = World();
        w.AddSphere(1f, default, BodyType.Static);
        var shape = ConvexShape3D.Sphere(1f);
        Assert.True(w.Penetration(shape, new Vector3D(1.5, 0, 0), default, out var hit));
        Assert.Equal(0.5f, hit.Distance, 3);
        Assert.Equal(1f, hit.Normal.X, 3);
        Assert.False(w.Penetration(shape, new Vector3D(2.1, 0, 0), default, out _));
        var all = new QueryHit3D[2];
        Assert.Equal(1, w.PenetrationAll(shape, new Vector3D(1.5, 0, 0), default, all));
        Assert.Equal(hit.Distance, all[0].Distance);
        var batch = new[] { QueryInput3D.Penetration(shape, new Vector3D(1.5, 0, 0), default) };
        Assert.Equal(1, w.QueryBatch(batch, all));
        Assert.Equal(hit.Distance, all[0].Distance);
    }

    [Fact]
    public void WholeBodySweepExcludesSelfAndLeavesWorldUnchanged()
    {
        var w = World();
        var source = w.AddSphere(1f, default);
        var target = w.AddSphere(1f, new Vector3D(5, 0, 0), BodyType.Static);
        ulong hash = w.ComputeHash();
        Assert.True(source.Sweep(new Vector3F(1, 0, 0), out var hit, 10f));
        Assert.Equal(target, hit.Body);
        Assert.Equal(3f, hit.Distance, 3);
        Assert.Equal(hash, w.ComputeHash());
        Assert.False(source.Sweep(new Vector3F(-1, 0, 0), out _, 10f));
        Assert.Throws<ArgumentOutOfRangeException>(() => source.Sweep(default, out _));
    }

    [Fact]
    public void MixedBatchBuffersAreAllocationFreeAfterWarmup()
    {
        var w = World();
        w.AddSphere(1f, new Vector3D(5, 0, 0), BodyType.Static);
        var queries = new PhysicsQuery3D(w);
        var inputs = new[] { QueryInput3D.Ray(default, new Vector3F(1, 0, 0), 10f) };
        var hits = new QueryHit3D[1];
        for (int i = 0; i < 20; i++) { queries.QueryBatch(inputs, hits); }
        Core.AllocMeter.AssertAllocatesNothing(() => { for (int i = 0; i < 100; i++) { queries.QueryBatch(inputs, hits); } }, "mixed query batches");
    }

    [Fact]
    public void ConvexPairPenetrationHasCorrectDepthAndClearMissOutputs()
    {
        var shape = ConvexShape3D.Sphere(1f);
        Assert.True(PhysicsWorld3D.ComputePenetration(shape, default, default, shape, new Vector3D(1.5, 0, 0), default, out var direction, out float depth, out _));
        Assert.Equal(0.5f, depth, 3); Assert.Equal(1f, direction.X, 3);
        Assert.False(PhysicsWorld3D.ComputePenetration(shape, default, default, shape, new Vector3D(3, 0, 0), default, out direction, out depth, out var point));
        Assert.Equal(default, direction); Assert.Equal(0f, depth); Assert.Equal(default, point);
    }

    [Fact]
    public void BodySweepRejectsForeignWorldEvenWhenIdsAndSlotsMatch()
    {
        var a = World(); var b = World();
        a.AddSphere(1f, default); var foreign = b.AddSphere(1f, default);
        Assert.Throws<InvalidHandleException>(() => a.SweepBody(foreign, new Vector3F(1, 0, 0), out _));
    }

    [Fact]
    public void CallbackReentryRejectsBatchAndLeavesCallerOutputUnchanged()
    {
        var world = World(); world.AddSphere(1f, new Vector3D(5, 0, 0), BodyType.Static);
        var query = new PhysicsQuery3D(world);
        var inputs = new[] { QueryInput3D.Ray(default, new Vector3F(1, 0, 0), 10f) };
        var hits = new QueryHit3D[1];
        query.Filter = new QueryFilter3D { Predicate = _ => { query.QueryBatch(inputs, hits); return true; } };
        Assert.Throws<InvalidOperationException>(() => query.QueryBatch(inputs, hits));
        Assert.False(hits[0].IsHit);
        query.Filter = default;
        Assert.Equal(1, query.QueryBatch(inputs, hits));
    }

    [Fact]
    public void BodySweepIncludesOffCentreShapesAndJointExclusion()
    {
        var w = World(); var source = w.AddSphere(1f, default);
        w.CreateCompound(source, new[] { new CompoundPart(ShapeSpec3D.Sphere(0.5f), new Vector3F(2, 0, 0)) });
        var target = w.AddSphere(1f, new Vector3D(5, 0, 0), BodyType.Static);
        Assert.True(source.Sweep(new Vector3F(1, 0, 0), out var hit, 10f));
        Assert.Equal(1.5f, hit.Distance, 2); Assert.Equal(target, hit.Body);
        w.AddDistance(source, target, default, new Vector3D(5, 0, 0));
        Assert.False(source.Sweep(new Vector3F(1, 0, 0), out _, 10f, true));
    }
}
