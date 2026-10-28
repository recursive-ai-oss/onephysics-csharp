using System;
using System.Collections.Generic;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Core.Reference;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Core;

internal static class Util
{
    public static readonly double[] Origin3 = { 0, 0, 0 };

    public static ulong EventDigest(ReadOnlySpan<PhysicsEvent> events)
    {
        ulong h = 1469598103934665603UL;
        foreach (var e in events)
        {
            foreach (long v in new long[] { (long)e.Kind, e.BodyA.Index, e.BodyA.Generation, e.BodyB.Index, e.BodyB.Generation, e.ShapeA.Index, e.ShapeB.Index, e.SubShapeA, e.SubShapeB, e.Joint.Index, BitConverter.SingleToInt32Bits(e.Value) })
            {
                h = (h ^ (ulong)v) * 1099511628211UL;
            }
        }

        return h;
    }

    /// <summary>Steps and returns per-step (world hash, event digest, event count).</summary>
    public static List<(ulong Hash, ulong Events, int Count)> Run(ParticleScene scene, int steps, float dt = 1f / 60f)
    {
        var list = new List<(ulong, ulong, int)>();
        for (int i = 0; i < steps; i++)
        {
            scene.Step(dt);
            list.Add((scene.World.ComputeHash(), EventDigest(scene.World.Events.Delivered), scene.World.Events.Delivered.Length));
        }

        return list;
    }

    public static int CountDelivered(WorldCore w, EventKind kind)
    {
        int n = 0;
        foreach (var e in w.Events.Delivered) { if (e.Kind == kind) { n++; } }
        return n;
    }
}

public class HandleTests
{
    private static readonly int[] ReusedOrder = { 1, 3, 4, 6 };

    [Fact]
    public void FreedSlotsAreReusedLowestFirstAndGenerationAdvances()
    {
        var w = new WorldCore();
        var h = new BodyHandle[6];
        for (int i = 0; i < h.Length; i++) { h[i] = w.CreateBody(BodyType.Dynamic, Util.Origin3); }
        w.DestroyBody(h[4]);
        w.DestroyBody(h[1]);
        w.DestroyBody(h[3]);
        var a = w.CreateBody(BodyType.Dynamic, Util.Origin3);
        var b = w.CreateBody(BodyType.Dynamic, Util.Origin3);
        var c = w.CreateBody(BodyType.Dynamic, Util.Origin3);
        var d = w.CreateBody(BodyType.Dynamic, Util.Origin3);
        Assert.Equal(ReusedOrder, new[] { a.Index, b.Index, c.Index, d.Index });
        Assert.Equal(2, a.Generation);
        Assert.Equal(1, d.Generation);
    }

    [Fact]
    public void StaleForeignAndNullHandlesFailWithTypedErrors()
    {
        var w1 = new WorldCore();
        var w2 = new WorldCore();
        var b = w1.CreateBody(BodyType.Dynamic, Util.Origin3);
        Assert.Equal(b.Index, w1.Resolve(b));
        Assert.Throws<ForeignHandleException>(() => w2.Resolve(b));
        Assert.Throws<InvalidHandleException>(() => w1.Resolve(default(BodyHandle)));
        w1.DestroyBody(b);
        var e1 = Assert.Throws<StaleHandleException>(() => w1.Resolve(b));
        Assert.Contains("stale", e1.Message, StringComparison.Ordinal);
        var again = w1.CreateBody(BodyType.Dynamic, Util.Origin3);
        Assert.Equal(b.Index, again.Index);
        Assert.Throws<StaleHandleException>(() => w1.Resolve(b));
        Assert.Throws<StaleHandleException>(() => w1.DestroyBody(b));
        Assert.Equal(again.Index, w1.Resolve(again));
    }

    [Fact]
    public void ShapesAndJointsHaveTheSameRules()
    {
        var w = new WorldCore();
        var a = w.CreateBody(BodyType.Dynamic, Util.Origin3);
        var b = w.CreateBody(BodyType.Dynamic, new double[] { 1, 0, 0 });
        var s = w.CreateShape(a);
        int c = w.AddConstraint(new ConstraintDesc { Kind = ConstraintKind.Joint, BodyA = a, BodyB = b });
        var j = w.HandleOfJoint(c);
        Assert.Equal(c, w.Resolve(j));
        w.DestroyJoint(j);
        Assert.Throws<StaleHandleException>(() => w.Resolve(j));
        w.DestroyBody(a);
        Assert.Throws<StaleHandleException>(() => w.Resolve(s));
        int contact = w.AddConstraint(new ConstraintDesc { Kind = ConstraintKind.Contact, BodyA = w.HandleOfBody(b.Index), BodyB = w.CreateBody(BodyType.Static, Util.Origin3) });
        Assert.Throws<InvalidHandleException>(() => w.Resolve(w.HandleOfJoint(contact)));
    }

    [Fact]
    public void HostileBodyInputsAreRejectedWithNamedErrors()
    {
        var w = new WorldCore();
        var far = Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateBody(BodyType.Dynamic, new double[] { 0, 3e6, 0 }));
        Assert.Contains("3000000", far.Message, StringComparison.Ordinal);
        Assert.Contains("2^21", far.Message, StringComparison.Ordinal);
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateBody(BodyType.Dynamic, new double[] { double.NaN, 0, 0 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateBody(BodyType.Dynamic, Util.Origin3, inverseMass: 0f));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateBody(BodyType.Dynamic, Util.Origin3, inverseMass: -1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateBody(BodyType.Dynamic, Util.Origin3, linearVelocity: new[] { float.NaN, 0f, 0f }));
        Assert.Throws<ArgumentException>(() => w.CreateBody(BodyType.Dynamic, new double[] { 0, 0 }));
        Assert.Throws<ArgumentException>(() => w.CreateBody(BodyType.Dynamic, Util.Origin3, rotation: new[] { 0f, 0f, 0f, 2f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Step(new ParticleScene().Stages, 0f));
        Assert.Throws<ArgumentOutOfRangeException>(() => new WorldCore(new WorldOptions { Substeps = 0 }));
        Assert.Throws<InvalidHandleException>(() => w.AddConstraint(new ConstraintDesc { Kind = ConstraintKind.Joint, BodyA = w.CreateBody(BodyType.Static, Util.Origin3), BodyB = default }));
    }
}

public class SchedulerTests
{
    private static readonly int[] ThreadCounts = { 1, 2, 8, 32 };

    [Fact]
    public void WorldHashesAndEventsAreIdenticalAcrossThreadCounts()
    {
        List<(ulong, ulong, int)>? reference = null;
        foreach (int threads in ThreadCounts)
        {
            var scene = ParticleScene.CreatePile(1500, 7, threads, worldId: 5);
            scene.ReportPersist = true;
            scene.World.Options.Events = EventMask.All;
            var run = Util.Run(scene, 90);
            if (reference == null) { reference = run; }
            else { Assert.Equal(reference, run); }
        }
    }

    [Fact]
    public void ColorsAreIdenticalAcrossThreadCountsAndNeverShareADynamicBody()
    {
        byte[]? reference = null;
        foreach (int threads in ThreadCounts)
        {
            var scene = ParticleScene.CreatePile(1200, 11, threads, worldId: 5);
            Util.Run(scene, 40);
            var w = scene.World;
            var colors = w.Constraints.Color.ToArray();
            if (reference == null) { reference = colors; } else { Assert.Equal(reference, colors); }

            var seen = new HashSet<(int Body, int Color)>();
            for (int c = 0; c < w.Constraints.HighWater; c++)
            {
                if (!w.Constraints.IsLive(c) || colors[c] >= ConstraintColoring.ColorCount) { continue; }
                foreach (int body in new[] { w.Constraints.BodyA[c], w.Constraints.BodyB[c] })
                {
                    if (w.Bodies.IsDynamic(body)) { Assert.True(seen.Add((body, colors[c])), "two constraints of one color share body " + body); }
                }
            }
        }
    }

    [Fact]
    public void StaticAndKinematicBodiesDoNotJoinColors()
    {
        var scene = new ParticleScene();
        var w = scene.World;
        var ground = scene.AddStaticParticle(0, 0.5, 0);
        var ground2 = scene.AddStaticParticle(0, 0.5, 1);
        var p = scene.AddParticle(0, 0.6, 0);
        var q = scene.AddParticle(0, 0.7, 1);
        w.AddConstraint(new ConstraintDesc { Kind = ConstraintKind.Joint, BodyA = ground, BodyB = p });
        w.AddConstraint(new ConstraintDesc { Kind = ConstraintKind.Joint, BodyA = ground2, BodyB = q });
        w.AddConstraint(new ConstraintDesc { Kind = ConstraintKind.Joint, BodyA = ground, BodyB = ground2 });
        // Both joints touch a static body, which may be in both; they use the same preferred color because no dynamic body is shared.
        Assert.Equal(w.Constraints.Color[0], w.Constraints.Color[1]);
        Assert.Equal(ConstraintStore.Uncolored, w.Constraints.Color[2]);
        Assert.Equal(0UL, w.Coloring.ColorMask(w.Resolve(ground)));
    }

    [Fact]
    public void ConstraintsBeyondTheColorLimitOverflowAndStillSolveDeterministically()
    {
        ulong? reference = null;
        foreach (int threads in ThreadCounts)
        {
            var scene = new ParticleScene(new WorldOptions { ThreadCount = threads, WorldId = 3 });
            var w = scene.World;
            var hub = scene.AddParticle(0, 5, 0);
            for (int i = 0; i < 40; i++)
            {
                var spoke = scene.AddParticle(Math.Cos(i * 0.157) * 1.0, 5, Math.Sin(i * 0.157) * 1.0);
                scene.AddDistanceJoint(hub, spoke);
            }

            Util.Run(scene, 5);
            Assert.True(w.Coloring.Overflow.Length > 0, "expected overflow constraints");
            Assert.Equal(40 - ConstraintColoring.ColorCount, CountOverflow(w));
            Util.Run(scene, 30);
            ulong h = w.ComputeHash();
            if (reference == null) { reference = h; } else { Assert.Equal(reference, h); }
        }
    }

    private static int CountOverflow(WorldCore w)
    {
        int n = 0;
        foreach (byte c in w.Constraints.Color) { if (c == ConstraintStore.Overflow) { n++; } }
        return n;
    }

    [Fact]
    public void BatchesPartitionEachColorInAscendingOrder()
    {
        var scene = ParticleScene.CreatePile(800, 3);
        Util.Run(scene, 20);
        var col = scene.World.Coloring;
        int total = col.Overflow.Length;
        for (int c = 0; c < ConstraintColoring.ColorCount; c++)
        {
            var list = col.Color(c);
            total += list.Length;
            for (int i = 1; i < list.Length; i++) { Assert.True(list[i] > list[i - 1]); }
        }

        Assert.Equal(col.ActiveCount, total);
    }
}

public class IslandTests
{
    private static readonly int[] AllSix = { 0, 1, 2, 3, 4, 5 };
    private static readonly int[] FirstThree = { 0, 1, 2 };
    private static readonly int[] LastThree = { 3, 4, 5 };

    private static ParticleScene Chain(int n, out BodyHandle[] bodies, out JointHandle[] joints)
    {
        var scene = new ParticleScene(new WorldOptions { WorldId = 9, TimeToSleep = 0.2f });
        bodies = new BodyHandle[n];
        joints = new JointHandle[n - 1];
        for (int i = 0; i < n; i++) { bodies[i] = scene.AddParticle(i * 0.6, 0.25, 0); }
        for (int i = 0; i + 1 < n; i++) { joints[i] = scene.AddDistanceJoint(bodies[i], bodies[i + 1]); }
        return scene;
    }

    [Fact]
    public void JointsMergeIslandsAndRemovingOneSplitsItBeforeSleep()
    {
        var scene = Chain(6, out var bodies, out var joints);
        var w = scene.World;
        Assert.Equal(1, w.Islands.Count);
        Assert.Equal(AllSix, w.Islands.Bodies(0).ToArray());
        w.DestroyJoint(joints[2]);
        Assert.Equal(1, w.Islands.Count); // removal alone does not split
        Assert.Equal(1, w.Islands.RemovedConstraints(0));
        for (int i = 0; i < 120 && w.Islands.Count == 1; i++) { scene.Step(1f / 60f); }
        Assert.Equal(2, w.Islands.Count);
        Assert.Equal(FirstThree, w.Islands.Bodies(0).ToArray());
        Assert.Equal(LastThree, w.Islands.Bodies(1).ToArray());
        Assert.Equal(0, w.Islands.RemovedConstraints(0));
        for (int i = 0; i < 120; i++) { scene.Step(1f / 60f); }
        Assert.True(w.Islands.IsAsleep(0) && w.Islands.IsAsleep(1));
        Assert.False(w.Bodies.IsAwake(bodies[0].Index));
    }

    [Fact]
    public void SleepingIslandsWakeWhenAnAwakeBodyTouchesThemAndWakeSpreadsToTheWholeIsland()
    {
        var scene = Chain(4, out var bodies, out _);
        var w = scene.World;
        for (int i = 0; i < 200; i++) { scene.Step(1f / 60f); }
        Assert.True(w.Islands.IsAsleep(0));
        var intruder = scene.AddParticle(0.0, 0.25, 0.4);
        var extra = scene.AddParticle(0.0, 3, 5);
        w.AddConstraint(new ConstraintDesc { Kind = ConstraintKind.Joint, BodyA = intruder, BodyB = extra });
        Assert.False(w.Islands.IsAsleep(w.Bodies.Island[intruder.Index]));
        // A joint between the sleeping chain and the awake pair merges everything into one awake island.
        w.AddConstraint(new ConstraintDesc { Kind = ConstraintKind.Joint, BodyA = bodies[2], BodyB = intruder });
        Assert.Equal(1, w.Islands.Count);
        Assert.True(w.Bodies.IsAwake(bodies[0].Index) && w.Bodies.IsAwake(bodies[3].Index));
        scene.Step(1f / 60f);
        Assert.Equal(4, Util.CountDelivered(w, EventKind.Wake));
    }

    [Fact]
    public void ExplicitWakeWakesTheWholeIsland()
    {
        var scene = Chain(3, out var bodies, out _);
        var w = scene.World;
        for (int i = 0; i < 200; i++) { scene.Step(1f / 60f); }
        Assert.False(w.Bodies.IsAwake(bodies[2].Index));
        w.Wake(bodies[0]);
        Assert.True(w.Bodies.IsAwake(bodies[2].Index));
        Assert.Equal(0f, w.Bodies.SleepTime[bodies[2].Index]);
    }

    [Fact]
    public void IslandStateIsTheSameForEveryThreadCount()
    {
        ulong? reference = null;
        foreach (int threads in new[] { 1, 2, 8, 32 })
        {
            var scene = ParticleScene.CreatePile(900, 21, threads, worldId: 2);
            var w = scene.World;
            Util.Run(scene, 400);
            ulong h = w.ComputeHash();
            Assert.True(w.Islands.Count > 0);
            if (reference == null) { reference = h; } else { Assert.Equal(reference, h); }
        }
    }
}

public class EventTests
{
    [Fact]
    public void ContactBeginAndEndFireWithShapesAndSortedOrder()
    {
        var scene = new ParticleScene(new WorldOptions { WorldId = 4 });
        var w = scene.World;
        var a = scene.AddParticle(0, 3, 0);
        var b = scene.AddParticle(0.4, 3, 0);
        var c = scene.AddParticle(0.2, 3, 0.3);
        scene.Step(1f / 60f);
        var begins = new List<PhysicsEvent>();
        foreach (var e in w.Events.Delivered) { if (e.Kind == EventKind.ContactBegin) { begins.Add(e); } }
        Assert.Equal(3, begins.Count);
        for (int i = 1; i < begins.Count; i++) { Assert.True(PhysicsEvent.Compare(begins[i - 1], begins[i]) < 0); }
        Assert.Equal(a.Index, begins[0].BodyA.Index);
        Assert.False(begins[0].ShapeA.IsNull);
        // Pull b away: the contact ends after the separation passes the removal distance.
        w.Bodies.PositionOf(b.Index)[0] = 5.0;
        scene.Step(1f / 60f);
        Assert.True(Util.CountDelivered(w, EventKind.ContactEnd) >= 1);
        Assert.NotEqual(0, c.Index);
    }

    [Fact]
    public void SensorsJointBreaksAndDestroyEventsAreDelivered()
    {
        var scene = new ParticleScene(new WorldOptions { WorldId = 4, Events = EventMask.All });
        var w = scene.World;
        var anchor = scene.AddStaticParticle(0, 6, 0);
        var weight = scene.AddParticle(0, 5, 0);
        var sa = w.CreateShape(anchor);
        var sb = w.CreateShape(weight);
        w.AddConstraint(new ConstraintDesc { Kind = ConstraintKind.Sensor, BodyA = anchor, BodyB = weight, ShapeA = sa, ShapeB = sb, SubShapeA = 3, SubShapeB = 7 });
        // A hanging weight loads the joint with its weight each substep, far above this threshold.
        var joint = scene.AddDistanceJoint(anchor, weight, breakImpulse: 1e-4f);
        scene.Step(1f / 60f);
        Assert.Equal(1, Util.CountDelivered(w, EventKind.SensorEnter));
        Assert.Equal(1, Util.CountDelivered(w, EventKind.JointBreak));
        PhysicsEvent brk = default;
        PhysicsEvent enter = default;
        foreach (var e in w.Events.Delivered)
        {
            if (e.Kind == EventKind.JointBreak) { brk = e; }
            if (e.Kind == EventKind.SensorEnter) { enter = e; }
        }

        Assert.Equal(joint.Index, brk.Joint.Index);
        Assert.True(brk.Value > 1e-4f);
        Assert.Equal(3, enter.SubShapeA);
        Assert.Equal(7, enter.SubShapeB);
        Assert.Equal(sb, enter.ShapeB);
        Assert.Throws<StaleHandleException>(() => w.Resolve(joint));
        // Destroying a body queues the sensor exit, which arrives with the next step.
        w.DestroyBody(weight);
        Assert.Equal(1, w.Events.PendingCount);
        scene.Step(1f / 60f);
        Assert.Equal(1, Util.CountDelivered(w, EventKind.SensorExit));
        Assert.Equal(0, w.Events.PendingCount);
    }
}

public class OriginTests
{
    [Fact]
    public void ShiftOriginMovesPositionsKeepsHostCoordinatesAndChangesTheHash()
    {
        var scene = ParticleScene.CreatePile(50, 1, worldId: 1);
        var w = scene.World;
        scene.Step(1f / 60f);
        double x0 = w.Bodies.PositionOf(7)[0];
        ulong h0 = w.ComputeHash();
        w.ShiftOrigin(new double[] { 1048576.0, 0.5, -3.0 });
        Assert.Equal(x0 - 1048576.0, w.Bodies.PositionOf(7)[0]);
        Assert.Equal(1048576.0, w.Origin[0]);
        Assert.NotEqual(h0, w.ComputeHash());
        var restored = new WorldCore(new WorldOptions { WorldId = 1 });
        restored.Restore(w.Snapshot());
        Assert.Equal(1048576.0, restored.Origin[0]);
        Assert.Equal(w.ComputeHash(), restored.ComputeHash());
    }

    [Fact]
    public void ShiftingAWorldWithADestroyedBodyStillSnapshotsAndRestores()
    {
        var scene = ParticleScene.CreatePile(50, 1, worldId: 1);
        var w = scene.World;
        scene.Step(1f / 60f);
        w.DestroyBody(w.HandleOfBody(7));
        w.ShiftOrigin(new double[] { 16.0, 0.5, -3.0 });
        Assert.Equal(0.0, w.Bodies.PositionOf(7)[0]);
        var restored = new WorldCore(new WorldOptions { WorldId = 1 });
        restored.Restore(w.Snapshot());
        Assert.Equal(w.ComputeHash(), restored.ComputeHash());
    }

    [Fact]
    public void OffsetsMustBeFiniteGridMultiplesWithinBounds()
    {
        var w = new WorldCore();
        Assert.Throws<ArgumentException>(() => w.ShiftOrigin(new[] { 0.1, 0, 0 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.ShiftOrigin(new[] { double.NaN, 0, 0 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.ShiftOrigin(new[] { 1e9, 0, 0 }));
        Assert.Throws<ArgumentException>(() => w.ShiftOrigin(new[] { 1.0, 0 }));
    }
}
