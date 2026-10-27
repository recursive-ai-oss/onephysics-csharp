using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

/// <summary>Joints together with the 2D world services: snapshot section combinations, filters, event order and debug geometry.</summary>
public class JointServicesTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D NewWorld(int id = 21)
        => new(new WorldOptions2D { Gravity = new Vector2F(0f, -9.81f), SleepEnabled = false, WorldId = id, ThreadCount = 1 });

    private static PhysicsWorld2D Make(bool joints, bool services)
    {
        var w = NewWorld();
        var floor = w.CreateBody(BodyType.Static, new Vector2D(0, -0.5));
        w.CreateBox(floor, 30f, 0.5f, default, 0f, 0f, new ShapeDef { Layer = services ? 2 : 0 });
        for (int i = 0; i < 6; i++)
        {
            var b = w.CreateBody(BodyType.Dynamic, new Vector2D(i * 1.2 - 3, 1.0 + 0.3 * i));
            w.CreateCircle(b, 0.5f, default, new ShapeDef { Layer = services ? 1 : 0 });
        }

        if (services)
        {
            w.IgnoreLayerCollision(1, 3);
            var s = w.CreateBody(BodyType.Static, new Vector2D(0, 1));
            w.CreateBox(s, 1f, 1f, default, 0f, 0f, new ShapeDef { IsSensor = true });
        }

        if (joints)
        {
            var pivot = w.AddCircle(0.1f, new Vector2D(5, 8), BodyType.Static);
            var bob = w.AddCircle(0.3f, new Vector2D(7, 8));
            w.AddHinge(pivot, bob, new Vector2D(5, 8));
            var c = w.AddBox(new Vector2F(1f, 1f), new Vector2D(9, 4));
            var d = w.AddBox(new Vector2F(1f, 1f), new Vector2D(10.1, 4));
            w.AddFixed(c, d, new Vector2D(9.5, 4));
        }

        return w;
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void EveryCombinationRoundTripsByteExactlyAndReplays(bool joints, bool services)
    {
        var w = Make(joints, services);
        for (int i = 0; i < 40; i++) { w.Step(Dt); }
        var snap = w.Snapshot();
        var fresh = NewWorld();
        fresh.Restore(snap);
        Assert.True(snap.AsSpan().SequenceEqual(fresh.Snapshot()));
        Assert.Equal(w.ComputeHash(), fresh.ComputeHash());
        for (int i = 0; i < 40; i++)
        {
            w.Step(Dt); fresh.Step(Dt);
            Assert.Equal(w.ComputeHash(), fresh.ComputeHash());
        }

        Assert.True(w.Snapshot().AsSpan().SequenceEqual(fresh.Snapshot()));
        Assert.Equal(joints ? 2 : 0, fresh.JointCount);
    }

    [Fact]
    public void FeatureFreeWorldsHaveTheSmallestSection()
    {
        // Each feature adds bytes; a world with neither writes neither block.
        int none = Make(false, false).Snapshot().Length, j = Make(true, false).Snapshot().Length, s = Make(false, true).Snapshot().Length;
        Assert.True(j > none);
        Assert.True(s > none);
    }

    [Fact]
    public void MalformedSectionCombinationsAreRejectedWithTypedErrorsAndLeaveTheWorldIntact()
    {
        foreach (var (joints, services) in new[] { (false, false), (true, false), (false, true), (true, true) })
        {
            var w = Make(joints, services);
            for (int i = 0; i < 5; i++) { w.Step(Dt); }
            var snap = w.Snapshot();
            ulong h = w.ComputeHash();

            // Rewrite the section version word (it precedes the first tunable, ContactHertz) to every other value.
            int at = FindVersionWord(snap, w);
            Assert.True(at > 0);
            uint have = BitConverter.ToUInt32(snap, at);
            Assert.Equal(joints ? 3u : services ? 2u : 1u, have);
            foreach (uint other in OtherVersions)
            {
                if (other == have) { continue; }
                var bad = (byte[])snap.Clone();
                BitConverter.TryWriteBytes(bad.AsSpan(at), other);
                var target = NewWorld();
                try
                {
                    target.Restore(bad);
                    // A different version can only be accepted when the bytes happen to be a valid other section; then it must not re-serialize as the same bytes.
                    Assert.False(bad.AsSpan().SequenceEqual(target.Snapshot()) && other != have);
                }
                catch (PhysicsException) { }
            }

            // Truncations at many points fail typed, and a failed restore keeps the world as it was.
            for (int cut = at; cut < snap.Length; cut += Math.Max(1, snap.Length / 97))
            {
                var target = Make(joints, services);
                for (int i = 0; i < 5; i++) { target.Step(Dt); }
                ulong before = target.ComputeHash();
                Assert.Throws<SnapshotFormatException>(() => target.Restore(snap.AsSpan(0, cut).ToArray()));
                Assert.Equal(before, target.ComputeHash());
            }

            Assert.Equal(h, w.ComputeHash());
        }
    }

    private static readonly uint[] OtherVersions = { 0, 1, 2, 3, 4 };

    private static int FindVersionWord(byte[] snap, PhysicsWorld2D w)
    {
        // The 2D section starts with the version, then ContactHertz as a float.
        var hertz = BitConverter.GetBytes(w.Options.ContactHertz);
        for (int i = 4; i + 8 <= snap.Length; i++)
        {
            if (snap[i + 4] == hertz[0] && snap[i + 5] == hertz[1] && snap[i + 6] == hertz[2] && snap[i + 7] == hertz[3] && snap[i + 1] == 0 && snap[i + 2] == 0 && snap[i + 3] == 0 && snap[i] >= 1 && snap[i] <= 3) { return i; }
        }

        return -1;
    }

    // ------------------------------------------------------------------ filters

    private static (Body A, Body B) Overlapping(PhysicsWorld2D w, int layerA, int layerB)
    {
        var a = w.CreateBody(BodyType.Dynamic, new Vector2D(0, 20));
        w.CreateCircle(a, 0.5f, default, new ShapeDef { Layer = layerA });
        var b = w.CreateBody(BodyType.Dynamic, new Vector2D(0.6, 20));
        w.CreateCircle(b, 0.5f, default, new ShapeDef { Layer = layerB });
        return (a, b);
    }

    private static int ContactsBetween(PhysicsWorld2D w, int steps)
    {
        int n = 0;
        for (int i = 0; i < steps; i++)
        {
            w.Step(Dt);
            n += w.GetEvents(EventKind.ContactBegin).Length;
        }

        return n;
    }

    [Fact]
    public void ACollisionNeedsEveryRulePermittingItInOrderOwnerJointLayerThenPredicates()
    {
        // Rule: shapes of different bodies; the layer matrix, group and mask; no joint without collide-connected between the bodies; then the pair predicate and contact filter.
        // 1. A plain joint blocks the pair even though the layers allow it.
        {
            var w = NewWorld();
            var (a, b) = Overlapping(w, 0, 1);
            w.AddHinge(a, b, new Vector2D(0.3, 20));
            Assert.Equal(0, ContactsBetween(w, 5));
        }

        // 2. collide-connected lets the layers decide: allowed, then ignored, then allowed again.
        {
            var w = NewWorld();
            var (a, b) = Overlapping(w, 0, 1);
            w.AddHinge(a, b, new Vector2D(0.3, 20), collideConnected: true);
            Assert.True(ContactsBetween(w, 3) > 0);

            var w2 = NewWorld();
            var (a2, b2) = Overlapping(w2, 0, 1);
            w2.AddHinge(a2, b2, new Vector2D(0.3, 20), collideConnected: true);
            w2.IgnoreLayerCollision(0, 1);
            Assert.Equal(0, ContactsBetween(w2, 5));
        }

        // 3. Turning a layer pair back on later does not defeat a plain joint.
        {
            var w = NewWorld();
            var (a, b) = Overlapping(w, 0, 1);
            w.IgnoreLayerCollision(0, 1);
            w.AddHinge(a, b, new Vector2D(0.3, 20));
            ContactsBetween(w, 3);
            w.IgnoreLayerCollision(0, 1, false);
            Assert.Equal(0, ContactsBetween(w, 5));
        }

        // 4. A pair predicate and a contact filter see only pairs the joint rule let through.
        {
            int asked = 0;
            var o = new WorldOptions2D { Gravity = new Vector2F(0f, -9.81f), WorldId = 22, ThreadCount = 1 };
            o.PairFilter = (x, y) => { asked++; return true; };
            var w = new PhysicsWorld2D(o);
            var (a, b) = Overlapping(w, 0, 0);
            w.AddHinge(a, b, new Vector2D(0.3, 20));
            w.IgnoreLayerCollision(0, 0, false);
            ContactsBetween(w, 5);
            Assert.Equal(0, asked);
        }
    }

    // ------------------------------------------------------------------ events

    [Fact]
    public void ABreakIsDeliveredInTheSortedEventOrderBetweenContactAndSleepEvents()
    {
        var w = NewWorld(23);
        var floor = w.CreateBody(BodyType.Static, new Vector2D(0, -0.5));
        w.CreateBox(floor, 30f, 0.5f, default, 0f, 0f);
        var ball = w.CreateBody(BodyType.Dynamic, new Vector2D(0, 0.45));
        w.CreateCircle(ball, 0.5f);
        var anchor = w.AddCircle(0.1f, new Vector2D(10, 10), BodyType.Static);
        var weight = w.AddBox(new Vector2F(1f, 1f), new Vector2D(10, 9));
        var weld = w.AddFixed(anchor, weight, new Vector2D(10, 9.5));
        weld.BreakForce = 0.1f;
        var log = new List<string>();
        ball.OnCollisionEnter += c => log.Add("begin");
        ball.OnCollisionStay += c => log.Add("stay");
        weld.OnJointBreak += b => log.Add("break");
        w.Step(Dt);

        var kinds = new List<EventKind>();
        foreach (var e in w.Events) { kinds.Add(e.Kind); }
        Assert.Contains(EventKind.JointBreak, kinds);
        Assert.Contains(EventKind.ContactBegin, kinds);
        Assert.True(kinds.IndexOf(EventKind.ContactBegin) < kinds.IndexOf(EventKind.JointBreak));
        Assert.Equal("begin,break", string.Join(",", log));
        Assert.Equal(1, w.GetEvents(EventKind.JointBreak).Length);
        w.Step(Dt);
        Assert.Equal(0, w.GetEvents(EventKind.JointBreak).Length);
    }

    // ------------------------------------------------------------------ debug geometry

    [Fact]
    public void DebugGeometryListsJointsWithAnchorsAndAxes()
    {
        var w = NewWorld(24);
        var ground = w.AddBox(new Vector2F(20f, 1f), new Vector2D(0, -1), type: BodyType.Static);
        var a = w.AddBox(new Vector2F(1f, 1f), new Vector2D(2, 3));
        var b = w.AddBox(new Vector2F(1f, 1f), new Vector2D(4, 3));
        var hinge = w.AddHinge(a, b, new Vector2D(3, 3));
        var c = w.AddBox(new Vector2F(1f, 1f), new Vector2D(2, 6));
        var slider = w.AddSlider(ground, c, new Vector2D(2, 6), new Vector2F(0f, 1f));
        var d = w.AddBox(new Vector2F(1f, 1f), new Vector2D(8, 3));
        var mouse = w.AddTarget(ground, d, new Vector2D(8.5, 3.5));
        var g = new DebugGeometry();
        w.GetDebugGeometry(DebugContent.Joints, g);
        Assert.Equal(3, g.Joints.Length);
        Assert.Equal(0, g.Shapes.Length);

        var jh = g.Joints[0];
        Assert.Equal(hinge.RawHandle, jh.Joint);
        Assert.Equal(a.RawHandle, jh.BodyA);
        Assert.Equal(3.0, jh.AnchorA.X, 5); Assert.Equal(3.0, jh.AnchorA.Y, 5);
        Assert.Equal(jh.AnchorA.X, jh.AnchorB.X, 5);
        Assert.Equal(0.0, jh.Axis.X); Assert.Equal(0.0, jh.Axis.Y);

        var js = g.Joints[1];
        Assert.Equal(slider.RawHandle, js.Joint);
        Assert.Equal(0.0, js.Axis.X, 6); Assert.Equal(1.0, js.Axis.Y, 6);

        var jm = g.Joints[2];
        Assert.Equal(mouse.RawHandle, jm.Joint);
        Assert.Equal(8.5, jm.AnchorA.X, 6); Assert.Equal(3.5, jm.AnchorA.Y, 6);

        // Destroyed joints disappear, and the buffer is reused.
        hinge.Destroy();
        w.GetDebugGeometry(DebugContent.All, g);
        Assert.Equal(2, g.Joints.Length);
        w.GetDebugGeometry(DebugContent.Shapes, g);
        Assert.Equal(0, g.Joints.Length);
    }
}
