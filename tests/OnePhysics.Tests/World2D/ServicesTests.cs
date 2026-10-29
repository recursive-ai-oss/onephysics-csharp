using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

/// <summary>Layers and filters, sensors, events and debug geometry of the 2D world.</summary>
public class ServicesTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D NewWorld(int threads = 1, WorldOptions2D? o = null)
    {
        o ??= new WorldOptions2D();
        o.ThreadCount = threads;
        o.WorldId = 7;
        return new PhysicsWorld2D(o);
    }

    private static Body Floor(PhysicsWorld2D w, int layer = 0)
    {
        var b = w.CreateBody(BodyType.Static, new Vector2D(0, -0.5));
        w.CreateBox(b, new Vector2F(60f, 1f), default, 0f, 0f, new ShapeDef { Layer = layer });
        return b;
    }

    private static Body Ball(PhysicsWorld2D w, double x, double y, int layer = 0, uint category = 1, uint mask = uint.MaxValue, int group = 0)
    {
        var b = w.CreateBody(BodyType.Dynamic, new Vector2D(x, y));
        w.CreateCircle(b, 0.5f, default, new ShapeDef { Layer = layer, CategoryBits = category, MaskBits = mask, GroupIndex = group });
        return b;
    }

    private static int Count(PhysicsWorld2D w, EventKind k) => w.GetEvents(k).Length;

    // ------------------------------------------------------------------ layers and filters

    [Fact]
    public void LayerMatrixIgnoresPairsAndRefiltersAtTheNextStep()
    {
        var w = NewWorld();
        Floor(w, layer: 3);
        var ball = Ball(w, 0, 0.52, layer: 5);
        w.Step(Dt);
        Assert.Equal(1, Count(w, EventKind.ContactBegin));

        // Ignoring the layers ends the contact in the very next step, with an end event, and the ball falls through the floor.
        w.IgnoreLayerCollision(3, 5);
        Assert.True(w.GetLayerCollision(5, 3) == false);
        w.Step(Dt);
        Assert.Equal(1, Count(w, EventKind.ContactEnd));
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        Assert.True(ball.Position.Y < -2.0);

        // Allowing them again lets a ball that is touching the floor again collide.
        w.IgnoreLayerCollision(3, 5, false);
        var ball2 = Ball(w, 3, 3.0, layer: 5);
        for (int i = 0; i < 90; i++) { w.Step(Dt); }
        Assert.InRange(ball2.Position.Y, 0.45, 0.55);
    }

    [Fact]
    public void RelaxingTheMatrixCreatesPairsThatWereRejected()
    {
        var w = NewWorld(o: new WorldOptions2D { Gravity = new Vector2F(0, 0) });
        w.SetLayerCollision(1, 2, false);
        var floor = w.CreateBody(BodyType.Static, new Vector2D(0, -0.5));
        w.CreateBox(floor, new Vector2F(60f, 1f), default, 0f, 0f, new ShapeDef { Layer = 1 });
        Ball(w, 0, 0.45, layer: 2);
        w.Step(Dt);
        Assert.Equal(0, Count(w, EventKind.ContactBegin));
        w.SetLayerCollision(1, 2, true);
        w.Step(Dt);
        Assert.Equal(1, Count(w, EventKind.ContactBegin));
    }

    [Fact]
    public void ShapeFilterSettersApplyAtTheNextStep()
    {
        var w = NewWorld();
        Floor(w);
        var ball = Ball(w, 0, 0.51);
        w.Step(Dt);
        Assert.Equal(1, Count(w, EventKind.ContactBegin));
        ball.GetShapes()[0].MaskBits = 0;
        w.Step(Dt);
        Assert.Equal(1, Count(w, EventKind.ContactEnd));
        ball.GetShapes()[0].MaskBits = uint.MaxValue;
        ball.GetShapes()[0].GroupIndex = -1;
        var other = Ball(w, 0.2, 5);
        other.GetShapes()[0].GroupIndex = -1;
        Assert.Equal(-1, other.GetShapes()[0].GroupIndex);
    }

    [Fact]
    public void GroupsOverrideCategoryAndMaskAndCategoryMaskBothWays()
    {
        var w = NewWorld(o: new WorldOptions2D { Gravity = new Vector2F(0, 0) });
        // Same positive group collides even when the masks refuse; same negative group never collides even when everything agrees.
        var a = Ball(w, 0, 3, category: 2, mask: 0, group: 4);
        var b = Ball(w, 0.2, 3.5, category: 2, mask: 0, group: 4);
        var c = Ball(w, 5, 3, group: -2);
        var d = Ball(w, 5, 3.5, group: -2);
        var begins = new List<PhysicsEvent>();
        for (int i = 0; i < 3; i++) { w.Step(Dt); begins.AddRange(w.GetEvents(EventKind.ContactBegin).ToArray()); }
        Assert.Contains(begins, e => (e.BodyA == a.RawHandle && e.BodyB == b.RawHandle) || (e.BodyA == b.RawHandle && e.BodyB == a.RawHandle));
        Assert.DoesNotContain(begins, e => (e.BodyA == c.RawHandle && e.BodyB == d.RawHandle) || (e.BodyA == d.RawHandle && e.BodyB == c.RawHandle));
    }

    [Fact]
    public void ThePairPredicateDelegateAndTheStructContactFilterAreReevaluated()
    {
        var w = NewWorld();
        Floor(w);
        var ball = Ball(w, 0, 0.51);
        w.Step(Dt);
        Assert.Equal(1, Count(w, EventKind.ContactBegin));

        // A new delegate is noticed by reference: the pair that it refuses ends.
        w.Options.PairFilter = (a, b) => false;
        w.Step(Dt);
        Assert.Equal(1, Count(w, EventKind.ContactEnd));
        w.Options.PairFilter = null;
        w.Step(Dt);
        Assert.Equal(1, Count(w, EventKind.ContactBegin));

        // The struct filter does the same without allocating a delegate.
        w.SetContactFilter(new RefuseBody(ball.RawHandle));
        w.Step(Dt);
        Assert.Equal(1, Count(w, EventKind.ContactEnd));
        w.ClearContactFilter();
        w.Step(Dt);
        Assert.Equal(1, Count(w, EventKind.ContactBegin));
    }

    private readonly struct RefuseBody : IContactFilter
    {
        private readonly BodyHandle _body;

        public RefuseBody(BodyHandle body) => _body = body;

        public bool ShouldCollide(Shape a, Shape b) => a.Body.RawHandle != _body && b.Body.RawHandle != _body;
    }

    [Fact]
    public void FilterStateIsDeterministicAcrossThreadCounts()
    {
        ulong[] hashes = new ulong[3];
        int[] threads = [1, 3, 8];
        for (int t = 0; t < 3; t++)
        {
            var w = NewWorld(threads[t]);
            Floor(w);
            for (int i = 0; i < 30; i++) { Ball(w, (i % 6) * 0.9, 0.6 + (i / 6) * 1.1, layer: i % 3); }
            for (int s = 0; s < 120; s++)
            {
                if (s == 40) { w.IgnoreLayerCollision(0, 1); }
                if (s == 80) { w.IgnoreLayerCollision(0, 1, false); }
                w.Step(Dt);
            }

            hashes[t] = w.ComputeHash();
        }

        Assert.Equal(hashes[0], hashes[1]);
        Assert.Equal(hashes[0], hashes[2]);
    }

    // ------------------------------------------------------------------ sensors and events

    private static Body Sensor(PhysicsWorld2D w, double x, double y, float half = 1f, BodyType type = BodyType.Static, int layer = 0)
    {
        var b = w.CreateBody(type, new Vector2D(x, y));
        w.CreateBox(b, new Vector2F(2f * half, 2f * half), default, 0f, 0f, new ShapeDef { IsSensor = true, Layer = layer });
        return b;
    }

    [Fact]
    public void ASensorReportsEnterAndExitOnceAndGivesNoResponse()
    {
        var w = NewWorld(o: new WorldOptions2D { Gravity = new Vector2F(0, 0) });
        var sensor = Sensor(w, 0, 0, 1f);
        var ball = Ball(w, -3, 0);
        ball.LinearVelocity = new Vector2F(2f, 0);
        var log = new List<string>();
        sensor.OnTriggerEnter += t => log.Add("enter:" + t.Body.RawHandle.Index + ":" + t.Other.RawHandle.Index + ":" + t.SelfIsSensor);
        sensor.OnTriggerExit += t => log.Add("exit:" + t.Body.RawHandle.Index + ":" + t.Other.RawHandle.Index);
        ball.OnTriggerEnter += t => log.Add("ball-enter:" + t.SelfIsSensor);
        ball.OnCollisionEnter += c => log.Add("collision!");
        for (int i = 0; i < 240; i++) { w.Step(Dt); }
        Assert.Equal(["enter:0:1:True", "ball-enter:False", "exit:0:1"], log);
        Assert.Equal(2f, ball.LinearVelocity.X);
        Assert.True(ball.Position.X > 3);
    }

    [Fact]
    public void AnAsleepBodyInsideASensorGetsItsEnterEventOnceAndExitsWhenDestroyed()
    {
        var w = NewWorld();
        var sensor = Sensor(w, 0, 0, 2f);
        var ball = Ball(w, 0, 0);
        ball.Sleep();
        var enter = 0;
        var exit = 0;
        ball.OnTriggerEnter += t => enter++;
        ball.OnTriggerExit += t => exit++;
        for (int i = 0; i < 5; i++) { w.Step(Dt); }
        Assert.Equal(1, enter);
        Assert.False(ball.IsAwake);
        for (int i = 0; i < 5; i++) { w.Step(Dt); }
        Assert.Equal(1, enter);
        Assert.Equal(0, exit);
        Assert.Equal(0, Count(w, EventKind.SensorEnter));

        // Destroying the sensor mid-overlap gives exactly one exit event (queued now, delivered by the next step) to the surviving body.
        var seen = new List<bool>();
        ball.OnTriggerExit += t => seen.Add(t.Other.IsValid);
        sensor.Destroy();
        w.Step(Dt);
        Assert.Equal(1, Count(w, EventKind.SensorExit));
        Assert.Equal(1, exit);
        Assert.Equal([false], seen);
        for (int i = 0; i < 5; i++) { w.Step(Dt); }
        Assert.Equal(1, exit);
    }

    [Fact]
    public void ASensorOnASleepingBodyDoesNotWakeItAndGivesOneEnterAndOneExit()
    {
        var w = NewWorld(o: new WorldOptions2D { Gravity = new Vector2F(0, 0) });
        var holder = Sensor(w, 0, 0, 2f, BodyType.Dynamic);
        holder.Sleep();
        var ball = Ball(w, -6, 0);
        ball.LinearVelocity = new Vector2F(3f, 0);
        int enter = 0, exit = 0;
        holder.OnTriggerEnter += t => enter++;
        holder.OnTriggerExit += t => exit++;
        for (int i = 0; i < 240; i++) { w.Step(Dt); }
        Assert.Equal(1, enter);
        Assert.Equal(1, exit);
        Assert.False(holder.IsAwake);
    }

    [Fact]
    public void SensorsAddNoMassAndNeverDetectOtherSensors()
    {
        var w = NewWorld();
        var b = w.CreateBody(BodyType.Dynamic, default);
        w.CreateBox(b, new Vector2F(1f, 1f));
        float mass = b.Mass;
        w.CreateBox(b, new Vector2F(10f, 10f), default, 0f, 0f, new ShapeDef { IsSensor = true });
        Assert.Equal(mass, b.Mass);
        Sensor(w, 0, 0, 3f);
        Sensor(w, 0.5, 0, 3f);
        w.Step(Dt);
        Assert.Equal(2, Count(w, EventKind.SensorEnter)); // the body's box against each static sensor; sensor pairs and the body's own shapes never count
        Assert.True(b.GetShapes()[1].IsSensor);
    }

    [Fact]
    public void CollisionCallbacksCarryPointsNormalAndImpulses()
    {
        var w = NewWorld();
        Floor(w);
        var box = w.CreateBody(BodyType.Dynamic, new Vector2D(0, 0.55));
        w.CreateBox(box, new Vector2F(1f, 1f));
        Collision enter = default, stay = default, exit = default;
        box.OnCollisionEnter += c => enter = c;
        box.OnCollisionStay += c => stay = c;
        box.OnCollisionExit += c => exit = c;
        w.Options.Events |= EventMask.ContactPersist;
        for (int i = 0; i < 20; i++) { w.Step(Dt); }
        Assert.Equal(2, enter.PointCount);
        Assert.InRange(enter.Normal.Y, -1.0001f, -0.9999f); // from the box towards the floor
        Assert.True(enter.NormalImpulse >= 0f);
        Assert.Equal(2, stay.PointCount);
        // A resting box of mass m under gravity g: impulse over one step is m g dt.
        Assert.InRange(stay.NormalImpulse, box.Mass * 9.81f * Dt * 0.9f, box.Mass * 9.81f * Dt * 1.1f);
        Assert.InRange(stay.GetPoint(0).Point.Y, -0.05, 0.05);
        Assert.Throws<ArgumentOutOfRangeException>(() => stay.GetPoint(2));
        box.Position = new Vector2D(0, 5);
        w.Step(Dt);
        w.Step(Dt);
        Assert.Equal(0, exit.PointCount);
        Assert.True(exit.Body.IsValid);
    }

    [Fact]
    public void HitEventsFireAboveTheApproachSpeedThreshold()
    {
        var o = new WorldOptions2D { Events = EventMask.Default | EventMask.ContactHit, HitSpeedThreshold = 3f };
        var w = NewWorld(o: o);
        Floor(w);
        var slow = Ball(w, -5, 0.6);
        var fast = Ball(w, 5, 8);
        var hits = new List<(int, float)>();
        slow.OnCollisionHit += c => hits.Add((c.Body.RawHandle.Index, c.ApproachSpeed));
        fast.OnCollisionHit += c => hits.Add((c.Body.RawHandle.Index, c.ApproachSpeed));
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.Single(hits);
        Assert.Equal(fast.RawHandle.Index, hits[0].Item1);
        Assert.InRange(hits[0].Item2, 8f, 13f);
    }

    [Fact]
    public void SleepAndWakeEventsAreDeliveredAndPolled()
    {
        var w = NewWorld();
        Floor(w);
        var ball = Ball(w, 0, 0.5);
        int slept = 0, woke = 0;
        ball.OnSleep += b => slept++;
        ball.OnWake += b => woke++;
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.Equal(1, slept);
        Assert.False(ball.IsAwake);
        ball.Wake();
        w.Step(Dt);
        Assert.Equal(1, woke);
        Assert.Equal(1, Count(w, EventKind.Wake));
    }

    // ------------------------------------------------------------------ snapshot

    private static (PhysicsWorld2D, Body, Body) Scene()
    {
        var w = NewWorld(o: new WorldOptions2D { Gravity = new Vector2F(0, 0), Events = EventMask.All });
        Floor(w, 2);
        var sensor = Sensor(w, 0, 3, 1.5f);
        var ball = Ball(w, -6, 3, layer: 4);
        ball.LinearVelocity = new Vector2F(2f, 0);
        w.SetLayerCollision(2, 4, false);
        w.Options.HitSpeedThreshold = 2f;
        return (w, sensor, ball);
    }

    [Fact]
    public void SnapshotMidOverlapWithFiltersAndSensorsIsByteExact()
    {
        var (a, _, _) = Scene();
        for (int i = 0; i < 100; i++) { a.Step(Dt); }
        a.SetLayerCollision(2, 4, true); // a pending re-filter is part of the state
        byte[] snap = a.Snapshot();
        var (b, _, _) = Scene();
        b.Restore(snap);
        Assert.Equal(snap, b.Snapshot());
        Assert.Equal(a.ComputeHash(), b.ComputeHash());
        var logA = new List<string>();
        var logB = new List<string>();
        for (int i = 0; i < 200; i++)
        {
            a.Step(Dt); b.Step(Dt);
            Assert.Equal(a.ComputeHash(), b.ComputeHash());
            Assert.Equal(a.Events.ToArray().Select(e => e.Kind + ":" + e.BodyA.Index + ":" + e.BodyB.Index).ToArray(), b.Events.ToArray().Select(e => e.Kind + ":" + e.BodyA.Index + ":" + e.BodyB.Index).ToArray());
            logA.AddRange(a.Events.ToArray().Select(e => e.Kind.ToString()));
        }

        Assert.Equal(1, logA.Count(k => k == "SensorExit"));
        Assert.Equal(a.Snapshot(), b.Snapshot());
    }

    [Fact]
    public void ARestoredWorldMatchesAnUninterruptedRunStepForStep()
    {
        var (a, _, _) = Scene();
        var (c, _, _) = Scene();
        var hashes = new List<ulong>();
        byte[]? snap = null;
        for (int i = 0; i < 250; i++)
        {
            if (i == 70) { snap = a.Snapshot(); }
            a.Step(Dt);
            hashes.Add(a.ComputeHash());
        }

        c.Restore(snap!);
        for (int i = 70; i < 250; i++)
        {
            c.Step(Dt);
            Assert.Equal(hashes[i], c.ComputeHash());
        }
    }

    [Fact]
    public void AWorldThatNeverUsedTheServicesWritesTheOldSectionVersion()
    {
        var w = NewWorld();
        Floor(w);
        Ball(w, 0, 3);
        byte[] plain = w.Snapshot();
        w.SetLayerCollision(1, 2, false);
        byte[] used = w.Snapshot();
        Assert.True(used.Length > plain.Length);
        w.SetLayerCollision(1, 2, true);
        w.Step(Dt);
        var w2 = NewWorld();
        Floor(w2);
        Ball(w2, 0, 3);
        w2.Step(Dt);
        // Setting and clearing a layer pair leaves a pending re-filter at most for one step; afterwards the bytes equal a world that never used it.
        w.Step(Dt); w2.Step(Dt);
        Assert.Equal(w2.Snapshot(), w.Snapshot());
    }

    // ------------------------------------------------------------------ debug geometry

    [Fact]
    public void DebugGeometryOfASmallSceneHasExactContents()
    {
        var w = NewWorld(o: new WorldOptions2D { Gravity = new Vector2F(0, 0) });
        var floor = w.CreateBody(BodyType.Static, new Vector2D(0, -1));
        w.CreateBox(floor, new Vector2F(4f, 2f));
        var ball = w.CreateBody(BodyType.Dynamic, new Vector2D(0.5, 0.25));
        w.CreateCircle(ball, 0.5f);
        var g = new DebugGeometry();
        w.GetDebugGeometry(DebugContent.All, g);
        Assert.Equal(2, g.Shapes.Length);
        Assert.Equal(DebugShapeKind.Polygon, g.Shapes[0].Kind);
        Assert.Equal(4, g.Shapes[0].PointCount);
        var pts = g.Points.Slice(g.Shapes[0].FirstPoint, 4).ToArray();
        Assert.Equal(new[] { (-2.0, -2.0), (2.0, -2.0), (2.0, 0.0), (-2.0, 0.0) }.OrderBy(p => p).ToArray(), pts.Select(p => (p.X, p.Y)).OrderBy(p => p).ToArray());
        Assert.Equal(DebugShapeKind.Circle, g.Shapes[1].Kind);
        Assert.Equal(0.5f, g.Shapes[1].Radius);
        Assert.Equal((0.5, 0.25), (g.Points[g.Shapes[1].FirstPoint].X, g.Points[g.Shapes[1].FirstPoint].Y));
        Assert.Equal(2, g.Bounds.Length);
        Assert.Equal((-2.0, -2.0, 2.0, 0.0), (g.Bounds[0].Min.X, g.Bounds[0].Min.Y, g.Bounds[0].Max.X, g.Bounds[0].Max.Y));
        Assert.Equal((0.0, -0.25, 1.0, 0.75), (g.Bounds[1].Min.X, g.Bounds[1].Min.Y, g.Bounds[1].Max.X, g.Bounds[1].Max.Y));
        Assert.Equal(2, g.BroadphaseBounds.Length);
        Assert.Equal(-2.0, g.BroadphaseBounds[0].Min.X); // static: no margin
        Assert.Equal(-0.1, g.BroadphaseBounds[1].Min.X, 8); // the margin is rounded to a multiple of 2^-32
        Assert.Equal(2, g.Bodies.Length);
        Assert.Equal(DebugBodyState.Static, g.Bodies[0].State);
        Assert.Equal(-1, g.Bodies[0].Island);
        Assert.Equal(DebugBodyState.Awake, g.Bodies[1].State);
        Assert.Equal(0.25, g.Bodies[1].CenterOfMass.Y);
        Assert.Empty(g.Joints.ToArray());

        // The ball overlaps the floor's top edge by 0.25: a contact at its lowest point... after one step the manifold exists.
        w.Step(Dt);
        w.GetDebugGeometry(DebugContent.Contacts | DebugContent.Bodies, g);
        Assert.Empty(g.Shapes.ToArray());
        Assert.Single(g.Contacts.ToArray());
        Assert.Equal(1f, g.Contacts[0].Normal.Y == 0f ? 1f : Math.Abs(g.Contacts[0].Normal.Y), 3);
        Assert.True(g.Contacts[0].Separation < 0f);
        Assert.Equal(0f, g.Contacts[0].Normal.X, 3);
        ball.Sleep();
        w.GetDebugGeometry(DebugContent.Bodies, g);
        Assert.Equal(DebugBodyState.Asleep, g.Bodies[1].State);
        Assert.Equal(0, g.Bodies[1].Island);
    }
}
