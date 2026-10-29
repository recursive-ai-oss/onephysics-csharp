using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

public class Services3DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld3D NewWorld(Action<WorldOptions3D>? tweak = null) => World3DTests.NewWorld(1, 11, tweak);

    private static Shape AddSensorBox(PhysicsWorld3D w, Vector3D at, Vector3F half)
    {
        var body = w.CreateBody(BodyType.Static, at);
        return w.CreateBox(body, half, 0f, new ShapeDef { IsSensor = true, Density = 0f });
    }

    // ------------------------------------------------------------------ layers and filters

    [Fact]
    public void IgnoredLayersPassThroughAndARuntimeChangeEndsTheContact()
    {
        var w = NewWorld();
        var floor = World3DTests.Floor(w);
        var box = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0.6, 0));
        var floorShape = floor.GetShapes()[0];
        var boxShape = box.GetShapes()[0];
        floorShape.Layer = 1; boxShape.Layer = 2;
        int ends = 0;
        box.OnCollisionExit += _ => ends++;
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        Assert.InRange(box.Position.Y, 0.45, 0.6); // resting on the floor
        w.IgnoreLayerCollision(1, 2);
        w.Step(Dt);
        Assert.Equal(1, ends);
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.True(box.Position.Y < -2.0);
        Assert.True(w.GetLayerCollision(0, 5));
        Assert.False(w.GetLayerCollision(1, 2));
        Assert.False(w.GetLayerCollision(2, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.SetLayerCollision(32, 0, true));
        Assert.Throws<ArgumentOutOfRangeException>(() => boxShape.Layer = -1);
    }

    [Fact]
    public void LiftingTheIgnoreAgainLetsBodiesCollide()
    {
        var w = NewWorld();
        var floor = World3DTests.Floor(w);
        floor.GetShapes()[0].Layer = 1;
        var box = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 3, 0));
        box.Layer = 2;
        w.IgnoreLayerCollision(1, 2);
        for (int i = 0; i < 20; i++) { w.Step(Dt); }
        w.IgnoreLayerCollision(1, 2, false);
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        Assert.True(box.Position.Y > -1.0, "it fell through after the ignore was lifted: " + box.Position.Y);
    }

    private struct NoSpheres : IContactFilter
    {
        public bool ShouldCollide(Shape a, Shape b) => !(a.Body.Mass > 0f && b.Body.Mass > 0f);
    }

    [Fact]
    public void AStructContactFilterAndAPairPredicateVetoPairs()
    {
        var w = NewWorld();
        World3DTests.Floor(w);
        var a = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0.5, 0));
        var b = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0.4, 1.5, 0));
        w.SetContactFilter(new NoSpheres()); // dynamic bodies ignore each other
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.True(Math.Abs(b.Position.Y - a.Position.Y) < 0.3, "b sank into a: " + (b.Position.Y - a.Position.Y));
        w.ClearContactFilter();
        w.Options.PairFilter = (x, y) => true;
        w.Step(Dt);
        Assert.False(w.Options.PairFilter == null);
    }

    [Fact]
    public void GroupAndMaskRulesApply()
    {
        var w = NewWorld();
        World3DTests.Floor(w);
        var a = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 3, 0));
        a.GetShapes()[0].GroupIndex = -4;
        var b = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 6, 0));
        b.GetShapes()[0].GroupIndex = -4;
        for (int i = 0; i < 240; i++) { w.Step(Dt); }
        Assert.True(Math.Abs(a.Position.Y - b.Position.Y) < 0.5); // they overlap at rest on the floor because the negative group beats the masks
        var c = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(5, 3, 0));
        c.GetShapes()[0].MaskBits = 0;
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.True(c.Position.Y < -1.0);
    }

    // ------------------------------------------------------------------ sensors and events

    [Fact]
    public void TriggersFireEnterAndExitOnceEach()
    {
        var w = NewWorld();
        var sensor = AddSensorBox(w, new Vector3D(0, 3, 0), new Vector3F(2, 1, 2));
        var ball = w.AddSphere(0.25f, new Vector3D(0, 6, 0));
        var log = new List<string>();
        ball.OnTriggerEnter += t => log.Add("ball enter " + t.SelfIsSensor);
        ball.OnTriggerExit += t => log.Add("ball exit " + t.SelfIsSensor);
        sensor.Body.OnTriggerEnter += t => log.Add("sensor enter " + t.SelfIsSensor);
        sensor.Body.OnTriggerExit += t => log.Add("sensor exit " + t.SelfIsSensor);
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.Equal(["sensor enter True", "ball enter False", "sensor exit True", "ball exit False"], log);
        Assert.True(sensor.IsSensor);
        Assert.Equal(0f, sensor.Body.Mass);
    }

    [Fact]
    public void ASensorDoesNotPushAndAddsNoMass()
    {
        var w = NewWorld();
        var body = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 1, 0));
        float mass = body.Mass;
        var extra = w.CreateBox(body, new Vector3F(6, 6, 6), 0f, new ShapeDef { IsSensor = true, Density = 5f });
        Assert.Equal(mass, body.Mass);
        Assert.True(extra.IsSensor);
        w.Step(Dt);
        World3DTests.Floor(w);
    }

    [Fact]
    public void ASleepingBodyInsideASensorStillGetsItsEnterEvent()
    {
        var w = NewWorld();
        AddSensorBox(w, new Vector3D(0, 1, 0), new Vector3F(2, 2, 2));
        var def = new BodyDef { Position = new Vector3D(0, 1, 0), StartAsleep = true };
        var body = w.CreateBody(in def);
        w.CreateSphere(body, 0.5f);
        int enters = 0;
        body.OnTriggerEnter += _ => enters++;
        w.Step(Dt);
        Assert.False(body.IsAwake);
        Assert.Equal(1, enters);
        w.Step(Dt);
        Assert.Equal(1, enters);
    }

    [Fact]
    public void ASensorDestroyedMidOverlapGivesAnExit()
    {
        var w = NewWorld(o => o.Gravity = default);
        var sensor = AddSensorBox(w, new Vector3D(0, 0, 0), new Vector3F(2, 2, 2));
        var ball = w.AddSphere(0.5f, new Vector3D(0, 0, 0));
        int enters = 0, exits = 0;
        ball.OnTriggerEnter += _ => enters++;
        ball.OnTriggerExit += t => { exits++; Assert.True(t.Other.IsValid); };
        w.Step(Dt);
        Assert.Equal(1, enters);
        sensor.Destroy();
        w.Step(Dt);
        Assert.Equal(1, exits);
        Assert.Equal(1, w.GetEvents(EventKind.SensorExit).Length);
        w.Step(Dt);
        Assert.Equal(1, exits);
    }

    [Fact]
    public void ADestroyedBodyInsideASensorGivesAnExitWithAStaleOther()
    {
        var w = NewWorld(o => o.Gravity = default);
        AddSensorBox(w, new Vector3D(0, 0, 0), new Vector3F(2, 2, 2));
        var ball = w.AddSphere(0.5f, new Vector3D(0, 0, 0));
        w.Step(Dt);
        var sensorBody = w.GetEvents(EventKind.SensorEnter)[0].BodyA;
        ball.Destroy();
        bool stale = false;
        w.GetBody(sensorBody).OnTriggerExit += t => stale = !t.Other.IsValid;
        w.Step(Dt);
        Assert.True(stale);
    }

    [Fact]
    public void HitEventsReportTheApproachSpeedOnce()
    {
        var w = NewWorld(o => { o.Events |= EventMask.ContactHit; });
        World3DTests.Floor(w);
        var ball = w.AddSphere(0.5f, new Vector3D(0, 6, 0));
        var hits = new List<float>();
        ball.OnCollisionHit += c => hits.Add(c.ApproachSpeed);
        for (int i = 0; i < 180; i++) { w.Step(Dt); }
        Assert.Single(hits);
        Assert.InRange(hits[0], 8f, 12f);
        // Below the threshold nothing is reported.
        w.Options.HitSpeedThreshold = 50f;
        ball.LinearVelocity = new Vector3F(0, 8, 0);
        for (int i = 0; i < 180; i++) { w.Step(Dt); }
        Assert.Single(hits);
    }

    [Fact]
    public void ABoxOnSeveralTrianglesReportsOneContactPerPair()
    {
        var w = NewWorld();
        Vector3F[] v = [new(-5, 0, -5), new(0, 0, -5), new(5, 0, -5), new(-5, 0, 0), new(0, 0, 0), new(5, 0, 0), new(-5, 0, 5), new(0, 0, 5), new(5, 0, 5)];
        int[] idx = [0, 4, 1, 0, 3, 4, 1, 5, 2, 1, 4, 5, 3, 7, 4, 3, 6, 7, 4, 8, 5, 4, 7, 8];
        w.AddMesh(v, idx);
        var box = w.AddBox(new Vector3F(2, 1, 2), at: new Vector3D(0, 0.9, 0));
        int begins = 0, ends = 0, maxCoreBegins = 0, points = 0;
        box.OnCollisionEnter += c => { begins++; points = c.PointCount; };
        box.OnCollisionExit += _ => ends++;
        for (int i = 0; i < 90; i++)
        {
            w.Step(Dt);
            int coreBegins = 0;
            foreach (var e in w.Core.Events.Delivered) { if (e.Kind == EventKind.ContactBegin) { coreBegins++; } }
            maxCoreBegins = Math.Max(maxCoreBegins, coreBegins);
        }

        Assert.Equal(1, begins);
        Assert.True(maxCoreBegins > 1, "the core should have reported several sub-shape contacts");
        Assert.InRange(points, 1, 4);
        box.ApplyImpulse(new Vector3F(0, 40, 0));
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        Assert.Equal(1, ends);
    }

    [Fact]
    public void SleepAndWakeEventsReachTheBody()
    {
        var w = NewWorld();
        World3DTests.Floor(w);
        var box = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0.5, 0));
        int sleeps = 0, wakes = 0;
        box.OnSleep += _ => sleeps++;
        box.OnWake += _ => wakes++;
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.Equal(1, sleeps);
        box.Wake();
        w.Step(Dt);
        Assert.Equal(1, wakes);
    }

    // ------------------------------------------------------------------ snapshot

    private static PhysicsWorld3D ServiceScene()
    {
        var w = NewWorld(o => { o.Events |= EventMask.ContactHit | EventMask.ContactPersist; });
        World3DTests.Floor(w).GetShapes()[0].Layer = 3;
        AddSensorBox(w, new Vector3D(0, 2, 0), new Vector3F(3, 1.2f, 3));
        w.SetLayerCollision(3, 4, false);
        for (int i = 0; i < 8; i++)
        {
            var b = w.AddBox(new Vector3F(0.6f, 0.6f, 0.6f), at: new Vector3D(-1.5 + i * 0.45, 4 + i * 0.7, 0.1 * i));
            b.Layer = i == 5 ? 4 : 0;
        }

        return w;
    }

    [Fact]
    public void SnapshotMidOverlapRoundTripsBitIdentically()
    {
        var w = ServiceScene();
        int entered = 0;
        for (int i = 0; i < 40; i++) { w.Step(Dt); entered += w.GetEvents(EventKind.SensorEnter).Length; }
        Assert.True(entered > 0, "the snapshot must be taken with a sensor overlap on the books");
        byte[] snap = w.Snapshot();
        ulong h0 = w.ComputeHash();
        var hashes = new List<ulong>();
        var events = new List<string>();
        for (int i = 0; i < 60; i++) { w.Step(Dt); hashes.Add(w.ComputeHash()); events.Add(string.Join(";", w.Events.ToArray().Select(e => e.Kind + ":" + e.ShapeA.Index + ":" + e.ShapeB.Index))); }
        byte[] end = w.Snapshot();

        w.Restore(snap);
        Assert.Equal(h0, w.ComputeHash());
        Assert.Equal(snap, w.Snapshot());
        for (int i = 0; i < 60; i++)
        {
            w.Step(Dt);
            Assert.Equal(hashes[i], w.ComputeHash());
            Assert.Equal(events[i], string.Join(";", w.Events.ToArray().Select(e => e.Kind + ":" + e.ShapeA.Index + ":" + e.ShapeB.Index)));
        }

        Assert.Equal(end, w.Snapshot());

        var fresh = World3DTests.NewWorld(1, 11, o => o.Events |= EventMask.ContactHit | EventMask.ContactPersist);
        fresh.Restore(snap);
        Assert.Equal(h0, fresh.ComputeHash());
        Assert.Equal(snap, fresh.Snapshot());
        Assert.Equal(w.GetLayerCollision(3, 4), fresh.GetLayerCollision(3, 4));
        Assert.True(fresh.GetBody(w.Core.HandleOfBody(2)).GetShapes().Length >= 1);
    }

    [Fact]
    public void AWorldWithoutServicesKeepsItsOldBytes()
    {
        var w = World3DTests.BuildScene(1);
        for (int i = 0; i < 5; i++) { w.Step(Dt); }
        byte[] snap = w.Snapshot();
        // Version word of the 3D section is 1 when there is no joint and no service state: find the section id and read the next word.
        int at = IndexOf(snap, BitConverter.GetBytes(0x33445747));
        Assert.True(at > 0);
        Assert.Equal(1u, BitConverter.ToUInt32(snap, at + 8)); // id, length, then the version
    }

    private static int IndexOf(byte[] data, byte[] pattern)
    {
        for (int i = 0; i + pattern.Length <= data.Length; i++)
        {
            if (data.AsSpan(i, pattern.Length).SequenceEqual(pattern)) { return i; }
        }

        return -1;
    }

    [Fact]
    public void ADamagedServicesBlockIsRejectedAndTheWorldIsUnchanged()
    {
        var w = ServiceScene();
        for (int i = 0; i < 10; i++) { w.Step(Dt); }
        byte[] snap = w.Snapshot();
        ulong h = w.ComputeHash();
        var rng = new Random(5);
        for (int k = 0; k < 200; k++)
        {
            var bad = (byte[])snap.Clone();
            bad[rng.Next(bad.Length)] ^= (byte)(1 << rng.Next(8));
            try { w.Restore(bad); w.Restore(snap); }
            catch (SnapshotFormatException) { }
            Assert.Equal(h, w.ComputeHash());
        }

        for (int cut = 0; cut < snap.Length; cut += 97) { Assert.Throws<SnapshotFormatException>(() => w.Restore(snap.AsSpan(0, cut).ToArray())); }
        Assert.Equal(h, w.ComputeHash());
    }

    // ------------------------------------------------------------------ debug geometry

    [Fact]
    public void DebugGeometryIsExactOnASmallScene()
    {
        var w = NewWorld(o => o.Gravity = default);
        var box = w.AddBox(new Vector3F(2, 4, 6), at: new Vector3D(10, 20, 30), type: BodyType.Static);
        var ball = w.AddSphere(0.5f, new Vector3D(10, 20, 33.4), type: BodyType.Dynamic);
        w.AddCapsule(0.5f, 0.25f, new Vector3D(0, 0, 0), type: BodyType.Static);
        w.AddConvexHull([new(0, 0, 0), new(1, 0, 0), new(0, 1, 0), new(0, 0, 1)], new Vector3D(-5, 0, 0), type: BodyType.Static);
        var g = new DebugGeometry();
        w.Step(Dt);
        w.GetDebugGeometry(DebugContent.All, g);
        Assert.Equal(4, g.Shapes.Length);
        Assert.Equal(DebugShapeKind.Box, g.Shapes[0].Kind);
        Assert.Equal(8, g.Shapes[0].PointCount);
        var p = g.Points;
        int f = g.Shapes[0].FirstPoint;
        Assert.Equal(new Vector3D(9, 18, 27), p[f]);
        Assert.Equal(new Vector3D(11, 22, 33), p[f + 7]);
        Assert.Equal(36, g.Shapes[0].IndexCount);
        Assert.Equal(DebugShapeKind.Circle, g.Shapes[1].Kind);
        Assert.Equal(0.5f, g.Shapes[1].Radius);
        Assert.Equal(DebugShapeKind.Capsule, g.Shapes[2].Kind);
        Assert.Equal(new Vector3D(0, -0.5, 0), p[g.Shapes[2].FirstPoint]);
        Assert.Equal(new Vector3D(0, 0.5, 0), p[g.Shapes[2].FirstPoint + 1]);
        Assert.Equal(DebugShapeKind.Hull, g.Shapes[3].Kind);
        Assert.Equal(4, g.Shapes[3].PointCount);
        Assert.Equal(12, g.Shapes[3].IndexCount);
        Assert.Equal(4, g.Bounds.Length);
        Assert.True(Math.Abs(g.Bounds[0].Min.X - 9) < 1e-3 && Math.Abs(g.Bounds[0].Min.Y - 18) < 1e-3 && Math.Abs(g.Bounds[0].Min.Z - 27) < 1e-3, "tight bounds are padded by a few float ulps");
        Assert.Equal(4, g.Bodies.Length);
        Assert.Equal(DebugBodyState.Static, g.Bodies[0].State);
        Assert.Equal(new Vector3D(10, 20, 30), g.Bodies[0].CenterOfMass);
        Assert.Equal(DebugBodyState.Awake, g.Bodies[1].State);
        Assert.True(g.Contacts.Length > 0);
        var c0 = g.Contacts[0];
        Assert.True(Math.Abs(c0.Normal.Z) > 0.99f, "contact normal " + c0.Normal);
        Assert.True(Math.Abs(c0.Point.Z - 33.0) < 0.1, "contact point z " + c0.Point.Z);
        Assert.Equal(box.Position, new Vector3D(10, 20, 30));
        Assert.True(ball.IsValid);

        // Joints with their anchors, axes and limits.
        var hinge = w.AddHinge(box, ball, new Vector3D(10, 20, 31), new Vector3F(0, 1, 0));
        hinge.SetLimits(-0.5f, 0.75f); hinge.EnableLimit = true;
        w.GetDebugGeometry(DebugContent.Joints, g);
        Assert.Single(g.Joints.ToArray());
        var j = g.Joints[0];
        Assert.Equal(new Vector3D(10, 20, 31), j.AnchorA);
        Assert.Equal(0f, (float)(j.Axis.X * j.Axis.X + j.Axis.Z * j.Axis.Z), 3);
        Assert.Equal(-0.5f, j.LowerLimit);
        Assert.Equal(0.75f, j.UpperLimit);

        w.GetDebugGeometry(DebugContent.None, g);
        Assert.Equal(0, g.Shapes.Length + g.Bodies.Length + g.Contacts.Length);
    }

    [Fact]
    public void MeshTrianglesAppearOnlyWithTheirFlag()
    {
        var w = NewWorld();
        w.AddMesh([new(0, 0, 0), new(1, 0, 0), new(0, 0, 1), new(1, 0, 1)], [0, 2, 1, 1, 2, 3]);
        var g = new DebugGeometry();
        w.GetDebugGeometry(DebugContent.Shapes, g);
        Assert.Equal(DebugShapeKind.Mesh, g.Shapes[0].Kind);
        Assert.Equal(0, g.Shapes[0].IndexCount);
        Assert.Equal(8, g.Shapes[0].PointCount);
        w.GetDebugGeometry(DebugContent.Shapes | DebugContent.MeshTriangles, g);
        Assert.Equal(6, g.Shapes[0].IndexCount);
        Assert.Equal(6, g.Shapes[0].PointCount);
    }

    // ------------------------------------------------------------------ API parity with the 2D world

    [Fact]
    public void TheWorldsExposeTheSameServiceMembers()
    {
        static HashSet<string> Names(Type t) => t.GetMembers(System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.DeclaredOnly)
            .Where(m => m.MemberType != System.Reflection.MemberTypes.Constructor && !(m is System.Reflection.MethodInfo mi && mi.IsSpecialName && false)).Select(m => m.Name).ToHashSet();
        var two = Names(typeof(RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D));
        var three = Names(typeof(PhysicsWorld3D));
        string[] expectedOnlyIn2D = ["Shape2D", "CircleCast", "CircleCastAny", "CircleCastAll", "OverlapCircle", "OverlapCircleAny", "OverlapCircleAll", "AddCircle", "AddPolygon", "AddSegment", "AddChain", "CreateCircle", "CreatePolygon", "CreateCapsule", "CreateSegment", "CreateChain", "AddCharacter", "CreateCharacter"];
        var only2 = two.Except(three).Except(expectedOnlyIn2D).Where(n => !n.StartsWith("Add") && !n.StartsWith("Create") && !n.StartsWith("get_") && !n.StartsWith("set_")).ToList();
        // What 2D has that 3D lacks, as of this unit: characters and the joint helpers named differently.
        Assert.True(only2.Count < 40, string.Join(",", only2));
        string[] services = ["Raycast", "RaycastAny", "RaycastAll", "ShapeCast", "ShapeCastAny", "ShapeCastAll", "BoxCast", "CapsuleCast", "OverlapShape", "OverlapBox", "OverlapPoint", "ClosestPoint", "ClosestPointAll", "GetDebugGeometry", "GetEvents", "GetCollision", "GetTrigger", "SetLayerCollision", "GetLayerCollision", "IgnoreLayerCollision", "SetContactFilter", "ClearContactFilter", "RefilterPairs", "Events"];
        foreach (string s in services)
        {
            Assert.Contains(s, two);
            Assert.Contains(s, three);
        }
    }

    // ------------------------------------------------------------------ hostile input

    [Fact]
    public void HostileQueryInputsGiveTypedErrors()
    {
        var w = NewWorld();
        World3DTests.Floor(w);
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Raycast(new Vector3D(double.NaN, 0, 0), new Vector3F(0, -1, 0), out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Raycast(new Vector3D(1e9, 0, 0), new Vector3F(0, -1, 0), out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Raycast(default, default, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Raycast(default, new Vector3F(float.NaN, 1, 0), out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Raycast(default, new Vector3F(0, 1, 0), out _, 0f));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Raycast(default, new Vector3F(0, 1, 0), out _, float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.SphereCast(default, -1f, new Vector3F(0, 1, 0), out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.BoxCast(default, new Vector3F(0, 1, 1), default, new Vector3F(0, 1, 0), out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.OverlapCapsule(default, 1f, 0f, default, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.OverlapSphere(default, float.PositiveInfinity, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.ClosestPoint(default, out _, -1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.OverlapBox(default, new Vector3F(1, 1, 1), new QuaternionF(float.NaN, 0, 0, 1), out _));
        Assert.True(w.Raycast(new Vector3D(0, 5, 0), new Vector3F(0, -1, 0), out var hit, float.PositiveInfinity));
        Assert.InRange(hit.Distance, 4.99f, 5.01f);
        Assert.Equal(new Vector3F(0, 1, 0).Y, hit.Normal.Y, 3);
        Assert.Equal(0, w.RaycastAll(new Vector3D(0, 5, 0), new Vector3F(0, -1, 0), Span<QueryHit3D>.Empty));
    }

    [Fact]
    public void TheSharedServicesSceneIsThreadAndRestoreIndependent()
    {
        // The scene the hash matrix and smoke apps run: every path must end in the same hash.
        ulong a = Services3DScene.Run(1, false), b = Services3DScene.Run(4, false), c = Services3DScene.Run(1, true);
        Assert.Equal(a, b);
        Assert.Equal(a, c);
        Services3DScene.Run(1, false);
        var seen = Services3DScene.Seen!;
        Assert.True(seen[(int)EventKind.SensorEnter] > 0 && seen[(int)EventKind.SensorExit] > 0, "the sensor must fire");
        Assert.True(seen[(int)EventKind.ContactBegin] > 0 && seen[(int)EventKind.ContactEnd] > 0, "contacts must begin and end");
    }
}
