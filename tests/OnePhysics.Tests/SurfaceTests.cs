using System;
using System.IO;
using RecursiveAI.OneMath;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public class SurfaceTests
{
    [Fact]
    public void FastJumpDoesNotGetClampedByContinuousCollision()
    {
        var w = new P2.PhysicsWorld2D(new() { Gravity = default });
        w.AddBox(new(10, 0.2f), at: new(0, 1), type: BodyType.Static, material: new P2.Material2D(0.6f, surface: new() { OneWayDirection = new(0, 1, 0) }));
        var b = w.AddCircle(0.1f, at: new(0, 0)); b.IsBullet = true; b.LinearVelocity = new(0, 30);
        for (int i = 0; i < 4; i++) w.Step(1f / 60);
        Assert.InRange(b.Position.Y, 1.9999, 2.0001);
        var v = new P3.PhysicsWorld3D(new() { Gravity = default });
        v.AddBox(new(10, 0.2f, 10), at: new(0, 1, 0), type: BodyType.Static, material: new P3.Material3D(0.6f, surface: new() { OneWayDirection = new(0, 1, 0) }));
        var s = v.AddSphere(0.1f, at: new(0, 0, 0)); s.LinearVelocity = new(0, 30, 0);
        for (int i = 0; i < 4; i++) v.Step(1f / 60);
        Assert.InRange(s.Position.Y, 1.9999, 2.0001);
    }

    [Fact]
    public void PendingMaterialChangeSurvivesRestoreBeforeTheNextStep()
    {
        var w = new P2.PhysicsWorld2D(new() { WorldId = 880 }); var g = w.AddBox(new(10, 1), at: new(0, -0.5), type: BodyType.Static); w.AddBox(new(1, 1), at: new(0, 0.5));
        w.Step(1f / 60); g.GetShapes()[0].SetMaterials(new P2.Material2D[] { new(0) }, new[] { 0 });
        var copy = new P2.PhysicsWorld2D(new() { WorldId = 880 }); copy.Restore(w.Snapshot());
        w.Step(1f / 60); copy.Step(1f / 60); Assert.Equal(w.Snapshot(), copy.Snapshot());
    }

    [Fact]
    public void SurfaceStateAndHostHooksAreThreadAndWidthIndependent()
    {
        Assert.Equal(Common.ContactSurfaceScene.Planar(1, 1, true), Common.ContactSurfaceScene.Planar(8, 8));
        Assert.Equal(Common.ContactSurfaceScene.Spatial(1, 1, true), Common.ContactSurfaceScene.Spatial(8, 8));
    }

    [Fact]
    public void BodyFrameConveyorsMoveBoxesAndSurviveForkRestoreAndScene()
    {
        var w = new P2.PhysicsWorld2D(new() { WorldId = 991, SleepEnabled = false });
        var ground = w.CreateBody(new() { Type = BodyType.Static, Position = new(0, -0.5) });
        w.CreateBox(ground, new(100, 1), def: new() { Surface = new() { Velocity = new(2, 0, 0) } });
        var box = w.AddBox(new(1, 1), at: new(0, 0.5));
        for (int i = 0; i < 160; i++) w.Step(1f / 60);
        Assert.InRange(box.LinearVelocity.X, 1.98f, 2.02f);
        var copy = w.Fork(); Assert.Equal(w.Snapshot(), copy.Snapshot());
        var loaded = P2.PhysicsWorld2D.LoadScene(new StringReader(w.SaveScene()));
        Assert.Equal(w.Snapshot(), loaded.Snapshot());
        for (int i = 0; i < 20; i++) { w.Step(1f / 60); copy.Step(1f / 60); loaded.Step(1f / 60); Assert.Equal(w.ComputeHash(), copy.ComputeHash()); Assert.Equal(w.ComputeHash(), loaded.ComputeHash()); }
        var spatial = new P3.PhysicsWorld3D(new() { SleepEnabled = false });
        var g = spatial.CreateBody(new() { Type = BodyType.Static, Position = new(0, -0.5, 0) });
        spatial.CreateBox(g, new(100, 1, 100), def: new() { Surface = new() { Velocity = new(2, 0, 1) } });
        var b = spatial.AddBox(new(1, 1, 1), at: new(0, 0.5, 0));
        for (int i = 0; i < 160; i++) spatial.Step(1f / 60);
        Assert.InRange(b.LinearVelocity.X, 1.97f, 2.03f); Assert.InRange(b.LinearVelocity.Z, 0.97f, 1.03f);
    }

    [Fact]
    public void OneWayJumpThroughAndLandInBothDimensions()
    {
        var w = new P2.PhysicsWorld2D(new() { SleepEnabled = false });
        var g = w.CreateBody(new() { Type = BodyType.Static, Position = new(0, 1) });
        w.CreateBox(g, new(6, 0.2f), def: new() { Surface = new() { OneWayDirection = new(0, 1, 0) } });
        var ball = w.AddCircle(0.25f, at: new(0, 0)); ball.LinearVelocity = new(0, 7);
        double maxY = 0;
        for (int i = 0; i < 180; i++) { w.Step(1f / 60); maxY = Math.Max(maxY, ball.Position.Y); }
        Assert.True(maxY > 2); Assert.InRange(ball.Position.Y, 1.33, 1.37);
        var v = new P3.PhysicsWorld3D(new() { SleepEnabled = false });
        var h = v.CreateBody(new() { Type = BodyType.Static, Position = new(0, 1, 0) });
        v.CreateBox(h, new(6, 0.2f, 6), def: new() { Surface = new() { OneWayDirection = new(0, 1, 0) } });
        var sphere = v.AddSphere(0.25f, at: new(0, 0, 0)); sphere.LinearVelocity = new(0, 7, 0);
        maxY = 0;
        for (int i = 0; i < 180; i++) { v.Step(1f / 60); maxY = Math.Max(maxY, sphere.Position.Y); }
        Assert.True(maxY > 2); Assert.InRange(sphere.Position.Y, 1.33, 1.37);
    }

    [Fact]
    public void DirectionalFrictionPreservesOnlyLowFrictionMotion()
    {
        var w = new P3.PhysicsWorld3D(new() { SleepEnabled = false });
        var g = w.CreateBody(new() { Type = BodyType.Static, Position = new(0, -0.5, 0) });
        w.CreateBox(g, new(100, 1, 100), def: new() { Surface = new() { FrictionAxis = new(1, 0, 0), LongitudinalFriction = 0, TransverseFriction = 1 } });
        var box = w.AddBox(new(1, 1, 1), at: new(0, 0.5, 0)); box.LinearVelocity = new(2, 0, 2);
        for (int i = 0; i < 120; i++) w.Step(1f / 60);
        Assert.InRange(box.LinearVelocity.X, 1.8f, 2.1f); Assert.InRange(box.LinearVelocity.Z, -0.02f, 0.02f);
    }

    [Fact]
    public void ContactEnumerationAndAudioDataAreReadOnlyAndSnapshotted()
    {
        var w = new P2.PhysicsWorld2D(new() { WorldId = 882, SleepEnabled = false, Events = EventMask.All });
        w.ContactMotionEnabled = true;
        w.AddBox(new(20, 1), at: new(0, -0.5), type: BodyType.Static);
        var ball = w.AddCircle(0.5f, at: new(0, 4.5));
        float impact = 0;
        ball.OnCollisionEnter += c => impact = c.ImpactSpeed;
        for (int i = 0; i < 90; i++) w.Step(1f / 60);
        Assert.InRange(impact, (float)Math.Sqrt(2 * 9.81 * 4) - 0.2f, (float)Math.Sqrt(2 * 9.81 * 4) + 0.2f);
        int count = 0;
        foreach (var c in ball.Contacts) { count++; Assert.True(c.PointCount > 0); Assert.Equal(0, c.Normal.Z); Assert.True(c.NormalImpulse(0) >= 0); }
        Assert.Equal(1, count);
        byte[] saved = w.Snapshot(); var copy = w.Fork(); copy.Restore(saved);
        Assert.Equal(saved, copy.Snapshot());
        for (int i = 0; i < 5; i++) { w.Step(1f / 60); copy.Step(1f / 60); Assert.Equal(w.ComputeHash(), copy.ComputeHash()); }
        Assert.True(w.Events.Length > 0);
        foreach (var e in w.Events) if (e.Kind == EventKind.ContactPersist) { Assert.Equal(impact, e.ImpactSpeed); Assert.InRange(e.SlidingSpeed, 0, 0.001f); }
        ball.Destroy();
        Assert.Throws<StaleHandleException>(() => { foreach (var c in ball.Contacts) { _ = c.PointCount; } });
    }

    [Fact]
    public void HostileSurfacePropertiesAreRejectedAndDefaultWorldIsUnchanged()
    {
        var w = new P2.PhysicsWorld2D(); var body = w.AddBox(new(1, 1)); var shape = body.GetShapes()[0]; byte[] before = w.Snapshot();
        Assert.Throws<ArgumentOutOfRangeException>(() => shape.Surface = new() { Velocity = new(float.NaN, 0, 0) });
        Assert.Throws<ArgumentOutOfRangeException>(() => shape.Surface = new() { OneWayDirection = new(0, 0, 1) });
        Assert.Equal(before, w.Snapshot());
        shape.Surface = new() { Velocity = new(1, 0, 0) }; shape.Surface = default;
        w.Step(1f / 60);
        Assert.Equal(default, shape.Surface);
    }

    [Fact]
    public void TriangleAndSegmentIndicesResolveTheCorrectMaterial()
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = 881, SleepEnabled = false });
        var g = w.CreateBody(new() { Type = BodyType.Static });
        var mesh = w.CreateMesh(g, new Vector3F[] { new(-4, 0, -2), new(0, 0, -2), new(4, 0, -2), new(-4, 0, 2), new(0, 0, 2), new(4, 0, 2) }, new[] { 0, 3, 1, 1, 3, 4, 1, 4, 2, 2, 4, 5 });
        mesh.SetMaterials(new P3.Material3D[] { new(0), new(1) }, new[] { 0, 0, 1, 1 });
        var left = w.AddBox(new(1, 1, 1), at: new(-2, 0.5, 0), material: new P3.Material3D(1));
        var right = w.AddBox(new(1, 1, 1), at: new(2, 0.5, 0), material: new P3.Material3D(1));
        w.Step(1f / 60);
        int count = 0; foreach (var c in left.Contacts) { count++; Assert.Equal(0f, c.Friction); } Assert.True(count > 0);
        count = 0; foreach (var c in right.Contacts) { count++; Assert.Equal(1f, c.Friction); } Assert.True(count > 0);
        var copy = w.Fork(); copy.Restore(w.Snapshot()); w.Step(1f / 60); copy.Step(1f / 60); Assert.Equal(w.Snapshot(), copy.Snapshot());
        var p = new P2.PhysicsWorld2D(new() { SleepEnabled = false }); var floor = p.CreateBody(new() { Type = BodyType.Static });
        p.CreateChain(floor, new Vector2F[] { new(-4, 0), new(0, 0), new(4, 0) }, new P2.Material2D[] { new(0), new(1) }, new[] { 0, 1 });
        var a = p.AddBox(new(1, 1), at: new(-2, 0.5), material: new P2.Material2D(1)); var b = p.AddBox(new(1, 1), at: new(2, 0.5), material: new P2.Material2D(1));
        p.Step(1f / 60);
        count = 0; foreach (var c in a.Contacts) { count++; Assert.Equal(0f, c.Friction); } Assert.True(count > 0);
        count = 0; foreach (var c in b.Contacts) { count++; Assert.Equal(1f, c.Friction); } Assert.True(count > 0);
    }

    [Fact]
    public void OneWaySideEntryDoesNotSnag()
    {
        var p = new P2.PhysicsWorld2D(new() { Gravity = default, SleepEnabled = false }); var floor = p.CreateBody(new() { Type = BodyType.Static, Position = new(0, 1) });
        p.CreateBox(floor, new(4, 0.2f), def: new() { Surface = new() { OneWayDirection = new(0, 1, 0) } });
        var b = p.AddCircle(0.2f, at: new(-3, 1)); b.IsBullet = true; b.LinearVelocity = new(6, -0.1f);
        for (int i = 0; i < 60; i++) p.Step(1f / 60);
        Assert.InRange(b.Position.X, 2.99, 3.01); Assert.InRange(b.Position.Y, 0.8999, 0.9001);
        var w = new P3.PhysicsWorld3D(new() { Gravity = default, SleepEnabled = false }); var g = w.CreateBody(new() { Type = BodyType.Static, Position = new(0, 1, 0) });
        w.CreateBox(g, new(4, 0.2f, 4), def: new() { Surface = new() { OneWayDirection = new(0, 1, 0) } });
        var s = w.AddSphere(0.2f, at: new(-3, 1, 0)); s.LinearVelocity = new(6, -0.1f, 0);
        for (int i = 0; i < 60; i++) w.Step(1f / 60);
        Assert.InRange(s.Position.X, 2.99, 3.01); Assert.InRange(s.Position.Y, 0.8999, 0.9001);
    }

    [Fact]
    public void TorsionalFrictionStopsNormalAxisSpin()
    {
        var w = new P3.PhysicsWorld3D(new() { SleepEnabled = false });
        var g = w.CreateBody(new() { Type = BodyType.Static, Position = new(0, -0.5, 0) });
        w.CreateBox(g, new(100, 1, 100), def: new() { Surface = new() { SpinningFriction = 0.3f } });
        var sphere = w.AddSphere(0.5f, at: new(0, 0.5, 0)); sphere.AngularVelocity = new(0, 5, 0);
        for (int i = 0; i < 120; i++) w.Step(1f / 60);
        Assert.InRange(sphere.AngularVelocity.Y, -0.001f, 0.001f);
    }
}
