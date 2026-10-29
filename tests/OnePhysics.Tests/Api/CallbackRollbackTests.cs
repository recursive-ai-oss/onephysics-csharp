using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Tests.Core;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Api;

/// <summary>Callbacks across restore and resimulation: handlers belong to handles, and a scope can mute them without touching polled events.</summary>
[Collection("Allocation")]
public class CallbackRollbackTests
{
    private const float Dt = 1f / 60f;

    private static P2.PhysicsWorld2D Scene2D(out P2.Body ball)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 3 });
        w.AddBox(new Vector2F(10, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        ball = w.AddCircle(0.5f, at: new Vector2D(0, 2));
        return w;
    }

    private static P3.PhysicsWorld3D Scene3D(out P3.Body ball)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 3 });
        w.AddBox(new Vector3F(10, 1, 10), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        ball = w.AddSphere(0.5f, at: new Vector3D(0, 2, 0));
        return w;
    }

    [Fact]
    public void ResimulationUnderSuppressCallbacksRaisesOneLandingAndKeepsPolledEvents2D()
    {
        var reference = Scene2D(out _);
        for (int i = 0; i < 90; i++) { reference.Step(Dt); }

        var world = Scene2D(out var ball);
        int sounds = 0;
        ball.OnCollisionEnter += _ => sounds++;
        var snap = world.Snapshot();
        for (int i = 0; i < 60; i++) { world.Step(Dt); }
        Assert.Equal(1, sounds);

        world.Restore(snap);
        int polled = 0;
        using (world.SuppressCallbacks())
        {
            for (int i = 0; i < 60; i++) { world.Step(Dt); polled += world.GetEvents(EventKind.ContactBegin).Length; }
        }

        Assert.Equal(1, polled);
        Assert.Equal(1, sounds);
        for (int i = 0; i < 30; i++) { world.Step(Dt); }
        Assert.Equal(1, sounds);
        Assert.Equal(reference.ComputeHash(), world.ComputeHash());
    }

    [Fact]
    public void ResimulationUnderSuppressCallbacksRaisesOneLandingAndKeepsPolledEvents3D()
    {
        var reference = Scene3D(out _);
        for (int i = 0; i < 90; i++) { reference.Step(Dt); }

        var world = Scene3D(out var ball);
        int sounds = 0;
        ball.OnCollisionEnter += _ => sounds++;
        var snap = world.Snapshot();
        for (int i = 0; i < 60; i++) { world.Step(Dt); }
        Assert.Equal(1, sounds);

        world.Restore(snap);
        int polled = 0;
        using (world.SuppressCallbacks())
        {
            for (int i = 0; i < 60; i++) { world.Step(Dt); polled += world.GetEvents(EventKind.ContactBegin).Length; }
        }

        Assert.Equal(1, polled);
        Assert.Equal(1, sounds);
        for (int i = 0; i < 30; i++) { world.Step(Dt); }
        Assert.Equal(1, sounds);
        Assert.Equal(reference.ComputeHash(), world.ComputeHash());
    }

    [Fact]
    public void CallbacksRunAgainWhenTheScopeEndsAndScopesNest2D()
    {
        var w = new P2.PhysicsWorld2D();
        w.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var a = w.AddCircle(0.5f, at: new Vector2D(0, 1.2));
        var b = w.AddCircle(0.5f, at: new Vector2D(5, 6));
        int na = 0, nb = 0;
        a.OnCollisionEnter += _ => na++;
        b.OnCollisionEnter += _ => nb++;
        using (w.SuppressCallbacks())
        {
            using (w.SuppressCallbacks()) { for (int i = 0; i < 20; i++) { w.Step(Dt); } }
            for (int i = 0; i < 20; i++) { w.Step(Dt); }
        }

        Assert.Equal(0, na);
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.Equal(1, nb);
    }

    [Fact]
    public void SuppressedControllerAndJointCallbacksStaySilent2D()
    {
        var w = new P2.PhysicsWorld2D();
        var ground = w.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        w.AddBox(new Vector2F(1, 3), at: new Vector2D(2, 1.5), type: BodyType.Static);
        var ch = w.AddCharacter(new Vector2D(0, 1));
        int hits = 0;
        ch.OnControllerColliderHit += _ => hits++;
        using (w.SuppressCallbacks())
        {
            for (int i = 0; i < 60; i++) { ch.Move(new Vector2F(0.05f, -0.01f), Dt); w.Step(Dt); }
        }

        Assert.Equal(0, hits);
        for (int i = 0; i < 20; i++) { ch.Move(new Vector2F(0.05f, -0.01f), Dt); w.Step(Dt); }
        Assert.True(hits > 0);

        var bob = w.AddCircle(0.3f, at: new Vector2D(-5, 3));
        var hinge = w.AddHinge(ground, bob, new Vector2D(-6, 3));
        hinge.BreakForce = 1f;
        int broke = 0;
        hinge.OnJointBreak += _ => broke++;
        byte[] snap = w.Snapshot();
        using (w.SuppressCallbacks())
        {
            for (int i = 0; i < 30; i++) { w.Step(Dt); }
        }

        Assert.Equal(0, broke);
        Assert.False(hinge.IsValid);

        // The joint comes back with its handler, so a normal resimulation raises the break.
        w.Restore(snap);
        Assert.True(hinge.IsValid);
        for (int i = 0; i < 30; i++) { w.Step(Dt); }
        Assert.Equal(1, broke);
    }

    [Fact]
    public void HandlersComeBackWhenARestoreBringsTheSameHandleBack2D()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 4 });
        w.AddBox(new Vector2F(10, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var coin = w.AddCircle(0.5f, at: new Vector2D(0, 3));
        int hits = 0, sleeps = 0;
        coin.OnCollisionEnter += _ => hits++;
        coin.OnSleep += _ => sleeps++;
        byte[] snap = w.Snapshot();
        coin.Destroy();
        for (int i = 0; i < 10; i++) { w.Step(Dt); }
        w.Restore(snap);
        Assert.True(coin.IsValid);
        for (int i = 0; i < 400; i++) { w.Step(Dt); }
        Assert.Equal(1, hits);
        Assert.Equal(1, sleeps);

        // A handler added after the snapshot survives the restore as well (handlers belong to the handle).
        var w2 = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 4 });
        w2.AddBox(new Vector2F(10, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var b2 = w2.AddCircle(0.5f, at: new Vector2D(0, 3));
        byte[] s2 = w2.Snapshot();
        int late = 0;
        b2.OnCollisionEnter += _ => late++;
        w2.Restore(s2);
        for (int i = 0; i < 120; i++) { w2.Step(Dt); }
        Assert.Equal(1, late);
    }

    [Fact]
    public void HandlersComeBackWhenARestoreBringsTheSameHandleBack3D()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 4 });
        w.AddBox(new Vector3F(10, 1, 10), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var coin = w.AddSphere(0.5f, at: new Vector3D(0, 3, 0));
        int hits = 0;
        coin.OnCollisionEnter += _ => hits++;
        byte[] snap = w.Snapshot();
        coin.Destroy();
        for (int i = 0; i < 10; i++) { w.Step(Dt); }
        w.Restore(snap);
        Assert.True(coin.IsValid);
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.Equal(1, hits);
    }

    [Fact]
    public void RestoringTwiceToDifferentDepthsKeepsTheHandlers2D()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 4 });
        w.AddBox(new Vector2F(10, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var coin = w.AddCircle(0.5f, at: new Vector2D(0, 30));
        int hits = 0;
        coin.OnCollisionEnter += _ => hits++;
        byte[] early = w.Snapshot();
        for (int i = 0; i < 5; i++) { w.Step(Dt); }
        byte[] late = w.Snapshot();
        coin.Destroy();
        w.Step(Dt);
        w.Restore(late);       // the coin is back, then gone again, then a deeper restore
        coin.Destroy();
        w.Step(Dt);
        w.Restore(early);
        Assert.True(coin.IsValid);
        for (int i = 0; i < 400; i++) { w.Step(Dt); }
        Assert.Equal(1, hits);
    }

    [Fact]
    public void HandlersOfDestroyedBodiesAreReleasedAndNeverReachAReusedSlot2D()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 4 });
        w.AddBox(new Vector2F(10, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var weak = Subscribe(w, out var body);
        body.Destroy();
        GC.Collect();
        GC.WaitForPendingFinalizers();
        GC.Collect();
        // A world that never took a snapshot has nothing to restore, so the handler is let go at once.
        Assert.False(weak.IsAlive);

        // A world that did take one remembers it, but never hands it to a different body that reuses the slot.
        var w2 = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 4 });
        w2.AddBox(new Vector2F(10, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var doomed = w2.AddCircle(0.5f, at: new Vector2D(0, 3));
        int count = 0;
        doomed.OnCollisionEnter += _ => count++;
        _ = w2.Snapshot();
        doomed.Destroy();
        var reuse = w2.AddCircle(0.5f, at: new Vector2D(0, 3));
        Assert.Equal(doomed.RawHandle.Index, reuse.RawHandle.Index);
        for (int i = 0; i < 120; i++) { w2.Step(Dt); }
        Assert.Equal(0, count);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static WeakReference Subscribe(P2.PhysicsWorld2D w, out P2.Body body)
    {
        body = w.AddCircle(0.5f, at: new Vector2D(0, 3));
        var token = new object();
        body.OnCollisionEnter += _ => GC.KeepAlive(token);
        return new WeakReference(token);
    }

    [Fact]
    public void RememberedHandlersAreBounded2D()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 4 });
        _ = w.Snapshot();
        var weak = CreateAndDestroy(w, 1100);
        GC.Collect();
        GC.WaitForPendingFinalizers();
        GC.Collect();
        Assert.False(weak.IsAlive);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static WeakReference CreateAndDestroy(P2.PhysicsWorld2D w, int count)
    {
        WeakReference first = null!;
        for (int i = 0; i < count; i++)
        {
            var b = w.AddCircle(0.5f, at: new Vector2D(0, 3));
            var token = new object();
            if (i == 0) { first = new WeakReference(token); }
            b.OnCollisionEnter += _ => GC.KeepAlive(token);
            b.Destroy();
        }

        return first;
    }

    [Fact]
    public void SuppressCallbacksAndHandlersAllocateNothingInSteadyState()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 8 });
        w.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        int hits = 0;
        for (int i = 0; i < 20; i++)
        {
            var b = w.AddCircle(0.3f, at: new Vector2D(i * 0.5 - 5, 2 + i * 0.1));
            b.OnCollisionEnter += _ => hits++;
            b.OnCollisionStay += _ => hits++;
        }

        w.Warmup();
        byte[] snap = w.Snapshot();
        for (int i = 0; i < 200; i++) { w.Step(Dt); }
        w.Restore(snap);
        for (int i = 0; i < 20; i++) { w.Step(Dt); }

        AllocMeter.Settle();
        long bytes = 0;
        Action step = () => w.Step(Dt);
        for (int i = 0; i < 100; i++) { bytes += AllocMeter.Of(step); }
        using (w.SuppressCallbacks())
        {
            for (int i = 0; i < 100; i++) { bytes += AllocMeter.Of(step); }
        }

        Assert.Equal(0, bytes);
        Assert.True(hits > 0);

        var w3 = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 8 });
        w3.AddBox(new Vector3F(20, 1, 20), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 10; i++)
        {
            var b = w3.AddSphere(0.3f, at: new Vector3D(i * 0.5 - 5, 2 + i * 0.1, 0));
            b.OnCollisionEnter += _ => hits++;
        }

        w3.Warmup();
        for (int i = 0; i < 100; i++) { w3.Step(Dt); }
        using (w3.SuppressCallbacks()) { }
        AllocMeter.Settle();
        bytes = 0;
        Action step3 = () => w3.Step(Dt);
        using (w3.SuppressCallbacks())
        {
            for (int i = 0; i < 100; i++) { bytes += AllocMeter.Of(step3); }
        }

        for (int i = 0; i < 100; i++) { bytes += AllocMeter.Of(step3); }
        Assert.Equal(0, bytes);
    }
}
