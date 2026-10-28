using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Collision2D;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

public class World2DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D NewWorld(int threads = 1, int id = 5)
        => new(new WorldOptions2D { ThreadCount = threads, WorldId = id });

    /// <summary>A static floor with its top at y = 0, a pyramid-ish stack and a few mixed shapes: enough contact variety for determinism checks.</summary>
    internal static PhysicsWorld2D BuildScene(int threads, int id = 5, int kernelWidth = 8)
    {
        var w = NewWorld(threads, id);
        w.Engine.KernelWidth = kernelWidth;
        w.AddBox(new Vector2F(60f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        w.AddBox(new Vector2F(1f, 20f), at: new Vector2D(-20, 10), type: BodyType.Static);
        w.AddBox(new Vector2F(1f, 20f), at: new Vector2D(20, 10), type: BodyType.Static);
        for (int row = 0; row < 8; row++)
        {
            for (int i = 0; i < 8 - row; i++) { w.AddBox(new Vector2F(1f, 1f), at: new Vector2D(-4 + row * 0.5 + i * 1.02, 0.5 + row * 1.0)); }
        }

        for (int i = 0; i < 40; i++)
        {
            double x = -8 + (i % 10) * 1.6, y = 12 + (i / 10) * 1.7;
            switch (i % 4)
            {
                case 0: w.AddCircle(0.4f, new Vector2D(x, y)); break;
                case 1: w.AddCapsule(0.3f, 0.25f, new Vector2D(x, y), 0.3f); break;
                case 2: w.AddBox(new Vector2F(0.8f, 0.6f), new Vector2D(x, y), 0.5f); break;
                default: w.AddPolygon([new Vector2F(0, 0.5f), new Vector2F(-0.47f, 0.15f), new Vector2F(-0.29f, -0.4f), new Vector2F(0.29f, -0.4f), new Vector2F(0.47f, 0.15f)], new Vector2D(x, y)); break;
            }
        }

        return w;
    }

    [Fact]
    public void FreeFallMatchesTheAnalyticAnswer()
    {
        var w = NewWorld();
        var ball = w.AddCircle(0.5f, new Vector2D(0, 100));
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        // Semi-implicit Euler with substeps: v = g t exactly, y = y0 - g t^2 / 2 within (g t dt / 2).
        Assert.Equal(-9.81f, ball.LinearVelocity.Y, 3);
        Assert.InRange(ball.Position.Y, 100 - 0.5 * 9.81 - 0.05, 100 - 0.5 * 9.81 + 0.001);
        Assert.Equal(0.0, ball.Position.X);
    }

    [Fact]
    public void ABoxSettlesOnTheGroundAndFallsAsleep()
    {
        var w = NewWorld();
        w.AddBox(new Vector2F(20f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var box = w.AddBox(new Vector2F(1f, 1f), at: new Vector2D(0, 3), angle: 0.3f);
        for (int i = 0; i < 360; i++) { w.Step(Dt); }
        Assert.InRange(box.Position.Y, 0.49, 0.51);
        Assert.False(box.IsAwake);
        Assert.Equal(0f, box.LinearVelocity.Y);
        Assert.InRange(Math.Abs(box.Angle) % (Math.PI / 2), 0, 0.02);
    }

    [Fact]
    public void ABodyOnAKinematicPlatformStaysAwakeWhileThePlatformMoves()
    {
        var w = NewWorld();
        var platform = w.CreateBody(new BodyDef { Type = BodyType.Kinematic, Position = new Vector2D(0, 0), LinearVelocity = new Vector2F(1f, 0f) });
        w.CreateBox(platform, 50f, 0.5f);
        var box = w.AddBox(new Vector2F(1f, 1f), at: new Vector2D(0, 1.1));
        for (int i = 0; i < 240; i++) { w.Step(Dt); }
        Assert.True(box.IsAwake);
        Assert.True(box.Position.X > 2.0, "the box rides along with the platform");
        platform.LinearVelocity = default;
        for (int i = 0; i < 240; i++) { w.Step(Dt); }
        Assert.False(box.IsAwake);
    }

    [Fact]
    public void ASleepingBodyWakesWhenSomethingLandsOnIt()
    {
        var w = NewWorld();
        w.AddBox(new Vector2F(20f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var low = w.AddBox(new Vector2F(1f, 1f), at: new Vector2D(0, 0.5));
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.False(low.IsAwake);
        var high = w.AddCircle(0.3f, new Vector2D(0.1, 3));
        bool woke = false;
        for (int i = 0; i < 120 && !woke; i++) { w.Step(Dt); woke = low.IsAwake; }
        Assert.True(woke);
        _ = high;
    }

    [Fact]
    public void ACollisionEnterFiresOnceAfterTheStepAndExitWhenTheShapeIsDestroyed()
    {
        var w = NewWorld();
        var ground = w.AddBox(new Vector2F(20f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var ball = w.AddCircle(0.5f, new Vector2D(0, 1.0));
        var enters = new List<Collision>();
        var exits = new List<Collision>();
        ball.OnCollisionEnter += enters.Add;
        ball.OnCollisionExit += exits.Add;
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        Assert.Single(enters);
        Assert.Empty(exits);
        var c = enters[0];
        Assert.True(c.Other == ground && c.Body == ball && c.HasPoint);
        Assert.Equal(-1f, c.Normal.Y, 2);
        Assert.InRange(c.Point.Y, -0.05, 0.05);
        ground.Destroy();
        w.Step(Dt);
        Assert.Single(exits);
        Assert.False(exits[0].Other.IsValid); // the ground is gone by the time the callback runs
        Assert.False(exits[0].HasPoint);
    }

    [Fact]
    public void StayEventsAreOptIn()
    {
        var o = new WorldOptions2D { Events = EventMask.All };
        var w = new PhysicsWorld2D(o);
        w.AddBox(new Vector2F(20f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var ball = w.AddCircle(0.5f, new Vector2D(0, 0.5));
        int stays = 0;
        ball.OnCollisionStay += _ => stays++;
        for (int i = 0; i < 20; i++) { w.Step(Dt); }
        Assert.True(stays >= 15);
    }

    [Fact]
    public void AStaleBodyFailsClearlyAndSlotReuseDoesNotRevive()
    {
        var w = NewWorld();
        var a = w.AddCircle(0.5f, new Vector2D(0, 5));
        a.Destroy();
        Assert.False(a.IsValid);
        var e = Assert.Throws<StaleHandleException>(() => a.Position);
        Assert.Contains("stale", e.Message);
        Assert.Throws<StaleHandleException>(() => a.ApplyImpulse(new Vector2F(1, 0)));
        Assert.Throws<StaleHandleException>(() => a.Destroy());
        var b = w.AddCircle(0.5f, new Vector2D(0, 5));
        Assert.Equal(a.RawHandle.Index, b.RawHandle.Index);
        Assert.True(b.IsValid);
        Assert.Throws<StaleHandleException>(() => a.LinearVelocity);
        Assert.Throws<InvalidHandleException>(() => default(Body).Position);
        var other = NewWorld(id: 6);
        Assert.Throws<ForeignHandleException>(() => other.Core.Resolve(b.RawHandle));
        w.Step(Dt);
    }

    [Fact]
    public void HostileInputIsRejectedWithTypedErrors()
    {
        var w = NewWorld();
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateBody(new BodyDef { Position = new Vector2D(3e6, 0) }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateBody(new BodyDef { Position = new Vector2D(double.NaN, 0) }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateBody(new BodyDef { LinearVelocity = new Vector2F(float.NaN, 0) }));
        var b = w.CreateBody(new BodyDef());
        Assert.Throws<InvalidShapeException>(() => w.CreateCircle(b, -1f));
        Assert.Throws<InvalidShapeException>(() => w.CreatePolygon(b, [new Vector2F(0, 0), new Vector2F(0, 0), new Vector2F(0, 0)]));
        Assert.Throws<InvalidShapeException>(() => w.CreatePolygon(b, [new Vector2F(0, 0), new Vector2F(1, 1), new Vector2F(2, 2)]));
        Assert.Throws<InvalidShapeException>(() => w.CreateBox(b, float.NaN, 1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateCircle(b, 1f, default, new ShapeDef { Density = -1f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => b.LinearVelocity = new Vector2F(float.PositiveInfinity, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Step(0f));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Step(float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => new PhysicsWorld2D(new WorldOptions2D { Substeps = 0 }));
        var s = w.AddBox(new Vector2F(1, 1), type: BodyType.Static);
        Assert.Throws<BodyTypeException>(() => s.LinearVelocity = new Vector2F(1, 0));
        Assert.Throws<BodyTypeException>(() => s.ApplyForce(new Vector2F(1, 0)));
        w.Step(Dt);
    }

    [Fact]
    public void ABodyLeavingTheSupportedRangeIsReportedBeforeAStepChangesAnything()
    {
        var w = NewWorld();
        var other = w.AddCircle(1f, at: new Vector2D(0, 5));
        var runaway = w.AddBox(new Vector2F(1, 1), at: new Vector2D(2000000, 0), type: BodyType.Kinematic);
        runaway.LinearVelocity = new Vector2F(1e7f, 0);
        BodyOutOfBoundsException? caught = null;
        ulong before = 0;
        for (int i = 0; i < 100 && caught == null; i++)
        {
            before = w.ComputeHash();
            try { w.Step(Dt); } catch (BodyOutOfBoundsException e) { caught = e; }
        }

        Assert.NotNull(caught);
        Assert.Equal(runaway.RawHandle, caught!.Body);
        Assert.Equal(before, w.ComputeHash());
        Assert.Throws<BodyOutOfBoundsException>(() => w.Step(Dt));
        w.GetBody(caught.Body).Destroy();
        Assert.False(runaway.IsValid);
        w.Step(Dt);
        Assert.True(other.IsValid);
    }

    [Fact]
    public void AFastBodyDoesNotTunnelThroughAThinWall()
    {
        var w = NewWorld();
        w.Options.Gravity = default;
        w.AddBox(new Vector2F(0.1f, 20f), at: new Vector2D(10, 0), type: BodyType.Static);
        var bullet = w.CreateBody(new BodyDef { Position = new Vector2D(0, 0), LinearVelocity = new Vector2F(300f, 0f), IsBullet = true });
        w.CreateCircle(bullet, 0.1f);
        for (int i = 0; i < 30; i++) { w.Step(Dt); }
        Assert.True(bullet.Position.X < 10.0, "bullet at " + bullet.Position);
    }

    [Fact]
    public void MaterialsMixWithTheDocumentedRulesAndTheSeamReplacesThem()
    {
        var w = NewWorld();
        w.AddBox(new Vector2F(20f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static, material: new Material2D(0.25f, 0.2f));
        w.AddBox(new Vector2F(1f, 1f), at: new Vector2D(0, 0.49), material: new Material2D(1.0f, 0.6f));
        w.Step(Dt);
        var cs = w.Core.Constraints;
        Assert.Equal(1, cs.Count);
        int c = 0;
        while (!cs.IsLive(c)) { c++; }
        var pl = cs.Payload(c);
        Assert.Equal(0.5f, pl[ManifoldLayout.Friction], 5);   // sqrt(0.25 * 1.0)
        Assert.Equal(0.6f, pl[ManifoldLayout.Restitution], 5); // max(0.2, 0.6)

        var w2 = new PhysicsWorld2D(new WorldOptions2D { MaterialMixer = (a, b) => new Material2D(a.Friction + b.Friction, 0f) });
        w2.AddBox(new Vector2F(20f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static, material: new Material2D(0.25f));
        w2.AddBox(new Vector2F(1f, 1f), at: new Vector2D(0, 0.49), material: new Material2D(1.0f));
        w2.Step(Dt);
        int c2 = 0;
        while (!w2.Core.Constraints.IsLive(c2)) { c2++; }
        Assert.Equal(1.25f, w2.Core.Constraints.Payload(c2)[ManifoldLayout.Friction], 5);
    }

    [Fact]
    public void RestitutionBouncesAndRollingResistanceStopsARollingWheel()
    {
        var w = NewWorld();
        w.AddBox(new Vector2F(100f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var ball = w.AddCircle(0.5f, new Vector2D(0, 5), material: new Material2D(0.6f, 0.8f));
        float peak = 0f;
        bool bounced = false;
        for (int i = 0; i < 240; i++)
        {
            w.Step(Dt);
            if (ball.LinearVelocity.Y > 1f) { bounced = true; }
            if (bounced && ball.LinearVelocity.Y > 0f) { peak = Math.Max(peak, ball.LinearVelocity.Y); }
        }

        Assert.True(bounced);
        Assert.InRange(peak, 5f, 9f); // fall speed at impact is about 9.4; 0.8 of that is about 7.5

        var w2 = NewWorld();
        w2.AddBox(new Vector2F(100f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var free = w2.AddCircle(0.5f, new Vector2D(0, 0.5), material: new Material2D(0.6f));
        var stiff = w2.AddCircle(0.5f, new Vector2D(10, 0.5), material: new Material2D(0.6f, 0f, 0.5f));
        free.LinearVelocity = new Vector2F(3, 0); free.AngularVelocity = -6f;
        stiff.LinearVelocity = new Vector2F(3, 0); stiff.AngularVelocity = -6f;
        for (int i = 0; i < 180; i++) { w2.Step(Dt); }
        Assert.True(free.LinearVelocity.X > 2.5f, "a free wheel keeps rolling: " + free.LinearVelocity.X);
        Assert.True(stiff.LinearVelocity.X < 0.5f * free.LinearVelocity.X, "rolling resistance slows the wheel: " + stiff.LinearVelocity.X);
    }

    [Fact]
    public void ForcesAndImpulsesChangeVelocityAsExpected()
    {
        var w = NewWorld();
        w.Options.Gravity = default;
        var b = w.AddBox(new Vector2F(1f, 1f), density: 2f); // mass 2
        b.ApplyImpulse(new Vector2F(4f, 0));
        Assert.Equal(2f, b.LinearVelocity.X, 5);
        b.ApplyForce(new Vector2F(0, 10f));
        w.Step(Dt);
        Assert.Equal(10f * Dt / 2f, b.LinearVelocity.Y, 4);
        b.ApplyImpulseAt(new Vector2F(0, 2f), new Vector2D(0.5, 0));
        Assert.True(b.AngularVelocity > 0f);
    }

    [Fact]
    public void ACompoundBodyKeepsItsOriginAndGetsTheRightCenterOfMass()
    {
        var w = NewWorld();
        w.Options.Gravity = default;
        var b = w.CreateBody(new BodyDef { Position = new Vector2D(5, 5), Angle = 0.5f });
        w.CreateBox(b, 0.5f, 0.5f, new Vector2F(-1, 0));
        w.CreateBox(b, 0.5f, 0.5f, new Vector2F(1, 0));
        w.CreateBox(b, 0.5f, 0.5f, new Vector2F(3, 0)); // centre of mass at x = 1
        Assert.Equal(3f, b.Mass, 4);
        Assert.Equal(5.0, b.Position.X, 5);
        Assert.Equal(5.0, b.Position.Y, 5);
        var com = b.WorldCenterOfMass;
        Assert.Equal(5.0 + Math.Cos(0.5), com.X, 4);
        Assert.Equal(5.0 + Math.Sin(0.5), com.Y, 4);
        w.Step(Dt);
        Assert.Equal(8.5, b.Inertia, 3); // three unit boxes: 3 / 6 plus 4 + 0 + 4 about the centre of mass
    }

    [Fact]
    public void StateHashAndPerStepHashesAreEqualForOneAndManyThreadsAndForEveryKernelWidth()
    {
        var baseline = new List<ulong>();
        var w1 = BuildScene(1);
        for (int i = 0; i < 90; i++) { w1.Step(Dt); baseline.Add(w1.ComputeHash()); }

        foreach (var (threads, width) in new[] { (4, 8), (1, 4), (1, 1), (3, 8) })
        {
            var w = BuildScene(threads, kernelWidth: width);
            for (int i = 0; i < 90; i++)
            {
                w.Step(Dt);
                Assert.True(baseline[i] == w.ComputeHash(), $"threads {threads}, width {width}: diverged at step {i}");
            }
        }
    }

    [Fact]
    public void SnapshotRestoreContinueIsBitIdenticalAndRestoreIntoAFreshWorldMatches()
    {
        var w = BuildScene(2, id: 9);
        w.Options.Events = EventMask.All;
        for (int i = 0; i < 70; i++) { w.Step(Dt); }
        byte[] snap = w.Snapshot();
        ulong h0 = w.ComputeHash();

        var hashes = new List<ulong>();
        var events = new List<string>();
        for (int i = 0; i < 70; i++) { w.Step(Dt); hashes.Add(w.ComputeHash()); foreach (var e in w.Events) { events.Add(e.Kind + ":" + e.BodyA.Index + ":" + e.BodyB.Index); } }
        byte[] end = w.Snapshot();

        w.Restore(snap);
        Assert.Equal(h0, w.ComputeHash());
        Assert.Equal(snap, w.Snapshot());
        var events2 = new List<string>();
        for (int i = 0; i < 70; i++)
        {
            w.Step(Dt);
            Assert.Equal(hashes[i], w.ComputeHash());
            foreach (var e in w.Events) { events2.Add(e.Kind + ":" + e.BodyA.Index + ":" + e.BodyB.Index); }
        }

        Assert.Equal(end, w.Snapshot());
        Assert.Equal(events, events2);

        var fresh = new PhysicsWorld2D(new WorldOptions2D { WorldId = 9 });
        fresh.Restore(snap);
        Assert.Equal(h0, fresh.ComputeHash());
        Assert.Equal(snap, fresh.Snapshot());
        for (int i = 0; i < 70; i++) { fresh.Step(Dt); Assert.Equal(hashes[i], fresh.ComputeHash()); }
        Assert.Equal(end, fresh.Snapshot());
    }

    [Fact]
    public void ARestoreOfDamagedBytesFailsAndLeavesTheWorldAlone()
    {
        var w = BuildScene(1);
        for (int i = 0; i < 30; i++) { w.Step(Dt); }
        byte[] snap = w.Snapshot();
        ulong h = w.ComputeHash();
        var rng = new Random(1234);
        for (int trial = 0; trial < 60; trial++)
        {
            var bad = (byte[])snap.Clone();
            int flips = 1 + trial % 3;
            for (int k = 0; k < flips; k++) { bad[rng.Next(bad.Length)] ^= (byte)(1 << rng.Next(8)); }
            if (trial % 7 == 0) { Array.Resize(ref bad, bad.Length - 1 - rng.Next(40)); }
            try { w.Restore(bad); }
            catch (SnapshotFormatException) { }

            // Whether the damaged snapshot happened to be valid or not, the world must still step.
            w.Restore(snap);
        }

        Assert.Equal(h, w.ComputeHash());
        w.Step(Dt);
    }

    [Fact]
    public void ARestoreWhoseStoredExtentIsSmallerThanTheShapeIsRejectedAndLeavesTheWorldAlone()
    {
        // Fuzzing found snapshots whose stored per-body extent was tiny while the shape was huge: the bounds shortcut trusted it and the next step failed half way.
        var w = NewWorld();
        w.AddBox(new Vector2F(20f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        w.AddBox(new Vector2F(1f, 1f), at: new Vector2D(0, 3));
        for (int i = 0; i < 5; i++) { w.Step(Dt); }
        byte[] snap = w.Snapshot();
        ulong h = w.ComputeHash();

        byte[] needle = new byte[4];
        System.Buffers.Binary.BinaryPrimitives.WriteSingleLittleEndian(needle, float.Sqrt(0.5f));
        byte[] tiny = new byte[4];
        System.Buffers.Binary.BinaryPrimitives.WriteSingleLittleEndian(tiny, 1e-3f);
        var bad = (byte[])snap.Clone();
        int patched = 0;
        for (int i = 0; i + 4 <= bad.Length; i++)
        {
            if (bad.AsSpan(i, 4).SequenceEqual(needle)) { tiny.CopyTo(bad, i); patched++; }
        }

        Assert.True(patched > 0);
        var e = Assert.Throws<SnapshotFormatException>(() => w.Restore(bad));
        Assert.Contains("maximum extent", e.Message);
        Assert.Equal(h, w.ComputeHash());
        w.Step(Dt);
    }

    private static PhysicsWorld2D StackScene()
    {
        var w = NewWorld();
        w.AddBox(new Vector2F(40f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 6; i++) { w.AddBox(new Vector2F(1f, 1f), at: new Vector2D(0.02 * i, 0.5 + 1.001 * i)); }
        w.AddCircle(0.5f, new Vector2D(3, 0.5));
        return w;
    }

    [Fact]
    public void RecycledManifoldsStayCloseToFreshOnesAndAreActuallyUsed()
    {
        var recycled = StackScene();
        var fresh = StackScene();
        fresh.Engine.RecycleManifolds = false;
        int used = 0;
        for (int i = 0; i < 240; i++)
        {
            recycled.Step(Dt); fresh.Step(Dt);
            { used += recycled.Engine.RecycledCount; Assert.Equal(0, fresh.Engine.RecycledCount); }
        }

        Assert.True(used > 0, "no manifold was recycled while the stack settled");
        var pa = recycled.Core.Bodies.Position; var pb = fresh.Core.Bodies.Position;
        double worst = 0;
        for (int b = 0; b < recycled.Core.Bodies.HighWater; b++)
        {
            if (!recycled.Core.Bodies.IsLive(b)) { continue; }
            worst = Math.Max(worst, Math.Max(Math.Abs(pa[2 * b] - pb[2 * b]), Math.Abs(pa[2 * b + 1] - pb[2 * b + 1])));
        }

        Assert.True(worst < 0.01, "recycled and fresh worlds differ by " + worst);
    }

    [Fact]
    public void AMaterialChangeReachesARecycledManifold()
    {
        var w = NewWorld();
        w.AddBox(new Vector2F(40f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var box = w.AddBox(new Vector2F(1f, 1f), at: new Vector2D(0, 0.5));
        for (int i = 0; i < 8; i++) { w.Step(Dt); }
        box.GetShapes()[0].Friction = 0.123f;
        w.Step(Dt);
        Assert.True(w.Engine.RecycledCount > 0, "the resting contact was not recycled");
        var cs = w.Core.Constraints;
        int c = 0;
        while (!cs.IsLive(c)) { c++; }
        Assert.Equal(float.Sqrt(0.123f * 0.6f), cs.Payload(c)[ManifoldLayout.Friction], 4);
    }

    [Fact]
    public void ASnapshotFromBeforeRecyclingOrWithGarbageInTheCacheSlotsStillSteps()
    {
        var w = StackScene();
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        // Zero the cache slots of every contact, as an older snapshot would have them, then fill them with huge finite values (the reader refuses NaN).
        var cs = w.Core.Constraints;
        int[] slots = [2, 7, 8, 11, 14, 18, 20, 23, 26, 32, 33, 34, 35, 36, 37, 38, 39];
        for (int pass = 0; pass < 2; pass++)
        {
            for (int c = 0; c < cs.HighWater; c++)
            {
                if (!cs.IsLive(c)) { continue; }
                var pl = cs.Payload(c);
                foreach (int s in slots) { pl[s] = pass == 0 ? 0f : 1e30f; }
            }

            byte[] snap = w.Snapshot();
            var w2 = new PhysicsWorld2D(new WorldOptions2D { WorldId = 5 });
            w2.Restore(snap);
            for (int i = 0; i < 30; i++) { w2.Step(Dt); }
            var top = w2.Core.Bodies.Position;
            for (int b = 0; b < w2.Core.Bodies.HighWater; b++) { if (w2.Core.Bodies.IsLive(b)) { Assert.True(double.IsFinite(top[2 * b + 1]) && Math.Abs(top[2 * b + 1]) < 20); } }
        }
    }

    [Fact]
    public void ShiftingTheOriginKeepsHostCoordinatesAndTheSimulationRunning()
    {
        var w = NewWorld();
        w.AddBox(new Vector2F(20f, 1f), at: new Vector2D(1000, -0.5), type: BodyType.Static);
        var box = w.AddBox(new Vector2F(1f, 1f), at: new Vector2D(1000, 3));
        for (int i = 0; i < 20; i++) { w.Step(Dt); }
        double before = box.Position.Y;
        w.ShiftOrigin(new Vector2D(1000, 0));
        Assert.Equal(1000.0, w.Origin.X);
        Assert.Equal(before, box.Position.Y);
        Assert.Equal(0.0, box.Position.X, 3);
        for (int i = 0; i < 200; i++) { w.Step(Dt); }
        Assert.InRange(box.Position.Y, 0.49, 0.51);
        Assert.Throws<ArgumentException>(() => w.ShiftOrigin(new Vector2D(0.1, 0)));
    }

    [Fact]
    public void ChainsLetBodiesSlideOverJointsWithoutSnagging()
    {
        var w = NewWorld();
        var pts = new Vector2D[40];
        for (int i = 0; i < pts.Length; i++) { pts[i] = new Vector2D(i * 0.5 - 10, 0.0); }
        w.AddChain(pts, twoSided: true, material: new Material2D(0f));
        var box = w.AddBox(new Vector2F(0.4f, 0.4f), at: new Vector2D(-8, 0.25), material: new Material2D(0f));
        box.LinearVelocity = new Vector2F(6, 0);
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.True(box.LinearVelocity.X > 5.9f, "speed " + box.LinearVelocity.X);
        Assert.InRange(box.Position.Y, 0.15, 0.3);
    }

    [Fact]
    public void FilteringByCategoryMaskGroupAndPredicate()
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { PairFilter = (a, b) => !(a.Body.Type == BodyType.Static && b.Body.Type == BodyType.Static) });
        var floor = w.CreateBody(BodyType.Static, new Vector2D(0, -0.5));
        w.CreateBox(floor, 20f, 0.5f, default, 0f, 0f, new ShapeDef { CategoryBits = 2u });
        var ghost = w.CreateBody(BodyType.Dynamic, new Vector2D(0, 3));
        w.CreateBox(ghost, 0.5f, 0.5f, default, 0f, 0f, new ShapeDef { MaskBits = 4u });   // does not include the floor's category
        var solid = w.CreateBody(BodyType.Dynamic, new Vector2D(3, 3));
        w.CreateBox(solid, 0.5f, 0.5f);
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.True(ghost.Position.Y < -1.0, "falls through: " + ghost.Position.Y);
        Assert.InRange(solid.Position.Y, 0.49, 0.51);
    }
}
