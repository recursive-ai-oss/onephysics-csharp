using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class InterestStream3DTests
{
    private const float Dt = 1f / 60f;

    private sealed class Scene
    {
        public P3.PhysicsWorld3D World = null!;
        public readonly List<P3.Body> Pile = new();
        public P3.Body Pivot;
        public P3.Body[] Chain = Array.Empty<P3.Body>();
        public object? Ragdoll;
    }

    /// <summary>A ground, a pile of boxes spread along x, a ragdoll lying at x = 3 and a horizontal chain that hangs from a pivot at x = -4 and spans -4 to 2.</summary>
    private static Scene Build(int id, bool sleep = false, bool chain = true, bool ragdoll = true, int boxes = 36)
    {
        var s = new Scene { World = new P3.PhysicsWorld3D(new() { WorldId = id, SleepEnabled = sleep }) };
        var w = s.World;
        w.AddBox(new Vector3F(200, 1, 200), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < boxes; i++)
        {
            double x = -15 + 30.0 * i / boxes, z = ((i * 7) % 5) - 2.0, y = 0.5 + (i % 3) * 1.05;
            s.Pile.Add(w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(x, y, z)));
        }

        if (ragdoll) { s.Ragdoll = w.AddRagdoll(P3.RagdollDef.Humanoid, new Vector3D(3, 0.4, 6), QuaternionF.CreateFromAxisAngle(new Vector3F(1, 0, 0), 1.5707964f)); }
        if (chain)
        {
            s.Pivot = w.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), at: new Vector3D(-4, 3, -6), type: BodyType.Static);
            var links = new P3.Body[6];
            var prev = s.Pivot;
            for (int i = 0; i < links.Length; i++)
            {
                links[i] = w.AddBox(new Vector3F(1, 0.2f, 0.2f), at: new Vector3D(-3.5 + i, 3, -6));
                w.CreateJoint(new P3.PointJointDef { BodyA = prev, BodyB = links[i], LocalFrameA = new P3.JointFrame3D(new Vector3F(i == 0 ? 0f : 0.5f, 0, 0), QuaternionF.Identity), LocalFrameB = new P3.JointFrame3D(new Vector3F(-0.5f, 0, 0), QuaternionF.Identity) });
                prev = links[i];
            }

            s.Chain = links;
        }

        return s;
    }

    private static InterestRegion Region(double centre) => new InterestRegion().AddBox(new Vector3D(centre - 6, -5, -12), new Vector3D(centre + 6, 40, 12));

    private static double Centre(int step) => -8.0 + 16.0 * step / 300.0;

    [Fact]
    public void ClientHashEqualsTheAuthoritysAtEveryStepWhileTheInterestMoves()
    {
        var a = Build(8601).World;
        var c = new P3.PhysicsWorld3D(new() { WorldId = 8602 });
        var writer = a.CreateInterestWriter();
        var client = c.CreateInterestClient();
        int entered = 0, left = 0, maxProxies = 0, maxShared = 0, joints = 0;
        for (int step = 0; step < 300; step++)
        {
            a.Step(Dt);
            var region = Region(Centre(step));
            var result = client.Apply(writer.Write(region));
            Assert.Equal(step, result.Sequence);
            entered += result.Entered; left += result.Left; joints += result.JointsAdded + result.JointsRemoved;
            maxProxies = Math.Max(maxProxies, client.ProxyCount); maxShared = Math.Max(maxShared, client.SharedBodyCount);
            Assert.Equal(a.ComputeInterestHash(region), client.ComputeSharedHash());
        }

        Assert.True(entered > 40 && left > 10, "entered " + entered + " left " + left);
        Assert.True(maxProxies > 0 && maxShared > 10);
        Assert.True(joints > 0, "the chain never crossed the region's edge");
    }

    [Fact]
    public void ClientThatStepsAheadAndThenAppliesEndsEqual()
    {
        var a = Build(8603).World;
        var c = new P3.PhysicsWorld3D(new() { WorldId = 8604 });
        var writer = a.CreateInterestWriter();
        var client = c.CreateInterestClient();
        for (int step = 0; step < 240; step++)
        {
            a.Step(Dt);
            var region = Region(Centre(step));
            var msg = writer.Write(region);
            if (step > 0) { int ahead = 1 + step % 6; for (int k = 0; k < ahead; k++) { c.Step(Dt); } }
            client.Apply(msg);
            Assert.Equal(a.ComputeInterestHash(region), client.ComputeSharedHash());
        }
    }

    [Fact]
    public void ProxiesAreKinematicAndTheClientsStepsNeverPushThem()
    {
        var scene = Build(8605, chain: false, ragdoll: false, boxes: 0);
        var a = scene.World;
        // Two boxes side by side on the ground and a third on top of the outer one: with the region's edge between the first two, the others are proxies.
        scene.Pile.Add(a.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(-0.5, 0.5, 0)));
        scene.Pile.Add(a.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0.5, 0.5, 0)));
        scene.Pile.Add(a.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0.5, 1.6, 0)));
        var c = new P3.PhysicsWorld3D(new() { WorldId = 8606 });
        var writer = a.CreateInterestWriter();
        var client = c.CreateInterestClient();
        for (int i = 0; i < 90; i++) { a.Step(Dt); }
        // A region that cuts the pile: boxes beyond its edge that touch inside boxes are proxies.
        var region = new InterestRegion().AddBox(new Vector3D(-20, -5, -12), new Vector3D(0.0, 40, 12));
        client.Apply(writer.Write(region));
        Assert.True(client.ProxyCount > 0);
        var proxies = new List<(P3.Body Body, Vector3D At, Vector3F V)>();
        foreach (var body in scene.Pile)
        {
            if (!client.TryGetLocal(body.Handle, out var local)) { continue; }
            var lb = c.GetBody(local);
            if (client.IsProxy(local))
            {
                Assert.NotEqual(BodyType.Dynamic, lb.Type);
                proxies.Add((lb, lb.Position, lb.LinearVelocity));
            }
        }

        Assert.NotEmpty(proxies);
        for (int k = 0; k < 6; k++) { c.Step(Dt); }
        foreach (var (lb, at, v) in proxies)
        {
            var p = lb.Position;
            Assert.True(Math.Abs(p.X - (at.X + v.X * Dt * 6)) < 1e-4 && Math.Abs(p.Y - (at.Y + v.Y * Dt * 6)) < 1e-4 && Math.Abs(p.Z - (at.Z + v.Z * Dt * 6)) < 1e-4, "a proxy was moved by the client's solver");
        }
    }

    [Fact]
    public void DeltasOfAMostlySleepingPileAreAFewDozenBytes()
    {
        var a = Build(8607, sleep: true, chain: false, ragdoll: false).World;
        var c = new P3.PhysicsWorld3D(new() { WorldId = 8608 });
        var writer = a.CreateInterestWriter();
        var client = c.CreateInterestClient();
        var region = Region(0);
        for (int i = 0; i < 400; i++) { a.Step(Dt); }
        int snapshotBytes = 0, sum = 0, max = 0;
        for (int i = 0; i < 40; i++)
        {
            a.Step(Dt);
            var msg = writer.Write(region);
            client.Apply(msg);
            if (i == 0) { snapshotBytes = msg.Length; } else { sum += msg.Length; max = Math.Max(max, msg.Length); }
        }

        Assert.True(snapshotBytes > 1000);
        Assert.True(max < 100, "largest quiet delta " + max + " bytes (snapshot " + snapshotBytes + ")");
    }

    [Fact]
    public void DeltaBytesFollowWhatChangesInsideTheRegion()
    {
        var a = Build(8609, sleep: true, chain: false, ragdoll: false).World;
        var c = new P3.PhysicsWorld3D(new() { WorldId = 8610 });
        var writer = a.CreateInterestWriter();
        var client = c.CreateInterestClient();
        var region = Region(0);
        for (int i = 0; i < 400; i++) { a.Step(Dt); }
        client.Apply(writer.Write(region));
        a.Step(Dt);
        var quietMessage = writer.Write(region);
        int quiet = quietMessage.Length;
        client.Apply(quietMessage);
        // Wake one box far outside the region: the stream does not change. Drop a box inside it: it does.
        a.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(100, 5, 0));
        a.Step(Dt);
        var outsideMessage = writer.Write(region);
        int outside = outsideMessage.Length;
        client.Apply(outsideMessage);
        var dropped = a.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 6, 8));
        a.Step(Dt);
        int inside = writer.Write(region).Length;
        Assert.True(outside <= quiet + 8, "outside " + outside + " quiet " + quiet);
        Assert.True(inside > quiet + 100, "inside " + inside + " quiet " + quiet);
        Assert.True(dropped.IsValid);
    }

    [Theory]
    [InlineData(0, 700)]
    [InlineData(7, 700)]
    [InlineData(7, 400)]
    [InlineData(3, 1200)]
    public void RegionRestoredIntoAWorldWithOtherContentsContinuesLikeTheSource(int others, int settled)
    {
        // Source: ground, an isolated pile at x = 100 and an unrelated pile at -50. Target: its own boxes first, so every slot differs.
        var s = new P3.PhysicsWorld3D(new() { WorldId = 8611, SleepEnabled = false });
        s.AddBox(new Vector3F(400, 1, 400), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var pile = new List<P3.Body>();
        for (int i = 0; i < 10; i++) { pile.Add(s.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(100 + 0.1 * i, 0.5 + 1.2 * i, 0))); }
        for (int i = 0; i < 12; i++) { s.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(-50 + 1.1 * i, 3 + (i % 3), 2)); }
        for (int i = 0; i < settled; i++) { s.Step(Dt); }
        var region = new InterestRegion().AddBox(new Vector3D(60, -5, -30), new Vector3D(140, 40, 30));
        var bytes = s.SnapshotInterest(region);

        var t = new P3.PhysicsWorld3D(new() { WorldId = 8612, SleepEnabled = false });
        for (int i = 0; i < others; i++) { t.AddSphere(0.5f, at: new Vector3D(-20 + 3 * i, 8, 0)); }
        for (int i = 0; i < settled; i++) { t.Step(Dt); }
        var client = t.RestoreInterest(bytes);
        Assert.Equal(10, client.SharedBodyCount);
        double worst = 0; int firstBad = -1;
        for (int step = 0; step < 240; step++)
        {
            s.Step(Dt); t.Step(Dt);
            foreach (var b in pile)
            {
                Assert.True(client.TryGetLocal(b.Handle, out var local));
                var m = t.GetBody(local);
                worst = Math.Max(worst, Math.Max(Math.Abs(b.Position.X - m.Position.X), Math.Max(Math.Abs(b.Position.Y - m.Position.Y), Math.Abs(b.Position.Z - m.Position.Z))));
                if (worst > 0 && firstBad < 0) { firstBad = step; }
                Assert.True(worst == 0, "first difference at step " + firstBad + " awake " + b.IsAwake + "/" + m.IsAwake + " v " + b.LinearVelocity + " " + m.LinearVelocity + " body " + b.Handle + " worst " + worst);
                Assert.Equal(b.Rotation, m.Rotation);
                Assert.Equal(b.LinearVelocity, m.LinearVelocity);
            }
        }

        Assert.Equal(0.0, worst);
    }

    [Fact]
    public void HostileMessagesAreRefusedWithTypedErrorsAndLeaveTheClientAlone()
    {
        var a = Build(8613).World;
        var writer = a.CreateInterestWriter();
        a.Step(Dt);
        var good = writer.Write(Region(0));
        a.Step(Dt);
        var delta = writer.Write(Region(0));
        var c = new P3.PhysicsWorld3D(new() { WorldId = 8614 });
        var client = c.CreateInterestClient();
        Assert.Throws<InterestStreamException>(() => client.Apply(delta));
        client.Apply(good);
        long hash = (long)c.ComputeHash();
        Assert.Throws<InterestStreamException>(() => client.Apply(good.AsSpan(0, good.Length / 2)));
        Assert.Throws<InterestStreamException>(() => client.Apply(new byte[7]));
        Assert.Equal(hash, (long)c.ComputeHash());
        var flipped = (byte[])delta.Clone();
        for (int i = 0; i < flipped.Length; i += 3)
        {
            var copy = (byte[])delta.Clone();
            copy[i] ^= 0xFF;
            try { client.Apply(copy); }
            catch (InterestStreamException) { }
            catch (BodySetTransferException) { }
            client.Reset();
            client.Apply(good);
        }

        client.Apply(delta);
        Assert.Equal(1, client.Sequence);
        var other = new P3.PhysicsWorld3D(new() { WorldId = 8615 });
        Assert.Throws<InterestStreamException>(() => other.CreateInterestClient().Apply(Array.Empty<byte>()));
    }

    private static InterestRegion Wide() => new InterestRegion().AddBox(new Vector3D(-40, -5, -40), new Vector3D(40, 40, 40));

    [Fact]
    public void DestroyedBodiesLeaveAndReusedSlotsEnterAsNewBodies()
    {
        var scene = Build(8616, chain: false, ragdoll: false, boxes: 8);
        var a = scene.World;
        var c = new P3.PhysicsWorld3D(new() { WorldId = 8617 });
        var writer = a.CreateInterestWriter();
        var client = c.CreateInterestClient();
        for (int i = 0; i < 20; i++) { a.Step(Dt); }
        client.Apply(writer.Write(Wide()));
        var victim = scene.Pile[3];
        var oldHandle = victim.Handle;
        victim.Destroy();
        var fresh = a.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 4, 0));
        Assert.Equal(oldHandle.Index, fresh.Handle.Index);
        a.Step(Dt);
        var result = client.Apply(writer.Write(Wide()));
        Assert.Equal(1, result.Entered); Assert.Equal(1, result.Left);
        Assert.False(client.TryGetLocal(oldHandle, out _));
        Assert.True(client.TryGetLocal(fresh.Handle, out _));
        Assert.Equal(a.ComputeInterestHash(Wide()), client.ComputeSharedHash());
    }

    [Fact]
    public void JointsCreatedAndDestroyedLaterReachTheClient()
    {
        var scene = Build(8618, chain: false, ragdoll: false, boxes: 6);
        var a = scene.World;
        var c = new P3.PhysicsWorld3D(new() { WorldId = 8619 });
        var writer = a.CreateInterestWriter();
        var client = c.CreateInterestClient();
        for (int i = 0; i < 20; i++) { a.Step(Dt); }
        client.Apply(writer.Write(Wide()));
        Assert.Equal(0, client.JointCount);
        var joint = a.CreateJoint(new P3.PointJointDef { BodyA = scene.Pile[1], BodyB = scene.Pile[2], LocalFrameA = new P3.JointFrame3D(new Vector3F(0, 0.5f, 0), QuaternionF.Identity), LocalFrameB = new P3.JointFrame3D(new Vector3F(0, 0.5f, 0), QuaternionF.Identity) });
        a.Step(Dt);
        var added = client.Apply(writer.Write(Wide()));
        Assert.Equal(1, added.JointsAdded); Assert.Equal(1, client.JointCount);
        a.Step(Dt);
        Assert.Equal(0, client.Apply(writer.Write(Wide())).JointsAdded);
        joint.Destroy();
        a.Step(Dt);
        Assert.Equal(1, client.Apply(writer.Write(Wide())).JointsRemoved);
        Assert.Equal(0, client.JointCount);
        Assert.Equal(a.ComputeInterestHash(Wide()), client.ComputeSharedHash());
    }

    [Fact]
    public void ANamedStaticBodyTakesItsRoleInPlaceAndASecondSnapshotReplacesWhatTheClientHeld()
    {
        var scene = Build(8620, chain: false, ragdoll: false, boxes: 6);
        var a = scene.World;
        var c = new P3.PhysicsWorld3D(new() { WorldId = 8621 });
        var writer = a.CreateInterestWriter();
        var client = c.CreateInterestClient();
        var region = Wide();
        for (int i = 0; i < 20; i++) { a.Step(Dt); }
        client.Apply(writer.Write(region));
        int before = c.BodyCount;
        Assert.True(client.ProxyCount >= 1);
        var ground = default(BodyHandle);
        foreach (var body in a.Bodies) { if (body.Type == BodyType.Static) { ground = body.Handle; break; } }
        region.AddBody(ground);
        a.Step(Dt);
        var result = client.Apply(writer.Write(region));
        Assert.Equal(0, result.Entered); Assert.Equal(0, result.Left);
        Assert.Equal(before, c.BodyCount);
        Assert.Equal(a.ComputeInterestHash(region), client.ComputeSharedHash());
        // A new snapshot makes the client forget and rebuild.
        writer.Reset();
        client.TryGetLocal(scene.Pile[0].Handle, out var oldLocal);
        client.Apply(writer.Write(region));
        Assert.Throws<StaleHandleException>(() => c.GetBody(oldLocal));
        Assert.Equal(before, c.BodyCount);
        Assert.Equal(a.ComputeInterestHash(region), client.ComputeSharedHash());
        client.Reset();
        Assert.Equal(0, c.BodyCount);
    }

    [Fact]
    public void WhatAPacketCannotCarryIsRefusedAndTheWriterStaysUsable()
    {
        var scene = Build(8622, chain: false, ragdoll: false, boxes: 4);
        var a = scene.World;
        var writer = a.CreateInterestWriter();
        var player = a.AddCharacter(new Vector3D(2, 1, 5), P3.CharacterDef.Humanoid);
        a.Step(Dt);
        var ex = Assert.Throws<BodySetTransferException>(() => writer.Write(Wide()));
        Assert.Contains("character", ex.Message);
        Assert.Equal(0, writer.NextSequence);
        player.Destroy();
        a.Step(Dt);
        var c = new P3.PhysicsWorld3D(new() { WorldId = 8623 });
        Assert.Equal(0, c.CreateInterestClient().Apply(writer.Write(Wide())).Sequence);
    }
}
