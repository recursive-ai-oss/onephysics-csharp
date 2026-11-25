using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OneMath;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class BodySetTransfer2DTests
{
    private const float Dt = 1f / 60f;

    private static P2.PhysicsWorld2D World(int id, bool ground = true, bool sleep = false)
    {
        var w = new P2.PhysicsWorld2D(new() { WorldId = id, SleepEnabled = sleep });
        if (ground) { w.AddBox(new Vector2F(60, 1), at: new Vector2D(0, -0.5), type: BodyType.Static); }
        return w;
    }

    private static P2.Body[] Stack(P2.PhysicsWorld2D w, int n)
    {
        var list = new List<P2.Body>();
        for (int i = 0; i < n; i++) { list.Add(w.AddBox(new Vector2F(1, 1), at: new Vector2D(0.02 * i, 0.5 + 1.001 * i))); }
        return list.ToArray();
    }

    private static double Gap(P2.Body a, P2.Body b)
    {
        var p = a.Position; var q = b.Position;
        double d = Math.Sqrt((p.X - q.X) * (p.X - q.X) + (p.Y - q.Y) * (p.Y - q.Y));
        return Math.Max(d, Math.Abs(a.Angle - b.Angle));
    }

    private static double Follow(P2.PhysicsWorld2D a, P2.Body[] src, P2.PhysicsWorld2D b, P2.Body[] copy, int steps)
    {
        double worst = 0;
        for (int step = 0; step < steps; step++)
        {
            a.Step(Dt); b.Step(Dt);
            for (int i = 0; i < src.Length; i++) { worst = Math.Max(worst, Gap(src[i], copy[i])); }
        }

        return worst;
    }

    private static P2.Body[] Bodies(P2.PhysicsWorld2D w, BodySetHandleMap map) => map.NewBodies.ToArray().Select(h => w.GetBody(h)).ToArray();

    [Theory]
    [InlineData(1)]
    [InlineData(10)]
    [InlineData(45)]
    [InlineData(300)]
    public void RestingStackContinuesLikeTheSourceInAWorldWithTheSameGround(int before)
    {
        var a = World(9201);
        var boxes = Stack(a, 6);
        for (int i = 0; i < before; i++) { a.Step(Dt); }
        var packet = a.ExtractBodies(boxes, new BodySetOptions { Move = false });
        Assert.Equal(2, packet.Dimension);
        var b = World(9202);
        var map = b.InsertBodies(packet);
        Assert.Equal(0.0, Follow(a, boxes, b, Bodies(b, map), 120));
    }

    [Fact]
    public void SleepingStackArrivesAsleepAndWakesOnATouch()
    {
        var a = World(9203, sleep: true);
        var boxes = Stack(a, 4);
        for (int i = 0; i < 400; i++) { a.Step(Dt); }
        Assert.All(boxes, x => Assert.False(x.IsAwake));
        var packet = a.ExtractBodies(boxes);
        var b = World(9204, sleep: true);
        var copies = Bodies(b, b.InsertBodies(packet));
        Assert.All(copies, x => Assert.False(x.IsAwake));
        b.Step(Dt);
        Assert.All(copies, x => Assert.False(x.IsAwake));
        b.AddBox(new Vector2F(1, 1), at: new Vector2D(0.1, 8)).LinearVelocity = new Vector2F(0, -8);
        for (int i = 0; i < 90; i++) { b.Step(Dt); }
        Assert.True(copies[3].IsAwake);
    }

    [Fact]
    public void LyingRagdollContinuesLikeTheSource()
    {
        var a = World(9205);
        a.AddRagdoll(P2.RagdollDef.Humanoid, new Vector2D(0, 1.2), 1.5707964f);
        for (int i = 0; i < 360; i++) { a.Step(Dt); }
        P2.Body part = default;
        foreach (var x in a.Bodies) { if (x.Type == BodyType.Dynamic) { part = x; break; } }
        var packet = a.ExtractBodies(new P2.Body[] { part }, new BodySetOptions { Move = false });
        Assert.Equal(P2.RagdollDef.Humanoid.Parts.Length, packet.BodyCount);
        var b = World(9206);
        var map = b.InsertBodies(packet);
        Assert.Single(map.NewRagdolls.ToArray());
        Assert.True(b.IsRagdollAlive(map.NewRagdolls[0]));
        var src = map.OldBodies.ToArray().Select(h => a.GetBody(h)).ToArray();
        Assert.Equal(0.0, Follow(a, src, b, Bodies(b, map), 120));
    }

    private static P2.Body Pendulum(P2.PhysicsWorld2D w, out P2.Body pivot)
    {
        pivot = w.AddBox(new Vector2F(0.4f, 0.4f), at: new Vector2D(0, 8), type: BodyType.Static);
        var arm = w.AddBox(new Vector2F(0.2f, 2f), at: new Vector2D(0, 7));
        arm.AngularVelocity = 2f;
        w.CreateJoint(new P2.RevoluteJointDef { BodyA = pivot, BodyB = arm, LocalAnchorA = default, LocalAnchorB = new Vector2F(0, 1) });
        return arm;
    }

    [Fact]
    public void BoundaryPoliciesInTwoDimensions()
    {
        var a = World(9207, ground: false);
        var arm = Pendulum(a, out var pivot);
        a.Step(Dt);
        ulong hash = a.ComputeHash();
        var ex = Assert.Throws<BodySetTransferException>(() => a.ExtractBodies(new[] { arm }));
        Assert.Contains(pivot.RawHandle.ToString(), ex.Message);
        Assert.Equal(hash, a.ComputeHash());
        var broken = a.ExtractBodies(new[] { arm }, new BodySetOptions { Move = false, Boundary = BoundaryPolicy.Break });
        Assert.Equal(0, broken.JointCount);
        var held = a.ExtractBodies(new[] { arm }, new BodySetOptions { Move = false, Boundary = BoundaryPolicy.HoldAsAnchor });
        Assert.Equal(1, held.JointCount);
        Assert.Equal(1, held.AnchorCount);
        var b = World(9208, ground: false);
        var map = b.InsertBodies(held);
        Assert.Equal(1, b.JointCount);
        var copy = b.GetBody(map.NewBodies[0]);
        double worst = 0;
        for (int i = 0; i < 120; i++) { a.Step(Dt); b.Step(Dt); worst = Math.Max(worst, Gap(arm, copy)); }
        Assert.Equal(0.0, worst);
    }

    [Fact]
    public void TwoPeersThatInsertTheSamePacketStayEqual()
    {
        var a = World(9209);
        Stack(a, 4);
        a.AddRagdoll(P2.RagdollDef.Humanoid, new Vector2D(6, 2));
        for (int i = 0; i < 40; i++) { a.Step(Dt); }
        var all = new List<P2.Body>();
        foreach (var x in a.Bodies) { if (x.Type == BodyType.Dynamic) { all.Add(x); } }
        var packet = a.ExtractBodies(all.ToArray(), new BodySetOptions { Move = false });
        var again = BodySetPacket.Read(packet.ToArray());
        Assert.Equal(packet.ToArray(), again.ToArray());
        var b = World(9210); var c = World(9210);
        foreach (var w in new[] { b, c }) { var t0 = w.AddCircle(0.3f, at: new Vector2D(20, 3)); w.AddCircle(0.3f, at: new Vector2D(22, 3)); t0.Destroy(); }
        b.InsertBodies(packet, new Vector2D(10, 0));
        c.InsertBodies(again, new Vector2D(10, 0));
        Assert.Equal(b.Snapshot(), c.Snapshot());
        for (int i = 0; i < 90; i++) { b.Step(Dt); c.Step(Dt); Assert.Equal(b.ComputeHash(), c.ComputeHash()); }
    }

    [Fact]
    public void ARotatedPortalKeepsSpeedAndSpinInThePortalFrame()
    {
        var a = World(9211, ground: false);
        var ball = a.AddBox(new Vector2F(0.5f, 0.5f), at: new Vector2D(3, 1));
        ball.LinearVelocity = new Vector2F(4, 3);
        ball.AngularVelocity = 2f;
        var packet = a.ExtractBodies(new[] { ball });
        var b = World(9212, ground: false);
        b.Options.Gravity = default;
        var copy = b.GetBody(b.InsertBodies(packet, new Vector2D(100, -50), 1.5707964f).NewBodies[0]);
        // A quarter turn sends (x, y) to (-y, x).
        var v = copy.LinearVelocity;
        Assert.Equal(-3.0, v.X, 4); Assert.Equal(4.0, v.Y, 4);
        Assert.Equal(5.0, Math.Sqrt(v.X * v.X + v.Y * v.Y), 4);
        Assert.Equal(2.0, copy.AngularVelocity, 5);
        var p = copy.Position;
        Assert.Equal(100.0 - 1.0, p.X, 4); Assert.Equal(-50.0 + 3.0, p.Y, 4);
        Assert.Equal(1.5707964, copy.Angle, 4);
        for (int i = 0; i < 60; i++) { b.Step(Dt); }
        Assert.Equal(100.0 - 1.0 - 3.0, copy.Position.X, 3); Assert.Equal(-50.0 + 3.0 + 4.0, copy.Position.Y, 3);
    }

    [Fact]
    public void PacketsOfAnotherDimensionAreRefusedAndHostileBytesAreTypedErrors()
    {
        var a = World(9213);
        var boxes = Stack(a, 3);
        a.AddRagdoll(P2.RagdollDef.Humanoid, new Vector2D(6, 1.2));
        for (int i = 0; i < 20; i++) { a.Step(Dt); }
        var all = new List<P2.Body>();
        foreach (var x in a.Bodies) { if (x.Type == BodyType.Dynamic) { all.Add(x); } }
        var bytes = a.ExtractBodies(all.ToArray(), new BodySetOptions { Move = false }).ToArray();
        Assert.Throws<BodySetTransferException>(() => new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D().InsertBodies(BodySetPacket.Read(bytes)));
        for (int n = 0; n < bytes.Length; n += Math.Max(1, n / 64 + 1)) { int len = n; Assert.Throws<BodySetPacketException>(() => BodySetPacket.Read(bytes.AsSpan(0, len))); }
        var rng = new Random(11);
        int rejected = 0;
        for (int i = 0; i < 1500; i++)
        {
            var copy = (byte[])bytes.Clone();
            int flips = 1 + rng.Next(3);
            for (int k = 0; k < flips; k++) { copy[rng.Next(copy.Length)] ^= (byte)(1 << rng.Next(8)); }
            try
            {
                var p = BodySetPacket.Read(copy);
                try { World(9214).InsertBodies(p); } catch (PhysicsException) { } catch (ArgumentException) { }
            }
            catch (BodySetPacketException) { rejected++; }
        }

        Assert.True(rejected > 100, "rejected " + rejected);
    }
}
