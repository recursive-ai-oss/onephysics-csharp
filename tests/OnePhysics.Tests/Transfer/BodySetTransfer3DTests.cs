using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using System.IO;
using RecursiveAI.OnePhysics.Replay;
using Xunit;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class BodySetTransfer3DTests
{
    private const float Dt = 1f / 60f;

    /// <summary>How far a transferred set may stray from the source world's own continuation over the next 120 steps, in metres. The stack and the ragdoll arrive bit for bit: the limit is zero because the packet carries the contact warm start, the joint state and the solve order.</summary>
    private const double PopLimit = 0.0;

    private static P3.PhysicsWorld3D World(int id, bool ground = true, bool sleep = false)
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = id, SleepEnabled = sleep });
        if (ground) { w.AddBox(new Vector3F(60, 1, 60), at: new Vector3D(0, -0.5, 0), type: BodyType.Static); }
        return w;
    }

    private static P3.Body[] Stack(P3.PhysicsWorld3D w, int n)
    {
        var list = new List<P3.Body>();
        for (int i = 0; i < n; i++) { list.Add(w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0.02 * i, 0.5 + 1.001 * i, 0))); }
        return list.ToArray();
    }

    private static double Gap(P3.Body a, P3.Body b)
    {
        var p = a.Position; var q = b.Position;
        double d = Math.Sqrt((p.X - q.X) * (p.X - q.X) + (p.Y - q.Y) * (p.Y - q.Y) + (p.Z - q.Z) * (p.Z - q.Z));
        var r = a.Rotation; var s = b.Rotation;
        // Sine of half the angle between the two orientations; exactly zero for equal quaternions whatever their float norm.
        double dot = (double)r.X * s.X + (double)r.Y * s.Y + (double)r.Z * s.Z + (double)r.W * s.W;
        double nr = (double)r.X * r.X + (double)r.Y * r.Y + (double)r.Z * r.Z + (double)r.W * r.W, ns = (double)s.X * s.X + (double)s.Y * s.Y + (double)s.Z * s.Z + (double)s.W * s.W;
        return Math.Max(d, Math.Sqrt(Math.Max(0.0, 1.0 - dot * dot / (nr * ns))));
    }

    private static double Follow(P3.PhysicsWorld3D a, P3.Body[] src, P3.PhysicsWorld3D b, P3.Body[] copy, int steps)
    {
        double worst = 0;
        for (int step = 0; step < steps; step++)
        {
            a.Step(Dt); b.Step(Dt);
            for (int i = 0; i < src.Length; i++) { worst = Math.Max(worst, Gap(src[i], copy[i])); }
        }

        return worst;
    }

    private static P3.Body[] Bodies(P3.PhysicsWorld3D w, BodySetHandleMap map)
    {
        var r = new P3.Body[map.NewBodies.Length];
        for (int i = 0; i < r.Length; i++) { r[i] = w.GetBody(map.NewBodies[i]); }
        return r;
    }

    [Theory]
    [InlineData(1)]
    [InlineData(10)]
    [InlineData(45)]
    [InlineData(300)]
    public void RestingStackContinuesLikeTheSourceInAWorldWithTheSameGround(int before)
    {
        var a = World(9101);
        var boxes = Stack(a, 5);
        for (int i = 0; i < before; i++) { a.Step(Dt); }
        var packet = a.ExtractBodies(boxes, new BodySetOptions { Move = false });
        Assert.Equal(5, packet.BodyCount);
        var b = World(9102);
        var map = b.InsertBodies(packet);
        double worst = Follow(a, boxes, b, Bodies(b, map), 120);
        Assert.True(worst <= PopLimit, "worst gap " + worst);
    }

    [Fact]
    public void SleepingStackArrivesAsleep()
    {
        var a = World(9103, sleep: true);
        var boxes = Stack(a, 4);
        for (int i = 0; i < 400; i++) { a.Step(Dt); }
        Assert.All(boxes, x => Assert.False(x.IsAwake));
        var packet = a.ExtractBodies(boxes);
        Assert.All(boxes, x => Assert.False(x.IsValid));
        var b = World(9104, sleep: true);
        var map = b.InsertBodies(packet);
        var copies = Bodies(b, map);
        Assert.All(copies, x => Assert.False(x.IsAwake));
        b.Step(Dt);
        Assert.All(copies, x => Assert.False(x.IsAwake));
    }

    [Fact]
    public void LyingRagdollContinuesLikeTheSource()
    {
        var a = World(9105);
        var rag = a.AddRagdoll(P3.RagdollDef.Humanoid, new Vector3D(0, 0.4, 0), QuaternionF.CreateFromAxisAngle(new Vector3F(1, 0, 0), 1.5707964f));
        for (int i = 0; i < 360; i++) { a.Step(Dt); }
        // Any one part names the whole ragdoll.
        P3.Body part = default;
        foreach (var x in a.Bodies) { if (x.Type == BodyType.Dynamic) { part = x; break; } }
        var packet = a.ExtractBodies(new P3.Body[] { part }, new BodySetOptions { Move = false });
        Assert.Equal(P3.RagdollDef.Humanoid.Parts.Length, packet.BodyCount);
        var b = World(9106);
        var map = b.InsertBodies(packet);
        Assert.Single(map.NewRagdolls.ToArray());
        Assert.True(b.IsRagdollAlive(map.NewRagdolls[0]));
        var src = packet.BodyCount == 0 ? Array.Empty<P3.Body>() : map.OldBodies.ToArray().Select(h => a.GetBody(h)).ToArray();
        double worst = Follow(a, src, b, Bodies(b, map), 120);
        Assert.True(worst <= PopLimit, "worst gap " + worst);
    }

    private static P3.Body Pendulum(P3.PhysicsWorld3D w, out P3.Body pivot)
    {
        pivot = w.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), at: new Vector3D(0, 8, 0), type: BodyType.Static);
        var arm = w.AddBox(new Vector3F(0.2f, 2f, 0.2f), at: new Vector3D(0, 7, 0));
        arm.AngularVelocity = new Vector3F(2, 0, 0);
        w.CreateJoint(new P3.HingeJointDef { BodyA = pivot, BodyB = arm, LocalFrameA = default, LocalFrameB = new P3.JointFrame3D(new Vector3F(0, 1, 0), QuaternionF.Identity) });
        return arm;
    }

    [Fact]
    public void AJointAcrossTheBoundaryIsRefusedWithoutAPolicyAndLeavesTheWorldAlone()
    {
        var a = World(9120);
        var arm = Pendulum(a, out var pivot);
        a.Step(Dt);
        ulong hash = a.ComputeHash();
        var ex = Assert.Throws<BodySetTransferException>(() => a.ExtractBodies(new[] { arm }));
        Assert.Contains("Joint", ex.Message);
        Assert.Contains(pivot.RawHandle.ToString(), ex.Message);
        Assert.Equal(hash, a.ComputeHash());
        Assert.True(arm.IsValid);
        // Naming the pivot as well puts the joint inside the set.
        var whole = a.ExtractBodies(new[] { arm, pivot }, new BodySetOptions { Move = false });
        Assert.Equal(1, whole.JointCount);
        Assert.Equal(0, whole.AnchorCount);
    }

    [Fact]
    public void BreakDropsTheJointAndHoldKeepsItWithAKinematicAnchor()
    {
        var a = World(9121, ground: false);
        var arm = Pendulum(a, out _);
        for (int i = 0; i < 20; i++) { a.Step(Dt); }

        var broken = a.ExtractBodies(new[] { arm }, new BodySetOptions { Move = false, Boundary = BoundaryPolicy.Break });
        Assert.Equal(0, broken.JointCount);
        Assert.Equal(0, broken.AnchorCount);
        var free = World(9122, ground: false);
        var freeArm = free.GetBody(free.InsertBodies(broken).NewBodies[0]);
        Assert.Equal(0, free.JointCount);
        var y0 = freeArm.Position.Y;
        for (int i = 0; i < 30; i++) { free.Step(Dt); }
        Assert.True(freeArm.Position.Y < y0 - 1.0, "without its joint the arm falls");

        var held = a.ExtractBodies(new[] { arm }, new BodySetOptions { Move = false, Boundary = BoundaryPolicy.HoldAsAnchor });
        Assert.Equal(1, held.JointCount);
        Assert.Equal(1, held.AnchorCount);
        var b = World(9123, ground: false);
        var map = b.InsertBodies(held);
        Assert.Equal(1, b.JointCount);
        Assert.Single(map.Anchors);
        var anchor = b.GetBody(map.Anchors[0].Anchor);
        Assert.Equal(BodyType.Static, anchor.Type);
        Assert.Equal(8.0, anchor.Position.Y, 6);
        var copy = b.GetBody(map.NewBodies[0]);
        double worst = 0;
        for (int i = 0; i < 120; i++) { a.Step(Dt); b.Step(Dt); worst = Math.Max(worst, Gap(arm, copy)); }
        Assert.True(worst <= 1e-6, "the held pendulum swings like the original: " + worst);
    }

    [Fact]
    public void ADynamicOutsideBodyBecomesAKinematicAnchorThatKeepsMoving()
    {
        var a = World(9140, ground: false);
        a.Options.Gravity = default;
        var cart = a.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 5, 0));
        cart.LinearVelocity = new Vector3F(2, 0, 0);
        var load = a.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), at: new Vector3D(0, 5, 2));
        a.CreateJoint(new P3.PointJointDef { BodyA = cart, BodyB = load, LocalFrameA = new P3.JointFrame3D(new Vector3F(0, 0, 1), QuaternionF.Identity), LocalFrameB = new P3.JointFrame3D(new Vector3F(0, 0, -1), QuaternionF.Identity) });
        a.Step(Dt);
        var packet = a.ExtractBodies(new[] { load }, new BodySetOptions { Move = false, Boundary = BoundaryPolicy.HoldAsAnchor });
        var b = World(9141, ground: false);
        b.Options.Gravity = default;
        var map = b.InsertBodies(packet);
        var anchor = b.GetBody(map.Anchors[0].Anchor);
        Assert.Equal(BodyType.Kinematic, anchor.Type);
        Assert.Equal(cart.LinearVelocity.X, anchor.LinearVelocity.X);
        Assert.Equal(map.Anchors[0].Outside, cart.RawHandle);
        double x0 = anchor.Position.X;
        for (int i = 0; i < 30; i++) { b.Step(Dt); }
        Assert.Equal(x0 + (double)cart.LinearVelocity.X * 30 * Dt, anchor.Position.X, 4);
    }

    [Fact]
    public void MovingRemovesTheBodiesAndCopyingKeepsThemUnchanged()
    {
        var a = World(9124);
        var boxes = Stack(a, 3);
        a.Step(Dt);
        ulong before = a.ComputeHash();
        var copy = a.ExtractBodies(boxes, new BodySetOptions { Move = false });
        Assert.Equal(before, a.ComputeHash());
        Assert.All(boxes, x => Assert.True(x.IsValid));
        var moved = a.ExtractBodies(boxes);
        Assert.All(boxes, x => Assert.False(x.IsValid));
        Assert.Equal(1, a.BodyCount);
        Assert.Equal(copy.ToArray(), moved.ToArray());
    }

    [Fact]
    public void TwoPeersThatInsertTheSamePacketStayEqual()
    {
        var a = World(9125);
        var boxes = Stack(a, 4);
        a.AddRagdoll(P3.RagdollDef.Humanoid, new Vector3D(6, 2, 0));
        for (int i = 0; i < 40; i++) { a.Step(Dt); }
        var all = new List<P3.Body>();
        foreach (var x in a.Bodies) { if (x.Type == BodyType.Dynamic) { all.Add(x); } }
        var packet = a.ExtractBodies(all.ToArray(), new BodySetOptions { Move = false });
        var again = BodySetPacket.Read(packet.ToArray());
        Assert.Equal(packet.ToArray(), again.ToArray());
        var b = World(9126); var c = World(9126);
        // A body that comes and goes first, so that the free slots are not simply the next ones.
        foreach (var w in new[] { b, c }) { var t0 = w.AddSphere(0.3f, at: new Vector3D(20, 3, 20)); w.AddSphere(0.3f, at: new Vector3D(22, 3, 20)); t0.Destroy(); }
        var mb = b.InsertBodies(packet, new Vector3D(10, 0, 0));
        var mc = c.InsertBodies(again, new Vector3D(10, 0, 0));
        Assert.Equal(b.ComputeHash(), c.ComputeHash());
        Assert.Equal(b.Snapshot(), c.Snapshot());
        for (int i = 0; i < 90; i++) { b.Step(Dt); c.Step(Dt); Assert.Equal(b.ComputeHash(), c.ComputeHash()); }
        // The lowest free slots, in packet order: the ground is slot 0, the sphere that stayed is slot 2, so the first body takes slot 1 and the rest follow from 3.
        var idx = mb.NewBodies.ToArray().Select(h => h.Index).ToArray();
        Assert.Equal(idx.OrderBy(x => x).ToArray(), idx);
        Assert.Equal(1, idx[0]);
        Assert.Equal(3, idx[1]);
        for (int i = 0; i < idx.Length; i++) { Assert.Equal(idx[i], mc.NewBodies[i].Index); }
    }

    [Fact]
    public void ARotatedPortalKeepsSpeedAndSpinInThePortalFrame()
    {
        var a = World(9127, ground: false);
        var ball = a.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), at: new Vector3D(3, 1, 0));
        ball.LinearVelocity = new Vector3F(4, 0, 3);
        ball.AngularVelocity = new Vector3F(0, 2, 1);
        var packet = a.ExtractBodies(new[] { ball });
        var b = World(9128, ground: false);
        b.Options.Gravity = default;
        var turn = QuaternionF.CreateFromAxisAngle(new Vector3F(0, 1, 0), 1.5707964f);
        var at = new Vector3D(100, 0, -50);
        var copy = b.GetBody(b.InsertBodies(packet, at, turn).NewBodies[0]);
        // A quarter turn about y sends x to -z and z to x.
        var v = copy.LinearVelocity;
        Assert.Equal(3.0, v.X, 4); Assert.Equal(0.0, v.Y, 4); Assert.Equal(-4.0, v.Z, 4);
        Assert.Equal(5.0, Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z), 4);
        var w = copy.AngularVelocity;
        Assert.Equal(1.0, w.X, 4); Assert.Equal(2.0, w.Y, 4); Assert.Equal(0.0, w.Z, 4);
        var p = copy.Position;
        Assert.Equal(100.0, p.X, 4); Assert.Equal(1.0, p.Y, 4); Assert.Equal(-53.0, p.Z, 4);
        for (int i = 0; i < 60; i++) { b.Step(Dt); }
        var q = copy.Position;
        Assert.Equal(100.0 + 3.0, q.X, 3); Assert.Equal(-53.0 - 4.0, q.Z, 3);
    }

    [Fact]
    public void ABodyOfACharacterIsRefusedWithItsName()
    {
        var a = World(9129);
        var player = a.AddCharacter(new Vector3D(0, 1, 0));
        P3.Body body = default;
        foreach (var x in a.Bodies) { if (x.Type == BodyType.Dynamic || x.Type == BodyType.Kinematic) { body = x; } }
        var ex = Assert.Throws<BodySetTransferException>(() => a.ExtractBodies(new[] { body }));
        Assert.Contains("character", ex.Message);
        Assert.True(body.IsValid);
    }

    [Fact]
    public void PacketsOfAnotherLengthUnitOrDimensionAreRefused()
    {
        var a = World(9130);
        var box = a.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 5, 0));
        var packet = a.ExtractBodies(new[] { box }, new BodySetOptions { Move = false });
        var mm = new P3.PhysicsWorld3D(new() { LengthUnit = LengthUnit.Millimetres });
        var ex = Assert.Throws<BodySetTransferException>(() => mm.InsertBodies(packet));
        Assert.Contains("length unit", ex.Message);
        Assert.Equal(0, mm.BodyCount);
        var flat = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D();
        Assert.Throws<BodySetTransferException>(() => flat.InsertBodies(packet));
        var far = World(9131, ground: false);
        far.ShiftOrigin(new Vector3D(2000000, 0, 0));
        Assert.Throws<BodySetTransferException>(() => far.InsertBodies(packet, new Vector3D(-3000000, 0, 0)));
        Assert.Equal(0, far.BodyCount);
    }

    [Fact]
    public void EveryTruncationAndManyCorruptionsOfAPacketAreTypedErrors()
    {
        var a = World(9132);
        var boxes = Stack(a, 3);
        a.AddRagdoll(P3.RagdollDef.Humanoid, new Vector3D(6, 1.2, 0));
        for (int i = 0; i < 20; i++) { a.Step(Dt); }
        var all = new List<P3.Body>();
        foreach (var x in a.Bodies) { if (x.Type == BodyType.Dynamic) { all.Add(x); } }
        var bytes = a.ExtractBodies(all.ToArray(), new BodySetOptions { Move = false }).ToArray();
        Assert.Equal(bytes, BodySetPacket.Read(bytes).ToArray());
        for (int n = 0; n < bytes.Length; n += Math.Max(1, n / 64 + 1)) { int len = n; Assert.Throws<BodySetPacketException>(() => BodySetPacket.Read(bytes.AsSpan(0, len))); }
        var rng = new Random(7);
        int rejected = 0, accepted = 0;
        for (int i = 0; i < 1500; i++)
        {
            var copy = (byte[])bytes.Clone();
            int flips = 1 + rng.Next(3);
            for (int k = 0; k < flips; k++) { copy[rng.Next(copy.Length)] ^= (byte)(1 << rng.Next(8)); }
            try
            {
                var p = BodySetPacket.Read(copy);
                accepted++;
                // What passes the reader must insert without anything but a typed error.
                var b = World(9133);
                try { b.InsertBodies(p); } catch (PhysicsException) { } catch (ArgumentException) { }
            }
            catch (BodySetPacketException e) { rejected++; Assert.False(string.IsNullOrEmpty(e.Field)); }
        }

        Assert.True(rejected > 100, "rejected " + rejected + ", accepted " + accepted);
    }

    [Fact]
    public void NamedFieldsOfHostilePacketsAreReported()
    {
        var a = World(9134, ground: false);
        var box = a.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 5, 0));
        var bytes = a.ExtractBodies(new[] { box }, new BodySetOptions { Move = false }).ToArray();
        // Header: magic, version, dimension, f64 unit, world, f64 step, six lane words, then the counts (member, anchor, shape, joint, contact, support, support contact, ragdoll).
        int bodies = 4 + 4 + 4 + 8 + 4 + 8 + 24 + 32;
        var nan = (byte[])bytes.Clone();
        BitConverter.GetBytes(double.NaN).CopyTo(nan, bodies + 16);
        var ex = Assert.Throws<BodySetPacketException>(() => BodySetPacket.Read(nan));
        Assert.Contains("bodies[0].position", ex.Field);
        var big = (byte[])bytes.Clone();
        BitConverter.GetBytes(int.MaxValue).CopyTo(big, 4 + 4 + 4 + 8 + 4 + 8 + 24);
        Assert.Contains("memberCount", Assert.Throws<BodySetPacketException>(() => BodySetPacket.Read(big)).Field);
        Assert.Contains("magic", Assert.Throws<BodySetPacketException>(() => BodySetPacket.Read(new byte[100])).Field);
        Assert.Throws<ArgumentOutOfRangeException>(() => BodySetPacket.Read(bytes, -1));
        Assert.Contains("limit", Assert.Throws<BodySetPacketException>(() => BodySetPacket.Read(bytes, 10)).Message);
    }

    [Fact]
    public void EveryKindOfShapeJointAndBodySettingTravels()
    {
        var a = World(9160);
        a.Options.Gravity = new Vector3F(0, -9.81f, 0);
        var b0 = a.AddSphere(0.4f, at: new Vector3D(0, 3, 0));
        var b1 = a.AddCapsule(0.4f, 0.2f, at: new Vector3D(1, 3, 0));
        var b2 = a.AddCylinder(0.3f, 0.3f, at: new Vector3D(2, 3, 0));
        var b3 = a.AddConvexHull(new[] { new Vector3F(-.4f, -.4f, -.4f), new Vector3F(.4f, -.4f, -.4f), new Vector3F(0, .5f, 0), new Vector3F(0, -.4f, .5f) }, at: new Vector3D(3, 3, 0));
        var b4 = a.CreateBody(BodyType.Dynamic, new Vector3D(4, 3, 0));
        a.CreateBox(b4, new Vector3F(0.5f, 0.5f, 0.5f));
        a.CreateSphere(b4, 0.2f, new P3.ShapeDef { IsSensor = true });
        var b5 = a.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), at: new Vector3D(5, 3, 0), material: new P3.Material3D(0.2f, 0.4f, 0.01f));
        var b6 = a.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), at: new Vector3D(6, 3, 0));
        var b7 = a.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), at: new Vector3D(7, 3, 0));
        var mesh = a.AddMesh(new[] { new Vector3F(-2, 0, -2), new Vector3F(2, 0, -2), new Vector3F(0, 0.3f, 2) }, new[] { 0, 1, 2 }, new Vector3D(0, 0.01, 10), material: new P3.Material3D(0.7f));
        b1.Mass = 3f; b1.CenterOfMass = new Vector3F(0.1f, 0, 0);
        b2.Controls = new BodyControlSettings(RigidbodyConstraints.FreezePositionZ, 20f, 10f, 5f);
        b3.SleepSettings = new BodySleepSettings(0.2f, 0.2f, 0.7f);
        b5.NeverSleep = true;
        b6.LinearVelocity = new Vector3F(0, 0, 1); b6.AngularVelocity = new Vector3F(1, 2, 3);
        a.CreateJoint(new P3.FixedJointDef { BodyA = b0, BodyB = b1, LocalFrameA = new P3.JointFrame3D(new Vector3F(0.5f, 0, 0), QuaternionF.Identity), LocalFrameB = new P3.JointFrame3D(new Vector3F(-0.5f, 0, 0), QuaternionF.Identity) });
        a.CreateJoint(new P3.HingeJointDef { BodyA = b1, BodyB = b2, EnableLimit = true, LowerAngle = -0.5f, UpperAngle = 0.5f, LocalFrameA = new P3.JointFrame3D(new Vector3F(0.6f, 0, 0), QuaternionF.Identity), LocalFrameB = new P3.JointFrame3D(new Vector3F(-0.4f, 0, 0), QuaternionF.Identity) });
        a.CreateJoint(new P3.SliderJointDef { BodyA = b2, BodyB = b3, EnableLimit = true, LowerTranslation = -0.2f, UpperTranslation = 0.2f, LocalFrameA = new P3.JointFrame3D(new Vector3F(0.5f, 0, 0), QuaternionF.Identity), LocalFrameB = default });
        a.CreateJoint(new P3.DistanceJointDef { BodyA = b3, BodyB = b4, Length = 1.2f, LocalFrameA = default, LocalFrameB = default });
        a.CreateJoint(new P3.SwingTwistJointDef { BodyA = b4, BodyB = b5, LocalFrameA = new P3.JointFrame3D(new Vector3F(0.5f, 0, 0), QuaternionF.Identity), LocalFrameB = new P3.JointFrame3D(new Vector3F(-0.5f, 0, 0), QuaternionF.Identity) });
        a.CreateJoint(new P3.ConeJointDef { BodyA = b5, BodyB = b6, LocalFrameA = new P3.JointFrame3D(new Vector3F(0.5f, 0, 0), QuaternionF.Identity), LocalFrameB = new P3.JointFrame3D(new Vector3F(-0.5f, 0, 0), QuaternionF.Identity) });
        for (int i = 0; i < 50; i++) { a.Step(Dt); }
        var all = new List<P3.Body>();
        foreach (var x in a.Bodies) { all.Add(x); }
        var packet = a.ExtractBodies(all.ToArray(), new BodySetOptions { Move = false });
        Assert.Equal(6, packet.JointCount);
        var b = new P3.PhysicsWorld3D(new() { WorldId = 9161, SleepEnabled = false });
        b.Options.Gravity = a.Options.Gravity;
        var map = b.InsertBodies(packet);
        var src = map.OldBodies.ToArray().Select(h => a.GetBody(h)).ToArray();
        var copy = Bodies(b, map);
        Assert.Equal(b6.SleepSettings.IsOverride, copy[Array.IndexOf(src, b6)].SleepSettings.IsOverride);
        Assert.Equal(b3.SleepSettings.TimeToSleep, copy[Array.IndexOf(src, b3)].SleepSettings.TimeToSleep);
        Assert.Equal(b2.Controls.Constraints, copy[Array.IndexOf(src, b2)].Controls.Constraints);
        Assert.Equal(b1.Mass, copy[Array.IndexOf(src, b1)].Mass);
        Assert.True(copy[Array.IndexOf(src, b5)].NeverSleep);
        Assert.Equal(0.0, Follow(a, src, b, copy, 100));
        // An inserted world restores byte for byte into a fresh one and then steps the same.
        var fresh = new P3.PhysicsWorld3D(new() { WorldId = 9161, SleepEnabled = false });
        fresh.Restore(b.Snapshot());
        Assert.Equal(b.ComputeHash(), fresh.ComputeHash());
        b.Step(Dt); fresh.Step(Dt);
        Assert.Equal(b.ComputeHash(), fresh.ComputeHash());
    }

    [Fact]
    public void AnInsertionIsRecordedAndPlaysBack()
    {
        var a = World(9170);
        var boxes = Stack(a, 3);
        for (int i = 0; i < 100; i++) { a.Step(Dt); }
        var packet = a.ExtractBodies(boxes, new BodySetOptions { Move = false });
        var b = World(9171);
        using var stream = new MemoryStream();
        using (b.StartRecording(stream)) { b.InsertBodies(packet, new Vector3D(5, 0, 0)); for (int i = 0; i < 30; i++) { b.Step(Dt); } }
        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var c = World(9171);
        while (player.ReadNextStep(c)) { }
        Assert.Equal(b.Snapshot(), c.Snapshot());
    }
}
