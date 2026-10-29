using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

/// <summary>Islands without joints run the contact-only schedule; an island with a joint runs the world's own, and adding a joint changes only the island it joins.</summary>
public class Schedule3DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld3D NewWorld(int threads = 1, int contactSubsteps = 2)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 7, ContactSubsteps = contactSubsteps, Events = EventMask.All });
        World3DTests.Floor(w);
        return w;
    }

    // A pile of boxes at x offset `x`; returns its bodies.
    private static List<Body> Pile(PhysicsWorld3D w, double x)
    {
        var list = new List<Body>();
        for (int i = 0; i < 18; i++)
        {
            var at = new Vector3D(x + (i % 3) * 1.05, 0.5 + (i / 9) * 1.02, ((i / 3) % 3) * 1.05);
            list.Add(i % 4 == 0 ? w.AddSphere(0.5f, at) : w.AddBox(new Vector3F(1f, 1f, 1f), at));
        }

        return list;
    }

    private static ulong HashOf(IEnumerable<Body> bodies)
    {
        ulong h = 14695981039346656037UL;
        foreach (var b in bodies)
        {
            var p = b.Position;
            var r = b.Rotation;
            var v = b.LinearVelocity;
            foreach (long bits in new[] { BitConverter.DoubleToInt64Bits(p.X), BitConverter.DoubleToInt64Bits(p.Y), BitConverter.DoubleToInt64Bits(p.Z),
                BitConverter.SingleToInt32Bits(r.X), BitConverter.SingleToInt32Bits(r.Y), BitConverter.SingleToInt32Bits(r.Z), BitConverter.SingleToInt32Bits(r.W),
                BitConverter.SingleToInt32Bits(v.X), BitConverter.SingleToInt32Bits(v.Y), BitConverter.SingleToInt32Bits(v.Z) })
            {
                h = (h ^ (ulong)bits) * 1099511628211UL;
            }
        }

        return h;
    }

    [Theory]
    [InlineData(1)]
    [InlineData(4)]
    public void AddingAJointChangesOnlyItsOwnIsland(int threads)
    {
        var plain = NewWorld(threads);
        var jointed = NewWorld(threads);
        var pa = Pile(plain, 0); var pb = Pile(plain, 60);
        var qa = Pile(jointed, 0); var qb = Pile(jointed, 60);
        var j = new PointJointDef { BodyA = qb[1], BodyB = qb[2] };
        jointed.CreateJoint(in j);
        bool otherChanged = false;
        for (int step = 0; step < 240; step++)
        {
            plain.Step(Dt); jointed.Step(Dt);
            Assert.Equal(HashOf(pa), HashOf(qa));
            if (HashOf(pb) != HashOf(qb)) { otherChanged = true; }
        }

        Assert.True(otherChanged, "the island that got the joint should step differently");
    }

    [Fact]
    public void ContactScheduleOffRunsEveryIslandOnTheWorldSchedule()
    {
        var off = NewWorld(1, 0);
        var same = NewWorld(1, 4);
        var a = Pile(off, 0); var b = Pile(same, 0);
        for (int i = 0; i < 120; i++) { off.Step(Dt); same.Step(Dt); Assert.Equal(off.ComputeHash(), same.ComputeHash()); }
        Assert.Equal(HashOf(a), HashOf(b));
        var on = NewWorld(1, 2);
        var c = Pile(on, 0);
        for (int i = 0; i < 120; i++) { on.Step(Dt); }
        Assert.NotEqual(HashOf(a), HashOf(c));
    }

    [Fact]
    public void ThreadCountDoesNotChangeAMixedWorld()
    {
        ulong Run(int threads)
        {
            var w = NewWorld(threads);
            var p = Pile(w, 0); var q = Pile(w, 60);
            w.CreateJoint(new PointJointDef { BodyA = q[3], BodyB = q[4] });
            ulong h = 0;
            for (int i = 0; i < 200; i++) { w.Step(Dt); h = h * 31 + w.ComputeHash(); }
            return h;
        }

        ulong one = Run(1);
        Assert.Equal(one, Run(3));
        Assert.Equal(one, Run(8));
    }

    [Fact]
    public void SnapshotRestoreAndContinueAcrossAScheduleChange()
    {
        var w = NewWorld();
        var p = Pile(w, 0); var q = Pile(w, 60);
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        var before = w.Snapshot();
        // The joint moves the second pile onto the world's schedule.
        w.CreateJoint(new PointJointDef { BodyA = q[1], BodyB = q[2] });
        var hashes = new List<ulong>();
        for (int i = 0; i < 90; i++) { w.Step(Dt); hashes.Add(w.ComputeHash()); }
        var after = w.Snapshot();

        var fresh = NewWorld();
        fresh.Restore(before);
        var bodies = new List<Body>();
        fresh.CreateJoint(new PointJointDef { BodyA = new Body(fresh, q[1].RawHandle), BodyB = new Body(fresh, q[2].RawHandle) });
        for (int i = 0; i < 90; i++) { fresh.Step(Dt); Assert.Equal(hashes[i], fresh.ComputeHash()); }
        Assert.Equal(after, fresh.Snapshot());

        // And from a snapshot taken after the change.
        var mid = NewWorld();
        mid.Restore(after);
        for (int i = 0; i < 40; i++) { w.Step(Dt); mid.Step(Dt); Assert.Equal(w.ComputeHash(), mid.ComputeHash()); }
        Assert.Equal(w.Snapshot(), mid.Snapshot());
    }

    [Fact]
    public void StepAllocatesNothingOnceWarm()
    {
        var w = NewWorld();
        var p = Pile(w, 0); var q = Pile(w, 60);
        w.CreateJoint(new PointJointDef { BodyA = q[1], BodyB = q[2] });
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 20; i++) { w.Step(Dt); }
        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);
    }

    [Theory]
    [InlineData(-1, 2, 1)]
    public void ContactScheduleOptionsAreRangeChecked(int substeps, int velocity, int relax)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new PhysicsWorld3D(new WorldOptions3D { ContactSubsteps = substeps, ContactVelocityIterations = velocity, ContactRelaxIterations = relax }));
        Assert.Throws<ArgumentOutOfRangeException>(() => new PhysicsWorld3D(new WorldOptions3D { ContactVelocityIterations = 0 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => new PhysicsWorld3D(new WorldOptions3D { ContactRelaxIterations = 257 }));
    }
}
