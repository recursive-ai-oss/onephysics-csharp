using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

/// <summary>Islands without joints, or with only mild joints, run the contact-only schedule; an island with a stiff joint (motor, spring, drive, heavy mass ratio, loops) runs the world's own, and adding a joint changes only the island it joins.</summary>
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
        for (int i = 0; i < 120; i++) { off.Step(Dt); same.Step(Dt); Assert.Equal(HashOf(a), HashOf(b)); }
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
    public void TheContactScheduleTravelsWithTheSnapshot()
    {
        // Saved with non-default contact options, restored into a world built with the defaults: the continuation matches the original, and the hash covers the options.
        var w = new PhysicsWorld3D(new WorldOptions3D { WorldId = 7, ContactSubsteps = 3, ContactVelocityIterations = 2, ContactRelaxIterations = 1, Substeps = 6 });
        World3DTests.Floor(w);
        var p = Pile(w, 0); var q = Pile(w, 60);
        w.CreateJoint(new PointJointDef { BodyA = q[1], BodyB = q[2] });
        for (int i = 0; i < 50; i++) { w.Step(Dt); }
        var snap = w.Snapshot();

        var fresh = new PhysicsWorld3D(new WorldOptions3D { WorldId = 7 });
        var changed = fresh.Restore(snap);
        Assert.True((changed & OptionChanges.ContactSchedule) != 0);
        Assert.Equal(3, fresh.Options.ContactSubsteps);
        Assert.Equal(2, fresh.Options.ContactVelocityIterations);
        Assert.Equal(1, fresh.Options.ContactRelaxIterations);
        Assert.Equal(w.ComputeHash(), fresh.ComputeHash());
        for (int i = 0; i < 80; i++) { w.Step(Dt); fresh.Step(Dt); Assert.Equal(w.ComputeHash(), fresh.ComputeHash()); }
        Assert.Equal(w.Snapshot(), fresh.Snapshot());

        // The options are part of the hash.
        var other = new PhysicsWorld3D(new WorldOptions3D { WorldId = 7, ContactSubsteps = 2 });
        var h = other.ComputeHash();
        var other2 = new PhysicsWorld3D(new WorldOptions3D { WorldId = 7, ContactSubsteps = 3 });
        Assert.NotEqual(h, other2.ComputeHash());
    }

    [Fact]
    public void ARestoreRejectsContactScheduleOptionsOutOfRange()
    {
        var w = NewWorld();
        Pile(w, 0);
        for (int i = 0; i < 5; i++) { w.Step(Dt); }
        var snap = w.Snapshot();
        // Options come first: magic, version, layout (7 x i32), gravity (3 x f32), three world i32, then the three contact i32.
        int at = 8 + 7 * 4 + 3 * 4 + 3 * 4;
        foreach (var (offset, value) in new[] { (0, 300), (0, -1), (4, 0), (4, 257), (8, -2), (8, 257) })
        {
            var bad = (byte[])snap.Clone();
            System.Buffers.Binary.BinaryPrimitives.WriteInt32LittleEndian(bad.AsSpan(at + offset), value);
            var before = w.ComputeHash();
            Assert.Throws<SnapshotFormatException>(() => w.Restore(bad));
            Assert.Equal(before, w.ComputeHash());
        }
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

    // A short chain of boxes in a row, joined by point joints, with the given joint kind and densities; returns a hash of the bodies after some steps with the given contact-only substep count.
    private static ulong ChainHash(int contactSubsteps, Func<int, float> density, Action<PhysicsWorld3D, Body, Body> join, bool ladder = false)
    {
        var w = NewWorld(1, contactSubsteps);
        var rows = ladder ? 2 : 1;
        var bodies = new Body[rows, 4];
        for (int r = 0; r < rows; r++)
        {
            for (int i = 0; i < 4; i++) { bodies[r, i] = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(i * 0.6, 0.5 + r * 0.6, 0), density: density(i)); }
        }

        for (int r = 0; r < rows; r++)
        {
            for (int i = 0; i < 3; i++) { join(w, bodies[r, i], bodies[r, i + 1]); }
        }

        if (ladder) { for (int i = 0; i < 4; i++) { join(w, bodies[0, i], bodies[1, i]); } }
        ulong h = 0;
        for (int i = 0; i < 120; i++) { w.Step(Dt); h = h * 31 + HashOf(bodies.Cast<Body>()); }
        return h;
    }

    private static void Point(PhysicsWorld3D w, Body a, Body b) => w.CreateJoint(new PointJointDef { BodyA = a, BodyB = b });

    [Fact]
    public void MildJointsRunOnTheContactSchedule()
    {
        Assert.NotEqual(ChainHash(0, _ => 1f, Point), ChainHash(2, _ => 1f, Point));
        // A mass ratio of 20 is still mild.
        Assert.NotEqual(ChainHash(0, i => i == 3 ? 20f : 1f, Point), ChainHash(2, i => i == 3 ? 20f : 1f, Point));
    }

    [Fact]
    public void StiffJointsKeepTheWorldSchedule()
    {
        // The contact-only schedule changes nothing for these, so switching it off or on gives the same bits.
        Assert.Equal(ChainHash(0, i => i == 3 ? 500f : 1f, Point), ChainHash(2, i => i == 3 ? 500f : 1f, Point));
        Assert.Equal(ChainHash(0, _ => 1f, (w, a, b) => w.CreateJoint(new HingeJointDef { BodyA = a, BodyB = b, EnableMotor = true, MaxMotorTorque = 10f })),
                     ChainHash(2, _ => 1f, (w, a, b) => w.CreateJoint(new HingeJointDef { BodyA = a, BodyB = b, EnableMotor = true, MaxMotorTorque = 10f })));
        Assert.Equal(ChainHash(0, _ => 1f, (w, a, b) => w.CreateJoint(new HingeJointDef { BodyA = a, BodyB = b, EnableSpring = true, SpringHertz = 2f })),
                     ChainHash(2, _ => 1f, (w, a, b) => w.CreateJoint(new HingeJointDef { BodyA = a, BodyB = b, EnableSpring = true, SpringHertz = 2f })));
        // A ladder has more joints than bodies.
        Assert.Equal(ChainHash(0, _ => 1f, Point, ladder: true), ChainHash(2, _ => 1f, Point, ladder: true));
    }

    [Fact]
    public void MildJointSnapshotContinuesBitEqual()
    {
        var w = NewWorld();
        var bodies = new List<Body>();
        for (int i = 0; i < 5; i++) { bodies.Add(w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(i * 0.6, 1.5, 0))); }
        for (int i = 0; i < 4; i++) { w.CreateJoint(new PointJointDef { BodyA = bodies[i], BodyB = bodies[i + 1] }); }
        for (int i = 0; i < 40; i++) { w.Step(Dt); }
        var snap = w.Snapshot();
        var fresh = NewWorld();
        fresh.Restore(snap);
        for (int i = 0; i < 80; i++) { w.Step(Dt); fresh.Step(Dt); Assert.Equal(w.ComputeHash(), fresh.ComputeHash()); }
        Assert.Equal(w.Snapshot(), fresh.Snapshot());
    }

    // A box hanging from a static one by a point joint that breaks under its weight; the joint is mild, so it is solved on the contact-only schedule.
    private static (PhysicsWorld3D World, Body Hanging) BreakingWorld()
    {
        var w = NewWorld();
        var anchor = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(0, 3, 0), type: BodyType.Static);
        var hang = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(0.6, 3, 0), density: 100f);
        w.CreateJoint(new PointJointDef { BodyA = anchor, BodyB = hang, BreakForce = 60f });
        return (w, hang);
    }

    [Fact]
    public void MildJointBreaksAndTheBreakStepSurvivesSnapshotRestore()
    {
        var (probe, _) = BreakingWorld();
        int breakStep = -1;
        for (int i = 0; i < 200 && breakStep < 0; i++)
        {
            probe.Step(Dt);
            if (probe.GetEvents(EventKind.JointBreak).Length > 0) { breakStep = i; }
        }

        Assert.True(breakStep >= 0, "the joint should break under the hanging box's weight");

        var (w, hang) = BreakingWorld();
        for (int i = 0; i < breakStep; i++) { w.Step(Dt); Assert.Empty(w.GetEvents(EventKind.JointBreak).ToArray()); }
        var snap = w.Snapshot();
        var fresh = NewWorld();
        fresh.Restore(snap);
        w.Step(Dt); fresh.Step(Dt);
        var ea = w.GetEvents(EventKind.JointBreak).ToArray();
        var eb = fresh.GetEvents(EventKind.JointBreak).ToArray();
        Assert.Single(ea);
        Assert.Equal(ea.Length, eb.Length);
        Assert.Equal(ea[0].Value, eb[0].Value);
        Assert.Equal(w.ComputeHash(), fresh.ComputeHash());
        Assert.Equal(w.Snapshot(), fresh.Snapshot());
        for (int i = 0; i < 60; i++) { w.Step(Dt); fresh.Step(Dt); Assert.Equal(w.ComputeHash(), fresh.ComputeHash()); }
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
