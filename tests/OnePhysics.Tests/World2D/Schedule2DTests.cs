using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

/// <summary>The 2D world takes the same contact-only schedule options as 3D, off by default; a joint island always runs the world's own schedule.</summary>
public class Schedule2DTests
{
    private const float Dt = 1f / 60f;

    private static (PhysicsWorld2D W, List<Body> Bodies) NewWorld(int contactSubsteps, bool hinge)
    {
        var list = new List<Body>();
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 9, ContactSubsteps = contactSubsteps, ContactRelaxIterations = 2, Events = EventMask.All });
        w.AddBox(new Vector2F(200f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        // A pile of boxes on the left (no joints), a hinged pendulum plus a few boxes on the right.
        for (int i = 0; i < 24; i++) { list.Add(w.AddBox(new Vector2F(1f, 1f), at: new Vector2D((i % 4) * 1.05 - 20, 0.5 + (i / 4) * 1.02))); }
        if (hinge)
        {
            var pivot = w.AddCircle(0.1f, new Vector2D(30, 8), BodyType.Static);
            var bob = w.AddCircle(0.3f, new Vector2D(32, 8)); list.Add(bob);
            w.AddHinge(pivot, bob, new Vector2D(30, 8));
            for (int i = 0; i < 6; i++) { list.Add(w.AddBox(new Vector2F(1f, 1f), at: new Vector2D(40 + (i % 3) * 1.05, 0.5 + (i / 3) * 1.02))); }
        }

        return (w, list);
    }

    // The state of the bodies whose x lies in [lo, hi) as one hash.
    private static ulong StateOf(List<Body> bodies, double lo, double hi)
    {
        ulong h = 14695981039346656037UL;
        foreach (var b in bodies)
        {
            var p = b.Position;
            if (p.X < lo || p.X >= hi) { continue; }
            var r = b.Rotation; var v = b.LinearVelocity;
            foreach (long bits in new[] { BitConverter.DoubleToInt64Bits(p.X), BitConverter.DoubleToInt64Bits(p.Y), BitConverter.SingleToInt32Bits(r.C), BitConverter.SingleToInt32Bits(r.S), BitConverter.SingleToInt32Bits(v.X), BitConverter.SingleToInt32Bits(v.Y) })
            {
                h = (h ^ (ulong)bits) * 1099511628211UL;
            }
        }

        return h;
    }

    [Fact]
    public void TheContactScheduleIsOffByDefault()
    {
        var o = new WorldOptions2D();
        Assert.Equal(0, o.ContactSubsteps);
        // A value not below Substeps is off as well: the world is bit-equal to one with the feature at 0.
        var (off, a) = NewWorld(0, true); var (same, b) = NewWorld(4, true);
        for (int i = 0; i < 150; i++) { off.Step(Dt); same.Step(Dt); Assert.Equal(StateOf(a, -1e9, 1e9), StateOf(b, -1e9, 1e9)); }
    }

    [Fact]
    public void AJointIslandStepsAsWithTheFeatureOff()
    {
        var (on, a) = NewWorld(2, true); var (off, b) = NewWorld(0, true);
        bool pileChanged = false;
        for (int i = 0; i < 240; i++)
        {
            on.Step(Dt); off.Step(Dt);
            Assert.Equal(StateOf(b, 25, 38), StateOf(a, 25, 38));
            if (StateOf(b, -1e9, 0) != StateOf(a, -1e9, 0)) { pileChanged = true; }
        }

        Assert.True(pileChanged, "the jointless pile should run the contact-only schedule");
    }

    [Fact]
    public void ContactScheduleOptionsAreRangeCheckedAndSaved()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new PhysicsWorld2D(new WorldOptions2D { ContactSubsteps = 257 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => new PhysicsWorld2D(new WorldOptions2D { ContactVelocityIterations = 0 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => new PhysicsWorld2D(new WorldOptions2D { ContactRelaxIterations = -1 }));
        var (w, _) = NewWorld(2, true);
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        var snap = w.Snapshot();
        var fresh = new PhysicsWorld2D(new WorldOptions2D { WorldId = 9 });
        Assert.True((fresh.Restore(snap) & OptionChanges.ContactSchedule) != 0);
        Assert.Equal(2, fresh.Options.ContactSubsteps);
        for (int i = 0; i < 60; i++) { w.Step(Dt); fresh.Step(Dt); Assert.Equal(w.ComputeHash(), fresh.ComputeHash()); }
    }
}
