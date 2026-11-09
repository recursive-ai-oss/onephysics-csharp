using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Destruction;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class Fracture3DTests
{
    private static (double mass, Vector3D momentum, Vector3D angular) Totals(IEnumerable<P3.Body> bodies, Vector3D about)
    {
        double m = 0; var p = default(Vector3D); var l = default(Vector3D);
        foreach (var b in bodies)
        {
            double mi = b.Mass; m += mi;
            var v = b.LinearVelocity; var w = b.AngularVelocity;
            p += new Vector3D(v.X, v.Y, v.Z) * mi;
            var rot = b.Rotation;
            var iw = rot.Rotate(b.Inertia * rot.Conjugated().Rotate(w));
            var r = b.WorldCenterOfMass - about;
            l += new Vector3D(iw.X, iw.Y, iw.Z) + Vector3D.Cross(r, new Vector3D(v.X, v.Y, v.Z)) * mi;
        }
        return (m, p, l);
    }

    private static QuaternionF Rot() { var q = new QuaternionF(0.3f, 0.5f, -0.2f, 0.8f); return q.Normalized(); }

    [Fact]
    public void ManualFractureConservesMassAndMomentum()
    {
        var world = new P3.PhysicsWorld3D(new() { Gravity = default });
        var box = world.AddBox(new(2, 1, 3), new Vector3D(1, 5, -2), Rot(), density: 700f);
        box.LinearVelocity = new(1, 2, -3); box.AngularVelocity = new(0.5f, -1f, 2f);
        var origin = new Vector3D(0, 0, 0);
        var before = Totals([box], origin);
        var parts = box.Fracture(new P3.FractureDef { CellCount = 24, Seed = 11 });
        Assert.True(parts.Length > 8);
        Assert.False(box.IsValid);
        var after = Totals(parts, origin);
        Assert.Equal(before.mass, after.mass, before.mass * 1e-5);
        Assert.True((before.momentum - after.momentum).Length() < 1e-4 * before.mass, "linear momentum " + (before.momentum - after.momentum).Length());
        Assert.True((before.angular - after.angular).Length() < 2e-4 * before.mass * 10, "angular momentum " + (before.angular - after.angular).Length());
        Assert.Equal(1, world.FractureEvents.Length);
        Assert.Equal(parts.Length, world.FractureEvents[0].FragmentCount);
        Assert.Equal(FractureKind.Manual, world.FractureEvents[0].Kind);
        Assert.Equal(parts.Length, world.DestructionCounters.FragmentsAlive);
        var again = Totals(parts, origin);
        Assert.Equal(after, again);
    }

    [Fact]
    public void FractureIsDeterministicAndSeedsMatter()
    {
        ulong Hash(ulong seed)
        {
            var w = new P3.PhysicsWorld3D();
            var b = w.AddBox(new(1, 1, 1), new Vector3D(0, 3, 0));
            b.Fracture(new P3.FractureDef { CellCount = 12, Seed = seed });
            for (int i = 0; i < 30; i++) w.Step(1f / 60);
            return w.ComputeHash();
        }
        Assert.Equal(Hash(5), Hash(5));
        Assert.NotEqual(Hash(5), Hash(6));
    }

    [Fact]
    public void ImpactFracturesWhenTheImpulseReachesTheThreshold()
    {
        var world = new P3.PhysicsWorld3D();
        world.AddBox(new(20, 1, 20), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var crate = world.AddBox(new(1, 1, 1), new Vector3D(0, 4, 0));
        crate.SetFracturable(new P3.FractureDef { CellCount = 16, ImpulseThreshold = 3f, Seed = 2 });
        double mass = crate.Mass;
        int step = 0;
        while (crate.IsValid && step < 240) { world.Step(1f / 60); step++; }
        Assert.False(crate.IsValid);
        Assert.True(step > 30);
        Assert.True(world.BodyCount > 8);
        double sum = 0; int n = 0;
        foreach (var b in world.Bodies) if (!b.IsStatic) { sum += b.Mass; n++; }
        Assert.Equal(mass, sum, mass * 1e-4);
        Assert.Equal(n, world.DestructionCounters.FragmentsAlive);
        Assert.Contains(world.FractureEvents.ToArray(), e => e.Kind == FractureKind.Impact && e.Impulse >= 3f);
        // The fragments settle on the floor and do not tunnel through it.
        for (int i = 0; i < 240; i++) world.Step(1f / 60);
        foreach (var b in world.Bodies) if (!b.IsStatic) Assert.True(b.WorldCenterOfMass.Y > -0.05);
    }

    [Fact]
    public void GentlerLandingDoesNotFracture()
    {
        var world = new P3.PhysicsWorld3D();
        world.AddBox(new(20, 1, 20), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var crate = world.AddBox(new(1, 1, 1), new Vector3D(0, 0.55, 0));
        crate.SetFracturable(new P3.FractureDef { CellCount = 16, ImpulseThreshold = 3f });
        for (int i = 0; i < 120; i++) world.Step(1f / 60);
        Assert.True(crate.IsValid);
    }

    [Fact]
    public void SliceConservesMassAndLeavesTwoHalves()
    {
        var world = new P3.PhysicsWorld3D(new() { Gravity = default });
        var box = world.AddBox(new(2, 2, 2), new Vector3D(0, 1, 0), Rot(), density: 3f);
        box.LinearVelocity = new(1, 0, 0); box.AngularVelocity = new(0, 2, 0);
        var before = Totals([box], default);
        var cut = world.Slice(box, new Vector3D(0.2, 1, 0.1), new Vector3F(1, 1, 0.5f));
        Assert.False(box.IsValid);
        var after = Totals([cut.Positive, cut.Negative], default);
        Assert.Equal(before.mass, after.mass, before.mass * 1e-5);
        Assert.True((before.momentum - after.momentum).Length() < 1e-4 * before.mass);
        Assert.True((before.angular - after.angular).Length() < 1e-3 * before.mass);
        // The positive half is on the side the normal points to.
        var n = new Vector3D(1, 1, 0.5); n /= n.Length();
        double dp = Vector3D.Dot(n, cut.Positive.WorldCenterOfMass - new Vector3D(0.2, 1, 0.1));
        double dn = Vector3D.Dot(n, cut.Negative.WorldCenterOfMass - new Vector3D(0.2, 1, 0.1));
        Assert.True(dp > 0 && dn < 0);
        Assert.Throws<FractureException>(() => world.Slice(cut.Positive, new Vector3D(100, 100, 100), new Vector3F(1, 0, 0)));
    }

    [Fact]
    public void DebrisBudgetCullsTheSmallestThenTheOldest()
    {
        var world = new P3.PhysicsWorld3D(new() { Gravity = default });
        world.Debris = new() { MaxFragments = 10 };
        var box = world.AddBox(new(2, 2, 2), new Vector3D(0, 0, 0));
        var parts = box.Fracture(new P3.FractureDef { CellCount = 30, Seed = 3 });
        Assert.True(parts.Length > 12);
        var volumes = new List<(double m, int id)>();
        foreach (var p in parts) volumes.Add((p.Mass, p.RawHandle.Index));
        world.Step(1f / 60);
        Assert.Equal(10, world.DestructionCounters.FragmentsAlive);
        Assert.Equal(10, world.BodyCount);
        volumes.Sort((a, b) => a.m != b.m ? a.m.CompareTo(b.m) : a.id.CompareTo(b.id));
        var survivors = new HashSet<int>();
        foreach (var b in world.Bodies) survivors.Add(b.RawHandle.Index);
        for (int i = volumes.Count - 10; i < volumes.Count; i++) Assert.Contains(volumes[i].id, survivors);
        Assert.Equal(volumes.Count - 10, (int)world.DestructionCounters.FragmentsCulled);
    }

    [Fact]
    public void LifetimeAndFreezeRemoveAndStiffenFragments()
    {
        var world = new P3.PhysicsWorld3D();
        world.AddBox(new(20, 1, 20), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        world.Debris = new() { LifetimeSteps = 400, SleepToStaticSteps = 30 };
        var box = world.AddBox(new(1, 1, 1), new Vector3D(0, 0.6, 0));
        box.Fracture(new P3.FractureDef { CellCount = 6, Seed = 9 });
        for (int i = 0; i < 300; i++) world.Step(1f / 60);
        Assert.True(world.DestructionCounters.FragmentsFrozen > 0);
        int dynamic = 0;
        foreach (var b in world.Bodies) if (!b.IsStatic) dynamic++;
        Assert.True(dynamic < world.BodyCount - 1);
        for (int i = 0; i < 200; i++) world.Step(1f / 60);
        Assert.Equal(1, world.BodyCount);
        Assert.Equal(0, world.DestructionCounters.FragmentsAlive);
        Assert.True(world.DestructionCounters.FragmentsExpired > 0);
    }

    [Fact]
    public void SnapshotRestoreReplaysTheSameFractures()
    {
        P3.PhysicsWorld3D Make()
        {
            var w = new P3.PhysicsWorld3D();
            w.AddBox(new(20, 1, 20), new Vector3D(0, -0.5, 0), type: BodyType.Static);
            for (int i = 0; i < 4; i++)
            {
                var c = w.AddBox(new(1, 1, 1), new Vector3D(i * 1.5 - 2, 3 + i, 0));
                c.SetFracturable(new P3.FractureDef { CellCount = 10, ImpulseThreshold = 2.5f, Seed = (ulong)(7 + i), Depth = 1 });
            }
            w.Debris = new() { MaxFragments = 60 };
            return w;
        }
        var a = Make();
        for (int i = 0; i < 70; i++) a.Step(1f / 60);
        var snap = a.Snapshot();
        var hashes = new List<ulong>();
        for (int i = 0; i < 80; i++) { a.Step(1f / 60); hashes.Add(a.ComputeHash()); }
        var b = Make();
        b.Restore(snap);
        for (int i = 0; i < 80; i++) { b.Step(1f / 60); Assert.Equal(hashes[i], b.ComputeHash()); }
        Assert.Equal(a.Snapshot(), b.Snapshot());
        Assert.True(a.DestructionCounters.Fractures > 0);
        // A fork continues identically.
        var c = a.Fork();
        for (int i = 0; i < 20; i++) { a.Step(1f / 60); c.Step(1f / 60); Assert.Equal(a.ComputeHash(), c.ComputeHash()); }
    }
}
