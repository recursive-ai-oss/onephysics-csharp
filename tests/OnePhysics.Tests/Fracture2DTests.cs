using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Destruction;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class Fracture2DTests
{
    private static (double mass, Vector2D momentum, double angular) Totals(IEnumerable<P2.Body> bodies)
    {
        double m = 0; var p = default(Vector2D); double l = 0;
        foreach (var b in bodies)
        {
            double mi = b.Mass; m += mi;
            var v = b.LinearVelocity; var c = b.WorldCenterOfMass;
            p += new Vector2D(v.X, v.Y) * mi;
            l += b.Inertia * b.AngularVelocity + mi * (c.X * v.Y - c.Y * v.X);
        }
        return (m, p, l);
    }

    private static Vector2F[] BoxCell(float x0, float x1, float h) => [new(x0, -h / 2), new(x1, -h / 2), new(x1, h / 2), new(x0, h / 2)];

    private static P2.FractureAsset Beam(int n, float length, float h)
    {
        var cells = new List<Vector2F[]>();
        for (int i = 0; i < n; i++) cells.Add(BoxCell(i * length, (i + 1) * length, h));
        return P2.FractureAsset.FromCells(cells);
    }

    [Fact]
    public void ManualFractureConservesMassAndMomentum()
    {
        var world = new P2.PhysicsWorld2D(new() { Gravity = default });
        var box = world.AddBox(new(2, 1), new Vector2D(1, 5), 0.7f, density: 700f);
        box.LinearVelocity = new(1, -3); box.AngularVelocity = 2f;
        var before = Totals([box]);
        var parts = box.Fracture(new P2.FractureDef { CellCount = 20, Seed = 11 });
        Assert.True(parts.Length > 8);
        Assert.False(box.IsValid);
        var after = Totals(parts);
        Assert.Equal(before.mass, after.mass, before.mass * 1e-5);
        Assert.True((before.momentum - after.momentum).Length() < 1e-4 * before.mass);
        Assert.Equal(before.angular, after.angular, Math.Abs(before.angular) * 1e-4 + 1e-3);
        Assert.Single(world.FractureEvents.ToArray());
    }

    [Fact]
    public void ImpactFracturesTheCrateAndFragmentsRestOnTheGround()
    {
        var world = new P2.PhysicsWorld2D();
        world.AddBox(new(30, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        var crate = world.AddBox(new(1, 1), new Vector2D(0, 4));
        crate.SetFracturable(new P2.FractureDef { CellCount = 10, ImpulseThreshold = 3f, Seed = 2 });
        double mass = crate.Mass;
        int step = 0;
        while (crate.IsValid && step < 240) { world.Step(1f / 60); step++; }
        Assert.False(crate.IsValid);
        double sum = 0;
        foreach (var b in world.Bodies) if (!b.IsStatic) sum += b.Mass;
        Assert.Equal(mass, sum, mass * 1e-3);
        for (int i = 0; i < 240; i++) world.Step(1f / 60);
        foreach (var b in world.Bodies) if (!b.IsStatic) Assert.True(b.WorldCenterOfMass.Y > -0.05);
    }

    [Fact]
    public void SliceCutsAPolygonIntoTwoHalvesOfTheSameMass()
    {
        var world = new P2.PhysicsWorld2D(new() { Gravity = default });
        var box = world.AddBox(new(2, 2), new Vector2D(0, 1), 0.3f, density: 3f);
        box.LinearVelocity = new(1, 0); box.AngularVelocity = 2f;
        var before = Totals([box]);
        var cut = world.Slice(box, new Vector2D(0.1, 1), new Vector2F(1, 1));
        var after = Totals([cut.Positive, cut.Negative]);
        Assert.Equal(before.mass, after.mass, before.mass * 1e-5);
        Assert.True((before.momentum - after.momentum).Length() < 1e-4 * before.mass);
        Assert.Equal(before.angular, after.angular, 1e-3);
        Assert.Throws<FractureException>(() => world.Slice(cut.Positive, new Vector2D(100, 100), new Vector2F(1, 0)));
    }

    [Fact]
    public void CantileverBondStressesMatchTheBendingMoment()
    {
        const int n = 6; const float L = 1f, h = 0.5f, rho = 1000f;
        var world = new P2.PhysicsWorld2D(new() { SleepEnabled = false });
        var beam = world.AddBreakable(Beam(n, L, h), new Vector2D(0, 5), def: new BreakableDef { Strength = 1e9f, AnchorParts = [0] }, type: BodyType.Static, density: rho);
        var block = world.AddBox(new(0.4f, 0.4f), new Vector2D((n - 0.5) * L, 5 + h / 2 + 0.2 + 0.001), density: 5000f);
        for (int i = 0; i < 240; i++) world.Step(1f / 60);
        var bonds = world.GetBonds(beam);
        double m = rho * L * h, wb = block.Mass, g = 9.81, A = h, W = A * A / 6;
        for (int i = 0; i < bonds.Length; i++)
        {
            int outboard = n - 1 - i;
            double moment = m * g * L * outboard * outboard / 2 + wb * g * (outboard - 0.5) * L;
            double shear = outboard * m * g + wb * g;
            double expected = shear / A + moment / W;
            Assert.InRange(bonds[i].Stress, expected * 0.9, expected * 1.1);
        }
    }

    [Fact]
    public void LoadedBeamBreaksAtTheBondNearestTheSupport()
    {
        const int n = 6; const float L = 1f, h = 0.5f, rho = 1000f;
        var world = new P2.PhysicsWorld2D(new() { SleepEnabled = false });
        var beam = world.AddBreakable(Beam(n, L, h), new Vector2D(0, 5), def: new BreakableDef { Strength = 2e7f, AnchorParts = [0] }, type: BodyType.Static, density: rho);
        var block = world.AddBox(new(0.4f, 0.4f), new Vector2D((n - 0.5) * L, 5 + h / 2 + 0.2 + 0.001), density: 500f);
        int broke = -1;
        for (int i = 0; i < 1500 && broke < 0; i++)
        {
            if (i % 10 == 0) block.Mass = block.Mass * 1.06f;
            world.Step(1f / 60);
            if (world.FractureEvents.Length > 0) broke = i;
        }
        Assert.True(broke > 0);
        var ev = world.FractureEvents[0];
        Assert.Equal(FractureKind.BondBreak, ev.Kind);
        Assert.Equal(2, ev.FragmentCount);
        var frags = world.FractureFragments.ToArray();
        var root = frags[0].IsStatic ? frags[0] : frags[1];
        var rest = frags[0].IsStatic ? frags[1] : frags[0];
        Assert.Empty(world.GetBonds(root));
        Assert.Equal(n - 2, world.GetBonds(rest).Length);
    }

    [Fact]
    public void DebrisRulesAndSnapshotRestore()
    {
        P2.PhysicsWorld2D Make()
        {
            var w = new P2.PhysicsWorld2D();
            w.AddBox(new(30, 1), new Vector2D(0, -0.5), type: BodyType.Static);
            for (int i = 0; i < 4; i++)
            {
                var c = w.AddBox(new(1, 1), new Vector2D(i * 1.5 - 2, 3 + i));
                c.SetFracturable(new P2.FractureDef { CellCount = 8, ImpulseThreshold = 2.5f, Seed = (ulong)(7 + i), Depth = 1 });
            }
            w.Debris = new() { MaxFragments = 30, SleepToStaticSteps = 40 };
            return w;
        }
        var a = Make();
        for (int i = 0; i < 70; i++) a.Step(1f / 60);
        var snap = a.Snapshot();
        var hashes = new List<ulong>();
        for (int i = 0; i < 120; i++) { a.Step(1f / 60); hashes.Add(a.ComputeHash()); }
        var b = Make();
        b.Restore(snap);
        for (int i = 0; i < 120; i++) { b.Step(1f / 60); Assert.Equal(hashes[i], b.ComputeHash()); }
        Assert.True(a.DestructionCounters.Fractures > 0);
        Assert.True(a.DestructionCounters.FragmentsAlive <= 30);
        var c2 = a.Fork();
        for (int i = 0; i < 20; i++) { a.Step(1f / 60); c2.Step(1f / 60); Assert.Equal(a.ComputeHash(), c2.ComputeHash()); }
    }

    [Fact]
    public void SlicedHalvesOfANonSpinningBodyStayTogetherInFreeFlight()
    {
        var world = new P2.PhysicsWorld2D(new() { Gravity = default });
        var box = world.AddBox(new(1.2f, 0.8f), new Vector2D(0, 6), 1.1f);
        box.LinearVelocity = new(2, -1);
        var cut = world.Slice(box, new Vector2D(0.05, 6.02), new Vector2F(0.3f, 1f));
        double Ke() { double e = 0; foreach (var b in new[] { cut.Positive, cut.Negative }) { var v = b.LinearVelocity; e += 0.5 * b.Mass * (v.X * v.X + v.Y * v.Y) + 0.5 * b.Inertia * b.AngularVelocity * b.AngularVelocity; } return e; }
        double e0 = Ke();
        double Dist() { var a = cut.Positive.WorldCenterOfMass; var b = cut.Negative.WorldCenterOfMass; return Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y)); }
        double d0 = Dist();
        for (int i = 0; i < 60; i++) world.Step(1f / 60);
        Assert.Equal(e0, Ke(), e0 * 1e-3);
        Assert.Equal(d0, Dist(), 1e-3);
    }
}
