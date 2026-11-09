using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Destruction;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class Breakable3DTests
{
    private static Vector3F[] BoxCell(float x0, float x1, float h)
    {
        var p = new Vector3F[8];
        for (int i = 0; i < 8; i++) p[i] = new Vector3F((i & 1) != 0 ? x1 : x0, (i & 2) != 0 ? h / 2 : -h / 2, (i & 4) != 0 ? h / 2 : -h / 2);
        return p;
    }

    private static P3.FractureAsset Beam(int n, float length, float h)
    {
        var cells = new List<Vector3F[]>();
        for (int i = 0; i < n; i++) cells.Add(BoxCell(i * length, (i + 1) * length, h));
        return P3.FractureAsset.FromCells(cells);
    }

    [Fact]
    public void AssetBondsTouchingCellsWithTheirFaceArea()
    {
        var asset = Beam(5, 1f, 0.5f);
        Assert.Equal(5, asset.CellCount);
        Assert.Equal(4, asset.BondCount);
        for (int i = 0; i < 4; i++)
        {
            var (a, b, area) = asset.GetBond(i);
            Assert.Equal(a + 1, b);
            Assert.Equal(0.25f, area, 4);
        }
        var cube = P3.FractureAsset.FromHull([new(-1, -1, -1), new(1, -1, -1), new(-1, 1, -1), new(1, 1, -1), new(-1, -1, 1), new(1, -1, 1), new(-1, 1, 1), new(1, 1, 1)], new P3.FractureDef { CellCount = 12, Seed = 4 });
        Assert.True(cube.CellCount >= 6);
        Assert.True(cube.BondCount >= cube.CellCount - 1);
    }

    [Fact]
    public void CantileverBondStressesMatchTheBendingMomentOfTheLoad()
    {
        const int n = 6; const float L = 1f, h = 0.5f, rho = 1000f;
        var world = new P3.PhysicsWorld3D(new() { SleepEnabled = false });
        var beam = world.AddBreakable(Beam(n, L, h), new Vector3D(0, 5, 0), def: new BreakableDef { Strength = 1e9f, AnchorParts = [0] }, type: BodyType.Static, density: rho);
        // A weight resting on the tip cell.
        var tipTop = 5 + h / 2;
        var block = world.AddBox(new(0.4f, 0.4f, 0.4f), new Vector3D((n - 0.5) * L, tipTop + 0.2 + 0.001, 0), density: 5000f);
        for (int i = 0; i < 240; i++) world.Step(1f / 60);
        var bonds = world.GetBonds(beam);
        Assert.Equal(n - 1, bonds.Length);
        double m = rho * L * h * h, wb = block.Mass, g = 9.81, A = h * h, W = Math.Pow(A, 1.5) / 6;
        for (int i = 0; i < bonds.Length; i++)
        {
            int outboard = n - 1 - i;
            double moment = m * g * L * outboard * outboard / 2 + wb * g * (outboard - 0.5) * L;
            double shear = outboard * m * g + wb * g;
            double expected = shear / A + moment / W;
            Assert.InRange(bonds[i].Stress, expected * 0.9, expected * 1.1);
        }
        // The stress peaks at the bond nearest the support, where the bending moment is largest.
        Assert.True(bonds[0].Stress > bonds[1].Stress && bonds[1].Stress > bonds[2].Stress);
    }

    [Fact]
    public void LoadedBeamBreaksAtTheBondNearestTheSupport()
    {
        const int n = 6; const float L = 1f, h = 0.5f, rho = 1000f;
        var world = new P3.PhysicsWorld3D(new() { SleepEnabled = false });
        // Strength chosen so the self-weight is safe and a growing tip load breaks the root bond first.
        var beam = world.AddBreakable(Beam(n, L, h), new Vector3D(0, 5, 0), def: new BreakableDef { Strength = 4e6f, AnchorParts = [0] }, type: BodyType.Static, density: rho);
        var block = world.AddBox(new(0.4f, 0.4f, 0.4f), new Vector3D((n - 0.5) * L, 5 + h / 2 + 0.2 + 0.001, 0), density: 500f);
        var bodies = new List<P3.Body>();
        int broke = -1;
        for (int i = 0; i < 1200 && broke < 0; i++)
        {
            if (i % 10 == 0 && block.IsValid) block.Mass = block.Mass * 1.06f;
            world.Step(1f / 60);
            if (world.FractureEvents.Length > 0) { broke = i; break; }
        }
        Assert.True(broke > 0, "the beam never broke");
        var ev = world.FractureEvents[0];
        Assert.Equal(FractureKind.BondBreak, ev.Kind);
        Assert.False(beam.IsValid);
        // The anchored root stays static; everything else came away together as one dynamic piece.
        var statics = 0; var dynamics = 0;
        foreach (var b in world.Bodies) { if (b.IsStatic) statics++; else if (b != block) dynamics++; }
        Assert.Equal(2, ev.FragmentCount);
        Assert.Equal(1, dynamics);
        Assert.Equal(1, statics);
        var frags = world.FractureFragments.ToArray();
        P3.Body root = frags[0].IsStatic ? frags[0] : frags[1];
        P3.Body rest = frags[0].IsStatic ? frags[1] : frags[0];
        Assert.Equal(1, world.GetBonds(root).Length + 1);
        Assert.Equal(n - 1, world.GetBonds(rest).Length + 1);
        Assert.Equal(L * h * h * rho, rest.Mass / (n - 1) * 1.0, 1e-2 * L * h * h * rho * 5);
    }

    [Fact]
    public void FreeCompoundBreaksAtImpactAndPiecesFly()
    {
        var world = new P3.PhysicsWorld3D();
        world.AddBox(new(20, 1, 20), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var wall = world.AddBreakable(Beam(4, 1f, 0.5f), new Vector3D(-2, 6, 0), def: new BreakableDef { Strength = 2e4f }, density: 800f);
        double mass = wall.Mass;
        for (int i = 0; i < 120 && wall.IsValid; i++) world.Step(1f / 60);
        Assert.False(wall.IsValid);
        double sum = 0;
        foreach (var b in world.Bodies) if (!b.IsStatic) sum += b.Mass;
        Assert.Equal(mass, sum, mass * 1e-4);
        Assert.True(world.DestructionCounters.BondBreaks > 0);
    }

    [Fact]
    public void ManualBondBreakSplitsAndSliceCutsAcrossParts()
    {
        var world = new P3.PhysicsWorld3D(new() { Gravity = default });
        var beam = world.AddBreakable(Beam(4, 1f, 1f), new Vector3D(0, 2, 0), density: 100f);
        var halves = world.BreakBond(beam, 1);
        Assert.Equal(2, halves.Length);
        Assert.Single(world.GetBonds(halves[0]));
        Assert.Single(world.GetBonds(halves[1]));
        var whole = world.AddBreakable(Beam(4, 1f, 1f), new Vector3D(10, 2, 0), density: 100f);
        var cut = world.Slice(whole, new Vector3D(10.5, 2, 0), new Vector3F(1, 0, 0));
        // The plane cuts the first cell in two: its outer half is alone on the negative side; the rest keeps its three bonds.
        Assert.Empty(world.GetBonds(cut.Negative));
        Assert.Equal(3, world.GetBonds(cut.Positive).Length);
    }
}
