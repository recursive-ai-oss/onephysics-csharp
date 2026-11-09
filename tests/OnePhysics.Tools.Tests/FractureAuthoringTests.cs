using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Destruction;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tools.Tests;

public class FractureAuthoringTests
{
    [Fact]
    public void LBlockIsDecomposedThenFracturedIntoBondedCells()
    {
        var m = TestMeshes.LBlock();
        var asset = FractureAuthoring.Prefracture(m.V, m.I, new P3.FractureDef { CellCount = 20, Seed = 5 });
        Assert.InRange(asset.CellCount, 8, 40);
        Assert.True(asset.BondCount >= asset.CellCount - 3);
        var again = FractureAuthoring.Prefracture(m.V, m.I, new P3.FractureDef { CellCount = 20, Seed = 5 });
        Assert.Equal(asset.CellCount, again.CellCount);
        for (int i = 0; i < asset.CellCount; i++) Assert.Equal(asset.GetCell(i).ToArray(), again.GetCell(i).ToArray());
        var world = new P3.PhysicsWorld3D();
        var body = world.AddBreakable(asset, new Vector3D(0, 3, 0), def: new BreakableDef { Strength = 1e5f }, density: 800f);
        // The cells fill the block: the body's mass is its density times the block's volume (2x2 L-prism, height 1: 3 m^3).
        Assert.Equal(3 * 800.0, body.Mass, 3 * 800.0 * 0.01);
    }

    [Fact]
    public void DecompositionCellsBondWhereTheyShareFaces()
    {
        var m = TestMeshes.Box(2, 1, 1);
        var decomposition = ConvexDecomposer.Decompose(m.V, m.I);
        var asset = FractureAuthoring.FromDecomposition(decomposition);
        Assert.Equal(1, asset.CellCount);
        Assert.Equal(0, asset.BondCount);
    }

    [Fact]
    public void PolygonOutlineIsFracturedInto2DCells()
    {
        Vector2F[] outline = [new(0, 0), new(4, 0), new(4, 1), new(1, 1), new(1, 3), new(0, 3)];
        var asset = FractureAuthoring.Prefracture(outline, null, new P2.FractureDef { CellCount = 12, Seed = 2 });
        Assert.True(asset.CellCount >= 6);
        Assert.True(asset.BondCount >= asset.CellCount - 2);
        var world = new P2.PhysicsWorld2D();
        var body = world.AddBreakable(asset, new Vector2D(0, 3), density: 100f);
        Assert.Equal(6.0 * 100, body.Mass, 6.0 * 100 * 0.01);
    }
}
