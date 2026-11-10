using System;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Interchange.Tests;

public sealed class MapTests
{
    private static Map2D Tmj() => TiledImporter.Import(Samples.Read("level.tmj"));

    private static Map2D Tmx() => TiledImporter.Import(Samples.Read("level.tmx"));

    private static Map2D Ldtk() => LdtkImporter.Import(Samples.Read("level.ldtk"));

    [Fact]
    public void ThreeFormatsGiveTheSameTileGrids()
    {
        foreach (var m in new[] { Tmj(), Tmx(), Ldtk() })
        {
            Assert.Equal(2, m.TileLayers.Count);
            var solid = m.TileLayers[0]; var one = m.TileLayers[1];
            Assert.Equal(24, solid.Width); Assert.Equal(8, solid.Height); Assert.Equal(1f, solid.TileSize);
            Assert.False(solid.OneWay); Assert.True(one.OneWay);
            // Bottom row (first in the array) is the floor; the next row up holds the ramp, the block and the other ramp.
            Assert.All(solid.Tiles.Take(24), t => Assert.Equal(TileKind2D.Solid, t));
            Assert.Equal(TileKind2D.SlopeUpRight, solid.Tiles[24 + 8]);
            Assert.Equal(TileKind2D.Solid, solid.Tiles[24 + 10]);
            Assert.Equal(TileKind2D.SlopeUpLeft, solid.Tiles[24 + 12]);
            Assert.Equal(TileKind2D.Solid, one.Tiles[3 * 24 + 15]);
            Assert.Equal(new Vector2D(0, 0), solid.Origin);
        }

        var a = Tmj(); var b = Tmx(); var c = Ldtk();
        Assert.True(a.TileLayers[0].Tiles.SequenceEqual(b.TileLayers[0].Tiles) && a.TileLayers[0].Tiles.SequenceEqual(c.TileLayers[0].Tiles));
        Assert.True(a.TileLayers[1].Tiles.SequenceEqual(b.TileLayers[1].Tiles) && a.TileLayers[1].Tiles.SequenceEqual(c.TileLayers[1].Tiles));
    }

    [Fact]
    public void ObjectsAndPoints()
    {
        foreach (var m in new[] { Tmj(), Tmx(), Ldtk() })
        {
            var spawn = Assert.Single(m.Points);
            Assert.Equal("Spawn", spawn.Type);
            Assert.Equal(2.0, spawn.Position.X); Assert.Equal(2.0, spawn.Position.Y);   // pixel (32, 96) in a 128 px tall map, 16 px per metre
            var crate = Assert.Single(m.Objects);
            Assert.Equal(MapBodyKind.Dynamic, crate.Body);
            Assert.Equal(MapShapeKind.Box, crate.Shape);
            Assert.Equal(2f, crate.Density);
            Assert.Equal(3.5, crate.Position.X, 9); Assert.Equal(5.5, crate.Position.Y, 9);
        }
    }

    private static (PhysicsWorld2D World, MapInstance2D Map) World(Map2D m)
    {
        var w = new PhysicsWorld2D();
        return (w, m.Apply(w, 0f));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void BoxSlidesAcrossSeamsWithoutGhostHits(int format)
    {
        var m = format == 0 ? Tmj() : format == 1 ? Tmx() : Ldtk();
        var (w, _) = World(m);
        var noFriction = new Material2D(0f);
        // On the floor (top at y = 1): 24 tiles of one merged edge.
        var floor = w.AddBox(new Vector2F(0.5f, 0.5f), new Vector2D(0.8, 1.25), material: noFriction);
        floor.LinearVelocity = new Vector2F(4f, 0f);
        // On the raised block (top at y = 2): its edge meets two slopes.
        var block = w.AddBox(new Vector2F(0.4f, 0.4f), new Vector2D(9.4, 2.2), material: noFriction);
        block.LinearVelocity = new Vector2F(1f, 0f);
        for (int i = 0; i < 90; i++)
        {
            w.Step(1f / 60f);
            Assert.InRange(floor.LinearVelocity.Y, -0.05f, 0.05f);
            Assert.InRange(floor.LinearVelocity.X, 3.99f, 4.01f);
        }

        Assert.InRange(floor.Position.X, 5.0, 7.0);
        Assert.InRange(floor.Position.Y, 1.24, 1.27);
        Assert.InRange(block.Position.X, 10.8, 11.0);
        Assert.InRange(block.Position.Y, 2.19, 2.22);
        Assert.InRange(block.LinearVelocity.X, 0.99f, 1.01f);
    }

    [Fact]
    public void OneWayPlatformPassesFromBelowAndCatchesFromAbove()
    {
        var (w, _) = World(Tmj());
        // The platform's tiles are on row 4 from the top: the tile occupies y 3..4 above the origin, so its top is at y = 4.
        var up = w.AddBox(new Vector2F(0.4f, 0.4f), new Vector2D(15.5, 1.5));
        up.LinearVelocity = new Vector2F(0f, 9f);
        for (int i = 0; i < 240; i++) { w.Step(1f / 60f); }
        Assert.InRange(up.Position.Y, 4.15, 4.3);   // stands on top of the platform after passing through it
        Assert.InRange(up.LinearVelocity.Y, -0.05f, 0.05f);
    }

    [Fact]
    public void RampLiftsABallOntoTheBlock()
    {
        var (w, _) = World(Tmj());
        var ball = w.AddCircle(0.3f, new Vector2D(6.0, 1.3), material: new Material2D(0f));
        ball.LinearVelocity = new Vector2F(8f, 0f);
        double top = 0;
        for (int i = 0; i < 60; i++) { w.Step(1f / 60f); top = Math.Max(top, ball.Position.Y); }
        Assert.InRange(top, 2.29, 3.0);   // up the ramp and onto the block, whose top is at y = 2
    }

    [Fact]
    public void ImportIsDeterministic()
    {
        string t = Samples.Read("level.tmj");
        var a = TiledImporter.Import(t); var b = TiledImporter.Import(t);
        Assert.True(a.TileLayers[0].Tiles.SequenceEqual(b.TileLayers[0].Tiles));
        Assert.Equal(a.Objects[0].Position, b.Objects[0].Position);
    }
}
