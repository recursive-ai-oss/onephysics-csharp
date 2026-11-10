using System;
using RecursiveAI.OneMath;                       // Vector2D
using RecursiveAI.OnePhysics;                    // TerrainAnchors
using RecursiveAI.OnePhysics.Physics2D;          // PhysicsWorld2D, Body

namespace GuideExamples;

/// <summary>The code block of docs/guide/terrain.md about 2D bitmask terrain.</summary>
public static class Terrain2D
{
    public static (int Pieces, ulong Hash) Run()
    {
        var world = new PhysicsWorld2D();
        var solid = new bool[128 * 64];                       // x-fastest; true = ground
        for (int y = 0; y < 32; y++) for (int x = 0; x < 128; x++) solid[y * 128 + x] = true;
        Body ground = world.AddBitmaskTerrain(128, 64, solid, cellSize: 0.25f);

        ground.CarveCircle(new Vector2D(10, 8), 2.5);         // a crater; bodies resting on it wake and fall
        ground.FillPolygon([new(4, 6), new(8, 6), new(8, 8)]);
        Body[] pieces = ground.DetachIslands();               // chunks cut loose become dynamic bodies
        for (int i = 0; i < 30; i++) world.Step(1f / 60);
        return (pieces.Length, world.ComputeHash());
    }
}
