#pragma warning disable OPHYS0001, OPHYS0002, OPHYS0003
using System;
using RecursiveAI.OneMath;                       // Vector3D, Vector3F
using RecursiveAI.OnePhysics;                    // TileKey, LodBand, TileUnloadPolicy
using RecursiveAI.OnePhysics.Physics3D;          // PhysicsWorld3D, TileDef3D, TimeScaleRegion3D

namespace GuideExamples;

/// <summary>The code block of docs/guide/world-scale.md.</summary>
public static class WorldScale
{
    public static (int Tiles, TileState Gone, float Scale, ulong Hash) Run()
    {
        var tileDef = new TileDef3D();
        tileDef.HeightFields.Add(new TileHeightField3D { Heights = new float[33 * 33], SamplesX = 33, SamplesZ = 33 });
        var player = new Vector3D(100, 1, 230);
        var min = new Vector3D(90, 0, 220);
        var max = new Vector3D(110, 10, 240);
        var world = new PhysicsWorld3D();

        // Terrain in tiles: the host builds each tile's geometry, the world builds the bodies at the start of the next step.
        world.LoadTile(new TileKey(3, 7), tileDef);          // a TileDef3D: height fields, meshes, boxes, voxel grids around an origin
        world.UnloadTile(new TileKey(0, 7));                  // waits until nothing rests on it (or pass TileUnloadPolicy.Immediate)

        // Far islands step less often: within 30 m every step, to 80 m every second, to 150 m every fourth, then every eighth.
        world.SetSimulationLod([player], [new LodBand(30, 1), new LodBand(80, 2), new LodBand(150, 4)], farPeriod: 8);
        world.SetLodFocus([player]);                         // each frame, as the player moves

        // Slow motion for one body and for a region.
        var body = world.AddSphere(0.5f, player);
        body.TimeScale = 0.25f;
        world.SetTimeScaleRegions([new TimeScaleRegion3D(min, max, 0.5f)]);

        for (int i = 0; i < 10; i++) { world.Step(1f / 60f); }
        return (world.TileCount, world.GetTileState(new TileKey(0, 7)), body.TimeScale, world.ComputeHash());
    }
}
