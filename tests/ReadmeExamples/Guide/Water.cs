#pragma warning disable OPHYS0001, OPHYS0002, OPHYS0003
using System;
using RecursiveAI.OneMath;                       // Vector3D, Vector3F
using RecursiveAI.OnePhysics;                    // BodyType
using RecursiveAI.OnePhysics.Physics3D;          // PhysicsWorld3D, Body, ShallowWater

namespace GuideExamples;

/// <summary>The code block of docs/guide/water.md.</summary>
public static class Water
{
    public static (double CrateY, double Surface, float Depth, float Hit, int Events, ulong Hash) Run()
    {
        double crateY = 0, surface = 0;
        float depth = 0, hitDistance = 0;
        int events = 0;
        var world = new PhysicsWorld3D();
        world.AddBox(new Vector3F(60f, 1f, 60f), at: new Vector3D(10, -0.5, 10), type: BodyType.Static);   // the bed is whatever static shapes lie under the water
        var water = world.AddShallowWater(minX: 0, minZ: 0, maxX: 20, maxZ: 20, level: 2.0, cellSize: 0.5f, floor: -1.0);

        var crate = world.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(10, 3, 10), density: 500f);   // floats at half depth
        world.BodyWaterChanged += e => events++;                                                          // a body entered or left the water

        for (int i = 0; i < 600; i++)
        {
            if (i == 200) water.AddVolume(0, 0, 2, 20, 20f);   // a wave maker: 20 m³ at the west end
            world.Step(1f / 60f);
        }

        if (world.QueryWater(new Vector3D(10, 0, 10), out var sample)) { surface = sample.Surface; depth = sample.Depth; }
        if (world.RaycastWater(new Vector3D(10, 10, 10), new Vector3F(0, -1, 0), out var hit)) { hitDistance = hit.Distance; }
        crateY = crate.Position.Y;
        return (crateY, surface, depth, hitDistance, events, world.ComputeHash());
    }
}
