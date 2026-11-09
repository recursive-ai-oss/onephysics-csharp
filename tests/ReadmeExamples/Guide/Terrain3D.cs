#pragma warning disable OPHYS0001, OPHYS0002, OPHYS0003
using System;
using RecursiveAI.OneMath;                       // Vector3D, Vector3F
using RecursiveAI.OnePhysics;                    // BodyType
using RecursiveAI.OnePhysics.LowLevel;           // SdfGrid3D
using RecursiveAI.OnePhysics.Physics3D;          // PhysicsWorld3D, Body, Shape

namespace GuideExamples;

/// <summary>The code blocks of docs/guide/terrain.md about 3D voxel edits and signed distance fields.</summary>
public static class Terrain3D
{
    public static (int Dug, ulong Hash) Voxels()
    {
        var occupancy = new bool[64 * 8 * 64];
        for (int z = 0; z < 64; z++) for (int y = 0; y < 4; y++) for (int x = 0; x < 64; x++) occupancy[(z * 8 + y) * 64 + x] = true;
        var world = new PhysicsWorld3D();
        Body ground = world.AddVoxelGrid(64, 8, 64, occupancy, cellSize: 0.5f);
        Shape voxels = ground.Voxels;

        voxels.DigSphere(new Vector3D(10, 2, 10), 1.8);       // cells whose centres are inside
        voxels.SetVoxels(14, 0, 14, 4, 4, 4, solid: true);    // a block of cells, in cell coordinates
        for (int i = 0; i < 30; i++) world.Step(1f / 60);
        return (world.GetVoxelOccupancy(voxels).Length, world.ComputeHash());
    }

    public static (double BallY, ulong Hash) Sdf()
    {
        var vertices = new Vector3F[] { new(-1, -1, -1), new(1, -1, -1), new(1, 1, -1), new(-1, 1, -1), new(-1, -1, 1), new(1, -1, 1), new(1, 1, 1), new(-1, 1, 1) };
        var indices = new int[] { 0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 2, 3, 7, 2, 7, 6, 0, 4, 7, 0, 7, 3, 1, 2, 6, 1, 6, 5 };
        var world = new PhysicsWorld3D();
        SdfGrid3D hill = SdfGrid3D.FromFunction(new(-8, -3, -8), new(8, 2, 8), 0.25f,
            SdfGrid3D.Union(SdfGrid3D.HalfSpace(default, new(0, 1, 0)), SdfGrid3D.Sphere(new(0, 0, 0), 2f)));
        world.AddSdf(hill);                                   // static terrain

        SdfGrid3D ball = SdfGrid3D.FromMesh(vertices, indices, cellSize: 0.1f);   // a closed, consistently oriented mesh
        Body body = world.AddSdf(ball, at: new Vector3D(0, 5, 0), type: BodyType.Dynamic);    // mass from the inside volume
        for (int i = 0; i < 240; i++) world.Step(1f / 60);
        return (body.Position.Y, world.ComputeHash());
    }
}
