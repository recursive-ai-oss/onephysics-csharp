using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>Adds immutable static voxel terrain at a double-precision position within ±2^21 metres. Occupancy is x-fastest then y then z. Dimensions are 1..1024 with at most 2^20 cells, cell size at least 1e-5 and extent at most 2^20 metres. Throws ArgumentOutOfRangeException for invalid input. Default rotation is identity; default material is Material3D.Default.</summary>
    public Body AddVoxelGrid(int sizeX, int sizeY, int sizeZ, ReadOnlySpan<bool> occupancy, float cellSize = 1f, Vector3D at = default, QuaternionF rotation = default, Material3D? material = null)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.AddVoxelGrid");
        var spec = ShapeSpec3D.VoxelGrid(sizeX,sizeY,sizeZ,occupancy,cellSize);
        return AddOne(BodyType.Static,at,rotation,spec,material,0f);
    }

    /// <summary>Attaches immutable voxel terrain to a static body, with ShapeDef defaults. Throws ArgumentNullException for null, ArgumentException for a foreign body, StaticOnlyShapeException for a moving body and the standard shape-definition errors. The grid is copied into the world's geometry recipe.</summary>
    public Shape CreateVoxelGrid(Body body, VoxelGrid3D grid, ShapeDef? def = null)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.CreateVoxelGrid");
        ArgumentNullException.ThrowIfNull(grid);
        return CreateShape(body,ShapeSpec3D.VoxelGrid(grid.SizeX,grid.SizeY,grid.SizeZ,grid.Occupancy,grid.CellSize),def);
    }
}
