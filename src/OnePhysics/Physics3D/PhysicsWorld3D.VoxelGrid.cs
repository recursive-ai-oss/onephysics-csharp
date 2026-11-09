using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>Adds immutable static voxel terrain at a double-precision position within ±2^21 metres. Occupancy is x-fastest then y then z. Dimensions are 1..1024 with at most 2^20 cells, cell size at least 1e-5 and extent at most 2^20 metres. Throws ArgumentOutOfRangeException for invalid input. Default rotation is identity; default material is Material3D.Default. Optional scalar materials are copied with one index per voxel, including empty cells; indexed entries require default Surface properties.</summary>
    public Body AddVoxelGrid(int sizeX, int sizeY, int sizeZ, ReadOnlySpan<bool> occupancy, float cellSize = 1f, Vector3D at = default, QuaternionF rotation = default, Material3D? material = null, ReadOnlySpan<Material3D> materials = default, ReadOnlySpan<int> materialIndices = default)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.AddVoxelGrid");
        var grid=new VoxelGrid3D(sizeX,sizeY,sizeZ,occupancy,cellSize);
        var body=CreateBody(BodyType.Static,at,rotation,nameof(at));
        try {CreateVoxelGrid(body,grid,ShapeFor(material,0f),materials,materialIndices);}
        catch(Exception){body.Destroy();throw;}
        return body;
    }

    /// <summary>Attaches immutable voxel terrain to a static body, with ShapeDef defaults. Throws ArgumentNullException for null, ArgumentException for a foreign body, StaticOnlyShapeException for a moving body and the standard shape-definition errors. The grid is copied into the world's geometry recipe. Optional scalar materials are copied with one index per voxel, including empty cells; indexed entries require default Surface properties.</summary>
    public Shape CreateVoxelGrid(Body body, VoxelGrid3D grid, ShapeDef? def = null, ReadOnlySpan<Material3D> materials = default, ReadOnlySpan<int> materialIndices = default)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.CreateVoxelGrid");
        ArgumentNullException.ThrowIfNull(grid);
        var table=GridMaterials.Convert(materials);var indices=Shape.ExpandVoxelMaterials(grid,materialIndices,table.Length);
        var shape=CreateShape(body,ShapeSpec3D.VoxelGrid(grid.SizeX,grid.SizeY,grid.SizeZ,grid.Occupancy,grid.CellSize),def);
        if(table.Length!=0)_core.SetContactMaterials(shape.RawHandle,table,indices);
        return shape;
    }
}
