using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public readonly partial struct Shape
{
    /// <summary>Assigns copied scalar materials with one index per voxel, including empty cells. Empty spans remove overrides. Indices expand to the twelve stable triangle ids of each voxel. Shape-wide Surface settings remain separate. Throws ArgumentException for a non-voxel shape, invalid lengths or per-material Surface settings, ArgumentOutOfRangeException for invalid indices, and InvalidHandleException for stale shapes.</summary>
    public void SetVoxelMaterials(ReadOnlySpan<Material3D> materials,ReadOnlySpan<int> indices)
    {
        using var mutation=_world?.RecordMutation("Shape.SetVoxelMaterials");int slot=Resolve();
        if(_world!.Engine.Geoms[_world.Engine.GeomOf[slot]]?.Soup is not VoxelGrid3D grid)throw new ArgumentException("Shape is not a voxel grid.");
        var table=GridMaterials.Convert(materials);var triangles=ExpandVoxelMaterials(grid,indices,table.Length);
        _world.Core.SetContactMaterials(_handle,table,triangles);
    }
    internal static int[] ExpandVoxelMaterials(VoxelGrid3D grid,ReadOnlySpan<int> indices,int materialCount)
    {
        if(indices.Length==0 && materialCount==0)return [];
        GridMaterials.Check(indices,grid.Occupancy.Length,materialCount);
        var triangles=new int[grid.TriangleIdCount];for(int i=0;i<indices.Length;i++)triangles.AsSpan(12*i,12).Fill(indices[i]);
        return triangles;
    }
}
