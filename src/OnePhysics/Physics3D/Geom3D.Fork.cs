using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Geom3D
{
    internal Geom3D CloneForFork()
    {
        // A voxel grid is edited in place by its world, so a fork shares it only until either side edits it.
        if (Soup is VoxelGrid3D grid) { grid.MarkShared(); }
        var c = (Geom3D)MemberwiseClone();
        c.Recipe = (byte[])Recipe.Clone();
        c.FastHull = (float[]?)FastHull?.Clone();
        c.Buoyancy = null;
        return c;
    }

    /// <summary>A copy for a shape that is about to edit a voxel grid it shares with another shape: its own recipe bytes and its own grid.</summary>
    internal Geom3D CloneForEdit()
    {
        var c = (Geom3D)MemberwiseClone();
        c.Recipe = (byte[])Recipe.Clone();
        c.Buoyancy = null;
        if (Soup is VoxelGrid3D grid) { c.Soup = grid.CloneForEdit(); }
        return c;
    }
}
