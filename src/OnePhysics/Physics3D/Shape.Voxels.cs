using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public readonly partial struct Shape
{
    private PhysicsWorld3D VoxelWorld => _world ?? throw new InvalidHandleException("The default Shape is not attached to a world.");

    /// <summary>Sets one cell of a voxel-grid shape; see <see cref="PhysicsWorld3D.SetVoxels"/>. Returns the number of cells that changed (0 or 1).</summary>
    public int SetVoxel(int x, int y, int z, bool solid = true) => VoxelWorld.SetVoxel(this, x, y, z, solid);

    /// <summary>Sets a block of cells of a voxel-grid shape; see <see cref="PhysicsWorld3D.SetVoxels"/>.</summary>
    public int SetVoxels(int x, int y, int z, int width, int height, int depth, bool solid = true) => VoxelWorld.SetVoxels(this, x, y, z, width, height, depth, solid);

    /// <summary>Clears the cells whose centres lie within a world-space sphere (a hole); see <see cref="PhysicsWorld3D.StampSphere"/>.</summary>
    public int DigSphere(Vector3D center, double radius) => VoxelWorld.StampSphere(this, center, radius, false);

    /// <summary>Fills the cells whose centres lie within a world-space sphere; see <see cref="PhysicsWorld3D.StampSphere"/>.</summary>
    public int FillSphere(Vector3D center, double radius) => VoxelWorld.StampSphere(this, center, radius, true);

    /// <summary>Clears or fills the cells whose centres lie inside a world-space box; see <see cref="PhysicsWorld3D.StampBox"/>.</summary>
    public int StampBox(Vector3D center, Vector3F halfExtents, QuaternionF rotation = default, bool solid = false) => VoxelWorld.StampBox(this, center, halfExtents, rotation, solid);

    /// <summary>Turns pieces no longer connected to an anchored face into dynamic bodies; see <see cref="PhysicsWorld3D.DetachVoxelIslands"/>.</summary>
    public Body[] DetachIslands(VoxelAnchors anchors = VoxelAnchors.Bottom, int minCells = 1, int maxBodies = 64) => VoxelWorld.DetachVoxelIslands(this, anchors, minCells, maxBodies);
}

public readonly partial struct Body
{
    /// <summary>The first voxel-grid shape of the body, for runtime edits. Throws ArgumentException when the body has none.</summary>
    public Shape Voxels { get { var (w, _) = Resolve(); return w.GetVoxelShape(this); } }
}
