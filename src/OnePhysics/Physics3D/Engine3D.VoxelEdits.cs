using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    private const int VoxelRecipeHeader = 1 + 3 * 4 + 4; // kind, three sizes, cell size

    /// <summary>The grid of a voxel shape, or null when the shape is something else.</summary>
    internal VoxelGrid3D? VoxelGridOf(int slot) => Geoms[GeomOf[slot]]?.Soup as VoxelGrid3D;

    /// <summary>
    /// Sets the listed cells (strictly increasing x-fastest indices) of a voxel shape. The grid, the shape's recipe bytes
    /// (what snapshots and hashes see) and the shape's per-cell materials change; faces away from the edit keep
    /// their ids. Contacts that touch the changed cells end so they are found again, and bodies near the region wake.
    /// Returns the number of cells that changed.
    /// </summary>
    internal int EditVoxels(int slot, ReadOnlySpan<int> cells, bool solid, int materialIndex, in Aabb3D region)
    {
        var geom = Geoms[GeomOf[slot]]!;
        if (geom.Soup is not VoxelGrid3D grid || geom.Recipe.Length < VoxelRecipeHeader || geom.Recipe[0] != ShapeSpec3D.KVoxelGrid) { throw new ArgumentException("The shape is not a voxel grid."); }
        if (geom.Refs > 1)
        {
            // Two shapes share this geometry (only possible after restoring an old snapshot): this one takes its own copy.
            var copy = geom.CloneForEdit(); copy.Refs = 1; geom.Refs--;
            int id = Geoms.IndexOf(null);
            if (id < 0) { id = Geoms.Count; Geoms.Add(copy); } else { Geoms[id] = copy; }
            GeomOf[slot] = id; geom = copy; grid = (VoxelGrid3D)copy.Soup!;
        }
        else if (grid.IsShared) { grid = grid.CloneForEdit(); geom.Soup = grid; }
        var result = grid.Edit(cells, solid);
        if (result is not { } r) { return 0; }
        foreach (int c in r.Changed) { geom.Recipe[VoxelRecipeHeader + c] = solid ? (byte)1 : (byte)0; }
        Core.ContactMaterials.EditSubshapes(slot, r.Changed, solid ? materialIndex : -1);
        ResetVoxelPairs(slot, grid, in r, in region);
        WakeBodiesIn(in region);
        return r.Changed.Length;
    }

    // Pairs of the voxel shape whose body reaches the edit region end (a merged flat contact names the first cell of
    // its patch, not the cell it rests on, so contacts cannot be matched to cells) and are found again on the next step; pairs
    // that had proven themselves apart forget it. Pairs of bodies farther away are untouched.
    private void ResetVoxelPairs(int slot, VoxelGrid3D grid, in VoxelGrid3D.EditResult r, in Aabb3D region)
    {
        var cons = Core.Constraints;
        for (int p = 0; p < PairHigh; p++)
        {
            if (!PairLive[p] || (PairA[p] != slot && PairB[p] != slot)) { continue; }
            bool soupIsA = PairA[p] == slot; int other = soupIsA ? PairB[p] : PairA[p];
            if (Core.Bodies.TypeOf(Core.Shapes.Owner[other]) == BodyType.Static) { continue; }
            ref readonly var fat = ref Bp.FatBoundsOfLive(Proxy[other]);
            if (fat.MinX > region.MaxX || fat.MaxX < region.MinX || fat.MinY > region.MaxY || fat.MaxY < region.MinY || fat.MinZ > region.MaxZ || fat.MaxZ < region.MinZ) { continue; }
            if (p < _skip.Length) { _skip[p].Clear = 0f; }
            for (int c = PairCons[p]; c >= 0;)
            {
                int next = ConsNext[c];
                Core.RemoveConstraint(c);
                c = next;
            }
            FreePair(p);
            Bp.Touch(Proxy[other]);
        }
    }

    private void WakeBodiesIn(in Aabb3D region)
    {
        var scratch = QueryBounds(in region, default(QueryFilter3D), int.MaxValue);
        try
        {
            for (int i = 0; i < scratch.Count; i++)
            {
                int owner = Core.Shapes.Owner[scratch.Hits[i].Slot];
                if (Core.Bodies.IsDynamic(owner)) { Core.WakeIsland(Core.Bodies.Island[owner]); Core.Bodies.SleepTime[owner] = 0f; }
            }
        }
        finally { scratch.Release(); }
    }
}
