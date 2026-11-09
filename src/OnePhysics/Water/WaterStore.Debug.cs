using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Debug;

namespace RecursiveAI.OnePhysics.Water;

internal sealed partial class WaterStore
{
    /// <summary>Emits every grid in slot order: its place, and its surface height and depth per cell.</summary>
    public void CollectDebug(DebugGeometry geometry)
    {
        for (int i = 0; i < Handles.HighWater; i++)
        {
            if (!Handles.IsLive(i)) { continue; }
            var g = Grids[i]!;
            int cells = g.Nx * g.Nz;
            int first = geometry.AddWater(i, g.Nx, g.Nz, g.Dx, new Vector3D(g.OriginX, g.BaseY, g.OriginZ));
            var heights = geometry.WaterHeightSlot(first, cells);
            var depths = geometry.WaterDepthSlot(first, cells);
            for (int k = 0; k < g.Nz; k++)
            {
                for (int x = 0; x < g.Nx; x++)
                {
                    int c = k * g.Nx + x;
                    heights[c] = g.H[c] + g.BedOfCell(x, k);
                    depths[c] = g.H[c] > WaterGrid.DryDepth ? g.H[c] : 0f;
                }
            }
        }
    }
}
