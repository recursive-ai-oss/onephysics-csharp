using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Debug;

/// <summary>One grid of shallow water as plain data: its place in the world and where its cells' heights are in <see cref="DebugGeometry.WaterHeights"/>.</summary>
public readonly struct DebugWaterSurface
{
    /// <summary>Creates an entry.</summary>
    public DebugWaterSurface(int index, int cellsX, int cellsZ, float cellSize, Vector3D origin, int firstCell)
    {
        Index = index; CellsX = cellsX; CellsZ = cellsZ; CellSize = cellSize; Origin = origin; FirstCell = firstCell;
    }

    /// <summary>The water's slot in the world.</summary>
    public int Index { get; }

    /// <summary>Cells along x.</summary>
    public int CellsX { get; }

    /// <summary>Cells along z (1 in a planar world, whose water lies along x with height along y).</summary>
    public int CellsZ { get; }

    /// <summary>Side of a cell, metres.</summary>
    public float CellSize { get; }

    /// <summary>The grid's minimum corner: x and z as stored in the world, y the height the grid's heights are measured from (its reference floor).</summary>
    public Vector3D Origin { get; }

    /// <summary>Index of the grid's first cell in <see cref="DebugGeometry.WaterHeights"/> and <see cref="DebugGeometry.WaterDepths"/>; its cells follow, x fastest, <c>CellsX * CellsZ</c> of them.</summary>
    public int FirstCell { get; }
}

public sealed partial class DebugGeometry
{
    private DebugWaterSurface[] _waterSurfaces = new DebugWaterSurface[2];
    private int _nWaterSurfaces;
    private float[] _waterHeights = new float[16], _waterDepths = new float[16];
    private int _nWaterCells;

    /// <summary>The shallow-water grids in slot order; valid until Clear or the next world collection. Filled by <see cref="DebugContent.ShallowWater"/>.</summary>
    public ReadOnlySpan<DebugWaterSurface> WaterSurfaces => _waterSurfaces.AsSpan(0, _nWaterSurfaces);

    /// <summary>The water surface height of every cell of every grid, metres above that grid's <see cref="DebugWaterSurface.Origin"/> y (the bed height where the cell is dry).</summary>
    public ReadOnlySpan<float> WaterHeights => _waterHeights.AsSpan(0, _nWaterCells);

    /// <summary>The water depth of every cell of every grid, metres; 0 where dry.</summary>
    public ReadOnlySpan<float> WaterDepths => _waterDepths.AsSpan(0, _nWaterCells);

    internal void ClearWater() { _nWaterSurfaces = 0; _nWaterCells = 0; }

    internal int AddWater(int index, int cellsX, int cellsZ, float cellSize, Vector3D origin)
    {
        int cells = cellsX * cellsZ;
        int first = _nWaterCells;
        if (_nWaterCells + cells > _waterHeights.Length)
        {
            int n = Math.Max(_nWaterCells + cells, _waterHeights.Length * 2);
            Array.Resize(ref _waterHeights, n); Array.Resize(ref _waterDepths, n);
        }

        _nWaterCells += cells;
        var surface = new DebugWaterSurface(index, cellsX, cellsZ, cellSize, origin, first);
        Push(ref _waterSurfaces, ref _nWaterSurfaces, in surface);
        return first;
    }

    internal Span<float> WaterHeightSlot(int first, int cells) => _waterHeights.AsSpan(first, cells);

    internal Span<float> WaterDepthSlot(int first, int cells) => _waterDepths.AsSpan(first, cells);
}
