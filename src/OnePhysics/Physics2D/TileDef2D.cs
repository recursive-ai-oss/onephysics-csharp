using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>A height field of a streamed 2D tile: sample <c>i</c> sits at <c>Offset + (i * Spacing, height)</c> from the tile's origin.</summary>
public sealed class TileHeightField2D
{
    /// <summary>Heights, copied when the tile is queued.</summary>
    public float[] Heights { get; set; } = Array.Empty<float>();

    /// <summary>Distance between samples.</summary>
    public float Spacing { get; set; } = 1f;

    /// <summary>Position of the first sample relative to the tile's origin.</summary>
    public Vector2F Offset { get; set; }

    /// <summary>Optional hole flags, one per interval.</summary>
    public bool[]? Holes { get; set; }

    /// <summary>Height of the sample just before the first one, from the neighbouring tile's terrain, so the seam is no ghost collision. Null keeps the end open.</summary>
    public float? HeightBefore { get; set; }

    /// <summary>Height of the sample just after the last one (see <see cref="HeightBefore"/>).</summary>
    public float? HeightAfter { get; set; }

    /// <summary>Surface material; the world default when null.</summary>
    public Material2D? Material { get; set; }
}

/// <summary>A tile grid of a streamed 2D tile. Separate grids are separate boundaries: a seam between two grid tiles can show a ghost where a flat floor continues across it (use height fields for seamless terrain).</summary>
public sealed class TileTileGrid2D
{
    /// <summary>Tiles along X.</summary>
    public int Width { get; set; } = 1;

    /// <summary>Tiles along Y.</summary>
    public int Height { get; set; } = 1;

    /// <summary>Tile kinds, x fastest.</summary>
    public TileKind2D[] Tiles { get; set; } = Array.Empty<TileKind2D>();

    /// <summary>Edge length of one tile.</summary>
    public float TileSize { get; set; } = 1f;

    /// <summary>Bottom-left corner relative to the tile's origin.</summary>
    public Vector2F Offset { get; set; }

    /// <summary>Surface material; the world default when null.</summary>
    public Material2D? Material { get; set; }
}

/// <summary>A static box of a streamed 2D tile.</summary>
public sealed class TileBox2D
{
    /// <summary>Full size.</summary>
    public Vector2F Size { get; set; } = new(1f, 1f);

    /// <summary>Centre relative to the tile's origin.</summary>
    public Vector2F Offset { get; set; }

    /// <summary>Angle in radians.</summary>
    public float Angle { get; set; }

    /// <summary>Surface material; the world default when null.</summary>
    public Material2D? Material { get; set; }
}

/// <summary>The static geometry of one tile of a streamed 2D world; see <see cref="PhysicsWorld2D.LoadTile"/>. Each part becomes one static body.</summary>
public sealed class TileDef2D
{
    /// <summary>The tile's origin in world coordinates as of the call that queues it.</summary>
    public Vector2D Origin { get; set; }

    /// <summary>Height fields.</summary>
    public List<TileHeightField2D> HeightFields { get; } = new();

    /// <summary>Tile grids.</summary>
    public List<TileTileGrid2D> TileGrids { get; } = new();

    /// <summary>Boxes.</summary>
    public List<TileBox2D> Boxes { get; } = new();

    /// <summary>The most parts one tile may have.</summary>
    public const int MaxParts = 4096;

    internal int PartCount => HeightFields.Count + TileGrids.Count + Boxes.Count;

    internal TileDef2D Copy()
    {
        var c = new TileDef2D { Origin = Origin };
        foreach (var h in HeightFields) { c.HeightFields.Add(new TileHeightField2D { Heights = (float[])h.Heights.Clone(), Spacing = h.Spacing, Offset = h.Offset, Holes = (bool[]?)h.Holes?.Clone(), HeightBefore = h.HeightBefore, HeightAfter = h.HeightAfter, Material = h.Material }); }
        foreach (var g in TileGrids) { c.TileGrids.Add(new TileTileGrid2D { Width = g.Width, Height = g.Height, Tiles = (TileKind2D[])g.Tiles.Clone(), TileSize = g.TileSize, Offset = g.Offset, Material = g.Material }); }
        foreach (var b in Boxes) { c.Boxes.Add(new TileBox2D { Size = b.Size, Offset = b.Offset, Angle = b.Angle, Material = b.Material }); }
        return c;
    }
}
