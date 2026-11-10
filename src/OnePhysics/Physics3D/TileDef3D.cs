using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>A height field of a streamed tile. Sample <c>(i, j)</c> sits at <c>Offset + (i * Scale.X, height * Scale.Y, j * Scale.Z)</c> from the tile's origin.</summary>
public sealed class TileHeightField3D
{
    /// <summary>Heights row by row (<c>j * SamplesX + i</c>), copied when the tile is queued.</summary>
    public float[] Heights { get; set; } = Array.Empty<float>();

    /// <summary>Samples along X, at least 2.</summary>
    public int SamplesX { get; set; }

    /// <summary>Samples along Z, at least 2.</summary>
    public int SamplesZ { get; set; }

    /// <summary>Spacing along X and Z and the multiplier of heights.</summary>
    public Vector3F Scale { get; set; } = new(1f, 1f, 1f);

    /// <summary>Position of the corner sample relative to the tile's origin.</summary>
    public Vector3F Offset { get; set; }

    /// <summary>Optional hole mask, one entry per cell.</summary>
    public bool[]? Holes { get; set; }

    /// <summary>
    /// The heights of the samples just outside the grid, from the neighbouring tiles' terrain. With them the border edges are judged like the edges inside the field, so a body crossing from one tile to the next sees no ghost bump.
    /// Null keeps the border edges active (correct at the edge of the world).
    /// </summary>
    public HeightFieldEdges3D? Edges { get; set; }

    /// <summary>Surface material; the world default when null.</summary>
    public Material3D? Material { get; set; }
}

/// <summary>A static triangle mesh of a streamed tile. Mesh borders are boundary edges: a body crossing two mesh tiles can meet a seam bump (use height fields for seamless terrain).</summary>
public sealed class TileMesh3D
{
    /// <summary>Vertices relative to <see cref="Offset"/>.</summary>
    public Vector3F[] Vertices { get; set; } = Array.Empty<Vector3F>();

    /// <summary>Triangle indices, three per triangle.</summary>
    public int[] Indices { get; set; } = Array.Empty<int>();

    /// <summary>Position relative to the tile's origin.</summary>
    public Vector3F Offset { get; set; }

    /// <summary>Surface material; the world default when null.</summary>
    public Material3D? Material { get; set; }
}

/// <summary>A static box of a streamed tile (a wall, a platform).</summary>
public sealed class TileBox3D
{
    /// <summary>Full size along each axis.</summary>
    public Vector3F Size { get; set; } = new(1f, 1f, 1f);

    /// <summary>Centre relative to the tile's origin.</summary>
    public Vector3F Offset { get; set; }

    /// <summary>Orientation; identity by default.</summary>
    public QuaternionF Rotation { get; set; }

    /// <summary>Surface material; the world default when null.</summary>
    public Material3D? Material { get; set; }
}

/// <summary>Static voxel terrain of a streamed tile.</summary>
public sealed class TileVoxelGrid3D
{
    /// <summary>Cells along X, Y and Z.</summary>
    public int SizeX { get; set; } = 1;

    /// <summary>Cells along Y.</summary>
    public int SizeY { get; set; } = 1;

    /// <summary>Cells along Z.</summary>
    public int SizeZ { get; set; } = 1;

    /// <summary>Occupancy, x fastest, then y, then z.</summary>
    public bool[] Occupancy { get; set; } = Array.Empty<bool>();

    /// <summary>Cell edge length.</summary>
    public float CellSize { get; set; } = 1f;

    /// <summary>Position relative to the tile's origin.</summary>
    public Vector3F Offset { get; set; }

    /// <summary>Surface material; the world default when null.</summary>
    public Material3D? Material { get; set; }
}

/// <summary>
/// The static geometry of one tile of a streamed world: a set of height fields, meshes, boxes and voxel grids around an origin. Hand it to <see cref="PhysicsWorld3D.LoadTile"/>; the world copies it.
/// Each part becomes one static body.
/// </summary>
public sealed class TileDef3D
{
    /// <summary>The tile's origin in world coordinates as of the call that queues it (the world origin may shift before the tile is built; the tile follows).</summary>
    public Vector3D Origin { get; set; }

    /// <summary>Height fields.</summary>
    public List<TileHeightField3D> HeightFields { get; } = new();

    /// <summary>Triangle meshes.</summary>
    public List<TileMesh3D> Meshes { get; } = new();

    /// <summary>Boxes.</summary>
    public List<TileBox3D> Boxes { get; } = new();

    /// <summary>Voxel grids.</summary>
    public List<TileVoxelGrid3D> VoxelGrids { get; } = new();

    /// <summary>The most parts one tile may have.</summary>
    public const int MaxParts = 4096;

    internal int PartCount => HeightFields.Count + Meshes.Count + Boxes.Count + VoxelGrids.Count;

    internal TileDef3D Copy()
    {
        var c = new TileDef3D { Origin = Origin };
        foreach (var h in HeightFields) { c.HeightFields.Add(new TileHeightField3D { Heights = (float[])h.Heights.Clone(), SamplesX = h.SamplesX, SamplesZ = h.SamplesZ, Scale = h.Scale, Offset = h.Offset, Holes = (bool[]?)h.Holes?.Clone(), Edges = h.Edges?.CopyPublic(), Material = h.Material }); }
        foreach (var m in Meshes) { c.Meshes.Add(new TileMesh3D { Vertices = (Vector3F[])m.Vertices.Clone(), Indices = (int[])m.Indices.Clone(), Offset = m.Offset, Material = m.Material }); }
        foreach (var b in Boxes) { c.Boxes.Add(new TileBox3D { Size = b.Size, Offset = b.Offset, Rotation = b.Rotation, Material = b.Material }); }
        foreach (var v in VoxelGrids) { c.VoxelGrids.Add(new TileVoxelGrid3D { SizeX = v.SizeX, SizeY = v.SizeY, SizeZ = v.SizeZ, Occupancy = (bool[])v.Occupancy.Clone(), CellSize = v.CellSize, Offset = v.Offset, Material = v.Material }); }
        return c;
    }
}
