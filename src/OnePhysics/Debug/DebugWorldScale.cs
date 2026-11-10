using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Debug;

/// <summary>One streamed tile as plain data: its key and which of the world's bodies make it up.</summary>
public readonly struct DebugTile
{
    /// <summary>Creates an entry.</summary>
    public DebugTile(TileKey key, bool unloading, int firstBody, int bodyCount) { Key = key; Unloading = unloading; FirstBody = firstBody; BodyCount = bodyCount; }

    /// <summary>The tile's key.</summary>
    public TileKey Key { get; }

    /// <summary>True while an unload waits for bodies to leave the tile.</summary>
    public bool Unloading { get; }

    /// <summary>Index of the tile's first body slot in <see cref="DebugGeometry.TileBodies"/>.</summary>
    public int FirstBody { get; }

    /// <summary>Number of body slots (one per part of the tile).</summary>
    public int BodyCount { get; }
}

/// <summary>What a <see cref="DebugScaleRegion"/> changes.</summary>
public enum DebugScaleRegionKind : byte
{
    /// <summary>Bodies inside step every <see cref="DebugScaleRegion.Period"/> steps.</summary>
    LodPeriod = 0,

    /// <summary>Bodies inside run at <see cref="DebugScaleRegion.Scale"/> times real time.</summary>
    TimeScale = 1,
}

/// <summary>A box of simulation level of detail or time scale as plain data, in world coordinates relative to the world's origin (z is zero in 2D).</summary>
public readonly struct DebugScaleRegion
{
    /// <summary>Creates an entry.</summary>
    public DebugScaleRegion(DebugScaleRegionKind kind, Vector3D minimum, Vector3D maximum, int period, float scale) { Kind = kind; Minimum = minimum; Maximum = maximum; Period = period; Scale = scale; }

    /// <summary>What the region changes.</summary>
    public DebugScaleRegionKind Kind { get; }

    /// <summary>Minimum corner.</summary>
    public Vector3D Minimum { get; }

    /// <summary>Maximum corner.</summary>
    public Vector3D Maximum { get; }

    /// <summary>The period of a <see cref="DebugScaleRegionKind.LodPeriod"/> region; 1 for a time scale.</summary>
    public int Period { get; }

    /// <summary>The scale of a <see cref="DebugScaleRegionKind.TimeScale"/> region; 1 for a level of detail.</summary>
    public float Scale { get; }
}

public sealed partial class DebugGeometry
{
    private DebugTile[] _tiles = new DebugTile[4];
    private int _nTiles;
    private int[] _tileBodies = new int[4];
    private int _nTileBodies;
    private DebugScaleRegion[] _scaleRegions = new DebugScaleRegion[4];
    private int _nScaleRegions;
    private Vector3D[] _lodFocus = new Vector3D[2];
    private int _nLodFocus;

    /// <summary>The streamed tiles in key order; valid until Clear or the next world collection. Filled by <see cref="DebugContent.WorldScale"/>.</summary>
    public ReadOnlySpan<DebugTile> Tiles => _tiles.AsSpan(0, _nTiles);

    /// <summary>The body slots of every tile, tile after tile (see <see cref="DebugTile.FirstBody"/>).</summary>
    public ReadOnlySpan<int> TileBodies => _tileBodies.AsSpan(0, _nTileBodies);

    /// <summary>The level-of-detail regions, then the time-scale regions, in the order they were set.</summary>
    public ReadOnlySpan<DebugScaleRegion> ScaleRegions => _scaleRegions.AsSpan(0, _nScaleRegions);

    /// <summary>The level-of-detail focus points, world coordinates relative to the world's origin.</summary>
    public ReadOnlySpan<Vector3D> LodFocusPoints => _lodFocus.AsSpan(0, _nLodFocus);

    internal void ClearWorldScale() { _nTiles = 0; _nTileBodies = 0; _nScaleRegions = 0; _nLodFocus = 0; }

    internal void CollectWorldScale(WorldCore core)
    {
        var s = core.ScaleStore;
        foreach (var t in s.Tiles)
        {
            if (_nTiles == _tiles.Length) { Array.Resize(ref _tiles, _nTiles * 2); }
            _tiles[_nTiles++] = new DebugTile(new TileKey(t.X, t.Y, t.Z), t.Unloading, _nTileBodies, t.Bodies.Length);
            foreach (int b in t.Bodies)
            {
                if (_nTileBodies == _tileBodies.Length) { Array.Resize(ref _tileBodies, _nTileBodies * 2); }
                _tileBodies[_nTileBodies++] = b;
            }
        }

        var o = core.Origin;
        double ox = o[0], oy = o[1], oz = o.Length == 3 ? o[2] : 0.0;
        for (int i = 0; i < s.LodBoxPeriod.Length; i++) { AddRegion(new DebugScaleRegion(DebugScaleRegionKind.LodPeriod, new(s.LodBoxes[6 * i] - ox, s.LodBoxes[6 * i + 1] - oy, s.LodBoxes[6 * i + 2] - oz), new(s.LodBoxes[6 * i + 3] - ox, s.LodBoxes[6 * i + 4] - oy, s.LodBoxes[6 * i + 5] - oz), s.LodBoxPeriod[i], 1f)); }
        for (int i = 0; i < s.ScaleBoxValue.Length; i++) { AddRegion(new DebugScaleRegion(DebugScaleRegionKind.TimeScale, new(s.ScaleBoxes[6 * i] - ox, s.ScaleBoxes[6 * i + 1] - oy, s.ScaleBoxes[6 * i + 2] - oz), new(s.ScaleBoxes[6 * i + 3] - ox, s.ScaleBoxes[6 * i + 4] - oy, s.ScaleBoxes[6 * i + 5] - oz), 1, s.ScaleBoxValue[i])); }
        for (int i = 0; i + 2 < s.Focus.Length; i += 3)
        {
            if (_nLodFocus == _lodFocus.Length) { Array.Resize(ref _lodFocus, _nLodFocus * 2); }
            _lodFocus[_nLodFocus++] = new Vector3D(s.Focus[i] - ox, s.Focus[i + 1] - oy, s.Focus[i + 2] - oz);
        }
    }

    private void AddRegion(in DebugScaleRegion r)
    {
        if (_nScaleRegions == _scaleRegions.Length) { Array.Resize(ref _scaleRegions, _nScaleRegions * 2); }
        _scaleRegions[_nScaleRegions++] = r;
    }
}
