using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>A box in world coordinates whose bodies step every <see cref="Period"/> steps (1, 2, 4 or 8).</summary>
public readonly record struct LodRegion3D(Vector3D Minimum, Vector3D Maximum, int Period);

/// <summary>A box in world coordinates whose bodies run at <see cref="Scale"/> times real time (1/1024 to 4).</summary>
public readonly record struct TimeScaleRegion3D(Vector3D Minimum, Vector3D Maximum, float Scale);

public sealed partial class PhysicsWorld3D
{
    private TileStreamer<TileDef3D>? _tileStreamer;

    private TileStreamer<TileDef3D> Streamer => _tileStreamer ??= new TileStreamer<TileDef3D>(_core, CreateTileBodies, DestroyTileBody, RecordMutation);

    // ------------------------------------------------------------------ streaming tiles

    /// <summary>
    /// Queues a tile of static geometry under <paramref name="key"/>. The world builds it, in ascending key order together with the other queued tiles, at the start of the next step (or when the world is hashed, snapshotted, forked
    /// or <see cref="FlushTiles"/> is called), so the result does not depend on the order loads were requested in. Each part becomes one static body with its broadphase entries; nothing else is rebuilt.
    /// The definition is copied. A tile whose geometry turns out invalid is dropped when it is built (see <see cref="LastTileError"/>).
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="def"/> is null.</exception>
    /// <exception cref="ArgumentException">The definition has too many parts, an array of the wrong length, or a non-finite origin.</exception>
    /// <exception cref="InvalidOperationException">The key is already loaded, queued, or being unloaded.</exception>
    public void LoadTile(TileKey key, TileDef3D def)
    {
        ArgumentNullException.ThrowIfNull(def);
        if (def.PartCount > TileDef3D.MaxParts) { throw new ArgumentException("A tile has at most " + TileDef3D.MaxParts + " parts.", nameof(def)); }
        if (!double.IsFinite(def.Origin.X) || !double.IsFinite(def.Origin.Y) || !double.IsFinite(def.Origin.Z)) { throw new ArgumentException("The tile origin is not finite.", nameof(def)); }
        foreach (var h in def.HeightFields)
        {
            if (h.Heights == null || h.SamplesX < 2 || h.SamplesZ < 2 || (long)h.SamplesX * h.SamplesZ != h.Heights.Length) { throw new ArgumentException("A height field needs SamplesX x SamplesZ heights, each side at least 2.", nameof(def)); }
        }

        foreach (var m in def.Meshes)
        {
            if (m.Vertices == null || m.Indices == null || m.Indices.Length % 3 != 0) { throw new ArgumentException("A mesh needs vertices and a multiple of three indices.", nameof(def)); }
        }

        foreach (var v in def.VoxelGrids)
        {
            if (v.Occupancy == null || v.SizeX < 1 || v.SizeY < 1 || v.SizeZ < 1 || (long)v.SizeX * v.SizeY * v.SizeZ != v.Occupancy.Length) { throw new ArgumentException("A voxel grid needs SizeX x SizeY x SizeZ occupancy entries.", nameof(def)); }
        }

        var o = _core.Origin;
        Streamer.Load(key, def.Copy(), def.Origin.X + o[0], def.Origin.Y + o[1], def.Origin.Z + o[2]);
    }

    /// <summary>
    /// Queues the removal of a tile. With <see cref="TileUnloadPolicy.WaitForContacts"/> (the default) the tile stays while any body touches it and goes, at the start of a later step, once nothing does; with
    /// <see cref="TileUnloadPolicy.Immediate"/> it goes at the next flush whatever touches it. Cancels a queued load of the same key. Returns false when there is nothing to unload.
    /// </summary>
    public bool UnloadTile(TileKey key, TileUnloadPolicy policy = TileUnloadPolicy.WaitForContacts)
    {
        if (policy != TileUnloadPolicy.WaitForContacts && policy != TileUnloadPolicy.Immediate) { throw new ArgumentOutOfRangeException(nameof(policy), policy, "Unknown unload policy."); }
        return Streamer.Unload(key, policy);
    }

    /// <summary>Applies queued tile loads and unloads now, in key order (every step does this first). Allocation-free when nothing is queued.</summary>
    public void FlushTiles() => _core.FlushForHost();

    /// <summary>The state of a tile; queued loads and unloads count as not yet happened.</summary>
    public TileState GetTileState(TileKey key) => _tileStreamer == null ? TileState.Absent : _tileStreamer.StateOf(key);

    /// <summary>True when the tile is built and is not being unloaded.</summary>
    public bool IsTileLoaded(TileKey key) => GetTileState(key) == TileState.Loaded;

    /// <summary>The number of built tiles (those being unloaded included).</summary>
    public int TileCount => _core.ScaleStore.Tiles.Count;

    /// <summary>The keys of the built tiles, ascending.</summary>
    public TileKey[] GetLoadedTiles()
    {
        var t = _core.ScaleStore.Tiles;
        var keys = new TileKey[t.Count];
        for (int i = 0; i < keys.Length; i++) { keys[i] = new TileKey(t[i].X, t[i].Y, t[i].Z); }
        return keys;
    }

    /// <summary>The static bodies of a built tile, in the order its parts were defined (height fields, meshes, boxes, voxel grids). Empty when the tile is not built.</summary>
    public Body[] GetTileBodies(TileKey key)
    {
        int at = _core.ScaleStore.FindTile(key.X, key.Y, key.Z);
        if (at < 0) { return Array.Empty<Body>(); }
        var slots = _core.ScaleStore.Tiles[at].Bodies;
        var bodies = new Body[slots.Length];
        for (int i = 0; i < slots.Length; i++) { bodies[i] = GetBody(_core.HandleOfBody(slots[i])); }
        return bodies;
    }

    /// <summary>Requests queued and not yet built.</summary>
    public int PendingTileRequests => _tileStreamer?.PendingCount ?? 0;

    /// <summary>Why the last tile could not be built (invalid geometry), or null.</summary>
    public string? LastTileError => _tileStreamer?.LastError;

    /// <summary>How many queued tiles were dropped because their geometry was invalid.</summary>
    public int FailedTileLoads => _tileStreamer?.FailedLoads ?? 0;

    private int[] CreateTileBodies(TileDef3D def, double ox, double oy, double oz)
    {
        var made = new List<Body>();
        try
        {
            foreach (var h in def.HeightFields)
            {
                var body = CreateBody(BodyType.Static, new Vector3D(ox + h.Offset.X, oy + h.Offset.Y, oz + h.Offset.Z), default, "tile");
                made.Add(body);
                AddShape(body, Geom3D.HeightFieldRecipe(h.Heights, h.SamplesX, h.SamplesZ, h.Scale, h.Holes ?? default, 0.0, h.Edges), ShapeFor(h.Material, 0f));
            }

            foreach (var m in def.Meshes)
            {
                var body = CreateBody(BodyType.Static, new Vector3D(ox + m.Offset.X, oy + m.Offset.Y, oz + m.Offset.Z), default, "tile");
                made.Add(body);
                AddShape(body, Geom3D.MeshRecipe(m.Vertices, m.Indices, 0.0), ShapeFor(m.Material, 0f));
            }

            foreach (var b in def.Boxes)
            {
                var spec = ShapeSpec3D.Box(b.Size);
                var body = CreateBody(BodyType.Static, new Vector3D(ox + b.Offset.X, oy + b.Offset.Y, oz + b.Offset.Z), b.Rotation, "tile");
                made.Add(body);
                CreateShape(body, spec, ShapeFor(b.Material, 0f));
            }

            foreach (var v in def.VoxelGrids)
            {
                var grid = new VoxelGrid3D(v.SizeX, v.SizeY, v.SizeZ, v.Occupancy, v.CellSize);
                var body = CreateBody(BodyType.Static, new Vector3D(ox + v.Offset.X, oy + v.Offset.Y, oz + v.Offset.Z), default, "tile");
                made.Add(body);
                CreateVoxelGrid(body, grid, ShapeFor(v.Material, 0f));
            }
        }
        catch (Exception)
        {
            foreach (var body in made) { body.Destroy(); }
            throw;
        }

        var slots = new int[made.Count];
        for (int i = 0; i < slots.Length; i++) { slots[i] = made[i].RawHandle.Index; }
        return slots;
    }

    private void DestroyTileBody(int slot) => GetBody(_core.HandleOfBody(slot)).Destroy();

    // ------------------------------------------------------------------ simulation level of detail and time scale

    /// <summary>
    /// Sets the simulation level of detail. A body's period is the smallest period of the <paramref name="regions"/> that hold it; outside every region it comes from the <paramref name="bands"/> by distance to the nearest focus point
    /// (a period of <paramref name="farPeriod"/> beyond the last band, 1 when there are no bands or focus points). An island steps every period-th step (when <c>StepCount + 1</c> is a multiple of the period) with <c>dt * period</c>, at the finest period of its bodies;
    /// an island touching a kinematic body, a vehicle or a machine the engine drives each step keeps period 1. Between its steps an island does not move, collide or wake. <paramref name="substepFactor"/> multiplies the substeps of such a step (0, the default, is 2 at period 8 and 1 below; 1 to 8 sets it).
    /// Positions are world coordinates (they follow <see cref="ShiftOrigin"/>).
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">A period is not 1, 2, 4 or 8, a distance or coordinate is invalid, or the factor is outside 1 to 8.</exception>
    /// <exception cref="ArgumentException">Too many focus points (64), bands (16) or regions (256), or an inverted region.</exception>
    public void SetSimulationLod(ReadOnlySpan<Vector3D> focusPoints, ReadOnlySpan<LodBand> bands, int farPeriod = 8, ReadOnlySpan<LodRegion3D> regions = default, int substepFactor = 0)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.SetSimulationLod");
        var focus = new double[3 * focusPoints.Length];
        for (int i = 0; i < focusPoints.Length; i++) { focus[3 * i] = focusPoints[i].X; focus[3 * i + 1] = focusPoints[i].Y; focus[3 * i + 2] = focusPoints[i].Z; }
        var dist = new float[bands.Length]; var per = new int[bands.Length];
        for (int i = 0; i < bands.Length; i++) { dist[i] = bands[i].MaxDistance; per[i] = bands[i].Period; }
        var boxes = new double[6 * regions.Length]; var rp = new int[regions.Length];
        for (int i = 0; i < regions.Length; i++)
        {
            var r = regions[i];
            boxes[6 * i] = r.Minimum.X; boxes[6 * i + 1] = r.Minimum.Y; boxes[6 * i + 2] = r.Minimum.Z; boxes[6 * i + 3] = r.Maximum.X; boxes[6 * i + 4] = r.Maximum.Y; boxes[6 * i + 5] = r.Maximum.Z;
            rp[i] = r.Period;
        }

        _core.SetSimulationLod(focus, dist, per, farPeriod, boxes, rp, substepFactor);
    }

    /// <summary>Moves the focus points of the level of detail (same count as before is not required); the bands and regions stay.</summary>
    public void SetLodFocus(ReadOnlySpan<Vector3D> focusPoints)
    {
        var s = _core.ScaleStore;
        var bands = new LodBand[s.BandPeriod.Length];
        for (int i = 0; i < bands.Length; i++) { bands[i] = new LodBand(s.BandDistance[i], s.BandPeriod[i]); }
        var regions = new LodRegion3D[s.LodBoxPeriod.Length];
        for (int i = 0; i < regions.Length; i++) { var b = s.LodBoxes; regions[i] = new LodRegion3D(new(b[6 * i], b[6 * i + 1], b[6 * i + 2]), new(b[6 * i + 3], b[6 * i + 4], b[6 * i + 5]), s.LodBoxPeriod[i]); }
        SetSimulationLod(focusPoints, bands, s.FarPeriod, regions, s.LodSubstepFactor);
    }

    /// <summary>Removes the level of detail: every island steps every step.</summary>
    public void ClearSimulationLod() { using var replayMutation = RecordMutation("PhysicsWorld3D.ClearSimulationLod"); _core.ClearSimulationLod(); }

    /// <summary>
    /// Sets the time-scale regions: a body inside runs at <c>Scale</c> times real time, multiplied by its own <see cref="Body.TimeScale"/>; an island runs at the smallest scale among its bodies, so a fast body entering a slow bubble slows down
    /// with everything it touches and momentum is conserved. An island advances <c>dt * scale</c> each step (scales above 1 add substeps to keep each substep as short).
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">A scale is outside [1/1024, 4] or a coordinate is not finite.</exception>
    public void SetTimeScaleRegions(ReadOnlySpan<TimeScaleRegion3D> regions)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.SetTimeScaleRegions");
        var boxes = new double[6 * regions.Length]; var scale = new float[regions.Length];
        for (int i = 0; i < regions.Length; i++)
        {
            var r = regions[i];
            boxes[6 * i] = r.Minimum.X; boxes[6 * i + 1] = r.Minimum.Y; boxes[6 * i + 2] = r.Minimum.Z; boxes[6 * i + 3] = r.Maximum.X; boxes[6 * i + 4] = r.Maximum.Y; boxes[6 * i + 5] = r.Maximum.Z;
            scale[i] = r.Scale;
        }

        _core.SetTimeScaleRegions(boxes, scale);
    }
}
