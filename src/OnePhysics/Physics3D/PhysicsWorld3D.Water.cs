using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Water;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>
    /// Adds a body of shallow water over a rectangle of the x–z plane, still, filled to a height (the happy path of <see cref="CreateShallowWater"/>). The bed is the top of the static shapes under it
    /// (ray-cast down at every cell centre now, so create the terrain first) and <paramref name="floor"/> elsewhere. Bodies float in it and displace it; waves, currents and wet and dry shores follow the shallow-water equations.
    /// It is named for the model: <see cref="AddWater"/> is an infinite flat pool with no motion of its own.
    /// </summary>
    /// <param name="minX">World x of the rectangle's low corner, metres.</param>
    /// <param name="minZ">World z of the low corner, metres.</param>
    /// <param name="maxX">World x of the high corner. The grid covers at least the rectangle, in whole cells.</param>
    /// <param name="maxZ">World z of the high corner.</param>
    /// <param name="level">Surface height, metres (world y).</param>
    /// <param name="cellSize">Side of a cell, metres. Default 0.5.</param>
    /// <param name="floor">The lowest bed height, metres. Default 10 m below <paramref name="level"/>.</param>
    /// <returns>A handle to the water.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite, the rectangle is empty or the grid would be too large (see <see cref="ShallowWaterDef"/>).</exception>
    public ShallowWater AddShallowWater(double minX, double minZ, double maxX, double maxZ, double level, float cellSize = 0.5f, double floor = double.NaN)
    {
        if (!double.IsFinite(minX) || !double.IsFinite(minZ) || !double.IsFinite(maxX) || !double.IsFinite(maxZ) || !(maxX > minX) || !(maxZ > minZ)) { throw new ArgumentOutOfRangeException(nameof(maxX), "The rectangle must be finite and not empty."); }
        if (!(cellSize >= 0.001f) || !float.IsFinite(cellSize)) { throw new ArgumentOutOfRangeException(nameof(cellSize), cellSize, "cellSize must be at least 0.001 metres."); }
        double cx = Math.Ceiling((maxX - minX) / cellSize), cz = Math.Ceiling((maxZ - minZ) / cellSize);
        if (cx > 1 << 20 || cz > 1 << 20) { throw new ArgumentOutOfRangeException(nameof(cellSize), cellSize, "The grid would have more than 1048576 cells along an axis."); }
        return CreateShallowWater(new ShallowWaterDef { MinX = minX, MinZ = minZ, CellsX = (int)Math.Max(cx, 2), CellsZ = (int)Math.Max(cz, 2), CellSize = cellSize, Level = level, FloorHeight = double.IsNaN(floor) ? level - 10.0 : floor });
    }

    /// <summary>Adds a body of shallow water from a definition (middle tier). The definition is copied; sleeping bodies stay as they are until the water moves them.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A field is out of range (see <see cref="ShallowWaterDef"/>).</exception>
    public ShallowWater CreateShallowWater(ShallowWaterDef definition)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.CreateShallowWater");
        ArgumentNullException.ThrowIfNull(definition);
        var spec = definition.ToSpec(nameof(definition));
        var (slot, gen) = _engine.Water.Create(spec, nameof(definition));
        return new ShallowWater(this, slot, gen);
    }

    /// <summary>Number of bodies of shallow water in the world.</summary>
    public int ShallowWaterCount => _engine.Water.Count;

    /// <summary>The body of shallow water behind a raw handle.</summary>
    /// <exception cref="InvalidHandleException">The handle is stale, null or from another world.</exception>
    public ShallowWater GetShallowWater(WaterHandle handle)
    {
        if (handle.IsNull || handle.World != Id || !_engine.Water.Handles.IsAlive(handle.Index, handle.Generation)) { throw new InvalidHandleException("Handle " + handle + " is stale, null or from another world."); }
        return new ShallowWater(this, handle.Index, handle.Generation);
    }

    /// <summary>The water at a point's x and z, from the first body of shallow water (in slot order) that holds it with water deeper than a millimetre. False when there is none.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The point is not finite.</exception>
    public bool QueryWater(Vector3D point, out WaterSample3D sample)
    {
        sample = default;
        if (!double.IsFinite(point.X) || !double.IsFinite(point.Y) || !double.IsFinite(point.Z)) { throw new ArgumentOutOfRangeException(nameof(point), point, "The point must be finite."); }
        var store = _engine.Water;
        if (!store.Find(point.X, point.Z, out int slot, out _, out _, out _)) { return false; }
        return new ShallowWater(this, slot, store.Handles.GenerationOf(slot)).TryGetSample(point.X, point.Z, out sample);
    }

    /// <summary>
    /// Casts a ray at the water surfaces and reports the nearest crossing, from above or below (dry cells have no surface). The surface is the bilinear height between cell centres, found by marching half a cell at a time and bisecting.
    /// </summary>
    /// <param name="origin">Ray origin.</param>
    /// <param name="direction">Direction; need not be unit length but must not be zero.</param>
    /// <param name="hit">The hit.</param>
    /// <param name="maxDistance">Longest distance along the normalized direction. Default unbounded.</param>
    /// <exception cref="ArgumentOutOfRangeException">The origin or direction is not finite or the direction is zero, or the distance is negative.</exception>
    public bool RaycastWater(Vector3D origin, Vector3F direction, out WaterHit3D hit, float maxDistance = float.PositiveInfinity)
    {
        hit = default;
        if (!double.IsFinite(origin.X) || !double.IsFinite(origin.Y) || !double.IsFinite(origin.Z)) { throw new ArgumentOutOfRangeException(nameof(origin), origin, "The origin must be finite."); }
        double l2 = (double)direction.X * direction.X + (double)direction.Y * direction.Y + (double)direction.Z * direction.Z;
        if (!double.IsFinite(l2) || !(l2 > 0)) { throw new ArgumentOutOfRangeException(nameof(direction), direction, "The direction must be finite and not zero."); }
        if (!(maxDistance >= 0f)) { throw new ArgumentOutOfRangeException(nameof(maxDistance), maxDistance, "The distance must not be negative."); }
        var store = _engine.Water;
        if (store.Count == 0) { return false; }
        float inv = (float)(1.0 / Math.Sqrt(l2));
        var d = new Vector3F(direction.X * inv, direction.Y * inv, direction.Z * inv);
        float limit = float.IsPositiveInfinity(maxDistance) ? 4.0e7f : maxDistance;
        if (!store.Raycast(origin, d, limit, out int slot, out float distance, out var point, out var normal)) { return false; }
        hit = new WaterHit3D(new WaterHandle(Id, slot, store.Handles.GenerationOf(slot)), distance, point, normal);
        return true;
    }

    /// <summary>The body-enters and body-leaves events of the last step, in body slot order within each water (valid until the next step; rebuilt on each read).</summary>
    public ReadOnlySpan<WaterEvent> WaterEvents
    {
        get
        {
            var ev = _engine.Water.Events;
            if (ev.Count == 0) { return default; }
            if (_waterEventScratch.Length < ev.Count) { _waterEventScratch = new WaterEvent[Math.Max(ev.Count, 2 * _waterEventScratch.Length)]; }
            for (int i = 0; i < ev.Count; i++)
            {
                var e = ev[i];
                _waterEventScratch[i] = new WaterEvent(e.Entered ? WaterEventKind.Entered : WaterEventKind.Left, new BodyHandle(Id, e.Body, e.BodyGeneration), new WaterHandle(Id, e.Water, e.WaterGeneration), e.Volume);
            }

            return _waterEventScratch.AsSpan(0, ev.Count);
        }
    }

    private WaterEvent[] _waterEventScratch = Array.Empty<WaterEvent>();

    /// <summary>Raised after a step, once for every body that entered or left water (unless callbacks are suppressed), in the order of <see cref="WaterEvents"/>.</summary>
    public event Action<WaterEvent>? BodyWaterChanged;

    private void DispatchWaterEvents()
    {
        var h = BodyWaterChanged;
        if (h == null) { return; }
        var events = WaterEvents.ToArray();
        for (int i = 0; i < events.Length; i++) { h(events[i]); }
    }
}
