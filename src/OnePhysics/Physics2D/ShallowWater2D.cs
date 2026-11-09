using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Water;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// Definition of a body of shallow water in a 2D world: a row of cells along x over a bed, holding water up to a level (the water has unit depth; its volume is an area, square metres). The world copies it.
/// The water obeys the one-dimensional shallow-water equations (waves, currents, wetting and drying), floats and pushes bodies through their buoyancy and drag, and is displaced by them.
/// </summary>
public sealed class ShallowWaterDef
{
    /// <summary>World x of the grid's low end, metres. Within ±2^21; rounded to the 2^-32 grid. Default 0.</summary>
    public double MinX { get; set; }

    /// <summary>Cells along x, 2 to 1048576. Default 128.</summary>
    public int Cells { get; set; } = 128;

    /// <summary>Width of a cell, metres, 0.001 to 1000. Default 0.5.</summary>
    public float CellSize { get; set; } = 0.5f;

    /// <summary>The lowest height of the bed, metres: the bed is never lower. It is also the height the grid's stored heights are measured from, so keep it near the water. Default 0.</summary>
    public double FloorHeight { get; set; }

    /// <summary>The surface height the water starts at, metres (world y); a cell whose bed is higher starts dry. Default 1.</summary>
    public double Level { get; set; } = 1.0;

    /// <summary>When true (default) the bed of each cell is the top of the static shapes under its centre, found by a ray cast straight down at creation, and <see cref="FloorHeight"/> where no shape is hit. Ignored when <see cref="Bed"/> is set.</summary>
    public bool SampleBed { get; set; } = true;

    /// <summary>How far above the higher of <see cref="Level"/> and <see cref="FloorHeight"/> the bed rays start, metres. Default 50.</summary>
    public double BedSearchHeight { get; set; } = 50.0;

    /// <summary>Explicit bed heights, metres, of the cells (each constant across its cell); heights below <see cref="FloorHeight"/> are raised to it. Null (default) samples or uses the floor.</summary>
    public float[]? Bed { get; set; }

    /// <summary>The end at the lowest x. Default a wall.</summary>
    public WaterBoundary West { get; set; }

    /// <summary>The end at the highest x. Default a wall.</summary>
    public WaterBoundary East { get; set; }

    /// <summary>Sources and drains along x (<see cref="WaterSource.Z"/> is ignored), at most 64. Default none.</summary>
    public WaterSource[] Sources { get; set; } = Array.Empty<WaterSource>();

    /// <summary>Density, kg/m² of unit-depth water, above 0 and at most 10^6. Default 1000.</summary>
    public float Density { get; set; } = 1000f;

    /// <summary>Quadratic drag coefficient of bodies against the water's motion, 0 to 10^6. Default 0.5.</summary>
    public float LinearDrag { get; set; } = 0.5f;

    /// <summary>Angular drag of bodies, 0 to 10^6. Default 0.5.</summary>
    public float AngularDrag { get; set; } = 0.5f;

    /// <summary>Viscous damping of bodies weighted by displaced mass, 1/s, 0 to 10^6. Default 1.</summary>
    public float ViscousDamping { get; set; } = 1f;

    /// <summary>Quadratic drag of the bed on the water (dimensionless, about 0.003 for a rough bed), 0 to 10^6. Default 0.</summary>
    public float BottomFriction { get; set; }

    /// <summary>Courant number of a substep, above 0 and at most 0.5. Default 0.25.</summary>
    public float Cfl { get; set; } = 0.25f;

    /// <summary>The most substeps one step may take (1 to 256). Default 16.</summary>
    public int MaxSubsteps { get; set; } = 16;

    /// <summary>Returns a copy (the bed and the sources are copied).</summary>
    public ShallowWaterDef Clone()
    {
        var c = (ShallowWaterDef)MemberwiseClone();
        c.Bed = (float[]?)Bed?.Clone();
        c.Sources = (WaterSource[])Sources.Clone();
        return c;
    }

    internal static WaterSourceSpec[] ConvertSources(WaterSource[] sources, double minX, string name)
    {
        ArgumentNullException.ThrowIfNull(sources);
        var r = new WaterSourceSpec[sources.Length];
        for (int i = 0; i < r.Length; i++)
        {
            var s = sources[i];
            if (!double.IsFinite(s.X) || Math.Abs(s.X - minX) > 4194304.0) { throw new ArgumentOutOfRangeException(name, "Source " + i.ToString(CultureInfo.InvariantCulture) + " has a position that is not finite or lies too far from the grid."); }
            r[i] = new WaterSourceSpec { X = (float)(s.X - minX), Z = 0f, Radius = s.Radius, Rate = s.Rate };
            WaterStore.ValidateSource(r[i], name);
        }

        return r;
    }

    internal WaterStore.Spec ToSpec(string name)
    {
        var s = new WaterStore.Spec
        {
            Nx = Cells, Nz = 1, CellSize = CellSize, OriginX = MinX, OriginZ = 0.0, FloorHeight = FloorHeight, Level = Level,
            SampleBed = SampleBed, Bed = (float[]?)Bed?.Clone(),
            Density = Density, LinearDrag = LinearDrag, AngularDrag = AngularDrag, ViscousDamping = ViscousDamping, BottomFriction = BottomFriction, Cfl = Cfl, MaxSubsteps = MaxSubsteps,
        };
        if (!double.IsFinite(BedSearchHeight) || BedSearchHeight < 0 || BedSearchHeight > 1.0e6) { throw new ArgumentOutOfRangeException(name, BedSearchHeight, "BedSearchHeight must be within 0..1e6 metres."); }
        s.SearchTop = Math.Max(Level, FloorHeight) + BedSearchHeight;
        var sides = new[] { West, East };
        for (int i = 0; i < 2; i++) { s.Edges[i] = (WaterEdge)(byte)sides[i].Kind; s.EdgeLevel[i] = (float)sides[i].Level; s.EdgeSpeed[i] = sides[i].Speed; }
        s.Sources = ConvertSources(Sources, MinX, name);
        return s;
    }
}

/// <summary>The water at a point of a 2D world.</summary>
public readonly struct WaterSample2D
{
    internal WaterSample2D(WaterHandle water, double surface, float depth, float velocity, Vector2F normal) { Water = water; Surface = surface; Depth = depth; Velocity = velocity; Normal = normal; }

    /// <summary>The water that holds the point.</summary>
    public WaterHandle Water { get; }

    /// <summary>Height of the surface above the point's x, metres (world y).</summary>
    public double Surface { get; }

    /// <summary>Depth of the water there, metres.</summary>
    public float Depth { get; }

    /// <summary>Velocity of the water there along x, m/s.</summary>
    public float Velocity { get; }

    /// <summary>The unit normal of the surface there, pointing up.</summary>
    public Vector2F Normal { get; }
}

/// <summary>Where a ray met the water surface in a 2D world.</summary>
public readonly struct WaterHit2D
{
    internal WaterHit2D(WaterHandle water, float distance, Vector2D point, Vector2F normal) { Water = water; Distance = distance; Point = point; Normal = normal; }

    /// <summary>The water that was hit.</summary>
    public WaterHandle Water { get; }

    /// <summary>Distance along the (unit) direction, metres.</summary>
    public float Distance { get; }

    /// <summary>The hit point in world space.</summary>
    public Vector2D Point { get; }

    /// <summary>The unit normal of the surface there, pointing up.</summary>
    public Vector2F Normal { get; }
}

/// <summary>A body of shallow water in a 2D world: the handle (world, slot, generation) of a row of cells. It allocates nothing and throws <see cref="StaleHandleException"/> once removed.</summary>
public readonly struct ShallowWater : IEquatable<ShallowWater>
{
    private readonly PhysicsWorld2D? _world;
    private readonly int _index, _generation;

    internal ShallowWater(PhysicsWorld2D world, int index, int generation) { _world = world; _index = index; _generation = generation; }

    private WaterGrid Grid
    {
        get
        {
            if (_world == null) { throw new InvalidHandleException("The default ShallowWater is not attached to a world."); }
            return _world.Engine.Water.Get(_index, _generation);
        }
    }

    /// <summary>True while the water exists.</summary>
    public bool IsValid => _world != null && _world.Engine.Water.Handles.IsAlive(_index, _generation);

    /// <summary>The raw handle.</summary>
    public WaterHandle RawHandle => new(_world?.Id ?? 0, _index, _generation);

    /// <summary>Number of cells.</summary>
    public int Cells => Grid.Nx;

    /// <summary>Width of a cell, metres.</summary>
    public float CellSize => Grid.Dx;

    /// <summary>World x of the low end, metres.</summary>
    public double MinX => Grid.OriginX;

    /// <summary>Total area of water in square metres, summed in cell order. Water that bodies displace is not counted.</summary>
    public double Volume
    {
        get
        {
            var g = Grid;
            double v = g.Volume();
            for (int r = 0; r < g.RecordCount; r++) { v -= g.Records[r].Volume; }
            return v;
        }
    }

    /// <summary>Water lost to negative depths, square metres. Normally 0.</summary>
    public double LostVolume => Grid.LostVolume;

    /// <summary>Depth of the water in a cell, metres; 0 where dry.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The cell is outside the grid.</exception>
    public float CellDepth(int ix)
    {
        var g = Grid;
        Check(g, ix);
        float h = g.H[ix];
        return h > WaterGrid.DryDepth ? h : 0f;
    }

    /// <summary>Height of the water surface over a cell's centre, metres (world y); the bed height where dry.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The cell is outside the grid.</exception>
    public double CellSurface(int ix)
    {
        var g = Grid;
        Check(g, ix);
        return g.BaseY + g.SurfaceOfCell(ix, 0);
    }

    private static void Check(WaterGrid g, int ix)
    {
        if ((uint)ix >= (uint)g.Nx) { throw new ArgumentOutOfRangeException(nameof(ix), "Cell " + ix.ToString(CultureInfo.InvariantCulture) + " is outside the " + g.Nx.ToString(CultureInfo.InvariantCulture) + "-cell grid."); }
    }

    /// <summary>The water at an x. False when x is outside the grid or the water there is dry.</summary>
    public bool TryGetSample(double x, out WaterSample2D sample)
    {
        sample = default;
        var g = Grid;
        if (!double.IsFinite(x) || !WaterStore.SampleAt(g, x, 0.0, out double surface, out float depth, out float u, out _) || !(depth > WaterGrid.DryDepth)) { return false; }
        WaterStore.SlopeAt(g, x, 0.0, g.Dx, out float sx, out _);
        float inv = 1f / float.Sqrt(1f + sx * sx);
        sample = new WaterSample2D(RawHandle, surface, depth, u, new Vector2F(-sx * inv, inv));
        return true;
    }

    /// <summary>Replaces the water with a still pool filled up to a height (nothing where the bed is higher). Bodies in the water keep floating (their displaced area stays recorded).</summary>
    /// <exception cref="ArgumentOutOfRangeException">The level is not finite or beyond ±2^21.</exception>
    public void Fill(double level)
    {
        using var replayMutation = _world?.RecordMutation("ShallowWater.Fill");
        var g = Grid;
        if (!double.IsFinite(level) || Math.Abs(level) > WaterStore.MaxCoordinate) { throw new ArgumentOutOfRangeException(nameof(level), level, "The level must be finite and within ±2^21 metres."); }
        g.Fill((float)(level - g.BaseY));
        for (int r = 0; r < g.RecordCount; r++) { var rec = g.Records[r]; g.AddVolume(rec.X0, rec.X1, rec.Z0, rec.Z1, rec.Volume); }
    }

    /// <summary>Adds water (negative removes) spread over an interval of the grid, in square metres.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A bound or the volume is not finite.</exception>
    public void AddVolume(double minX, double maxX, float volume)
    {
        using var replayMutation = _world?.RecordMutation("ShallowWater.AddVolume");
        var g = Grid;
        if (!double.IsFinite(minX) || !double.IsFinite(maxX) || !float.IsFinite(volume) || Math.Abs(volume) > 1e12f) { throw new ArgumentOutOfRangeException(nameof(volume), volume, "The interval and the volume must be finite (volume within ±1e12)."); }
        g.AddVolume((float)(minX - g.OriginX), (float)(maxX - g.OriginX), 0f, 1f, volume);
    }

    /// <summary>Sets what happens at an end (0 west, 1 east).</summary>
    /// <exception cref="ArgumentOutOfRangeException">The side is not 0 or 1, or the boundary is invalid.</exception>
    public void SetBoundary(int side, WaterBoundary boundary)
    {
        using var replayMutation = _world?.RecordMutation("ShallowWater.SetBoundary");
        var g = Grid;
        if ((uint)side > 1u) { throw new ArgumentOutOfRangeException(nameof(side), side, "side must be 0 (west) or 1 (east)."); }
        if (!Enum.IsDefined(boundary.Kind) || !double.IsFinite(boundary.Level) || Math.Abs(boundary.Level) > WaterStore.MaxCoordinate || !(Math.Abs(boundary.Speed) <= 1000f)) { throw new ArgumentOutOfRangeException(nameof(boundary), "The boundary kind is unknown, or its level or speed is out of range."); }
        g.Edges[side] = (WaterEdge)(byte)boundary.Kind; g.EdgeLevel[side] = (float)(boundary.Level - g.BaseY); g.EdgeSpeed[side] = boundary.Speed;
    }

    /// <summary>Replaces the sources and drains (at most 64).</summary>
    /// <exception cref="ArgumentOutOfRangeException">A source is not finite, or there are too many.</exception>
    public void SetSources(ReadOnlySpan<WaterSource> sources)
    {
        using var replayMutation = _world?.RecordMutation("ShallowWater.SetSources");
        var g = Grid;
        if (sources.Length > WaterGrid.MaxSources) { throw new ArgumentOutOfRangeException(nameof(sources), sources.Length, "At most 64 sources."); }
        var specs = ShallowWaterDef.ConvertSources(sources.ToArray(), g.OriginX, nameof(sources));
        g.Sources = specs;
    }

    /// <summary>Samples the bed again from the static shapes now in the world; the water keeps its depth.</summary>
    /// <param name="searchHeight">How far above the grid's floor the rays start, metres. Default 50.</param>
    public void ResampleBed(double searchHeight = 50.0)
    {
        using var replayMutation = _world?.RecordMutation("ShallowWater.ResampleBed");
        var g = Grid;
        _world!.Engine.Water.ResampleBed(g, g.BaseY + searchHeight + 1.0);
    }

    /// <summary>Removes the water. The area of bodies that displaced it is simply forgotten.</summary>
    /// <exception cref="StaleHandleException">Already removed.</exception>
    public void Remove()
    {
        using var replayMutation = _world?.RecordMutation("ShallowWater.Remove");
        _ = Grid;
        _world!.Engine.Water.Destroy(_index, _generation);
    }

    /// <inheritdoc/>
    public bool Equals(ShallowWater other) => ReferenceEquals(_world, other._world) && _index == other._index && _generation == other._generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is ShallowWater w && Equals(w);

    /// <inheritdoc/>
    public override int GetHashCode() => HashCode.Combine(_world?.Id ?? 0, _index, _generation);

    /// <summary>Equality of world, slot and generation.</summary>
    public static bool operator ==(ShallowWater a, ShallowWater b) => a.Equals(b);

    /// <summary>Inequality of any part.</summary>
    public static bool operator !=(ShallowWater a, ShallowWater b) => !a.Equals(b);
}
