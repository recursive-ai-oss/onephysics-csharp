using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Water;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// Definition of a body of shallow water: a regular grid of square cells in the x–z plane over a bed, holding water up to a level. The world copies it. The water obeys the shallow-water equations
/// (waves, currents, wetting and drying), floats and pushes bodies through their buoyancy and drag, and is displaced by them. Every value is validated when the water is created.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public sealed class ShallowWaterDef
{
    /// <summary>World x of the grid's minimum corner, metres. Within ±2^21; rounded to the 2^-32 grid. Default 0.</summary>
    public double MinX { get; set; }

    /// <summary>World z of the grid's minimum corner, metres. Default 0.</summary>
    public double MinZ { get; set; }

    /// <summary>Cells along x, 2 to 1048576. Default 64. A grid holds at most 4194304 cells.</summary>
    public int CellsX { get; set; } = 64;

    /// <summary>Cells along z, 2 to 1048576. Default 64.</summary>
    public int CellsZ { get; set; } = 64;

    /// <summary>Side of a cell, metres, 0.001 to 1000. Default 0.5. Waves shorter than a few cells are not resolved.</summary>
    public float CellSize { get; set; } = 0.5f;

    /// <summary>
    /// The lowest height of the bed, metres: the bed is never lower, and nothing under it is simulated. It is also the height the grid's stored heights are measured from, so keep it near the water (a few hundred metres at most
    /// for full float precision). Default 0.
    /// </summary>
    public double FloorHeight { get; set; }

    /// <summary>The surface height the water starts at, metres (world y); a cell whose bed is higher starts dry. Default 1.</summary>
    public double Level { get; set; } = 1.0;

    /// <summary>When true (default) the bed of each cell is the top of the static shapes under its centre, found by a ray cast straight down at creation, and <see cref="FloorHeight"/> where no shape is hit. Ignored when <see cref="Bed"/> is set.</summary>
    public bool SampleBed { get; set; } = true;

    /// <summary>How far above the higher of <see cref="Level"/> and <see cref="FloorHeight"/> the bed rays start, metres. Shapes above that (a bridge) do not count as bed. Default 50.</summary>
    public double BedSearchHeight { get; set; } = 50.0;

    /// <summary>Explicit bed heights, metres, of the CellsX × CellsZ cells (each constant across its cell), x fastest; heights below <see cref="FloorHeight"/> are raised to it. Null (default) samples or uses the floor.</summary>
    public float[]? Bed { get; set; }

    /// <summary>The side at the grid's lowest x. Default a wall.</summary>
    public WaterBoundary West { get; set; }

    /// <summary>The side at the grid's highest x. Default a wall.</summary>
    public WaterBoundary East { get; set; }

    /// <summary>The side at the grid's lowest z. Default a wall.</summary>
    public WaterBoundary South { get; set; }

    /// <summary>The side at the grid's highest z. Default a wall.</summary>
    public WaterBoundary North { get; set; }

    /// <summary>Sources and drains, at most 64. Default none.</summary>
    public WaterSource[] Sources { get; set; } = Array.Empty<WaterSource>();

    /// <summary>Density, kg/m³, above 0 and at most 10^6. Default 1000.</summary>
    public float Density { get; set; } = 1000f;

    /// <summary>Quadratic drag coefficient of bodies against the water's motion, 0 to 10^6. Default 0.5.</summary>
    public float LinearDrag { get; set; } = 0.5f;

    /// <summary>Angular drag of bodies, 0 to 10^6. Default 0.5.</summary>
    public float AngularDrag { get; set; } = 0.5f;

    /// <summary>Viscous damping of bodies weighted by displaced mass, 1/s, 0 to 10^6. Default 1, which settles floating bodies in a few seconds.</summary>
    public float ViscousDamping { get; set; } = 1f;

    /// <summary>Quadratic drag of the bed on the water (dimensionless, about 0.003 for a rough bed), 0 to 10^6. Default 0: no bed friction.</summary>
    public float BottomFriction { get; set; }

    /// <summary>Courant number of a substep: the largest wave crosses this fraction of a cell, above 0 and at most 0.5. Default 0.25.</summary>
    public float Cfl { get; set; } = 0.25f;

    /// <summary>The most substeps one step may take (1 to 256). A step that needs more is stretched to the cap and is then less accurate; water lost to negative depths is counted by <see cref="ShallowWater.LostVolume"/>. Default 16.</summary>
    public int MaxSubsteps { get; set; } = 16;

    /// <summary>Returns a copy (the bed and the sources are copied).</summary>
    public ShallowWaterDef Clone()
    {
        var c = (ShallowWaterDef)MemberwiseClone();
        c.Bed = (float[]?)Bed?.Clone();
        c.Sources = (WaterSource[])Sources.Clone();
        return c;
    }

    internal WaterStore.Spec ToSpec(string name)
    {
        var s = new WaterStore.Spec
        {
            Nx = CellsX, Nz = CellsZ, CellSize = CellSize, OriginX = MinX, OriginZ = MinZ, FloorHeight = FloorHeight, Level = Level,
            SampleBed = SampleBed, Bed = (float[]?)Bed?.Clone(),
            Density = Density, LinearDrag = LinearDrag, AngularDrag = AngularDrag, ViscousDamping = ViscousDamping, BottomFriction = BottomFriction, Cfl = Cfl, MaxSubsteps = MaxSubsteps,
        };
        if (!double.IsFinite(BedSearchHeight) || BedSearchHeight < 0 || BedSearchHeight > 1.0e6) { throw new ArgumentOutOfRangeException(name, BedSearchHeight, "BedSearchHeight must be within 0..1e6 metres."); }
        s.SearchTop = Math.Max(Level, FloorHeight) + BedSearchHeight;
        var sides = new[] { West, East, South, North };
        for (int i = 0; i < 4; i++) { s.Edges[i] = (WaterEdge)(byte)sides[i].Kind; s.EdgeLevel[i] = (float)sides[i].Level; s.EdgeSpeed[i] = sides[i].Speed; }
        s.Sources = ConvertSources(Sources, MinX, MinZ, name);
        return s;
    }

    internal static WaterSourceSpec[] ConvertSources(WaterSource[] sources, double minX, double minZ, string name)
    {
        ArgumentNullException.ThrowIfNull(sources);
        var r = new WaterSourceSpec[sources.Length];
        for (int i = 0; i < r.Length; i++)
        {
            var s = sources[i];
            if (!double.IsFinite(s.X) || !double.IsFinite(s.Z) || Math.Abs(s.X - minX) > 4194304.0 || Math.Abs(s.Z - minZ) > 4194304.0) { throw new ArgumentOutOfRangeException(name, "Source " + i.ToString(CultureInfo.InvariantCulture) + " has a position that is not finite or lies too far from the grid."); }
            r[i] = new WaterSourceSpec { X = (float)(s.X - minX), Z = (float)(s.Z - minZ), Radius = s.Radius, Rate = s.Rate };
        }

        return r;
    }
}

/// <summary>The water at a point of a 3D world.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct WaterSample3D
{
    internal WaterSample3D(WaterHandle water, double surface, float depth, Vector3F velocity, Vector3F normal) { Water = water; Surface = surface; Depth = depth; Velocity = velocity; Normal = normal; }

    /// <summary>The water that holds the point.</summary>
    public WaterHandle Water { get; }

    /// <summary>Height of the surface above the point's x and z, metres (world y).</summary>
    public double Surface { get; }

    /// <summary>Depth of the water there, metres (the surface above the bed).</summary>
    public float Depth { get; }

    /// <summary>Velocity of the water there, m/s, horizontal (y is 0).</summary>
    public Vector3F Velocity { get; }

    /// <summary>The unit normal of the surface there, pointing up.</summary>
    public Vector3F Normal { get; }
}

/// <summary>Where a ray met the water surface in a 3D world.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct WaterHit3D
{
    internal WaterHit3D(WaterHandle water, float distance, Vector3D point, Vector3F normal) { Water = water; Distance = distance; Point = point; Normal = normal; }

    /// <summary>The water that was hit.</summary>
    public WaterHandle Water { get; }

    /// <summary>Distance along the (unit) direction, metres.</summary>
    public float Distance { get; }

    /// <summary>The hit point in world space.</summary>
    public Vector3D Point { get; }

    /// <summary>The unit normal of the surface there, pointing up.</summary>
    public Vector3F Normal { get; }
}

/// <summary>
/// A body of shallow water: the handle (world, slot, generation) of a grid. It allocates nothing and throws <see cref="StaleHandleException"/> once removed.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct ShallowWater : IEquatable<ShallowWater>
{
    private readonly PhysicsWorld3D? _world;
    private readonly int _index, _generation;

    internal ShallowWater(PhysicsWorld3D world, int index, int generation) { _world = world; _index = index; _generation = generation; }

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

    /// <summary>Cells along x.</summary>
    public int CellsX => Grid.Nx;

    /// <summary>Cells along z.</summary>
    public int CellsZ => Grid.Nz;

    /// <summary>Side of a cell, metres.</summary>
    public float CellSize => Grid.Dx;

    /// <summary>World x of the minimum corner, metres.</summary>
    public double MinX => Grid.OriginX;

    /// <summary>World z of the minimum corner, metres.</summary>
    public double MinZ => Grid.OriginZ;

    /// <summary>Total volume of water in cubic metres, summed in cell order. Water that bodies displace is not counted. Conserved to float rounding with closed sides, no sources or drains and no bodies.</summary>
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

    /// <summary>Water lost to negative depths (a body pulled out of shallow water, or a step the substep cap stretched), cubic metres. Normally 0.</summary>
    public double LostVolume => Grid.LostVolume;

    /// <summary>Depth of the water in a cell (ix along x, iz along z), metres; 0 where dry.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The cell is outside the grid.</exception>
    public float CellDepth(int ix, int iz)
    {
        var g = Grid;
        Check(g, ix, iz);
        float h = g.H[iz * g.Nx + ix];
        return h > WaterGrid.DryDepth ? h : 0f;
    }

    /// <summary>Height of the water surface over a cell's centre, metres (world y); the bed height where dry.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The cell is outside the grid.</exception>
    public double CellSurface(int ix, int iz)
    {
        var g = Grid;
        Check(g, ix, iz);
        return g.BaseY + g.SurfaceOfCell(ix, iz);
    }

    private static void Check(WaterGrid g, int ix, int iz)
    {
        if ((uint)ix >= (uint)g.Nx || (uint)iz >= (uint)g.Nz) { throw new ArgumentOutOfRangeException(nameof(ix), "Cell (" + ix.ToString(CultureInfo.InvariantCulture) + ", " + iz.ToString(CultureInfo.InvariantCulture) + ") is outside the " + g.Nx.ToString(CultureInfo.InvariantCulture) + " × " + g.Nz.ToString(CultureInfo.InvariantCulture) + " grid."); }
    }

    /// <summary>The water at a point's x and z. False when the point is outside the grid or the water there is dry.</summary>
    public bool TryGetSample(double x, double z, out WaterSample3D sample)
    {
        sample = default;
        var g = Grid;
        if (!double.IsFinite(x) || !double.IsFinite(z) || !WaterStore.SampleAt(g, x, z, out double surface, out float depth, out float u, out float v) || !(depth > WaterGrid.DryDepth)) { return false; }
        WaterStore.SlopeAt(g, x, z, g.Dx, out float sx, out float sz);
        float inv = 1f / float.Sqrt(1f + sx * sx + sz * sz);
        sample = new WaterSample3D(RawHandle, surface, depth, new Vector3F(u, 0f, v), new Vector3F(-sx * inv, inv, -sz * inv));
        return true;
    }

    /// <summary>Replaces the water with a still pool filled up to a height (nothing where the bed is higher). Bodies in the water keep floating (their displaced volume stays recorded), so call it with the bodies where they should be.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The level is not finite or beyond ±2^21.</exception>
    public void Fill(double level)
    {
        using var replayMutation = _world?.RecordMutation("ShallowWater.Fill");
        var g = Grid;
        if (!double.IsFinite(level) || Math.Abs(level) > WaterStore.MaxCoordinate) { throw new ArgumentOutOfRangeException(nameof(level), level, "The level must be finite and within ±2^21 metres."); }
        g.Fill((float)(level - g.BaseY));
        for (int r = 0; r < g.RecordCount; r++) { var rec = g.Records[r]; g.AddVolume(rec.X0, rec.X1, rec.Z0, rec.Z1, rec.Volume); }
    }

    /// <summary>Adds water (negative removes) spread over a box of the grid, in cubic metres.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A bound or the volume is not finite.</exception>
    public void AddVolume(double minX, double minZ, double maxX, double maxZ, float volume)
    {
        using var replayMutation = _world?.RecordMutation("ShallowWater.AddVolume");
        var g = Grid;
        if (!double.IsFinite(minX) || !double.IsFinite(minZ) || !double.IsFinite(maxX) || !double.IsFinite(maxZ) || !float.IsFinite(volume) || Math.Abs(volume) > 1e12f) { throw new ArgumentOutOfRangeException(nameof(volume), volume, "The box and the volume must be finite (volume within ±1e12)."); }
        g.AddVolume((float)(minX - g.OriginX), (float)(maxX - g.OriginX), (float)(minZ - g.OriginZ), (float)(maxZ - g.OriginZ), volume);
    }

    /// <summary>Sets what happens at a side (0 west, 1 east, 2 south, 3 north).</summary>
    /// <exception cref="ArgumentOutOfRangeException">The side is not 0 to 3, or the boundary is invalid.</exception>
    public void SetBoundary(int side, WaterBoundary boundary)
    {
        using var replayMutation = _world?.RecordMutation("ShallowWater.SetBoundary");
        var g = Grid;
        if ((uint)side > 3u) { throw new ArgumentOutOfRangeException(nameof(side), side, "side must be 0 (west), 1 (east), 2 (south) or 3 (north)."); }
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
        var arr = sources.ToArray();
        var specs = ShallowWaterDef.ConvertSources(arr, g.OriginX, g.OriginZ, nameof(sources));
        foreach (var s in specs) { WaterStore.ValidateSource(s, nameof(sources)); }
        g.Sources = specs;
    }

    /// <summary>Samples the bed again from the static shapes now in the world (see <see cref="ShallowWaterDef.SampleBed"/>); the water keeps its depth.</summary>
    /// <param name="searchHeight">How far above the higher of the floor and the current mean surface the rays start, metres. Default 50.</param>
    public void ResampleBed(double searchHeight = 50.0)
    {
        using var replayMutation = _world?.RecordMutation("ShallowWater.ResampleBed");
        var g = Grid;
        _world!.Engine.Water.ResampleBed(g, g.BaseY + searchHeight + 1.0);
    }

    /// <summary>Removes the water. The volume of bodies that displaced it is simply forgotten.</summary>
    /// <exception cref="StaleHandleException">Already removed.</exception>
    public void Remove()
    {
        using var replayMutation = _world?.RecordMutation("ShallowWater.Remove");
        Grid.ToString();
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
