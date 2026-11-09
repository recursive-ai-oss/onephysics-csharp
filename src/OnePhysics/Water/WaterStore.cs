using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Water;

/// <summary>What the water store needs from an engine. Planar engines present positions as 3D vectors with z = 0 and take only the components in their plane.</summary>
internal interface IWaterHost
{
    WorldCore Core { get; }
    bool Planar { get; }
    int EngineSectionId { get; }
    WaterStore Water { get; }

    /// <summary>Gravity along -y, m/s²; water does not flow when it is zero or negative.</summary>
    float GravityDown { get; }

    /// <summary>The box around every shape of a body, world coordinates; false when the body has no shape.</summary>
    bool BodyBounds(int body, out double minX, out double minY, out double minZ, out double maxX, out double maxY, out double maxZ);

    /// <summary>The body's farthest point from its centre of mass, metres.</summary>
    float ReachOf(int body);

    /// <summary>
    /// Adds the buoyancy and drag a body would get from water whose surface is the plane with the given unit normal and offset (normal · point = offset) moving at <paramref name="flow"/>,
    /// to the body's force and torque for the coming step. Returns the submerged volume (area in a planar world); 0 when clear of the water.
    /// </summary>
    float ApplyWater(int body, Vector3F normal, double offset, Vector3F flow, WaterGrid water, float dt);

    /// <summary>The rods of the world, if any were created (they float and push water like any other particles).</summary>
    Rods.RodStore? WaterRods { get; }

    /// <summary>Whether the world holds soft bodies or cloth.</summary>
    bool HasSoftBodies { get; }

    /// <summary>
    /// Gives the soft bodies and cloth the buoyancy and drag of one grid as changes of particle velocity, and records what each submerged particle displaces with <see cref="WaterStore.Deposit"/>,
    /// in body slot then particle order.
    /// </summary>
    void ApplyWaterToSoftBodies(WaterStore store, WaterGrid water, float dt);

    /// <summary>The volume of a body under the surface plane (like <see cref="ApplyWater"/>, but nothing is added to the body's force or torque): what a kinematic body displaces.</summary>
    float SubmergedVolume(int body, Vector3F normal, double offset, Vector3F flow, WaterGrid water, float dt);

    /// <summary>The height where a ray cast straight down from <paramref name="fromY"/> first meets a static shape, within <paramref name="maxDistance"/>.</summary>
    bool CastDownStatic(double x, double z, double fromY, double maxDistance, out double hitY);

    /// <summary>Wakes the body's island and resets its sleep timer.</summary>
    void WakeBody(int body);

}

/// <summary>One enter or leave event as the store collects it.</summary>
internal struct WaterEventRecord
{
    public bool Entered;
    public int Body, BodyGeneration, Water, WaterGeneration;
    public float Volume;
}

/// <summary>
/// Shallow-water bodies of both dimensions: grids of water that bodies float in, displace and are pushed by. One store serves both engines through <see cref="IWaterHost"/> (a planar world has one-row grids).
/// It runs in two serial stages of the step: before the rigid step it adds each awake body's buoyancy and drag from the local surface and records the volume each body displaces; after it, the displacement is
/// added to the water and the water advances by the step. State lives in an optional snapshot section that is written only while a grid exists.
/// </summary>
internal sealed partial class WaterStore : ISnapshotSection
{
    /// <summary>Section id: "SWTR" read as a little-endian integer.</summary>
    public const int Id = 0x52545753;

    public const double MaxCoordinate = 2097152.0;   // 2^21: every position on the 2^-32 grid inside is exact in double
    private const double Grid32 = 1.0 / 4294967296.0;

    private readonly IWaterHost _host;
    public HandleTable Handles = new();
    public WaterGrid?[] Grids = new WaterGrid?[2];
    public readonly List<WaterEventRecord> Events = new();

    public WaterStore(IWaterHost host) { _host = host; }

    public int Count => Handles.Count;
    public bool HasWork => Handles.Count > 0;

    public WaterGrid Get(int slot, int generation)
    {
        if (!Handles.IsAlive(slot, generation)) { throw new StaleHandleException("ShallowWater(#" + slot.ToString(CultureInfo.InvariantCulture) + ", gen " + generation.ToString(CultureInfo.InvariantCulture) + ") is stale: the water was removed."); }
        return Grids[slot]!;
    }

    // ---------------------------------------------------------------- creation

    /// <summary>The settings of a new grid; the definition types of both dimensions fill one of these.</summary>
    internal sealed class Spec
    {
        public int Nx, Nz;
        public float CellSize;
        public double OriginX, OriginZ, FloorHeight, Level, SearchTop;
        public bool SampleBed;
        public float[]? Bed;
        public WaterEdge[] Edges = new WaterEdge[4];
        public float[] EdgeLevel = new float[4], EdgeSpeed = new float[4];
        public WaterSourceSpec[] Sources = Array.Empty<WaterSourceSpec>();
        public float Density = 1000f, LinearDrag = 0.5f, AngularDrag = 0.5f, ViscousDamping = 1f, BottomFriction, Damping = 0.1f, Displacement = 0.5f;
        public float Cfl = 0.25f;
        public int MaxSubsteps = 16;
    }

    private static double Snap(double v) => Math.Round(v / Grid32) * Grid32;

    /// <summary>Validates a specification; throws ArgumentOutOfRangeException naming the field.</summary>
    internal static void Validate(Spec s, bool planar, string name)
    {
        if (s.Nx < 2 || s.Nx > 1 << 20) { throw new ArgumentOutOfRangeException(name, s.Nx, "The number of cells along x must be within 2..1048576."); }
        if (planar) { if (s.Nz != 1) { throw new ArgumentOutOfRangeException(name, s.Nz, "A planar grid has one row."); } }
        else if (s.Nz < 2 || s.Nz > 1 << 20) { throw new ArgumentOutOfRangeException(name, s.Nz, "The number of cells along z must be within 2..1048576."); }
        if ((long)s.Nx * s.Nz > WaterGrid.MaxCellsPerGrid) { throw new ArgumentOutOfRangeException(name, (long)s.Nx * s.Nz, "A grid holds at most " + WaterGrid.MaxCellsPerGrid.ToString(CultureInfo.InvariantCulture) + " cells."); }
        if (!(s.CellSize >= 0.001f && s.CellSize <= 1000f)) { throw new ArgumentOutOfRangeException(name, s.CellSize, "CellSize must be within 0.001..1000 metres."); }
        Bound(s.OriginX, "the grid's x origin", name); Bound(s.OriginZ, "the grid's z origin", name); Bound(s.FloorHeight, "FloorHeight", name); Bound(s.Level, "Level", name); Bound(s.SearchTop, "BedSearchTop", name);
        if (Math.Abs(s.OriginX + s.Nx * (double)s.CellSize) > MaxCoordinate || Math.Abs(s.OriginZ + s.Nz * (double)s.CellSize) > MaxCoordinate) { throw new ArgumentOutOfRangeException(name, "The grid's far corner lies beyond ±2^21 metres."); }
        if (s.Bed != null)
        {
            long expected = (long)s.Nx * s.Nz;
            if (s.Bed.Length != expected) { throw new ArgumentOutOfRangeException(name, s.Bed.Length, "Bed must hold " + expected.ToString(CultureInfo.InvariantCulture) + " cell heights (" + s.Nx.ToString(CultureInfo.InvariantCulture) + (planar ? ")." : " by " + s.Nz.ToString(CultureInfo.InvariantCulture) + ", x fastest).")); }
            for (int i = 0; i < s.Bed.Length; i++) { if (!float.IsFinite(s.Bed[i]) || Math.Abs(s.Bed[i]) > MaxCoordinate) { throw new ArgumentOutOfRangeException(name, s.Bed[i], "Bed[" + i.ToString(CultureInfo.InvariantCulture) + "] must be finite and within ±2^21."); } }
        }

        for (int e = 0; e < 4; e++)
        {
            if (!Enum.IsDefined(s.Edges[e])) { throw new ArgumentOutOfRangeException(name, s.Edges[e], "Unknown boundary kind."); }
            if (!float.IsFinite(s.EdgeLevel[e]) || Math.Abs(s.EdgeLevel[e]) > MaxCoordinate) { throw new ArgumentOutOfRangeException(name, s.EdgeLevel[e], "An inflow level must be finite and within ±2^21."); }
            if (!(Math.Abs(s.EdgeSpeed[e]) <= 1000f)) { throw new ArgumentOutOfRangeException(name, s.EdgeSpeed[e], "An inflow speed must be finite and within ±1000 m/s."); }
        }

        if (s.Sources.Length > WaterGrid.MaxSources) { throw new ArgumentOutOfRangeException(name, s.Sources.Length, "At most " + WaterGrid.MaxSources.ToString(CultureInfo.InvariantCulture) + " sources."); }
        foreach (var src in s.Sources) { ValidateSource(src, name); }
        if (!(s.Density > 0f && s.Density <= 1e6f)) { throw new ArgumentOutOfRangeException(name, s.Density, "Density must be within (0, 1e6] kg/m³."); }
        Coefficient(s.LinearDrag, "LinearDrag", name); Coefficient(s.AngularDrag, "AngularDrag", name); Coefficient(s.ViscousDamping, "ViscousDamping", name); Coefficient(s.BottomFriction, "BottomFriction", name); Coefficient(s.Damping, "Damping", name);
        if (!(s.Displacement >= 0f && s.Displacement <= 1f)) { throw new ArgumentOutOfRangeException(name, s.Displacement, "Displacement must be within 0..1."); }
        if (!(s.Cfl > 0f && s.Cfl <= 0.5f)) { throw new ArgumentOutOfRangeException(name, s.Cfl, "Cfl must be within (0, 0.5]."); }
        if (s.MaxSubsteps < 1 || s.MaxSubsteps > 256) { throw new ArgumentOutOfRangeException(name, s.MaxSubsteps, "MaxSubsteps must be within 1..256."); }
    }

    internal static void ValidateSource(WaterSourceSpec src, string name)
    {
        if (!float.IsFinite(src.X) || !float.IsFinite(src.Z) || !float.IsFinite(src.Radius) || src.Radius < 0f || !float.IsFinite(src.Rate) || Math.Abs(src.Rate) > 1e6f) { throw new ArgumentOutOfRangeException(name, "A source needs a finite position, a radius of at least 0 and a rate within ±1e6 m³/s."); }
    }

    private static void Bound(double v, string what, string name)
    {
        if (!double.IsFinite(v) || Math.Abs(v) > MaxCoordinate) { throw new ArgumentOutOfRangeException(name, v, what + " must be finite and within ±2^21 metres."); }
    }

    private static void Coefficient(float v, string what, string name)
    {
        if (!(v >= 0f && v <= 1e6f)) { throw new ArgumentOutOfRangeException(name, v, what + " must be within 0..1e6."); }
    }

    /// <summary>Creates a grid: validates, snaps the origin to the 2^-32 grid, builds the bed (given, or ray-cast against static shapes, or flat), and fills it with water up to the level.</summary>
    public (int Slot, int Generation) Create(Spec s, string name)
    {
        Validate(s, _host.Planar, name);
        var g = new WaterGrid
        {
            Planar = _host.Planar, Nx = s.Nx, Nz = s.Nz, Dx = s.CellSize,
            OriginX = Snap(s.OriginX), OriginZ = _host.Planar ? 0.0 : Snap(s.OriginZ), BaseY = Snap(s.FloorHeight),
            Density = s.Density, LinearDrag = s.LinearDrag, AngularDrag = s.AngularDrag, ViscousDamping = s.ViscousDamping, BottomFriction = s.BottomFriction, Damping = s.Damping, Displacement = s.Displacement,
            Cfl = s.Cfl, MaxSubsteps = s.MaxSubsteps,
            Sources = (WaterSourceSpec[])s.Sources.Clone(),
        };
        Array.Copy(s.Edges, g.Edges, 4); Array.Copy(s.EdgeLevel, g.EdgeLevel, 4); Array.Copy(s.EdgeSpeed, g.EdgeSpeed, 4);
        for (int e = 0; e < 4; e++) { g.EdgeLevel[e] = (float)(s.EdgeLevel[e] - g.BaseY); }
        g.Allocate();
        FillBed(g, s.Bed, s.SampleBed, s.SearchTop);
        g.Derive();
        g.Fill((float)(s.Level - g.BaseY));
        int slot = Handles.Allocate();
        if (slot >= Grids.Length) { Array.Resize(ref Grids, Math.Max(slot + 1, Grids.Length * 2)); }
        Grids[slot] = g;
        return (slot, Handles.GenerationOf(slot));
    }

    private void FillBed(WaterGrid g, float[]? given, bool sample, double searchTop)
    {
        for (int k = 0; k < g.Nz; k++)
        {
            for (int i = 0; i < g.Nx; i++)
            {
                int c = k * g.Nx + i;
                float v = 0f;
                if (given != null) { v = (float)(given[c] - g.BaseY); }
                else if (sample && _host.CastDownStatic(g.OriginX + (i + 0.5) * g.Dx, g.Planar ? 0.0 : g.OriginZ + (k + 0.5) * g.Dx, searchTop, searchTop - g.BaseY, out double hit)) { v = (float)(hit - g.BaseY); }
                g.Bed[c] = v < 0f ? 0f : v;
            }
        }
    }

    /// <summary>Samples the bed again from the static shapes now in the world (the grid's present floor stays where nothing is hit). The water keeps its depth; cells the new bed leaves dry hold no water.</summary>
    public void ResampleBed(WaterGrid g, double searchTop)
    {
        Bound(searchTop, "BedSearchTop", nameof(searchTop));
        FillBed(g, null, true, searchTop);
        g.Derive();
        for (int c = 0; c < g.H.Length; c++) { if (!(g.H[c] > WaterGrid.DryDepth)) { g.Hu[c] = 0f; g.Hv[c] = 0f; } }
    }

    public void Destroy(int slot, int generation)
    {
        var g = Get(slot, generation);
        _ = g;
        Handles.Free(slot);
        Grids[slot] = null;
    }

    // ---------------------------------------------------------------- queries

    /// <summary>The first grid (slot order) whose footprint holds the point and whose water there is deeper than the dry tolerance.</summary>
    public bool Find(double x, double z, out int slot, out WaterGrid? grid, out float fx, out float fz)
    {
        for (int i = 0; i < Handles.HighWater; i++)
        {
            if (!Handles.IsLive(i)) { continue; }
            var g = Grids[i]!;
            if (!g.Locate(x, z, out fx, out fz)) { continue; }
            g.Sample(fx, fz, out _, out float depth, out _, out _);
            if (!(depth > WaterGrid.DryDepth)) { continue; }
            slot = i; grid = g;
            return true;
        }

        slot = -1; grid = null; fx = fz = 0f;
        return false;
    }

    /// <summary>Offers every grid (slot order) that holds the point with water over it to a character's probe: the surface there, the grid's density and the water's velocity.</summary>
    public void Probe(double x, double z, double bottom, float height, ref Characters.WaterProbe probe)
    {
        for (int i = 0; i < Handles.HighWater; i++)
        {
            if (!Handles.IsLive(i)) { continue; }
            var g = Grids[i]!;
            if (!g.Locate(x, z, out float fx, out float fz)) { continue; }
            g.Sample(fx, fz, out float s, out float depth, out float u, out float v);
            if (!(depth > WaterGrid.DryDepth)) { continue; }
            probe.Offer(CharacterWaterSource.ShallowWater, g.BaseY + s - bottom, g.Density, new Vector3F(u, 0f, g.Planar ? 0f : v), height);
        }
    }

    /// <summary>Surface, depth and velocity at a point of one grid, in world terms.</summary>
    public static bool SampleAt(WaterGrid g, double x, double z, out double surface, out float depth, out float u, out float v)
    {
        bool inside = g.Locate(x, z, out float fx, out float fz);
        g.Sample(fx, fz, out float s, out depth, out u, out v);
        surface = g.BaseY + s;
        return inside;
    }

    /// <summary>The slope of the surface at a point (rise per metre along x and z), from differences over a span that grows with the size of what floats there.</summary>
    public static void SlopeAt(WaterGrid g, double x, double z, float span, out float sx, out float sz)
    {
        g.Locate(x + span, z, out float ax, out float az);
        g.Sample(ax, az, out float e1, out _, out _, out _);
        g.Locate(x - span, z, out ax, out az);
        g.Sample(ax, az, out float e0, out _, out _, out _);
        sx = (e1 - e0) / (2f * span);
        sz = 0f;
        if (!g.Planar)
        {
            g.Locate(x, z + span, out ax, out az);
            g.Sample(ax, az, out e1, out _, out _, out _);
            g.Locate(x, z - span, out ax, out az);
            g.Sample(ax, az, out e0, out _, out _, out _);
            sz = (e1 - e0) / (2f * span);
        }

        if (sx > 1f) { sx = 1f; } else if (sx < -1f) { sx = -1f; }
        if (sz > 1f) { sz = 1f; } else if (sz < -1f) { sz = -1f; }
    }

    /// <summary>
    /// The water as a boat sees it at a point: the tangent plane of the surface there, the density and the velocity. False when no grid holds the point or it is dry there.
    /// </summary>
    public bool TryPlane(Vector3D p, float reach, out Aero.WaterSample water)
    {
        water = default;
        if (Handles.Count == 0) { return false; }
        if (!Find(p.X, p.Z, out int slot, out var g, out _, out _)) { return false; }
        var grid = g!;
        SampleAt(grid, p.X, p.Z, out double surface, out _, out float u, out float v);
        float span = Math.Min(Math.Max(0.5f * reach, grid.Dx), 4f * grid.Dx);
        SlopeAt(grid, p.X, p.Z, span, out float sx, out float sz);
        float inv = 1f / float.Sqrt(1f + sx * sx + sz * sz);
        var n = new Vector3F(-sx * inv, inv, -sz * inv);
        water = new Aero.WaterSample { Normal = n, Offset = n.X * p.X + n.Y * surface + n.Z * (_host.Planar ? 0.0 : p.Z), Density = grid.Density, Flow = new Vector3F(u, 0f, v), Limited = false };
        return true;
    }

    /// <summary>
    /// Casts a ray at the water surfaces and returns the nearest crossing from above or below, by marching a half cell at a time and refining the crossing by bisection (the surface is bilinear between cell centres).
    /// Dry cells have no surface. In a planar world the ray's z is ignored.
    /// </summary>
    public bool Raycast(Vector3D origin, Vector3F dir, float maxDistance, out int slot, out float distance, out Vector3D point, out Vector3F normal)
    {
        slot = -1; distance = 0f; point = default; normal = default;
        float best = float.PositiveInfinity;
        for (int i = 0; i < Handles.HighWater; i++)
        {
            if (!Handles.IsLive(i)) { continue; }
            if (RaycastGrid(Grids[i]!, origin, dir, Math.Min(maxDistance, best), out float d, out var n) && d < best)
            {
                best = d; slot = i; normal = n;
            }
        }

        if (slot < 0) { return false; }
        distance = best;
        point = new Vector3D(origin.X + dir.X * (double)best, origin.Y + dir.Y * (double)best, origin.Z + dir.Z * (double)best);
        return true;
    }

    private static bool RaycastGrid(WaterGrid g, Vector3D o, Vector3F dir, float maxDistance, out float distance, out Vector3F normal)
    {
        distance = 0f; normal = default;
        float step = 0.5f * g.Dx;
        double lenSq = (double)dir.X * dir.X + (double)dir.Y * dir.Y + (double)dir.Z * dir.Z;
        if (!(lenSq > 0.0)) { return false; }
        // Clip the ray to the grid's box (its footprint, and from the floor to the highest surface) so that a ray that misses costs nothing.
        double t0 = 0.0, t1 = maxDistance;
        double ymax = double.MinValue;
        for (int c = 0; c < g.H.Length; c++) { double e = (double)g.H[c] + g.Bed[c]; if (e > ymax) { ymax = e; } }
        if (!Slab(o.X, dir.X, g.OriginX, g.OriginX + g.Width, ref t0, ref t1)) { return false; }
        if (!g.Planar && !Slab(o.Z, dir.Z, g.OriginZ, g.OriginZ + g.Depth, ref t0, ref t1)) { return false; }
        if (!Slab(o.Y, dir.Y, g.BaseY, g.BaseY + ymax, ref t0, ref t1)) { return false; }
        if (!(t1 >= t0)) { return false; }
        maxDistance = (float)Math.Min(t1 + step, maxDistance);
        float tStart = (float)Math.Max(t0 - step, 0.0);
        float prevT = 0f, prevF = float.NaN;
        bool havePrev = false;
        for (float t = tStart; ; t += step)
        {
            if (t > maxDistance) { t = maxDistance; }
            double x = o.X + dir.X * (double)t, z = g.Planar ? 0.0 : o.Z + dir.Z * (double)t, y = o.Y + dir.Y * (double)t;
            bool inside = g.Locate(x, z, out float fx, out float fz);
            if (inside)
            {
                g.Sample(fx, fz, out float s, out float depth, out _, out _);
                float f = (float)(y - (g.BaseY + s));
                if (depth > WaterGrid.DryDepth)
                {
                    if (t == 0f && f == 0f) { distance = 0f; normal = Normal(g, x, z); return true; }
                    if (havePrev && ((prevF > 0f && f <= 0f) || (prevF < 0f && f >= 0f)))
                    {
                        float lo = prevT, hi = t, flo = prevF;
                        for (int it = 0; it < 24; it++)
                        {
                            float mid = 0.5f * (lo + hi);
                            double mx = o.X + dir.X * (double)mid, mz = g.Planar ? 0.0 : o.Z + dir.Z * (double)mid, my = o.Y + dir.Y * (double)mid;
                            g.Locate(mx, mz, out float gx, out float gz);
                            g.Sample(gx, gz, out float ms, out _, out _, out _);
                            float fm = (float)(my - (g.BaseY + ms));
                            if ((flo > 0f) == (fm > 0f)) { lo = mid; flo = fm; } else { hi = mid; }
                        }

                        distance = hi;
                        double hx = o.X + dir.X * (double)hi, hz = g.Planar ? 0.0 : o.Z + dir.Z * (double)hi;
                        normal = Normal(g, hx, hz);
                        return true;
                    }

                    prevT = t; prevF = f; havePrev = true;
                }
                else { havePrev = false; }
            }
            else { havePrev = false; }

            if (t >= maxDistance) { return false; }
        }
    }

    // Clips [t0, t1] to the slab lo <= o + t d <= hi; false when nothing is left.
    private static bool Slab(double o, double d, double lo, double hi, ref double t0, ref double t1)
    {
        if (d == 0.0) { return o >= lo && o <= hi; }
        double a = (lo - o) / d, b = (hi - o) / d;
        if (a > b) { (a, b) = (b, a); }
        if (a > t0) { t0 = a; }
        if (b < t1) { t1 = b; }
        return t1 >= t0;
    }

    private static Vector3F Normal(WaterGrid g, double x, double z)
    {
        SlopeAt(g, x, z, g.Dx, out float sx, out float sz);
        float inv = 1f / float.Sqrt(1f + sx * sx + sz * sz);
        return new Vector3F(-sx * inv, inv, -sz * inv);
    }

    // ---------------------------------------------------------------- shifting

    /// <summary>Throws when shifting the origin would carry a grid beyond the coordinate bound.</summary>
    public void CheckShift(Vector3D offset)
    {
        for (int i = 0; i < Handles.HighWater; i++)
        {
            if (!Handles.IsLive(i)) { continue; }
            var g = Grids[i]!;
            double x = g.OriginX - offset.X, y = g.BaseY - offset.Y, z = g.OriginZ - offset.Z;
            double far = Math.Max(Math.Abs(x), Math.Abs(x + g.Width));
            if (!(far <= 4194304.0) || !(Math.Abs(y) <= 4194304.0) || !(Math.Abs(z) <= 4194304.0 && Math.Abs(z + g.Depth) <= 4194304.0)) { throw new ArgumentOutOfRangeException(nameof(offset), offset, "Shifting the origin by this offset would carry shallow water #" + i.ToString(CultureInfo.InvariantCulture) + " beyond ±2^22 metres."); }
        }
    }

    /// <summary>Moves every grid with the world origin. The cells hold heights relative to the grid, so none of them changes.</summary>
    public void Shift(Vector3D offset)
    {
        for (int i = 0; i < Handles.HighWater; i++)
        {
            if (!Handles.IsLive(i)) { continue; }
            var g = Grids[i]!;
            g.OriginX -= offset.X; g.BaseY -= offset.Y;
            if (!g.Planar) { g.OriginZ -= offset.Z; }
        }
    }

    // ---------------------------------------------------------------- bodies going away

    /// <summary>Called when a body is destroyed: the water it displaced flows back (its volume is taken out where it was added).</summary>
    public void DropBody(int body)
    {
        for (int i = 0; i < Handles.HighWater; i++)
        {
            if (!Handles.IsLive(i)) { continue; }
            var g = Grids[i]!;
            int r = FindRecord(g, body);
            if (r < 0) { continue; }
            var rec = g.Records[r];
            g.AddVolume(rec.X0, rec.X1, rec.Z0, rec.Z1, -rec.Volume);
            RemoveRecord(g, r);
        }
    }

    internal static int FindRecord(WaterGrid g, int body)
    {
        int lo = 0, hi = g.RecordCount - 1;
        while (lo <= hi)
        {
            int mid = (lo + hi) >> 1;
            int s = g.Records[mid].Slot;
            if (s == body) { return mid; }
            if (s < body) { lo = mid + 1; } else { hi = mid - 1; }
        }

        return -1;
    }

    internal static void InsertRecord(WaterGrid g, in WaterBodyRecord rec)
    {
        int lo = 0, hi = g.RecordCount;
        while (lo < hi) { int mid = (lo + hi) >> 1; if (g.Records[mid].Slot < rec.Slot) { lo = mid + 1; } else { hi = mid; } }
        if (g.RecordCount == g.Records.Length) { Array.Resize(ref g.Records, g.Records.Length * 2); }
        Array.Copy(g.Records, lo, g.Records, lo + 1, g.RecordCount - lo);
        g.Records[lo] = rec;
        g.RecordCount++;
    }

    internal static void RemoveRecord(WaterGrid g, int index)
    {
        Array.Copy(g.Records, index + 1, g.Records, index, g.RecordCount - index - 1);
        g.RecordCount--;
    }

    /// <summary>After a restore: every record names a live dynamic body of the generation it was saved with.</summary>
    public bool Consistent()
    {
        var bodies = _host.Core.Bodies;
        for (int i = 0; i < Handles.HighWater; i++)
        {
            if (!Handles.IsLive(i)) { continue; }
            var g = Grids[i]!;
            for (int r = 0; r < g.RecordCount; r++)
            {
                ref readonly var rec = ref g.Records[r];
                if (!bodies.Handles.IsAlive(rec.Slot, rec.Generation) || (bodies.TypeOf(rec.Slot) != BodyType.Dynamic && bodies.TypeOf(rec.Slot) != BodyType.Kinematic)) { return false; }
            }
        }

        return true;
    }

    /// <summary>Region selection: a grid is a world field, kept whole in every region, with the records of the selected bodies.</summary>
    internal void SelectRegion(RegionSelection s)
    {
        for (int i = 0; i < Handles.HighWater; i++) { if (Handles.IsLive(i)) { s.Add("shallow water", i); } }
    }
}
