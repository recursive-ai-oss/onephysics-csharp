using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Destruction;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>How a body fractures into Voronoi cells, and when it does so by itself. Copied when assigned; defaults give 16 uniform cells and no automatic fracture.</summary>
public sealed class FractureDef
{
    /// <summary>Number of seeds, hence cells, 2 to 512. Default 16. A cell that clips away or is merged into a neighbour leaves fewer bodies.</summary>
    public int CellCount { get; set; } = 16;

    /// <summary>Seed placement. Default <see cref="FracturePattern.Uniform"/>.</summary>
    public FracturePattern Pattern { get; set; }

    /// <summary>Seed of the deterministic generator for <see cref="FracturePattern.Uniform"/> and <see cref="FracturePattern.Clustered"/>. Equal seeds give equal cells on every platform. Default 1.</summary>
    public ulong Seed { get; set; } = 1;

    /// <summary>The seeds for <see cref="FracturePattern.Points"/>, in the body's own frame (2 to 512 points; they may lie outside the body, the cells still tile it). Default none.</summary>
    public Vector3F[]? Points { get; set; }

    /// <summary>World position of the impact for <see cref="FracturePattern.Clustered"/> on a manual fracture; null uses the middle of the body. An automatic fracture uses the strongest contact point instead.</summary>
    public Vector3D? ImpactPoint { get; set; }

    /// <summary>Distance in metres over which clustered seeds spread around the impact; 0 uses the distance to the farthest corner. Default 0.</summary>
    public float ImpactRadius { get; set; }

    /// <summary>Clustering strength, 1 to 16: the seed distance is the radius times a uniform number raised to this power. 1 is uniform in distance, larger crowds seeds at the impact. Default 2.</summary>
    public float ClusterPower { get; set; } = 2f;

    /// <summary>Automatic fracture: the body breaks at the end of a step in which the contact impulse on it reaches this value, in kg·m/s. The measure is the sum, over the points of the strongest manifold, of the largest normal impulse the solver applied at each point in one substep. 0 = never automatic. Default 0.</summary>
    public float ImpulseThreshold { get; set; }

    /// <summary>How many more generations the fragments may break themselves, 0 to 8. Default 0.</summary>
    public int Depth { get; set; }

    /// <summary>Cell count of the fragments' own fracture; 0 halves the parent's (at least 2).</summary>
    public int ChildCellCount { get; set; }

    /// <summary>The fragments' impulse threshold is the parent's times this. Default 2.</summary>
    public float ChildThresholdScale { get; set; } = 2f;

    /// <summary>A cell smaller than this fraction of the body's volume is merged into its nearest neighbour (or deleted when <see cref="MergeSmall"/> is false). 0 to below 1. Default 0.001.</summary>
    public float MinFragmentVolume { get; set; } = 0.001f;

    /// <summary>True (default) merges small cells into the nearest big cell, keeping all the mass; false deletes them, which loses their mass and momentum.</summary>
    public bool MergeSmall { get; set; } = true;

    internal FractureSettings ToSettings()
    {
        var s = new FractureSettings
        {
            CellCount = CellCount, Pattern = Pattern, Seed = Seed, Points = Points == null ? [] : (Vector3F[])Points.Clone(), ImpactPoint = ImpactPoint, ImpactRadius = ImpactRadius, ClusterPower = ClusterPower,
            ImpulseThreshold = ImpulseThreshold, Depth = Depth, ChildCellCount = ChildCellCount, ChildThresholdScale = ChildThresholdScale, MinFragmentVolume = MinFragmentVolume, MergeSmall = MergeSmall,
        };
        DestructionStore.Validate(s);
        return s;
    }
}

/// <summary>Convex cells in a body's frame with the bonds between touching cells: the authored form of a pre-fractured object, turned into a body by <see cref="PhysicsWorld3D.AddBreakable"/>.</summary>
public sealed class FractureAsset
{
    private readonly Vector3F[][] _cells;
    internal readonly Bond[] Bonds;
    internal readonly Piece[] Pieces;

    private FractureAsset(Vector3F[][] cells, Piece[] pieces, Bond[] bonds) { _cells = cells; Pieces = pieces; Bonds = bonds; }

    /// <summary>Number of cells (parts).</summary>
    public int CellCount => _cells.Length;

    /// <summary>Number of bonds between touching cells.</summary>
    public int BondCount => Bonds.Length;

    /// <summary>The hull points of a cell, in the body's frame.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The index is outside 0..CellCount-1.</exception>
    public ReadOnlySpan<Vector3F> GetCell(int index) => (uint)index < (uint)_cells.Length ? _cells[index] : throw new ArgumentOutOfRangeException(nameof(index), index, "No such cell.");

    /// <summary>The cells a bond joins and its face area in square metres.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The index is outside 0..BondCount-1.</exception>
    public (int CellA, int CellB, float Area) GetBond(int index)
    {
        if ((uint)index >= (uint)Bonds.Length) throw new ArgumentOutOfRangeException(nameof(index), index, "No such bond.");
        var b = Bonds[index];
        return (b.A, b.B, b.Measure);
    }

    /// <summary>Fractures the convex hull of <paramref name="points"/> into Voronoi cells (pattern, count and seed from <paramref name="def"/>; the automatic-fracture fields are ignored) and bonds the touching cells.</summary>
    /// <exception cref="ArgumentException">The points are degenerate or not finite (the message says which).</exception>
    /// <exception cref="ArgumentOutOfRangeException">A fracture setting is out of range.</exception>
    public static FractureAsset FromHull(ReadOnlySpan<Vector3F> points, FractureDef? def = null)
    {
        var s = (def ?? new FractureDef()).ToSettings();
        var whole = Piece.FromPoints3D(points) ?? throw new ArgumentException("The points do not span a volume.", nameof(points));
        whole.Measure(out _, out var c);
        whole.Bounds(out var lo, out var hi);
        var impact = s.ImpactPoint is { } ip ? ip : (lo + hi) * 0.5;
        double radius = s.ImpactRadius > 0 ? s.ImpactRadius : whole.Radius(impact);
        var user = new Vector3D[s.Points.Length];
        for (int i = 0; i < user.Length; i++) user[i] = new Vector3D(s.Points[i].X, s.Points[i].Y, s.Points[i].Z);
        var seeds = Fracturer.MakeSeeds([whole], s.CellCount, s.Pattern, s.Seed, impact, radius, s.ClusterPower, user);
        var cells = Fracturer.Cells(whole, seeds);
        var list = new List<Vector3F[]>();
        foreach (var cell in cells) if (cell != null) list.Add(cell.FloatPoints());
        _ = c;
        return FromCells(list, s.MinFragmentVolume);
    }

    /// <summary>Builds an asset from convex point sets in the body's frame (for instance the pieces of an authoring tool), bonding every pair of cells that share a face.</summary>
    /// <exception cref="ArgumentException">A cell is degenerate or not finite.</exception>
    public static FractureAsset FromCells(IReadOnlyList<Vector3F[]> cells) => FromCells(cells, 0f);

    private static FractureAsset FromCells(IReadOnlyList<Vector3F[]> cells, float minFraction)
    {
        ArgumentNullException.ThrowIfNull(cells);
        if (cells.Count < 1 || cells.Count > 4096) throw new ArgumentOutOfRangeException(nameof(cells), cells.Count, "An asset needs 1 to 4096 cells.");
        var pieces = new List<Piece>(cells.Count);
        var points = new List<Vector3F[]>(cells.Count);
        double total = 0;
        for (int i = 0; i < cells.Count; i++)
        {
            var p = Piece.FromPoints3D(cells[i] ?? throw new ArgumentNullException(nameof(cells))) ?? throw new ArgumentException("Cell " + i.ToString(System.Globalization.CultureInfo.InvariantCulture) + " does not span a volume.", nameof(cells));
            p.Measure(out double m, out _);
            total += m; pieces.Add(p);
        }
        // Cells below the minimum fraction are folded away by dropping them (an asset has fixed parts; merging is a runtime-fracture feature).
        var keep = new List<Piece>(); var keepPts = new List<Vector3F[]>();
        foreach (var p in pieces)
        {
            p.Measure(out double m, out _);
            if (m < total * minFraction) continue;
            // Canonical form: the hull of the float points that become the shape.
            var fp = p.FloatPoints();
            var canon = Piece.FromPoints3D(fp);
            if (canon == null) continue;
            keep.Add(canon); keepPts.Add(fp);
        }
        if (keep.Count == 0) throw new ArgumentException("No cell spans a volume.", nameof(cells));
        var bonds = new List<Bond>();
        var lo = new Vector3D[keep.Count]; var hi = new Vector3D[keep.Count];
        for (int i = 0; i < keep.Count; i++) keep[i].Bounds(out lo[i], out hi[i]);
        for (int i = 0; i < keep.Count; i++)
            for (int j = i + 1; j < keep.Count; j++)
            {
                double pad = 1e-4 * Math.Max(hi[i].X - lo[i].X, Math.Max(hi[i].Y - lo[i].Y, hi[i].Z - lo[i].Z));
                if (lo[j].X > hi[i].X + pad || lo[i].X > hi[j].X + pad || lo[j].Y > hi[i].Y + pad || lo[i].Y > hi[j].Y + pad || lo[j].Z > hi[i].Z + pad || lo[i].Z > hi[j].Z + pad) continue;
                var face = Fracturer.Shared(keep[i], keep[j]);
                if (face == null) continue;
                var f = face.Value;
                bonds.Add(new Bond { A = i, B = j, Measure = (float)f.Measure, Centroid = new Vector3F((float)f.Centroid.X, (float)f.Centroid.Y, (float)f.Centroid.Z), Normal = new Vector3F((float)f.Normal.X, (float)f.Normal.Y, (float)f.Normal.Z) });
            }
        return new FractureAsset(keepPts.ToArray(), keep.ToArray(), bonds.ToArray());
    }
}

/// <summary>A fracture, slice or bond break of the last step (or a manual one since). The source body no longer exists.</summary>
public readonly struct FractureEvent
{
    internal FractureEvent(FractureKind kind, BodyHandle source, int first, int count, float impulse, Vector3D point)
    {
        Kind = kind; Source = source; FirstFragment = first; FragmentCount = count; Impulse = impulse; Point = point;
    }

    /// <summary>Why the body broke.</summary>
    public FractureKind Kind { get; }
    /// <summary>The destroyed body's handle (stale).</summary>
    public BodyHandle Source { get; }
    /// <summary>Index of the first fragment in <see cref="PhysicsWorld3D.FractureFragments"/>.</summary>
    public int FirstFragment { get; }
    /// <summary>How many bodies replaced it.</summary>
    public int FragmentCount { get; }
    /// <summary>The impact impulse that triggered an <see cref="FractureKind.Impact"/>, kg·m/s; 0 otherwise.</summary>
    public float Impulse { get; }
    /// <summary>World position of the impact (the strongest contact), or the body's centre of mass for the other kinds.</summary>
    public Vector3D Point { get; }
}

/// <summary>The two bodies a slice made.</summary>
public readonly struct SliceResult
{
    internal SliceResult(Body positive, Body negative) { Positive = positive; Negative = negative; }
    /// <summary>The body on the side the plane normal points to.</summary>
    public Body Positive { get; }
    /// <summary>The body on the other side.</summary>
    public Body Negative { get; }
}

public sealed partial class PhysicsWorld3D
{
    private FractureEvent[] _fractureEvents = new FractureEvent[4];
    private int _fractureEventCount;
    private Body[] _fractureFragments = new Body[8];
    private int _fractureFragmentCount;
    private Action<FractureEvent>? _onFracture;

    /// <summary>Raised after a step, or right after a manual fracture or slice, once per fracture in order, unless callbacks are suppressed. The source body is already destroyed; the fragments are alive.</summary>
    public event Action<FractureEvent>? OnFracture { add => _onFracture += value; remove => _onFracture -= value; }

    /// <summary>The fractures, slices and bond breaks of the last step and of any manual ones since, in the order they happened. Valid until the next step.</summary>
    public ReadOnlySpan<FractureEvent> FractureEvents => _fractureEvents.AsSpan(0, _fractureEventCount);

    /// <summary>The bodies the <see cref="FractureEvents"/> created; each event names its range.</summary>
    public ReadOnlySpan<Body> FractureFragments => _fractureFragments.AsSpan(0, _fractureFragmentCount);

    /// <summary>The debris rules. Reading returns a copy; assign a modified copy to change them.</summary>
    /// <exception cref="ArgumentNullException">Assigning null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is negative.</exception>
    public DebrisOptions Debris
    {
        get { var d = _engine.Destruction; return new DebrisOptions { MaxFragments = d.MaxFragments, SleepToStaticSteps = d.FreezeAfterSteps, LifetimeSteps = d.LifetimeSteps }; }
        set
        {
            ArgumentNullException.ThrowIfNull(value);
            value.Validate();
            using var mutation = RecordMutation("PhysicsWorld3D.Debris");
            var d = _engine.Destruction; d.MaxFragments = value.MaxFragments; d.FreezeAfterSteps = value.SleepToStaticSteps; d.LifetimeSteps = value.LifetimeSteps;
        }
    }

    /// <summary>Cumulative counts of fractures, slices, bond breaks and fragments. Saved with the world.</summary>
    public DestructionCounters DestructionCounters => _engine.Destruction.Counters;

    /// <summary>Makes a body fracture by itself when a contact impulse reaches <see cref="FractureDef.ImpulseThreshold"/> (which must be above 0). Replaces an earlier setting. The body must be dynamic, made of convex hull or box shapes, and not part of a joint, vehicle, character or ragdoll when it breaks (otherwise it is left whole and forgotten).</summary>
    /// <exception cref="ArgumentNullException">The definition is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A setting is out of range or the threshold is 0.</exception>
    /// <exception cref="InvalidOperationException">The body is a breakable compound (it breaks through its bonds).</exception>
    public void SetFracturable(Body body, FractureDef def)
    {
        ArgumentNullException.ThrowIfNull(def);
        int b = ResolveBody(body.RawHandle);
        var s = def.ToSettings();
        if (!(s.ImpulseThreshold > 0f)) throw new ArgumentOutOfRangeException(nameof(def), s.ImpulseThreshold, "ImpulseThreshold must be above 0 for a body that fractures on impact.");
        using var mutation = RecordMutation("PhysicsWorld3D.SetFracturable");
        var rec = _engine.Destruction.GetOrAdd(b);
        if (rec.Bonds != null) throw new InvalidOperationException("The body is a breakable compound; it breaks through its bonds.");
        rec.Fracture = s;
    }

    /// <summary>Stops a body from fracturing by itself. Does nothing when it was not set.</summary>
    public void ClearFracturable(Body body)
    {
        int b = ResolveBody(body.RawHandle);
        using var mutation = RecordMutation("PhysicsWorld3D.ClearFracturable");
        var rec = _engine.Destruction.Find(b);
        if (rec == null || rec.Fracture == null) return;
        rec.Fracture = null;
        if (rec.Bonds == null) _engine.Destruction.Remove(b);
    }

    /// <summary>True when the body fractures by itself on impact.</summary>
    public bool IsFracturable(Body body) => _engine.Destruction.Find(ResolveBody(body.RawHandle))?.Fracture != null;

    /// <summary>
    /// Replaces a body by the Voronoi cells of its shapes now. Each fragment is a new dynamic body with the shapes' materials, the body's damping and gravity scale, and the velocity field of the original (v + w x r), so mass,
    /// linear momentum and angular momentum are conserved to rounding (cells deleted by <see cref="FractureDef.MergeSmall"/> = false excepted). The original is destroyed. Fragments count against the <see cref="Debris"/> rules.
    /// Runs between steps: do not call it from an event handler while a step is running.
    /// </summary>
    /// <returns>The fragments in cell order.</returns>
    /// <exception cref="FractureException">The body has a shape that is not a sharp convex hull or box, belongs to a joint, character, vehicle or ragdoll, or the cut leaves fewer than two cells.</exception>
    public Body[] Fracture(Body body, FractureDef? def = null)
    {
        int b = ResolveBody(body.RawHandle);
        var s = (def ?? new FractureDef()).ToSettings();
        using var mutation = RecordMutation("PhysicsWorld3D.Fracture");
        var store = _engine.Destruction;
        int firstEvent = store.Events.Count;
        var result = store.Fracture(b, s, null, 0f, FractureKind.Manual);
        if (result == null) throw new FractureException(store.Failure);
        SyncFractureEvents(); DispatchFractureEvents(firstEvent);
        var bodies = new Body[result.Count];
        for (int i = 0; i < bodies.Length; i++) bodies[i] = new Body(this, result[i]);
        return bodies;
    }

    /// <summary>
    /// Cuts a body with the plane through <paramref name="point"/> with the given normal (world axes) into two bodies with exact convex clipping, recomputed mass properties and the original's velocity field.
    /// A breakable compound is cut part by part; bonds across the plane are gone. The original is destroyed. With <paramref name="trackAsDebris"/> the halves count against the <see cref="Debris"/> rules (default false).
    /// </summary>
    /// <exception cref="FractureException">The plane misses the body, a half is too thin to be a body, or the body has an unsupported shape or belongs to a machine.</exception>
    public SliceResult Slice(Body body, Vector3D point, Vector3F normal, bool trackAsDebris = false)
    {
        int b = ResolveBody(body.RawHandle);
        if (!double.IsFinite(point.X) || !double.IsFinite(point.Y) || !double.IsFinite(point.Z)) throw new ArgumentOutOfRangeException(nameof(point), point, "The plane point must be finite.");
        using var mutation = RecordMutation("PhysicsWorld3D.Slice");
        var store = _engine.Destruction;
        int firstEvent = store.Events.Count;
        var r = store.Slice(b, point, normal, trackAsDebris);
        if (r == null) throw new FractureException(store.Failure);
        SyncFractureEvents(); DispatchFractureEvents(firstEvent);
        return new SliceResult(new Body(this, r.Value.positive), new Body(this, r.Value.negative));
    }

    /// <summary>
    /// Creates a body from a pre-fractured asset: one convex shape per cell, bonds between touching cells, held together until stress breaks them. Mass comes from <paramref name="density"/> (kg/m³) and the cell volumes.
    /// Stress comes from contact impulses, weight and the inertia of the body's motion (see <see cref="BreakableDef"/>); when bonds break the body is replaced by one body per connected piece, each a <see cref="FractureKind.BondBreak"/> event.
    /// </summary>
    /// <exception cref="ArgumentNullException">The asset is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A setting is out of range or the position is not finite.</exception>
    public Body AddBreakable(FractureAsset asset, Vector3D at = default, QuaternionF rotation = default, BreakableDef? def = null, BodyType type = BodyType.Dynamic, Material3D? material = null, float density = 1f)
    {
        ArgumentNullException.ThrowIfNull(asset);
        def ??= new BreakableDef();
        def.Validate(asset.CellCount);
        if (!(density > 0f) || !float.IsFinite(density)) throw new ArgumentOutOfRangeException(nameof(density), density, "Density must be finite and above 0.");
        using var mutation = RecordMutation("PhysicsWorld3D.AddBreakable");
        var body = CreateBody(type, at, rotation, nameof(at));
        try
        {
            var shapeDef = ShapeFor(material, density);
            var parts = new Part[asset.CellCount];
            for (int i = 0; i < parts.Length; i++)
            {
                var shape = CreateShape(body, ShapeSpec3D.ConvexHull(asset.GetCell(i)), shapeDef);
                asset.Pieces[i].Measure(out double m, out var c);
                parts[i] = new Part { Shape = shape.RawHandle.Index, ShapeGeneration = shape.RawHandle.Generation, Measure = (float)m, Mass = (float)(m * density), Centroid = new Vector3F((float)c.X, (float)c.Y, (float)c.Z) };
            }
            if (def.AnchorParts != null) foreach (int a in def.AnchorParts) parts[a].Anchored = true;
            var rec = _engine.Destruction.GetOrAdd(body.RawHandle.Index);
            rec.Parts = parts;
            rec.Bonds = new BondSettings { Strength = def.Strength, ShearFactor = def.ShearFactor, OverloadSteps = def.OverloadSteps };
            rec.BondList = new Bond[asset.Bonds.Length];
            for (int i = 0; i < rec.BondList.Length; i++) { var b = asset.Bonds[i]; rec.BondList[i] = new Bond { A = b.A, B = b.B, Measure = b.Measure, Centroid = b.Centroid, Normal = b.Normal }; }
        }
        catch (Exception) { if (body.IsValid) { _engine.Destruction.Remove(body.RawHandle.Index); body.Destroy(); } throw; }
        return body;
    }

    /// <summary>Bonds of a breakable body: for each, the parts it joins, its face area and the stress of the last step in pascals (0 when broken or unloaded).</summary>
    /// <exception cref="InvalidHandleException">The body is stale or foreign.</exception>
    public (int PartA, int PartB, float Area, float Stress, bool Intact)[] GetBonds(Body body)
    {
        var rec = _engine.Destruction.Find(ResolveBody(body.RawHandle));
        if (rec == null || rec.Bonds == null) return [];
        var r = new (int, int, float, float, bool)[rec.BondList.Length];
        for (int i = 0; i < r.Length; i++) { var b = rec.BondList[i]; r[i] = (b.A, b.B, b.Measure, b.Alive ? b.Stress : 0f, b.Alive); }
        return r;
    }

    /// <summary>Breaks a bond of a breakable body (by index from <see cref="GetBonds"/>) and splits the body if that disconnects it. Returns the new bodies, or an empty array when the body stayed whole.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The bond index is out of range.</exception>
    /// <exception cref="InvalidOperationException">The body is not a breakable compound.</exception>
    public Body[] BreakBond(Body body, int bond)
    {
        int b = ResolveBody(body.RawHandle);
        var store = _engine.Destruction;
        var rec = store.Find(b);
        if (rec == null || rec.Bonds == null) throw new InvalidOperationException("The body is not a breakable compound.");
        if ((uint)bond >= (uint)rec.BondList.Length) throw new ArgumentOutOfRangeException(nameof(bond), bond, "No such bond.");
        using var mutation = RecordMutation("PhysicsWorld3D.BreakBond");
        if (!rec.BondList[bond].Alive) return [];
        rec.BondList[bond].Alive = false;
        int firstEvent = store.Events.Count, firstFragment = store.EventFragments.Count;
        if (!store.SplitComponents(rec)) return [];
        SyncFractureEvents(); DispatchFractureEvents(firstEvent);
        var made = new Body[store.EventFragments.Count - firstFragment];
        for (int i = 0; i < made.Length; i++) made[i] = new Body(this, store.EventFragments[firstFragment + i]);
        return made;
    }

    private void SyncFractureEvents()
    {
        var store = _engine.Destruction;
        int n = store.Events.Count;
        if (_fractureEvents.Length < n) Array.Resize(ref _fractureEvents, Math.Max(n, 2 * _fractureEvents.Length));
        for (int i = 0; i < n; i++)
        {
            var e = store.Events[i];
            _fractureEvents[i] = new FractureEvent((FractureKind)e.Kind, e.Source, e.FirstFragment, e.FragmentCount, e.Impulse, e.Point);
        }
        _fractureEventCount = n;
        int m = store.EventFragments.Count;
        if (_fractureFragments.Length < m) Array.Resize(ref _fractureFragments, Math.Max(m, 2 * _fractureFragments.Length));
        for (int i = 0; i < m; i++) _fractureFragments[i] = new Body(this, store.EventFragments[i]);
        _fractureFragmentCount = m;
    }

    private void DispatchFractureEvents(int from)
    {
        if (_onFracture == null || _suppressed != 0) return;
        for (int i = from; i < _fractureEventCount; i++) _onFracture(_fractureEvents[i]);
    }

    // Called at the start of a step.
    private void BeginDestruction()
    {
        var store = _engine.Destruction;
        if (store.Events.Count != 0 || _fractureEventCount != 0) { store.BeginStep(); _fractureEventCount = 0; _fractureFragmentCount = 0; }
        var c = Counters; c.Fractures = 0; c.BondBreaks = 0; c.FragmentsCulled = 0;
    }

    // Called at the end of a step, after events have been delivered.
    private void EndDestruction(float dt)
    {
        var store = _engine.Destruction;
        if (store.Idle) { Counters.Fragments = 0; return; }
        var before = store.Counters;
        store.AfterStep(dt);
        var after = store.Counters;
        var counters = Counters;
        counters.Fractures = (int)(after.Fractures + after.Slices - before.Fractures - before.Slices);
        counters.BondBreaks = (int)(after.BondBreaks - before.BondBreaks);
        counters.FragmentsCulled = (int)(after.FragmentsCulled + after.FragmentsExpired - before.FragmentsCulled - before.FragmentsExpired);
        counters.Fragments = (int)after.FragmentsAlive;
        if (store.Events.Count != 0) { SyncFractureEvents(); DispatchFractureEvents(0); }
    }
}
