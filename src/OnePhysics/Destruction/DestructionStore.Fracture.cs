using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Destruction;

internal sealed class Group
{
    public readonly List<Piece> Pieces = new();
    public readonly List<ShapeTemplate> Templates = new();
    public readonly List<int> PartTags = new();   // source part index per piece, -1 when the source is not a bonded compound
    public BodyType Type = BodyType.Dynamic;
    public int Seed;                              // the seed index a cell came from, for merging neighbours

    public void Add(Piece p, in ShapeTemplate t, int tag) { Pieces.Add(p); Templates.Add(t); PartTags.Add(tag); }

    public double Measure()
    {
        double m = 0;
        foreach (var p in Pieces) { p.Measure(out double v, out _); m += v; }
        return m;
    }
}

internal sealed class Created
{
    public BodyHandle Body;
    public ShapeHandle[] Shapes = [];
    public Group Group = null!;
}

internal sealed partial class DestructionStore
{
    internal const int MaxPiecesPerBody = 4096;

    /// <summary>Why the last Fracture or Slice returned null.</summary>
    internal string Failure = "";

    private static Vector3F Vec3(ReadOnlySpan<float> v) => v.Length >= 3 ? new Vector3F(v[0], v[1], v[2]) : new Vector3F(v[0], v[1], 0f);

    private static void SetVec(Span<float> dst, Vector3F v) { dst[0] = v.X; dst[1] = v.Y; if (dst.Length >= 3) dst[2] = v.Z; }

    internal Vector3D CentreOfMass(int body)
    {
        var p = Host.DestructionCore.Bodies.PositionOf(body);
        return new Vector3D(p[0], p[1], p.Length >= 3 ? p[2] : 0.0);
    }

    internal Vector3F AngularVelocityOf(int body)
    {
        var a = Host.DestructionCore.Bodies.AngularVelocityOf(body);
        return a.Length >= 3 ? new Vector3F(a[0], a[1], a[2]) : new Vector3F(0f, 0f, a[0]);
    }

    internal static Vector3F Cross(Vector3F a, Vector3F b) => new(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);

    // Wakes the bodies resting on the one about to disappear, so they fall with their support instead of hanging asleep in the air.
    private void WakeNeighbours(int body)
    {
        var core = Host.DestructionCore;
        var bodies = core.Bodies; var cs = core.Constraints;
        if (bodies.IsDynamic(body))
        {
            for (int e = bodies.FirstEdge(body); e >= 0; e = cs.NextEdge(e)) WakeOther(core, cs.BodyA[e >> 1] == body ? cs.BodyB[e >> 1] : cs.BodyA[e >> 1]);
        }
        else
        {
            for (int c = 0; c < cs.HighWater; c++)
                if (cs.IsLive(c) && (cs.BodyA[c] == body || cs.BodyB[c] == body)) WakeOther(core, cs.BodyA[c] == body ? cs.BodyB[c] : cs.BodyA[c]);
        }
    }

    private static void WakeOther(WorldCore core, int other)
    {
        if (other < 0 || !core.Bodies.IsDynamic(other) || core.Bodies.IsAwake(other)) return;
        core.WakeIsland(core.Bodies.Island[other]);
        core.Bodies.SleepTime[other] = 0f;
    }

    /// <summary>
    /// Creates one body per group from the source body's frame, with the pieces as hull shapes. A piece the hull builder refuses is dropped and counted. A group left with no shape creates no body.
    /// Each new body inherits the velocity field of the source: its centre of mass moves as v + w x r and it turns with w, which conserves linear and angular momentum when the groups tile the source.
    /// </summary>
    private List<Created> Materialize(int source, List<Group> groups)
    {
        var host = Host; var core = host.DestructionCore; var bodies = core.Bodies;
        var result = new List<Created>(groups.Count);
        foreach (var g in groups)
        {
            if (g.Pieces.Count == 0) continue;
            var handle = host.CloneBody(source, g.Type);
            var shapes = new List<ShapeHandle>(g.Pieces.Count);
            var kept = new Group { Type = g.Type, Seed = g.Seed };
            for (int i = 0; i < g.Pieces.Count; i++)
            {
                try
                {
                    var sh = host.AddPiece(handle, g.Pieces[i].FloatPoints(), g.Templates[i]);
                    shapes.Add(sh); kept.Add(g.Pieces[i], g.Templates[i], g.PartTags[i]);
                }
                catch (ArgumentException) { _dropped++; }
            }
            if (shapes.Count == 0) { host.DestroyBody(handle); continue; }
            result.Add(new Created { Body = handle, Shapes = shapes.ToArray(), Group = kept });
        }
        // Velocity field of the source.
        var cSrc = CentreOfMass(source);
        var v = Vec3(bodies.LinearVelocityOf(source));
        var w = AngularVelocityOf(source);
        bool movable = bodies.TypeOf(source) != BodyType.Static;
        foreach (var c in result)
        {
            if (c.Group.Type != BodyType.Dynamic) continue;
            int b = c.Body.Index;
            var r = ToF(CentreOfMass(b) - cSrc);
            if (movable) SetVec(bodies.LinearVelocityOf(b), v + Cross(w, r));
            var av = bodies.AngularVelocityOf(b);
            if (movable) { if (av.Length >= 3) { av[0] = w.X; av[1] = w.Y; av[2] = w.Z; } else av[0] = w.Z; }
        }
        return result;
    }

    private static Vector3F ToF(Vector3D d) => new((float)d.X, (float)d.Y, (float)d.Z);

    // ------------------------------------------------------------------ Voronoi fracture

    /// <summary>
    /// Replaces a body by the Voronoi cells of its shapes. <paramref name="impactRel"/> is the impact point relative to the centre of mass in world axes (or null: use the definition's point, else the centre).
    /// Returns the handles of the new bodies in cell order, or null when nothing could be cut (unsupported shapes, a body in a machine, fewer than two cells).
    /// </summary>
    internal List<BodyHandle>? Fracture(int body, FractureSettings f, Vector3F? impactRel, float impulse, FractureKind kind)
    {
        var host = Host; var core = host.DestructionCore; var bodies = core.Bodies;
        if (!bodies.IsLive(body)) { Failure = "The body is not alive."; return null; }
        if (!host.CanReplace(body)) { Failure = "The body belongs to a joint, character, vehicle or ragdoll and cannot be replaced."; return null; }
        var infos = new List<ShapeInfo>();
        if (!host.ReadShapes(body, infos) || infos.Count == 0) { Failure = "Only bodies made of convex hull, box (3D) or polygon (2D) shapes with no rounding can be fractured."; return null; }
        var pieces = new List<Piece>(infos.Count);
        double total = 0;
        foreach (var s in infos) { pieces.Add(s.Piece); s.Piece.Measure(out double m, out _); total += m; }
        if (!(total > 0)) { Failure = "The body has no volume."; return null; }

        var com = CentreOfMass(body);
        var lc = host.LocalCenter(body);
        Vector3F rel;
        bool haveImpact = true;
        if (impactRel is { } ir) rel = ir;
        else if (f.ImpactPoint is { } ip) rel = ToF(ip - com);
        else { rel = default; haveImpact = false; }
        Vector3D impactLocal;
        if (haveImpact) { var b = host.ToBody(body, rel) + lc; impactLocal = new Vector3D(b.X, b.Y, host.Planar ? 0 : b.Z); }
        else
        {
            var lo = new Vector3D(double.MaxValue, double.MaxValue, double.MaxValue); var hi = -lo;
            foreach (var p in pieces) { p.Bounds(out var l, out var h); lo = new Vector3D(Math.Min(lo.X, l.X), Math.Min(lo.Y, l.Y), Math.Min(lo.Z, l.Z)); hi = new Vector3D(Math.Max(hi.X, h.X), Math.Max(hi.Y, h.Y), Math.Max(hi.Z, h.Z)); }
            impactLocal = (lo + hi) * 0.5;
        }
        double radius = f.ImpactRadius;
        if (!(radius > 0)) { radius = 0; foreach (var p in pieces) radius = Math.Max(radius, p.Radius(impactLocal)); }

        var user = new Vector3D[f.Points.Length];
        for (int i = 0; i < user.Length; i++) user[i] = new Vector3D(f.Points[i].X, f.Points[i].Y, host.Planar ? 0 : f.Points[i].Z);
        var seeds = Fracturer.MakeSeeds(pieces, f.CellCount, f.Pattern, f.Seed, impactLocal, radius, f.ClusterPower, user);
        var cells = new Piece?[pieces.Count][];
        for (int p = 0; p < pieces.Count; p++) cells[p] = Fracturer.Cells(pieces[p], seeds);

        var groups = new List<Group>(seeds.Length);
        for (int j = 0; j < seeds.Length; j++)
        {
            var g = new Group { Type = BodyType.Dynamic, Seed = j };
            for (int p = 0; p < pieces.Count; p++) if (cells[p][j] is { } cell) g.Add(cell, infos[p].Template, -1);
            groups.Add(g);
        }
        if (!MergeSmall(groups, seeds, total * f.MinFragmentVolume, f.MergeSmall)) { Failure = "The cut leaves fewer than two cells above the minimum fragment size."; return null; }

        WakeNeighbours(body);
        var made = Materialize(body, groups);
        if (made.Count < 2)
        {
            // The cut left fewer than two usable bodies: undo.
            foreach (var c in made) host.DestroyBody(c.Body);
            Failure = "The cut leaves fewer than two usable bodies.";
            return null;
        }

        var srcHandle = core.HandleOfBody(body);
        host.DestroyBody(srcHandle);
        Remove(body);
        var handles = new List<BodyHandle>(made.Count);
        int first = EventFragments.Count;
        long step = host.StepCount;
        for (int k = 0; k < made.Count; k++)
        {
            var c = made[k];
            handles.Add(c.Body);
            EventFragments.Add(c.Body);
            double m = c.Group.Measure();
            AddFragment(c.Body, (float)m, step);
            if (f.Depth > 0)
            {
                var child = f.Clone();
                child.Depth = f.Depth - 1;
                child.CellCount = f.ChildCellCount > 0 ? f.ChildCellCount : Math.Max(2, f.CellCount / 2);
                child.ImpulseThreshold = f.ImpulseThreshold * f.ChildThresholdScale;
                child.Seed = unchecked(f.Seed * 0x9E3779B97F4A7C15UL + (ulong)(k + 1));
                child.Pattern = FracturePattern.Uniform; child.Points = []; child.ImpactPoint = null;
                GetOrAdd(c.Body.Index).Fracture = child;
            }
        }
        _fractures++;
        Events.Add(new FractureRecord((int)kind, srcHandle, first, made.Count, impulse, com + new Vector3D(rel.X, rel.Y, rel.Z)));
        return handles;
    }

    // Groups under the volume limit join the group of the nearest seed that is big enough, or are deleted. False when fewer than two groups would remain.
    private bool MergeSmall(List<Group> groups, Vector3D[] seeds, double limit, bool merge)
    {
        int n = groups.Count;
        var measure = new double[n];
        int bigCount = 0;
        for (int i = 0; i < n; i++)
        {
            measure[i] = groups[i].Pieces.Count == 0 ? 0 : groups[i].Measure();
            if (groups[i].Pieces.Count > 0 && measure[i] >= limit) bigCount++;
        }
        if (bigCount < 2) return false;
        for (int i = 0; i < n; i++)
        {
            var g = groups[i];
            if (g.Pieces.Count == 0 || measure[i] >= limit) continue;
            int best = -1; double bd = double.MaxValue;
            if (merge)
                for (int j = 0; j < n; j++)
                {
                    if (j == i || groups[j].Pieces.Count == 0 || measure[j] < limit) continue;
                    double d = (seeds[j] - seeds[i]).LengthSquared();
                    if (d < bd) { bd = d; best = j; }
                }
            if (best >= 0)
            {
                for (int k = 0; k < g.Pieces.Count; k++) groups[best].Add(g.Pieces[k], g.Templates[k], g.PartTags[k]);
                _merged++;
            }
            else _dropped++;
            g.Pieces.Clear(); g.Templates.Clear(); g.PartTags.Clear();
        }
        return true;
    }

    internal void AddFragment(BodyHandle body, float measure, long step)
    {
        Fragments.Add(new Fragment { Body = body.Index, Generation = body.Generation, Sequence = _nextSequence++, BirthStep = step, Measure = measure });
        _created++;
    }

    // ------------------------------------------------------------------ slicing

    /// <summary>
    /// Cuts a body by a plane (point and normal in world axes). Returns the bodies on the positive and negative side of the normal, or null when the plane misses the body or the body can't be cut.
    /// A bonded compound is cut part by part: bonds between pieces that stay on one side survive (their faces recomputed), bonds across the plane are gone.
    /// </summary>
    internal (BodyHandle positive, BodyHandle negative)? Slice(int body, Vector3D point, Vector3F normal, bool trackDebris)
    {
        var host = Host; var core = host.DestructionCore; var bodies = core.Bodies;
        if (!bodies.IsLive(body)) { Failure = "The body is not alive."; return null; }
        if (!host.CanReplace(body)) { Failure = "The body belongs to a joint, character, vehicle or ragdoll and cannot be replaced."; return null; }
        double len = Math.Sqrt((double)normal.X * normal.X + (double)normal.Y * normal.Y + (double)normal.Z * normal.Z);
        if (!(len > 0) || !double.IsFinite(len)) { Failure = "The cutting plane needs a finite, non-zero normal."; return null; }
        var infos = new List<ShapeInfo>();
        if (!host.ReadShapes(body, infos) || infos.Count == 0) { Failure = "Only bodies made of convex hull, box (3D) or polygon (2D) shapes with no rounding can be sliced."; return null; }
        var com = CentreOfMass(body);
        var nl = host.ToBody(body, normal / (float)len);
        var n = new Vector3D(nl.X, nl.Y, host.Planar ? 0 : nl.Z);
        double nlen = n.Length();
        if (!(nlen > 0)) { Failure = "The cutting plane normal is parallel to the body's missing axis."; return null; }
        n /= nlen;
        var pl = host.ToBody(body, ToF(point - com)) + host.LocalCenter(body);
        double d = n.X * pl.X + n.Y * pl.Y + (host.Planar ? 0 : n.Z * pl.Z);

        var rec = Find(body);
        var neg = new Group { Type = BodyType.Dynamic }; var pos = new Group { Type = BodyType.Dynamic };
        var srcType = bodies.TypeOf(body);
        // For a bonded compound: which new group and index each source part went to.
        int parts = rec?.Parts.Length ?? 0;
        var partOfShape = new Dictionary<long, int>();
        if (rec != null) for (int i = 0; i < parts; i++) partOfShape[((long)rec.Parts[i].Shape << 32) | (uint)rec.Parts[i].ShapeGeneration] = i;
        double total = 0; foreach (var s in infos) { s.Piece.Measure(out double m, out _); total += m; }
        double eps = total * 1e-9;
        bool cutAny = false;
        foreach (var s in infos)
        {
            int tag = -1;
            if (rec != null && partOfShape.TryGetValue(((long)s.Shape << 32) | (uint)s.Generation, out int pi)) tag = pi;
            var a = s.Piece.Clip(n, d);              // negative side
            var b = s.Piece.Clip(-n, -d);            // positive side
            double ma = 0, mb = 0;
            if (a != null) a.Measure(out ma, out _);
            if (b != null) b.Measure(out mb, out _);
            if (ma <= eps) a = null;
            if (mb <= eps) b = null;
            if (a != null && b != null) cutAny = true;
            if (a == null && b == null) continue;
            if (a != null) neg.Add(a, s.Template, tag);
            if (b != null) pos.Add(b, s.Template, tag);
        }
        if (!cutAny || neg.Pieces.Count == 0 || pos.Pieces.Count == 0) { Failure = "The plane does not cut through the body."; return null; }
        if (srcType == BodyType.Static) { neg.Type = BodyType.Static; pos.Type = BodyType.Static; }

        WakeNeighbours(body);
        var made = Materialize(body, [neg, pos]);
        if (made.Count != 2) { foreach (var c in made) host.DestroyBody(c.Body); Failure = "A half of the cut is too thin to be a body."; return null; }
        var srcHandle = core.HandleOfBody(body);
        host.DestroyBody(srcHandle);
        Remove(body);

        if (rec != null)
        {
            foreach (var c in made)
            {
                var nr = GetOrAdd(c.Body.Index);
                nr.Fracture = rec.Fracture?.Clone();
                if (rec.Bonds != null) RebuildBonded(nr, rec, c);
            }
        }
        long step = host.StepCount;
        int first = EventFragments.Count;
        foreach (var c in made)
        {
            EventFragments.Add(c.Body);
            if (trackDebris) AddFragment(c.Body, (float)c.Group.Measure(), step); else _created++;
        }
        _slices++;
        Events.Add(new FractureRecord((int)FractureKind.Slice, srcHandle, first, 2, 0f, point));
        // Positive body first, whichever order Materialize produced them in: the order of `made` follows the groups (negative, positive).
        return (made[1].Body, made[0].Body);
    }

    // Builds the bond graph of a half of a sliced compound: the parts of this half, and the bonds of the source whose two ends both survive on this side, with their faces recomputed.
    private static void RebuildBonded(Record nr, Record src, Created c)
    {
        nr.Bonds = new BondSettings { Strength = src.Bonds!.Strength, ShearFactor = src.Bonds.ShearFactor, OverloadSteps = src.Bonds.OverloadSteps };
        var newParts = new List<Part>();
        var map = new Dictionary<int, int>(); // source part -> new part
        for (int i = 0; i < c.Group.Pieces.Count; i++)
        {
            int tag = c.Group.PartTags[i];
            var piece = c.Group.Pieces[i];
            piece.Measure(out double m, out var cen);
            var part = new Part { Shape = c.Shapes[i].Index, ShapeGeneration = c.Shapes[i].Generation, Measure = (float)m, Mass = (float)(m * c.Group.Templates[i].Density), Centroid = ToF(cen), Anchored = tag >= 0 && src.Parts[tag].Anchored };
            if (tag >= 0) map[tag] = newParts.Count;
            newParts.Add(part);
        }
        nr.Parts = newParts.ToArray();
        var bonds = new List<Bond>();
        var pieceOfPart = new Dictionary<int, Piece>();
        for (int i = 0; i < c.Group.Pieces.Count; i++) if (c.Group.PartTags[i] >= 0) pieceOfPart[c.Group.PartTags[i]] = c.Group.Pieces[i];
        foreach (var b in src.BondList)
        {
            if (!b.Alive || !map.TryGetValue(b.A, out int na) || !map.TryGetValue(b.B, out int nb)) continue;
            var face = Fracturer.Shared(pieceOfPart[b.A], pieceOfPart[b.B]);
            if (face == null) continue;
            var f = face.Value;
            bonds.Add(new Bond { A = na, B = nb, Measure = (float)f.Measure, Centroid = ToF(f.Centroid), Normal = ToF(f.Normal), Strength = b.Strength });
        }
        nr.BondList = bonds.ToArray();
    }
}
