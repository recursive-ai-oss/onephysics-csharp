using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal enum QueryKind : byte { Ray, Cast, Overlap, Closest }

internal enum QueryMode : byte { Closest, Any, All }

/// <summary>One accepted shape during a query. <c>Key</c> orders the results: ray and cast fraction, closest-point distance, overlap centre distance (closest mode) or shape slot (all mode); ties break on the shape slot.</summary>
internal struct RawHit
{
    public double Key;
    public int Slot, Sub;
    public double Px, Py, Pz;
    public float Nx, Ny, Nz, Fraction, Distance;
}

/// <summary>Per-thread scratch for results and for the collision code's candidate lists, so concurrent queries do not share state and steady-state queries do not allocate.</summary>
internal sealed class QueryScratch
{
    [ThreadStatic] private static QueryScratch? t_cached;

    public RawHit[] Hits = new RawHit[8];
    public int Count;
    public int Capacity;
    public readonly List<SoupTriangle3D> Tris = new();
    public readonly List<int> Ids = new();
    public readonly TriangleShapeBuffer3D TriBuf = new();
    internal readonly Engine3D.QueryState State = new();
    private bool _inUse;

    /// <summary>Creates the calling thread's scratch with room for <paramref name="capacity"/> hits, so its first queries allocate nothing.</summary>
    public static void Warm(int capacity)
    {
        var s = t_cached ??= new QueryScratch();
        if (s.Hits.Length < capacity) { s.Hits = new RawHit[capacity]; }
    }

    public static QueryScratch Acquire(int capacity)
    {
        var s = t_cached ??= new QueryScratch();
        if (s._inUse) { s = new QueryScratch(); } // a filter delegate ran a query of its own
        s._inUse = true;
        s.Count = 0;
        s.Capacity = capacity;
        if (s.Hits.Length < capacity) { s.Hits = new RawHit[Math.Max(capacity, s.Hits.Length * 2)]; }
        return s;
    }

    public void Release()
    {
        State.Filter = default; // do not keep a caller's predicate alive
        _inUse = false;
    }

    /// <summary>Keeps the best <see cref="Capacity"/> hits ordered by (key, slot).</summary>
    public void Insert(in RawHit h)
    {
        int i = Count;
        while (i > 0 && (Hits[i - 1].Key > h.Key || (Hits[i - 1].Key == h.Key && Hits[i - 1].Slot > h.Slot))) { i--; }
        if (i >= Capacity) { return; }
        int last = Count < Capacity ? Count : Capacity - 1;
        for (int j = last; j > i; j--) { Hits[j] = Hits[j - 1]; }
        Hits[i] = h;
        if (Count < Capacity) { Count++; }
    }

    public bool Full => Count == Capacity;
}

/// <summary>A ray or box sweep whose boxes are grown before the test (so stale or float-rounded boxes cannot hide a shape).</summary>
internal readonly struct PaddedRay<TRay> : IRay<Aabb3D> where TRay : struct, IRay<Aabb3D>
{
    private readonly TRay _ray;
    private readonly double _pad;

    public PaddedRay(TRay ray, double pad) { _ray = ray; _pad = pad; }

    public bool Hit(in Aabb3D box, double maxFraction, out double enter) =>
        _ray.Hit(new Aabb3D(box.MinX - _pad, box.MinY - _pad, box.MinZ - _pad, box.MaxX + _pad, box.MaxY + _pad, box.MaxZ + _pad), maxFraction, out enter);
}

internal struct QueryParams
{
    public QueryKind Kind;
    public double X, Y, Z;       // ray origin, mover position, or closest-point query point
    public float Tx, Ty, Tz;     // ray or cast translation (world frame)
    public ConvexShape3D Mover;  // cast and overlap shape in its own frame
    public QuaternionF MoverRot;
    public float MaxDistance;    // closest point
}

internal sealed partial class Engine3D
{
    private const double QueryEpsilon = 1e-4;

    /// <summary>Test hook: visit every shape instead of using the trees (same narrow phase and ordering).</summary>
    internal bool BruteQueries;

    /// <summary>Test hook: false sends every candidate to the exact test, so tests can check the closed-form shortcuts and box rejections change no result.</summary>
    internal bool QuickRejects = true;

    private double _queryPad;
    private bool _queryPadValid;

    public void InvalidatePad() => _queryPadValid = false;

    /// <summary>
    /// How far any moving proxy's stored (fat) box may be behind the body's pose: bodies move during a step but their proxies are updated at the start of the next one, so between steps a box can be stale.
    /// Queries grow the moving-body trees' boxes by this (computed lazily once per step; it never changes simulation state).
    /// </summary>
    private double QueryPad()
    {
        if (_queryPadValid) { return _queryPad; }
        double pad = 0;
        var shapes = Core.Shapes;
        var bodies = Core.Bodies;
        for (int s = 0; s < shapes.HighWater; s++)
        {
            if (!shapes.Handles.IsLive(s)) { continue; }
            int b = shapes.Owner[s];
            if (bodies.TypeOf(b) == BodyType.Static) { continue; }
            var fat = Bp.GetFatBounds(Proxy[s]);
            var t = TightBox(b, s);
            double o = Math.Max(Math.Max(Math.Max(fat.MinX - t.MinX, fat.MinY - t.MinY), Math.Max(fat.MinZ - t.MinZ, t.MaxX - fat.MaxX)), Math.Max(t.MaxY - fat.MaxY, t.MaxZ - fat.MaxZ));
            if (o > pad) { pad = o; }
        }

        _queryPad = pad + pad * 1e-9;
        _queryPadValid = true;
        return _queryPad;
    }

    private bool Passes(in QueryFilter3D f, int s)
    {
        if (IsSensor(s) && !f.IncludeSensors) { return false; }
        if (((f.LayerMask >> ShapeLayer[s]) & 1u) == 0) { return false; }
        if ((Category[s] & f.MaskBits) == 0 || (f.CategoryBits & Mask[s]) == 0) { return false; }
        var p = f.Predicate;
        return p == null || p(WrapShape(s));
    }

    /// <summary>The shape's frame in world axes: orientation and the double position of its origin (centre of mass plus the rotated offset), like the contact stage.</summary>
    private void ShapeFrame(int s, out double ox, out double oy, out double oz, out QuaternionF rs)
    {
        int b = Core.Shapes.Owner[s];
        var pos = Core.Bodies.PositionOf(b);
        var q = RotationOfBody(b);
        var l = Local[s];
        rs = q * l.Rotation;
        var t = q.Rotate(l.Translation - LocalCenterOf(b));
        ox = pos[0] + (double)t.X;
        oy = pos[1] + (double)t.Y;
        oz = pos[2] + (double)t.Z;
    }

    /// <summary>
    /// Smallest distance between a geometry and a convex shape placed at <paramref name="moverInShape"/> (both in the geometry's frame), 0 when they overlap, +infinity when farther than <paramref name="maxDist"/>.
    /// Reports the closest point on the geometry, the unit normal from the geometry towards the mover (zero when the cores overlap) and the sub-shape (compound child or triangle id). Ties go to the lowest sub-shape.
    /// </summary>
    private static float MinDistance(Geom3D g, in ConvexShape3D mover, in RigidTransformF moverInShape, float maxDist, QueryScratch sc, out Vector3F point, out Vector3F normal, out int sub)
    {
        point = default; normal = default; sub = 0;
        if (g.IsConvex)
        {
            var d = Gjk3D.Distance(g.Convex, mover, moverInShape);
            point = d.PointA; normal = d.Normal;
            float dist = d.Overlapping ? 0f : d.Distance;
            return dist <= maxDist ? dist : float.PositiveInfinity;
        }

        float best = float.PositiveInfinity;
        if (g.Compound != null)
        {
            var comp = g.Compound;
            for (int j = 0; j < comp.SlotCount; j++)
            {
                if (!comp.IsLive(j)) { continue; }
                var child = comp.GetChild(j);
                var d = Gjk3D.Distance(child.Shape, mover, child.Local.Inverted() * moverInShape);
                float dist = d.Overlapping ? 0f : d.Distance;
                if (dist < best && dist <= maxDist)
                {
                    best = dist; sub = j;
                    point = child.Local.TransformPoint(d.PointA); normal = child.Local.Rotation.Rotate(d.Normal);
                }
            }

            return best;
        }

        var soup = g.Soup!;
        float r = Math.Min(maxDist, 1f);
        for (;;)
        {
            NarrowScratch.Tris.Clear();
            soup.GetCandidates(mover, moverInShape, r, NarrowScratch.Tris, NarrowScratch.Ids);
            best = float.PositiveInfinity;
            for (int i = 0; i < NarrowScratch.Tris.Count; i++)
            {
                var tri = NarrowScratch.Tris[i];
                var d = Gjk3D.Distance(NarrowScratch.TriBuf.Set(tri.V0, tri.V1, tri.V2), mover, moverInShape);
                float dist = d.Overlapping ? 0f : d.Distance;
                if (dist < best && dist <= maxDist) { best = dist; sub = tri.Id; point = d.PointA; normal = d.Normal; }
            }

            if (best <= r || r >= maxDist) { return best; }
            r = Math.Min(maxDist, Math.Max(r * 2f, float.IsFinite(best) ? best : 0f));
        }
    }

    // Cuts the segment from..from+delta to the cube the compound accepts (half its coordinate range, so float rounding stays clear of the limit); a segment already inside is returned untouched.
    private static bool ClipToReach(Vector3F from, Vector3F delta, out Vector3F origin, out Vector3F span, out float t0, out float t1)
    {
        origin = from; span = delta; t0 = 0f; t1 = 1f;
        double lim = BoxLimits.MaxCoordinate * 0.5;
        double a0 = 0.0, a1 = 1.0;
        bool inside = true;
        for (int k = 0; k < 3; k++)
        {
            double o = k == 0 ? from.X : k == 1 ? from.Y : from.Z;
            double d = k == 0 ? delta.X : k == 1 ? delta.Y : delta.Z;
            double e = o + d;
            if (Math.Abs(o) <= lim && Math.Abs(e) <= lim) { continue; }
            inside = false;
            if (d == 0.0) { return false; }
            double u = (-lim - o) / d, v = (lim - o) / d;
            if (u > v) { (u, v) = (v, u); }
            if (u > a0) { a0 = u; }
            if (v < a1) { a1 = v; }
            if (a0 > a1) { return false; }
        }

        if (inside) { return true; }
        t0 = (float)a0; t1 = (float)a1;
        origin = from + (delta * t0);
        span = delta * (t1 - t0);
        return true;
    }

    // Cheap rejection of a convex candidate before the exact test. Every shape lies inside its padded local box (Geom3D.ConvexMin/ConvexMax), so a ray, sweep or shape whose reach misses that box grown by a
    // slack well above the narrow phase's float32 error cannot touch the shape, and the exact test would have said no. Anything that might touch goes on to the exact test unchanged, so results are the same.
    private static float RejectSlack(float scale) => 1e-3f + (scale * (1f / 4096f));

    private static float Extent(Vector3F a) => Math.Max(Math.Abs(a.X), Math.Max(Math.Abs(a.Y), Math.Abs(a.Z)));

    // True when the segment o + t d (t in [0,1]) cannot reach the box [bmin, bmax] grown by e.
    private static bool SegmentMisses(Vector3F o, Vector3F d, Vector3F bmin, Vector3F bmax, float e)
    {
        double t0 = 0.0, t1 = 1.0;
        return !(Slab.Clip(o.X, d.X, 1.0 / d.X, (double)bmin.X - e, (double)bmax.X + e, ref t0, ref t1)
            && Slab.Clip(o.Y, d.Y, 1.0 / d.Y, (double)bmin.Y - e, (double)bmax.Y + e, ref t0, ref t1)
            && Slab.Clip(o.Z, d.Z, 1.0 / d.Z, (double)bmin.Z - e, (double)bmax.Z + e, ref t0, ref t1));
    }

    // The mover's box in the shape's frame, when that is cheap (not for hulls and tapered capsules).
    private static bool MoverBox(in ConvexShape3D mover, in RigidTransformF rel, out Vector3F min, out Vector3F max)
    {
        switch (mover.Kind)
        {
            case ConvexShapeKind3D.Sphere:
            {
                float r = mover.ConvexRadius; var c = rel.Translation;
                min = new Vector3F(c.X - r, c.Y - r, c.Z - r); max = new Vector3F(c.X + r, c.Y + r, c.Z + r);
                return true;
            }
            case ConvexShapeKind3D.Box:
            case ConvexShapeKind3D.Capsule:
            case ConvexShapeKind3D.Cylinder:
                ShapeBounds3D.Compute(in mover, in rel, out min, out max);
                return true;
            default:
                min = default; max = default;
                return false;
        }
    }

    // Sphere against a sphere, box or capsule, decided in closed form when the answer is clear. The distance between the sphere's centre (lo, in the shape's frame) and the shape's core, less both radii, is
    // negative when they overlap; GJK reports overlap for such a pair too. Within the slack of the exact test the answer is left to GJK, so the overlap set is the one GJK alone would give.
    // Closest-point formulas: Ericson, Real-Time Collision Detection, 5.1.
    private static bool TryDecideSphereOverlap(in ConvexShape3D shape, float moverRadius, Vector3F lo, float slack, out bool overlaps)
    {
        float gap; // distance from the centre to the shape's surface, less the mover's radius
        float r = shape.ConvexRadius;
        switch (shape.Kind)
        {
            case ConvexShapeKind3D.Sphere:
                gap = float.Sqrt(lo.X * lo.X + lo.Y * lo.Y + lo.Z * lo.Z) - r - moverRadius;
                break;
            case ConvexShapeKind3D.Box:
            {
                var h = shape.CoreSize;
                float dx = Math.Max(Math.Abs(lo.X) - h.X, 0f), dy = Math.Max(Math.Abs(lo.Y) - h.Y, 0f), dz = Math.Max(Math.Abs(lo.Z) - h.Z, 0f);
                gap = float.Sqrt(dx * dx + dy * dy + dz * dz) - r - moverRadius;
                break;
            }
            case ConvexShapeKind3D.Capsule:
            {
                float hy = shape.CoreSize.Y;
                float dy = lo.Y - Math.Min(Math.Max(lo.Y, -hy), hy);
                gap = float.Sqrt(lo.X * lo.X + dy * dy + lo.Z * lo.Z) - r - moverRadius;
                break;
            }
            case ConvexShapeKind3D.Hull:
            {
                // Past a face plane the centre is at least that far from the hull; inside every plane it is inside the hull. In between (near an edge or corner) GJK decides.
                float s = float.NegativeInfinity;
                foreach (var f in shape.Hull!.Faces)
                {
                    float d = lo.X * f.Normal.X + lo.Y * f.Normal.Y + lo.Z * f.Normal.Z - f.Offset;
                    if (d > s) { s = d; }
                }

                if (s <= -slack) { overlaps = true; return true; }
                gap = s - r - moverRadius;
                if (gap >= slack) { overlaps = false; return true; }
                overlaps = false;
                return false;
            }

            default:
                overlaps = false;
                return false;
        }

        if (gap <= -slack) { overlaps = true; return true; }
        if (gap >= slack) { overlaps = false; return true; }
        overlaps = false;
        return false;
    }

    /// <summary>The narrow phase for one candidate shape. All of it is float arithmetic in the shape's own frame after one double subtraction, like the contact stage.</summary>
    private bool Narrow(in QueryParams q, QueryMode mode, int s, QueryScratch sc, out RawHit h)
    {
        h = default;
        h.Slot = s;
        ShapeFrame(s, out double ox, out double oy, out double oz, out QuaternionF rs);
        var inv = rs.Conjugated();
        var g = Geoms[GeomOf[s]]!;
        var lo = inv.Rotate(new Vector3F((float)(q.X - ox), (float)(q.Y - oy), (float)(q.Z - oz)));
        switch (q.Kind)
        {
            case QueryKind.Ray:
            {
                var lt = inv.Rotate(new Vector3F(q.Tx, q.Ty, q.Tz));
                float f; Vector3F n; int sub = 0;
                if (g.IsConvex)
                {
                    if (QuickRejects && SegmentMisses(lo, lt, g.ConvexMin, g.ConvexMax, RejectSlack(Math.Max(Extent(lo) + Extent(lt), Math.Max(Extent(g.ConvexMin), Extent(g.ConvexMax)))))) { return false; }
                    if (!Gjk3D.RayCast(g.Convex, RigidTransformF.Identity, lo, lt, 1f, out var hit)) { return false; }
                    f = hit.Fraction; n = hit.Normal;
                }
                else if (g.Compound != null)
                {
                    // The compound refuses segments that leave its coordinate range, so a long ray is cut to the range first.
                    if (!ClipToReach(lo, lt, out var co, out var cd, out float t0, out float t1)) { return false; }
                    if (!g.Compound.RayCast(co, cd, 1f, out var hit)) { return false; }
                    f = t0 + (hit.Fraction * (t1 - t0)); n = hit.Normal; sub = hit.Child;
                }
                else
                {
                    if (!g.Soup!.RayCast(lo, lt, 1f, out var hit)) { return false; }
                    f = hit.Fraction; n = hit.BackFace ? -hit.Normal : hit.Normal; sub = hit.TriangleId;
                }

                var wp = rs.Rotate(lo + lt * f); var wn = rs.Rotate(n);
                h.Key = f; h.Fraction = f; h.Sub = sub;
                h.Px = ox + (double)wp.X; h.Py = oy + (double)wp.Y; h.Pz = oz + (double)wp.Z; h.Nx = wn.X; h.Ny = wn.Y; h.Nz = wn.Z;
                return true;
            }

            case QueryKind.Cast:
            {
                var motion = inv.Rotate(new Vector3F(q.Tx, q.Ty, q.Tz));
                var rel = new RigidTransformF(inv * q.MoverRot, lo);
                float f; Vector3F n, p; int sub = 0; bool startsOverlapping;
                if (g.IsConvex)
                {
                    if (QuickRejects && MoverBox(in q.Mover, in rel, out var mmin, out var mmax))
                    {
                        // The mover's centre travels the motion; the shape's box grows by the mover's half sizes (Minkowski sum), which makes the sweep a segment test.
                        var half = (mmax - mmin) * 0.5f; var centre = (mmax + mmin) * 0.5f;
                        float e = RejectSlack(Math.Max(Extent(centre) + Extent(motion) + Extent(half), Math.Max(Extent(g.ConvexMin), Extent(g.ConvexMax))));
                        if (SegmentMisses(centre, motion, g.ConvexMin - half, g.ConvexMax + half, e)) { return false; }
                    }

                    if (!Gjk3D.ShapeCast(g.Convex, q.Mover, rel, -motion, out var hit)) { return false; }
                    f = hit.Fraction; n = hit.Normal; p = hit.PointB + motion * f; startsOverlapping = hit.StartsOverlapping;
                }
                else if (g.Compound != null)
                {
                    if (!ClipToReach(lo, motion, out var co, out var cd, out float t0, out float t1)) { return false; }
                    var crel = new RigidTransformF(rel.Rotation, co);
                    if (!g.Compound.ShapeCast(q.Mover, crel, cd, out var hit)) { return false; }
                    f = t0 + (hit.Fraction * (t1 - t0)); n = -hit.Normal; p = hit.PointOnChild; sub = hit.Child; startsOverlapping = hit.StartsOverlapping;
                }
                else
                {
                    if (!g.Soup!.ShapeCast(q.Mover, rel, motion, out var hit, NarrowScratch.Ids)) { return false; }
                    f = hit.Fraction; n = hit.Normal; p = hit.Point; sub = hit.TriangleId; startsOverlapping = hit.StartsOverlapping;
                }

                if (startsOverlapping) { n = default; }
                var wp = rs.Rotate(p); var wn = rs.Rotate(n);
                h.Key = f; h.Fraction = f; h.Sub = sub;
                h.Px = ox + (double)wp.X; h.Py = oy + (double)wp.Y; h.Pz = oz + (double)wp.Z; h.Nx = wn.X; h.Ny = wn.Y; h.Nz = wn.Z;
                return true;
            }

            case QueryKind.Overlap:
            {
                var rel = new RigidTransformF(inv * q.MoverRot, lo);
                if (QuickRejects && g.IsConvex && MoverBox(in q.Mover, in rel, out var omin, out var omax))
                {
                    float e = RejectSlack(Math.Max(Math.Max(Extent(omin), Extent(omax)), Math.Max(Extent(g.ConvexMin), Extent(g.ConvexMax))));
                    if (omin.X > g.ConvexMax.X + e || omax.X < g.ConvexMin.X - e || omin.Y > g.ConvexMax.Y + e || omax.Y < g.ConvexMin.Y - e || omin.Z > g.ConvexMax.Z + e || omax.Z < g.ConvexMin.Z - e) { return false; }
                }

                if (QuickRejects && g.IsConvex && q.Mover.Kind == ConvexShapeKind3D.Sphere
                    && TryDecideSphereOverlap(in g.Convex, q.Mover.ConvexRadius, lo, RejectSlack(Math.Max(Extent(lo), Math.Max(Extent(g.ConvexMin), Extent(g.ConvexMax)))), out bool touching))
                {
                    if (!touching) { return false; }
                }
                else if (!(MinDistance(g, q.Mover, rel, 0f, sc, out _, out _, out _) <= 0f)) { return false; }
                h.Sub = -1;
                if (mode != QueryMode.Closest) { h.Key = s; return true; }
                // Closest overlap: the shape whose surface is nearest the query shape's centre (0 when the centre is inside it).
                var at = new RigidTransformF(QuaternionF.Identity, lo);
                float d = MinDistance(g, ConvexShape3D.Point, at, float.MaxValue, sc, out var pt, out _, out _);
                var wp = rs.Rotate(pt);
                h.Key = d; h.Distance = d;
                h.Px = ox + (double)wp.X; h.Py = oy + (double)wp.Y; h.Pz = oz + (double)wp.Z;
                return true;
            }

            default:
            {
                var at = new RigidTransformF(QuaternionF.Identity, lo);
                float d = MinDistance(g, ConvexShape3D.Point, at, q.MaxDistance, sc, out var pt, out var nrm, out int sub);
                if (!(d <= q.MaxDistance)) { return false; }
                var wp = rs.Rotate(pt);
                h.Key = d; h.Distance = d; h.Sub = sub;
                h.Px = ox + (double)wp.X; h.Py = oy + (double)wp.Y; h.Pz = oz + (double)wp.Z;
                if (d == 0f) { h.Px = q.X; h.Py = q.Y; h.Pz = q.Z; }
                else
                {
                    var wn = rs.Rotate(nrm);
                    h.Nx = wn.X; h.Ny = wn.Y; h.Nz = wn.Z;
                }

                return true;
            }
        }
    }

    /// <summary>What one running query needs, kept in the thread's scratch so the callbacks the trees copy around are one reference (a struct of this size copied with its references costs a write-barriered block copy per tree).</summary>
    internal sealed class QueryState
    {
        public Engine3D E = null!;
        public QueryParams Q;
        public QueryFilter3D Filter;
        public QueryMode Mode;
        public QueryScratch S = null!;
    }

    private struct Visitor : IRayCallback, IOverlapCallback
    {
        public QueryState St;

        public readonly Engine3D E => St.E;
        public readonly ref readonly QueryParams Q => ref St.Q;
        public readonly ref readonly QueryFilter3D Filter => ref St.Filter;
        public readonly QueryMode Mode => St.Mode;
        public readonly QueryScratch S => St.S;

        private bool Visit(int slot)
        {
            if (!E.Passes(in Filter, slot) || !E.Narrow(in Q, Mode, slot, S, out RawHit h)) { return false; }
            S.Insert(in h);
            return true;
        }

        public double Report(int proxy, int userId, double maxFraction)
        {
            if (!Visit(userId)) { return maxFraction; }
            if (Mode == QueryMode.Any) { return 0.0; }
            if (!S.Full) { return maxFraction; }
            // Clip to the worst kept hit; never to 0, which would stop the cast before equal-fraction ties are seen.
            double worst = S.Hits[S.Count - 1].Key;
            return worst > double.Epsilon ? worst : double.Epsilon;
        }

        public bool Report(int proxy, int userId)
        {
            Visit(userId);
            return !(Mode == QueryMode.Any && S.Count > 0);
        }
    }

    /// <summary>
    /// Runs one query and returns the thread's scratch holding the best hits (at most <paramref name="capacity"/>; 1 for closest and any), ordered. The caller reads <see cref="QueryScratch.Hits"/>
    /// and must call <see cref="QueryScratch.Release"/> (in a finally block).
    /// </summary>
    internal QueryScratch Run(in QueryParams q, in QueryFilter3D filter, QueryMode mode, int capacity)
    {
        if (mode != QueryMode.All) { capacity = 1; }
        capacity = Math.Max(0, Math.Min(capacity, Core.Shapes.HighWater));
        var scratch = QueryScratch.Acquire(capacity);
        try
        {
            if (capacity == 0) { return scratch; }
            var st = scratch.State;
            st.E = this; st.Q = q; st.Filter = filter; st.Mode = mode; st.S = scratch;
            var v = new Visitor { St = st };
            if (BruteQueries) { Brute(ref v); }
            else if (q.Kind == QueryKind.Closest) { ClosestSearch(ref v); }
            else { Trees(ref v); }
            return scratch;
        }
        catch
        {
            scratch.Release();
            throw;
        }
    }

    private void Brute(ref Visitor v)
    {
        var shapes = Core.Shapes;
        for (int s = 0; s < shapes.HighWater; s++)
        {
            if (!shapes.Handles.IsLive(s)) { continue; }
            v.Report(0, s, 1.0);
            if (v.Mode == QueryMode.Any && v.S.Count > 0) { return; }
        }
    }

    private void Trees(ref Visitor v)
    {
        double pad = QueryPad();
        const BodyTypeMask moving = BodyTypeMask.Kinematic | BodyTypeMask.Dynamic;
        var q = v.Q;
        if (q.Kind == QueryKind.Ray)
        {
            var ray = new Ray3D(q.X, q.Y, q.Z, q.Tx, q.Ty, q.Tz);
            double f = Bp.RayCast(new PaddedRay<Ray3D>(ray, QueryEpsilon), 1.0, ref v, BodyTypeMask.Static);
            if (f > 0.0 || !(v.Mode == QueryMode.Any && v.S.Count > 0)) { Bp.RayCast(new PaddedRay<Ray3D>(ray, pad + QueryEpsilon), f, ref v, moving); }
            return;
        }

        ShapeBounds3D.Compute(q.Mover, new RigidTransformF(q.MoverRot, default), out var lmin, out var lmax);
        var box = new Aabb3D(q.X + (double)lmin.X, q.Y + (double)lmin.Y, q.Z + (double)lmin.Z, q.X + (double)lmax.X, q.Y + (double)lmax.Y, q.Z + (double)lmax.Z);
        if (q.Kind == QueryKind.Cast)
        {
            var sweep = new BoxCast3D(box, q.Tx, q.Ty, q.Tz);
            double f = Bp.RayCast(new PaddedRay<BoxCast3D>(sweep, QueryEpsilon), 1.0, ref v, BodyTypeMask.Static);
            if (f > 0.0 || !(v.Mode == QueryMode.Any && v.S.Count > 0)) { Bp.RayCast(new PaddedRay<BoxCast3D>(sweep, pad + QueryEpsilon), f, ref v, moving); }
            return;
        }

        var sb = Grow(box, QueryEpsilon);
        Bp.QueryAabb(in sb, ref v, BodyTypeMask.Static);
        if (v.Mode == QueryMode.Any && v.S.Count > 0) { return; }
        var mb = Grow(box, pad + QueryEpsilon);
        Bp.QueryAabb(in mb, ref v, moving);
    }

    private static Aabb3D Grow(in Aabb3D b, double e) => new(b.MinX - e, b.MinY - e, b.MinZ - e, b.MaxX + e, b.MaxY + e, b.MaxZ + e);

    /// <summary>Nearest shapes to a point: search a box that doubles from 1 until it holds a hit within its half width (then the nearest is in it) or reaches the maximum distance.</summary>
    private void ClosestSearch(ref Visitor v)
    {
        double pad = QueryPad();
        var q = v.Q;
        const BodyTypeMask moving = BodyTypeMask.Kinematic | BodyTypeMask.Dynamic;
        double max = q.MaxDistance;
        double r = Math.Min(1.0, max);
        for (;;)
        {
            v.S.Count = 0;
            var sb = new Aabb3D(q.X - r - QueryEpsilon, q.Y - r - QueryEpsilon, q.Z - r - QueryEpsilon, q.X + r + QueryEpsilon, q.Y + r + QueryEpsilon, q.Z + r + QueryEpsilon);
            var mb = Grow(sb, pad);
            Bp.QueryAabb(in sb, ref v, BodyTypeMask.Static);
            Bp.QueryAabb(in mb, ref v, moving);
            bool covered = r >= max || (v.S.Full && v.S.Hits[v.S.Count - 1].Key <= r);
            if (covered) { return; }
            r = Math.Min(max, Math.Max(r * 2.0, v.S.Full ? v.S.Hits[v.S.Count - 1].Key : 0.0));
        }
    }
}
