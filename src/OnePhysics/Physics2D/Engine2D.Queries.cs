using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal enum QueryKind : byte { Ray, Cast, Overlap, Closest }

internal enum QueryMode : byte { Closest, Any, All }

/// <summary>One accepted shape during a query. <c>Key</c> orders the results: ray and cast fraction, closest-point distance, overlap centre distance (closest mode) or shape slot (all mode); ties break on the shape slot.</summary>
internal struct RawHit
{
    public double Key;
    public int Slot;
    public double Px, Py;
    public float Nx, Ny, Fraction, Distance;
}

/// <summary>Per-thread scratch for results, so concurrent queries and batches do not share state and steady-state queries do not allocate.</summary>
internal sealed class QueryScratch
{
    [ThreadStatic] private static QueryScratch? t_cached;

    public RawHit[] Hits = new RawHit[8];
    public int Count;
    public int Capacity;
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

    public void Release() => _inUse = false;

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
internal readonly struct PaddedRay<TRay> : IRay<Aabb2D> where TRay : struct, IRay<Aabb2D>
{
    private readonly TRay _ray;
    private readonly double _pad;

    public PaddedRay(TRay ray, double pad) { _ray = ray; _pad = pad; }

    public bool Hit(in Aabb2D box, double maxFraction, out double enter) =>
        _ray.Hit(new Aabb2D(box.MinX - _pad, box.MinY - _pad, box.MaxX + _pad, box.MaxY + _pad), maxFraction, out enter);
}

internal struct QueryParams
{
    public QueryKind Kind;
    public double X, Y;          // ray origin, mover position, or closest-point query point
    public float Tx, Ty;         // ray or cast translation (world frame)
    public Shape2D Mover;        // cast and overlap shape in its own frame
    public Rot2 MoverRot;
    public float MaxDistance;    // closest point
}

internal sealed partial class Engine2D
{
    private const double QueryEpsilon = 1e-4;
    internal static readonly Shape2D s_point = MakePoint();

    /// <summary>Test hook: visit every shape instead of using the trees (same narrow phase and ordering).</summary>
    internal bool BruteQueries;

    private double _queryPad;
    private bool _queryPadValid;

    private static Shape2D MakePoint()
    {
        var s = new Shape2D { _kind = ShapeKind2D.Circle, _count = 1, _radius = 0f };
        return s;
    }

    public void InvalidatePad() => _queryPadValid = false;

    /// <summary>
    /// How far any moving proxy's stored (fat) box may be behind the body's pose: bodies move during a step but their proxies are updated at the start of the next one, so between steps a box can be stale.
    /// Queries grow the kinematic and dynamic trees' boxes by this (computed lazily once per step; it never changes simulation state).
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
            var t = TightBox(b, in Geom[s]);
            double o = Math.Max(Math.Max(fat.MinX - t.MinX, fat.MinY - t.MinY), Math.Max(t.MaxX - fat.MaxX, t.MaxY - fat.MaxY));
            if (o > pad) { pad = o; }
        }

        _queryPad = pad + pad * 1e-9;
        _queryPadValid = true;
        return _queryPad;
    }

    private bool Passes(in QueryFilter2D f, int s)
    {
        if (IsSensor(s) && !f.IncludeSensors) { return false; }
        if (((f.LayerMask >> ShapeLayer[s]) & 1u) == 0) { return false; }
        if ((Category[s] & f.MaskBits) == 0 || (f.CategoryBits & Mask[s]) == 0) { return false; }
        var p = f.Predicate;
        return p == null || p(WrapShape(s));
    }

    private void FrameOf(int b, out double ox, out double oy, out Rot2 q)
    {
        var pos = Core.Bodies.PositionOf(b);
        var r = Core.Bodies.RotationOf(b);
        q = new Rot2(r[0], r[1]);
        var w = q.Rotate(new Vec2(LocalCenter[2 * b], LocalCenter[2 * b + 1]));
        ox = pos[0] - (double)w.X;
        oy = pos[1] - (double)w.Y;
    }

    /// <summary>The narrow phase for one candidate shape. All of it is float arithmetic in the shape's own frame after one double subtraction, like the contact stage.</summary>
    private bool Narrow(in QueryParams q, QueryMode mode, int s, out RawHit h)
    {
        h = default;
        h.Slot = s;
        int b = Core.Shapes.Owner[s];
        FrameOf(b, out double ox, out double oy, out Rot2 bq);
        switch (q.Kind)
        {
            case QueryKind.Ray:
            {
                Vec2 lo = bq.InverseRotate(new Vec2((float)(q.X - ox), (float)(q.Y - oy)));
                Vec2 lt = bq.InverseRotate(new Vec2(q.Tx, q.Ty));
                var o = Geom[s].RayCast(lo, lt, 1f);
                if (!o.Hit) { return false; }
                Vec2 wp = bq.Rotate(o.Point), wn = bq.Rotate(o.Normal);
                h.Key = o.Fraction; h.Fraction = o.Fraction;
                h.Px = ox + (double)wp.X; h.Py = oy + (double)wp.Y; h.Nx = wn.X; h.Ny = wn.Y;
                return true;
            }

            case QueryKind.Cast:
            {
                var xf = Transform2.InvMul(new Transform2(default, bq), new Transform2(new Vec2((float)(q.X - ox), (float)(q.Y - oy)), q.MoverRot));
                Vec2 lt = bq.InverseRotate(new Vec2(q.Tx, q.Ty));
                var o = Distance2D.ShapeCast(in Geom[s], in q.Mover, xf, lt, 1f);
                if (!o.Hit) { return false; }
                Vec2 wp = bq.Rotate(o.Point), wn = bq.Rotate(o.Normal);
                h.Key = o.Fraction; h.Fraction = o.Fraction;
                h.Px = ox + (double)wp.X; h.Py = oy + (double)wp.Y; h.Nx = wn.X; h.Ny = wn.Y;
                return true;
            }

            case QueryKind.Overlap:
            {
                var xf = Transform2.InvMul(new Transform2(default, bq), new Transform2(new Vec2((float)(q.X - ox), (float)(q.Y - oy)), q.MoverRot));
                var cache = new SimplexCache2D();
                var d = Distance2D.Compute(in Geom[s], in q.Mover, xf, ref cache, true);
                if (!(d.Distance <= 0f)) { return false; }
                if (mode == QueryMode.All || mode == QueryMode.Any) { h.Key = s; return true; }
                // Closest overlap: the shape whose surface is nearest the query shape's centre (0 when the centre is inside it).
                Vec2 c = xf.Apply(q.Mover.Centroid);
                var pc = new SimplexCache2D();
                var dc = Distance2D.Compute(in Geom[s], in s_point, new Transform2(c, Rot2.Identity), ref pc, true);
                Vec2 wp = bq.Rotate(dc.PointA);
                h.Key = dc.Distance; h.Distance = dc.Distance;
                h.Px = ox + (double)wp.X; h.Py = oy + (double)wp.Y;
                return true;
            }

            default:
            {
                var xf = new Transform2(bq.InverseRotate(new Vec2((float)(q.X - ox), (float)(q.Y - oy))), Rot2.Identity);
                var cache = new SimplexCache2D();
                var d = Distance2D.Compute(in Geom[s], in s_point, xf, ref cache, true);
                if (!(d.Distance <= q.MaxDistance)) { return false; }
                Vec2 wp = bq.Rotate(d.PointA);
                h.Key = d.Distance; h.Distance = d.Distance;
                h.Px = ox + (double)wp.X; h.Py = oy + (double)wp.Y;
                if (d.Distance == 0f) { h.Px = q.X; h.Py = q.Y; }
                else
                {
                    Vec2 n = bq.Rotate(new Vec2(xf.P.X - d.PointA.X, xf.P.Y - d.PointA.Y)) * (1f / d.Distance);
                    h.Nx = n.X; h.Ny = n.Y;
                }

                return true;
            }
        }
    }

    private struct Visitor : IRayCallback, IOverlapCallback
    {
        public Engine2D E;
        public QueryParams Q;
        public QueryFilter2D Filter;
        public QueryMode Mode;
        public QueryScratch S;

        private bool Visit(int slot)
        {
            if (!E.Passes(in Filter, slot) || !E.Narrow(in Q, Mode, slot, out RawHit h)) { return false; }
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
    internal QueryScratch Run(in QueryParams q, in QueryFilter2D filter, QueryMode mode, int capacity)
    {
        if (mode != QueryMode.All) { capacity = 1; }
        capacity = Math.Max(0, Math.Min(capacity, Core.Shapes.HighWater));
        var scratch = QueryScratch.Acquire(capacity);
        try
        {
            if (capacity == 0) { return scratch; }
            var v = new Visitor { E = this, Q = q, Filter = filter, Mode = mode, S = scratch };
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
            var ray = new Ray2D(q.X, q.Y, q.Tx, q.Ty);
            double f = Bp.RayCast(new PaddedRay<Ray2D>(ray, QueryEpsilon), 1.0, ref v, BodyTypeMask.Static);
            if (f > 0.0 || !(v.Mode == QueryMode.Any && v.S.Count > 0)) { Bp.RayCast(new PaddedRay<Ray2D>(ray, pad + QueryEpsilon), f, ref v, moving); }
            return;
        }

        var local = q.Kind == QueryKind.Cast || q.Kind == QueryKind.Overlap ? q.Mover.ComputeAabb(new Transform2(default, q.MoverRot)) : default;
        var box = new Aabb2D(q.X + (double)local.Lower.X, q.Y + (double)local.Lower.Y, q.X + (double)local.Upper.X, q.Y + (double)local.Upper.Y);
        if (q.Kind == QueryKind.Cast)
        {
            var sweep = new BoxCast2D(box, q.Tx, q.Ty);
            double f = Bp.RayCast(new PaddedRay<BoxCast2D>(sweep, QueryEpsilon), 1.0, ref v, BodyTypeMask.Static);
            if (f > 0.0 || !(v.Mode == QueryMode.Any && v.S.Count > 0)) { Bp.RayCast(new PaddedRay<BoxCast2D>(sweep, pad + QueryEpsilon), f, ref v, moving); }
            return;
        }

        var sb = new Aabb2D(box.MinX - QueryEpsilon, box.MinY - QueryEpsilon, box.MaxX + QueryEpsilon, box.MaxY + QueryEpsilon);
        Bp.QueryAabb(in sb, ref v, BodyTypeMask.Static);
        if (v.Mode == QueryMode.Any && v.S.Count > 0) { return; }
        var mb = new Aabb2D(box.MinX - pad - QueryEpsilon, box.MinY - pad - QueryEpsilon, box.MaxX + pad + QueryEpsilon, box.MaxY + pad + QueryEpsilon);
        Bp.QueryAabb(in mb, ref v, moving);
    }

    /// <summary>
    /// Nearest shapes to a point: search a box that doubles from 1 until it holds a hit within its half width (then the nearest is in it) or reaches the maximum distance.
    /// </summary>
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
            var sb = new Aabb2D(q.X - r - QueryEpsilon, q.Y - r - QueryEpsilon, q.X + r + QueryEpsilon, q.Y + r + QueryEpsilon);
            var mb = new Aabb2D(sb.MinX - pad, sb.MinY - pad, sb.MaxX + pad, sb.MaxY + pad);
            Bp.QueryAabb(in sb, ref v, BodyTypeMask.Static);
            Bp.QueryAabb(in mb, ref v, moving);
            bool covered = r >= max || (v.S.Full && v.S.Hits[v.S.Count - 1].Key <= r);
            if (covered) { return; }
            r = Math.Min(max, Math.Max(r * 2.0, v.S.Full ? v.S.Hits[v.S.Count - 1].Key : 0.0));
        }
    }
}
