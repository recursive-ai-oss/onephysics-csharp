using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// A reusable query object (middle tier): a world and a filter that every call uses, plus batch forms that run many queries in parallel on the world's worker threads.
/// Queries only read the world; they never change simulation state, so querying or not does not change a hash. Do not query while the world is stepping or being modified.
/// Single queries may run concurrently from several threads; each thread has its own scratch, so steady-state queries with caller buffers allocate nothing.
/// </summary>
public sealed class PhysicsQuery2D
{
    private readonly PhysicsWorld2D _world;

    /// <summary>Creates a query object for a world with the default filter.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="world"/> is null.</exception>
    public PhysicsQuery2D(PhysicsWorld2D world)
    {
        ArgumentNullException.ThrowIfNull(world);
        _world = world;
    }

    /// <summary>The filter every query of this object uses. Default: all layers and categories, no sensors.</summary>
    public QueryFilter2D Filter { get => _filter; set => _filter = value; }

    private QueryFilter2D _filter;

    /// <summary>Nearest hit of a ray (see <see cref="PhysicsWorld2D.Raycast"/>).</summary>
    public bool Raycast(Vector2D origin, Vector2F direction, out QueryHit2D hit, float maxDistance = float.PositiveInfinity) => _world.Raycast(origin, direction, out hit, maxDistance, in _filter);

    /// <summary>All hits of a ray, nearest first, into a caller buffer.</summary>
    public int RaycastAll(Vector2D origin, Vector2F direction, Span<QueryHit2D> results, float maxDistance = float.PositiveInfinity) => _world.RaycastAll(origin, direction, results, maxDistance, in _filter);

    /// <summary>Nearest hit of a swept shape (see <see cref="PhysicsWorld2D.ShapeCast"/>).</summary>
    public bool ShapeCast(in Shape2D shape, Vector2D position, float angle, Vector2F direction, out QueryHit2D hit, float maxDistance = float.PositiveInfinity) => _world.ShapeCast(in shape, position, angle, direction, out hit, maxDistance, in _filter);

    /// <summary>All shapes a shape overlaps, by slot, into a caller buffer.</summary>
    public int OverlapShapeAll(in Shape2D shape, Vector2D position, float angle, Span<QueryHit2D> results) => _world.OverlapShapeAll(in shape, position, angle, results, in _filter);

    /// <summary>Nearest shape to a point (see <see cref="PhysicsWorld2D.ClosestPoint"/>).</summary>
    public bool ClosestPoint(Vector2D point, out QueryHit2D hit, float maxDistance = 1000f) => _world.ClosestPoint(point, out hit, maxDistance, in _filter);

    // ------------------------------------------------------------------ batches

    private readonly struct BatchBody : IParallelBody
    {
        private readonly PhysicsWorld2D _w;
        private readonly int _kind; // 0 ray, 1 circle cast, 2 circle overlap count
        private readonly RayInput2D[]? _rays;
        private readonly CircleInput2D[]? _circles;
        private readonly QueryHit2D[]? _hits;
        private readonly int[]? _counts;
        private readonly QueryFilter2D _filter;
        private readonly int _n;

        public BatchBody(PhysicsWorld2D w, int kind, RayInput2D[]? rays, CircleInput2D[]? circles, QueryHit2D[]? hits, int[]? counts, in QueryFilter2D filter, int n)
        {
            _w = w; _kind = kind; _rays = rays; _circles = circles; _hits = hits; _counts = counts; _filter = filter; _n = n;
        }

        public void Run(int task)
        {
            int lo = task * 64, hi = Math.Min(lo + 64, _n);
            for (int i = lo; i < hi; i++)
            {
                if (_kind == 0)
                {
                    ref readonly var r = ref _rays![i];
                    _w.Raycast(r.Origin, r.Direction, out _hits![i], r.MaxDistance, in _filter);
                }
                else if (_kind == 1)
                {
                    ref readonly var c = ref _circles![i];
                    _w.CircleCast(c.Center, c.Radius, c.Direction, out _hits![i], c.MaxDistance, in _filter);
                }
                else
                {
                    ref readonly var c = ref _circles![i];
                    _counts![i] = _w.OverlapCircleCount(c.Center, c.Radius, in _filter);
                }
            }
        }
    }

    /// <summary>
    /// Casts every ray and stores its nearest hit in the same index of <paramref name="hits"/> (<see cref="QueryHit2D.IsHit"/> false when it hits nothing). Inputs are validated before any work starts; the results do not depend on the thread count.
    /// Returns the number of rays that hit. The filter's predicate, if any, is called from several threads.
    /// </summary>
    /// <exception cref="ArgumentException"><paramref name="hits"/> is shorter than <paramref name="rays"/>.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A ray is invalid (the message names the index).</exception>
    public int RaycastBatch(RayInput2D[] rays, QueryHit2D[] hits)
    {
        ArgumentNullException.ThrowIfNull(rays);
        ArgumentNullException.ThrowIfNull(hits);
        if (hits.Length < rays.Length) { throw new ArgumentException("hits must have room for one entry per ray.", nameof(hits)); }
        for (int i = 0; i < rays.Length; i++)
        {
            try { PhysicsWorld2D.ValidateRay(rays[i].Origin, rays[i].Direction, rays[i].MaxDistance); }
            catch (ArgumentOutOfRangeException e) { throw new ArgumentOutOfRangeException(nameof(rays), "Ray " + i.ToString(System.Globalization.CultureInfo.InvariantCulture) + ": " + e.Message); }
        }

        Array.Clear(hits, 0, rays.Length);
        Run(new BatchBody(_world, 0, rays, null, hits, null, in _filter, rays.Length), rays.Length);
        int n = 0;
        for (int i = 0; i < rays.Length; i++) { if (hits[i].IsHit) { n++; } }
        return n;
    }

    /// <summary>Casts every circle along its direction and stores the nearest hit per circle; like <see cref="RaycastBatch"/>.</summary>
    /// <exception cref="ArgumentException"><paramref name="hits"/> is shorter than <paramref name="circles"/>.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A circle is invalid.</exception>
    public int CircleCastBatch(CircleInput2D[] circles, QueryHit2D[] hits)
    {
        ArgumentNullException.ThrowIfNull(circles);
        ArgumentNullException.ThrowIfNull(hits);
        if (hits.Length < circles.Length) { throw new ArgumentException("hits must have room for one entry per circle.", nameof(hits)); }
        ValidateCircles(circles, true);
        Array.Clear(hits, 0, circles.Length);
        Run(new BatchBody(_world, 1, null, circles, hits, null, in _filter, circles.Length), circles.Length);
        int n = 0;
        for (int i = 0; i < circles.Length; i++) { if (hits[i].IsHit) { n++; } }
        return n;
    }

    /// <summary>For every circle, the number of shapes it overlaps.</summary>
    /// <exception cref="ArgumentException"><paramref name="counts"/> is shorter than <paramref name="circles"/>.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A circle is invalid.</exception>
    public void OverlapCircleCountBatch(CircleInput2D[] circles, int[] counts)
    {
        ArgumentNullException.ThrowIfNull(circles);
        ArgumentNullException.ThrowIfNull(counts);
        if (counts.Length < circles.Length) { throw new ArgumentException("counts must have room for one entry per circle.", nameof(counts)); }
        ValidateCircles(circles, false);
        Run(new BatchBody(_world, 2, null, circles, null, counts, in _filter, circles.Length), circles.Length);
    }

    private static void ValidateCircles(CircleInput2D[] circles, bool cast)
    {
        for (int i = 0; i < circles.Length; i++)
        {
            try
            {
                PhysicsWorld2D.ValidateCircle(circles[i].Center, circles[i].Radius);
                if (cast) { PhysicsWorld2D.ValidateRay(circles[i].Center, circles[i].Direction, circles[i].MaxDistance); }
            }
            catch (ArgumentOutOfRangeException e) { throw new ArgumentOutOfRangeException(nameof(circles), "Circle " + i.ToString(System.Globalization.CultureInfo.InvariantCulture) + ": " + e.Message); }
        }
    }

    private void Run(BatchBody body, int n) => _world.Core.Executor.For((n + 63) / 64, body);
}
