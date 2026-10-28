using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// A reusable query object (middle tier): a world and a filter that every call uses, plus batch forms that run many queries in parallel on the world's worker threads.
/// Queries only read the world; they never change simulation state, so querying or not does not change a hash. Do not query while the world is stepping or being modified.
/// Single queries may run concurrently from several threads; each thread has its own scratch, so steady-state queries with caller buffers allocate nothing.
/// </summary>
public sealed class PhysicsQuery3D
{
    private readonly PhysicsWorld3D _world;

    /// <summary>Creates a query object for a world with the default filter.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="world"/> is null.</exception>
    public PhysicsQuery3D(PhysicsWorld3D world)
    {
        ArgumentNullException.ThrowIfNull(world);
        _world = world;
    }

    /// <summary>The filter every query of this object uses. Default: all layers and categories, no sensors.</summary>
    public QueryFilter3D Filter { get => _filter; set => _filter = value; }

    private QueryFilter3D _filter;

    /// <summary>Nearest hit of a ray (see <see cref="PhysicsWorld3D.Raycast"/>).</summary>
    public bool Raycast(Vector3D origin, Vector3F direction, out QueryHit3D hit, float maxDistance = float.PositiveInfinity) => _world.Raycast(origin, direction, out hit, maxDistance, in _filter);

    /// <summary>All hits of a ray, nearest first, into a caller buffer.</summary>
    public int RaycastAll(Vector3D origin, Vector3F direction, Span<QueryHit3D> results, float maxDistance = float.PositiveInfinity) => _world.RaycastAll(origin, direction, results, maxDistance, in _filter);

    /// <summary>Nearest hit of a swept shape (see <see cref="PhysicsWorld3D.ShapeCast"/>).</summary>
    public bool ShapeCast(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, Vector3F direction, out QueryHit3D hit, float maxDistance = float.PositiveInfinity) => _world.ShapeCast(in shape, position, rotation, direction, out hit, maxDistance, in _filter);

    /// <summary>All shapes a shape overlaps, by slot, into a caller buffer.</summary>
    public int OverlapShapeAll(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, Span<QueryHit3D> results) => _world.OverlapShapeAll(in shape, position, rotation, results, in _filter);

    /// <summary>Nearest shape to a point (see <see cref="PhysicsWorld3D.ClosestPoint"/>).</summary>
    public bool ClosestPoint(Vector3D point, out QueryHit3D hit, float maxDistance = 1000f) => _world.ClosestPoint(point, out hit, maxDistance, in _filter);

    // ------------------------------------------------------------------ batches

    private readonly struct BatchBody : IParallelBody
    {
        private readonly PhysicsWorld3D _w;
        private readonly int _kind; // 0 ray, 1 sphere cast, 2 sphere overlap count
        private readonly RayInput3D[]? _rays;
        private readonly SphereInput3D[]? _spheres;
        private readonly QueryHit3D[]? _hits;
        private readonly int[]? _counts;
        private readonly QueryFilter3D _filter;
        private readonly int _n;

        public BatchBody(PhysicsWorld3D w, int kind, RayInput3D[]? rays, SphereInput3D[]? spheres, QueryHit3D[]? hits, int[]? counts, in QueryFilter3D filter, int n)
        {
            _w = w; _kind = kind; _rays = rays; _spheres = spheres; _hits = hits; _counts = counts; _filter = filter; _n = n;
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
                    ref readonly var c = ref _spheres![i];
                    _w.SphereCast(c.Center, c.Radius, c.Direction, out _hits![i], c.MaxDistance, in _filter);
                }
                else
                {
                    ref readonly var c = ref _spheres![i];
                    _counts![i] = _w.OverlapSphereCount(c.Center, c.Radius, in _filter);
                }
            }
        }
    }

    /// <summary>
    /// Casts every ray and stores its nearest hit in the same index of <paramref name="hits"/> (<see cref="QueryHit3D.IsHit"/> false when it hits nothing). Inputs are validated before any work starts; the results do not depend on the thread count.
    /// Returns the number of rays that hit. The filter's predicate, if any, is called from several threads.
    /// </summary>
    /// <exception cref="ArgumentException"><paramref name="hits"/> is shorter than <paramref name="rays"/>.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A ray is invalid (the message names the index).</exception>
    public int RaycastBatch(RayInput3D[] rays, QueryHit3D[] hits)
    {
        ArgumentNullException.ThrowIfNull(rays);
        ArgumentNullException.ThrowIfNull(hits);
        if (hits.Length < rays.Length) { throw new ArgumentException("hits must have room for one entry per ray.", nameof(hits)); }
        for (int i = 0; i < rays.Length; i++)
        {
            try { PhysicsWorld3D.ValidateRay(rays[i].Origin, rays[i].Direction, rays[i].MaxDistance); }
            catch (ArgumentOutOfRangeException e) { throw new ArgumentOutOfRangeException(nameof(rays), "Ray " + i.ToString(System.Globalization.CultureInfo.InvariantCulture) + ": " + e.Message); }
        }

        Array.Clear(hits, 0, rays.Length);
        Run(new BatchBody(_world, 0, rays, null, hits, null, in _filter, rays.Length), rays.Length);
        int n = 0;
        for (int i = 0; i < rays.Length; i++) { if (hits[i].IsHit) { n++; } }
        return n;
    }

    /// <summary>Casts every sphere along its direction and stores the nearest hit per sphere; like <see cref="RaycastBatch"/>.</summary>
    /// <exception cref="ArgumentException"><paramref name="hits"/> is shorter than <paramref name="spheres"/>.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A sphere is invalid.</exception>
    public int SphereCastBatch(SphereInput3D[] spheres, QueryHit3D[] hits)
    {
        ArgumentNullException.ThrowIfNull(spheres);
        ArgumentNullException.ThrowIfNull(hits);
        if (hits.Length < spheres.Length) { throw new ArgumentException("hits must have room for one entry per sphere.", nameof(hits)); }
        ValidateSpheres(spheres, true);
        Array.Clear(hits, 0, spheres.Length);
        Run(new BatchBody(_world, 1, null, spheres, hits, null, in _filter, spheres.Length), spheres.Length);
        int n = 0;
        for (int i = 0; i < spheres.Length; i++) { if (hits[i].IsHit) { n++; } }
        return n;
    }

    /// <summary>For every sphere, the number of shapes it overlaps.</summary>
    /// <exception cref="ArgumentException"><paramref name="counts"/> is shorter than <paramref name="spheres"/>.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A sphere is invalid.</exception>
    public void OverlapSphereCountBatch(SphereInput3D[] spheres, int[] counts)
    {
        ArgumentNullException.ThrowIfNull(spheres);
        ArgumentNullException.ThrowIfNull(counts);
        if (counts.Length < spheres.Length) { throw new ArgumentException("counts must have room for one entry per sphere.", nameof(counts)); }
        ValidateSpheres(spheres, false);
        Run(new BatchBody(_world, 2, null, spheres, null, counts, in _filter, spheres.Length), spheres.Length);
    }

    private static void ValidateSpheres(SphereInput3D[] spheres, bool cast)
    {
        for (int i = 0; i < spheres.Length; i++)
        {
            try
            {
                PhysicsWorld3D.ValidateSphere(spheres[i].Center, spheres[i].Radius);
                if (cast) { PhysicsWorld3D.ValidateRay(spheres[i].Center, spheres[i].Direction, spheres[i].MaxDistance); }
            }
            catch (ArgumentOutOfRangeException e) { throw new ArgumentOutOfRangeException(nameof(spheres), "Sphere " + i.ToString(System.Globalization.CultureInfo.InvariantCulture) + ": " + e.Message); }
        }
    }

    private void Run(BatchBody body, int n) => _world.Core.Executor.For((n + 63) / 64, body);
}
