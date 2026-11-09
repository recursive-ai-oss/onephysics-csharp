using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsQuery2D
{
    /// <summary>The filter the deformable queries of this object use. Default: all kinds, layers and categories.</summary>
        public DeformableFilter2D DeformableFilter { get => _deformableFilter; set => _deformableFilter = value; }

    private DeformableFilter2D _deformableFilter;

    /// <summary>Nearest deformable element a ray hits (see <see cref="PhysicsWorld2D.RaycastDeformables"/>).</summary>
        public bool RaycastDeformables(Vector2D origin, Vector2F direction, out DeformableHit2D hit, float maxDistance = float.PositiveInfinity) => _world.RaycastDeformables(origin, direction, out hit, maxDistance, in _deformableFilter);

    /// <summary>All deformable elements a ray hits, nearest first, into a caller buffer.</summary>
        public int RaycastDeformablesAll(Vector2D origin, Vector2F direction, Span<DeformableHit2D> results, float maxDistance = float.PositiveInfinity) => _world.RaycastDeformablesAll(origin, direction, results, maxDistance, in _deformableFilter);

    /// <summary>Nearest deformable element a swept shape touches.</summary>
        public bool ShapeCastDeformables(in Shape2D shape, Vector2D position, float angleRadians, Vector2F direction, out DeformableHit2D hit, float maxDistance = float.PositiveInfinity) => _world.ShapeCastDeformables(in shape, position, angleRadians, direction, out hit, maxDistance, in _deformableFilter);

    /// <summary>All deformable elements a shape overlaps, into a caller buffer.</summary>
        public int OverlapDeformablesAll(in Shape2D shape, Vector2D position, float angleRadians, Span<DeformableHit2D> results) => _world.OverlapDeformablesAll(in shape, position, angleRadians, results, in _deformableFilter);

    /// <summary>Nearest deformable element to a point.</summary>
        public bool ClosestDeformable(Vector2D point, out DeformableHit2D hit, float maxDistance = 1000f) => _world.ClosestDeformable(point, out hit, maxDistance, in _deformableFilter);

    private readonly struct DeformBatchBody : IParallelBody
    {
        private readonly PhysicsWorld2D _w;
        private readonly int _kind; // 0 ray, 1 circle cast, 2 circle overlap count
        private readonly RayInput2D[]? _rays;
        private readonly CircleInput2D[]? _circles;
        private readonly DeformableHit2D[]? _hits;
        private readonly int[]? _counts;
        private readonly DeformableFilter2D _filter;
        private readonly int _n;

        public DeformBatchBody(PhysicsWorld2D w, int kind, RayInput2D[]? rays, CircleInput2D[]? circles, DeformableHit2D[]? hits, int[]? counts, in DeformableFilter2D filter, int n)
        {
            _w = w; _kind = kind; _rays = rays; _circles = circles; _hits = hits; _counts = counts; _filter = filter; _n = n;
        }

        public void Run(int task)
        {
            int lo = task * 64, hi = Math.Min(lo + 64, _n);
            var scratch = Engine2D.DeformScratch(1);
            for (int i = lo; i < hi; i++)
            {
                if (_kind == 0)
                {
                    ref readonly var r = ref _rays![i];
                    _hits![i] = _w.RunDeformableRay(r.Origin, r.Direction, r.MaxDistance, in _filter, scratch, 1, false) > 0 ? scratch[0] : default;
                }
                else if (_kind == 1)
                {
                    ref readonly var c = ref _circles![i];
                    _hits![i] = _w.RunDeformableCast(c.Radius > 0f ? Shape2D.CreateCircle(default, c.Radius) : Engine2D.s_point, c.Center, 0f, c.Direction, c.MaxDistance, in _filter, scratch, 1, false) > 0 ? scratch[0] : default;
                }
                else
                {
                    ref readonly var c = ref _circles![i];
                    _counts![i] = CountOverlaps(_w, c.Radius, c.Center, in _filter);
                }
            }
        }

        private static int CountOverlaps(PhysicsWorld2D w, float radius, Vector2D center, in DeformableFilter2D filter)
        {
            return w.CountDeformableOverlaps(radius > 0f ? Shape2D.CreateCircle(default, radius) : Engine2D.s_point, center, in filter);
        }
    }

    /// <summary>
    /// Casts every ray at the deformables and stores each ray's nearest hit in the same index of <paramref name="hits"/> (<see cref="DeformableHit2D.IsHit"/> false when it hits nothing). Inputs are validated before any
    /// work starts; the results do not depend on the thread count and equal those of a loop over <see cref="RaycastDeformables"/>. Returns the number of rays that hit.
    /// </summary>
    /// <exception cref="ArgumentException"><paramref name="hits"/> is shorter than <paramref name="rays"/>.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A ray is invalid (the message names the index).</exception>
        public int RaycastDeformablesBatch(RayInput2D[] rays, DeformableHit2D[] hits)
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
        _world.QueryEngine.PrepareQueries();
        _world.Core.Executor.For((rays.Length + 63) / 64, new DeformBatchBody(_world, 0, rays, null, hits, null, in _deformableFilter, rays.Length));
        int n = 0;
        for (int i = 0; i < rays.Length; i++) { if (hits[i].IsHit) { n++; } }
        return n;
    }

    /// <summary>Casts every circle along its direction and stores the nearest deformable element hit per circle; like <see cref="RaycastDeformablesBatch"/>.</summary>
    /// <exception cref="ArgumentException"><paramref name="hits"/> is shorter than <paramref name="circles"/>.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A circle is invalid.</exception>
        public int CircleCastDeformablesBatch(CircleInput2D[] circles, DeformableHit2D[] hits)
    {
        ArgumentNullException.ThrowIfNull(circles);
        ArgumentNullException.ThrowIfNull(hits);
        if (hits.Length < circles.Length) { throw new ArgumentException("hits must have room for one entry per circle.", nameof(hits)); }
        ValidateCircles(circles, true);
        Array.Clear(hits, 0, circles.Length);
        _world.QueryEngine.PrepareQueries();
        _world.Core.Executor.For((circles.Length + 63) / 64, new DeformBatchBody(_world, 1, null, circles, hits, null, in _deformableFilter, circles.Length));
        int n = 0;
        for (int i = 0; i < circles.Length; i++) { if (hits[i].IsHit) { n++; } }
        return n;
    }

    /// <summary>For every circle, the number of deformable elements it overlaps.</summary>
    /// <exception cref="ArgumentException"><paramref name="counts"/> is shorter than <paramref name="circles"/>.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A circle is invalid.</exception>
        public void OverlapCircleDeformablesCountBatch(CircleInput2D[] circles, int[] counts)
    {
        ArgumentNullException.ThrowIfNull(circles);
        ArgumentNullException.ThrowIfNull(counts);
        if (counts.Length < circles.Length) { throw new ArgumentException("counts must have room for one entry per circle.", nameof(counts)); }
        ValidateCircles(circles, false);
        Array.Clear(counts, 0, circles.Length);
        _world.QueryEngine.PrepareQueries();
        _world.Core.Executor.For((circles.Length + 63) / 64, new DeformBatchBody(_world, 2, null, circles, null, counts, in _deformableFilter, circles.Length));
    }
}
