using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsQuery3D
{
    /// <summary>The filter the deformable queries of this object use. Default: all kinds, layers and categories.</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public DeformableFilter3D DeformableFilter { get => _deformableFilter; set => _deformableFilter = value; }

    private DeformableFilter3D _deformableFilter;

    /// <summary>Nearest deformable element a ray hits (see <see cref="PhysicsWorld3D.RaycastDeformables"/>).</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public bool RaycastDeformables(Vector3D origin, Vector3F direction, out DeformableHit3D hit, float maxDistance = float.PositiveInfinity) => _world.RaycastDeformables(origin, direction, out hit, maxDistance, in _deformableFilter);

    /// <summary>All deformable elements a ray hits, nearest first, into a caller buffer.</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public int RaycastDeformablesAll(Vector3D origin, Vector3F direction, Span<DeformableHit3D> results, float maxDistance = float.PositiveInfinity) => _world.RaycastDeformablesAll(origin, direction, results, maxDistance, in _deformableFilter);

    /// <summary>Nearest deformable element a swept shape touches.</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public bool ShapeCastDeformables(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, Vector3F direction, out DeformableHit3D hit, float maxDistance = float.PositiveInfinity) => _world.ShapeCastDeformables(in shape, position, rotation, direction, out hit, maxDistance, in _deformableFilter);

    /// <summary>All deformable elements a shape overlaps, into a caller buffer.</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public int OverlapDeformablesAll(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, Span<DeformableHit3D> results) => _world.OverlapDeformablesAll(in shape, position, rotation, results, in _deformableFilter);

    /// <summary>Nearest deformable element to a point.</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public bool ClosestDeformable(Vector3D point, out DeformableHit3D hit, float maxDistance = 1000f) => _world.ClosestDeformable(point, out hit, maxDistance, in _deformableFilter);

    private readonly struct DeformBatchBody : IParallelBody
    {
        private readonly PhysicsWorld3D _w;
        private readonly int _kind; // 0 ray, 1 sphere cast, 2 sphere overlap count
        private readonly RayInput3D[]? _rays;
        private readonly SphereInput3D[]? _spheres;
        private readonly DeformableHit3D[]? _hits;
        private readonly int[]? _counts;
        private readonly DeformableFilter3D _filter;
        private readonly int _n;

        public DeformBatchBody(PhysicsWorld3D w, int kind, RayInput3D[]? rays, SphereInput3D[]? spheres, DeformableHit3D[]? hits, int[]? counts, in DeformableFilter3D filter, int n)
        {
            _w = w; _kind = kind; _rays = rays; _spheres = spheres; _hits = hits; _counts = counts; _filter = filter; _n = n;
        }

        public void Run(int task)
        {
            int lo = task * 64, hi = Math.Min(lo + 64, _n);
            var scratch = Engine3D.DeformScratch(1);
            for (int i = lo; i < hi; i++)
            {
                if (_kind == 0)
                {
                    ref readonly var r = ref _rays![i];
                    _hits![i] = _w.RunDeformableRay(r.Origin, r.Direction, r.MaxDistance, in _filter, scratch, 1, false) > 0 ? scratch[0] : default;
                }
                else if (_kind == 1)
                {
                    ref readonly var c = ref _spheres![i];
                    _hits![i] = _w.RunDeformableCast(c.Radius > 0f ? ConvexShape3D.Sphere(c.Radius) : ConvexShape3D.Point, c.Center, default, c.Direction, c.MaxDistance, in _filter, scratch, 1, false) > 0 ? scratch[0] : default;
                }
                else
                {
                    ref readonly var c = ref _spheres![i];
                    _counts![i] = CountOverlaps(_w, c.Radius, c.Center, in _filter);
                }
            }
        }

        private static int CountOverlaps(PhysicsWorld3D w, float radius, Vector3D center, in DeformableFilter3D filter)
        {
            return w.CountDeformableOverlaps(radius > 0f ? ConvexShape3D.Sphere(radius) : ConvexShape3D.Point, center, in filter);
        }
    }

    /// <summary>
    /// Casts every ray at the deformables and stores each ray's nearest hit in the same index of <paramref name="hits"/> (<see cref="DeformableHit3D.IsHit"/> false when it hits nothing). Inputs are validated before any
    /// work starts; the results do not depend on the thread count and equal those of a loop over <see cref="RaycastDeformables"/>. Returns the number of rays that hit.
    /// </summary>
    /// <exception cref="ArgumentException"><paramref name="hits"/> is shorter than <paramref name="rays"/>.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A ray is invalid (the message names the index).</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public int RaycastDeformablesBatch(RayInput3D[] rays, DeformableHit3D[] hits)
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
        _world.Core.Executor.For((rays.Length + 63) / 64, new DeformBatchBody(_world, 0, rays, null, hits, null, in _deformableFilter, rays.Length));
        int n = 0;
        for (int i = 0; i < rays.Length; i++) { if (hits[i].IsHit) { n++; } }
        return n;
    }

    /// <summary>Casts every sphere along its direction and stores the nearest deformable element hit per sphere; like <see cref="RaycastDeformablesBatch"/>.</summary>
    /// <exception cref="ArgumentException"><paramref name="hits"/> is shorter than <paramref name="spheres"/>.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A sphere is invalid.</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public int SphereCastDeformablesBatch(SphereInput3D[] spheres, DeformableHit3D[] hits)
    {
        ArgumentNullException.ThrowIfNull(spheres);
        ArgumentNullException.ThrowIfNull(hits);
        if (hits.Length < spheres.Length) { throw new ArgumentException("hits must have room for one entry per sphere.", nameof(hits)); }
        ValidateSpheres(spheres, true);
        Array.Clear(hits, 0, spheres.Length);
        _world.Core.Executor.For((spheres.Length + 63) / 64, new DeformBatchBody(_world, 1, null, spheres, hits, null, in _deformableFilter, spheres.Length));
        int n = 0;
        for (int i = 0; i < spheres.Length; i++) { if (hits[i].IsHit) { n++; } }
        return n;
    }

    /// <summary>For every sphere, the number of deformable elements it overlaps.</summary>
    /// <exception cref="ArgumentException"><paramref name="counts"/> is shorter than <paramref name="spheres"/>.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A sphere is invalid.</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public void OverlapSphereDeformablesCountBatch(SphereInput3D[] spheres, int[] counts)
    {
        ArgumentNullException.ThrowIfNull(spheres);
        ArgumentNullException.ThrowIfNull(counts);
        if (counts.Length < spheres.Length) { throw new ArgumentException("counts must have room for one entry per sphere.", nameof(counts)); }
        ValidateSpheres(spheres, false);
        Array.Clear(counts, 0, spheres.Length);
        _world.Core.Executor.For((spheres.Length + 63) / 64, new DeformBatchBody(_world, 2, null, spheres, null, counts, in _deformableFilter, spheres.Length));
    }
}
