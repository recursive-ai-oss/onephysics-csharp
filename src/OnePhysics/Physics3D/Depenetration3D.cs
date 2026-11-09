using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>A tested partial or complete correction from iterative depenetration. Not a globally minimum translation. The world is unchanged.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct DepenetrationResult3D
{
    internal DepenetrationResult3D(Vector3D translation, float remainingDepth, int iterations, DepenetrationStatus status)
    { Translation = translation; RemainingDepth = remainingDepth; Iterations = iterations; Status = status; }

    /// <summary>Translation in metres to add to the original query position, including on failure. Its length cannot exceed the requested path budget.</summary>
    public Vector3D Translation { get; }
    /// <summary>Maximum accepted obstacle penetration at the returned pose, in metres; zero when no overlap remains.</summary>
    public float RemainingDepth { get; }
    /// <summary>Number of corrections applied (0 means the original pose was returned).</summary>
    public int Iterations { get; }
    /// <summary>Why the solver stopped. Cleared alone certifies the residual bound.</summary>
    public DepenetrationStatus Status { get; }
    /// <summary>Whether every accepted obstacle's remaining penetration is at most the requested tolerance.</summary>
    public bool Converged => Status == DepenetrationStatus.Cleared;
}

public sealed partial class PhysicsWorld3D
{
    /// <summary>Deterministically depenetrates a query from nonconvex unions and world obstacles without moving any body. Repeatedly corrects the deepest convex-leaf contact (ties by target slot/leaf), then rechecks the whole selection. This is NOT a globally minimum translation. Success certifies residual penetration ≤ tolerance; limits return Converged=false and a tested partial correction. Default 32 corrections, 0.0001 metre tolerance, 1000 metre cumulative path budget. Bounds: iterations 1..1024, finite tolerance (0,1], finite distance (0,MaxQueryDistance]. Empty geometry succeeds with zero correction. Impossible enclosures can exhaust the iteration limit.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The pose, limits or tolerance is invalid.</exception>
    /// <exception cref="ArgumentException">The query shape is invalid or both query and obstacle are infinite half-spaces.</exception>
    public DepenetrationResult3D Depenetrate(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, int maxIterations = 32, float tolerance = 0.0001f, float maxDistance = 1000f, in QueryFilter3D filter = default)
    {
        var q = OverlapParams(shape, position, rotation); q.Kind = QueryKind.Penetration;
        return DepenetratePrepared(q, maxIterations, tolerance, maxDistance, in filter);
    }

    internal DepenetrationResult3D DepenetratePrepared(QueryParams query, int maxIterations, float tolerance, float maxDistance, in QueryFilter3D filter)
    {
        if (maxIterations < 1 || maxIterations > 1024) { throw new ArgumentOutOfRangeException(nameof(maxIterations), maxIterations, "Correction count must be within 1..1024."); }
        if (!(tolerance > 0 && tolerance <= 1) || !float.IsFinite(tolerance)) { throw new ArgumentOutOfRangeException(nameof(tolerance), tolerance, "Tolerance must be finite in (0,1] metres."); }
        if (!(maxDistance > 0 && maxDistance <= MaxQueryDistance) || !float.IsFinite(maxDistance)) { throw new ArgumentOutOfRangeException(nameof(maxDistance), maxDistance, "Correction distance budget must be positive and at most MaxQueryDistance metres."); }
        double x = 0, y = 0, z = 0;
        double originX = query.X; double originY = query.Y; double originZ = query.Z;
        double remaining = maxDistance;
        for (int iteration = 0; ; iteration++)
        {
            float depth = QueryOne(query, 0f, in filter, QueryMode.Closest, out var hit) ? hit.Distance : 0f;
            var correction = new Vector3D(x, y, z);
            if (depth <= tolerance) { return new(correction, depth, iteration, DepenetrationStatus.Cleared); }
            if (iteration == maxIterations) { return new(correction, depth, iteration, DepenetrationStatus.IterationLimit); }
            if (remaining <= 0) { return new(correction, depth, iteration, DepenetrationStatus.DistanceLimit); }
            double norm = double.Sqrt((double)hit.Normal.X * hit.Normal.X + (double)hit.Normal.Y * hit.Normal.Y + (double)hit.Normal.Z * hit.Normal.Z);
            if (!(norm > 0) || !double.IsFinite(norm)) { return new(correction, depth, iteration, DepenetrationStatus.DegenerateContact); }
            double length = Math.Min((double)depth + tolerance * 0.5, remaining);
            double dx = hit.Normal.X / norm * length; double dy = hit.Normal.Y / norm * length; double dz = hit.Normal.Z / norm * length;
            double nextX = x + dx; double nextY = y + dy; double nextZ = z + dz;
            if (Math.Abs(originX + nextX) > 4194304 || Math.Abs(originY + nextY) > 4194304 || Math.Abs(originZ + nextZ) > 4194304) { return new(correction, depth, iteration, DepenetrationStatus.CoordinateLimit); }
            // Certify exactly the pose the caller obtains from original + Translation,
            // rather than a differently parenthesized sum of successive positions.
            x = nextX; query.X = originX + x; y = nextY; query.Y = originY + y; z = nextZ; query.Z = originZ + z;
            remaining = Math.Max(0, remaining - length);
        }
    }
}

public sealed partial class PhysicsQuery3D
{
    /// <summary>Bounded iterative depenetration with this object's filter. See the world method for residual certification, limits and non-minimal correction semantics.</summary>
    public DepenetrationResult3D Depenetrate(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, int maxIterations = 32, float tolerance = 0.0001f, float maxDistance = 1000f) => _world.Depenetrate(shape, position, rotation, maxIterations, tolerance, maxDistance, in _filter);
}

public readonly partial struct Body
{
    /// <summary>Iteratively depenetrates a query shape from only this body, honoring filters. Same units, limits, residual certificate and failure statuses as the world method. A destroyed target throws StaleHandleException.</summary>
    public DepenetrationResult3D Depenetrate(in ConvexShape3D mover, Vector3D position, QuaternionF rotation, int maxIterations = 32, float tolerance = 0.0001f, float maxDistance = 1000f, in QueryFilter3D filter = default)
    {
        var (world, slot) = Resolve();
        return world.SelectedDepenetrate(mover, position, rotation, slot, true, maxIterations, tolerance, maxDistance, in filter);
    }
}

public readonly partial struct Shape
{
    /// <summary>Iteratively depenetrates a query shape from only this shape, honoring filters. Same units, limits, residual certificate and failure statuses as the world method. A destroyed target throws StaleHandleException.</summary>
    public DepenetrationResult3D Depenetrate(in ConvexShape3D mover, Vector3D position, QuaternionF rotation, int maxIterations = 32, float tolerance = 0.0001f, float maxDistance = 1000f, in QueryFilter3D filter = default)
    {
        int slot = Resolve(); var world = _world!;
        return world.SelectedDepenetrate(mover, position, rotation, slot, false, maxIterations, tolerance, maxDistance, in filter);
    }
}

public sealed partial class PhysicsWorld3D
{
    internal DepenetrationResult3D SelectedDepenetrate(in ConvexShape3D shape, Vector3D position, QuaternionF rotation, int slot, bool body, int maxIterations, float tolerance, float maxDistance, in QueryFilter3D filter)
    {
        var q = OverlapParams(shape, position, rotation); q.Kind = QueryKind.Penetration;
        if (body) { q.OnlyBodyPlusOne = slot + 1; } else { q.OnlyShapePlusOne = slot + 1; }
        return DepenetratePrepared(q, maxIterations, tolerance, maxDistance, in filter);
    }
}
