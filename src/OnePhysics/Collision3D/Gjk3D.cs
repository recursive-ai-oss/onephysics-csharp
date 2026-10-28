using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Collision3D;

/// <summary>Closest points between two convex shapes (see <see cref="Gjk3D.Distance"/>). All vectors are in the frame of shape A.</summary>
public readonly struct ConvexDistance3D
{
    internal ConvexDistance3D(float distance, bool overlapping, Vector3F normal, Vector3F pointA, Vector3F pointB, int iterations)
    {
        Distance = distance;
        Overlapping = overlapping;
        Normal = normal;
        PointA = pointA;
        PointB = pointB;
        Iterations = iterations;
    }

    /// <summary>Distance between the surfaces; 0 when they overlap or touch.</summary>
    public float Distance { get; }

    /// <summary>True when the shapes (including their convex radii) overlap; the points and normal then describe the cores' closest approach, or are zero when the cores themselves overlap. Use <see cref="Epa3D"/> for the penetration.</summary>
    public bool Overlapping { get; }

    /// <summary>Unit direction from A towards B along the closest points; zero when the cores overlap.</summary>
    public Vector3F Normal { get; }

    /// <summary>The closest point on the surface of A.</summary>
    public Vector3F PointA { get; }

    /// <summary>The closest point on the surface of B.</summary>
    public Vector3F PointB { get; }

    /// <summary>GJK iterations used.</summary>
    public int Iterations { get; }
}

/// <summary>Result of a shape cast or ray cast. Vectors are in the frame of shape A (shape casts) or of the ray's frame (ray casts).</summary>
public readonly struct ConvexCastHit3D
{
    internal ConvexCastHit3D(float fraction, Vector3F normal, Vector3F pointA, Vector3F pointB, bool startsOverlapping)
    {
        Fraction = fraction;
        Normal = normal;
        PointA = pointA;
        PointB = pointB;
        StartsOverlapping = startsOverlapping;
    }

    /// <summary>How far along the motion the first contact is, in [0, 1] times the motion vector.</summary>
    public float Fraction { get; }

    /// <summary>Unit contact normal pointing from A towards B (shape casts), or the outward surface normal of the shape that was hit (ray casts). Zero when the cast starts with the cores overlapping.</summary>
    public Vector3F Normal { get; }

    /// <summary>The contact point on the moving shape A at the time of impact.</summary>
    public Vector3F PointA { get; }

    /// <summary>The contact point on shape B.</summary>
    public Vector3F PointB { get; }

    /// <summary>True when the shapes already overlap at the start (the hit is at fraction 0).</summary>
    public bool StartsOverlapping { get; }
}

/// <summary>
/// The Minkowski difference A - B of two convex shapes, evaluated through their support mappings, with B placed in A's frame.
/// </summary>
internal readonly struct ConvexPair
{
    public readonly ConvexShape3D A;
    public readonly ConvexShape3D B;
    public readonly Matrix3x3F R;
    public readonly Matrix3x3F Rt;
    public readonly Vector3F T;

    public ConvexPair(in ConvexShape3D a, in ConvexShape3D b, in RigidTransformF bInA)
    {
        A = a;
        B = b;
        R = bInA.Rotation.ToMatrix();
        Rt = R.Transpose();
        T = bInA.Translation;
    }

    public float Radii => A.ConvexRadius + B.ConvexRadius;

    /// <summary>Support of the core difference in direction u: s_A(u) - s_B(-u).</summary>
    public void Support(Vector3F u, out Vector3F p, out Vector3F a, out Vector3F b)
    {
        a = A.CoreSupport(u);
        b = (R * B.CoreSupport(Rt * -u)) + T;
        p = a - b;
    }
}

/// <summary>
/// GJK on pairs of convex shapes: distance and closest points (Gilbert, Johnson, Keerthi 1988; van den Bergen 1999, "A fast and robust GJK implementation"),
/// and ray and shape casts by conservative advancement along the Minkowski difference (van den Bergen 2004, "Ray casting against general convex objects").
/// Shapes are float32 in their own local frames; B's placement relative to A is a rigid transform. Rotations must be unit quaternions.
/// Results are deterministic: the same inputs give the same bits on every platform.
/// </summary>
public static class Gjk3D
{
    // A step is accepted as converged when v.v - v.w <= max(RelTol * v.v, AbsTol * |v| * scale): the distance error is then about AbsTol * scale.
    private const float RelTol = 2e-6f;
    private const float AbsTol = 4e-7f;
    private const float OverlapTol = 2e-5f;
    private const int MaxIterations = 48;

    // Distance from the ray's origin to the shape beyond which a ray cast clips against the shape's bounds first.
    private const float FarRayDistance = 1024f;

    /// <summary>Distance and closest points between two convex shapes.</summary>
    /// <param name="a">Shape A, in its own frame.</param>
    /// <param name="b">Shape B, in its own frame.</param>
    /// <param name="bInA">Placement of B's frame in A's frame.</param>
    /// <returns>The distance between the surfaces and the closest points in A's frame. Accuracy is about 1e-6 of the shapes' size plus their separation.</returns>
    public static ConvexDistance3D Distance(in ConvexShape3D a, in ConvexShape3D b, in RigidTransformF bInA)
    {
        var pair = new ConvexPair(a, b, bInA);
        Simplex s = default;
        Run(pair, ref s, out Vector3F v, out bool coresOverlap, out int iterations, out _);
        float radii = pair.Radii;
        if (coresOverlap)
        {
            return new ConvexDistance3D(0f, true, default, default, default, iterations);
        }

        float coreDist = ConvexShape3D.Sqrt(v.LengthSquared());
        Vector3F n = -v / coreDist; // v = pointA - pointB, so A -> B is -v
        Vector3F pa = s.WitnessA(), pb = s.WitnessB();
        if (coreDist > radii)
        {
            return new ConvexDistance3D(coreDist - radii, false, n, pa + (n * pair.A.ConvexRadius), pb - (n * pair.B.ConvexRadius), iterations);
        }

        return new ConvexDistance3D(0f, true, n, pa + (n * pair.A.ConvexRadius), pb - (n * pair.B.ConvexRadius), iterations);
    }

    /// <summary>
    /// Runs GJK on the cores. On return the simplex holds the closest features. <paramref name="coresOverlap"/> is true when the cores intersect (or touch within
    /// round-off); then the simplex contains the origin (possibly as a lower-dimensional simplex) and <paramref name="v"/> is near zero.
    /// </summary>
    internal static void Run(in ConvexPair pair, ref Simplex s, out Vector3F v, out bool coresOverlap, out int iterations, out float scale2)
    {
        Vector3F first = pair.T.LengthSquared() > 0f ? pair.T : Vector3F.UnitX;
        pair.Support(first, out Vector3F p, out Vector3F pa, out Vector3F pb);
        s.Count = 0;
        s.Add(p, pa, pb);
        s.L[0] = 1f;
        v = p;
        float maxW2 = p.LengthSquared();
        coresOverlap = false;
        iterations = 0;
        float vv = v.LengthSquared();
        while (iterations < MaxIterations)
        {
            if (vv <= OverlapTol * OverlapTol * maxW2 || vv == 0f)
            {
                coresOverlap = true;
                break;
            }

            iterations++;
            pair.Support(-v, out p, out pa, out pb);
            float w2 = p.LengthSquared();
            if (w2 > maxW2)
            {
                maxW2 = w2;
            }

            float gap = vv - Vector3F.Dot(v, p);
            float lenV = ConvexShape3D.Sqrt(vv);
            float tol = Math.Max(RelTol * vv, AbsTol * lenV * ConvexShape3D.Sqrt(maxW2));
            if (gap <= tol)
            {
                break;
            }

            bool dup = false;
            for (int i = 0; i < s.Count; i++)
            {
                if (s.P[i] == p)
                {
                    dup = true;
                }
            }

            if (dup)
            {
                break;
            }

            Simplex saved = s;
            s.Add(p, pa, pb);
            bool inside = s.Solve(out Vector3F nv);
            if (inside && Vector3F.Dot(v, p) > 0.5f * vv)
            {
                // Everything in the simplex lies on the far side of the plane through the previous closest point, as p does, so the origin cannot be inside: the tetrahedron is a numerical sliver.
                s = saved;
                break;
            }

            if (inside)
            {
                coresOverlap = true;
                v = default;
                break;
            }

            float nvv = nv.LengthSquared();
            if (!(nvv < vv))
            {
                s = saved;
                break;
            }

            v = nv;
            vv = nvv;
        }

        if (iterations >= MaxIterations && !coresOverlap && vv <= OverlapTol * OverlapTol * maxW2)
        {
            coresOverlap = true;
        }

        scale2 = maxW2;
    }

    /// <summary>
    /// Sweeps shape A along a motion and finds the first contact with shape B (shape cast, B stays still).
    /// </summary>
    /// <param name="a">The moving shape, in its own frame.</param>
    /// <param name="b">The stationary shape, in its own frame.</param>
    /// <param name="bInA">Placement of B's frame in A's frame at the start.</param>
    /// <param name="motionOfA">How far A moves, in A's frame; the cast covers fractions 0 to 1 of it.</param>
    /// <param name="hit">The first contact; unset when false is returned.</param>
    /// <returns>True when the surfaces touch within the motion. Shapes that already overlap hit at fraction 0.</returns>
    public static bool ShapeCast(in ConvexShape3D a, in ConvexShape3D b, in RigidTransformF bInA, Vector3F motionOfA, out ConvexCastHit3D hit)
    {
        hit = default;
        if (!float.IsFinite(motionOfA.X) || !float.IsFinite(motionOfA.Y) || !float.IsFinite(motionOfA.Z))
        {
            throw new ArgumentOutOfRangeException(nameof(motionOfA), motionOfA, $"motionOfA is {motionOfA}; finite components are expected.");
        }

        var pair = new ConvexPair(a, b, bInA);
        float radii = pair.Radii;
        // A moving by m overlaps B when -m*t lies in D = A - B: a ray from the origin along r = -m against D inflated by the radii.
        Vector3F r = -motionOfA;
        Vector3F first = pair.T.LengthSquared() > 0f ? pair.T : Vector3F.UnitX;
        pair.Support(first, out Vector3F p0, out Vector3F a0, out Vector3F b0);
        Simplex s = default;
        s.Add(p0, a0, b0);
        Vector3F x = default;
        Vector3F v = x - p0;
        float lambda = 0f;
        Vector3F normal = default;
        bool moved = false;
        float maxW2 = p0.LengthSquared();
        bool converged = false;
        for (int iter = 0; iter < 128; iter++)
        {
            float vv = v.LengthSquared();
            float scale = ConvexShape3D.Sqrt(maxW2);
            float tol = (AbsTol * 4f * scale) + 1e-30f;
            float lim = radii + tol;
            if (vv <= lim * lim)
            {
                converged = true;
                break;
            }

            float nv = ConvexShape3D.Sqrt(vv);
            Vector3F nhat = v / nv;
            pair.Support(v, out Vector3F p, out Vector3F pa, out Vector3F pb);
            float w2 = p.LengthSquared();
            if (w2 > maxW2)
            {
                maxW2 = w2;
            }

            float sep = Vector3F.Dot(nhat, x - p) - radii;
            bool dup = false;
            for (int i = 0; i < s.Count; i++)
            {
                if (s.P[i] == p)
                {
                    dup = true;
                }
            }

            bool movedNow = false;
            if (sep > tol)
            {
                float vr = Vector3F.Dot(nhat, r);
                if (vr >= 0f)
                {
                    return false;
                }

                lambda -= sep / vr;
                if (lambda > 1f)
                {
                    return false;
                }

                x = r * lambda;
                normal = nhat;
                moved = true;
                movedNow = true;
            }
            else if (dup)
            {
                converged = true;
                break;
            }

            Simplex saved = s;
            if (!dup)
            {
                if (s.Count == 4)
                {
                    // Cannot happen when x is outside; stop with what we have.
                    converged = true;
                    break;
                }

                s.Add(p, pa, pb);
            }

            for (int i = 0; i < s.Count; i++)
            {
                s.W[i] = x - s.P[i];
            }

            if (s.Solve(out Vector3F nvv))
            {
                v = default;
            }
            else if (!movedNow && nvv.LengthSquared() >= vv)
            {
                // No progress without moving x: the closest point is as good as round-off allows.
                s = saved;
                for (int i = 0; i < s.Count; i++)
                {
                    s.W[i] = x - s.P[i];
                }

                converged = true;
                break;
            }
            else
            {
                v = nvv;
            }
        }

        if (!converged)
        {
            return false;
        }

        Vector3F n = moved ? normal : default;
        if (!moved)
        {
            float vv = v.LengthSquared();
            if (vv > 0f)
            {
                n = v / ConvexShape3D.Sqrt(vv);
            }
        }

        Vector3F wa = s.WitnessA(), wb = s.WitnessB();
        Vector3F pointA = wa + (motionOfA * lambda) + (n * pair.A.ConvexRadius);
        Vector3F pointB = wb - (n * pair.B.ConvexRadius);
        hit = new ConvexCastHit3D(lambda, n, pointA, pointB, !moved);
        return true;
    }

    private const float TinyRayLength = 1e-6f;

    /// <summary>Casts a ray against a convex shape.</summary>
    /// <param name="shape">The shape, in its own frame.</param>
    /// <param name="shapeInRay">Placement of the shape's frame in the ray's frame.</param>
    /// <param name="origin">Ray origin in the ray's frame.</param>
    /// <param name="direction">Ray direction; its length scales the returned fraction.</param>
    /// <param name="maxFraction">The ray covers fractions 0 to this value of <paramref name="direction"/>; finite and positive.</param>
    /// <param name="hit">The first hit; <see cref="ConvexCastHit3D.Normal"/> is the shape's outward surface normal, <see cref="ConvexCastHit3D.PointB"/> the hit point (both in the ray's frame).</param>
    /// <returns>True when the ray enters the shape. A ray starting inside hits at fraction 0 with a zero normal.</returns>
    public static bool RayCast(in ConvexShape3D shape, in RigidTransformF shapeInRay, Vector3F origin, Vector3F direction, float maxFraction, out ConvexCastHit3D hit)
    {
        if (!(maxFraction > 0f) || !float.IsFinite(maxFraction))
        {
            throw new ArgumentOutOfRangeException(nameof(maxFraction), maxFraction, $"maxFraction is {maxFraction}; a finite positive value is expected.");
        }

        // A shape far from the origin makes the cast's tolerances (relative to the largest coordinate it sees) coarser than the shape's own features, so a ray passing well outside the
        // shape could report a hit. Such a ray is first clipped to the shape's padded bounds (the same test the compound trees use) and the cast starts where it enters them.
        double startFraction = 0.0;
        Vector3F castOrigin = origin;
        float castMax = maxFraction;
        Vector3F far = shapeInRay.Translation - origin;
        // A ray whose whole length is a hair (a denormal fraction, say) gives the cast no motion to measure against its tolerances, which are then coarser than the ray: it would report a hit on a shape
        // metres away. It takes the same bounds test.
        float reach = Math.Max(Math.Abs(direction.X), Math.Max(Math.Abs(direction.Y), Math.Abs(direction.Z))) * maxFraction;
        if (far.LengthSquared() > FarRayDistance * FarRayDistance || !(reach > TinyRayLength))
        {
            ShapeBounds3D.Compute(shape, shapeInRay, out Vector3F lo, out Vector3F hi);
            var clip = new Broadphase.Ray3D(origin.X, origin.Y, origin.Z, direction.X, direction.Y, direction.Z);
            if (!clip.Hit(new Broadphase.Aabb3D(lo.X, lo.Y, lo.Z, hi.X, hi.Y, hi.Z), maxFraction, out double enter))
            {
                hit = default;
                return false;
            }

            if (enter > 0.0)
            {
                startFraction = enter;
                castOrigin = new Vector3F((float)(origin.X + (direction.X * enter)), (float)(origin.Y + (direction.Y * enter)), (float)(origin.Z + (direction.Z * enter)));
                castMax = (float)(maxFraction - enter);
                if (!(castMax > 0f))
                {
                    castMax = float.Epsilon;
                }
            }
        }

        var rel = new RigidTransformF(shapeInRay.Rotation, shapeInRay.Translation - castOrigin);
        ConvexShape3D point = ConvexShape3D.Point;
        if (!ShapeCast(point, shape, rel, direction * castMax, out ConvexCastHit3D h))
        {
            hit = default;
            return false;
        }

        // The cast normal points from the point towards the shape; the surface normal is the opposite.
        float fraction = startFraction > 0.0 ? (float)(startFraction + (h.Fraction * (double)castMax)) : h.Fraction * maxFraction;
        hit = new ConvexCastHit3D(fraction, -h.Normal, castOrigin + h.PointA, castOrigin + h.PointB, startFraction <= 0.0 && h.StartsOverlapping);
        return true;
    }
}
