using System;
using System.Runtime.CompilerServices;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>
/// Warm-start data for <see cref="Distance2D.Compute"/>: the vertices of the last simplex. Pass the same cache for the same pair on the next call (a default value is a valid empty cache).
/// A cache with out-of-range content is ignored, never an error.
/// </summary>
[Experimental("OPHYS0001")]
public struct SimplexCache2D
{
    /// <summary>Number of simplex vertices stored (0 to 3).</summary>
    public int Count { get; set; }

    internal byte A0, A1, A2, B0, B1, B2;
}

/// <summary>Result of a distance query. Points are in the frame of shape A.</summary>
[Experimental("OPHYS0001")]
public readonly struct DistanceOutput2D
{
    /// <summary>Closest point on shape A.</summary>
    public Vec2 PointA { get; }

    /// <summary>Closest point on shape B.</summary>
    public Vec2 PointB { get; }

    /// <summary>Distance between the shapes (between the cores when radii are not used); 0 when they overlap.</summary>
    public float Distance { get; }

    /// <summary>GJK iterations used.</summary>
    public int Iterations { get; }

    /// <summary>Creates a result.</summary>
    public DistanceOutput2D(Vec2 a, Vec2 b, float distance, int iterations)
    {
        PointA = a;
        PointB = b;
        Distance = distance;
        Iterations = iterations;
    }
}

/// <summary>Motion of a body over a step for continuous collision: the centre of mass moves in a straight line and the rotation is interpolated by normalised linear interpolation of (cos, sin), so no trigonometry is needed.</summary>
[Experimental("OPHYS0001")]
public readonly struct Sweep2D
{
    /// <summary>Centre of mass in the body frame (the point the rotation turns about).</summary>
    public Vec2 LocalCenter { get; }

    /// <summary>Centre of mass at the start, in the common frame.</summary>
    public Vec2 C1 { get; }

    /// <summary>Centre of mass at the end.</summary>
    public Vec2 C2 { get; }

    /// <summary>Rotation at the start.</summary>
    public Rot2 Q1 { get; }

    /// <summary>Rotation at the end.</summary>
    public Rot2 Q2 { get; }

    /// <summary>Creates a sweep.</summary>
    public Sweep2D(Vec2 localCenter, Vec2 c1, Vec2 c2, Rot2 q1, Rot2 q2)
    {
        LocalCenter = localCenter;
        C1 = c1;
        C2 = c2;
        Q1 = q1;
        Q2 = q2;
    }

    internal bool IsFinite => float.IsFinite(LocalCenter.X + LocalCenter.Y + C1.X + C1.Y + C2.X + C2.Y + Q1.C + Q1.S + Q2.C + Q2.S);

    /// <summary>The body transform at fraction <paramref name="t"/> of the step.</summary>
    public Transform2 GetTransform(float t)
    {
        Vec2 c = (1f - t) * C1 + t * C2;
        Rot2 q = Rot2.FromVector((1f - t) * Q1.C + t * Q2.C, (1f - t) * Q1.S + t * Q2.S);
        return new Transform2(c - q.Rotate(LocalCenter), q);
    }
}

/// <summary>Outcome of a time-of-impact search.</summary>
[Experimental("OPHYS0001")]
public enum ToiState : byte
{
    /// <summary>The search ran out of iterations; the fraction is a time before contact (safe).</summary>
    Failed = 0,

    /// <summary>The shapes overlap at the start; the fraction is 0.</summary>
    Overlapped = 1,

    /// <summary>The shapes come within the target gap at the reported fraction.</summary>
    Hit = 2,

    /// <summary>The shapes do not touch during the sweep; the fraction is the maximum.</summary>
    Separated = 3,
}

/// <summary>Result of <see cref="Distance2D.TimeOfImpact"/>.</summary>
[Experimental("OPHYS0001")]
public readonly struct ToiOutput2D
{
    /// <summary>What happened.</summary>
    public ToiState State { get; }

    /// <summary>Fraction of the sweep.</summary>
    public float Fraction { get; }

    /// <summary>Creates a result.</summary>
    public ToiOutput2D(ToiState state, float fraction)
    {
        State = state;
        Fraction = fraction;
    }
}

[InlineArray(3)]
internal struct SimplexVertices
{
    private SimplexVertex _e0;
}

internal struct SimplexVertex
{
    public Vec2 WA, WB, W;
    public float A;
    public int IA, IB;
}

/// <summary>Distance between convex shapes (GJK), shape casts and time of impact.</summary>
[Experimental("OPHYS0001")]
public static class Distance2D
{
    private const float Epsilon = 1.1920929e-7f;

    internal static float PointSegmentSquared(Vec2 p, Vec2 a, Vec2 b)
    {
        Vec2 ab = b - a;
        float l2 = Vec2.Dot(ab, ab);
        float t = l2 > 0f ? Vec2.Dot(p - a, ab) / l2 : 0f;
        t = t < 0f ? 0f : t > 1f ? 1f : t;
        return (p - (a + t * ab)).LengthSquared;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int Support(in Vec2x8 v, int n, Vec2 d)
    {
        int best = 0;
        float bv = Vec2.Dot(v[0], d);
        for (int i = 1; i < n; i++)
        {
            float val = Vec2.Dot(v[i], d);
            if (val > bv) { best = i; bv = val; }
        }

        return best;
    }

    /// <summary>
    /// Closest points between two convex shapes by GJK on the Minkowski difference (Gilbert, Johnson and Keerthi 1988; Catto, GDC 2010, "Computing Distance").
    /// </summary>
    /// <param name="a">First shape, in its own frame (the frame all results are in).</param>
    /// <param name="b">Second shape.</param>
    /// <param name="xfBInA">Pose of shape B's frame in A's frame.</param>
    /// <param name="cache">Warm-start simplex; updated.</param>
    /// <param name="useRadii">When true the shapes' radii are included: the distance is between the rounded surfaces and the points lie on them (0 and a shared point when they overlap). When false the cores are compared.</param>
    public static DistanceOutput2D Compute(in Shape2D a, in Shape2D b, Transform2 xfBInA, ref SimplexCache2D cache, bool useRadii = true)
    {
        if (a._count == 0 || b._count == 0 || !xfBInA.IsFinite) { return default; }
        var vb = new Vec2x8();
        for (int i = 0; i < b._count; i++) { vb[i] = xfBInA.Apply(b._v[i]); }
        float d = Core(in a._v, a._count, in vb, b._count, ref cache, out Vec2 pA, out Vec2 pB, out int iters);
        if (useRadii)
        {
            float rA = a._radius, rB = b._radius;
            if (d > rA + rB && d > Epsilon)
            {
                Vec2 n = (pB - pA) * (1f / d);
                pA = pA + rA * n;
                pB = pB - rB * n;
                d -= rA + rB;
            }
            else
            {
                Vec2 p = 0.5f * (pA + pB);
                pA = p;
                pB = p;
                d = 0f;
            }
        }

        return new DistanceOutput2D(pA, pB, d, iters);
    }

    // GJK on the point sets va (shape A) and vb (shape B, already in A's frame); returns the distance between the convex hulls.
    internal static float Core(in Vec2x8 va, int na, in Vec2x8 vb, int nb, ref SimplexCache2D cache, out Vec2 pA, out Vec2 pB, out int iterations)
    {
        var s = new SimplexVertices();
        int count = 0;
        if (cache.Count >= 1 && cache.Count <= 3)
        {
            byte ia0 = cache.A0, ia1 = cache.A1, ia2 = cache.A2, ib0 = cache.B0, ib1 = cache.B1, ib2 = cache.B2;
            bool ok = ia0 < na && ib0 < nb && (cache.Count < 2 || (ia1 < na && ib1 < nb)) && (cache.Count < 3 || (ia2 < na && ib2 < nb));
            if (ok)
            {
                count = cache.Count;
                s[0].IA = ia0; s[0].IB = ib0;
                s[1].IA = ia1; s[1].IB = ib1;
                s[2].IA = ia2; s[2].IB = ib2;
            }
        }

        for (int i = 0; i < count; i++)
        {
            s[i].WA = va[s[i].IA];
            s[i].WB = vb[s[i].IB];
            s[i].W = s[i].WB - s[i].WA;
            s[i].A = -1f;
        }

        if (count == 0)
        {
            s[0].IA = 0; s[0].IB = 0;
            s[0].WA = va[0]; s[0].WB = vb[0];
            s[0].W = s[0].WB - s[0].WA;
            s[0].A = 1f;
            count = 1;
        }

        int saveA0 = 0, saveA1 = 0, saveA2 = 0, saveB0 = 0, saveB1 = 0, saveB2 = 0;
        int iter = 0;
        while (iter < 20)
        {
            int saveCount = count;
            saveA0 = s[0].IA; saveA1 = s[1].IA; saveA2 = s[2].IA;
            saveB0 = s[0].IB; saveB1 = s[1].IB; saveB2 = s[2].IB;

            if (count == 2) { Solve2(ref s, ref count); }
            else if (count == 3) { Solve3(ref s, ref count); }
            if (count == 3) { break; }

            Vec2 dir;
            if (count == 1) { dir = -s[0].W; }
            else
            {
                Vec2 e12 = s[1].W - s[0].W;
                float sgn = Vec2.Cross(e12, -s[0].W);
                dir = sgn > 0f ? Vec2.LeftPerp(e12) : Vec2.RightPerp(e12);
            }

            if (dir.LengthSquared < Epsilon * Epsilon) { break; }

            int ia = Support(in va, na, -dir);
            int ib = Support(in vb, nb, dir);
            iter++;

            bool dup = false;
            for (int i = 0; i < saveCount; i++)
            {
                int sa = i == 0 ? saveA0 : i == 1 ? saveA1 : saveA2;
                int sb = i == 0 ? saveB0 : i == 1 ? saveB1 : saveB2;
                if (ia == sa && ib == sb) { dup = true; break; }
            }

            if (dup) { break; }
            s[count].IA = ia;
            s[count].IB = ib;
            s[count].WA = va[ia];
            s[count].WB = vb[ib];
            s[count].W = s[count].WB - s[count].WA;
            count++;
        }

        // Witness points.
        if (count == 1) { pA = s[0].WA; pB = s[0].WB; }
        else if (count == 2)
        {
            pA = s[0].A * s[0].WA + s[1].A * s[1].WA;
            pB = s[0].A * s[0].WB + s[1].A * s[1].WB;
        }
        else
        {
            pA = s[0].A * s[0].WA + s[1].A * s[1].WA + s[2].A * s[2].WA;
            pB = pA;
        }

        cache.Count = count;
        cache.A0 = (byte)s[0].IA; cache.A1 = (byte)s[1].IA; cache.A2 = (byte)s[2].IA;
        cache.B0 = (byte)s[0].IB; cache.B1 = (byte)s[1].IB; cache.B2 = (byte)s[2].IB;
        iterations = iter;
        return (pB - pA).Length;
    }

    // Barycentric coordinates of the origin's closest point on the segment w1 w2; drops a vertex when the closest point is an endpoint.
    private static void Solve2(ref SimplexVertices s, ref int count)
    {
        Vec2 w1 = s[0].W, w2 = s[1].W;
        Vec2 e12 = w2 - w1;
        float d12_2 = -Vec2.Dot(w1, e12);
        if (d12_2 <= 0f) { s[0].A = 1f; count = 1; return; }
        float d12_1 = Vec2.Dot(w2, e12);
        if (d12_1 <= 0f) { s[1].A = 1f; s[0] = s[1]; count = 1; return; }
        float inv = 1f / (d12_1 + d12_2);
        s[0].A = d12_1 * inv;
        s[1].A = d12_2 * inv;
        count = 2;
    }

    // Voronoi regions of the triangle: three vertices, three edges, the inside.
    private static void Solve3(ref SimplexVertices s, ref int count)
    {
        Vec2 w1 = s[0].W, w2 = s[1].W, w3 = s[2].W;
        Vec2 e12 = w2 - w1;
        float d12_1 = Vec2.Dot(w2, e12), d12_2 = -Vec2.Dot(w1, e12);
        Vec2 e13 = w3 - w1;
        float d13_1 = Vec2.Dot(w3, e13), d13_2 = -Vec2.Dot(w1, e13);
        Vec2 e23 = w3 - w2;
        float d23_1 = Vec2.Dot(w3, e23), d23_2 = -Vec2.Dot(w2, e23);
        float n123 = Vec2.Cross(e12, e13);
        float d123_1 = n123 * Vec2.Cross(w2, w3);
        float d123_2 = n123 * Vec2.Cross(w3, w1);
        float d123_3 = n123 * Vec2.Cross(w1, w2);

        if (d12_2 <= 0f && d13_2 <= 0f) { s[0].A = 1f; count = 1; return; }
        if (d12_1 > 0f && d12_2 > 0f && d123_3 <= 0f)
        {
            float inv = 1f / (d12_1 + d12_2);
            s[0].A = d12_1 * inv;
            s[1].A = d12_2 * inv;
            count = 2;
            return;
        }

        if (d13_1 > 0f && d13_2 > 0f && d123_2 <= 0f)
        {
            float inv = 1f / (d13_1 + d13_2);
            s[0].A = d13_1 * inv;
            s[2].A = d13_2 * inv;
            s[1] = s[2];
            count = 2;
            return;
        }

        if (d12_1 <= 0f && d23_2 <= 0f) { s[1].A = 1f; s[0] = s[1]; count = 1; return; }
        if (d13_1 <= 0f && d23_1 <= 0f) { s[2].A = 1f; s[0] = s[2]; count = 1; return; }
        if (d23_1 > 0f && d23_2 > 0f && d123_1 <= 0f)
        {
            float inv = 1f / (d23_1 + d23_2);
            s[1].A = d23_1 * inv;
            s[2].A = d23_2 * inv;
            s[0] = s[2];
            count = 2;
            return;
        }

        float invA = 1f / (d123_1 + d123_2 + d123_3);
        s[0].A = d123_1 * invA;
        s[1].A = d123_2 * invA;
        s[2].A = d123_3 * invA;
        count = 3;
    }

    // ---------------------------------------------------------------- time of impact and shape cast

    // Largest distance of a core vertex from the body's centre of mass (bounds how far rotation can move the shape).
    private static float MaxRadius(in Shape2D s, Vec2 localCenter)
    {
        float m = 0f;
        for (int i = 0; i < s._count; i++)
        {
            float l = (s._v[i] - localCenter).LengthSquared;
            if (l > m) { m = l; }
        }

        return float.Sqrt(m);
    }

    // The most any point's position can change per unit of the sweep fraction because of rotation. The rotation is q(t) = normalise((1-t) q1 + t q2),
    // whose speed is at most |q2 - q1| / min|lerp|; the minimum norm of the lerp is |q1 + q2| / 2 (at t = 1/2). Returns false when the rotation is too large to bound.
    private static bool RotationRate(Rot2 q1, Rot2 q2, out float rate)
    {
        float chord = new Vec2(q2.C - q1.C, q2.S - q1.S).Length;
        float lmin = 0.5f * new Vec2(q2.C + q1.C, q2.S + q1.S).Length;
        rate = 0f;
        if (chord == 0f) { return true; }
        if (!(lmin > 1e-3f)) { return false; }
        rate = chord / lmin;
        return true;
    }

    /// <summary>
    /// Time of impact by conservative advancement (Catto, GDC 2013, "Continuous Collision"; Ericson, Real-Time Collision Detection ch. 9.5; Mirtich 1996):
    /// advance time by the gap divided by an upper bound on the speed at which the gap can close, so the reported time is never past the first touch.
    /// </summary>
    /// <param name="a">First shape (in its body's frame).</param>
    /// <param name="sweepA">Motion of the first body.</param>
    /// <param name="b">Second shape.</param>
    /// <param name="sweepB">Motion of the second body.</param>
    /// <param name="maxFraction">Largest fraction of the sweep to search, in [0, 1].</param>
    /// <param name="targetGap">Gap between the rounded surfaces at which to stop. Must be at least 0; the time reported is when the gap first falls to within a quarter of <see cref="Constants2D.LinearSlop"/> of it.</param>
    /// <returns>
    /// <see cref="ToiState.Hit"/> with the fraction; <see cref="ToiState.Overlapped"/> when the surfaces already overlap at the start; <see cref="ToiState.Separated"/> when they never come within the target gap;
    /// <see cref="ToiState.Failed"/> after 50 iterations or when a body rotates by about 170° or more in the sweep (the fraction is still a time before contact).
    /// </returns>
    public static ToiOutput2D TimeOfImpact(in Shape2D a, in Sweep2D sweepA, in Shape2D b, in Sweep2D sweepB, float maxFraction = 1f, float targetGap = Constants2D.LinearSlop)
    {
        return Advance(in a, in sweepA, in b, in sweepB, maxFraction, targetGap, 0.25f * Constants2D.LinearSlop);
    }

    private static ToiOutput2D Advance(in Shape2D a, in Sweep2D sa, in Shape2D b, in Sweep2D sb, float tMax, float target, float tolerance)
    {
        if (a._count == 0 || b._count == 0 || !(tMax >= 0f) || !(target >= 0f) || !float.IsFinite(tMax + target) || !sa.IsFinite || !sb.IsFinite) { return default; }
        if (!RotationRate(sa.Q1, sa.Q2, out float kA) || !RotationRate(sb.Q1, sb.Q2, out float kB)) { return default; }
        if (tMax > 1f) { tMax = 1f; }
        float rTotal = a._radius + b._radius;
        float rotBound = MaxRadius(in a, sa.LocalCenter) * kA + MaxRadius(in b, sb.LocalCenter) * kB;
        Vec2 dv = (sb.C2 - sb.C1) - (sa.C2 - sa.C1);
        var cache = new SimplexCache2D();
        float t = 0f;
        for (int iter = 0; iter < 50; iter++)
        {
            Transform2 xa = sa.GetTransform(t);
            Transform2 xb = sb.GetTransform(t);
            Transform2 rel = Transform2.InvMul(xa, xb);
            var vb = new Vec2x8();
            for (int i = 0; i < b._count; i++) { vb[i] = rel.Apply(b._v[i]); }
            float dCore = Core(in a._v, a._count, in vb, b._count, ref cache, out Vec2 pA, out Vec2 pB, out _);
            float gap = dCore - rTotal;
            if (gap <= 0f) { return new ToiOutput2D(iter == 0 ? ToiState.Overlapped : ToiState.Hit, t); }
            if (gap <= target + tolerance) { return new ToiOutput2D(ToiState.Hit, t); }
            Vec2 nA = dCore > Epsilon ? (pB - pA) * (1f / dCore) : new Vec2(1f, 0f);
            Vec2 nWorld = xa.Q.Rotate(nA);
            float bound = -Vec2.Dot(nWorld, dv) + rotBound;
            if (!(bound > 0f)) { return new ToiOutput2D(ToiState.Separated, tMax); }
            t += (gap - target) / bound;
            if (t >= tMax) { return new ToiOutput2D(ToiState.Separated, tMax); }
        }

        return new ToiOutput2D(ToiState.Failed, t);
    }

    /// <summary>Gap tolerance of <see cref="ShapeCast"/>: a cast reports a hit when the surfaces come within this distance.</summary>
    public const float ShapeCastTolerance = 1e-4f;

    /// <summary>
    /// Sweeps shape <paramref name="mover"/> along a straight translation against the fixed shape <paramref name="target"/> (a shape cast), using conservative advancement
    /// with pure translation, which converges to the first touch from before it. The hit fraction is within <see cref="ShapeCastTolerance"/> of the translation's length
    /// short of the exact touch time. A mover that already overlaps the target hits at fraction 0 with a zero normal.
    /// </summary>
    /// <param name="target">The shape that stays still, in its frame (the frame of the result).</param>
    /// <param name="mover">The shape that moves.</param>
    /// <param name="xfMoverInTarget">Start pose of the mover in the target's frame.</param>
    /// <param name="translation">Displacement of the mover over fraction 1, in the target's frame.</param>
    /// <param name="maxFraction">Largest fraction to test, in [0, 1] (default 1).</param>
    /// <returns>The fraction, the normal on the target pointing towards the mover, and the contact point on the target's surface (all in the target's frame).</returns>
    public static RayCastOutput2D ShapeCast(in Shape2D target, in Shape2D mover, Transform2 xfMoverInTarget, Vec2 translation, float maxFraction = 1f)
    {
        if (target._count == 0 || mover._count == 0 || !xfMoverInTarget.IsFinite || !translation.IsFinite) { return default; }
        var still = new Sweep2D(default, default, default, Rot2.Identity, Rot2.Identity);
        var move = new Sweep2D(default, xfMoverInTarget.P, xfMoverInTarget.P + translation, xfMoverInTarget.Q, xfMoverInTarget.Q);
        ToiOutput2D o = Advance(in target, in still, in mover, in move, maxFraction, 0f, ShapeCastTolerance);
        if (o.State != ToiState.Hit && o.State != ToiState.Overlapped) { return default; }
        Transform2 xf = move.GetTransform(o.Fraction);
        var cache = new SimplexCache2D();
        DistanceOutput2D d = Compute(in target, in mover, xf, ref cache, false);
        if (o.State == ToiState.Overlapped || d.Distance <= Epsilon)
        {
            return new RayCastOutput2D(true, o.Fraction, default, d.PointA);
        }

        Vec2 n = (d.PointB - d.PointA) * (1f / d.Distance);
        return new RayCastOutput2D(true, o.Fraction, n, d.PointA + target._radius * n);
    }
}
