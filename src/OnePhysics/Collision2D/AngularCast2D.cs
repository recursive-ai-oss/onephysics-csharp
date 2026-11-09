using System;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>Continuous convex cast with constant centre translation and constant angular displacement, against a stationary target. Implements the separating-plane motion bound of Mirtich 1996 §2.3.2 and Catto 2013, Conservative Advancement. Rotations are evaluated with sine/cosine, not sampled or linearly interpolated.</summary>
[Experimental("OPHYS0001")]
public static class AngularCast2D
{
    /// <summary>Bound on advancement work per convex pair. Exhaustion throws QueryConvergenceException, never reports a miss.</summary>
    public const int MaximumIterations = 4096;

    /// <summary>Tests a convex pair in the stationary target's frame. The mover origin and orientation start at initialPose, rotating about centre while centre translates by translation. Angular displacement is a signed angle in radians, magnitude at most 64. Translation length is at most 2^23 metres; raw relative pose and centre components are within +-2^25. Empty shapes miss. Tolerance is finite in (0,1], default 0.0001 metre, plus a 8e-6 relative float roundoff margin. Initial overlap reports fraction zero; a core-overlap witness may be unspecified with a zero normal. Otherwise point and normal are on the target in its frame; the normal faces the mover. On a miss outputs are zero.</summary>
    /// <exception cref="ArgumentException">A default shape has no geometry.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A pose/motion is nonfinite, outside the limits, or a rotation is nonunit.</exception>
    /// <exception cref="QueryConvergenceException">The bounded advancement cannot certify further progress. No impact is silently skipped.</exception>
    public static bool Test(in Shape2D target, in Shape2D mover, in Transform2 initialPose, Vec2 centre, Vec2 translation, float angularDisplacement, out float fraction, out Vec2 point, out Vec2 normal, float tolerance = 0.0001f)
    {
        fraction = 0f; point = default; normal = default;
        Validate(initialPose, centre, translation, angularDisplacement, tolerance);
        if (target.Kind == ShapeKind2D.Empty || mover.Kind == ShapeKind2D.Empty) { return false; }
        if (target.Count == 0 || mover.Count == 0) { throw new ArgumentException("Default shapes have no query geometry; use a shape factory."); }
        double angle = Math.Abs((double)angularDisplacement);
        var bounds = mover.ComputeAabb(Transform2.Identity);
        var arm = initialPose.P - centre;
        var targetBounds = target.ComputeAabb(Transform2.Identity);
        float scale = Math.Max(1f, Math.Max(Math.Max(MaxAbs(bounds.Lower), MaxAbs(bounds.Upper)), Math.Max(MaxAbs(targetBounds.Lower), MaxAbs(targetBounds.Upper))));
        var extent = new Vec2(Math.Max(Math.Abs(bounds.Lower.X), Math.Abs(bounds.Upper.X)), Math.Max(Math.Abs(bounds.Lower.Y), Math.Abs(bounds.Upper.Y)));
        // Keep cancellation error covered when the translated centre and a large
        // rotated offset nearly cancel at impact.
        scale = Math.Max(scale, MaxAbs(arm));
        double radius = (Length(arm) + Length(extent)) * 1.0005;
        double time = 0;
        for (int iteration = 0; iteration < MaximumIterations; iteration++)
        {
            var (sin, cos) = Math64.SinCos(angularDisplacement * time);
            var delta = new Rot2((float)cos, (float)sin);
            var pose = new Transform2(centre + translation * (float)time + delta.Rotate(arm), Rot2.Mul(delta, initialPose.Q));
            SimplexCache2D cache = default;
            var distance = Distance2D.Compute(target, mover, pose, ref cache, false);
            float d = Math.Max(0f, distance.Distance - (target.Radius + mover.Radius));
            var n = d > 0 ? (distance.PointB - distance.PointA).Normalized() : default;
            float slack = 8e-6f * Math.Max(scale, Math.Max(MaxAbs(pose.P), MaxAbs(centre)));
            if (d <= tolerance + slack)
            {
                fraction = (float)time; point = distance.PointA + n * target.Radius; normal = n;
                return true;
            }
            double gap = Separation(target, mover, pose, n);
            // Core witnesses avoid normalizing an almost-cancelled, radii-adjusted
            // witness difference. Sharp polygon faces also supply well-conditioned axes.
            foreach (var axis in target.Normals) { ImproveAxis(target, mover, pose, axis, ref gap, ref n); }
            foreach (var axis in mover.Normals) { ImproveAxis(target, mover, pose, pose.Q.Rotate(axis), ref gap, ref n); }
            double closing = Math.Max(0, -((double)n.X * translation.X + (double)n.Y * translation.Y)) + angle * radius;
            if (time == 1 || closing == 0) { return false; }
            // The support projection is a lower separation bound even when GJK's
            // closest-feature iteration stopped early. Never advance on witness distance alone.
            double step = 0.9 * (gap - slack - tolerance * 0.5) / closing;
            if (!(step > 0)) { throw new QueryConvergenceException(time); }
            double next = Math.Min(1, time + step);
            if ((float)next <= (float)time) { throw new QueryConvergenceException(time); }
            time = next;
        }
        throw new QueryConvergenceException(time);
    }

    private static double Separation(in Shape2D target, in Shape2D mover, Transform2 pose, Vec2 n) =>
        Vec2.Dot(n, pose.Apply(Support(mover, pose.Q.InverseRotate(-n))) - Support(target, n));

    private static void ImproveAxis(in Shape2D target, in Shape2D mover, Transform2 pose, Vec2 axis, ref double gap, ref Vec2 normal)
    {
        var n = axis.Normalized();
        double positive = Separation(target, mover, pose, n);
        if (positive > gap) { gap = positive; normal = n; }
        double negative = Separation(target, mover, pose, -n);
        if (negative > gap) { gap = negative; normal = -n; }
    }

    private static Vec2 Support(in Shape2D shape, Vec2 direction)
    {
        int best = 0; float value = Vec2.Dot(shape.Vertices[0], direction);
        for (int i = 1; i < shape.Count; i++)
        {
            float dot = Vec2.Dot(shape.Vertices[i], direction);
            if (dot > value) { best = i; value = dot; }
        }
        return shape.Vertices[best] + direction * shape.Radius;
    }

    private static double Length(Vec2 value) => double.Sqrt((double)value.X * value.X + (double)value.Y * value.Y);
    private static float MaxAbs(Vec2 value) => Math.Max(Math.Abs(value.X), Math.Abs(value.Y));

    private static void Validate(in Transform2 pose, Vec2 centre, Vec2 translation, float angularDisplacement, float tolerance)
    {
        if (!(tolerance > 0 && tolerance <= 1) || !float.IsFinite(tolerance)) { throw new ArgumentOutOfRangeException(nameof(tolerance), tolerance, "Contact tolerance must be finite in (0,1] metres."); }
        if (!(float.IsFinite(translation.X) && float.IsFinite(translation.Y)) || Length(translation) > 8388608)
        { throw new ArgumentOutOfRangeException(nameof(translation), translation, "Translation must be finite and at most 8388608 metres long."); }
        if (!(float.IsFinite(centre.X) && float.IsFinite(centre.Y)) || MaxAbs(centre) > 33554432)
        { throw new ArgumentOutOfRangeException(nameof(centre), centre, "Rotation centre must be finite with components within +-33554432 metres."); }
        if (!(float.IsFinite(pose.P.X) && float.IsFinite(pose.P.Y)) || MaxAbs(pose.P) > 33554432)
        { throw new ArgumentOutOfRangeException(nameof(pose), "Relative pose translation must be finite with components within +-33554432 metres."); }
        if (!float.IsFinite(angularDisplacement) || Math.Abs(angularDisplacement) > 64) { throw new ArgumentOutOfRangeException(nameof(angularDisplacement), angularDisplacement, "Angular displacement must be finite within +-64 radians."); }
        double norm = (double)pose.Q.C * pose.Q.C + (double)pose.Q.S * pose.Q.S;
        if (!double.IsFinite(norm) || Math.Abs(norm - 1) > 1e-4) { throw new ArgumentOutOfRangeException(nameof(pose), "Rotation must be finite and unit length."); }
    }
}
