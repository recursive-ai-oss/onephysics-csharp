using System;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>Continuous convex cast with constant centre translation and constant angular displacement, against a stationary target. Implements the separating-plane motion bound of Mirtich 1996 §2.3.2 and Catto 2013, Conservative Advancement. Rotations are evaluated with sine/cosine, not sampled or linearly interpolated.</summary>
[Experimental("OPHYS0001")]
public static class AngularCast3D
{
    /// <summary>Bound on advancement work per convex pair. Exhaustion throws QueryConvergenceException, never reports a miss.</summary>
    public const int MaximumIterations = 4096;

    /// <summary>Tests a convex pair in the stationary target's frame. The mover origin and orientation start at initialPose, rotating about centre while centre translates by translation. Angular displacement is a world-axis rotation vector in radians, magnitude at most 64. Translation length is at most 2^23 metres; raw relative pose and centre components are within +-2^25. Empty shapes miss. Tolerance is finite in (0,1], default 0.0001 metre, plus a 8e-6 relative float roundoff margin. Initial overlap reports fraction zero; a core-overlap witness may be unspecified with a zero normal. Otherwise point and normal are on the target in its frame; the normal faces the mover. On a miss outputs are zero.</summary>
    /// <exception cref="ArgumentException">A shape is invalid, both shapes are infinite.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A pose/motion is nonfinite, outside the limits, or a rotation is nonunit.</exception>
    /// <exception cref="QueryConvergenceException">The bounded advancement cannot certify further progress. No impact is silently skipped.</exception>
    public static bool Test(in ConvexShape3D target, in ConvexShape3D mover, in RigidTransformF initialPose, Vector3F centre, Vector3F translation, Vector3F angularDisplacement, out float fraction, out Vector3F point, out Vector3F normal, float tolerance = 0.0001f)
    {
        fraction = 0f; point = default; normal = default;
        Validate(initialPose, centre, translation, angularDisplacement, tolerance);
        if (target.Kind == ConvexShapeKind3D.Empty || mover.Kind == ConvexShapeKind3D.Empty) { return false; }
        double angle = Length(angularDisplacement);
        if (target.Kind == ConvexShapeKind3D.Plane && mover.Kind == ConvexShapeKind3D.Plane) { throw new ArgumentException("Two infinite half-spaces have no finite query contact."); }
        ShapeBounds3D.Compute(mover, RigidTransformF.Identity, out var lower, out var upper);
        var arm = initialPose.Translation - centre;
        float scale = Math.Max(1f, mover.Kind == ConvexShapeKind3D.Plane ? Math.Abs(mover.PlaneOffset) : Math.Max(MaxAbs(lower), MaxAbs(upper)));
        if (target.Kind == ConvexShapeKind3D.Plane) { scale = Math.Max(scale, Math.Abs(target.PlaneOffset)); }
        else
        {
            ShapeBounds3D.Compute(target, RigidTransformF.Identity, out var targetMin, out var targetMax);
            scale = Math.Max(scale, Math.Max(MaxAbs(targetMin), MaxAbs(targetMax)));
        }
        var extent = new Vector3F(Math.Max(Math.Abs(lower.X), Math.Abs(upper.X)), Math.Max(Math.Abs(lower.Y), Math.Abs(upper.Y)), Math.Max(Math.Abs(lower.Z), Math.Abs(upper.Z)));
        // Keep cancellation error covered when the translated centre and a large
        // rotated offset nearly cancel at impact.
        scale = Math.Max(scale, MaxAbs(arm));
        double radius = (Length(arm) + Length(extent)) * 1.0005;
        if (mover.Kind == ConvexShapeKind3D.Plane)
        {
            // A rotating half-space has no finite point-speed bound. Its signed
            // support distance against a finite target does: |w|*(|p-c|+|v|)+|v|.
            ShapeBounds3D.Compute(target, RigidTransformF.Identity, out var targetMin, out var targetMax);
            var targetExtent = new Vector3F(Math.Max(Math.Abs(targetMin.X), Math.Abs(targetMax.X)), Math.Max(Math.Abs(targetMin.Y), Math.Abs(targetMax.Y)), Math.Max(Math.Abs(targetMin.Z), Math.Abs(targetMax.Z)));
            radius = (Length(targetExtent) + Length(centre) + Length(translation)) * 1.0005;
        }
        var axis = angle > 0 ? angularDisplacement / (float)angle : default;
        double time = 0;
        for (int iteration = 0; iteration < MaximumIterations; iteration++)
        {
            var (sin, cos) = Math64.SinCos(angle * time * 0.5);
            var delta = new QuaternionF(axis.X * (float)sin, axis.Y * (float)sin, axis.Z * (float)sin, (float)cos);
            var pose = new RigidTransformF(delta * initialPose.Rotation, centre + translation * (float)time + delta.Rotate(arm));
            var distance = Gjk3D.Distance(target, mover, pose);
            var n = distance.Normal;
            float d = distance.Distance;
            float slack = 8e-6f * Math.Max(scale, Math.Max(MaxAbs(pose.Translation), MaxAbs(centre)));
            if (d <= tolerance + slack)
            {
                fraction = (float)time; point = distance.PointA; normal = n;
                return true;
            }
            double gap;
            if (HalfSpace3D.HasPlane(target, mover)) { gap = d; }
            else
            {
                var a = target.Support(n);
                var b = pose.TransformPoint(mover.Support(pose.Rotation.Conjugated().Rotate(-n)));
                gap = Vector3F.Dot(n, b - a);
            }
            double linearBound = mover.Kind == ConvexShapeKind3D.Plane && angle > 0 ? Length(translation) : Math.Max(0, -((double)n.X * translation.X + (double)n.Y * translation.Y + (double)n.Z * translation.Z));
            double closing = linearBound + angle * radius;
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

    private static double Length(Vector3F value) => double.Sqrt((double)value.X * value.X + (double)value.Y * value.Y + (double)value.Z * value.Z);
    private static float MaxAbs(Vector3F value) => Math.Max(Math.Abs(value.X), Math.Max(Math.Abs(value.Y), Math.Abs(value.Z)));

    private static void Validate(in RigidTransformF pose, Vector3F centre, Vector3F translation, Vector3F angularDisplacement, float tolerance)
    {
        if (!(tolerance > 0 && tolerance <= 1) || !float.IsFinite(tolerance)) { throw new ArgumentOutOfRangeException(nameof(tolerance), tolerance, "Contact tolerance must be finite in (0,1] metres."); }
        if (!(float.IsFinite(translation.X) && float.IsFinite(translation.Y) && float.IsFinite(translation.Z)) || Length(translation) > 8388608)
        { throw new ArgumentOutOfRangeException(nameof(translation), translation, "Translation must be finite and at most 8388608 metres long."); }
        if (!(float.IsFinite(centre.X) && float.IsFinite(centre.Y) && float.IsFinite(centre.Z)) || MaxAbs(centre) > 33554432)
        { throw new ArgumentOutOfRangeException(nameof(centre), centre, "Rotation centre must be finite with components within +-33554432 metres."); }
        if (!(float.IsFinite(pose.Translation.X) && float.IsFinite(pose.Translation.Y) && float.IsFinite(pose.Translation.Z)) || MaxAbs(pose.Translation) > 33554432)
        { throw new ArgumentOutOfRangeException(nameof(pose), "Relative pose translation must be finite with components within +-33554432 metres."); }
        if (!float.IsFinite(angularDisplacement.X) || !float.IsFinite(angularDisplacement.Y) || !float.IsFinite(angularDisplacement.Z) || Length(angularDisplacement) > 64) { throw new ArgumentOutOfRangeException(nameof(angularDisplacement), angularDisplacement, "Angular displacement must be finite with magnitude at most 64 radians."); }
        double norm = pose.Rotation.LengthSquared();
        if (!double.IsFinite(norm) || Math.Abs(norm - 1) > 1e-4) { throw new ArgumentOutOfRangeException(nameof(pose), "Rotation must be finite and unit length."); }
    }
}
