using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

// A half-space's separation is the minimum signed distance of the other shape's support.
// No proxy box participates in collision arithmetic: broadphase bounds only cover the supported world domain.
internal static class HalfSpace3D
{
    internal static bool HasPlane(in ConvexShape3D a, in ConvexShape3D b) => a.Kind == ConvexShapeKind3D.Plane || b.Kind == ConvexShapeKind3D.Plane;

    internal static ConvexContact3D Contact(in ConvexShape3D a, in ConvexShape3D b, in RigidTransformF bInA)
    {
        if (a.Kind == ConvexShapeKind3D.Plane && b.Kind == ConvexShapeKind3D.Plane)
            throw new ArgumentException("Two infinite half-spaces have no finite contact manifold.");
        if (a.Kind == ConvexShapeKind3D.Plane)
        {
            var n = a.PlaneNormal;
            var p = bInA.TransformPoint(b.Support(bInA.Rotation.Conjugated().Rotate(-n)));
            float separation = Vector3F.Dot(n, p) - a.PlaneOffset;
            return new ConvexContact3D(separation, n, p - n * separation, p);
        }
        var outward = bInA.Rotation.Rotate(b.PlaneNormal);
        float offset = b.PlaneOffset + Vector3F.Dot(outward, bInA.Translation);
        var pa = a.Support(-outward);
        float sep = Vector3F.Dot(outward, pa) - offset;
        return new ConvexContact3D(sep, -outward, pa, pa - outward * sep);
    }

    internal static ConvexDistance3D Distance(in ConvexShape3D a, in ConvexShape3D b, in RigidTransformF bInA)
    {
        var c = Contact(a, b, bInA);
        return new ConvexDistance3D(Math.Max(c.Separation, 0f), c.Separation <= 0f, c.Normal, c.PointA, c.PointB, 0);
    }

    internal static bool Cast(in ConvexShape3D a, in ConvexShape3D b, in RigidTransformF bInA, Vector3F motion, out ConvexCastHit3D hit)
    {
        hit = default;
        var c = Contact(a, b, bInA);
        float closing = Vector3F.Dot(c.Normal, motion);
        float fraction = c.Separation <= 0f ? 0f : closing > 0f ? c.Separation / closing : float.PositiveInfinity;
        if (!(fraction <= 1f)) return false;
        var atHit = Contact(a, b, new RigidTransformF(bInA.Rotation, bInA.Translation - motion * fraction));
        var shift = motion * fraction;
        hit = new ConvexCastHit3D(fraction, c.Normal, atHit.PointA + shift, atHit.PointB + shift, c.Separation <= 0f);
        return true;
    }

    internal static bool Ray(in ConvexShape3D plane, in RigidTransformF pose, Vector3F origin, Vector3F direction, float maxFraction, out ConvexCastHit3D hit)
    {
        hit = default;
        var n = pose.Rotation.Rotate(plane.PlaneNormal);
        float distance = Vector3F.Dot(n, origin - pose.Translation) - plane.PlaneOffset;
        float closing = -Vector3F.Dot(n, direction);
        float fraction = distance <= 0f ? 0f : closing > 0f ? distance / closing : float.PositiveInfinity;
        if (!(fraction <= maxFraction)) return false;
        var p = origin + direction * fraction;
        hit = new ConvexCastHit3D(fraction, distance <= 0f ? default : n, p, p, distance <= 0f);
        return true;
    }
}

public sealed partial class ManifoldGenerator3D
{
    private Manifold3D PlaneManifold(in ConvexShape3D a, in ConvexShape3D b, in RigidTransformF bInA, float margin)
    {
        var c = HalfSpace3D.Contact(a, b, bInA);
        if (c.Separation > margin) return Empty;
        bool planeA = a.Kind == ConvexShapeKind3D.Plane;
        var shape = planeA ? b : a;
        var outward = planeA ? a.PlaneNormal : bInA.Rotation.Rotate(b.PlaneNormal);
        var direction = planeA ? bInA.Rotation.Conjugated().Rotate(-outward) : -outward;
        Extract(shape, direction, margin + 1e-4f * Extent(shape), _fa);
        float offset = planeA ? a.PlaneOffset : b.PlaneOffset + Vector3F.Dot(outward, bInA.Translation);
        for (int i = 0; i < _fa.N; i++)
        {
            var p = planeA ? bInA.TransformPoint(_fa.P[i]) : _fa.P[i];
            float sep = Vector3F.Dot(outward, p) - offset;
            if (sep <= margin) Push(p - outward * (0.5f * sep), sep, planeA ? FCode.Pack(0, _fa.V[i]) : FCode.Pack(_fa.V[i], 0));
        }
        return Finish(c.Normal);
    }
}
