using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

internal static class SurfaceContacts
{
    internal static void Apply(WorldCore world, int c, ref ContactModification view, Func<int, float>? friction)
    {
        var cs = world.Constraints;
        int a = cs.BodyA[c], b = cs.BodyB[c];
        var sa = world.Surfaces.Get(cs.ShapeA[c]); var sb = world.Surfaces.Get(cs.ShapeB[c]);
        if (sa.IsDefault && sb.IsDefault) return;
        var n = new Vector3F(view.NormalX, view.NormalY, view.NormalZ);
        var extra = view.Extra;
        Vector3F axis = sa.FrictionAxis != default ? Rotate(world, a, sa.FrictionAxis) : Rotate(world, b, sb.FrictionAxis);
        Tangents(n, world.Layout.PositionLanes, axis, out var t1, out var t2);
        if (axis != default)
        {
            extra[4] = t1.X; extra[5] = t1.Y; extra[6] = t1.Z; extra[9] = 1f;
            float ma = friction?.Invoke(cs.ShapeA[c]) ?? view.Friction, mb = friction?.Invoke(cs.ShapeB[c]) ?? view.Friction;
            if (world.ContactMaterials.IsPresent)
            {
                ma = world.ContactMaterials.Resolve(cs.ShapeA[c], cs.SubShapeA[c], new ContactMaterial(ma)).Friction;
                mb = world.ContactMaterials.Resolve(cs.ShapeB[c], cs.SubShapeB[c], new ContactMaterial(mb)).Friction;
            }
            view.Friction = float.Sqrt(Coefficient(world, a, sa, t1, ma) * Coefficient(world, b, sb, t1, mb));
            extra[3] = float.Sqrt(Coefficient(world, a, sa, t2, ma) * Coefficient(world, b, sb, t2, mb));
            // Stored tangent impulses refer to yesterday's material frame; do not reinterpret them after a frame rotation.
            var pl = cs.Payload(c);
            for (int p = 0; p < view.PointCount; p++) { pl[ManifoldLayout.Tangent1Impulse(p)] = 0; pl[ManifoldLayout.Tangent2Impulse(p)] = 0; }
        }
        var velocity = Rotate(world, a, sa.Velocity) - Rotate(world, b, sb.Velocity);
        view.TangentSpeed = Vector3F.Dot(velocity, t1);
        extra[2] = Vector3F.Dot(velocity, t2);
        extra[7] = Math.Max(sa.SpinningFriction, sb.SpinningFriction);
        if (sa.OneWayDirection != default && !Allows(world, c, a, b, sa.OneWayDirection, n, false)) view.Enabled = false;
        if (sb.OneWayDirection != default && !Allows(world, c, b, a, sb.OneWayDirection, n, true)) view.Enabled = false;
    }
    internal static bool SweepApproaches(WorldCore world, int movingShape, int otherShape, Vector3F relativeMotion)
    {
        var own = world.Surfaces.Get(movingShape); var other = world.Surfaces.Get(otherShape);
        if (other.OneWayDirection != default && Vector3F.Dot(relativeMotion, Rotate(world, world.Shapes.Owner[otherShape], other.OneWayDirection)) >= 0f) return false;
        if (own.OneWayDirection != default && Vector3F.Dot(relativeMotion, Rotate(world, world.Shapes.Owner[movingShape], own.OneWayDirection)) <= 0f) return false;
        return true;
    }
    internal static bool SweepNormalAllowed(WorldCore world, int movingShape, int otherShape, Vector3F outwardOther)
    {
        bool Faces(Vector3F direction, Vector3F normal)
        {
            float product = Vector3F.Dot(direction, direction) * Vector3F.Dot(normal, normal);
            return Vector3F.Dot(direction, normal) >= 0.5f * float.Sqrt(product);
        }
        var own = world.Surfaces.Get(movingShape); var other = world.Surfaces.Get(otherShape);
        if (other.OneWayDirection != default && !Faces(Rotate(world, world.Shapes.Owner[otherShape], other.OneWayDirection), outwardOther)) return false;
        if (own.OneWayDirection != default && !Faces(Rotate(world, world.Shapes.Owner[movingShape], own.OneWayDirection), -outwardOther)) return false;
        return true;
    }
    private static float Coefficient(WorldCore world, int body, SurfaceProperties surface, Vector3F tangent, float ordinary)
    {
        if (surface.FrictionAxis == default) return ordinary;
        var axis = Rotate(world, body, surface.FrictionAxis);
        axis *= 1f / float.Sqrt(Vector3F.Dot(axis, axis));
        float d = Vector3F.Dot(axis, tangent); float along = Math.Min(1f, d * d);
        return float.Sqrt(surface.LongitudinalFriction * surface.LongitudinalFriction * along + surface.TransverseFriction * surface.TransverseFriction * (1f - along));
    }
    private static bool Allows(WorldCore w, int c, int platform, int other, Vector3F direction, Vector3F normal, bool flip)
    {
        var n = Rotate(w, platform, direction); n *= 1f / float.Sqrt(Vector3F.Dot(n, n));
        float alignment = Vector3F.Dot(flip ? -normal : normal, n);
        if (alignment < 0.5f) return false; // Side and underside contacts are not a landing surface.
        var pl = w.Constraints.Payload(c);
        int ap = flip ? ManifoldLayout.AnchorB(0) : ManifoldLayout.AnchorA(0);
        int ao = flip ? ManifoldLayout.AnchorA(0) : ManifoldLayout.AnchorB(0);
        var vp = PointVelocity(w, platform, pl, ap); var vo = PointVelocity(w, other, pl, ao);
        float velocity = Vector3F.Dot(vo - vp, n);
        if (velocity > 0.01f) return false; // Moving up through a platform never acquires a response.
        // Pre-solve allowed-side test on the previous separation. A falling body can land even if a discrete step overshot the plane.
        return pl[ManifoldLayout.Separation(0)] - velocity * w.ContactStepDt >= -0.02f || pl[ManifoldLayout.NormalImpulse(0)] > 0f;
    }
    internal static Vector3F PointVelocity(WorldCore w, int body, ReadOnlySpan<float> pl, int anchor)
    {
        var v = w.Bodies.LinearVelocityOf(body); var spin = w.Bodies.AngularVelocityOf(body);
        if (w.Layout.PositionLanes == 2) return new(v[0] - spin[0] * pl[anchor + 1], v[1] + spin[0] * pl[anchor], 0);
        return new Vector3F(v[0], v[1], v[2]) + Vector3F.Cross(new(spin[0], spin[1], spin[2]), new(pl[anchor], pl[anchor + 1], pl[anchor + 2]));
    }
    internal static Vector3F Rotate(WorldCore w, int body, Vector3F v)
    {
        var r = w.Bodies.RotationOf(body);
        if (w.Layout.PositionLanes == 2) return new(r[0] * v.X - r[1] * v.Y, r[1] * v.X + r[0] * v.Y, 0);
        return new QuaternionF(r[0], r[1], r[2], r[3]).Rotate(v);
    }
    internal static void Tangents(Vector3F n, int dimension, Vector3F preferred, out Vector3F t1, out Vector3F t2)
    {
        if (dimension == 2) { t1 = new(n.Y, -n.X, 0); t2 = default; return; }
        Vector3F t = preferred - n * Vector3F.Dot(preferred, n);
        float length = Vector3F.Dot(t, t);
        if (!(length > 1e-12f)) { t = Math.Abs(n.X) >= 0.57735f ? new(n.Y, -n.X, 0) : new(0, n.Z, -n.Y); length = Vector3F.Dot(t, t); }
        t1 = length > 0f ? t * (1f / float.Sqrt(length)) : new(1, 0, 0); t2 = Vector3F.Cross(n, t1);
    }
}
