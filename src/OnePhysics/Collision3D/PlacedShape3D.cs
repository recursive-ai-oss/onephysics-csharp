using System;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>
/// Padded axis-aligned bounds of a convex shape under a rigid transform, found from six support points (Ericson, <i>Real-Time Collision Detection</i>, 4.2.6 style: the box of a
/// support-mapped shape along the axes), so they are tight for every shape kind, offset cores and hulls included.
/// </summary>
[Experimental("OPHYS0001")]
public static class ShapeBounds3D
{
    // Each side is grown by 2^-16 of (|lo| + |hi| + hi - lo): float32 rounding in the transform and the tolerance of the GJK casts (a few 1e-6 of the shape's size)
    // must never let a shape poke out of its box, because the boxes decide which children a query visits.
    private const float PadFactor = 1.0f / 65536f;

    /// <summary>The bounds of <paramref name="shape"/> placed at <paramref name="pose"/>, grown by a relative padding of 2^-16.</summary>
    /// <param name="shape">The shape in its own frame; must not be the point shape.</param>
    /// <param name="pose">Placement of the shape's frame in the frame the bounds are wanted in; a unit rotation.</param>
    /// <param name="min">Lower corner.</param>
    /// <param name="max">Upper corner.</param>
    public static void Compute(in ConvexShape3D shape, in RigidTransformF pose, out Vector3F min, out Vector3F max)
    {
        Matrix3x3F m = pose.Rotation.ToMatrix();
        Vector3F t = pose.Translation;
        // The primitives are symmetric about their origin, so each half extent is the support function along a row of the rotation, in closed form (Ericson, Real-Time Collision Detection, 4.2.6:
        // the half extents of a rotated box are |R| times the half sizes). This replaces six support queries, each of which normalises its direction when the shape is rounded.
        float r = shape.ConvexRadius;
        switch (shape.Kind)
        {
            case ConvexShapeKind3D.Sphere:
                Symmetric(t, r, r, r, out min, out max);
                return;
            case ConvexShapeKind3D.Box:
                {
                    Vector3F h = shape.CoreSize;
                    Symmetric(t,
                        Math.Abs(m.M11) * h.X + Math.Abs(m.M12) * h.Y + Math.Abs(m.M13) * h.Z + r,
                        Math.Abs(m.M21) * h.X + Math.Abs(m.M22) * h.Y + Math.Abs(m.M23) * h.Z + r,
                        Math.Abs(m.M31) * h.X + Math.Abs(m.M32) * h.Y + Math.Abs(m.M33) * h.Z + r, out min, out max);
                    return;
                }

            case ConvexShapeKind3D.Capsule:
                {
                    float hy = shape.CoreSize.Y;
                    Symmetric(t, Math.Abs(m.M12) * hy + r, Math.Abs(m.M22) * hy + r, Math.Abs(m.M32) * hy + r, out min, out max);
                    return;
                }

            case ConvexShapeKind3D.Cylinder:
                {
                    Vector3F h = shape.CoreSize;
                    Symmetric(t,
                        float.Sqrt(m.M11 * m.M11 + m.M13 * m.M13) * h.X + Math.Abs(m.M12) * h.Y + r,
                        float.Sqrt(m.M21 * m.M21 + m.M23 * m.M23) * h.X + Math.Abs(m.M22) * h.Y + r,
                        float.Sqrt(m.M31 * m.M31 + m.M33 * m.M33) * h.X + Math.Abs(m.M32) * h.Y + r, out min, out max);
                    return;
                }
        }

        float x0 = Extreme(shape, m.Row(0), t.X, -1f), x1 = Extreme(shape, m.Row(0), t.X, 1f);
        float y0 = Extreme(shape, m.Row(1), t.Y, -1f), y1 = Extreme(shape, m.Row(1), t.Y, 1f);
        float z0 = Extreme(shape, m.Row(2), t.Z, -1f), z1 = Extreme(shape, m.Row(2), t.Z, 1f);
        min = new Vector3F(x0 - Pad(x0, x1), y0 - Pad(y0, y1), z0 - Pad(z0, z1));
        max = new Vector3F(x1 + Pad(x0, x1), y1 + Pad(y0, y1), z1 + Pad(z0, z1));
    }

    private static void Symmetric(Vector3F t, float ex, float ey, float ez, out Vector3F min, out Vector3F max)
    {
        float x0 = t.X - ex, x1 = t.X + ex, y0 = t.Y - ey, y1 = t.Y + ey, z0 = t.Z - ez, z1 = t.Z + ez;
        min = new Vector3F(x0 - Pad(x0, x1), y0 - Pad(y0, y1), z0 - Pad(z0, z1));
        max = new Vector3F(x1 + Pad(x0, x1), y1 + Pad(y0, y1), z1 + Pad(z0, z1));
    }

    // World coordinate i of R*s + t is row_i . s + t_i, so the extreme along axis i is the support of the shape along +-row_i.
    private static float Extreme(in ConvexShape3D shape, Vector3F row, float t, float sign)
    {
        Vector3F p = shape.Support(row * sign);
        return (Vector3F.Dot(row, p)) + t;
    }

    private static float Pad(float lo, float hi) => (Math.Abs(lo) + Math.Abs(hi) + (hi - lo)) * PadFactor;
}

/// <summary>
/// A convex shape with a local rigid transform (a rotated and translated shape): the shape's frame sits at <see cref="Local"/> in the frame of whatever holds it, a body or a compound.
/// Placed shapes answer in that holder's frame. Pair queries take the placement of the other holder's frame in this holder's frame and compose the local transforms
/// (<c>bInShapeA = localA^-1 * otherInThis * localB</c>), so GJK, EPA, casts and the manifold generator run on the shapes' own frames and see nothing new.
/// A default value is invalid; make one with <see cref="Create(in ConvexShape3D, in RigidTransformF)"/>.
/// </summary>
[Experimental("OPHYS0001")]
public readonly struct PlacedShape3D
{
    /// <summary>Largest translation component of a local transform, 2^20; with a shape of up to 2^21 the bounds stay within what the broadphase trees accept.</summary>
    public const float MaxLocalOffset = 1048576f;

    private PlacedShape3D(in ConvexShape3D shape, in RigidTransformF local)
    {
        Shape = shape;
        Local = local;
    }

    /// <summary>The shape, in its own frame.</summary>
    public ConvexShape3D Shape { get; }

    /// <summary>Placement of the shape's frame in the holder's frame; the rotation is a unit quaternion.</summary>
    public RigidTransformF Local { get; }

    /// <summary>A shape at the holder's origin with no rotation.</summary>
    /// <exception cref="ShapeCompositionException"><paramref name="shape"/> is the point shape.</exception>
    public static PlacedShape3D Create(in ConvexShape3D shape) => Create(shape, RigidTransformF.Identity);

    /// <summary>A shape at a local transform.</summary>
    /// <param name="shape">Any shape except the zero-size point.</param>
    /// <param name="local">The rotation must have a squared length within 1% of 1 (it is normalised when stored); each translation component finite and at most <see cref="MaxLocalOffset"/>.</param>
    /// <exception cref="ShapeCompositionException"><paramref name="shape"/> is the point shape, or <paramref name="local"/> is not usable.</exception>
    public static PlacedShape3D Create(in ConvexShape3D shape, in RigidTransformF local)
    {
        if (shape.Kind == ConvexShapeKind3D.Sphere && shape.ConvexRadius == 0f)
        {
            throw new ShapeCompositionException(ShapeCompositionError.InvalidShape, "shape is the zero-size point shape; a sphere, box, capsule, cylinder or hull is expected.", nameof(shape));
        }

        CheckTransform(local, nameof(local), "local");
        return new PlacedShape3D(shape, new RigidTransformF(local.Rotation.Normalized(), local.Translation));
    }

    internal static void CheckTransform(in RigidTransformF t, string param, string what, float limit = MaxLocalOffset)
    {
        QuaternionF q = t.Rotation;
        float ql = (q.X * q.X) + (q.Y * q.Y) + (q.Z * q.Z) + (q.W * q.W);
        if (!(ql >= 0.99f && ql <= 1.01f))
        {
            throw new ShapeCompositionException(ShapeCompositionError.InvalidTransform, $"{what} has rotation {q} with squared length {ql}; a unit quaternion (squared length within 1% of 1) is expected.", param);
        }

        Vector3F p = t.Translation;
        if (!(Math.Abs(p.X) <= limit && Math.Abs(p.Y) <= limit && Math.Abs(p.Z) <= limit))
        {
            throw new ShapeCompositionException(ShapeCompositionError.InvalidTransform, $"{what} has translation {p}; finite components with magnitude at most {limit} are expected.", param);
        }
    }

    private void Require()
    {
        QuaternionF q = Local.Rotation;
        if (!((q.X * q.X) + (q.Y * q.Y) + (q.Z * q.Z) + (q.W * q.W) >= 0.99f))
        {
            throw new InvalidOperationException("This PlacedShape3D is a default value; make one with PlacedShape3D.Create.");
        }
    }

    /// <summary>The farthest point of the shape along a direction, in the holder's frame.</summary>
    /// <param name="direction">Any non-zero direction in the holder's frame.</param>
    public Vector3F Support(Vector3F direction)
    {
        Require();
        Vector3F inShape = Local.Rotation.Conjugated().Rotate(direction);
        return Local.TransformPoint(Shape.Support(inShape));
    }

    /// <summary>The padded bounds of the shape in the holder's frame (see <see cref="ShapeBounds3D"/>).</summary>
    public void GetBounds(out Vector3F min, out Vector3F max)
    {
        Require();
        ShapeBounds3D.Compute(Shape, Local, out min, out max);
    }

    /// <summary>Volume, centre of mass and inertia about the centre of mass at unit density, in the holder's frame: the shape's own properties with the centre moved and the tensor rotated.</summary>
    public MassPropertiesF GetMassProperties()
    {
        Require();
        MassPropertiesF m = Shape.GetMassProperties();
        return new MassPropertiesF(m.Mass, Local.TransformPoint(m.Center), InertiaF.Rotate(m.Inertia, Local.Rotation));
    }

    // Placement of the other shape's own frame in this shape's own frame.
    internal RigidTransformF ToOwnFrame(in PlacedShape3D other, in RigidTransformF otherInThis) => Local.Inverted() * otherInThis * other.Local;

    /// <summary>Distance and closest points to another placed shape (<see cref="Gjk3D.Distance"/>), in this holder's frame.</summary>
    /// <param name="other">The other shape.</param>
    /// <param name="otherInThis">Placement of the other shape's holder in this shape's holder.</param>
    public ConvexDistance3D Distance(in PlacedShape3D other, in RigidTransformF otherInThis)
    {
        Require();
        other.Require();
        ConvexDistance3D d = Gjk3D.Distance(Shape, other.Shape, ToOwnFrame(other, otherInThis));
        if (d.Normal.LengthSquared() == 0f)
        {
            return d;
        }

        return new ConvexDistance3D(d.Distance, d.Overlapping, Local.TransformVector(d.Normal), Local.TransformPoint(d.PointA), Local.TransformPoint(d.PointB), d.Iterations);
    }

    /// <summary>Signed separation, normal and contact points against another placed shape (<see cref="Epa3D.Collide"/>), in this holder's frame.</summary>
    /// <param name="epa">Working storage; not shared between threads.</param>
    /// <param name="other">The other shape.</param>
    /// <param name="otherInThis">Placement of the other shape's holder in this shape's holder.</param>
    public ConvexContact3D Collide(Epa3D epa, in PlacedShape3D other, in RigidTransformF otherInThis)
    {
        ArgumentNullException.ThrowIfNull(epa);
        Require();
        other.Require();
        ConvexContact3D c = epa.Collide(Shape, other.Shape, ToOwnFrame(other, otherInThis));
        return new ConvexContact3D(c.Separation, Local.TransformVector(c.Normal), Local.TransformPoint(c.PointA), Local.TransformPoint(c.PointB));
    }

    /// <summary>Sweeps this shape along a motion against another placed shape (<see cref="Gjk3D.ShapeCast"/>); the other stays still.</summary>
    /// <param name="other">The stationary shape.</param>
    /// <param name="otherInThis">Placement of the other shape's holder in this shape's holder at the start.</param>
    /// <param name="motion">Motion of this shape in this holder's frame; finite.</param>
    /// <param name="hit">The first contact in this holder's frame.</param>
    public bool ShapeCast(in PlacedShape3D other, in RigidTransformF otherInThis, Vector3F motion, out ConvexCastHit3D hit)
    {
        Require();
        other.Require();
        Vector3F inOwn = Local.Rotation.Conjugated().Rotate(motion);
        if (!Gjk3D.ShapeCast(Shape, other.Shape, ToOwnFrame(other, otherInThis), inOwn, out ConvexCastHit3D h))
        {
            hit = default;
            return false;
        }

        hit = new ConvexCastHit3D(h.Fraction, Local.TransformVector(h.Normal), Local.TransformPoint(h.PointA), Local.TransformPoint(h.PointB), h.StartsOverlapping);
        return true;
    }

    /// <summary>Casts a ray against the shape (<see cref="Gjk3D.RayCast"/>); everything is in the holder's frame.</summary>
    /// <param name="origin">Ray origin.</param>
    /// <param name="direction">Ray direction; its length scales the fraction.</param>
    /// <param name="maxFraction">Finite and positive.</param>
    /// <param name="hit">The first hit; the normal is the outward surface normal.</param>
    public bool RayCast(Vector3F origin, Vector3F direction, float maxFraction, out ConvexCastHit3D hit)
    {
        Require();
        return Gjk3D.RayCast(Shape, Local, origin, direction, maxFraction, out hit);
    }
}
