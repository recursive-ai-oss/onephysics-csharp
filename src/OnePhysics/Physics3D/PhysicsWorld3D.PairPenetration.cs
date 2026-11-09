using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    [System.ThreadStatic] private static Epa3D? t_pairPenetration;

    /// <summary>Computes the minimum translation separating two convex shapes at world poses. Returns false for separated or exactly touching shapes; outputs then are zero. When true, moving B by direction * depth separates the pair to solver tolerance. Point is one contact point. Metres and radians, with normal world query pose limits; invalid input throws ArgumentException or ArgumentOutOfRangeException. No filtering or world state is involved.</summary>
    public static bool ComputePenetration(in ConvexShape3D a, Vector3D positionA, QuaternionF rotationA, in ConvexShape3D b, Vector3D positionB, QuaternionF rotationB, out Vector3F direction, out float depth, out Vector3D point)
    {
        CheckPosition(positionA, nameof(positionA)); CheckPosition(positionB, nameof(positionB));
        direction = default; depth = 0f; point = default;
        var qa = CheckRotation(rotationA); var qb = CheckRotation(rotationB);
        ShapeBounds3D.Compute(a, new RigidTransformF(qa, default), out var minA, out var maxA);
        ShapeBounds3D.Compute(b, new RigidTransformF(qb, default), out var minB, out var maxB);
        if (positionA.X + maxA.X < positionB.X + minB.X || positionB.X + maxB.X < positionA.X + minA.X ||
            positionA.Y + maxA.Y < positionB.Y + minB.Y || positionB.Y + maxB.Y < positionA.Y + minA.Y ||
            positionA.Z + maxA.Z < positionB.Z + minB.Z || positionB.Z + maxB.Z < positionA.Z + minA.Z) { return false; }
        var relative = new RigidTransformF(qa.Conjugated() * qb, qa.Conjugated().Rotate(new Vector3F((float)(positionB.X - positionA.X), (float)(positionB.Y - positionA.Y), (float)(positionB.Z - positionA.Z))));
        var contact = (t_pairPenetration ??= new Epa3D()).Collide(a, b, relative);
        if (!(contact.Separation < 0f)) { return false; }
        direction = qa.Rotate(contact.Normal); depth = -contact.Separation;
        var localPoint = qa.Rotate(contact.PointA);
        point = new Vector3D(positionA.X + localPoint.X, positionA.Y + localPoint.Y, positionA.Z + localPoint.Z);
        return true;
    }
}
