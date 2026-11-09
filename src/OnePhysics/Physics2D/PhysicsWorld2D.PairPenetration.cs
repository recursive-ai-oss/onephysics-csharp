using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{

    /// <summary>Computes the minimum translation separating two convex shapes at world poses. Returns false for separated or exactly touching shapes; outputs then are zero. When true, moving B by direction * depth separates the pair to solver tolerance. Point is one contact point. Metres and radians, with normal world query pose limits; invalid input throws ArgumentException or ArgumentOutOfRangeException. No filtering or world state is involved.</summary>
    public static bool ComputePenetration(in Shape2D a, Vector2D positionA, float rotationA, in Shape2D b, Vector2D positionB, float rotationB, out Vector2F direction, out float depth, out Vector2D point)
    {
        CheckPosition(positionA, nameof(positionA)); CheckPosition(positionB, nameof(positionB));
        direction = default; depth = 0f; point = default;
        CheckShape(a); CheckShape(b);
        var qa = RotationOf(rotationA); var qb = RotationOf(rotationB);
        var boundsA = a.ComputeAabb(new Transform2(default, qa));
        var boundsB = b.ComputeAabb(new Transform2(default, qb));
        if (positionA.X + boundsA.Upper.X < positionB.X + boundsB.Lower.X || positionB.X + boundsB.Upper.X < positionA.X + boundsA.Lower.X ||
            positionA.Y + boundsA.Upper.Y < positionB.Y + boundsB.Lower.Y || positionB.Y + boundsB.Upper.Y < positionA.Y + boundsA.Lower.Y) { return false; }
        var relative = Transform2.InvMul(new Transform2(default, qa), new Transform2(new Vec2((float)(positionB.X - positionA.X), (float)(positionB.Y - positionA.Y)), qb));
        var contact = Collide2D.Collide(a, b, relative, 0f);
        if (contact.PointCount == 0) { return false; }
        var deepest = contact.Point0;
        for (int i = 1; i < contact.PointCount; i++) { if (contact[i].Separation < deepest.Separation) { deepest = contact[i]; } }
        if (!(deepest.Separation < 0f)) { return false; }
        var normal = qa.Rotate(contact.Normal);
        direction = new Vector2F(normal.X, normal.Y); depth = -deepest.Separation;
        var localPoint = qa.Rotate(deepest.Point);
        point = new Vector2D(positionA.X + localPoint.X, positionA.Y + localPoint.Y);
        return true;
    }
}
