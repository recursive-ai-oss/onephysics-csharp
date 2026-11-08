using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    internal Vector3D InspectionDampingTorque(int b, Vector3F delta)
    {
        if (delta.X == 0f && delta.Y == 0f && delta.Z == 0f) { return default; }
        var q = RotationOfBody(b);
        var local = q.Conjugated().Rotate(delta);
        int i = b * 6;
        var momentum = new Vector3F(Inertia[i] * local.X + Inertia[i + 1] * local.Y + Inertia[i + 2] * local.Z,
            Inertia[i + 1] * local.X + Inertia[i + 3] * local.Y + Inertia[i + 4] * local.Z,
            Inertia[i + 2] * local.X + Inertia[i + 4] * local.Y + Inertia[i + 5] * local.Z);
        var world = q.Rotate(momentum);
        return new(world.X, world.Y, world.Z);
    }
}
