using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>
/// What a full separating-axis query learned about a touching pair of polyhedra, kept with the pair's contact between steps: how far the face axes lead the edge-pair axes
/// (<see cref="Gap"/>, zero when there is nothing to reuse) and the pose of B in A at that query (translation and the vector part of the rotation, with a non-negative scalar part).
/// Seven floats, stored in the spare tail of the contact payload, so they are part of snapshots and world hashes.
/// </summary>
internal readonly struct EdgeHint3D
{
    public readonly float Gap, Tx, Ty, Tz, Qx, Qy, Qz;

    public EdgeHint3D(float gap, in RigidTransformF pose)
    {
        Gap = gap;
        Tx = pose.Translation.X; Ty = pose.Translation.Y; Tz = pose.Translation.Z;
        QuaternionF q = pose.Rotation;
        float sgn = q.W < 0f ? -1f : 1f;
        Qx = sgn * q.X; Qy = sgn * q.Y; Qz = sgn * q.Z;
    }

    public EdgeHint3D(ReadOnlySpan<float> seven)
    {
        Gap = seven[0]; Tx = seven[1]; Ty = seven[2]; Tz = seven[3]; Qx = seven[4]; Qy = seven[5]; Qz = seven[6];
    }

    public void CopyTo(Span<float> seven)
    {
        seven[0] = Gap; seven[1] = Tx; seven[2] = Ty; seven[3] = Tz; seven[4] = Qx; seven[5] = Qy; seven[6] = Qz;
    }

    /// <summary>
    /// How far the relative pose has moved since the hint was made: the translation, plus the rotation angle (twice the vector part of the relative rotation, an upper bound for angles
    /// up to a right angle and still a monotone measure beyond) times <paramref name="reach"/>, the sum of the shapes' radii around their centres. Not finite if the stored pose is.
    /// </summary>
    public float PoseChange(in RigidTransformF now, float reach)
    {
        float dx = now.Translation.X - Tx, dy = now.Translation.Y - Ty, dz = now.Translation.Z - Tz;
        float move = ConvexSqrt(dx * dx + dy * dy + dz * dz);
        float w0 = ConvexSqrt(Math.Max(0f, 1f - (Qx * Qx + Qy * Qy + Qz * Qz)));
        QuaternionF q = now.Rotation;
        float sgn = q.W < 0f ? -1f : 1f;
        float x = sgn * q.X, y = sgn * q.Y, z = sgn * q.Z, w = sgn * q.W;
        // Vector part of q * conj(q0).
        float rx = w0 * x - w * Qx + (y * -Qz - z * -Qy);
        float ry = w0 * y - w * Qy + (z * -Qx - x * -Qz);
        float rz = w0 * z - w * Qz + (x * -Qy - y * -Qx);
        float turn = 2f * ConvexSqrt(rx * rx + ry * ry + rz * rz);
        return move + (turn * reach);
    }

    private static float ConvexSqrt(float v) => float.Sqrt(v);
}
