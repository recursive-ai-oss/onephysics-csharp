using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics;

/// <summary>
/// The quaternion and matrix operations the contact stage runs several times per pair, written out with exactly the operations and order of <see cref="QuaternionF.Rotate"/> and the quaternion product so the results
/// are the same bits, but marked for inlining: the runtime does not inline the library versions into the hot loops, and each call copied its structs through memory.
/// </summary>
internal static class QuaternionInline
{
    /// <summary>The same result as <see cref="QuaternionF.Rotate"/>: with <c>t = 2 (q.xyz x v)</c>, <c>(v + w t) + q.xyz x t</c>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3F RotateInline(this QuaternionF q, Vector3F v)
    {
        float tx = (q.Y * v.Z - q.Z * v.Y) * 2f, ty = (q.Z * v.X - q.X * v.Z) * 2f, tz = (q.X * v.Y - q.Y * v.X) * 2f;
        return new Vector3F(v.X + tx * q.W + (q.Y * tz - q.Z * ty), v.Y + ty * q.W + (q.Z * tx - q.X * tz), v.Z + tz * q.W + (q.X * ty - q.Y * tx));
    }

    /// <summary>The same result as <see cref="QuaternionF.ToMatrix"/>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix3x3F ToMatrixInline(this QuaternionF q)
    {
        float xx = q.X * q.X, yy = q.Y * q.Y, zz = q.Z * q.Z;
        float xy = q.X * q.Y, xz = q.X * q.Z, yz = q.Y * q.Z;
        float wx = q.W * q.X, wy = q.W * q.Y, wz = q.W * q.Z;
        return new Matrix3x3F(
            1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy),
            2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx),
            2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy));
    }

    /// <summary>The same result as <see cref="Matrix3x3F.Transform"/>: each component is <c>(Mr1*x + Mr2*y) + Mr3*z</c>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3F TransformInline(this Matrix3x3F m, Vector3F v) => new(
        m.M11 * v.X + m.M12 * v.Y + m.M13 * v.Z,
        m.M21 * v.X + m.M22 * v.Y + m.M23 * v.Z,
        m.M31 * v.X + m.M32 * v.Y + m.M33 * v.Z);

    /// <summary>The same result as <see cref="RigidTransformF.TransformPoint"/>: <c>Rotation.Rotate(p) + Translation</c>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3F TransformPointInline(this RigidTransformF t, Vector3F p) => t.Rotation.RotateInline(p) + t.Translation;

    /// <summary>The same result as the quaternion product <c>a * b</c>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static QuaternionF Times(this QuaternionF a, QuaternionF b) => new(
        a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
        a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X,
        a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W,
        a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z);
}
