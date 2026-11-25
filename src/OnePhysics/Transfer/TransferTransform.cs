using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics;

/// <summary>
/// The rigid transform an insertion applies to a packet: points go to <c>R p + t</c> (rotation about the world origin, then translation), vectors to <c>R v</c>, orientations to <c>R q</c>.
/// Positions are rounded onto the 2^-32 grid afterwards, so an inserted body is exactly on the grid like any other.
/// </summary>
internal sealed class TransferTransform
{
    public readonly int Dimension;
    /// <summary>True when the rotation is exactly the identity; vectors and orientations are then copied untouched (a product with the identity could turn a -0 into +0).</summary>
    public readonly bool IsIdentityRotation;
    private readonly double[] _t = new double[3];
    private readonly double[] _m = new double[9];
    private readonly float[] _q = new float[4];
    private readonly float _cos, _sin;

    private TransferTransform(int dimension, bool identity, float cos = 1f, float sin = 0f)
    {
        Dimension = dimension; IsIdentityRotation = identity; _cos = cos; _sin = sin;
    }

    public bool IsIdentity => IsIdentityRotation && _t[0] == 0.0 && _t[1] == 0.0 && _t[2] == 0.0;

    public static TransferTransform Create3D(Vector3D at, QuaternionF? rotation)
    {
        CheckTranslation(at.X, at.Y, at.Z, nameof(at));
        var q = rotation ?? QuaternionF.Identity;
        double x = q.X, y = q.Y, z = q.Z, w = q.W;
        double n2 = x * x + y * y + z * z + w * w;
        if (!double.IsFinite(n2) || !(n2 > 1e-30)) { throw new ArgumentOutOfRangeException(nameof(rotation), q, "The portal rotation must be a finite, non-zero quaternion."); }
        double inv = 1.0 / Math.Sqrt(n2);
        x *= inv; y *= inv; z *= inv; w *= inv;
        bool identity = q.X == 0f && q.Y == 0f && q.Z == 0f && q.W > 0f;
        var t = new TransferTransform(3, identity);
        t._t[0] = at.X; t._t[1] = at.Y; t._t[2] = at.Z;
        t._q[0] = (float)x; t._q[1] = (float)y; t._q[2] = (float)z; t._q[3] = (float)w;
        if (identity) { t._q[0] = 0f; t._q[1] = 0f; t._q[2] = 0f; t._q[3] = 1f; }
        var m = t._m;
        m[0] = 1.0 - 2.0 * (y * y + z * z); m[1] = 2.0 * (x * y - z * w); m[2] = 2.0 * (x * z + y * w);
        m[3] = 2.0 * (x * y + z * w); m[4] = 1.0 - 2.0 * (x * x + z * z); m[5] = 2.0 * (y * z - x * w);
        m[6] = 2.0 * (x * z - y * w); m[7] = 2.0 * (y * z + x * w); m[8] = 1.0 - 2.0 * (x * x + y * y);
        if (identity) { Array.Clear(m); m[0] = m[4] = m[8] = 1.0; }
        return t;
    }

    public static TransferTransform Create2D(Vector2D at, float angle)
    {
        CheckTranslation(at.X, at.Y, 0.0, nameof(at));
        if (!float.IsFinite(angle)) { throw new ArgumentOutOfRangeException(nameof(angle), angle, "The portal angle must be finite."); }
        var (s, c) = Math64.SinCos((double)angle);
        bool identity = angle == 0f;
        var t = new TransferTransform(2, identity, identity ? 1f : (float)c, identity ? 0f : (float)s);
        t._t[0] = at.X; t._t[1] = at.Y;
        double cd = identity ? 1.0 : c, sd = identity ? 0.0 : s;
        t._m[0] = cd; t._m[1] = -sd; t._m[3] = sd; t._m[4] = cd;
        return t;
    }

    private static void CheckTranslation(double x, double y, double z, string param)
    {
        if (!double.IsFinite(x) || !double.IsFinite(y) || !double.IsFinite(z) || Math.Abs(x) > 4194304.0 || Math.Abs(y) > 4194304.0 || Math.Abs(z) > 4194304.0)
        { throw new ArgumentOutOfRangeException(param, "(" + x.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ", " + y.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ", " + z.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ")", "The portal translation must be finite and within 2^22 per component."); }
    }

    /// <summary>The point <c>R p + t</c> on the 2^-32 grid. <paramref name="p"/> and <paramref name="o"/> have one lane per dimension and may be the same span.</summary>
    public void Position(ReadOnlySpan<double> p, Span<double> o)
    {
        int n = Dimension;
        if (IsIdentity) { if (!p.SequenceEqual(o)) { p.CopyTo(o); } return; }
        var r = new double[3];
        for (int i = 0; i < n; i++)
        {
            double s = IsIdentityRotation ? p[i] : 0.0;
            if (!IsIdentityRotation) { for (int k = 0; k < n; k++) { s += _m[i * 3 + k] * p[k]; } }
            r[i] = Grid(s + _t[i]);
        }

        for (int i = 0; i < n; i++) { o[i] = r[i]; }
    }

    private static double Grid(double x) => Math.Round(x * 4294967296.0) * (1.0 / 4294967296.0);

    /// <summary>The vector <c>R v</c>. Spans have one lane per dimension and may be the same.</summary>
    public void Vector(ReadOnlySpan<float> v, Span<float> o)
    {
        if (IsIdentityRotation) { if (!v.SequenceEqual(o)) { v.CopyTo(o); } return; }
        int n = Dimension;
        float r0 = 0f, r1 = 0f, r2 = 0f;
        for (int i = 0; i < n; i++)
        {
            double s = 0.0;
            for (int k = 0; k < n; k++) { s += _m[i * 3 + k] * (double)v[k]; }
            if (i == 0) { r0 = (float)s; } else if (i == 1) { r1 = (float)s; } else { r2 = (float)s; }
        }

        o[0] = r0; o[1] = r1; if (n == 3) { o[2] = r2; }
    }

    /// <summary>The angular velocity as seen after the rotation: the vector <c>R w</c> in 3D, unchanged in 2D.</summary>
    public void Angular(ReadOnlySpan<float> w, Span<float> o)
    {
        if (Dimension == 3) { Vector(w, o); } else if (!w.SequenceEqual(o)) { w.CopyTo(o); }
    }

    /// <summary>The orientation <c>R q</c>: a quaternion product in 3D, an angle sum on (cosine, sine) in 2D.</summary>
    public void Orientation(ReadOnlySpan<float> q, Span<float> o)
    {
        if (IsIdentityRotation) { if (!q.SequenceEqual(o)) { q.CopyTo(o); } return; }
        if (Dimension == 3)
        {
            float ax = _q[0], ay = _q[1], az = _q[2], aw = _q[3], bx = q[0], by = q[1], bz = q[2], bw = q[3];
            float x = aw * bx + ax * bw + ay * bz - az * by;
            float y = aw * by - ax * bz + ay * bw + az * bx;
            float z = aw * bz + ax * by - ay * bx + az * bw;
            float w = aw * bw - ax * bx - ay * by - az * bz;
            float inv = 1f / float.Sqrt(x * x + y * y + z * z + w * w);
            o[0] = x * inv; o[1] = y * inv; o[2] = z * inv; o[3] = w * inv;
        }
        else
        {
            float c = _cos * q[0] - _sin * q[1], s = _sin * q[0] + _cos * q[1];
            float inv = 1f / float.Sqrt(c * c + s * s);
            o[0] = c * inv; o[1] = s * inv;
        }
    }
}
