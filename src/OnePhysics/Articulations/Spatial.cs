using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Articulations;

/// <summary>A double-precision 3-vector for the articulation maths. Only IEEE-exact operations; every operation is spelled out so the order of additions is fixed.</summary>
internal struct D3
{
    public double X, Y, Z;

    public D3(double x, double y, double z) { X = x; Y = y; Z = z; }

    public static D3 operator +(D3 a, D3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
    public static D3 operator -(D3 a, D3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
    public static D3 operator -(D3 a) => new(-a.X, -a.Y, -a.Z);
    public static D3 operator *(D3 a, double s) => new(a.X * s, a.Y * s, a.Z * s);
    public static D3 operator *(double s, D3 a) => new(a.X * s, a.Y * s, a.Z * s);

    public static double Dot(D3 a, D3 b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;
    public static D3 Cross(D3 a, D3 b) => new(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
    public readonly double Length => Math.Sqrt(X * X + Y * Y + Z * Z);
    public static D3 Zero => default;
}

/// <summary>A row-major 3×3 matrix of doubles.</summary>
internal struct M3
{
    public double A, B, C, D, E, F, G, H, I;   // rows (A B C) (D E F) (G H I)

    public M3(double a, double b, double c, double d, double e, double f, double g, double h, double i) { A = a; B = b; C = c; D = d; E = e; F = f; G = g; H = h; I = i; }

    public static readonly M3 Identity = new(1, 0, 0, 0, 1, 0, 0, 0, 1);

    public static M3 operator *(M3 p, M3 q) => new(
        p.A * q.A + p.B * q.D + p.C * q.G, p.A * q.B + p.B * q.E + p.C * q.H, p.A * q.C + p.B * q.F + p.C * q.I,
        p.D * q.A + p.E * q.D + p.F * q.G, p.D * q.B + p.E * q.E + p.F * q.H, p.D * q.C + p.E * q.F + p.F * q.I,
        p.G * q.A + p.H * q.D + p.I * q.G, p.G * q.B + p.H * q.E + p.I * q.H, p.G * q.C + p.H * q.F + p.I * q.I);

    public static D3 operator *(M3 m, D3 v) => new(m.A * v.X + m.B * v.Y + m.C * v.Z, m.D * v.X + m.E * v.Y + m.F * v.Z, m.G * v.X + m.H * v.Y + m.I * v.Z);

    public readonly M3 Transposed() => new(A, D, G, B, E, H, C, F, I);

    /// <summary>The cross-product matrix: <c>Skew(a) * b = a × b</c>.</summary>
    public static M3 Skew(D3 a) => new(0, -a.Z, a.Y, a.Z, 0, -a.X, -a.Y, a.X, 0);

    public static M3 Outer(D3 a, D3 b) => new(a.X * b.X, a.X * b.Y, a.X * b.Z, a.Y * b.X, a.Y * b.Y, a.Y * b.Z, a.Z * b.X, a.Z * b.Y, a.Z * b.Z);

    public static M3 operator +(M3 p, M3 q) => new(p.A + q.A, p.B + q.B, p.C + q.C, p.D + q.D, p.E + q.E, p.F + q.F, p.G + q.G, p.H + q.H, p.I + q.I);
    public static M3 operator *(M3 p, double s) => new(p.A * s, p.B * s, p.C * s, p.D * s, p.E * s, p.F * s, p.G * s, p.H * s, p.I * s);
}

/// <summary>A unit quaternion (x, y, z, w) in doubles with the operations the articulation maths needs.</summary>
internal struct Q4
{
    public double X, Y, Z, W;

    public Q4(double x, double y, double z, double w) { X = x; Y = y; Z = z; W = w; }

    public static readonly Q4 Identity = new(0, 0, 0, 1);

    public static Q4 operator *(Q4 a, Q4 b) => new(
        a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
        a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X,
        a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W,
        a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z);

    public readonly Q4 Conjugated() => new(-X, -Y, -Z, W);

    public readonly Q4 Normalized()
    {
        double l2 = X * X + Y * Y + Z * Z + W * W;
        if (!(l2 > 1e-300)) { return Identity; }
        double inv = 1.0 / Math.Sqrt(l2);
        return new Q4(X * inv, Y * inv, Z * inv, W * inv);
    }

    public readonly M3 ToMatrix()
    {
        double xx = X * X, yy = Y * Y, zz = Z * Z, xy = X * Y, xz = X * Z, yz = Y * Z, wx = W * X, wy = W * Y, wz = W * Z;
        return new M3(1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy), 2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx), 2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy));
    }

    public readonly D3 Rotate(D3 v)
    {
        double tx = 2 * (Y * v.Z - Z * v.Y), ty = 2 * (Z * v.X - X * v.Z), tz = 2 * (X * v.Y - Y * v.X);
        return new D3(v.X + W * tx + (Y * tz - Z * ty), v.Y + W * ty + (Z * tx - X * tz), v.Z + W * tz + (X * ty - Y * tx));
    }

    /// <summary>The rotation by the vector <paramref name="w"/> (axis times angle, radians): the exponential map, with sine and cosine from onemath.</summary>
    public static Q4 Exp(D3 w)
    {
        double a = w.Length;
        if (a < 1e-9) { return new Q4(w.X * 0.5, w.Y * 0.5, w.Z * 0.5, 1.0).Normalized(); }
        var (s, c) = Math64.SinCos(a * 0.5);
        double k = s / a;
        return new Q4(w.X * k, w.Y * k, w.Z * k, c);
    }

    public static Q4 FromAxisAngle(D3 axis, double angle)
    {
        var (s, c) = Math64.SinCos(angle * 0.5);
        return new Q4(axis.X * s, axis.Y * s, axis.Z * s, c);
    }

    /// <summary>Rotation vector (axis times angle) of this rotation along the shorter arc, small-angle safe: <c>2·atan2(|v|, w)/|v|·v</c>.</summary>
    public readonly D3 Log()
    {
        double sg = W < 0 ? -1.0 : 1.0;
        var v = new D3(X * sg, Y * sg, Z * sg);
        double n = v.Length;
        double w = W * sg;
        if (n < 1e-12) { return v * 2.0; }
        double ang = 2.0 * Math64.Atan2(n, w);
        return v * (ang / n);
    }
}

/// <summary>Small dense-matrix and 6×6 spatial helpers on spans: row-major, every loop in a fixed order.</summary>
internal static class Dense
{
    /// <summary>c = a · b for 6×6 matrices.</summary>
    public static void Mul66(ReadOnlySpan<double> a, ReadOnlySpan<double> b, Span<double> c)
    {
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                double s = 0;
                for (int k = 0; k < 6; k++) { s += a[i * 6 + k] * b[k * 6 + j]; }
                c[i * 6 + j] = s;
            }
        }
    }

    /// <summary>c = aᵀ · b for 6×6 matrices.</summary>
    public static void MulT66(ReadOnlySpan<double> a, ReadOnlySpan<double> b, Span<double> c)
    {
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                double s = 0;
                for (int k = 0; k < 6; k++) { s += a[k * 6 + i] * b[k * 6 + j]; }
                c[i * 6 + j] = s;
            }
        }
    }

    /// <summary>out = a · v for a 6×6 matrix and a 6-vector.</summary>
    public static void Mul6(ReadOnlySpan<double> a, ReadOnlySpan<double> v, Span<double> o)
    {
        for (int i = 0; i < 6; i++)
        {
            double s = 0;
            for (int k = 0; k < 6; k++) { s += a[i * 6 + k] * v[k]; }
            o[i] = s;
        }
    }

    /// <summary>out = aᵀ · v.</summary>
    public static void MulT6(ReadOnlySpan<double> a, ReadOnlySpan<double> v, Span<double> o)
    {
        for (int i = 0; i < 6; i++)
        {
            double s = 0;
            for (int k = 0; k < 6; k++) { s += a[k * 6 + i] * v[k]; }
            o[i] = s;
        }
    }

    /// <summary>
    /// Motion transform from a parent frame to a child frame (Featherstone 2008, 2.8): <c>[E 0; −E r̃ E]</c> with <paramref name="e"/> taking parent coordinates to child coordinates and
    /// <paramref name="r"/> the child's origin in parent coordinates. Spatial vectors are (angular, linear).
    /// </summary>
    public static void BuildX(M3 e, D3 r, Span<double> x)
    {
        var er = e * M3.Skew(r);
        x[0] = e.A; x[1] = e.B; x[2] = e.C; x[3] = 0; x[4] = 0; x[5] = 0;
        x[6] = e.D; x[7] = e.E; x[8] = e.F; x[9] = 0; x[10] = 0; x[11] = 0;
        x[12] = e.G; x[13] = e.H; x[14] = e.I; x[15] = 0; x[16] = 0; x[17] = 0;
        x[18] = -er.A; x[19] = -er.B; x[20] = -er.C; x[21] = e.A; x[22] = e.B; x[23] = e.C;
        x[24] = -er.D; x[25] = -er.E; x[26] = -er.F; x[27] = e.D; x[28] = e.E; x[29] = e.F;
        x[30] = -er.G; x[31] = -er.H; x[32] = -er.I; x[33] = e.G; x[34] = e.H; x[35] = e.I;
    }

    /// <summary>Spatial inertia about the link origin from mass, centre of mass and the inertia tensor about the centre of mass, all in link axes (Featherstone 2008, 2.62).</summary>
    public static void BuildInertia(double m, D3 c, M3 ic, Span<double> o)
    {
        var cs = M3.Skew(c);
        var cc = cs * cs.Transposed();
        var ibar = ic + cc * m;
        var mc = cs * m;
        o[0] = ibar.A; o[1] = ibar.B; o[2] = ibar.C; o[3] = mc.A; o[4] = mc.B; o[5] = mc.C;
        o[6] = ibar.D; o[7] = ibar.E; o[8] = ibar.F; o[9] = mc.D; o[10] = mc.E; o[11] = mc.F;
        o[12] = ibar.G; o[13] = ibar.H; o[14] = ibar.I; o[15] = mc.G; o[16] = mc.H; o[17] = mc.I;
        // lower-left block is m c̃ᵀ = −m c̃
        o[18] = -mc.A; o[19] = -mc.B; o[20] = -mc.C; o[21] = m; o[22] = 0; o[23] = 0;
        o[24] = -mc.D; o[25] = -mc.E; o[26] = -mc.F; o[27] = 0; o[28] = m; o[29] = 0;
        o[30] = -mc.G; o[31] = -mc.H; o[32] = -mc.I; o[33] = 0; o[34] = 0; o[35] = m;
    }

    /// <summary>Spatial motion cross product v × m for (angular, linear) vectors.</summary>
    public static void CrossMotion(ReadOnlySpan<double> v, ReadOnlySpan<double> m, Span<double> o)
    {
        var w = new D3(v[0], v[1], v[2]); var vl = new D3(v[3], v[4], v[5]);
        var wm = new D3(m[0], m[1], m[2]); var vm = new D3(m[3], m[4], m[5]);
        var a = D3.Cross(w, wm);
        var b = D3.Cross(w, vm) + D3.Cross(vl, wm);
        o[0] = a.X; o[1] = a.Y; o[2] = a.Z; o[3] = b.X; o[4] = b.Y; o[5] = b.Z;
    }

    /// <summary>Spatial force cross product v ×* f (the dual): <c>(ω × n + v × f, ω × f)</c>.</summary>
    public static void CrossForce(ReadOnlySpan<double> v, ReadOnlySpan<double> f, Span<double> o)
    {
        var w = new D3(v[0], v[1], v[2]); var vl = new D3(v[3], v[4], v[5]);
        var n = new D3(f[0], f[1], f[2]); var fl = new D3(f[3], f[4], f[5]);
        var a = D3.Cross(w, n) + D3.Cross(vl, fl);
        var b = D3.Cross(w, fl);
        o[0] = a.X; o[1] = a.Y; o[2] = a.Z; o[3] = b.X; o[4] = b.Y; o[5] = b.Z;
    }

    /// <summary>Inverts a symmetric positive-definite n×n matrix in place by Gauss–Jordan elimination without pivoting; false when a pivot is not positive.</summary>
    public static bool InvertSpd(Span<double> m, int n)
    {
        for (int k = 0; k < n; k++)
        {
            double p = m[k * n + k];
            if (!(p > 0.0) || !double.IsFinite(p)) { return false; }
            double inv = 1.0 / p;
            m[k * n + k] = 1.0;
            for (int j = 0; j < n; j++) { m[k * n + j] *= inv; }
            for (int i = 0; i < n; i++)
            {
                if (i == k) { continue; }
                double f = m[i * n + k];
                m[i * n + k] = 0.0;
                if (f == 0.0) { continue; }
                for (int j = 0; j < n; j++) { m[i * n + j] -= f * m[k * n + j]; }
            }
        }

        return true;
    }

    /// <summary>In-place Cholesky factorization of a symmetric positive-definite n×n matrix (lower triangle holds L); false when it is not positive definite.</summary>
    public static bool Cholesky(Span<double> a, int n)
    {
        for (int j = 0; j < n; j++)
        {
            double d = a[j * n + j];
            for (int k = 0; k < j; k++) { d -= a[j * n + k] * a[j * n + k]; }
            if (!(d > 0.0) || !double.IsFinite(d)) { return false; }
            double ljj = Math.Sqrt(d);
            a[j * n + j] = ljj;
            double inv = 1.0 / ljj;
            for (int i = j + 1; i < n; i++)
            {
                double s = a[i * n + j];
                for (int k = 0; k < j; k++) { s -= a[i * n + k] * a[j * n + k]; }
                a[i * n + j] = s * inv;
            }
        }

        return true;
    }

    /// <summary>Solves L Lᵀ x = b in place given the Cholesky factor in the lower triangle of <paramref name="l"/>.</summary>
    public static void CholeskySolve(ReadOnlySpan<double> l, int n, Span<double> x)
    {
        for (int i = 0; i < n; i++)
        {
            double s = x[i];
            for (int k = 0; k < i; k++) { s -= l[i * n + k] * x[k]; }
            x[i] = s / l[i * n + i];
        }

        for (int i = n - 1; i >= 0; i--)
        {
            double s = x[i];
            for (int k = i + 1; k < n; k++) { s -= l[k * n + i] * x[k]; }
            x[i] = s / l[i * n + i];
        }
    }
}
