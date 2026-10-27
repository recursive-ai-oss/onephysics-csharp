using System;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// Sine, cosine and arctangent built only from <c>+ - * /</c>, <c>sqrt</c> and <c>floor</c>, so every platform gets the same bits. Angles are reduced to a quarter turn and summed as
/// a Taylor series in double precision, which is accurate to about 1e-15 for angles of a few thousand radians and far better than the float32 results they are rounded to.
/// They exist for turning an angle in a body or shape definition into a rotation; the solver itself only works with (cos, sin) pairs.
/// </summary>
internal static class Trig2D
{
    private const double PiOver2Hi = 1.5707963267948966;
    private const double PiOver2Lo = 6.123233995736766e-17;
    private const double TwoOverPi = 0.6366197723675814;

    /// <summary>Sine and cosine of <paramref name="x"/> radians (a finite value; larger than about 1e9 loses accuracy).</summary>
    public static void SinCos(double x, out double sin, out double cos)
    {
        double k = Math.Floor(x * TwoOverPi + 0.5);
        double r = (x - k * PiOver2Hi) - k * PiOver2Lo;
        double r2 = r * r;

        // sin r = r (1 - r^2/3! (1 - r^2/(4*5) (1 - ...))), evaluated from the inside out.
        double s = 1.0;
        for (int n = 17; n >= 3; n -= 2) { s = 1.0 - r2 / ((double)n * (n - 1)) * s; }
        s *= r;
        double c = 1.0;
        for (int n = 16; n >= 2; n -= 2) { c = 1.0 - r2 / ((double)n * (n - 1)) * c; }

        int q = (int)(((long)k) & 3);
        switch (q)
        {
            case 0: sin = s; cos = c; break;
            case 1: sin = c; cos = -s; break;
            case 2: sin = -s; cos = -c; break;
            default: sin = -c; cos = s; break;
        }
    }

    /// <summary>Arctangent of y/x in (-pi, pi], like the usual atan2; (0, 0) gives 0.</summary>
    public static double Atan2(double y, double x)
    {
        if (x == 0.0 && y == 0.0) { return 0.0; }
        double ax = Math.Abs(x), ay = Math.Abs(y);
        bool swap = ay > ax;
        double t = swap ? ax / ay : ay / ax; // in [0, 1]
        // atan t = 2 atan(t / (1 + sqrt(1 + t^2))) shrinks the argument to at most tan(pi/8) before the series.
        double u = t / (1.0 + Math.Sqrt(1.0 + t * t));
        double u2 = u * u;
        double sum = 0.0;
        for (int n = 41; n >= 1; n -= 2) { sum = 1.0 / n - u2 * sum; }
        double a = 2.0 * u * sum; // atan t in [0, pi/4]
        if (swap) { a = PiOver2Hi - a; }
        if (x < 0.0) { a = 3.141592653589793 - a; }
        return y < 0.0 ? -a : a;
    }
}
