using System;
using RecursiveAI.OneMath.Predicates;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>
/// The exact orientation and in-circle tests for any finite coordinates. The predicates of the math library are exact for coordinates that are zero or have a magnitude in
/// [2^-140, 2^200] and refuse others; mesh coordinates can be float32 subnormals and lattice points computed from them can be smaller still. A sign does not change when every
/// coordinate is multiplied by the same power of two, so out-of-range input is scaled (exactly) until its largest coordinate is just below 2^200. A coordinate that is then below
/// 2^-140, that is more than 340 binades under the largest one, is taken as zero: it moves the points by less than 2^-340 of the figure's size.
/// </summary>
internal static class SafePredicates
{
    private const int MaxExponent = 199;
    private static readonly double s_min = Pow2(-140), s_max = Pow2(200);

    public static int Orient2D(double ax, double ay, double bx, double by, double cx, double cy)
    {
        if (InRange(ax) && InRange(ay) && InRange(bx) && InRange(by) && InRange(cx) && InRange(cy))
        {
            return ExactPredicates.Orient2D(ax, ay, bx, by, cx, cy);
        }

        double s = ShiftFor(Math.Max(Math.Max(Math.Max(Math.Abs(ax), Math.Abs(ay)), Math.Max(Math.Abs(bx), Math.Abs(by))), Math.Max(Math.Abs(cx), Math.Abs(cy))), out double s2);
        return ExactPredicates.Orient2D(Fit(ax, s, s2), Fit(ay, s, s2), Fit(bx, s, s2), Fit(by, s, s2), Fit(cx, s, s2), Fit(cy, s, s2));
    }

    public static int InCircle(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy)
    {
        if (InRange(ax) && InRange(ay) && InRange(bx) && InRange(by) && InRange(cx) && InRange(cy) && InRange(dx) && InRange(dy))
        {
            return ExactPredicates.InCircle(ax, ay, bx, by, cx, cy, dx, dy);
        }

        double big = Math.Max(Math.Max(Math.Max(Math.Abs(ax), Math.Abs(ay)), Math.Max(Math.Abs(bx), Math.Abs(by))), Math.Max(Math.Max(Math.Abs(cx), Math.Abs(cy)), Math.Max(Math.Abs(dx), Math.Abs(dy))));
        double s = ShiftFor(big, out double s2);
        return ExactPredicates.InCircle(Fit(ax, s, s2), Fit(ay, s, s2), Fit(bx, s, s2), Fit(by, s, s2), Fit(cx, s, s2), Fit(cy, s, s2), Fit(dx, s, s2), Fit(dy, s, s2));
    }

    // Zero, or finite with a magnitude inside the exact range. NaN and infinity are out of range and then reach the library's own refusal.
    private static bool InRange(double v)
    {
        double m = Math.Abs(v);
        return m == 0.0 || (m >= s_min && m <= s_max);
    }

    // The scale 2^k that brings the largest magnitude just below 2^200, as two factors (each a power of two inside double range) whose product is 2^k. A non-finite or zero maximum scales by 1.
    private static double ShiftFor(double biggest, out double second)
    {
        second = 1.0;
        if (!(biggest > 0.0) || !double.IsFinite(biggest))
        {
            return 1.0;
        }

        int exp = (int)((BitConverter.DoubleToInt64Bits(biggest) >> 52) & 0x7FF) - 1023; // -1023 for subnormals, whose true exponent is lower; the product stays under 2^200 anyway
        int k = MaxExponent - exp;
        int k1 = k / 2, k2 = k - k1;
        second = Pow2(k2);
        return Pow2(k1);
    }

    private static double Pow2(int k) => BitConverter.Int64BitsToDouble((long)(k + 1023) << 52);

    private static double Fit(double v, double s, double s2)
    {
        if (!double.IsFinite(v))
        {
            return v;
        }

        double r = v * s * s2;
        return Math.Abs(r) < s_min ? 0.0 : r;
    }
}
