using System;

namespace RecursiveAI.OnePhysics;

/// <summary>Closest-point distances for the soft-body surface search, in double precision (Ericson, Real-Time Collision Detection, 5.1).</summary>
internal static class SoftGeometry
{
    /// <summary>Squared distance from a point to a triangle.</summary>
    public static double PointTriangleSq(double px, double py, double pz, double ax, double ay, double az, double bx, double by, double bz, double cx, double cy, double cz)
    {
        double abx = bx - ax, aby = by - ay, abz = bz - az, acx = cx - ax, acy = cy - ay, acz = cz - az;
        double apx = px - ax, apy = py - ay, apz = pz - az;
        double d1 = abx * apx + aby * apy + abz * apz, d2 = acx * apx + acy * apy + acz * apz;
        if (d1 <= 0.0 && d2 <= 0.0) { return Sq(apx, apy, apz); }
        double bpx = px - bx, bpy = py - by, bpz = pz - bz;
        double d3 = abx * bpx + aby * bpy + abz * bpz, d4 = acx * bpx + acy * bpy + acz * bpz;
        if (d3 >= 0.0 && d4 <= d3) { return Sq(bpx, bpy, bpz); }
        double vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
        {
            double v = d1 / (d1 - d3);
            return Sq(px - (ax + v * abx), py - (ay + v * aby), pz - (az + v * abz));
        }

        double cpx = px - cx, cpy = py - cy, cpz = pz - cz;
        double d5 = abx * cpx + aby * cpy + abz * cpz, d6 = acx * cpx + acy * cpy + acz * cpz;
        if (d6 >= 0.0 && d5 <= d6) { return Sq(cpx, cpy, cpz); }
        double vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
        {
            double w = d2 / (d2 - d6);
            return Sq(px - (ax + w * acx), py - (ay + w * acy), pz - (az + w * acz));
        }

        double va = d3 * d6 - d5 * d4;
        if (va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0)
        {
            double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return Sq(px - (bx + w * (cx - bx)), py - (by + w * (cy - by)), pz - (bz + w * (cz - bz)));
        }

        double den = 1.0 / (va + vb + vc);
        double vv = vb * den, ww = vc * den;
        return Sq(px - (ax + abx * vv + acx * ww), py - (ay + aby * vv + acy * ww), pz - (az + abz * vv + acz * ww));
    }

    /// <summary>Squared distance between two segments.</summary>
    public static double SegmentSegmentSq(double p1x, double p1y, double p1z, double q1x, double q1y, double q1z, double p2x, double p2y, double p2z, double q2x, double q2y, double q2z)
    {
        double d1x = q1x - p1x, d1y = q1y - p1y, d1z = q1z - p1z, d2x = q2x - p2x, d2y = q2y - p2y, d2z = q2z - p2z;
        double rx = p1x - p2x, ry = p1y - p2y, rz = p1z - p2z;
        double a = d1x * d1x + d1y * d1y + d1z * d1z, e = d2x * d2x + d2y * d2y + d2z * d2z, f = d2x * rx + d2y * ry + d2z * rz;
        double s, t;
        if (a <= 1e-30 && e <= 1e-30) { return Sq(rx, ry, rz); }
        if (a <= 1e-30) { s = 0.0; t = Clamp01(f / e); }
        else
        {
            double c = d1x * rx + d1y * ry + d1z * rz;
            if (e <= 1e-30) { t = 0.0; s = Clamp01(-c / a); }
            else
            {
                double b = d1x * d2x + d1y * d2y + d1z * d2z, den = a * e - b * b;
                s = den > 1e-30 ? Clamp01((b * f - c * e) / den) : 0.0;
                t = (b * s + f) / e;
                if (t < 0.0) { t = 0.0; s = Clamp01(-c / a); }
                else if (t > 1.0) { t = 1.0; s = Clamp01((b - c) / a); }
            }
        }

        return Sq(rx + d1x * s - d2x * t, ry + d1y * s - d2y * t, rz + d1z * s - d2z * t);
    }

    /// <summary>Squared distance from a point to a segment in the plane.</summary>
    public static double PointSegmentSq(double px, double py, double ax, double ay, double bx, double by)
    {
        double ex = bx - ax, ey = by - ay, l2 = ex * ex + ey * ey;
        double t = l2 > 0.0 ? Clamp01(((px - ax) * ex + (py - ay) * ey) / l2) : 0.0;
        double dx = px - (ax + t * ex), dy = py - (ay + t * ey);
        return dx * dx + dy * dy;
    }

    private static double Clamp01(double v) => v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);

    private static double Sq(double x, double y, double z) => x * x + y * y + z * z;
}
