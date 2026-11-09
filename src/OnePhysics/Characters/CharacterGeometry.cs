using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Characters;

/// <summary>Small geometry shared by the 2D and 3D character abilities.</summary>
internal static class CharacterGeometry
{
    public static double Dot(Vector3D a, Vector3D b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;

    public static double Clamp01(double v) => v < 0.0 ? 0.0 : v > 1.0 ? 1.0 : v;

    /// <summary>Closest points of two segments (Ericson, Real-Time Collision Detection, 5.1.9): parameters s and t in [0,1] and the points.</summary>
    public static void SegmentSegment(Vector3D p1, Vector3D q1, Vector3D p2, Vector3D q2, out double s, out double t, out Vector3D c1, out Vector3D c2)
    {
        var d1 = q1 - p1; var d2 = q2 - p2; var r = p1 - p2;
        double a = Dot(d1, d1), e = Dot(d2, d2), f = Dot(d2, r);
        const double Eps = 1e-12;
        if (a <= Eps && e <= Eps) { s = t = 0.0; }
        else if (a <= Eps) { s = 0.0; t = Clamp01(f / e); }
        else
        {
            double cc = Dot(d1, r);
            if (e <= Eps) { t = 0.0; s = Clamp01(-cc / a); }
            else
            {
                double b = Dot(d1, d2), denom = a * e - b * b;
                s = denom > Eps * a * e ? Clamp01((b * f - cc * e) / denom) : 0.0;
                t = (b * s + f) / e;
                if (t < 0.0) { t = 0.0; s = Clamp01(-cc / a); }
                else if (t > 1.0) { t = 1.0; s = Clamp01((b - cc) / a); }
            }
        }

        c1 = p1 + d1 * s;
        c2 = p2 + d2 * t;
    }
}
