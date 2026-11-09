using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Cables;

/// <summary>
/// Tangent geometry of a taut rope over circular wheels (Euler–Eytelwein capstan geometry). A wheel is a circle in the plane perpendicular to its unit axis; the rope turns about the axis
/// in direction <c>side</c> (+1 counter-clockwise seen from the axis tip). The rope touches the rim at a point where its direction is tangent to the circle in the wheel's plane; out of plane it may
/// leave at a fleet angle. Everything uses IEEE-exact double arithmetic and square roots only.
/// </summary>
internal static class CableGeometry
{
    /// <summary>Tangent point on a wheel for a rope arriving from <paramref name="q"/> (incoming) or leaving towards it (outgoing). False when the point lies on or inside the rim circle seen along the axis.</summary>
    internal static bool PointToWheel(Vector3D q, Vector3D c, Vector3D n, double r, int side, bool incoming, out Vector3D p)
    {
        var w = q - c; w = w - n * Vector3D.Dot(n, w);
        double d2 = w.LengthSquared(), d = Math.Sqrt(d2);
        p = default;
        if (!(d > r * (1 + 1e-12))) return false;
        var wh = w / d;
        // Tangent from an outside point: the radial direction makes angle acos(r/d) with the direction to the point. With r cos(phi) = r*r/d and r sin(phi) = r*sqrt(d^2-r^2)/d.
        double cosPhi = r / d, sinPhi = Math.Sqrt((d - r) * (d + r)) / d;
        double sigma = incoming ? side : -side;
        var u = wh * cosPhi + Vector3D.Cross(n, wh) * (sigma * sinPhi);
        p = c + u * r;
        return true;
    }

    /// <summary>
    /// Tangent points of a rope leaving wheel A and arriving at wheel B. Exact for parallel axes (the common tangent of two circles in the plane perpendicular to the axis, with the axial offset of the
    /// centres carried along); for skew axes eight alternating point-to-wheel tangents converge to the line touching both rims.
    /// </summary>
    internal static bool WheelToWheel(Vector3D ca, Vector3D na, double ra, int sa, Vector3D cb, Vector3D nb, double rb, int sb, out Vector3D pa, out Vector3D pb)
    {
        pa = pb = default;
        double dotN = Vector3D.Dot(na, nb);
        if (Math.Abs(dotN) > 1 - 1e-9)
        {
            int sbe = dotN > 0 ? sb : -sb;
            var w = cb - ca; w = w - na * Vector3D.Dot(na, w);
            double d = w.Length();
            double a = sa * ra - sbe * rb;
            if (!(d > 1e-12) || !(Math.Abs(a) < d)) return false;
            var wh = w / d;
            double sinA = a / d, cosA = Math.Sqrt((d - a) * (d + a)) / d;
            var nw = Vector3D.Cross(na, wh);
            var t = wh * cosA + nw * sinA;
            var nt = Vector3D.Cross(na, t);
            pa = ca + nt * (-sa * ra);
            pb = cb + nt * (-sbe * rb);
            return true;
        }
        var q = cb;
        for (int i = 0; i < 8; i++)
        {
            if (!PointToWheel(q, ca, na, ra, sa, false, out pa)) return false;
            if (!PointToWheel(pa, cb, nb, rb, sb, true, out pb)) return false;
            q = pb;
        }
        return PointToWheel(pb, ca, na, ra, sa, false, out pa);
    }

    /// <summary>Two unit vectors perpendicular to <paramref name="n"/> and to each other (e1, then n × e1), chosen deterministically.</summary>
    internal static void Basis(Vector3F n, out Vector3F e1, out Vector3F e2)
    {
        var helper = Math.Abs(n.X) < 0.9f ? new Vector3F(1, 0, 0) : new Vector3F(0, 1, 0);
        var c = Vector3F.Cross(n, helper);
        float l = (float)Math.Sqrt((double)c.X * c.X + (double)c.Y * c.Y + (double)c.Z * c.Z);
        e1 = new Vector3F(c.X / l, c.Y / l, c.Z / l);
        e2 = Vector3F.Cross(n, e1);
    }
}
