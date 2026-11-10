using System;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;

namespace RecursiveAI.OnePhysics.Interchange;

/// <summary>Volumes and inertia tensors of solid primitives, in double, about their own centres with their symmetry axes along the local axes (capsules, cylinders along z as MJCF writes them).</summary>
internal static class MassMath
{
    public const double Pi = 3.14159265358979323846;

    public static double SphereVolume(double r) => 4.0 / 3.0 * Pi * r * r * r;

    public static double CapsuleVolume(double r, double h) => Pi * r * r * 2 * h + SphereVolume(r);

    public static double CylinderVolume(double r, double h) => Pi * r * r * 2 * h;

    public static double EllipsoidVolume(double a, double b, double c) => 4.0 / 3.0 * Pi * a * b * c;

    public static Matrix3x3D Diagonal(double x, double y, double z) => new(x, 0, 0, 0, y, 0, 0, 0, z);

    public static Matrix3x3D Sphere(double m, double r) => Diagonal(0.4 * m * r * r, 0.4 * m * r * r, 0.4 * m * r * r);

    public static Matrix3x3D Box(double m, double hx, double hy, double hz) => Diagonal(m * (hy * hy + hz * hz) / 3, m * (hx * hx + hz * hz) / 3, m * (hx * hx + hy * hy) / 3);

    public static Matrix3x3D Ellipsoid(double m, double a, double b, double c) => Diagonal(m * (b * b + c * c) / 5, m * (a * a + c * c) / 5, m * (a * a + b * b) / 5);

    /// <summary>Cylinder along z with half length h.</summary>
    public static Matrix3x3D CylinderZ(double m, double r, double h) => Diagonal(m * (3 * r * r + 4 * h * h) / 12, m * (3 * r * r + 4 * h * h) / 12, 0.5 * m * r * r);

    /// <summary>Capsule along z: a cylinder of half length h and two hemispheres, mass shared by volume.</summary>
    public static Matrix3x3D CapsuleZ(double m, double r, double h)
    {
        double vs = SphereVolume(r), vc = Pi * r * r * 2 * h;
        double ms = m * vs / (vs + vc), mc = m - ms;
        // Hemisphere about its own centre of mass is 2/5 m r^2 - m (3r/8)^2; moved to the capsule's centre that gives m (2/5 r^2 + h^2 + 3 r h / 4) across the axis.
        double across = mc * (3 * r * r + 4 * h * h) / 12 + ms * (0.4 * r * r + h * h + 0.75 * r * h);
        double along = 0.5 * mc * r * r + 0.4 * ms * r * r;
        return Diagonal(across, across, along);
    }

    /// <summary>Volume, centroid and unit-density inertia about the origin of a closed triangle mesh (signed tetrahedra against the origin; Eberly's covariance formulas). Returns false when the mesh encloses no volume.</summary>
    public static bool Mesh(Vector3F[] v, int[] idx, double sx, double sy, double sz, out double volume, out Vector3D centroid, out Matrix3x3D inertiaAboutOrigin)
    {
        double vol = 0, cx = 0, cy = 0, cz = 0;
        double xx = 0, yy = 0, zz = 0, xy = 0, xz = 0, yz = 0;
        for (int t = 0; t + 2 < idx.Length; t += 3)
        {
            var a = v[idx[t]]; var b = v[idx[t + 1]]; var c = v[idx[t + 2]];
            double ax = a.X * sx, ay = a.Y * sy, az = a.Z * sz, bx = b.X * sx, by = b.Y * sy, bz = b.Z * sz, qx = c.X * sx, qy = c.Y * sy, qz = c.Z * sz;
            double d = ax * (by * qz - bz * qy) - ay * (bx * qz - bz * qx) + az * (bx * qy - by * qx);   // six times the tetrahedron's signed volume
            vol += d / 6;
            cx += d * (ax + bx + qx) / 24; cy += d * (ay + by + qy) / 24; cz += d * (az + bz + qz) / 24;
            // Second moments of a tetrahedron with one vertex at the origin: d/60 * (sum of squares + sum of pairwise products) per axis pair.
            xx += d / 60 * (ax * ax + bx * bx + qx * qx + ax * bx + ax * qx + bx * qx);
            yy += d / 60 * (ay * ay + by * by + qy * qy + ay * by + ay * qy + by * qy);
            zz += d / 60 * (az * az + bz * bz + qz * qz + az * bz + az * qz + bz * qz);
            xy += d / 120 * (2 * ax * ay + 2 * bx * by + 2 * qx * qy + ax * by + bx * ay + ax * qy + qx * ay + bx * qy + qx * by);
            xz += d / 120 * (2 * ax * az + 2 * bx * bz + 2 * qx * qz + ax * bz + bx * az + ax * qz + qx * az + bx * qz + qx * bz);
            yz += d / 120 * (2 * ay * az + 2 * by * bz + 2 * qy * qz + ay * bz + by * az + ay * qz + qy * az + by * qz + qy * bz);
        }

        volume = vol; centroid = default; inertiaAboutOrigin = default;
        if (!(Math.Abs(vol) > 1e-15)) { return false; }
        double sign = vol < 0 ? -1 : 1;
        volume = Math.Abs(vol);
        centroid = new Vector3D(cx / vol, cy / vol, cz / vol);
        xx *= sign; yy *= sign; zz *= sign; xy *= sign; xz *= sign; yz *= sign;
        inertiaAboutOrigin = new Matrix3x3D(yy + zz, -xy, -xz, -xy, xx + zz, -yz, -xz, -yz, xx + yy);
        return true;
    }
}
