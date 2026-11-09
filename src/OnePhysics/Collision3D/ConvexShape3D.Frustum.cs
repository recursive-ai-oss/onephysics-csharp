using System;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;

namespace RecursiveAI.OnePhysics;

public readonly partial struct ConvexShape3D
{
    /// <summary>Creates a circular frustum along Y, centred geometrically at the origin. A zero end radius makes a cone; equal radii return the existing cylinder representation without changing its results.</summary>
    /// <param name="halfHeight">Half the height in metres, finite and positive, at most 2^21.</param>
    /// <param name="topRadius">Radius at +halfHeight in metres, finite and non-negative, at most 2^21.</param>
    /// <param name="bottomRadius">Radius at -halfHeight in metres, finite and non-negative, at most 2^21. At least one radius must be positive.</param>
    /// <exception cref="ArgumentOutOfRangeException">Dimensions are out of range or the longest to shortest nonzero extent exceeds 2^20.</exception>
    public static ConvexShape3D TaperedCylinder(float halfHeight, float topRadius, float bottomRadius)
    {
        RequireDim(halfHeight, nameof(halfHeight));
        RequireDim(topRadius, nameof(topRadius), true);
        RequireDim(bottomRadius, nameof(bottomRadius), true);
        float radius = Math.Max(topRadius, bottomRadius);
        RequireDim(radius, nameof(bottomRadius));
        RequireAspect(Math.Max(halfHeight, radius), Math.Min(halfHeight, radius), nameof(halfHeight));
        if (topRadius == bottomRadius) { return Cylinder(halfHeight, topRadius); }
        return new ConvexShape3D(ConvexShapeKind3D.TaperedCylinder, new Vector3F(topRadius, bottomRadius, halfHeight), 0f, null);
    }

    private Vector3F FrustumSupport(Vector3F direction)
    {
        Vector3F u = Normalize(direction);
        float rho = Sqrt(u.X * u.X + u.Z * u.Z);
        // Maximising a linear function over a frustum needs only the two rim circles. Top wins ties.
        bool top = _size.Z * u.Y + _size.X * rho >= -_size.Z * u.Y + _size.Y * rho;
        float radius = top ? _size.X : _size.Y;
        return new Vector3F(rho > 0f ? radius * u.X / rho : 0f, top ? _size.Z : -_size.Z, rho > 0f ? radius * u.Z / rho : 0f);
    }

    private MassPropertiesF FrustumMass()
    {
        // Integrate disks with r(t)=bottom+(top-bottom)*t, t in [0,1].
        // Integral r^2 dt, t*r^2 dt, t^2*r^2 dt and r^4 dt give volume, centroid and both principal moments.
        double b = _size.Y, t = _size.X, d = t - b, h = 2.0 * _size.Z;
        double a2 = (b * b + b * t + t * t) / 3.0;
        double b2 = b * b / 2.0 + 2.0 * b * d / 3.0 + d * d / 4.0;
        double c2 = b * b / 3.0 + b * d / 2.0 + d * d / 5.0;
        double a4 = (b * b * b * b + b * b * b * t + b * b * t * t + b * t * t * t + t * t * t * t) / 5.0;
        double volume = Math.PI * h * a2;
        double y = h * (b2 / a2 - 0.5);
        double axial = Math.PI * h * a4 / 2.0;
        double transverse = Math.PI * h * a4 / 4.0 + Math.PI * h * h * h * (c2 - b2 * b2 / a2);
        return new MassPropertiesF((float)volume, new Vector3F(0f, (float)y, 0f),
            new Matrix3x3F((float)transverse, 0f, 0f, 0f, (float)axial, 0f, 0f, 0f, (float)transverse));
    }
}
