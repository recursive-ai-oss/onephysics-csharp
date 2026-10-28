using System;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;

namespace RecursiveAI.OnePhysics.Collision3D;

/// <summary>The kinds of support-mapped convex shape.</summary>
public enum ConvexShapeKind3D : byte
{
    /// <summary>A ball.</summary>
    Sphere,

    /// <summary>A box, optionally with rounded edges.</summary>
    Box,

    /// <summary>A cylinder capped by two hemispheres, axis along Y.</summary>
    Capsule,

    /// <summary>The convex hull of two spheres of different radii, axis along Y.</summary>
    TaperedCapsule,

    /// <summary>A cylinder with its axis along Y, optionally with rounded edges.</summary>
    Cylinder,

    /// <summary>A convex hull, optionally inflated by a radius.</summary>
    Hull,
}

/// <summary>
/// A convex shape described by its support mapping, in its own local frame centred on its origin (Gilbert, Johnson, Keerthi 1988;
/// van den Bergen 2003, ch. 4). Every shape is a <em>core</em> shape inflated by a <see cref="ConvexRadius"/>: queries run on the cores and add the radii back,
/// which keeps GJK well conditioned for rounded shapes and thin ones. Create shapes with the static factory methods.
/// </summary>
public readonly struct ConvexShape3D
{
    /// <summary>Largest dimension accepted for any shape, 2^21.</summary>
    public const float MaxDimension = 2097152f;

    private readonly Vector3F _size;
    private readonly float _radius;
    private readonly ConvexHull3D? _hull;

    private ConvexShape3D(ConvexShapeKind3D kind, Vector3F size, float radius, ConvexHull3D? hull)
    {
        Kind = kind;
        _size = size;
        _radius = radius;
        _hull = hull;
    }

    /// <summary>Which kind of shape this is. A default value is a zero-size sphere (a point).</summary>
    public ConvexShapeKind3D Kind { get; }

    /// <summary>The radius the core is inflated by, 0 for a sharp shape. For a sphere or capsule it is the whole radius; for a tapered capsule it is 0 because the support mapping carries both radii.</summary>
    public float ConvexRadius => _radius;

    // The shape's core dimensions as the factories store them: box half extents of the core; capsule (0, half height, 0); tapered capsule (top radius, bottom radius, half height);
    // cylinder (core radius, core half height, 0).
    internal Vector3F CoreSize => _size;

    /// <summary>The hull of a <see cref="ConvexShapeKind3D.Hull"/> shape, otherwise null.</summary>
    public ConvexHull3D? Hull => _hull;

    /// <summary>A zero-size shape: a point. Useful as the moving side of a ray cast.</summary>
    public static ConvexShape3D Point => default;

    /// <summary>Largest ratio of a shape's longest outer dimension to its shortest, 2^20. The cast and distance routines work in float32 with tolerances relative to the largest coordinate they see, so a shape much thinner than that is not resolved.</summary>
    public const float MaxAspectRatio = 1048576f;

    private static void RequireAspect(float longest, float shortest, string name)
    {
        if (!(longest <= shortest * MaxAspectRatio))
        {
            throw new ArgumentOutOfRangeException(name, shortest, $"The shape is {longest} long but only {shortest} thick (ratio above {MaxAspectRatio}); a thickness of at least 1/{MaxAspectRatio} of the length is expected.");
        }
    }

    private static void RequireDim(float value, string name, bool allowZero = false)
    {
        if (!float.IsFinite(value) || value > MaxDimension || (allowZero ? value < 0 : value <= 0))
        {
            throw new ArgumentOutOfRangeException(name, value, $"{name} is {value}; a finite value in {(allowZero ? "[0" : "(0")}, {MaxDimension}] is expected.");
        }
    }

    /// <summary>A sphere centred on the origin.</summary>
    /// <param name="radius">Radius, finite and positive, at most <see cref="MaxDimension"/>.</param>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="radius"/> is out of range.</exception>
    public static ConvexShape3D Sphere(float radius)
    {
        RequireDim(radius, nameof(radius));
        return new ConvexShape3D(ConvexShapeKind3D.Sphere, default, radius, null);
    }

    /// <summary>A box centred on the origin with edges along the axes.</summary>
    /// <param name="halfExtents">Half the size along each axis; each finite and positive.</param>
    /// <param name="convexRadius">Edge rounding radius, 0 for sharp; at most the smallest half extent. The outer size stays <paramref name="halfExtents"/>.</param>
    /// <exception cref="ArgumentOutOfRangeException">A parameter is out of range, or the shape is thinner than <see cref="MaxAspectRatio"/> allows for its length.</exception>
    public static ConvexShape3D Box(Vector3F halfExtents, float convexRadius = 0f)
    {
        RequireDim(halfExtents.X, "halfExtents.X");
        RequireDim(halfExtents.Y, "halfExtents.Y");
        RequireDim(halfExtents.Z, "halfExtents.Z");
        float m = Math.Min(halfExtents.X, Math.Min(halfExtents.Y, halfExtents.Z));
        RequireAspect(Math.Max(halfExtents.X, Math.Max(halfExtents.Y, halfExtents.Z)), m, nameof(halfExtents));
        if (!(convexRadius >= 0 && convexRadius <= m))
        {
            throw new ArgumentOutOfRangeException(nameof(convexRadius), convexRadius, $"convexRadius is {convexRadius}; a value in [0, {m}] (the smallest half extent) is expected.");
        }

        return new ConvexShape3D(ConvexShapeKind3D.Box, halfExtents - new Vector3F(convexRadius, convexRadius, convexRadius), convexRadius, null);
    }

    /// <summary>A capsule along Y centred on the origin: a cylinder of half height <paramref name="halfHeight"/> capped by hemispheres.</summary>
    /// <param name="halfHeight">Half the length of the cylindrical part; finite, positive.</param>
    /// <param name="radius">Radius; finite, positive.</param>
    /// <exception cref="ArgumentOutOfRangeException">A parameter is out of range, or the shape is thinner than <see cref="MaxAspectRatio"/> allows for its length.</exception>
    public static ConvexShape3D Capsule(float halfHeight, float radius)
    {
        RequireDim(halfHeight, nameof(halfHeight));
        RequireDim(radius, nameof(radius));
        RequireAspect(halfHeight + radius, radius, nameof(radius));
        return new ConvexShape3D(ConvexShapeKind3D.Capsule, new Vector3F(0, halfHeight, 0), radius, null);
    }

    /// <summary>The convex hull of two spheres on the Y axis, centred on the midpoint between their centres.</summary>
    /// <param name="halfHeight">Half the distance between the sphere centres; finite, positive.</param>
    /// <param name="topRadius">Radius of the sphere at +halfHeight; finite, positive.</param>
    /// <param name="bottomRadius">Radius of the sphere at -halfHeight; finite, positive.</param>
    /// <exception cref="ArgumentOutOfRangeException">A parameter is out of range, the shape is thinner than <see cref="MaxAspectRatio"/> allows, or one sphere contains the other (the radii differ by at least twice <paramref name="halfHeight"/>).</exception>
    public static ConvexShape3D TaperedCapsule(float halfHeight, float topRadius, float bottomRadius)
    {
        RequireDim(halfHeight, nameof(halfHeight));
        RequireDim(topRadius, nameof(topRadius));
        RequireDim(bottomRadius, nameof(bottomRadius));
        RequireAspect(halfHeight + Math.Max(topRadius, bottomRadius), Math.Min(topRadius, bottomRadius), nameof(halfHeight));
        if (!(Math.Abs((double)topRadius - bottomRadius) < 2.0 * halfHeight))
        {
            throw new ArgumentOutOfRangeException(nameof(bottomRadius), bottomRadius, $"The radii {topRadius} and {bottomRadius} differ by at least the centre distance {2.0 * halfHeight}, so one sphere contains the other; radii differing by less than twice halfHeight are expected.");
        }

        return new ConvexShape3D(ConvexShapeKind3D.TaperedCapsule, new Vector3F(topRadius, bottomRadius, halfHeight), 0f, null);
    }

    /// <summary>A cylinder along Y centred on the origin.</summary>
    /// <param name="halfHeight">Half the height; finite, positive.</param>
    /// <param name="radius">Radius; finite, positive.</param>
    /// <param name="convexRadius">Edge rounding radius, 0 for sharp; at most the smaller of the two. The outer size stays as given.</param>
    /// <exception cref="ArgumentOutOfRangeException">A parameter is out of range, or the shape is thinner than <see cref="MaxAspectRatio"/> allows for its length.</exception>
    public static ConvexShape3D Cylinder(float halfHeight, float radius, float convexRadius = 0f)
    {
        RequireDim(halfHeight, nameof(halfHeight));
        RequireDim(radius, nameof(radius));
        float m = Math.Min(halfHeight, radius);
        RequireAspect(Math.Max(halfHeight, radius), m, nameof(halfHeight));
        if (!(convexRadius >= 0 && convexRadius <= m))
        {
            throw new ArgumentOutOfRangeException(nameof(convexRadius), convexRadius, $"convexRadius is {convexRadius}; a value in [0, {m}] is expected.");
        }

        return new ConvexShape3D(ConvexShapeKind3D.Cylinder, new Vector3F(radius - convexRadius, halfHeight - convexRadius, 0), convexRadius, null);
    }

    /// <summary>A convex hull, optionally inflated.</summary>
    /// <param name="hull">The hull.</param>
    /// <param name="convexRadius">Outward inflation of the hull (a Minkowski sum with a sphere), 0 for sharp; finite, non-negative.</param>
    /// <exception cref="ArgumentNullException"><paramref name="hull"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="convexRadius"/> is out of range.</exception>
    public static ConvexShape3D FromHull(ConvexHull3D hull, float convexRadius = 0f)
    {
        ArgumentNullException.ThrowIfNull(hull);
        RequireDim(convexRadius, nameof(convexRadius), allowZero: true);
        return new ConvexShape3D(ConvexShapeKind3D.Hull, default, convexRadius, hull);
    }

    /// <summary>The farthest point of the core (without the convex radius) along a direction. Ties resolve the same way on every platform.</summary>
    /// <param name="direction">Any direction; its length does not matter. A zero direction gives a valid but unspecified point.</param>
    public Vector3F CoreSupport(Vector3F direction)
    {
        switch (Kind)
        {
            case ConvexShapeKind3D.Box:
                return new Vector3F(direction.X < 0 ? -_size.X : _size.X, direction.Y < 0 ? -_size.Y : _size.Y, direction.Z < 0 ? -_size.Z : _size.Z);
            case ConvexShapeKind3D.Capsule:
                return new Vector3F(0f, direction.Y < 0 ? -_size.Y : _size.Y, 0f);
            case ConvexShapeKind3D.TaperedCapsule:
                {
                    // The farther of the two sphere points c_i + r_i * d/|d|; compared without dividing: h*dy + r*|d|.
                    Vector3F u = Normalize(direction);
                    float len = Sqrt(u.LengthSquared());
                    if (!(len > 0f))
                    {
                        return new Vector3F(0f, _size.Z, 0f);
                    }

                    float top = _size.Z * u.Y + _size.X * len;
                    float bottom = -_size.Z * u.Y + _size.Y * len;
                    if (top >= bottom)
                    {
                        return new Vector3F(_size.X * u.X / len, _size.Z + (_size.X * u.Y / len), _size.X * u.Z / len);
                    }

                    return new Vector3F(_size.Y * u.X / len, -_size.Z + (_size.Y * u.Y / len), _size.Y * u.Z / len);
                }

            case ConvexShapeKind3D.Cylinder:
                {
                    Vector3F u = Normalize(direction);
                    float len = Sqrt((u.X * u.X) + (u.Z * u.Z));
                    float px = 0f, pz = 0f;
                    if (len > 0f)
                    {
                        px = _size.X * u.X / len;
                        pz = _size.X * u.Z / len;
                    }

                    return new Vector3F(px, direction.Y < 0 ? -_size.Y : _size.Y, pz);
                }

            case ConvexShapeKind3D.Hull:
                return _hull!.Support(direction);
            default:
                return default;
        }
    }

    /// <summary>The farthest point of the shape (core plus convex radius) along a direction.</summary>
    /// <param name="direction">Any non-zero direction.</param>
    public Vector3F Support(Vector3F direction)
    {
        Vector3F core = CoreSupport(direction);
        if (_radius == 0f)
        {
            return core;
        }

        Vector3F u = Normalize(direction);
        float len = Sqrt(u.LengthSquared());
        return len > 0f ? core + (u * (_radius / len)) : core;
    }

    /// <summary>The axis-aligned half extents of the shape in its own frame.</summary>
    public Vector3F HalfExtents => new(
        Support(Vector3F.UnitX).X,
        Support(Vector3F.UnitY).Y,
        Support(Vector3F.UnitZ).Z);

    /// <summary>Volume, centre of mass and inertia tensor about the centre of mass, for unit density, in the shape's frame.
    /// Box and cylinder with a convex radius report the unrounded outer shape; a hull reports the hull without its inflation radius; a tapered capsule is the exact hull of its two spheres.</summary>
    public MassPropertiesF GetMassProperties()
    {
        switch (Kind)
        {
            case ConvexShapeKind3D.Sphere:
                {
                    float vol = (float)(4.0 / 3.0 * Math.PI * _radius * _radius * _radius);
                    return new MassPropertiesF(vol, default, InertiaF.SolidSphere(vol, _radius));
                }

            case ConvexShapeKind3D.Box:
                {
                    Vector3F h = _size + new Vector3F(_radius, _radius, _radius);
                    float vol = 8f * h.X * h.Y * h.Z;
                    return new MassPropertiesF(vol, default, InertiaF.SolidBox(vol, h));
                }

            case ConvexShapeKind3D.Cylinder:
                {
                    float r = _size.X + _radius, hh = _size.Y + _radius;
                    float vol = (float)(Math.PI * r * r * 2.0 * hh);
                    return new MassPropertiesF(vol, default, InertiaF.SolidCylinder(vol, r, hh));
                }

            case ConvexShapeKind3D.Capsule:
                return Revolution(_size.Y, _radius, _radius);
            case ConvexShapeKind3D.TaperedCapsule:
                return Revolution(_size.Z, _size.X, _size.Y);
            case ConvexShapeKind3D.Hull:
                return _hull!.MassProperties;
            default:
                return default;
        }
    }

    // Solid of revolution about Y: spheres of radius rt (at +h) and rb (at -h) and their common tangent cone.
    // Each piece's profile rho(y) has rho^2 polynomial in y, so volume (pi*rho^2), first moment (pi*y*rho^2) and the inertia integrands
    // (pi*rho^4/4 + pi*rho^2*y^2, pi*rho^4/2) are polynomials of degree <= 4, integrated exactly by 3-point Gauss-Legendre quadrature in double.
    private static MassPropertiesF Revolution(float hf, float rtf, float rbf)
    {
        double h = hf, rt = rtf, rb = rbf;
        double d = 2.0 * h;
        double s = (rb - rt) / d;
        double c = Math.Sqrt(1.0 - (s * s));
        double y1 = -h + (rb * s), y2 = h + (rt * s);
        double rho1 = rb * c, rho2 = rt * c;
        double vol = 0, my = 0, i4 = 0, ixx = 0;

        void Piece(double y0, double yEnd, Func<double, double> rho2Fn)
        {
            // Gauss-Legendre, 3 points on [y0, yEnd].
            double mid = 0.5 * (y0 + yEnd), half = 0.5 * (yEnd - y0);
            double off = Math.Sqrt(0.6) * half;
            double[] ys = { mid - off, mid, mid + off };
            double[] ws = { 5.0 / 9.0 * half, 8.0 / 9.0 * half, 5.0 / 9.0 * half };
            for (int i = 0; i < 3; i++)
            {
                double r2 = rho2Fn(ys[i]);
                double w = ws[i] * Math.PI;
                vol += w * r2;
                my += w * r2 * ys[i];
                i4 += w * r2 * r2 * 0.5;
                ixx += w * ((r2 * r2 * 0.25) + (r2 * ys[i] * ys[i]));
            }
        }

        Piece(-h - rb, y1, y => (rb * rb) - ((y + h) * (y + h)));
        double slope = (rho2 - rho1) / (y2 - y1);
        Piece(y1, y2, y => { double r = rho1 + (slope * (y - y1)); return r * r; });
        Piece(y2, h + rt, y => (rt * rt) - ((y - h) * (y - h)));

        double yc = my / vol;
        double ix = ixx - (vol * yc * yc);
        return new MassPropertiesF((float)vol, new Vector3F(0f, (float)yc, 0f), new Matrix3x3F((float)ix, 0f, 0f, 0f, (float)i4, 0f, 0f, 0f, (float)ix));
    }

    internal static float Sqrt(float x) => (float)Math.Sqrt(x);

    // Scales a direction so its largest component has magnitude 1: keeps squares and square roots in range for tiny and huge inputs.
    internal static Vector3F Normalize(Vector3F d)
    {
        float m = Math.Max(Math.Abs(d.X), Math.Max(Math.Abs(d.Y), Math.Abs(d.Z)));
        if (!(m > 0f) || float.IsInfinity(m))
        {
            return m == 0f ? default : d;
        }

        return d / m;
    }
}
