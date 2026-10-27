using System;
using System.Globalization;

namespace RecursiveAI.OnePhysics.Broadphase;

internal static class Slab
{
    // Clips [t0, t1] to the part of the line o + t d inside [lo, hi]. A direction too small to invert is treated as parallel to the slab.
    // A box bound can never produce 0 * infinity here: the parallel case is decided before any division result is used.
    internal static bool Clip(double o, double d, double inv, double lo, double hi, ref double t0, ref double t1)
    {
        if (!double.IsFinite(inv) || d == 0.0)
        {
            return o >= lo && o <= hi;
        }

        double a = (lo - o) * inv, b = (hi - o) * inv;
        if (a > b) { (a, b) = (b, a); }
        if (a > t0) { t0 = a; }
        if (b < t1) { t1 = b; }
        return t0 <= t1;
    }

    internal static void CheckFinite(double v, string name)
    {
        if (!double.IsFinite(v))
        {
            throw new ArgumentOutOfRangeException(name, v, name + " must be finite, got " + v.ToString("R", CultureInfo.InvariantCulture) + ".");
        }
    }
}

/// <summary>A 2D ray: the points <c>origin + t * direction</c> for t in [0, maxFraction]. The direction need not be a unit vector; fractions are in units of the direction.</summary>
public readonly struct Ray2D : IRay<Aabb2D>
{
    private readonly double _ox, _oy, _dx, _dy, _ix, _iy;

    /// <summary>Creates a ray. Throws <see cref="ArgumentOutOfRangeException"/> if any component is not finite. A zero direction is a point query.</summary>
    public Ray2D(double originX, double originY, double directionX, double directionY)
    {
        Slab.CheckFinite(originX, nameof(originX));
        Slab.CheckFinite(originY, nameof(originY));
        Slab.CheckFinite(directionX, nameof(directionX));
        Slab.CheckFinite(directionY, nameof(directionY));
        _ox = originX;
        _oy = originY;
        _dx = directionX;
        _dy = directionY;
        _ix = 1.0 / directionX;
        _iy = 1.0 / directionY;
    }

    /// <inheritdoc/>
    public bool Hit(in Aabb2D box, double maxFraction, out double enter)
    {
        double t0 = 0.0, t1 = maxFraction;
        bool hit = Slab.Clip(_ox, _dx, _ix, box.MinX, box.MaxX, ref t0, ref t1) && Slab.Clip(_oy, _dy, _iy, box.MinY, box.MaxY, ref t0, ref t1);
        enter = t0;
        return hit;
    }
}

/// <summary>A 3D ray: the points <c>origin + t * direction</c> for t in [0, maxFraction]. The direction need not be a unit vector.</summary>
public readonly struct Ray3D : IRay<Aabb3D>
{
    private readonly double _ox, _oy, _oz, _dx, _dy, _dz, _ix, _iy, _iz;

    /// <summary>Creates a ray. Throws <see cref="ArgumentOutOfRangeException"/> if any component is not finite. A zero direction is a point query.</summary>
    public Ray3D(double originX, double originY, double originZ, double directionX, double directionY, double directionZ)
    {
        Slab.CheckFinite(originX, nameof(originX));
        Slab.CheckFinite(originY, nameof(originY));
        Slab.CheckFinite(originZ, nameof(originZ));
        Slab.CheckFinite(directionX, nameof(directionX));
        Slab.CheckFinite(directionY, nameof(directionY));
        Slab.CheckFinite(directionZ, nameof(directionZ));
        _ox = originX;
        _oy = originY;
        _oz = originZ;
        _dx = directionX;
        _dy = directionY;
        _dz = directionZ;
        _ix = 1.0 / directionX;
        _iy = 1.0 / directionY;
        _iz = 1.0 / directionZ;
    }

    /// <inheritdoc/>
    public bool Hit(in Aabb3D box, double maxFraction, out double enter)
    {
        double t0 = 0.0, t1 = maxFraction;
        bool hit = Slab.Clip(_ox, _dx, _ix, box.MinX, box.MaxX, ref t0, ref t1)
            && Slab.Clip(_oy, _dy, _iy, box.MinY, box.MaxY, ref t0, ref t1)
            && Slab.Clip(_oz, _dz, _iz, box.MinZ, box.MaxZ, ref t0, ref t1);
        enter = t0;
        return hit;
    }
}

/// <summary>
/// A 2D box swept along a translation (a shape cast's broadphase test): the start box's centre travels along the translation and every tree box is grown by the start box's half extents
/// (the Minkowski sum), so the sweep reduces to a ray test. The fraction is in units of the translation.
/// </summary>
public readonly struct BoxCast2D : IRay<Aabb2D>
{
    private readonly Ray2D _ray;
    private readonly double _hx, _hy;

    /// <summary>Creates the sweep of <paramref name="start"/> by (<paramref name="translationX"/>, <paramref name="translationY"/>). Throws <see cref="ArgumentOutOfRangeException"/> for an invalid box or a non-finite translation.</summary>
    public BoxCast2D(in Aabb2D start, double translationX, double translationY)
    {
        if (!start.IsValid(out string reason)) { throw new ArgumentOutOfRangeException(nameof(start), reason); }
        _hx = (start.MaxX - start.MinX) * 0.5;
        _hy = (start.MaxY - start.MinY) * 0.5;
        _ray = new Ray2D(start.MinX + _hx, start.MinY + _hy, translationX, translationY);
    }

    /// <inheritdoc/>
    public bool Hit(in Aabb2D box, double maxFraction, out double enter) =>
        _ray.Hit(new Aabb2D(box.MinX - _hx, box.MinY - _hy, box.MaxX + _hx, box.MaxY + _hy), maxFraction, out enter);
}

/// <summary>A 3D box swept along a translation; see <see cref="BoxCast2D"/>.</summary>
public readonly struct BoxCast3D : IRay<Aabb3D>
{
    private readonly Ray3D _ray;
    private readonly double _hx, _hy, _hz;

    /// <summary>Creates the sweep of <paramref name="start"/> by the translation. Throws <see cref="ArgumentOutOfRangeException"/> for an invalid box or a non-finite translation.</summary>
    public BoxCast3D(in Aabb3D start, double translationX, double translationY, double translationZ)
    {
        if (!start.IsValid(out string reason)) { throw new ArgumentOutOfRangeException(nameof(start), reason); }
        _hx = (start.MaxX - start.MinX) * 0.5;
        _hy = (start.MaxY - start.MinY) * 0.5;
        _hz = (start.MaxZ - start.MinZ) * 0.5;
        _ray = new Ray3D(start.MinX + _hx, start.MinY + _hy, start.MinZ + _hz, translationX, translationY, translationZ);
    }

    /// <inheritdoc/>
    public bool Hit(in Aabb3D box, double maxFraction, out double enter) =>
        _ray.Hit(new Aabb3D(box.MinX - _hx, box.MinY - _hy, box.MinZ - _hz, box.MaxX + _hx, box.MaxY + _hy, box.MaxZ + _hz), maxFraction, out enter);
}
