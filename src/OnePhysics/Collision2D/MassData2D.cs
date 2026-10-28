using System;

using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>
/// Mass properties of a shape per unit density, in the shape's frame. Everything is computed in double precision relative to a vertex of the shape,
/// so a shape far from the body origin keeps its accuracy; multiply by a density for physical values.
/// </summary>
public readonly struct MassData2D
{
    /// <summary>Area (mass per unit density).</summary>
    public double Area { get; }

    /// <summary>X of the area centroid.</summary>
    public double CentroidX { get; }

    /// <summary>Y of the area centroid.</summary>
    public double CentroidY { get; }

    /// <summary>Rotational inertia about the centroid, per unit density.</summary>
    public double InertiaAtCentroid { get; }

    internal MassData2D(double area, double cx, double cy, double inertiaAtCentroid)
    {
        Area = area;
        CentroidX = cx;
        CentroidY = cy;
        InertiaAtCentroid = inertiaAtCentroid;
    }

    /// <summary>The centroid as a float32 vector.</summary>
    public Vec2 Centroid => new((float)CentroidX, (float)CentroidY);

    /// <summary>Rotational inertia per unit density about another point (usually the body origin): the parallel-axis shift, <c>Ic + area · |p − c|²</c>, evaluated in double.</summary>
    public double InertiaAbout(double x, double y)
    {
        double dx = x - CentroidX, dy = y - CentroidY;
        return InertiaAtCentroid + Area * (dx * dx + dy * dy);
    }

    // Pieces are accumulated about a reference point O (the shape's first vertex): area, first moment, and polar moment of inertia.
    private struct Acc
    {
        public double A, Mx, My, J;

        // A piece given by its area, centroid (relative to O) and polar inertia about that centroid.
        public void AddAtCentroid(double area, double cx, double cy, double jc)
        {
            A += area;
            Mx += area * cx;
            My += area * cy;
            J += jc + area * (cx * cx + cy * cy);
        }

        // A piece given by its area, first moment and polar inertia about a point (ax, ay) relative to O.
        public void AddAboutPoint(double area, double mx, double my, double jp, double ax, double ay)
        {
            A += area;
            Mx += area * ax + mx;
            My += area * ay + my;
            J += jp + 2.0 * (ax * mx + ay * my) + area * (ax * ax + ay * ay);
        }
    }

    internal static MassData2D Compute(in Shape2D s)
    {
        if (s._count == 0) { return default; }
        double r = s._radius;
        switch (s._kind)
        {
            case ShapeKind2D.Circle:
            {
                double a = Math.PI * r * r;
                return new MassData2D(a, s._v[0].X, s._v[0].Y, 0.5 * a * r * r);
            }

            case ShapeKind2D.Segment:
            case ShapeKind2D.ChainSegment:
                return new MassData2D(0, s._centroid.X, s._centroid.Y, 0);
        }

        int n = s._count;
        double ox = s._v[0].X, oy = s._v[0].Y;
        var acc = new Acc();

        if (n >= 3)
        {
            // Fan of triangles (O, v_i, v_i+1): Catto, "Computing the mass of a polygon". The triangle's integrals are exact in the edge vectors.
            for (int i = 1; i < n - 1; i++)
            {
                double e1x = (double)s._v[i].X - ox, e1y = (double)s._v[i].Y - oy;
                double e2x = (double)s._v[i + 1].X - ox, e2y = (double)s._v[i + 1].Y - oy;
                double d = e1x * e2y - e1y * e2x;
                double ta = 0.5 * d;
                double intx2 = e1x * e1x + e2x * e1x + e2x * e2x;
                double inty2 = e1y * e1y + e2y * e2y + e2y * e1y;
                acc.A += ta;
                acc.Mx += ta * (e1x + e2x) / 3.0;
                acc.My += ta * (e1y + e2y) / 3.0;
                acc.J += d / 12.0 * (intx2 + inty2);
            }
        }

        if (r > 0.0)
        {
            // The rounded rim is the Minkowski sum with a disc: a rectangle r wide on every edge and a circular sector at every vertex
            // whose angle is the turn between the neighbouring edge normals.
            var px = new double[n];
            var py = new double[n];
            var nx = new double[n];
            var ny = new double[n];
            for (int i = 0; i < n; i++)
            {
                px[i] = (double)s._v[i].X - ox;
                py[i] = (double)s._v[i].Y - oy;
            }

            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n;
                double ex = px[j] - px[i], ey = py[j] - py[i];
                double len = Math.Sqrt(ex * ex + ey * ey);
                nx[i] = ey / len;
                ny[i] = -ex / len;
                double area = r * len;
                double cx = 0.5 * (px[i] + px[j]) + 0.5 * r * nx[i];
                double cy = 0.5 * (py[i] + py[j]) + 0.5 * r * ny[i];
                acc.AddAtCentroid(area, cx, cy, area * (len * len + r * r) / 12.0);
            }

            for (int i = 0; i < n; i++)
            {
                int p = (i + n - 1) % n;
                double theta = Angle(nx[p], ny[p], nx[i], ny[i]);
                double area = 0.5 * r * r * theta;
                double k = r * r * r / 3.0;
                double mx = k * (ny[i] - ny[p]);
                double my = k * (nx[p] - nx[i]);
                acc.AddAboutPoint(area, mx, my, 0.25 * r * r * r * r * theta, px[i], py[i]);
            }
        }

        if (!(acc.A > 0.0)) { return default; }
        double mcx = acc.Mx / acc.A, mcy = acc.My / acc.A;
        double ic = acc.J - acc.A * (mcx * mcx + mcy * mcy);
        return new MassData2D(acc.A, ox + mcx, oy + mcy, ic);
    }

    // The angle in [0, pi] between two unit vectors from the chord between them and repeated half-angle steps (Archimedes), using only
    // + - * / and sqrt, so it is identical everywhere. After 30 halvings the arc and chord agree to about 1e-19 relative.
    internal static double Angle(double ax, double ay, double bx, double by)
    {
        double dx = bx - ax, dy = by - ay;
        double c = Math.Sqrt(dx * dx + dy * dy);
        if (c > 2.0) { c = 2.0; }
        double scale = 1.0;
        for (int i = 0; i < 30; i++)
        {
            c /= Math.Sqrt(2.0 + Math.Sqrt(4.0 - c * c));
            scale *= 2.0;
        }

        return c * scale;
    }
}
