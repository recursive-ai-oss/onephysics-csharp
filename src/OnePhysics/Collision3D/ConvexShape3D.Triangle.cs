using System;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;

namespace RecursiveAI.OnePhysics;

public readonly partial struct ConvexShape3D
{
    /// <summary>Creates a double-sided triangle, optionally inflated by a spherical radius. The uninflated core is a two-sided lamina. Its mass properties use unit area density and ignore inflation, as inflated hulls do.</summary>
    /// <param name="a">First vertex, in metres; finite components within ±2^21.</param>
    /// <param name="b">Second vertex.</param>
    /// <param name="c">Third vertex. Winding chooses the first face; collision is double-sided.</param>
    /// <param name="convexRadius">Spherical inflation in metres, finite in [0, 2^21], default zero.</param>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate or radius is out of range, or the triangle is degenerate or has aspect ratio above 2^20.</exception>
    public static ConvexShape3D Triangle(Vector3F a, Vector3F b, Vector3F c, float convexRadius = 0f)
    {
        CheckTriangleVertex(a, nameof(a)); CheckTriangleVertex(b, nameof(b)); CheckTriangleVertex(c, nameof(c));
        RequireDim(convexRadius, nameof(convexRadius), true);
        var ad = a.ToDouble(); var bd = b.ToDouble(); var cd = c.ToDouble();
        var cross = Vector3D.Cross(bd - ad, cd - ad);
        double twiceArea = double.Sqrt(cross.LengthSquared());
        double length2 = Math.Max((bd - ad).LengthSquared(), Math.Max((cd - bd).LengthSquared(), (ad - cd).LengthSquared()));
        if (!(twiceArea > 0) || twiceArea * MaxAspectRatio < length2)
        {
            throw new ArgumentOutOfRangeException(nameof(c), c, "Triangle vertices must form a nonzero area with aspect ratio at most 1048576.");
        }
        var normal = (cross / twiceArea).ToSingle();
        var center = (ad + bd + cd) / 3.0;
        // A uniform triangle's covariance is sum((vertex-centroid)*(vertex-centroid)^T)/12.
        var da = ad - center; var db = bd - center; var dc = cd - center;
        double mass = twiceArea * 0.5;
        double xx = mass * (da.X * da.X + db.X * db.X + dc.X * dc.X) / 12;
        double yy = mass * (da.Y * da.Y + db.Y * db.Y + dc.Y * dc.Y) / 12;
        double zz = mass * (da.Z * da.Z + db.Z * db.Z + dc.Z * dc.Z) / 12;
        double xy = -mass * (da.X * da.Y + db.X * db.Y + dc.X * dc.Y) / 12;
        double xz = -mass * (da.X * da.Z + db.X * db.Z + dc.X * dc.Z) / 12;
        double yz = -mass * (da.Y * da.Z + db.Y * db.Z + dc.Y * dc.Z) / 12;
        var inertia = new Matrix3x3F((float)(yy + zz), (float)xy, (float)xz, (float)xy, (float)(xx + zz), (float)yz, (float)xz, (float)yz, (float)(xx + yy));
        // Each edge has a reverse twin on the opposite face; both face loops are closed.
        var hull = new ConvexHull3D([a, b, c],
            [new(normal, Vector3F.Dot(normal, a), 0, 3), new(-normal, Vector3F.Dot(-normal, a), 1, 3)],
            [new(0, 2, 0), new(1, 5, 1), new(1, 4, 0), new(2, 1, 1), new(2, 0, 0), new(0, 3, 1)],
            [0, 2, 4], new MassPropertiesF((float)mass, center.ToSingle(), inertia));
        return new ConvexShape3D(ConvexShapeKind3D.Triangle, default, convexRadius, hull);
    }

    private static void CheckTriangleVertex(Vector3F value, string name)
    {
        if (!float.IsFinite(value.X) || !float.IsFinite(value.Y) || !float.IsFinite(value.Z) ||
            Math.Abs(value.X) > MaxDimension || Math.Abs(value.Y) > MaxDimension || Math.Abs(value.Z) > MaxDimension)
        {
            throw new ArgumentOutOfRangeException(name, value, "Triangle vertex components must be finite within ±2097152 metres.");
        }
    }
}
