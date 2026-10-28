using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

// Fuzz targets for the 2D collision code. The bytes are read as shape parameters: most values are small numbers, some are NaN, infinities, huge or tiny,
// so hostile input reaches the constructors. The only exception allowed is InvalidShapeException from a constructor; every number that comes out must be finite.
internal static class Fuzz2D
{
    private static int s_polygons, s_pairs, s_contacts;

    static Fuzz2D()
    {
        AppDomain.CurrentDomain.ProcessExit += (_, _) => Console.WriteLine("2d targets: polygons built " + s_polygons + ", shape pairs built " + s_pairs + ", pairs with contact points " + s_contacts);
    }

    private sealed class Reader
    {
        private readonly byte[] _d;
        private int _i;

        public Reader(byte[] d) { _d = d; }

        public byte Byte() => _i < _d.Length ? _d[_i++] : (byte)0;

        public float Float()
        {
            byte b = Byte();
            switch (b)
            {
                case 250: return float.NaN;
                case 251: return float.PositiveInfinity;
                case 252: return float.NegativeInfinity;
                case 253: return 3e6f;
                case 254: return 1e-9f;
                case 255: return -0f;
            }

            int v = (short)(b | (Byte() << 8));
            return v / 512f;
        }

        public Vec2 Vec() => new(Float(), Float());
    }

    private static void Finite(float v, string what)
    {
        if (!float.IsFinite(v)) { throw new InvalidOperationException(what + " is not finite: " + v); }
    }

    private static void CheckShape(in Shape2D s)
    {
        for (int i = 0; i < s.Count; i++)
        {
            Finite(s.Vertices[i].X + s.Vertices[i].Y, "vertex");
        }

        MassData2D m = s.ComputeMass();
        Finite((float)m.Area, "area");
        Finite((float)m.InertiaAtCentroid, "inertia");
        Aabb2 bb = s.ComputeAabb();
        Finite(bb.Lower.X + bb.Lower.Y + bb.Upper.X + bb.Upper.Y, "aabb");
        if (s.Kind == ShapeKind2D.Polygon)
        {
            if (s.Count < 3 || s.Count > Constants2D.MaxPolygonVertices) { throw new InvalidOperationException("polygon vertex count " + s.Count); }
            for (int i = 0; i < s.Count; i++)
            {
                Vec2 a = s.Vertices[i], b = s.Vertices[(i + 1) % s.Count], c = s.Vertices[(i + 2) % s.Count];
                if (!(Vec2.Cross(b - a, c - b) > 0f)) { throw new InvalidOperationException("polygon is not strictly convex and counter-clockwise"); }
            }

            if (!(m.Area > 0.0)) { throw new InvalidOperationException("polygon area " + m.Area); }
        }
    }

    public static void PolygonTarget(byte[] data)
    {
        var r = new Reader(data);
        int n = r.Byte() % 24;
        float radius = (r.Byte() & 1) == 0 ? 0f : Math.Abs(r.Float());
        var pts = new Vec2[n];
        for (int i = 0; i < n; i++) { pts[i] = r.Vec(); }
        try
        {
            Shape2D s = Shape2D.CreatePolygon(pts, radius);
            CheckShape(in s);
            s_polygons++;
        }
        catch (InvalidShapeException) { }
    }

    private static Shape2D? MakeShape(Reader r)
    {
        try
        {
            switch (r.Byte() % 6)
            {
                case 0: return Shape2D.CreateCircle(r.Vec(), Math.Abs(r.Float()));
                case 1: return Shape2D.CreateCapsule(r.Vec(), r.Vec(), Math.Abs(r.Float()));
                case 2: return Shape2D.CreateSegment(r.Vec(), r.Vec());
                case 3: return Shape2D.CreateBox(Math.Abs(r.Float()), Math.Abs(r.Float()), r.Vec(), Rot2.FromVector(r.Float(), r.Float()), (r.Byte() & 1) == 0 ? 0f : Math.Abs(r.Float()));
                case 4:
                {
                    int n = 3 + r.Byte() % 5;
                    var pts = new Vec2[n];
                    for (int i = 0; i < n; i++) { pts[i] = r.Vec(); }
                    return Shape2D.CreatePolygon(pts, (r.Byte() & 1) == 0 ? 0f : Math.Abs(r.Float()));
                }

                default:
                {
                    Vec2? g1 = (r.Byte() & 1) == 0 ? null : r.Vec();
                    Vec2 p1 = r.Vec(), p2 = r.Vec();
                    Vec2? g2 = (r.Byte() & 1) == 0 ? null : r.Vec();
                    return Shape2D.CreateChainSegment(g1, p1, p2, g2, (r.Byte() & 1) == 0);
                }
            }
        }
        catch (InvalidShapeException)
        {
            return null;
        }
    }

    public static void ManifoldTarget(byte[] data)
    {
        var r = new Reader(data);
        if (MakeShape(r) is not Shape2D a || MakeShape(r) is not Shape2D b) { return; }
        CheckShape(in a);
        CheckShape(in b);
        s_pairs++;
        var xf = new Transform2(r.Vec(), Rot2.FromVector(r.Float(), r.Float()));
        float margin = (r.Byte() & 1) == 0 ? Constants2D.SpeculativeDistance : Math.Abs(r.Float());

        Manifold2D m = Collide2D.Collide(in a, in b, xf, margin);
        if (m.PointCount < 0 || m.PointCount > 2) { throw new InvalidOperationException("point count " + m.PointCount); }
        if (m.PointCount > 0)
        {
            s_contacts++;
            Finite(m.Normal.X + m.Normal.Y, "normal");
            float len = m.Normal.Length;
            if (!(len > 0.999f && len < 1.001f)) { throw new InvalidOperationException("normal length " + len); }
        }

        for (int i = 0; i < m.PointCount; i++)
        {
            Finite(m[i].Point.X + m[i].Point.Y, "point");
            Finite(m[i].Separation, "separation");
        }

        var cache = new SimplexCache2D();
        DistanceOutput2D d = Distance2D.Compute(in a, in b, xf, ref cache);
        Finite(d.Distance + d.PointA.X + d.PointA.Y + d.PointB.X + d.PointB.Y, "distance output");

        Vec2 o = r.Vec(), t = r.Vec();
        RayCastOutput2D rc = a.RayCast(o, t);
        Finite(rc.Fraction + rc.Point.X + rc.Point.Y + rc.Normal.X + rc.Normal.Y, "ray cast output");
        RayCastOutput2D sc = Distance2D.ShapeCast(in a, in b, xf, t);
        Finite(sc.Fraction + sc.Point.X + sc.Point.Y + sc.Normal.X + sc.Normal.Y, "shape cast output");

        var sa = new Sweep2D(a.Centroid, default, r.Vec(), Rot2.Identity, Rot2.FromVector(r.Float(), r.Float()));
        var sb = new Sweep2D(b.Centroid, xf.P, r.Vec(), xf.Q, Rot2.FromVector(r.Float(), r.Float()));
        ToiOutput2D toi = Distance2D.TimeOfImpact(in a, in sa, in b, in sb);
        Finite(toi.Fraction, "time of impact");
    }
}
