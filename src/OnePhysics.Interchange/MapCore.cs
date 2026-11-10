using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Interchange;

/// <summary>Geometry and property conversion shared by the Tiled and LDtk importers.</summary>
internal static class MapCore
{
    public const int MaxPolygonPoints = 8;

    public static void CheckOptions(MapOptions o, string format)
    {
        if (!(o.PixelsPerMeter > 0) || !double.IsFinite(o.PixelsPerMeter) || o.PixelsPerMeter > 1e6) { throw new ArgumentOutOfRangeException(nameof(o), "PixelsPerMeter must be within (0, 1e6]."); }
        if (!double.IsFinite(o.Origin.X) || !double.IsFinite(o.Origin.Y) || Math.Abs(o.Origin.X) > 2097152 || Math.Abs(o.Origin.Y) > 2097152) { throw new ArgumentOutOfRangeException(nameof(o), "Origin must be finite and within ±2^21."); }
        _ = format;
    }

    public static bool IsCollisionName(string name, MapOptions o)
    {
        if (name.Equals("collision", StringComparison.OrdinalIgnoreCase) || name.Equals("collisions", StringComparison.OrdinalIgnoreCase)) { return true; }
        foreach (var n in o.CollisionLayers) { if (name.Equals(n, StringComparison.OrdinalIgnoreCase)) { return true; } }
        return false;
    }

    public static bool IsTrue(IReadOnlyDictionary<string, string> p, string key) => p.TryGetValue(key, out var v) && v == "true";

    public static double Prop(IReadOnlyDictionary<string, string> p, string key, double fallback, string format, string where)
    {
        if (!p.TryGetValue(key, out var v)) { return fallback; }
        if (!double.TryParse(v, NumberStyles.Float, CultureInfo.InvariantCulture, out double d) || !double.IsFinite(d)) { throw new InterchangeException(format, where, "property '" + key + "' must be a number, found '" + Io.Clip(v) + "'."); }
        return d;
    }

    public static MapBodyKind? BodyKind(IReadOnlyDictionary<string, string> p, string format, string where)
    {
        if (!p.TryGetValue("body", out var v)) { return null; }
        return v switch
        {
            "static" => MapBodyKind.Static, "dynamic" => MapBodyKind.Dynamic, "kinematic" => MapBodyKind.Kinematic, "sensor" => MapBodyKind.Sensor,
            _ => throw new InterchangeException(format, where, "property 'body' must be static, dynamic, kinematic or sensor, found '" + Io.Clip(v) + "'."),
        };
    }

    public static void Material(MapObject2D o, IReadOnlyDictionary<string, string> p, string format, string where)
    {
        o.Properties = p;
        double density = Prop(p, "density", 1, format, where), friction = Prop(p, "friction", 0.6, format, where), rest = Prop(p, "restitution", 0, format, where);
        if (!(density > 0) || density > 1e9) { throw new InterchangeException(format, where, "property 'density' must be within (0, 1e9]."); }
        if (friction < 0 || friction > 1e6) { throw new InterchangeException(format, where, "property 'friction' must be within [0, 1e6]."); }
        if (rest < 0 || rest > 1) { throw new InterchangeException(format, where, "property 'restitution' must be within [0, 1]."); }
        o.Density = (float)density; o.Friction = (float)friction; o.Restitution = (float)rest; o.OneWay = IsTrue(p, "oneway");
    }

    /// <summary>Signed area of a polygon (positive counter-clockwise).</summary>
    public static double Area(IReadOnlyList<Vector2D> p)
    {
        double a = 0;
        for (int i = 0; i < p.Count; i++) { var u = p[i]; var v = p[(i + 1) % p.Count]; a += u.X * v.Y - v.X * u.Y; }
        return 0.5 * a;
    }

    public static bool IsConvex(IReadOnlyList<Vector2D> p, double eps)
    {
        int n = p.Count;
        for (int i = 0; i < n; i++)
        {
            var a = p[i]; var b = p[(i + 1) % n]; var c = p[(i + 2) % n];
            double cross = (b.X - a.X) * (c.Y - b.Y) - (b.Y - a.Y) * (c.X - b.X);
            if (cross < -eps) { return false; }
        }

        return true;
    }

    /// <summary>A polygon outline in object-frame metres (y up) as a shape: a convex polygon of up to 8 points, or, for a static body, a loop chain. Returns null with a warning when it can be neither.</summary>
    public static MapObject2D? PolygonObject(string name, string type, MapBodyKind body, Vector2D position, float rotation, List<Vector2D> pts, List<InterchangeWarning> warnings, string where)
    {
        // Duplicate consecutive points and a repeated closing point make no geometry.
        var clean = new List<Vector2D>();
        foreach (var p in pts) { if (clean.Count == 0 || Math.Abs(p.X - clean[^1].X) > 1e-12 || Math.Abs(p.Y - clean[^1].Y) > 1e-12) { clean.Add(p); } }
        if (clean.Count > 1 && Math.Abs(clean[0].X - clean[^1].X) <= 1e-12 && Math.Abs(clean[0].Y - clean[^1].Y) <= 1e-12) { clean.RemoveAt(clean.Count - 1); }
        if (clean.Count < 3 || Math.Abs(Area(clean)) < 1e-12) { warnings.Add(new InterchangeWarning("degenerate-polygon", where, "a polygon with fewer than three points or no area is left out.")); return null; }
        if (Area(clean) < 0) { clean.Reverse(); }
        var arr = new Vector2F[clean.Count];
        for (int i = 0; i < arr.Length; i++) { arr[i] = new Vector2F((float)clean[i].X, (float)clean[i].Y); }
        if (clean.Count <= MaxPolygonPoints && IsConvex(clean, 1e-12)) { return new MapObject2D(name, type, MapShapeKind.Polygon, body, position, rotation, default, 0, arr); }
        if (body is MapBodyKind.Static) { return new MapObject2D(name, type, MapShapeKind.Loop, body, position, rotation, default, 0, arr); }
        warnings.Add(new InterchangeWarning("concave-polygon", where, "a concave polygon, or one with more than " + MaxPolygonPoints + " points, is only supported on static bodies (as a loop of segments); it is left out."));
        return null;
    }

    public static List<Vector2D> Ellipse(double a, double b)
    {
        var r = new List<Vector2D>();
        for (int i = 0; i < 16; i++)
        {
            var (s, c) = RecursiveAI.OneMath.Math64.SinCos(2 * Math.PI * i / 16);
            r.Add(new Vector2D(a * c, b * s));
        }

        return r;
    }

    public static bool Near(double a, double b, double eps) => Math.Abs(a - b) <= eps;
}
