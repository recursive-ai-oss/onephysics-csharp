using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Tests.Collision2D;

internal struct TRng
{
    private ulong _s;

    public TRng(ulong seed) { _s = seed * 0x9E3779B97F4A7C15UL + 0x1234567UL; if (_s == 0) { _s = 1; } }

    public uint Next()
    {
        _s ^= _s >> 12;
        _s ^= _s << 25;
        _s ^= _s >> 27;
        return (uint)((_s * 0x2545F4914F6CDD1DUL) >> 32);
    }

    public float Range(float lo, float hi) => lo + (hi - lo) * ((Next() >> 8) * (1f / 16777216f));

    public int Int(int n) => (int)(Next() % (uint)n);

    public Vec2 Point(float r) => new(Range(-r, r), Range(-r, r));

    public Rot2 Rot() => Rot2.FromVector(Range(-1f, 1f), Range(-1f, 1f));
}

internal static class TU
{
    public static Shape2D RandomPolygon(ref TRng r, float radius = 0f)
    {
        while (true)
        {
            var pts = new Vec2[3 + r.Int(8)];
            for (int i = 0; i < pts.Length; i++) { pts[i] = r.Point(1.5f); }
            try { return Shape2D.CreatePolygon(pts, radius); }
            catch (InvalidShapeException) { }
        }
    }

    // A random valid shape of any non-chain kind.
    public static Shape2D RandomShape(ref TRng r, bool allowRadius = true)
    {
        while (true)
        {
            try
            {
                switch (r.Int(5))
                {
                    case 0: return Shape2D.CreateCircle(r.Point(0.5f), r.Range(0.05f, 1f));
                    case 1: return Shape2D.CreateCapsule(r.Point(1f), r.Point(1f), r.Range(0.05f, 0.7f));
                    case 2: return Shape2D.CreateSegment(r.Point(1.5f), r.Point(1.5f));
                    case 3: return RandomPolygon(ref r, allowRadius && r.Int(3) == 0 ? r.Range(0.02f, 0.4f) : 0f);
                    default: return Shape2D.CreateBox(r.Range(0.1f, 1f), r.Range(0.1f, 1f), r.Point(0.3f), r.Rot(), allowRadius && r.Int(3) == 0 ? r.Range(0.02f, 0.3f) : 0f);
                }
            }
            catch (InvalidShapeException) { }
        }
    }

    public static Transform2 Pose(ref TRng r, float range) => new(r.Point(range), r.Rot());

    // Distance from point p to the segment ab.
    public static double PointSeg(double px, double py, double ax, double ay, double bx, double by)
    {
        double dx = bx - ax, dy = by - ay;
        double l2 = dx * dx + dy * dy;
        double t = l2 > 0 ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0;
        t = Math.Clamp(t, 0, 1);
        double cx = ax + t * dx - px, cy = ay + t * dy - py;
        return Math.Sqrt(cx * cx + cy * cy);
    }

    public static (double x, double y)[] World(in Shape2D s, Transform2 xf)
    {
        var res = new (double, double)[s.Count];
        for (int i = 0; i < s.Count; i++)
        {
            Vec2 p = xf.Apply(s.Vertices[i]);
            res[i] = (p.X, p.Y);
        }

        return res;
    }

    static bool SegmentsCross(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy)
    {
        double d1 = Cr(cx, cy, dx, dy, ax, ay), d2 = Cr(cx, cy, dx, dy, bx, by), d3 = Cr(ax, ay, bx, by, cx, cy), d4 = Cr(ax, ay, bx, by, dx, dy);
        return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
    }

    static double Cr(double ax, double ay, double bx, double by, double px, double py) => (bx - ax) * (py - ay) - (by - ay) * (px - ax);

    static bool InsidePoly((double x, double y)[] p, double x, double y)
    {
        if (p.Length < 3) { return false; }
        for (int i = 0; i < p.Length; i++)
        {
            var a = p[i];
            var b = p[(i + 1) % p.Length];
            if (Cr(a.x, a.y, b.x, b.y, x, y) < 0) { return false; }
        }

        return true;
    }

    // Brute-force distance between the cores (convex hulls of the vertex sets) of two shapes; 0 when they intersect. Independent of GJK: all vertex/edge pairs.
    public static double CoreDistance(in Shape2D a, in Shape2D b, Transform2 xfB)
    {
        var pa = World(in a, Transform2.Identity);
        var pb = World(in b, xfB);
        // Treat a 1-vertex or 2-vertex core as degenerate polygons.
        int na = pa.Length, nb = pb.Length;
        double best = double.MaxValue;
        foreach (var v in pa) { if (InsidePoly(pb, v.x, v.y)) { return 0; } }
        foreach (var v in pb) { if (InsidePoly(pa, v.x, v.y)) { return 0; } }
        int ea = na == 1 ? 0 : na == 2 ? 1 : na;
        int eb = nb == 1 ? 0 : nb == 2 ? 1 : nb;
        for (int i = 0; i < ea; i++)
        {
            var a1 = pa[i];
            var a2 = pa[(i + 1) % na];
            for (int j = 0; j < eb; j++)
            {
                var b1 = pb[j];
                var b2 = pb[(j + 1) % nb];
                if (SegmentsCross(a1.x, a1.y, a2.x, a2.y, b1.x, b1.y, b2.x, b2.y)) { return 0; }
            }
        }

        foreach (var v in pa)
        {
            if (nb == 1) { best = Math.Min(best, Math.Sqrt((v.x - pb[0].x) * (v.x - pb[0].x) + (v.y - pb[0].y) * (v.y - pb[0].y))); }
            for (int j = 0; j < eb; j++) { best = Math.Min(best, PointSeg(v.x, v.y, pb[j].x, pb[j].y, pb[(j + 1) % nb].x, pb[(j + 1) % nb].y)); }
        }

        foreach (var v in pb)
        {
            for (int i = 0; i < ea; i++) { best = Math.Min(best, PointSeg(v.x, v.y, pa[i].x, pa[i].y, pa[(i + 1) % na].x, pa[(i + 1) % na].y)); }
        }

        return best;
    }
}
