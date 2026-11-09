using System;
using System.Collections.Generic;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>
/// Covers the inside of closed contour loops (outer loops counter-clockwise, holes clockwise: the orientation <see cref="SdfGrid2D.ExtractContours"/> returns) with convex polygons of at most
/// <see cref="Constants2D.MaxPolygonVertices"/> vertices that do not overlap and tile the region exactly.
/// Holes are joined to their enclosing loop by a bridge to a visible vertex (Eberly, Triangulation by Ear Clipping), the result is ear-clipped, and neighbouring triangles are then merged across
/// diagonals while the union stays convex (Hertel and Mehlhorn 1983). Every step walks indices in ascending order, so the same loops give the same pieces on every platform.
/// </summary>
internal static class ConvexCover2D
{
    /// <summary>Returns the pieces, each a counter-clockwise convex vertex list. Throws ArgumentOutOfRangeException when the loops hold more than <paramref name="maxVertices"/> vertices in total.</summary>
    public static List<Vec2[]> Cover(IReadOnlyList<Vec2[]> loops, int maxVertices = 8192)
    {
        int total = 0; foreach (var l in loops) total += l.Length;
        if (total > maxVertices) throw new ArgumentOutOfRangeException(nameof(loops), total, "The contour has more than " + maxVertices + " vertices; use a larger tolerance or a coarser field.");
        var outers = new List<int>(); var holes = new List<int>(); var area = new double[loops.Count];
        for (int i = 0; i < loops.Count; i++) { area[i] = SignedArea(loops[i]); if (area[i] > 0) outers.Add(i); else if (area[i] < 0) holes.Add(i); }
        // Each hole belongs to the smallest outer loop that contains one of its vertices (ties on index).
        var holesOf = new List<int>[loops.Count];
        foreach (int h in holes)
        {
            int best = -1;
            foreach (int o in outers)
                if (Inside(loops[o], loops[h][0]) && (best < 0 || area[o] < area[best])) best = o;
            if (best >= 0) (holesOf[best] ??= new List<int>()).Add(h);
        }
        var pieces = new List<Vec2[]>();
        foreach (int o in outers)
        {
            var poly = new List<Vec2>(loops[o]);
            if (holesOf[o] != null)
            {
                // Bridge the holes from the one with the largest rightmost x down, as the ear clipping paper does.
                var order = holesOf[o]; var keys = new double[order.Count];
                for (int k = 0; k < order.Count; k++) keys[k] = Rightmost(loops[order[k]], out _);
                var idx = new int[order.Count]; for (int k = 0; k < idx.Length; k++) idx[k] = k;
                Array.Sort(idx, (a, b) => keys[a] != keys[b] ? keys[b].CompareTo(keys[a]) : order[a].CompareTo(order[b]));
                foreach (int k in idx) Bridge(poly, loops[order[k]]);
            }
            var tris = EarClip(poly);
            Merge(poly, tris, pieces);
        }
        return pieces;
    }

    private static double SignedArea(Vec2[] p)
    {
        double a = 0; for (int i = 0, j = p.Length - 1; i < p.Length; j = i++) a += (double)p[j].X * p[i].Y - (double)p[i].X * p[j].Y;
        return 0.5 * a;
    }

    private static bool Inside(Vec2[] poly, Vec2 q)
    {
        bool c = false;
        for (int i = 0, j = poly.Length - 1; i < poly.Length; j = i++)
        {
            if ((poly[i].Y > q.Y) != (poly[j].Y > q.Y) && q.X < ((double)poly[j].X - poly[i].X) * (q.Y - poly[i].Y) / ((double)poly[j].Y - poly[i].Y) + poly[i].X) c = !c;
        }
        return c;
    }

    private static double Rightmost(Vec2[] p, out int at)
    {
        at = 0; for (int i = 1; i < p.Length; i++) if (p[i].X > p[at].X) at = i;
        return p[at].X;
    }

    private static double Cross(Vec2 o, Vec2 a, Vec2 b) => ((double)a.X - o.X) * ((double)b.Y - o.Y) - ((double)a.Y - o.Y) * ((double)b.X - o.X);

    // Eberly: shoot a ray from the hole's rightmost vertex M towards +x, take the nearest crossed edge, and use its right-hand endpoint P (or a reflex vertex inside triangle M, I, P closer to the ray) as the bridge end.
    private static void Bridge(List<Vec2> poly, Vec2[] hole)
    {
        Rightmost(hole, out int hm); var m = hole[hm];
        double bestX = double.MaxValue; int edge = -1;
        for (int i = 0; i < poly.Count; i++)
        {
            var a = poly[i]; var b = poly[(i + 1) % poly.Count];
            if (a.Y == b.Y) continue;
            if ((a.Y > m.Y) == (b.Y > m.Y) && a.Y != m.Y && b.Y != m.Y) continue;
            if (Math.Min(a.Y, b.Y) > m.Y || Math.Max(a.Y, b.Y) < m.Y) continue;
            double x = a.X + ((double)b.X - a.X) * (m.Y - a.Y) / ((double)b.Y - a.Y);
            if (x >= m.X && x < bestX) { bestX = x; edge = i; }
        }
        if (edge < 0) { edge = 0; }
        int p = poly[edge].X > poly[(edge + 1) % poly.Count].X ? edge : (edge + 1) % poly.Count;
        var ip = new Vec2((float)bestX, m.Y);
        if (bestX == double.MaxValue) ip = m;
        var pv = poly[p];
        double bestAngle = double.MaxValue, bestDist = double.MaxValue; int pick = p;
        for (int i = 0; i < poly.Count; i++)
        {
            var q = poly[i];
            if (i == p || q.X < m.X) continue;
            bool orientIp = Cross(m, ip, pv) > 0;
            // q inside triangle (m, ip, pv) in either winding
            double d1 = Cross(m, ip, q), d2 = Cross(ip, pv, q), d3 = Cross(pv, m, q);
            bool neg = d1 < 0 || d2 < 0 || d3 < 0, pos = d1 > 0 || d2 > 0 || d3 > 0;
            if (neg && pos) continue;
            _ = orientIp;
            double dx = (double)q.X - m.X, dy = (double)q.Y - m.Y, dist = dx * dx + dy * dy, ang = dx == 0 ? double.MaxValue : Math.Abs(dy) / dx;
            if (ang < bestAngle || (ang == bestAngle && dist < bestDist)) { bestAngle = ang; bestDist = dist; pick = i; }
        }
        p = pick;
        // Splice: ..., poly[p], hole[hm], hole[hm+1] ..., hole[hm-1], hole[hm], poly[p], poly[p+1] ...
        var ins = new List<Vec2>(hole.Length + 2);
        for (int k = 0; k <= hole.Length; k++) ins.Add(hole[(hm + k) % hole.Length]);
        ins.Add(poly[p]);
        poly.InsertRange(p + 1, ins);
    }

    // Triangles as index triples into poly (counter-clockwise). Ear clipping with the ear tip chosen as the lowest index that is a valid ear, so the order is fixed.
    private static List<(int A, int B, int C)> EarClip(List<Vec2> poly)
    {
        var tris = new List<(int, int, int)>(); var idx = new List<int>(poly.Count); for (int i = 0; i < poly.Count; i++) idx.Add(i);
        int guard = 0;
        while (idx.Count > 3 && guard++ < poly.Count * poly.Count + 8)
        {
            bool clipped = false;
            for (int k = 0; k < idx.Count; k++)
            {
                int ia = idx[(k + idx.Count - 1) % idx.Count], ib = idx[k], ic = idx[(k + 1) % idx.Count];
                var a = poly[ia]; var b = poly[ib]; var c = poly[ic];
                double cr = Cross(a, b, c);
                if (cr <= 0)
                {
                    if (cr == 0 && SameDirection(a, b, c)) { }
                    else continue;
                }
                if (cr == 0) { idx.RemoveAt(k); clipped = true; break; }   // collinear spike or run: drop the middle vertex, it adds no area
                bool ear = true;
                for (int j = 0; j < idx.Count && ear; j++)
                {
                    int ij = idx[j]; if (ij == ia || ij == ib || ij == ic) continue;
                    var q = poly[ij];
                    if (Same(q, a) || Same(q, b) || Same(q, c)) continue;   // bridge duplicates
                    if (Cross(a, b, q) >= 0 && Cross(b, c, q) >= 0 && Cross(c, a, q) >= 0) ear = false;
                }
                if (!ear) continue;
                tris.Add((ia, ib, ic)); idx.RemoveAt(k); clipped = true; break;
            }
            if (!clipped)
            {
                // Degenerate input (self-touching sliver): drop the vertex with the smallest absolute turn so the loop still ends.
                int worst = 0; double w = double.MaxValue;
                for (int k = 0; k < idx.Count; k++) { double c = Math.Abs(Cross(poly[idx[(k + idx.Count - 1) % idx.Count]], poly[idx[k]], poly[idx[(k + 1) % idx.Count]])); if (c < w) { w = c; worst = k; } }
                idx.RemoveAt(worst);
            }
        }
        if (idx.Count == 3 && Cross(poly[idx[0]], poly[idx[1]], poly[idx[2]]) > 0) tris.Add((idx[0], idx[1], idx[2]));
        return tris;
    }

    // Coordinates as bits, with -0 folded into +0 so that equal coordinates give equal keys.
    private static ulong Key(Vec2 v) => ((ulong)BitConverter.SingleToUInt32Bits(v.X + 0f) << 32) | BitConverter.SingleToUInt32Bits(v.Y + 0f);
    private static bool Same(Vec2 a, Vec2 b) => a.X == b.X && a.Y == b.Y;
    private static bool SameDirection(Vec2 a, Vec2 b, Vec2 c) => ((double)b.X - a.X) * ((double)c.X - b.X) + ((double)b.Y - a.Y) * ((double)c.Y - b.Y) > 0;

    // Hertel-Mehlhorn: pieces are index loops; repeatedly merge two pieces sharing an edge (a, b) / (b, a) when the union is convex and has at most MaxPolygonVertices vertices. Scans in ascending piece order.
    private static void Merge(List<Vec2> poly, List<(int A, int B, int C)> tris, List<Vec2[]> output)
    {
        var parts = new List<List<int>?>(tris.Count);
        foreach (var t in tris) parts.Add(new List<int> { t.A, t.B, t.C });
        bool changed = true;
        while (changed)
        {
            changed = false;
            // Directed edge (poly vertex positions by coordinates, so bridge duplicates match) -> piece.
            var owner = new Dictionary<(ulong, ulong), int>();
            for (int i = 0; i < parts.Count; i++)
            {
                var p = parts[i]; if (p == null) continue;
                for (int k = 0; k < p.Count; k++) owner[(Key(poly[p[k]]), Key(poly[p[(k + 1) % p.Count]]))] = i;
            }
            for (int i = 0; i < parts.Count && !changed; i++)
            {
                var p = parts[i]; if (p == null) continue;
                for (int k = 0; k < p.Count && !changed; k++)
                {
                    if (!owner.TryGetValue((Key(poly[p[(k + 1) % p.Count]]), Key(poly[p[k]])), out int j) || j <= i) continue;
                    var q = parts[j]!; if (p.Count + q.Count - 2 > Constants2D.MaxPolygonVertices) continue;
                    // p has edge a->b at k; q has b->a at some m. Union: p from b around to a, then q from a around to b.
                    int m = -1; for (int s = 0; s < q.Count; s++) if (Same(poly[q[s]], poly[p[(k + 1) % p.Count]]) && Same(poly[q[(s + 1) % q.Count]], poly[p[k]])) { m = s; break; }
                    if (m < 0) continue;
                    var u = new List<int>(p.Count + q.Count - 2);
                    for (int s2 = 0; s2 < p.Count; s2++) u.Add(p[(k + 1 + s2) % p.Count]);   // b ... a
                    for (int s2 = 2; s2 < q.Count; s2++) u.Add(q[(m + s2) % q.Count]);       // the rest of q after a, before b
                    if (!Convex(poly, u)) continue;
                    parts[i] = u; parts[j] = null; changed = true;
                }
            }
        }
        foreach (var p in parts)
        {
            if (p == null) continue;
            var v = new List<Vec2>();
            foreach (int i in p) { var c = poly[i]; if (v.Count == 0 || !Same(v[^1], c)) v.Add(c); }
            if (v.Count > 1 && Same(v[0], v[^1])) v.RemoveAt(v.Count - 1);
            if (v.Count >= 3) output.Add(v.ToArray());
        }
    }

    private static bool Convex(List<Vec2> poly, List<int> u)
    {
        int n = u.Count;
        for (int k = 0; k < n; k++)
        {
            var a = poly[u[k]]; var b = poly[u[(k + 1) % n]]; var c = poly[u[(k + 2) % n]];
            if (Cross(a, b, c) < 0) return false;
        }
        return true;
    }
}
