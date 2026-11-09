using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Destruction;

/// <summary>
/// A convex piece of a body, in the body's frame, in double precision: a polytope as outward counter-clockwise faces (3D) or a counter-clockwise polygon (2D, stored as a
/// single face with z = 0). Pieces are cut by half-spaces with exact-order plane clipping (Sutherland and Hodgman 1974 on each face, the new face closed from the vertices
/// that lie on the cutting plane), which is all Voronoi fracture and slicing need (Ericson, Real-Time Collision Detection, ch. 8).
/// A piece is rebuilt from float points through the hull builder whenever it becomes shape geometry, so a world restored from a snapshot continues from the same pieces.
/// </summary>
internal sealed class Piece
{
    private readonly Vector3D[][] _faces;

    private Piece(bool planar, Vector3D[][] faces)
    {
        Planar = planar;
        _faces = faces;
    }

    /// <summary>True for a 2D polygon.</summary>
    public bool Planar { get; }

    /// <summary>Faces, outward and counter-clockwise seen from outside; a 2D piece has one.</summary>
    public Vector3D[][] Faces => _faces;

    /// <summary>The unique vertices in a fixed order (lexicographic by x, y, z).</summary>
    public Vector3D[] Vertices()
    {
        var all = new List<Vector3D>();
        foreach (var f in _faces) all.AddRange(f);
        all.Sort(CompareVertex);
        var unique = new List<Vector3D>(all.Count);
        foreach (var v in all) if (unique.Count == 0 || unique[^1] != v) unique.Add(v);
        return unique.ToArray();
    }

    private static int CompareVertex(Vector3D a, Vector3D b)
    {
        int c = a.X.CompareTo(b.X); if (c != 0) return c;
        c = a.Y.CompareTo(b.Y); if (c != 0) return c;
        return a.Z.CompareTo(b.Z);
    }

    /// <summary>Number of distinct vertices of a polygon piece (a 2D piece's single face).</summary>
    public int PolygonCount => Planar ? _faces[0].Length : Vertices().Length;

    /// <summary>The polygon with the least significant vertices dropped until at most <paramref name="max"/> remain (each step drops the vertex whose triangle with its neighbours is smallest, lowest index on ties). 2D only.</summary>
    public Piece Simplified(int max)
    {
        var poly = new List<Vector3D>(_faces[0]);
        while (poly.Count > max)
        {
            int best = 0; double bestArea = double.MaxValue;
            for (int i = 0; i < poly.Count; i++)
            {
                var a = poly[(i + poly.Count - 1) % poly.Count]; var b = poly[i]; var c = poly[(i + 1) % poly.Count];
                double area = Math.Abs((b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X));
                if (area < bestArea) { bestArea = area; best = i; }
            }
            poly.RemoveAt(best);
        }
        return new Piece(true, [poly.ToArray()]);
    }

    /// <summary>Splits a polygon into convex polygons of at most <paramref name="max"/> vertices by fanning from its first vertex. 2D only.</summary>
    public List<Piece> FanSplit(int max)
    {
        var result = new List<Piece>();
        var poly = new List<Vector3D>(_faces[0]);
        while (poly.Count > max)
        {
            var head = new Vector3D[max];
            for (int i = 0; i < max; i++) head[i] = poly[i];
            result.Add(new Piece(true, [head]));
            poly.RemoveRange(1, max - 2);
        }
        result.Add(new Piece(true, [poly.ToArray()]));
        return result;
    }

    /// <summary>Float vertices for a hull or polygon shape.</summary>
    public Vector3F[] FloatPoints()
    {
        var v = Vertices();
        var r = new Vector3F[v.Length];
        for (int i = 0; i < v.Length; i++) r[i] = new Vector3F((float)v[i].X, (float)v[i].Y, (float)v[i].Z);
        return r;
    }

    /// <summary>Builds a piece from the hull of float points (3D). Null when the points are degenerate.</summary>
    public static Piece? FromPoints3D(ReadOnlySpan<Vector3F> points)
    {
        ConvexHull3D hull;
        try { hull = ConvexHull3D.Build(points); }
        catch (HullBuildException) { return null; }
        return FromHull(hull);
    }

    /// <summary>The faces of a built hull.</summary>
    public static Piece FromHull(ConvexHull3D hull)
    {
        var verts = hull.Vertices;
        var faces = new Vector3D[hull.Faces.Length][];
        var edges = hull.Edges;
        for (int i = 0; i < faces.Length; i++)
        {
            var f = hull.Faces[i];
            var loop = new Vector3D[f.EdgeCount];
            int e = f.FirstEdge;
            for (int k = 0; k < loop.Length; k++)
            {
                var p = verts[edges[e].Origin];
                loop[k] = new Vector3D(p.X, p.Y, p.Z);
                e = edges[e].Next;
            }
            faces[i] = loop;
        }
        return new Piece(false, faces);
    }

    /// <summary>Builds a piece from an axis-aligned box in the body frame.</summary>
    public static Piece Box(Vector3D min, Vector3D max)
    {
        var c = new Vector3D[8];
        for (int i = 0; i < 8; i++) c[i] = new Vector3D((i & 1) != 0 ? max.X : min.X, (i & 2) != 0 ? max.Y : min.Y, (i & 4) != 0 ? max.Z : min.Z);
        var faces = new Vector3D[6][];
        // Outward counter-clockwise loops: -x, +x, -y, +y, -z, +z.
        faces[0] = [c[0], c[4], c[6], c[2]];
        faces[1] = [c[1], c[3], c[7], c[5]];
        faces[2] = [c[0], c[1], c[5], c[4]];
        faces[3] = [c[2], c[6], c[7], c[3]];
        faces[4] = [c[0], c[2], c[3], c[1]];
        faces[5] = [c[4], c[5], c[7], c[6]];
        return new Piece(false, faces);
    }

    /// <summary>Builds a 2D piece from the convex hull of the points (monotone chain), counter-clockwise. Null when degenerate.</summary>
    public static Piece? FromPoints2D(ReadOnlySpan<Vector3F> points)
    {
        var p = new List<Vector3D>(points.Length);
        foreach (var q in points) p.Add(new Vector3D(q.X, q.Y, 0));
        p.Sort(CompareVertex);
        var u = new List<Vector3D>();
        foreach (var q in p) if (u.Count == 0 || u[^1] != q) u.Add(q);
        if (u.Count < 3) return null;
        var h = new Vector3D[2 * u.Count];
        int k = 0;
        for (int i = 0; i < u.Count; i++)
        {
            while (k >= 2 && Cross2(h[k - 2], h[k - 1], u[i]) <= 0) k--;
            h[k++] = u[i];
        }
        for (int i = u.Count - 2, t = k + 1; i >= 0; i--)
        {
            while (k >= t && Cross2(h[k - 2], h[k - 1], u[i]) <= 0) k--;
            h[k++] = u[i];
        }
        k--;
        if (k < 3) return null;
        var poly = new Vector3D[k];
        Array.Copy(h, poly, k);
        var piece = new Piece(true, [poly]);
        piece.Measure(out double area, out _);
        return area > 0 ? piece : null;
    }

    private static double Cross2(Vector3D o, Vector3D a, Vector3D b) => (a.X - o.X) * (b.Y - o.Y) - (a.Y - o.Y) * (b.X - o.X);

    /// <summary>Volume (3D) or area (2D) and the centroid, by tetrahedra (triangles) fanned from the first vertex.</summary>
    public void Measure(out double measure, out Vector3D centroid)
    {
        if (Planar)
        {
            var poly = _faces[0];
            double a = 0, cx = 0, cy = 0;
            var o = poly[0];
            for (int i = 1; i + 1 < poly.Length; i++)
            {
                double x1 = poly[i].X - o.X, y1 = poly[i].Y - o.Y, x2 = poly[i + 1].X - o.X, y2 = poly[i + 1].Y - o.Y;
                double cr = x1 * y2 - x2 * y1;
                a += cr;
                cx += cr * (x1 + x2);
                cy += cr * (y1 + y2);
            }
            if (a <= 0) { measure = 0; centroid = default; return; }
            measure = a * 0.5;
            centroid = new Vector3D(o.X + cx / (3 * a), o.Y + cy / (3 * a), 0);
            return;
        }
        // Reference point: the first vertex, so tiny pieces far from the origin keep their precision.
        var r = _faces[0][0];
        double vol = 0, sx = 0, sy = 0, sz = 0;
        foreach (var f in _faces)
        {
            var f0 = f[0] - r;
            for (int i = 1; i + 1 < f.Length; i++)
            {
                var b = f[i] - r;
                var c = f[i + 1] - r;
                double v = Vector3D.Dot(f0, Vector3D.Cross(b, c));
                vol += v;
                sx += v * (f0.X + b.X + c.X);
                sy += v * (f0.Y + b.Y + c.Y);
                sz += v * (f0.Z + b.Z + c.Z);
            }
        }
        if (vol <= 0) { measure = 0; centroid = default; return; }
        measure = vol / 6;
        centroid = new Vector3D(r.X + sx / (4 * vol), r.Y + sy / (4 * vol), r.Z + sz / (4 * vol));
    }

    /// <summary>The largest distance from a point to a vertex.</summary>
    public double Radius(Vector3D from)
    {
        double best = 0;
        foreach (var f in _faces) foreach (var v in f) { double d = (v - from).LengthSquared(); if (d > best) best = d; }
        return Math.Sqrt(best);
    }

    /// <summary>Smallest and largest coordinates.</summary>
    public void Bounds(out Vector3D min, out Vector3D max)
    {
        min = new Vector3D(double.MaxValue, double.MaxValue, double.MaxValue); max = -min;
        foreach (var f in _faces)
            foreach (var v in f)
            {
                min = new Vector3D(Math.Min(min.X, v.X), Math.Min(min.Y, v.Y), Math.Min(min.Z, v.Z));
                max = new Vector3D(Math.Max(max.X, v.X), Math.Max(max.Y, v.Y), Math.Max(max.Z, v.Z));
            }
    }

    /// <summary>True when the point is inside or on every face plane within the tolerance.</summary>
    public bool Contains(Vector3D p, double tolerance)
    {
        if (Planar)
        {
            var poly = _faces[0];
            for (int i = 0; i < poly.Length; i++)
            {
                var a = poly[i]; var b = poly[(i + 1) % poly.Length];
                double cr = (b.X - a.X) * (p.Y - a.Y) - (b.Y - a.Y) * (p.X - a.X);
                if (cr < -tolerance * Math.Sqrt((b - a).LengthSquared())) return false;
            }
            return true;
        }
        foreach (var f in _faces)
        {
            var n = FaceNormal(f);
            if (Vector3D.Dot(n, p - f[0]) > tolerance) return false;
        }
        return true;
    }

    /// <summary>Unit outward normal of a face by Newell's method.</summary>
    public static Vector3D FaceNormal(Vector3D[] f)
    {
        double nx = 0, ny = 0, nz = 0;
        for (int i = 0; i < f.Length; i++)
        {
            var a = f[i]; var b = f[(i + 1) % f.Length];
            nx += (a.Y - b.Y) * (a.Z + b.Z);
            ny += (a.Z - b.Z) * (a.X + b.X);
            nz += (a.X - b.X) * (a.Y + b.Y);
        }
        double l = Math.Sqrt(nx * nx + ny * ny + nz * nz);
        return l > 0 ? new Vector3D(nx / l, ny / l, nz / l) : default;
    }

    /// <summary>
    /// Keeps the part with <c>Dot(n, x) &lt;= d</c> (n a unit vector; z = 0 for a 2D piece). Returns this piece when nothing is cut and null when the result is empty or flat.
    /// </summary>
    public Piece? Clip(Vector3D n, double d)
    {
        Bounds(out var lo, out var hi);
        double scale = Math.Max(Math.Max(hi.X - lo.X, hi.Y - lo.Y), Math.Max(hi.Z - lo.Z, 1e-30));
        double eps = 1e-11 * scale + 1e-13 * (Math.Abs(lo.X) + Math.Abs(hi.X) + Math.Abs(lo.Y) + Math.Abs(hi.Y) + Math.Abs(lo.Z) + Math.Abs(hi.Z));
        bool anyOut = false, anyIn = false;
        foreach (var f in _faces)
            foreach (var v in f)
            {
                double dist = Vector3D.Dot(n, v) - d;
                if (dist > eps) anyOut = true; else if (dist < -eps) anyIn = true;
            }
        if (!anyOut) return this;
        if (!anyIn) return null;

        var faces = new List<Vector3D[]>(_faces.Length + 1);
        var onPlane = new List<Vector3D>();
        var tmp = new List<Vector3D>();
        foreach (var f in _faces)
        {
            tmp.Clear();
            int m = f.Length;
            for (int i = 0; i < m; i++)
            {
                var a = f[i]; var b = f[(i + 1) % m];
                double da = Vector3D.Dot(n, a) - d, db = Vector3D.Dot(n, b) - d;
                bool aIn = da <= eps, bIn = db <= eps;
                if (aIn) tmp.Add(a);
                if ((da < -eps && db > eps) || (da > eps && db < -eps))
                {
                    // Intersect from the lexicographically smaller endpoint so both faces sharing the edge get the same bits.
                    bool swap = CompareVertex(a, b) > 0;
                    var p = swap ? b : a; var q = swap ? a : b;
                    double dp = swap ? db : da, dq = swap ? da : db;
                    double t = dp / (dp - dq);
                    tmp.Add(new Vector3D(p.X + (q.X - p.X) * t, p.Y + (q.Y - p.Y) * t, p.Z + (q.Z - p.Z) * t));
                }
            }
            // Drop repeated points that the on-plane tolerance can leave behind.
            var cleaned = new List<Vector3D>(tmp.Count);
            foreach (var v in tmp) if (cleaned.Count == 0 || cleaned[^1] != v) cleaned.Add(v);
            while (cleaned.Count > 1 && cleaned[0] == cleaned[^1]) cleaned.RemoveAt(cleaned.Count - 1);
            if (cleaned.Count < 3) { CollectOnPlane(cleaned, n, d, eps, onPlane); continue; }
            if (Planar) { faces.Add(cleaned.ToArray()); continue; }
            var arr = cleaned.ToArray();
            faces.Add(arr);
            CollectOnPlane(cleaned, n, d, eps, onPlane);
        }
        if (Planar)
        {
            if (faces.Count == 0) return null;
            var res = new Piece(true, [faces[0]]);
            res.Measure(out double area, out _);
            return area > 0 ? res : null;
        }
        if (faces.Count < 3) return null;
        var cap = BuildCap(onPlane, n, eps);
        if (cap == null) return null;
        faces.Add(cap);
        var result = new Piece(false, faces.ToArray());
        result.Measure(out double vol, out _);
        return vol > 0 ? result : null;
    }

    private static void CollectOnPlane(List<Vector3D> poly, Vector3D n, double d, double eps, List<Vector3D> into)
    {
        foreach (var v in poly) if (Math.Abs(Vector3D.Dot(n, v) - d) <= eps) into.Add(v);
    }

    // The cap is the convex polygon of every vertex on the cutting plane, ordered counter-clockwise about n by cross-product comparisons (no angles).
    private static Vector3D[]? BuildCap(List<Vector3D> pts, Vector3D n, double eps)
    {
        pts.Sort(CompareVertex);
        var u = new List<Vector3D>();
        foreach (var p in pts) if (u.Count == 0 || u[^1] != p) u.Add(p);
        // Merge points closer than the tolerance (a vertex that sat on the plane next to an intersection point).
        var m = new List<Vector3D>();
        foreach (var p in u)
        {
            bool dup = false;
            foreach (var q in m) if ((p - q).LengthSquared() <= eps * eps) { dup = true; break; }
            if (!dup) m.Add(p);
        }
        if (m.Count < 3) return null;
        var c = default(Vector3D);
        foreach (var p in m) c += p;
        c /= m.Count;
        // An orthonormal basis (a, b) of the plane with a x b = n.
        var axis = Math.Abs(n.X) < 0.57735 ? new Vector3D(1, 0, 0) : (Math.Abs(n.Y) < 0.57735 ? new Vector3D(0, 1, 0) : new Vector3D(0, 0, 1));
        var a = Vector3D.Cross(n, axis); a /= a.Length();
        var b = Vector3D.Cross(n, a);
        var keyed = new List<(double x, double y, Vector3D p)>(m.Count);
        foreach (var p in m) { var w = p - c; keyed.Add((Vector3D.Dot(w, a), Vector3D.Dot(w, b), p)); }
        keyed.Sort((l, r) =>
        {
            int hl = (l.y > 0 || (l.y == 0 && l.x > 0)) ? 0 : 1, hr = (r.y > 0 || (r.y == 0 && r.x > 0)) ? 0 : 1;
            if (hl != hr) return hl.CompareTo(hr);
            double cr = l.x * r.y - l.y * r.x;
            if (cr > 0) return -1;
            if (cr < 0) return 1;
            double dl = l.x * l.x + l.y * l.y, dr = r.x * r.x + r.y * r.y;
            int k = dl.CompareTo(dr);
            return k != 0 ? k : CompareVertex(l.p, r.p);
        });
        var loop = new Vector3D[keyed.Count];
        for (int i = 0; i < loop.Length; i++) loop[i] = keyed[i].p;
        // A convex polygon around its centroid; collinear points stay (harmless), a zero-area loop is refused.
        double area = 0;
        for (int i = 0; i < loop.Length; i++) { var p = loop[i] - loop[0]; var q = loop[(i + 1) % loop.Length] - loop[0]; area += Vector3D.Dot(Vector3D.Cross(p, q), n); }
        return area > eps * eps ? loop : null;
    }
}
