using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Predicates;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>Options for <see cref="TriangleMesher.Generate"/>.</summary>
public readonly struct TriangleMeshOptions
{
    /// <summary>Target triangle edge length, m: triangles with an area above that of an equilateral triangle of this edge are refined. 0 chooses the longest extent of the polygon divided by 16.</summary>
    public float TargetEdgeLength { get; init; }

    /// <summary>Smallest angle wanted in any triangle, degrees, above 0 and at most 34. 0 chooses 25. Refinement is guaranteed to stop for angles up to 20.7 where the polygon's own corners are at least 90 degrees wide; above that it normally stops too, and the result reports the angle reached.</summary>
    public float MinAngleDegrees { get; init; }

    /// <summary>The most vertices the mesh may have, 3 to 1,000,000; 0 chooses 50,000. Refinement stops there and <see cref="TriangleMeshResult.QualityMet"/> says whether it had finished.</summary>
    public int MaxVertices { get; init; }
}

/// <summary>A triangle mesh of a polygon and how well it fits.</summary>
public sealed class TriangleMeshResult
{
    internal TriangleMeshResult(Vector2F[] vertices, int[] triangles, double minAngle, double maxAngle, double area, double polygonArea, bool qualityMet, int inputVertexCount)
    {
        Vertices = vertices; Triangles = triangles; MinAngleDegrees = minAngle; MaxAngleDegrees = maxAngle; Area = area; PolygonArea = polygonArea; QualityMet = qualityMet; InputVertexCount = inputVertexCount;
    }

    /// <summary>Vertex positions. The first <see cref="InputVertexCount"/> are the polygon's own points, outline first and then each hole, in the order given (consecutive duplicates removed).</summary>
    public Vector2F[] Vertices { get; }

    /// <summary>Three vertex indices per triangle, counter-clockwise.</summary>
    public int[] Triangles { get; }

    /// <summary>The smallest angle of any triangle, degrees.</summary>
    public double MinAngleDegrees { get; }

    /// <summary>The largest angle of any triangle, degrees.</summary>
    public double MaxAngleDegrees { get; }

    /// <summary>Total triangle area, m².</summary>
    public double Area { get; }

    /// <summary>Area of the polygon (outline minus holes), m².</summary>
    public double PolygonArea { get; }

    /// <summary>True when refinement finished: no triangle in the mesh is worse than the requested minimum angle or larger than the target size (except next to polygon corners narrower than the angle, which cannot be improved, and below the smallest feature size).</summary>
    public bool QualityMet { get; }

    /// <summary>Number of polygon points at the start of <see cref="Vertices"/>.</summary>
    public int InputVertexCount { get; }

    /// <summary>Makes the 2D soft-body mesh.</summary>
    /// <exception cref="RecursiveAI.OnePhysics.Physics2D.SoftBodyException">The soft-body mesh limits are exceeded.</exception>
    public RecursiveAI.OnePhysics.Physics2D.SoftBodyMesh ToSoftBodyMesh() => RecursiveAI.OnePhysics.Physics2D.SoftBodyMesh.FromTriangles(Vertices, Triangles);
}

/// <summary>
/// Triangulates a polygon with holes into well-shaped triangles by Delaunay refinement: Ruppert's algorithm (Ruppert, "A Delaunay refinement algorithm for quality 2-dimensional mesh generation", J. Algorithms 1995)
/// with Chew's size rule and Shewchuk's treatment of encroached segments ("Delaunay refinement algorithms for triangular mesh generation", 2002). Points are added by the Bowyer-Watson cavity method; every
/// orientation and in-circle decision uses onemath's exact predicates, so the mesh is the same on every platform. Boundary edges are subdivided at midpoints until their diametral disks hold no other vertex, which
/// makes them edges of the Delaunay triangulation; triangles worse than the angle bound or larger than the size are split at their circumcentres, unless that point would encroach a boundary edge, which is split instead.
/// </summary>
public static class TriangleMesher
{
    /// <summary>Triangulates a polygon.</summary>
    /// <param name="outline">The outer boundary in either winding: 3 to 16384 finite points within ±2^20 (and no coordinate smaller than 2^-80 unless zero); no self-crossing.</param>
    /// <param name="holes">Hole boundaries, or null; each strictly inside the outline, inside no other hole, not touching any other boundary.</param>
    /// <param name="options">Size and angle bound.</param>
    /// <returns>The triangles; every polygon point is a vertex and every boundary edge is a chain of triangle edges.</returns>
    /// <exception cref="GeometryToolException">The polygon is invalid (see <see cref="PolygonDecomposer"/>), or an option is out of range.</exception>
    public static TriangleMeshResult Generate(ReadOnlySpan<Vector2F> outline, IReadOnlyList<Vector2F[]>? holes = null, TriangleMeshOptions options = default)
    {
        float angle = options.MinAngleDegrees == 0 ? 25f : options.MinAngleDegrees;
        if (!(angle > 0 && angle <= 34)) { throw Bad("MinAngleDegrees", angle.ToString("R", CultureInfo.InvariantCulture), "above 0 and at most 34", nameof(options)); }
        if (!(options.TargetEdgeLength >= 0) || !float.IsFinite(options.TargetEdgeLength)) { throw Bad("TargetEdgeLength", options.TargetEdgeLength.ToString("R", CultureInfo.InvariantCulture), "finite and at least 0", nameof(options)); }
        int maxVertices = options.MaxVertices == 0 ? 50000 : options.MaxVertices;
        if (maxVertices < 3 || maxVertices > 1000000) { throw Bad("MaxVertices", maxVertices.ToString(CultureInfo.InvariantCulture), "from 3 to 1000000", nameof(options)); }

        var loops = PolygonDomain.Prepare(outline, holes, out double polygonArea);
        int input = 0;
        foreach (var l in loops) { input += l.Length; }
        if (input > maxVertices) { throw Bad("MaxVertices", maxVertices.ToString(CultureInfo.InvariantCulture), "at least the number of polygon points (" + input.ToString(CultureInfo.InvariantCulture) + ")", nameof(options)); }

        double x0 = double.MaxValue, y0 = double.MaxValue, x1 = double.MinValue, y1 = double.MinValue;
        foreach (var l in loops) { foreach (var (px, py) in l) { x0 = Math.Min(x0, px); x1 = Math.Max(x1, px); y0 = Math.Min(y0, py); y1 = Math.Max(y1, py); } }
        double extent = Math.Max(x1 - x0, y1 - y0);
        double edge = options.TargetEdgeLength > 0 ? options.TargetEdgeLength : extent / 16.0;
        double maxArea = 0.4330127018922193 * edge * edge;   // equilateral triangle of that edge: sqrt(3) / 4 e^2
        double ratioBound = 1.0 / (2.0 * Math64.Sin(angle * (Math.PI / 180.0)));
        var mesher = new Mesher(x0, y0, x1, y1, extent, maxArea, ratioBound, maxVertices);
        mesher.Run(loops);
        return mesher.Result(angle, polygonArea, input);
    }

    private static GeometryToolException Bad(string name, string value, string expected, string paramName) => new(GeometryToolError.InvalidOptions, name + " = " + value + " must be " + expected, paramName);

    private sealed class Mesher
    {
        // Points; 0..2 are the corners of the enclosing triangle.
        private double[] _x = new double[256], _y = new double[256];
        private int _np;
        // Triangles: vertices counter-clockwise; _n[3t + i] is the neighbour across the edge opposite vertex i, or -1.
        private int[] _v = new int[3 * 512], _n = new int[3 * 512];
        private bool[] _alive = new bool[512], _inside = new bool[512];
        private int _nt;
        // Boundary pieces.
        private readonly List<int> _sa = new(), _sb = new();
        private readonly List<bool> _sLive = new();
        private readonly HashSet<long> _segKeys = new();

        private readonly double _maxArea, _ratioBound, _minLen;
        private readonly int _maxVertices;
        private bool _hitLimit;
        private int _lastTriangle;

        public Mesher(double x0, double y0, double x1, double y1, double extent, double maxArea, double ratioBound, int maxVertices)
        {
            _maxArea = maxArea; _ratioBound = ratioBound; _maxVertices = maxVertices;
            _minLen = extent * 1e-5;
            // An enclosing triangle far larger than the polygon, so every polygon edge that is Delaunay among the polygon's points stays an edge.
            double cx = 0.5 * (x0 + x1), cy = 0.5 * (y0 + y1), m = Math.Max(extent, 1e-30) * 2000.0;
            AddPoint(cx - 2 * m, cy - m); AddPoint(cx + 2 * m, cy - m); AddPoint(cx, cy + 2 * m);
            NewTriangle(0, 1, 2, -1, -1, -1, false);
        }

        private int AddPoint(double x, double y)
        {
            if (_np == _x.Length) { Array.Resize(ref _x, _np * 2); Array.Resize(ref _y, _np * 2); }
            _x[_np] = x; _y[_np] = y;
            return _np++;
        }

        private int NewTriangle(int a, int b, int c, int n0, int n1, int n2, bool inside)
        {
            if (_nt == _alive.Length)
            {
                int cap = _nt * 2;
                Array.Resize(ref _alive, cap); Array.Resize(ref _inside, cap); Array.Resize(ref _v, 3 * cap); Array.Resize(ref _n, 3 * cap);
            }

            int t = _nt++;
            _v[3 * t] = a; _v[3 * t + 1] = b; _v[3 * t + 2] = c;
            _n[3 * t] = n0; _n[3 * t + 1] = n1; _n[3 * t + 2] = n2;
            _alive[t] = true; _inside[t] = inside;
            _lastTriangle = t;
            return t;
        }

        private static long Key(int a, int b) => a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;

        private int Orient(int a, int b, double px, double py) => ExactPredicates.Orient2D(_x[a], _y[a], _x[b], _y[b], px, py);

        // ------------------------------------------------------------ insertion

        // Finds a triangle containing (px, py) (on its boundary counts), walking from 'from'. Returns -1 if none does.
        private int Locate(double px, double py, int from)
        {
            int t = from;
            if (t < 0 || t >= _nt || !_alive[t]) { t = _lastTriangle; while (t > 0 && !_alive[t]) { t--; } }
            for (int step = 0; step < 4 * _nt + 16; step++)
            {
                bool moved = false;
                for (int i = 0; i < 3; i++)
                {
                    int a = _v[3 * t + (i + 1) % 3], b = _v[3 * t + (i + 2) % 3];
                    if (Orient(a, b, px, py) < 0)
                    {
                        int nb = _n[3 * t + i];
                        if (nb < 0) { return -1; }
                        t = nb; moved = true; break;
                    }
                }

                if (!moved) { return t; }
            }

            for (int u = 0; u < _nt; u++)
            {
                if (!_alive[u]) { continue; }
                bool ok = true;
                for (int i = 0; i < 3 && ok; i++) { if (Orient(_v[3 * u + (i + 1) % 3], _v[3 * u + (i + 2) % 3], px, py) < 0) { ok = false; } }
                if (ok) { return u; }
            }

            return -1;
        }

        private bool InCircleOf(int t, double px, double py)
        {
            int a = _v[3 * t], b = _v[3 * t + 1], c = _v[3 * t + 2];
            return ExactPredicates.InCircle(_x[a], _y[a], _x[b], _y[b], _x[c], _y[c], px, py) > 0;
        }

        private bool IsSegmentEdge(int a, int b) => _segKeys.Contains(Key(a, b));

        // Bowyer-Watson insertion of (px, py). The cavity is the connected set of triangles whose circumcircle holds the point, starting at the triangle that contains it; with
        // crossSegments false it does not grow across boundary pieces. Returns the new vertex, or -1 when the point could not be inserted (outside, duplicate, or a non-star-shaped cavity).
        private int Insert(double px, double py, int hint, bool crossSegments, out int firstNew)
        {
            firstNew = -1;
            int seed = Locate(px, py, hint);
            if (seed < 0) { return -1; }
            for (int i = 0; i < 3; i++) { int q = _v[3 * seed + i]; if (_x[q] == px && _y[q] == py) { return -1; } }
            var cavity = new List<int> { seed };
            var inCavity = new HashSet<int> { seed };
            for (int h = 0; h < cavity.Count; h++)
            {
                int t = cavity[h];
                for (int i = 0; i < 3; i++)
                {
                    int nb = _n[3 * t + i];
                    if (nb < 0 || inCavity.Contains(nb)) { continue; }
                    if (!crossSegments && IsSegmentEdge(_v[3 * t + (i + 1) % 3], _v[3 * t + (i + 2) % 3])) { continue; }
                    if (InCircleOf(nb, px, py)) { cavity.Add(nb); inCavity.Add(nb); }
                }
            }

            // Boundary edges of the cavity, each with the triangle outside it and the cavity triangle's label.
            var ba = new List<int>(); var bb = new List<int>(); var bn = new List<int>(); var bl = new List<bool>();
            foreach (int t in cavity)
            {
                for (int i = 0; i < 3; i++)
                {
                    int nb = _n[3 * t + i];
                    if (nb >= 0 && inCavity.Contains(nb)) { continue; }
                    ba.Add(_v[3 * t + (i + 1) % 3]); bb.Add(_v[3 * t + (i + 2) % 3]); bn.Add(nb); bl.Add(_inside[t]);
                }
            }

            // Every new triangle (a, b, p) must be counter-clockwise, or the cavity is not star-shaped about p.
            for (int e = 0; e < ba.Count; e++) { if (Orient(ba[e], bb[e], px, py) <= 0) { return -1; } }

            int p = AddPoint(px, py);
            foreach (int t in cavity) { _alive[t] = false; }
            var startAt = new Dictionary<int, int>();
            var created = new int[ba.Count];
            for (int e = 0; e < ba.Count; e++)
            {
                int t = NewTriangle(ba[e], bb[e], p, -1, -1, bn[e], bl[e]);
                created[e] = t;
                startAt[ba[e]] = t;
                if (bn[e] >= 0)
                {
                    // Point the outside neighbour at the new triangle across the same edge.
                    for (int i = 0; i < 3; i++)
                    {
                        if (_n[3 * bn[e] + i] >= 0 && inCavity.Contains(_n[3 * bn[e] + i]) && _v[3 * bn[e] + (i + 1) % 3] == bb[e] && _v[3 * bn[e] + (i + 2) % 3] == ba[e]) { _n[3 * bn[e] + i] = t; break; }
                    }
                }
            }

            for (int e = 0; e < ba.Count; e++)
            {
                int t = created[e];
                int tb = startAt[bb[e]];          // the triangle whose first vertex is this one's second
                _n[3 * t] = tb;                   // edge (b, p) is opposite vertex a
                _n[3 * tb + 1] = t;               // that triangle's edge (p, a') is opposite its second vertex
            }

            firstNew = created[0];
            return p;
        }

        // ------------------------------------------------------------ boundary pieces

        private int AddSegment(int a, int b)
        {
            _sa.Add(a); _sb.Add(b); _sLive.Add(true); _segKeys.Add(Key(a, b));
            return _sa.Count - 1;
        }

        private bool Encroaches(int s, double px, double py)
        {
            int a = _sa[s], b = _sb[s];
            return (px - _x[a]) * (px - _x[b]) + (py - _y[a]) * (py - _y[b]) <= 0;
        }

        private bool SegmentEncroachedByAnyVertex(int s)
        {
            int a = _sa[s], b = _sb[s];
            for (int q = 3; q < _np; q++)
            {
                if (q == a || q == b) { continue; }
                if (Encroaches(s, _x[q], _y[q])) { return true; }
            }

            return false;
        }

        private double SegLen2(int s) { double dx = _x[_sa[s]] - _x[_sb[s]], dy = _y[_sa[s]] - _y[_sb[s]]; return dx * dx + dy * dy; }

        // Splits segment s at its midpoint; the pieces are checked against every vertex and the other pieces against the new one, and encroached ones queued.
        private bool Split(int s, Queue<int> queue)
        {
            if (_np >= _maxVertices + 3) { _hitLimit = true; return false; }
            int a = _sa[s], b = _sb[s];
            double mx = 0.5 * (_x[a] + _x[b]), my = 0.5 * (_y[a] + _y[b]);
            int hint = _lastTriangle;
            int m = Insert(mx, my, hint, true, out _);
            if (m < 0) { return false; }
            _sLive[s] = false; _segKeys.Remove(Key(a, b));
            int s1 = AddSegment(a, m), s2 = AddSegment(m, b);
            if (SegmentEncroachedByAnyVertex(s1)) { queue.Enqueue(s1); }
            if (SegmentEncroachedByAnyVertex(s2)) { queue.Enqueue(s2); }
            for (int o = 0; o < _sa.Count; o++)
            {
                if (!_sLive[o] || o == s1 || o == s2) { continue; }
                if (_sa[o] == m || _sb[o] == m) { continue; }
                if (Encroaches(o, mx, my)) { queue.Enqueue(o); }
            }

            return true;
        }

        private void DrainQueue(Queue<int> queue)
        {
            while (queue.Count > 0 && !_hitLimit)
            {
                int s = queue.Dequeue();
                if (!_sLive[s]) { continue; }
                if (SegLen2(s) < 4 * _minLen * _minLen) { continue; }   // below the smallest feature size
                Split(s, queue);
            }
        }

        // ------------------------------------------------------------ driver

        public void Run(List<(double X, double Y)[]> loops)
        {
            var first = new List<int>();
            var counts = new List<int>();
            foreach (var l in loops)
            {
                int start = _np;
                foreach (var (px, py) in l)
                {
                    int p = Insert(px, py, _lastTriangle, false, out _);
                    if (p < 0) { throw new GeometryToolException(GeometryToolError.Degenerate, "a polygon point at (" + px.ToString("R", CultureInfo.InvariantCulture) + ", " + py.ToString("R", CultureInfo.InvariantCulture) + ") coincides with another point or could not be placed", null); }
                }

                first.Add(start); counts.Add(l.Length);
            }

            for (int li = 0; li < first.Count; li++)
            {
                for (int i = 0; i < counts[li]; i++) { AddSegment(first[li] + i, first[li] + (i + 1) % counts[li]); }
            }

            // Phase 1: split boundary pieces until none has another vertex in its diametral disk, and every piece is an edge of the triangulation.
            var queue = new Queue<int>();
            for (int round = 0; round < 64; round++)
            {
                for (int s = 0; s < _sa.Count; s++) { if (_sLive[s] && SegmentEncroachedByAnyVertex(s)) { queue.Enqueue(s); } }
                DrainQueue(queue);
                if (_hitLimit) { break; }
                var edges = new HashSet<long>();
                for (int t = 0; t < _nt; t++)
                {
                    if (!_alive[t]) { continue; }
                    for (int i = 0; i < 3; i++) { edges.Add(Key(_v[3 * t + i], _v[3 * t + (i + 1) % 3])); }
                }

                bool missing = false;
                for (int s = 0; s < _sa.Count; s++)
                {
                    if (_sLive[s] && !edges.Contains(Key(_sa[s], _sb[s]))) { queue.Enqueue(s); missing = true; }
                }

                if (!missing) { break; }
                DrainQueue(queue);
            }

            Label();
            if (!_hitLimit) { Refine(queue); }
        }

        // Triangles touching the enclosing triangle's corners are outside; crossing a boundary piece flips inside and outside.
        private void Label()
        {
            var seen = new bool[_nt];
            var order = new List<int>();
            for (int t = 0; t < _nt; t++)
            {
                if (!_alive[t]) { continue; }
                if (_v[3 * t] < 3 || _v[3 * t + 1] < 3 || _v[3 * t + 2] < 3) { _inside[t] = false; seen[t] = true; order.Add(t); }
            }

            for (int h = 0; h < order.Count; h++)
            {
                int t = order[h];
                for (int i = 0; i < 3; i++)
                {
                    int nb = _n[3 * t + i];
                    if (nb < 0 || seen[nb]) { continue; }
                    bool flip = IsSegmentEdge(_v[3 * t + (i + 1) % 3], _v[3 * t + (i + 2) % 3]);
                    _inside[nb] = flip ? !_inside[t] : _inside[t];
                    seen[nb] = true; order.Add(nb);
                }
            }
        }

        private void Circumcircle(int t, out double cx, out double cy, out double r2, out double shortest2, out double area)
        {
            int a = _v[3 * t], b = _v[3 * t + 1], c = _v[3 * t + 2];
            double bx = _x[b] - _x[a], by = _y[b] - _y[a], ccx = _x[c] - _x[a], ccy = _y[c] - _y[a];
            double d = 2 * (bx * ccy - by * ccx);
            double b2 = bx * bx + by * by, c2 = ccx * ccx + ccy * ccy;
            double ux = (ccy * b2 - by * c2) / d, uy = (bx * c2 - ccx * b2) / d;
            cx = _x[a] + ux; cy = _y[a] + uy; r2 = ux * ux + uy * uy;
            double e3 = (_x[c] - _x[b]) * (_x[c] - _x[b]) + (_y[c] - _y[b]) * (_y[c] - _y[b]);
            shortest2 = Math.Min(b2, Math.Min(c2, e3));
            area = 0.5 * Math.Abs(d) * 0.5;
        }

        private void Refine(Queue<int> segQueue)
        {
            var work = new Queue<int>();
            for (int t = 0; t < _nt; t++) { if (_alive[t] && _inside[t]) { work.Enqueue(t); } }
            int guard = 0;
            while (work.Count > 0 && !_hitLimit)
            {
                if (++guard > 20_000_000) { _hitLimit = true; break; }
                int t = work.Dequeue();
                if (!_alive[t] || !_inside[t]) { continue; }
                Circumcircle(t, out double cx, out double cy, out double r2, out double shortest2, out double area);
                bool skinny = r2 > _ratioBound * _ratioBound * shortest2;
                bool big = area > _maxArea;
                if (!skinny && !big) { continue; }
                // Below the smallest feature size nothing is split: this is where polygon corners narrower than the angle bound end the refinement.
                if (shortest2 < _minLen * _minLen && !big) { continue; }
                if (!double.IsFinite(cx) || !double.IsFinite(cy)) { continue; }

                // A circumcentre inside the diametral disk of a boundary piece splits that piece instead.
                bool encroached = false;
                for (int s = 0; s < _sa.Count; s++)
                {
                    if (_sLive[s] && Encroaches(s, cx, cy)) { segQueue.Enqueue(s); encroached = true; }
                }

                if (encroached)
                {
                    int before = _nt, pointsBefore = _np;
                    DrainQueue(segQueue);
                    if (_np == pointsBefore) { continue; }   // every encroached piece is below the smallest feature size: leave this triangle
                    for (int u = before; u < _nt; u++) { if (_alive[u] && _inside[u]) { work.Enqueue(u); } }
                    if (_alive[t] && _inside[t]) { work.Enqueue(t); }
                    continue;
                }

                if (_np >= _maxVertices + 3) { _hitLimit = true; break; }
                int loc = Locate(cx, cy, t);
                if (loc < 0 || !_inside[loc]) { continue; }
                int mark = _nt;
                int p = Insert(cx, cy, loc, false, out _);
                if (p < 0) { continue; }
                for (int u = mark; u < _nt; u++) { if (_alive[u] && _inside[u]) { work.Enqueue(u); } }
            }
        }

        // ------------------------------------------------------------ result

        public TriangleMeshResult Result(float requestedAngle, double polygonArea, int inputCount)
        {
            var map = new int[_np];
            Array.Fill(map, -1);
            var tris = new List<int>();
            for (int t = 0; t < _nt; t++)
            {
                if (!_alive[t] || !_inside[t]) { continue; }
                for (int i = 0; i < 3; i++) { tris.Add(_v[3 * t + i]); }
            }

            int used = 0;
            // Polygon points keep their input order at the front.
            for (int p = 3; p < 3 + inputCount; p++) { map[p] = used++; }
            for (int i = 0; i < tris.Count; i++) { if (map[tris[i]] < 0) { map[tris[i]] = 0; } }
            for (int p = 3 + inputCount; p < _np; p++) { if (map[p] == 0) { map[p] = used++; } }
            var verts = new Vector2F[used];
            for (int p = 3; p < _np; p++) { if (map[p] >= 0) { verts[map[p]] = new Vector2F((float)_x[p], (float)_y[p]); } }
            var outTris = new int[tris.Count];
            double minA = 180, maxA = 0, area = 0;
            for (int i = 0; i < tris.Count; i += 3)
            {
                for (int k = 0; k < 3; k++) { outTris[i + k] = map[tris[i + k]]; }
                var a = verts[outTris[i]]; var b = verts[outTris[i + 1]]; var c = verts[outTris[i + 2]];
                area += 0.5 * (((double)b.X - a.X) * ((double)c.Y - a.Y) - ((double)b.Y - a.Y) * ((double)c.X - a.X));
                Angle(a, b, c, ref minA, ref maxA); Angle(b, c, a, ref minA, ref maxA); Angle(c, a, b, ref minA, ref maxA);
            }

            // The angle bound is met when the smallest angle reaches it, or when refinement finished and what remains is next to a polygon corner narrower than the bound.
            bool met = !_hitLimit;
            return new TriangleMeshResult(verts, outTris, minA, maxA, area, polygonArea, met, inputCount);
        }

        private static void Angle(Vector2F p, Vector2F q, Vector2F r, ref double min, ref double max)
        {
            double ux = (double)q.X - p.X, uy = (double)q.Y - p.Y, vx = (double)r.X - p.X, vy = (double)r.Y - p.Y;
            double c = (ux * vx + uy * vy) / Math.Sqrt((ux * ux + uy * uy) * (vx * vx + vy * vy));
            double deg = Math64.Acos(Math.Max(-1.0, Math.Min(1.0, c))) * (180.0 / Math.PI);
            if (deg < min) { min = deg; }
            if (deg > max) { max = deg; }
        }
    }
}
