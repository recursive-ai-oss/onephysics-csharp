using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>Why a 2D soft-body mesh or definition was refused.</summary>
public enum SoftBodyError
{
    /// <summary>No vertices, no elements, or more than the limits allow.</summary>
    BadCount,

    /// <summary>A coordinate or parameter is NaN or infinite.</summary>
    NonFinite,

    /// <summary>A coordinate or parameter is outside its range.</summary>
    OutOfRange,

    /// <summary>An index is negative or not below the vertex count, or an element names one vertex twice.</summary>
    BadIndex,

    /// <summary>A triangle has (almost) no area, or an outline has none or winds clockwise.</summary>
    Degenerate,

    /// <summary>An edge is shared by more than two triangles, or by two triangles that run along it in the same direction.</summary>
    NonManifold,

    /// <summary>A vertex belongs to no element, so nothing would hold it to the body.</summary>
    UnusedVertex,

    /// <summary>The operation needs a mesh whose boundary is one closed loop and the mesh is not one.</summary>
    NotClosed,

    /// <summary>A definition field is invalid (mass, compliance, radius, friction and so on).</summary>
    InvalidDefinition,
}

/// <summary>Thrown when a 2D soft-body mesh or definition cannot be used. The message names the field or element and what was expected.</summary>
public sealed class SoftBodyException : ArgumentException
{
    /// <summary>Creates the exception.</summary>
    public SoftBodyException(SoftBodyError error, string message, string? subject = null)
        : base(message, subject)
    {
        Error = error;
    }

    /// <summary>The kind of failure.</summary>
    public SoftBodyError Error { get; }
}

/// <summary>
/// The rest shape of a 2D soft body: vertices, and either triangles (a triangulated area), or just an outline (a closed loop of segments, the skin of a balloon). Meshes are immutable and may be shared by any number of bodies.
/// Everything is validated when the mesh is made, so a mesh that exists is safe to build bodies from: finite coordinates within ±2^21, indices in range, no repeated vertex in a triangle, no triangle that is a sliver,
/// at most two triangles on an edge, no vertex left out.
/// </summary>
/// <remarks>
/// Triangles are counter-clockwise; one given clockwise is repaired by swapping its last two vertices. For a triangle mesh the <see cref="Outline"/> is derived: the boundary, when it is one closed loop
/// (counter-clockwise, so the area is to its left). A mesh made from an outline has no triangles and its outline is the vertices in order.
/// </remarks>
public sealed class SoftBodyMesh
{
    /// <summary>Largest number of vertices in one mesh (and so particles in one body), 2^20.</summary>
    public const int MaxVertices = 1 << 20;

    /// <summary>Largest number of triangles in one mesh, 2^22.</summary>
    public const int MaxElements = 1 << 22;

    /// <summary>Largest vertex coordinate magnitude, 2^21 metres.</summary>
    public const float MaxCoordinate = 2097152f;

    // A triangle whose doubled area is below this fraction of its longest edge squared is a sliver.
    private const double SliverRatio = 1e-6;

    private readonly Vector2F[] _vertices;
    private readonly int[] _triangles;
    private readonly int[] _outline;

    private SoftBodyMesh(Vector2F[] vertices, int[] triangles, int[] outline)
    {
        _vertices = vertices;
        _triangles = triangles;
        _outline = outline;
    }

    /// <summary>Rest positions of the vertices, metres, in the mesh's own frame.</summary>
    public ReadOnlySpan<Vector2F> Vertices => _vertices;

    /// <summary>Triangle vertex indices, three per triangle, counter-clockwise; empty for an outline mesh.</summary>
    public ReadOnlySpan<int> Triangles => _triangles;

    /// <summary>The boundary loop as vertex indices, counter-clockwise; empty when the mesh's boundary is not a single closed loop (a mesh with a hole or a pinched corner).</summary>
    public ReadOnlySpan<int> Outline => _outline;

    /// <summary>Number of vertices.</summary>
    public int VertexCount => _vertices.Length;

    /// <summary>Number of triangles.</summary>
    public int TriangleCount => _triangles.Length / 3;

    /// <summary>True when the mesh has a single closed <see cref="Outline"/>, so the body can hold pressure.</summary>
    public bool IsClosed => _outline.Length >= 3;

    // ------------------------------------------------------------------ construction

    /// <summary>Makes an area mesh from triangles.</summary>
    /// <param name="vertices">Rest positions; copied.</param>
    /// <param name="triangles">Three vertex indices per triangle; copied. A clockwise triangle is repaired (its last two indices swapped).</param>
    /// <exception cref="SoftBodyException">The input breaks a rule listed on <see cref="SoftBodyMesh"/>; the message names the element.</exception>
    public static SoftBodyMesh FromTriangles(ReadOnlySpan<Vector2F> vertices, ReadOnlySpan<int> triangles)
    {
        CheckVertices(vertices);
        if (triangles.Length == 0 || triangles.Length % 3 != 0 || triangles.Length / 3 > MaxElements)
        {
            throw new SoftBodyException(SoftBodyError.BadCount, "triangles has " + triangles.Length.ToString(CultureInfo.InvariantCulture) + " indices; expected a non-zero multiple of 3 with at most " + MaxElements.ToString(CultureInfo.InvariantCulture) + " triangles.", nameof(triangles));
        }

        var v = vertices.ToArray();
        var t = triangles.ToArray();
        CheckTriangles(v, t);
        CheckUsed(v.Length, t);
        var outline = Analyse(t, v.Length);
        return new SoftBodyMesh(v, t, outline);
    }

    /// <summary>Makes a closed skin from an outline: the vertices in order, counter-clockwise, each joined to the next and the last to the first. It has no triangles; with <see cref="SoftBodyDef.InternalPressure"/> it is a balloon.</summary>
    /// <param name="vertices">Three or more points; copied.</param>
    /// <exception cref="SoftBodyException">A point is not finite or out of range, two neighbours coincide, or the outline has no area or winds clockwise.</exception>
    public static SoftBodyMesh FromOutline(ReadOnlySpan<Vector2F> vertices)
    {
        CheckVertices(vertices);
        var v = vertices.ToArray();
        double twiceArea = 0.0;
        for (int i = 0; i < v.Length; i++)
        {
            var a = v[i]; var b = v[(i + 1) % v.Length];
            if ((double)a.X == b.X && (double)a.Y == b.Y)
            {
                throw new SoftBodyException(SoftBodyError.Degenerate, "outline vertices " + i.ToString(CultureInfo.InvariantCulture) + " and " + ((i + 1) % v.Length).ToString(CultureInfo.InvariantCulture) + " coincide.", nameof(vertices));
            }

            twiceArea += (double)a.X * b.Y - (double)a.Y * b.X;
        }

        if (!(twiceArea > 0.0))
        {
            throw new SoftBodyException(SoftBodyError.Degenerate, "the outline encloses an area of " + (0.5 * twiceArea).ToString("R", CultureInfo.InvariantCulture) + "; it must wind counter-clockwise around a positive area.", nameof(vertices));
        }

        var outline = new int[v.Length];
        for (int i = 0; i < outline.Length; i++) { outline[i] = i; }
        return new SoftBodyMesh(v, Array.Empty<int>(), outline);
    }

    /// <summary>Triangulates a simple polygon by ear clipping and makes an area mesh from it (the polygon's vertices keep their indices). Long thin triangles result from long thin polygons; use a generator such as <see cref="Box"/> for even cells.</summary>
    /// <param name="points">Three to 4096 points, either winding; copied. The polygon must not cross itself.</param>
    /// <exception cref="SoftBodyException">The polygon is not valid or not simple.</exception>
    public static SoftBodyMesh FromPolygon(ReadOnlySpan<Vector2F> points)
    {
        CheckVertices(points);
        if (points.Length > 4096) { throw new SoftBodyException(SoftBodyError.BadCount, "points has " + points.Length.ToString(CultureInfo.InvariantCulture) + " entries; ear clipping accepts at most 4096.", nameof(points)); }
        var v = points.ToArray();
        int n = v.Length;
        double twiceArea = 0.0;
        for (int i = 0; i < n; i++) { var a = v[i]; var b = v[(i + 1) % n]; twiceArea += (double)a.X * b.Y - (double)a.Y * b.X; }
        if (!(twiceArea != 0.0)) { throw new SoftBodyException(SoftBodyError.Degenerate, "the polygon encloses no area.", nameof(points)); }
        var ring = new List<int>(n);
        if (twiceArea > 0.0) { for (int i = 0; i < n; i++) { ring.Add(i); } }
        else { for (int i = n - 1; i >= 0; i--) { ring.Add(i); } }
        var tris = new List<int>(3 * (n - 2));
        while (ring.Count > 3)
        {
            bool clipped = false;
            for (int k = 0; k < ring.Count; k++)
            {
                int ia = ring[(k + ring.Count - 1) % ring.Count], ib = ring[k], ic = ring[(k + 1) % ring.Count];
                if (Cross(v[ia], v[ib], v[ic]) <= 0.0) { continue; }
                bool ear = true;
                for (int m = 0; m < ring.Count && ear; m++)
                {
                    int ip = ring[m];
                    if (ip == ia || ip == ib || ip == ic) { continue; }
                    if (Cross(v[ia], v[ib], v[ip]) >= 0.0 && Cross(v[ib], v[ic], v[ip]) >= 0.0 && Cross(v[ic], v[ia], v[ip]) >= 0.0) { ear = false; }
                }

                if (!ear) { continue; }
                tris.Add(ia); tris.Add(ib); tris.Add(ic);
                ring.RemoveAt(k);
                clipped = true;
                break;
            }

            if (!clipped) { throw new SoftBodyException(SoftBodyError.Degenerate, "no ear could be clipped; the polygon crosses itself or has collinear runs.", nameof(points)); }
        }

        tris.Add(ring[0]); tris.Add(ring[1]); tris.Add(ring[2]);
        return FromTriangles(v, tris.ToArray());
    }

    private static double Cross(Vector2F a, Vector2F b, Vector2F c) => ((double)b.X - a.X) * ((double)c.Y - a.Y) - ((double)b.Y - a.Y) * ((double)c.X - a.X);

    private static void CheckVertices(ReadOnlySpan<Vector2F> v)
    {
        if (v.Length < 3 || v.Length > MaxVertices)
        {
            throw new SoftBodyException(SoftBodyError.BadCount, "vertices has " + v.Length.ToString(CultureInfo.InvariantCulture) + " entries; expected 3 to " + MaxVertices.ToString(CultureInfo.InvariantCulture) + ".", "vertices");
        }

        for (int i = 0; i < v.Length; i++)
        {
            float x = v[i].X, y = v[i].Y;
            if (!float.IsFinite(x) || !float.IsFinite(y))
            {
                throw new SoftBodyException(SoftBodyError.NonFinite, "vertex " + i.ToString(CultureInfo.InvariantCulture) + " is (" + x.ToString("R", CultureInfo.InvariantCulture) + ", " + y.ToString("R", CultureInfo.InvariantCulture) + "); coordinates must be finite.", "vertices");
            }

            if (Math.Abs(x) > MaxCoordinate || Math.Abs(y) > MaxCoordinate)
            {
                throw new SoftBodyException(SoftBodyError.OutOfRange, "vertex " + i.ToString(CultureInfo.InvariantCulture) + " is (" + x.ToString("R", CultureInfo.InvariantCulture) + ", " + y.ToString("R", CultureInfo.InvariantCulture) + "); coordinates must be within ±" + MaxCoordinate.ToString("R", CultureInfo.InvariantCulture) + ".", "vertices");
            }
        }
    }

    private static double Edge2(Vector2F a, Vector2F b)
    {
        double dx = (double)a.X - b.X, dy = (double)a.Y - b.Y;
        return dx * dx + dy * dy;
    }

    private static void CheckTriangles(Vector2F[] v, int[] t)
    {
        for (int e = 0; e < t.Length / 3; e++)
        {
            for (int k = 0; k < 3; k++)
            {
                int a = t[3 * e + k];
                if ((uint)a >= (uint)v.Length)
                {
                    throw new SoftBodyException(SoftBodyError.BadIndex, "triangle " + e.ToString(CultureInfo.InvariantCulture) + " uses index " + a.ToString(CultureInfo.InvariantCulture) + "; vertex indices must be in 0 to " + (v.Length - 1).ToString(CultureInfo.InvariantCulture) + ".", "triangles");
                }

                for (int j = 0; j < k; j++)
                {
                    if (t[3 * e + j] == a) { throw new SoftBodyException(SoftBodyError.BadIndex, "triangle " + e.ToString(CultureInfo.InvariantCulture) + " uses vertex " + a.ToString(CultureInfo.InvariantCulture) + " twice.", "triangles"); }
                }
            }
        }

        for (int e = 0; e < t.Length / 3; e++)
        {
            var a = v[t[3 * e]]; var b = v[t[3 * e + 1]]; var c = v[t[3 * e + 2]];
            double cross = Cross(a, b, c);
            double longest = Math.Max(Edge2(a, b), Math.Max(Edge2(b, c), Edge2(c, a)));
            if (!(Math.Abs(cross) > SliverRatio * longest) || !(longest > 0.0))
            {
                throw new SoftBodyException(SoftBodyError.Degenerate, "triangle " + e.ToString(CultureInfo.InvariantCulture) + " (vertices " + t[3 * e].ToString(CultureInfo.InvariantCulture) + ", " + t[3 * e + 1].ToString(CultureInfo.InvariantCulture) + ", " + t[3 * e + 2].ToString(CultureInfo.InvariantCulture) + ") has no usable area (zero, collinear or a sliver).", "triangles");
            }

            if (cross < 0.0) { (t[3 * e + 1], t[3 * e + 2]) = (t[3 * e + 2], t[3 * e + 1]); }
        }
    }

    private static void CheckUsed(int vertexCount, int[] triangles)
    {
        var used = new bool[vertexCount];
        foreach (int i in triangles) { used[i] = true; }
        for (int i = 0; i < used.Length; i++)
        {
            if (!used[i]) { throw new SoftBodyException(SoftBodyError.UnusedVertex, "vertex " + i.ToString(CultureInfo.InvariantCulture) + " belongs to no triangle.", "vertices"); }
        }
    }

    // Finds the boundary: directed edges whose reverse is not in the mesh. Returns the boundary as one loop of vertices when it is exactly one simple cycle, else an empty array.
    // An edge used by three triangles, or twice in the same direction (two neighbours of opposite orientation), is refused.
    private static int[] Analyse(int[] t, int vertexCount)
    {
        int n = t.Length;
        var directed = new long[n];
        for (int e = 0; e < n / 3; e++)
        {
            for (int k = 0; k < 3; k++)
            {
                long a = t[3 * e + k], b = t[3 * e + (k + 1) % 3];
                directed[3 * e + k] = (a << 20) | b;
            }
        }

        Array.Sort(directed);
        for (int i = 1; i < n; i++)
        {
            if (directed[i] == directed[i - 1])
            {
                long k = directed[i];
                throw new SoftBodyException(SoftBodyError.NonManifold, "the edge from vertex " + (k >> 20).ToString(CultureInfo.InvariantCulture) + " to " + (k & 0xFFFFF).ToString(CultureInfo.InvariantCulture) + " is used by two triangles in the same direction; their orientations disagree or three triangles share it.", "triangles");
            }
        }

        var boundary = new List<long>();
        for (int i = 0; i < n; i++)
        {
            long a = directed[i] >> 20, b = directed[i] & 0xFFFFF;
            long rev = (b << 20) | a;
            if (Array.BinarySearch(directed, rev) < 0) { boundary.Add(directed[i]); }
        }

        if (boundary.Count < 3) { return Array.Empty<int>(); }
        // boundary is sorted by (from, to). Every vertex must start at most one boundary edge, else the boundary pinches and is not a single loop.
        var next = new Dictionary<int, int>(boundary.Count);
        foreach (long e in boundary)
        {
            int from = (int)(e >> 20), to = (int)(e & 0xFFFFF);
            if (!next.TryAdd(from, to)) { return Array.Empty<int>(); }
        }

        int start = (int)(boundary[0] >> 20);
        var loop = new List<int>(boundary.Count);
        int cur = start;
        do
        {
            loop.Add(cur);
            if (!next.TryGetValue(cur, out int nx)) { return Array.Empty<int>(); }
            cur = nx;
        }
        while (cur != start && loop.Count <= boundary.Count);
        if (cur != start || loop.Count != boundary.Count) { return Array.Empty<int>(); }
        return loop.ToArray();
    }

    // ------------------------------------------------------------------ generators

    /// <summary>
    /// A rectangle centred on the origin made of <paramref name="cellsX"/> × <paramref name="cellsY"/> cells of two triangles each (the diagonal alternates in a checkerboard so the mesh has no preferred shear direction).
    /// Vertex (i, j) has index <c>j * (cellsX + 1) + i</c>.
    /// </summary>
    /// <param name="size">Width and height, metres; each finite and positive.</param>
    /// <param name="cellsX">Cells along x, 1 to 1024.</param>
    /// <param name="cellsY">Cells along y, 1 to 1024.</param>
    /// <exception cref="SoftBodyException">An argument is out of range.</exception>
    public static SoftBodyMesh Box(Vector2F size, int cellsX, int cellsY)
    {
        CheckSize(size.X, "size.X"); CheckSize(size.Y, "size.Y");
        CheckCells(cellsX, "cellsX"); CheckCells(cellsY, "cellsY");
        int nx = cellsX + 1, ny = cellsY + 1;
        var v = new Vector2F[nx * ny];
        for (int j = 0; j < ny; j++)
        {
            for (int i = 0; i < nx; i++)
            {
                v[j * nx + i] = new Vector2F(size.X * ((float)i / cellsX - 0.5f), size.Y * ((float)j / cellsY - 0.5f));
            }
        }

        var t = new int[cellsX * cellsY * 6];
        int n = 0;
        for (int j = 0; j < cellsY; j++)
        {
            for (int i = 0; i < cellsX; i++)
            {
                int a = j * nx + i, b = a + 1, c = a + nx, d = c + 1;
                if (((i + j) & 1) == 0) { t[n++] = a; t[n++] = b; t[n++] = d; t[n++] = a; t[n++] = d; t[n++] = c; }
                else { t[n++] = a; t[n++] = b; t[n++] = c; t[n++] = b; t[n++] = d; t[n++] = c; }
            }
        }

        return FromTriangles(v, t);
    }

    /// <summary>
    /// A disc made of a centre vertex and <paramref name="rings"/> rings of <paramref name="segments"/> vertices each (vertex 0 is the centre; ring k, number i, is vertex <c>1 + (k - 1) * segments + i</c>), triangulated between neighbouring rings.
    /// The outline is the outermost ring.
    /// </summary>
    /// <param name="radius">Metres; finite, positive. The outer ring's vertices lie on it.</param>
    /// <param name="rings">Rings, 1 to 256.</param>
    /// <param name="segments">Vertices per ring, 6 to 1024.</param>
    /// <exception cref="SoftBodyException">An argument is out of range.</exception>
    public static SoftBodyMesh Disc(float radius, int rings, int segments)
    {
        CheckSize(radius, "radius");
        if (rings < 1 || rings > 256) { throw new SoftBodyException(SoftBodyError.OutOfRange, "rings is " + rings.ToString(CultureInfo.InvariantCulture) + "; expected 1 to 256.", nameof(rings)); }
        if (segments < 6 || segments > 1024) { throw new SoftBodyException(SoftBodyError.OutOfRange, "segments is " + segments.ToString(CultureInfo.InvariantCulture) + "; expected 6 to 1024.", nameof(segments)); }
        var v = new Vector2F[1 + rings * segments];
        v[0] = new Vector2F(0f, 0f);
        for (int k = 1; k <= rings; k++)
        {
            double r = (double)radius * k / rings;
            for (int i = 0; i < segments; i++)
            {
                var (s, c) = Math64.SinCos(6.283185307179586 * i / segments);
                v[1 + (k - 1) * segments + i] = new Vector2F((float)(r * c), (float)(r * s));
            }
        }

        var t = new List<int>(3 * segments * (2 * rings - 1));
        for (int i = 0; i < segments; i++)
        {
            int i1 = (i + 1) % segments;
            t.Add(0); t.Add(1 + i); t.Add(1 + i1);
        }

        for (int k = 1; k < rings; k++)
        {
            int inner = 1 + (k - 1) * segments, outer = 1 + k * segments;
            for (int i = 0; i < segments; i++)
            {
                int i1 = (i + 1) % segments;
                int a = inner + i, b = inner + i1, c = outer + i, d = outer + i1;
                t.Add(a); t.Add(c); t.Add(d);
                t.Add(a); t.Add(d); t.Add(b);
            }
        }

        return FromTriangles(v, t.ToArray());
    }

    /// <summary>A circular skin: <paramref name="segments"/> vertices on a circle, counter-clockwise, with no triangles (see <see cref="FromOutline"/>). Vertex i is at angle 2πi/segments.</summary>
    /// <param name="radius">Metres; finite, positive.</param>
    /// <param name="segments">Vertices, 3 to 4096.</param>
    /// <exception cref="SoftBodyException">An argument is out of range.</exception>
    public static SoftBodyMesh Ring(float radius, int segments)
    {
        CheckSize(radius, "radius");
        if (segments < 3 || segments > 4096) { throw new SoftBodyException(SoftBodyError.OutOfRange, "segments is " + segments.ToString(CultureInfo.InvariantCulture) + "; expected 3 to 4096.", nameof(segments)); }
        var v = new Vector2F[segments];
        for (int i = 0; i < segments; i++)
        {
            var (s, c) = Math64.SinCos(6.283185307179586 * i / segments);
            v[i] = new Vector2F((float)(radius * c), (float)(radius * s));
        }

        return FromOutline(v);
    }

    private static void CheckSize(float v, string name)
    {
        if (!float.IsFinite(v) || !(v > 0f) || v > MaxCoordinate) { throw new SoftBodyException(SoftBodyError.OutOfRange, name + " is " + v.ToString("R", CultureInfo.InvariantCulture) + "; expected a finite size above 0 and at most " + MaxCoordinate.ToString("R", CultureInfo.InvariantCulture) + ".", name); }
    }

    private static void CheckCells(int n, string name)
    {
        if (n < 1 || n > 1024) { throw new SoftBodyException(SoftBodyError.OutOfRange, name + " is " + n.ToString(CultureInfo.InvariantCulture) + "; expected 1 to 1024.", name); }
    }
}
