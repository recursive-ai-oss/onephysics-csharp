using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Why a soft-body mesh or definition was refused.</summary>
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

    /// <summary>A triangle has (almost) no area or a tetrahedron (almost) no volume.</summary>
    Degenerate,

    /// <summary>An edge is shared by more than two triangles, or a face by more than two tetrahedra.</summary>
    NonManifold,

    /// <summary>A vertex belongs to no element, so nothing would hold it to the body.</summary>
    UnusedVertex,

    /// <summary>The operation needs a closed, consistently oriented triangle surface and the mesh is not one.</summary>
    NotClosed,

    /// <summary>A definition field is invalid (mass, compliance, radius, friction and so on).</summary>
    InvalidDefinition,
}

/// <summary>Thrown when a soft-body mesh or definition cannot be used. The message names the field or element and what was expected.</summary>
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
/// The rest shape of a soft body: vertices, and either triangles (cloth, shells, closed surfaces) or tetrahedra (volumes). Meshes are immutable and may be shared by any number of bodies.
/// Everything is validated when the mesh is made, so a mesh that exists is safe to build bodies from: finite coordinates within ±2^21, indices in range, no repeated vertex in an element,
/// no triangle or tetrahedron that is a sliver, at most two triangles on an edge, at most two tetrahedra on a face, and no vertex left out.
/// </summary>
/// <remarks>
/// Triangles are counter-clockwise seen from the side the normal points to. A tetrahedron given with negative volume is repaired by swapping its last two vertices.
/// A tetrahedral mesh also carries its outward-facing boundary triangles (<see cref="Triangles"/>), so a volume has a surface to draw and to apply pressure to.
/// </remarks>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public sealed class SoftBodyMesh
{
    /// <summary>Largest number of vertices in one mesh (and so particles in one body), 2^20.</summary>
    public const int MaxVertices = 1 << 20;

    /// <summary>Largest number of triangles or tetrahedra in one mesh, 2^22.</summary>
    public const int MaxElements = 1 << 22;

    /// <summary>Largest vertex coordinate magnitude, 2^21 metres.</summary>
    public const float MaxCoordinate = 2097152f;

    // A triangle whose doubled area is below this fraction of its longest edge squared, or a tetrahedron whose sixfold volume is below this fraction of its longest edge cubed, is a sliver.
    private const double SliverRatio = 1e-6;

    private readonly Vector3F[] _vertices;
    private readonly int[] _triangles;
    private readonly int[] _tetrahedra;
    private readonly bool _closed;

    private SoftBodyMesh(Vector3F[] vertices, int[] triangles, int[] tetrahedra, bool closed)
    {
        _vertices = vertices;
        _triangles = triangles;
        _tetrahedra = tetrahedra;
        _closed = closed;
    }

    /// <summary>Rest positions of the vertices, metres, in the mesh's own frame.</summary>
    public ReadOnlySpan<Vector3F> Vertices => _vertices;

    /// <summary>Triangle vertex indices, three per triangle: the mesh's own triangles, or for a tetrahedral mesh its outward boundary faces.</summary>
    public ReadOnlySpan<int> Triangles => _triangles;

    /// <summary>Tetrahedron vertex indices, four per tetrahedron; empty for a surface mesh.</summary>
    public ReadOnlySpan<int> Tetrahedra => _tetrahedra;

    /// <summary>Number of vertices.</summary>
    public int VertexCount => _vertices.Length;

    /// <summary>Number of triangles.</summary>
    public int TriangleCount => _triangles.Length / 3;

    /// <summary>Number of tetrahedra.</summary>
    public int TetrahedronCount => _tetrahedra.Length / 4;

    /// <summary>True when the triangles form a closed, consistently oriented surface (every edge has exactly two triangles, running in opposite directions), so the body can hold pressure.</summary>
    public bool IsClosed => _closed;

    // ------------------------------------------------------------------ construction

    /// <summary>Makes a surface mesh (cloth, a shell, or a closed surface that can hold pressure) from triangles.</summary>
    /// <param name="vertices">Rest positions; copied.</param>
    /// <param name="triangles">Three vertex indices per triangle, counter-clockwise; copied.</param>
    /// <exception cref="SoftBodyException">The input breaks a rule listed on <see cref="SoftBodyMesh"/>; the message names the element.</exception>
    public static SoftBodyMesh FromTriangles(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> triangles)
    {
        CheckVertices(vertices);
        if (triangles.Length == 0 || triangles.Length % 3 != 0 || triangles.Length / 3 > MaxElements)
        {
            throw new SoftBodyException(SoftBodyError.BadCount, "triangles has " + triangles.Length.ToString(CultureInfo.InvariantCulture) + " indices; expected a non-zero multiple of 3 with at most " + MaxElements.ToString(CultureInfo.InvariantCulture) + " triangles.", nameof(triangles));
        }

        var v = vertices.ToArray();
        var t = triangles.ToArray();
        CheckTriangles(v, t);
        bool closed = Analyse(t, v.Length);
        CheckUsed(v.Length, t, ReadOnlySpan<int>.Empty);
        return new SoftBodyMesh(v, t, Array.Empty<int>(), closed);
    }

    /// <summary>Makes a volume mesh from tetrahedra. The boundary triangles are derived, facing outward.</summary>
    /// <param name="vertices">Rest positions; copied.</param>
    /// <param name="tetrahedra">Four vertex indices per tetrahedron; copied. A negative-volume tetrahedron is repaired (its last two indices swapped).</param>
    /// <exception cref="SoftBodyException">The input breaks a rule listed on <see cref="SoftBodyMesh"/>; the message names the element.</exception>
    public static SoftBodyMesh FromTetrahedra(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> tetrahedra)
    {
        CheckVertices(vertices);
        if (tetrahedra.Length == 0 || tetrahedra.Length % 4 != 0 || tetrahedra.Length / 4 > MaxElements)
        {
            throw new SoftBodyException(SoftBodyError.BadCount, "tetrahedra has " + tetrahedra.Length.ToString(CultureInfo.InvariantCulture) + " indices; expected a non-zero multiple of 4 with at most " + MaxElements.ToString(CultureInfo.InvariantCulture) + " tetrahedra.", nameof(tetrahedra));
        }

        var v = vertices.ToArray();
        var t = tetrahedra.ToArray();
        CheckTetrahedra(v, t);
        CheckUsed(v.Length, ReadOnlySpan<int>.Empty, t);
        var surface = BoundaryFaces(v, t);
        return new SoftBodyMesh(v, surface, t, false);
    }

    private static void CheckVertices(ReadOnlySpan<Vector3F> v)
    {
        if (v.Length < 3 || v.Length > MaxVertices)
        {
            throw new SoftBodyException(SoftBodyError.BadCount, "vertices has " + v.Length.ToString(CultureInfo.InvariantCulture) + " entries; expected 3 to " + MaxVertices.ToString(CultureInfo.InvariantCulture) + ".", nameof(v));
        }

        for (int i = 0; i < v.Length; i++)
        {
            float x = v[i].X, y = v[i].Y, z = v[i].Z;
            if (!float.IsFinite(x) || !float.IsFinite(y) || !float.IsFinite(z))
            {
                throw new SoftBodyException(SoftBodyError.NonFinite, "vertex " + i.ToString(CultureInfo.InvariantCulture) + " is (" + x.ToString("R", CultureInfo.InvariantCulture) + ", " + y.ToString("R", CultureInfo.InvariantCulture) + ", " + z.ToString("R", CultureInfo.InvariantCulture) + "); coordinates must be finite.", "vertices");
            }

            if (Math.Abs(x) > MaxCoordinate || Math.Abs(y) > MaxCoordinate || Math.Abs(z) > MaxCoordinate)
            {
                throw new SoftBodyException(SoftBodyError.OutOfRange, "vertex " + i.ToString(CultureInfo.InvariantCulture) + " is (" + x.ToString("R", CultureInfo.InvariantCulture) + ", " + y.ToString("R", CultureInfo.InvariantCulture) + ", " + z.ToString("R", CultureInfo.InvariantCulture) + "); coordinates must be within ±" + MaxCoordinate.ToString("R", CultureInfo.InvariantCulture) + ".", "vertices");
            }
        }
    }

    private static void CheckIndices(Vector3F[] v, int[] idx, int perElement, string name)
    {
        for (int e = 0; e < idx.Length / perElement; e++)
        {
            for (int k = 0; k < perElement; k++)
            {
                int a = idx[perElement * e + k];
                if ((uint)a >= (uint)v.Length)
                {
                    throw new SoftBodyException(SoftBodyError.BadIndex, name + " " + e.ToString(CultureInfo.InvariantCulture) + " uses index " + a.ToString(CultureInfo.InvariantCulture) + "; vertex indices must be in 0 to " + (v.Length - 1).ToString(CultureInfo.InvariantCulture) + ".", name == "triangle" ? "triangles" : "tetrahedra");
                }

                for (int j = 0; j < k; j++)
                {
                    if (idx[perElement * e + j] == a)
                    {
                        throw new SoftBodyException(SoftBodyError.BadIndex, name + " " + e.ToString(CultureInfo.InvariantCulture) + " uses vertex " + a.ToString(CultureInfo.InvariantCulture) + " twice.", name == "triangle" ? "triangles" : "tetrahedra");
                    }
                }
            }
        }
    }

    private static double Edge2(Vector3F a, Vector3F b)
    {
        double dx = (double)a.X - b.X, dy = (double)a.Y - b.Y, dz = (double)a.Z - b.Z;
        return dx * dx + dy * dy + dz * dz;
    }

    private static void CheckTriangles(Vector3F[] v, int[] t)
    {
        CheckIndices(v, t, 3, "triangle");
        for (int e = 0; e < t.Length / 3; e++)
        {
            var a = v[t[3 * e]]; var b = v[t[3 * e + 1]]; var c = v[t[3 * e + 2]];
            double ux = (double)b.X - a.X, uy = (double)b.Y - a.Y, uz = (double)b.Z - a.Z;
            double wx = (double)c.X - a.X, wy = (double)c.Y - a.Y, wz = (double)c.Z - a.Z;
            double nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx;
            double area2 = nx * nx + ny * ny + nz * nz; // (twice the area) squared
            double longest = Math.Max(Edge2(a, b), Math.Max(Edge2(b, c), Edge2(c, a)));
            if (!(area2 > SliverRatio * SliverRatio * longest * longest) || !(longest > 0.0))
            {
                throw new SoftBodyException(SoftBodyError.Degenerate, "triangle " + e.ToString(CultureInfo.InvariantCulture) + " (vertices " + t[3 * e].ToString(CultureInfo.InvariantCulture) + ", " + t[3 * e + 1].ToString(CultureInfo.InvariantCulture) + ", " + t[3 * e + 2].ToString(CultureInfo.InvariantCulture) + ") has no usable area (zero, collinear or a sliver).", "triangles");
            }
        }
    }

    private static void CheckTetrahedra(Vector3F[] v, int[] t)
    {
        CheckIndices(v, t, 4, "tetrahedron");
        for (int e = 0; e < t.Length / 4; e++)
        {
            var a = v[t[4 * e]]; var b = v[t[4 * e + 1]]; var c = v[t[4 * e + 2]]; var d = v[t[4 * e + 3]];
            double ux = (double)b.X - a.X, uy = (double)b.Y - a.Y, uz = (double)b.Z - a.Z;
            double wx = (double)c.X - a.X, wy = (double)c.Y - a.Y, wz = (double)c.Z - a.Z;
            double sx = (double)d.X - a.X, sy = (double)d.Y - a.Y, sz = (double)d.Z - a.Z;
            double vol6 = sx * (uy * wz - uz * wy) + sy * (uz * wx - ux * wz) + sz * (ux * wy - uy * wx);
            double longest = Math.Max(Math.Max(Edge2(a, b), Math.Max(Edge2(a, c), Edge2(a, d))), Math.Max(Edge2(b, c), Math.Max(Edge2(b, d), Edge2(c, d))));
            if (!(Math.Abs(vol6) > SliverRatio * longest * Math.Sqrt(longest)) || !(longest > 0.0))
            {
                throw new SoftBodyException(SoftBodyError.Degenerate, "tetrahedron " + e.ToString(CultureInfo.InvariantCulture) + " (vertices " + t[4 * e].ToString(CultureInfo.InvariantCulture) + ", " + t[4 * e + 1].ToString(CultureInfo.InvariantCulture) + ", " + t[4 * e + 2].ToString(CultureInfo.InvariantCulture) + ", " + t[4 * e + 3].ToString(CultureInfo.InvariantCulture) + ") has no usable volume (flat or a sliver).", "tetrahedra");
            }

            if (vol6 < 0.0) { (t[4 * e + 2], t[4 * e + 3]) = (t[4 * e + 3], t[4 * e + 2]); }
        }
    }

    private static void CheckUsed(int vertexCount, ReadOnlySpan<int> triangles, ReadOnlySpan<int> tets)
    {
        var used = new bool[vertexCount];
        foreach (int i in triangles) { used[i] = true; }
        foreach (int i in tets) { used[i] = true; }
        for (int i = 0; i < used.Length; i++)
        {
            if (!used[i]) { throw new SoftBodyException(SoftBodyError.UnusedVertex, "vertex " + i.ToString(CultureInfo.InvariantCulture) + " belongs to no element.", "vertices"); }
        }
    }

    // Directed edge key: from * 2^20 + to (both below 2^20). Sorting the keys of every triangle edge puts the two sides of an edge together.
    private static bool Analyse(int[] t, int vertexCount)
    {
        int n = t.Length;
        var undirected = new long[n];
        var directed = new long[n];
        for (int e = 0; e < n / 3; e++)
        {
            for (int k = 0; k < 3; k++)
            {
                long a = t[3 * e + k], b = t[3 * e + (k + 1) % 3];
                directed[3 * e + k] = (a << 20) | b;
                undirected[3 * e + k] = a < b ? (a << 20) | b : (b << 20) | a;
            }
        }

        Array.Sort(undirected);
        Array.Sort(directed);
        bool closed = true;
        for (int i = 0; i < n;)
        {
            int j = i;
            while (j < n && undirected[j] == undirected[i]) { j++; }
            int count = j - i;
            if (count > 2)
            {
                long k = undirected[i];
                throw new SoftBodyException(SoftBodyError.NonManifold, "the edge between vertices " + (k >> 20).ToString(CultureInfo.InvariantCulture) + " and " + (k & 0xFFFFF).ToString(CultureInfo.InvariantCulture) + " is shared by " + count.ToString(CultureInfo.InvariantCulture) + " triangles; at most 2 are allowed.", "triangles");
            }

            if (count != 2) { closed = false; }
            i = j;
        }

        // Closed and consistently oriented: no directed edge appears twice.
        if (closed)
        {
            for (int i = 1; i < n; i++)
            {
                if (directed[i] == directed[i - 1]) { closed = false; break; }
            }
        }

        return closed;
    }

    // Faces of the tetrahedra used once, oriented away from the tetrahedron's fourth vertex. A face used by three or more tetrahedra is non-manifold.
    private static int[] BoundaryFaces(Vector3F[] v, int[] t)
    {
        int nt = t.Length / 4;
        var keys = new long[4 * nt];
        var owner = new int[4 * nt];
        for (int e = 0; e < nt; e++)
        {
            for (int f = 0; f < 4; f++)
            {
                int a = t[4 * e + (f == 0 ? 1 : 0)], b = t[4 * e + (f <= 1 ? 2 : 1)], c = t[4 * e + (f <= 2 ? 3 : 2)];
                // sort the three indices for the key
                if (a > b) { (a, b) = (b, a); }
                if (b > c) { (b, c) = (c, b); }
                if (a > b) { (a, b) = (b, a); }
                keys[4 * e + f] = ((long)a << 40) | ((long)b << 20) | (long)c;
                owner[4 * e + f] = 4 * e + f;
            }
        }

        Array.Sort(keys, owner);
        var faces = new List<int>();
        for (int i = 0; i < keys.Length;)
        {
            int j = i;
            while (j < keys.Length && keys[j] == keys[i]) { j++; }
            int count = j - i;
            if (count > 2)
            {
                throw new SoftBodyException(SoftBodyError.NonManifold, "a face is shared by " + count.ToString(CultureInfo.InvariantCulture) + " tetrahedra; at most 2 are allowed.", "tetrahedra");
            }

            if (count == 1)
            {
                int o = owner[i], e = o >> 2, f = o & 3;
                // face f is opposite vertex f of the tetrahedron
                int i0 = t[4 * e + (f == 0 ? 1 : 0)], i1 = t[4 * e + (f <= 1 ? 2 : 1)], i2 = t[4 * e + (f <= 2 ? 3 : 2)], opp = t[4 * e + f];
                var p0 = v[i0]; var p1 = v[i1]; var p2 = v[i2]; var po = v[opp];
                double ux = (double)p1.X - p0.X, uy = (double)p1.Y - p0.Y, uz = (double)p1.Z - p0.Z;
                double wx = (double)p2.X - p0.X, wy = (double)p2.Y - p0.Y, wz = (double)p2.Z - p0.Z;
                double nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx;
                double side = nx * ((double)po.X - p0.X) + ny * ((double)po.Y - p0.Y) + nz * ((double)po.Z - p0.Z);
                if (side > 0.0) { (i1, i2) = (i2, i1); }
                faces.Add(i0); faces.Add(i1); faces.Add(i2);
            }

            i = j;
        }

        return faces.ToArray();
    }

    // ------------------------------------------------------------------ generators

    /// <summary>
    /// A flat sheet in the x–z plane centred on the origin, facing +y, made of (<paramref name="segmentsX"/> × <paramref name="segmentsZ"/> × 2) triangles. Vertex (i, j) has index <c>j * (segmentsX + 1) + i</c>.
    /// </summary>
    /// <param name="width">Size along x, metres; finite, positive.</param>
    /// <param name="depth">Size along z, metres; finite, positive.</param>
    /// <param name="segmentsX">Cells along x, 1 to 1024.</param>
    /// <param name="segmentsZ">Cells along z, 1 to 1024.</param>
    /// <exception cref="SoftBodyException">An argument is out of range.</exception>
    public static SoftBodyMesh Cloth(float width, float depth, int segmentsX, int segmentsZ)
    {
        CheckSize(width, "width"); CheckSize(depth, "depth");
        CheckSegments(segmentsX, "segmentsX"); CheckSegments(segmentsZ, "segmentsZ");
        int nx = segmentsX + 1, nz = segmentsZ + 1;
        var v = new Vector3F[nx * nz];
        for (int j = 0; j < nz; j++)
        {
            for (int i = 0; i < nx; i++)
            {
                v[j * nx + i] = new Vector3F(width * ((float)i / segmentsX - 0.5f), 0f, depth * ((float)j / segmentsZ - 0.5f));
            }
        }

        var t = new int[segmentsX * segmentsZ * 6];
        int n = 0;
        for (int j = 0; j < segmentsZ; j++)
        {
            for (int i = 0; i < segmentsX; i++)
            {
                int a = j * nx + i, b = a + 1, c = a + nx, d = c + 1;
                // counter-clockwise seen from +y: with x to the right and z towards the viewer, (a, c, b) turns left
                t[n++] = a; t[n++] = c; t[n++] = b;
                t[n++] = b; t[n++] = c; t[n++] = d;
            }
        }

        return FromTriangles(v, t);
    }

    /// <summary>
    /// A box of tetrahedra centred on the origin, <paramref name="cellsX"/> × <paramref name="cellsY"/> × <paramref name="cellsZ"/> cells of six tetrahedra each (a conforming split along each cell's diagonal).
    /// Vertex (i, j, k) has index <c>(k * (cellsY + 1) + j) * (cellsX + 1) + i</c>.
    /// </summary>
    /// <param name="size">Edge lengths, metres; each finite and positive.</param>
    /// <param name="cellsX">Cells along x, 1 to 64.</param>
    /// <param name="cellsY">Cells along y, 1 to 64.</param>
    /// <param name="cellsZ">Cells along z, 1 to 64.</param>
    /// <exception cref="SoftBodyException">An argument is out of range.</exception>
    public static SoftBodyMesh Box(Vector3F size, int cellsX, int cellsY, int cellsZ)
    {
        CheckSize(size.X, "size.X"); CheckSize(size.Y, "size.Y"); CheckSize(size.Z, "size.Z");
        CheckCells(cellsX, "cellsX"); CheckCells(cellsY, "cellsY"); CheckCells(cellsZ, "cellsZ");
        int nx = cellsX + 1, ny = cellsY + 1, nz = cellsZ + 1;
        var v = new Vector3F[nx * ny * nz];
        for (int k = 0; k < nz; k++)
        {
            for (int j = 0; j < ny; j++)
            {
                for (int i = 0; i < nx; i++)
                {
                    v[(k * ny + j) * nx + i] = new Vector3F(size.X * ((float)i / cellsX - 0.5f), size.Y * ((float)j / cellsY - 0.5f), size.Z * ((float)k / cellsZ - 0.5f));
                }
            }
        }

        // Kuhn's split: the six paths from a cell's lowest corner to its highest through one step along each axis in turn.
        ReadOnlySpan<int> perms = [0, 1, 2, 0, 2, 1, 1, 0, 2, 1, 2, 0, 2, 0, 1, 2, 1, 0];
        var t = new int[cellsX * cellsY * cellsZ * 24];
        int n = 0;
        for (int k = 0; k < cellsZ; k++)
        {
            for (int j = 0; j < cellsY; j++)
            {
                for (int i = 0; i < cellsX; i++)
                {
                    for (int p = 0; p < 6; p++)
                    {
                        int x = i, y = j, z = k;
                        t[n++] = (z * ny + y) * nx + x;
                        for (int s = 0; s < 3; s++)
                        {
                            int axis = perms[3 * p + s];
                            if (axis == 0) { x++; } else if (axis == 1) { y++; } else { z++; }
                            t[n++] = (z * ny + y) * nx + x;
                        }
                    }
                }
            }
        }

        return FromTetrahedra(v, t);
    }

    /// <summary>
    /// A closed, consistently oriented sphere surface (a cube whose faces are divided into <paramref name="segments"/> × <paramref name="segments"/> cells, pushed out to the sphere), which can hold pressure.
    /// Its triangles face outward.
    /// </summary>
    /// <param name="radius">Metres; finite, positive.</param>
    /// <param name="segments">Cells along each cube edge, 1 to 128.</param>
    /// <exception cref="SoftBodyException">An argument is out of range.</exception>
    public static SoftBodyMesh Sphere(float radius, int segments)
    {
        CheckSize(radius, "radius");
        if (segments < 1 || segments > 128) { throw new SoftBodyException(SoftBodyError.OutOfRange, "segments is " + segments.ToString(CultureInfo.InvariantCulture) + "; expected 1 to 128.", nameof(segments)); }
        int m = segments + 1;
        var verts = new List<Vector3F>();
        var index = new Dictionary<long, int>();
        var tris = new List<int>();
        // Vertices on the cube surface are shared between faces by their integer grid position (0..segments on each axis), looked up through a dictionary used only for index lookup, never iterated.
        int Vertex(int x, int y, int z)
        {
            long key = ((long)x << 40) | ((long)y << 20) | (long)z;
            if (index.TryGetValue(key, out int found)) { return found; }
            float fx = 2f * x / segments - 1f, fy = 2f * y / segments - 1f, fz = 2f * z / segments - 1f;
            // Nowhere near 0: a surface point of the cube has one coordinate at ±1.
            float len = float.Sqrt(fx * fx + fy * fy + fz * fz);
            verts.Add(new Vector3F(radius * fx / len, radius * fy / len, radius * fz / len));
            index[key] = verts.Count - 1;
            return verts.Count - 1;
        }

        for (int face = 0; face < 6; face++)
        {
            int axis = face >> 1, hi = face & 1;
            for (int a = 0; a < segments; a++)
            {
                for (int b = 0; b < segments; b++)
                {
                    int Corner(int da, int db)
                    {
                        int u = a + da, w = b + db;
                        int x, y, z;
                        if (axis == 0) { x = hi * segments; y = u; z = w; }
                        else if (axis == 1) { y = hi * segments; z = u; x = w; }
                        else { z = hi * segments; x = u; y = w; }
                        return Vertex(x, y, z);
                    }

                    int c00 = Corner(0, 0), c10 = Corner(1, 0), c01 = Corner(0, 1), c11 = Corner(1, 1);
                    // (u, w) axes of face 0 (+/-x) are (y, z); of face 1 (y) are (z, x); of face 2 (z) are (x, y): all right-handed with the axis, so the +side faces outward counter-clockwise.
                    if (hi == 1) { tris.Add(c00); tris.Add(c10); tris.Add(c11); tris.Add(c00); tris.Add(c11); tris.Add(c01); }
                    else { tris.Add(c00); tris.Add(c11); tris.Add(c10); tris.Add(c00); tris.Add(c01); tris.Add(c11); }
                }
            }
        }

        _ = m;
        return FromTriangles(verts.ToArray(), tris.ToArray());
    }

    private static void CheckSize(float v, string name)
    {
        if (!float.IsFinite(v) || !(v > 0f) || v > MaxCoordinate) { throw new SoftBodyException(SoftBodyError.OutOfRange, name + " is " + v.ToString("R", CultureInfo.InvariantCulture) + "; expected a finite size above 0 and at most " + MaxCoordinate.ToString("R", CultureInfo.InvariantCulture) + ".", name); }
    }

    private static void CheckSegments(int n, string name)
    {
        if (n < 1 || n > 1024) { throw new SoftBodyException(SoftBodyError.OutOfRange, name + " is " + n.ToString(CultureInfo.InvariantCulture) + "; expected 1 to 1024.", name); }
    }

    private static void CheckCells(int n, string name)
    {
        if (n < 1 || n > 64) { throw new SoftBodyException(SoftBodyError.OutOfRange, name + " is " + n.ToString(CultureInfo.InvariantCulture) + "; expected 1 to 64.", name); }
    }
}
