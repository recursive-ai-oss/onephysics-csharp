using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>Options for <see cref="Tetrahedralizer.Generate"/>. The default (all zero) is a mesh with about 16 tetrahedra across the longest axis, radius-edge ratio at most 2.</summary>
public readonly struct TetrahedralizeOptions
{
    /// <summary>Side of the lattice cell the tetrahedra are cut from, m; the edges of the interior tetrahedra are this long (a third of them) or 0.866 times it. 0 chooses the longest axis extent divided by 16 (or, when <see cref="MaxVolume"/> is set, the cell that gives that volume).</summary>
    public float TargetEdgeLength { get; init; }

    /// <summary>Largest tetrahedron volume wanted, m³ (interior tetrahedra have a twelfth of the cell cubed). 0 means no limit. When both this and <see cref="TargetEdgeLength"/> are set the finer one wins.</summary>
    public float MaxVolume { get; init; }

    /// <summary>The radius-edge ratio (circumradius over shortest edge) a boundary vertex move may not exceed; moves that would make a tetrahedron worse than this are not made. From 0.8 to 100; 0 chooses 2 (the ratio of the lattice's own tetrahedra is 0.79).</summary>
    public float MaxRadiusEdgeRatio { get; init; }

    /// <summary>Threads for the classification and measuring passes; 0 or 1 is single-threaded. The result is the same for every value.</summary>
    public int MaxThreads { get; init; }

    /// <summary>Skips the surface deviation measurement (<see cref="TetrahedralMesh.SurfaceDeviation"/> is then NaN); the surface bindings are made either way.</summary>
    public bool SkipDeviation { get; init; }
}

/// <summary>Where a vertex of the input surface sits in the tetrahedral mesh: a point of a boundary face, as weights on the four vertices of the tetrahedron that owns the face (the fourth vertex's weight is zero). Interpolating the tetrahedra's displacement with these weights skins the input mesh onto a soft body.</summary>
public readonly struct SurfaceBinding
{
    /// <summary>Creates a binding.</summary>
    public SurfaceBinding(int tetrahedron, float w0, float w1, float w2, float w3) { Tetrahedron = tetrahedron; W0 = w0; W1 = w1; W2 = w2; W3 = w3; }

    /// <summary>Index of the tetrahedron.</summary>
    public int Tetrahedron { get; }

    /// <summary>Weight of the tetrahedron's first vertex.</summary>
    public float W0 { get; }

    /// <summary>Weight of the second vertex.</summary>
    public float W1 { get; }

    /// <summary>Weight of the third vertex.</summary>
    public float W2 { get; }

    /// <summary>Weight of the fourth vertex.</summary>
    public float W3 { get; }
}

/// <summary>A tetrahedral mesh of a closed surface and how well it fits.</summary>
public sealed class TetrahedralMesh
{
    internal TetrahedralMesh(Vector3F[] vertices, int[] tetrahedra, int[] surface, SurfaceBinding[] bindings, double volume, double meshVolume, double worstRadiusEdge, double minVolumeRatio, double deviation, double edgeLength, bool qualityMet)
    {
        Vertices = vertices; Tetrahedra = tetrahedra; SurfaceTriangles = surface; Bindings = bindings; Volume = volume; MeshVolume = meshVolume;
        WorstRadiusEdge = worstRadiusEdge; MinVolumeRatio = minVolumeRatio; SurfaceDeviation = deviation; CellSize = edgeLength; QualityMet = qualityMet;
    }

    /// <summary>Vertex positions, in the input mesh's coordinates.</summary>
    public Vector3F[] Vertices { get; }

    /// <summary>Four vertex indices per tetrahedron; every tetrahedron has positive volume (vertices 0, 1, 2 seen from vertex 3 run counter-clockwise).</summary>
    public int[] Tetrahedra { get; }

    /// <summary>The outward-facing boundary triangles, three indices each, counter-clockwise seen from outside.</summary>
    public int[] SurfaceTriangles { get; }

    /// <summary>One binding per vertex of the input surface (in the input's vertex order): the point of the boundary nearest to it.</summary>
    public SurfaceBinding[] Bindings { get; }

    /// <summary>Total volume of the tetrahedra, m³.</summary>
    public double Volume { get; }

    /// <summary>Volume enclosed by the input surface, m³.</summary>
    public double MeshVolume { get; }

    /// <summary>The largest circumradius-to-shortest-edge ratio over all tetrahedra.</summary>
    public double WorstRadiusEdge { get; }

    /// <summary>The smallest ratio of six times the volume to the cube of the longest edge over all tetrahedra (0.5 for the lattice's own tetrahedra, 0.71 for a regular one).</summary>
    public double MinVolumeRatio { get; }

    /// <summary>The Hausdorff distance, m, between the input surface and the mesh boundary, measured both ways at the vertices, edge midpoints and face centres of the boundary and at the input's vertices; NaN when skipped.</summary>
    public double SurfaceDeviation { get; }

    /// <summary>The lattice cell side that was used, m.</summary>
    public double CellSize { get; }

    /// <summary>True when every tetrahedron meets <see cref="TetrahedralizeOptions.MaxRadiusEdgeRatio"/> (moves of boundary vertices that would break it are refused, so this is false only when the lattice itself is worse, which cannot happen at the default).</summary>
    public bool QualityMet { get; }

    /// <summary>Makes the soft-body mesh of the tetrahedra.</summary>
    /// <exception cref="RecursiveAI.OnePhysics.Physics3D.SoftBodyException">The soft-body mesh limits are exceeded (vertex and element counts).</exception>
    public RecursiveAI.OnePhysics.Physics3D.SoftBodyMesh ToSoftBodyMesh() => RecursiveAI.OnePhysics.Physics3D.SoftBodyMesh.FromTetrahedra(Vertices, Tetrahedra);
}

/// <summary>
/// Fills a closed triangle mesh with well-shaped tetrahedra for soft bodies. The tetrahedra are cut from a body-centred cubic lattice (the Delaunay tetrahedralization of that lattice, with radius-edge ratio 0.79 and
/// edges of two lengths), the way isosurface stuffing builds its meshes (Labelle and Shewchuk, SIGGRAPH 2007): a tetrahedron is kept when its centroid is inside the surface, and the vertices of kept tetrahedra near
/// the surface are moved onto it, a move being made only if every tetrahedron around the vertex keeps positive volume, a volume ratio above 0.02 and a radius-edge ratio within bounds (Si, TetGen, ACM TOMS 2015, for the quality measures).
/// Inside tests use exact predicates (onemath), so the result is a pure function of the input and options, the same on every platform and at every thread count.
/// </summary>
public static class Tetrahedralizer
{
    private const double MinVolumeRatioGate = 0.3;
    private const int SnapSweeps = 3;
    private static readonly double[] SnapLadder = { 1.0, 0.75, 0.5, 0.25 };
    private static readonly double[] SampleU = { 1, 0, 0, 0.5, 0, 0.5, 1.0 / 3 };
    private static readonly double[] SampleV = { 0, 1, 0, 0.5, 0.5, 0, 1.0 / 3 };

    /// <summary>Tetrahedralizes a closed, consistently oriented surface.</summary>
    /// <param name="vertices">Vertex positions, finite, within ±2^21.</param>
    /// <param name="indices">Three vertex indices per triangle, counter-clockwise seen from outside. Use <see cref="MeshCleaner"/> first on meshes with duplicate vertices or mixed winding. A self-intersecting surface is not detected; its result is unspecified but valid.</param>
    /// <param name="options">Sizes and quality; see <see cref="TetrahedralizeOptions"/>.</param>
    /// <returns>The tetrahedra, the boundary, one binding per input vertex and the measured quality.</returns>
    /// <exception cref="GeometryToolException">The mesh is empty, has non-finite or out-of-range values or bad indices, is open (<see cref="GeometryToolError.OpenMesh"/>), non-manifold, has a negative volume or none, an option is invalid, the requested size is too fine for the limits, or no tetrahedron fits inside (the features are thinner than the cell size).</exception>
    public static TetrahedralMesh Generate(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices, TetrahedralizeOptions options = default)
    {
        MeshInput.Check(vertices, indices);
        MeshMass.ComputeDouble(vertices, indices);   // closed, manifold and positively oriented, or a typed error
        int threads = options.MaxThreads <= 1 ? 1 : options.MaxThreads;
        if (threads > 256) { throw Bad("MaxThreads", options.MaxThreads.ToString(System.Globalization.CultureInfo.InvariantCulture), "at most 256", nameof(options)); }
        double bound = options.MaxRadiusEdgeRatio == 0 ? 2.0 : options.MaxRadiusEdgeRatio;
        if (!(bound >= 0.8 && bound <= 100)) { throw Bad("MaxRadiusEdgeRatio", bound.ToString("R", System.Globalization.CultureInfo.InvariantCulture), "from 0.8 to 100", nameof(options)); }
        if (!(options.TargetEdgeLength >= 0) || !float.IsFinite(options.TargetEdgeLength)) { throw Bad("TargetEdgeLength", options.TargetEdgeLength.ToString("R", System.Globalization.CultureInfo.InvariantCulture), "finite and at least 0", nameof(options)); }
        if (!(options.MaxVolume >= 0) || !float.IsFinite(options.MaxVolume)) { throw Bad("MaxVolume", options.MaxVolume.ToString("R", System.Globalization.CultureInfo.InvariantCulture), "finite and at least 0", nameof(options)); }

        int nv = vertices.Length, nt = indices.Length / 3;
        var mx = new double[nv]; var my = new double[nv]; var mz = new double[nv];
        double x0 = double.MaxValue, y0 = double.MaxValue, z0 = double.MaxValue, x1 = double.MinValue, y1 = double.MinValue, z1 = double.MinValue;
        for (int i = 0; i < nv; i++)
        {
            mx[i] = vertices[i].X; my[i] = vertices[i].Y; mz[i] = vertices[i].Z;
            x0 = Math.Min(x0, mx[i]); x1 = Math.Max(x1, mx[i]); y0 = Math.Min(y0, my[i]); y1 = Math.Max(y1, my[i]); z0 = Math.Min(z0, mz[i]); z1 = Math.Max(z1, mz[i]);
        }

        double extent = Math.Max(x1 - x0, Math.Max(y1 - y0, z1 - z0));
        double a = options.TargetEdgeLength > 0 ? options.TargetEdgeLength : (options.MaxVolume > 0 ? double.MaxValue : extent / 16.0);
        if (options.MaxVolume > 0) { a = Math.Min(a, Math64.Cbrt(12.0 * options.MaxVolume)); }
        if (!(a > 0) || !double.IsFinite(a)) { throw Bad("TargetEdgeLength", a.ToString("R", System.Globalization.CultureInfo.InvariantCulture), "positive", nameof(options)); }
        if (extent / a > 512) { throw Bad("TargetEdgeLength", a.ToString("R", System.Globalization.CultureInfo.InvariantCulture), "at least the longest extent (" + extent.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ") divided by 512", nameof(options)); }

        // The lattice origin sits a little over one and a half cells before the mesh, at an irregular fraction, so lattice planes rarely coincide with flat faces of the input.
        double ox = x0 - a * 1.818, oy = y0 - a * 1.641, oz = z0 - a * 1.777;
        int nx = (int)Math.Ceiling((x1 - ox) / a) + 2, ny = (int)Math.Ceiling((y1 - oy) / a) + 2, nz = (int)Math.Ceiling((z1 - oz) / a) + 2;
        long cellCount = (long)nx * ny * nz;
        if (cellCount > (1L << 23)) { throw Bad("TargetEdgeLength", a.ToString("R", System.Globalization.CultureInfo.InvariantCulture), "larger: the lattice would need " + cellCount.ToString(System.Globalization.CultureInfo.InvariantCulture) + " cells, the limit is 2^23", nameof(options)); }

        var tri = indices.ToArray();
        var grid = new TriangleGrid(mx, my, mz, tri, ox, oy, oz, a, nx, ny, nz);

        // ---- classify cells and tetrahedra
        int gx = nx + 1, gy = ny + 1, gz = nz + 1;
        long gridPoints = (long)gx * gy * gz;
        var cellInside = new byte[cellCount];       // parity at the cell centre
        var cellEmpty = new byte[cellCount];
        ParallelRange(nz, threads, (k0, k1) =>
        {
            for (int k = k0; k < k1; k++)
            {
                for (int j = 0; j < ny; j++)
                {
                    for (int i = 0; i < nx; i++)
                    {
                        long c = i + (long)nx * (j + (long)ny * k);
                        cellEmpty[c] = grid.CellHasTriangles(i, j, k) ? (byte)0 : (byte)1;
                        cellInside[c] = grid.Inside(ox + (i + 0.5) * a, oy + (j + 0.5) * a, oz + (k + 0.5) * a) ? (byte)1 : (byte)0;
                    }
                }
            }
        });

        // Orientation of the four tetrahedra around an edge along each axis: the sign of the determinant in lattice units (grid points at even coordinates, cell centres at odd).
        var swap = new bool[3, 4];
        var cellOffset = new int[3, 4, 2, 2];   // [axis, pair, which centre, (db, dc)] in cell offsets from the edge's grid point
        int[][] around = { new[] { -1, -1 }, new[] { 0, -1 }, new[] { 0, 0 }, new[] { -1, 0 } };
        for (int axis = 0; axis < 3; axis++)
        {
            for (int pair = 0; pair < 4; pair++)
            {
                var c1 = around[pair]; var c2 = around[(pair + 1) % 4];
                cellOffset[axis, pair, 0, 0] = c1[0]; cellOffset[axis, pair, 0, 1] = c1[1]; cellOffset[axis, pair, 1, 0] = c2[0]; cellOffset[axis, pair, 1, 1] = c2[1];
                // lattice coordinates relative to the edge's grid point: edge end (2 along axis), centres at (1 along axis, 2*d + 1 across)
                long[] e = new long[3], p1 = new long[3], p2 = new long[3];
                int b = (axis + 1) % 3, c = (axis + 2) % 3;
                e[axis] = 2;
                p1[axis] = 1; p1[b] = 2 * c1[0] + 1; p1[c] = 2 * c1[1] + 1;
                p2[axis] = 1; p2[b] = 2 * c2[0] + 1; p2[c] = 2 * c2[1] + 1;
                long det = e[0] * (p1[1] * p2[2] - p1[2] * p2[1]) - e[1] * (p1[0] * p2[2] - p1[2] * p2[0]) + e[2] * (p1[0] * p2[1] - p1[1] * p2[0]);
                swap[axis, pair] = det < 0;
            }
        }

        // keep[(axis * gridPoints + gridPoint) * 4 + pair]
        var keep = new byte[3 * gridPoints * 4];
        ParallelRange(gz, threads, (k0, k1) =>
        {
            for (int k = k0; k < k1; k++)
            {
                for (int j = 0; j < gy; j++)
                {
                    for (int i = 0; i < gx; i++)
                    {
                        for (int axis = 0; axis < 3; axis++)
                        {
                            int ia = axis == 0 ? i : axis == 1 ? j : k;
                            int lim = axis == 0 ? nx : axis == 1 ? ny : nz;
                            if (ia >= lim) { continue; }
                            long baseIndex = (axis * gridPoints + i + (long)gx * (j + (long)gy * k)) * 4;
                            for (int pair = 0; pair < 4; pair++)
                            {
                                bool ok = true;
                                int allEmpty;
                                double cxs = 0, cys = 0, czs = 0;
                                for (int w = 0; w < 2 && ok; w++)
                                {
                                    int[] pos = { i, j, k };
                                    pos[(axis + 1) % 3] += cellOffset[axis, pair, w, 0];
                                    pos[(axis + 2) % 3] += cellOffset[axis, pair, w, 1];
                                    if (pos[0] < 0 || pos[1] < 0 || pos[2] < 0 || pos[0] >= nx || pos[1] >= ny || pos[2] >= nz) { ok = false; break; }
                                    cxs += ox + (pos[0] + 0.5) * a; cys += oy + (pos[1] + 0.5) * a; czs += oz + (pos[2] + 0.5) * a;
                                }

                                if (!ok) { continue; }
                                // The four cells around the edge; the tetrahedron lies inside their union.
                                bool inside;
                                allEmpty = 1;
                                int first = -1;
                                for (int q = 0; q < 4; q++)
                                {
                                    int[] pos = { i, j, k };
                                    pos[(axis + 1) % 3] += around[q][0];
                                    pos[(axis + 2) % 3] += around[q][1];
                                    if (pos[0] < 0 || pos[1] < 0 || pos[2] < 0 || pos[0] >= nx || pos[1] >= ny || pos[2] >= nz) { allEmpty = 0; continue; }
                                    long cidx = pos[0] + (long)nx * (pos[1] + (long)ny * pos[2]);
                                    if (cellEmpty[cidx] == 0) { allEmpty = 0; }
                                    else if (first < 0) { first = (int)cidx; }
                                }

                                if (allEmpty == 1)
                                {
                                    inside = cellInside[first] != 0;
                                }
                                else
                                {
                                    // centroid of the tetrahedron: the edge's midpoint and the two centres, a quarter weight on each end and a quarter on each centre
                                    double ex = ox + i * a, ey = oy + j * a, ez = oz + k * a;
                                    double mxp = ex + (axis == 0 ? 0.5 * a : 0), myp = ey + (axis == 1 ? 0.5 * a : 0), mzp = ez + (axis == 2 ? 0.5 * a : 0);
                                    double px = 0.5 * mxp + 0.25 * cxs, py = 0.5 * myp + 0.25 * cys, pz = 0.5 * mzp + 0.25 * czs;
                                    inside = grid.Inside(px, py, pz);
                                }

                                keep[baseIndex + pair] = inside ? (byte)1 : (byte)0;
                            }
                        }
                    }
                }
            }
        });

        // ---- collect kept tetrahedra with lattice vertex ids
        long centreBase = gridPoints;
        var tetList = new List<long>();   // four ids per tetrahedron
        for (int axis = 0; axis < 3; axis++)
        {
            for (int k = 0; k < gz; k++)
            {
                for (int j = 0; j < gy; j++)
                {
                    for (int i = 0; i < gx; i++)
                    {
                        long gp = i + (long)gx * (j + (long)gy * k);
                        long baseIndex = (axis * gridPoints + gp) * 4;
                        for (int pair = 0; pair < 4; pair++)
                        {
                            if (keep[baseIndex + pair] == 0) { continue; }
                            int[] e1 = { i, j, k };
                            e1[axis]++;
                            long pA = gp, pB = e1[0] + (long)gx * (e1[1] + (long)gy * e1[2]);
                            long c1, c2;
                            {
                                int[] pos = { i, j, k };
                                pos[(axis + 1) % 3] += cellOffset[axis, pair, 0, 0]; pos[(axis + 2) % 3] += cellOffset[axis, pair, 0, 1];
                                c1 = centreBase + pos[0] + (long)nx * (pos[1] + (long)ny * pos[2]);
                                pos = new[] { i, j, k };
                                pos[(axis + 1) % 3] += cellOffset[axis, pair, 1, 0]; pos[(axis + 2) % 3] += cellOffset[axis, pair, 1, 1];
                                c2 = centreBase + pos[0] + (long)nx * (pos[1] + (long)ny * pos[2]);
                            }

                            if (swap[axis, pair]) { (c1, c2) = (c2, c1); }
                            tetList.Add(pA); tetList.Add(pB); tetList.Add(c1); tetList.Add(c2);
                        }
                    }
                }
            }
        }

        if (tetList.Count == 0) { throw new GeometryToolException(GeometryToolError.Degenerate, "no tetrahedron fits inside the surface at an edge length of " + a.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + "; use a smaller TargetEdgeLength", nameof(options)); }

        // ---- renumber the vertices that are used, in lattice id order
        long totalIds = gridPoints + cellCount;
        var remap = new int[totalIds];
        Array.Fill(remap, -1);
        foreach (long id in tetList) { remap[id] = 0; }
        int used = 0;
        for (long id = 0; id < totalIds; id++) { if (remap[id] == 0) { remap[id] = used++; } }
        if (used > 1 << 20 || tetList.Count / 4 > 1 << 22) { throw Bad("TargetEdgeLength", a.ToString("R", System.Globalization.CultureInfo.InvariantCulture), "larger: the mesh would have " + used.ToString(System.Globalization.CultureInfo.InvariantCulture) + " vertices and " + (tetList.Count / 4).ToString(System.Globalization.CultureInfo.InvariantCulture) + " tetrahedra, over the soft-body limits", nameof(options)); }
        var px_ = new double[used]; var py_ = new double[used]; var pz_ = new double[used];
        for (long id = 0; id < totalIds; id++)
        {
            int r = remap[id];
            if (r < 0) { continue; }
            double fx, fy, fz;
            if (id < gridPoints) { long q = id; int i = (int)(q % gx); q /= gx; int j = (int)(q % gy); int k = (int)(q / gy); fx = ox + i * a; fy = oy + j * a; fz = oz + k * a; }
            else { long q = id - gridPoints; int i = (int)(q % nx); q /= nx; int j = (int)(q % ny); int k = (int)(q / ny); fx = ox + (i + 0.5) * a; fy = oy + (j + 0.5) * a; fz = oz + (k + 0.5) * a; }
            px_[r] = (float)fx; py_[r] = (float)fy; pz_[r] = (float)fz;
        }

        int[] tets = new int[tetList.Count];
        for (int i = 0; i < tets.Length; i++) { tets[i] = remap[tetList[i]]; }
        tetList = null!;

        // ---- snap boundary vertices onto the surface
        Snap(grid, px_, py_, pz_, tets, a, bound);

        // ---- boundary faces; remove tetrahedra that would pinch the surface or hang off it
        RemovePinches(ref tets, px_.Length);
        // Drop vertices no tetrahedron uses.
        var vmap = new int[px_.Length];
        Array.Fill(vmap, -1);
        for (int i = 0; i < tets.Length; i++) { vmap[tets[i]] = 0; }
        int nvUsed = 0;
        for (int i = 0; i < vmap.Length; i++) { if (vmap[i] == 0) { vmap[i] = nvUsed++; } }
        if (nvUsed == 0) { throw new GeometryToolException(GeometryToolError.Degenerate, "no tetrahedron fits inside the surface at an edge length of " + a.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + "; use a smaller TargetEdgeLength", nameof(options)); }
        var fxv = new double[nvUsed]; var fyv = new double[nvUsed]; var fzv = new double[nvUsed];
        for (int i = 0; i < vmap.Length; i++) { if (vmap[i] >= 0) { fxv[vmap[i]] = px_[i]; fyv[vmap[i]] = py_[i]; fzv[vmap[i]] = pz_[i]; } }
        for (int i = 0; i < tets.Length; i++) { tets[i] = vmap[tets[i]]; }

        var surface = BoundaryFaces(tets, out int[] owner);

        // ---- quality, volume
        double worst = 0, minVr = double.MaxValue, vol = 0;
        for (int t = 0; t < tets.Length / 4; t++)
        {
            Quality(fxv, fyv, fzv, tets, t, out double rr, out double vr, out double v6);
            if (rr > worst) { worst = rr; }
            if (vr < minVr) { minVr = vr; }
            vol += v6 / 6;
        }

        var outVerts = new Vector3F[nvUsed];
        for (int i = 0; i < nvUsed; i++) { outVerts[i] = new Vector3F((float)fxv[i], (float)fyv[i], (float)fzv[i]); }

        // ---- bindings and deviation
        var outGrid = new TriangleGrid(fxv, fyv, fzv, surface, ox, oy, oz, a, nx, ny, nz);
        var bindings = new SurfaceBinding[nv];
        var devIn = new double[nv];
        ParallelRange(nv, threads, (n0, n1) =>
        {
            for (int v = n0; v < n1; v++)
            {
                if (!outGrid.Nearest(mx[v], my[v], mz[v], 6, out int ft, out double qx, out double qy, out double qz, out double d2)) { BruteNearest(outGrid, fxv, fyv, fzv, surface, mx[v], my[v], mz[v], out ft, out d2); }
                devIn[v] = Math.Sqrt(d2);
                int ia = surface[3 * ft], ib = surface[3 * ft + 1], ic = surface[3 * ft + 2];
                TriangleGrid.Barycentric(fxv[ia], fyv[ia], fzv[ia], fxv[ib], fyv[ib], fzv[ib], fxv[ic], fyv[ic], fzv[ic], mx[v], my[v], mz[v], out double u, out double vv, out double w);
                int tet = owner[ft];
                float[] wts = new float[4];
                for (int c = 0; c < 4; c++)
                {
                    int tv = tets[4 * tet + c];
                    wts[c] = tv == ia ? (float)u : tv == ib ? (float)vv : tv == ic ? (float)w : 0f;
                }

                bindings[v] = new SurfaceBinding(tet, wts[0], wts[1], wts[2], wts[3]);
            }
        });

        double deviation = double.NaN;
        if (!options.SkipDeviation)
        {
            double din = 0;
            foreach (double d in devIn) { if (d > din) { din = d; } }
            int nf = surface.Length / 3;
            var devOut = new double[nf];
            ParallelRange(nf, threads, (f0, f1) =>
            {
                for (int f = f0; f < f1; f++)
                {
                    int ia = surface[3 * f], ib = surface[3 * f + 1], ic = surface[3 * f + 2];
                    double worstF = 0;
                    for (int s = 0; s < 7; s++)
                    {
                        double u = SampleU[s], v = SampleV[s], w = 1 - u - v;
                        double qx = u * fxv[ia] + v * fxv[ib] + w * fxv[ic], qy = u * fyv[ia] + v * fyv[ib] + w * fyv[ic], qz = u * fzv[ia] + v * fzv[ib] + w * fzv[ic];
                        if (!grid.Nearest(qx, qy, qz, 6, out _, out _, out _, out _, out double d2)) { d2 = BruteMin(grid, mx, my, mz, tri, qx, qy, qz); }
                        worstF = Math.Max(worstF, Math.Sqrt(d2));
                    }

                    devOut[f] = worstF;
                }
            });
            double dout = 0;
            foreach (double d in devOut) { if (d > dout) { dout = d; } }
            deviation = Math.Max(din, dout);
        }

        var meshVol = MeshMass.ComputeDouble(vertices, indices).Mass;
        return new TetrahedralMesh(outVerts, tets, surface, bindings, vol, meshVol, worst, minVr, deviation, a, worst <= bound * (1 + 1e-12));
    }

    private static double BruteMin(TriangleGrid g, double[] x, double[] y, double[] z, int[] tri, double px, double py, double pz)
    {
        double best = double.PositiveInfinity;
        for (int t = 0; t < tri.Length / 3; t++)
        {
            g.ClosestOnTriangle(t, px, py, pz, out double qx, out double qy, out double qz);
            double d2 = (qx - px) * (qx - px) + (qy - py) * (qy - py) + (qz - pz) * (qz - pz);
            if (d2 < best) { best = d2; }
        }

        return best;
    }

    private static void BruteNearest(TriangleGrid g, double[] x, double[] y, double[] z, int[] tri, double px, double py, double pz, out int face, out double d2)
    {
        face = 0; d2 = double.PositiveInfinity;
        for (int t = 0; t < tri.Length / 3; t++)
        {
            g.ClosestOnTriangle(t, px, py, pz, out double qx, out double qy, out double qz);
            double d = (qx - px) * (qx - px) + (qy - py) * (qy - py) + (qz - pz) * (qz - pz);
            if (d < d2) { d2 = d; face = t; }
        }
    }

    private static GeometryToolException Bad(string name, string value, string expected, string paramName) => new(GeometryToolError.InvalidOptions, name + " = " + value + " must be " + expected, paramName);

    // Runs body(begin, end) over fixed blocks of the range; blocks write disjoint outputs, so the thread count cannot change the result.
    private static void ParallelRange(int n, int threads, Action<int, int> body)
    {
        if (n <= 0) { return; }
        int size = Math.Max(1, Math.Min(64, (n + 7) / 8));
        int blocks = (n + size - 1) / size;
        void Work(int b) { int s = b * size; body(s, Math.Min(n, s + size)); }
        if (threads > 1 && blocks > 1) { Parallel.For(0, blocks, new ParallelOptions { MaxDegreeOfParallelism = threads }, Work); }
        else { for (int b = 0; b < blocks; b++) { Work(b); } }
    }

    // ------------------------------------------------------------ quality

    private static void Quality(double[] x, double[] y, double[] z, int[] t, int tet, out double radiusEdge, out double volumeRatio, out double vol6)
    {
        int a = t[4 * tet], b = t[4 * tet + 1], c = t[4 * tet + 2], d = t[4 * tet + 3];
        QualityOf(x[a], y[a], z[a], x[b], y[b], z[b], x[c], y[c], z[c], x[d], y[d], z[d], out radiusEdge, out volumeRatio, out vol6);
    }

    private static void QualityOf(double ax, double ay, double az, double bx, double by, double bz, double cx, double cy, double cz, double dx, double dy, double dz, out double radiusEdge, out double volumeRatio, out double vol6)
    {
        double ux = bx - ax, uy = by - ay, uz = bz - az, vx = cx - ax, vy = cy - ay, vz = cz - az, wx = dx - ax, wy = dy - ay, wz = dz - az;
        // vol6 = (w . (u x v)), positive when a, b, c seen from d are counter-clockwise
        double cxx = uy * vz - uz * vy, cyy = uz * vx - ux * vz, czz = ux * vy - uy * vx;
        vol6 = wx * cxx + wy * cyy + wz * czz;
        double lu = ux * ux + uy * uy + uz * uz, lv = vx * vx + vy * vy + vz * vz, lw = wx * wx + wy * wy + wz * wz;
        double e1 = (bx - cx) * (bx - cx) + (by - cy) * (by - cy) + (bz - cz) * (bz - cz);
        double e2 = (bx - dx) * (bx - dx) + (by - dy) * (by - dy) + (bz - dz) * (bz - dz);
        double e3 = (cx - dx) * (cx - dx) + (cy - dy) * (cy - dy) + (cz - dz) * (cz - dz);
        double lmin2 = Math.Min(Math.Min(lu, Math.Min(lv, lw)), Math.Min(e1, Math.Min(e2, e3)));
        double lmax2 = Math.Max(Math.Max(lu, Math.Max(lv, lw)), Math.Max(e1, Math.Max(e2, e3)));
        double lmax = Math.Sqrt(lmax2);
        volumeRatio = vol6 / (lmax2 * lmax);
        if (!(vol6 > 0) || !(lmin2 > 0)) { radiusEdge = double.PositiveInfinity; if (!(vol6 > 0)) { volumeRatio = Math.Min(volumeRatio, 0); } return; }
        // circumcentre offset from a: (lu (v x w) + lv (w x u) + lw (u x v)) / (2 det); det = vol6 here up to sign, taken from the same orientation as (u, v, w)
        double vwx = vy * wz - vz * wy, vwy = vz * wx - vx * wz, vwz = vx * wy - vy * wx;
        double wux = wy * uz - wz * uy, wuy = wz * ux - wx * uz, wuz = wx * uy - wy * ux;
        double sx = lu * vwx + lv * wux + lw * cxx, sy = lu * vwy + lv * wuy + lw * cyy, sz = lu * vwz + lv * wuz + lw * czz;
        double r = Math.Sqrt(sx * sx + sy * sy + sz * sz) / (2 * vol6);
        radiusEdge = r / Math.Sqrt(lmin2);
    }

    // ------------------------------------------------------------ snapping

    private static void Snap(TriangleGrid grid, double[] x, double[] y, double[] z, int[] tets, double a, double bound)
    {
        int nv = x.Length, nt = tets.Length / 4;
        // vertex -> tetrahedra (compressed rows)
        var start = new int[nv + 1];
        for (int i = 0; i < tets.Length; i++) { start[tets[i] + 1]++; }
        for (int i = 0; i < nv; i++) { start[i + 1] += start[i]; }
        var fill = new int[nv];
        var adj = new int[tets.Length];
        for (int t = 0; t < nt; t++) { for (int c = 0; c < 4; c++) { int v = tets[4 * t + c]; adj[start[v] + fill[v]++] = t; } }

        bool Acceptable(int v, double nx, double ny, double nz)
        {
            double ox = x[v], oy = y[v], oz = z[v];
            for (int e = start[v]; e < start[v + 1]; e++)
            {
                int t = adj[e];
                Quality(x, y, z, tets, t, out double r0, out double vr0, out _);
                x[v] = nx; y[v] = ny; z[v] = nz;
                Quality(x, y, z, tets, t, out double r1, out double vr1, out double v61);
                x[v] = ox; y[v] = oy; z[v] = oz;
                if (!(v61 > 0)) { return false; }
                if (vr1 < MinVolumeRatioGate && vr1 < vr0) { return false; }
                if (r1 > bound && r1 > r0) { return false; }
            }

            return true;
        }

        // A few sweeps in vertex order: a move refused at first can become acceptable once its neighbours have moved. Each vertex tries the full move, then shorter ones.
        var onBoundary = new bool[nv];
        var faces0 = BoundaryFaces(tets, out _);
        foreach (int f in faces0) { onBoundary[f] = true; }
        for (int sweep = 0; sweep < SnapSweeps; sweep++)
        {
            for (int v = 0; v < nv; v++)
            {
                if (!onBoundary[v]) { continue; }
                if (!grid.Nearest(x[v], y[v], z[v], 3, out _, out double qx, out double qy, out double qz, out double d2)) { continue; }
                double d = Math.Sqrt(d2);
                if (d == 0) { continue; }
                if (d > 2 * a) { continue; }
                for (int step = 0; step < SnapLadder.Length; step++)
                {
                    double f = SnapLadder[step];
                    double tx = (float)(x[v] + f * (qx - x[v])), ty = (float)(y[v] + f * (qy - y[v])), tz = (float)(z[v] + f * (qz - z[v]));
                    if (Acceptable(v, tx, ty, tz)) { x[v] = tx; y[v] = ty; z[v] = tz; break; }
                }
            }
        }
    }

    // ------------------------------------------------------------ boundary

    // Faces used by exactly one tetrahedron, oriented away from the tetrahedron's fourth vertex, ordered by (tetrahedron, face index). owner gives each face's tetrahedron.
    private static int[] BoundaryFaces(int[] t, out int[] owner)
    {
        int nt = t.Length / 4;
        var keys = new long[4 * nt];
        var slot = new int[4 * nt];
        for (int e = 0; e < nt; e++)
        {
            for (int f = 0; f < 4; f++)
            {
                int p = t[4 * e + (f == 0 ? 1 : 0)], q = t[4 * e + (f <= 1 ? 2 : 1)], r = t[4 * e + (f <= 2 ? 3 : 2)];
                if (p > q) { (p, q) = (q, p); }
                if (q > r) { (q, r) = (r, q); }
                if (p > q) { (p, q) = (q, p); }
                keys[4 * e + f] = ((long)p << 42) | ((long)q << 21) | (long)r;
                slot[4 * e + f] = 4 * e + f;
            }
        }

        Array.Sort(keys, slot);
        var single = new List<int>();
        for (int i = 0; i < keys.Length;)
        {
            int j = i;
            while (j < keys.Length && keys[j] == keys[i]) { j++; }
            if (j - i == 1) { single.Add(slot[i]); }
            i = j;
        }

        single.Sort();
        var faces = new int[3 * single.Count];
        owner = new int[single.Count];
        for (int n = 0; n < single.Count; n++)
        {
            int e = single[n] / 4, f = single[n] % 4;
            // Face f lies opposite vertex f. With positive volume, vertices 0, 1, 2 run counter-clockwise seen from vertex 3, so the outward faces are (1,2,3), (0,3,2), (0,1,3), (0,2,1).
            int[] v = { t[4 * e], t[4 * e + 1], t[4 * e + 2], t[4 * e + 3] };
            int p, q, r;
            switch (f)
            {
                case 0: p = v[1]; q = v[2]; r = v[3]; break;
                case 1: p = v[0]; q = v[3]; r = v[2]; break;
                case 2: p = v[0]; q = v[1]; r = v[3]; break;
                default: p = v[0]; q = v[2]; r = v[1]; break;
            }

            faces[3 * n] = p; faces[3 * n + 1] = q; faces[3 * n + 2] = r;
            owner[n] = e;
        }

        return faces;
    }

    // Removes tetrahedra until no boundary edge is shared by more than two boundary faces (a pinch: two lumps meeting at an edge), lowest tetrahedron index first.
    private static void RemovePinches(ref int[] tets, int vertexCount)
    {
        for (int round = 0; round < 64; round++)
        {
            var faces = BoundaryFaces(tets, out int[] owner);
            var keys = new long[3 * (faces.Length / 3)];
            var own = new int[keys.Length];
            for (int f = 0; f < faces.Length / 3; f++)
            {
                for (int k = 0; k < 3; k++)
                {
                    long p = faces[3 * f + k], q = faces[3 * f + (k + 1) % 3];
                    if (p > q) { (p, q) = (q, p); }
                    keys[3 * f + k] = (p << 21) | q;
                    own[3 * f + k] = owner[f];
                }
            }

            Array.Sort(keys, own);
            var drop = new SortedSet<int>();
            for (int i = 0; i < keys.Length;)
            {
                int j = i;
                while (j < keys.Length && keys[j] == keys[i]) { j++; }
                if (j - i > 2)
                {
                    // Of the tetrahedra on a pinched edge, drop the one with the highest index (the later-added lump); one removal per pinch per round.
                    int worst = own[i];
                    for (int m = i + 1; m < j; m++) { worst = Math.Max(worst, own[m]); }
                    drop.Add(worst);
                }

                i = j;
            }

            if (drop.Count == 0) { return; }
            var kept = new List<int>(tets.Length);
            for (int t = 0; t < tets.Length / 4; t++)
            {
                if (drop.Contains(t)) { continue; }
                for (int c = 0; c < 4; c++) { kept.Add(tets[4 * t + c]); }
            }

            tets = kept.ToArray();
            if (tets.Length == 0) { return; }
        }
    }
}
