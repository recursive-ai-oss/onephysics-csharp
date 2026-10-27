using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Collision3D;

/// <summary>
/// An immutable height field for static terrain: a regular grid of height samples in the shape's own frame, Y up. Sample <c>(i, j)</c> sits at
/// <c>(i * scale.X, height * scale.Y, j * scale.Z)</c> (the origin is the corner sample; place the shape with its body transform). Each grid cell is two triangles split along the diagonal from
/// its (i, j) corner to its (i + 1, j + 1) corner, wound so the front side faces up (+Y). Cells can be holes, which have no triangles.
/// <para>
/// Storage: float32 heights (4 bytes per sample, exact: the surface passes through every sample as given), one bit per cell for holes, one byte per cell for the active-edge flags of its two triangles,
/// and a pyramid of per-block height ranges (blocks of 4 x 4 cells, then 8 x 8, and so on) for traversal, about half a byte per cell. Triangle ids are <c>2 * cell + k</c> with cell index
/// <c>j * (samplesX - 1) + i</c>; k = 0 is the triangle (i,j), (i,j+1), (i+1,j+1) and k = 1 is (i,j), (i+1,j+1), (i+1,j).
/// Height fields are static-only: <see cref="TriangleSoup3D.GetMassProperties"/> throws.
/// </para>
/// </summary>
public sealed class HeightField3D : TriangleSoup3D
{
    /// <summary>Largest number of samples along one side, 2^14.</summary>
    public const int MaxSamplesPerSide = 1 << 14;

    /// <summary>Largest total number of samples, 2^26.</summary>
    public const int MaxSamples = 1 << 26;

    private const int BlockShift = 2;
    private const double DefaultFlatTolerance = 1e-3;

    private readonly int _nx, _nz, _cx, _cz;
    private readonly float[] _heights;
    private readonly Vector3F _scale;
    private readonly ulong[]? _holes;
    private readonly byte[] _flags;
    private readonly int _triangleCount;
    private readonly int[] _levelW, _levelH;
    private readonly float[][] _levelMin, _levelMax;
    private readonly Bounds3F _bounds;

    private HeightField3D(int nx, int nz, float[] heights, Vector3F scale, ulong[]? holes, int triangleCount, double tolerance)
    {
        _nx = nx;
        _nz = nz;
        _cx = nx - 1;
        _cz = nz - 1;
        _heights = heights;
        _scale = scale;
        _holes = holes;
        _triangleCount = triangleCount;

        // Pyramid of height ranges.
        var w = new List<int>();
        var h = new List<int>();
        var mins = new List<float[]>();
        var maxs = new List<float[]>();
        int bw = (_cx + (1 << BlockShift) - 1) >> BlockShift, bh = (_cz + (1 << BlockShift) - 1) >> BlockShift;
        var mn = new float[bw * bh];
        var mx = new float[bw * bh];
        for (int bz = 0; bz < bh; bz++)
        {
            for (int bx = 0; bx < bw; bx++)
            {
                float lo = float.PositiveInfinity, hi = float.NegativeInfinity;
                int i1 = Math.Min((bx + 1) << BlockShift, _cx), j1 = Math.Min((bz + 1) << BlockShift, _cz);
                for (int j = bz << BlockShift; j <= j1; j++)
                {
                    for (int i = bx << BlockShift; i <= i1; i++)
                    {
                        float y = _heights[(j * _nx) + i] * _scale.Y;
                        lo = Math.Min(lo, y);
                        hi = Math.Max(hi, y);
                    }
                }

                mn[(bz * bw) + bx] = lo;
                mx[(bz * bw) + bx] = hi;
            }
        }

        w.Add(bw);
        h.Add(bh);
        mins.Add(mn);
        maxs.Add(mx);
        while (bw > 1 || bh > 1)
        {
            int nw = (bw + 1) >> 1, nh = (bh + 1) >> 1;
            var pmn = new float[nw * nh];
            var pmx = new float[nw * nh];
            for (int z = 0; z < nh; z++)
            {
                for (int x = 0; x < nw; x++)
                {
                    float lo = float.PositiveInfinity, hi = float.NegativeInfinity;
                    for (int dz = 0; dz < 2; dz++)
                    {
                        for (int dx = 0; dx < 2; dx++)
                        {
                            int cx = (2 * x) + dx, cz = (2 * z) + dz;
                            if (cx < bw && cz < bh)
                            {
                                lo = Math.Min(lo, mn[(cz * bw) + cx]);
                                hi = Math.Max(hi, mx[(cz * bw) + cx]);
                            }
                        }
                    }

                    pmn[(z * nw) + x] = lo;
                    pmx[(z * nw) + x] = hi;
                }
            }

            w.Add(nw);
            h.Add(nh);
            mins.Add(pmn);
            maxs.Add(pmx);
            mn = pmn;
            mx = pmx;
            bw = nw;
            bh = nh;
        }

        _levelW = w.ToArray();
        _levelH = h.ToArray();
        _levelMin = mins.ToArray();
        _levelMax = maxs.ToArray();
        _bounds = new Bounds3F(new Vector3F(0f, _levelMin[^1][0], 0f), new Vector3F(_cx * _scale.X, _levelMax[^1][0], _cz * _scale.Z));

        _flags = new byte[_cx * _cz];
        for (int cell = 0; cell < _flags.Length; cell++)
        {
            if (IsHole(cell))
            {
                continue;
            }

            int f = 0;
            for (int k = 0; k < 2; k++)
            {
                Tri(cell, k, out Vector3F a, out Vector3F b, out Vector3F c);
                for (int e = 0; e < 3; e++)
                {
                    if (NeighbourActive(cell, k, e, a, b, c, tolerance))
                    {
                        f |= 1 << ((3 * k) + e);
                    }
                }
            }

            _flags[cell] = (byte)f;
        }
    }

    /// <inheritdoc/>
    public override int TriangleIdCount => 2 * _flags.Length;

    /// <inheritdoc/>
    public override int TriangleCount => _triangleCount;

    /// <inheritdoc/>
    public override Bounds3F Bounds => _bounds;

    /// <summary>The number of samples along X.</summary>
    public int SamplesX => _nx;

    /// <summary>The number of samples along Z.</summary>
    public int SamplesZ => _nz;

    /// <summary>The scale given at construction.</summary>
    public Vector3F Scale => _scale;

    /// <summary>The raw heights, row by row (index <c>j * SamplesX + i</c>), before the Y scale.</summary>
    public ReadOnlySpan<float> Heights => _heights;

    /// <summary>True when the cell <c>(cellX, cellZ)</c> is a hole.</summary>
    /// <param name="cellX">Cell column in [0, SamplesX - 1).</param>
    /// <param name="cellZ">Cell row in [0, SamplesZ - 1).</param>
    /// <exception cref="ArgumentOutOfRangeException">The cell is outside the grid.</exception>
    public bool IsHoleAt(int cellX, int cellZ)
    {
        if ((uint)cellX >= (uint)_cx || (uint)cellZ >= (uint)_cz)
        {
            throw new ArgumentOutOfRangeException(nameof(cellX), cellX, $"The cell ({cellX}, {cellZ}) is outside the {_cx} x {_cz} cells of the grid.");
        }

        return IsHole((cellZ * _cx) + cellX);
    }

    private bool IsHole(int cell) => _holes is not null && ((_holes[cell >> 6] >> (cell & 63)) & 1UL) != 0;

    private Vector3F P(int i, int j) => new((float)i * _scale.X, _heights[(j * _nx) + i] * _scale.Y, (float)j * _scale.Z);

    private void Tri(int cell, int k, out Vector3F a, out Vector3F b, out Vector3F c)
    {
        int i = cell % _cx, j = cell / _cx;
        Vector3F v00 = P(i, j), v10 = P(i + 1, j), v01 = P(i, j + 1), v11 = P(i + 1, j + 1);
        if (k == 0)
        {
            a = v00;
            b = v01;
            c = v11;
        }
        else
        {
            a = v00;
            b = v11;
            c = v10;
        }
    }

    // The neighbour across each edge, by the cell layout in the type's remarks: A edges 0, 1, 2 meet B of the cells (i-1, j), (i, j+1) and the same cell (edges 1, 2 and 0 of B);
    // B edges 0, 1, 2 meet A of the same cell, (i+1, j) and (i, j-1) (edges 2, 0 and 1 of A). A missing or hole neighbour makes the edge a boundary, which is active.
    private bool NeighbourActive(int cell, int k, int e, Vector3F a, Vector3F b, Vector3F c, double tol)
    {
        int i = cell % _cx, j = cell / _cx;
        int ni, nj, nk;
        if (k == 0)
        {
            (ni, nj, nk) = e == 0 ? (i - 1, j, 1) : e == 1 ? (i, j + 1, 1) : (i, j, 1);
        }
        else
        {
            (ni, nj, nk) = e == 0 ? (i, j, 0) : e == 1 ? (i + 1, j, 0) : (i, j - 1, 0);
        }

        if ((uint)ni >= (uint)_cx || (uint)nj >= (uint)_cz)
        {
            return true;
        }

        int ncell = (nj * _cx) + ni;
        if (IsHole(ncell))
        {
            return true;
        }

        Tri(ncell, nk, out Vector3F b0, out Vector3F b1, out Vector3F b2);
        return TriangleTests3D.IsEdgeActive(a, b, c, e, b0, b1, b2, tol);
    }

    /// <summary>Builds a height field.</summary>
    /// <param name="heights">Heights row by row: sample <c>(i, j)</c> is <c>heights[j * samplesX + i]</c>. All finite; each height times the Y scale at most 2^21 in magnitude.</param>
    /// <param name="samplesX">Samples along X, in [2, <see cref="MaxSamplesPerSide"/>].</param>
    /// <param name="samplesZ">Samples along Z, in [2, <see cref="MaxSamplesPerSide"/>]; the total is at most <see cref="MaxSamples"/>.</param>
    /// <param name="scale">Spacing along X and Z and the multiplier of heights; each finite and positive, with the grid's extent at most 2^21.</param>
    /// <param name="holes">Optional hole mask, one entry per cell (<c>(samplesX - 1) * (samplesZ - 1)</c> entries, row by row); true marks a hole. Empty means no holes.</param>
    /// <param name="options">Options; the default chooses everything.</param>
    /// <returns>The height field. The same input gives the same field, bit for bit, on every platform.</returns>
    /// <exception cref="MeshBuildException">The grid size, the scale or the mask is invalid, a height is non-finite or out of range, every cell is a hole, or the options are invalid. The message names the input and what was expected.</exception>
    public static HeightField3D Create(ReadOnlySpan<float> heights, int samplesX, int samplesZ, Vector3F scale, ReadOnlySpan<bool> holes = default, MeshBuildOptions options = default)
    {
        double tol = options.FlatEdgeTolerance;
        if (tol == 0.0)
        {
            tol = DefaultFlatTolerance;
        }
        else if (!(tol > 0.0 && tol <= 1.0))
        {
            throw new MeshBuildException(MeshBuildError.InvalidOptions, $"options.FlatEdgeTolerance is {tol.ToString("R", CultureInfo.InvariantCulture)}; 0 (automatic) or a value in (0, 1] is expected.", nameof(options));
        }

        if (samplesX < 2 || samplesZ < 2 || samplesX > MaxSamplesPerSide || samplesZ > MaxSamplesPerSide || (long)samplesX * samplesZ > MaxSamples)
        {
            throw new MeshBuildException(MeshBuildError.InvalidGrid, $"The grid is {samplesX} x {samplesZ} samples; each side in [2, {MaxSamplesPerSide}] and at most {MaxSamples} samples in total are expected.", nameof(samplesX));
        }

        if (heights.Length != samplesX * samplesZ)
        {
            throw new MeshBuildException(MeshBuildError.InvalidGrid, $"heights has {heights.Length} entries; {samplesX} x {samplesZ} = {samplesX * samplesZ} are expected.", nameof(heights));
        }

        for (int a = 0; a < 3; a++)
        {
            float s = scale[a];
            string axis = "XYZ"[a].ToString();
            if (!float.IsFinite(s))
            {
                throw new MeshBuildException(MeshBuildError.NonFinite, $"scale.{axis} is {s.ToString("R", CultureInfo.InvariantCulture)}; a finite positive value is expected.", nameof(scale));
            }

            if (!(s > 0f) || s > TriangleMesh3D.MaxCoordinate)
            {
                throw new MeshBuildException(MeshBuildError.OutOfRange, $"scale.{axis} is {s.ToString("R", CultureInfo.InvariantCulture)}; a value in (0, {TriangleMesh3D.MaxCoordinate}] is expected.", nameof(scale));
            }
        }

        if ((double)(samplesX - 1) * scale.X > TriangleMesh3D.MaxCoordinate || (double)(samplesZ - 1) * scale.Z > TriangleMesh3D.MaxCoordinate)
        {
            throw new MeshBuildException(MeshBuildError.OutOfRange, $"The grid spans {(samplesX - 1) * (double)scale.X} x {(samplesZ - 1) * (double)scale.Z}; an extent of at most {TriangleMesh3D.MaxCoordinate} along each axis is expected.", nameof(scale));
        }

        for (int n = 0; n < heights.Length; n++)
        {
            float hgt = heights[n];
            if (!float.IsFinite(hgt))
            {
                throw new MeshBuildException(MeshBuildError.NonFinite, $"heights[{n}] (sample {n % samplesX}, {n / samplesX}) is {hgt.ToString("R", CultureInfo.InvariantCulture)}; a finite height is expected.", nameof(heights));
            }

            if (Math.Abs((double)hgt * scale.Y) > TriangleMesh3D.MaxCoordinate)
            {
                throw new MeshBuildException(MeshBuildError.OutOfRange, $"heights[{n}] (sample {n % samplesX}, {n / samplesX}) is {hgt.ToString("R", CultureInfo.InvariantCulture)}; with scale.Y = {scale.Y.ToString("R", CultureInfo.InvariantCulture)} a height of magnitude at most {TriangleMesh3D.MaxCoordinate / scale.Y} is expected.", nameof(heights));
            }
        }

        int cells = (samplesX - 1) * (samplesZ - 1);
        ulong[]? mask = null;
        int holeCount = 0;
        if (holes.Length != 0)
        {
            if (holes.Length != cells)
            {
                throw new MeshBuildException(MeshBuildError.InvalidGrid, $"holes has {holes.Length} entries; {samplesX - 1} x {samplesZ - 1} = {cells} (one per cell) or none are expected.", nameof(holes));
            }

            mask = new ulong[(cells + 63) >> 6];
            for (int n = 0; n < cells; n++)
            {
                if (holes[n])
                {
                    mask[n >> 6] |= 1UL << (n & 63);
                    holeCount++;
                }
            }

            if (holeCount == cells)
            {
                throw new MeshBuildException(MeshBuildError.NoTriangles, "Every cell is a hole; at least one solid cell is expected.", nameof(holes));
            }
        }

        return new HeightField3D(samplesX, samplesZ, heights.ToArray(), scale, mask, 2 * (cells - holeCount), tol);
    }

    /// <inheritdoc/>
    public override bool TryGetTriangle(int id, out SoupTriangle3D triangle)
    {
        triangle = default;
        if ((uint)id >= (uint)(2 * _flags.Length))
        {
            return false;
        }

        int cell = id >> 1, k = id & 1;
        if (IsHole(cell))
        {
            return false;
        }

        Tri(cell, k, out Vector3F a, out Vector3F b, out Vector3F c);
        triangle = new SoupTriangle3D(id, a, b, c, (byte)((_flags[cell] >> (3 * k)) & 7));
        return true;
    }

    // Box of the block at (level, bx, bz) in the shape's frame; the x and z bounds are the same products the vertices use.
    private void NodeBox(int level, int bx, int bz, out float x0, out float y0, out float z0, out float x1, out float y1, out float z1)
    {
        int size = 1 << (BlockShift + level);
        int i0 = bx * size, i1 = Math.Min((bx + 1) * size, _cx), j0 = bz * size, j1 = Math.Min((bz + 1) * size, _cz);
        x0 = (float)i0 * _scale.X;
        x1 = (float)i1 * _scale.X;
        z0 = (float)j0 * _scale.Z;
        z1 = (float)j1 * _scale.Z;
        y0 = _levelMin[level][(bz * _levelW[level]) + bx];
        y1 = _levelMax[level][(bz * _levelW[level]) + bx];
    }

    // Level in the top four bits (at most 12 levels), then two 14-bit block coordinates; read back with an unsigned shift.
    private static int Pack(int level, int bx, int bz) => (level << 28) | (bx << 14) | bz;

    internal override void RayWalk(in RayD ray, ref RayCollector c)
    {
        int top = _levelW.Length - 1;
        Span<int> stack = stackalloc int[96];
        Span<double> enters = stackalloc double[96];
        Span<int> kid = stackalloc int[4];
        Span<double> kt = stackalloc double[4];
        NodeBox(top, 0, 0, out float x0, out float y0, out float z0, out float x1, out float y1, out float z1);
        if (!ray.HitBox(x0, y0, z0, x1, y1, z1, c.Limit, out double e0))
        {
            return;
        }

        int sp = 0;
        stack[sp] = Pack(top, 0, 0);
        enters[sp++] = e0;
        Vector3D o = ray.Origin, d = ray.Direction;
        while (sp > 0)
        {
            sp--;
            int node = stack[sp];
            if (enters[sp] > c.Limit)
            {
                continue;
            }

            int level = (int)((uint)node >> 28), bx = (node >> 14) & 0x3FFF, bz = node & 0x3FFF;
            if (level == 0)
            {
                int i0 = bx << BlockShift, j0 = bz << BlockShift;
                int i1 = Math.Min(i0 + (1 << BlockShift), _cx), j1 = Math.Min(j0 + (1 << BlockShift), _cz);
                for (int j = j0; j < j1; j++)
                {
                    for (int i = i0; i < i1; i++)
                    {
                        int cell = (j * _cx) + i;
                        if (IsHole(cell))
                        {
                            continue;
                        }

                        for (int k = 0; k < 2; k++)
                        {
                            Tri(cell, k, out Vector3F a, out Vector3F b, out Vector3F cc);
                            if (TriangleTests3D.RayTri(o, d, a.ToDouble(), b.ToDouble(), cc.ToDouble(), c.CullBack, out double t, out bool back))
                            {
                                c.Offer((2 * cell) + k, t, back);
                                if (c.Done)
                                {
                                    return;
                                }
                            }
                        }
                    }
                }

                continue;
            }

            int n = 0;
            for (int dz = 0; dz < 2; dz++)
            {
                for (int dx = 0; dx < 2; dx++)
                {
                    int cbx = (2 * bx) + dx, cbz = (2 * bz) + dz;
                    if (cbx >= _levelW[level - 1] || cbz >= _levelH[level - 1])
                    {
                        continue;
                    }

                    NodeBox(level - 1, cbx, cbz, out x0, out y0, out z0, out x1, out y1, out z1);
                    if (ray.HitBox(x0, y0, z0, x1, y1, z1, c.Limit, out double te))
                    {
                        // Insertion by entry fraction, nearest first; equal fractions keep the (dz, dx) order.
                        int p = n++;
                        while (p > 0 && kt[p - 1] > te)
                        {
                            kt[p] = kt[p - 1];
                            kid[p] = kid[p - 1];
                            p--;
                        }

                        kt[p] = te;
                        kid[p] = Pack(level - 1, cbx, cbz);
                    }
                }
            }

            for (int p = n - 1; p >= 0; p--)
            {
                stack[sp] = kid[p];
                enters[sp++] = kt[p];
            }
        }
    }

    internal override void BoxWalk(in Bounds3F box, List<int> ids)
    {
        int top = _levelW.Length - 1;
        Span<int> stack = stackalloc int[96];
        int sp = 0;
        stack[sp++] = Pack(top, 0, 0);
        while (sp > 0)
        {
            int node = stack[--sp];
            int level = (int)((uint)node >> 28), bx = (node >> 14) & 0x3FFF, bz = node & 0x3FFF;
            NodeBox(level, bx, bz, out float x0, out float y0, out float z0, out float x1, out float y1, out float z1);
            if (!(x0 <= box.Max.X && box.Min.X <= x1 && y0 <= box.Max.Y && box.Min.Y <= y1 && z0 <= box.Max.Z && box.Min.Z <= z1))
            {
                continue;
            }

            if (level == 0)
            {
                int i0 = bx << BlockShift, j0 = bz << BlockShift;
                int i1 = Math.Min(i0 + (1 << BlockShift), _cx), j1 = Math.Min(j0 + (1 << BlockShift), _cz);
                for (int j = j0; j < j1; j++)
                {
                    for (int i = i0; i < i1; i++)
                    {
                        int cell = (j * _cx) + i;
                        if (IsHole(cell))
                        {
                            continue;
                        }

                        for (int k = 0; k < 2; k++)
                        {
                            Tri(cell, k, out Vector3F a, out Vector3F b, out Vector3F c);
                            if (TriangleTests3D.BoxTriangle(box, a, b, c))
                            {
                                ids.Add((2 * cell) + k);
                            }
                        }
                    }
                }

                continue;
            }

            for (int dz = 0; dz < 2; dz++)
            {
                for (int dx = 0; dx < 2; dx++)
                {
                    int cbx = (2 * bx) + dx, cbz = (2 * bz) + dz;
                    if (cbx < _levelW[level - 1] && cbz < _levelH[level - 1])
                    {
                        stack[sp++] = Pack(level - 1, cbx, cbz);
                    }
                }
            }
        }
    }
}
