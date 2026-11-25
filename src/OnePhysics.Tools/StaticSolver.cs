using System;
using System.Globalization;
using System.Threading.Tasks;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>
/// Linear-elastic static solve on a tetrahedral mesh (the output of <see cref="Tetrahedralizer"/>, or any conforming mesh): displacement per vertex, strain, stress, von Mises stress and safety factor per element.
/// Finite elements from the textbooks (Zienkiewicz and Taylor; Hughes; Cook et al.): isotropic Hooke law, ten-node tetrahedra with a four-point rule (or four-node constant-strain ones), consistent loads, supports eliminated
/// from the system; the system is solved by preconditioned conjugate gradients (Saad, Iterative Methods for Sparse Linear Systems, ch. 9) in double precision, the preconditioner a symmetric block Gauss-Seidel sweep
/// within fixed blocks of 256 nodes. Every sum runs in a fixed order over blocks that do not depend on the thread count, so the result is the same bits on every platform and at every <see cref="StaticSolveOptions.MaxThreads"/>.
/// Units are SI throughout; <see cref="StaticSolveOptions.MetresPerUnit"/> says what a mesh unit is.
/// </summary>
public static class StaticSolver
{
    private const int NodeChunk = 256;
    private const int MaxVertexCount = 1 << 20;
    private const int MaxTetCount = 1 << 20;
    private const double MaxCoordinateMetres = 1e9;
    private const double MaxLoad = 1e30;

    private static readonly int[] EdgeA = { 0, 0, 0, 1, 1, 2 };
    private static readonly int[] EdgeB = { 1, 2, 3, 2, 3, 3 };

    // Four-point rule of degree 2 on the tetrahedron (Zienkiewicz and Taylor, table of tetrahedron rules): barycentric (a, a, a, b) and permutations, equal weights 1/4 of the volume.
    private static readonly double GpA = (5.0 - Math.Sqrt(5.0)) / 20.0;
    private static readonly double GpB = (5.0 + 3.0 * Math.Sqrt(5.0)) / 20.0;

    /// <summary>Solves a static problem on a tetrahedral mesh from <see cref="Tetrahedralizer"/>.</summary>
    /// <param name="mesh">The mesh; its vertices are in the mesh's own units (see <see cref="StaticSolveOptions.MetresPerUnit"/>).</param>
    /// <param name="problem">Material, supports and loads.</param>
    /// <param name="options">Element order, units, tolerance and threads.</param>
    /// <returns>The solution.</returns>
    /// <exception cref="StaticSolveException">An input is invalid; the message names it and what was expected.</exception>
    public static StaticSolution Solve(TetrahedralMesh mesh, StaticProblem problem, StaticSolveOptions options = default)
    {
        if (mesh is null) { throw new StaticSolveException(StaticSolveError.Empty, "mesh is null", nameof(mesh)); }
        return Solve(mesh.Vertices, mesh.Tetrahedra, problem, options);
    }

    /// <summary>Solves a static problem on a tetrahedral mesh given as arrays.</summary>
    /// <param name="vertices">Vertex positions in mesh units; finite, each coordinate within 1e9 m after scaling.</param>
    /// <param name="tetrahedra">Four vertex indices per tetrahedron, each with positive volume (vertices 0, 1, 2 counter-clockwise seen from vertex 3); at most 2^20 tetrahedra and 2^20 vertices; every vertex used.</param>
    /// <param name="problem">Material, supports and loads.</param>
    /// <param name="options">Element order, units, tolerance and threads.</param>
    /// <returns>The solution. When the iteration limit is reached first, <see cref="StaticSolution.Converged"/> is false and the displacement is the last iterate.</returns>
    /// <exception cref="StaticSolveException">An input is invalid (degenerate or inverted tetrahedra, no supports or a mechanism, NaN, non-positive modulus, Poisson ratio of 0.5 or more, bad indices, bad faces, bad options); the message names it.</exception>
    public static StaticSolution Solve(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> tetrahedra, StaticProblem problem, StaticSolveOptions options = default)
    {
        if (problem is null) { throw new StaticSolveException(StaticSolveError.Empty, "problem is null", nameof(problem)); }
        var fem = new Fem(options);
        fem.Prepare(vertices, tetrahedra, problem);
        fem.Assemble();
        fem.ApplySupportsAndLoads(problem);
        fem.Solve();
        return fem.Results(problem);
    }

    private static string Inv(int v) => v.ToString(CultureInfo.InvariantCulture);

    private static string Inv(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    private static StaticSolveException Err(StaticSolveError e, string message, string param) => new(e, message, param);

    private static void For(int count, int threads, Action<int> body)
    {
        if (threads <= 1 || count < 2)
        {
            for (int i = 0; i < count; i++) { body(i); }
            return;
        }

        Parallel.For(0, count, new ParallelOptions { MaxDegreeOfParallelism = threads }, body);
    }

    // Inverse of a symmetric positive definite 3x3 block by cofactors; the same operations in the same order everywhere.
    private static void Invert3(ReadOnlySpan<double> m, Span<double> o)
    {
        double a = m[0], b = m[1], c = m[2], d = m[3], e = m[4], f = m[5], g = m[6], h = m[7], k = m[8];
        double c00 = e * k - f * h, c01 = f * g - d * k, c02 = d * h - e * g;
        double det = a * c00 + b * c01 + c * c02;
        double s = 1.0 / det;
        o[0] = c00 * s; o[1] = (c * h - b * k) * s; o[2] = (b * f - c * e) * s;
        o[3] = c01 * s; o[4] = (a * k - c * g) * s; o[5] = (c * d - a * f) * s;
        o[6] = c02 * s; o[7] = (b * g - a * h) * s; o[8] = (a * e - b * d) * s;
    }

    private sealed class Fem
    {
        private readonly int _threads;
        private readonly bool _quad;
        private readonly int _nn;
        private readonly int _ngp;
        private double _scale = 1, _tol = 1e-8, _lambda, _mu;
        private int _maxIter = 20000;

        private int _v, _t, _n, _dofs, _chunks;
        private double[] _x = Array.Empty<double>();      // vertex positions, m
        private double[] _nodePos = Array.Empty<double>(); // node positions (vertices and edge midpoints), m
        private int[] _tets = Array.Empty<int>();
        private double[] _geo = Array.Empty<double>();    // per tetrahedron: 4 gradients of the barycentric coordinates (12), volume
        private int[] _nodes = Array.Empty<int>();        // per tetrahedron: node ids, corners then the six edge midpoints
        private int[] _nodeOfVertex = Array.Empty<int>();
        private long[] _edgeKeys = Array.Empty<long>();   // sorted min * V + max
        private int[] _nodeOfEdge = Array.Empty<int>();
        private int[] _incPtr = Array.Empty<int>(), _incTet = Array.Empty<int>();
        private int[] _rowPtr = Array.Empty<int>(), _cols = Array.Empty<int>();
        private double[] _vals = Array.Empty<double>();
        private double[] _invD = Array.Empty<double>();
        private byte[] _fixed = Array.Empty<byte>();
        private double[] _presc = Array.Empty<double>();
        private double[] _f = Array.Empty<double>();
        private double[] _u = Array.Empty<double>();
        private double _volume;
        private int _aggSize, _aggCount;
        private double[] _coarseP = Array.Empty<double>();   // per node: the 3 x 6 rigid-motion basis of its aggregate, held components zeroed
        private double[] _coarseL = Array.Empty<double>();   // Cholesky factor of the coarse matrix, row-major m x m
        private double _residual;
        private int _iterations;
        private bool _converged;
        private double[] _unitWeak = new double[3];
        private double _weakRatio = 1;
        private bool _hasWeak;
        private double _strength;
        private double _density;

        public Fem(StaticSolveOptions o)
        {
            _quad = o.ElementOrder == StaticElementOrder.Quadratic;
            _nn = _quad ? 10 : 4;
            _ngp = _quad ? 4 : 1;
            if (o.ElementOrder != StaticElementOrder.Quadratic && o.ElementOrder != StaticElementOrder.Linear)
            {
                throw Err(StaticSolveError.InvalidOptions, "options.ElementOrder = " + Inv((int)o.ElementOrder) + " is not Quadratic or Linear", nameof(o));
            }

            _threads = o.MaxThreads;
            if (_threads < 0 || _threads > 256) { throw Err(StaticSolveError.InvalidOptions, "options.MaxThreads = " + Inv(_threads) + " must be from 0 to 256", "options"); }
            _threads = Math.Max(1, _threads);
            if (o.MetresPerUnit != 0)
            {
                if (!double.IsFinite(o.MetresPerUnit) || o.MetresPerUnit <= 0 || o.MetresPerUnit < 1e-9 || o.MetresPerUnit > 1e9) { throw Err(StaticSolveError.InvalidOptions, "options.MetresPerUnit = " + Inv(o.MetresPerUnit) + " must be from 1e-9 to 1e9 (or 0 for 1)", "options"); }
                _scale = o.MetresPerUnit;
            }
            else if (!o.LengthUnit.IsMetres) { _scale = o.LengthUnit.MetresPerUnit; }

            if (o.Tolerance != 0)
            {
                if (!(o.Tolerance >= 1e-15 && o.Tolerance <= 1e-2)) { throw Err(StaticSolveError.InvalidOptions, "options.Tolerance = " + Inv(o.Tolerance) + " must be from 1e-15 to 1e-2 (or 0 for 1e-8)", "options"); }
                _tol = o.Tolerance;
            }

            if (o.MaxIterations != 0)
            {
                if (o.MaxIterations < 1 || o.MaxIterations > 10_000_000) { throw Err(StaticSolveError.InvalidOptions, "options.MaxIterations = " + Inv(o.MaxIterations) + " must be from 1 to 10000000 (or 0 for 20000)", "options"); }
                _maxIter = o.MaxIterations;
            }
        }

        public void Prepare(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> tetrahedra, StaticProblem problem)
        {
            ValidateMaterial(problem);
            if (vertices.Length == 0) { throw Err(StaticSolveError.Empty, "vertices is empty; a mesh needs at least four vertices", "vertices"); }
            if (tetrahedra.Length == 0) { throw Err(StaticSolveError.Empty, "tetrahedra is empty; a mesh needs at least one tetrahedron", "tetrahedra"); }
            if (tetrahedra.Length % 4 != 0) { throw Err(StaticSolveError.BadIndex, "tetrahedra has " + Inv(tetrahedra.Length) + " entries, which is not a multiple of 4", "tetrahedra"); }
            if (vertices.Length > MaxVertexCount) { throw Err(StaticSolveError.OutOfRange, "vertices has " + Inv(vertices.Length) + " entries, the limit is " + Inv(MaxVertexCount), "vertices"); }
            if (tetrahedra.Length / 4 > MaxTetCount) { throw Err(StaticSolveError.OutOfRange, "tetrahedra holds " + Inv(tetrahedra.Length / 4) + " tetrahedra, the limit is " + Inv(MaxTetCount), "tetrahedra"); }
            _v = vertices.Length; _t = tetrahedra.Length / 4;
            _x = new double[3 * _v];
            for (int i = 0; i < _v; i++)
            {
                Vector3F p = vertices[i];
                if (!float.IsFinite(p.X) || !float.IsFinite(p.Y) || !float.IsFinite(p.Z)) { throw Err(StaticSolveError.NonFinite, "vertices[" + Inv(i) + "] = " + p + " is not finite", "vertices"); }
                double x = (double)p.X * _scale, y = (double)p.Y * _scale, z = (double)p.Z * _scale;
                if (Math.Abs(x) > MaxCoordinateMetres || Math.Abs(y) > MaxCoordinateMetres || Math.Abs(z) > MaxCoordinateMetres) { throw Err(StaticSolveError.OutOfRange, "vertices[" + Inv(i) + "] = " + p + " is beyond 1e9 m after scaling by MetresPerUnit", "vertices"); }
                _x[3 * i] = x; _x[3 * i + 1] = y; _x[3 * i + 2] = z;
            }

            _tets = tetrahedra.ToArray();
            var used = new bool[_v];
            for (int i = 0; i < _tets.Length; i++)
            {
                if ((uint)_tets[i] >= (uint)_v) { throw Err(StaticSolveError.BadIndex, "tetrahedra[" + Inv(i) + "] = " + Inv(_tets[i]) + " is outside the " + Inv(_v) + " vertices", "tetrahedra"); }
                used[_tets[i]] = true;
            }

            // Geometry: gradients of the barycentric coordinates and the volume (Zienkiewicz and Taylor, linear tetrahedron).
            _geo = new double[13 * _t];
            for (int t = 0; t < _t; t++)
            {
                int i0 = _tets[4 * t], i1 = _tets[4 * t + 1], i2 = _tets[4 * t + 2], i3 = _tets[4 * t + 3];
                if (i0 == i1 || i0 == i2 || i0 == i3 || i1 == i2 || i1 == i3 || i2 == i3) { throw Err(StaticSolveError.DegenerateTetrahedron, "tetrahedron " + Inv(t) + " repeats a vertex (" + Inv(i0) + ", " + Inv(i1) + ", " + Inv(i2) + ", " + Inv(i3) + ")", "tetrahedra"); }
                double ax = _x[3 * i1] - _x[3 * i0], ay = _x[3 * i1 + 1] - _x[3 * i0 + 1], az = _x[3 * i1 + 2] - _x[3 * i0 + 2];
                double bx = _x[3 * i2] - _x[3 * i0], by = _x[3 * i2 + 1] - _x[3 * i0 + 1], bz = _x[3 * i2 + 2] - _x[3 * i0 + 2];
                double cx = _x[3 * i3] - _x[3 * i0], cy = _x[3 * i3 + 1] - _x[3 * i0 + 1], cz = _x[3 * i3 + 2] - _x[3 * i0 + 2];
                double bcx = by * cz - bz * cy, bcy = bz * cx - bx * cz, bcz = bx * cy - by * cx;
                double det = ax * bcx + ay * bcy + az * bcz;
                double longest = Math.Max(Math.Max(ax * ax + ay * ay + az * az, bx * bx + by * by + bz * bz), cx * cx + cy * cy + cz * cz);
                double ex = bx - ax, ey = by - ay, ez = bz - az, fx = cx - ax, fy = cy - ay, fz = cz - az, gx = cx - bx, gy = cy - by, gz = cz - bz;
                longest = Math.Max(longest, Math.Max(ex * ex + ey * ey + ez * ez, Math.Max(fx * fx + fy * fy + fz * fz, gx * gx + gy * gy + gz * gz)));
                double tol = 1e-9 * longest * Math.Sqrt(longest);
                if (!(det > tol))
                {
                    if (det < -tol) { throw Err(StaticSolveError.InvertedTetrahedron, "tetrahedron " + Inv(t) + " (" + Inv(i0) + ", " + Inv(i1) + ", " + Inv(i2) + ", " + Inv(i3) + ") has negative volume; vertices 0, 1, 2 must run counter-clockwise seen from vertex 3", "tetrahedra"); }
                    throw Err(StaticSolveError.DegenerateTetrahedron, "tetrahedron " + Inv(t) + " (" + Inv(i0) + ", " + Inv(i1) + ", " + Inv(i2) + ", " + Inv(i3) + ") has no volume (flat or sliver)", "tetrahedra");
                }

                double s = 1.0 / det;
                int o = 13 * t;
                // grad l1 = (b x c)/det, grad l2 = (c x a)/det, grad l3 = (a x b)/det, grad l0 = -(sum)
                double g1x = bcx * s, g1y = bcy * s, g1z = bcz * s;
                double g2x = (cy * az - cz * ay) * s, g2y = (cz * ax - cx * az) * s, g2z = (cx * ay - cy * ax) * s;
                double g3x = (ay * bz - az * by) * s, g3y = (az * bx - ax * bz) * s, g3z = (ax * by - ay * bx) * s;
                _geo[o] = -(g1x + g2x + g3x); _geo[o + 1] = -(g1y + g2y + g3y); _geo[o + 2] = -(g1z + g2z + g3z);
                _geo[o + 3] = g1x; _geo[o + 4] = g1y; _geo[o + 5] = g1z;
                _geo[o + 6] = g2x; _geo[o + 7] = g2y; _geo[o + 8] = g2z;
                _geo[o + 9] = g3x; _geo[o + 10] = g3y; _geo[o + 11] = g3z;
                _geo[o + 12] = det / 6.0;
                _volume += det / 6.0;
            }

            for (int i = 0; i < _v; i++)
            {
                if (!used[i]) { throw Err(StaticSolveError.UnusedVertex, "vertices[" + Inv(i) + "] belongs to no tetrahedron; remove it or connect it", "vertices"); }
            }

            BuildNodes();
            BuildPattern();
        }

        private void ValidateMaterial(StaticProblem problem)
        {
            ElasticMaterial m = problem.Material;
            if (!double.IsFinite(m.YoungsModulus) || !double.IsFinite(m.PoissonRatio) || !double.IsFinite(m.Density) || !double.IsFinite(m.Strength) || !double.IsFinite(m.WeakAxisStrengthRatio))
            {
                throw Err(StaticSolveError.NonFinite, "problem.Material has a NaN or infinite value (E = " + Inv(m.YoungsModulus) + ", nu = " + Inv(m.PoissonRatio) + ", density = " + Inv(m.Density) + ", strength = " + Inv(m.Strength) + ")", "problem");
            }

            if (!(m.YoungsModulus > 0) || m.YoungsModulus > 1e18) { throw Err(StaticSolveError.OutOfRange, "problem.Material.YoungsModulus = " + Inv(m.YoungsModulus) + " must be above 0 and at most 1e18 Pa", "problem"); }
            if (!(m.PoissonRatio > -1) || !(m.PoissonRatio < 0.5)) { throw Err(StaticSolveError.OutOfRange, "problem.Material.PoissonRatio = " + Inv(m.PoissonRatio) + " must be above -1 and below 0.5 (0.5 is incompressible)", "problem"); }
            if (m.PoissonRatio > 0.499) { throw Err(StaticSolveError.OutOfRange, "problem.Material.PoissonRatio = " + Inv(m.PoissonRatio) + " is within 0.001 of 0.5, where the displacement formulation locks; use at most 0.499", "problem"); }
            if (m.Density < 0 || m.Density > 1e9) { throw Err(StaticSolveError.OutOfRange, "problem.Material.Density = " + Inv(m.Density) + " must be from 0 to 1e9 kg/m3", "problem"); }
            if (m.Strength < 0 || m.Strength > 1e30) { throw Err(StaticSolveError.OutOfRange, "problem.Material.Strength = " + Inv(m.Strength) + " must be from 0 to 1e30 Pa", "problem"); }
            _strength = m.Strength; _density = m.Density;
            double e = m.YoungsModulus, nu = m.PoissonRatio;
            _lambda = e * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
            _mu = e / (2.0 * (1.0 + nu));
            Vector3F w = m.WeakAxis;
            bool hasAxis = w.X != 0 || w.Y != 0 || w.Z != 0;
            double ratio = m.WeakAxisStrengthRatio == 0 ? 1.0 : m.WeakAxisStrengthRatio;
            if (!(ratio > 0 && ratio <= 1)) { throw Err(StaticSolveError.OutOfRange, "problem.Material.WeakAxisStrengthRatio = " + Inv(m.WeakAxisStrengthRatio) + " must be in (0, 1] (0 means 1)", "problem"); }
            if (hasAxis)
            {
                if (!float.IsFinite(w.X) || !float.IsFinite(w.Y) || !float.IsFinite(w.Z)) { throw Err(StaticSolveError.NonFinite, "problem.Material.WeakAxis = " + w + " is not finite", "problem"); }
                double x = w.X, y = w.Y, z = w.Z;
                double len = Math.Sqrt(x * x + y * y + z * z);
                if (!(len > 0) || !double.IsFinite(len)) { throw Err(StaticSolveError.OutOfRange, "problem.Material.WeakAxis = " + w + " has no usable length", "problem"); }
                _unitWeak[0] = x / len; _unitWeak[1] = y / len; _unitWeak[2] = z / len;
            }

            if (ratio < 1)
            {
                if (!hasAxis) { throw Err(StaticSolveError.OutOfRange, "problem.Material.WeakAxisStrengthRatio = " + Inv(ratio) + " needs a WeakAxis", "problem"); }
                if (!(m.Strength > 0)) { throw Err(StaticSolveError.OutOfRange, "problem.Material.WeakAxisStrengthRatio = " + Inv(ratio) + " needs a Strength above 0", "problem"); }
                _hasWeak = true; _weakRatio = ratio;
            }
        }

        // Provisional node numbering: vertex nodes and edge-midpoint nodes interleaved in the order (vertex v, then the edges whose smaller end is v by larger end);
        // Renumber then orders them in space.
        private void BuildNodes()
        {
            _nodes = new int[_t * _nn];
            _nodeOfVertex = new int[_v];
            if (!_quad)
            {
                for (int i = 0; i < _v; i++) { _nodeOfVertex[i] = i; }
                for (int t = 0; t < _t; t++) { for (int k = 0; k < 4; k++) { _nodes[4 * t + k] = _tets[4 * t + k]; } }
                _n = _v;
            }
            else
            {
                var keys = new long[6 * _t];
                for (int t = 0; t < _t; t++)
                {
                    for (int m = 0; m < 6; m++)
                    {
                        int a = _tets[4 * t + EdgeA[m]], b = _tets[4 * t + EdgeB[m]];
                        if (a > b) { (a, b) = (b, a); }
                        keys[6 * t + m] = (long)a * _v + b;
                    }
                }

                var sorted = (long[])keys.Clone();
                Array.Sort(sorted);
                int e = 0;
                for (int i = 0; i < sorted.Length; i++) { if (i == 0 || sorted[i] != sorted[i - 1]) { sorted[e++] = sorted[i]; } }
                _edgeKeys = new long[e];
                Array.Copy(sorted, _edgeKeys, e);
                _nodeOfEdge = new int[e];
                int node = 0, ei = 0;
                for (int v = 0; v < _v; v++)
                {
                    _nodeOfVertex[v] = node++;
                    while (ei < e && _edgeKeys[ei] / _v == v) { _nodeOfEdge[ei++] = node++; }
                }

                _n = node;
                for (int t = 0; t < _t; t++)
                {
                    for (int k = 0; k < 4; k++) { _nodes[10 * t + k] = _nodeOfVertex[_tets[4 * t + k]]; }
                    for (int m = 0; m < 6; m++) { _nodes[10 * t + 4 + m] = _nodeOfEdge[Array.BinarySearch(_edgeKeys, keys[6 * t + m])]; }
                }
            }

            Renumber();
            _dofs = 3 * _n;
            _chunks = (_n + NodeChunk - 1) / NodeChunk;
        }

        // Orders the nodes along a Morton curve of their positions (ties by the earlier number), so that blocks of consecutive nodes are compact pieces of the body:
        // the block preconditioner and the coarse space both work on such blocks. Keys carry the old number, so the sort has no ties and its result is fixed.
        private void Renumber()
        {
            var pos = new double[3 * _n];
            for (int v = 0; v < _v; v++) { int nd = _nodeOfVertex[v]; pos[3 * nd] = _x[3 * v]; pos[3 * nd + 1] = _x[3 * v + 1]; pos[3 * nd + 2] = _x[3 * v + 2]; }
            for (int e = 0; e < _edgeKeys.Length; e++)
            {
                int a = (int)(_edgeKeys[e] / _v), b = (int)(_edgeKeys[e] % _v), nd = _nodeOfEdge[e];
                for (int k = 0; k < 3; k++) { pos[3 * nd + k] = 0.5 * (_x[3 * a + k] + _x[3 * b + k]); }
            }

            double x0 = double.MaxValue, y0 = double.MaxValue, z0 = double.MaxValue, x1 = double.MinValue, y1 = double.MinValue, z1 = double.MinValue;
            for (int i = 0; i < _n; i++)
            {
                x0 = Math.Min(x0, pos[3 * i]); x1 = Math.Max(x1, pos[3 * i]);
                y0 = Math.Min(y0, pos[3 * i + 1]); y1 = Math.Max(y1, pos[3 * i + 1]);
                z0 = Math.Min(z0, pos[3 * i + 2]); z1 = Math.Max(z1, pos[3 * i + 2]);
            }

            double ext = Math.Max(x1 - x0, Math.Max(y1 - y0, z1 - z0));
            double sc = ext > 0 ? 8191.0 / ext : 0;
            var keys = new long[_n];
            for (int i = 0; i < _n; i++)
            {
                long code = 0;
                long qx = (long)((pos[3 * i] - x0) * sc), qy = (long)((pos[3 * i + 1] - y0) * sc), qz = (long)((pos[3 * i + 2] - z0) * sc);
                for (int b = 0; b < 13; b++) { code |= (((qx >> b) & 1) << (3 * b)) | (((qy >> b) & 1) << (3 * b + 1)) | (((qz >> b) & 1) << (3 * b + 2)); }
                keys[i] = (code << 24) | (long)i;
            }

            Array.Sort(keys);
            var perm = new int[_n];
            _nodePos = new double[3 * _n];
            for (int i = 0; i < _n; i++)
            {
                int old = (int)(keys[i] & 0xFFFFFF);
                perm[old] = i;
                _nodePos[3 * i] = pos[3 * old]; _nodePos[3 * i + 1] = pos[3 * old + 1]; _nodePos[3 * i + 2] = pos[3 * old + 2];
            }

            for (int i = 0; i < _nodes.Length; i++) { _nodes[i] = perm[_nodes[i]]; }
            for (int v = 0; v < _v; v++) { _nodeOfVertex[v] = perm[_nodeOfVertex[v]]; }
            for (int e = 0; e < _nodeOfEdge.Length; e++) { _nodeOfEdge[e] = perm[_nodeOfEdge[e]]; }
        }

        private void BuildPattern()
        {
            // Node -> tetrahedra (ascending), then each row's columns are the nodes of its tetrahedra, sorted and unique.
            _incPtr = new int[_n + 1];
            for (int i = 0; i < _nodes.Length; i++) { _incPtr[_nodes[i] + 1]++; }
            int maxInc = 0;
            for (int i = 0; i < _n; i++) { maxInc = Math.Max(maxInc, _incPtr[i + 1]); _incPtr[i + 1] += _incPtr[i]; }
            _incTet = new int[_nodes.Length];
            var fill = new int[_n];
            for (int t = 0; t < _t; t++)
            {
                for (int k = 0; k < _nn; k++)
                {
                    int nd = _nodes[t * _nn + k];
                    _incTet[_incPtr[nd] + fill[nd]++] = t;
                }
            }

            int cap = maxInc * _nn;
            var counts = new int[_n];
            For(_chunks, _threads, c =>
            {
                var buf = new int[cap];
                int hi = Math.Min(_n, (c + 1) * NodeChunk);
                for (int i = c * NodeChunk; i < hi; i++) { counts[i] = RowColumns(i, buf); }
            });
            _rowPtr = new int[_n + 1];
            long total = 0;
            for (int i = 0; i < _n; i++) { total += counts[i]; _rowPtr[i + 1] = (int)Math.Min(total, int.MaxValue); }
            if (total * 9 > 2_000_000_000L) { throw Err(StaticSolveError.OutOfRange, "the system has " + total.ToString(CultureInfo.InvariantCulture) + " 3x3 blocks, which is more than this solver stores; use fewer or larger tetrahedra, or linear elements", "tetrahedra"); }
            _cols = new int[(int)total];
            For(_chunks, _threads, c =>
            {
                var buf = new int[cap];
                int hi = Math.Min(_n, (c + 1) * NodeChunk);
                for (int i = c * NodeChunk; i < hi; i++)
                {
                    int len = RowColumns(i, buf);
                    Array.Copy(buf, 0, _cols, _rowPtr[i], len);
                }
            });
        }

        private int RowColumns(int i, int[] buf)
        {
            int len = 0;
            for (int q = _incPtr[i]; q < _incPtr[i + 1]; q++)
            {
                int t = _incTet[q];
                for (int k = 0; k < _nn; k++) { buf[len++] = _nodes[t * _nn + k]; }
            }

            Array.Sort(buf, 0, len);
            int u = 0;
            for (int j = 0; j < len; j++) { if (j == 0 || buf[j] != buf[j - 1]) { buf[u++] = buf[j]; } }
            return u;
        }

        // Gradients of the shape functions at the barycentric point l, into g[3 * node + axis].
        private void Gradients(int t, double l0, double l1, double l2, double l3, Span<double> g)
        {
            int o = 13 * t;
            if (!_quad)
            {
                for (int k = 0; k < 12; k++) { g[k] = _geo[o + k]; }
                return;
            }

            // Corner nodes N = l (2 l - 1), edge nodes N = 4 l_i l_j (Zienkiewicz and Taylor, ten-node tetrahedron).
            Span<double> l = stackalloc double[4];
            l[0] = l0; l[1] = l1; l[2] = l2; l[3] = l3;
            for (int i = 0; i < 4; i++)
            {
                double c = 4.0 * l[i] - 1.0;
                g[3 * i] = c * _geo[o + 3 * i]; g[3 * i + 1] = c * _geo[o + 3 * i + 1]; g[3 * i + 2] = c * _geo[o + 3 * i + 2];
            }

            for (int m = 0; m < 6; m++)
            {
                int a = EdgeA[m], b = EdgeB[m];
                for (int j = 0; j < 3; j++) { g[3 * (4 + m) + j] = 4.0 * (l[a] * _geo[o + 3 * b + j] + l[b] * _geo[o + 3 * a + j]); }
            }
        }

        private void GaussPoint(int gp, out double l0, out double l1, out double l2, out double l3)
        {
            if (!_quad) { l0 = l1 = l2 = l3 = 0.25; return; }
            l0 = gp == 0 ? GpB : GpA; l1 = gp == 1 ? GpB : GpA; l2 = gp == 2 ? GpB : GpA; l3 = gp == 3 ? GpB : GpA;
        }

        public void Assemble()
        {
            _vals = new double[_cols.Length * 9];
            For(_chunks, _threads, c =>
            {
                int nn = _nn;
                Span<double> g = stackalloc double[4 * 30];
                Span<double> acc = stackalloc double[9];
                int hi = Math.Min(_n, (c + 1) * NodeChunk);
                for (int i = c * NodeChunk; i < hi; i++)
                {
                    int rp = _rowPtr[i], len = _rowPtr[i + 1] - rp;
                    for (int q = _incPtr[i]; q < _incPtr[i + 1]; q++)
                    {
                        int t = _incTet[q];
                        int la = 0;
                        while (_nodes[t * nn + la] != i) { la++; }
                        double vol = _geo[13 * t + 12];
                        double w = vol / _ngp;
                        for (int gp = 0; gp < _ngp; gp++)
                        {
                            GaussPoint(gp, out double l0, out double l1, out double l2, out double l3);
                            Gradients(t, l0, l1, l2, l3, g.Slice(gp * 3 * nn, 3 * nn));
                        }

                        for (int lb = 0; lb < nn; lb++)
                        {
                            for (int k = 0; k < 9; k++) { acc[k] = 0; }
                            for (int gp = 0; gp < _ngp; gp++)
                            {
                                int ga = gp * 3 * nn + 3 * la, gb = gp * 3 * nn + 3 * lb;
                                double dot = g[ga] * g[gb] + g[ga + 1] * g[gb + 1] + g[ga + 2] * g[gb + 2];
                                for (int r = 0; r < 3; r++)
                                {
                                    for (int s = 0; s < 3; s++)
                                    {
                                        // K_ab[r,s] = lambda dN_a/dx_r dN_b/dx_s + mu dN_a/dx_s dN_b/dx_r + mu delta_rs grad N_a . grad N_b
                                        double v = _lambda * (g[ga + r] * g[gb + s]) + _mu * (g[ga + s] * g[gb + r]);
                                        if (r == s) { v += _mu * dot; }
                                        acc[3 * r + s] += w * v;
                                    }
                                }
                            }

                            int col = _nodes[t * nn + lb];
                            int pos = Array.BinarySearch(_cols, rp, len, col);
                            int vo = 9 * pos;
                            for (int k = 0; k < 9; k++) { _vals[vo + k] += acc[k]; }
                        }
                    }
                }
            });
        }

        public void ApplySupportsAndLoads(StaticProblem problem)
        {
            _fixed = new byte[_dofs];
            _presc = new double[_dofs];
            _f = new double[_dofs];
            if (problem.Supports.Count == 0) { throw Err(StaticSolveError.NoSupports, "problem.Supports is empty; a static solve needs supports that hold every rigid-body motion", "problem"); }
            var seen = new bool[_v];
            foreach (StaticSupport s in problem.Supports)
            {
                if ((uint)s.Vertex >= (uint)_v) { throw Err(StaticSolveError.BadIndex, "a support names vertex " + Inv(s.Vertex) + ", outside the " + Inv(_v) + " vertices", "problem"); }
                if (((int)s.Axes & ~7) != 0 || s.Axes == SupportAxes.None) { throw Err(StaticSolveError.BadSupport, "the support at vertex " + Inv(s.Vertex) + " has axes " + Inv((int)s.Axes) + "; expected a combination of X, Y and Z, not none", "problem"); }
                if (seen[s.Vertex]) { throw Err(StaticSolveError.BadSupport, "vertex " + Inv(s.Vertex) + " has two supports; merge them into one", "problem"); }
                seen[s.Vertex] = true;
                Vector3D d = s.Displacement;
                if (!double.IsFinite(d.X) || !double.IsFinite(d.Y) || !double.IsFinite(d.Z) || Math.Abs(d.X) > MaxCoordinateMetres || Math.Abs(d.Y) > MaxCoordinateMetres || Math.Abs(d.Z) > MaxCoordinateMetres)
                {
                    throw Err(StaticSolveError.NonFinite, "the support at vertex " + Inv(s.Vertex) + " has displacement " + d + ", which is not finite or beyond 1e9 m", "problem");
                }

                int nd = _nodeOfVertex[s.Vertex];
                for (int a = 0; a < 3; a++)
                {
                    if (((int)s.Axes & (1 << a)) != 0)
                    {
                        _fixed[3 * nd + a] = 1;
                        _presc[3 * nd + a] = a == 0 ? d.X : a == 1 ? d.Y : d.Z;
                    }
                }
            }

            // An edge whose two ends are held on an axis is held there at their mean, so a held face holds its midside nodes too.
            for (int e = 0; e < _edgeKeys.Length; e++)
            {
                int a = (int)(_edgeKeys[e] / _v), b = (int)(_edgeKeys[e] % _v);
                int na = _nodeOfVertex[a], nb = _nodeOfVertex[b], ne = _nodeOfEdge[e];
                for (int k = 0; k < 3; k++)
                {
                    if (_fixed[3 * na + k] != 0 && _fixed[3 * nb + k] != 0)
                    {
                        _fixed[3 * ne + k] = 1;
                        _presc[3 * ne + k] = 0.5 * (_presc[3 * na + k] + _presc[3 * nb + k]);
                    }
                }
            }

            CheckMechanism();

            // Loads.
            Vector3D g = problem.Gravity;
            if (!double.IsFinite(g.X) || !double.IsFinite(g.Y) || !double.IsFinite(g.Z) || Math.Abs(g.X) > 1e12 || Math.Abs(g.Y) > 1e12 || Math.Abs(g.Z) > 1e12) { throw Err(StaticSolveError.NonFinite, "problem.Gravity = " + g + " is not finite or beyond 1e12 m/s2", "problem"); }
            if (_density != 0 && (g.X != 0 || g.Y != 0 || g.Z != 0)) { AddGravity(g); }
            foreach (PointLoad p in problem.PointLoads)
            {
                if ((uint)p.Vertex >= (uint)_v) { throw Err(StaticSolveError.BadIndex, "a point load names vertex " + Inv(p.Vertex) + ", outside the " + Inv(_v) + " vertices", "problem"); }
                Vector3D fo = p.Force;
                CheckLoad(fo, "point load at vertex " + Inv(p.Vertex));
                int nd = _nodeOfVertex[p.Vertex];
                _f[3 * nd] += fo.X; _f[3 * nd + 1] += fo.Y; _f[3 * nd + 2] += fo.Z;
            }

            foreach (FaceLoad fl in problem.FaceLoads) { AddFace(fl); }
        }

        private static void CheckLoad(Vector3D v, string what)
        {
            if (!double.IsFinite(v.X) || !double.IsFinite(v.Y) || !double.IsFinite(v.Z) || Math.Abs(v.X) > MaxLoad || Math.Abs(v.Y) > MaxLoad || Math.Abs(v.Z) > MaxLoad)
            {
                throw Err(StaticSolveError.NonFinite, "the " + what + " has value " + v + ", which is not finite or beyond 1e30", "problem");
            }
        }

        private void AddGravity(Vector3D g)
        {
            Span<double> sh = stackalloc double[10];
            Span<double> l = stackalloc double[4];
            for (int t = 0; t < _t; t++)
            {
                double w = _density * _geo[13 * t + 12] / _ngp;
                for (int gp = 0; gp < _ngp; gp++)
                {
                    GaussPoint(gp, out double l0, out double l1, out double l2, out double l3);
                    if (!_quad)
                    {
                        sh[0] = sh[1] = sh[2] = sh[3] = 0.25;
                    }
                    else
                    {
                        l[0] = l0; l[1] = l1; l[2] = l2; l[3] = l3;
                        for (int i = 0; i < 4; i++) { sh[i] = l[i] * (2.0 * l[i] - 1.0); }
                        for (int m = 0; m < 6; m++) { sh[4 + m] = 4.0 * l[EdgeA[m]] * l[EdgeB[m]]; }
                    }

                    for (int a = 0; a < _nn; a++)
                    {
                        int nd = _nodes[t * _nn + a];
                        double s = w * sh[a];
                        _f[3 * nd] += s * g.X; _f[3 * nd + 1] += s * g.Y; _f[3 * nd + 2] += s * g.Z;
                    }
                }
            }
        }

        private void AddFace(FaceLoad fl)
        {
            int a = fl.V0, b = fl.V1, c = fl.V2;
            if ((uint)a >= (uint)_v || (uint)b >= (uint)_v || (uint)c >= (uint)_v) { throw Err(StaticSolveError.BadIndex, "a face load names vertices (" + Inv(a) + ", " + Inv(b) + ", " + Inv(c) + "), outside the " + Inv(_v) + " vertices", "problem"); }
            if (a == b || a == c || b == c) { throw Err(StaticSolveError.BadFace, "a face load repeats a vertex (" + Inv(a) + ", " + Inv(b) + ", " + Inv(c) + ")", "problem"); }
            if (!double.IsFinite(fl.Pressure) || Math.Abs(fl.Pressure) > MaxLoad) { throw Err(StaticSolveError.NonFinite, "the face load on (" + Inv(a) + ", " + Inv(b) + ", " + Inv(c) + ") has pressure " + Inv(fl.Pressure) + ", which is not finite or beyond 1e30", "problem"); }
            CheckLoad(fl.Traction, "face traction on (" + Inv(a) + ", " + Inv(b) + ", " + Inv(c) + ")");

            // The owning tetrahedron: the one holding all three vertices, which must be unique (a boundary face).
            int owner = -1, count = 0, opp = -1;
            int na = _nodeOfVertex[a];
            for (int q = _incPtr[na]; q < _incPtr[na + 1]; q++)
            {
                int t = _incTet[q];
                int hit = 0, other = -1;
                for (int k = 0; k < 4; k++)
                {
                    int vk = _tets[4 * t + k];
                    if (vk == a || vk == b || vk == c) { hit++; } else { other = vk; }
                }

                if (hit == 3) { count++; if (owner < 0) { owner = t; opp = other; } }
            }

            if (count != 1) { throw Err(StaticSolveError.BadFace, "(" + Inv(a) + ", " + Inv(b) + ", " + Inv(c) + ") is " + (count == 0 ? "not a face of any tetrahedron" : "an interior face shared by two tetrahedra") + "; face loads go on boundary faces", "problem"); }

            double ax = _x[3 * a], ay = _x[3 * a + 1], az = _x[3 * a + 2];
            double ux = _x[3 * b] - ax, uy = _x[3 * b + 1] - ay, uz = _x[3 * b + 2] - az;
            double vx = _x[3 * c] - ax, vy = _x[3 * c + 1] - ay, vz = _x[3 * c + 2] - az;
            double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
            double dx = _x[3 * opp] - ax, dy = _x[3 * opp + 1] - ay, dz = _x[3 * opp + 2] - az;
            if (nx * dx + ny * dy + nz * dz > 0) { nx = -nx; ny = -ny; nz = -nz; }
            double area2 = Math.Sqrt(nx * nx + ny * ny + nz * nz);
            double area = 0.5 * area2;
            if (!(area > 0)) { throw Err(StaticSolveError.BadFace, "face (" + Inv(a) + ", " + Inv(b) + ", " + Inv(c) + ") has no area", "problem"); }
            double inv = 1.0 / area2;
            // Outward unit normal (nx, ny, nz) * inv; pressure pushes inward.
            double tx = fl.Traction.X - fl.Pressure * nx * inv, ty = fl.Traction.Y - fl.Pressure * ny * inv, tz = fl.Traction.Z - fl.Pressure * nz * inv;
            double share = area / 3.0;
            // Consistent nodal loads of a uniform traction on a flat face: zero at the corners and a third of the area at each midside node for quadratic elements (integral of the shape functions), a third at each corner for linear ones.
            if (!_quad)
            {
                AddNodal(_nodeOfVertex[a], share, tx, ty, tz); AddNodal(_nodeOfVertex[b], share, tx, ty, tz); AddNodal(_nodeOfVertex[c], share, tx, ty, tz);
            }
            else
            {
                AddNodal(EdgeNode(a, b), share, tx, ty, tz); AddNodal(EdgeNode(b, c), share, tx, ty, tz); AddNodal(EdgeNode(a, c), share, tx, ty, tz);
            }
        }

        private int EdgeNode(int a, int b)
        {
            if (a > b) { (a, b) = (b, a); }
            return _nodeOfEdge[Array.BinarySearch(_edgeKeys, (long)a * _v + b)];
        }

        private void AddNodal(int nd, double share, double tx, double ty, double tz)
        {
            _f[3 * nd] += share * tx; _f[3 * nd + 1] += share * ty; _f[3 * nd + 2] += share * tz;
        }

        // A support set that leaves a rigid motion of the mesh, or of a connected part of it, free is a mechanism: the six rigid modes sampled at the held
        // degrees of freedom must have a positive definite Gram matrix (checked by Cholesky), per connected component of the mesh.
        private void CheckMechanism()
        {
            var parent = new int[_v];
            for (int i = 0; i < _v; i++) { parent[i] = i; }
            int Find(int x) { while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; }
            for (int t = 0; t < _t; t++)
            {
                int r0 = Find(_tets[4 * t]);
                for (int k = 1; k < 4; k++)
                {
                    int r = Find(_tets[4 * t + k]);
                    if (r != r0) { if (r < r0) { parent[r0] = r; r0 = r; } else { parent[r] = r0; } }
                }
            }

            // Component of each node: that of its (smaller-end) vertex; centre and size per component.
            var comp = new int[_v];
            int comps = 0;
            var compIndex = new int[_v];
            Array.Fill(compIndex, -1);
            for (int v = 0; v < _v; v++)
            {
                int r = Find(v);
                if (compIndex[r] < 0) { compIndex[r] = comps++; }
                comp[v] = compIndex[r];
            }

            var lo = new double[comps * 3]; var hi = new double[comps * 3];
            Array.Fill(lo, double.MaxValue); Array.Fill(hi, double.MinValue);
            for (int v = 0; v < _v; v++)
            {
                for (int k = 0; k < 3; k++)
                {
                    lo[3 * comp[v] + k] = Math.Min(lo[3 * comp[v] + k], _x[3 * v + k]);
                    hi[3 * comp[v] + k] = Math.Max(hi[3 * comp[v] + k], _x[3 * v + k]);
                }
            }

            var gram = new double[comps * 36];
            var size = new double[comps];
            for (int c = 0; c < comps; c++)
            {
                size[c] = Math.Max(Math.Max(hi[3 * c] - lo[3 * c], hi[3 * c + 1] - lo[3 * c + 1]), Math.Max(hi[3 * c + 2] - lo[3 * c + 2], 1e-300));
            }

            var m = new double[6];
            void Accumulate(int node, int vertexA, int vertexB)
            {
                // Position of the node: a vertex, or the midpoint of an edge.
                double px, py, pz;
                if (vertexB < 0) { px = _x[3 * vertexA]; py = _x[3 * vertexA + 1]; pz = _x[3 * vertexA + 2]; }
                else { px = 0.5 * (_x[3 * vertexA] + _x[3 * vertexB]); py = 0.5 * (_x[3 * vertexA + 1] + _x[3 * vertexB + 1]); pz = 0.5 * (_x[3 * vertexA + 2] + _x[3 * vertexB + 2]); }
                int c = comp[vertexA];
                double cx = 0.5 * (lo[3 * c] + hi[3 * c]), cy = 0.5 * (lo[3 * c + 1] + hi[3 * c + 1]), cz = 0.5 * (lo[3 * c + 2] + hi[3 * c + 2]);
                double rx = (px - cx) / size[c], ry = (py - cy) / size[c], rz = (pz - cz) / size[c];
                for (int a = 0; a < 3; a++)
                {
                    if (_fixed[3 * node + a] == 0) { continue; }
                    // translation a; rotation about x, y, z: (e_k x r)_a
                    m[0] = a == 0 ? 1 : 0; m[1] = a == 1 ? 1 : 0; m[2] = a == 2 ? 1 : 0;
                    m[3] = a == 0 ? 0 : a == 1 ? -rz : ry;
                    m[4] = a == 0 ? rz : a == 1 ? 0 : -rx;
                    m[5] = a == 0 ? -ry : a == 1 ? rx : 0;
                    for (int i = 0; i < 6; i++) { for (int j = 0; j < 6; j++) { gram[36 * c + 6 * i + j] += m[i] * m[j]; } }
                }
            }

            for (int v = 0; v < _v; v++) { Accumulate(_nodeOfVertex[v], v, -1); }
            for (int e = 0; e < _edgeKeys.Length; e++) { Accumulate(_nodeOfEdge[e], (int)(_edgeKeys[e] / _v), (int)(_edgeKeys[e] % _v)); }

            for (int c = 0; c < comps; c++)
            {
                if (!PositiveDefinite(gram.AsSpan(36 * c, 36)))
                {
                    int first = 0;
                    for (int v = 0; v < _v; v++) { if (comp[v] == c) { first = v; break; } }
                    throw Err(StaticSolveError.Mechanism, comps == 1
                        ? "the supports leave a rigid-body motion of the mesh free (a translation or rotation); hold at least three non-collinear points, or one point and two rollers"
                        : "the supports leave a rigid-body motion free for the connected part of the mesh that contains vertex " + Inv(first) + " (the mesh has " + Inv(comps) + " separate parts, each needs its own supports)", "problem");
                }
            }
        }

        private static bool PositiveDefinite(Span<double> g)
        {
            // In-place Cholesky of the 6x6 Gram matrix; a pivot below 1e-10 of its diagonal entry means a free motion.
            Span<double> d = stackalloc double[6];
            for (int i = 0; i < 6; i++) { d[i] = g[6 * i + i]; }
            for (int j = 0; j < 6; j++)
            {
                double s = g[6 * j + j];
                for (int k = 0; k < j; k++) { s -= g[6 * j + k] * g[6 * j + k]; }
                double scale = Math.Max(d[j], 1e-300);
                if (!(s > 1e-10 * scale)) { return false; }
                double dj = Math.Sqrt(s);
                g[6 * j + j] = dj;
                for (int i = j + 1; i < 6; i++)
                {
                    double t = g[6 * i + j];
                    for (int k = 0; k < j; k++) { t -= g[6 * i + k] * g[6 * j + k]; }
                    g[6 * i + j] = t / dj;
                }
            }

            return true;
        }

        // y = K x over the rows of one chunk; masked zeroes the held components of the result (the operator of the reduced system).
        private void MultiplyChunk(int c, double[] x, double[] y, bool masked)
        {
            int hi = Math.Min(_n, (c + 1) * NodeChunk);
            for (int i = c * NodeChunk; i < hi; i++)
            {
                double s0 = 0, s1 = 0, s2 = 0;
                for (int q = _rowPtr[i]; q < _rowPtr[i + 1]; q++)
                {
                    int j = 3 * _cols[q], o = 9 * q;
                    double x0 = x[j], x1 = x[j + 1], x2 = x[j + 2];
                    s0 += _vals[o] * x0 + _vals[o + 1] * x1 + _vals[o + 2] * x2;
                    s1 += _vals[o + 3] * x0 + _vals[o + 4] * x1 + _vals[o + 5] * x2;
                    s2 += _vals[o + 6] * x0 + _vals[o + 7] * x1 + _vals[o + 8] * x2;
                }

                if (masked)
                {
                    if (_fixed[3 * i] != 0) { s0 = 0; }
                    if (_fixed[3 * i + 1] != 0) { s1 = 0; }
                    if (_fixed[3 * i + 2] != 0) { s2 = 0; }
                }

                y[3 * i] = s0; y[3 * i + 1] = s1; y[3 * i + 2] = s2;
            }
        }

        // z = M^-1 r for one chunk: symmetric block Gauss-Seidel (D + L) D^-1 (D + U) inside the chunk, with held components replaced by identity rows.
        private void PreconditionChunk(int c, double[] r, double[] t, double[] w, double[] z)
        {
            int lo = c * NodeChunk, hi = Math.Min(_n, lo + NodeChunk);
            for (int i = lo; i < hi; i++)
            {
                double s0 = r[3 * i], s1 = r[3 * i + 1], s2 = r[3 * i + 2];
                for (int q = _rowPtr[i]; q < _rowPtr[i + 1]; q++)
                {
                    int j = _cols[q];
                    if (j >= i) { break; }
                    if (j < lo) { continue; }
                    int o = 9 * q;
                    double x0 = t[3 * j], x1 = t[3 * j + 1], x2 = t[3 * j + 2];
                    s0 -= _vals[o] * x0 + _vals[o + 1] * x1 + _vals[o + 2] * x2;
                    s1 -= _vals[o + 3] * x0 + _vals[o + 4] * x1 + _vals[o + 5] * x2;
                    s2 -= _vals[o + 6] * x0 + _vals[o + 7] * x1 + _vals[o + 8] * x2;
                }

                if (_fixed[3 * i] != 0) { s0 = 0; }
                if (_fixed[3 * i + 1] != 0) { s1 = 0; }
                if (_fixed[3 * i + 2] != 0) { s2 = 0; }
                w[3 * i] = s0; w[3 * i + 1] = s1; w[3 * i + 2] = s2;
                int d = 9 * i;
                t[3 * i] = _invD[d] * s0 + _invD[d + 1] * s1 + _invD[d + 2] * s2;
                t[3 * i + 1] = _invD[d + 3] * s0 + _invD[d + 4] * s1 + _invD[d + 5] * s2;
                t[3 * i + 2] = _invD[d + 6] * s0 + _invD[d + 7] * s1 + _invD[d + 8] * s2;
            }

            for (int i = hi - 1; i >= lo; i--)
            {
                double s0 = w[3 * i], s1 = w[3 * i + 1], s2 = w[3 * i + 2];
                for (int q = _rowPtr[i + 1] - 1; q >= _rowPtr[i]; q--)
                {
                    int j = _cols[q];
                    if (j <= i) { break; }
                    if (j >= hi) { continue; }
                    int o = 9 * q;
                    double x0 = z[3 * j], x1 = z[3 * j + 1], x2 = z[3 * j + 2];
                    s0 -= _vals[o] * x0 + _vals[o + 1] * x1 + _vals[o + 2] * x2;
                    s1 -= _vals[o + 3] * x0 + _vals[o + 4] * x1 + _vals[o + 5] * x2;
                    s2 -= _vals[o + 6] * x0 + _vals[o + 7] * x1 + _vals[o + 8] * x2;
                }

                if (_fixed[3 * i] != 0) { s0 = 0; }
                if (_fixed[3 * i + 1] != 0) { s1 = 0; }
                if (_fixed[3 * i + 2] != 0) { s2 = 0; }
                int d = 9 * i;
                z[3 * i] = _invD[d] * s0 + _invD[d + 1] * s1 + _invD[d + 2] * s2;
                z[3 * i + 1] = _invD[d + 3] * s0 + _invD[d + 4] * s1 + _invD[d + 5] * s2;
                z[3 * i + 2] = _invD[d + 6] * s0 + _invD[d + 7] * s1 + _invD[d + 8] * s2;
            }
        }

        private void BuildDiagonal()
        {
            _invD = new double[9 * _n];
            For(_chunks, _threads, c =>
            {
                Span<double> blk = stackalloc double[9];
                int hi = Math.Min(_n, (c + 1) * NodeChunk);
                for (int i = c * NodeChunk; i < hi; i++)
                {
                    int pos = Array.BinarySearch(_cols, _rowPtr[i], _rowPtr[i + 1] - _rowPtr[i], i);
                    for (int k = 0; k < 9; k++) { blk[k] = _vals[9 * pos + k]; }
                    for (int a = 0; a < 3; a++)
                    {
                        if (_fixed[3 * i + a] == 0) { continue; }
                        for (int k = 0; k < 3; k++) { blk[3 * a + k] = 0; blk[3 * k + a] = 0; }
                        blk[3 * a + a] = 1;
                    }

                    Invert3(blk, _invD.AsSpan(9 * i, 9));
                }
            });
        }

        // Two-level preconditioner (additive; Smith, Bjorstad and Gropp, ch. 3, on coarse spaces; Vanek, Mandel and Brezina on rigid-body-mode aggregates): the coarse space is the six rigid motions
        // of each aggregate of consecutive (Morton-ordered) nodes, the coarse matrix its Galerkin product with the reduced stiffness, solved by a dense Cholesky factor. It carries the long
        // wavelength bending and twisting modes, which the block Gauss-Seidel sweep alone needs thousands of iterations to remove on a slender body.
        private void BuildCoarse()
        {
            int chunksPerAgg = Math.Max(1, (_chunks + 127) / 128);
            _aggSize = chunksPerAgg * NodeChunk;
            _aggCount = (_n + _aggSize - 1) / _aggSize;
            int m = 6 * _aggCount;
            _coarseP = new double[18 * _n];
            For(_aggCount, _threads, g =>
            {
                int lo = g * _aggSize, hi = Math.Min(_n, lo + _aggSize);
                double cx = 0, cy = 0, cz = 0;
                for (int i = lo; i < hi; i++) { cx += _nodePos[3 * i]; cy += _nodePos[3 * i + 1]; cz += _nodePos[3 * i + 2]; }
                double inv = 1.0 / (hi - lo);
                cx *= inv; cy *= inv; cz *= inv;
                double r2 = 0;
                for (int i = lo; i < hi; i++)
                {
                    double dx = _nodePos[3 * i] - cx, dy = _nodePos[3 * i + 1] - cy, dz = _nodePos[3 * i + 2] - cz;
                    r2 += dx * dx + dy * dy + dz * dz;
                }

                double radius = Math.Sqrt(r2 * inv);
                if (!(radius > 0)) { radius = 1; }
                for (int i = lo; i < hi; i++)
                {
                    double rx = (_nodePos[3 * i] - cx) / radius, ry = (_nodePos[3 * i + 1] - cy) / radius, rz = (_nodePos[3 * i + 2] - cz) / radius;
                    int o = 18 * i;
                    // u = t + (w x r): rows are the displacement components, columns t_x t_y t_z w_x w_y w_z.
                    _coarseP[o] = 1; _coarseP[o + 4] = rz; _coarseP[o + 5] = -ry;
                    _coarseP[o + 7] = 1; _coarseP[o + 9] = -rz; _coarseP[o + 11] = rx;
                    _coarseP[o + 14] = 1; _coarseP[o + 15] = ry; _coarseP[o + 16] = -rx;
                    for (int a = 0; a < 3; a++)
                    {
                        if (_fixed[3 * i + a] != 0) { for (int c = 0; c < 6; c++) { _coarseP[o + 6 * a + c] = 0; } }
                    }
                }
            });

            var ac = new double[m * m];
            For(_aggCount, _threads, g =>
            {
                var row = new double[6 * m];
                var tt = new double[18];
                int lo = g * _aggSize, hi = Math.Min(_n, lo + _aggSize);
                for (int i = lo; i < hi; i++)
                {
                    int po = 18 * i;
                    for (int q = _rowPtr[i]; q < _rowPtr[i + 1]; q++)
                    {
                        int j = _cols[q], jo = 18 * j, ko = 9 * q, jg = j / _aggSize;
                        for (int r = 0; r < 3; r++)
                        {
                            for (int c = 0; c < 6; c++) { tt[6 * r + c] = _vals[ko + 3 * r] * _coarseP[jo + c] + _vals[ko + 3 * r + 1] * _coarseP[jo + 6 + c] + _vals[ko + 3 * r + 2] * _coarseP[jo + 12 + c]; }
                        }

                        for (int a = 0; a < 6; a++)
                        {
                            int ro = a * m + 6 * jg;
                            double p0 = _coarseP[po + a], p1 = _coarseP[po + 6 + a], p2 = _coarseP[po + 12 + a];
                            for (int c = 0; c < 6; c++) { row[ro + c] += p0 * tt[c] + p1 * tt[6 + c] + p2 * tt[12 + c]; }
                        }
                    }
                }

                Array.Copy(row, 0, ac, 6 * g * m, 6 * m);
            });

            // Cholesky of the lower triangle; a coarse unknown with no stiffness left (an aggregate that is held, or whose nodes are collinear) is dropped: it solves to zero and decouples.
            var orig = new double[m];
            for (int i = 0; i < m; i++) { orig[i] = ac[i * m + i]; }
            _coarseDropped = new bool[m];
            for (int j = 0; j < m; j++)
            {
                double d = ac[j * m + j];
                for (int k = 0; k < j; k++) { d -= ac[j * m + k] * ac[j * m + k]; }
                if (!(orig[j] > 0) || !(d > 1e-12 * orig[j]))
                {
                    _coarseDropped[j] = true;
                    for (int k = 0; k < j; k++) { ac[j * m + k] = 0; }
                    ac[j * m + j] = 1;
                    for (int i = j + 1; i < m; i++) { ac[i * m + j] = 0; }
                    continue;
                }

                double dj = Math.Sqrt(d);
                ac[j * m + j] = dj;
                for (int i = j + 1; i < m; i++)
                {
                    double t = ac[i * m + j];
                    for (int k = 0; k < j; k++) { t -= ac[i * m + k] * ac[j * m + k]; }
                    ac[i * m + j] = t / dj;
                }
            }

            _coarseL = ac;
        }

        private bool[] _coarseDropped = Array.Empty<bool>();

        private void CoarseSolve(double[] rc, double[] yc)
        {
            int m = 6 * _aggCount;
            for (int i = 0; i < m; i++)
            {
                double t = rc[i];
                for (int k = 0; k < i; k++) { t -= _coarseL[i * m + k] * yc[k]; }
                yc[i] = t / _coarseL[i * m + i];
            }

            for (int i = m - 1; i >= 0; i--)
            {
                double t = yc[i];
                for (int k = i + 1; k < m; k++) { t -= _coarseL[k * m + i] * yc[k]; }
                yc[i] = t / _coarseL[i * m + i];
            }

            for (int i = 0; i < m; i++) { if (_coarseDropped[i]) { yc[i] = 0; } }
        }

        private void Restrict(int g, double[] r, double[] rc)
        {
            int lo = g * _aggSize, hi = Math.Min(_n, lo + _aggSize);
            double s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0;
            for (int i = lo; i < hi; i++)
            {
                int o = 18 * i;
                double r0 = r[3 * i], r1 = r[3 * i + 1], r2 = r[3 * i + 2];
                s0 += _coarseP[o] * r0 + _coarseP[o + 6] * r1 + _coarseP[o + 12] * r2;
                s1 += _coarseP[o + 1] * r0 + _coarseP[o + 7] * r1 + _coarseP[o + 13] * r2;
                s2 += _coarseP[o + 2] * r0 + _coarseP[o + 8] * r1 + _coarseP[o + 14] * r2;
                s3 += _coarseP[o + 3] * r0 + _coarseP[o + 9] * r1 + _coarseP[o + 15] * r2;
                s4 += _coarseP[o + 4] * r0 + _coarseP[o + 10] * r1 + _coarseP[o + 16] * r2;
                s5 += _coarseP[o + 5] * r0 + _coarseP[o + 11] * r1 + _coarseP[o + 17] * r2;
            }

            rc[6 * g] = s0; rc[6 * g + 1] = s1; rc[6 * g + 2] = s2; rc[6 * g + 3] = s3; rc[6 * g + 4] = s4; rc[6 * g + 5] = s5;
        }

        private void Prolong(int c, double[] yc, double[] z)
        {
            int lo = c * NodeChunk, hi = Math.Min(_n, lo + NodeChunk);
            for (int i = lo; i < hi; i++)
            {
                int g = 6 * (i / _aggSize), o = 18 * i;
                for (int a = 0; a < 3; a++)
                {
                    z[3 * i + a] += _coarseP[o + 6 * a] * yc[g] + _coarseP[o + 6 * a + 1] * yc[g + 1] + _coarseP[o + 6 * a + 2] * yc[g + 2] + _coarseP[o + 6 * a + 3] * yc[g + 3] + _coarseP[o + 6 * a + 4] * yc[g + 4] + _coarseP[o + 6 * a + 5] * yc[g + 5];
                }
            }
        }

        // z = (block Gauss-Seidel + coarse correction)^-1 r, with the dot products r.z and r.r of each chunk left in pRz and pRr.
        private void ApplyPreconditioner(double[] r, double[] z, double[] tmp, double[] wrk, double[] rc, double[] yc, double[] pRz, double[] pRr)
        {
            For(_aggCount, _threads, g => Restrict(g, r, rc));
            CoarseSolve(rc, yc);
            For(_chunks, _threads, c =>
            {
                PreconditionChunk(c, r, tmp, wrk, z);
                Prolong(c, yc, z);
                pRz[c] = DotRange(c, r, z);
                pRr[c] = DotRange(c, r, r);
            });
        }

        public void Solve()
        {
            BuildDiagonal();
            BuildCoarse();
            int nd = _dofs;
            var uc = new double[nd];
            for (int i = 0; i < nd; i++) { if (_fixed[i] != 0) { uc[i] = _presc[i]; } }
            var y = new double[nd];
            For(_chunks, _threads, c => MultiplyChunk(c, uc, y, false));
            // Reduced right-hand side: the loads less the stiffness acting on the prescribed displacements, at the free components.
            var bvec = new double[nd];
            for (int i = 0; i < nd; i++) { bvec[i] = _fixed[i] != 0 ? 0 : _f[i] - y[i]; }
            var x = new double[nd];
            var r = new double[nd]; var z = new double[nd]; var p = new double[nd]; var q = new double[nd]; var tmp = new double[nd]; var wrk = new double[nd];
            var part1 = new double[_chunks]; var part2 = new double[_chunks]; var part3 = new double[_chunks];
            var rc = new double[6 * _aggCount]; var yc = new double[6 * _aggCount];

            double bnorm = Math.Sqrt(Dot(bvec, bvec, part1));
            _iterations = 0;
            _converged = false;
            _residual = 0;
            if (bnorm == 0) { _converged = true; }
            else
            {
                double stop = _tol * bnorm;
                // The iteration's own residual drifts from b - A x by rounding on stiff, long models, so each pass starts from the measured residual and a pass that stops on its own
                // estimate is checked against the measured one; up to eight restarts (Saad, section 6.7, on residual replacement).
                double previous = double.MaxValue;
                for (int pass = 0; pass < 8; pass++)
                {
                    For(_chunks, _threads, c => MultiplyChunk(c, x, q, true));
                    for (int i = 0; i < nd; i++) { r[i] = _fixed[i] != 0 ? 0 : bvec[i] - q[i]; }
                    double measured = Math.Sqrt(Dot(r, r, part1));
                    _residual = measured / bnorm;
                    if (!double.IsFinite(measured)) { break; }
                    if (measured <= stop) { _converged = true; break; }
                    if (_iterations >= _maxIter || measured > 0.5 * previous) { break; } // out of iterations, or at the rounding floor of this model
                    previous = measured;

                    ApplyPreconditioner(r, z, tmp, wrk, rc, yc, part2, part3);
                    Array.Copy(z, p, nd);
                    double rz = Sum(part2);
                    bool broke = false;
                    while (_iterations < _maxIter)
                    {
                        For(_chunks, _threads, c =>
                        {
                            MultiplyChunk(c, p, q, true);
                            part1[c] = DotRange(c, p, q);
                        });
                        double pq = Sum(part1);
                        _iterations++;
                        if (!(pq > 0) || !double.IsFinite(pq)) { broke = true; break; }
                        double alpha = rz / pq;
                        For(_chunks, _threads, c =>
                        {
                            int lo = 3 * c * NodeChunk, hi = Math.Min(nd, 3 * (c + 1) * NodeChunk);
                            for (int i = lo; i < hi; i++) { x[i] += alpha * p[i]; r[i] -= alpha * q[i]; }
                        });
                        ApplyPreconditioner(r, z, tmp, wrk, rc, yc, part2, part3);
                        double rzNew = Sum(part2), rr = Sum(part3);
                        if (!double.IsFinite(rr)) { broke = true; break; }
                        if (Math.Sqrt(rr) <= stop) { break; }
                        double beta = rzNew / rz;
                        rz = rzNew;
                        For(_chunks, _threads, c =>
                        {
                            int lo = 3 * c * NodeChunk, hi = Math.Min(nd, 3 * (c + 1) * NodeChunk);
                            for (int i = lo; i < hi; i++) { p[i] = z[i] + beta * p[i]; }
                        });
                    }

                    if (broke) { break; }
                    if (pass == 7 || _iterations >= _maxIter)
                    {
                        For(_chunks, _threads, c => MultiplyChunk(c, x, q, true));
                        for (int i = 0; i < nd; i++) { r[i] = _fixed[i] != 0 ? 0 : bvec[i] - q[i]; }
                        _residual = Math.Sqrt(Dot(r, r, part1)) / bnorm;
                        _converged = _residual <= _tol;
                        break;
                    }
                }
            }

            _u = new double[nd];
            for (int i = 0; i < nd; i++) { _u[i] = _fixed[i] != 0 ? _presc[i] : x[i]; }
        }

        private double DotRange(int c, double[] a, double[] b)
        {
            int lo = 3 * c * NodeChunk, hi = Math.Min(_dofs, 3 * (c + 1) * NodeChunk);
            double s = 0;
            for (int i = lo; i < hi; i++) { s += a[i] * b[i]; }
            return s;
        }

        private double Dot(double[] a, double[] b, double[] partial)
        {
            For(_chunks, _threads, c => partial[c] = DotRange(c, a, b));
            return Sum(partial);
        }

        private static double Sum(double[] partial)
        {
            double s = 0;
            for (int i = 0; i < partial.Length; i++) { s += partial[i]; }
            return s;
        }

        // Strain at barycentric point l of tetrahedron t from the nodal displacements ue (3 per node).
        private void StrainAt(int t, ReadOnlySpan<double> ue, double l0, double l1, double l2, double l3, Span<double> g, Span<double> eps)
        {
            Gradients(t, l0, l1, l2, l3, g);
            double h00 = 0, h01 = 0, h02 = 0, h10 = 0, h11 = 0, h12 = 0, h20 = 0, h21 = 0, h22 = 0;
            for (int a = 0; a < _nn; a++)
            {
                double ux = ue[3 * a], uy = ue[3 * a + 1], uz = ue[3 * a + 2];
                double gx = g[3 * a], gy = g[3 * a + 1], gz = g[3 * a + 2];
                h00 += ux * gx; h01 += ux * gy; h02 += ux * gz;
                h10 += uy * gx; h11 += uy * gy; h12 += uy * gz;
                h20 += uz * gx; h21 += uz * gy; h22 += uz * gz;
            }

            eps[0] = h00; eps[1] = h11; eps[2] = h22;
            eps[3] = 0.5 * (h01 + h10); eps[4] = 0.5 * (h12 + h21); eps[5] = 0.5 * (h20 + h02);
        }

        private void StressFrom(ReadOnlySpan<double> eps, Span<double> sig)
        {
            double tr = eps[0] + eps[1] + eps[2];
            double l = _lambda * tr, m2 = 2.0 * _mu;
            sig[0] = l + m2 * eps[0]; sig[1] = l + m2 * eps[1]; sig[2] = l + m2 * eps[2];
            sig[3] = m2 * eps[3]; sig[4] = m2 * eps[4]; sig[5] = m2 * eps[5];
        }

        private static double Vm(ReadOnlySpan<double> s)
        {
            double a = s[0] - s[1], b = s[1] - s[2], c = s[2] - s[0];
            return Math.Sqrt(0.5 * (a * a + b * b + c * c) + 3.0 * (s[3] * s[3] + s[4] * s[4] + s[5] * s[5]));
        }

        // Utilisation (stress over strength) at a point: von Mises against the strength, and with a weak axis the tension across the layers and the shear on them against the reduced strength.
        private double Utilisation(ReadOnlySpan<double> s, double vm)
        {
            double u = vm / _strength;
            if (_hasWeak)
            {
                double nx = _unitWeak[0], ny = _unitWeak[1], nz = _unitWeak[2];
                double tx = s[0] * nx + s[3] * ny + s[5] * nz, ty = s[3] * nx + s[1] * ny + s[4] * nz, tz = s[5] * nx + s[4] * ny + s[2] * nz;
                double sn = tx * nx + ty * ny + tz * nz;
                double sx = tx - sn * nx, sy = ty - sn * ny, sz = tz - sn * nz;
                double tau2 = sx * sx + sy * sy + sz * sz;
                double tens = Math.Max(sn, 0.0);
                double inter = Math.Sqrt(tens * tens + 3.0 * tau2) / (_weakRatio * _strength);
                if (inter > u) { u = inter; }
            }

            return u;
        }

        public StaticSolution Results(StaticProblem problem)
        {
            var sol = new StaticSolution();
            var disp = new Vector3D[_v];
            double maxD = 0;
            for (int v = 0; v < _v; v++)
            {
                int nd = _nodeOfVertex[v];
                disp[v] = new Vector3D(_u[3 * nd], _u[3 * nd + 1], _u[3 * nd + 2]);
                double m = Math.Sqrt(_u[3 * nd] * _u[3 * nd] + _u[3 * nd + 1] * _u[3 * nd + 1] + _u[3 * nd + 2] * _u[3 * nd + 2]);
                if (m > maxD) { maxD = m; }
            }

            var strains = new SymmetricTensor[_t]; var stresses = new SymmetricTensor[_t];
            var vmC = new double[_t]; var vmP = new double[_t]; var sf = new double[_t];
            var cornerVm = new double[4 * _t];
            bool hasStrength = _strength > 0;
            int blocks = (_t + 63) / 64;
            For(blocks, _threads, bk =>
            {
                Span<double> ue = stackalloc double[30];
                Span<double> g = stackalloc double[30];
                Span<double> eps = stackalloc double[6];
                Span<double> sig = stackalloc double[6];
                int hi = Math.Min(_t, (bk + 1) * 64);
                for (int t = bk * 64; t < hi; t++)
                {
                    for (int a = 0; a < _nn; a++)
                    {
                        int nd = _nodes[t * _nn + a];
                        ue[3 * a] = _u[3 * nd]; ue[3 * a + 1] = _u[3 * nd + 1]; ue[3 * a + 2] = _u[3 * nd + 2];
                    }

                    StrainAt(t, ue, 0.25, 0.25, 0.25, 0.25, g, eps);
                    StressFrom(eps, sig);
                    strains[t] = new SymmetricTensor(eps[0], eps[1], eps[2], eps[3], eps[4], eps[5]);
                    stresses[t] = new SymmetricTensor(sig[0], sig[1], sig[2], sig[3], sig[4], sig[5]);
                    double vm = Vm(sig);
                    vmC[t] = vm;
                    double peak = vm;
                    double util = hasStrength ? Utilisation(sig, vm) : 0;
                    for (int k = 0; k < 4; k++)
                    {
                        if (_quad)
                        {
                            StrainAt(t, ue, k == 0 ? 1 : 0, k == 1 ? 1 : 0, k == 2 ? 1 : 0, k == 3 ? 1 : 0, g, eps);
                            StressFrom(eps, sig);
                        }

                        double vk = Vm(sig);
                        cornerVm[4 * t + k] = vk;
                        if (vk > peak) { peak = vk; }
                        if (hasStrength) { double uk = Utilisation(sig, vk); if (uk > util) { util = uk; } }
                    }

                    vmP[t] = peak;
                    sf[t] = hasStrength ? (util > 0 ? 1.0 / util : double.PositiveInfinity) : double.NaN;
                }
            });

            var vertexSum = new double[_v]; var vertexW = new double[_v];
            double mass = 0;
            for (int t = 0; t < _t; t++)
            {
                double vol = _geo[13 * t + 12];
                mass += _density * vol;
                for (int k = 0; k < 4; k++)
                {
                    int v = _tets[4 * t + k];
                    vertexSum[v] += vol * cornerVm[4 * t + k]; vertexW[v] += vol;
                }
            }

            var vv = new double[_v];
            for (int v = 0; v < _v; v++) { vv[v] = vertexSum[v] / vertexW[v]; }

            int worst = 0;
            double maxVm = 0;
            for (int t = 0; t < _t; t++)
            {
                if (vmP[t] > maxVm) { maxVm = vmP[t]; }
                if (hasStrength ? sf[t] < sf[worst] : vmP[t] > vmP[worst]) { worst = t; }
            }

            double wx = 0, wy = 0, wz = 0;
            for (int k = 0; k < 4; k++) { int v = _tets[4 * worst + k]; wx += _x[3 * v]; wy += _x[3 * v + 1]; wz += _x[3 * v + 2]; }
            double back = 0.25 / _scale;

            // Reactions at the held degrees of freedom: K u - f.
            var ku = new double[_dofs];
            For(_chunks, _threads, c => MultiplyChunk(c, _u, ku, false));
            double rx = 0, ry = 0, rz = 0;
            for (int i = 0; i < _n; i++)
            {
                if (_fixed[3 * i] != 0) { rx += ku[3 * i] - _f[3 * i]; }
                if (_fixed[3 * i + 1] != 0) { ry += ku[3 * i + 1] - _f[3 * i + 1]; }
                if (_fixed[3 * i + 2] != 0) { rz += ku[3 * i + 2] - _f[3 * i + 2]; }
            }

            sol.Displacements = disp; sol.Strains = strains; sol.Stresses = stresses; sol.VonMises = vmC; sol.PeakVonMises = vmP; sol.SafetyFactors = sf; sol.VertexVonMises = vv;
            sol.WorstElement = worst; sol.WorstLocation = new Vector3D(wx * back, wy * back, wz * back);
            sol.MinSafetyFactor = hasStrength ? sf[worst] : double.NaN;
            sol.MaxVonMises = maxVm; sol.MaxDisplacement = maxD; sol.TotalReaction = new Vector3D(rx, ry, rz);
            sol.Volume = _volume; sol.Mass = mass; sol.Residual = _residual; sol.Iterations = _iterations; sol.Converged = _converged; sol.DegreesOfFreedom = _dofs;
            return sol;
        }
    }
}
