using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>Options for <see cref="ConvexDecomposer.Decompose"/>. The default (all zeros) takes the documented defaults.</summary>
public readonly struct ConvexDecompositionOptions
{
    /// <summary>The most hulls to produce, at least 1; 0 chooses 16.</summary>
    public int MaxHulls { get; init; }

    /// <summary>
    /// A part is split further while its concavity exceeds this, as a fraction of the mesh's bounding-box diagonal. A part's concavity is the largest depth of any of its surface points below the surface of its convex hull
    /// (the Hausdorff-style measure of Lien and Amato, 2004, "Approximate convex decomposition of polyhedra"), so it is a distance, not a volume. 0 chooses 0.01; otherwise it must be finite and positive.
    /// </summary>
    public float MaxConcavity { get; init; }

    /// <summary>The most vertices a hull may have, at least 4; 0 chooses 64. Larger hulls are reduced with <see cref="HullSimplifier"/>.</summary>
    public int MaxVerticesPerHull { get; init; }

    /// <summary>Grid cells along the mesh's longest axis, from 8 to 160; 0 chooses 32. Higher values find smaller features and cost time roughly with the cube.</summary>
    public int Resolution { get; init; }

    /// <summary>Threads used to evaluate candidate cuts; 0 or 1 is single-threaded. The result is the same for every value: candidates fill fixed slots and are compared in index order.</summary>
    public int MaxThreads { get; init; }
}

/// <summary>The result of <see cref="ConvexDecomposer.Decompose"/>: convex hulls whose union approximates the mesh.</summary>
public sealed class ConvexDecomposition
{
    internal ConvexDecomposition(ConvexHull3D[] hulls, float[] concavities, float diagonal, bool thresholdMet)
    {
        Hulls = hulls;
        Concavities = concavities;
        Diagonal = diagonal;
        ThresholdMet = thresholdMet;
    }

    /// <summary>The convex pieces, in the order they were created. All are in the mesh's own coordinates.</summary>
    public IReadOnlyList<ConvexHull3D> Hulls { get; }

    /// <summary>Per hull, the concavity of the part it was made from, as a fraction of <see cref="Diagonal"/>. Simplifying a hull for <see cref="ConvexDecompositionOptions.MaxVerticesPerHull"/> does not change it.</summary>
    public IReadOnlyList<float> Concavities { get; }

    /// <summary>The bounding-box diagonal of the mesh, the unit of concavity.</summary>
    public float Diagonal { get; }

    /// <summary>True when every part is within <see cref="ConvexDecompositionOptions.MaxConcavity"/>; false when the hull limit stopped the splitting first.</summary>
    public bool ThresholdMet { get; }

    /// <summary>The largest entry of <see cref="Concavities"/>.</summary>
    public float MaxConcavity
    {
        get
        {
            float m = 0f;
            foreach (float c in Concavities) { if (c > m) { m = c; } }
            return m;
        }
    }

    /// <summary>The sum of the hulls' volumes. Neighbouring hulls overlap a little, so this is at least the mesh's volume for a good decomposition.</summary>
    public double TotalHullVolume
    {
        get
        {
            double s = 0;
            foreach (var h in Hulls) { s += h.MassProperties.Mass; }
            return s;
        }
    }

    /// <summary>The hulls as compound parts, each at the origin since the hulls share the mesh's frame; pass them to <c>PhysicsWorld3D.AddCompound</c>.</summary>
    /// <returns>One part per hull, in hull order.</returns>
    public CompoundPart[] ToCompoundParts()
    {
        var parts = new CompoundPart[Hulls.Count];
        for (int i = 0; i < parts.Length; i++) { parts[i] = new CompoundPart(ShapeSpec3D.ConvexHull(Hulls[i].Vertices), Vector3F.Zero); }
        return parts;
    }

    /// <summary>The shape to attach to a body: the one hull itself, or a compound of the hulls (each at the origin, since the hulls share the mesh's frame).</summary>
    /// <returns>A shape description ready for <c>PhysicsWorld3D.CreateShape</c>.</returns>
    public ShapeSpec3D ToShapeSpec()
    {
        if (Hulls.Count == 1) { return ShapeSpec3D.ConvexHull(Hulls[0].Vertices); }
        return ShapeSpec3D.Compound(ToCompoundParts());
    }
}

/// <summary>
/// Approximate convex decomposition of a triangle mesh into a few convex hulls, after Mamou and Ghorbel, "A simple and efficient approach for 3D mesh approximate convex decomposition" (ICIP 2009),
/// with the concavity measure of Lien and Amato (2004) and the worst-part-first cutting of Wei et al., "Approximate Convex Decomposition for 3D Meshes with Collision-Aware Concavity and Tree Search" (SIGGRAPH 2022).
/// The mesh is voxelized, the part with the largest concavity is cut by the axis-aligned plane that minimizes the children's concavity plus a balance term, and this repeats until every part is within the threshold or the hull limit is reached.
/// </summary>
public static class ConvexDecomposer
{
    private const double BalanceWeight = 0.02;
    private const int PlanesPerAxis = 12;

    /// <summary>Decomposes a triangle mesh. The mesh need not be closed or consistently wound; it is cleaned first with <see cref="MeshCleaner"/>'s defaults.</summary>
    /// <param name="vertices">Mesh vertices, finite, magnitude at most 2^21.</param>
    /// <param name="indices">Triangle corner indices, three per triangle.</param>
    /// <param name="options">Limits; the default uses 16 hulls, concavity 0.01, 64 vertices per hull and 32 cells.</param>
    /// <returns>The hulls. Identical input and options give identical output on every platform and at every thread count.</returns>
    /// <exception cref="GeometryToolException">The mesh is empty, has non-finite or out-of-range vertices or bad indices, has no area, or an option is invalid.</exception>
    public static ConvexDecomposition Decompose(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices, ConvexDecompositionOptions options = default)
    {
        MeshInput.Check(vertices, indices);
        int maxHulls = options.MaxHulls == 0 ? 16 : options.MaxHulls;
        float maxConc = options.MaxConcavity == 0f ? 0.01f : options.MaxConcavity;
        int maxVerts = options.MaxVerticesPerHull == 0 ? 64 : options.MaxVerticesPerHull;
        int res = options.Resolution == 0 ? 32 : options.Resolution;
        int threads = Math.Max(1, options.MaxThreads);
        if (maxHulls < 1 || maxHulls > 4096) { throw Bad("MaxHulls", MeshInput.Inv(maxHulls), "from 1 to 4096", nameof(options)); }
        if (!(maxConc > 0f) || float.IsInfinity(maxConc)) { throw Bad("MaxConcavity", maxConc.ToString(System.Globalization.CultureInfo.InvariantCulture), "finite and positive", nameof(options)); }
        if (maxVerts < 4) { throw Bad("MaxVerticesPerHull", MeshInput.Inv(maxVerts), "at least 4", nameof(options)); }
        if (res < 8 || res > 160) { throw Bad("Resolution", MeshInput.Inv(res), "from 8 to 160", nameof(options)); }
        if (threads > 256) { throw Bad("MaxThreads", MeshInput.Inv(threads), "at most 256", nameof(options)); }

        var cleaned = MeshCleaner.Clean(vertices, indices);
        if (cleaned.Indices.Length == 0)
        {
            throw new GeometryToolException(GeometryToolError.Degenerate, "every triangle of the mesh is degenerate (zero area)", nameof(indices));
        }

        double ex = 0, ey = 0, ez = 0;
        {
            double x0 = double.MaxValue, x1 = double.MinValue, y0 = double.MaxValue, y1 = double.MinValue, z0 = double.MaxValue, z1 = double.MinValue;
            foreach (var v in cleaned.Vertices)
            {
                x0 = Math.Min(x0, v.X); x1 = Math.Max(x1, v.X);
                y0 = Math.Min(y0, v.Y); y1 = Math.Max(y1, v.Y);
                z0 = Math.Min(z0, v.Z); z1 = Math.Max(z1, v.Z);
            }

            ex = x1 - x0; ey = y1 - y0; ez = z1 - z0;
        }

        if (Math.Min(ex, Math.Min(ey, ez)) <= 1e-5 * Math.Max(ex, Math.Max(ey, ez)))
        {
            throw new GeometryToolException(GeometryToolError.Degenerate, "the mesh is flat: its extents are " + ex.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + " x " + ey.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + " x " + ez.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ", so it has no volume to decompose", nameof(vertices));
        }

        var model = VoxelModel.Build(cleaned.Vertices, cleaned.Indices, res);
        double diag = Diagonal(cleaned.Vertices);
        var run = new Run(model, cleaned.Vertices, diag, maxVerts, threads);
        return run.Execute(maxHulls, maxConc);
    }

    private static GeometryToolException Bad(string name, string value, string expected, string paramName) =>
        new(GeometryToolError.InvalidOptions, name + " = " + value + " must be " + expected, paramName);

    private static double Diagonal(Vector3F[] v)
    {
        double x0 = v[0].X, x1 = x0, y0 = v[0].Y, y1 = y0, z0 = v[0].Z, z1 = z0;
        for (int i = 1; i < v.Length; i++)
        {
            x0 = Math.Min(x0, v[i].X); x1 = Math.Max(x1, v[i].X);
            y0 = Math.Min(y0, v[i].Y); y1 = Math.Max(y1, v[i].Y);
            z0 = Math.Min(z0, v[i].Z); z1 = Math.Max(z1, v[i].Z);
        }

        return Math.Sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
    }

    private sealed class Part
    {
        public readonly double[] Lo = new double[3];
        public readonly double[] Hi = new double[3];
        public ConvexHull3D? Hull;
        public double Concavity;
        public bool Alive = true;
        public bool Splittable = true;
    }

    private struct Candidate
    {
        public bool Valid;
        public double Cost;
        public int Axis;
        public double Plane;
        public ConvexHull3D? HullA, HullB;
        public double ConcA, ConcB;
        public int CountA, CountB;
    }

    private sealed class Run
    {
        // Hulls used to measure concavity keep this many vertices at most: a hull of a curved part has as many vertices as there are surface points otherwise.
        private const int MeasureHullVertices = 128;

        private readonly VoxelModel _m;
        private readonly double _diag;
        private readonly int _maxVerts;
        private readonly int _threads;
        private readonly double[][] _axisCoords;
        private readonly double[] _meshLo = new double[3], _meshHi = new double[3];
        private readonly List<Part> _parts = new();

        public Run(VoxelModel m, Vector3F[] meshVertices, double diag, int maxVerts, int threads)
        {
            _m = m;
            _diag = diag;
            _maxVerts = maxVerts;
            _threads = threads;
            _axisCoords = new double[3][];
            for (int a = 0; a < 3; a++)
            {
                var c = new double[meshVertices.Length];
                for (int i = 0; i < c.Length; i++) { c[i] = a == 0 ? meshVertices[i].X : a == 1 ? meshVertices[i].Y : meshVertices[i].Z; }
                Array.Sort(c);
                var u = new List<double>();
                foreach (double d in c) { if (u.Count == 0 || d != u[^1]) { u.Add(d); } }
                _axisCoords[a] = u.ToArray();
                _meshLo[a] = u[0];
                _meshHi[a] = u[^1];
            }
        }

        public ConvexDecomposition Execute(int maxHulls, double maxConc)
        {
            var root = new Part();
            for (int a = 0; a < 3; a++) { root.Lo[a] = _meshLo[a]; root.Hi[a] = _meshHi[a]; }
            Evaluate(root.Lo, root.Hi, out root.Hull, out root.Concavity, out _);
            _parts.Add(root);
            int alive = 1;
            while (alive < maxHulls)
            {
                int pick = -1;
                for (int i = 0; i < _parts.Count; i++)
                {
                    var p = _parts[i];
                    if (p.Alive && p.Splittable && p.Concavity > maxConc && (pick < 0 || p.Concavity > _parts[pick].Concavity)) { pick = i; }
                }

                if (pick < 0) { break; }
                if (!Split(_parts[pick])) { _parts[pick].Splittable = false; continue; }
                alive++;
            }

            var hulls = new List<ConvexHull3D>();
            var conc = new List<float>();
            bool met = true;
            foreach (var p in _parts)
            {
                if (!p.Alive) { continue; }
                var hull = p.Hull ?? BoxHull(p);
                if (hull.Vertices.Length > _maxVerts)
                {
                    hull = HullSimplifier.Simplify(hull, new HullSimplifyOptions { MaxVertices = _maxVerts }).Hull;
                }

                hulls.Add(hull);
                conc.Add((float)p.Concavity);
                if (p.Concavity > maxConc) { met = false; }
            }

            return new ConvexDecomposition(hulls.ToArray(), conc.ToArray(), (float)_diag, met);
        }

        private bool Split(Part part)
        {
            var cands = new List<Candidate>();
            double margin = 0.25 * _m.H;
            for (int axis = 0; axis < 3; axis++)
            {
                double lo = part.Lo[axis], hi = part.Hi[axis];
                if (hi - lo <= 2 * margin) { continue; }
                var planes = new List<double>();
                for (int i = 0; i < UniformPlanes; i++) { planes.Add(lo + (hi - lo) * (i + 1) / (UniformPlanes + 1)); }
                // Mesh vertex coordinates are where flat features of the shape begin and end, so cutting there separates blocky shapes exactly.
                var coords = _axisCoords[axis];
                var inside = new List<double>();
                foreach (double c in coords) { if (c > lo + margin && c < hi - margin) { inside.Add(c); } }
                if (inside.Count <= VertexPlanes) { planes.AddRange(inside); }
                else { for (int i = 0; i < VertexPlanes; i++) { planes.Add(inside[(int)((long)i * (inside.Count - 1) / (VertexPlanes - 1))]); } }
                planes.Sort();
                double last = double.NegativeInfinity;
                foreach (double p in planes)
                {
                    if (p - last < margin) { continue; }
                    last = p;
                    cands.Add(new Candidate { Axis = axis, Plane = p });
                }
            }

            if (cands.Count == 0) { return false; }
            var arr = cands.ToArray();
            void Work(int i)
            {
                ref Candidate c = ref arr[i];
                double[] hiA = { part.Hi[0], part.Hi[1], part.Hi[2] }, loB = { part.Lo[0], part.Lo[1], part.Lo[2] };
                hiA[c.Axis] = c.Plane;
                loB[c.Axis] = c.Plane;
                Evaluate(part.Lo, hiA, out c.HullA, out c.ConcA, out c.CountA);
                Evaluate(loB, part.Hi, out c.HullB, out c.ConcB, out c.CountB);
                c.Valid = c.CountA > 0 && c.CountB > 0;
                double balance = Math.Abs((double)c.CountA - c.CountB) / Math.Max(1, c.CountA + c.CountB);
                // The worse child counts fully and the better one half: a cut that peels a convex sliver off a concave part leaves it as concave as before, and must not beat a cut through the middle.
                c.Cost = Math.Max(c.ConcA, c.ConcB) + 0.5 * Math.Min(c.ConcA, c.ConcB) + BalanceWeight * balance;
            }

            if (_threads > 1)
            {
                Parallel.For(0, arr.Length, new ParallelOptions { MaxDegreeOfParallelism = _threads }, Work);
            }
            else
            {
                for (int i = 0; i < arr.Length; i++) { Work(i); }
            }

            int best = -1;
            for (int i = 0; i < arr.Length; i++)
            {
                if (arr[i].Valid && (best < 0 || arr[i].Cost < arr[best].Cost)) { best = i; }
            }

            if (best < 0) { return false; }
            ref Candidate w = ref arr[best];
            var pa = new Part { Hull = w.HullA, Concavity = w.ConcA };
            var pb = new Part { Hull = w.HullB, Concavity = w.ConcB };
            for (int a = 0; a < 3; a++)
            {
                pa.Lo[a] = pb.Lo[a] = part.Lo[a];
                pa.Hi[a] = pb.Hi[a] = part.Hi[a];
            }

            pa.Hi[w.Axis] = w.Plane;
            pb.Lo[w.Axis] = w.Plane;
            part.Alive = false;
            _parts.Add(pa);
            _parts.Add(pb);
            return true;
        }

        private const int UniformPlanes = 6;
        private const int VertexPlanes = 6;

        // The part is the mesh's solid inside an axis-aligned box. Its surface points are the mesh's own triangle pieces clipped to the box, plus the corners of the cross-sections of interior voxels where
        // the box cuts them (the flat caps a cut leaves). The count is the number of solid voxels whose centre lies in the box, for balancing.
        private void Evaluate(double[] lo, double[] hi, out ConvexHull3D? hull, out double concavity, out int count)
        {
            var pts = new List<Vector3F>();
            var nrm = new List<Vector3F>();
            var caps = new List<Vector3F>();
            count = 0;
            double h = _m.H;
            int i0 = Cell((lo[0] - _m.Ox) / h, _m.Nx), i1 = Cell((hi[0] - _m.Ox) / h, _m.Nx);
            int j0 = Cell((lo[1] - _m.Oy) / h, _m.Ny), j1 = Cell((hi[1] - _m.Oy) / h, _m.Ny);
            int k0 = Cell((lo[2] - _m.Oz) / h, _m.Nz), k1 = Cell((hi[2] - _m.Oz) / h, _m.Nz);
            var bufA = new double[24 * 3];
            var bufB = new double[24 * 3];
            for (int k = k0; k <= k1; k++)
            {
                double vz = _m.Oz + k * h;
                for (int j = j0; j <= j1; j++)
                {
                    double vy = _m.Oy + j * h;
                    for (int i = i0; i <= i1; i++)
                    {
                        int idx = _m.Index(i, j, k);
                        byte sol = _m.Solid[idx];
                        if (sol == VoxelModel.Empty) { continue; }
                        double vx = _m.Ox + i * h;
                        double cx = vx + 0.5 * h, cy = vy + 0.5 * h, cz = vz + 0.5 * h;
                        if (cx >= lo[0] && cx < hi[0] && cy >= lo[1] && cy < hi[1] && cz >= lo[2] && cz < hi[2]) { count++; }
                        bool whole = vx >= lo[0] && vx + h <= hi[0] && vy >= lo[1] && vy + h <= hi[1] && vz >= lo[2] && vz + h <= hi[2];
                        if (sol == VoxelModel.Surface)
                        {
                            for (int p = _m.PolyOfVoxel[idx]; p < _m.PolyOfVoxel[idx + 1]; p++)
                            {
                                int s = _m.PolyStart[p], e = _m.PolyStart[p + 1];
                                Vector3F normal = _m.PolyNormal[p];
                                if (whole)
                                {
                                    for (int q = s; q < e; q++) { pts.Add(_m.Points[q]); nrm.Add(normal); }
                                    continue;
                                }

                                int n = e - s;
                                for (int q = 0; q < n; q++)
                                {
                                    Vector3F v = _m.Points[s + q];
                                    bufA[3 * q] = v.X; bufA[3 * q + 1] = v.Y; bufA[3 * q + 2] = v.Z;
                                }

                                n = VoxelModel.ClipPolygon(bufA, n, lo[0], lo[1], lo[2], hi[0], hi[1], hi[2], bufB, out double[] res);
                                if (n == 0) { continue; }
                                // A polygon that only touches the box along an edge or at a point belongs to the neighbouring part.
                                if (IsSliver(res, n, h)) { continue; }

                                if (LiesOnFarSide(res, n, lo, hi, normal)) { continue; }
                                for (int q = 0; q < n; q++) { pts.Add(new Vector3F((float)res[3 * q], (float)res[3 * q + 1], (float)res[3 * q + 2])); nrm.Add(normal); }
                            }

                            continue;
                        }

                        if (whole) { continue; }
                        // An interior voxel the box cuts: add the corners of its cross-section on each face plane of the box that passes through it.
                        double[] vlo = { vx, vy, vz };
                        for (int axis = 0; axis < 3; axis++)
                        {
                            for (int side = 0; side < 2; side++)
                            {
                                double plane = side == 0 ? lo[axis] : hi[axis];
                                if (plane < vlo[axis] || plane > vlo[axis] + h) { continue; }
                                int u = (axis + 1) % 3, v = (axis + 2) % 3;
                                double u0 = Math.Max(vlo[u], lo[u]), u1 = Math.Min(vlo[u] + h, hi[u]);
                                double v0 = Math.Max(vlo[v], lo[v]), v1 = Math.Min(vlo[v] + h, hi[v]);
                                if (u0 > u1 || v0 > v1) { continue; }
                                AddCap(caps, axis, plane, u0, u1, v0, v1);
                            }
                        }
                    }
                }
            }

            int surfaceCount = pts.Count;
            if (caps.Count > 0)
            {
                caps.Sort((x, y) =>
                {
                    int c = x.X.CompareTo(y.X);
                    if (c != 0) { return c; }
                    c = x.Y.CompareTo(y.Y);
                    return c != 0 ? c : x.Z.CompareTo(y.Z);
                });
                for (int q = 0; q < caps.Count; q++)
                {
                    if (q == 0 || caps[q] != caps[q - 1]) { pts.Add(caps[q]); }
                }
            }

            hull = null;
            concavity = 0;
            if (pts.Count < 4) { return; }
            var arr = pts.ToArray();
            try { hull = ConvexHull3D.Build(arr, new HullBuildOptions { MaxVertices = MeasureHullVertices }); }
            catch (HullBuildException) { return; }
            concavity = Concavity(hull, arr, nrm, surfaceCount) / _diag;
        }

        private static int Cell(double v, int n)
        {
            int c = (int)Math.Floor(v);
            return c < 0 ? 0 : (c >= n ? n - 1 : c);
        }

        private static void AddCap(List<Vector3F> caps, int axis, double plane, double u0, double u1, double v0, double v1)
        {
            for (int a = 0; a < 2; a++)
            {
                for (int b = 0; b < 2; b++)
                {
                    double u = a == 0 ? u0 : u1, v = b == 0 ? v0 : v1;
                    double x = axis == 0 ? plane : axis == 1 ? v : u;
                    double y = axis == 1 ? plane : axis == 2 ? v : u;
                    double z = axis == 2 ? plane : axis == 0 ? v : u;
                    caps.Add(new Vector3F((float)x, (float)y, (float)z));
                }
            }
        }

        private static bool IsSliver(double[] p, int n, double h)
        {
            double sx = 0, sy = 0, sz = 0;
            for (int q = 1; q + 1 < n; q++)
            {
                double ax = p[3 * q] - p[0], ay = p[3 * q + 1] - p[1], az = p[3 * q + 2] - p[2];
                double bx = p[3 * q + 3] - p[0], by = p[3 * q + 4] - p[1], bz = p[3 * q + 5] - p[2];
                sx += ay * bz - az * by; sy += az * bx - ax * bz; sz += ax * by - ay * bx;
            }

            return sx * sx + sy * sy + sz * sz <= 1e-18 * h * h * h * h;
        }

        // A polygon lying exactly in a face plane of the box belongs to the part only if the solid is on the box's side of it (the polygon's outward normal points out of the box);
        // otherwise it is the wall of the neighbouring part and would add a sheet of air to this one's hull.
        private static bool LiesOnFarSide(double[] pts, int n, double[] lo, double[] hi, Vector3F normal)
        {
            for (int axis = 0; axis < 3; axis++)
            {
                float na = axis == 0 ? normal.X : axis == 1 ? normal.Y : normal.Z;
                bool onLo = true, onHi = true;
                for (int q = 0; q < n; q++)
                {
                    double c = pts[3 * q + axis];
                    if (c != lo[axis]) { onLo = false; }
                    if (c != hi[axis]) { onHi = false; }
                }

                if (onLo && na > 0f) { return true; }
                if (onHi && na < 0f) { return true; }
            }

            return false;
        }

        // The concavity of a surface point is how far it lies behind the hull along its outward normal: the distance the normal ray travels inside the hull before leaving it (Mamou and Ghorbel 2009;
        // Lien and Amato 2004). That, unlike the distance to the nearest face, does not shrink for a thin part. A point on the hull's surface gives 0. The face that decided the previous point is tried first,
        // because neighbouring points are decided by the same face.
        private static double Concavity(ConvexHull3D hull, Vector3F[] pts, List<Vector3F> normals, int count)
        {
            var faces = hull.Faces;
            double worst = 0;
            int lastFace = 0;
            for (int i = 0; i < count; i++)
            {
                Vector3F p = pts[i];
                Vector3F nm = normals[i];
                double best = double.PositiveInfinity;
                int deciding = lastFace;
                // Try the previous deciding face first, then the rest.
                for (int step = -1; step < faces.Length; step++)
                {
                    int f = step < 0 ? lastFace : step;
                    Vector3F fn = faces[f].Normal;
                    double dn = (double)fn.X * nm.X + (double)fn.Y * nm.Y + (double)fn.Z * nm.Z;
                    if (dn <= 1e-9) { continue; }
                    double t = (faces[f].Offset - ((double)fn.X * p.X + (double)fn.Y * p.Y + (double)fn.Z * p.Z)) / dn;
                    if (t < best)
                    {
                        best = t;
                        deciding = f;
                        if (best <= worst) { break; }
                    }
                }

                if (best <= worst) { lastFace = deciding; continue; }
                lastFace = deciding;
                if (best < double.PositiveInfinity) { worst = best; }
            }

            return worst;
        }

        private static ConvexHull3D BoxHull(Part part)
        {
            // A part whose surface points were flat (a sliver): use the corners of its box so it still has volume.
            var pts = new Vector3F[8];
            for (int c = 0; c < 8; c++)
            {
                pts[c] = new Vector3F((float)((c & 1) == 0 ? part.Lo[0] : part.Hi[0]), (float)((c & 2) == 0 ? part.Lo[1] : part.Hi[1]), (float)((c & 4) == 0 ? part.Lo[2] : part.Hi[2]));
            }

            return ConvexHull3D.Build(pts);
        }
    }
}
