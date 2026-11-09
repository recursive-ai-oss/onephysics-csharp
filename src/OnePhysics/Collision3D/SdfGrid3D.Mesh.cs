using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class SdfGrid3D
{
    /// <summary>
    /// Builds the signed distance field of a closed triangle mesh. The distance at each sample is the exact distance to the nearest triangle; the sign comes from the angle-weighted
    /// pseudo-normal of the nearest feature (Bærentzen and Aanæs, "Signed distance computation using the angle weighted pseudonormal", 2005), which is correct for any closed,
    /// consistently oriented, non-self-intersecting surface. Triangles wind counter-clockwise seen from outside. The grid covers the mesh bounds plus <paramref name="padding"/> cells on every side.
    /// Throws SdfBuildException for an empty mesh, non-finite or huge coordinates, an index outside the vertex array, a zero-area triangle, a surface that is not closed and consistently oriented (weld duplicate vertices first), or a grid larger than the limits.
    /// </summary>
    public static SdfGrid3D FromMesh(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices, float cellSize, int padding = 2)
    {
        if (!float.IsFinite(cellSize) || cellSize < 1e-4f) throw new SdfBuildException(SdfBuildError.OutOfRange, "Cell size must be finite and at least 1e-4.", nameof(cellSize));
        if (padding < 1 || padding > 64) throw new SdfBuildException(SdfBuildError.OutOfRange, "Padding must be 1..64 cells.", nameof(padding));
        if (vertices.Length < 4 || indices.Length < 12) throw new SdfBuildException(SdfBuildError.Empty, "A closed mesh needs at least 4 vertices and 4 triangles.");
        if (vertices.Length > (1 << 20) || indices.Length > 3 * (1 << 20)) throw new SdfBuildException(SdfBuildError.OutOfRange, "Mesh has too many vertices or triangles (limits 2^20 each).");
        if (indices.Length % 3 != 0) throw new SdfBuildException(SdfBuildError.Degenerate, "Index count " + indices.Length + " is not a multiple of three.", nameof(indices));
        var mesh = new ClosedMesh(vertices, indices);
        double h = cellSize, pad = padding * h;
        double x0 = mesh.Min[0] - pad, y0 = mesh.Min[1] - pad, z0 = mesh.Min[2] - pad;
        int nx = Count((float)(mesh.Max[0] + pad - x0), cellSize), ny = Count((float)(mesh.Max[1] + pad - y0), cellSize), nz = Count((float)(mesh.Max[2] + pad - z0), cellSize);
        var origin = new Vector3F((float)x0, (float)y0, (float)z0);
        Validate(nx, ny, nz, cellSize, origin);
        var samples = new float[nx * ny * nz]; int i = 0;
        for (int z = 0; z < nz; z++) for (int y = 0; y < ny; y++) for (int x = 0; x < nx; x++)
            samples[i++] = mesh.SignedDistance(origin.X + x * cellSize, origin.Y + y * cellSize, origin.Z + z * cellSize);
        return new SdfGrid3D(nx, ny, nz, samples, cellSize, origin);
    }

    /// <summary>The exact signed distance from a point to a closed mesh, with the same rules as <see cref="FromMesh"/>; used to check a grid against the surface it came from. Slow (a bucket search per call).</summary>
    public static float ExactMeshDistance(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices, Vector3F point)
    {
        CheckPoint(point);
        return new ClosedMesh(vertices, indices).SignedDistance(point.X, point.Y, point.Z);
    }

    private sealed class ClosedMesh
    {
        private readonly double[] _v; private readonly int[] _t; private readonly int _triangles;
        private readonly double[] _faceNormal, _vertexNormal; private readonly double[] _edgeNormal; // per triangle edge slot: pseudo-normal of the undirected edge
        internal readonly double[] Min = new double[3], Max = new double[3];
        private readonly int[] _bucketStart, _bucketTri; private readonly int _gx, _gy, _gz; private readonly double _bx, _by, _bz, _cell; private readonly int[] _stamp; private int _query;

        internal ClosedMesh(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices)
        {
            int nv = vertices.Length; _triangles = indices.Length / 3; _t = indices.ToArray(); _v = new double[3 * nv];
            if (nv < 4 || _triangles < 4) throw new SdfBuildException(SdfBuildError.Empty, "A closed mesh needs at least 4 vertices and 4 triangles.");
            Min[0] = Min[1] = Min[2] = double.MaxValue; Max[0] = Max[1] = Max[2] = double.MinValue;
            for (int i = 0; i < nv; i++)
            {
                var p = vertices[i];
                if (!float.IsFinite(p.X) || !float.IsFinite(p.Y) || !float.IsFinite(p.Z)) throw new SdfBuildException(SdfBuildError.NonFinite, "Vertex " + i + " is not finite.");
                if (Math.Abs(p.X) > 1048576 || Math.Abs(p.Y) > 1048576 || Math.Abs(p.Z) > 1048576) throw new SdfBuildException(SdfBuildError.OutOfRange, "Vertex " + i + " is beyond 2^20 metres.");
                _v[3 * i] = p.X; _v[3 * i + 1] = p.Y; _v[3 * i + 2] = p.Z;
                for (int a = 0; a < 3; a++) { if (_v[3 * i + a] < Min[a]) Min[a] = _v[3 * i + a]; if (_v[3 * i + a] > Max[a]) Max[a] = _v[3 * i + a]; }
            }
            double scale = Math.Max(Max[0] - Min[0], Math.Max(Max[1] - Min[1], Max[2] - Min[2]));
            _faceNormal = new double[3 * _triangles]; _vertexNormal = new double[3 * nv]; _edgeNormal = new double[3 * _triangles * 3];
            var directed = new Dictionary<long, int>(); var edgeIndex = new Dictionary<long, int>(); var edgeSum = new List<double>();
            for (int f = 0; f < _triangles; f++)
            {
                int a = _t[3 * f], b = _t[3 * f + 1], c = _t[3 * f + 2];
                if ((uint)a >= (uint)nv || (uint)b >= (uint)nv || (uint)c >= (uint)nv) throw new SdfBuildException(SdfBuildError.Degenerate, "Triangle " + f + " names a vertex outside 0.." + (nv - 1) + ".", nameof(indices));
                double ux = V(b, 0) - V(a, 0), uy = V(b, 1) - V(a, 1), uz = V(b, 2) - V(a, 2), wx = V(c, 0) - V(a, 0), wy = V(c, 1) - V(a, 1), wz = V(c, 2) - V(a, 2);
                double nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx, len = Math.Sqrt(nx * nx + ny * ny + nz * nz);
                if (!(len > 1e-9 * scale * scale)) throw new SdfBuildException(SdfBuildError.Degenerate, "Triangle " + f + " has no area (or is a sliver below 1e-9 of the mesh size squared).", nameof(indices));
                _faceNormal[3 * f] = nx / len; _faceNormal[3 * f + 1] = ny / len; _faceNormal[3 * f + 2] = nz / len;
                for (int e = 0; e < 3; e++)
                {
                    int p = _t[3 * f + e], q = _t[3 * f + (e + 1) % 3];
                    if (p == q) throw new SdfBuildException(SdfBuildError.Degenerate, "Triangle " + f + " repeats a vertex.", nameof(indices));
                    long dk = ((long)p << 32) | (uint)q;
                    if (!directed.TryAdd(dk, f)) throw new SdfBuildException(SdfBuildError.NotClosed, "Edge " + p + "->" + q + " is used by two triangles in the same direction (inconsistent orientation or a non-manifold edge).", nameof(indices));
                    long uk = ((long)Math.Min(p, q) << 32) | (uint)Math.Max(p, q);
                    if (!edgeIndex.TryGetValue(uk, out int slot)) { slot = edgeSum.Count / 3; edgeIndex.Add(uk, slot); edgeSum.Add(0); edgeSum.Add(0); edgeSum.Add(0); }
                    edgeSum[3 * slot] += _faceNormal[3 * f]; edgeSum[3 * slot + 1] += _faceNormal[3 * f + 1]; edgeSum[3 * slot + 2] += _faceNormal[3 * f + 2];
                    _edgeNormal[3 * (3 * f + e)] = slot; // temporarily the slot; filled below
                }
                // Angle-weighted vertex normals: the angle of the triangle at each corner.
                for (int e = 0; e < 3; e++)
                {
                    int p = _t[3 * f + e], q = _t[3 * f + (e + 1) % 3], r = _t[3 * f + (e + 2) % 3];
                    double ax = V(q, 0) - V(p, 0), ay = V(q, 1) - V(p, 1), az = V(q, 2) - V(p, 2), bx = V(r, 0) - V(p, 0), by = V(r, 1) - V(p, 1), bz = V(r, 2) - V(p, 2);
                    double cx = ay * bz - az * by, cy = az * bx - ax * bz, cz = ax * by - ay * bx;
                    double angle = Math64.Atan2(Math.Sqrt(cx * cx + cy * cy + cz * cz), ax * bx + ay * by + az * bz);
                    _vertexNormal[3 * p] += angle * _faceNormal[3 * f]; _vertexNormal[3 * p + 1] += angle * _faceNormal[3 * f + 1]; _vertexNormal[3 * p + 2] += angle * _faceNormal[3 * f + 2];
                }
            }
            foreach (var pair in directed) { long rk = ((long)(uint)pair.Key << 32) | (uint)(pair.Key >> 32); if (!directed.ContainsKey(rk)) throw new SdfBuildException(SdfBuildError.NotClosed, "Edge " + (pair.Key >> 32) + "->" + (uint)pair.Key + " has no triangle walking it the other way: the surface is not closed (weld duplicate vertices first).", nameof(indices)); }
            for (int f = 0; f < _triangles; f++) for (int e = 0; e < 3; e++)
            {
                int slot = (int)_edgeNormal[3 * (3 * f + e)];
                _edgeNormal[3 * (3 * f + e)] = edgeSum[3 * slot]; _edgeNormal[3 * (3 * f + e) + 1] = edgeSum[3 * slot + 1]; _edgeNormal[3 * (3 * f + e) + 2] = edgeSum[3 * slot + 2];
            }
            // Bucket grid over the triangles: about one triangle per bucket.
            int target = 1; while (target < 64 && (long)(target + 1) * (target + 1) * (target + 1) <= _triangles / 2) target++;   // integer cube root of triangles/2, capped at 64
            _cell = Math.Max(scale / target, 1e-12); _bx = Min[0]; _by = Min[1]; _bz = Min[2];
            _gx = Math.Max(1, (int)Math.Ceiling((Max[0] - Min[0]) / _cell)); _gy = Math.Max(1, (int)Math.Ceiling((Max[1] - Min[1]) / _cell)); _gz = Math.Max(1, (int)Math.Ceiling((Max[2] - Min[2]) / _cell));
            var count = new int[_gx * _gy * _gz + 1];
            for (int f = 0; f < _triangles; f++)
            {
                Box(f, out int xa, out int ya, out int za, out int xb, out int yb, out int zb);
                for (int z = za; z <= zb; z++) for (int y = ya; y <= yb; y++) for (int x = xa; x <= xb; x++) count[(z * _gy + y) * _gx + x + 1]++;
            }
            for (int i = 1; i < count.Length; i++) count[i] += count[i - 1];
            _bucketTri = new int[count[^1]]; var fill = new int[count.Length - 1];
            for (int i = 0; i < fill.Length; i++) fill[i] = count[i];
            for (int f = 0; f < _triangles; f++)
            {
                Box(f, out int xa, out int ya, out int za, out int xb, out int yb, out int zb);
                for (int z = za; z <= zb; z++) for (int y = ya; y <= yb; y++) for (int x = xa; x <= xb; x++) _bucketTri[fill[(z * _gy + y) * _gx + x]++] = f;
            }
            _bucketStart = count; _stamp = new int[_triangles];
        }
        private double V(int i, int a) => _v[3 * i + a];

        private void Box(int f, out int xa, out int ya, out int za, out int xb, out int yb, out int zb)
        {
            double lx = double.MaxValue, ly = double.MaxValue, lz = double.MaxValue, hx = double.MinValue, hy = double.MinValue, hz = double.MinValue;
            for (int e = 0; e < 3; e++) { int p = _t[3 * f + e]; lx = Math.Min(lx, V(p, 0)); hx = Math.Max(hx, V(p, 0)); ly = Math.Min(ly, V(p, 1)); hy = Math.Max(hy, V(p, 1)); lz = Math.Min(lz, V(p, 2)); hz = Math.Max(hz, V(p, 2)); }
            xa = Cl((lx - _bx) / _cell, _gx); xb = Cl((hx - _bx) / _cell, _gx); ya = Cl((ly - _by) / _cell, _gy); yb = Cl((hy - _by) / _cell, _gy); za = Cl((lz - _bz) / _cell, _gz); zb = Cl((hz - _bz) / _cell, _gz);
        }
        private static int Cl(double q, int n) => (int)Math.Clamp(Math.Floor(q), 0, n - 1);

        internal float SignedDistance(double px, double py, double pz)
        {
            _query++;
            int cx = Cl((px - _bx) / _cell, _gx), cy = Cl((py - _by) / _cell, _gy), cz = Cl((pz - _bz) / _cell, _gz);
            double best = double.MaxValue; int bestTri = -1, bestFeature = 0; double bqx = 0, bqy = 0, bqz = 0;
            // Distance from the point to the box of the bucket grid, so rings entirely behind the best hit are skipped.
            int maxRing = Math.Max(_gx, Math.Max(_gy, _gz));
            for (int ring = 0; ring <= maxRing; ring++)
            {
                if (bestTri >= 0)
                {
                    // Every unvisited bucket is at least (ring-1) cells plus the offset inside the current bucket away; use the conservative (ring - 1) cells.
                    double reach = (ring - 1) * _cell; if (reach > 0 && best <= reach * reach) break;
                }
                for (int z = cz - ring; z <= cz + ring; z++) for (int y = cy - ring; y <= cy + ring; y++) for (int x = cx - ring; x <= cx + ring; x++)
                {
                    if (Math.Max(Math.Abs(x - cx), Math.Max(Math.Abs(y - cy), Math.Abs(z - cz))) != ring) continue;
                    if ((uint)x >= (uint)_gx || (uint)y >= (uint)_gy || (uint)z >= (uint)_gz) continue;
                    int cell = (z * _gy + y) * _gx + x;
                    for (int k = _bucketStart[cell]; k < _bucketStart[cell + 1]; k++)
                    {
                        int f = _bucketTri[k]; if (_stamp[f] == _query) continue; _stamp[f] = _query;
                        ClosestOnTriangle(f, px, py, pz, out double qx, out double qy, out double qz, out int feature);
                        double dx = px - qx, dy = py - qy, dz = pz - qz, d2 = dx * dx + dy * dy + dz * dz;
                        if (d2 < best || (d2 == best && f < bestTri)) { best = d2; bestTri = f; bestFeature = feature; bqx = qx; bqy = qy; bqz = qz; }
                    }
                }
            }
            double dist = Math.Sqrt(best);
            // Sign by the pseudo-normal of the feature the closest point lies on.
            int a = _t[3 * bestTri], b = _t[3 * bestTri + 1], c = _t[3 * bestTri + 2]; double nx, ny, nz;
            switch (bestFeature)
            {
                case 1: nx = _vertexNormal[3 * a]; ny = _vertexNormal[3 * a + 1]; nz = _vertexNormal[3 * a + 2]; break;
                case 2: nx = _vertexNormal[3 * b]; ny = _vertexNormal[3 * b + 1]; nz = _vertexNormal[3 * b + 2]; break;
                case 3: nx = _vertexNormal[3 * c]; ny = _vertexNormal[3 * c + 1]; nz = _vertexNormal[3 * c + 2]; break;
                case 4: case 5: case 6: { int e = bestFeature - 4; nx = _edgeNormal[3 * (3 * bestTri + e)]; ny = _edgeNormal[3 * (3 * bestTri + e) + 1]; nz = _edgeNormal[3 * (3 * bestTri + e) + 2]; break; }
                default: nx = _faceNormal[3 * bestTri]; ny = _faceNormal[3 * bestTri + 1]; nz = _faceNormal[3 * bestTri + 2]; break;
            }
            double s = (px - bqx) * nx + (py - bqy) * ny + (pz - bqz) * nz;
            return (float)(s < 0 ? -dist : dist);
        }

        // Closest point on a triangle (Ericson, Real-Time Collision Detection, 5.1.5) and the feature it lies on:
        // 0 face, 1..3 vertex a,b,c, 4 edge ab, 5 edge bc, 6 edge ca.
        private void ClosestOnTriangle(int f, double px, double py, double pz, out double qx, out double qy, out double qz, out int feature)
        {
            int ia = _t[3 * f], ib = _t[3 * f + 1], ic = _t[3 * f + 2];
            double ax = V(ia, 0), ay = V(ia, 1), az = V(ia, 2), bx = V(ib, 0), by = V(ib, 1), bz = V(ib, 2), cx = V(ic, 0), cy = V(ic, 1), cz = V(ic, 2);
            double abx = bx - ax, aby = by - ay, abz = bz - az, acx = cx - ax, acy = cy - ay, acz = cz - az, apx = px - ax, apy = py - ay, apz = pz - az;
            double d1 = abx * apx + aby * apy + abz * apz, d2 = acx * apx + acy * apy + acz * apz;
            if (d1 <= 0 && d2 <= 0) { qx = ax; qy = ay; qz = az; feature = 1; return; }
            double bpx = px - bx, bpy = py - by, bpz = pz - bz, d3 = abx * bpx + aby * bpy + abz * bpz, d4 = acx * bpx + acy * bpy + acz * bpz;
            if (d3 >= 0 && d4 <= d3) { qx = bx; qy = by; qz = bz; feature = 2; return; }
            double vc = d1 * d4 - d3 * d2;
            if (vc <= 0 && d1 >= 0 && d3 <= 0) { double v = d1 / (d1 - d3); qx = ax + v * abx; qy = ay + v * aby; qz = az + v * abz; feature = 4; return; }
            double cpx = px - cx, cpy = py - cy, cpz = pz - cz, d5 = abx * cpx + aby * cpy + abz * cpz, d6 = acx * cpx + acy * cpy + acz * cpz;
            if (d6 >= 0 && d5 <= d6) { qx = cx; qy = cy; qz = cz; feature = 3; return; }
            double vb = d5 * d2 - d1 * d6;
            if (vb <= 0 && d2 >= 0 && d6 <= 0) { double w = d2 / (d2 - d6); qx = ax + w * acx; qy = ay + w * acy; qz = az + w * acz; feature = 6; return; }
            double va = d3 * d6 - d5 * d4;
            if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) { double w = (d4 - d3) / ((d4 - d3) + (d5 - d6)); qx = bx + w * (cx - bx); qy = by + w * (cy - by); qz = bz + w * (cz - bz); feature = 5; return; }
            double denom = 1.0 / (va + vb + vc), vv = vb * denom, ww = vc * denom;
            qx = ax + abx * vv + acx * ww; qy = ay + aby * vv + acy * ww; qz = az + abz * vv + acz * ww; feature = 0;
        }
    }
}
