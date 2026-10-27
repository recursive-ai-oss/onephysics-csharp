using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Collision3D;

/// <summary>
/// Quickhull in double precision over float32 input points (Barber, Dobkin, Huhdanpaa 1996; Gregorius, GDC 2014).
/// Phase 1 builds a triangle hull with conflict lists. Phase 2 merges coplanar triangles into polygons, drops vertices
/// that sit in the middle of an edge, and builds the half-edge structure. Every order is by index, so output depends only on the input.
/// </summary>
internal sealed class QuickhullBuilder
{
    private sealed class Face
    {
        public readonly int[] V = new int[3];
        public readonly Face?[] Nb = new Face?[3];
        public double Nx, Ny, Nz, Off;
        public bool Dead;
        public int Id;
        public int Mark;
        public List<int>? Out;
        public double FarDist;
        public int FarPt = -1;
    }

    private readonly double[] _x, _y, _z;
    private readonly int _n;
    private readonly HullBuildOptions _options;
    private readonly Vector3F[] _points;
    private double _eps;
    private double _visEps; // a face is visible from a point when it is farther above the face than this: far below the merge tolerance so that faces stay convex
    private readonly List<Face> _all = new();
    private int _alive;
    private int[] _refCount = Array.Empty<int>();
    private int _vertexCount;

    public QuickhullBuilder(ReadOnlySpan<Vector3F> points, HullBuildOptions options)
    {
        _options = options;
        _n = points.Length;
        if (options.MaxVertices != 0 && options.MaxVertices < 4)
        {
            throw new HullBuildException(HullBuildError.InvalidOptions, $"options.MaxVertices is {options.MaxVertices}; 0 (no limit) or at least 4 is expected.", nameof(options));
        }

        if (options.MaxFaces != 0 && options.MaxFaces < 4)
        {
            throw new HullBuildException(HullBuildError.InvalidOptions, $"options.MaxFaces is {options.MaxFaces}; 0 (no limit) or at least 4 is expected.", nameof(options));
        }

        if (options.Tolerance != 0 && !(options.Tolerance > 0 && float.IsFinite(options.Tolerance)))
        {
            throw new HullBuildException(HullBuildError.InvalidOptions, $"options.Tolerance is {options.Tolerance}; 0 (automatic) or a finite positive value is expected.", nameof(options));
        }

        if (_n < 4)
        {
            throw new HullBuildException(HullBuildError.TooFewPoints, $"points has {_n} entries; at least 4 non-coplanar points are expected.", nameof(points));
        }

        if (_n > ConvexHull3D.MaxInputPoints)
        {
            throw new HullBuildException(HullBuildError.TooManyPoints, $"points has {_n} entries; at most {ConvexHull3D.MaxInputPoints} are accepted.", nameof(points));
        }

        _points = points.ToArray();
        _x = new double[_n];
        _y = new double[_n];
        _z = new double[_n];
        for (int i = 0; i < _n; i++)
        {
            Vector3F p = _points[i];
            if (!float.IsFinite(p.X) || !float.IsFinite(p.Y) || !float.IsFinite(p.Z))
            {
                throw new HullBuildException(HullBuildError.NonFinite, $"points[{i}] = {p} has a NaN or infinite coordinate; finite coordinates are expected.", nameof(points));
            }

            if (Math.Abs(p.X) > ConvexHull3D.MaxCoordinate || Math.Abs(p.Y) > ConvexHull3D.MaxCoordinate || Math.Abs(p.Z) > ConvexHull3D.MaxCoordinate)
            {
                throw new HullBuildException(HullBuildError.OutOfRange, $"points[{i}] = {p} is outside +-{ConvexHull3D.MaxCoordinate}; coordinates within that bound are expected.", nameof(points));
            }

            _x[i] = p.X;
            _y[i] = p.Y;
            _z[i] = p.Z;
        }
    }

    private double Dist(Face f, int p) => f.Nx * _x[p] + f.Ny * _y[p] + f.Nz * _z[p] - f.Off;

    private void SetPlane(Face f)
    {
        int a = f.V[0], b = f.V[1], c = f.V[2];
        double e1x = _x[b] - _x[a], e1y = _y[b] - _y[a], e1z = _z[b] - _z[a];
        double e2x = _x[c] - _x[a], e2y = _y[c] - _y[a], e2z = _z[c] - _z[a];
        double nx = e1y * e2z - e1z * e2y, ny = e1z * e2x - e1x * e2z, nz = e1x * e2y - e1y * e2x;
        double len = Math.Sqrt(nx * nx + ny * ny + nz * nz);
        if (len > 0)
        {
            nx /= len;
            ny /= len;
            nz /= len;
        }
        else
        {
            nx = ny = nz = 0;
        }

        f.Nx = nx;
        f.Ny = ny;
        f.Nz = nz;
        f.Off = nx * (_x[a] + _x[b] + _x[c]) / 3.0 + ny * (_y[a] + _y[b] + _y[c]) / 3.0 + nz * (_z[a] + _z[b] + _z[c]) / 3.0;
    }

    private Face NewFace(int a, int b, int c)
    {
        var f = new Face { Id = _all.Count };
        f.V[0] = a;
        f.V[1] = b;
        f.V[2] = c;
        SetPlane(f);
        _all.Add(f);
        _alive++;
        for (int i = 0; i < 3; i++)
        {
            if (_refCount[f.V[i]]++ == 0)
            {
                _vertexCount++;
            }
        }

        return f;
    }

    private void Kill(Face f)
    {
        f.Dead = true;
        _alive--;
        for (int i = 0; i < 3; i++)
        {
            if (--_refCount[f.V[i]] == 0)
            {
                _vertexCount--;
            }
        }
    }

    private static void Assign(Face f, int p, double d)
    {
        f.Out ??= new List<int>();
        f.Out.Add(p);
        if (d > f.FarDist || f.FarPt < 0)
        {
            f.FarDist = d;
            f.FarPt = p;
        }
    }

    private static void Link(Face f, int u, int v, Face g)
    {
        for (int i = 0; i < 3; i++)
        {
            if (f.V[i] == u && f.V[(i + 1) % 3] == v)
            {
                f.Nb[i] = g;
                return;
            }
        }
    }

    private HullBuildException Degenerate(string what) =>
        new(HullBuildError.Degenerate, $"points do not span a volume: {what} (tolerance {_eps:R}); at least 4 non-coplanar points are expected.");

    public ConvexHull3D Build()
    {
        double minX = double.MaxValue, minY = double.MaxValue, minZ = double.MaxValue, maxX = -double.MaxValue, maxY = -double.MaxValue, maxZ = -double.MaxValue;
        int[] ext = new int[6]; // min x, max x, min y, max y, min z, max z; lowest index wins ties
        for (int i = 0; i < _n; i++)
        {
            if (_x[i] < minX) { minX = _x[i]; ext[0] = i; }
            if (_x[i] > maxX) { maxX = _x[i]; ext[1] = i; }
            if (_y[i] < minY) { minY = _y[i]; ext[2] = i; }
            if (_y[i] > maxY) { maxY = _y[i]; ext[3] = i; }
            if (_z[i] < minZ) { minZ = _z[i]; ext[4] = i; }
            if (_z[i] > maxZ) { maxZ = _z[i]; ext[5] = i; }
        }

        double extentSum = (maxX - minX) + (maxY - minY) + (maxZ - minZ);
        double magSum = Math.Max(Math.Abs(minX), Math.Abs(maxX)) + Math.Max(Math.Abs(minY), Math.Abs(maxY)) + Math.Max(Math.Abs(minZ), Math.Abs(maxZ));
        _eps = _options.Tolerance > 0 ? _options.Tolerance : 3.0 * 1.1920928955078125e-7 * extentSum;
        _eps = Math.Max(_eps, 1e-14 * magSum);
        if (_eps <= 0)
        {
            throw Degenerate("all points coincide");
        }

        _visEps = _eps * 1e-3;

        // Initial tetrahedron: the farthest pair among the axis extremes, then the point farthest from their line, then from their plane.
        int i0 = -1, i1 = -1;
        double best = -1;
        for (int a = 0; a < 6; a++)
        {
            for (int b = a + 1; b < 6; b++)
            {
                double dx = _x[ext[a]] - _x[ext[b]], dy = _y[ext[a]] - _y[ext[b]], dz = _z[ext[a]] - _z[ext[b]];
                double d2 = dx * dx + dy * dy + dz * dz;
                if (d2 > best)
                {
                    best = d2;
                    i0 = ext[a];
                    i1 = ext[b];
                }
            }
        }

        if (!(Math.Sqrt(best) > _eps))
        {
            throw Degenerate("all points coincide");
        }

        double lx = _x[i1] - _x[i0], ly = _y[i1] - _y[i0], lz = _z[i1] - _z[i0];
        double llen = Math.Sqrt(lx * lx + ly * ly + lz * lz);
        int i2 = -1;
        best = -1;
        for (int i = 0; i < _n; i++)
        {
            double px = _x[i] - _x[i0], py = _y[i] - _y[i0], pz = _z[i] - _z[i0];
            double cx = ly * pz - lz * py, cy = lz * px - lx * pz, cz = lx * py - ly * px;
            double d = Math.Sqrt(cx * cx + cy * cy + cz * cz) / llen;
            if (d > best)
            {
                best = d;
                i2 = i;
            }
        }

        if (!(best > _eps))
        {
            throw Degenerate("all points are collinear");
        }

        double e2x = _x[i2] - _x[i0], e2y = _y[i2] - _y[i0], e2z = _z[i2] - _z[i0];
        double nx = ly * e2z - lz * e2y, ny = lz * e2x - lx * e2z, nz = lx * e2y - ly * e2x;
        double nlen = Math.Sqrt(nx * nx + ny * ny + nz * nz);
        nx /= nlen;
        ny /= nlen;
        nz /= nlen;
        int i3 = -1;
        best = -1;
        double bestSigned = 0;
        for (int i = 0; i < _n; i++)
        {
            double d = nx * (_x[i] - _x[i0]) + ny * (_y[i] - _y[i0]) + nz * (_z[i] - _z[i0]);
            if (Math.Abs(d) > best)
            {
                best = Math.Abs(d);
                bestSigned = d;
                i3 = i;
            }
        }

        if (!(best > _eps))
        {
            throw Degenerate("all points are coplanar");
        }

        _refCount = new int[_n];
        // Orient so that the fourth point is below the plane of (i0, i1, i2) as seen from the side the normal points to.
        if (bestSigned > 0)
        {
            (i1, i2) = (i2, i1);
        }

        int ia = i0, ib = i1, ic = i2, id = i3;
        Face f0 = NewFace(ia, ib, ic), f1 = NewFace(ib, ia, id), f2 = NewFace(ic, ib, id), f3 = NewFace(ia, ic, id);
        Face[] tet = { f0, f1, f2, f3 };
        foreach (Face f in tet)
        {
            foreach (Face g in tet)
            {
                if (f == g)
                {
                    continue;
                }

                for (int e = 0; e < 3; e++)
                {
                    Link(f, g.V[(e + 1) % 3], g.V[e], g);
                }
            }
        }

        // Conflict lists: each remaining point goes to the face it is farthest above.
        for (int p = 0; p < _n; p++)
        {
            if (p == ia || p == ib || p == ic || p == id)
            {
                continue;
            }

            Face? bestFace = null;
            double bestD = _eps;
            foreach (Face f in tet)
            {
                double d = Dist(f, p);
                if (d > bestD)
                {
                    bestD = d;
                    bestFace = f;
                }
            }

            if (bestFace != null)
            {
                Assign(bestFace, p, bestD);
            }
        }

        var stack = new List<Face>();
        foreach (Face f in tet)
        {
            if (f.Out != null)
            {
                stack.Add(f);
            }
        }

        int maxV = _options.MaxVertices == 0 ? int.MaxValue : _options.MaxVertices;
        int maxF = _options.MaxFaces == 0 ? int.MaxValue : _options.MaxFaces;
        int visibleMark = 0;
        var visible = new List<Face>();
        var horizonFace = new List<Face>();
        var horizonEdge = new List<int>();
        var fromIndex = new Dictionary<int, int>();
        var orphans = new List<int>();
        var vertexVis = new Dictionary<int, int>();
        bool stop = false;

        while (stack.Count > 0 && !stop)
        {
            Face seed = stack[^1];
            stack.RemoveAt(stack.Count - 1);
            if (seed.Dead || seed.Out == null || seed.Out.Count == 0)
            {
                continue;
            }

            int apex = seed.FarPt;
            visibleMark++;
            visible.Clear();
            seed.Mark = visibleMark;
            visible.Add(seed);
            for (int k = 0; k < visible.Count; k++)
            {
                Face v = visible[k];
                for (int e = 0; e < 3; e++)
                {
                    Face? g = v.Nb[e];
                    if (g != null && g.Mark != visibleMark && Dist(g, apex) > _visEps)
                    {
                        g.Mark = visibleMark;
                        visible.Add(g);
                    }
                }
            }

            int h = 0;
            bool ok = false;
            int[] order = Array.Empty<int>();
            for (int attempt = 0; attempt < 2 && !ok; attempt++)
            {
                if (attempt == 1)
                {
                    // The horizon pinched: faces with two visible neighbours form a one-triangle notch; take them too and try again.
                    for (bool changed = true; changed;)
                    {
                        changed = false;
                        for (int k = 0; k < visible.Count; k++)
                        {
                            for (int e = 0; e < 3; e++)
                            {
                                Face? g = visible[k].Nb[e];
                                if (g != null && g.Mark != visibleMark)
                                {
                                    int vis = 0;
                                    for (int q = 0; q < 3; q++)
                                    {
                                        if (g.Nb[q]!.Mark == visibleMark)
                                        {
                                            vis++;
                                        }
                                    }

                                    if (vis >= 2)
                                    {
                                        g.Mark = visibleMark;
                                        visible.Add(g);
                                        changed = true;
                                    }
                                }
                            }
                        }
                    }

                }

                horizonFace.Clear();
                horizonEdge.Clear();
                foreach (Face v in visible)
                {
                    for (int e = 0; e < 3; e++)
                    {
                        if (v.Nb[e]!.Mark != visibleMark)
                        {
                            horizonFace.Add(v);
                            horizonEdge.Add(e);
                        }
                    }
                }

                // Order the horizon edges into one cycle; anything else is a numerical pinch and the apex is skipped.
                h = horizonFace.Count;
                ok = h >= 3;
                fromIndex.Clear();
                for (int k = 0; k < h && ok; k++)
                {
                    int from = horizonFace[k].V[horizonEdge[k]];
                    ok = fromIndex.TryAdd(from, k);
                }

                order = new int[ok ? h : 0];
                if (ok)
                {
                    bool[] seen = new bool[h];
                    int cur = 0;
                    order[0] = 0;
                    seen[0] = true;
                    for (int k = 1; k < h && ok; k++)
                    {
                        int to = horizonFace[cur].V[(horizonEdge[cur] + 1) % 3];
                        ok = fromIndex.TryGetValue(to, out cur) && !seen[cur];
                        if (ok)
                        {
                            seen[cur] = true;
                            order[k] = cur;
                        }
                    }

                    if (ok)
                    {
                        ok = horizonFace[cur].V[(horizonEdge[cur] + 1) % 3] == horizonFace[0].V[horizonEdge[0]];
                    }
                }

            }

            if (!ok)
            {
                DropPoint(seed, apex);
                if (seed.Out != null && seed.Out.Count > 0)
                {
                    stack.Add(seed);
                }

                continue;
            }

            // Limits: vertices that lose every face disappear, the apex appears.
            vertexVis.Clear();
            foreach (Face v in visible)
            {
                for (int e = 0; e < 3; e++)
                {
                    vertexVis[v.V[e]] = vertexVis.GetValueOrDefault(v.V[e]) + 1;
                }
            }

            int removed = 0;
            foreach (Face v in visible)
            {
                for (int e = 0; e < 3; e++)
                {
                    int vv = v.V[e];
                    if (vertexVis.TryGetValue(vv, out int cnt) && cnt == _refCount[vv])
                    {
                        removed++;
                        vertexVis.Remove(vv);
                    }
                }
            }

            if (_vertexCount + 1 - removed > maxV || _alive - visible.Count + h > maxF)
            {
                stop = true;
                break;
            }

            var created = new Face[h];
            for (int k = 0; k < h; k++)
            {
                Face vf = horizonFace[order[k]];
                int e = horizonEdge[order[k]];
                int a = vf.V[e], b = vf.V[(e + 1) % 3];
                Face outside = vf.Nb[e]!;
                Face nf = NewFace(a, b, apex);
                nf.Nb[0] = outside;
                Link(outside, b, a, nf);
                created[k] = nf;
            }

            for (int k = 0; k < h; k++)
            {
                created[k].Nb[1] = created[(k + 1) % h];
                created[k].Nb[2] = created[(k + h - 1) % h];
            }

            orphans.Clear();
            foreach (Face v in visible)
            {
                if (v.Out != null)
                {
                    foreach (int p in v.Out)
                    {
                        if (p != apex)
                        {
                            orphans.Add(p);
                        }
                    }
                }
            }

            foreach (Face v in visible)
            {
                Kill(v);
                v.Out = null;
            }

            foreach (int p in orphans)
            {
                Face? bestFace = null;
                double bestD = _eps;
                foreach (Face nf in created)
                {
                    double d = Dist(nf, p);
                    if (d > bestD)
                    {
                        bestD = d;
                        bestFace = nf;
                    }
                }

                if (bestFace != null)
                {
                    Assign(bestFace, p, bestD);
                }
            }

            foreach (Face nf in created)
            {
                if (nf.Out != null)
                {
                    stack.Add(nf);
                }
            }
        }

        return Finish();
    }

    // True when the triangle's smallest altitude is within the tolerance; longestEdge is the index of its longest edge (ties to the lowest).
    private bool IsSliver(Face f, out int longestEdge)
    {
        double best = -1;
        longestEdge = 0;
        for (int e = 0; e < 3; e++)
        {
            int a = f.V[e], b = f.V[(e + 1) % 3];
            double dx = _x[b] - _x[a], dy = _y[b] - _y[a], dz = _z[b] - _z[a];
            double l2 = (dx * dx) + (dy * dy) + (dz * dz);
            if (l2 > best)
            {
                best = l2;
                longestEdge = e;
            }
        }

        // Twice the area is |cross|; the smallest altitude is twice the area over the longest edge.
        int p = f.V[0], q = f.V[1], r = f.V[2];
        double ux = _x[q] - _x[p], uy = _y[q] - _y[p], uz = _z[q] - _z[p], vx = _x[r] - _x[p], vy = _y[r] - _y[p], vz = _z[r] - _z[p];
        double cx = (uy * vz) - (uz * vy), cy = (uz * vx) - (ux * vz), cz = (ux * vy) - (uy * vx);
        double area2 = Math.Sqrt((cx * cx) + (cy * cy) + (cz * cz));
        return area2 <= _eps * Math.Sqrt(best);
    }

    private void DropPoint(Face f, int p)
    {
        f.Out!.Remove(p);
        f.FarPt = -1;
        f.FarDist = 0;
        foreach (int q in f.Out)
        {
            double d = Dist(f, q);
            if (f.FarPt < 0 || d > f.FarDist)
            {
                f.FarDist = d;
                f.FarPt = q;
            }
        }
    }

    // Phase 2: triangles to polygons to half-edges.
    private ConvexHull3D Finish()
    {
        var tris = new List<Face>();
        int[] index = new int[_all.Count];
        foreach (Face f in _all)
        {
            if (!f.Dead)
            {
                index[f.Id] = tris.Count;
                tris.Add(f);
            }
        }

        int t = tris.Count;
        int[] parent = new int[t];
        for (int i = 0; i < t; i++)
        {
            parent[i] = i;
        }

        int Find(int i)
        {
            while (parent[i] != i)
            {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }

            return i;
        }

        for (int i = 0; i < t; i++)
        {
            Face f = tris[i];
            for (int e = 0; e < 3; e++)
            {
                Face g = f.Nb[e]!;
                int j = index[g.Id];
                if (j <= i)
                {
                    continue;
                }

                int fo = f.V[(e + 2) % 3];
                int go = -1;
                for (int q = 0; q < 3; q++)
                {
                    if (g.V[q] != f.V[e] && g.V[q] != f.V[(e + 1) % 3])
                    {
                        go = g.V[q];
                    }
                }

                // A sliver (a triangle thinner than the tolerance) has no meaningful plane of its own: it joins the neighbour across its longest edge, judged by that neighbour's plane alone.
                bool fSliver = IsSliver(f, out int fLong), gSliver = IsSliver(g, out int gLong);
                bool ok;
                if (fSliver && gSliver)
                {
                    ok = false;
                }
                else if (fSliver)
                {
                    ok = fLong == e && Math.Abs(Dist(g, fo)) <= _eps;
                }
                else if (gSliver)
                {
                    int ge = -1;
                    for (int q = 0; q < 3; q++)
                    {
                        if (g.V[q] == f.V[(e + 1) % 3] && g.V[(q + 1) % 3] == f.V[e])
                        {
                            ge = q;
                        }
                    }

                    ok = gLong == ge && Math.Abs(Dist(f, go)) <= _eps;
                }
                else
                {
                    ok = Math.Abs(Dist(f, go)) <= _eps && Math.Abs(Dist(g, fo)) <= _eps;
                }

                if (ok)
                {
                    int ri = Find(i), rj = Find(j);
                    if (ri != rj)
                    {
                        parent[Math.Max(ri, rj)] = Math.Min(ri, rj);
                    }
                }
            }
        }

        int[] group = new int[t];
        for (int i = 0; i < t; i++)
        {
            group[i] = Find(i);
        }

        var members = new List<int>[t];
        for (int i = 0; i < t; i++)
        {
            (members[group[i]] ??= new List<int>()).Add(i);
        }

        var polys = new List<int[]>();
        var next = new Dictionary<int, int>();
        for (int r = 0; r < t; r++)
        {
            List<int>? m = members[r];
            if (m == null)
            {
                continue;
            }

            int[]? cycle = null;
            if (m.Count > 1)
            {
                next.Clear();
                var order = new List<int>();
                bool ok = true;
                foreach (int ti in m)
                {
                    for (int e = 0; e < 3 && ok; e++)
                    {
                        if (group[index[tris[ti].Nb[e]!.Id]] != r)
                        {
                            int from = tris[ti].V[e];
                            ok = next.TryAdd(from, tris[ti].V[(e + 1) % 3]);
                            order.Add(from);
                        }
                    }
                }

                if (ok && order.Count >= 3)
                {
                    var cyc = new List<int>();
                    int start = order[0], cur = start;
                    do
                    {
                        cyc.Add(cur);
                        ok = next.TryGetValue(cur, out cur);
                    }
                    while (ok && cur != start && cyc.Count <= order.Count);
                    if (ok && cyc.Count == order.Count)
                    {
                        cycle = cyc.ToArray();
                    }
                }

                if (cycle == null)
                {
                    foreach (int ti in m)
                    {
                        group[ti] = ti;
                    }
                }
            }

            if (cycle != null)
            {
                polys.Add(cycle);
            }
            else
            {
                foreach (int ti in m)
                {
                    polys.Add(new[] { tris[ti].V[0], tris[ti].V[1], tris[ti].V[2] });
                }
            }
        }

        // A vertex lying on exactly two faces is in the middle of an edge, not a corner.
        int[] valence = new int[_n];
        foreach (int[] p in polys)
        {
            foreach (int v in p)
            {
                valence[v]++;
            }
        }

        for (int i = 0; i < polys.Count; i++)
        {
            int[] p = polys[i];
            int keep = 0;
            foreach (int v in p)
            {
                if (valence[v] != 2)
                {
                    keep++;
                }
            }

            if (keep == p.Length)
            {
                continue;
            }

            if (keep < 3)
            {
                // A sliver face squeezed between two faces; they meet along its long edge.
                polys[i] = Array.Empty<int>();
                continue;
            }

            int[] q = new int[keep];
            int w = 0;
            foreach (int v in p)
            {
                if (valence[v] != 2)
                {
                    q[w++] = v;
                }
            }

            polys[i] = q;
        }

        polys.RemoveAll(static p => p.Length == 0);

        // Canonical form: rotate each polygon to start at its lowest vertex, order polygons by that vertex then the next.
        for (int i = 0; i < polys.Count; i++)
        {
            int[] p = polys[i];
            int lo = 0;
            for (int k = 1; k < p.Length; k++)
            {
                if (p[k] < p[lo])
                {
                    lo = k;
                }
            }

            if (lo != 0)
            {
                int[] q = new int[p.Length];
                for (int k = 0; k < p.Length; k++)
                {
                    q[k] = p[(lo + k) % p.Length];
                }

                polys[i] = q;
            }
        }

        polys.Sort(static (a, b) =>
        {
            int c = a[0].CompareTo(b[0]);
            if (c != 0)
            {
                return c;
            }

            c = a[1].CompareTo(b[1]);
            return c != 0 ? c : a[2].CompareTo(b[2]);
        });

        // Compact the vertices (input order) and remap.
        int[] remap = new int[_n];
        Array.Fill(remap, -1);
        foreach (int[] p in polys)
        {
            foreach (int v in p)
            {
                remap[v] = 0;
            }
        }

        int vc = 0;
        var verts = new List<Vector3F>();
        for (int i = 0; i < _n; i++)
        {
            if (remap[i] == 0)
            {
                remap[i] = vc++;
                verts.Add(_points[i]);
            }
        }

        foreach (int[] p in polys)
        {
            for (int k = 0; k < p.Length; k++)
            {
                p[k] = remap[p[k]];
            }
        }

        Vector3F[] vertices = verts.ToArray();
        int faceCount = polys.Count;
        int edgeTotal = 0;
        foreach (int[] p in polys)
        {
            edgeTotal += p.Length;
        }

        if (vertices.Length - edgeTotal / 2 + faceCount != 2 || (edgeTotal & 1) != 0)
        {
            throw Degenerate("the hull surface is not a closed polyhedron");
        }

        // Pair directed edges by sorting on (undirected key, direction): twins land next to each other.
        long[] keys = new long[edgeTotal];
        int[] ids = new int[edgeTotal];
        int[] from0 = new int[edgeTotal];
        int[] faceOf = new int[edgeTotal];
        int[] nextOf = new int[edgeTotal];
        int cursor = 0;
        for (int fi = 0; fi < faceCount; fi++)
        {
            int[] p = polys[fi];
            for (int k = 0; k < p.Length; k++)
            {
                int a = p[k], b = p[(k + 1) % p.Length];
                int lo = Math.Min(a, b), hi = Math.Max(a, b);
                keys[cursor] = ((long)lo * vertices.Length + hi) * 2 + (a < b ? 0 : 1);
                ids[cursor] = cursor;
                from0[cursor] = a;
                faceOf[cursor] = fi;
                nextOf[cursor] = k + 1 == p.Length ? cursor - k : cursor + 1;
                cursor++;
            }
        }

        Array.Sort(keys, ids);
        int[] newIndex = new int[edgeTotal];
        for (int s = 0; s < edgeTotal; s++)
        {
            newIndex[ids[s]] = s;
        }

        for (int s = 0; s + 1 < edgeTotal; s += 2)
        {
            if (keys[s] / 2 != keys[s + 1] / 2 || (keys[s] & 1) != 0 || (keys[s + 1] & 1) != 1 || (s + 2 < edgeTotal && keys[s + 2] / 2 == keys[s] / 2))
            {
                throw Degenerate("the hull surface is not a closed polyhedron");
            }
        }

        var edges = new HullEdge3D[edgeTotal];
        for (int old = 0; old < edgeTotal; old++)
        {
            edges[newIndex[old]] = new HullEdge3D(from0[old], newIndex[nextOf[old]], faceOf[old]);
        }

        int[] vertexEdge = new int[vertices.Length];
        Array.Fill(vertexEdge, -1);
        for (int e = 0; e < edgeTotal; e++)
        {
            if (vertexEdge[edges[e].Origin] < 0)
            {
                vertexEdge[edges[e].Origin] = e;
            }
        }

        var faces = new HullFace3D[faceCount];
        cursor = 0;
        for (int fi = 0; fi < faceCount; fi++)
        {
            int[] p = polys[fi];
            // Newell's method about the polygon's mean: robust for slightly non-planar polygons.
            double mx = 0, my = 0, mz = 0;
            foreach (int v in p)
            {
                mx += vertices[v].X;
                my += vertices[v].Y;
                mz += vertices[v].Z;
            }

            mx /= p.Length;
            my /= p.Length;
            mz /= p.Length;
            double nx = 0, ny = 0, nz = 0;
            for (int k = 0; k < p.Length; k++)
            {
                Vector3F a = vertices[p[k]], b = vertices[p[(k + 1) % p.Length]];
                double ax = a.X - mx, ay = a.Y - my, az = a.Z - mz, bx = b.X - mx, by = b.Y - my, bz = b.Z - mz;
                nx += ay * bz - az * by;
                ny += az * bx - ax * bz;
                nz += ax * by - ay * bx;
            }

            double len = Math.Sqrt(nx * nx + ny * ny + nz * nz);
            if (!(len > 0))
            {
                throw Degenerate("a face has no area");
            }

            nx /= len;
            ny /= len;
            nz /= len;
            double off = nx * mx + ny * my + nz * mz;
            faces[fi] = new HullFace3D(new Vector3F((float)nx, (float)ny, (float)nz), (float)off, newIndex[cursor], p.Length);
            cursor += p.Length;
        }

        var mass = ConvexHull3D.ComputeMass(vertices, polys.ToArray());
        return new ConvexHull3D(vertices, faces, edges, vertexEdge, mass);
    }
}
