using System;
using RecursiveAI.OneMath;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>Result of a penetration or separation query between two convex shapes, in the frame of shape A.</summary>
[Experimental("OPHYS0001")]
public readonly struct ConvexContact3D
{
    internal ConvexContact3D(float separation, Vector3F normal, Vector3F pointA, Vector3F pointB)
    {
        Separation = separation;
        Normal = normal;
        PointA = pointA;
        PointB = pointB;
    }

    /// <summary>Signed distance between the surfaces: positive when apart, negative (minus the penetration depth) when overlapping, 0 when touching.</summary>
    public float Separation { get; }

    /// <summary>True when the shapes overlap or touch.</summary>
    public bool Colliding => Separation <= 0f;

    /// <summary>Unit direction from A towards B: moving B along it by the penetration depth (or A against it) separates the shapes by the least distance.</summary>
    public Vector3F Normal { get; }

    /// <summary>The deepest point of A, or the closest point on A's surface when apart.</summary>
    public Vector3F PointA { get; }

    /// <summary>The deepest point of B, or the closest point on B's surface when apart. Moving B by <c>Normal * depth</c> brings <see cref="PointB"/> onto <see cref="PointA"/>'s surface.</summary>
    public Vector3F PointB { get; }
}

/// <summary>
/// Expanding Polytope Algorithm: penetration depth and direction of overlapping convex shapes (van den Bergen 2001, "Proximity queries and penetration depth computation on 3D game objects"; Ericson 2005, ch. 9).
/// An instance owns its working storage and reuses it, so queries do not allocate; it is not safe to share between threads.
/// </summary>
[Experimental("OPHYS0001")]
public sealed class Epa3D
{
    private const int MaxVertices = 128;
    private const int MaxFaces = (2 * MaxVertices) + 16;
    private const float Tol = 4e-6f;

    // A face is removed when the new point is farther than this above its plane (relative to the shapes' size); nearly coplanar faces stay, which keeps the polytope closed in the face of round-off.
    private const float VisTol = 1e-6f;

    // The six axis directions the starting simplex is grown along; shared because the method runs for every deep contact and must not allocate.
    private static readonly Vector3F[] s_axes = { Vector3F.UnitX, -Vector3F.UnitX, Vector3F.UnitY, -Vector3F.UnitY, Vector3F.UnitZ, -Vector3F.UnitZ };

    private readonly Vector3F[] _p = new Vector3F[MaxVertices];
    private readonly Vector3F[] _a = new Vector3F[MaxVertices];
    private readonly Vector3F[] _b = new Vector3F[MaxVertices];
    private int _nv;

    // Faces: vertex indices, the neighbouring face across edge k (k -> k+1), outward unit normal and plane distance. Freed slots are reused.
    private readonly int[] _fi = new int[MaxFaces * 3];
    private readonly int[] _nb = new int[MaxFaces * 3];
    private readonly Vector3F[] _fn = new Vector3F[MaxFaces];
    private readonly float[] _fd = new float[MaxFaces];
    private readonly bool[] _alive = new bool[MaxFaces];
    private readonly int[] _stamp = new int[MaxFaces];
    private int _stampCounter;
    private int _slots;
    private readonly int[] _free = new int[MaxFaces];
    private int _freeCount;

    private readonly int[] _stack = new int[MaxFaces];
    private readonly int[] _visList = new int[MaxFaces];
    private readonly int[] _hf = new int[MaxFaces * 3]; // horizon: visible face and edge
    private readonly int[] _he = new int[MaxFaces * 3];
    private readonly int[] _newFaces = new int[MaxFaces * 3];

    /// <summary>Signed separation, normal and contact points of two convex shapes: runs GJK, then EPA when they overlap.</summary>
    /// <param name="a">Shape A, in its own frame.</param>
    /// <param name="b">Shape B, in its own frame.</param>
    /// <param name="bInA">Placement of B's frame in A's frame.</param>
    /// <returns>For separated shapes, the distance and closest points; for overlapping shapes, minus the penetration depth, the direction of least penetration and the deepest points. Accuracy is about 1e-5 of the shapes' size.</returns>
    public ConvexContact3D Collide(in ConvexShape3D a, in ConvexShape3D b, in RigidTransformF bInA)
    {
        var pair = new ConvexPair(a, b, bInA);
        Simplex s = default;
        Gjk3D.Run(pair, ref s, out Vector3F v, out bool coresOverlap, out _, out float scale2);
        float rA = a.ConvexRadius, rB = b.ConvexRadius, radii = rA + rB;
        if (!coresOverlap)
        {
            float coreDist = ConvexShape3D.Sqrt(v.LengthSquared());
            Vector3F n = -v / coreDist;
            Vector3F pa = s.WitnessA() + (n * rA), pb = s.WitnessB() - (n * rB);
            return new ConvexContact3D(coreDist - radii, n, pa, pb);
        }

        if (!Penetrate(pair, ref s, scale2, out Vector3F normal, out float depth, out Vector3F wa, out Vector3F wb))
        {
            // The cores' difference is flat or a point (for instance two crossing segments): the depth is the radii and the direction is the best guess.
            Vector3F toB = bInA.Translation;
            if (normal.LengthSquared() > 0f)
            {
                normal = Vector3F.Dot(normal, toB) < 0f ? -normal : normal;
            }
            else
            {
                normal = toB.LengthSquared() > 0f ? toB.Normalized() : Vector3F.UnitX;
            }

            depth = 0f;
        }

        return new ConvexContact3D(-(depth + radii), normal, wa + (normal * rA), wb - (normal * rB));
    }

    private bool Penetrate(in ConvexPair pair, ref Simplex s, float scale2, out Vector3F normal, out float depth, out Vector3F wa, out Vector3F wb)
    {
        normal = default;
        depth = 0f;
        wa = default;
        wb = default;
        float scale = ConvexShape3D.Sqrt(scale2);
        _nv = 0;
        for (int i = 0; i < s.Count; i++)
        {
            AddVertex(s.P[i], s.A[i], s.B[i]);
        }

        if (_nv == 4)
        {
            Vector3F ta = _p[1] - _p[0], tb = _p[2] - _p[0], tc = _p[3] - _p[0];
            if (Math.Abs(Vector3F.Dot(Vector3F.Cross(ta, tb), tc)) <= 1e-7f * scale * scale * scale)
            {
                _nv = 3;
            }
        }

        if (!Expand(pair, scale, out normal))
        {
            // Flat Minkowski difference: report the barycentric witness of what we have.
            wa = s.Count > 0 ? s.WitnessA() : default;
            wb = s.Count > 0 ? s.WitnessB() : default;
            return false;
        }

        // Initial tetrahedron faces, oriented away from the centroid.
        Vector3F c = (_p[0] + _p[1] + _p[2] + _p[3]) * 0.25f;
        _slots = 0;
        _freeCount = 0;
        AddInitialFace(0, 1, 2, c);
        AddInitialFace(0, 3, 1, c);
        AddInitialFace(0, 2, 3, c);
        AddInitialFace(1, 3, 2, c);
        for (int f = 0; f < 4; f++)
        {
            for (int k = 0; k < 3; k++)
            {
                int u = _fi[(f * 3) + k], v = _fi[(f * 3) + ((k + 1) % 3)];
                for (int g = 0; g < 4; g++)
                {
                    for (int kk = 0; kk < 3; kk++)
                    {
                        if (g != f && _fi[(g * 3) + kk] == v && _fi[(g * 3) + ((kk + 1) % 3)] == u)
                        {
                            _nb[(f * 3) + k] = g;
                        }
                    }
                }
            }
        }

        int best;
        bool converged = false, degraded = false;
        float seenDist = -1f;
        Vector3F seenNormal = default;
        for (int iter = 0; iter < MaxVertices - 4; iter++)
        {
            best = FindClosestFace();
            Vector3F n = _fn[best];
            float dist = _fd[best];
            // In exact arithmetic the closest face only moves outwards. If it moves in, round-off has broken the polytope: keep the best answer seen.
            if (dist >= seenDist)
            {
                seenDist = dist;
                seenNormal = n;
            }
            else if (dist < seenDist - (1e-4f * scale))
            {
                degraded = true;
                break;
            }

            pair.Support(n, out Vector3F w, out Vector3F wa1, out Vector3F wb1);
            float wn = Vector3F.Dot(n, w);
            float w2 = w.LengthSquared();
            if (w2 > scale2)
            {
                scale2 = w2;
                scale = ConvexShape3D.Sqrt(w2);
            }

            if (wn - dist <= Tol * scale)
            {
                converged = true;
                break;
            }

            if (_nv >= MaxVertices || !Grow(best, w, wa1, wb1, scale, scale2))
            {
                break;
            }
        }

        if (degraded)
        {
            normal = seenNormal;
            depth = seenDist;
            pair.Support(normal, out Vector3F ds, out wa, out wb);
            _ = ds;
            return true;
        }

        best = FindClosestFace();
        normal = _fn[best];
        depth = _fd[best];
        if (!converged && Polish(pair, ref normal, ref depth, out wa, out wb))
        {
            return true;
        }

        // Barycentric coordinates of the projection of the origin onto the closest face give the witness points.
        Vector3F t0 = _p[_fi[best * 3]], t1 = _p[_fi[(best * 3) + 1]], t2 = _p[_fi[(best * 3) + 2]];
        Vector3F proj = normal * depth;
        Vector3F e0 = t1 - t0, e1 = t2 - t0, e2v = proj - t0;
        float d00 = Vector3F.Dot(e0, e0), d01 = Vector3F.Dot(e0, e1), d11 = Vector3F.Dot(e1, e1);
        float d20 = Vector3F.Dot(e2v, e0), d21 = Vector3F.Dot(e2v, e1);
        float den = (d00 * d11) - (d01 * d01);
        float l1 = 1f / 3f, l2 = 1f / 3f;
        if (den > 0f)
        {
            l1 = ((d11 * d20) - (d01 * d21)) / den;
            l2 = ((d00 * d21) - (d01 * d20)) / den;
        }

        float l0 = 1f - l1 - l2;
        l0 = l0 < 0f ? 0f : l0;
        l1 = l1 < 0f ? 0f : l1;
        l2 = l2 < 0f ? 0f : l2;
        float sum = l0 + l1 + l2;
        l0 /= sum;
        l1 /= sum;
        l2 /= sum;
        int i0 = _fi[best * 3], i1 = _fi[(best * 3) + 1], i2 = _fi[(best * 3) + 2];
        wa = (_a[i0] * l0) + (_a[i1] * l1) + (_a[i2] * l2);
        wb = (_b[i0] * l0) + (_b[i1] * l1) + (_b[i2] * l2);
        return true;
    }

    // A smooth shape keeps the polytope from converging in a bounded number of vertices (its faces only approximate the curved surface from inside).
    // The support distance h(n) = n . s(n) is an upper bound on the depth for every direction n, its minimum is the depth, and its gradient on the unit sphere
    // is the part of s(n) perpendicular to n; so steps downhill from the polytope's best normal close most of the gap. Returns false when no step improved on the polytope.
    private static bool Polish(in ConvexPair pair, ref Vector3F normal, ref float depth, out Vector3F wa, out Vector3F wb)
    {
        Vector3F n = normal;
        pair.Support(n, out Vector3F s, out wa, out wb);
        float h = Vector3F.Dot(n, s);
        Vector3F bestN = n;
        for (int iter = 0; iter < 16; iter++)
        {
            Vector3F g = s - (n * h);
            float step = 1f / h;
            bool moved = false;
            for (int tries = 0; tries < 6 && !moved; tries++, step *= 0.5f)
            {
                Vector3F cand = n - (g * step);
                float cl = ConvexShape3D.Sqrt(cand.LengthSquared());
                if (!(cl > 0f))
                {
                    break;
                }

                cand /= cl;
                pair.Support(cand, out Vector3F cs, out Vector3F ca, out Vector3F cb);
                float ch = Vector3F.Dot(cand, cs);
                if (ch < h)
                {
                    bool tiny = h - ch <= 1e-7f * h;
                    n = cand;
                    s = cs;
                    h = ch;
                    wa = ca;
                    wb = cb;
                    bestN = cand;
                    moved = true;
                    if (tiny)
                    {
                        iter = 99; // converged: stop after this step
                    }
                }
            }

            if (!moved)
            {
                break;
            }
        }

        if (!(h >= depth) || h > depth * 1.05f)
        {
            return false; // inconsistent with the polytope bound: keep the polytope's answer
        }

        depth = h;
        normal = bestN;
        return true;
    }

    // The live face nearest the origin; the lowest slot wins ties.
    private int FindClosestFace()
    {
        int best = -1;
        for (int f = 0; f < _slots; f++)
        {
            if (_alive[f] && (best < 0 || _fd[f] < _fd[best]))
            {
                best = f;
            }
        }

        return best;
    }

    // Adds point w: removes the connected set of faces that can see it (found by walking neighbours from the closest face)
    // and closes the hole with a fan to w (van den Bergen 2001). Returns false, leaving the polytope as it was, when the hole is not a simple loop.
    private bool Grow(int seed, Vector3F w, Vector3F wa, Vector3F wb, float scale, float scale2)
    {
        int stamp = ++_stampCounter;
        int visCount = 0, sp = 0;
        _stamp[seed] = stamp;
        _stack[sp++] = seed;
        float visTol = VisTol * scale;
        while (sp > 0)
        {
            int f = _stack[--sp];
            _visList[visCount++] = f;
            for (int k = 0; k < 3; k++)
            {
                int g = _nb[(f * 3) + k];
                if (_stamp[g] != stamp && Vector3F.Dot(_fn[g], w) - _fd[g] > visTol)
                {
                    _stamp[g] = stamp;
                    _stack[sp++] = g;
                }
            }
        }

        int ne = 0;
        for (int i = 0; i < visCount; i++)
        {
            int f = _visList[i];
            for (int k = 0; k < 3; k++)
            {
                if (_stamp[_nb[(f * 3) + k]] != stamp)
                {
                    _hf[ne] = f;
                    _he[ne] = k;
                    ne++;
                }
            }
        }

        if (ne < 3 || ne > MaxFaces - 2 || _nv >= MaxVertices)
        {
            return false;
        }

        // The horizon must be one loop: each start vertex once, each end vertex a start vertex.
        for (int i = 0; i < ne; i++)
        {
            int from = _fi[(_hf[i] * 3) + _he[i]];
            int to = _fi[(_hf[i] * 3) + ((_he[i] + 1) % 3)];
            bool next = false;
            for (int j = 0; j < ne; j++)
            {
                int fj = _fi[(_hf[j] * 3) + _he[j]];
                if (j != i && fj == from)
                {
                    return false;
                }

                if (fj == to)
                {
                    next = true;
                }
            }

            if (!next)
            {
                return false;
            }
        }

        // New faces must have area; check before changing anything.
        for (int i = 0; i < ne; i++)
        {
            Vector3F pu = _p[_fi[(_hf[i] * 3) + _he[i]]], pv = _p[_fi[(_hf[i] * 3) + ((_he[i] + 1) % 3)]];
            Vector3F cr = Vector3F.Cross(pv - pu, w - pu);
            if (!(cr.LengthSquared() > 1e-18f * scale2 * scale2))
            {
                return false;
            }
        }

        int wi = _nv;
        AddVertex(w, wa, wb);
        // Record the horizon edges' outer neighbours before the visible faces are released.
        for (int i = 0; i < ne; i++)
        {
            int f = _hf[i], k = _he[i];
            _newFaces[(i * 3)] = _fi[(f * 3) + k];
            _newFaces[(i * 3) + 1] = _fi[(f * 3) + ((k + 1) % 3)];
            _newFaces[(i * 3) + 2] = _nb[(f * 3) + k];
        }

        for (int i = 0; i < visCount; i++)
        {
            int f = _visList[i];
            _alive[f] = false;
            _free[_freeCount++] = f;
        }

        int[] created = _hf; // reuse: horizon faces are no longer needed
        for (int i = 0; i < ne; i++)
        {
            int u = _newFaces[i * 3], v = _newFaces[(i * 3) + 1], outer = _newFaces[(i * 3) + 2];
            int f = _freeCount > 0 ? _free[--_freeCount] : _slots++;
            _alive[f] = true;
            _fi[f * 3] = u;
            _fi[(f * 3) + 1] = v;
            _fi[(f * 3) + 2] = wi;
            Vector3F pu = _p[u], pv = _p[v];
            Vector3F fn = Vector3F.Cross(pv - pu, w - pu);
            fn /= ConvexShape3D.Sqrt(fn.LengthSquared());
            float fd = Vector3F.Dot(fn, pu);
            _fn[f] = fn;
            _fd[f] = fd < 0f ? 0f : fd; // the origin cannot be outside; negative values are round-off
            _nb[f * 3] = outer;
            for (int k = 0; k < 3; k++)
            {
                if (_fi[(outer * 3) + k] == v && _fi[(outer * 3) + ((k + 1) % 3)] == u)
                {
                    _nb[(outer * 3) + k] = f;
                }
            }

            created[i] = f;
        }

        for (int i = 0; i < ne; i++)
        {
            int f = created[i];
            int v = _fi[(f * 3) + 1], u = _fi[f * 3];
            for (int j = 0; j < ne; j++)
            {
                int g = created[j];
                if (_fi[g * 3] == v)
                {
                    _nb[(f * 3) + 1] = g; // edge v -> w meets g's edge w -> v
                }

                if (_fi[(g * 3) + 1] == u)
                {
                    _nb[(f * 3) + 2] = g; // edge w -> u meets g's edge u -> w
                }
            }
        }

        return true;
    }

    private void AddVertex(Vector3F p, Vector3F a, Vector3F b)
    {
        _p[_nv] = p;
        _a[_nv] = a;
        _b[_nv] = b;
        _nv++;
    }

    private void AddInitialFace(int i, int j, int k, Vector3F centroid)
    {
        Vector3F n = Vector3F.Cross(_p[j] - _p[i], _p[k] - _p[i]);
        float len = ConvexShape3D.Sqrt(n.LengthSquared());
        n /= len;
        if (Vector3F.Dot(n, _p[i] - centroid) < 0f)
        {
            n = -n;
            (j, k) = (k, j);
        }

        float d = Vector3F.Dot(n, _p[i]);
        int f = _slots++;
        _alive[f] = true;
        _fi[f * 3] = i;
        _fi[(f * 3) + 1] = j;
        _fi[(f * 3) + 2] = k;
        _fn[f] = n;
        _fd[f] = d < 0f ? 0f : d;
    }

    // Grows the GJK simplex (which contains the origin) into a non-degenerate tetrahedron by probing the support mapping.
    // Returns false when the Minkowski difference has no volume; the normal then is the best guess for the flat direction.
    private bool Expand(in ConvexPair pair, float scale, out Vector3F flatNormal)
    {
        flatNormal = default;
        float minDist = 1e-5f * scale;
        Vector3F[] axes = s_axes;
        while (_nv < 4)
        {
            bool added = false;
            if (_nv == 1)
            {
                for (int k = 0; k < 6 && !added; k++)
                {
                    pair.Support(axes[k], out Vector3F w, out Vector3F wa, out Vector3F wb);
                    if ((w - _p[0]).LengthSquared() > minDist * minDist)
                    {
                        AddVertex(w, wa, wb);
                        added = true;
                    }
                }
            }
            else if (_nv == 2)
            {
                Vector3F e = _p[1] - _p[0];
                // Perpendicular directions: the cross product with each axis, longest first (ties to the lower axis).
                Vector3F best = default;
                float bestLen = 0f;
                for (int k = 0; k < 3; k++)
                {
                    Vector3F c = Vector3F.Cross(e, axes[k * 2]);
                    float l = c.LengthSquared();
                    if (l > bestLen)
                    {
                        bestLen = l;
                        best = c;
                    }
                }

                for (int sgn = 0; sgn < 2 && !added; sgn++)
                {
                    Vector3F dir = sgn == 0 ? best : -best;
                    // Two probes around the axis: the direction and the one perpendicular to both.
                    for (int rot = 0; rot < 2 && !added; rot++)
                    {
                        Vector3F d2 = rot == 0 ? dir : Vector3F.Cross(e, dir);
                        pair.Support(d2, out Vector3F w, out Vector3F wa, out Vector3F wb);
                        Vector3F cr = Vector3F.Cross(e, w - _p[0]);
                        if (cr.LengthSquared() > minDist * minDist * e.LengthSquared())
                        {
                            AddVertex(w, wa, wb);
                            added = true;
                        }
                    }
                }

                if (!added)
                {
                    flatNormal = best.LengthSquared() > 0f ? best.Normalized() : default;
                }
            }
            else
            {
                Vector3F n = Vector3F.Cross(_p[1] - _p[0], _p[2] - _p[0]);
                float nl = ConvexShape3D.Sqrt(n.LengthSquared());
                if (!(nl > 0f))
                {
                    return false;
                }

                n /= nl;
                for (int sgn = 0; sgn < 2 && !added; sgn++)
                {
                    Vector3F dir = sgn == 0 ? n : -n;
                    pair.Support(dir, out Vector3F w, out Vector3F wa, out Vector3F wb);
                    if (Math.Abs(Vector3F.Dot(n, w - _p[0])) > minDist)
                    {
                        AddVertex(w, wa, wb);
                        added = true;
                    }
                }

                if (!added)
                {
                    flatNormal = n;
                }
            }

            if (!added)
            {
                return false;
            }
        }

        return true;
    }
}
