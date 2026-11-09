using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>An edge of a soft body broke at the end of a step (see <see cref="SoftBodyDef.TearStrain"/> and <see cref="SoftBodyDef.TearForce"/>).</summary>
[Experimental("OPHYS0002")]
public readonly struct SoftTearEvent
{
    internal SoftTearEvent(SoftBody body, int a, int b, int particleCount)
    {
        Body = body; ParticleA = a; ParticleB = b; ParticleCount = particleCount;
    }

    /// <summary>The body that tore. It stays one body: pieces that come apart are separate parts of one particle set.</summary>
    public SoftBody Body { get; }

    /// <summary>First particle of the broken edge (its number before any split of the step; the smaller of the two).</summary>
    public int ParticleA { get; }

    /// <summary>Second particle of the broken edge (the larger).</summary>
    public int ParticleB { get; }

    /// <summary>The body's particle count after the step's tears. Particles the tear split off are numbered from the old count upwards.</summary>
    public int ParticleCount { get; }
}

internal sealed partial class Engine3D
{
    // Surface contacts (Bridson, Fedkiw, Anderson 2002, robust treatment of collisions for cloth; Harmon et al. 2008 for the pair types; Macklin et al. 2014 for contacts as position constraints): a particle against
    // a surface triangle, and an edge against an edge. The candidate pairs are listed once per step with the side the pair starts on; every substep and pass the pair is a constraint that keeps that side at the
    // distance of the two bodies' particle radii, wherever the closest features lie inside the triangle or the two edges (outside them the spheres of the particles hold the contact).

    private readonly SoftCellGrid _triGrid = new(), _edgeGrid = new();
    private int[] _vtP = new int[64], _vtT = new int[64];
    private float[] _vtS = new float[64];
    private int _vtCount;
    private int[] _eeA = new int[64], _eeB = new int[64];
    private float[] _eeS = new float[64];
    private int _eeCount;
    private SoftBodyData[] _gtBody = new SoftBodyData[64];
    private int[] _gtLocal = new int[64];
    private double[] _gtBox = new double[384];
    private SoftBodyData[] _geBody = new SoftBodyData[64];
    private int[] _geLocal = new int[64];
    private double[] _geBox = new double[384];
    private readonly List<SoftTearEvent> _softTears = new();

    /// <summary>The tears of the last step, in body and edge order.</summary>
    public ReadOnlySpan<SoftTearEvent> SoftTearEvents => System.Runtime.InteropServices.CollectionsMarshal.AsSpan(_softTears);

    private const float InsideTolerance = 0.05f;

    // Lists the surface pairs for one substep, from the positions it starts at, and the travel of each body in that substep.
    private void BuildSurfacePairs(float h, float gmag)
    {
        _vtCount = 0; _eeCount = 0;
        int bodies = 0;
        bool self = false;
        for (int a = 0; a < _pairSoftCount; a++)
        {
            var d = _awakeSoft[a];
            d.SurfTriBase = -1;
            // How far the body can carry a feature in the step: its fastest particle's travel, the fall under gravity and the speculative distance. A sleeping (passive) body does not move.
            d.SurfTravel = (a < _awakeSoftCount ? MaxParticleSpeed(d) * h + 0.5f * gmag * h * h : 0f);
            if (!d.Def.SurfaceCollision || d.SurfTris.Length == 0) { continue; }
            bodies++;
            self |= d.Def.SelfCollision;
        }

        if (bodies == 0 || (bodies < 2 && !self)) { return; }
        int nt = 0, ne = 0;
        for (int a = 0; a < _pairSoftCount; a++)
        {
            var d = _awakeSoft[a];
            if (!d.Def.SurfaceCollision || d.SurfTris.Length == 0) { continue; }
            d.SurfTriBase = nt; d.SurfEdgeBase = ne;
            nt += d.SurfTris.Length / 3; ne += d.SurfEdgeA.Length;
        }

        if (nt >= SoftCellGrid.MaxItems || ne >= SoftCellGrid.MaxItems) { return; }
        GrowTo(ref _gtBody, nt); GrowTo(ref _gtLocal, nt); GrowTo(ref _gtBox, 6 * nt);
        GrowTo(ref _geBody, ne); GrowTo(ref _geLocal, ne); GrowTo(ref _geBox, 6 * ne);
        _triGrid.Grow(nt); _edgeGrid.Grow(ne);
        // A feature's box is grown by its body's radius and twice its travel: two features can come within the sum of their radii during the step only if their grown boxes overlap.
        double extT = 0.0, extE = 0.0, extP = 0.0;
        for (int a = 0; a < _pairSoftCount; a++)
        {
            var d = _awakeSoft[a];
            if (d.SurfTriBase < 0) { continue; }
            double grow = d.Radius + 2.0 * d.SurfTravel;
            if (grow > extP) { extP = grow; }
            for (int i = 0; i < d.SurfTris.Length / 3; i++)
            {
                int g = d.SurfTriBase + i, o = 6 * g;
                double ext = BoxOf(d, d.SurfTris[3 * i], d.SurfTris[3 * i + 1], d.SurfTris[3 * i + 2], grow, _gtBox, o);
                _gtBody[g] = d; _gtLocal[g] = i;
                if (ext > extT) { extT = ext; }
            }

            for (int i = 0; i < d.SurfEdgeA.Length; i++)
            {
                int g = d.SurfEdgeBase + i, o = 6 * g;
                double ext = BoxOf(d, d.SurfEdgeA[i], d.SurfEdgeB[i], d.SurfEdgeB[i], grow, _geBox, o);
                _geBody[g] = d; _geLocal[g] = i;
                if (ext > extE) { extE = ext; }
            }
        }

        double cst = Math.Max(extT, 1e-3), icT = 1.0 / cst, cse = Math.Max(extE, 1e-3), icE = 1.0 / cse;
        var tk = _triGrid.Keys;
        for (int g = 0; g < nt; g++)
        {
            int o = 6 * g;
            tk[g] = (SoftCellGrid.Hash((long)Math.Floor(_gtBox[o] * icT), (long)Math.Floor(_gtBox[o + 1] * icT), (long)Math.Floor(_gtBox[o + 2] * icT)) << 24) | (long)g;
        }

        _triGrid.Build(nt);
        var ek = _edgeGrid.Keys;
        for (int g = 0; g < ne; g++)
        {
            int o = 6 * g;
            ek[g] = (SoftCellGrid.Hash((long)Math.Floor(_geBox[o] * icE), (long)Math.Floor(_geBox[o + 1] * icE), (long)Math.Floor(_geBox[o + 2] * icE)) << 24) | (long)g;
        }

        _edgeGrid.Build(ne);

        // Particle against triangle: the particle, grown like a feature of its body, looks at every cell that can hold the minimum corner of a box that meets its own.
        Span<long> seen = stackalloc long[64];
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            var dp = _awakeSoft[a];
            if (dp.SurfTriBase < 0) { continue; }
            double ep = dp.Radius + 2.0 * dp.SurfTravel;
            for (int pi = 0; pi < dp.SurfParticles.Length; pi++)
            {
                int p = dp.SurfParticles[pi];
                double px = dp.Ox + dp.X[3 * p], py = dp.Oy + dp.X[3 * p + 1], pz = dp.Oz + dp.X[3 * p + 2];
                long x0 = (long)Math.Floor((px - ep) * icT) - 1, x1 = (long)Math.Floor((px + ep) * icT);
                long y0 = (long)Math.Floor((py - ep) * icT) - 1, y1 = (long)Math.Floor((py + ep) * icT);
                long z0 = (long)Math.Floor((pz - ep) * icT) - 1, z1 = (long)Math.Floor((pz + ep) * icT);
                int nseen = 0;
                for (long cz = z0; cz <= z1; cz++)
                {
                    for (long cy = y0; cy <= y1; cy++)
                    {
                        for (long cx = x0; cx <= x1; cx++)
                        {
                            long hsh = SoftCellGrid.Hash(cx, cy, cz);
                            bool dup = false;
                            for (int u = 0; u < nseen; u++) { if (seen[u] == hsh) { dup = true; break; } }
                            if (dup) { continue; }
                            if (nseen < seen.Length) { seen[nseen++] = hsh; }
                            if (!_triGrid.Find(hsh, out int s0, out int s1)) { continue; }
                            for (int k = s0; k < s1; k++)
                            {
                                int g = (int)(tk[k] & 0xFFFFFF);
                                int o = 6 * g;
                                if (px + ep < _gtBox[o] || px - ep > _gtBox[o + 3] || py + ep < _gtBox[o + 1] || py - ep > _gtBox[o + 4] || pz + ep < _gtBox[o + 2] || pz - ep > _gtBox[o + 5]) { continue; }
                                TryAddVertexTriangle(dp, p, px, py, pz, g);
                            }
                        }
                    }
                }
            }
        }

        // Edge against edge, each pair once: the edge looks at every cell that can hold the minimum corner of a box that meets its own.
        for (int g = 0; g < ne; g++)
        {
            int o = 6 * g;
            long x0 = (long)Math.Floor(_geBox[o] * icE) - 1, x1 = (long)Math.Floor(_geBox[o + 3] * icE);
            long y0 = (long)Math.Floor(_geBox[o + 1] * icE) - 1, y1 = (long)Math.Floor(_geBox[o + 4] * icE);
            long z0 = (long)Math.Floor(_geBox[o + 2] * icE) - 1, z1 = (long)Math.Floor(_geBox[o + 5] * icE);
            int nseen = 0;
            for (long cz = z0; cz <= z1; cz++)
            {
                for (long cy = y0; cy <= y1; cy++)
                {
                    for (long cx = x0; cx <= x1; cx++)
                    {
                        long hsh = SoftCellGrid.Hash(cx, cy, cz);
                        bool dup = false;
                        for (int u = 0; u < nseen; u++) { if (seen[u] == hsh) { dup = true; break; } }
                        if (dup) { continue; }
                        if (nseen < seen.Length) { seen[nseen++] = hsh; }
                        if (!_edgeGrid.Find(hsh, out int s0, out int s1)) { continue; }
                        for (int k = s0; k < s1; k++)
                        {
                            int h2 = (int)(ek[k] & 0xFFFFFF);
                            if (h2 <= g) { continue; }
                            int o2 = 6 * h2;
                            if (_geBox[o2] > _geBox[o + 3] || _geBox[o2 + 3] < _geBox[o] || _geBox[o2 + 1] > _geBox[o + 4] || _geBox[o2 + 4] < _geBox[o + 1] || _geBox[o2 + 2] > _geBox[o + 5] || _geBox[o2 + 5] < _geBox[o + 2]) { continue; }
                            TryAddEdgeEdge(g, h2);
                        }
                    }
                }
            }
        }
    }

    private static float MaxParticleSpeed(SoftBodyData d)
    {
        float m2 = 0f;
        var v = d.Vel;
        for (int i = 0; i < d.N; i++)
        {
            float v2 = v[3 * i] * v[3 * i] + v[3 * i + 1] * v[3 * i + 1] + v[3 * i + 2] * v[3 * i + 2];
            if (v2 > m2) { m2 = v2; }
        }

        return float.Sqrt(m2);
    }

    // The box of a triangle (or an edge, given its end twice) at the start of the step in world coordinates, grown by `grow`; returns its largest extent.
    private static double BoxOf(SoftBodyData d, int a, int b, int c, double grow, double[] box, int o)
    {
        double x0 = double.MaxValue, y0 = double.MaxValue, z0 = double.MaxValue, x1 = double.MinValue, y1 = double.MinValue, z1 = double.MinValue;
        for (int k = 0; k < 3; k++)
        {
            int p = 3 * (k == 0 ? a : (k == 1 ? b : c));
            double x = d.Ox + d.X[p], y = d.Oy + d.X[p + 1], z = d.Oz + d.X[p + 2];
            if (x < x0) { x0 = x; } if (x > x1) { x1 = x; }
            if (y < y0) { y0 = y; } if (y > y1) { y1 = y; }
            if (z < z0) { z0 = z; } if (z > z1) { z1 = z; }
        }

        box[o] = x0 - grow; box[o + 1] = y0 - grow; box[o + 2] = z0 - grow; box[o + 3] = x1 + grow; box[o + 4] = y1 + grow; box[o + 5] = z1 + grow;
        return Math.Max(x1 - x0, Math.Max(y1 - y0, z1 - z0)) + 2.0 * grow;
    }

    private void TryAddVertexTriangle(SoftBodyData dp, int p, double px, double py, double pz, int g)
    {
        var dt = _gtBody[g];
        int el = _gtLocal[g];
        int a = dt.SurfTris[3 * el], b = dt.SurfTris[3 * el + 1], c = dt.SurfTris[3 * el + 2];
        float thick = dp.Radius + dt.Radius;
        if (dp == dt)
        {
            if (!dp.Def.SelfCollision || p == a || p == b || p == c) { return; }
            float near = 2.1f * thick;
            if (dp.RestDistance(p, a) < near || dp.RestDistance(p, b) < near || dp.RestDistance(p, c) < near) { return; }
        }
        else if (!SoftPairAllowed(dp.Def, dt.Def)) { return; }

        if (dp.InvMass[p] == 0f && dt.InvMass[a] == 0f && dt.InvMass[b] == 0f && dt.InvMass[c] == 0f) { return; }
        // The exact distance at the start of the step against what the two bodies can close in it.
        double reach = thick + 2.0 * (dp.SurfTravel + dt.SurfTravel);
        double ax = dt.Ox + dt.X[3 * a], ay = dt.Oy + dt.X[3 * a + 1], az = dt.Oz + dt.X[3 * a + 2];
        double bx = dt.Ox + dt.X[3 * b], by = dt.Oy + dt.X[3 * b + 1], bz = dt.Oz + dt.X[3 * b + 2];
        double cx = dt.Ox + dt.X[3 * c], cy = dt.Oy + dt.X[3 * c + 1], cz = dt.Oz + dt.X[3 * c + 2];
        if (SoftGeometry.PointTriangleSq(px, py, pz, ax, ay, az, bx, by, bz, cx, cy, cz) >= reach * reach) { return; }
        bool inside = VertexTriangle(dp, p, dt, a, b, c, out _, out _, out _, out float dist, out _, out _, out _);
        float side = inside ? (dist >= 0f ? 1f : -1f) : 0f;
        GrowTo(ref _vtP, _vtCount + 1); GrowTo(ref _vtT, _vtCount + 1); GrowTo(ref _vtS, _vtCount + 1);
        _vtP[_vtCount] = dp.Base + p; _vtT[_vtCount] = g; _vtS[_vtCount] = side;
        _vtCount++;
    }

    private void TryAddEdgeEdge(int g1, int g2)
    {
        var d1 = _geBody[g1]; var d2 = _geBody[g2];
        int l1 = _geLocal[g1], l2 = _geLocal[g2];
        int a = d1.SurfEdgeA[l1], b = d1.SurfEdgeB[l1], c = d2.SurfEdgeA[l2], d = d2.SurfEdgeB[l2];
        float thick = d1.Radius + d2.Radius;
        if (d1 == d2)
        {
            if (!d1.Def.SelfCollision || a == c || a == d || b == c || b == d) { return; }
            float near = 2.1f * thick;
            if (d1.RestDistance(a, c) < near || d1.RestDistance(a, d) < near || d1.RestDistance(b, c) < near || d1.RestDistance(b, d) < near) { return; }
        }
        else if (!SoftPairAllowed(d1.Def, d2.Def)) { return; }

        if (d1.InvMass[a] == 0f && d1.InvMass[b] == 0f && d2.InvMass[c] == 0f && d2.InvMass[d] == 0f) { return; }
        double reach = thick + 2.0 * (d1.SurfTravel + d2.SurfTravel);
        double ax = d1.Ox + d1.X[3 * a], ay = d1.Oy + d1.X[3 * a + 1], az = d1.Oz + d1.X[3 * a + 2];
        double bx = d1.Ox + d1.X[3 * b], by = d1.Oy + d1.X[3 * b + 1], bz = d1.Oz + d1.X[3 * b + 2];
        double cx = d2.Ox + d2.X[3 * c], cy = d2.Oy + d2.X[3 * c + 1], cz = d2.Oz + d2.X[3 * c + 2];
        double dx = d2.Ox + d2.X[3 * d], dy = d2.Oy + d2.X[3 * d + 1], dz = d2.Oz + d2.X[3 * d + 2];
        if (SoftGeometry.SegmentSegmentSq(ax, ay, az, bx, by, bz, cx, cy, cz, dx, dy, dz) >= reach * reach) { return; }
        bool inside = EdgeEdge(d1, a, b, d2, c, d, out _, out _, out _, out float dist, out _, out _);
        float side = inside ? (dist >= 0f ? 1f : -1f) : 0f;
        GrowTo(ref _eeA, _eeCount + 1); GrowTo(ref _eeB, _eeCount + 1); GrowTo(ref _eeS, _eeCount + 1);
        _eeA[_eeCount] = g1; _eeB[_eeCount] = g2; _eeS[_eeCount] = side;
        _eeCount++;
    }

    // The particle's projection on the triangle's plane: unit normal (right-handed around the triangle), signed distance of the particle along it, barycentric coordinates of the projection.
    // True when the projection is inside the triangle (within a small tolerance).
    private static bool VertexTriangle(SoftBodyData dp, int p, SoftBodyData dt, int a, int b, int c, out float nx, out float ny, out float nz, out float dist, out float ba, out float bb, out float bc)
    {
        nx = ny = nz = dist = ba = bb = bc = 0f;
        int kp = 3 * p;
        float ax = SoftDelta(dt, dp, 3 * a, kp, 0), ay = SoftDelta(dt, dp, 3 * a, kp, 1), az = SoftDelta(dt, dp, 3 * a, kp, 2);
        float e1x = SoftDelta(dt, dp, 3 * b, kp, 0) - ax, e1y = SoftDelta(dt, dp, 3 * b, kp, 1) - ay, e1z = SoftDelta(dt, dp, 3 * b, kp, 2) - az;
        float e2x = SoftDelta(dt, dp, 3 * c, kp, 0) - ax, e2y = SoftDelta(dt, dp, 3 * c, kp, 1) - ay, e2z = SoftDelta(dt, dp, 3 * c, kp, 2) - az;
        nx = e1y * e2z - e1z * e2y; ny = e1z * e2x - e1x * e2z; nz = e1x * e2y - e1y * e2x;
        float nl2 = nx * nx + ny * ny + nz * nz;
        if (!(nl2 > 1e-24f)) { return false; }
        float inl = 1f / float.Sqrt(nl2);
        nx *= inl; ny *= inl; nz *= inl;
        float wx = -ax, wy = -ay, wz = -az;
        dist = nx * wx + ny * wy + nz * wz;
        float d00 = e1x * e1x + e1y * e1y + e1z * e1z, d01 = e1x * e2x + e1y * e2y + e1z * e2z, d11 = e2x * e2x + e2y * e2y + e2z * e2z;
        float d20 = wx * e1x + wy * e1y + wz * e1z, d21 = wx * e2x + wy * e2y + wz * e2z;
        float den = d00 * d11 - d01 * d01;
        if (!(den > 0f)) { return false; }
        float v = (d11 * d20 - d01 * d21) / den, w2 = (d00 * d21 - d01 * d20) / den;
        ba = 1f - v - w2; bb = v; bc = w2;
        return ba >= -InsideTolerance && bb >= -InsideTolerance && bc >= -InsideTolerance;
    }

    // The closest points of two edges (a,b) and (c,d), when both lie inside the edges: unit direction n of d1 x d2, signed distance from the second point to the first along it, parameters s and t.
    private static bool EdgeEdge(SoftBodyData d1, int a, int b, SoftBodyData d2, int c, int d, out float nx, out float ny, out float nz, out float dist, out float s, out float t)
    {
        nx = ny = nz = dist = s = t = 0f;
        int ka = 3 * a;
        float e1x = SoftDelta(d1, d1, 3 * b, ka, 0), e1y = SoftDelta(d1, d1, 3 * b, ka, 1), e1z = SoftDelta(d1, d1, 3 * b, ka, 2);
        float cx = SoftDelta(d2, d1, 3 * c, ka, 0), cy = SoftDelta(d2, d1, 3 * c, ka, 1), cz = SoftDelta(d2, d1, 3 * c, ka, 2);
        float e2x = SoftDelta(d2, d1, 3 * d, ka, 0) - cx, e2y = SoftDelta(d2, d1, 3 * d, ka, 1) - cy, e2z = SoftDelta(d2, d1, 3 * d, ka, 2) - cz;
        float rx = -cx, ry = -cy, rz = -cz;
        float aa = e1x * e1x + e1y * e1y + e1z * e1z, ee = e2x * e2x + e2y * e2y + e2z * e2z;
        float ff = e2x * rx + e2y * ry + e2z * rz, c1 = e1x * rx + e1y * ry + e1z * rz, bb = e1x * e2x + e1y * e2y + e1z * e2z;
        float den = aa * ee - bb * bb;
        if (!(aa > 0f) || !(ee > 0f) || !(den > 1e-9f * aa * ee)) { return false; }
        s = (bb * ff - c1 * ee) / den;
        if (!(s > 0f && s < 1f)) { return false; }
        t = (bb * s + ff) / ee;
        if (!(t > 0f && t < 1f)) { return false; }
        nx = e1y * e2z - e1z * e2y; ny = e1z * e2x - e1x * e2z; nz = e1x * e2y - e1y * e2x;
        float nl = float.Sqrt(nx * nx + ny * ny + nz * nz);
        if (!(nl > 0f)) { return false; }
        float inl = 1f / nl;
        nx *= inl; ny *= inl; nz *= inl;
        float qx = rx + s * e1x - t * e2x, qy = ry + s * e1y - t * e2y, qz = rz + s * e1z - t * e2z;
        dist = nx * qx + ny * qy + nz * qz;
        return true;
    }

    private void SolveSurfacePairs()
    {
        for (int k = 0; k < _vtCount; k++) { SolveVertexTriangle(k); }
        for (int k = 0; k < _eeCount; k++) { SolveEdgeEdge(k); }
    }

    private void SolveVertexTriangle(int k)
    {
        int g = _vtP[k];
        var dp = _gData[g];
        int p = _gPart[g];
        int gt = _vtT[k];
        var dt = _gtBody[gt];
        int el = _gtLocal[gt];
        int a = dt.SurfTris[3 * el], b = dt.SurfTris[3 * el + 1], c = dt.SurfTris[3 * el + 2];
        if (!VertexTriangle(dp, p, dt, a, b, c, out float nx, out float ny, out float nz, out float dist, out float ba, out float bb, out float bc)) { return; }
        float s = _vtS[k];
        if (s == 0f) { s = dist >= 0f ? 1f : -1f; _vtS[k] = s; }
        float thick = dp.Radius + dt.Radius;
        float cv = s * dist - thick;
        if (!(cv < 0f)) { return; }
        // A particle that is already well behind the triangle is brought back by at most the contact thickness per pass, so a wrong side never launches it.
        if (cv < -thick) { cv = -thick; }
        float wp = dp.InvMass[p], wa = dt.InvMass[a], wb = dt.InvMass[b], wc = dt.InvMass[c];
        float w = wp + ba * ba * wa + bb * bb * wb + bc * bc * wc;
        if (!(w > 0f)) { return; }
        float lam = -cv / w;
        float sx = s * nx, sy = s * ny, sz = s * nz;
        int kp = 3 * p, ka = 3 * a, kb = 3 * b, kc = 3 * c;
        float f = wp * lam;
        dp.X[kp] += sx * f; dp.X[kp + 1] += sy * f; dp.X[kp + 2] += sz * f;
        f = ba * wa * lam; dt.X[ka] -= sx * f; dt.X[ka + 1] -= sy * f; dt.X[ka + 2] -= sz * f;
        f = bb * wb * lam; dt.X[kb] -= sx * f; dt.X[kb + 1] -= sy * f; dt.X[kb + 2] -= sz * f;
        f = bc * wc * lam; dt.X[kc] -= sx * f; dt.X[kc + 1] -= sy * f; dt.X[kc + 2] -= sz * f;
        float mu = float.Sqrt(dp.Def.Friction * dt.Def.Friction);
        if (!(mu > 0f)) { return; }
        // Coulomb friction as a position correction: the part of the particle's displacement relative to the point of the triangle under it that lies in the triangle's plane is removed, up to mu times the normal correction.
        float ux = (dp.X[kp] - dp.XP[kp]) - (ba * (dt.X[ka] - dt.XP[ka]) + bb * (dt.X[kb] - dt.XP[kb]) + bc * (dt.X[kc] - dt.XP[kc]));
        float uy = (dp.X[kp + 1] - dp.XP[kp + 1]) - (ba * (dt.X[ka + 1] - dt.XP[ka + 1]) + bb * (dt.X[kb + 1] - dt.XP[kb + 1]) + bc * (dt.X[kc + 1] - dt.XP[kc + 1]));
        float uz = (dp.X[kp + 2] - dp.XP[kp + 2]) - (ba * (dt.X[ka + 2] - dt.XP[ka + 2]) + bb * (dt.X[kb + 2] - dt.XP[kb + 2]) + bc * (dt.X[kc + 2] - dt.XP[kc + 2]));
        float un = ux * nx + uy * ny + uz * nz;
        float tx = ux - nx * un, ty = uy - ny * un, tz = uz - nz * un;
        float len = float.Sqrt(tx * tx + ty * ty + tz * tz);
        if (!(len > 1e-12f)) { return; }
        float il = 1f / len;
        tx *= il; ty *= il; tz *= il;
        float lt = len / w;
        float cap = mu * lam;
        if (lt > cap) { lt = cap; }
        f = wp * lt; dp.X[kp] -= tx * f; dp.X[kp + 1] -= ty * f; dp.X[kp + 2] -= tz * f;
        f = ba * wa * lt; dt.X[ka] += tx * f; dt.X[ka + 1] += ty * f; dt.X[ka + 2] += tz * f;
        f = bb * wb * lt; dt.X[kb] += tx * f; dt.X[kb + 1] += ty * f; dt.X[kb + 2] += tz * f;
        f = bc * wc * lt; dt.X[kc] += tx * f; dt.X[kc + 1] += ty * f; dt.X[kc + 2] += tz * f;
    }

    private void SolveEdgeEdge(int k)
    {
        int g1 = _eeA[k], g2 = _eeB[k];
        var d1 = _geBody[g1]; var d2 = _geBody[g2];
        int l1 = _geLocal[g1], l2 = _geLocal[g2];
        int a = d1.SurfEdgeA[l1], b = d1.SurfEdgeB[l1], c = d2.SurfEdgeA[l2], d = d2.SurfEdgeB[l2];
        if (!EdgeEdge(d1, a, b, d2, c, d, out float nx, out float ny, out float nz, out float dist, out float s, out float t)) { return; }
        float sg = _eeS[k];
        if (sg == 0f) { sg = dist >= 0f ? 1f : -1f; _eeS[k] = sg; }
        float thick = d1.Radius + d2.Radius;
        float cv = sg * dist - thick;
        if (!(cv < 0f)) { return; }
        if (cv < -thick) { cv = -thick; }
        float wa = d1.InvMass[a], wb = d1.InvMass[b], wc = d2.InvMass[c], wd = d2.InvMass[d];
        float ca = 1f - s, cb = s, cc = 1f - t, cd = t;
        float w = ca * ca * wa + cb * cb * wb + cc * cc * wc + cd * cd * wd;
        if (!(w > 0f)) { return; }
        float lam = -cv / w;
        float sx = sg * nx, sy = sg * ny, sz = sg * nz;
        int ka = 3 * a, kb = 3 * b, kc = 3 * c, kd = 3 * d;
        float f = ca * wa * lam; d1.X[ka] += sx * f; d1.X[ka + 1] += sy * f; d1.X[ka + 2] += sz * f;
        f = cb * wb * lam; d1.X[kb] += sx * f; d1.X[kb + 1] += sy * f; d1.X[kb + 2] += sz * f;
        f = cc * wc * lam; d2.X[kc] -= sx * f; d2.X[kc + 1] -= sy * f; d2.X[kc + 2] -= sz * f;
        f = cd * wd * lam; d2.X[kd] -= sx * f; d2.X[kd + 1] -= sy * f; d2.X[kd + 2] -= sz * f;
        float mu = float.Sqrt(d1.Def.Friction * d2.Def.Friction);
        if (!(mu > 0f)) { return; }
        float ux = (ca * (d1.X[ka] - d1.XP[ka]) + cb * (d1.X[kb] - d1.XP[kb])) - (cc * (d2.X[kc] - d2.XP[kc]) + cd * (d2.X[kd] - d2.XP[kd]));
        float uy = (ca * (d1.X[ka + 1] - d1.XP[ka + 1]) + cb * (d1.X[kb + 1] - d1.XP[kb + 1])) - (cc * (d2.X[kc + 1] - d2.XP[kc + 1]) + cd * (d2.X[kd + 1] - d2.XP[kd + 1]));
        float uz = (ca * (d1.X[ka + 2] - d1.XP[ka + 2]) + cb * (d1.X[kb + 2] - d1.XP[kb + 2])) - (cc * (d2.X[kc + 2] - d2.XP[kc + 2]) + cd * (d2.X[kd + 2] - d2.XP[kd + 2]));
        float un = ux * nx + uy * ny + uz * nz;
        float tx = ux - nx * un, ty = uy - ny * un, tz = uz - nz * un;
        float len = float.Sqrt(tx * tx + ty * ty + tz * tz);
        if (!(len > 1e-12f)) { return; }
        float il = 1f / len;
        tx *= il; ty *= il; tz *= il;
        float lt = len / w;
        float cap = mu * lam;
        if (lt > cap) { lt = cap; }
        f = ca * wa * lt; d1.X[ka] -= tx * f; d1.X[ka + 1] -= ty * f; d1.X[ka + 2] -= tz * f;
        f = cb * wb * lt; d1.X[kb] -= tx * f; d1.X[kb + 1] -= ty * f; d1.X[kb + 2] -= tz * f;
        f = cc * wc * lt; d2.X[kc] += tx * f; d2.X[kc + 1] += ty * f; d2.X[kc + 2] += tz * f;
        f = cd * wd * lt; d2.X[kd] += tx * f; d2.X[kd + 1] += ty * f; d2.X[kd + 2] += tz * f;
    }

    // ------------------------------------------------------------------ the net speed that decides rest

    // The motion of the step as a whole: the substeps' last velocity can swing back and forth around a position that barely moves, so rest is judged from the net displacement of the step,
    // smoothed over about ten steps. The mass-weighted mean square speed of the free particles.
    private const float SmoothK = 0.1f;
    private const float MaxNetSpeed2 = 1e30f;

    private static void NetSpeed(SoftBodyData d, float invDt)
    {
        double ke = 0.0, mass = 0.0;
        for (int i = 0; i < d.N; i++)
        {
            if (d.InvMass[i] == 0f) { continue; }
            float dx = (d.X[3 * i] - d.X0[3 * i]) * invDt, dy = (d.X[3 * i + 1] - d.X0[3 * i + 1]) * invDt, dz = (d.X[3 * i + 2] - d.X0[3 * i + 2]) * invDt;
            double mi = 1.0 / d.InvMass[i];
            ke += mi * (dx * dx + dy * dy + dz * dz); mass += mi;
        }

        float now = mass > 0.0 ? (float)(ke / mass) : 0f;
        // A diverged body (a particle at infinity or not a number) reads as moving as fast as the format allows, so the value stays finite and a snapshot of it restores.
        if (!(now <= MaxNetSpeed2)) { now = MaxNetSpeed2; }
        d.StepSpeed2 += (now - d.StepSpeed2) * SmoothK;
    }

    // ------------------------------------------------------------------ plasticity and tearing at the end of a step

    private long[]?[] _tearKeys = new long[]?[4];

    private void DetectSoftTears(float h)
    {
        GrowTo(ref _tearKeys, _awakeSoftCount);
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            var d = _awakeSoft[a];
            _tearKeys[a] = d.Def.PlasticYield > 0f || d.Def.TearStrain > 0f || d.Def.TearForce > 0f ? PlasticAndTears(d, h) : null;
        }
    }

    private static long[]? PlasticAndTears(SoftBodyData d, float h)
    {
        var keys = d.DetectTears(h);
        d.ApplyPlasticity();
        return keys.Length == 0 ? null : keys;
    }

    private void ApplySoftTears()
    {
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            var keys = _tearKeys[a];
            if (keys == null) { continue; }
            _tearKeys[a] = null;
            var d = _awakeSoft[a];
            // A tear can add at most one particle per element corner; stop tearing before the particle numbering runs out.
            if ((long)_softParticles + d.Tets.Length + d.Tris.Length > MaxSoftParticles) { continue; }
            int added = d.Tear(keys);
            int slot = Array.IndexOf(SoftData, d);
            var handle = new SoftBody(World!, slot, SoftHandles.GenerationOf(slot));
            foreach (long key in keys) { _softTears.Add(new SoftTearEvent(handle, (int)(key >> 20), (int)(key & 0xFFFFF), d.N)); }
            _softParticles += added;
            d.SleepTime = 0f;
            d.HadMovingContact = true;
        }
    }
}
