using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>An edge of a soft body broke at the end of a step (see <see cref="SoftBodyDef.TearStrain"/> and <see cref="SoftBodyDef.TearForce"/>).</summary>
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

internal sealed partial class Engine2D
{
    // Surface contacts: a particle against the boundary edges of the soft bodies (Bridson, Fedkiw, Anderson 2002 for the side that is kept; Macklin et al. 2014 for contacts as position constraints). The edges of each body are
    // directed with the material on the left. Candidate (particle, edge) pairs are listed once per step with the side of the edge the particle starts on; every substep the pair is a constraint that keeps the particle on that
    // side, at the distance of the two bodies' particle radii, wherever the particle projects onto the edge.

    private readonly SoftCellGrid _edgeGrid = new();
    private int[] _sePG = new int[64], _seEG = new int[64];
    private float[] _seS = new float[64];
    private int _seCount;
    private SoftBodyData[] _geBody = new SoftBodyData[64];
    private int[] _geLocal = new int[64];
    private double[] _geBox = new double[256];
    private readonly List<SoftTearEvent> _softTears = new();

    /// <summary>The tears of the last step, in body and edge order.</summary>
    public ReadOnlySpan<SoftTearEvent> SoftTearEvents => System.Runtime.InteropServices.CollectionsMarshal.AsSpan(_softTears);

    // Lists the surface pairs for one substep, from the positions it starts at, with the travel of the bodies in that substep.
    private void BuildSurfacePairs(float h, float gmag)
    {
        _seCount = 0;
        int bodies = 0;
        bool self = false;
        for (int a = 0; a < _pairSoftCount; a++)
        {
            var d = _awakeSoft[a];
            d.SurfBase = -1;
            d.SurfTravel = 0f;
            if (!d.Def.SurfaceCollision || d.SurfA.Length == 0) { continue; }
            bodies++;
            self |= d.Def.SelfCollision;
        }

        if (bodies == 0 || (bodies < 2 && !self)) { return; }
        for (int a = 0; a < _pairSoftCount; a++)
        {
            var d = _awakeSoft[a];
            if (!d.Def.SurfaceCollision || d.SurfA.Length == 0) { continue; }
            // How far the body can carry a feature in the substep: its fastest particle's travel and the fall under gravity. A sleeping (passive) body does not move.
            d.SurfTravel = (a < _awakeSoftCount ? MaxParticleSpeed(d) * h + 0.5f * gmag * h * h : 0f);
        }

        int ne = 0;
        for (int a = 0; a < _pairSoftCount; a++)
        {
            var d = _awakeSoft[a];
            if (!d.Def.SurfaceCollision || d.SurfA.Length == 0) { continue; }
            d.SurfBase = ne;
            ne += d.SurfA.Length;
        }

        if (ne >= SoftCellGrid.MaxItems) { return; }
        GrowTo(ref _geBody, ne); GrowTo(ref _geLocal, ne); GrowTo(ref _geBox, 4 * ne);
        _edgeGrid.Grow(ne);
        // An edge's box is grown by its body's radius and twice its travel: a particle and an edge can come within the sum of their radii during the substep only if the particle, grown the same way, meets the box.
        double maxExt = 0.0;
        for (int a = 0; a < _pairSoftCount; a++)
        {
            var d = _awakeSoft[a];
            if (d.SurfBase < 0) { continue; }
            double grow = d.Radius + 2.0 * d.SurfTravel;
            for (int i = 0; i < d.SurfA.Length; i++)
            {
                int ge = d.SurfBase + i, ka = 2 * d.SurfA[i], kb = 2 * d.SurfB[i];
                double ax = d.Ox + d.X[ka], ay = d.Oy + d.X[ka + 1], bx = d.Ox + d.X[kb], by = d.Oy + d.X[kb + 1];
                double x0 = Math.Min(ax, bx) - grow, y0 = Math.Min(ay, by) - grow, x1 = Math.Max(ax, bx) + grow, y1 = Math.Max(ay, by) + grow;
                _geBody[ge] = d; _geLocal[ge] = i;
                _geBox[4 * ge] = x0; _geBox[4 * ge + 1] = y0; _geBox[4 * ge + 2] = x1; _geBox[4 * ge + 3] = y1;
                double ext = Math.Max(x1 - x0, y1 - y0);
                if (ext > maxExt) { maxExt = ext; }
            }
        }

        double cs = Math.Max(maxExt, 1e-3), icsz = 1.0 / cs;
        var keys = _edgeGrid.Keys;
        for (int ge = 0; ge < ne; ge++)
        {
            keys[ge] = (SoftCellGrid.Hash((long)Math.Floor(_geBox[4 * ge] * icsz), (long)Math.Floor(_geBox[4 * ge + 1] * icsz), 0) << 24) | (long)ge;
        }

        _edgeGrid.Build(ne);
        // The queries are independent reads, so they run in tasks of SurfaceChunk particles, each into its own list; joining the lists in task order gives the order one pass over the particles would.
        int nq = 0;
        for (int a = 0; a < _awakeSoftCount; a++) { if (_awakeSoft[a].SurfBase >= 0) { nq += _awakeSoft[a].SurfParticles.Length; } }
        GrowTo(ref _sqG, nq);
        nq = 0;
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            var dp = _awakeSoft[a];
            if (dp.SurfBase < 0) { continue; }
            for (int pi = 0; pi < dp.SurfParticles.Length; pi++) { _sqG[nq++] = dp.Base + dp.SurfParticles[pi]; }
        }

        _sfIcsz = icsz; _sfCount = nq; _geCount = ne;
        int tasks = (nq + SurfaceChunk - 1) / SurfaceChunk;
        EnsureTaskBufs(tasks);
        for (int t = 0; t < tasks; t++) { _tb[t].SurfCount = 0; }
        Core.Executor.For(tasks, new SurfaceProgram(this));
        for (int t = 0; t < tasks; t++)
        {
            var tb = _tb[t];
            GrowTo(ref _sePG, _seCount + tb.SurfCount); GrowTo(ref _seEG, _seCount + tb.SurfCount); GrowTo(ref _seS, _seCount + tb.SurfCount);
            for (int k = 0; k < tb.SurfCount; k++)
            {
                _sePG[_seCount] = (int)(tb.SurfPairs[k] >> 32); _seEG[_seCount] = (int)(tb.SurfPairs[k] & 0xFFFFFFFFL); _seS[_seCount] = tb.SurfSide[k];
                _seCount++;
            }
        }
    }

    private int[] _sqG = new int[64];
    private double _sfIcsz;
    private int _sfCount;
    internal const int SurfaceChunk = 64;
    private const double MaxCellVisits = 256.0, MaxCellCoordinate = 1e15;
    private int _geCount;

    private readonly struct SurfaceProgram : IParallelBody
    {
        private readonly Engine2D _e;
        public SurfaceProgram(Engine2D e) { _e = e; }
        public void Run(int task) => _e.SurfaceTask(task);
    }

    // Particle against edge for the query particles of one task: the particle, grown like a feature of its body, looks at the cells that can hold an edge's minimum corner whose box meets its own.
    private void SurfaceTask(int t)
    {
        var buf = _tb[t];
        buf.SurfCount = 0;
        int q0 = t * SurfaceChunk, q1 = Math.Min(_sfCount, q0 + SurfaceChunk);
        double icsz = _sfIcsz;
        var keys = _edgeGrid.Keys;
        Span<long> seen = stackalloc long[16];
        for (int q = q0; q < q1; q++)
        {
            int g = _sqG[q];
            var dp = _gData[g];
            int p = _gPart[g];
            double ep = dp.Radius + 2.0 * dp.SurfTravel;
            double px = dp.Ox + dp.X[2 * p], py = dp.Oy + dp.X[2 * p + 1];
            // A particle that can travel very far, or sits very far out (a diverged body), would visit an enormous range of cells (or overflow the cell numbers); outside a small, sane range every edge is tested instead.
            double fx0 = (px - ep) * icsz, fx1 = (px + ep) * icsz, fy0 = (py - ep) * icsz, fy1 = (py + ep) * icsz;
            bool wide = !(Math.Abs(fx0) < MaxCellCoordinate && Math.Abs(fx1) < MaxCellCoordinate && Math.Abs(fy0) < MaxCellCoordinate && Math.Abs(fy1) < MaxCellCoordinate && (fx1 - fx0 + 3.0) * (fy1 - fy0 + 3.0) <= MaxCellVisits);
            long x0 = wide ? 0 : (long)Math.Floor(fx0) - 1, x1 = wide ? 0 : (long)Math.Floor(fx1);
            long y0 = wide ? 0 : (long)Math.Floor(fy0) - 1, y1 = wide ? 0 : (long)Math.Floor(fy1);
            int nseen = 0;
            if (wide)
            {
                for (int ge = 0; ge < _geCount; ge++)
                {
                    int o = 4 * ge;
                    if (px + ep < _geBox[o] || px - ep > _geBox[o + 2] || py + ep < _geBox[o + 1] || py - ep > _geBox[o + 3]) { continue; }
                    TryAddSurfacePair(dp, p, px, py, ge, buf);
                }

                continue;
            }

            for (long cy = y0; cy <= y1; cy++)
            {
                for (long cx = x0; cx <= x1; cx++)
                {
                    long hsh = SoftCellGrid.Hash(cx, cy, 0);
                    bool dup = false;
                    for (int u = 0; u < nseen; u++) { if (seen[u] == hsh) { dup = true; break; } }
                    if (dup) { continue; }
                    if (nseen < seen.Length) { seen[nseen++] = hsh; }
                    if (!_edgeGrid.Find(hsh, out int s0, out int s1)) { continue; }
                    for (int k = s0; k < s1; k++)
                    {
                        int ge = (int)(keys[k] & 0xFFFFFF);
                        int o = 4 * ge;
                        if (px + ep < _geBox[o] || px - ep > _geBox[o + 2] || py + ep < _geBox[o + 1] || py - ep > _geBox[o + 3]) { continue; }
                        TryAddSurfacePair(dp, p, px, py, ge, buf);
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
            float v2 = v[2 * i] * v[2 * i] + v[2 * i + 1] * v[2 * i + 1];
            if (v2 > m2) { m2 = v2; }
        }

        return float.Sqrt(m2);
    }

    private void TryAddSurfacePair(SoftBodyData dp, int p, double px, double py, int ge, SoftTaskBuf buf)
    {
        var de = _geBody[ge];
        int el = _geLocal[ge];
        int a = de.SurfA[el], b = de.SurfB[el];
        if (dp == de)
        {
            if (!dp.Def.SelfCollision || p == a || p == b) { return; }
            if (dp.RestDistanceToEdge(p, a, b) < 2.1f * (dp.Radius + de.Radius)) { return; }
        }
        else if (!SoftPairAllowed(dp.Def, de.Def)) { return; }

        if (dp.InvMass[p] == 0f && de.InvMass[a] == 0f && de.InvMass[b] == 0f) { return; }
        double ax = de.Ox + de.X[2 * a], ay = de.Oy + de.X[2 * a + 1], bx = de.Ox + de.X[2 * b], by = de.Oy + de.X[2 * b + 1];
        double ex = bx - ax, ey = by - ay, l2 = ex * ex + ey * ey;
        if (!(l2 > 0.0)) { return; }
        // The exact distance at the start of the substep against what the two bodies can close in it.
        double reach = dp.Radius + de.Radius + 2.0 * (dp.SurfTravel + de.SurfTravel);
        if (SoftGeometry.PointSegmentSq(px, py, ax, ay, bx, by) >= reach * reach) { return; }
        // The side the particle starts on: the right of the directed edge is outside the body. It means something only where the particle projects onto the edge; elsewhere the sign is left open (0) and the
        // first substep that finds the particle over the edge fixes it from where the particle is then.
        double side = (ey * (px - ax) - ex * (py - ay));
        double t0 = ((px - ax) * ex + (py - ay) * ey) / l2;
        GrowTo(ref buf.SurfPairs, buf.SurfCount + 1); GrowTo(ref buf.SurfSide, buf.SurfCount + 1);
        buf.SurfPairs[buf.SurfCount] = ((long)(dp.Base + p) << 32) | (long)ge; buf.SurfSide[buf.SurfCount] = t0 < 0.0 || t0 > 1.0 ? 0f : (side >= 0.0 ? 1f : -1f);
        buf.SurfCount++;
    }

    // One pass over the listed pairs. The constraint C = s (n . (p - a)) - T, with n the unit normal of the edge on its right, T the two particle radii, and the projection of p on the edge inside the segment (outside it the
    // end particles' discs hold the contact). The particle moves along s n, the edge's particles by their barycentric shares the other way; Coulomb friction removes the relative tangential displacement up to mu times the normal correction.
    private void SolveSurfacePairs()
    {
        for (int k = 0; k < _seCount; k++)
        {
            int g = _sePG[k];
            var dp = _gData[g];
            int p = _gPart[g];
            int ge = _seEG[k];
            var de = _geBody[ge];
            int el = _geLocal[ge];
            int a = de.SurfA[el], b = de.SurfB[el];
            int kp = 2 * p, ka = 2 * a, kb = 2 * b;
            float vax = SoftDelta(de, dp, ka, kp, 0), vay = SoftDelta(de, dp, ka, kp, 1);
            float vbx = SoftDelta(de, dp, kb, kp, 0), vby = SoftDelta(de, dp, kb, kp, 1);
            float ex = vbx - vax, ey = vby - vay;
            float l2 = ex * ex + ey * ey;
            if (!(l2 > 1e-18f)) { continue; }
            float t = -(vax * ex + vay * ey) / l2;
            if (t < 0f || t > 1f) { continue; }
            float il = 1f / float.Sqrt(l2);
            float nx = ey * il, ny = -ex * il;
            float s = _seS[k];
            if (s == 0f)
            {
                s = -(nx * vax + ny * vay) >= 0f ? 1f : -1f;
                _seS[k] = s;
            }

            float thick = dp.Radius + de.Radius;
            float c = s * -(nx * vax + ny * vay) - thick;
            if (!(c < 0f)) { continue; }
            // A particle that is already well behind the edge is brought back by at most the contact thickness per pass, so a wrong side never launches it.
            if (c < -thick) { c = -thick; }
            float wp = dp.InvMass[p], wa = de.InvMass[a], wb = de.InvMass[b];
            float sa = (1f - t), sb = t;
            float w = wp + sa * sa * wa + sb * sb * wb;
            if (!(w > 0f)) { continue; }
            float lam = -c / w;
            float sx = s * nx, sy = s * ny;
            dp.X[kp] += sx * wp * lam; dp.X[kp + 1] += sy * wp * lam;
            de.X[ka] -= sx * sa * wa * lam; de.X[ka + 1] -= sy * sa * wa * lam;
            de.X[kb] -= sx * sb * wb * lam; de.X[kb + 1] -= sy * sb * wb * lam;
            float mu = float.Sqrt(dp.Def.Friction * de.Def.Friction);
            if (mu > 0f)
            {
                float tx = ex * il, ty = ey * il;
                float ux = (dp.X[kp] - dp.XP[kp]) - (sa * (de.X[ka] - de.XP[ka]) + sb * (de.X[kb] - de.XP[kb]));
                float uy = (dp.X[kp + 1] - dp.XP[kp + 1]) - (sa * (de.X[ka + 1] - de.XP[ka + 1]) + sb * (de.X[kb + 1] - de.XP[kb + 1]));
                float ut = ux * tx + uy * ty;
                float len = float.Abs(ut);
                if (len > 1e-12f)
                {
                    float dir = ut > 0f ? 1f : -1f;
                    float lt = len / w;
                    float cap = mu * lam;
                    if (lt > cap) { lt = cap; }
                    dp.X[kp] -= tx * dir * wp * lt; dp.X[kp + 1] -= ty * dir * wp * lt;
                    de.X[ka] += tx * dir * sa * wa * lt; de.X[ka + 1] += ty * dir * sa * wa * lt;
                    de.X[kb] += tx * dir * sb * wb * lt; de.X[kb + 1] += ty * dir * sb * wb * lt;
                }
            }
        }
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
            // A tear can add at most one particle per triangle corner; stop tearing before the particle numbering runs out.
            if ((long)_softParticles + d.Tris.Length > MaxSoftParticles) { continue; }
            int added = d.Tear(keys);
            int slot = Array.IndexOf(SoftData, d);
            var handle = new SoftBody(World!, slot, SoftHandles.GenerationOf(slot));
            foreach (long k in keys) { _softTears.Add(new SoftTearEvent(handle, (int)(k >> 20), (int)(k & 0xFFFFF), d.N)); }
            _softParticles += added;
            d.SleepTime = 0f;
            d.HadMovingContact = true;
        }
    }
}
