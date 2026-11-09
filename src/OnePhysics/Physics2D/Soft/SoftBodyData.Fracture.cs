using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>Plasticity, tearing and the collision surface of a 2D soft body (see docs/spec/softbody.md, Tearing, Plasticity, Surface contacts).</summary>
internal sealed partial class SoftBodyData
{
    // Stored (solver) order to public order of each constraint kind, kept so that rest values changed by plasticity reach both arrays.
    public int[] EdgeOrder = Array.Empty<int>(), BendOrder = Array.Empty<int>(), TriOrder = Array.Empty<int>();

    /// <summary>Rest offsets of the shape-matching clusters as the mesh gave them (the plastic limit is measured from these).</summary>
    public float[] ClusterQ0 = Array.Empty<float>();

    /// <summary>Whether plasticity has changed any rest value from the mesh's own: the rest arrays are then part of a snapshot.</summary>
    public bool PlasticTouched;

    // A sleeping body touched by an awake one is an immovable obstacle for the step: its inverse masses read as zero until RestorePassive.
    private float[]? _liveInvMass;
    private float[]? _zeroInvMass;

    public void MakePassive()
    {
        _liveInvMass = InvMass;
        if (_zeroInvMass == null || _zeroInvMass.Length != N) { _zeroInvMass = new float[N]; }
        InvMass = _zeroInvMass;
    }

    public void RestorePassive()
    {
        if (_liveInvMass != null) { InvMass = _liveInvMass; _liveInvMass = null; }
    }

    // ------------------------------------------------------------------ surface

    /// <summary>The boundary edges, directed so that the body's material is on the left (the outward normal is on the right): the edges that belong to one triangle, or the unbroken segments of an outline.</summary>
    public int[] SurfA = Array.Empty<int>(), SurfB = Array.Empty<int>();

    /// <summary>The particles on the surface, ascending.</summary>
    public int[] SurfParticles = Array.Empty<int>();

    /// <summary>Index of this body's first surface edge in the step's numbering.</summary>
    public int SurfBase;

    private void BuildSurface()
    {
        var a = new List<int>(); var b = new List<int>();
        if (Tris.Length > 0)
        {
            int n = Tris.Length;
            var directed = new long[n];
            for (int e = 0; e < n / 3; e++)
            {
                for (int k = 0; k < 3; k++) { directed[3 * e + k] = ((long)Tris[3 * e + k] << 20) | (long)Tris[3 * e + (k + 1) % 3]; }
            }

            Array.Sort(directed);
            for (int i = 0; i < n; i++)
            {
                long k = directed[i];
                long rev = ((k & 0xFFFFF) << 20) | (k >> 20);
                if (Array.BinarySearch(directed, rev) < 0) { a.Add((int)(k >> 20)); b.Add((int)(k & 0xFFFFF)); }
            }
        }
        else if (Outline.Length >= 3)
        {
            int n = Outline.Length;
            for (int i = 0; i < n; i++)
            {
                int p = Outline[i], q = Outline[(i + 1) % n];
                if (Broken.Length != 0 && SoftTopology.Has(Broken, EdgeOf(p, q))) { continue; }
                a.Add(p); b.Add(q);
            }
        }

        SurfA = a.ToArray(); SurfB = b.ToArray();
        var mark = new bool[N];
        for (int i = 0; i < SurfA.Length; i++) { mark[SurfA[i]] = true; mark[SurfB[i]] = true; }
        int count = 0;
        for (int i = 0; i < N; i++) { if (mark[i]) { count++; } }
        SurfParticles = new int[count];
        count = 0;
        for (int i = 0; i < N; i++) { if (mark[i]) { SurfParticles[count++] = i; } }
    }

    /// <summary>Distance at rest from particle <paramref name="p"/> to the segment between two particles (for the self-collision exclusion).</summary>
    public float RestDistanceToEdge(int p, int a, int b)
    {
        float px = Rest[2 * p], py = Rest[2 * p + 1], ax = Rest[2 * a], ay = Rest[2 * a + 1], bx = Rest[2 * b], by = Rest[2 * b + 1];
        float ex = bx - ax, ey = by - ay;
        float l2 = ex * ex + ey * ey;
        float t = l2 > 0f ? ((px - ax) * ex + (py - ay) * ey) / l2 : 0f;
        t = t < 0f ? 0f : (t > 1f ? 1f : t);
        float dx = px - (ax + t * ex), dy = py - (ay + t * ey);
        return float.Sqrt(dx * dx + dy * dy);
    }

    // ------------------------------------------------------------------ plasticity

    private void ClonePlastic()
    {
        EdgeRest = (float[])EdgeRest.Clone(); TriRest2 = (float[])TriRest2.Clone();
        SEdgeRest = (float[])SEdgeRest.Clone(); STriRest2 = (float[])STriRest2.Clone();
        ClusterQ = (float[])ClusterQ.Clone(); ClusterAinv = (float[])ClusterAinv.Clone(); ClusterScale = (float[])ClusterScale.Clone();
    }

    /// <summary>Copies the public rest arrays to the solver's copies (after a restore or a rebuild changed them).</summary>
    public void SyncSolverRests()
    {
        for (int i = 0; i < SEdgeRest.Length; i++) { SEdgeRest[i] = EdgeRest[EdgeOrder[i]]; }
        for (int i = 0; i < STriRest2.Length; i++) { STriRest2[i] = TriRest2[TriOrder[i]]; }
    }

    // The rest value follows the strain beyond the yield strain: rest' = rest (1 + creep (strain -+ yield)), kept within the plastic limit of the original value (Müller, Keiser, Nealen, Pauly, Gross, Alexa 2004,
    // plastic deformation by rest-shape update; O'Brien, Bargteil, Hodgins 2002 for the yield and creep model).
    private static float Yielded(float rest, float rest0, float strain, float yield, float creep, float max)
    {
        float excess = strain > yield ? strain - yield : (strain < -yield ? strain + yield : 0f);
        if (excess == 0f) { return rest; }
        float r = rest * (1f + creep * excess);
        float lo = rest0 * (1f - max), hi = rest0 * (1f + max);
        if (lo < rest0 * 1e-3f) { lo = rest0 * 1e-3f; }
        return r < lo ? lo : (r > hi ? hi : r);
    }

    /// <summary>Updates the rest shape from the strain at the end of a step. Edges, triangle areas and shape-matching offsets, in index order.</summary>
    public void ApplyPlasticity()
    {
        float yield = Def.PlasticYield;
        if (!(yield > 0f)) { return; }
        float creep = Def.PlasticCreep, max = Def.PlasticMax;
        var x = X;
        if (UsesNetwork)
        {
            for (int s = 0; s < SEdgeA.Length; s++)
            {
                int ka = 2 * SEdgeA[s], kb = 2 * SEdgeB[s];
                float dx = x[ka] - x[kb], dy = x[ka + 1] - x[kb + 1];
                float len = float.Sqrt(dx * dx + dy * dy);
                float rest = SEdgeRest[s];
                if (!(rest > 0f)) { continue; }
                float nr = Yielded(rest, EdgeRest0[EdgeOrder[s]], (len - rest) / rest, yield, creep, max);
                if (nr != rest) { SEdgeRest[s] = nr; EdgeRest[EdgeOrder[s]] = nr; PlasticTouched = true; }
            }

            for (int s = 0; s < STriRest2.Length; s++)
            {
                int a = 2 * STris[3 * s], b = 2 * STris[3 * s + 1], c = 2 * STris[3 * s + 2];
                float area2 = (x[b] - x[a]) * (x[c + 1] - x[a + 1]) - (x[b + 1] - x[a + 1]) * (x[c] - x[a]);
                float rest = STriRest2[s];
                if (!(rest > 0f)) { continue; }
                float nr = Yielded(rest, TriRest20[TriOrder[s]], (area2 - rest) / rest, yield, creep, max);
                if (nr != rest) { STriRest2[s] = nr; TriRest2[TriOrder[s]] = nr; PlasticTouched = true; }
            }
        }

        if (UsesMatching) { PlasticClusters(yield, creep, max); }
    }

    // Shape matching: where a member sits further from its goal than the yield strain (against the cluster's reach), its rest offset moves towards the offset it would need to be on the goal, in the frame of the cluster's
    // best rotation (Müller et al. 2005, plastic deformation). The offsets are re-centred so the cluster stays momentum-neutral.
    private void PlasticClusters(float yield, float creep, float max)
    {
        var x = X;
        for (int c = 0; c < ClusterCount; c++)
        {
            int m0 = ClusterStart[c], m1 = ClusterStart[c + 1];
            double mass = ClusterTotalMass[c], cx = 0.0, cy = 0.0;
            for (int m = m0; m < m1; m++) { int k = 2 * ClusterMember[m]; double mi = ClusterMass[m]; cx += mi * x[k]; cy += mi * x[k + 1]; }
            cx /= mass; cy /= mass;
            double a00 = 0.0, a01 = 0.0, a10 = 0.0, a11 = 0.0, reach = 0.0;
            for (int m = m0; m < m1; m++)
            {
                int k = 2 * ClusterMember[m];
                double mi = ClusterMass[m], px = x[k] - cx, py = x[k + 1] - cy, qx = ClusterQ[2 * m], qy = ClusterQ[2 * m + 1];
                a00 += mi * px * qx; a01 += mi * px * qy; a10 += mi * py * qx; a11 += mi * py * qy;
                reach = Math.Max(reach, Math.Max(Math.Abs(qx), Math.Abs(qy)));
            }

            if (!(reach > 0.0)) { continue; }
            double rc = a00 + a11, rs = a10 - a01, rl = Math.Sqrt(rc * rc + rs * rs);
            if (rl > 1e-30) { rc /= rl; rs /= rl; } else { rc = 1.0; rs = 0.0; }
            bool changed = false;
            double mx = 0.0, my = 0.0;
            for (int m = m0; m < m1; m++)
            {
                int k = 2 * ClusterMember[m];
                double px = x[k] - cx, py = x[k + 1] - cy, qx = ClusterQ[2 * m], qy = ClusterQ[2 * m + 1];
                double dvx = px - (rc * qx - rs * qy), dvy = py - (rs * qx + rc * qy);
                double dl = Math.Sqrt(dvx * dvx + dvy * dvy), strain = dl / reach;
                if (strain > yield)
                {
                    // the deviation in the rest frame (rotate by the inverse rotation)
                    double bx = rc * dvx + rs * dvy, by = -rs * dvx + rc * dvy;
                    double f = creep * (1.0 - yield / strain);
                    double nqx = qx + f * bx, nqy = qy + f * by;
                    double ox = ClusterQ0[2 * m], oy = ClusterQ0[2 * m + 1], lim = max * reach;
                    double ddx = nqx - ox, ddy = nqy - oy, dd = Math.Sqrt(ddx * ddx + ddy * ddy);
                    if (dd > lim) { nqx = ox + ddx * lim / dd; nqy = oy + ddy * lim / dd; }
                    ClusterQ[2 * m] = (float)nqx; ClusterQ[2 * m + 1] = (float)nqy;
                    changed = true;
                }

                mx += ClusterMass[m] * ClusterQ[2 * m]; my += ClusterMass[m] * ClusterQ[2 * m + 1];
            }

            if (!changed) { continue; }
            PlasticTouched = true;
            mx /= mass; my /= mass;
            for (int m = m0; m < m1; m++) { ClusterQ[2 * m] = (float)(ClusterQ[2 * m] - mx); ClusterQ[2 * m + 1] = (float)(ClusterQ[2 * m + 1] - my); }
            RefitQuadratic(c);
        }
    }

    /// <summary>Recomputes the quadratic fit of cluster c from its current rest offsets (after plasticity or a restore).</summary>
    public void RefitQuadratic(int c)
    {
        ClusterScale[c] = 0f;
        if (!Def.MatchQuadratic) { return; }
        int m0 = ClusterStart[c], m1 = ClusterStart[c + 1];
        double reach = 0.0;
        for (int m = m0; m < m1; m++) { reach = Math.Max(reach, Math.Max(Math.Abs(ClusterQ[2 * m]), Math.Abs(ClusterQ[2 * m + 1]))); }
        if (!(reach > 0.0)) { return; }
        var a = new double[25];
        var qt = new double[5];
        var inv = new double[25];
        for (int m = m0; m < m1; m++)
        {
            double x = ClusterQ[2 * m] / reach, y = ClusterQ[2 * m + 1] / reach, mi = ClusterMass[m];
            qt[0] = x; qt[1] = y; qt[2] = x * x; qt[3] = y * y; qt[4] = x * y;
            for (int r = 0; r < 5; r++) { for (int k = 0; k < 5; k++) { a[5 * r + k] += mi * qt[r] * qt[k]; } }
        }

        if (!Invert5(a, inv)) { return; }
        ClusterScale[c] = (float)reach;
        for (int k = 0; k < 25; k++) { ClusterAinv[25 * c + k] = (float)inv[k]; }
    }

    // ------------------------------------------------------------------ tearing

    /// <summary>Collects the keys of the edges that break at the end of a step: strain above <see cref="SoftBodyDef.TearStrain"/>, or the last substep's constraint force above <see cref="SoftBodyDef.TearForce"/>. Ascending, no repeats.</summary>
    public long[] DetectTears(float h)
    {
        float strainLimit = Def.TearStrain, forceLimit = Def.TearForce;
        if (!UsesNetwork || SEdgeA.Length == 0 || (strainLimit <= 0f && forceLimit <= 0f)) { return Array.Empty<long>(); }
        List<long>? keys = null;
        var x = X;
        float ih2 = 1f / (h * h);
        for (int s = 0; s < SEdgeA.Length; s++)
        {
            int a = SEdgeA[s], b = SEdgeB[s];
            int ka = 2 * a, kb = 2 * b;
            float dx = x[ka] - x[kb], dy = x[ka + 1] - x[kb + 1];
            float len = float.Sqrt(dx * dx + dy * dy);
            float rest = SEdgeRest[s];
            bool tear = strainLimit > 0f && rest > 0f && (len - rest) / rest > strainLimit;
            if (!tear && forceLimit > 0f) { tear = float.Abs(Lambda[s]) * ih2 > forceLimit; }
            if (tear) { (keys ??= new List<long>()).Add(EdgeOf(a, b)); }
        }

        if (keys == null) { return Array.Empty<long>(); }
        keys.Sort();
        return keys.ToArray();
    }

    /// <summary>
    /// Breaks the given edges: their constraints go, and where the broken edges separate the triangles around a vertex the vertex is split (see <see cref="SoftTopology"/>), which rebuilds the body's constraints.
    /// The particle count grows by the number of split vertices; the new particles start at the position and velocity of the one they were split from. Returns the number of new particles.
    /// </summary>
    public int Tear(long[] keys)
    {
        var all = SoftTopology.Union(Broken, keys);
        int nt = Tris.Length / 3;
        var src = new List<int>();
        int[] elems = (int[])Tris.Clone();
        int added = 0;
        long[] broken = all;
        int[] srcAll;
        if (nt > 0)
        {
            added = SoftTopology.Split(N, elems, 3, all, src);
            srcAll = new int[N + added];
            for (int i = 0; i < N; i++) { srcAll[i] = i; }
            for (int i = 0; i < added; i++) { srcAll[N + i] = src[i]; }
            broken = SoftTopology.RemapBroken(all, Tris, elems, 3, srcAll);
        }
        else
        {
            srcAll = new int[N];
            for (int i = 0; i < N; i++) { srcAll[i] = i; }
        }

        Rebuild(elems, added, srcAll, broken);
        return added;
    }

    private void Rebuild(int[] elems, int added, int[] srcAll, long[] broken)
    {
        int oldN = N;
        var oldPos = Pos; var oldPrev = Prev; var oldVel = Vel; var oldPinned = Pinned;
        var oldEdgeKey = EdgeKey; var oldEdgeRest = EdgeRest; var oldTriRest2 = TriRest2;
        bool hadTargets = HasTargets;
        var oldTargetOn = TargetOn; var oldTargetMax = TargetMax; var oldTargetPos = TargetPos; var oldTargetPrev = TargetPrev;
        var oldQ = ClusterQ; var oldQ0 = ClusterQ0;
        var oldMesh = Def.Mesh!;
        bool plastic = PlasticTouched;
        if (Tris.Length > 0)
        {
            var verts = new Vector2F[oldN + added];
            var ov = oldMesh.Vertices;
            for (int i = 0; i < verts.Length; i++) { verts[i] = ov[srcAll[i]]; }
            Def.Mesh = SoftBodyMesh.FromTriangles(verts, elems);
        }

        // Freeze what the mesh would otherwise derive again, so that the same definition rebuilds the same body after a restore.
        Def.ParticleRadius = Radius;
        Def.Mass = TotalMass;
        if (!Def.Mesh!.IsClosed || (Tris.Length == 0 && broken.Length != 0)) { Def.InternalPressure = 0f; }
        Broken = broken;
        HasTargets = false;
        Build();
        for (int i = 0; i < N; i++)
        {
            int s = srcAll[i];
            Pos[2 * i] = oldPos[2 * s]; Pos[2 * i + 1] = oldPos[2 * s + 1];
            Prev[2 * i] = oldPrev[2 * s]; Prev[2 * i + 1] = oldPrev[2 * s + 1];
            Vel[2 * i] = oldVel[2 * s]; Vel[2 * i + 1] = oldVel[2 * s + 1];
            Pinned[i] = oldPinned[s];
        }

        if (hadTargets)
        {
            EnsureTargets();
            for (int i = 0; i < N; i++)
            {
                int s = srcAll[i];
                TargetOn[i] = oldTargetOn[s]; TargetMax[i] = oldTargetMax[s];
                TargetPos[2 * i] = oldTargetPos[2 * s]; TargetPos[2 * i + 1] = oldTargetPos[2 * s + 1];
                TargetPrev[2 * i] = oldTargetPrev[2 * s]; TargetPrev[2 * i + 1] = oldTargetPrev[2 * s + 1];
            }
        }

        for (int i = 0; i < N; i++) { RefreshInvMass(i); }
        // Rest values the plasticity changed carry over: an edge keeps the rest length of the edge it came from, triangles keep their numbers.
        if (plastic)
        {
            for (int e = 0; e < EdgeKey.Length; e++)
            {
                long pk = EdgeOf(srcAll[(int)(EdgeKey[e] >> 20)], srcAll[(int)(EdgeKey[e] & 0xFFFFF)]);
                int j = Array.BinarySearch(oldEdgeKey, pk);
                if (j >= 0) { EdgeRest[e] = oldEdgeRest[j]; }
            }

            if (TriRest2.Length == oldTriRest2.Length) { Array.Copy(oldTriRest2, TriRest2, TriRest2.Length); }
            if (ClusterQ.Length == oldQ.Length && ClusterCount > 0) { Array.Copy(oldQ, ClusterQ, oldQ.Length); Array.Copy(oldQ0, ClusterQ0, oldQ0.Length); for (int c = 0; c < ClusterCount; c++) { RefitQuadratic(c); } }
            SyncSolverRests();
        }
    }
}
