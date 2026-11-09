using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Plasticity, tearing and the collision surface of a 3D soft body (see docs/spec/softbody.md, Tearing, Plasticity, Surface contacts).</summary>
internal sealed partial class SoftBodyData
{
    // Stored (solver) order to public order of each constraint kind, kept so that rest values changed by plasticity reach both arrays.
    public int[] EdgeOrder = Array.Empty<int>(), TetOrder = Array.Empty<int>();

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

    /// <summary>The surface triangles (three particles each): the mesh's triangles, or the boundary faces of a tetrahedral mesh.</summary>
    public int[] SurfTris = Array.Empty<int>();

    /// <summary>The distinct edges of the surface triangles, by (smaller, larger) particle number, ascending.</summary>
    public int[] SurfEdgeA = Array.Empty<int>(), SurfEdgeB = Array.Empty<int>();

    /// <summary>The particles on the surface, ascending.</summary>
    public int[] SurfParticles = Array.Empty<int>();

    /// <summary>Index of this body's first surface triangle and first surface edge in the step's numbering.</summary>
    public int SurfTriBase, SurfEdgeBase;

    private void BuildSurface()
    {
        SurfTris = Tris;
        var keys = new long[Tris.Length];
        for (int e = 0; e < Tris.Length / 3; e++)
        {
            for (int k = 0; k < 3; k++) { keys[3 * e + k] = EdgeOf(Tris[3 * e + k], Tris[3 * e + (k + 1) % 3]); }
        }

        Array.Sort(keys);
        var a = new List<int>(keys.Length / 2 + 1); var b = new List<int>(keys.Length / 2 + 1);
        for (int i = 0; i < keys.Length; i++)
        {
            if (i > 0 && keys[i] == keys[i - 1]) { continue; }
            a.Add((int)(keys[i] >> 20)); b.Add((int)(keys[i] & 0xFFFFF));
        }

        SurfEdgeA = a.ToArray(); SurfEdgeB = b.ToArray();
        var mark = new bool[N];
        for (int i = 0; i < Tris.Length; i++) { mark[Tris[i]] = true; }
        int count = 0;
        for (int i = 0; i < N; i++) { if (mark[i]) { count++; } }
        SurfParticles = new int[count];
        count = 0;
        for (int i = 0; i < N; i++) { if (mark[i]) { SurfParticles[count++] = i; } }
    }

    /// <summary>Distance at rest between two particles (for the self-collision exclusions).</summary>
    public float RestDistance(int a, int b)
    {
        float dx = Rest[3 * a] - Rest[3 * b], dy = Rest[3 * a + 1] - Rest[3 * b + 1], dz = Rest[3 * a + 2] - Rest[3 * b + 2];
        return float.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    // ------------------------------------------------------------------ plasticity

    /// <summary>Copies the public rest arrays to the solver's copies (after a restore or a rebuild changed them).</summary>
    public void SyncSolverRests()
    {
        for (int i = 0; i < SEdgeRest.Length; i++) { SEdgeRest[i] = EdgeRest[EdgeOrder[i]]; }
        for (int i = 0; i < STetRest.Length; i++) { STetRest[i] = TetRest[TetOrder[i]]; }
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

    /// <summary>Updates the rest shape from the strain at the end of a step: edge lengths and tetrahedron volumes, in index order.</summary>
    public void ApplyPlasticity()
    {
        float yield = Def.PlasticYield;
        if (!(yield > 0f)) { return; }
        float creep = Def.PlasticCreep, max = Def.PlasticMax;
        var x = X;
        for (int s = 0; s < SEdgeA.Length; s++)
        {
            int ka = 3 * SEdgeA[s], kb = 3 * SEdgeB[s];
            float dx = x[ka] - x[kb], dy = x[ka + 1] - x[kb + 1], dz = x[ka + 2] - x[kb + 2];
            float len = float.Sqrt(dx * dx + dy * dy + dz * dz);
            float rest = SEdgeRest[s];
            if (!(rest > 0f)) { continue; }
            float nr = Yielded(rest, EdgeRest0[EdgeOrder[s]], (len - rest) / rest, yield, creep, max);
            if (nr != rest) { SEdgeRest[s] = nr; EdgeRest[EdgeOrder[s]] = nr; PlasticTouched = true; }
        }

        for (int s = 0; s < STetRest.Length; s++)
        {
            int a = 3 * STets[4 * s], b = 3 * STets[4 * s + 1], c = 3 * STets[4 * s + 2], d = 3 * STets[4 * s + 3];
            float e1x = x[b] - x[a], e1y = x[b + 1] - x[a + 1], e1z = x[b + 2] - x[a + 2];
            float e2x = x[c] - x[a], e2y = x[c + 1] - x[a + 1], e2z = x[c + 2] - x[a + 2];
            float e3x = x[d] - x[a], e3y = x[d + 1] - x[a + 1], e3z = x[d + 2] - x[a + 2];
            float vol6 = e1x * (e2y * e3z - e2z * e3y) + e1y * (e2z * e3x - e2x * e3z) + e1z * (e2x * e3y - e2y * e3x);
            float rest = STetRest[s];
            if (!(rest > 0f)) { continue; }
            float nr = Yielded(rest, TetRest0[TetOrder[s]], (vol6 - rest) / rest, yield, creep, max);
            if (nr != rest) { STetRest[s] = nr; TetRest[TetOrder[s]] = nr; PlasticTouched = true; }
        }
    }

    // ------------------------------------------------------------------ tearing

    /// <summary>Collects the keys of the edges that break at the end of a step: strain above <see cref="SoftBodyDef.TearStrain"/>, or the last substep's constraint force above <see cref="SoftBodyDef.TearForce"/>. Ascending, no repeats.</summary>
    public long[] DetectTears(float h)
    {
        float strainLimit = Def.TearStrain, forceLimit = Def.TearForce;
        if (IsSkinned || SEdgeA.Length == 0 || (strainLimit <= 0f && forceLimit <= 0f)) { return Array.Empty<long>(); }
        List<long>? keys = null;
        var x = X;
        float ih2 = 1f / (h * h);
        for (int s = 0; s < SEdgeA.Length; s++)
        {
            int a = SEdgeA[s], b = SEdgeB[s];
            int ka = 3 * a, kb = 3 * b;
            float dx = x[ka] - x[kb], dy = x[ka + 1] - x[kb + 1], dz = x[ka + 2] - x[kb + 2];
            float len = float.Sqrt(dx * dx + dy * dy + dz * dz);
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
    /// Breaks the given edges: their constraints go, and where the broken edges separate the elements around a vertex (triangles joined across edges, tetrahedra across faces) the vertex is split
    /// (see <see cref="SoftTopology"/>), which rebuilds the body's constraints. The particle count grows by the number of split vertices; the new particles start at the position and velocity of the one they were split from.
    /// Returns the number of new particles.
    /// </summary>
    public int Tear(long[] keys)
    {
        var all = SoftTopology.Union(Broken, keys);
        bool tets = Tets.Length > 0;
        int per = tets ? 4 : 3;
        int[] oldElems = tets ? Tets : Tris;
        int[] elems = (int[])oldElems.Clone();
        var src = new List<int>();
        int added = SoftTopology.Split(N, elems, per, all, src);
        var srcAll = new int[N + added];
        for (int i = 0; i < N; i++) { srcAll[i] = i; }
        for (int i = 0; i < added; i++) { srcAll[N + i] = src[i]; }
        long[] broken = SoftTopology.RemapBroken(all, oldElems, elems, per, srcAll);
        Rebuild(elems, tets, added, srcAll, broken);
        return added;
    }

    private void Rebuild(int[] elems, bool tets, int added, int[] srcAll, long[] broken)
    {
        int oldN = N;
        var oldPos = Pos; var oldPrev = Prev; var oldVel = Vel; var oldPinned = Pinned;
        var oldEdgeKey = EdgeKey; var oldEdgeRest = EdgeRest; var oldTetRest = TetRest;
        var oldMesh = Def.Mesh!;
        bool plastic = PlasticTouched;
        var verts = new Vector3F[oldN + added];
        var ov = oldMesh.Vertices;
        for (int i = 0; i < verts.Length; i++) { verts[i] = ov[srcAll[i]]; }
        Def.Mesh = tets ? SoftBodyMesh.FromTetrahedra(verts, elems) : SoftBodyMesh.FromTriangles(verts, elems);

        // Freeze what the mesh would otherwise derive again, so that the same definition rebuilds the same body after a restore.
        Def.ParticleRadius = Radius;
        Def.Mass = TotalMass;
        if (!Def.Mesh.IsClosed) { Def.InternalPressure = 0f; }
        Broken = broken;
        Build();
        for (int i = 0; i < N; i++)
        {
            int s = srcAll[i];
            for (int c = 0; c < 3; c++) { Pos[3 * i + c] = oldPos[3 * s + c]; Prev[3 * i + c] = oldPrev[3 * s + c]; Vel[3 * i + c] = oldVel[3 * s + c]; }
            Pinned[i] = oldPinned[s];
        }

        for (int i = 0; i < N; i++) { RefreshInvMass(i); }
        // Rest values the plasticity changed carry over: an edge keeps the rest length of the edge it came from, tetrahedra keep their numbers.
        if (plastic)
        {
            for (int e = 0; e < EdgeKey.Length; e++)
            {
                long pk = EdgeOf(srcAll[(int)(EdgeKey[e] >> 20)], srcAll[(int)(EdgeKey[e] & 0xFFFFF)]);
                int j = Array.BinarySearch(oldEdgeKey, pk);
                if (j >= 0) { EdgeRest[e] = oldEdgeRest[j]; }
            }

            if (TetRest.Length == oldTetRest.Length) { Array.Copy(oldTetRest, TetRest, TetRest.Length); }
            SyncSolverRests();
        }
    }
}
