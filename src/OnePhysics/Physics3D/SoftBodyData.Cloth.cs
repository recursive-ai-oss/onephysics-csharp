using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class SoftBodyData
{
    // Fabric data derived from Def.Fabric and the rest mesh (never saved; rebuilt with the body).

    /// <summary>Compliance per solver edge when the weave is anisotropic; null otherwise.</summary>
    public float[]? SEdgeComp;

    /// <summary>Rest dihedral angle (sine, cosine) per solver bend constraint when the bend is dihedral; null otherwise.</summary>
    public float[]? SBendSin0, SBendCos0;

    /// <summary>Tethers: particle, anchor, longest allowed distance.</summary>
    public int[] TethP = Array.Empty<int>(), TethA = Array.Empty<int>();
    public float[] TethLen = Array.Empty<float>();

    /// <summary>The solver edges in the order the strain limit visits them: by breadth-first distance from the pinned particles outwards (ties by edge number), so a sweep from the pins carries corrections along a hanging chain
    /// (Provot's iteration converges slowly in any other order). Derived from the pins; rebuilt when they change.</summary>
    public int[] LimitOrder = Array.Empty<int>();

    private long _limitPinSig = -1;

    /// <summary>Rebuilds <see cref="LimitOrder"/> when the set of pinned particles has changed since it was made.</summary>
    public void EnsureLimitOrder()
    {
        long sig = 0;
        for (int i = 0; i < N; i++) { if (InvMass[i] == 0f) { sig = sig * 1000003L + i + 1; } }
        if (sig == _limitPinSig && LimitOrder.Length == SEdgeA.Length) { return; }
        _limitPinSig = sig;
        int ne = SEdgeA.Length;
        var level = new int[N];
        Array.Fill(level, int.MaxValue);
        var queue = new int[N];
        int qh = 0, qt = 0;
        for (int i = 0; i < N; i++) { if (InvMass[i] == 0f) { level[i] = 0; queue[qt++] = i; } }
        // adjacency from the edge lists
        var start = new int[N + 1];
        for (int e = 0; e < ne; e++) { start[SEdgeA[e] + 1]++; start[SEdgeB[e] + 1]++; }
        for (int i = 0; i < N; i++) { start[i + 1] += start[i]; }
        var adj = new int[2 * ne]; var fill = new int[N];
        for (int e = 0; e < ne; e++) { int a = SEdgeA[e], b = SEdgeB[e]; adj[start[a] + fill[a]++] = b; adj[start[b] + fill[b]++] = a; }
        while (qh < qt)
        {
            int p = queue[qh++];
            for (int k = start[p]; k < start[p + 1]; k++)
            {
                int q = adj[k];
                if (level[q] == int.MaxValue) { level[q] = level[p] + 1; queue[qt++] = q; }
            }
        }

        var keys = new long[ne];
        for (int e = 0; e < ne; e++)
        {
            int la = level[SEdgeA[e]], lb = level[SEdgeB[e]];
            int l = Math.Min(la, lb);
            keys[e] = ((long)(l == int.MaxValue ? 0 : l) << 32) | (long)e;
        }

        Array.Sort(keys);
        LimitOrder = new int[ne];
        for (int e = 0; e < ne; e++) { LimitOrder[e] = (int)(keys[e] & 0xFFFFFFFF); }
    }

    public bool HasClothLimits => Def.Fabric != null && (Def.Fabric.StrainLimit > 0f || TethP.Length > 0);

    private void BuildCloth()
    {
        SEdgeComp = null; SBendSin0 = null; SBendCos0 = null;
        TethP = Array.Empty<int>(); TethA = Array.Empty<int>(); TethLen = Array.Empty<float>();
        var cl = Def.Fabric;
        if (cl == null) { return; }
        int ne = SEdgeA.Length;
        if (cl.Anisotropic)
        {
            var warp = ClothParams.Norm(cl.WarpAxis); var weft = ClothParams.Norm(cl.WeftAxis);
            SEdgeComp = new float[ne];
            for (int e = 0; e < ne; e++)
            {
                int a = SEdgeA[e], b = SEdgeB[e];
                float ux = Rest[3 * b] - Rest[3 * a], uy = Rest[3 * b + 1] - Rest[3 * a + 1], uz = Rest[3 * b + 2] - Rest[3 * a + 2];
                float l = float.Sqrt(ux * ux + uy * uy + uz * uz);
                float cw = 0f, ce = 0f;
                if (l > 0f)
                {
                    cw = float.Abs(ux * warp.X + uy * warp.Y + uz * warp.Z) / l;
                    ce = float.Abs(ux * weft.X + uy * weft.Y + uz * weft.Z) / l;
                }

                // Within about 26 degrees (cosine 0.9) of an axis: a thread of that direction; otherwise shear.
                float c = cw >= 0.9f ? cl.WarpCompliance : (ce >= 0.9f ? cl.WeftCompliance : cl.ShearCompliance);
                SEdgeComp[e] = c >= 0f ? c : Def.EdgeCompliance;
            }
        }

        if (cl.DihedralBend)
        {
            int nb = SBendS.Length;
            SBendSin0 = new float[nb]; SBendCos0 = new float[nb];
            for (int i = 0; i < nb; i++)
            {
                if (!ClothMath.Dihedral(Rest, SBendIdx[4 * i], SBendIdx[4 * i + 1], SBendIdx[4 * i + 2], SBendIdx[4 * i + 3], out float s, out float c)) { s = 0f; c = 1f; }
                SBendSin0[i] = s; SBendCos0[i] = c;
            }
        }

        if (cl.TetherAnchors != null && cl.TetherAnchors.Count > 0)
        {
            var anchors = new List<int>();
            foreach (int k in cl.TetherAnchors) { if (k < N) { anchors.Add(k); } }
            anchors.Sort();
            int w = 0;
            for (int i = 0; i < anchors.Count; i++) { if (i == 0 || anchors[i] != anchors[i - 1]) { anchors[w++] = anchors[i]; } }
            anchors.RemoveRange(w, anchors.Count - w);
            var tp = new List<int>(); var ta = new List<int>(); var tl = new List<float>();
            float slack = 1f + cl.TetherStrain;
            for (int p = 0; p < N; p++)
            {
                int best = -1; float bd = float.MaxValue;
                foreach (int a in anchors)
                {
                    if (a == p) { best = -1; break; }
                    float dd = RestDistance2(p, a);
                    if (dd < bd) { bd = dd; best = a; }
                }

                if (best < 0) { continue; }
                tp.Add(p); ta.Add(best); tl.Add(float.Sqrt(bd) * slack);
            }

            TethP = tp.ToArray(); TethA = ta.ToArray(); TethLen = tl.ToArray();
        }
    }
}

/// <summary>Geometry of a hinge shared by the dihedral bend of cloth.</summary>
internal static class ClothMath
{
    /// <summary>
    /// The signed dihedral angle between the triangles (a, b, c) and (a, b, d) sharing the edge a-b, as its sine and cosine: with e = b - a, N1 = e x (c - a) and N2 = (d - a) x e, cos = n1.n2 and sin = (n1 x n2).e/|e|
    /// (the two normals agree when the four points are flat). False for a degenerate hinge.
    /// </summary>
    public static bool Dihedral(float[] x, int a, int b, int c, int d, out float sin, out float cos)
    {
        sin = 0f; cos = 1f;
        int ka = 3 * a, kb = 3 * b, kc = 3 * c, kd = 3 * d;
        float ex = x[kb] - x[ka], ey = x[kb + 1] - x[ka + 1], ez = x[kb + 2] - x[ka + 2];
        float cx = x[kc] - x[ka], cy = x[kc + 1] - x[ka + 1], cz = x[kc + 2] - x[ka + 2];
        float dx = x[kd] - x[ka], dy = x[kd + 1] - x[ka + 1], dz = x[kd + 2] - x[ka + 2];
        float n1x = ey * cz - ez * cy, n1y = ez * cx - ex * cz, n1z = ex * cy - ey * cx;
        float n2x = dy * ez - dz * ey, n2y = dz * ex - dx * ez, n2z = dx * ey - dy * ex;
        float l1 = float.Sqrt(n1x * n1x + n1y * n1y + n1z * n1z), l2 = float.Sqrt(n2x * n2x + n2y * n2y + n2z * n2z), el = float.Sqrt(ex * ex + ey * ey + ez * ez);
        if (!(l1 > 1e-20f) || !(l2 > 1e-20f) || !(el > 1e-20f)) { return false; }
        float i1 = 1f / l1, i2 = 1f / l2;
        n1x *= i1; n1y *= i1; n1z *= i1; n2x *= i2; n2y *= i2; n2z *= i2;
        cos = n1x * n2x + n1y * n2y + n1z * n2z;
        float sx = n1y * n2z - n1z * n2y, sy = n1z * n2x - n1x * n2z, sz = n1x * n2y - n1y * n2x;
        sin = (sx * ex + sy * ey + sz * ez) / el;
        return true;
    }

    /// <summary>
    /// The angle of the vector (x, y) from the +x axis, in (-pi, pi]: octant reduction and the polynomial atan(z) ~ z (0.9998660 - 0.3302995 z^2 + 0.1801410 z^4 - 0.0851330 z^6 + 0.0208351 z^8) on 0 to 1
    /// (error under 1e-5 rad; Abramowitz and Stegun 4.4.49 style fit). Only + - * / are used, so the result is the same everywhere.
    /// </summary>
    public static float Atan2(float y, float x)
    {
        float ax = float.Abs(x), ay = float.Abs(y);
        float mx = ax > ay ? ax : ay, mn = ax > ay ? ay : ax;
        if (!(mx > 0f)) { return 0f; }
        float z = mn / mx, z2 = z * z;
        float r = z * (0.999866f + z2 * (-0.3302995f + z2 * (0.180141f + z2 * (-0.085133f + z2 * 0.0208351f))));
        if (ay > ax) { r = 1.5707964f - r; }
        if (x < 0f) { r = 3.1415927f - r; }
        return y < 0f ? -r : r;
    }
}
