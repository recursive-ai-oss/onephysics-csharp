using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class SoftBodyData
{
    /// <summary>Compliance per solver edge when the weave is anisotropic; null otherwise (derived from the rest mesh, never saved).</summary>
    public float[]? SEdgeComp;

    public bool HasStripLimits => Def.Strip != null && Def.Strip.StrainLimit > 0f;

    private void BuildStrip()
    {
        SEdgeComp = null;
        var st = Def.Strip;
        if (st == null || !st.Anisotropic) { return; }
        float al = float.Sqrt(st.AlongAxis.X * st.AlongAxis.X + st.AlongAxis.Y * st.AlongAxis.Y);
        float ax = st.AlongAxis.X / al, ay = st.AlongAxis.Y / al;
        int ne = SEdgeA.Length;
        SEdgeComp = new float[ne];
        for (int e = 0; e < ne; e++)
        {
            int a = SEdgeA[e], b = SEdgeB[e];
            float ux = Rest[2 * b] - Rest[2 * a], uy = Rest[2 * b + 1] - Rest[2 * a + 1];
            float l = float.Sqrt(ux * ux + uy * uy);
            float ca = 0f, cs = 0f;
            if (l > 0f) { ca = float.Abs(ux * ax + uy * ay) / l; cs = float.Abs(-ux * ay + uy * ax) / l; }
            float c = ca >= 0.9f ? st.AlongCompliance : (cs >= 0.9f ? st.AcrossCompliance : st.ShearCompliance);
            SEdgeComp[e] = c >= 0f ? c : Def.EdgeCompliance;
        }
    }
}
