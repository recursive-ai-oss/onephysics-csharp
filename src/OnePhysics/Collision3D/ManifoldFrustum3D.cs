using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class ManifoldGenerator3D
{
    private static void ExtractFrustum(Feat f, float h, float rt, float rb, Vector3F u, float thick)
    {
        float rho = ConvexShape3D.Sqrt(u.X * u.X + u.Z * u.Z);
        float dt = h * u.Y + rt * rho, db = -h * u.Y + rb * rho;
        bool top = dt >= db;
        float radius = top ? rt : rb, y = top ? h : -h;
        int cap = top ? 0 : 1;
        if (2f * radius * rho <= thick && Math.Abs(dt - db) > thick)
        {
            if (radius == 0f)
            {
                f.Kind = Feat.Point;
                f.Add(new Vector3F(0f, y, 0f), FCode.Vertex(cap * 16), FCode.Edge(cap * 16));
                return;
            }
            f.Kind = Feat.Polygon;
            f.Face = FCode.Face(cap);
            for (int k = 0; k < RimSamples; k++)
            {
                int idx = cap * 16 + k;
                f.Add(new Vector3F(radius * s_rimCos[k], y, radius * s_rimSin[k]), FCode.Vertex(idx), FCode.Edge(idx));
            }
            return;
        }
        float x = rho > 0f ? u.X / rho : 1f, z = rho > 0f ? u.Z / rho : 0f;
        if (Math.Abs(dt - db) <= thick)
        {
            f.Kind = Feat.Segment;
            f.Add(new Vector3F(rt * x, h, rt * z), FCode.Vertex(32), FCode.Edge(32));
            f.Add(new Vector3F(rb * x, -h, rb * z), FCode.Vertex(33), FCode.Edge(32));
            return;
        }
        f.Kind = Feat.Point;
        f.Add(new Vector3F(radius * x, y, radius * z), FCode.Vertex(34 + cap), FCode.Edge(34 + cap));
    }
}
