using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Collision3D;

public sealed partial class ManifoldGenerator3D
{
    // Cosines and sines of the twelve rim samples of a cylinder cap, every 30 degrees (literals, so no trigonometry is evaluated).
    private static readonly float[] s_rimCos = { 1f, 0.8660254f, 0.5f, 0f, -0.5f, -0.8660254f, -1f, -0.8660254f, -0.5f, 0f, 0.5f, 0.8660254f };
    private static readonly float[] s_rimSin = { 0f, 0.5f, 0.8660254f, 1f, 0.8660254f, 0.5f, 0f, -0.5f, -0.8660254f, -1f, -0.8660254f, -0.5f };
    private const int RimSamples = 12;

    // A box is handled as the hull of the unit cube's corners with its vertices scaled by the core half extents; the faces and edges keep the cube's topology.
    private static readonly ConvexHull3D s_unitBox = BuildUnitBox();

    private static ConvexHull3D BuildUnitBox()
    {
        var corners = new Vector3F[8];
        for (int i = 0; i < 8; i++)
        {
            corners[i] = new Vector3F((i & 1) == 0 ? -1f : 1f, (i & 2) == 0 ? -1f : 1f, (i & 4) == 0 ? -1f : 1f);
        }

        return ConvexHull3D.Build(corners);
    }

    private static ConvexHull3D HullOf(in ConvexShape3D s) => s.Kind == ConvexShapeKind3D.Box ? s_unitBox : s.Hull!;

    private static Vector3F PolyVertex(in ConvexShape3D s, ConvexHull3D h, int i) =>
        s.Kind == ConvexShapeKind3D.Box ? h.Vertices[i] * s.CoreSize : h.Vertices[i];

    // The GJK/EPA route: the contact gives the normal; the supporting features of both shapes along it are clipped against each other.
    private Manifold3D Generic(in ConvexShape3D a, in ConvexShape3D b, in RigidTransformF bInA, in ConvexContact3D c, float margin)
    {
        if (c.Separation > margin)
        {
            return Empty;
        }

        Vector3F n = c.Normal;
        float len2 = n.LengthSquared();
        if (!(len2 > 0.25f) || !float.IsFinite(len2))
        {
            n = bInA.Translation.LengthSquared() > 0f ? bInA.Translation.Normalized() : Vector3F.UnitY;
        }
        else
        {
            n /= ConvexShape3D.Sqrt(len2);
        }

        Matrix3x3F r = bInA.Rotation.ToMatrix();
        float thick = margin + (1e-4f * (Extent(a) + Extent(b)));
        Extract(a, n, thick, _fa);
        Extract(b, r.Transpose() * -n, thick, _fb);
        _fb.Transform(r, bInA.Translation);
        SetHint(c, a, b);
        return FromFeatures(_fa, _fb, n, margin);
    }

    private void SetHint(in ConvexContact3D c, in ConvexShape3D a, in ConvexShape3D b)
    {
        _hint = c;
        _hasHint = true;
        _closeTol = 2e-4f * (Extent(a) + Extent(b));
    }

    // The set of surface points of a shape that touch a supporting plane, thickened by `thick`, for the outward unit direction u (shape-local frame):
    // a polygon when a whole face lies within the thickness, a segment for an edge, otherwise the single support point.
    private static void Extract(in ConvexShape3D s, Vector3F u, float thick, Feat f)
    {
        f.Reset();
        float r = s.ConvexRadius;
        Vector3F z = s.CoreSize;
        switch (s.Kind)
        {
            case ConvexShapeKind3D.Sphere:
                f.Kind = Feat.Point;
                f.Add(u * r, FCode.Vertex(0), FCode.Edge(0));
                return;
            case ConvexShapeKind3D.Capsule:
                ExtractAxis(f, z.Y, r, r, u, thick);
                return;
            case ConvexShapeKind3D.TaperedCapsule:
                ExtractAxis(f, z.Z, z.X, z.Y, u, thick);
                return;
            case ConvexShapeKind3D.Cylinder:
                ExtractCylinder(f, z.X, z.Y, r, u, thick);
                return;
            default:
                ExtractPoly(s, u, thick, f);
                return;
        }
    }

    // Capsule-like shapes: a segment on the Y axis with a sphere of radius rt at +h and rb at -h. Both end points touch the plane when they are within the thickness of each other.
    private static void ExtractAxis(Feat f, float h, float rt, float rb, Vector3F u, float thick)
    {
        Vector3F top = (new Vector3F(0f, h, 0f)) + (u * rt), bottom = (new Vector3F(0f, -h, 0f)) + (u * rb);
        float dt = Vector3F.Dot(top, u), db = Vector3F.Dot(bottom, u);
        if (Math.Abs(dt - db) <= thick)
        {
            f.Kind = Feat.Segment;
            f.Add(top, FCode.Vertex(0), FCode.Edge(0));
            f.Add(bottom, FCode.Vertex(1), FCode.Edge(0));
        }
        else if (dt > db)
        {
            f.Kind = Feat.Point;
            f.Add(top, FCode.Vertex(0), FCode.Edge(0));
        }
        else
        {
            f.Kind = Feat.Point;
            f.Add(bottom, FCode.Vertex(1), FCode.Edge(0));
        }
    }

    // Cylinder core of radius rc and half height hc inflated by cr. A cap becomes a twelve-sided polygon when its rim lies within the thickness of the plane;
    // a side becomes the segment along the axis; between them the touching rim point stands alone.
    private static void ExtractCylinder(Feat f, float rc, float hc, float cr, Vector3F u, float thick)
    {
        float rho2 = (u.X * u.X) + (u.Z * u.Z);
        float rho = ConvexShape3D.Sqrt(rho2);
        float side = u.Y >= 0f ? 1f : -1f;
        int cap = u.Y >= 0f ? 0 : 1;
        if (2f * rc * rho <= thick)
        {
            f.Kind = Feat.Polygon;
            f.Face = FCode.Face(cap);
            Vector3F shift = u * cr;
            for (int k = 0; k < RimSamples; k++)
            {
                int idx = (cap * 16) + k;
                f.Add(new Vector3F(rc * s_rimCos[k], side * hc, rc * s_rimSin[k]) + shift, FCode.Vertex(idx), FCode.Edge(idx));
            }

            return;
        }

        Vector3F radial = new Vector3F(u.X / rho, 0f, u.Z / rho) * rc;
        Vector3F sh = u * cr;
        if (2f * hc * Math.Abs(u.Y) <= thick)
        {
            f.Kind = Feat.Segment;
            f.Add(radial + new Vector3F(0f, hc, 0f) + sh, FCode.Vertex(32), FCode.Edge(32));
            f.Add(radial + new Vector3F(0f, -hc, 0f) + sh, FCode.Vertex(33), FCode.Edge(32));
            return;
        }

        f.Kind = Feat.Point;
        f.Add(radial + new Vector3F(0f, side * hc, 0f) + sh, FCode.Vertex(34 + cap), FCode.Edge(34 + cap));
    }

    private static void ExtractPoly(in ConvexShape3D s, Vector3F u, float thick, Feat f)
    {
        ConvexHull3D h = HullOf(s);
        bool box = s.Kind == ConvexShapeKind3D.Box;
        float r = s.ConvexRadius;
        int v = h.SupportIndex(box ? u * s.CoreSize : u);
        float top = Vector3F.Dot(PolyVertex(s, h, v), u);

        int bestFace = -1;
        float bestDot = float.NegativeInfinity;
        ReadOnlySpan<HullFace3D> faces = h.Faces;
        for (int i = 0; i < faces.Length; i++)
        {
            float d = Vector3F.Dot(faces[i].Normal, u);
            if (d > bestDot)
            {
                bestDot = d;
                bestFace = i;
            }
        }

        ReadOnlySpan<HullEdge3D> edges = h.Edges;
        if (bestFace >= 0)
        {
            HullFace3D face = faces[bestFace];
            bool flat = true;
            int e = face.FirstEdge;
            for (int k = 0; k < face.EdgeCount; k++)
            {
                if (top - Vector3F.Dot(PolyVertex(s, h, edges[e].Origin), u) > thick)
                {
                    flat = false;
                    break;
                }

                e = edges[e].Next;
            }

            if (flat)
            {
                f.Kind = Feat.Polygon;
                f.Face = FCode.Face(bestFace);
                Vector3F shift = u * r;
                e = face.FirstEdge;
                for (int k = 0; k < face.EdgeCount; k++)
                {
                    int vi = edges[e].Origin;
                    f.Add(PolyVertex(s, h, vi) + shift, FCode.Vertex(vi), FCode.Edge(e >> 1));
                    e = edges[e].Next;
                }

                return;
            }
        }

        // The edge leaving the support vertex that lies flattest against the plane.
        int e0 = h.VertexEdges[v];
        int ee = e0, bestEdge = -1, guard = 0;
        float bestDiff = float.PositiveInfinity;
        do
        {
            int w = edges[ee ^ 1].Origin;
            float diff = top - Vector3F.Dot(PolyVertex(s, h, w), u);
            if (diff < bestDiff)
            {
                bestDiff = diff;
                bestEdge = ee;
            }

            ee = edges[ee ^ 1].Next;
        }
        while (ee != e0 && ++guard < edges.Length);

        Vector3F sh = u * r;
        if (bestEdge >= 0 && bestDiff <= thick)
        {
            int w = edges[bestEdge ^ 1].Origin;
            f.Kind = Feat.Segment;
            f.Add(PolyVertex(s, h, v) + sh, FCode.Vertex(v), FCode.Edge(bestEdge >> 1));
            f.Add(PolyVertex(s, h, w) + sh, FCode.Vertex(w), FCode.Edge(bestEdge >> 1));
            return;
        }

        f.Kind = Feat.Point;
        f.Add(PolyVertex(s, h, v) + sh, FCode.Vertex(v), FCode.Edge(0));
    }
}
