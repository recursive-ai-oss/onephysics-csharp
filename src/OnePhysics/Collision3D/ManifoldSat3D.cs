using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Collision3D;

public sealed partial class ManifoldGenerator3D
{
    // A polyhedron core placed in A's frame: vertices, face normals and plane offsets, plus its half-edge topology.
    private sealed class PolyData
    {
        public ConvexHull3D Hull = null!;
        public Vector3F[] V = new Vector3F[16];
        public Vector3F[] N = new Vector3F[16];
        public float[] D = new float[16];
        public int NV, NF;
        public Vector3F Center;
        public float Radius;

        public void Load(in ConvexShape3D s, bool transform, in Matrix3x3F r, Vector3F t)
        {
            Hull = HullOf(s);
            NV = Hull.Vertices.Length;
            NF = Hull.Faces.Length;
            if (V.Length < NV)
            {
                V = new Vector3F[NV];
            }

            if (N.Length < NF)
            {
                N = new Vector3F[NF];
                D = new float[NF];
            }

            Vector3F sum = default;
            for (int i = 0; i < NV; i++)
            {
                Vector3F v = PolyVertex(s, Hull, i);
                V[i] = transform ? (r * v) + t : v;
                sum += V[i];
            }

            Center = sum / NV;
            for (int f = 0; f < NF; f++)
            {
                Vector3F n = Hull.Faces[f].Normal;
                N[f] = transform ? r * n : n;
                D[f] = Vector3F.Dot(N[f], V[Hull.Edges[Hull.Faces[f].FirstEdge].Origin]);
            }

            float m = 0f;
            for (int i = 0; i < NV; i++)
            {
                m = Math.Max(m, (V[i] - Center).LengthSquared());
            }

            Radius = ConvexShape3D.Sqrt(m);
        }
    }

    private readonly PolyData _pa = new();
    private readonly PolyData _pb = new();

    // Box and hull pairs: separating axis test over the faces of both and the edge pairs that form faces of the Minkowski difference (Gregorius, GDC 2013),
    // then clipping. Penetrating cores always take this route; separated cores keep it only when the best axis is also the true closest direction, otherwise GJK decides.
    private Manifold3D PolyPoly(in ConvexShape3D a, in ConvexShape3D b, in RigidTransformF bInA, float margin)
    {
        float rA = a.ConvexRadius, rB = b.ConvexRadius, rr = rA + rB;
        Matrix3x3F r = bInA.Rotation.ToMatrix();
        _pa.Load(a, false, r, default);
        _pb.Load(b, true, r, bInA.Translation);
        PolyData pa = _pa, pb = _pb;
        float scale = pa.Radius + pb.Radius + Math.Max(1e-20f, (pb.Center - pa.Center).Length());
        float tol = 5e-4f * (pa.Radius + pb.Radius);
        float limit = rr + margin;

        float sepA = FaceQuery(pa, pb, out int faceA);
        if (sepA > limit)
        {
            return Empty;
        }

        float sepB = FaceQuery(pb, pa, out int faceB);
        if (sepB > limit)
        {
            return Empty;
        }

        float sepE = EdgeQuery(pa, pb, out int edgeA, out int edgeB, out Vector3F axisE);

        bool useB = sepB > sepA + (0.005f * Math.Abs(sepA)) + (0.2f * tol);
        float bestFace = useB ? sepB : sepA;
        bool useEdge = edgeA >= 0 && sepE > bestFace + (0.02f * Math.Abs(bestFace)) + tol;
        float best = useEdge ? sepE : bestFace;
        if (best > limit)
        {
            return Empty;
        }

        ConvexContact3D c = default;
        bool haveC = false;
        if (best > 0f)
        {
            // Separated cores: the axis test may under-estimate the gap near vertex-vertex and vertex-edge closest pairs. Ask GJK and defer to it when it disagrees.
            c = _epa.Collide(a, b, bInA);
            haveC = true;
            Vector3F nSat = useEdge ? axisE : (useB ? -pb.N[faceB] : pa.N[faceA]);
            if (c.Separation - (best - rr) > 2e-4f * scale || Vector3F.Dot(nSat, c.Normal) < 0.9995f)
            {
                return Generic(a, b, bInA, c, margin);
            }
        }

        _count = 0;
        if (useEdge)
        {
            // Edge against edge: one point between the two edges.
            Vector3F n = axisE;
            _fa.Reset();
            _fb.Reset();
            _fa.Kind = Feat.Segment;
            _fb.Kind = Feat.Segment;
            AddEdge(_fa, pa, edgeA, n * rA);
            AddEdge(_fb, pb, edgeB, -n * rB);
            if (haveC)
            {
                SetHint(c, a, b);
            }

            return FromFeatures(_fa, _fb, n, margin);
        }

        bool refIsA = !useB;
        PolyData rp = refIsA ? pa : pb, ip = refIsA ? pb : pa;
        int rf = refIsA ? faceA : faceB;
        float rRef = refIsA ? rA : rB, rInc = refIsA ? rB : rA;
        Vector3F nRef = rp.N[rf];
        int inc = IncidentFace(ip, nRef);
        _fa.Reset();
        _fb.Reset();
        Feat fr = refIsA ? _fa : _fb, fi = refIsA ? _fb : _fa;
        AddFace(fr, rp, rf, nRef * rRef);
        AddFace(fi, ip, inc, -nRef * rInc);
        Vector3F nAB = refIsA ? nRef : -nRef;
        if (haveC)
        {
            SetHint(c, a, b);
        }

        Manifold3D m = FromFeatures(_fa, _fb, nAB, margin);
        if (m.PointCount == 0)
        {
            ConvexContact3D fallback = haveC ? c : _epa.Collide(a, b, bInA);
            return FromContact(fallback, _fa.Code(), _fb.Code(), margin);
        }

        return m;
    }

    private static void AddEdge(Feat f, PolyData p, int halfEdge, Vector3F shift)
    {
        ReadOnlySpan<HullEdge3D> edges = p.Hull.Edges;
        int o = edges[halfEdge].Origin, e = edges[halfEdge ^ 1].Origin;
        ushort code = FCode.Edge(halfEdge >> 1);
        f.Add(p.V[o] + shift, FCode.Vertex(o), code);
        f.Add(p.V[e] + shift, FCode.Vertex(e), code);
    }

    private static void AddFace(Feat f, PolyData p, int face, Vector3F shift)
    {
        ReadOnlySpan<HullEdge3D> edges = p.Hull.Edges;
        HullFace3D hf = p.Hull.Faces[face];
        f.Kind = Feat.Polygon;
        f.Face = FCode.Face(face);
        int e = hf.FirstEdge;
        for (int k = 0; k < hf.EdgeCount; k++)
        {
            int vi = edges[e].Origin;
            f.Add(p.V[vi] + shift, FCode.Vertex(vi), FCode.Edge(e >> 1));
            e = edges[e].Next;
        }
    }

    // The face of `p` most opposed to the reference normal.
    private static int IncidentFace(PolyData p, Vector3F nRef)
    {
        int best = 0;
        float bd = float.PositiveInfinity;
        for (int f = 0; f < p.NF; f++)
        {
            float d = Vector3F.Dot(p.N[f], nRef);
            if (d < bd)
            {
                bd = d;
                best = f;
            }
        }

        return best;
    }

    // Largest separation of the faces of p against q: for each face the distance from its plane to the support point of q behind it.
    private static float FaceQuery(PolyData p, PolyData q, out int face)
    {
        face = 0;
        float best = float.NegativeInfinity;
        for (int f = 0; f < p.NF; f++)
        {
            Vector3F n = p.N[f];
            float lo = float.PositiveInfinity;
            for (int i = 0; i < q.NV; i++)
            {
                float d = Vector3F.Dot(n, q.V[i]);
                if (d < lo)
                {
                    lo = d;
                }
            }

            float sep = lo - p.D[f];
            if (sep > best)
            {
                best = sep;
                face = f;
            }
        }

        return best;
    }

    // Edge pairs that span a face of the Minkowski difference: the arcs of the face normals of the two edges (one negated) must cross on the Gauss map
    // (Gregorius, GDC 2013, "The Separating Axis Test between Convex Polyhedra"). Only those pairs can give a separating axis; parallel pairs have no axis.
    private static float EdgeQuery(PolyData p, PolyData q, out int edgeP, out int edgeQ, out Vector3F axis)
    {
        edgeP = -1;
        edgeQ = -1;
        axis = default;
        float best = float.NegativeInfinity;
        ReadOnlySpan<HullEdge3D> ep = p.Hull.Edges, eq = q.Hull.Edges;
        for (int i = 0; i < ep.Length; i += 2)
        {
            Vector3F a = p.N[ep[i].Face], b = p.N[ep[i + 1].Face];
            Vector3F bxa = Vector3F.Cross(b, a);
            Vector3F pi0 = p.V[ep[i].Origin];
            Vector3F dirP = p.V[ep[i + 1].Origin] - pi0;
            float lenP2 = dirP.LengthSquared();
            for (int j = 0; j < eq.Length; j += 2)
            {
                Vector3F c = -q.N[eq[j].Face], d = -q.N[eq[j + 1].Face];
                Vector3F dxc = Vector3F.Cross(d, c);
                float adc = Vector3F.Dot(a, dxc), bdc = Vector3F.Dot(b, dxc);
                float cba = Vector3F.Dot(c, bxa), dba = Vector3F.Dot(d, bxa);
                if (!(cba * dba < 0f && adc * bdc < 0f))
                {
                    continue;
                }

                // The great circles cross at two antipodal points; the arcs meet only if one of them lies on both. Where each arc crosses the other's plane
                // (a positive mix of its ends), the two crossing points must point the same way.
                Vector3F xa = a + ((b - a) * (adc / (adc - bdc)));
                Vector3F xc = c + ((d - c) * (cba / (cba - dba)));
                if (!(Vector3F.Dot(xa, xc) > 0f))
                {
                    continue;
                }

                Vector3F qj0 = q.V[eq[j].Origin];
                Vector3F dirQ = q.V[eq[j + 1].Origin] - qj0;
                Vector3F ax = Vector3F.Cross(dirP, dirQ);
                float l2 = ax.LengthSquared();
                if (!(l2 > 1e-10f * lenP2 * dirQ.LengthSquared()))
                {
                    continue;
                }

                ax /= ConvexShape3D.Sqrt(l2);
                if (Vector3F.Dot(ax, pi0 - p.Center) < 0f)
                {
                    ax = -ax;
                }

                float sep = Vector3F.Dot(ax, qj0 - pi0);
                if (sep > best)
                {
                    best = sep;
                    edgeP = i;
                    edgeQ = j;
                    axis = ax;
                }
            }
        }

        return best;
    }
}
