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

        // Per edge pair (one entry for each pair of half-edges) of this polyhedron when it is the second argument of EdgeQuery: the negated face normals, their cross product, the first vertex, the direction and its squared length.
        public Vector3F[] EC = new Vector3F[24], ED = new Vector3F[24], EDxC = new Vector3F[24], EQ0 = new Vector3F[24], EDir = new Vector3F[24];
        public float[] EDir2 = new float[24];

        public void PrepareEdges()
        {
            ReadOnlySpan<HullEdge3D> e = Hull.Edges;
            int n = e.Length >> 1;
            if (EC.Length < n)
            {
                EC = new Vector3F[n]; ED = new Vector3F[n]; EDxC = new Vector3F[n]; EQ0 = new Vector3F[n]; EDir = new Vector3F[n]; EDir2 = new float[n];
            }

            for (int j = 0; j < n; j++)
            {
                Vector3F c = -N[e[2 * j].Face], d = -N[e[2 * j + 1].Face];
                EC[j] = c; ED[j] = d; EDxC[j] = Vector3F.Cross(d, c);
                Vector3F q0 = V[e[2 * j].Origin];
                EQ0[j] = q0;
                Vector3F dir = V[e[2 * j + 1].Origin] - q0;
                EDir[j] = dir; EDir2[j] = dir.LengthSquared();
            }
        }

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

    private const float EdgeSkipSafety = 8f;

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

        float sepE;
        int edgeA, edgeB;
        Vector3F axisE;
        // The edge search is skipped while the relative motion since the last full query (translation plus rotation times the shapes' radii) stays below an eighth of the margin by which
        // the face axes led; an edge axis moves with the pose about as fast as the pose itself, so it cannot close that margin unnoticed.
        bool skipEdges = _edgeHint.Gap > 0f && _edgeHint.Gap > EdgeSkipSafety * _edgeHint.PoseChange(bInA, pa.Radius + pb.Radius);
        if (skipEdges)
        {
            sepE = float.NegativeInfinity; edgeA = -1; edgeB = -1; axisE = default;
            HintOut = _edgeHint;
        }
        else
        {
            sepE = EdgeQuery(pa, pb, out edgeA, out edgeB, out axisE);
            HintOut = default;
        }

        bool useB = sepB > sepA + (0.005f * Math.Abs(sepA)) + (0.2f * tol);
        float bestFace = useB ? sepB : sepA;
        bool useEdge = edgeA >= 0 && sepE > bestFace + (0.02f * Math.Abs(bestFace)) + tol;
        if (!skipEdges && !useEdge)
        {
            // How far the best edge axis is below the threshold at which it would be used. With no edge pair spanning a face of the Minkowski difference there is no edge axis to measure, but a turn
            // of the shapes can create one, so the margin is capped at the shapes' size: no edge axis can sit farther from the face axes than that.
            float reach = pa.Radius + pb.Radius;
            float gap = edgeA < 0 ? reach : Math.Min(reach, bestFace + (0.02f * Math.Abs(bestFace)) + tol - sepE);
            if (gap > 0f) { HintOut = new EdgeHint3D(gap, bInA); }
        }
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
        ReadOnlySpan<HullEdge3D> ep = p.Hull.Edges;
        int nq = q.Hull.Edges.Length >> 1;
        q.PrepareEdges();
        for (int i = 0; i < ep.Length; i += 2)
        {
            Vector3F a = p.N[ep[i].Face], b = p.N[ep[i + 1].Face];
            Vector3F bxa = Vector3F.Cross(b, a);
            Vector3F pi0 = p.V[ep[i].Origin];
            Vector3F dirP = p.V[ep[i + 1].Origin] - pi0;
            float lenP2 = dirP.LengthSquared();
            for (int j = 0; j < nq; j++)
            {
                Vector3F c = q.EC[j], d = q.ED[j];
                Vector3F dxc = q.EDxC[j];
                // Either test rejects most pairs, so the second pair of dot products is only computed for the ones that pass the first.
                float cba = Vector3F.Dot(c, bxa), dba = Vector3F.Dot(d, bxa);
                if (!(cba * dba < 0f))
                {
                    continue;
                }

                float adc = Vector3F.Dot(a, dxc), bdc = Vector3F.Dot(b, dxc);
                if (!(adc * bdc < 0f))
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

                Vector3F qj0 = q.EQ0[j];
                Vector3F ax = Vector3F.Cross(dirP, q.EDir[j]);
                float l2 = ax.LengthSquared();
                if (!(l2 > 1e-10f * lenP2 * q.EDir2[j]))
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
                    edgeQ = 2 * j;
                    axis = ax;
                }
            }
        }

        return best;
    }
}
