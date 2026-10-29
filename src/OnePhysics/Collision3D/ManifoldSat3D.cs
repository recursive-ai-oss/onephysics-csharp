using System;
using System.Numerics;
using System.Runtime.Intrinsics;
using RecursiveAI.OneMath;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

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

        // The same data as component arrays padded with zeros to a multiple of four, for the four-lane edge search: the negated face normals (C, D), their cross product (E), the direction (G) and the first vertex (Q).
        public float[] Cx = new float[24], Cy = new float[24], Cz = new float[24], Dx = new float[24], Dy = new float[24], Dz = new float[24], Ex = new float[24], Ey = new float[24], Ez = new float[24];
        public float[] Gx = new float[24], Gy = new float[24], Gz = new float[24], Qx = new float[24], Qy = new float[24], Qz = new float[24], G2 = new float[24];

        // PrepareEdges is a function of what Load wrote, so a polyhedron that stays loaded keeps its edge data.
        private bool _edgesReady;

        public void PrepareEdges()
        {
            if (_edgesReady) { return; }
            _edgesReady = true;
            ReadOnlySpan<HullEdge3D> e = Hull.Edges;
            int n = e.Length >> 1;
            if (EC.Length < n)
            {
                EC = new Vector3F[n]; ED = new Vector3F[n]; EDxC = new Vector3F[n]; EQ0 = new Vector3F[n]; EDir = new Vector3F[n]; EDir2 = new float[n];
            }

            int padded = (n + 3) & ~3;
            if (Cx.Length < padded)
            {
                Cx = new float[padded]; Cy = new float[padded]; Cz = new float[padded]; Dx = new float[padded]; Dy = new float[padded]; Dz = new float[padded]; Ex = new float[padded]; Ey = new float[padded]; Ez = new float[padded];
                Gx = new float[padded]; Gy = new float[padded]; Gz = new float[padded]; Qx = new float[padded]; Qy = new float[padded]; Qz = new float[padded]; G2 = new float[padded];
            }

            // Lanes past the last edge pair hold zeros: the sign tests multiply to zero there, which neither accepts.
            for (int j = n; j < padded; j++) { Cx[j] = 0f; Cy[j] = 0f; Cz[j] = 0f; Dx[j] = 0f; Dy[j] = 0f; Dz[j] = 0f; Ex[j] = 0f; Ey[j] = 0f; Ez[j] = 0f; Gx[j] = 0f; Gy[j] = 0f; Gz[j] = 0f; Qx[j] = 0f; Qy[j] = 0f; Qz[j] = 0f; G2[j] = 0f; }

            for (int j = 0; j < n; j++)
            {
                Vector3F c = -N[e[2 * j].Face], d = -N[e[2 * j + 1].Face];
                EC[j] = c; ED[j] = d; EDxC[j] = Vector3F.Cross(d, c);
                Vector3F q0 = V[e[2 * j].Origin];
                EQ0[j] = q0;
                Vector3F dir = V[e[2 * j + 1].Origin] - q0;
                EDir[j] = dir; EDir2[j] = dir.LengthSquared();
                Cx[j] = c.X; Cy[j] = c.Y; Cz[j] = c.Z; Dx[j] = d.X; Dy[j] = d.Y; Dz[j] = d.Z; Ex[j] = EDxC[j].X; Ey[j] = EDxC[j].Y; Ez[j] = EDxC[j].Z;
                Gx[j] = dir.X; Gy[j] = dir.Y; Gz[j] = dir.Z; Qx[j] = q0.X; Qy[j] = q0.Y; Qz[j] = q0.Z; G2[j] = EDir2[j];
            }
        }

        public void Load(in ConvexShape3D s, bool transform, in Matrix3x3F r, Vector3F t)
        {
            _edgesReady = false;
            var hull = HullOf(s);
            Hull = hull;
            ReadOnlySpan<Vector3F> hv = hull.Vertices;
            ReadOnlySpan<HullFace3D> hf = hull.Faces;
            ReadOnlySpan<HullEdge3D> he = hull.Edges;
            int nv = hv.Length, nf = hf.Length;
            NV = nv;
            NF = nf;
            if (V.Length < nv)
            {
                V = new Vector3F[nv];
            }

            if (N.Length < nf)
            {
                N = new Vector3F[nf];
                D = new float[nf];
            }

            // A box is the unit cube's hull scaled by its half extents; the scaling and the placement are done per vertex exactly as PolyVertex and the matrix product do them.
            bool box = s.Kind == ConvexShapeKind3D.Box;
            Vector3F size = s.CoreSize;
            Vector3F[] vs = V, ns = N;
            float[] ds = D;
            Vector3F sum = default;
            bool placedBox = box && transform;
            if (placedBox)
            {
                // A placed box is its centre plus or minus its three half-extent columns: the corners are sums of the scaled columns with signs, not eight matrix products.
                Vector3F c0 = r.Transform(new Vector3F(size.X, 0f, 0f)), c1 = r.Transform(new Vector3F(0f, size.Y, 0f)), c2 = r.Transform(new Vector3F(0f, 0f, size.Z));
                for (int i = 0; i < nv; i++)
                {
                    Vector3F h = hv[i];
                    vs[i] = new Vector3F(
                        t.X + (h.X > 0f ? c0.X : -c0.X) + (h.Y > 0f ? c1.X : -c1.X) + (h.Z > 0f ? c2.X : -c2.X),
                        t.Y + (h.X > 0f ? c0.Y : -c0.Y) + (h.Y > 0f ? c1.Y : -c1.Y) + (h.Z > 0f ? c2.Y : -c2.Y),
                        t.Z + (h.X > 0f ? c0.Z : -c0.Z) + (h.Y > 0f ? c1.Z : -c1.Z) + (h.Z > 0f ? c2.Z : -c2.Z));
                }

                Center = t;
            }
            else
            {
                for (int i = 0; i < nv; i++)
                {
                    Vector3F v = box ? hv[i] * size : hv[i];
                    if (transform) { v = r.Transform(v) + t; }
                    vs[i] = v;
                    sum += v;
                }

                Center = sum / nv;
            }

            for (int f = 0; f < nf; f++)
            {
                Vector3F n = hf[f].Normal;
                if (transform) { n = r.Transform(n); }
                ns[f] = n;
                ds[f] = Vector3F.Dot(n, vs[he[hf[f].FirstEdge].Origin]);
            }

            if (box)
            {
                // Every corner of a box is the same distance from its centre.
                Radius = ConvexShape3D.Sqrt(size.LengthSquared());
                return;
            }

            float m = 0f;
            Vector3F c = Center;
            for (int i = 0; i < nv; i++)
            {
                m = Math.Max(m, (vs[i] - c).LengthSquared());
            }

            Radius = ConvexShape3D.Sqrt(m);
        }
    }

    private const float EdgeSkipSafety = 8f;

    private readonly PolyData _pa = new();
    private bool _paLoaded;
    private ConvexHull3D? _paHull;
    private Vector3F _paSize;
    private readonly PolyData _pb = new();

    // Box and hull pairs: separating axis test over the faces of both and the edge pairs that form faces of the Minkowski difference (Gregorius, GDC 2013),
    // then clipping. Penetrating cores always take this route; separated cores keep it only when the best axis is also the true closest direction, otherwise GJK decides.
    private Manifold3D PolyPoly(in ConvexShape3D a, in ConvexShape3D b, in RigidTransformF bInA, float margin)
    {
        float rA = a.ConvexRadius, rB = b.ConvexRadius, rr = rA + rB;
        Matrix3x3F r = bInA.Rotation.ToMatrix();
        PolyData pa = _pa, pb = _pb;
        bool boxes = a.Kind == ConvexShapeKind3D.Box && b.Kind == ConvexShapeKind3D.Box;
        Vector3F tB = bInA.Translation;
        // The radii and the distance between centres are kept in locals: the cached polyhedron of A must not be touched by a pair that never loads it.
        float radA, radB, centreGap;
        float sepA, sepB;
        int faceA, faceB;
        if (boxes)
        {
            // Two boxes: the face axes come from the boxes' half extents and the relative rotation; neither box is expanded into vertices unless the test finds a contact to build.
            BoxFaces(a.CoreSize, b.CoreSize, r, tB, out sepA, out faceA, out sepB, out faceB);
            radA = ConvexShape3D.Sqrt(a.CoreSize.LengthSquared()); radB = ConvexShape3D.Sqrt(b.CoreSize.LengthSquared());
            centreGap = tB.Length();
        }
        else
        {
            LoadPair(a, b, r, tB);
            sepA = float.NaN; sepB = float.NaN; faceA = 0; faceB = 0;
            radA = pa.Radius; radB = pb.Radius;
            centreGap = (pb.Center - pa.Center).Length();
        }

        float scale = radA + radB + Math.Max(1e-20f, centreGap);
        float tol = 5e-4f * (radA + radB);
        float limit = rr + margin;

        if (!boxes) { sepA = FaceQuery(pa, pb, out faceA); }
        if (sepA > limit)
        {
            // The face plane separates the cores by sepA, so the shapes are at least this far apart whatever the edges say.
            ClearanceOut = sepA - limit;
            return Empty;
        }

        if (!boxes) { sepB = FaceQuery(pb, pa, out faceB); }
        if (sepB > limit)
        {
            ClearanceOut = sepB - limit;
            return Empty;
        }

        float sepE;
        int edgeA, edgeB;
        Vector3F axisE;
        // The edge search is skipped while the relative motion since the last full query (translation plus rotation times the shapes' radii) stays below an eighth of the margin by which
        // the face axes led; an edge axis moves with the pose about as fast as the pose itself, so it cannot close that margin unnoticed.
        bool skipEdges = _edgeHint.Gap > 0f && _edgeHint.Gap > EdgeSkipSafety * _edgeHint.PoseChange(bInA, radA + radB);
        if (skipEdges)
        {
            sepE = float.NegativeInfinity; edgeA = -1; edgeB = -1; axisE = default;
            HintOut = _edgeHint;
        }
        else
        {
            sepE = boxes ? BoxEdges(a.CoreSize, b.CoreSize, r, tB, out edgeA, out edgeB, out axisE) : EdgeQuery(pa, pb, out edgeA, out edgeB, out axisE);
            HintOut = default;
        }

        bool useB = sepB > sepA + (0.005f * Math.Abs(sepA)) + (0.2f * tol);
        float bestFace = useB ? sepB : sepA;
        bool useEdge = edgeA >= 0 && sepE > bestFace + (0.02f * Math.Abs(bestFace)) + tol;
        if (!skipEdges && !useEdge)
        {
            // How far the best edge axis is below the threshold at which it would be used. With no edge pair spanning a face of the Minkowski difference there is no edge axis to measure, but a turn
            // of the shapes can create one, so the margin is capped at the shapes' size: no edge axis can sit farther from the face axes than that.
            float reach = radA + radB;
            float gap = edgeA < 0 ? reach : Math.Min(reach, bestFace + (0.02f * Math.Abs(bestFace)) + tol - sepE);
            if (gap > 0f) { HintOut = new EdgeHint3D(gap, bInA); }
        }
        float best = useEdge ? sepE : bestFace;
        if (best > limit)
        {
            return Empty;
        }

        if (boxes && !useEdge && BoxFaceManifold(a, b, r, tB, useB, faceA, faceB, best, rA, rB, margin, out Manifold3D boxFace)) { return boxFace; }
        if (boxes) { LoadPair(a, b, r, tB); }
        ConvexContact3D c = default;
        bool haveC = false;
        // A face axis whose nearest vertex of the other core lies over the face itself measures the true distance: that vertex and its projection on the face are the closest points of the two cores,
        // exactly what GJK would return, so they are written down directly. Only the other cases (a vertex or edge past the face's rim, where the axis test can under-estimate the gap) need GJK to confirm.
        if (best > 0f && !useEdge && NearestVertexOverFace(useB ? pb : pa, useB ? pa : pb, useB ? faceB : faceA, out Vector3F nearest))
        {
            Vector3F nFace = useB ? -pb.N[faceB] : pa.N[faceA];
            Vector3F coreA = useB ? nearest : nearest - (nFace * best);
            Vector3F coreB = useB ? nearest + (nFace * best) : nearest;
            c = new ConvexContact3D(best - rr, nFace, coreA + (nFace * rA), coreB - (nFace * rB));
            haveC = true;
        }
        else if (best > 0f && useEdge && EdgeEdgeContact(pa, pb, edgeA, edgeB, axisE, best, rr, rA, rB, scale, out c))
        {
            haveC = true;
        }
        else if (best > 0f)
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

    // Shape A is loaded in its own frame, so what Load makes depends on the shape alone; consecutive pairs of a chunk often share A (pairs are listed by A first), and the work is skipped for them.
    // Only boxes are remembered: their hull is the shared unit cube, whereas a hull-kind shape may be a reused buffer (the triangle of a mesh contact) whose points change between calls.
    private void LoadPair(in ConvexShape3D a, in ConvexShape3D b, in Matrix3x3F r, Vector3F t)
    {
        bool keepA = _pinSide == 1 && _pinned, keepB = _pinSide == 2 && _pinned;
        if (_pinSide != 0) { _pinned = true; }
        if (!keepA && !(_paLoaded && a.Kind == ConvexShapeKind3D.Box && _paHull != null && a.CoreSize.X == _paSize.X && a.CoreSize.Y == _paSize.Y && a.CoreSize.Z == _paSize.Z))
        {
            _pa.Load(a, false, r, default);
            _paHull = a.Kind == ConvexShapeKind3D.Box ? s_unitBox : null;
            _paSize = a.CoreSize;
            _paLoaded = true;
        }

        if (!keepB) { _pb.Load(b, true, r, t); }
    }

    private int _pinSide;
    private bool _pinned;

    /// <summary>Declares that the following calls share one convex shape and one relative pose, and only the other shape changes (a convex shape against the triangles of a mesh): the shared shape on the given side (1 = A, 2 = B) is loaded for the first call and kept for the rest. End with <see cref="EndPin"/>.</summary>
    internal void BeginPin(int side) { _pinSide = side; _pinned = false; }

    /// <summary>Ends <see cref="BeginPin"/>.</summary>
    internal void EndPin() { _pinSide = 0; _pinned = false; }

    // Whether the vertex of q nearest to the plane of p's face lies, projected along the face normal, inside the face's polygon (on its boundary counts).
    private static bool NearestVertexOverFace(PolyData p, PolyData q, int face, out Vector3F nearest)
    {
        Vector3F n = p.N[face];
        int v = 0;
        float lo = float.PositiveInfinity;
        for (int i = 0; i < q.NV; i++)
        {
            float d = Vector3F.Dot(n, q.V[i]);
            if (d < lo) { lo = d; v = i; }
        }

        Vector3F x = q.V[v];
        nearest = x;
        ReadOnlySpan<HullEdge3D> edges = p.Hull.Edges;
        HullFace3D hf = p.Hull.Faces[face];
        int count = hf.EdgeCount;
        Vector3F cen = default;
        int e = hf.FirstEdge;
        for (int k = 0; k < count; k++) { cen += p.V[edges[e].Origin]; e = edges[e].Next; }
        cen /= count;
        e = hf.FirstEdge;
        for (int k = 0; k < count; k++)
        {
            Vector3F a = p.V[edges[e].Origin], b = p.V[edges[edges[e].Next].Origin];
            Vector3F s = Vector3F.Cross(b - a, n);
            if (Vector3F.Dot(s, cen - a) < 0f) { s = -s; }
            if (Vector3F.Dot(s, x - a) < 0f) { return false; }
            e = edges[e].Next;
        }

        return true;
    }

    // Separated cores whose best axis is an edge pair: the closest points of the two edge segments (Ericson, Real-Time Collision Detection, 5.1.9) are the closest points of the cores when the gap between
    // them equals the axis separation and points along the axis (the axis test is a lower bound on the distance and the segment pair an upper bound, so when they meet that is the distance), which is what
    // GJK would return. Returns false when they do not meet (a vertex or an edge end is the nearest feature), and GJK decides.
    private static bool EdgeEdgeContact(PolyData pa, PolyData pb, int edgeA, int edgeB, Vector3F axis, float best, float rr, float rA, float rB, float scale, out ConvexContact3D c)
    {
        c = default;
        ReadOnlySpan<HullEdge3D> ea = pa.Hull.Edges, eb = pb.Hull.Edges;
        Vector3F a0 = pa.V[ea[edgeA].Origin], a1 = pa.V[ea[edgeA ^ 1].Origin];
        Vector3F b0 = pb.V[eb[edgeB].Origin], b1 = pb.V[eb[edgeB ^ 1].Origin];
        ClosestSegmentSegment(a0, a1, b0, b1, out float s, out float t);
        Vector3F p = a0 + ((a1 - a0) * s), q = b0 + ((b1 - b0) * t);
        Vector3F d = q - p;
        float len2 = d.LengthSquared();
        if (!(len2 > 1e-12f) || !float.IsFinite(len2)) { return false; }
        float dist = ConvexShape3D.Sqrt(len2);
        if (dist - best > 2e-4f * scale) { return false; }
        Vector3F n = d / dist;
        if (Vector3F.Dot(axis, n) < 0.9995f) { return false; }
        c = new ConvexContact3D(dist - rr, n, p + (n * rA), q - (n * rB));
        return true;
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
        Vector128<float> signBit = Vector128.Create(-0f);
        Vector128<float> zero = Vector128<float>.Zero;
        for (int i = 0; i < ep.Length; i += 2)
        {
            Vector3F a = p.N[ep[i].Face], b = p.N[ep[i + 1].Face];
            Vector3F bxa = Vector3F.Cross(b, a);
            Vector3F pi0 = p.V[ep[i].Origin];
            Vector3F dirP = p.V[ep[i + 1].Origin] - pi0;
            float lenP2 = dirP.LengthSquared();
            Vector3F side = pi0 - p.Center, ba = b - a;
            Vector128<float> bxX = Vector128.Create(bxa.X), bxY = Vector128.Create(bxa.Y), bxZ = Vector128.Create(bxa.Z);
            Vector128<float> aX = Vector128.Create(a.X), aY = Vector128.Create(a.Y), aZ = Vector128.Create(a.Z);
            Vector128<float> bX = Vector128.Create(b.X), bY = Vector128.Create(b.Y), bZ = Vector128.Create(b.Z);
            Vector128<float> baX = Vector128.Create(ba.X), baY = Vector128.Create(ba.Y), baZ = Vector128.Create(ba.Z);
            Vector128<float> dpX = Vector128.Create(dirP.X), dpY = Vector128.Create(dirP.Y), dpZ = Vector128.Create(dirP.Z);
            Vector128<float> sdX = Vector128.Create(side.X), sdY = Vector128.Create(side.Y), sdZ = Vector128.Create(side.Z);
            Vector128<float> p0X = Vector128.Create(pi0.X), p0Y = Vector128.Create(pi0.Y), p0Z = Vector128.Create(pi0.Z);
            Vector128<float> thr = Vector128.Create(1e-10f * lenP2);
            for (int j0 = 0; j0 < nq; j0 += 4)
            {
                // Four edge pairs of q at a time. Every lane does exactly the scalar arithmetic of one pair (Gregorius's arc test, then the axis and its separation); lanes past the end are zeros
                // that fail the first test. Candidates are compared in ascending pair order, so ties go to the lowest index as in a one-at-a-time search.
                var cx = Vector128.Create(q.Cx, j0); var cy = Vector128.Create(q.Cy, j0); var cz = Vector128.Create(q.Cz, j0);
                var dx = Vector128.Create(q.Dx, j0); var dy = Vector128.Create(q.Dy, j0); var dz = Vector128.Create(q.Dz, j0);
                var cba = (cx * bxX) + (cy * bxY) + (cz * bxZ);
                var dba = (dx * bxX) + (dy * bxY) + (dz * bxZ);
                var m = Vector128.LessThan(cba * dba, zero);
                var ex = Vector128.Create(q.Ex, j0); var ey = Vector128.Create(q.Ey, j0); var ez = Vector128.Create(q.Ez, j0);
                var adc = (aX * ex) + (aY * ey) + (aZ * ez);
                var bdc = (bX * ex) + (bY * ey) + (bZ * ez);
                m &= Vector128.LessThan(adc * bdc, zero);
                if (m == zero) { continue; }

                // The great circles cross at two antipodal points; the arcs meet only if one of them lies on both. Where each arc crosses the other's plane
                // (a positive mix of its ends), the two crossing points must point the same way.
                var ta = adc / (adc - bdc);
                var tc = cba / (cba - dba);
                var xaX = aX + (baX * ta); var xaY = aY + (baY * ta); var xaZ = aZ + (baZ * ta);
                var xcX = cx + ((dx - cx) * tc); var xcY = cy + ((dy - cy) * tc); var xcZ = cz + ((dz - cz) * tc);
                m &= Vector128.GreaterThan((xaX * xcX) + (xaY * xcY) + (xaZ * xcZ), zero);
                var gx = Vector128.Create(q.Gx, j0); var gy = Vector128.Create(q.Gy, j0); var gz = Vector128.Create(q.Gz, j0);
                var axX = (dpY * gz) - (dpZ * gy); var axY = (dpZ * gx) - (dpX * gz); var axZ = (dpX * gy) - (dpY * gx);
                var l2 = (axX * axX) + (axY * axY) + (axZ * axZ);
                m &= Vector128.GreaterThan(l2, thr * Vector128.Create(q.G2, j0));
                if (m == zero) { continue; }

                var len = Vector128.Sqrt(l2);
                axX /= len; axY /= len; axZ /= len;
                var flip = Vector128.LessThan((axX * sdX) + (axY * sdY) + (axZ * sdZ), zero);
                axX = Vector128.ConditionalSelect(flip, axX ^ signBit, axX);
                axY = Vector128.ConditionalSelect(flip, axY ^ signBit, axY);
                axZ = Vector128.ConditionalSelect(flip, axZ ^ signBit, axZ);
                var sep = (axX * (Vector128.Create(q.Qx, j0) - p0X)) + (axY * (Vector128.Create(q.Qy, j0) - p0Y)) + (axZ * (Vector128.Create(q.Qz, j0) - p0Z));
                uint bits = m.ExtractMostSignificantBits();
                while (bits != 0)
                {
                    int lane = BitOperations.TrailingZeroCount(bits);
                    bits &= bits - 1;
                    float s = sep.GetElement(lane);
                    if (s > best)
                    {
                        best = s;
                        edgeP = i;
                        edgeQ = 2 * (j0 + lane);
                        axis = new Vector3F(axX.GetElement(lane), axY.GetElement(lane), axZ.GetElement(lane));
                    }
                }
            }
        }

        return best;
    }
}
