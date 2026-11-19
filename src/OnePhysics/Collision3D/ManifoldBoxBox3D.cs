using System;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class ManifoldGenerator3D
{
    // The separating axis test for two boxes, written for the box instead of run over a general polyhedron (Ericson, Real-Time Collision Detection, 4.4.1: the fifteen axes of two oriented boxes;
    // Gregorius, GDC 2013, for the choice of face and edge axes). A box's faces are a signed axis each and its twelve edges are four per axis, so the support of a box along any direction is a sum of
    // absolute values and nothing is loaded or transformed vertex by vertex. The answers (face index, edge index, separation) are the ones the general test gives, in the numbering of the shared unit cube.
    private sealed class BoxTopology
    {
        /// <summary>For each face of the unit cube: its axis (0, 1, 2) and the sign of its outward normal.</summary>
        public readonly int[] FaceAxis = new int[6];
        public readonly float[] FaceSign = new float[6];
        public readonly int[] SignedAxis = new int[6];

        /// <summary>The unit cube hull's outward face normals, its corners, and per face the four corners and edge codes (half-edge index / 2) in the order the face's edge loop visits them.</summary>
        public readonly Vector3F[] FaceNormal = new Vector3F[6];
        public readonly Vector3F[] Corner = new Vector3F[8];
        public readonly int[] FaceCorner = new int[24];
        public readonly int[] FaceEdge = new int[24];

        /// <summary>For each edge of each face's loop: the lateral axis it runs perpendicular to inside the face (the axis on which its two corners agree) and the sign of that coordinate.</summary>
        public readonly int[] FaceLatAxis = new int[24];
        public readonly float[] FaceLatSign = new float[24];

        /// <summary>The first half-edge of the cube's edge that runs along an axis at the corner whose signs on the other two axes (in increasing axis order) are the bits of the index (bit set = positive).</summary>
        public readonly int[] Edge = new int[12];

        public BoxTopology()
        {
            // Built again here (it is deterministic, so the numbering is the shared cube's) because static fields of a partial class initialize in no fixed order across files.
            ConvexHull3D hull = BuildUnitBox();
            ReadOnlySpan<HullFace3D> faces = hull.Faces;
            ReadOnlySpan<Vector3F> corners = hull.Vertices;
            for (int i = 0; i < 8; i++) { Corner[i] = corners[i]; }
            ReadOnlySpan<HullEdge3D> loopEdges = hull.Edges;
            for (int f = 0; f < faces.Length; f++)
            {
                FaceNormal[f] = faces[f].Normal;
                int le = faces[f].FirstEdge;
                for (int k = 0; k < 4; k++)
                {
                    FaceCorner[(4 * f) + k] = loopEdges[le].Origin;
                    FaceEdge[(4 * f) + k] = le >> 1;
                    le = loopEdges[le].Next;
                }

                Vector3F n = faces[f].Normal;
                int axis = Math.Abs(n.X) > 0.5f ? 0 : Math.Abs(n.Y) > 0.5f ? 1 : 2;
                float c = axis == 0 ? n.X : axis == 1 ? n.Y : n.Z;
                FaceAxis[f] = axis;
                FaceSign[f] = c > 0f ? 1f : -1f;
                SignedAxis[f] = 2 * axis + (c > 0f ? 0 : 1);
            }

            for (int f = 0; f < 6; f++)
            {
                for (int i = 0; i < 4; i++)
                {
                    Vector3F c1 = Corner[FaceCorner[(4 * f) + i]], c2 = Corner[FaceCorner[(4 * f) + ((i + 1) & 3)]];
                    int skip = FaceAxis[f];
                    int ax = skip != 0 && c1.X == c2.X ? 0 : skip != 1 && c1.Y == c2.Y ? 1 : 2;
                    FaceLatAxis[(4 * f) + i] = ax;
                    FaceLatSign[(4 * f) + i] = (ax == 0 ? c1.X : ax == 1 ? c1.Y : c1.Z) > 0f ? 1f : -1f;
                }
            }

            ReadOnlySpan<HullEdge3D> edges = hull.Edges;
            ReadOnlySpan<Vector3F> verts = hull.Vertices;
            for (int h = 0; h < edges.Length; h += 2)
            {
                Vector3F o = verts[edges[h].Origin], e = verts[edges[h + 1].Origin];
                int axis = o.X != e.X ? 0 : o.Y != e.Y ? 1 : 2;
                // The two other axes in increasing order: for axis 0 they are y, z; for axis 1 x, z; for axis 2 x, y.
                float s0 = axis == 0 ? o.Y : o.X;
                float s1 = axis == 2 ? o.Y : o.Z;
                Edge[(axis * 4) + (s0 > 0f ? 1 : 0) + (s1 > 0f ? 2 : 0)] = h;
            }
        }
    }

    private static readonly BoxTopology s_boxTopo = new();

    /// <summary>The axis tests of a box–box pair, computed ahead of the query (BoxAxesBatch): what BoxFaces and BoxEdges return for the pair's pose.</summary>
    internal struct BoxAxes
    {
        public float SepA, SepB, SepE;
        public int FaceA, FaceB, EdgeA, EdgeB;
        public Vector3F AxisE;
        public bool FacesDone, EdgesDone, SkipEdges;
    }

    // BoxFaces (edges false) or BoxEdges (edges true) for the queued pairs idx[t0 .. t0+w-1], one pair a lane (lanes past w repeat the first pair and are not stored). Every lane does the scalar code's operations in the scalar
    // order on the same floats, so the results are the same bits: ToMatrixInline, the reaches and offsets of BoxFaces, and for each of the nine edge axes l2, the cone tests, m, du, the reaches and sep.
    // Negation is a sign-bit flip (as the scalar unary minus), the products by +-1 are kept, sqrt is correctly rounded in both (the scalar one rounds a double sqrt of a float, which is the same).
    // The `continue` tests become a lane mask, and each best-axis choice is a select under (mask and sep > best) taken in the scalar loop's order, so ties and NaN go the same way.
    // The winner's normalization and edge numbering (a handful of operations on one axis) run per lane in scalar code, as in BoxEdges.
    [SkipLocalsInit]
    private static void BoxAxes256(Engine3D.BoxJob[] jobs, BoxAxes[] res, ReadOnlySpan<int> idx, int t0, int w, bool edges)
    {
        BoxTopology topo = s_boxTopo;
        Span<float> g = stackalloc float[13 * 8];
        for (int l = 0; l < 8; l++)
        {
            ref Engine3D.BoxJob j = ref jobs[idx[t0 + (l < w ? l : 0)]];
            QuaternionF q = j.BInA.Rotation;
            Vector3F t = j.BInA.Translation, ha = j.Ga.Convex.CoreSize, hb = j.Gb.Convex.CoreSize;
            g[l] = q.X; g[8 + l] = q.Y; g[16 + l] = q.Z; g[24 + l] = q.W;
            g[32 + l] = t.X; g[40 + l] = t.Y; g[48 + l] = t.Z;
            g[56 + l] = ha.X; g[64 + l] = ha.Y; g[72 + l] = ha.Z;
            g[80 + l] = hb.X; g[88 + l] = hb.Y; g[96 + l] = hb.Z;
        }

        ref float g0 = ref Mem.Ref(g);
        Vector256<float> qx = Vector256.LoadUnsafe(ref g0, 0), qy = Vector256.LoadUnsafe(ref g0, 8), qz = Vector256.LoadUnsafe(ref g0, 16), qw = Vector256.LoadUnsafe(ref g0, 24);
        Vector256<float> tx = Vector256.LoadUnsafe(ref g0, 32), ty = Vector256.LoadUnsafe(ref g0, 40), tz = Vector256.LoadUnsafe(ref g0, 48);
        Vector256<float> hAx = Vector256.LoadUnsafe(ref g0, 56), hAy = Vector256.LoadUnsafe(ref g0, 64), hAz = Vector256.LoadUnsafe(ref g0, 72);
        Vector256<float> hBx = Vector256.LoadUnsafe(ref g0, 80), hBy = Vector256.LoadUnsafe(ref g0, 88), hBz = Vector256.LoadUnsafe(ref g0, 96);
        Vector256<float> one = Vector256.Create(1f), two = Vector256.Create(2f), mone = Vector256.Create(-1f), sign = Vector256.Create(-0f);

        // ToMatrixInline.
        Vector256<float> xx = qx * qx, yy = qy * qy, zz = qz * qz;
        Vector256<float> xy = qx * qy, xz = qx * qz, yz = qy * qz;
        Vector256<float> wx = qw * qx, wy = qw * qy, wz = qw * qz;
        Vector256<float> m11 = one - two * (yy + zz), m12 = two * (xy - wz), m13 = two * (xz + wy);
        Vector256<float> m21 = two * (xy + wz), m22 = one - two * (xx + zz), m23 = two * (yz - wx);
        Vector256<float> m31 = two * (xz - wy), m32 = two * (yz + wx), m33 = one - two * (xx + yy);

        if (!edges)
        {
            BoxFaces256(res, idx, t0, w, m11, m12, m13, m21, m22, m23, m31, m32, m33, tx, ty, tz, hAx, hAy, hAz, hBx, hBy, hBz);
            return;
        }

        // BoxEdges: the nine axes in the scalar loop's order (i over A's axes, j over B's columns).
        const float cutMargin = ConeMargin * ConeMargin * 0.998f;
        Vector256<float> zero = Vector256<float>.Zero, tiny = Vector256.Create(1e-10f), cutM = Vector256.Create(cutMargin);
        Vector256<float> best = Vector256.Create(float.NegativeInfinity);
        Vector256<int> bij = Vector256<int>.Zero;
        Vector256<float> bs = one, bl = one, bl0 = zero, bl1 = zero, bl2 = zero, bm0 = zero, bm1 = zero, bm2 = zero;
        for (int i = 0; i < 3; i++)
        {
            for (int jj = 0; jj < 3; jj++)
            {
                Vector256<float> cX = jj == 0 ? m11 : jj == 1 ? m12 : m13;
                Vector256<float> cY = jj == 0 ? m21 : jj == 1 ? m22 : m23;
                Vector256<float> cZ = jj == 0 ? m31 : jj == 1 ? m32 : m33;
                Vector256<float> lX = i == 0 ? zero : i == 1 ? cZ : (cY ^ sign);
                Vector256<float> lY = i == 0 ? (cZ ^ sign) : i == 1 ? zero : cX;
                Vector256<float> lZ = i == 0 ? cY : i == 1 ? (cX ^ sign) : zero;
                Vector256<float> l2 = lX * lX + lY * lY + lZ * lZ;
                Vector256<float> cut = cutM * l2;
                Vector256<float> pX = lX * lX, pY = lY * lY, pZ = lZ * lZ;
                Vector256<float> coneA = i == 0 ? (Vector256.LessThan(pY, cut) | Vector256.LessThan(pZ, cut)) : i == 1 ? (Vector256.LessThan(pX, cut) | Vector256.LessThan(pZ, cut)) : (Vector256.LessThan(pX, cut) | Vector256.LessThan(pY, cut));
                Vector256<float> m0 = (m11 * lX) + (m21 * lY) + (m31 * lZ);
                Vector256<float> m1 = (m12 * lX) + (m22 * lY) + (m32 * lZ);
                Vector256<float> m2 = (m13 * lX) + (m23 * lY) + (m33 * lZ);
                Vector256<float> mx = m0 * m0, my = m1 * m1, mz = m2 * m2;
                Vector256<float> coneB = jj == 0 ? (Vector256.LessThan(my, cut) | Vector256.LessThan(mz, cut)) : jj == 1 ? (Vector256.LessThan(mx, cut) | Vector256.LessThan(mz, cut)) : (Vector256.LessThan(mx, cut) | Vector256.LessThan(my, cut));
                Vector256<float> valid = Vector256.AndNot(Vector256.AndNot(Vector256.GreaterThan(l2, tiny), coneA), coneB);
                Vector256<float> du = lX * tx + lY * ty + lZ * tz;
                Vector256<float> neg = Vector256.LessThan(du, zero);
                du ^= neg & sign;
                Vector256<float> sg = Vector256.ConditionalSelect(neg, mone, one);
                Vector256<float> reachA = (Vector256.Abs(lX) * hAx) + (Vector256.Abs(lY) * hAy) + (Vector256.Abs(lZ) * hAz);
                Vector256<float> reachB = (Vector256.Abs(m0) * hBx) + (Vector256.Abs(m1) * hBy) + (Vector256.Abs(m2) * hBz);
                Vector256<float> len = Vector256.Sqrt(l2);
                Vector256<float> sep = (du - reachA - reachB) / len;
                Vector256<float> take = valid & Vector256.GreaterThan(sep, best);
                best = Vector256.ConditionalSelect(take, sep, best);
                bij = Vector256.ConditionalSelect(take.AsInt32(), Vector256.Create((i * 3) + jj), bij);
                bs = Vector256.ConditionalSelect(take, sg, bs); bl = Vector256.ConditionalSelect(take, len, bl);
                bl0 = Vector256.ConditionalSelect(take, lX, bl0); bl1 = Vector256.ConditionalSelect(take, lY, bl1); bl2 = Vector256.ConditionalSelect(take, lZ, bl2);
                bm0 = Vector256.ConditionalSelect(take, m0, bm0); bm1 = Vector256.ConditionalSelect(take, m1, bm1); bm2 = Vector256.ConditionalSelect(take, m2, bm2);
            }
        }

        Span<float> o = stackalloc float[13 * 8];
        ref float o0 = ref Mem.Ref(o);
        best.StoreUnsafe(ref o0, 16); bs.StoreUnsafe(ref o0, 24); bl.StoreUnsafe(ref o0, 32);
        bl0.StoreUnsafe(ref o0, 40); bl1.StoreUnsafe(ref o0, 48); bl2.StoreUnsafe(ref o0, 56); bm0.StoreUnsafe(ref o0, 64); bm1.StoreUnsafe(ref o0, 72); bm2.StoreUnsafe(ref o0, 80);
        Span<int> oi = stackalloc int[3 * 8];
        ref int oi0 = ref Mem.Ref(oi);
        bij.StoreUnsafe(ref oi0, 16);
        for (int l = 0; l < w; l++)
        {
            ref BoxAxes r = ref res[idx[t0 + l]];
            r.EdgesDone = true;
            float bst = o[16 + l];
            r.SepE = bst;
            r.EdgeA = -1; r.EdgeB = -1; r.AxisE = default;
            if (bst > float.NegativeInfinity)
            {
                // BoxEdges' tail, on this lane's winner.
                int bi = oi[16 + l] / 3, bj = oi[16 + l] - (3 * bi);
                float sb = o[24 + l], len = o[32 + l];
                float nx = sb * o[40 + l] / len, ny = sb * o[48 + l] / len, nz = sb * o[56 + l] / len;
                float x0 = sb * o[64 + l] / len, x1 = sb * o[72 + l] / len, x2 = sb * o[80 + l] / len;
                r.AxisE = new Vector3F(nx, ny, nz);
                int ka = (bi == 0 ? (ny > 0f ? 1 : 0) + (nz > 0f ? 2 : 0) : bi == 1 ? (nx > 0f ? 1 : 0) + (nz > 0f ? 2 : 0) : (nx > 0f ? 1 : 0) + (ny > 0f ? 2 : 0));
                int kb = (bj == 0 ? (x1 < 0f ? 1 : 0) + (x2 < 0f ? 2 : 0) : bj == 1 ? (x0 < 0f ? 1 : 0) + (x2 < 0f ? 2 : 0) : (x0 < 0f ? 1 : 0) + (x1 < 0f ? 2 : 0));
                r.EdgeA = topo.Edge[(bi * 4) + ka];
                r.EdgeB = topo.Edge[(bj * 4) + kb];
            }
        }
    }

    // BoxFaces on eight lanes (see BoxAxes256).
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void BoxFaces256(BoxAxes[] res, ReadOnlySpan<int> idx, int t0, int w, Vector256<float> m11, Vector256<float> m12, Vector256<float> m13, Vector256<float> m21, Vector256<float> m22,
        Vector256<float> m23, Vector256<float> m31, Vector256<float> m32, Vector256<float> m33, Vector256<float> tx, Vector256<float> ty, Vector256<float> tz,
        Vector256<float> hAx, Vector256<float> hAy, Vector256<float> hAz, Vector256<float> hBx, Vector256<float> hBy, Vector256<float> hBz)
    {
        BoxTopology topo = s_boxTopo;
        Vector256<float> one = Vector256.Create(1f), mone = Vector256.Create(-1f), sign = Vector256.Create(-0f);
        Vector256<float> a00 = Vector256.Abs(m11), a01 = Vector256.Abs(m12), a02 = Vector256.Abs(m13);
        Vector256<float> a10 = Vector256.Abs(m21), a11 = Vector256.Abs(m22), a12 = Vector256.Abs(m23);
        Vector256<float> a20 = Vector256.Abs(m31), a21 = Vector256.Abs(m32), a22 = Vector256.Abs(m33);
        Vector256<float> rx = (a00 * hBx) + (a01 * hBy) + (a02 * hBz);
        Vector256<float> ry = (a10 * hBx) + (a11 * hBy) + (a12 * hBz);
        Vector256<float> rz = (a20 * hBx) + (a21 * hBy) + (a22 * hBz);
        Vector256<float> lx = (rx ^ sign) - hAx, ly = (ry ^ sign) - hAy, lz = (rz ^ sign) - hAz;
        Span<Vector256<float>> sv = stackalloc Vector256<float>[6];
        sv[0] = (one * tx) + lx; sv[1] = (mone * tx) + lx;
        sv[2] = (one * ty) + ly; sv[3] = (mone * ty) + ly;
        sv[4] = (one * tz) + lz; sv[5] = (mone * tz) + lz;
        Vector256<float> sepA = Vector256.Create(float.NegativeInfinity);
        Vector256<int> faceA = Vector256<int>.Zero;
        for (int f = 0; f < 6; f++)
        {
            Vector256<float> sep = sv[topo.SignedAxis[f]];
            Vector256<float> take = Vector256.GreaterThan(sep, sepA);
            sepA = Vector256.ConditionalSelect(take, sep, sepA);
            faceA = Vector256.ConditionalSelect(take.AsInt32(), Vector256.Create(f), faceA);
        }

        Vector256<float> ca0 = (a00 * hAx) + (a10 * hAy) + (a20 * hAz);
        Vector256<float> ca1 = (a01 * hAx) + (a11 * hAy) + (a21 * hAz);
        Vector256<float> ca2 = (a02 * hAx) + (a12 * hAy) + (a22 * hAz);
        Vector256<float> u0 = (m11 * tx) + (m21 * ty) + (m31 * tz);
        Vector256<float> u1 = (m12 * tx) + (m22 * ty) + (m32 * tz);
        Vector256<float> u2 = (m13 * tx) + (m23 * ty) + (m33 * tz);
        lx = (ca0 ^ sign) - hBx; ly = (ca1 ^ sign) - hBy; lz = (ca2 ^ sign) - hBz;
        sv[0] = (mone * u0) + lx; sv[1] = (one * u0) + lx;
        sv[2] = (mone * u1) + ly; sv[3] = (one * u1) + ly;
        sv[4] = (mone * u2) + lz; sv[5] = (one * u2) + lz;
        Vector256<float> sepB = Vector256.Create(float.NegativeInfinity);
        Vector256<int> faceB = Vector256<int>.Zero;
        for (int f = 0; f < 6; f++)
        {
            Vector256<float> sep = sv[topo.SignedAxis[f]];
            Vector256<float> take = Vector256.GreaterThan(sep, sepB);
            sepB = Vector256.ConditionalSelect(take, sep, sepB);
            faceB = Vector256.ConditionalSelect(take.AsInt32(), Vector256.Create(f), faceB);
        }

        Span<float> o = stackalloc float[16];
        Span<int> oi = stackalloc int[16];
        sepA.StoreUnsafe(ref Mem.Ref(o), 0); sepB.StoreUnsafe(ref Mem.Ref(o), 8);
        faceA.StoreUnsafe(ref Mem.Ref(oi), 0); faceB.StoreUnsafe(ref Mem.Ref(oi), 8);
        for (int l = 0; l < w; l++)
        {
            ref BoxAxes r = ref res[idx[t0 + l]];
            r.SepA = o[l]; r.FaceA = oi[l]; r.SepB = o[8 + l]; r.FaceB = oi[8 + l]; r.FacesDone = true; r.EdgesDone = false;
        }
    }

    /// <summary>Set by the caller for one Collide call of a box–box pair whose axis tests are already in <see cref="Pre"/>; PolyPoly then takes them instead of running BoxFaces and BoxEdges.</summary>
    internal BoxAxes Pre;
    internal bool HavePre;

    /// <summary>The axis tests each queued box–box pair's query will read (for the pose it will get: bInA's rotation as a matrix, its translation, the two core sizes). A pair whose bounding
    /// spheres are apart (Need 0) gets none, since Collide returns before PolyPoly; the others get the face axes, and the edge axes too when PolyPoly will search them: the edge hint does not skip them
    /// (Need 2) and neither face axis separates by more than the limit. A pair without edge axes has EdgesDone false, and PolyPoly then runs BoxEdges itself if it needs them, so the choice only ever
    /// saves work. With Vector256 the pairs go eight at a time through BoxAxes256 (one pair a lane); otherwise, and as the reference, each pair runs the scalar BoxFaces and BoxEdges.</summary>
    [SkipLocalsInit]
    internal static void BoxAxesBatch(Engine3D.BoxJob[] jobs, BoxAxes[] res, int n, bool wide)
    {
        Span<int> idx = stackalloc int[n];
        int m = 0;
        for (int k = 0; k < n; k++) { res[k].FacesDone = false; res[k].EdgesDone = false; res[k].SkipEdges = jobs[k].Need == 1; idx[m] = k; m += jobs[k].Need != 0 ? 1 : 0; }
        // wide: the caller's kernel width is eight lanes (Engine3D.KernelWidth); any other width takes the scalar BoxFaces / BoxEdges per pair, which is also the reference.
        if (wide) { for (int t0 = 0; t0 < m; t0 += 8) { BoxAxes256(jobs, res, idx, t0, Math.Min(8, m - t0), false); } }
        else
        {
            for (int t = 0; t < m; t++)
            {
                ref Engine3D.BoxJob j = ref jobs[idx[t]];
                ref BoxAxes o = ref res[idx[t]];
                BoxFaces(j.Ga.Convex.CoreSize, j.Gb.Convex.CoreSize, j.BInA.Rotation.ToMatrixInline(), j.BInA.Translation, out o.SepA, out o.FaceA, out o.SepB, out o.FaceB);
                o.FacesDone = true; o.EdgesDone = false;
            }
        }

        int e = 0;
        for (int t = 0; t < m; t++)
        {
            int k = idx[t];
            ref Engine3D.BoxJob j = ref jobs[k];
            idx[e] = k;
            e += j.Need == 2 && !(res[k].SepA > j.Limit) && !(res[k].SepB > j.Limit) ? 1 : 0;
        }

        if (wide) { for (int t0 = 0; t0 < e; t0 += 8) { BoxAxes256(jobs, res, idx, t0, Math.Min(8, e - t0), true); } }
        else
        {
            for (int t = 0; t < e; t++)
            {
                ref Engine3D.BoxJob j = ref jobs[idx[t]];
                ref BoxAxes o = ref res[idx[t]];
                o.SepE = BoxEdges(j.Ga.Convex.CoreSize, j.Gb.Convex.CoreSize, j.BInA.Rotation.ToMatrixInline(), j.BInA.Translation, out o.EdgeA, out o.EdgeB, out o.AxisE);
                o.EdgesDone = true;
            }
        }
    }

    /// <summary>What Collide and PolyPoly decide for a box–box pair before they read its axes, worked out with their operations: 0 when the bounding spheres are apart (Collide returns before
    /// PolyPoly), 1 when the edge hint skips the edge search, else 2. <paramref name="limit"/> is PolyPoly's (rA + rB) + margin.</summary>
    internal static int BoxAxesNeed(in ConvexShape3D a, in ConvexShape3D b, in RigidTransformF bInA, float spec, in EdgeHint3D hint, out float limit)
    {
        float rA = a.ConvexRadius, rB = b.ConvexRadius, rr = rA + rB;
        limit = rr + spec;
        float radA = ConvexShape3D.Sqrt(a.CoreSize.LengthSquared()), radB = ConvexShape3D.Sqrt(b.CoreSize.LengthSquared());
        float reach = (radA + rA) + (radB + rB) + spec; // Extent of two boxes
        float t2 = bInA.Translation.LengthSquared();
        if (t2 > reach * reach) { return 0; }
        return hint.Gap > 0f && hint.Gap > EdgeSkipSafety * hint.PoseChange(bInA, radA + radB) ? 1 : 2;
    }

    // How far inside a normal cone (as a component of the unit axis) an edge axis must lie to count; closer to the rim it coincides with a face axis to within a few parts in ten thousand.
    private const float ConeMargin = 1e-4f;

    // Separation of the best face axis of each box against the other, with the face's index in the cube's numbering. A is in its own frame; B is placed by r (its axes as columns) and t.
    // Face axis f of A: the lowest point of B along the normal minus the plane offset; the lowest point of a box along an axis is its centre's coordinate minus the sum of |axis component| times the half extents.
    private static void BoxFaces(Vector3F hA, Vector3F hB, in Matrix3x3F r, Vector3F t, out float sepA, out int faceA, out float sepB, out int faceB)
    {
        BoxTopology topo = s_boxTopo;
        float a00 = Math.Abs(r.M11), a01 = Math.Abs(r.M12), a02 = Math.Abs(r.M13);
        float a10 = Math.Abs(r.M21), a11 = Math.Abs(r.M22), a12 = Math.Abs(r.M23);
        float a20 = Math.Abs(r.M31), a21 = Math.Abs(r.M32), a22 = Math.Abs(r.M33);
        // Reach of B along each of A's axes.
        float rx = (a00 * hB.X) + (a01 * hB.Y) + (a02 * hB.Z);
        float ry = (a10 * hB.X) + (a11 * hB.Y) + (a12 * hB.Z);
        float rz = (a20 * hB.X) + (a21 * hB.Y) + (a22 * hB.Z);
        float lx = -rx - hA.X, ly = -ry - hA.Y, lz = -rz - hA.Z;
        float tx = t.X, ty = t.Y, tz = t.Z;
        // Evaluate the signed axes once, then visit them in topology order so ties
        // keep precisely the same face. This removes axis selection from the loop.
        Span<float> separations = stackalloc float[6];
        separations[0] = (1f * tx) + lx; separations[1] = (-1f * tx) + lx;
        separations[2] = (1f * ty) + ly; separations[3] = (-1f * ty) + ly;
        separations[4] = (1f * tz) + lz; separations[5] = (-1f * tz) + lz;
        sepA = float.NegativeInfinity;
        faceA = 0;
        for (int f = 0; f < 6; f++)
        {
            float sep = separations[topo.SignedAxis[f]];
            if (sep > sepA) { sepA = sep; faceA = f; }
        }

        // Reach of A along each of B's axes, and B's centre along its axes.
        float ca0 = (a00 * hA.X) + (a10 * hA.Y) + (a20 * hA.Z);
        float ca1 = (a01 * hA.X) + (a11 * hA.Y) + (a21 * hA.Z);
        float ca2 = (a02 * hA.X) + (a12 * hA.Y) + (a22 * hA.Z);
        float u0 = (r.M11 * tx) + (r.M21 * ty) + (r.M31 * tz);
        float u1 = (r.M12 * tx) + (r.M22 * ty) + (r.M32 * tz);
        float u2 = (r.M13 * tx) + (r.M23 * ty) + (r.M33 * tz);
        lx = -ca0 - hB.X; ly = -ca1 - hB.Y; lz = -ca2 - hB.Z;
        separations[0] = (-1f * u0) + lx; separations[1] = (1f * u0) + lx;
        separations[2] = (-1f * u1) + ly; separations[3] = (1f * u1) + ly;
        separations[4] = (-1f * u2) + lz; separations[5] = (1f * u2) + lz;
        sepB = float.NegativeInfinity;
        faceB = 0;
        for (int f = 0; f < 6; f++)
        {
            float sep = separations[topo.SignedAxis[f]];
            if (sep > sepB) { sepB = sep; faceB = f; }
        }
    }

    // The nine edge-edge axes (an edge of A crossed with an edge of B). The axis is oriented from A towards B; its separation is the distance between the centres along it minus the reach of each box. The edges
    // reported are the ones that support each box along the axis (A's along the axis, B's against it). Parallel pairs have no axis. Returns negative infinity and edge -1 when no pair has one.
    // The cone tests, the reaches and the centre distance are worked out for the unnormalized axis l = e_i x c_j (Ericson, Real-Time Collision Detection, 4.4.1) and are all multiples of |l|, so the square root
    // and the divide are paid only by an axis that survives the cone tests.
    private static float BoxEdges(Vector3F hA, Vector3F hB, in Matrix3x3F r, Vector3F t, out int edgeA, out int edgeB, out Vector3F axis)
    {
        BoxTopology topo = s_boxTopo;
        edgeA = -1;
        edgeB = -1;
        axis = default;
        float best = float.NegativeInfinity;
        int bi = 0, bj = 0;
        float bs = 1f, bl = 1f;
        float bl0 = 0f, bl1 = 0f, bl2 = 0f, bm0 = 0f, bm1 = 0f, bm2 = 0f;
        const float cutMargin = ConeMargin * ConeMargin * 0.998f;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                Vector3F c = j == 0 ? new Vector3F(r.M11, r.M21, r.M31) : j == 1 ? new Vector3F(r.M12, r.M22, r.M32) : new Vector3F(r.M13, r.M23, r.M33);
                // e_i x c
                Vector3F l = i == 0 ? new Vector3F(0f, -c.Z, c.Y) : i == 1 ? new Vector3F(c.Z, 0f, -c.X) : new Vector3F(-c.Y, c.X, 0f);
                float l2 = l.LengthSquared();
                if (!(l2 > 1e-10f)) { continue; }
                float cut = cutMargin * l2;
                if (i == 0 ? (l.Y * l.Y < cut || l.Z * l.Z < cut) : i == 1 ? (l.X * l.X < cut || l.Z * l.Z < cut) : (l.X * l.X < cut || l.Y * l.Y < cut)) { continue; }
                // The axis in B's frame, times |l|: the dot product of each of B's axes with l.
                float m0 = (r.M11 * l.X) + (r.M21 * l.Y) + (r.M31 * l.Z);
                float m1 = (r.M12 * l.X) + (r.M22 * l.Y) + (r.M32 * l.Z);
                float m2 = (r.M13 * l.X) + (r.M23 * l.Y) + (r.M33 * l.Z);
                // Only an axis that lies strictly inside the normal cones of both edges is a face of the Minkowski difference (Gregorius: the arcs of the adjacent face normals must cross). An axis on a cone's rim is
                // a face axis of one of the boxes, which the face queries already measured, so it is not an edge axis.
                float mx = m0 * m0, my = m1 * m1, mz = m2 * m2;
                if (j == 0 ? (my < cut || mz < cut) : j == 1 ? (mx < cut || mz < cut) : (mx < cut || my < cut)) { continue; }
                float du = Vector3F.Dot(l, t);
                // The sign of du depends on the pair's geometry and mispredicted as a branch; this is the same result without one: du < 0 flips du's sign bit (so -0 stays -0, as
                // the branch left it) and gives s = -1 exactly.
                int neg = du < 0f ? 1 : 0;
                du = BitConverter.Int32BitsToSingle(BitConverter.SingleToInt32Bits(du) ^ (neg << 31));
                float s = 1f - 2f * neg;
                float reachA = (Math.Abs(l.X) * hA.X) + (Math.Abs(l.Y) * hA.Y) + (Math.Abs(l.Z) * hA.Z);
                float reachB = (Math.Abs(m0) * hB.X) + (Math.Abs(m1) * hB.Y) + (Math.Abs(m2) * hB.Z);
                float len = ConvexShape3D.Sqrt(l2);
                float sep = (du - reachA - reachB) / len;
                if (sep > best)
                {
                    best = sep;
                    bi = i; bj = j; bs = s; bl = len;
                    bl0 = l.X; bl1 = l.Y; bl2 = l.Z; bm0 = m0; bm1 = m1; bm2 = m2;
                }
            }
        }

        if (best > float.NegativeInfinity)
        {
            // The winner is normalized once: n = s l / |l|, and B's axis components m = s m_u / |l|.
            float nx = bs * bl0 / bl, ny = bs * bl1 / bl, nz = bs * bl2 / bl;
            float x0 = bs * bm0 / bl, x1 = bs * bm1 / bl, x2 = bs * bm2 / bl;
            axis = new Vector3F(nx, ny, nz);
            // Corner signs of the supporting edges on the two axes each edge does not run along.
            int ka = (bi == 0 ? (ny > 0f ? 1 : 0) + (nz > 0f ? 2 : 0) : bi == 1 ? (nx > 0f ? 1 : 0) + (nz > 0f ? 2 : 0) : (nx > 0f ? 1 : 0) + (ny > 0f ? 2 : 0));
            int kb = (bj == 0 ? (x1 < 0f ? 1 : 0) + (x2 < 0f ? 2 : 0) : bj == 1 ? (x0 < 0f ? 1 : 0) + (x2 < 0f ? 2 : 0) : (x0 < 0f ? 1 : 0) + (x1 < 0f ? 2 : 0));
            edgeA = topo.Edge[(bi * 4) + ka];
            edgeB = topo.Edge[(bj * 4) + kb];
        }

        return best;
    }
}
