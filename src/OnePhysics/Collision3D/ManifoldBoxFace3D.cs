using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class ManifoldGenerator3D
{
    // Two boxes in face contact, clipped in the frame of the reference box. The separating axis test has chosen the reference face (the face with the largest separation, as in Gregorius, GDC 2013); the
    // incident face is the face of the other box most opposed to it, and its four corners are clipped against the four sides of the reference face (Sutherland and Hodgman; Ericson, Real-Time Collision
    // Detection, 5.x). In the reference box's own axes the sides are the planes |u| = half extent, so a corner is tested with one subtraction and a sign instead of a cross product and a dot product, and
    // no polyhedron is loaded. Feature ids are those of the general route: the reference face against a surviving incident corner, or a reference edge against the incident edge it cut.
    // Returns false when the cores are apart and the nearest corner of the other box is not over the reference face (GJK then decides, as it always did), or when clipping leaves nothing.
    private bool BoxFaceManifold(in ConvexShape3D a, in ConvexShape3D b, in Matrix3x3F r, Vector3F tB, bool useB, int faceA, int faceB, float best, float rA, float rB, float margin, out Manifold3D result)
    {
        BoxTopology topo = s_boxTopo;
        int rf = useB ? faceB : faceA;
        int k = topo.FaceAxis[rf];
        float sg = topo.FaceSign[rf];
        Vector3F hp = useB ? b.CoreSize : a.CoreSize, hq = useB ? a.CoreSize : b.CoreSize;
        float rRef = useB ? rB : rA, rInc = useB ? rA : rB;

        // The other box's axes (columns) and centre in the reference box's frame: M[i, j] is component i of axis j.
        float m00, m01, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz;
        if (!useB)
        {
            m00 = r.M11; m01 = r.M12; m02 = r.M13; m10 = r.M21; m11 = r.M22; m12 = r.M23; m20 = r.M31; m21 = r.M32; m22 = r.M33;
            tx = tB.X; ty = tB.Y; tz = tB.Z;
        }
        else
        {
            m00 = r.M11; m01 = r.M21; m02 = r.M31; m10 = r.M12; m11 = r.M22; m12 = r.M32; m20 = r.M13; m21 = r.M23; m22 = r.M33;
            tx = -((r.M11 * tB.X) + (r.M21 * tB.Y) + (r.M31 * tB.Z));
            ty = -((r.M12 * tB.X) + (r.M22 * tB.Y) + (r.M32 * tB.Z));
            tz = -((r.M13 * tB.X) + (r.M23 * tB.Y) + (r.M33 * tB.Z));
        }

        // Incident face: the face normal (sign times an axis column) with the smallest dot product against the reference normal, which is sg times axis k; the first of equals.
        float row0 = k == 0 ? m00 : k == 1 ? m10 : m20, row1 = k == 0 ? m01 : k == 1 ? m11 : m21, row2 = k == 0 ? m02 : k == 1 ? m12 : m22;
        int inc = 0;
        float bd = float.PositiveInfinity;
        for (int f = 0; f < 6; f++)
        {
            int ax = topo.FaceAxis[f];
            float d = topo.FaceSign[f] * (ax == 0 ? row0 : ax == 1 ? row1 : row2) * sg;
            if (d < bd)
            {
                bd = d;
                inc = f;
            }
        }

        // The incident face's corners in the reference frame, in the face's loop order, each with its vertex code and the code of the edge that leaves it.
        BoxClipPt[] src = _bc0, dst = _bc1;
        float hk = k == 0 ? hp.X : k == 1 ? hp.Y : hp.Z;
        float h0 = hp.X, h1 = hp.Y, h2 = hp.Z;
        float e0 = hq.X, e1 = hq.Y, e2 = hq.Z;
        ushort refFace = FCode.Face(rf);
        for (int c = 0; c < 4; c++)
        {
            int vi = topo.FaceCorner[(4 * inc) + c];
            Vector3F u = topo.Corner[vi];
            float s0 = u.X > 0f ? e0 : -e0, s1 = u.Y > 0f ? e1 : -e1, s2 = u.Z > 0f ? e2 : -e2;
            src[c] = new BoxClipPt
            {
                X = tx + (m00 * s0) + (m01 * s1) + (m02 * s2),
                Y = ty + (m10 * s0) + (m11 * s1) + (m12 * s2),
                Z = tz + (m20 * s0) + (m21 * s1) + (m22 * s2),
                Ref = refFace,
                Inc = FCode.Vertex(vi),
                Out = FCode.Edge(topo.FaceEdge[(4 * inc) + c]),
            };
        }

        if (best > 0f)
        {
            // Separated cores: when the nearest corner of the incident face lies over the reference face it and its projection are the closest points of the cores, exactly what GJK would find.
            int nv = 0;
            float lo = float.PositiveInfinity;
            for (int c = 0; c < 4; c++)
            {
                float w = sg * (k == 0 ? src[c].X : k == 1 ? src[c].Y : src[c].Z);
                if (w < lo)
                {
                    lo = w;
                    nv = c;
                }
            }

            if (!((k == 0 || Math.Abs(src[nv].X) <= h0) && (k == 1 || Math.Abs(src[nv].Y) <= h1) && (k == 2 || Math.Abs(src[nv].Z) <= h2)))
            {
                result = default;
                return false;
            }
        }

        int m = 4;
        // Clip against the four sides of the reference face, in the order of its edge loop.
        for (int i = 0; i < 4 && m > 0; i++)
        {
            int ax = topo.FaceLatAxis[(4 * rf) + i];
            float side = topo.FaceLatSign[(4 * rf) + i];
            float half = ax == 0 ? h0 : ax == 1 ? h1 : h2;
            ushort refEdge = FCode.Edge(topo.FaceEdge[(4 * rf) + i]);
            int outCount = 0;
            for (int j = 0; j < m; j++)
            {
                BoxClipPt cur = src[j], nxt = src[j + 1 == m ? 0 : j + 1];
                float dc = half - (side * (ax == 0 ? cur.X : ax == 1 ? cur.Y : cur.Z));
                float dn = half - (side * (ax == 0 ? nxt.X : ax == 1 ? nxt.Y : nxt.Z));
                bool curIn = dc >= 0f, nxtIn = dn >= 0f;
                if (curIn == nxtIn)
                {
                    if (nxtIn) { dst[outCount++] = nxt; }
                    continue;
                }

                float t = dc / (dc - dn);
                dst[outCount++] = new BoxClipPt
                {
                    X = cur.X + ((nxt.X - cur.X) * t),
                    Y = cur.Y + ((nxt.Y - cur.Y) * t),
                    Z = cur.Z + ((nxt.Z - cur.Z) * t),
                    Ref = refEdge,
                    Inc = cur.Out,
                    Out = curIn ? FCode.Derived(refEdge) : cur.Out,
                };
                if (nxtIn) { dst[outCount++] = nxt; }
            }

            (src, dst) = (dst, src);
            m = outCount;
        }

        _count = 0;
        float rr = rA + rB;
        for (int i = 0; i < m; i++)
        {
            BoxClipPt p = src[i];
            float w = k == 0 ? p.X : k == 1 ? p.Y : p.Z;
            float sep = (sg * w) - hk - rr;
            if (sep > margin) { continue; }
            // The point midway between the surfaces: the incident point moved back along the normal by half the separation, in A's frame.
            float shift = sg * sep * 0.5f;
            float px = p.X, py = p.Y, pz = p.Z;
            if (k == 0) { px -= shift; } else if (k == 1) { py -= shift; } else { pz -= shift; }
            Vector3F pos = useB ? new Vector3F((r.M11 * px) + (r.M12 * py) + (r.M13 * pz) + tB.X, (r.M21 * px) + (r.M22 * py) + (r.M23 * pz) + tB.Y, (r.M31 * px) + (r.M32 * py) + (r.M33 * pz) + tB.Z) : new Vector3F(px, py, pz);
            Push(pos, sep, useB ? FCode.Pack(p.Inc, p.Ref) : FCode.Pack(p.Ref, p.Inc));
        }

        if (_count == 0)
        {
            result = default;
            return false;
        }

        // The normal points from A to B: the reference normal when A carries the face, its negation when B does.
        Vector3F nRef = k == 0 ? new Vector3F(sg, 0f, 0f) : k == 1 ? new Vector3F(0f, sg, 0f) : new Vector3F(0f, 0f, sg);
        Vector3F nAB = useB ? -(r.Transform(nRef)) : nRef;
        result = Finish(nAB);
        return true;
    }

    private struct BoxClipPt
    {
        public float X, Y, Z;
        public ushort Ref, Inc, Out;
    }

    private BoxClipPt[] _bc0 = new BoxClipPt[16];
    private BoxClipPt[] _bc1 = new BoxClipPt[16];
}
