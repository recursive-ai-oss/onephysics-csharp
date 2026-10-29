using System;
using RecursiveAI.OneMath;

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
        Span<float> lo = stackalloc float[3];
        lo[0] = -rx - hA.X; lo[1] = -ry - hA.Y; lo[2] = -rz - hA.Z;
        float tx = t.X, ty = t.Y, tz = t.Z;
        sepA = float.NegativeInfinity;
        faceA = 0;
        for (int f = 0; f < 6; f++)
        {
            int axis = topo.FaceAxis[f];
            float tc = axis == 0 ? tx : axis == 1 ? ty : tz;
            float sep = (topo.FaceSign[f] * tc) + lo[axis];
            if (sep > sepA) { sepA = sep; faceA = f; }
        }

        // Reach of A along each of B's axes, and B's centre along its axes.
        float ca0 = (a00 * hA.X) + (a10 * hA.Y) + (a20 * hA.Z);
        float ca1 = (a01 * hA.X) + (a11 * hA.Y) + (a21 * hA.Z);
        float ca2 = (a02 * hA.X) + (a12 * hA.Y) + (a22 * hA.Z);
        float u0 = (r.M11 * tx) + (r.M21 * ty) + (r.M31 * tz);
        float u1 = (r.M12 * tx) + (r.M22 * ty) + (r.M32 * tz);
        float u2 = (r.M13 * tx) + (r.M23 * ty) + (r.M33 * tz);
        Span<float> loB = stackalloc float[3];
        loB[0] = -ca0 - hB.X; loB[1] = -ca1 - hB.Y; loB[2] = -ca2 - hB.Z;
        sepB = float.NegativeInfinity;
        faceB = 0;
        for (int f = 0; f < 6; f++)
        {
            int axis = topo.FaceAxis[f];
            float uc = axis == 0 ? u0 : axis == 1 ? u1 : u2;
            float sep = (-topo.FaceSign[f] * uc) + loB[axis];
            if (sep > sepB) { sepB = sep; faceB = f; }
        }
    }

    // The nine edge-edge axes (an edge of A crossed with an edge of B). The axis is oriented from A towards B; its separation is the distance between the centres along it minus the reach of each box. The edges
    // reported are the ones that support each box along the axis (A's along the axis, B's against it). Parallel pairs have no axis. Returns negative infinity and edge -1 when no pair has one.
    private static float BoxEdges(Vector3F hA, Vector3F hB, in Matrix3x3F r, Vector3F t, out int edgeA, out int edgeB, out Vector3F axis)
    {
        BoxTopology topo = s_boxTopo;
        edgeA = -1;
        edgeB = -1;
        axis = default;
        float best = float.NegativeInfinity;
        int bi = 0, bj = 0;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                Vector3F c = j == 0 ? new Vector3F(r.M11, r.M21, r.M31) : j == 1 ? new Vector3F(r.M12, r.M22, r.M32) : new Vector3F(r.M13, r.M23, r.M33);
                // e_i x c
                Vector3F l = i == 0 ? new Vector3F(0f, -c.Z, c.Y) : i == 1 ? new Vector3F(c.Z, 0f, -c.X) : new Vector3F(-c.Y, c.X, 0f);
                float l2 = l.LengthSquared();
                if (!(l2 > 1e-10f)) { continue; }
                Vector3F n = l / ConvexShape3D.Sqrt(l2);
                float d = Vector3F.Dot(n, t);
                if (d < 0f) { n = -n; d = -d; }
                float m0 = (r.M11 * n.X) + (r.M21 * n.Y) + (r.M31 * n.Z);
                float m1 = (r.M12 * n.X) + (r.M22 * n.Y) + (r.M32 * n.Z);
                float m2 = (r.M13 * n.X) + (r.M23 * n.Y) + (r.M33 * n.Z);
                // Only an axis that lies strictly inside the normal cones of both edges is a face of the Minkowski difference (Gregorius: the arcs of the adjacent face normals must cross). An axis on a cone's rim is
                // a face axis of one of the boxes, which the face queries already measured, so it is not an edge axis.
                float nx = Math.Abs(n.X), ny = Math.Abs(n.Y), nz = Math.Abs(n.Z), mx = Math.Abs(m0), my = Math.Abs(m1), mz = Math.Abs(m2);
                bool insideA = i == 0 ? ny > ConeMargin && nz > ConeMargin : i == 1 ? nx > ConeMargin && nz > ConeMargin : nx > ConeMargin && ny > ConeMargin;
                bool insideB = j == 0 ? my > ConeMargin && mz > ConeMargin : j == 1 ? mx > ConeMargin && mz > ConeMargin : mx > ConeMargin && my > ConeMargin;
                if (!(insideA && insideB)) { continue; }
                float reachA = (Math.Abs(n.X) * hA.X) + (Math.Abs(n.Y) * hA.Y) + (Math.Abs(n.Z) * hA.Z);
                float reachB = (Math.Abs(m0) * hB.X) + (Math.Abs(m1) * hB.Y) + (Math.Abs(m2) * hB.Z);
                float sep = d - reachA - reachB;
                if (sep > best)
                {
                    best = sep;
                    bi = i; bj = j;
                    axis = n;
                    // Corner signs of the supporting edges on the two axes each edge does not run along.
                    int ka = (bi == 0 ? (n.Y > 0f ? 1 : 0) + (n.Z > 0f ? 2 : 0) : bi == 1 ? (n.X > 0f ? 1 : 0) + (n.Z > 0f ? 2 : 0) : (n.X > 0f ? 1 : 0) + (n.Y > 0f ? 2 : 0));
                    int kb = (bj == 0 ? (m1 < 0f ? 1 : 0) + (m2 < 0f ? 2 : 0) : bj == 1 ? (m0 < 0f ? 1 : 0) + (m2 < 0f ? 2 : 0) : (m0 < 0f ? 1 : 0) + (m1 < 0f ? 2 : 0));
                    edgeA = topo.Edge[(bi * 4) + ka];
                    edgeB = topo.Edge[(bj * 4) + kb];
                }
            }
        }

        return best;
    }
}
