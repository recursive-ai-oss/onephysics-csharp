using System;
using RecursiveAI.OneMath;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

[Experimental("OPHYS0001")]
public sealed partial class ManifoldGenerator3D
{
    // Spheres and capsules are a core segment (a point for a sphere) plus a radius, so one routine covers sphere-sphere, sphere-capsule and capsule-capsule.
    // Parallel capsules get the two ends of their overlap, which keeps a capsule lying on a capsule from rocking on one point.
    private Manifold3D SegmentSegment(in ConvexShape3D a, in ConvexShape3D b, in RigidTransformF bInA, float margin)
    {
        float rA = a.ConvexRadius, rB = b.ConvexRadius, rr = rA + rB;
        bool capA = a.Kind == ConvexShapeKind3D.Capsule, capB = b.Kind == ConvexShapeKind3D.Capsule;
        Vector3F a0 = capA ? new Vector3F(0f, a.CoreSize.Y, 0f) : default, a1 = capA ? new Vector3F(0f, -a.CoreSize.Y, 0f) : default;
        Vector3F b0 = bInA.TransformPoint(capB ? new Vector3F(0f, b.CoreSize.Y, 0f) : default);
        Vector3F b1 = bInA.TransformPoint(capB ? new Vector3F(0f, -b.CoreSize.Y, 0f) : default);
        Vector3F da = a1 - a0, db = b1 - b0;
        float la2 = da.LengthSquared(), lb2 = db.LengthSquared();
        Vector3F cr = Vector3F.Cross(da, db);

        _count = 0;
        if (la2 > 0f && lb2 > 0f && cr.LengthSquared() <= 1e-8f * la2 * lb2)
        {
            float la = ConvexShape3D.Sqrt(la2), lb = ConvexShape3D.Sqrt(lb2);
            Vector3F ea = da / la, eb = db / lb;
            float tb0 = Vector3F.Dot(b0 - a0, ea), tb1 = Vector3F.Dot(b1 - a0, ea);
            float bLo = Math.Min(tb0, tb1), bHi = Math.Max(tb0, tb1);
            float lo = Math.Max(0f, bLo), hi = Math.Min(la, bHi);
            if (hi - lo > 1e-5f * la)
            {
                float mid = (lo + hi) * 0.5f;
                Vector3F pam = a0 + (ea * mid);
                Vector3F w = (b0 + (eb * Vector3F.Dot(pam - b0, eb))) - pam;
                Vector3F n = LineNormal(w, ea, (b0 + b1) * 0.5f - ((a0 + a1) * 0.5f));
                for (int k = 0; k < 2; k++)
                {
                    float at = k == 0 ? lo : hi;
                    Vector3F pa = a0 + (ea * at);
                    Vector3F pb = b0 + (eb * Vector3F.Dot(pa - b0, eb));
                    ushort ca, cb;
                    if (k == 0)
                    {
                        bool fromA = bLo <= 0f;
                        ca = fromA ? FCode.Vertex(0) : FCode.Edge(0);
                        cb = fromA ? FCode.Edge(0) : FCode.Vertex(tb0 < tb1 ? 0 : 1);
                    }
                    else
                    {
                        bool fromA = bHi >= la;
                        ca = fromA ? FCode.Vertex(1) : FCode.Edge(0);
                        cb = fromA ? FCode.Edge(0) : FCode.Vertex(tb0 < tb1 ? 1 : 0);
                    }

                    PushSurface(pa, pb, n, rA, rB, ca, cb, margin);
                }

                return Finish(n);
            }
        }

        ClosestSegmentSegment(a0, a1, b0, b1, out float s, out float t);
        Vector3F c1 = a0 + (da * s), c2 = b0 + (db * t);
        Vector3F d = c2 - c1;
        float dist2 = d.LengthSquared();
        Vector3F nn;
        float scale = Math.Max(1e-20f, Math.Max(la2, lb2));
        if (dist2 > 1e-12f * scale && dist2 > 0f)
        {
            nn = d / ConvexShape3D.Sqrt(dist2);
        }
        else
        {
            // The core segments cross or touch: the direction is ambiguous, so take the common perpendicular, else the line between the centres.
            Vector3F centres = ((b0 + b1) * 0.5f) - ((a0 + a1) * 0.5f);
            Vector3F p = cr.LengthSquared() > 0f ? cr.Normalized() : default;
            if (p.LengthSquared() > 0f)
            {
                nn = Vector3F.Dot(p, centres) < 0f ? -p : p;
            }
            else
            {
                nn = centres.LengthSquared() > 0f ? centres.Normalized() : Vector3F.UnitY;
            }
        }

        ushort sa = s <= 0f ? FCode.Vertex(0) : s >= 1f ? FCode.Vertex(1) : FCode.Edge(0);
        ushort sb = t <= 0f ? FCode.Vertex(0) : t >= 1f ? FCode.Vertex(1) : FCode.Edge(0);
        PushSurface(c1, c2, nn, rA, rB, sa, sb, margin);
        if (_count == 0 && dist2 > 0f)
        {
            // Apart by more than the margin: the core segments are ConvexShape3D.Sqrt(dist2) apart, the surfaces that much less the radii.
            float clear = ConvexShape3D.Sqrt(dist2) - rr - margin;
            if (clear > 0f) { ClearanceOut = clear; }
        }

        return Finish(nn);
    }

    // The unit direction from line A to line B perpendicular to the axis; falls back to the offset between the centres, then to any perpendicular.
    private static Vector3F LineNormal(Vector3F w, Vector3F axis, Vector3F centres)
    {
        float l2 = w.LengthSquared();
        if (l2 > 1e-12f)
        {
            return w / ConvexShape3D.Sqrt(l2);
        }

        Vector3F perp = centres - (axis * Vector3F.Dot(centres, axis));
        l2 = perp.LengthSquared();
        if (l2 > 1e-12f)
        {
            return perp / ConvexShape3D.Sqrt(l2);
        }

        Vector3F any = Math.Abs(axis.X) < 0.5f ? Vector3F.UnitX : Vector3F.UnitY;
        return Vector3F.Cross(axis, any).Normalized();
    }

    // A point from core points on A and B: the surfaces are the cores moved along the normal by the radii.
    private void PushSurface(Vector3F pa, Vector3F pb, Vector3F n, float rA, float rB, ushort ca, ushort cb, float margin)
    {
        Vector3F sa = pa + (n * rA), sb = pb - (n * rB);
        float sep = Vector3F.Dot(n, sb - sa);
        if (sep > margin)
        {
            return;
        }

        Push((sa + sb) * 0.5f, sep, FCode.Pack(ca, cb));
    }

    // A sphere against a box or hull, in the polyhedron's frame: the closest point of the core polyhedron to the sphere's centre, face by face (the interior of the face,
    // then its edges), or for a centre inside the core the nearest face plane. Exact to float rounding, unlike EPA on curved shapes.
    private Manifold3D SpherePoly(in ConvexShape3D sphere, in ConvexShape3D poly, bool sphereIsA, in RigidTransformF bInA, float margin)
    {
        Matrix3x3F rot = bInA.Rotation.ToMatrix();
        Vector3F c = sphereIsA ? (rot.Transposed() * -bInA.Translation) : bInA.Translation;
        ConvexHull3D h = HullOf(poly);
        float rS = sphere.ConvexRadius, rP = poly.ConvexRadius;
        ReadOnlySpan<HullFace3D> faces = h.Faces;
        ReadOnlySpan<HullEdge3D> edges = h.Edges;

        // The poly's frame is A's when the poly is A, otherwise B's; c is the sphere's centre there.
        // (When the sphere is A, B is the poly and c = B^-1 * 0; when the poly is A, c = bInA * 0.)
        int bestFace = 0;
        float maxD = float.NegativeInfinity;
        for (int f = 0; f < faces.Length; f++)
        {
            Vector3F nf = faces[f].Normal;
            float d = Vector3F.Dot(nf, c) - FaceOffset(poly, h, f);
            if (d > maxD)
            {
                maxD = d;
                bestFace = f;
            }
        }

        Vector3F nOut; // from the polyhedron towards the sphere
        Vector3F onPoly; // the closest point on the core polyhedron, or the projection onto the nearest face when inside
        float distCore; // signed: negative when the centre is inside
        ushort polyCode;
        if (maxD <= 0f)
        {
            nOut = faces[bestFace].Normal;
            onPoly = c - (nOut * maxD);
            distCore = maxD;
            polyCode = FCode.Face(bestFace);
        }
        else
        {
            float best = float.PositiveInfinity;
            onPoly = default;
            polyCode = 0;
            for (int f = 0; f < faces.Length; f++)
            {
                Vector3F nf = faces[f].Normal;
                float d = Vector3F.Dot(nf, c) - FaceOffset(poly, h, f);
                if (d <= 0f)
                {
                    continue;
                }

                HullFace3D face = faces[f];
                Vector3F p = c - (nf * d);
                bool inside = true;
                int e = face.FirstEdge;
                for (int k = 0; k < face.EdgeCount; k++)
                {
                    int e2 = edges[e].Next;
                    Vector3F va = PolyVertex(poly, h, edges[e].Origin), vb = PolyVertex(poly, h, edges[e2].Origin);
                    if (Vector3F.Dot(Vector3F.Cross(nf, vb - va), p - va) < 0f)
                    {
                        inside = false;
                        break;
                    }

                    e = e2;
                }

                if (inside)
                {
                    if (d < best)
                    {
                        best = d;
                        onPoly = p;
                        polyCode = FCode.Face(f);
                    }

                    continue;
                }

                e = face.FirstEdge;
                for (int k = 0; k < face.EdgeCount; k++)
                {
                    int e2 = edges[e].Next;
                    int ia = edges[e].Origin, ib = edges[e2].Origin;
                    Vector3F va = PolyVertex(poly, h, ia), vb = PolyVertex(poly, h, ib);
                    Vector3F ab = vb - va;
                    float l2 = ab.LengthSquared();
                    float t = l2 > 0f ? Clamp01(Vector3F.Dot(c - va, ab) / l2) : 0f;
                    Vector3F q = va + (ab * t);
                    float dd = (c - q).LengthSquared();
                    if (dd < best * best)
                    {
                        best = ConvexShape3D.Sqrt(dd);
                        onPoly = q;
                        polyCode = t <= 0f ? FCode.Vertex(ia) : t >= 1f ? FCode.Vertex(ib) : FCode.Edge(e >> 1);
                    }

                    e = e2;
                }
            }

            Vector3F w = c - onPoly;
            float wl = ConvexShape3D.Sqrt(w.LengthSquared());
            if (wl > 0f)
            {
                nOut = w / wl;
                distCore = wl;
            }
            else
            {
                nOut = faces[bestFace].Normal;
                distCore = 0f;
            }
        }

        float sep = distCore - rS - rP;
        if (sep > margin)
        {
            // The distance is exact up to rounding, so a caller that knows the shapes' motion can skip the next queries until the motion has used it up.
            ClearanceOut = sep - margin;
            return Empty;
        }

        Vector3F surfPoly = onPoly + (nOut * rP), surfSphere = c - (nOut * rS);
        Vector3F mid = (surfPoly + surfSphere) * 0.5f;
        Vector3F normal = nOut;
        _count = 0;
        if (sphereIsA)
        {
            // The poly is B: map the point into A's frame and flip the normal so it points from the sphere to the poly.
            mid = bInA.TransformPoint(mid);
            normal = -(rot * nOut);
            Push(mid, sep, FCode.Pack(FCode.Vertex(0), polyCode));
        }
        else
        {
            Push(mid, sep, FCode.Pack(polyCode, FCode.Vertex(0)));
        }

        return Finish(normal);
    }

    private static float FaceOffset(in ConvexShape3D s, ConvexHull3D h, int f)
    {
        HullFace3D face = h.Faces[f];
        if (s.Kind != ConvexShapeKind3D.Box)
        {
            return face.Offset;
        }

        return Vector3F.Dot(face.Normal, PolyVertex(s, h, h.Edges[face.FirstEdge].Origin));
    }
}
