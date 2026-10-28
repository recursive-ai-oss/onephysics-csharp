using System;
using RecursiveAI.OneMath;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class ManifoldGenerator3D
{
    // A capsule against a box or hull that is clearly apart: one face plane of the polyhedron separates the capsule's core segment from the polyhedron's core by more than the radii and the
    // speculative distance (the face axes are part of the separating axis test, Gregorius, GDC 2013; a segment is the convex hull of its two ends, so its nearest point to a plane is an end).
    // Most capsule/polyhedron pairs the broadphase reports in a pile are like this, and the check costs a few dot products where the general route runs GJK. When it finds a gap it also reports it
    // as the clearance for callers that skip queries (see ClearanceOut); the general route would return an empty manifold for these pairs as well.
    private bool CapsulePolyApart(in ConvexShape3D a, in ConvexShape3D b, bool capsuleIsB, in RigidTransformF bInA, float margin)
    {
        ConvexShape3D cap = capsuleIsB ? b : a, poly = capsuleIsB ? a : b;
        float hy = cap.CoreSize.Y;
        // The capsule's core ends in the polyhedron's frame.
        Vector3F e0, e1;
        if (capsuleIsB)
        {
            Matrix3x3F r = bInA.Rotation.ToMatrix();
            Vector3F axis = r * new Vector3F(0f, hy, 0f);
            e0 = bInA.Translation + axis;
            e1 = bInA.Translation - axis;
        }
        else
        {
            QuaternionF inv = bInA.Rotation.Conjugated();
            e0 = inv.Rotate(new Vector3F(0f, hy, 0f) - bInA.Translation);
            e1 = inv.Rotate(new Vector3F(0f, -hy, 0f) - bInA.Translation);
        }

        float limit = poly.ConvexRadius + cap.ConvexRadius + margin;
        float best = float.NegativeInfinity;
        if (poly.Kind == ConvexShapeKind3D.Box)
        {
            Vector3F h = poly.CoreSize;
            best = Math.Max(Math.Min(e0.X, e1.X) - h.X, -Math.Max(e0.X, e1.X) - h.X);
            best = Math.Max(best, Math.Max(Math.Min(e0.Y, e1.Y) - h.Y, -Math.Max(e0.Y, e1.Y) - h.Y));
            best = Math.Max(best, Math.Max(Math.Min(e0.Z, e1.Z) - h.Z, -Math.Max(e0.Z, e1.Z) - h.Z));
        }
        else
        {
            ConvexHull3D hull = poly.Hull!;
            ReadOnlySpan<HullFace3D> faces = hull.Faces;
            ReadOnlySpan<HullEdge3D> edges = hull.Edges;
            ReadOnlySpan<Vector3F> verts = hull.Vertices;
            for (int f = 0; f < faces.Length; f++)
            {
                Vector3F n = faces[f].Normal;
                float sep = Math.Min(Vector3F.Dot(n, e0), Vector3F.Dot(n, e1)) - Vector3F.Dot(n, verts[edges[faces[f].FirstEdge].Origin]);
                if (sep > best) { best = sep; }
            }
        }

        if (!(best > limit)) { return false; }
        ClearanceOut = best - limit;
        return true;
    }
}
