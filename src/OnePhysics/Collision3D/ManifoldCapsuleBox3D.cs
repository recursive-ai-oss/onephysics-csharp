using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class ManifoldGenerator3D
{
    /// <summary>Test switch: send capsule-against-box pairs through GJK/EPA instead of the closest-point routine below, to compare the two.</summary>
    internal bool GeneralCapsuleBox;

    // A capsule against a box whose core segment and core box do not overlap: the closest pair of points between the core segment and the core box, found exactly, in place of GJK.
    // The closest pair of a segment and a convex polyhedron is one of: a segment end against the polyhedron (a clamp, for an axis-aligned box), or the segment against an edge of the
    // polyhedron (Ericson, Real-Time Collision Detection, 5.1.9 for the segment pair and 5.1.3 for the clamp). A segment interior against a face interior is either an end (the distance to a plane
    // is linear along the segment) or a parallel case in which the ends are equally close. The pair gives the same contact normal and points GJK would, and the supporting features along that
    // normal are clipped by the same routines as for any other pair, so a capsule lying along a face still gets both ends of its contact.
    // Returns false when the cores overlap or touch (the penetration direction then needs EPA).
    private static bool CapsuleBoxContact(in ConvexShape3D a, in ConvexShape3D b, bool capsuleIsB, in RigidTransformF bInA, out ConvexContact3D contact)
    {
        contact = default;
        ConvexShape3D cap = capsuleIsB ? b : a, box = capsuleIsB ? a : b;
        float hy = cap.CoreSize.Y;
        // The capsule's core ends in the box's frame (the same construction as CapsulePolyApart).
        Vector3F e0, e1;
        if (capsuleIsB)
        {
            Vector3F axis = bInA.Rotation.Rotate(new Vector3F(0f, hy, 0f));
            e0 = bInA.Translation + axis;
            e1 = bInA.Translation - axis;
        }
        else
        {
            QuaternionF inv = bInA.Rotation.Conjugated();
            e0 = inv.Rotate(new Vector3F(0f, hy, 0f) - bInA.Translation);
            e1 = inv.Rotate(new Vector3F(0f, -hy, 0f) - bInA.Translation);
        }

        Vector3F h = box.CoreSize;
        float rc = cap.ConvexRadius, rp = box.ConvexRadius;

        // Candidates in a fixed order; the first of equal distances wins.
        float best = float.PositiveInfinity;
        Vector3F onSeg = default, onBox = default;

        // The segment ends against the box.
        for (int i = 0; i < 2; i++)
        {
            Vector3F p = i == 0 ? e0 : e1;
            Vector3F q = new(Clamp(p.X, h.X), Clamp(p.Y, h.Y), Clamp(p.Z, h.Z));
            float d2 = (p - q).LengthSquared();
            if (!(d2 > 1e-12f)) { return false; } // an end inside or on the core box (also NaN)
            if (d2 < best) { best = d2; onSeg = p; onBox = q; }
        }

        // The segment crossing the core box: slab test (Ericson 5.3.3 for segments).
        {
            Vector3F d = e1 - e0;
            float t0 = 0f, t1 = 1f;
            for (int k = 0; k < 3; k++)
            {
                float o = k == 0 ? e0.X : k == 1 ? e0.Y : e0.Z, dk = k == 0 ? d.X : k == 1 ? d.Y : d.Z, hk = k == 0 ? h.X : k == 1 ? h.Y : h.Z;
                if (dk == 0f)
                {
                    if (o < -hk || o > hk) { t0 = 2f; break; }
                    continue;
                }

                float ta = (-hk - o) / dk, tb = (hk - o) / dk;
                if (ta > tb) { (ta, tb) = (tb, ta); }
                if (ta > t0) { t0 = ta; }
                if (tb < t1) { t1 = tb; }
                if (t0 > t1) { break; }
            }

            if (t0 <= t1) { return false; }
        }

        // The segment against the twelve edges of the core box.
        for (int axis = 0; axis < 3; axis++)
        {
            for (int corner = 0; corner < 4; corner++)
            {
                float s1 = (corner & 1) == 0 ? -1f : 1f, s2 = (corner & 2) == 0 ? -1f : 1f;
                Vector3F lo, hi;
                if (axis == 0) { lo = new Vector3F(-h.X, s1 * h.Y, s2 * h.Z); hi = new Vector3F(h.X, s1 * h.Y, s2 * h.Z); }
                else if (axis == 1) { lo = new Vector3F(s1 * h.X, -h.Y, s2 * h.Z); hi = new Vector3F(s1 * h.X, h.Y, s2 * h.Z); }
                else { lo = new Vector3F(s1 * h.X, s2 * h.Y, -h.Z); hi = new Vector3F(s1 * h.X, s2 * h.Y, h.Z); }
                ClosestSegmentSegment(e0, e1, lo, hi, out float s, out float t);
                Vector3F c1 = e0 + ((e1 - e0) * s), c2 = lo + ((hi - lo) * t);
                float d2 = (c1 - c2).LengthSquared();
                if (d2 < best) { best = d2; onSeg = c1; onBox = c2; }
            }
        }

        if (!(best > 1e-12f) || !float.IsFinite(best)) { return false; }
        float dist = ConvexShape3D.Sqrt(best);
        Vector3F nBoxToCap = (onSeg - onBox) / dist; // in the box's frame
        float sep = dist - rc - rp;
        if (capsuleIsB)
        {
            contact = new ConvexContact3D(sep, nBoxToCap, onBox + (nBoxToCap * rp), onSeg - (nBoxToCap * rc));
        }
        else
        {
            // The box is B and its frame is B's; the contact is reported in A's frame, from A (the capsule) to B.
            Vector3F nAB = bInA.Rotation.Rotate(-nBoxToCap);
            contact = new ConvexContact3D(sep, nAB, bInA.TransformPoint(onSeg - (nBoxToCap * rc)), bInA.TransformPoint(onBox + (nBoxToCap * rp)));
        }

        return true;
    }

    private static float Clamp(float x, float h) => x < -h ? -h : x > h ? h : x;
}
